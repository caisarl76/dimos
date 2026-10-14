# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Thin pc2 relay: atomic bounded inputs and an independent cancellation slot."""

from __future__ import annotations

import threading
from time import monotonic
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_link import (
    NavigationPollReply,
    SonicNavigationLink,
    bound_navigation_input,
    readiness_deadline,
)


class SonicNavigationRelayConfig(ModuleConfig):
    peer_endpoint: str = Field(pattern=r"^tcp://", max_length=2048)


class SonicNavigationRelayModule(Module):
    config: SonicNavigationRelayConfig
    navigation_input: Out[NavigationInput]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._link = SonicNavigationLink(self.config.peer_endpoint)
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._request_id: str | None = None
        self._latest: NavigationInput | None = None
        self._cancel_pending = False
        self._goal_pending = False
        self._cancel_confirmed = False
        self._cancel_previous_request_id: str | None = None
        self._retired_requests: set[str] = set()
        self._request_generation = 0
        self._reset_fault: str | None = None
        # Peer instance epoch: pc2-side source sequences stay monotonic across peer
        # restarts. A new epoch starts only without a live session or after the
        # serving peer confirms cancellation; motion still needs an explicit begin.
        self._sequence_offset = 0

    @rpc
    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            super().start()
            self._thread = threading.Thread(
                target=self._poll, name="sonic-navigation-relay", daemon=True
            )
            self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=0.6)
        self._link.close()
        super().stop()

    def _publish_input(self, value: NavigationInput) -> None:
        if self.navigation_input.transport is not None:
            self.navigation_input.publish(value)

    def _closed_input(self, fault: str) -> NavigationInput:
        previous = self._latest
        return NavigationInput(
            Twist() if previous is None else previous.twist,
            0 if previous is None else previous.source_sequence,
            0.0 if previous is None else previous.velocity_deadline,
            monotonic(),
            False,
            fault,
        )

    def _in_epoch(self, value: NavigationInput) -> NavigationInput:
        return NavigationInput(
            value.twist,
            value.source_sequence + self._sequence_offset,
            value.velocity_deadline,
            value.readiness_deadline,
            value.ready,
            value.fault,
        )

    def _new_epoch(self) -> None:
        # Keep the pc2 high-water: closed inputs still carry it, new values exceed it.
        if self._latest is not None:
            self._sequence_offset = self._latest.source_sequence + 1

    def accept_poll(self, reply: NavigationPollReply, *, t0: float, t1: float) -> NavigationInput:
        raw = bound_navigation_input(reply, t0=t0, t1=t1)
        with self._lock:
            value = self._in_epoch(raw)
            if self._goal_pending:
                # Poll/control sockets have no ordering guarantee. Until the goal
                # acknowledgment, keep the previous motion proof (sequence, velocity,
                # velocity deadline) and refresh only sensor readiness, so a slow
                # goal round trip cannot expire readiness and end the session.
                previous = self._latest
                if (
                    previous is None
                    or self._reset_fault is not None
                    or self._cancel_pending
                    or value.source_sequence < previous.source_sequence
                ):
                    return self._closed_input("goal_pending")
                held = NavigationInput(
                    previous.twist,
                    previous.source_sequence,
                    previous.velocity_deadline,
                    value.readiness_deadline,
                    value.ready,
                    value.fault,
                )
                self._publish_input(held)
                return held
            if self._cancel_pending:
                value = self._closed_input(self._reset_fault or "cancellation_pending")
                self._publish_input(value)
                return value
            previous = self._latest
            if reply.request_id != self._request_id:
                self._reset_fault = "peer_request_changed"
            if (
                previous is not None
                and value.source_sequence < previous.source_sequence
                and (self._request_id is None or self._cancel_confirmed)
            ):
                # A restarted peer with no live session: nothing to protect.
                self._new_epoch()
                value = self._in_epoch(raw)
            if previous is not None:
                if value.source_sequence < previous.source_sequence:
                    self._reset_fault = "peer_source_rollback"
                elif value.source_sequence == previous.source_sequence:
                    if value._velocity != previous._velocity:
                        self._reset_fault = "cached_velocity_changed"
                    value = NavigationInput(
                        value.twist,
                        value.source_sequence,
                        min(value.velocity_deadline, previous.velocity_deadline),
                        value.readiness_deadline,
                        value.ready,
                        value.fault,
                    )
            if self._reset_fault is not None or self._cancel_pending:
                value = self._closed_input(self._reset_fault or "cancellation_pending")
            else:
                self._latest = value
            self._publish_input(value)
            return value

    def _poll(self) -> None:
        while not self._stopping.is_set():
            started = monotonic()
            with self._lock:
                request = self._request_id
                generation = self._request_generation
            try:
                reply, t0, t1 = self._link.poll(request)
                with self._lock:
                    if request == self._request_id and generation == self._request_generation:
                        self.accept_poll(reply, t0=t0, t1=t1)
            except (TimeoutError, ValueError, RuntimeError, OverflowError):
                pass  # No refreshed proof: the existing source/readiness deadlines expire.
            self._stopping.wait(max(0.0, started + 0.100 - monotonic()))

    @rpc
    def set_navigation_goal(self, request_id: str, goal: PoseStamped) -> bool:
        with self._lock:
            if (
                self._reset_fault is not None
                or self._cancel_pending
                or self._goal_pending
                or request_id in self._retired_requests
                or (self._request_id not in (None, request_id) and not self._cancel_confirmed)
            ):
                return False
            if self._request_id is not None and self._request_id != request_id:
                self._cancel_previous_request_id = self._request_id
                self._retired_requests.add(self._request_id)
            self._request_id = request_id
            self._goal_pending = True
            self._cancel_confirmed = False
            self._request_generation += 1
        try:
            result, _, _ = self._link.control("goal", request_id, goal)
            with self._lock:
                accepted = (
                    result["request_id"] == request_id
                    and result["accepted"] is True
                    and result["fault"] is None
                    and not self._cancel_pending
                    and self._request_id == request_id
                )
                if not accepted:
                    self._cancel_pending = True
                    self._publish_input(self._closed_input("goal_unresolved"))
                return accepted
        except Exception:
            with self._lock:
                self._cancel_pending = True
                self._publish_input(self._closed_input("goal_unresolved"))
            raise
        finally:
            with self._lock:
                self._goal_pending = False
                self._request_generation += 1

    @rpc
    def cancel_navigation(self, request_id: str) -> dict[str, Any]:
        with self._lock:
            if request_id in self._retired_requests or (
                self._request_id not in (None, request_id) and not self._cancel_confirmed
            ):
                return {
                    "request_id": request_id,
                    "cancelled": False,
                    "state": None,
                    "fault": "request_mismatch",
                }
            if self._request_id is not None and self._request_id != request_id:
                self._cancel_previous_request_id = self._request_id
                self._retired_requests.add(self._request_id)
            previous_request_id = self._cancel_previous_request_id
            self._request_id = request_id
            self._cancel_pending = True
            self._cancel_confirmed = False
            self._request_generation += 1
            self._publish_input(self._closed_input("cancellation_pending"))
        result, _, _ = self._link.control(
            "cancel", request_id, previous_request_id=previous_request_id
        )
        confirmed = (
            result["request_id"] == request_id
            and result["cancelled"] is True
            and result["state"] == "idle"
            and result["fault"] is None
        )
        with self._lock:
            if confirmed and self._request_id == request_id:
                if self._reset_fault is not None:
                    # The serving peer confirmed IDLE through its public planner: a
                    # fresh handshake, so its sequences may start a new epoch.
                    self._reset_fault = None
                    self._new_epoch()
                self._cancel_pending = False
                self._cancel_confirmed = True
                self._cancel_previous_request_id = None
                self._request_generation += 1
        return {
            "request_id": result["request_id"],
            "cancelled": confirmed,
            "state": result["state"],
            "fault": result["fault"],
        }

    @rpc
    def get_navigation_peer_status(self) -> dict[str, Any]:
        with self._lock:
            request = self._request_id
        result, t0, t1 = self._link.control("status", request)
        deadline, ready = readiness_deadline(result["readiness"], t0=t0, t1=t1)
        with self._lock:
            ready = ready and request == self._request_id and self._reset_fault is None
        return {
            "request_id": result["request_id"],
            "cancelled": result["cancelled"],
            "state": result["state"],
            "ready": ready,
            "readiness_deadline": min(deadline, t1 + 0.350),
            "linear_speed_mps": result["linear_speed_mps"],
            "yaw_rate_radps": result["yaw_rate_radps"],
            "settled_for_s": result["settled_for_s"],
            "fault": result["fault"],
        }
