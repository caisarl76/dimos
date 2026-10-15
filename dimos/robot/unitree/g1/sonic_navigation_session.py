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

"""Serialized navigation admission for the existing SONIC reference publisher.

No sockets, peer RPCs or motor writers. Its owner must call every method on one
event loop; a blocked peer operation cannot delay native IDLE or lease expiry.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum
import math
from typing import Any
from uuid import uuid4

from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_navigation import (
    NavigationInput,
    NavigationLimits,
    NavigationVelocityError,
    PlannerIntentTracker,
    encode_navigation_control,
    encode_planner,
)


class NavigationPhase(StrEnum):
    DISARMED = "disarmed"
    REARMING = "rearming"
    ACTIVE = "active"
    STOPPING = "stopping"
    FAULT = "fault"
    POSE = "pose"


@dataclass(frozen=True)
class NavigationStatus:
    phase: NavigationPhase = NavigationPhase.DISARMED
    request_id: str | None = None
    source_sequence: int = -1
    guard_ack: int = 0
    cancellation_pending: bool = False
    fault: str | None = None
    pose_ready: bool = True
    rejected_input: str | None = None


class SonicNavigationSession:
    HEARTBEAT = 0.020
    FEEDBACK_TIMEOUT = 0.200
    SOURCE_TIMEOUT = 0.350
    OUTAGE_END = 2.0
    # Timing lapses of the relayed readiness proof (a late relay reply, a stale peer
    # input) stop walking at the same deadline but recover on fresh proof, like a
    # velocity outage; integrity and configuration faults stay terminal.
    TIMING_UNREADY = ("stale:", "upstream_lag:", "upstream_unready:", "missing:")
    SETTLE_SECONDS = 0.500

    def __init__(self, *, send: Callable[[bytes], None]) -> None:
        self._send = send
        self._status = NavigationStatus()
        self._tracker = PlannerIntentTracker(NavigationLimits())
        self._sequence = 0
        self._input: NavigationInput | None = None
        self._applied_source_sequence = -1
        self._readiness_deadline = -math.inf
        self._source_barrier = -1
        self._last_send = -math.inf
        self._last_feedback: SonicFeedback | None = None
        self._rearm_sequence = 0
        self._idle_sequence = 0
        self._end_sequence = 0
        self._control: bytes | None = None
        self._control_deadline = -math.inf
        self._cancel_confirmed = False
        self._idle_since: float | None = None
        self._peer_settled_until = -math.inf
        self._outage_since: float | None = None
        self._publisher_ready_at = math.inf

    def publisher_started(self, *, now: float) -> None:
        """Record a fresh socket binding; drain receipts before a new handshake."""
        if self.status.phase != NavigationPhase.DISARMED or self.status.request_id is not None:
            raise RuntimeError("publisher startup requires a fresh navigation owner")
        if not math.isfinite(now):
            raise ValueError("finite publisher start time required")
        self._input = None
        self._readiness_deadline = -math.inf
        self._publisher_ready_at = now + 0.250

    def publisher_ready(self, *, now: float) -> bool:
        return now >= self._publisher_ready_at

    @property
    def status(self) -> NavigationStatus:
        return self._status

    def pose_ready(self, *, now: float) -> bool:
        return self.status.pose_ready and (
            self.status.request_id is None or self._peer_settled_until > now
        )

    def _set(self, **values: Any) -> None:
        self._status = replace(self._status, **values)

    def _next(self, now: float, high_water: int = 0) -> int:
        # Shared-host monotonic nanoseconds also fence delayed packets across a
        # publisher restart. Native packets still contain only sequence/deadline.
        self._sequence = max(self._sequence + 1, high_water + 1, int(now * 1_000_000_000))
        if not 0 < self._sequence < 2**63:
            raise ValueError("native sequence exhausted")
        return self._sequence

    @staticmethod
    def _fresh(feedback: SonicFeedback | None, now: float) -> bool:
        return (
            feedback is not None
            and feedback.received_monotonic <= now < feedback.received_monotonic + 0.2
        )

    def begin(
        self, feedback: SonicFeedback, readiness: NavigationInput, *, now: float
    ) -> NavigationStatus:
        if not self.publisher_ready(now=now):
            raise RuntimeError("publisher startup quiet interval unresolved")
        if self._status.phase not in (NavigationPhase.DISARMED, NavigationPhase.FAULT) or (
            self._status.request_id is not None and not self._status.pose_ready
        ):
            raise RuntimeError("navigation or unresolved cancellation is active")
        if (
            not readiness.ready
            or readiness.fault is not None
            or readiness.readiness_deadline <= now
        ):
            raise ValueError("fresh navigation readiness required")
        if (
            not self._fresh(feedback, now)
            or feedback.planner_guard is None
            or (feedback.planner_reference_active is not True)
        ):
            raise ValueError("fresh native planner feedback required")
        self._tracker = PlannerIntentTracker(NavigationLimits())
        self._tracker.stop(feedback)
        self._last_feedback = feedback
        self._input = None
        self._applied_source_sequence = -1
        self._source_barrier = readiness.source_sequence
        self._readiness_deadline = readiness.readiness_deadline
        self._cancel_confirmed = False
        self._idle_since = None
        self._peer_settled_until = -math.inf
        self._outage_since = None
        self._end_sequence = self._idle_sequence = 0
        self._status = NavigationStatus(NavigationPhase.REARMING, uuid4().hex, pose_ready=False)
        self._rearm_sequence = self._next(now, feedback.planner_guard.high_water)
        self._control_deadline = now + 0.25
        self._control = encode_navigation_control(
            "rearm", sequence=self._rearm_sequence, deadline=self._control_deadline
        )
        self._send(self._control)
        self._last_send = -math.inf
        return self.status

    @classmethod
    def _timing_lapse(cls, value: NavigationInput) -> bool:
        fault = value.fault
        return (
            fault is None
            or fault == "navigation_inputs_unready"
            or fault.startswith(cls.TIMING_UNREADY)
        )

    def receive(self, value: NavigationInput, *, now: float) -> bool:
        if self._status.phase not in (NavigationPhase.REARMING, NavigationPhase.ACTIVE):
            return False
        if (
            value.source_sequence < self._source_barrier
            or value.source_sequence < self.status.source_sequence
        ):
            return False
        # Readiness can advance independently of a frozen velocity source. Never
        # change the held velocity deadline when the source sequence repeats.
        if value.ready and value.fault is None and value.readiness_deadline > now:
            self._readiness_deadline = min(value.readiness_deadline, now + self.SOURCE_TIMEOUT)
        elif self._timing_lapse(value):
            return False  # tick holds IDLE until fresh proof returns or the outage ends
        else:
            self.fail(value.fault or "navigation_input_unready", now=now)
            return False
        high_water = max(self._source_barrier, self._status.source_sequence)
        if value.source_sequence <= high_water:
            return False
        self._input = value
        self._set(source_sequence=value.source_sequence)
        return value.velocity_deadline > now

    def observe(self, feedback: SonicFeedback, *, now: float) -> None:
        if not self._fresh(feedback, now):
            return
        self._last_feedback = feedback
        guard = feedback.planner_guard
        if guard is None:
            return
        if self.status.phase == NavigationPhase.REARMING:
            if (
                guard.rearm_barrier == self._rearm_sequence
                and guard.lease_armed
                and (
                    guard.idle_ack_sequence > self._rearm_sequence
                    and guard.idle_ack_sequence >= self._idle_sequence
                )
                and not guard.latched
                and guard.mode == 0
            ):
                self._set(phase=NavigationPhase.ACTIVE, guard_ack=guard.idle_ack_sequence)
                self._control = None
        elif self.status.phase in (NavigationPhase.STOPPING, NavigationPhase.FAULT):
            if (
                guard.idle_ack_sequence == self._end_sequence
                and guard.mode == 0
                and not guard.lease_armed
            ):
                self._idle_since = now if self._idle_since is None else self._idle_since
                self._set(guard_ack=guard.idle_ack_sequence)
            else:
                self._idle_since = None

    def end(self, *, now: float) -> NavigationStatus:
        if self.status.phase in (NavigationPhase.DISARMED, NavigationPhase.POSE):
            return self.status
        if self.status.phase not in (NavigationPhase.STOPPING, NavigationPhase.FAULT):
            self._set(phase=NavigationPhase.STOPPING, cancellation_pending=True, pose_ready=False)
            self._input = None
            self._outage_since = None
            self._idle_since = None
            self._cancel_confirmed = False
            self._peer_settled_until = -math.inf
            self._send_end(now)
        return self.status

    def _send_end(self, now: float) -> None:
        hwm = (
            0
            if self._last_feedback is None or self._last_feedback.planner_guard is None
            else self._last_feedback.planner_guard.high_water
        )
        try:
            self._tracker.stop(self._last_feedback)
        except ValueError:
            # Malformed frame feedback must not prevent the native End command.
            self._tracker.stop(None)
        self._end_sequence = self._next(now, hwm)
        self._control_deadline = now + 0.25
        self._control = encode_navigation_control(
            "end", sequence=self._end_sequence, deadline=self._control_deadline
        )
        self._send(self._control)
        self._last_send = now

    def fail(self, reason: str, *, now: float) -> None:
        if self.status.phase in (NavigationPhase.DISARMED, NavigationPhase.POSE):
            return
        if self.status.phase != NavigationPhase.FAULT:
            self.end(now=now)
            self._set(
                phase=NavigationPhase.FAULT,
                fault=reason,
                cancellation_pending=True,
                pose_ready=False,
            )

    def confirm_cancel(self, reply: dict[str, Any], *, now: float) -> bool:
        if (
            reply.get("request_id") != self.status.request_id
            or reply.get("cancelled") is not True
            or reply.get("state") != "idle"
        ):
            return False
        self._cancel_confirmed = True
        return True

    def confirm_peer_status(self, reply: dict[str, Any], *, now: float) -> bool:
        if reply.get("request_id") != self.status.request_id:
            return False
        # Current, correlated negative evidence supersedes an earlier proof.
        # The execution owner rejects expired RPC replies before calling here.
        self._peer_settled_until = -math.inf
        if not self._cancel_confirmed or (
            reply.get("cancelled") is not True
            or reply.get("state") != "idle"
            or reply.get("ready") is not True
        ):
            return False
        numbers: list[float] = []
        for key in ("readiness_deadline", "linear_speed_mps", "yaw_rate_radps", "settled_for_s"):
            value = reply.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                return False
            numbers.append(float(value))
        deadline, linear, yaw, duration = numbers
        if deadline <= now or not 0 <= linear <= 0.03 or abs(yaw) > 0.03 or duration < 0.5:
            return False
        self._peer_settled_until = min(deadline, now + 0.35)
        return True

    def tick(
        self, feedback: SonicFeedback | None, *, subscriptions_ready: bool, now: float
    ) -> None:
        phase = self.status.phase
        if phase in (NavigationPhase.DISARMED, NavigationPhase.POSE):
            return
        if not subscriptions_ready:
            self.fail("subscriptions_lost", now=now)
        if not self._fresh(feedback, now):
            self._idle_since = None
            self.fail("feedback_lost", now=now)
            return
        assert feedback is not None
        self.observe(feedback, now=now)
        phase = self.status.phase
        if phase in (NavigationPhase.REARMING, NavigationPhase.ACTIVE):
            guard = feedback.planner_guard
            if phase == NavigationPhase.ACTIVE and (
                guard is None
                or guard.latched
                or not guard.lease_armed
                or feedback.planner_reference_active is not True
            ):
                self.fail("native_planner_lease_lost", now=now)
                return
            if phase == NavigationPhase.REARMING and now >= self._control_deadline:
                self.fail("rearm_timeout", now=now)
                return
            if now < self._last_send + self.HEARTBEAT:
                return
            if phase == NavigationPhase.REARMING:
                assert self._control is not None
                self._send(self._control)
                intent = self._tracker.stop(feedback)
                sequence = self._next(
                    now, feedback.planner_guard.high_water if feedback.planner_guard else 0
                )
                if self._idle_sequence == 0:
                    self._idle_sequence = sequence
                packet = encode_planner(intent, sequence=sequence, deadline=now + 0.25)
            else:
                value = self._input
                readiness_lapsed = self._readiness_deadline <= now
                expired = value is None or value.velocity_deadline <= now or readiness_lapsed
                if expired:
                    if self._outage_since is None:
                        lapsed = [
                            d
                            for d in (
                                now if value is None else value.velocity_deadline,
                                self._readiness_deadline,
                            )
                            if d <= now
                        ]
                        self._outage_since = min(lapsed)
                    if now >= self._outage_since + self.OUTAGE_END:
                        if readiness_lapsed:
                            self.fail("readiness_expired", now=now)
                        else:
                            self.end(now=now)
                        return
                    intent = self._tracker.stop(feedback)
                    deadline = now + 0.25
                else:
                    assert value is not None
                    self._outage_since = None
                    try:
                        if value.source_sequence != self._applied_source_sequence:
                            intent = self._tracker.update(value.twist, feedback)
                            self._applied_source_sequence = value.source_sequence
                            self._set(rejected_input=None)
                        else:
                            intent = self._tracker.heartbeat()
                    except NavigationVelocityError as exc:
                        intent = self._tracker.stop(feedback)
                        self._applied_source_sequence = value.source_sequence
                        self._set(rejected_input=str(exc))
                    except ValueError:
                        self.fail("invalid_navigation_feedback", now=now)
                        return
                    deadline = min(value.velocity_deadline, self._readiness_deadline, now + 0.25)
                packet = encode_planner(
                    intent,
                    sequence=self._next(now, guard.high_water if guard else 0),
                    deadline=deadline,
                )
            self._send(packet)
            self._last_send = now
        else:
            if (
                self._idle_since is not None
                and subscriptions_ready
                and now >= self._idle_since + self.SETTLE_SECONDS
                and (self._cancel_confirmed and self._peer_settled_until > now)
            ):
                self._set(
                    phase=NavigationPhase.FAULT if self.status.fault else NavigationPhase.DISARMED,
                    cancellation_pending=False,
                    pose_ready=True,
                )
                self._control = None
                return
            if (
                self._control is not None
                and now < self._control_deadline
                and now >= self._last_send + self.HEARTBEAT
            ):
                self._send(self._control)
                self._last_send = now
            elif now >= self._control_deadline and self.status.guard_ack != self._end_sequence:
                self._send_end(now)

    def enter_pose(self, *, now: float) -> None:
        if not self.pose_ready(now=now):
            raise RuntimeError("navigation cancellation and measured IDLE settling unresolved")
        self._input = None
        self._control = None
        self._set(phase=NavigationPhase.POSE, pose_ready=False)

    def handback(self) -> None:
        self._input = None
        self._control = None
        self._status = NavigationStatus()
