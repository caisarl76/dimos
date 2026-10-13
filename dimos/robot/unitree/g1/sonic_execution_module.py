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

"""DimOS RPC boundary for an independently running, guarded SONIC controller.

Owns a TCP reference publisher and telemetry subscriber, never a motor writer.
All socket and session operations run on Module's async RPC event loop. Source
inference stays outside this loop. Planner handback does not establish physical
settling, object retention or task success. Hardware commissioning is separate.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
import copy
from dataclasses import asdict
from pathlib import Path
import threading
import time
from typing import Any, Protocol
from uuid import uuid4

from numpy.typing import NDArray
from pydantic import Field, model_validator
import zmq

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.g1.sonic_feedback import (
    MAX_PACKET_BYTES,
    TOPIC,
    FeedbackBuffer,
    SonicFeedback,
    decode_feedback,
)
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_session import NavigationPhase, SonicNavigationSession
from dimos.robot.unitree.g1.sonic_session import (
    JsonlSessionJournal,
    SessionPhase,
    SessionStatus,
    SonicModelProfile,
    SonicTaskSession,
)
from dimos.spec.utils import Spec


class SonicNavigationControlSpec(Spec, Protocol):
    def cancel_navigation(self, request_id: str) -> dict[str, Any]: ...
    def set_navigation_goal(self, request_id: str, goal: PoseStamped) -> bool: ...
    def get_navigation_peer_status(self) -> dict[str, Any]: ...


_NATIVE_TOPICS = {b"command", b"planner", b"pose"}
_NAVIGATION_METHODS = ("cancel_navigation", "set_navigation_goal", "get_navigation_peer_status")


class SonicExecutionConfig(ModuleConfig):
    reference_bind: str = Field(pattern=r"^tcp://", max_length=2048)
    feedback_endpoint: str = Field(pattern=r"^tcp://", max_length=2048)
    profile: SonicModelProfile
    journal_dir: Path
    navigation_enabled: bool = False
    navigation_rpc_timeout_seconds: float = Field(default=0.5, gt=0, le=0.5, allow_inf_nan=False)

    @model_validator(mode="after")
    def bounded_shutdown(self) -> SonicExecutionConfig:
        if self.profile.handback_timeout > 5:
            raise ValueError("module handback_timeout must be at most 5 seconds")
        return self


class SonicExecutionModule(Module):
    """One active sonic78 session, accessible through typed DimOS RPCs.

    Binding prevents another local publisher from binding the same TCP endpoint.
    It cannot exclude publishers on other endpoints/hosts; v4 has no ownership
    nonce. Use only with the sole native reference source and receiver expiry.
    """

    config: SonicExecutionConfig
    feedback: Out[SonicFeedback]
    execution_status: Out[SessionStatus]
    navigation_input: In[NavigationInput]
    _navigation_control: SonicNavigationControlSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        self._lifecycle_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._snapshot: dict[str, Any] = {}
        self._feedback_snapshot: SonicFeedback | None = None
        self._running = False
        self._closing = False
        self._error: str | None = None
        self._session: SonicTaskSession | None = None
        self._journal_path: Path | None = None
        self._publisher: zmq.Socket[bytes] | None = None
        self._subscriber: zmq.Socket[bytes] | None = None
        self._subscriptions: set[bytes] = set()
        self._reference_endpoint = ""
        self._invalid_packets = 0
        self._published_status: SessionStatus | None = None
        self._navigation = SonicNavigationSession(send=self._send)
        self._navigation_input: NavigationInput | None = None
        self._pending_navigation: NavigationInput | None = None
        self._navigation_workers: ThreadPoolExecutor | None = None
        self._peer_call: asyncio.Future[Any] | None = None
        self._peer_call_kind = ""
        self._goal_call_owned = False
        self._peer_call_request: str | None = None
        self._peer_call_deadline = 0.0
        self._peer_cancel_confirmed = False
        self._peer_retry_after = 0.0
        super().__init__(**kwargs)
        self._buffer = FeedbackBuffer(stale_after=self.config.profile.feedback_timeout)
        self._cache()

    def _active(self) -> bool:
        return self._session is not None and self._session.status.phase not in (
            SessionPhase.IDLE,
            SessionPhase.DONE,
        )

    def _cache(self) -> None:
        with self._snapshot_lock:
            self._feedback_snapshot = self._buffer.latest(now=time.monotonic())
            self._snapshot = {
                "state": "fault" if self._error else "running" if self._running else "stopped",
                "accepting_requests": self._running and not self._closing and self._error is None,
                "reference_endpoint": self._reference_endpoint,
                "feedback_endpoint": self.config.feedback_endpoint,
                "profile": asdict(self.config.profile),
                "subscriptions_ready": _NATIVE_TOPICS.issubset(self._subscriptions),
                "navigation_enabled": self.config.navigation_enabled,
                "navigation": {
                    **asdict(self._navigation.status),
                    "pose_ready": self._navigation.pose_ready(now=time.monotonic()),
                },
                "task": None if self._session is None else asdict(self._session.status),
                "task_success": None,
                "journal_path": None if self._journal_path is None else str(self._journal_path),
                "invalid_packets": self._invalid_packets,
                "error": self._error,
            }

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Return execution and receipt freshness; task_success is unverified."""
        with self._snapshot_lock:
            result = copy.deepcopy(self._snapshot)
            state = self._feedback_snapshot
        age = None if state is None else time.monotonic() - state.received_monotonic
        fresh = (
            result["state"] == "running"
            and age is not None
            and 0 <= age < self.config.profile.feedback_timeout
        )
        guard = state.reference_guard if fresh and state is not None else None
        task = result["task"]
        result.update(
            feedback_age_s=age,
            feedback_fresh=fresh,
            ready=bool(
                result["accepting_requests"]
                and result["subscriptions_ready"]
                and guard is not None
                and guard.planner_confirmed
                and not guard.armed
                and state is not None
                and state.token_state is not None
                and (
                    not self.config.navigation_enabled
                    or (
                        result["navigation"]["pose_ready"]
                        and state.planner_guard is not None
                        and state.planner_guard.mode == 0
                        and not state.planner_guard.lease_armed
                        and not state.planner_guard.latched
                    )
                )
                and (task is None or task["phase"] in (SessionPhase.IDLE, SessionPhase.DONE))
            ),
        )
        return result

    @rpc
    def get_feedback(self) -> SonicFeedback | None:
        """Return a fresh immutable sample on this host's monotonic clock."""
        with self._snapshot_lock:
            state = self._feedback_snapshot
            running = self._snapshot["state"] == "running"
        if (
            running
            and state is not None
            and (
                0
                <= time.monotonic() - state.received_monotonic
                < self.config.profile.feedback_timeout
            )
        ):
            return state
        return None

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if self._running:
                return
            try:
                super().start()
                if self.config.navigation_enabled:
                    self.process_observable(
                        self.navigation_input.pure_observable(), self._queue_navigation_input
                    )
            except Exception as exc:
                self._error = f"{type(exc).__name__}: {exc}"
                self._cache()
                raise

    @rpc
    def stop(self) -> None:
        # RPC shutdown and worker cleanup can arrive concurrently. The second
        # caller must not close the event loop while the first awaits handback.
        with self._lifecycle_lock:
            super().stop()

    async def main(self) -> AsyncIterator[None]:
        if self.config.navigation_enabled and self._navigation_control is None:
            raise RuntimeError("navigation requires the local relay control Spec")
        if self.config.navigation_enabled:
            if self.navigation_input.transport is None:
                raise RuntimeError("navigation requires a bound atomic input stream")
        self._navigation_workers = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="sonic-navigation"
        )
        self.config.journal_dir.mkdir(parents=True, exist_ok=True)
        with (
            zmq.Context() as context,
            context.socket(zmq.XPUB) as pub,
            context.socket(zmq.SUB) as sub,
        ):
            for socket in (pub, sub):
                socket.setsockopt(zmq.LINGER, 0)
            pub.setsockopt(zmq.XPUB_VERBOSE, 1)
            pub.setsockopt(zmq.XPUB_NODROP, 1)
            pub.setsockopt(zmq.SNDHWM, 16)
            pub.bind(self.config.reference_bind)
            sub.setsockopt(zmq.RCVHWM, 128)
            sub.setsockopt(zmq.MAXMSGSIZE, MAX_PACKET_BYTES)
            sub.setsockopt(zmq.SUBSCRIBE, TOPIC)
            sub.connect(self.config.feedback_endpoint)
            self._reference_endpoint = pub.getsockopt_string(zmq.LAST_ENDPOINT)
            self._publisher, self._subscriber = pub, sub
            self._running, self._closing = True, False
            self._cache()
            driver = asyncio.create_task(self._drive())
            try:
                yield
            finally:
                self._closing = True
                # Check pending faults before shutdown can label the task cancelled.
                self._step()
                if self.config.navigation_enabled:
                    with suppress(Exception):
                        self._navigation.end(now=time.monotonic())
                        self._advance_peer(time.monotonic())
                if self._active():
                    assert self._session is not None
                    try:
                        self._session.cancel(now=time.monotonic())
                    except Exception as exc:
                        self._fault(exc)
                await driver
                self._running = False
                self._publisher = self._subscriber = None
                self._cache()
                self._navigation_workers.shutdown(wait=False, cancel_futures=True)

    @rpc
    def set_module_ref(self, name: str, module_ref: RPCClient) -> None:
        if name == "_navigation_control":
            for method in _NAVIGATION_METHODS:
                module_ref.rpc.rpc_timeouts[f"{module_ref.remote_name}/{method}"] = (
                    self.config.navigation_rpc_timeout_seconds
                )
        super().set_module_ref(name, module_ref)

    @rpc
    async def queue_navigation_input(self, value: NavigationInput) -> None:
        """Queue one owned atomic sample on the serialized module event loop."""
        await self._queue_navigation_input(value)

    async def _queue_navigation_input(self, value: NavigationInput) -> None:
        if self.config.navigation_enabled:
            owned = NavigationInput(
                value.twist,
                value.source_sequence,
                value.velocity_deadline,
                value.readiness_deadline,
                value.ready,
                value.fault,
            )
            previous = self._pending_navigation or self._navigation_input
            if previous is None or owned.source_sequence >= previous.source_sequence:
                self._pending_navigation = owned

    @rpc
    def get_navigation_status(self) -> dict[str, Any]:
        """Return admission, cancellation and native IDLE acknowledgment state."""
        return self.get_status()["navigation"]  # type: ignore[no-any-return]

    @rpc
    async def begin_navigation(self) -> dict[str, Any]:
        """Explicitly rearm from fresh readiness and native planner feedback."""
        self._request()
        if not self.config.navigation_enabled:
            raise RuntimeError("navigation is disabled")
        if self._active():
            raise RuntimeError("a pose execution task is active")
        state = self.get_feedback()
        if (
            not _NATIVE_TOPICS.issubset(self._subscriptions)
            or state is None
            or self._navigation_input is None
        ):
            raise RuntimeError("fresh navigation input and all native subscriptions required")
        guard = state.reference_guard
        if guard is None or not guard.planner_confirmed or guard.armed:
            raise RuntimeError("confirmed native planner handback required")
        self._peer_cancel_confirmed = False
        status = self._navigation.begin(state, self._navigation_input, now=time.monotonic())
        self._cache()
        return asdict(status)

    @rpc
    async def end_navigation(self) -> dict[str, Any]:
        """Close admission before issuing asynchronous peer cancellation."""
        self._request()
        self._navigation.end(now=time.monotonic())
        self._advance_peer(time.monotonic())
        self._cache()
        return asdict(self._navigation.status)

    @rpc
    async def set_navigation_goal(self, request_id: str, goal: PoseStamped) -> bool:
        """Bind a peer goal to the current explicit navigation request."""
        self._request()
        if (
            self._navigation.status.request_id != request_id
            or self._navigation.status.phase
            not in (NavigationPhase.REARMING, NavigationPhase.ACTIVE)
        ):
            raise ValueError("goal belongs to an inactive navigation request")
        if self._peer_call is not None or self._navigation_control is None:
            raise RuntimeError("navigation peer control is busy")
        assert self._navigation_workers is not None
        future = asyncio.get_running_loop().run_in_executor(
            self._navigation_workers, self._navigation_control.set_navigation_goal, request_id, goal
        )
        self._peer_call, self._peer_call_kind = future, "goal"
        self._goal_call_owned = True
        self._peer_call_deadline = time.monotonic() + self.config.navigation_rpc_timeout_seconds
        try:
            accepted = await asyncio.wait_for(
                asyncio.shield(future), timeout=self.config.navigation_rpc_timeout_seconds
            )
            if (
                accepted is not True
                or self._navigation.status.request_id != request_id
                or self._navigation.status.phase
                not in (NavigationPhase.REARMING, NavigationPhase.ACTIVE)
            ):
                raise RuntimeError("navigation goal rejected or admission closed")
            return True
        except Exception:
            self._navigation.end(now=time.monotonic())
            raise
        finally:
            self._goal_call_owned = False
            if future.done():
                self._peer_call = None
            self._cache()

    def _advance_peer(self, now: float) -> None:
        """One bounded outstanding control call; it never owns the heartbeat loop."""
        if self._peer_call is not None:
            if self._goal_call_owned or not self._peer_call.done():
                return
            call, self._peer_call = self._peer_call, None
            try:
                reply = call.result()
                if (
                    now < self._peer_call_deadline
                    and self._peer_call_request == self._navigation.status.request_id
                    and isinstance(reply, dict)
                ):
                    if self._peer_call_kind == "cancel":
                        self._peer_cancel_confirmed = self._navigation.confirm_cancel(
                            reply, now=now
                        )
                    else:
                        self._navigation.confirm_peer_status(reply, now=now)
            except Exception:
                pass  # No acknowledgment: admission remains closed, retry bounded RPC.
            self._peer_retry_after = now + 0.05
        if (
            self._navigation.status.request_id is None
            or self._navigation.status.phase
            not in (NavigationPhase.STOPPING, NavigationPhase.FAULT, NavigationPhase.DISARMED)
            or now < self._peer_retry_after
            or self._navigation_control is None
        ):
            return
        assert self._navigation_workers is not None
        request = self._navigation.status.request_id
        assert request is not None
        self._peer_call_request = request
        self._peer_call_deadline = now + self.config.navigation_rpc_timeout_seconds
        if self._peer_cancel_confirmed:
            self._peer_call_kind = "status"
            self._peer_call = asyncio.get_running_loop().run_in_executor(
                self._navigation_workers, self._navigation_control.get_navigation_peer_status
            )
        else:
            self._peer_call_kind = "cancel"
            self._peer_call = asyncio.get_running_loop().run_in_executor(
                self._navigation_workers, self._navigation_control.cancel_navigation, request
            )

    def _send(self, packet: bytes) -> None:
        if self._publisher is None:
            raise RuntimeError("reference publisher is closed")
        self._publisher.send(packet, flags=zmq.NOBLOCK)

    def _fault(self, exc: Exception) -> None:
        if self._error is None:
            self._error = f"{type(exc).__name__}: {exc}"
        if self.config.navigation_enabled:
            with suppress(Exception):
                self._navigation.fail("execution_fault", now=time.monotonic())
        if self._active():
            assert self._session is not None
            with suppress(Exception):
                # Core journal failures retain their own terminal failure evidence.
                self._session.fail(self._error, now=time.monotonic())

    def _receive(self) -> None:
        assert self._publisher is not None and self._subscriber is not None
        for _ in range(64):
            if not self._publisher.poll(0):
                break
            notice = self._publisher.recv(flags=zmq.NOBLOCK)
            if notice[:1] == b"\x01":
                self._subscriptions.add(notice[1:])
            else:
                self._subscriptions.discard(notice[1:])
        else:
            if self._publisher.poll(0):
                raise RuntimeError("subscription backlog exceeded receive budget")
        for _ in range(64):
            if not self._subscriber.poll(0):
                break
            parts = self._subscriber.recv_multipart(flags=zmq.NOBLOCK)
            try:
                if len(parts) != 1:
                    raise ValueError("single-part feedback required")
                state = decode_feedback(
                    parts[0], received_at=time.time(), received_monotonic=time.monotonic()
                )
            except ValueError:
                self._invalid_packets += 1
                continue
            if self._buffer.accept(state):
                if self._active():
                    assert self._session is not None
                    self._session.observe(state, now=time.monotonic())
                self.feedback.publish(state)
        else:
            if self._subscriber.poll(0):
                raise RuntimeError("feedback backlog exceeded receive budget")
        if self._active() and not _NATIVE_TOPICS.issubset(self._subscriptions):
            raise RuntimeError("native reference subscriptions lost")

    def _step(self) -> None:
        try:
            self._receive()
        except Exception as exc:
            self._fault(exc)
        if self.config.navigation_enabled:
            try:
                now = time.monotonic()
                if self._pending_navigation is not None:
                    self._navigation_input, self._pending_navigation = (
                        self._pending_navigation,
                        None,
                    )
                    self._navigation.receive(self._navigation_input, now=now)
                self._navigation.tick(
                    self._buffer.latest(now=now),
                    subscriptions_ready=_NATIVE_TOPICS.issubset(self._subscriptions),
                    now=now,
                )
                self._advance_peer(now)
                if (
                    self._navigation.status.phase == NavigationPhase.POSE
                    and self._session is not None
                    and self._session.status.phase == SessionPhase.DONE
                    and self._session.status.handback_confirmed
                ):
                    self._navigation.handback()
                    self._navigation_input = self._pending_navigation = None
            except Exception as exc:
                self._fault(exc)
        if self._session is not None:
            try:
                self._session.tick(now=time.monotonic())
                if self._session.status != self._published_status:
                    self.execution_status.publish(self._session.status)
                    self._published_status = self._session.status
            except Exception as exc:
                self._fault(exc)
        self._cache()

    async def _drive(self) -> None:
        while not self._closing or self._active():
            self._step()
            await asyncio.sleep(0.005)

    def _request(self, task_id: str | None = None) -> None:
        if not self._running or self._closing:
            raise RuntimeError("execution module is not accepting requests")
        self._step()
        if self._error is not None:
            raise RuntimeError(self._error)
        if task_id is not None and (
            self._session is None or self._session.status.task_id != task_id
        ):
            raise ValueError("request belongs to another task")

    @rpc
    async def start_task(self) -> dict[str, Any]:
        """Start one reference session from fresh, confirmed guarded planner state."""
        self._request()
        if self._active():
            raise RuntimeError("an execution task is already active")
        if not self.get_status()["ready"]:
            if self.config.navigation_enabled:
                self._navigation.end(now=time.monotonic())
                self._advance_peer(time.monotonic())
                self._cache()
            raise RuntimeError(
                "fresh guarded planner, cancellation/settling and command/planner/pose subscriptions required"
            )
        state = self.get_feedback()
        assert state is not None
        task_id = uuid4().hex
        journal_path = self.config.journal_dir / f"{task_id}.jsonl"
        session = SonicTaskSession(
            task_id,
            self.config.profile,
            send=self._send,
            journal=JsonlSessionJournal(journal_path),
        )
        # No await between admission and session assignment: async RPCs serialize here.
        self._session, self._journal_path = session, journal_path
        try:
            if self.config.navigation_enabled:
                self._navigation.enter_pose(now=time.monotonic())
            session.start(state, now=time.monotonic())
        finally:
            self._cache()
        return asdict(session.status)

    @rpc
    async def submit_chunk(
        self, task_id: str, sequence: int, observed_at: float, actions: NDArray[Any]
    ) -> dict[str, Any]:
        """Accept sonic78 actions using their original local observation timestamp."""
        self._request(task_id)
        assert self._session is not None
        try:
            self._session.submit_chunk(
                task_id=task_id,
                sequence=sequence,
                observed_at=observed_at,
                actions=actions,
                now=time.monotonic(),
            )
        finally:
            self._cache()
        return asdict(self._session.status)

    @rpc
    async def cancel_task(self, task_id: str) -> dict[str, Any]:
        """Cancel streaming and request native planner control; poll for confirmation."""
        self._request(task_id)
        assert self._session is not None
        self._session.cancel(now=time.monotonic())
        self._cache()
        return asdict(self._session.status)

    @rpc
    async def finish_task(self, task_id: str) -> dict[str, Any]:
        """End the action source; this does not report successful manipulation."""
        self._request(task_id)
        assert self._session is not None
        self._session.finish(now=time.monotonic())
        self._cache()
        return asdict(self._session.status)

    @rpc
    async def fail_task(self, task_id: str, reason: str) -> dict[str, Any]:
        """Report a policy/source failure and request native planner control."""
        self._request(task_id)
        assert self._session is not None
        self._session.fail(reason, now=time.monotonic())
        self._cache()
        return asdict(self._session.status)
