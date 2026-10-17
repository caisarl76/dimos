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

"""Off-board velocity/readiness owner and public planner cancellation bridge."""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace
import math
import threading
from time import monotonic
from typing import TYPE_CHECKING, Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
import zmq

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationState
from dimos.navigation.go2.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_link import (
    MAX_FRAME_BYTES,
    NavigationPollReply,
    decode_goal,
    decode_navigation_message,
    encode_control_payload,
    encode_navigation_message,
    encode_poll_payload,
)
from dimos.robot.unitree.g1.sonic_navigation_readiness import NavigationReadiness

if TYPE_CHECKING:
    from dimos.core.rpc_client import RPCClient


SETTLE_WINDOW = 0.5  # seconds of pose history behind each settling rate
# A turn that stops abruptly overshoots and settles slowly (sim: 3.0-3.2 s); the relayed
# turn rate may fall by at most this much per second (0.20 rad/s -> 0 over 0.6 s).
TURN_RAMP_RATE = 0.2 / 0.6
TURN_RAMP_GAP = 0.350  # a longer source gap already stopped pc2: no ramp across it


def ramp_turn_rate(prior: float, wz: float, dt: float) -> float:
    """The yaw rate to relay after `prior` when `wz` is requested `dt` seconds later."""
    if abs(wz) >= abs(prior) and wz * prior >= 0:
        return wz
    step = TURN_RAMP_RATE * dt
    ramped = max(prior - step, wz) if prior > 0 else min(prior + step, wz)
    return wz if abs(ramped - wz) < 1e-9 else ramped


class SonicNavigationPeerConfig(ModuleConfig):
    link_bind: str = Field(pattern=r"^tcp://", max_length=2048)
    extrinsic_valid: bool = Field(default=False, strict=True)
    clock_domains: dict[str, str] = Field(default_factory=dict)
    odom_frame: str = Field(default="odom", min_length=1)
    base_frame: str = Field(default="base_link", min_length=1)


class SonicNavigationPeerModule(Module):
    config: SonicNavigationPeerConfig
    cmd_vel: In[Twist]
    lidar_raw: In[PointCloud2]
    imu_raw: In[Imu]
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    tf: In[TFMessage]
    global_map: In[PointCloud2]
    global_costmap: In[OccupancyGrid]
    _planner_spec: ReplanningAStarPlannerSpec | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._lifecycle = threading.Lock()
        self._stopping = threading.Event()
        self._started = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None
        self._bound_endpoint: str | None = None
        self._monitor = NavigationReadiness(
            extrinsic_valid=self.config.extrinsic_valid, clock_domains=self.config.clock_domains
        )
        self._source_sequence = 0
        self._source_at: float | None = None
        self._retired_requests: set[str] = set()
        self._velocity: NavigationInput | None = None
        self._command_fault: str | None = None
        self._request_id: str | None = None
        self._accepted = self._cancelled = False
        self._state: str | None = None
        # (stamp, x, y, yaw, receipt) spanning the settling window; a standing robot's
        # LIO pose jitters ~5 mm per sample, so rates use the pose SETTLE_WINDOW back.
        self._poses: deque[tuple[float, float, float, float, float]] = deque()
        self._linear: float | None = None
        self._yaw_rate: float | None = None
        self._settled_since: float | None = None
        self._settled_for = 0.0

    @property
    def bound_endpoint(self) -> str:
        if self._bound_endpoint is None:
            raise RuntimeError("navigation peer is not listening")
        return self._bound_endpoint

    @rpc
    def set_module_ref(self, name: str, module_ref: RPCClient) -> None:
        if name == "_planner_spec":
            for method in ("set_goal", "cancel_goal", "get_state"):
                module_ref.rpc.rpc_timeouts[f"{module_ref.remote_name}/{method}"] = 0.5
        super().set_module_ref(name, module_ref)

    @rpc
    def start(self) -> None:
        with self._lifecycle:
            if self._thread is not None:
                return
            if self._planner_spec is None:
                raise RuntimeError("navigation peer requires its public planner Spec")
            super().start()
            handlers = {
                "cmd_vel": self.observe_cmd_vel,
                "lidar_raw": self.observe_lidar_raw,
                "imu_raw": self.observe_imu_raw,
                "lidar": self.observe_lidar,
                "odometry": self.observe_odometry,
                "tf": self.observe_tf,
                "global_map": self.observe_global_map,
                "global_costmap": self.observe_global_costmap,
            }
            for name, port in self.inputs.items():
                if port.transport is not None:
                    self.register_disposable(Disposable(port.subscribe(handlers[name])))
            self._thread = threading.Thread(
                target=self._serve, name="sonic-navigation-peer", daemon=True
            )
            self._thread.start()
            if not self._started.wait(2) or self._error is not None:
                self._stopping.set()
                raise RuntimeError("navigation peer startup failed") from self._error

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=0.6)
        super().stop()

    def _ramp_turn(self, msg: Twist, now: float) -> Twist:
        """Bound how fast the relayed yaw rate slows or reverses; never delay an increase."""
        previous, wz = self._velocity, msg.angular.z
        if previous is None or self._source_at is None or now - self._source_at > TURN_RAMP_GAP:
            return msg
        ramped = ramp_turn_rate(previous.twist.angular.z, wz, now - self._source_at)
        if ramped == wz:
            return msg
        return Twist(linear=msg.linear, angular=Vector3(msg.angular.x, msg.angular.y, ramped))

    def observe_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            msg = self._ramp_turn(msg, monotonic())
            try:
                self._velocity = NavigationInput(
                    msg, self._source_sequence + 1, 0.0, 0.0, False, None
                )
            except (ValueError, OverflowError):
                self._command_fault = "invalid_velocity"
                return
            self._source_sequence += 1
            self._source_at = monotonic()
            self._command_fault = None

    def _cloud(self, name: str, msg: PointCloud2) -> None:
        attributes = msg.pointcloud_tensor.point
        valid = False
        if "positions" in attributes:
            points = attributes.positions.numpy()
            valid = len(points) > 0 and bool(np.isfinite(points).all())
        with self._lock:
            self._monitor.observe(name, msg.ts, valid=valid, received_monotonic=monotonic())

    def observe_lidar_raw(self, msg: PointCloud2) -> None:
        self._cloud("lidar_raw", msg)

    def observe_lidar(self, msg: PointCloud2) -> None:
        self._cloud("lio_cloud", msg)

    def observe_global_map(self, msg: PointCloud2) -> None:
        self._cloud("global_map", msg)

    def observe_imu_raw(self, msg: Imu) -> None:
        values = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        valid = all(math.isfinite(v) for v in values) and math.hypot(*values[3:]) > 0
        with self._lock:
            self._monitor.observe("imu_raw", msg.ts, valid=valid, received_monotonic=monotonic())

    def observe_global_costmap(self, msg: OccupancyGrid) -> None:
        valid = (
            msg.grid.size > 0 and bool(np.any(msg.grid >= 0)) and bool(np.isfinite(msg.grid).all())
        )
        with self._lock:
            self._monitor.observe(
                "global_costmap", msg.ts, valid=valid, received_monotonic=monotonic()
            )

    def observe_tf(self, msg: TFMessage) -> None:
        matching = [
            t
            for t in msg.transforms
            if t.frame_id == self.config.odom_frame and t.child_frame_id == self.config.base_frame
        ]
        # Unrelated static/waist messages cannot invalidate or refresh a base edge.
        if not matching:
            return
        with self._lock:
            for transform in matching:
                q, p = transform.rotation, transform.translation
                values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                valid = (
                    all(math.isfinite(v) for v in values)
                    and abs(math.hypot(*values[3:]) - 1) < 1e-3
                )
                self._monitor.observe(
                    "base_tf", transform.ts, valid=valid, received_monotonic=monotonic()
                )

    def observe_odometry(self, msg: Odometry) -> None:
        p, q = msg.position, msg.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        valid = (
            msg.frame_id == self.config.odom_frame
            and msg.child_frame_id == self.config.base_frame
            and all(math.isfinite(v) for v in values)
            and abs(math.hypot(*values[3:]) - 1) < 1e-3
        )
        with self._lock:
            now = monotonic()
            self._monitor.observe("odom", msg.ts, valid=valid, received_monotonic=now)
            if not valid or not math.isfinite(msg.ts) or msg.ts <= 0:
                self._poses.clear()
                self._linear = self._yaw_rate = self._settled_since = None
                self._settled_for = 0.0
                return
            previous = self._poses[-1] if self._poses else None
            if previous is not None and msg.ts <= previous[0]:
                return
            yaw = msg.yaw
            if (
                previous is None
                or not 0 < msg.ts - previous[0] <= 0.350
                or now - previous[4] > 0.350
            ):
                self._poses.clear()
                self._poses.append((msg.ts, p.x, p.y, yaw, now))
                self._linear = self._yaw_rate = self._settled_since = None
                self._settled_for = 0.0
                return
            self._poses.append((msg.ts, p.x, p.y, yaw, now))
            while len(self._poses) > 1 and self._poses[1][0] <= msg.ts - SETTLE_WINDOW:
                self._poses.popleft()
            base = self._poses[0]
            dt = msg.ts - base[0]
            if dt < SETTLE_WINDOW - 1e-6:  # window not yet spanned: no rate claimed
                self._linear = self._yaw_rate = self._settled_since = None
                self._settled_for = 0.0
                return
            self._linear = math.hypot(p.x - base[1], p.y - base[2]) / dt
            self._yaw_rate = math.remainder(yaw - base[3], 2 * math.pi) / dt
            if self._linear <= 0.03 and abs(self._yaw_rate) <= 0.03:
                if self._settled_since is None:
                    self._settled_since = base[4]
                self._settled_for = now - self._settled_since
            else:
                self._settled_since = None
                self._settled_for = 0.0

    def poll_reply(self, correlation_id: str) -> NavigationPollReply:
        with self._lock:
            now = monotonic()
            readiness = self._monitor.snapshot(now=now)
            if self._command_fault is not None:
                readiness = replace(readiness, ready=False, fault=self._command_fault)
            return NavigationPollReply(
                correlation_id,
                self._request_id,
                self._source_sequence,
                None if self._velocity is None else self._velocity.twist,
                0.350 if self._source_at is None else max(0.0, now - self._source_at),
                readiness,
            )

    def peer_status(self, *, refresh_state: bool = False) -> dict[str, Any]:
        if refresh_state:
            assert self._planner_spec is not None
            state = self._planner_spec.get_state()
            with self._lock:
                self._state = state.value
        with self._lock:
            return {
                "request_id": self._request_id,
                "accepted": self._accepted,
                "cancelled": self._cancelled and self._state == "idle",
                "state": self._state,
                "readiness": self._monitor.snapshot(now=monotonic()),
                "linear_speed_mps": self._linear,
                "yaw_rate_radps": self._yaw_rate,
                "settled_for_s": self._settled_for,
                "fault": None,
            }

    def set_goal(self, request_id: str, goal: PoseStamped) -> bool:
        with self._lock:
            if (
                not request_id
                or request_id in self._retired_requests
                or goal.frame_id != self.config.odom_frame
                or not self._monitor.snapshot(now=monotonic()).ready
                or (self._request_id not in (None, request_id) and not self._cancelled)
            ):
                return False
            if self._request_id is not None and self._request_id != request_id:
                self._retired_requests.add(self._request_id)
            self._request_id = request_id
            self._accepted = self._cancelled = False
            self._state = None
        assert self._planner_spec is not None
        accepted = self._planner_spec.set_goal(goal) is True
        with self._lock:
            self._accepted = accepted
        return accepted

    def cancel(self, request_id: str, *, previous_request_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            if request_id in self._retired_requests or (
                self._request_id not in (None, request_id)
                and not (
                    self._cancelled
                    and self._state == "idle"
                    and previous_request_id == self._request_id
                )
            ):
                result = self.peer_status()
                result.update(cancelled=False, fault="request_mismatch")
                return result
            if self._request_id is not None and self._request_id != request_id:
                self._retired_requests.add(self._request_id)
            self._request_id = request_id
            self._cancelled = False
        assert self._planner_spec is not None
        succeeded = self._planner_spec.cancel_goal() is True
        state = self._planner_spec.get_state()
        with self._lock:
            self._state = state.value
            self._cancelled = succeeded and state == NavigationState.IDLE
        return self.peer_status()

    def _control(self, request: dict[str, Any]) -> dict[str, Any]:
        operation, request_id = request["operation"], request["request_id"]
        if operation == "goal":
            accepted = self.set_goal(request_id, decode_goal(request["payload"]))
            result = self.peer_status()
            result.update(accepted=accepted, fault=None if accepted else "goal_rejected")
            return result
        if operation == "cancel":
            return self.cancel(
                request_id, previous_request_id=request["payload"].get("previous_request_id")
            )
        if request_id != self._request_id:
            result = self.peer_status()
            result.update(cancelled=False, fault="request_mismatch")
            return result
        return self.peer_status(refresh_state=True)

    def _serve(self) -> None:
        worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="sonic-planner-rpc")
        pending: tuple[bytes, dict[str, Any], Future[dict[str, Any]]] | None = None
        try:
            with zmq.Context() as ctx, ctx.socket(zmq.ROUTER) as sock:
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.SNDHWM, 2)
                sock.setsockopt(zmq.RCVHWM, 2)
                sock.setsockopt(zmq.MAXMSGSIZE, MAX_FRAME_BYTES)
                sock.bind(self.config.link_bind)
                self._bound_endpoint = sock.getsockopt_string(zmq.LAST_ENDPOINT)
                self._started.set()
                while not self._stopping.is_set():
                    if pending is not None and pending[2].done():
                        identity, request, future = pending
                        pending = None
                        try:
                            result = future.result()
                        except Exception as exc:
                            result = self.peer_status()
                            result.update(accepted=False, cancelled=False, fault=type(exc).__name__)
                        self._reply(sock, identity, request, encode_control_payload(result))
                    if not sock.poll(10):
                        continue
                    parts = sock.recv_multipart()
                    if len(parts) != 2:
                        continue
                    identity, packet = parts
                    try:
                        request = decode_navigation_message(packet)
                    except ValueError:
                        continue
                    if request["reply"]:
                        continue
                    if request["operation"] == "poll":
                        payload = encode_poll_payload(self.poll_reply(request["correlation_id"]))
                        self._reply(sock, identity, request, payload)
                    elif pending is None:
                        pending = identity, request, worker.submit(self._control, request)
                    else:
                        result = self.peer_status()
                        result.update(accepted=False, cancelled=False, fault="control_busy")
                        self._reply(sock, identity, request, encode_control_payload(result))
        except Exception as exc:
            self._error = exc
            self._started.set()
        finally:
            worker.shutdown(wait=False, cancel_futures=True)

    @staticmethod
    def _reply(
        sock: zmq.Socket[Any], identity: bytes, request: dict[str, Any], payload: dict[str, Any]
    ) -> None:
        reply = dict(request, reply=True, payload=payload)
        try:
            sock.send_multipart([identity, encode_navigation_message(reply)], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass  # No receipt acknowledgment; the caller's deadline still expires.
