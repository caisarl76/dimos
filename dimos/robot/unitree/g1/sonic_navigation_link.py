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

"""Bounded private JSON link; only relative ages cross the host boundary."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
from time import monotonic
from typing import Any, Literal, Self
from uuid import uuid4

import zmq

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_readiness import (
    INPUT_BUDGETS,
    InputProgress,
    ReadinessSnapshot,
)

MAX_FRAME_BYTES = 65536
Operation = Literal["poll", "goal", "cancel", "status"]
_ENVELOPE = {"version", "operation", "correlation_id", "request_id", "reply", "payload"}
_CONTROL = {
    "request_id",
    "accepted",
    "cancelled",
    "state",
    "readiness",
    "linear_speed_mps",
    "yaw_rate_radps",
    "settled_for_s",
    "fault",
}


def _number(value: Any, *, nonnegative: bool = False) -> float:
    if type(value) not in (int, float):
        raise ValueError("finite JSON number required")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError("finite JSON number required") from exc
    if not math.isfinite(result) or (nonnegative and result < 0):
        raise ValueError(
            "finite nonnegative age required" if nonnegative else "finite number required"
        )
    return result


def _text(value: Any, *, optional: bool = False, limit: int = 256) -> str | None:
    if optional and value is None:
        return None
    if not isinstance(value, str) or not 0 < len(value) <= limit:
        raise ValueError("bounded nonempty text required")
    return value


def _keys(value: Any, expected: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError("exact JSON object schema required")
    return value


def _velocity(twist: Twist) -> list[float]:
    return [
        _number(v)
        for v in (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
    ]


def encode_readiness(snapshot: ReadinessSnapshot) -> dict[str, Any]:
    return {
        "ready": snapshot.ready,
        "fault": snapshot.fault,
        "inputs": [
            {
                "name": p.name,
                "source_stamp": p.source_stamp,
                "age": p.age,
                "upstream_lag": p.upstream_lag,
                "valid": p.valid,
            }
            for p in snapshot.inputs
        ],
    }


def decode_readiness(value: Any) -> ReadinessSnapshot:
    data = _keys(value, {"ready", "fault", "inputs"})
    if type(data["ready"]) is not bool or not isinstance(data["inputs"], list):
        raise ValueError("typed readiness required")
    fault = _text(data["fault"], optional=True)
    progress = []
    for item in data["inputs"]:
        p = _keys(item, {"name", "source_stamp", "age", "upstream_lag", "valid"})
        if p["name"] not in INPUT_BUDGETS or type(p["valid"]) is not bool:
            raise ValueError("allowlisted progress input required")
        progress.append(
            InputProgress(
                p["name"],
                _number(p["source_stamp"], nonnegative=True),
                _number(p["age"], nonnegative=True),
                _number(p["upstream_lag"], nonnegative=True),
                p["valid"],
            )
        )
    if len(progress) != len(INPUT_BUDGETS) or {p.name for p in progress} != set(INPUT_BUDGETS):
        raise ValueError("one progress record per required input required")
    if data["ready"] and (fault is not None or not all(p.valid for p in progress)):
        raise ValueError("inconsistent readiness proof")
    return ReadinessSnapshot(data["ready"], tuple(progress), fault)


@dataclass(frozen=True)
class NavigationPollReply:
    correlation_id: str
    request_id: str | None
    source_sequence: int
    twist: Twist | None
    source_age: float
    readiness: ReadinessSnapshot


def readiness_deadline(snapshot: ReadinessSnapshot, *, t0: float, t1: float) -> tuple[float, bool]:
    t0, t1 = _number(t0, nonnegative=True), _number(t1, nonnegative=True)
    if not 0 <= t1 - t0 <= 0.100 + 1e-12:
        raise ValueError("navigation round trip exceeds 100 ms")
    if len(snapshot.inputs) != len(INPUT_BUDGETS) or {p.name for p in snapshot.inputs} != set(
        INPUT_BUDGETS
    ):
        return t0, False
    deadlines = [
        t0 + INPUT_BUDGETS[p.name] - _number(p.age, nonnegative=True) for p in snapshot.inputs
    ]
    deadline = min(deadlines)
    ready = (
        snapshot.ready
        and snapshot.fault is None
        and all(
            p.valid and _number(p.upstream_lag, nonnegative=True) < INPUT_BUDGETS[p.name]
            for p in snapshot.inputs
        )
    )
    return deadline, ready and deadline > t1


def bound_navigation_input(reply: NavigationPollReply, *, t0: float, t1: float) -> NavigationInput:
    t0, t1 = _number(t0, nonnegative=True), _number(t1, nonnegative=True)
    if not 0 <= t1 - t0 <= 0.100 + 1e-12:
        raise ValueError("navigation round trip exceeds 100 ms")
    age = _number(reply.source_age, nonnegative=True)
    deadline, ready = readiness_deadline(reply.readiness, t0=t0, t1=t1)
    return NavigationInput(
        reply.twist or Twist(),
        reply.source_sequence,
        t0 if reply.twist is None else t0 + 0.350 - age,
        deadline,
        ready,
        reply.readiness.fault or (None if ready else "navigation_inputs_unready"),
    )


def encode_poll_payload(reply: NavigationPollReply) -> dict[str, Any]:
    return {
        "request_id": reply.request_id,
        "source_sequence": reply.source_sequence,
        "twist": None if reply.twist is None else _velocity(reply.twist),
        "source_age": reply.source_age,
        "readiness": encode_readiness(reply.readiness),
    }


def decode_poll_payload(value: Any, *, correlation_id: str) -> NavigationPollReply:
    data = _keys(value, {"request_id", "source_sequence", "twist", "source_age", "readiness"})
    seq = data["source_sequence"]
    if type(seq) is not int or not 0 <= seq < 2**63:
        raise ValueError("nonnegative source int64 required")
    twist = None
    if data["twist"] is not None:
        if not isinstance(data["twist"], list) or len(data["twist"]) != 6:
            raise ValueError("six finite velocity components required")
        values = [_number(v) for v in data["twist"]]
        twist = Twist(linear=Vector3(*values[:3]), angular=Vector3(*values[3:]))
    return NavigationPollReply(
        correlation_id,
        _text(data["request_id"], optional=True, limit=64),
        seq,
        twist,
        _number(data["source_age"], nonnegative=True),
        decode_readiness(data["readiness"]),
    )


def goal_payload(goal: PoseStamped) -> dict[str, Any]:
    return {
        "frame_id": goal.frame_id,
        "pose": [
            goal.position.x,
            goal.position.y,
            goal.position.z,
            goal.orientation.x,
            goal.orientation.y,
            goal.orientation.z,
            goal.orientation.w,
        ],
    }


def decode_goal(value: Any) -> PoseStamped:
    data = _keys(value, {"frame_id", "pose"})
    _text(data["frame_id"])
    if not isinstance(data["pose"], list) or len(data["pose"]) != 7:
        raise ValueError("seven finite pose components required")
    values = [_number(v) for v in data["pose"]]
    norm = math.hypot(*values[3:])
    if norm < 1e-6 or abs(norm - 1.0) > 1e-3:
        raise ValueError("unit goal orientation required")
    return PoseStamped(
        ts=0.0, frame_id=data["frame_id"], position=Vector3(*values[:3]), orientation=values[3:]
    )


def encode_control_payload(status: dict[str, Any]) -> dict[str, Any]:
    result = dict(status)
    result["readiness"] = encode_readiness(status["readiness"])
    return result


def decode_control_payload(value: Any) -> dict[str, Any]:
    data = dict(_keys(value, _CONTROL))
    _text(data["request_id"], optional=True, limit=64)
    _text(data["fault"], optional=True)
    if type(data["accepted"]) is not bool or type(data["cancelled"]) is not bool:
        raise ValueError("explicit control acknowledgment required")
    if data["state"] not in (None, "idle", "following_path", "recovery"):
        raise ValueError("known planner state required")
    for name in ("linear_speed_mps", "yaw_rate_radps", "settled_for_s"):
        if data[name] is not None:
            data[name] = _number(data[name], nonnegative=name != "yaw_rate_radps")
    data["readiness"] = decode_readiness(data["readiness"])
    return data


def _validate_message(message: Any) -> dict[str, Any]:
    data = _keys(message, _ENVELOPE)
    if type(data["version"]) is not int or data["version"] != 1:
        raise ValueError("navigation link version 1 required")
    if (
        data["operation"] not in ("poll", "goal", "cancel", "status")
        or type(data["reply"]) is not bool
    ):
        raise ValueError("allowlisted operation and explicit reply marker required")
    _text(data["correlation_id"], limit=64)
    _text(data["request_id"], optional=True, limit=64)
    if data["reply"]:
        if data["operation"] == "poll":
            decode_poll_payload(data["payload"], correlation_id=data["correlation_id"])
        else:
            decode_control_payload(data["payload"])
    elif data["operation"] == "goal":
        if data["request_id"] is None:
            raise ValueError("explicit goal request required")
        decode_goal(data["payload"])
    elif data["operation"] == "cancel":
        if data["request_id"] is None:
            raise ValueError("explicit cancellation request required")
        if data["payload"]:
            payload = _keys(data["payload"], {"previous_request_id"})
            _text(payload["previous_request_id"], limit=64)
        else:
            _keys(data["payload"], set())
    else:
        _keys(data["payload"], set())
    return data


def encode_navigation_message(message: dict[str, Any]) -> bytes:
    packet = json.dumps(_validate_message(message), allow_nan=False, separators=(",", ":")).encode()
    if len(packet) > MAX_FRAME_BYTES:
        raise ValueError("navigation frame exceeds 64 KiB")
    return packet


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def decode_navigation_message(packet: bytes) -> dict[str, Any]:
    if not isinstance(packet, bytes) or len(packet) > MAX_FRAME_BYTES:
        raise ValueError("one bounded navigation frame required")
    try:
        return _validate_message(json.loads(packet, object_pairs_hook=_unique_object))
    except (UnicodeError, RecursionError, TypeError) as exc:
        raise ValueError("valid bounded navigation JSON required") from exc


class SonicNavigationLink:
    """Independent bounded poll/control slots, with sockets owned by each caller.

    Socket lifetimes end at every timeout. A reconnect never carries queued old
    requests forward, and no blocked poll owns the cancellation slot.
    """

    def __init__(self, endpoint: str) -> None:
        if (
            not isinstance(endpoint, str)
            or not endpoint.startswith("tcp://")
            or len(endpoint) > 2048
        ):
            raise ValueError("explicit unicast TCP endpoint required")
        self._endpoint = endpoint
        self._context = zmq.Context()
        self._closed = threading.Event()
        self._poll_slot = threading.Lock()
        self._control_slot = threading.Lock()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def close(self) -> None:
        self._closed.set()
        with self._poll_slot, self._control_slot:
            if not self._context.closed:
                self._context.term()

    def _request(
        self, operation: Operation, request_id: str | None, payload: dict[str, Any], timeout: float
    ) -> tuple[dict[str, Any], float, float]:
        slot = self._poll_slot if operation == "poll" else self._control_slot
        if not slot.acquire(blocking=False):
            raise RuntimeError("navigation request slot is busy")
        try:
            if self._closed.is_set():
                raise RuntimeError("navigation link is closed")
            t0 = monotonic()
            correlation = uuid4().hex
            request = {
                "version": 1,
                "operation": operation,
                "correlation_id": correlation,
                "request_id": request_id,
                "reply": False,
                "payload": payload,
            }
            packet = encode_navigation_message(request)
            with self._context.socket(zmq.DEALER) as sock:
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.SNDHWM, 1)
                sock.setsockopt(zmq.RCVHWM, 2)
                sock.setsockopt(zmq.MAXMSGSIZE, MAX_FRAME_BYTES)
                sock.connect(self._endpoint)
                sock.send(packet, flags=zmq.NOBLOCK)
                for _ in range(64):
                    remaining = t0 + timeout - monotonic()
                    if (
                        remaining <= 0
                        or self._closed.is_set()
                        or not sock.poll(math.ceil(remaining * 1000))
                    ):
                        raise TimeoutError("navigation peer did not reply within its bounded slot")
                    parts = sock.recv_multipart()
                    t1 = monotonic()
                    if len(parts) != 1 or t1 > t0 + timeout:
                        continue
                    try:
                        reply = decode_navigation_message(parts[0])
                    except ValueError:
                        continue
                    if (
                        reply["reply"]
                        and reply["operation"] == operation
                        and (
                            reply["correlation_id"] == correlation
                            and reply["request_id"] == request_id
                        )
                    ):
                        return reply, t0, t1
                raise ValueError("navigation receive budget exceeded")
        finally:
            slot.release()

    def poll(self, request_id: str | None) -> tuple[NavigationPollReply, float, float]:
        reply, t0, t1 = self._request("poll", request_id, {}, 0.100)
        return decode_poll_payload(reply["payload"], correlation_id=reply["correlation_id"]), t0, t1

    def control(
        self,
        operation: Literal["goal", "cancel", "status"],
        request_id: str | None,
        goal: PoseStamped | None = None,
        *,
        previous_request_id: str | None = None,
    ) -> tuple[dict[str, Any], float, float]:
        if operation == "goal" and goal is None:
            raise ValueError("a goal pose is required")
        payload = goal_payload(goal) if goal is not None else {}
        if previous_request_id is not None:
            if operation != "cancel":
                raise ValueError("previous request binding is only valid for cancellation")
            payload = {"previous_request_id": previous_request_id}
        reply, t0, t1 = self._request(operation, request_id, payload, 0.500)
        return decode_control_payload(reply["payload"]), t0, t1
