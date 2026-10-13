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

"""Pure planner wire encoding and held body-frame navigation intent.

No sockets, mode ownership or robot side effects live here. Guarded extensions
are supported only by the matching private native navigation runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from numbers import Real
import struct
from typing import Any, Literal

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback

HEADER_SIZE = 1280
Vector = tuple[float, float, float]


class NavigationVelocityError(ValueError):
    """A finite source velocity outside the admitted navigation envelope."""


@dataclass(frozen=True, init=False)
class NavigationInput:
    """Atomic input with an owned velocity snapshot and execution-host deadlines.

    Twist messages are mutable. Store immutable numbers and return a fresh Twist
    on access so neither a publisher nor a callback can mutate the queued intent.
    """

    _velocity: tuple[float, float, float, float, float, float]
    source_sequence: int
    velocity_deadline: float
    readiness_deadline: float
    ready: bool
    fault: str | None

    def __init__(
        self,
        twist: Twist,
        source_sequence: int,
        velocity_deadline: float,
        readiness_deadline: float,
        ready: bool,
        fault: str | None,
    ) -> None:
        if not isinstance(twist, Twist):
            raise ValueError("a Twist snapshot is required")
        if type(source_sequence) is not int or not 0 <= source_sequence < 2**63:
            raise ValueError("source_sequence must be a nonnegative int64")
        if type(ready) is not bool or (
            fault is not None and (not isinstance(fault, str) or len(fault) > 256)
        ):
            raise ValueError("typed readiness and bounded fault text required")
        values = tuple(
            _finite(v, "velocity")
            for v in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )
        object.__setattr__(self, "_velocity", values)
        object.__setattr__(self, "source_sequence", source_sequence)
        object.__setattr__(
            self, "velocity_deadline", _finite(velocity_deadline, "velocity_deadline")
        )
        object.__setattr__(
            self, "readiness_deadline", _finite(readiness_deadline, "readiness_deadline")
        )
        object.__setattr__(self, "ready", ready)
        object.__setattr__(self, "fault", fault)

    @property
    def twist(self) -> Twist:
        return Twist(linear=Vector3(*self._velocity[:3]), angular=Vector3(*self._velocity[3:]))


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite numeric data")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite numeric data") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite numeric data")
    return result


def _f32(value: float) -> float:
    try:
        return float(struct.unpack("<f", struct.pack("<f", value))[0])
    except (struct.error, OverflowError) as exc:
        raise ValueError("motion must fit float32") from exc


def _vector(value: Vector, *, nonzero: bool) -> Vector:
    if len(value) != 3:
        raise ValueError("horizontal vector must have three values")
    x, y, z = (_f32(_finite(v, "vector")) for v in value)
    if z != 0 or (nonzero and not math.isclose(math.hypot(x, y), 1, abs_tol=1e-5)):
        raise ValueError("direction must be horizontal and normalized")
    return x, y, z


def _message(topic: bytes, fields: list[dict[str, Any]], payload: bytes) -> bytes:
    header = json.dumps(
        {"v": 1, "endian": "le", "count": 1, "fields": fields}, separators=(",", ":")
    ).encode("ascii")
    if len(header) > HEADER_SIZE:
        raise ValueError("planner header exceeds 1280 bytes")
    return topic + header.ljust(HEADER_SIZE, b"\0") + payload


def _guard(sequence: int, deadline: float) -> tuple[list[dict[str, Any]], bytes]:
    if type(sequence) is not int or not 0 <= sequence < 2**63:
        raise ValueError("sequence must be a nonnegative signed int64")
    deadline = _finite(deadline, "deadline")
    if deadline <= 0:
        raise ValueError("deadline must be positive")
    return [
        {"name": "planner_sequence", "dtype": "i64", "shape": [1]},
        {"name": "planner_deadline", "dtype": "f64", "shape": [1]},
    ], struct.pack("<qd", sequence, deadline)


@dataclass(frozen=True)
class PlannerIntent:
    mode: int
    movement: Vector
    facing: Vector
    speed: float
    height: float = -1.0


def encode_planner(
    intent: PlannerIntent, *, sequence: int | None = None, deadline: float | None = None
) -> bytes:
    """Encode legacy motion or sequence/deadline guarded motion, without sending."""
    if type(intent.mode) is not int or intent.mode not in (0, 1):
        raise ValueError("only IDLE/SLOW_WALK modes are admitted")
    movement = _vector(intent.movement, nonzero=intent.mode == 1)
    facing = _vector(intent.facing, nonzero=True)
    speed = _f32(_finite(intent.speed, "speed"))
    if intent.mode == 0:
        if movement != (0, 0, 0) or speed != 0:
            raise ValueError("IDLE requires exact-zero movement and speed")
    elif not _f32(0.1) <= speed <= _f32(0.2):
        raise ValueError("walking speed must be within commissioned 0.10-0.20 range")
    if _finite(intent.height, "height") != -1:
        raise ValueError("navigation height override is disabled")
    fields = [
        {"name": "mode", "dtype": "i32", "shape": [1]},
        {"name": "movement", "dtype": "f32", "shape": [3]},
        {"name": "facing", "dtype": "f32", "shape": [3]},
        {"name": "speed", "dtype": "f32", "shape": [1]},
        {"name": "height", "dtype": "f32", "shape": [1]},
    ]
    payload = struct.pack("<i8f", intent.mode, *movement, *facing, speed, -1.0)
    if (sequence is None) != (deadline is None):
        raise ValueError("sequence and deadline must appear together")
    if sequence is not None and deadline is not None:
        extra, data = _guard(sequence, deadline)
        fields.extend(extra)
        payload += data
    return _message(b"planner", fields, payload)


def encode_navigation_control(
    action: Literal["rearm", "end"], *, sequence: int, deadline: float
) -> bytes:
    """Extend mode-only command flags with a sequenced native navigation action."""
    if action not in ("rearm", "end"):
        raise ValueError("navigation control must be rearm or end")
    fields = [{"name": name, "dtype": "u8", "shape": [1]} for name in ("start", "stop", "planner")]
    fields.append({"name": "navigation_control", "dtype": "i32", "shape": [1]})
    extra, data = _guard(sequence, deadline)
    fields.extend(extra)
    return _message(
        b"command", fields, struct.pack("<BBBi", 0, 0, 1, 1 if action == "rearm" else 2) + data
    )


@dataclass(frozen=True)
class NavigationLimits:
    min_walk_speed: float = 0.10
    max_walk_speed: float = 0.20
    max_yaw_rate: float = 0.20
    turn_deadband: float = 0.01
    facing_lookahead: float = 0.50
    max_facing_offset: float = 0.10
    angular_threshold: float = 0.02
    speed_threshold: float = 0.01

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if _finite(value, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not 0.1 <= self.min_walk_speed <= self.max_walk_speed <= 0.2:
            raise ValueError("walking limits must be inside the commissioned envelope")
        if self.max_yaw_rate > 0.2 or self.max_facing_offset > 0.1:
            raise ValueError("turning limits exceed the commissioned envelope")
        if self.turn_deadband >= self.max_yaw_rate:
            raise ValueError("turn deadband must leave an admitted turning range")


def _yaw(quaternion: tuple[float, ...] | None) -> float:
    if quaternion is None or len(quaternion) != 4:
        raise ValueError("fresh measured/reference quaternion required")
    values = tuple(_finite(v, "quaternion") for v in quaternion)
    norm = math.hypot(*values)
    if norm == 0 or not math.isfinite(norm):
        raise ValueError("valid measured/reference quaternion required")
    w, x, y, z = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _heading(yaw: float) -> Vector:
    return _f32(math.cos(yaw)), _f32(math.sin(yaw)), 0.0


def _angle(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _direction_difference(a: Vector, b: Vector) -> float:
    if a == (0, 0, 0) or b == (0, 0, 0):
        return math.inf if a != b else 0.0
    return _angle(math.atan2(a[1], a[0]), math.atan2(b[1], b[0]))


class PlannerIntentTracker:
    """Update on fresh source intent; heartbeats reuse exact float32 state."""

    def __init__(self, limits: NavigationLimits) -> None:
        self.limits = limits
        self._intent = PlannerIntent(0, (0, 0, 0), (1, 0, 0), 0.0)
        self._facing_world: float | None = None
        self._reference_yaw: float | None = None
        self._stopped = True

    def heartbeat(self) -> PlannerIntent:
        return self._intent

    def stop(self, feedback: SonicFeedback | None) -> PlannerIntent:
        """Snapshot yaw on stop entry; missing feedback retains the held facing."""
        if feedback is not None and feedback.planner_reference_active is True:
            measured = _yaw(feedback.base_quat_measured)
            reference = _yaw(feedback.reference_heading_quat)
            entering = not self._stopped or self._facing_world is None
            reset = (
                self._reference_yaw is None
                or _angle(reference, self._reference_yaw) >= self.limits.angular_threshold
            )
            if entering:
                self._facing_world = measured
            if not entering and not reset:
                return self._intent
            assert self._facing_world is not None
            self._reference_yaw = reference
            facing = _heading(self._facing_world - reference)
        else:
            facing = self._intent.facing
        self._intent = PlannerIntent(0, (0, 0, 0), facing, 0.0)
        self._stopped = True
        return self._intent

    def update(
        self, twist: Twist, feedback: SonicFeedback, *, stopping: bool = False
    ) -> PlannerIntent:
        components = tuple(
            _finite(v, "Twist")
            for v in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )
        vx, vy, vz, wx, wy, wz = components
        if vy != 0 or vz != 0 or wx != 0 or wy != 0:
            raise NavigationVelocityError("navigation supports only forward/backward and yaw")
        if abs(vx) > self.limits.max_walk_speed or abs(wz) > self.limits.max_yaw_rate:
            raise NavigationVelocityError("navigation velocity exceeds commissioned limits")
        if stopping:
            return self.stop(feedback)
        if feedback.planner_reference_active is not True:
            raise ValueError("active planner reference frame required")
        measured = _yaw(feedback.base_quat_measured)
        reference = _yaw(feedback.reference_heading_quat)
        # Small follower corrections must hold facing even when measured yaw
        # drifts after a turn. Thresholding the resulting vector alone would
        # eventually chase that drift and trigger a new native replan.
        if abs(wz) <= self.limits.turn_deadband:
            wz = 0.0
        if abs(vx) < self.limits.min_walk_speed and wz == 0:
            return self.stop(feedback)
        if self._facing_world is None:
            self._facing_world = measured
        target = self._facing_world
        if wz != 0:
            lead = max(
                -self.limits.max_facing_offset,
                min(self.limits.max_facing_offset, wz * self.limits.facing_lookahead),
            )
            target = measured + lead
        moving = abs(vx) >= self.limits.min_walk_speed
        candidate = PlannerIntent(
            1 if moving else 0,
            _heading(measured + (math.pi if vx < 0 else 0) - reference) if moving else (0, 0, 0),
            _heading(target - reference),
            _f32(abs(vx)) if moving else 0.0,
        )
        previous = self._intent
        reset = (
            self._reference_yaw is None
            or _angle(reference, self._reference_yaw) >= self.limits.angular_threshold
        )
        transition = candidate.mode != previous.mode or self._stopped or reset
        change_facing = (
            transition
            or _direction_difference(candidate.facing, previous.facing)
            >= self.limits.angular_threshold
        )
        self._intent = PlannerIntent(
            candidate.mode,
            candidate.movement
            if transition
            or _direction_difference(candidate.movement, previous.movement)
            >= self.limits.angular_threshold
            else previous.movement,
            candidate.facing if change_facing else previous.facing,
            candidate.speed
            if transition or abs(candidate.speed - previous.speed) >= self.limits.speed_threshold
            else previous.speed,
        )
        if change_facing:
            self._facing_world = target
        if reset:
            self._reference_yaw = reference
        self._stopped = False
        return self._intent
