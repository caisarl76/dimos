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

from dataclasses import replace
import json
import math
import struct

import msgpack
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_feedback import decode_feedback
from dimos.robot.unitree.g1.sonic_navigation import (
    NavigationLimits,
    PlannerIntent,
    PlannerIntentTracker,
    encode_navigation_control,
    encode_planner,
)


def feedback(yaw=0.0, reference_yaw=0.0, active=True):
    payload = {
        "index": 1,
        "ros_timestamp": 0.0,
        "body_q": [0.0] * 29,
        "body_dq": [0.0] * 29,
        "left_hand_q": [0.0] * 7,
        "left_hand_dq": [0.0] * 7,
        "right_hand_q": [0.0] * 7,
        "right_hand_dq": [0.0] * 7,
        "base_quat_measured": [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
        "reference_heading_quat": [math.cos(reference_yaw / 2), 0, 0, math.sin(reference_yaw / 2)],
        "planner_reference_active": [int(active)],
    }
    return decode_feedback(
        b"g1_debug" + msgpack.packb(payload), received_at=10.0, received_monotonic=10.0
    )


def twist(vx=0.0, wz=0.0):
    return Twist(linear=Vector3(vx, 0, 0), angular=Vector3(0, 0, wz))


def test_planner_wire_contract():
    intent = PlannerIntent(1, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.2)
    legacy = encode_planner(intent)
    assert len(legacy) == 1323
    header = json.loads(legacy[7:1287].rstrip(b"\0"))
    assert header == {
        "v": 1,
        "endian": "le",
        "count": 1,
        "fields": [
            {"name": "mode", "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing", "dtype": "f32", "shape": [3]},
            {"name": "speed", "dtype": "f32", "shape": [1]},
            {"name": "height", "dtype": "f32", "shape": [1]},
        ],
    }
    assert legacy[1287:] == struct.pack("<i8f", 1, 1, 0, 0, 1, 0, 0, 0.2, -1)
    guarded = encode_planner(intent, sequence=12, deadline=10.23)
    assert len(guarded) == 1339
    assert struct.unpack("<qd", guarded[-16:]) == (12, 10.23)
    assert guarded[1287:1323] == legacy[1287:]
    # Strafe is codec-only, never admitted from navigation Twist.
    assert len(encode_planner(replace(intent, movement=(0, 1, 0)))) == 1323


@pytest.mark.parametrize(
    "kwargs",
    [
        {"sequence": 1},
        {"deadline": 10.0},
        {"sequence": -1, "deadline": 10.0},
        {"sequence": True, "deadline": 10.0},
        {"sequence": 2**63, "deadline": 10.0},
        {"sequence": 1, "deadline": math.inf},
    ],
)
def test_invalid_guard_envelope_rejected(kwargs):
    with pytest.raises(ValueError):
        encode_planner(PlannerIntent(0, (0, 0, 0), (1, 0, 0), 0), **kwargs)


@pytest.mark.parametrize(
    "change",
    [
        {"mode": 2},
        {"mode": True},
        {"speed": math.nan},
        {"speed": 0.21},
        {"height": 0.8},
        {"movement": (1, 0)},
        {"movement": (1, 0, 1)},
        {"facing": (0, 0, 0)},
    ],
)
def test_invalid_planner_intent_rejected(change):
    with pytest.raises(ValueError):
        encode_planner(replace(PlannerIntent(1, (1, 0, 0), (1, 0, 0), 0.2), **change))


def test_control_extension_keeps_start_stop_false():
    packet = encode_navigation_control("rearm", sequence=10, deadline=10.25)
    assert packet[1287:1290] == bytes([0, 0, 1])
    assert struct.unpack("<iqd", packet[1290:]) == (1, 10, 10.25)
    packet = encode_navigation_control("end", sequence=11, deadline=10.25)
    assert struct.unpack("<iqd", packet[1290:]) == (2, 11, 10.25)
    with pytest.raises(ValueError):
        encode_navigation_control("start", sequence=12, deadline=10.25)


@pytest.mark.parametrize("reference_yaw", [0.0, math.pi / 2, -1.2])
def test_forward_backward_reference_frame(reference_yaw):
    tracker = PlannerIntentTracker(NavigationLimits())
    state = feedback(0.0, reference_yaw)
    result = tracker.update(twist(0.2), state)
    assert result.movement == pytest.approx((math.cos(reference_yaw), -math.sin(reference_yaw), 0))
    assert result.speed == pytest.approx(0.2)
    backwards = tracker.update(twist(-0.2), state)
    assert backwards.movement == pytest.approx(
        (-math.cos(reference_yaw), math.sin(reference_yaw), 0)
    )
    assert backwards.speed == pytest.approx(0.2)


def test_intent_holds_heartbeat_and_stop_heading():
    tracker = PlannerIntentTracker(NavigationLimits())
    forward = tracker.update(twist(0.2), feedback(0.4))
    for _ in range(100):
        assert encode_planner(tracker.heartbeat()) == encode_planner(forward)
    stopped = tracker.stop(feedback(0.6))
    assert stopped.mode == 0 and stopped.speed == 0 and stopped.movement == (0, 0, 0)
    assert stopped.facing == pytest.approx((math.cos(0.6), math.sin(0.6), 0))
    assert tracker.update(twist(), feedback(0.61)).facing == stopped.facing
    assert tracker.stop(None).facing == stopped.facing


def test_pure_turn_and_tiny_speed_are_idle():
    tracker = PlannerIntentTracker(NavigationLimits())
    turning = tracker.update(twist(0, 0.2), feedback())
    assert turning.mode == 0 and turning.speed == 0 and turning.movement == (0, 0, 0)
    assert math.atan2(turning.facing[1], turning.facing[0]) == pytest.approx(0.1)
    tiny = tracker.update(twist(0.09), feedback())
    assert tiny.mode == 0 and tiny.speed == 0 and tiny.movement == (0, 0, 0)


def test_small_sign_changing_yaw_does_not_chatter():
    tracker = PlannerIntentTracker(NavigationLimits())
    held = tracker.update(twist(0.2, 0.005), feedback())
    for index in range(100):
        value = tracker.update(
            twist(0.2, -0.005 if index % 2 else 0.005), feedback(0.005 if index % 2 else -0.005)
        )
        assert value.facing == held.facing
        assert value.movement == held.movement


@pytest.mark.parametrize("vx", [0.0, 0.2])
def test_tiny_turn_intent_does_not_chase_measured_heading_drift(vx):
    tracker = PlannerIntentTracker(NavigationLimits())
    held = tracker.update(twist(vx, 0.005), feedback())
    for index in range(50):
        # Model the measured post-turn drift from the native noisy-trace run,
        # plus +/-0.005 rad sensor noise, while the follower crosses zero.
        measured = 0.001 * index + (0.005 if index % 2 else -0.005)
        command = twist(vx, -0.005 if index % 2 else 0.005)
        assert tracker.update(command, feedback(measured)).facing == held.facing


def test_commissioned_turn_rate_still_advances_facing_with_measured_turn():
    tracker = PlannerIntentTracker(NavigationLimits())
    first = tracker.update(twist(0, 0.1), feedback())
    assert math.atan2(first.facing[1], first.facing[0]) == pytest.approx(0.05)
    second = tracker.update(twist(0, 0.1), feedback(0.025))
    assert math.atan2(second.facing[1], second.facing[0]) == pytest.approx(0.075)


def test_reference_reset_reanchors_held_heading():
    tracker = PlannerIntentTracker(NavigationLimits())
    tracker.update(twist(0.2), feedback())
    result = tracker.update(twist(0.2), feedback(0.0, math.pi / 2))
    assert result.movement == pytest.approx((0, -1, 0), abs=1e-7)
    assert result.facing == pytest.approx((0, -1, 0), abs=1e-7)


@pytest.mark.parametrize(
    "bad",
    [
        twist(0.21),
        twist(0, 0.21),
        Twist(linear=Vector3(0, 0.1, 0)),
        Twist(angular=Vector3(0.1, 0, 0)),
        twist(math.nan),
    ],
)
def test_unsupported_navigation_velocity_rejected(bad):
    tracker = PlannerIntentTracker(NavigationLimits())
    with pytest.raises(ValueError):
        tracker.update(bad, feedback())


def test_missing_or_inactive_frame_blocks_movement():
    tracker = PlannerIntentTracker(NavigationLimits())
    with pytest.raises(ValueError):
        tracker.update(twist(0.2), feedback(active=False))
    with pytest.raises(ValueError):
        tracker.update(twist(0.2), replace(feedback(), reference_heading_quat=None))
    assert tracker.stop(None).mode == 0


def test_stopped_heading_ignores_small_reference_noise():
    tracker = PlannerIntentTracker(NavigationLimits())
    stopped = tracker.stop(feedback(0.6))
    for reference_yaw in [0.005, -0.005, 0.004]:
        assert tracker.update(twist(), feedback(0.61, reference_yaw)) == stopped
    reset = tracker.update(twist(), feedback(0.61, 0.4))
    assert reset.facing == pytest.approx((math.cos(0.2), math.sin(0.2), 0))
