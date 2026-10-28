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

import struct
from typing import Any

import numpy as np
import pytest

from dimos.robot.unitree.g1 import imported_gestures as gestures, tool_ardy_clip
from dimos.robot.unitree.g1.sonic_arm_reference import MOTOR_NAMES, RATE
from dimos.robot.unitree.g1.sonic_reference import encode_token_reference

DATA = gestures.DATA_DIR
# Their upper-body order: left and right interleaved, unlike MOTOR_NAMES.
ARM_NAMES = [
    f"{side}_{joint}"
    for joint in (
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    )
    for side in ("left", "right")
]
BASE = np.zeros(29)
BASE_HANDS = np.zeros(14)


def _gesture(*keyframes: dict[str, Any], return_s: float = 1.0) -> dict[str, Any]:
    return {"name": "synthetic", "return_s": return_s, "keyframes": [{"t": 0.0}, *keyframes]}


def test_arm_names_are_the_fourteen_arm_joints() -> None:
    assert sorted(ARM_NAMES) == sorted(MOTOR_NAMES[15:29])
    assert ARM_NAMES != MOTOR_NAMES[15:29]


def test_every_retained_gesture_loads() -> None:
    loaded = gestures.load_gestures()
    assert set(gestures.RETAINED + gestures.AUTHORED) == set(loaded)
    for name in gestures.RETAINED + gestures.AUTHORED:
        g = loaded[name]
        assert g.name == name
        keyframes = gestures.resolve(g, BASE, BASE_HANDS)
        assert len(keyframes) == len(g.keyframes) - 1  # the t=0 start pose is not emitted
        assert all(k.target.shape == (29,) and k.hands is not None for k in keyframes)


@pytest.mark.parametrize("name", gestures.RETAINED + gestures.AUTHORED)
def test_first_resolved_keyframe_moves_away_from_base(name: str) -> None:
    g = gestures.load_gestures()[name]
    base = np.zeros(29)
    base[MOTOR_NAMES.index("left_elbow")] = base[MOTOR_NAMES.index("right_elbow")] = 0.3
    first = gestures.resolve(g, base, BASE_HANDS)[0]
    assert not np.array_equal(first.target, base)
    assert first.min_s == g.keyframes[1].t


def test_joint_permutation_by_name() -> None:
    values = {name: 0.01 * (i + 1) for i, name in enumerate(ARM_NAMES)}
    g = gestures.parse_gesture(_gesture({"t": 1.0, "absolute": values}))
    (k,) = gestures.resolve(g, BASE, BASE_HANDS)
    for name, value in values.items():
        assert k.target[MOTOR_NAMES.index(name)] == value
    assert np.all(k.target[:15] == 0.0)
    # Deltas map the same way.
    g = gestures.parse_gesture(_gesture({"t": 1.0, "delta": values}))
    base = np.linspace(-0.2, 0.2, 29)
    (k,) = gestures.resolve(g, base, BASE_HANDS)
    for name, value in values.items():
        i = MOTOR_NAMES.index(name)
        assert k.target[i] == pytest.approx(base[i] + value)


def test_finger_packet_slots() -> None:
    # Distinct, float32-exact values inside each side's command limits, by finger name.
    left = {f: -(i + 1) / 16 for i, f in enumerate(gestures.HAND_ORDER)}
    left["thumb_2"] = 0.5  # left thumb_2 is non-negative
    left["thumb_1"] = 0.25
    left["thumb_0"] = 0.125
    right = {f: (i + 1) / 32 for i, f in enumerate(gestures.HAND_ORDER)}
    right["thumb_2"] = -0.5  # right thumb_2 is non-positive
    right["thumb_1"] = -0.375
    right["thumb_0"] = -0.0625
    g = gestures.parse_gesture(_gesture({"t": 1.0, "hands": {"left": left, "right": right}}))
    (k,) = gestures.resolve(g, BASE, np.full(14, 0.0))
    assert k.hands is not None
    packet = encode_token_reference(
        token=[0.0] * 64, left_hand=list(k.hands[:7]), right_hand=list(k.hands[7:]), frame_index=0
    )
    decoded_left = struct.unpack_from("<7f", packet, 1548)
    decoded_right = struct.unpack_from("<7f", packet, 1576)
    order = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
    assert gestures.HAND_ORDER == order
    for slot, finger in enumerate(order):
        assert decoded_left[slot] == left[finger]
        assert decoded_right[slot] == right[finger]


def test_unnamed_fingers_take_the_deploy_fist_and_hands_carry_forward() -> None:
    g = gestures.parse_gesture(
        _gesture(
            {"t": 0.5, "delta": {"left_elbow": 0.05}},
            {"t": 1.0, "hands": {"right": {"index_0": 0.0}}},
            {"t": 2.0, "delta": {"left_elbow": 0.1}},
        )
    )
    base_hands = np.full(14, 0.01)
    k0, k1, k2 = gestures.resolve(g, BASE, base_hands)
    assert k0.hands is not None and k1.hands is not None and k2.hands is not None
    assert np.array_equal(k0.hands, base_hands)
    expected = np.array([*gestures.HAND_DEFAULT["left"], *gestures.HAND_DEFAULT["right"]])
    expected[7 + gestures.HAND_ORDER.index("index_0")] = 0.0
    assert np.array_equal(k1.hands, expected)
    assert np.array_equal(k2.hands, expected)


def test_timing_and_holds() -> None:
    loaded = gestures.load_gestures()
    g = loaded["support_person"]
    base = np.zeros(29)
    keyframes = gestures.resolve(g, base, BASE_HANDS)
    assert [k.min_s for k in keyframes] == pytest.approx([2.0, 6.0])
    # Repeated identical keyframes are zero-displacement segments (holds).
    assert np.array_equal(keyframes[0].target, keyframes[1].target)
    assert g.return_s == 2.0


@pytest.mark.parametrize(
    "frame",
    [
        {"t": 1.0, "delta": {"waist_yaw": 0.1}},
        {"t": 1.0, "absolute": {"waist_pitch": 0.0}},
        {"t": 1.0, "delta": {"left_knee": 0.1}},
        {"t": 1.0, "delta": {"left_elbw": 0.1}},
        {"t": 1.0, "delay_s": {}},
        {"t": 1.0, "hands": {"right": {"ring_0": 0.0}}},
        {"t": 1.0, "hands": {"middle": {"index_0": 0.0}}},
        {"t": 1.0, "absolute": {"left_elbow": -1.2}},
        {"t": 1.0, "absolute": {"right_shoulder_roll": 1.7}},
        {"t": 1.0, "hands": {"right": {"index_0": -0.1}}},
        {"t": 1.0, "hands": {"left": {"thumb_2": -0.1}}},
        {"t": 0.5, "delta": {"left_elbow": float("nan")}},
    ],
)
def test_refusals(frame: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        gestures.parse_gesture(_gesture(frame))


@pytest.mark.parametrize(
    "start",
    [
        {"t": 0.0, "delta": {"left_elbow": 0.1}},
        {"t": 0.0, "absolute": {"left_elbow": 0.1}},
        {"t": 0.0, "hands": {"right": {"index_0": 0.0}}},
    ],
)
def test_start_pose_keyframe_must_not_move(start: dict[str, Any]) -> None:
    obj = _gesture({"t": 1.0, "delta": {"left_elbow": 0.1}})
    obj["keyframes"][0] = start
    with pytest.raises(ValueError, match="start pose"):
        gestures.parse_gesture(obj)
    obj["keyframes"][0] = {"t": 0.0, "delta": {"left_elbow": 0.0}}
    gestures.parse_gesture(obj)
    with pytest.raises(ValueError):
        gestures.parse_gesture({**obj, "keyframes": obj["keyframes"][:1]})


def test_unmoved_joint_beyond_its_limit_is_not_rechecked() -> None:
    g = gestures.load_gestures()["support_person"]
    base = np.zeros(29)
    base[MOTOR_NAMES.index("left_wrist_roll")] = gestures.UB_LIMITS["left_wrist_roll"][1] + 0.01
    keyframes = gestures.resolve(g, base, BASE_HANDS)
    assert all(
        k.target[MOTOR_NAMES.index("left_wrist_roll")] == base[MOTOR_NAMES.index("left_wrist_roll")]
        for k in keyframes
    )


def test_waist_zero_delta_is_allowed_and_unknown_top_level_key_refused() -> None:
    gestures.parse_gesture(_gesture({"t": 1.0, "delta": {"waist_yaw": 0.0}}))
    with pytest.raises(ValueError, match="aliases"):
        gestures.parse_gesture({**_gesture({"t": 1.0}), "aliases": ["x"]})
    with pytest.raises(ValueError):
        gestures.parse_gesture(
            _gesture({"t": 0.0, "delta": {"left_elbow": 0.1}})
        )  # t not ascending


def test_out_of_limit_messages_name_joint_value_and_limit() -> None:
    with pytest.raises(ValueError, match=r"left_elbow.*-1\.2.*-1\.0472"):
        gestures.parse_gesture(_gesture({"t": 1.0, "absolute": {"left_elbow": -1.2}}))
    with pytest.raises(ValueError, match=r"index_0.*-0\.1"):
        gestures.parse_gesture(_gesture({"t": 1.0, "hands": {"right": {"index_0": -0.1}}}))


def test_resolved_target_refusal_support_person() -> None:
    g = gestures.load_gestures()["support_person"]
    base = np.zeros(29)
    base[MOTOR_NAMES.index("left_elbow")] = -0.8
    base[MOTOR_NAMES.index("right_elbow")] = -0.8
    with pytest.raises(ValueError, match=r"(left|right)_elbow.*-1\.3.*-1\.0472"):
        gestures.resolve(g, base, BASE_HANDS)


def test_doc_and_aliases_stripped() -> None:
    files = sorted(DATA.glob("*.json"))
    assert {f.stem for f in files} == set(gestures.RETAINED + gestures.AUTHORED)
    for f in files:
        text = f.read_text()
        assert "_doc" not in text and "aliases" not in text


def test_source_md_complete() -> None:
    text = (DATA / "SOURCE.md").read_text()
    assert "3430faa0059b656f6dba665fbdeeed7ed3899f45" in text
    assert "physical_ai_agent" in text
    assert "motion_control/motion_control/gestures/data" in text
    assert "2026-10-23" in text
    assert "Dropped" in text
    assert "Hand-authored" in text and all(f"`{n}`" in text for n in gestures.AUTHORED)
    assert "ARDY clips" in text and all(n in text for n in gestures.CLIP_GESTURES)


def _named(name: str, frame: dict[str, Any]) -> dict[str, Any]:
    return {**_gesture(frame), "name": name}


@pytest.mark.parametrize(
    ("name", "joint"), [("invite", "waist_yaw"), ("thanks", "waist_pitch"), ("bow", "waist_pitch")]
)
def test_waist_gestures_may_set_their_waist_joint(name: str, joint: str) -> None:
    g = gestures.parse_gesture(_named(name, {"t": 1.0, "absolute": {joint: 0.349}}))
    (k,) = gestures.resolve(g, BASE, BASE_HANDS)
    assert k.target[MOTOR_NAMES.index(joint)] == 0.349


@pytest.mark.parametrize("name", ["invite", "thanks", "bow"])
def test_waist_roll_is_refused_even_for_the_waist_gestures(name: str) -> None:
    with pytest.raises(ValueError, match="moves the waist"):
        gestures.parse_gesture(_named(name, {"t": 1.0, "absolute": {"waist_roll": 0.1}}))


@pytest.mark.parametrize(
    "name",
    [n for n in gestures.RETAINED + gestures.AUTHORED if n not in gestures.WAIST_GESTURES],
)
def test_other_gestures_still_refuse_waist(name: str) -> None:
    for joint in ("waist_yaw", "waist_pitch"):
        with pytest.raises(ValueError, match="moves the waist"):
            gestures.parse_gesture(_named(name, {"t": 1.0, "absolute": {joint: 0.1}}))


@pytest.mark.parametrize(
    ("name", "other"), [("invite", "waist_pitch"), ("thanks", "waist_yaw"), ("bow", "waist_yaw")]
)
def test_gesture_may_not_set_the_other_waist_joint(name: str, other: str) -> None:
    with pytest.raises(ValueError, match="moves the waist"):
        gestures.parse_gesture(_named(name, {"t": 1.0, "absolute": {other: 0.1}}))


@pytest.mark.parametrize("value", [0.41, -0.41, 4.0])
def test_waist_over_the_cap_is_refused(value: float) -> None:
    with pytest.raises(ValueError, match=r"waist_yaw.*0\.4"):
        gestures.parse_gesture(_named("invite", {"t": 1.0, "absolute": {"waist_yaw": value}}))


def test_waist_delta_is_refused_for_invite() -> None:
    with pytest.raises(ValueError, match="moves the waist"):
        gestures.parse_gesture(_named("invite", {"t": 1.0, "delta": {"waist_yaw": 0.1}}))


@pytest.mark.parametrize("name", ["invite", "thanks", "synthetic"])
def test_zero_waist_delta_is_still_allowed(name: str) -> None:
    gestures.parse_gesture(_named(name, {"t": 1.0, "delta": {"waist_yaw": 0.0}}))


def test_convert_resamples_slows_and_picks_the_arm_columns() -> None:
    t = np.arange(150) / 25
    qpos = np.zeros((150, 36))
    # a smooth bump (sin^2 window) so the spline does not ring at kinks
    window = np.sin(np.pi / 2 * np.clip(np.minimum(t - 1, 3 - t) / 0.1, 0, 1)) ** 2
    qpos[:, 7 + 15] = 0.6 * window * np.sin(2 * np.pi * (t - 1))
    qpos[:, 7 + 0] = 1.0  # a leg column: never copied
    arms = tool_ardy_clip.convert(qpos, 0.5)
    assert arms.shape[1] == 14 and np.all(arms[:, 1:] == 0)
    moving_s = np.count_nonzero(np.abs(np.diff(arms[:, 0])) * RATE > 0.05) / RATE
    assert moving_s == pytest.approx(4.0, abs=0.1)  # 2 s of motion at 0.5x
    assert len(arms) / RATE == pytest.approx(
        4.0 + 2 * tool_ardy_clip.SETTLE_S + tool_ardy_clip.EASE_S, abs=0.1
    )  # trimmed, plus the ease


def test_convert_eases_in_and_out_to_rest() -> None:
    t = np.arange(76) / 25
    qpos = np.zeros((76, 36))
    qpos[:, 7 + 15] = t  # 1 rad/s for 3 s
    arms = tool_ardy_clip.convert(qpos, 1.0)
    steps = np.abs(np.diff(arms[:, 0])) * RATE
    assert steps[0] < 0.05 and steps[-1] < 0.05
    assert np.all(steps[25:-25] == pytest.approx(1.0, abs=0.02))
    assert arms[0, 0] == pytest.approx(0.0, abs=1e-6)
    assert arms[-1, 0] == pytest.approx(2.98, abs=1e-6)  # the last 50 Hz row inside the source


def test_every_clip_gesture_loads_at_50_hz_within_limits() -> None:
    clips = gestures.load_clips()
    assert set(clips) == set(gestures.CLIP_GESTURES)
    for name, rows in clips.items():
        assert rows.shape[1] == 14 and len(rows) > RATE, name
        for j, joint in enumerate(gestures.CLIP_COLUMNS):
            low, high = gestures.UB_LIMITS[joint]
            assert low <= rows[:, j].min() and rows[:, j].max() <= high, (name, joint)


@pytest.mark.parametrize(
    "bad, match",
    [("header", "columns"), ("limit", "left_elbow"), ("nan", "finite"), ("short", "rows")],
)
def test_clip_refusals(bad: str, match: str) -> None:
    header = ",".join(gestures.CLIP_COLUMNS)
    row = ",".join("0" for _ in gestures.CLIP_COLUMNS)
    cols = list(gestures.CLIP_COLUMNS)
    lines = [header, row, row, row]
    if bad == "header":
        lines[0] = ",".join(reversed(cols))
    elif bad == "limit":
        vals = ["0"] * 14
        vals[cols.index("left_elbow")] = "5.0"
        lines[2] = ",".join(vals)
    elif bad == "nan":
        vals = ["0"] * 14
        vals[3] = "nan"
        lines[2] = ",".join(vals)
    else:
        lines = lines[:2]
    with pytest.raises(ValueError, match=match):
        gestures.parse_clip("\n".join(lines) + "\n", "synthetic")
