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

"""Pure keyframe planning, bounded retreats and stationary session splits."""

import math
import os
from pathlib import Path
import time
from typing import get_args

import numpy as np
import pytest

from dimos.robot.unitree.g1 import sonic_arm_reference
from dimos.robot.unitree.g1.sonic_arm_reference import (
    AIM_RANGE_M,
    FOREARM_PREF,
    FOREARM_WEIGHTS,
    HEAD_CAMERA_XYZ,
    MOTOR_NAMES,
    PEAK_SPEED,
    POINT_AZIMUTH_DEG,
    POINT_ELEVATION_DEG,
    RATE,
    ArmCollisionChecker,
    ArmKinematics,
    Keyframe,
    SonicEncoder,
    hand_rows,
    keyframe_plan,
    min_jerk,
    pixel_ray,
    retreat_hands,
    retreat_poses,
    retreat_route,
    stroke_plan,
    two_arm_margins,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    GESTURE_MAX_S,
    GESTURES,
    LEFT_ARM_GESTURES,
    GestureName,
    chunk_rows,
    gesture_keyframes,
    gesture_plan_inputs,
    token_still,
)
from dimos.robot.unitree.g1.sonic_test_support import ROBOT_IDLE, SCENE, needs_scene

BASE = np.zeros(29)
BASE[[15, 16, 18]] = 0.19, 0.19, 0.59
ENCODER = Path(os.environ.get("GEAR_SONIC_ENCODER", "/nonexistent"))
needs_encoder = pytest.mark.skipif(
    not ENCODER.is_file(), reason="GEAR_SONIC_ENCODER is unset or missing"
)


def kf(pitch, roll, yaw, elbow, hold=0.0):
    target = BASE.copy()
    target[[15, 16, 17, 18]] = pitch, roll, yaw, elbow
    return Keyframe(target, hold)


def plan_keyframe_holds(plan):
    return [
        (after.start - before.end) / RATE
        for before, after in zip(plan.segments, plan.segments[1:], strict=False)
    ]


@pytest.mark.parametrize("joint,delta", [(15, -0.35), (16, 0.7), (18, -1.05), (15, -2.99)])
def test_one_keyframe_is_a_symmetric_stroke_at_or_under_the_peak_speed(joint, delta):
    displacement = np.zeros(29)
    displacement[joint] = delta
    plan = keyframe_plan(BASE, [Keyframe(BASE + displacement, 1.0)])
    np.testing.assert_array_equal(plan.poses, stroke_plan(BASE, displacement))
    np.testing.assert_array_equal(plan.poses, plan.poses[::-1])
    assert np.abs(np.diff(plan.poses, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert np.count_nonzero(np.all(plan.poses == BASE + displacement, axis=1)) == 1 + 50 + 1
    rows = math.ceil(max(2.0, 1.875 * abs(delta) / PEAK_SPEED) * RATE) + 1
    assert len(plan.poses) == 150 + 2 * rows
    assert (plan.segments[0].start, plan.segments[0].end) == (50, 50 + rows)


def test_every_segment_respects_the_peak_speed():
    plan = keyframe_plan(
        BASE, [kf(-0.3, 1.3, 0, 0), kf(-0.3, 1.3, -0.35, 0), kf(-0.3, 1.3, 0.35, 0)]
    )
    assert np.abs(np.diff(plan.poses, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert [(s.src, s.dst) for s in plan.segments] == [(-1, 0), (0, 1), (1, 2), (2, -1)]
    for segment in plan.segments:
        src = BASE if segment.src == -1 else plan.targets[segment.src]
        dst = BASE if segment.dst == -1 else plan.targets[segment.dst]
        np.testing.assert_array_equal(plan.poses[segment.start], src)
        np.testing.assert_array_equal(plan.poses[segment.end - 1], dst)


def test_every_retreat_of_one_keyframe_is_a_suffix_of_the_plan():
    displacement = np.zeros(29)
    displacement[15] = -0.35
    plan = keyframe_plan(BASE, [Keyframe(BASE + displacement, 1.0)])
    for s in range(len(plan.segments)):
        unit = retreat_poses(plan, s)
        np.testing.assert_array_equal(unit, plan.poses[-len(unit) :])
    home_rows = [row for row in range(len(plan.poses)) if retreat_route(plan, row) is None]
    assert home_rows == list(range(plan.segments[-1].start, len(plan.poses)))
    assert retreat_route(plan, 0) == (0, plan.segments[0].end - plan.segments[0].start)


def test_hold_after_the_last_keyframe_retreats_only_the_way_home():
    plan = keyframe_plan(BASE, [kf(-2.6, 0.45, 0.0, 1.2, hold=2.0)])
    row = plan.segments[0].end + 20
    assert retreat_route(plan, row) == (len(plan.segments) - 1, 0)
    assert len(retreat_poses(plan, len(plan.segments) - 1)) + 100 <= 1000


def test_stop_mid_swing_reverses_to_the_previous_keyframe_then_goes_home():
    plan = keyframe_plan(
        BASE, [kf(-0.3, 1.3, 0, 0), kf(-0.3, 1.3, -0.35, 0), kf(-0.3, 1.3, 0.35, 0)]
    )
    seg = plan.segments[2]
    row = (seg.start + seg.end) // 2
    s, offset = retreat_route(plan, row)
    unit = retreat_poses(plan, s)
    np.testing.assert_allclose(unit[offset], plan.poses[row])
    np.testing.assert_allclose(unit[offset + (row - seg.start)], plan.targets[1])
    np.testing.assert_allclose(unit[-1], BASE)
    assert len(unit) - offset < len(plan.poses) - row + 300
    assert np.abs(np.diff(unit, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9


def test_stop_in_a_hold_goes_straight_home():
    plan = keyframe_plan(BASE, [kf(-0.3, 1.3, 0, 0, hold=1.0), kf(-0.3, 1.3, -0.35, 0)])
    row = plan.segments[0].end + 10
    s, offset = retreat_route(plan, row)
    seg = plan.segments[1]
    assert (s, offset) == (1, seg.end - seg.start)
    np.testing.assert_allclose(retreat_poses(plan, s)[offset], plan.targets[0])


def test_retreats_keep_custom_speed_move_and_lead_durations():
    plan = keyframe_plan(
        BASE,
        [kf(-0.3, 0.3, 0, 0.5), kf(-0.5, 0.3, 0, 0.5)],
        lead_s=0.2,
        move_s=3.0,
        peak_speed=0.1,
    )
    unit = retreat_poses(plan, 1)
    reverse_rows = plan.segments[1].end - plan.segments[1].start
    home_rows = math.ceil(max(3.0, 1.875 * 0.49 / 0.1) * RATE) + 1
    assert len(unit) == reverse_rows + home_rows + 10
    assert np.abs(np.diff(unit, axis=0)).max() * RATE <= 0.1 + 1e-9
    np.testing.assert_array_equal(unit[-10:], np.tile(BASE, (10, 1)))


def test_long_sequences_get_pauses_only_where_a_split_needs_one():
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    split_every = 1000 - 25 - 100
    plan = keyframe_plan(BASE, swings, split_every=split_every)
    still_rows = np.nonzero(plan.still)[0]
    assert np.diff(still_rows).max() <= split_every
    assert 0 < sum(h > 0 for h in plan_keyframe_holds(plan)) < len(swings)


def test_the_final_lead_out_counts_toward_the_split_interval():
    # A short lead-out has no still rows of its own, so it extends the last chunk.
    pad, split_every = 100, 705
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 6]
    plan = keyframe_plan(BASE, swings, lead_s=0.2, split_every=split_every)
    assert len(plan.poses) - np.flatnonzero(plan.still)[-1] <= split_every
    mask = np.zeros(pad + len(plan.poses) + pad, dtype=bool)
    mask[pad : pad + len(plan.poses)] = plan.still
    assert all(end - start <= 1000 for start, end in chunk_rows(len(mask), mask))


@pytest.mark.parametrize("split_every", [301, 302, 900])
def test_split_intervals_include_the_first_still_row_after_arrival(split_every):
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    plan = keyframe_plan(BASE, swings, split_every=split_every)
    assert np.diff(np.flatnonzero(plan.still)).max() <= split_every


def test_split_lengthens_short_holds_and_rejects_an_oversize_segment():
    keyframes = [kf(-2.6, 0.45, 0, 1.2, hold=1.0)]
    plan = keyframe_plan(BASE, keyframes, split_every=600)
    assert plan_keyframe_holds(plan) == [1.24]
    assert plan.still.any()
    assert plan_keyframe_holds(keyframe_plan(BASE, keyframes)) == [1.0]
    with pytest.raises(ValueError, match="segment longer than a session chunk"):
        keyframe_plan(BASE, keyframes, split_every=300)


def test_pose_still_requires_the_whole_encoder_lookahead_and_gradient_window():
    plan = keyframe_plan(BASE, [kf(-0.3, 0.3, 0, 0.5, hold=1.24)])
    arrival = plan.segments[0].end - 1
    np.testing.assert_array_equal(
        np.flatnonzero(plan.still), [arrival + 1, arrival + 2, arrival + 3]
    )


BASE_HANDS = np.linspace(-0.3, 0.3, 14)


def hands(**fingers):
    """Base hands with some of the 14 Dex3 slots set: hands(h3=1.0)."""
    values = BASE_HANDS.copy()
    for name, value in fingers.items():
        values[int(name[1:])] = value
    return values


def rows_for(seconds):
    return math.ceil(seconds * RATE) + 1


def test_hand_rows_follow_the_joint_min_jerk_profile():
    target = kf(-0.6, 0.3, 0, 0.6, hold=1.0).target
    hand_target = hands(h0=0.9, h9=-1.0)
    plan = keyframe_plan(BASE, [Keyframe(target, 1.0, hands=hand_target)], base_hands=BASE_HANDS)
    assert plan.hands.shape == (len(plan.poses), 14)
    assert hand_rows(plan) is plan.hands
    out, back = plan.segments
    profile = min_jerk(out.end - out.start)[:, None]
    np.testing.assert_allclose(
        plan.hands[out.start : out.end], BASE_HANDS + profile * (hand_target - BASE_HANDS)
    )
    np.testing.assert_allclose(plan.poses[out.start : out.end], BASE + profile * (target - BASE))
    assert (plan.hands[: out.start] == BASE_HANDS).all()
    assert (plan.hands[out.end - 1 : back.start + 1] == hand_target).all()
    assert (plan.hands[back.end - 1 :] == BASE_HANDS).all()
    np.testing.assert_array_equal(plan.hands, plan.hands[::-1])  # the return mirrors the way out


def test_a_finger_change_stretches_the_segment_to_the_peak_speed():
    target = kf(0.29, 0.19, 0, 0.59).target  # 0.1 rad of shoulder pitch: 2 s alone
    plan = keyframe_plan(
        BASE, [Keyframe(target, hands=hands(h4=BASE_HANDS[4] + 1.75))], base_hands=BASE_HANDS
    )
    for segment in plan.segments:
        assert segment.end - segment.start == rows_for(1.875 * 1.75 / PEAK_SPEED)
    assert np.abs(np.diff(plan.hands, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9


def test_a_keyframe_without_hands_keeps_the_previous_hand_target():
    held = hands(h2=0.5)
    plan = keyframe_plan(
        BASE,
        [Keyframe(kf(-0.3, 0.3, 0, 0.5).target, hands=held), kf(-0.5, 0.3, 0, 0.5)],
        base_hands=BASE_HANDS,
    )
    second = plan.segments[1]
    assert (plan.hands[second.start : second.end] == held).all()


def test_min_s_stretches_a_short_move_and_return_min_s_the_final_segment():
    target = kf(0.29, 0.19, 0, 0.59).target
    plan = keyframe_plan(BASE, [Keyframe(target, min_s=1.5)], move_s=0.5, return_min_s=2.5)
    out, back = plan.segments
    assert (out.end - out.start, back.end - back.start) == (rows_for(1.5), rows_for(2.5))
    plain = keyframe_plan(BASE, [Keyframe(target)], move_s=0.5).segments
    assert [s.end - s.start for s in plain] == [rows_for(0.5)] * 2


def test_retreat_hands_reverse_and_return_home_row_aligned_with_the_poses():
    keyframes = [
        Keyframe(kf(-0.3, 1.3, 0, 0).target, 0.5, hands=hands(h0=1.0)),
        Keyframe(kf(-0.3, 1.3, -0.35, 0).target),
        Keyframe(kf(-0.3, 1.3, 0.35, 0).target, hands=hands(h0=-0.5, h8=1.2), min_s=1.0),
    ]
    plan = keyframe_plan(BASE, keyframes, base_hands=BASE_HANDS, return_min_s=3.0)
    for s, seg in enumerate(plan.segments):
        poses, unit = retreat_poses(plan, s), retreat_hands(plan, s)
        assert unit.shape == (len(poses), 14), s
        np.testing.assert_array_equal(unit[-1], BASE_HANDS)
        np.testing.assert_array_equal(poses[-1], BASE)
        if seg.dst == -1:
            np.testing.assert_array_equal(unit, plan.hands[seg.start :])
            continue
        n = seg.end - seg.start
        np.testing.assert_array_equal(unit[:n], plan.hands[seg.start : seg.end][::-1])
        if seg.src >= 0:  # the direct way home moves joints and hands on one min-jerk profile
            home = slice(n, len(unit) - (len(plan.poses) - plan.segments[-1].end))
            assert home.stop - home.start >= rows_for(3.0)
            profile = min_jerk(home.stop - home.start)[:, None]
            src = plan.hands[seg.start]
            np.testing.assert_allclose(unit[home], src + profile * (BASE_HANDS - src))
        for rows in (poses, unit):
            assert np.abs(np.diff(rows, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9


def test_a_finger_only_move_is_never_still_but_the_following_hold_is():
    target = kf(-0.6, 0.3, 0, 0.6).target
    plan = keyframe_plan(
        BASE,
        [Keyframe(target, 2.0, hands=hands(h3=1.0)), Keyframe(target, 2.0, hands=hands(h3=-1.0))],
        base_hands=BASE_HANDS,
    )
    fingers, home = plan.segments[1], plan.segments[2]
    assert (plan.poses[fingers.start : fingers.end] == target).all()  # the joints do not move
    assert not plan.still[fingers.start : fingers.end].any()
    assert plan.still[fingers.end : home.start].any()


def test_without_hands_the_plan_has_zero_hand_rows_and_unchanged_poses():
    keyframes = [kf(-0.3, 1.3, 0, 0, hold=1.0), kf(-0.3, 1.3, -0.35, 0), kf(-1.2, 0.3, 0, 0.6)]
    plan = keyframe_plan(BASE, keyframes, split_every=600)
    np.testing.assert_array_equal(plan.hands, np.zeros((len(plan.poses), 14)))
    for s in range(len(plan.segments)):
        np.testing.assert_array_equal(
            retreat_hands(plan, s), np.zeros((len(retreat_poses(plan, s)), 14))
        )
    # Constant hands add no displacement, so the joint rows are byte for byte the same.
    steady = [Keyframe(k.target, k.hold_s, hands=BASE_HANDS) for k in keyframes]
    with_hands = keyframe_plan(BASE, steady, base_hands=BASE_HANDS, split_every=600)
    assert with_hands.poses.tobytes() == plan.poses.tobytes()
    assert with_hands.still.tobytes() == plan.still.tobytes()


@needs_scene
def test_retreats_keep_the_measured_start_contact_baseline():
    checker = ArmCollisionChecker(SCENE)
    raised = ROBOT_IDLE.copy()
    raised[15] = -1.5
    raised2 = raised.copy()
    raised2[16] = 0.6
    plan = keyframe_plan(ROBOT_IDLE, [Keyframe(raised, 1.0), Keyframe(raised2)])
    unit = retreat_poses(plan, 1)
    assert checker.first_contact(unit) is not None
    assert checker.first_contact(unit, start=ROBOT_IDLE) is None


@needs_scene
def test_a_wrist_resting_just_outside_reach_may_come_a_little_closer():
    """A wrist resting 2.0-2.1 cm from its hip (222's right wrist, often) passes a path that brings
    it 1 mm closer, as one resting at 1.97 cm does; one that comes 6 mm closer is a contact."""
    checker, ruler = ArmCollisionChecker(SCENE), ArmCollisionChecker(SCENE, margin=0.02)

    def at(roll_offset):
        pose = ROBOT_IDLE.copy()
        pose[16] += roll_offset  # left shoulder roll: out from the hip
        return pose

    def hip_gap(pose):
        return ruler.gaps(pose)["left_hip_roll_link"]

    rest, nudged, swung = at(0.01), at(0.007), at(-0.008)
    assert 0.020 < hip_gap(rest) < 0.0215 and hip_gap(nudged) < 0.020
    assert hip_gap(rest) - hip_gap(nudged) < 0.002
    assert checker.first_contact(np.array([rest, nudged]), every=1) is None
    assert checker.first_contact(np.array([rest, swung]), every=1) == "left_hip_roll_link"


BOTH_ARMS = ("left", "right")


def both_arms(left, right):
    """ROBOT_IDLE with shoulder pitch, roll, yaw and elbow of each arm set."""
    pose = ROBOT_IDLE.copy()
    pose[15:19] = left
    pose[22:26] = right
    return pose


@needs_scene
def test_two_arms_crossed_in_front_meet_each_other_not_the_body():
    # Shoulders yawed inward: the wrists meet 0.33 m ahead of the torso, well clear of it.
    crossed = both_arms([-1.0, 0.2, -0.8, 0.8], [-1.0, -0.2, 0.8, 0.8])
    path = stroke_plan(ROBOT_IDLE, crossed - ROBOT_IDLE)
    contact = ArmCollisionChecker(SCENE, arms=BOTH_ARMS).first_contact(path, start=ROBOT_IDLE)
    assert contact == "right_wrist_yaw_link (other arm)"
    # The left-arm checker sees the right arm as body, by name, as before.
    left = ArmCollisionChecker(SCENE, arms=("left",))
    assert left.first_contact(path, start=ROBOT_IDLE) == "right_wrist_yaw_link"


@needs_scene
def test_a_right_arm_swung_into_the_torso_is_reported():
    inward = np.zeros(29)
    inward[23] = 0.7  # right shoulder roll into the torso, the mirror of the left-arm check
    path = stroke_plan(ROBOT_IDLE, inward)
    both = ArmCollisionChecker(SCENE, arms=BOTH_ARMS)
    assert both.first_contact(path, start=ROBOT_IDLE) == "torso_link"
    assert ArmCollisionChecker(SCENE).first_contact(path, start=ROBOT_IDLE) is None  # left only


@needs_scene
def test_explicit_left_arm_matches_the_default_on_the_checker_cases():
    explicit, default = ArmCollisionChecker(SCENE, arms=("left",)), ArmCollisionChecker(SCENE)
    rest = ROBOT_IDLE.copy()
    rest[15:19] = rest[22:26] * [1, -1, 1, 1]
    cases = []
    for base, joint, delta in ((rest, 16, -0.7), (ROBOT_IDLE, 16, -0.25), (ROBOT_IDLE, 15, -1.5)):
        move = np.zeros(29)
        move[joint] = delta
        cases.append((stroke_plan(base, move), base))
    for poses, base in cases:
        for start in (None, base):
            assert explicit.first_contact(poses, start=start) == default.first_contact(
                poses, start=start
            )
    assert explicit.first_contact(cases[0][0]) == "torso_link"
    assert explicit.first_contact(cases[1][0]) == "left_hip_roll_link"


@needs_scene
@pytest.mark.parametrize("arms", [("left",), BOTH_ARMS])
@pytest.mark.parametrize("right_pitch", [ROBOT_IDLE[22], -1.0])
def test_left_wrist_on_the_right_shoulder_links_is_still_reported(arms, right_pitch):
    """An arm ignores only its own shoulder pitch and roll links, never the other arm's."""
    reach = both_arms([-2.4, 0.2, -1.5, -0.18], ROBOT_IDLE[22:26])  # left hand over the head
    reach[22] = right_pitch
    contact = ArmCollisionChecker(SCENE, arms=arms).first_contact(reach[None], start=ROBOT_IDLE)
    assert contact in ("right_shoulder_pitch_link", "right_shoulder_roll_link")


@needs_scene
@pytest.mark.parametrize("left_pitch", [ROBOT_IDLE[15], -1.0])
def test_right_wrist_on_the_left_shoulder_links_is_reported(left_pitch):
    reach = both_arms(ROBOT_IDLE[15:19], [-2.4, -0.2, 1.5, -0.18])  # the mirror pose
    reach[15] = left_pitch
    checker = ArmCollisionChecker(SCENE, arms=BOTH_ARMS)
    contact = checker.first_contact(reach[None], start=ROBOT_IDLE)
    assert contact in ("left_shoulder_pitch_link", "left_shoulder_roll_link")


@needs_scene
def test_the_measured_start_pose_has_no_false_contacts_with_both_arms():
    """At idle both arms touch their own shoulder links; only the real left wrist-to-hip gap
    (about 2 cm) is near, exactly as with the left arm alone."""
    both = ArmCollisionChecker(SCENE, arms=BOTH_ARMS)
    left = ArmCollisionChecker(SCENE, arms=("left",))
    assert both.gaps(ROBOT_IDLE) == left.gaps(ROBOT_IDLE)
    assert set(left.gaps(ROBOT_IDLE)) == {"left_hip_roll_link"}
    forward = both_arms(ROBOT_IDLE[15:19] - [0.4, 0, 0, 0], ROBOT_IDLE[22:26] - [0.4, 0, 0, 0])
    assert both.first_contact(stroke_plan(ROBOT_IDLE, forward - ROBOT_IDLE)) is None


def test_two_arm_margins_cover_both_arms():
    margins = two_arm_margins(0.15, 0.45)
    assert margins.shape == (29,)
    assert np.all(margins[:15] == 0.15)
    assert np.all(margins[15:29] == 0.45)


@needs_encoder
def test_a_split_pause_gives_still_tokens_for_the_session_blend():
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    plan = keyframe_plan(BASE, swings, split_every=300)
    tokens = SonicEncoder(ENCODER).encode(plan.poses)
    still = token_still(tokens)
    assert plan.still.any()
    for row in np.nonzero(plan.still)[0]:
        assert still[row]


@needs_encoder
def test_encoder_anchor_defaults_to_identity_and_is_used() -> None:
    encoder = SonicEncoder(ENCODER)
    target = BASE.copy()
    target[[15, 16, 17, 18]] = -0.5, 0.3, 0, 0.6
    poses = keyframe_plan(BASE, [Keyframe(target, 0.2)]).poses[:40]
    default = encoder.encode(poses)
    assert default.tobytes() == encoder.encode(poses, np.array([1.0, 0, 0, 1, 0, 0])).tobytes()
    tilted = np.array([math.cos(0.1), 0, 0, 1, -math.sin(0.1), 0])  # 0.1 rad pitch
    assert not np.allclose(default, encoder.encode(poses, tilted))


def test_token_still_compares_all_fifteen_rows_to_the_first_with_absolute_tolerance():
    tokens = np.ones((40, 64))
    tokens[15:30] += 0.5e-9
    tokens[30:, 20] += 2e-9
    np.testing.assert_array_equal(np.flatnonzero(token_still(tokens)), np.arange(16))
    assert not token_still(np.ones((14, 64))).any()


def test_chunks_split_only_where_still_and_fit_the_session():
    still = np.zeros(2600, bool)
    still[[900, 1700]] = True
    chunks = chunk_rows(2600, still)
    assert chunks == [(0, 925), (900, 1725), (1700, 2600)]
    assert all(e - s <= 1000 for s, e in chunks)
    with pytest.raises(ValueError, match="plan cannot be split into session chunks"):
        chunk_rows(2600, np.zeros(2600, bool))


def test_chunks_greedily_take_the_latest_still_row_with_room_for_overlap():
    still = np.zeros(1950, bool)
    still[[0, 900, 975, 976]] = True
    # A middle chunk may fill all 1000 rows; the last keeps 25 free for its finish hold.
    assert chunk_rows(1950, still) == [(0, 1000), (975, 1950)]
    assert chunk_rows(975, np.zeros(975, bool)) == [(0, 975)]
    with pytest.raises(ValueError, match="cannot be split"):
        chunk_rows(976, np.zeros(976, bool))


@pytest.mark.parametrize("name", get_args(GestureName))
def test_each_gesture_plans_within_the_limits(name):
    keyframes, move_s, return_min_s, _, _ = gesture_plan_inputs(name, BASE, np.zeros(14))
    plan = keyframe_plan(
        BASE,
        keyframes,
        base_hands=np.zeros(14),
        move_s=move_s,
        return_min_s=return_min_s,
        split_every=1000 - 25 - 100,
    )
    assert len(plan.poses) / RATE <= GESTURE_MAX_S
    assert all(len(retreat_poses(plan, s)) + 100 <= 1000 for s in range(len(plan.segments)))
    rows = np.hstack([plan.poses, plan.hands])  # fingers share the joints' peak speed
    assert np.abs(np.diff(rows, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert set(GESTURES) == set(get_args(GestureName))


def test_gesture_keyframes_move_only_the_left_arm_from_the_measured_base():
    base = ROBOT_IDLE.copy()
    for name in LEFT_ARM_GESTURES:
        for keyframe in gesture_keyframes(name, base):
            assert np.array_equal(
                np.delete(keyframe.target, [15, 16, 17, 18]), np.delete(base, [15, 16, 17, 18])
            )
    wave = gesture_keyframes("wave", base)
    assert len(wave) == 8 and np.allclose(wave[0].target[15:19], (-0.3, 1.3, 0.0, 0.0))
    assert [k.target[17] for k in wave[1:7]] == [-0.35, 0.35] * 3
    assert wave[-1].hold_s == 0.5 and np.array_equal(wave[-1].target, wave[0].target)


@needs_scene
@pytest.mark.parametrize("name", get_args(GestureName))
def test_each_gesture_and_its_retreats_clear_the_body(name):
    """With the gesture's own checker arms and timing, from the robot's measured idle."""
    keyframes, move_s, return_min_s, arms, envelope = gesture_plan_inputs(
        name, ROBOT_IDLE, np.zeros(14)
    )
    assert envelope == ("pose" if arms == ("left",) else "two_arm")
    checker = ArmCollisionChecker(SCENE, arms=arms)
    plan = keyframe_plan(
        ROBOT_IDLE, keyframes, base_hands=np.zeros(14), move_s=move_s, return_min_s=return_min_s
    )
    assert checker.first_contact(plan.poses, start=ROBOT_IDLE) is None
    for s in range(len(plan.segments)):
        assert checker.first_contact(retreat_poses(plan, s), start=ROBOT_IDLE) is None, s


def angle_between(a, b):
    """Great-circle angle between two (azimuth, elevation) directions, deg."""
    (az1, el1), (az2, el2) = np.radians(a), np.radians(b)
    cos = np.sin(el1) * np.sin(el2) + np.cos(el1) * np.cos(el2) * np.cos(az1 - az2)
    return math.degrees(math.acos(min(1.0, max(-1.0, cos))))


@pytest.fixture(scope="module")
def kinematics():
    return ArmKinematics(SCENE)


@needs_scene
# The brief's -30 and 100 are dropped: with yaw 0 the shoulder bounds cannot reach -30, or 100 high.
# Body contact is the path check's business, not the IK's.
@pytest.mark.parametrize("az", [0.0, 45.0, 80.0])
@pytest.mark.parametrize("el", [-70.0, -30.0, 0.0, 40.0, 70.0])
def test_solved_arm_points_where_asked(kinematics, az, el):
    q = ROBOT_IDLE.copy()
    q[[15, 16, 17, 18]] = kinematics.solve(ROBOT_IDLE, az, el)
    assert q[17] == 0.0 and q[18] == 1.35
    assert angle_between(kinematics.direction(q), (az, el)) <= 3.0


@needs_scene
def test_solve_refuses_a_direction_the_arm_cannot_reach(kinematics):
    with pytest.raises(ValueError, match="more than 3 deg off"):
        kinematics.solve(ROBOT_IDLE, -150.0, 0.0)  # behind the robot, across the torso


@needs_scene
def test_solve_is_fast(kinematics):
    # CPU time: the solver's own cost, not how long a loaded runner kept it waiting.
    started = time.process_time()
    kinematics.solve(ROBOT_IDLE, 45.0, -30.0)
    assert time.process_time() - started < 0.5


@needs_scene
@pytest.mark.parametrize("az,el", [(0, 0), (15, -30), (30, -45), (45, -30), (60, 0), (80, 30)])
def test_common_directions_solve_and_clear_the_body(kinematics, az, el):
    solved = ROBOT_IDLE.copy()
    solved[[15, 16, 17, 18]] = kinematics.solve(ROBOT_IDLE, az, el)
    assert angle_between(kinematics.direction(solved), (az, el)) <= 3.0
    plan = stroke_plan(ROBOT_IDLE, solved - ROBOT_IDLE)
    assert ArmCollisionChecker(SCENE).first_contact(plan, start=ROBOT_IDLE) is None


@needs_scene
def test_cone_grid_is_solved_or_refused_for_a_stated_reason(kinematics):
    """Every direction in the coarse bound clears the body, is out of reach, or meets the body."""
    checker = ArmCollisionChecker(SCENE)
    outcomes = {"clear": 0, "out_of_reach": 0, "contact": 0}
    for az in np.linspace(*POINT_AZIMUTH_DEG, 7):
        for el in np.linspace(*POINT_ELEVATION_DEG, 7):
            solved = ROBOT_IDLE.copy()
            try:
                solved[[15, 16, 17, 18]] = kinematics.solve(ROBOT_IDLE, az, el)
            except ValueError as exc:
                assert "more than 3 deg off" in str(exc)
                outcomes["out_of_reach"] += 1
                continue
            assert angle_between(kinematics.direction(solved), (az, el)) <= 3.0
            plan = stroke_plan(ROBOT_IDLE, solved - ROBOT_IDLE)
            contact = checker.first_contact(plan, start=ROBOT_IDLE)
            outcomes["clear" if contact is None else "contact"] += 1
    print(outcomes)
    assert outcomes["clear"] > outcomes["out_of_reach"] + outcomes["contact"]


def test_image_centre_looks_down_the_camera_axis():
    ray = pixel_ray(325.54, 245.35)
    np.testing.assert_allclose(ray, [np.cos(0.8306), 0.0, -np.sin(0.8306)], atol=2e-3)  # 47.6 deg


def test_pixel_left_and_up_turn_left_and_up():
    c = pixel_ray(325.54, 245.35)
    assert pixel_ray(100.0, 245.35)[1] > c[1]  # left of centre -> +y (robot's left)
    assert pixel_ray(325.54, 50.0)[2] > c[2]  # above centre -> higher


FOREARM_US = [0, 106, 213, 320, 426, 533, 639]
FOREARM_VS = [0, 80, 160, 240, 320, 400, 479]
SOLE_OFFSET_M = (
    0.035  # scene_43dof foot collision box: 5 mm half-height, centred 30 mm below the ankle
)


def with_arm(base, q4):
    pose = np.asarray(base, np.float64).copy()
    pose[15:19] = q4
    return pose


def floor_check(q):
    """(torso origin, torso rotation, floor height) in the MuJoCo world, computed independently."""
    import mujoco  # optional dependency, as in sonic_arm_reference

    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    d.qpos[2] = 0.79
    d.qpos[[m.jnt_qposadr[m.joint(n + "_joint").id] for n in MOTOR_NAMES]] = q
    mujoco.mj_kinematics(m, d)
    ankles = [d.xpos[m.body(f"{side}_ankle_roll_link").id][2] for side in ("left", "right")]
    torso = m.body("torso_link").id
    return d.xpos[torso].copy(), d.xmat[torso].reshape(3, 3).copy(), min(ankles) - SOLE_OFFSET_M


def path_clear_from(checker, base):
    return lambda pose: checker.first_contact(stroke_plan(base, pose - base), start=base)


@needs_scene
def test_forearm_covers_the_whole_image(kinematics):
    clear = path_clear_from(ArmCollisionChecker(SCENE), ROBOT_IDLE)
    rows, times = [], []
    for v in FOREARM_VS:
        for u in FOREARM_US:
            point, t = kinematics.aim_point(ROBOT_IDLE, u, v)
            assert AIM_RANGE_M[0] <= t <= AIM_RANGE_M[1]
            started = time.process_time()  # CPU time, as in test_solve_is_fast
            q4 = kinematics.solve_forearm(ROBOT_IDLE, point, clear)
            times.append(time.process_time() - started)
            pose = with_arm(ROBOT_IDLE, q4)
            error = kinematics.forearm_error(pose, point)
            rows.append((u, v, t, error, times[-1]))
            assert error <= 3.0, (u, v)
            assert kinematics.natural_pose(pose) is None, (u, v)
    for u, v, t, error, took in rows:
        print(f"u {u:3d} v {v:3d} ray {t:.2f} m error {error:.2f} deg solve {took:.2f} s")
    print(f"worst error {max(r[3] for r in rows):.2f} deg, slowest {max(times):.2f} s")
    assert float(np.median(times)) < 1.0


@needs_scene
def test_aim_point_ray_length_matches_the_camera_height(kinematics):
    origin, rotation, floor = floor_check(ROBOT_IDLE)
    for (u, v), length in [
        ((320, 240), 1.68),
        ((320, 479), 1.32),
        ((0, 0), 3.18),
        ((639, 0), 3.18),
    ]:
        point, t = kinematics.aim_point(ROBOT_IDLE, u, v)
        assert t == pytest.approx(length, abs=0.05), (u, v)
        np.testing.assert_allclose(point, np.array(HEAD_CAMERA_XYZ) + t * pixel_ray(u, v))
        assert (origin + rotation @ point)[2] == pytest.approx(floor, abs=1e-3)


@needs_scene
def test_aim_point_follows_the_measured_waist(kinematics):
    bent = ROBOT_IDLE.copy()
    bent[14] += 0.1  # waist pitch
    _, idle_t = kinematics.aim_point(ROBOT_IDLE, 320, 240)
    point, t = kinematics.aim_point(bent, 320, 240)
    assert abs(t - idle_t) > 0.02
    origin, rotation, floor = floor_check(bent)
    assert (origin + rotation @ point)[2] == pytest.approx(floor, abs=1e-3)


@needs_scene
@pytest.mark.parametrize("ray", [[1.0, 0.0, 0.0], [0.8, 0.0, 0.6]])
def test_aim_point_clamps_a_level_ray(kinematics, monkeypatch, ray):
    monkeypatch.setattr(sonic_arm_reference, "pixel_ray", lambda u, v: np.array(ray))
    point, t = kinematics.aim_point(ROBOT_IDLE, 320, 240)
    assert t == AIM_RANGE_M[1]
    assert np.all(np.isfinite(point))


@needs_scene
def test_refined_candidates_are_rescored_before_the_path_check(kinematics, monkeypatch):
    point, _ = kinematics.aim_point(ROBOT_IDLE, 320, 240)
    near_pref = FOREARM_PREF.copy()
    far_pref = np.radians([-15.0, 50.0, 0.0, 30.0])
    for q4 in (near_pref, far_pref):
        assert kinematics.natural_pose(with_arm(ROBOT_IDLE, q4)) is None
    weighted = [
        np.linalg.norm((q4 - FOREARM_PREF) * FOREARM_WEIGHTS) for q4 in (near_pref, far_pref)
    ]
    assert weighted[0] < weighted[1]
    refined = iter([(far_pref, 0.5), (near_pref, 0.5)])
    monkeypatch.setattr(
        sonic_arm_reference,
        "_refine_forearm",
        lambda kin, base, seed, aim: next(refined, (seed, 90.0)),
    )
    checked = []
    q4 = kinematics.solve_forearm(ROBOT_IDLE, point, lambda pose: checked.append(pose) or None)
    np.testing.assert_allclose(q4, near_pref)
    np.testing.assert_allclose(checked[0][15:19], near_pref)


@needs_scene
def test_unreachable_point_is_refused(kinematics):
    with pytest.raises(ValueError, match="cannot point within 3 deg|no natural pose points there"):
        kinematics.solve_forearm(ROBOT_IDLE, np.array([-1.0, 0.1, 0.0]), lambda pose: None)


@needs_scene
def test_all_blocked_is_refused_with_the_body(kinematics):
    point, _ = kinematics.aim_point(ROBOT_IDLE, 320, 240)
    with pytest.raises(ValueError, match="every pose that points there comes near the torso_link"):
        kinematics.solve_forearm(ROBOT_IDLE, point, lambda pose: "torso_link")


@needs_scene
def test_natural_pose_names_the_first_failed_check(kinematics):
    assert kinematics.natural_pose(with_arm(ROBOT_IDLE, FOREARM_PREF)) is None
    straight = with_arm(ROBOT_IDLE, np.radians([-45.0, 25.0, 0.0, 80.0]))
    assert kinematics.natural_pose(straight) in ("elbow_bend", "joint_limit")
    wide = with_arm(ROBOT_IDLE, np.radians([-45.0, 70.0, 0.0, 30.0]))
    assert kinematics.natural_pose(wide) == "shoulder_roll"
    back = with_arm(ROBOT_IDLE, np.radians([30.0, 25.0, 0.0, 30.0]))
    assert kinematics.natural_pose(back) == "wrist_behind"


def _gesture_waist(name, joint):
    keyframes, *_ = gesture_plan_inputs(name, BASE, np.zeros(14))
    return [k.target[MOTOR_NAMES.index(joint)] for k in keyframes]


def test_invite_turns_the_waist_both_ways():
    yaw = _gesture_waist("invite", "waist_yaw")
    assert yaw[0] == 0.0 and max(yaw) == pytest.approx(0.40) and min(yaw) == pytest.approx(-0.40)
    assert yaw[-1] == 0.0


def test_thanks_bows_forward():
    pitch = _gesture_waist("thanks", "waist_pitch")
    assert max(pitch) == pytest.approx(0.40) and min(pitch) >= 0.0 and pitch[-1] == 0.0
    keyframes, *_ = gesture_plan_inputs("thanks", BASE, np.zeros(14))
    bow = next(k for k in keyframes if k.target[14] == pytest.approx(0.40))
    # the bow is on the hands-at-the-belly pose: upper arms hanging, elbows bent past 90 degrees
    assert bow.target[15] == bow.target[22] == pytest.approx(0.0)
    assert bow.target[18] == bow.target[25] == pytest.approx(-0.3)


def test_bow_bows_forward_with_the_arms_hanging():
    pitch = _gesture_waist("bow", "waist_pitch")
    assert max(pitch) == pytest.approx(0.40) and min(pitch) >= 0.0
    keyframes, *_ = gesture_plan_inputs("bow", BASE, np.zeros(14))
    bow = next(k for k in keyframes if k.target[14] == pytest.approx(0.40))
    # shoulders pitched forward so the hanging arms clear the thighs as the torso tips
    assert bow.target[15] == bow.target[22] == pytest.approx(-0.40)
    assert bow.target[18] == bow.target[25] == pytest.approx(1.2)


@needs_scene
@pytest.mark.parametrize("name", ["invite", "thanks", "bow"])
def test_waist_gestures_pass_the_path_check(name):
    keyframes, move_s, return_min_s, arms, _ = gesture_plan_inputs(name, ROBOT_IDLE, np.zeros(14))
    plan = keyframe_plan(
        ROBOT_IDLE, keyframes, base_hands=np.zeros(14), move_s=move_s, return_min_s=return_min_s
    )
    checker = ArmCollisionChecker(SCENE, arms=("left", "right"))
    assert checker.first_contact(plan.poses, start=ROBOT_IDLE) is None
