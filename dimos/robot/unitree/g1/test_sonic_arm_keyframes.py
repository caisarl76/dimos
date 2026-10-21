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
from typing import get_args

import numpy as np
import pytest

from dimos.robot.unitree.g1.sonic_arm_reference import (
    PEAK_SPEED,
    RATE,
    ArmCollisionChecker,
    Keyframe,
    SonicEncoder,
    keyframe_plan,
    retreat_poses,
    retreat_route,
    stroke_plan,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    GESTURE_MAX_S,
    GESTURES,
    GestureName,
    chunk_rows,
    gesture_keyframes,
    token_still,
)
from dimos.robot.unitree.g1.test_sonic_arm_skill import ROBOT_IDLE, SCENE, needs_scene

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
    plan = keyframe_plan(BASE, keyframes, split_every=900)
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


@needs_encoder
def test_a_split_pause_gives_still_tokens_for_the_session_blend():
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    plan = keyframe_plan(BASE, swings, split_every=300)
    tokens = SonicEncoder(ENCODER).encode(plan.poses)
    still = token_still(tokens)
    assert plan.still.any()
    for row in np.nonzero(plan.still)[0]:
        assert still[row]


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
    still = np.zeros(1975, bool)
    still[[0, 900, 975, 976]] = True
    assert chunk_rows(1975, still) == [(0, 1000), (975, 1975)]
    assert chunk_rows(1000, np.zeros(1000, bool)) == [(0, 1000)]


@pytest.mark.parametrize("name", get_args(GestureName))
def test_each_gesture_plans_within_the_limits(name):
    plan = keyframe_plan(BASE, gesture_keyframes(name, BASE), split_every=1000 - 25 - 100)
    assert len(plan.poses) / RATE <= GESTURE_MAX_S
    assert all(len(retreat_poses(plan, s)) + 100 <= 1000 for s in range(len(plan.segments)))
    assert np.abs(np.diff(plan.poses, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert set(GESTURES) == set(get_args(GestureName))


def test_gesture_keyframes_move_only_the_left_arm_from_the_measured_base():
    base = ROBOT_IDLE.copy()
    for name in GESTURES:
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
    checker = ArmCollisionChecker(SCENE)
    plan = keyframe_plan(ROBOT_IDLE, gesture_keyframes(name, ROBOT_IDLE))
    assert checker.first_contact(plan.poses, start=ROBOT_IDLE) is None
    for s in range(len(plan.segments)):
        assert checker.first_contact(retreat_poses(plan, s), start=ROBOT_IDLE) is None, s
