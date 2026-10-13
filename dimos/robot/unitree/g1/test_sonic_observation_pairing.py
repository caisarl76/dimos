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

import numpy as np
import pytest

from dimos.robot.unitree.g1.sonic_observation import POLICY_JOINT_NAMES
from dimos.robot.unitree.g1.sonic_observation_pairing import (
    SonicMeasuredState,
    SonicObservationPairer,
)


@pytest.fixture
def state():
    return SonicMeasuredState(
        sequence=7,
        positions={name: i / 10 for i, name in reversed(list(enumerate(POLICY_JOINT_NAMES)))},
        body_observed_at=10.01,
        left_hand_observed_at=10.02,
        right_hand_observed_at=10.0,
    )


@pytest.fixture
def rgb():
    return np.full((480, 640, 3), [10, 20, 30], np.uint8)


def test_pairs_named_joints_and_owned_rgb_using_oldest_acquisition_time(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    assert pairer.push_image(rgb, observed_at=10.015, now=10.03) is None
    rgb[:] = 0
    obs = pairer.push_state(state, now=10.04)
    assert obs is not None and obs.observed_at == 10.0 and obs.sequence == 0
    np.testing.assert_allclose(obs.state, np.arange(28) / 10)
    np.testing.assert_array_equal(obs.rgb[0, 0], [10, 20, 30])
    assert (
        pairer.push_state(
            replace(
                state,
                sequence=8,
                body_observed_at=10.03,
                left_hand_observed_at=10.04,
                right_hand_observed_at=10.02,
            ),
            now=10.05,
        )
        is None
    )


def test_buffer_matches_delayed_image_to_earlier_state(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    pairer.push_state(
        replace(
            state,
            sequence=8,
            body_observed_at=10.08,
            left_hand_observed_at=10.08,
            right_hand_observed_at=10.08,
        ),
        now=10.09,
    )
    obs = pairer.push_image(rgb, observed_at=10.015, now=10.1)
    assert obs is not None and obs.observed_at == 10.0


def test_excess_skew_or_expired_buffer_never_produces_observation(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.01)
    pairer.push_state(state, now=10.03)
    assert pairer.push_image(rgb, observed_at=10.03, now=10.04) is None
    assert pairer.push_image(rgb, observed_at=10.25, now=10.26) is None
    assert pairer.buffered_states == 0


@pytest.mark.parametrize(
    "change",
    [
        {"body_observed_at": 9.0},
        {"left_hand_observed_at": 10.2},
        {"right_hand_observed_at": float("nan")},
        {"positions": {}},
        {"sequence": True},
    ],
)
def test_invalid_or_stale_component_cannot_enter_pairer(state, change):
    pairer = SonicObservationPairer(max_skew=0.03)
    with pytest.raises(ValueError):
        pairer.push_state(replace(state, **change), now=10.05)
    assert pairer.buffered_states == 0


def test_replayed_state_or_frozen_hand_timestamp_rejected(state):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    with pytest.raises(ValueError, match="newer"):
        pairer.push_state(state, now=10.04)
    with pytest.raises(ValueError, match="newer"):
        pairer.push_state(
            replace(state, sequence=8, body_observed_at=10.04, left_hand_observed_at=10.04),
            now=10.05,
        )
    assert pairer.buffered_states == 1


def test_replayed_image_rejected_and_consumed_state_not_reused(state, rgb):
    pairer = SonicObservationPairer(max_skew=0.03)
    pairer.push_state(state, now=10.03)
    assert pairer.push_image(rgb, observed_at=10.02, now=10.04) is not None
    with pytest.raises(ValueError, match="newer"):
        pairer.push_image(rgb, observed_at=10.02, now=10.05)
    assert pairer.push_image(rgb, observed_at=10.025, now=10.05) is None


def test_buffer_count_is_bounded(state):
    pairer = SonicObservationPairer(max_skew=0.03, capacity=2)
    for index in range(5):
        stamp = 10 + index / 100
        pairer.push_state(
            replace(
                state,
                sequence=index,
                body_observed_at=stamp,
                left_hand_observed_at=stamp,
                right_hand_observed_at=stamp,
            ),
            now=10.1,
        )
    assert pairer.buffered_states == 2


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_skew": 0},
        {"max_skew": float("inf")},
        {"max_skew": 0.03, "max_age": 0.3},
        {"max_skew": 0.03, "capacity": 0},
    ],
)
def test_invalid_pairing_budget_rejected(kwargs):
    with pytest.raises(ValueError):
        SonicObservationPairer(**kwargs)
