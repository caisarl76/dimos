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

"""Preserved native failures cannot be hidden by a diagnostic runner exit code."""

import json
from pathlib import Path

import pytest

from docs.artifacts.g1_navigation_sonic_20261013.demo_navigation_verdict import transition_verdict


@pytest.fixture
def trials():
    return json.loads(
        Path(__file__)
        .with_name("motion-resolution")
        .joinpath("trial-verdict-fixtures.json")
        .read_text()
    )


@pytest.mark.parametrize(
    "name", ["absolute-1", "absolute-2", "absolute-3", "relative-1", "relative-3"]
)
def test_preserved_diagnostic_direction_failures_are_rejected(trials, name):
    assert not transition_verdict(trials[name]["cases"], 3)


def test_all_original_transition_cases_pass(trials):
    assert transition_verdict(trials["relative-2"]["cases"], 3)


def test_missing_stop_case_cannot_pass(trials):
    cases = trials["relative-2"]["cases"][:-1]
    assert not transition_verdict(cases, 3)


def test_forged_pass_flag_does_not_hide_reverse_motion(trials):
    cases = trials["absolute-1"]["cases"]
    for case in cases:
        case["passed"] = True
    assert not transition_verdict(cases, 3)
