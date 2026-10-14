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

"""Native timestamp evidence must remain tied to the original sensor clock."""

import pytest

from docs.artifacts.g1_navigation_sonic_20261013.demo_navigation_stamp_replay import (
    validate_stamp_chain,
)


@pytest.fixture
def output_stamps():
    return {
        "lidar": [100.10031, 100.20031],
        "odometry": [100.10031, 100.20031],
        "tf": [100.10031, 100.20031],
        "global_map": [100.10031, 100.20031],
        "global_costmap": [100.20031],
    }


def test_recorded_scan_end_propagates_through_the_chain(output_stamps):
    validate_stamp_chain(output_stamps, scan_stamps=[100.0, 100.1], scan_duration=0.10031)


def test_fresh_common_output_clock_cannot_prove_source_progress(output_stamps):
    unrelated = {name: [stamp + 9000 for stamp in stamps] for name, stamps in output_stamps.items()}
    with pytest.raises(AssertionError, match="raw scan end"):
        validate_stamp_chain(unrelated, scan_stamps=[100.0, 100.1], scan_duration=0.10031)


def test_incoherent_lio_cloud_and_odometry_are_rejected(output_stamps):
    output_stamps["odometry"] = [100.20031]
    output_stamps["tf"] = [100.20031]
    with pytest.raises(AssertionError, match="odometry"):
        validate_stamp_chain(output_stamps, scan_stamps=[100.0, 100.1], scan_duration=0.10031)
