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

"""Progress, dependency clocks and static calibration fence navigation readiness."""

import math

import pytest

from dimos.robot.unitree.g1.sonic_navigation_readiness import (
    INPUT_BUDGETS,
    NavigationReadiness,
)


@pytest.fixture
def monitor():
    return NavigationReadiness(
        extrinsic_valid=True, clock_domains={name: "sensor" for name in INPUT_BUDGETS}
    )


def observe_all(monitor, *, stamp=100.0, now=10.0, frozen=None):
    for name in INPUT_BUDGETS:
        monitor.observe(
            name, 100.0 if name == frozen else stamp, valid=True, received_monotonic=now
        )


@pytest.mark.parametrize("frozen,budget", INPUT_BUDGETS.items())
def test_cached_inputs_cannot_refresh_readiness(monitor, frozen, budget):
    observe_all(monitor)
    assert monitor.snapshot(now=10.0).ready
    observe_all(monitor, stamp=100.0 + budget, now=10.0 + budget, frozen=frozen)
    result = monitor.snapshot(now=10.0 + budget)
    assert not result.ready
    progress = {item.name: item for item in result.inputs}
    assert not progress[frozen].valid
    assert progress[frozen].source_stamp == 100.0


def test_stamp_reset_requires_new_session(monitor):
    observe_all(monitor)
    monitor.observe("lidar_raw", 99.0, valid=True, received_monotonic=10.1)
    observe_all(monitor, stamp=101.0, now=10.2)
    assert monitor.snapshot(now=10.2).fault == "source_stamp_rollback:lidar_raw"
    fresh = NavigationReadiness(
        extrinsic_valid=True, clock_domains={n: "sensor" for n in INPUT_BUDGETS}
    )
    observe_all(fresh, stamp=101.0, now=10.2)
    assert fresh.snapshot(now=10.2).ready


@pytest.mark.parametrize("invalid", ["odom", "global_map", "global_costmap"])
def test_invalid_data_never_advances_progress(monitor, invalid):
    observe_all(monitor)
    monitor.observe(invalid, 9000.0, valid=False, received_monotonic=10.1)
    assert not monitor.snapshot(now=10.1).ready
    # Empty costmaps may have a regenerated wall-clock stamp; it cannot establish
    # a clock barrier or make the next genuine source sample look rolled back.
    observe_all(monitor, stamp=100.2, now=10.2)
    assert monitor.snapshot(now=10.2).ready


def test_map_new_arrivals_cannot_hide_upstream_lag(monitor):
    observe_all(monitor)
    for name in INPUT_BUDGETS:
        stamp = 100.1 if name in ("global_map", "global_costmap") else 102.0
        monitor.observe(name, stamp, valid=True, received_monotonic=10.1)
    snapshot = monitor.snapshot(now=10.1)
    progress = {item.name: item for item in snapshot.inputs}
    assert not snapshot.ready
    assert progress["global_map"].upstream_lag == pytest.approx(1.9)
    assert not progress["global_costmap"].valid


def test_static_transform_does_not_expire(monitor):
    observe_all(monitor)
    observe_all(monitor, stamp=10000.0, now=200.0)
    assert monitor.snapshot(now=200.0).ready
    uncalibrated = NavigationReadiness(clock_domains={n: "sensor" for n in INPUT_BUDGETS})
    observe_all(uncalibrated)
    assert uncalibrated.snapshot(now=10.0).fault == "extrinsic_unverified"


def test_cross_clock_stamps_are_not_subtracted():
    domains = {n: "sensor" for n in INPUT_BUDGETS}
    domains["global_costmap"] = "different_clock"
    monitor = NavigationReadiness(extrinsic_valid=True, clock_domains=domains)
    observe_all(monitor)
    result = monitor.snapshot(now=10.0)
    assert not result.ready
    assert result.fault == "clock_domain_mismatch:global_costmap"
    assert all(
        math.isfinite(item.age) and math.isfinite(item.upstream_lag) for item in result.inputs
    )


def test_unknown_clock_domain_is_unready():
    monitor = NavigationReadiness(extrinsic_valid=True)
    observe_all(monitor)
    assert monitor.snapshot(now=10.0).fault == "clock_domain_unknown:lidar_raw"


def test_monotonic_age_is_independent_of_source_clock_origin(monitor):
    observe_all(monitor, stamp=1_791_771_642.0)
    observe_all(monitor, stamp=1_791_781_642.0, now=10.05)
    assert monitor.snapshot(now=10.05).ready
    assert not monitor.snapshot(now=10.25).ready


def test_missing_inputs_are_finite_and_unready(monitor):
    result = monitor.snapshot(now=10.0)
    assert not result.ready
    assert result.fault == "missing:lidar_raw"
    assert all(
        math.isfinite(item.age) and math.isfinite(item.source_stamp) for item in result.inputs
    )
