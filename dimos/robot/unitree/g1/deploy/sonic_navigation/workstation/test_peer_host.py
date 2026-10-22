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

"""The operator map refresh: which voxels around the robot it clears."""

import numpy as np
import pytest

from dimos.robot.unitree.g1.deploy.sonic_navigation.workstation import peer_host

BASE = (2.0, -1.0, -0.44)  # base_link in odom: the pelvis about 0.84 m above the floor


def floor_and_person() -> np.ndarray:
    """Floor voxel centres in two layers, a stale person 1 m ahead, a wall far out."""
    xs, ys = np.meshgrid(np.arange(-1.0, 5.0, 0.08), np.arange(-4.0, 2.0, 0.08))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -1.28)], axis=1)
    upper = floor[::5].copy()
    upper[:, 2] = -1.20
    person = np.array([[3.0, -1.0, z] for z in (-0.92, -0.6, -0.36)])
    wall = np.array([[8.0, -1.0, z] for z in (-1.0, -0.5)])
    return np.vstack([floor, upper, person, wall]).astype(np.float32)


def test_refresh_region_clears_above_the_mapped_floor_around_the_robot() -> None:
    region = peer_host.refresh_region(floor_and_person(), *BASE)

    assert region is not None
    assert (region["cx"], region["cy"]) == (BASE[0], BASE[1])
    assert region["radius"] == peer_host.REFRESH_RADIUS_M
    assert region["floor"] == pytest.approx(-1.28, abs=0.02)
    # Both floor layers stay, the stale person goes.
    assert -1.20 < region["z_min"] < -0.92
    assert region["z_max"] > -0.36


def test_refresh_region_refuses_without_a_mapped_floor() -> None:
    overhead = np.array([[2.5, -1.0, -0.3], [2.6, -1.0, 0.2]], dtype=np.float32)

    assert peer_host.refresh_region(overhead, *BASE) is None
