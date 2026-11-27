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

"""Cached maps retain source stamps; empty grids cannot prove navigation readiness."""

import numpy as np
import pytest

from dimos.mapping.costmapper import CostMapper
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def mapper():
    module = CostMapper(initial_safe_radius_meters=0.0)
    try:
        yield module
    finally:
        module.stop()


def test_nonempty_costmap_retains_stamp_on_cached_recalculation(mapper):
    xy = np.array([(x * 0.05, y * 0.05, 0.0) for x in range(9) for y in range(9)], dtype=np.float32)
    cloud = PointCloud2.from_numpy(xy, frame_id="odom")
    cloud.ts = 100.0
    first = mapper._calculate_costmap(cloud)
    second = mapper._calculate_costmap(cloud)
    assert first.ts == second.ts == cloud.ts
    assert np.any(first.grid >= 0)
    assert np.any(first.grid == -1)


def test_empty_costmap_is_unobserved_even_with_regenerated_stamp(mapper):
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="odom")
    cloud.ts = 100.0
    grid = mapper._calculate_costmap(cloud)
    assert not np.any(grid.grid >= 0)
    # The existing empty-grid path assigns a default timestamp. Content validity,
    # not that fresh-looking stamp, must reject it; shared mapper behavior stays.
    assert grid.grid.size == 1
