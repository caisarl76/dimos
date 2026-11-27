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

"""End to end: the real voxel_ray_tracing binary clears a stale column on clear_region."""

from __future__ import annotations

import socket
import threading
import time

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.g1.deploy.sonic_navigation.workstation.peer_host import (
    open_router,
    refresh_region,
)
from dimos.utils.data import get_project_root

# Without a prebuilt binary the module would start a nix build: skip instead.
_BINARY = get_project_root() / "dimos/mapping/ray_tracing/rust/result/bin/voxel_ray_tracing"
FLOOR_Z = -1.28


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


@pytest.mark.skipif(not _BINARY.exists(), reason=f"needs a built {_BINARY}")
def test_clear_region_removes_a_column_rays_cannot_clear() -> None:
    assert RayTracingVoxelMapConfig().executable.endswith(_BINARY.name)
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    router = open_router(endpoint)
    blueprint = RayTracingVoxelMap.blueprint(voxel_size=0.08, worker_threads=2)
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={
            "g": {
                "transport": "zenoh",
                "viewer": "none",
                "zenoh_mode": "client",
                "zenoh_connect": endpoint,
                "zenoh_multicast": False,
                "zenoh_scouting": False,
                "zenoh_gossip": True,
                "serve_coordinator_rpc": False,
            }
        },
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    latest: list[PointCloud2 | None] = [None]
    lock = threading.Lock()

    def keep(msg: PointCloud2) -> None:
        with lock:
            latest[0] = msg

    unsub = coordinator.transports[("global_map", PointCloud2)].subscribe(keep)
    lidar = coordinator.transports[("lidar", PointCloud2)]
    clear = coordinator.transports[("clear_region", PoseStamped)]

    xs, ys = np.meshgrid(np.arange(0.3, 3.0, 0.04), np.arange(-1.5, 1.5, 0.04))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, FLOOR_Z)], axis=1)
    # A person 1 m ahead, standing in the near field.
    person = np.array(
        [
            [1.0 + dx, dy, z]
            for dx in (0, 0.04)
            for dy in (0, 0.04)
            for z in np.arange(-0.9, -0.3, 0.04)
        ]
    )

    def feed(points: np.ndarray, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            cloud = PointCloud2.from_numpy(
                points.astype(np.float32), frame_id="odom", timestamp=time.time()
            )
            lidar.broadcast(None, cloud)
            time.sleep(0.1)

    def mapped() -> np.ndarray:
        with lock:
            cloud = latest[0]
        assert cloud is not None, "no global map"
        return cloud.points_f32()

    def counts() -> tuple[int, int]:
        """(above-floor, floor) voxel counts of the latest global map."""
        z = mapped()[:, 2]
        return int((z > FLOOR_Z + 0.18).sum()), int((z < FLOOR_Z + 0.18).sum())

    try:
        feed(np.vstack([floor, person]), 2.0)
        assert counts()[0] > 0
        feed(floor, 1.0)  # the person left: rays from the origin do not clear the column
        stale = counts()
        assert stale[0] > 0
        region = refresh_region(mapped(), 0.0, 0.0, -0.44)
        assert region is not None, "no mapped floor"
        clear.broadcast(
            None,
            PoseStamped(
                position=Vector3(region["cx"], region["cy"], 0.0),
                orientation=Quaternion(region["radius"], region["z_min"], region["z_max"], 0.0),
                frame_id="odom",
                ts=time.time(),
            ),
        )
        feed(floor, 1.0)
        above, floor_voxels = counts()
        assert above == 0
        assert floor_voxels >= 0.9 * stale[1]
    finally:
        unsub()
        coordinator.stop()
        router.close()
