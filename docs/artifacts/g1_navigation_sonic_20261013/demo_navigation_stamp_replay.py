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

"""Offline timestamp proof in the existing sensor image; no DDS or robot controller.

Replay recorded IMU/timestamps with the one saved cloud repeated at each recorded
cloud stamp. This proves stamp propagation, not fidelity of a full sensor replay.
Run --check-fixture locally, or mount this directory at /verification in pc2's
existing task image with --network none for the native pipeline checks.
"""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
from typing import Any

from dimos_lcm.sensor_msgs import Imu as LCMImu, PointCloud2 as LCMPointCloud2
import numpy as np
import zenoh

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if __package__:
    from .demo_dds_lio_adapter import encode_cloud, encode_imu
else:
    from demo_dds_lio_adapter import encode_cloud, encode_imu


def header(stamp: float) -> SimpleNamespace:
    ns = round(stamp * 1e9)
    return SimpleNamespace(stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9))


def cloud_sample(points: Any, stamp: float) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(stamp),
        is_bigendian=False,
        is_dense=True,
        height=1,
        width=len(points),
        point_step=points.dtype.itemsize,
        row_step=points.nbytes,
        data=points.tobytes(),
        fields=[
            SimpleNamespace(name=name, offset=points.dtype.fields[name][1], datatype=7, count=1)
            for name in ("x", "y", "z", "time")
        ],
    )


def imu_sample(row: Any, *, stamp: float | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(float(row[1]) if stamp is None else stamp),
        angular_velocity=SimpleNamespace(x=row[5], y=row[6], z=row[7]),
        linear_acceleration=SimpleNamespace(x=row[2], y=row[3], z=row[4]),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[0.0] * 9,
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration_covariance=[0.0] * 9,
    )


def check_fixture(fixture: Path) -> dict[str, Any]:
    with np.load(fixture) as data:
        points, row = data["last_cloud"], data["imu"][0]
        stamp = float(data["cloud_stamps"][0, 1])
        cloud = LCMPointCloud2.lcm_decode(encode_cloud(cloud_sample(points, stamp)))
        assert abs(cloud.header.stamp.sec + cloud.header.stamp.nsec * 1e-9 - stamp) < 5e-7
        assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == [
            ("x", 0, 7),
            ("y", 4, 7),
            ("z", 8, 7),
            ("offset_time", 12, 6),
        ]
        packed = np.frombuffer(
            cloud.data, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("t", "<u4")]
        )
        for name in ("x", "y", "z"):
            np.testing.assert_array_equal(packed[name], points[name])
        np.testing.assert_array_equal(packed["t"], np.rint(points["time"]).astype("<u4"))
        imu = LCMImu.lcm_decode(encode_imu(imu_sample(row)))
        assert abs(imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9 - row[1]) < 5e-7
        np.testing.assert_allclose(
            [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z],
            row[2:5] * 9.80665,
        )
        np.testing.assert_allclose(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z], row[5:8]
        )
        return {
            "fixture_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest(),
            "imu_samples": len(data["imu"]),
            "cloud_stamps": len(data["cloud_stamps"]),
            "saved_raw_clouds": 1,
            "saved_cloud_points": len(points),
            "adapter_checks_passed": True,
        }


def validate_stamp_chain(
    rows: dict[str, list[float]], *, scan_stamps: list[float], scan_duration: float
) -> None:
    expected_ends = [stamp + scan_duration for stamp in scan_stamps]
    for name in ("lidar", "odometry", "tf", "global_map", "global_costmap"):
        assert rows.get(name), f"missing {name} output"
        assert np.isfinite(rows[name]).all(), name
        assert np.all(np.diff(rows[name]) > 0), f"nonadvancing {name}"
    for name in ("lidar", "odometry"):
        assert all(any(abs(stamp - end) < 5e-7 for end in expected_ends) for stamp in rows[name]), (
            f"{name} stamp does not match an original raw scan end"
        )
    for name, parent in [
        ("lidar", "odometry"),
        ("tf", "odometry"),
        ("global_map", "lidar"),
        ("global_costmap", "global_map"),
    ]:
        assert all(any(abs(stamp - s) < 5e-7 for s in rows[parent]) for stamp in rows[name]), (
            f"{name} stamp does not match consumed {parent}"
        )


def run(fixture: Path, output: Path) -> None:
    summary = check_fixture(fixture)
    with np.load(fixture) as data:
        points, imus, clouds = data["last_cloud"], data["imu"], data["cloud_stamps"]
    with socket.socket() as candidate:
        candidate.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{candidate.getsockname()[1]}"
    config = zenoh.Config()
    for key, value in {
        "mode": "router",
        "listen/endpoints": [endpoint],
        "scouting/multicast/enabled": False,
        "scouting/gossip/enabled": True,
    }.items():
        config.insert_json5(key, json.dumps(value))
    router = zenoh.open(config)
    blueprint = autoconnect(
        PointLio.blueprint(),
        RayTracingVoxelMap.blueprint(voxel_size=0.05, worker_threads=2),
        CostMapper.blueprint(initial_safe_radius_meters=0.0),
    ).global_config(
        transport="zenoh",
        viewer="none",
        n_workers=3,
        robot_ip=None,
        robot_ips=None,
        zenoh_mode="client",
        zenoh_connect=endpoint,
        zenoh_multicast=False,
        zenoh_scouting=False,
        zenoh_gossip=True,
        serve_coordinator_rpc=False,
    )
    assert {a.module for a in blueprint.active_blueprints} == {
        PointLio,
        RayTracingVoxelMap,
        CostMapper,
    }
    condition = threading.Condition()
    rows: dict[str, list[float]] = defaultdict(list)
    last_receipt = [time.monotonic()]
    subscriptions: list[Any] = []
    coordinator = None

    def receive(name: str, msg: Any) -> None:
        if name == "tf":
            stamps = [t.ts for t in msg.transforms]
        else:
            stamps = [msg.ts]
        with condition:
            rows[name].extend(stamps)
            last_receipt[0] = time.monotonic()
            condition.notify_all()

    def quiet() -> dict[str, int]:
        end = time.monotonic() + 20
        with condition:
            last_receipt[0] = time.monotonic()
            while time.monotonic() - last_receipt[0] < 1.0:
                remaining = end - time.monotonic()
                assert remaining > 0, "pipeline did not drain within 20 seconds"
                condition.wait(min(remaining, 1.0))
            return {name: len(stamps) for name, stamps in rows.items()}

    try:
        coordinator = ModuleCoordinator.build(blueprint)
        for (name, _), transport in coordinator.transports.items():
            if name in {"lidar", "odometry", "tf", "global_map", "global_costmap"}:
                subscriptions.append(transport.subscribe(lambda msg, name=name: receive(name, msg)))
        inputs = {
            "imu_raw": coordinator.transports[("imu_raw", Imu)],
            "lidar_raw": coordinator.transports[("lidar_raw", PointCloud2)],
        }
        events = sorted(
            [(float(r[1]), "imu_raw", r) for r in imus]
            + [(float(r[1]), "lidar_raw", r) for r in clouds],
            key=lambda e: e[0],
        )
        source_start, started = events[0][0], time.monotonic()
        last_wires = {}
        for stamp, name, row in events:
            delay = started + (stamp - source_start) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            wire = (
                encode_imu(imu_sample(row))
                if name == "imu_raw"
                else encode_cloud(cloud_sample(points, stamp))
            )
            inputs[name].broadcast(None, wire)
            last_wires[name] = wire
        # Every scan needs an IMU at/after its final point before PointLIO drains it.
        inputs["imu_raw"].broadcast(
            None, encode_imu(imu_sample(imus[-1], stamp=float(clouds[-1, 1]) + 0.2))
        )
        counts = quiet()
        required = {"lidar", "odometry", "tf", "global_map", "global_costmap"}
        assert required <= rows.keys(), f"missing streams: {required - rows.keys()}"
        for name in required:
            assert len(rows[name]) >= 5, f"insufficient {name} outputs"
        # Pinned preprocessing skips point 0 and retains every third valid point.
        # In this recorded cloud, its last retained offset is 100.310 ms.
        assert points["time"][-3] == 100_310_000
        validate_stamp_chain(rows, scan_stamps=clouds[:, 1].tolist(), scan_duration=0.10031)
        # Idle, exact duplicate inputs and fresh IMUs without a LiDAR scan cannot
        # publish progressing localization/map stamps.
        assert quiet() == counts, "idle pipeline manufactured output"
        for name, wire in last_wires.items():
            inputs[name].broadcast(None, wire)
        assert quiet() == counts, "duplicate inputs manufactured progressing output"
        for n in range(20):
            inputs["imu_raw"].broadcast(
                None, encode_imu(imu_sample(imus[-1], stamp=float(clouds[-1, 1]) + 0.3 + n * 0.005))
            )
        assert quiet() == counts, "IMU-only input manufactured progressing output"
        summary.update(
            {
                "counts": counts,
                "stamp_propagation_passed": True,
                "idle_duplicate_imu_only_passed": True,
                "network": "none; loopback Zenoh only",
                "modules": sorted(a.module.__name__ for a in blueprint.active_blueprints),
                "dds_writers": False,
                "motor_controller": False,
                "limits": "One saved raw cloud reused at recorded stamps; final IMUs extended to drain scans. No full spatial replay or base extrinsic proof.",
            }
        )
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output / "output-stamps.npz", **{n: np.asarray(s) for n, s in rows.items()}
        )
        (output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
    finally:
        for unsubscribe in subscriptions:
            unsubscribe()
        if coordinator is not None:
            coordinator.stop()
        router.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("/verification/results"))
    parser.add_argument("--check-fixture", action="store_true")
    args = parser.parse_args()
    if args.check_fixture:
        print(json.dumps(check_fixture(args.fixture), indent=2))
    else:
        run(args.fixture, args.output)
