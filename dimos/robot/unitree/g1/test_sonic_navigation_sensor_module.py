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

"""Read-only sensor relay: preserved source stamps and unready progression on faults."""

import ast
import inspect
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1 import sonic_navigation_sensor_module as sensor
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_readiness import INPUT_BUDGETS
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
    MAX_PAYLOAD_BYTES,
    RESET_HOLDOFF_SECONDS,
    SonicNavigationSensorReceiver,
    SonicNavigationSensorSender,
    encode_cloud,
    encode_imu,
    encode_waist,
)

G = 9.80665
UPSIDE_DOWN = (0.0, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0)  # torso->sensor: z 0.4 m, roll pi


def header(sec: int, nanosec: int) -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id="livox")


def vector(x: float, y: float, z: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z)


def dds_imu(sec: int, nanosec: int = 0) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(sec, nanosec),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[0.0] * 9,
        angular_velocity=vector(0.1, -0.2, 0.3),
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration=vector(0.0, 0.0, -1.0),
        linear_acceleration_covariance=[1.0] + [0.0] * 8,
    )


def dds_cloud(sec: int, nanosec: int = 0, points: int = 3) -> SimpleNamespace:
    # MID360S layout: x y z intensity(f32) time(f32) ring; excluded fields stay out.
    step = 24
    data = np.zeros(
        points,
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("time", "<f4"),
            ("ring", "<u4"),
        ],
    )
    data["x"], data["y"], data["z"] = np.arange(points), 1.0, 2.0
    data["intensity"] = 50.0
    data["time"] = np.arange(points) * 1000.0
    fields = [
        SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in [
            ("x", 0, 7),
            ("y", 4, 7),
            ("z", 8, 7),
            ("intensity", 12, 7),
            ("time", 16, 7),
            ("ring", 20, 6),
        ]
    ]
    return SimpleNamespace(
        header=header(sec, nanosec),
        height=1,
        width=points,
        fields=fields,
        is_bigendian=False,
        point_step=step,
        row_step=step * points,
        data=data.tobytes(),
        is_dense=True,
    )


def lowstate(tick: int, yaw: float = 0.0) -> SimpleNamespace:
    motors = [SimpleNamespace(q=0.0) for _ in range(29)]
    motors[12].q = yaw
    return SimpleNamespace(tick=tick, motor_state=motors)


def waist(stamp: float, yaw: float = 0.0) -> bytes:
    return TFMessage(
        Transform(
            rotation=Quaternion.from_euler(Vector3(0, 0, yaw)),
            frame_id="torso_link",
            child_frame_id="base_link",
            ts=stamp,
        )
    ).lcm_encode()


def lio(stamp: float, *, x: float = 1.0, yaw: float = 0.3) -> Odometry:
    rotation = Quaternion.from_euler(Vector3(0, 0, yaw)) * Quaternion.from_euler(
        Vector3(math.pi, 0, 0)
    )
    return Odometry(
        ts=stamp,
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(position=Vector3(x, 2.0, 0.4), orientation=rotation),
    )


@pytest.fixture
def clock(mocker):
    value = [10.0]
    for target in (
        "dimos.robot.unitree.g1.sonic_navigation_sensor_module.monotonic",
        "dimos.robot.unitree.g1.sonic_navigation_peer_module.monotonic",
    ):
        mocker.patch(target, side_effect=lambda: value[0])
    return value


@pytest.fixture
def stopped():
    modules: list = []
    yield modules.append
    for module in modules:
        module.stop()


@pytest.fixture
def receiver(stopped):
    module = SonicNavigationSensorReceiver(
        cloud_endpoint="tcp://127.0.0.1:7101",
        state_endpoint="tcp://127.0.0.1:7102",
        torso_to_sensor=UPSIDE_DOWN,
    )
    outputs: dict[str, list] = {name: [] for name in ("lidar_raw", "imu_raw", "odometry", "tf")}
    for name, values in outputs.items():
        getattr(module, name).subscribe(values.append)
    stopped(module)
    return module, outputs


def test_conversion_preserves_stamps_offsets_and_imu_units() -> None:
    imu = Imu.lcm_decode(encode_imu(dds_imu(1700000000, 123456789)))
    assert imu.ts == pytest.approx(1700000000.123456789)
    assert imu.frame_id == "mid360_link"
    assert (imu.angular_velocity.x, imu.angular_velocity.y) == pytest.approx((0.1, -0.2))
    assert imu.linear_acceleration.z == pytest.approx(-G)
    assert imu.linear_acceleration_covariance[0] == pytest.approx(G**2)

    cloud = PointCloud2.lcm_decode(encode_cloud(dds_cloud(1700000001, 5)))
    assert cloud.ts == pytest.approx(1700000001.000000005)
    assert cloud.frame_id == "mid360_link"
    np.testing.assert_allclose(cloud.as_numpy()[0], [[0, 1, 2], [1, 1, 2], [2, 1, 2]])
    np.testing.assert_array_equal(cloud.offset_times_u32(), [0, 1000, 2000])


def test_waist_transform_uses_lowstate_tick_as_its_source_stamp() -> None:
    (edge,) = TFMessage.lcm_decode(encode_waist(lowstate(25_000, yaw=0.5))).transforms
    assert (edge.frame_id, edge.child_frame_id) == ("torso_link", "base_link")
    assert edge.ts == pytest.approx(25.0)
    assert edge.rotation.to_euler().z == pytest.approx(-0.5, abs=1e-6)


def test_sensor_module_has_no_dds_writer_or_motor_api() -> None:
    tree = ast.parse(Path(inspect.getfile(sensor)).read_text())
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    names |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
    forbidden = {
        "DataWriter",
        "Publisher",
        "ChannelPublisher",
        "LocoClient",
        "MotionSwitcherClient",
        "LowCmd_",
        "write",
    }
    assert not names & forbidden


def test_receiver_relays_allowlisted_frames_and_bounds_cloud_drain(receiver, clock) -> None:
    module, out = receiver
    clouds = [[b"lidar_raw", encode_cloud(dds_cloud(100 + i))] for i in range(3)]
    module.handle_frames("cloud", clouds)
    assert [c.ts for c in out["lidar_raw"]] == [102.0]  # drain to latest cloud

    imus = [[b"imu_raw", encode_imu(dds_imu(100, i * 5_000_000))] for i in range(3)]
    module.handle_frames("state", imus)
    assert [round(m.ts, 3) for m in out["imu_raw"]] == [100.0, 100.005, 100.01]  # IMU keeps order

    rejected = [
        [b"imu_raw", encode_imu(dds_imu(101))],  # IMU on the cloud socket
        [b"lidar_raw"],  # one part
        [b"lidar_raw", encode_cloud(dds_cloud(103)), b"extra"],
        [b"motor", b"x"],
        [b"lidar_raw", b"x" * (MAX_PAYLOAD_BYTES + 1)],
        [b"lidar_raw", encode_cloud(dds_cloud(0))],  # no source stamp
        [b"lidar_raw", b"not lcm"],
    ]
    module.handle_frames("cloud", rejected)
    assert len(out["lidar_raw"]) == 1
    assert module.rejected_frames == len(rejected)


def test_base_odometry_requires_commissioned_extrinsic_and_fresh_waist(
    receiver, clock, stopped
) -> None:
    module, out = receiver
    module.observe_lio_odometry(lio(100.0))
    assert out["odometry"] == []  # no waist yet

    module.handle_frames("state", [[b"waist_tf", waist(5.0, yaw=-0.5)]])
    module.observe_lio_odometry(lio(100.1))
    (odom,) = out["odometry"]
    assert (odom.frame_id, odom.child_frame_id, odom.ts) == ("odom", "base_link", 100.1)
    assert (odom.position.x, odom.position.y, odom.position.z) == pytest.approx((1, 2, 0), abs=1e-6)
    assert odom.yaw == pytest.approx(-0.2, abs=1e-6)
    (edge,) = out["tf"][0].transforms
    assert (edge.frame_id, edge.child_frame_id, edge.ts) == ("odom", "base_link", 100.1)

    clock[0] += 0.150
    module.handle_frames("state", [[b"waist_tf", waist(5.0, yaw=-0.5)]])  # cached tick
    clock[0] += 0.060
    module.observe_lio_odometry(lio(100.2))
    assert len(out["odometry"]) == 1  # duplicate waist did not refresh its age

    module.handle_frames("state", [[b"waist_tf", waist(5.02)]])
    module.observe_lio_odometry(lio(100.3, x=1.5))
    assert len(out["odometry"]) == 2
    module.observe_lio_odometry(
        Odometry(ts=100.4, frame_id="odom", child_frame_id="base_link", pose=lio(1).pose)
    )
    assert len(out["odometry"]) == 2  # only sensor-frame LIO input is composed

    unknown = SonicNavigationSensorReceiver(
        cloud_endpoint="tcp://127.0.0.1:7101", state_endpoint="tcp://127.0.0.1:7102"
    )
    stopped(unknown)
    produced: list = []
    unknown.odometry.subscribe(produced.append)
    unknown.handle_frames("state", [[b"waist_tf", waist(6.0)]])
    unknown.observe_lio_odometry(lio(101.0))
    assert produced == []


def peer_monitor(module: SonicNavigationSensorReceiver, stopped) -> SonicNavigationPeerModule:
    peer = SonicNavigationPeerModule(
        link_bind="tcp://127.0.0.1:*",
        extrinsic_valid=True,
        clock_domains={n: "sensor" for n in INPUT_BUDGETS},
    )
    stopped(peer)
    module.lidar_raw.subscribe(peer.observe_lidar_raw)
    module.imu_raw.subscribe(peer.observe_imu_raw)
    module.odometry.subscribe(peer.observe_odometry)
    module.tf.subscribe(peer.observe_tf)
    return peer


def feed(
    module: SonicNavigationSensorReceiver,
    peer: SonicNavigationPeerModule,
    stamp: float,
    *,
    cloud: SimpleNamespace | None = None,
    tick: int | None = None,
) -> None:
    module.handle_frames("cloud", [[b"lidar_raw", encode_cloud(cloud or dds_cloud(int(stamp)))]])
    module.handle_frames(
        "state",
        [
            [b"imu_raw", encode_imu(dds_imu(int(stamp)))],
            [b"waist_tf", encode_waist(lowstate(tick if tick is not None else int(stamp * 1000)))],
        ],
    )
    module.observe_lio_odometry(lio(stamp))
    cloud_out = PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32))
    cloud_out.ts = stamp
    peer.observe_lidar(cloud_out)
    peer.observe_global_map(cloud_out)
    peer.observe_global_costmap(OccupancyGrid(grid=np.zeros((2, 2), dtype=np.int8), ts=stamp))


def snapshot(peer: SonicNavigationPeerModule, clock: list[float]):
    return peer._monitor.snapshot(now=clock[0])


def test_relayed_chain_becomes_ready_then_cached_or_empty_clouds_go_unready(
    receiver, clock, stopped
) -> None:
    module, _ = receiver
    peer = peer_monitor(module, stopped)
    feed(module, peer, 100)
    assert snapshot(peer, clock).ready, snapshot(peer, clock).fault

    clock[0] += 0.2
    feed(module, peer, 101, cloud=dds_cloud(100))  # cached raw cloud stamp
    clock[0] += 0.2
    feed(module, peer, 102, cloud=dds_cloud(100))
    assert snapshot(peer, clock).fault == "stale:lidar_raw"

    feed(module, peer, 103, cloud=dds_cloud(103, points=0))  # empty cloud
    assert snapshot(peer, clock).fault == "invalid:lidar_raw"


def test_unknown_extrinsic_never_produces_base_inputs(clock, stopped) -> None:
    module = SonicNavigationSensorReceiver(
        cloud_endpoint="tcp://127.0.0.1:7101", state_endpoint="tcp://127.0.0.1:7102"
    )
    stopped(module)
    peer = peer_monitor(module, stopped)
    feed(module, peer, 100)
    assert snapshot(peer, clock).fault in {"missing:odom", "missing:base_tf"}


def test_transport_reset_discards_queued_data_and_goes_unready(receiver, clock, stopped) -> None:
    module, out = receiver
    peer = peer_monitor(module, stopped)
    feed(module, peer, 100)
    assert snapshot(peer, clock).ready

    module.handle_reset("state")
    feed(module, peer, 100.1)  # queued pre-reset IMU/waist arrive after the reset
    assert [round(m.ts, 3) for m in out["imu_raw"]] == [100.0]
    assert [m.ts for m in out["odometry"]] == [100]  # pre-reset waist no longer composes
    clock[0] += 0.21
    assert snapshot(peer, clock).fault == "stale:imu_raw"

    clock[0] += RESET_HOLDOFF_SECONDS
    feed(module, peer, 101)
    assert snapshot(peer, clock).ready  # live progression after the hold-off


def test_real_sockets_round_trip_two_part_frames(mocker) -> None:
    mocker.patch.object(SonicNavigationSensorSender, "_open_readers", return_value={})
    sender = SonicNavigationSensorSender(
        cloud_bind="tcp://127.0.0.1:*", state_bind="tcp://127.0.0.1:*"
    )
    sender.start()
    module = SonicNavigationSensorReceiver(
        cloud_endpoint=sender.bound_endpoints["cloud"],
        state_endpoint=sender.bound_endpoints["state"],
    )
    received: list = []
    module.imu_raw.subscribe(received.append)
    module.lidar_raw.subscribe(received.append)
    try:
        module.start()
        deadline = time.monotonic() + 5
        while len(received) < 2 and time.monotonic() < deadline:  # wait out slow joiner
            sender.relay("imu_raw", dds_imu(200))
            sender.relay("lidar_raw", dds_cloud(201))
            time.sleep(0.05)
        assert {type(m).__name__ for m in received} >= {"Imu", "PointCloud2"}
        assert {m.ts for m in received} <= {200.0, 201.0}
    finally:
        module.stop()
        sender.stop()
