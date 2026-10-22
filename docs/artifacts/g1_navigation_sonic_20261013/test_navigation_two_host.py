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

"""Two-host harness helpers: replay continuity, fault proxy and the scoped DDS shim."""

import importlib
import importlib.util
import os
from pathlib import Path
import socket
import sys
import threading
import time
import typing
from unittest.mock import patch

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPLAY = HERE / "stamp-replay/dds-inputs.npz"


@pytest.fixture
def shim(tmp_path, monkeypatch):
    monkeypatch.setenv("SONIC_TWO_HOST_OUTPUT", str(tmp_path))
    monkeypatch.setenv("SONIC_TWO_HOST_REPLAY", str(REPLAY))
    with patch.dict(sys.modules):
        spec = importlib.util.spec_from_file_location(
            "cyclonedds", HERE / "two-host-dds-shim/cyclonedds/__init__.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module


def stamps(shim, channel, count):
    replay = shim._Replay(channel)
    rows = replay.recording.imu if channel == "imu_raw" else replay.recording.clouds
    values = []
    for index in range(count):
        replay.index = index
        values.append(replay.stamp(rows)[0])
    return np.array(values)


def test_replay_loops_without_artificial_gaps(shim):
    with np.load(REPLAY) as data:
        recorded = {"imu_raw": data["imu"][:, 1], "lidar_raw": data["cloud_stamps"][:, 1]}
    for channel, count in (("imu_raw", 6000), ("lidar_raw", 300)):
        gaps = np.diff(stamps(shim, channel, count))
        assert (gaps > 0).all()
        assert gaps.max() <= np.diff(recorded[channel]).max() + 1e-9


def test_restarted_sender_resumes_replay_clock(shim):
    first = shim._Replay("imu_raw")
    time.sleep(0.3)
    restarted = shim._Replay("imu_raw")
    assert restarted.index > first.index  # a restarted process never rewinds the sensor clock


@pytest.fixture
def proxy(monkeypatch):
    monkeypatch.syspath_prepend(str(HERE))
    from demo_navigation_two_host import FaultProxy

    server = socket.create_server(("127.0.0.1", 0))

    def serve(client):
        with client:
            client.sendall(client.recv(100))

    def echo():
        while True:
            try:
                client, _ = server.accept()
            except OSError:
                return
            threading.Thread(target=serve, args=(client,), daemon=True).start()

    threading.Thread(target=echo, daemon=True).start()
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        listen = probe.getsockname()[1]
    forwarder = FaultProxy(listen, server.getsockname()[1])
    yield forwarder, listen
    forwarder.close()
    server.close()


def round_trip(port, timeout=2.0):
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as client:
        started = time.monotonic()
        client.sendall(b"ping")
        assert client.recv(10) == b"ping"
        return time.monotonic() - started


def test_proxy_delays_holds_and_releases_connections(proxy):
    forwarder, port = proxy
    assert round_trip(port) < 0.05
    forwarder.delay = 0.05
    assert round_trip(port) >= 0.1  # added in each direction
    forwarder.delay, forwarder.hold = 0.0, True
    with pytest.raises(TimeoutError):
        round_trip(port, timeout=0.3)
    forwarder.hold = False
    descriptors = len(os.listdir("/proc/self/fd"))
    for _ in range(200):
        round_trip(port)
    time.sleep(0.5)
    assert len(os.listdir("/proc/self/fd")) - descriptors < 20


def test_dds_shim_is_served_only_to_the_sensor_sender(tmp_path, monkeypatch):
    monkeypatch.setenv("SONIC_TWO_HOST_OUTPUT", str(tmp_path))
    monkeypatch.setenv("SONIC_TWO_HOST_REPLAY", str(REPLAY))
    monkeypatch.syspath_prepend(str(HERE))
    monkeypatch.setattr(sys, "meta_path", list(sys.meta_path))
    with patch.dict(sys.modules):
        import demo_navigation_dds_finder

        demo_navigation_dds_finder.install()
        with pytest.raises(ImportError):
            importlib.import_module("cyclonedds")
        from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
            SonicNavigationSensorSender,
        )

        readers = SonicNavigationSensorSender(
            cloud_bind="tcp://127.0.0.1:*", state_bind="tcp://127.0.0.1:*"
        )._open_readers()
        assert sorted(readers) == ["imu_raw", "lidar_raw", "waist_tf"]


def test_proxy_releases_overlapping_connections(proxy):
    forwarder, port = proxy
    held = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    assert round_trip(port) < 0.5  # a newer connection opens and finishes meanwhile
    held.sendall(b"late")
    assert held.recv(10) == b"late"
    held.close()
    deadline = time.monotonic() + 2.0
    while forwarder._sockets and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not forwarder._sockets  # each connection releases only its own sockets


def test_sender_idl_types_resolve_like_cyclonedds(tmp_path, monkeypatch):
    # CycloneDDS resolves IDL field annotations with get_type_hints; lazily imported
    # names must not be left as unresolvable strings (failed on pc2 2026-10-15).
    monkeypatch.setenv("SONIC_TWO_HOST_OUTPUT", str(tmp_path))
    monkeypatch.setenv("SONIC_TWO_HOST_REPLAY", str(REPLAY))
    monkeypatch.syspath_prepend(str(HERE))
    monkeypatch.setattr(sys, "meta_path", list(sys.meta_path))
    with patch.dict(sys.modules):
        import demo_navigation_dds_finder

        demo_navigation_dds_finder.install()
        from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
            SonicNavigationSensorSender,
        )

        readers = SonicNavigationSensorSender(
            cloud_bind="tcp://127.0.0.1:*", state_bind="tcp://127.0.0.1:*"
        )._open_readers()
        hints = typing.get_type_hints(readers["imu_raw"].topic.kind)
        assert set(hints) >= {"header", "angular_velocity", "linear_acceleration"}


def test_snapshot_keeps_points_near_the_robot_with_grid_and_pose(tmp_path):
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    spec = importlib.util.spec_from_file_location(
        "two_host_peer",
        HERE.parents[2] / "dimos/robot/unitree/g1/deploy/sonic_navigation/workstation/peer_host.py",
    )
    peer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(peer)
    points = np.array([[1.2, 1.0, 0.3], [2.9, 1.0, 0.1], [4.0, 1.0, 0.5]], dtype=np.float32)
    cloud = PointCloud2.from_numpy(points, frame_id="odom")
    grid = OccupancyGrid(
        grid=np.full((4, 5), -1, dtype=np.int8),
        resolution=0.05,
        origin=Pose(position=Vector3(0.5, 0.5, 0.0)),
        frame_id="odom",
    )
    odom = Odometry(pose=Pose(position=Vector3(1.0, 1.0, 0.0)), frame_id="odom")
    path = tmp_path / "snap.npz"
    peer.save_snapshot(str(path), cloud, grid, odom)
    with np.load(path) as saved:
        assert saved["points"].shape == (2, 3)  # 4.0 m away is dropped
        assert (saved["grid"] == -1).all() and saved["grid"].shape == (4, 5)
        assert list(saved["origin"]) == [0.5, 0.5] and float(saved["resolution"]) == 0.05
        assert list(saved["robot"][:2]) == [1.0, 1.0]


def test_costmap_summary_splits_near_occupancy_behind_the_robot():
    import math

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.nav_msgs.Odometry import Odometry

    spec = importlib.util.spec_from_file_location(
        "two_host_peer",
        HERE.parents[2] / "dimos/robot/unitree/g1/deploy/sonic_navigation/workstation/peer_host.py",
    )
    peer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(peer)
    cells = np.zeros((40, 40), dtype=np.int8)  # 2 m x 2 m, origin -1,-1, robot at 0,0
    cells[20, 20 + 6] = 100  # 0.3 m along +x
    cells[20 - 6, 20] = 100  # 0.3 m along -y
    grid = OccupancyGrid(grid=cells, resolution=0.05, origin=Pose(position=Vector3(-1, -1, 0)))
    facing_y = Quaternion.from_euler(Vector3(0, 0, math.pi / 2))
    odom = Odometry(pose=Pose(position=Vector3(0, 0, 0), orientation=facing_y), frame_id="odom")
    summary = peer.costmap_summary(grid, odom, [])
    assert summary["occupied_within_0.5m"] == 2
    assert summary["occupied_within_0.5m_behind"] == 1  # the -y cell, behind a +y heading


def test_peer_call_skips_a_late_reply_to_an_earlier_timed_out_call(monkeypatch):
    monkeypatch.syspath_prepend(str(HERE))
    import subprocess

    from demo_navigation_two_host import PeerHost

    # A child that answers "tele" twice (the second one late) and then "stop".
    child = (
        "import json, sys\n"
        "for line in sys.stdin:\n"
        "    op = json.loads(line)['op']\n"
        "    if op == 'stop':\n"
        "        print(json.dumps({'op': 'tele'}), flush=True)\n"
        "    print('a DimOS log line', flush=True)\n"
        "    print(json.dumps({'op': op}), flush=True)\n"
    )
    peer = object.__new__(PeerHost)
    peer.process = subprocess.Popen(
        [sys.executable, "-c", child], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
    )
    peer._start_reader()
    try:
        assert peer.call("tele")["op"] == "tele"
        assert peer.call("stop")["op"] == "stop"
    finally:
        peer.process.kill()


def load_peer_host():
    spec = importlib.util.spec_from_file_location(
        "two_host_peer",
        HERE.parents[2] / "dimos/robot/unitree/g1/deploy/sonic_navigation/workstation/peer_host.py",
    )
    peer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(peer)
    return peer


def test_truth_pose_composes_back_to_the_simulated_pelvis(mocker):
    """The sim-only PointLIO stand-in must reproduce the robot's frame chain: the
    production receiver turns its sensor pose back into the MuJoCo pelvis pose."""
    import math
    from types import SimpleNamespace

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.robot.unitree.g1.g1_tf_publisher import torso_to_mid360
    from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
        SonicNavigationSensorReceiver,
        encode_waist,
    )

    peer = load_peer_host()
    yaw = 0.7
    joints = [0.0] * 29
    joints[12], joints[13], joints[14] = 0.3, 0.05, -0.08  # waist yaw, roll, pitch
    q = Quaternion.from_euler(Vector3(0.02, -0.03, yaw))
    row = {"qpos": [1.5, -0.4, 0.79, q.w, q.x, q.y, q.z, *joints]}
    mount = torso_to_mid360()
    sensor = peer.truth_sensor_pose(row, mount)

    t, r = mount.translation, mount.rotation
    receiver = SonicNavigationSensorReceiver(
        cloud_endpoint="tcp://127.0.0.1:7101",
        state_endpoint="tcp://127.0.0.1:7102",
        torso_to_sensor=(t.x, t.y, t.z, r.x, r.y, r.z, r.w),
    )
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_navigation_sensor_module.monotonic", return_value=10.0
    )
    out: list = []
    receiver.odometry.subscribe(out.append)
    lowstate = SimpleNamespace(tick=5000, motor_state=[SimpleNamespace(q=j) for j in joints])
    receiver.handle_frames("state", [[b"waist_tf", encode_waist(lowstate)]])
    receiver.observe_lio_odometry(
        Odometry(
            ts=100.0,
            frame_id="odom",
            child_frame_id="mid360_link",
            pose=Pose(position=sensor.translation, orientation=sensor.rotation),
        )
    )
    receiver.stop()
    (base,) = out
    assert (base.x, base.y, base.position.z) == pytest.approx((1.5, -0.4, 0.79), abs=1e-6)
    assert base.yaw == pytest.approx(yaw, abs=1e-6)
    assert not math.isclose(sensor.translation.z, 0.79)  # the sensor sits above the pelvis


def test_synthetic_grid_marks_oriented_boxes_only():
    import math

    peer = load_peer_host()
    # A 0.4 x 0.2 m box centred at (1.6, 2.0), long side along +y.
    grid = peer.synthetic_grid(1.0, 2.0, [[1.6, 2.0, 0.2, 0.1, math.pi / 2]], ts=5.0)
    cells = np.asarray(grid.grid)
    assert cells.shape == (200, 200) and grid.ts == 5.0 and grid.frame_id == "odom"
    assert (grid.origin.position.x, grid.origin.position.y) == (-4.0, -3.0)
    occupied = np.argwhere(cells == 100)
    xs = grid.origin.position.x + (occupied[:, 1] + 0.5) * 0.05
    ys = grid.origin.position.y + (occupied[:, 0] + 0.5) * 0.05
    assert len(occupied) == 32  # 4 x 8 cells at 0.05 m
    assert xs.min() > 1.49 and xs.max() < 1.71 and ys.min() > 1.79 and ys.max() < 2.21
    assert (cells[cells != 100] == 0).all()


def test_corridor_geometry_for_obstacle_cases(monkeypatch):
    import math

    monkeypatch.syspath_prepend(str(HERE))
    from demo_navigation_two_host import clearance, rect

    # Facing +y from (1, 1): 1.5-2.1 m ahead, from 1.2 m right to 0.12 m right.
    box = rect((1.0, 1.0, math.pi / 2), ahead=(1.5, 2.1), left=(-1.2, -0.12))
    assert box == pytest.approx([1.0 + 0.66, 1.0 + 1.8, 0.3, 0.54, math.pi / 2])
    # Straight ahead along the centre line passes 0.12 m from the box's left edge.
    assert clearance([(1.0, 1.0 + d / 10) for d in range(40)], box) == pytest.approx(0.12)
    assert clearance([(1.0 + 0.5, 1.0 + 1.8)], box) == 0.0  # inside
