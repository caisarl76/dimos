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
        "two_host_peer", HERE / "demo_navigation_two_host_peer.py"
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
        "two_host_peer", HERE / "demo_navigation_two_host_peer.py"
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
