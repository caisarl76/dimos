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
