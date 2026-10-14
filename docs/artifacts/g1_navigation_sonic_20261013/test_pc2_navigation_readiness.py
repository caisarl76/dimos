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

"""Passive pc2 readiness checks evaluated from recorded observations, never live commands."""

import ast
import inspect
from pathlib import Path

from docs.artifacts.g1_navigation_sonic_20261013 import demo_pc2_navigation_readiness as probe


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def test_stream_progress_requires_advancing_gap_free_source_stamps() -> None:
    steady = [(10.0 + 0.1 * i, 500.0 + 0.1 * i) for i in range(20)]
    assert probe.stream_progress(steady, 0.35)["ready"]
    gap = steady[:10] + [(t + 0.5, s + 0.5) for t, s in steady[10:]]
    assert probe.stream_progress(gap, 0.35)["max_receipt_gap_s"] > 0.35
    assert not probe.stream_progress(gap, 0.35)["ready"]
    cached = [(10.0 + 0.1 * i, 500.0) for i in range(20)]
    assert probe.stream_progress(cached, 0.35)["duplicates"] == 19
    assert not probe.stream_progress(cached, 0.35)["ready"]
    rollback = [*steady, (12.1, 400.0)]
    assert probe.stream_progress(rollback, 0.35)["rollbacks"] == 1
    assert not probe.stream_progress(rollback, 0.35)["ready"]
    assert not probe.stream_progress([], 0.35)["ready"]


def test_clocks_pinned_matches_jetson_clocks_state(tmp_path) -> None:
    for cpu in (0, 1):
        write(tmp_path / f"devices/system/cpu/cpu{cpu}/cpufreq/scaling_min_freq", "1728000")
        write(tmp_path / f"devices/system/cpu/cpu{cpu}/cpufreq/cpuinfo_max_freq", "1728000")
    write(tmp_path / "class/devfreq/17000000.gpu/min_freq", "918000000")
    write(tmp_path / "class/devfreq/17000000.gpu/max_freq", "918000000")
    assert probe.clocks_pinned(tmp_path)["pinned"]
    write(tmp_path / "devices/system/cpu/cpu1/cpufreq/scaling_min_freq", "729600")
    result = probe.clocks_pinned(tmp_path)
    assert not result["pinned"] and "cpu1" in result["issues"][0]
    assert not probe.clocks_pinned(tmp_path / "missing")["pinned"]  # nothing readable


def test_listening_ports_and_sonic_processes_from_proc(tmp_path) -> None:
    header = "  sl  local_address rem_address   st\n"
    write(
        tmp_path / "net/tcp",
        header + "   0: 0100007F:15B4 00000000:0000 0A\n   1: 0100007F:15B5 0100007F:9C40 01\n",
    )
    write(
        tmp_path / "net/tcp6",
        header
        + "   0: 00000000000000000000000000000000:15B5 00000000000000000000000000000000:0000 0A\n",
    )
    assert probe.listening_ports(tmp_path) == {5556, 5557}
    write(
        tmp_path / "4242/cmdline", "/home/unitree/x/target/release/g1_deploy_onnx_ref\0enP8p1s0\0"
    )
    write(tmp_path / "77/cmdline", "python3\0other.py\0")
    assert [p["pid"] for p in probe.running(tmp_path, ("g1_deploy_onnx_ref",))] == [4242]


def test_lowstate_health_needs_29_finite_joints_imu_and_advancing_tick() -> None:
    good = [{"tick": 100 + i, "q": [0.0] * 29, "imu": [1, 0, 0, 0, 0, 0, 9.8]} for i in range(5)]
    assert probe.lowstate_health(good, expected_joints=29)["ready"]
    nan = [*good[:-1], {**good[-1], "q": [float("nan"), *[0.0] * 28]}]
    assert not probe.lowstate_health(nan, expected_joints=29)["ready"]
    short = [{**s, "q": [0.0] * 27} for s in good]
    assert not probe.lowstate_health(short, expected_joints=29)["ready"]
    frozen = [{**s, "tick": 100} for s in good]
    assert not probe.lowstate_health(frozen, expected_joints=29)["ready"]


def test_verdict_blocks_on_any_unready_prerequisite() -> None:
    ready = {
        "clocks": {"pinned": True},
        "sonic_processes": [],
        "ports_in_use": [],
        "streams": {name: {"ready": True} for name in ("lidar_raw", "imu_raw", "lowstate")},
        "lowstate": {"ready": True},
        "lowcmd_traffic_samples": 0,
        "conversion_errors": 0,
        "extrinsic_commissioned": True,
    }
    assert probe.verdict(ready) == []
    cases = {
        "clocks_not_pinned": {"clocks": {"pinned": False}},
        "sonic_process_running": {"sonic_processes": [{"pid": 1}]},
        "task_ports_in_use:5556": {"ports_in_use": [5556]},
        "stream_unready:imu_raw": {"streams": {**ready["streams"], "imu_raw": {"ready": False}}},
        "lowstate_unready": {"lowstate": {"ready": False}},
        "active_motor_commands": {"lowcmd_traffic_samples": 3},
        "sensor_conversion_errors": {"conversion_errors": 1},
        "extrinsic_uncommissioned": {"extrinsic_commissioned": False},
    }
    for issue, change in cases.items():
        assert probe.verdict({**ready, **change}) == [issue]


def test_probe_never_creates_writers_or_command_sockets() -> None:
    tree = ast.parse(Path(inspect.getfile(probe)).read_text())
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    names |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
    assert not names & {
        "DataWriter",
        "Publisher",
        "ChannelPublisher",
        "LocoClient",
        "PUB",
        "send",
        "write",
    }
