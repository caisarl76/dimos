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

"""Passive pc2 navigation readiness probe for `launch.sh --check-only`.

Creates DDS readers only (sensor, lowstate, lowcmd traffic and endpoint
discovery) and reads /proc and /sys. It never creates a DDS writer or ZMQ
socket and sends no start, stop, mode or motion command. It runs the production
sensor conversions on live samples to validate them on ARM64. The JSON report is
always written; the exit status is nonzero when any blocking check fails.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from itertools import pairwise
import json
import math
import os
from pathlib import Path
import time
from typing import Any

BUDGETS = {"lidar_raw": 0.350, "imu_raw": 0.200, "lowstate": 0.200}
SONIC_PROCESS = "g1_deploy_onnx_ref"
GPU_DEVFREQ = "class/devfreq/17000000.gpu"


def source_stall(rows: list[tuple[float, float]]) -> float:
    """Longest receipt time from a source stamp's first arrival until the stamp advances."""
    if not rows:
        return math.inf
    worst, since = 0.0, rows[0][0]
    for a, b in pairwise(rows):
        if b[1] > a[1]:
            worst, since = max(worst, b[0] - since), b[0]
    return max(worst, rows[-1][0] - since)


def stream_progress(
    rows: list[tuple[float, float]], budget: float, *, repeats_ok: bool = False
) -> dict[str, Any]:
    """rows: (local receipt monotonic, original source stamp) in receipt order.

    repeats_ok: distinct samples may share a coarse stamp (lowstate's 1 ms tick at
    ~1040 Hz); the stamp must still advance within the budget.
    """
    rollbacks = sum(b[1] < a[1] for a, b in pairwise(rows))
    duplicates = sum(b[1] == a[1] for a, b in pairwise(rows))
    gaps = [b[0] - a[0] for a, b in pairwise(rows)]
    span = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0.0
    result = {
        "samples": len(rows),
        "rate_hz": (len(rows) - 1) / span if span > 0 else 0.0,
        "max_receipt_gap_s": max(gaps, default=math.inf),
        "max_source_stall_s": source_stall(rows),
        "rollbacks": rollbacks,
        "duplicates": duplicates,
        "first_source_stamp": rows[0][1] if rows else None,
        "last_source_stamp": rows[-1][1] if rows else None,
        "budget_s": budget,
    }
    result["ready"] = (
        len(rows) >= 2
        and rollbacks == 0
        and (repeats_ok or duplicates == 0)
        and rows[-1][1] > rows[0][1]
        and result["max_receipt_gap_s"] < budget
        and result["max_source_stall_s"] < budget
    )
    return result


def clocks_pinned(sysfs: Path) -> dict[str, Any]:
    """The state `sudo jetson_clocks` produces: CPU min == hardware max, GPU min == max."""
    issues, checked = [], 0
    for cpufreq in sorted(sysfs.glob("devices/system/cpu/cpu[0-9]*/cpufreq")):
        low, high = cpufreq / "scaling_min_freq", cpufreq / "cpuinfo_max_freq"
        if low.is_file() and high.is_file():
            checked += 1
            if low.read_text().strip() != high.read_text().strip():
                issues.append(
                    f"{cpufreq.parent.name} min={low.read_text().strip()} max={high.read_text().strip()}"
                )
    gpu = sysfs / GPU_DEVFREQ
    if (gpu / "min_freq").is_file() and (gpu / "max_freq").is_file():
        checked += 1
        low_gpu, high_gpu = (
            (gpu / "min_freq").read_text().strip(),
            (gpu / "max_freq").read_text().strip(),
        )
        if low_gpu != high_gpu:
            issues.append(f"gpu min={low_gpu} max={high_gpu}")
    if not checked:
        issues.append("no readable CPU/GPU frequency limits")
    return {"pinned": not issues, "checked": checked, "issues": issues}


def listening_ports(proc: Path) -> set[int]:
    ports = set()
    for name in ("net/tcp", "net/tcp6"):
        path = proc / name
        if not path.is_file():
            continue
        for line in path.read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) > 3 and fields[3] == "0A":  # TCP_LISTEN
                ports.add(int(fields[1].rsplit(":", 1)[1], 16))
    return ports


def running(proc: Path, names: tuple[str, ...]) -> list[dict[str, Any]]:
    found = []
    for cmdline in proc.glob("[0-9]*/cmdline"):
        try:
            argv = cmdline.read_bytes().split(b"\0")
        except OSError:
            continue
        program = Path(argv[0].decode(errors="replace")).name if argv and argv[0] else ""
        if program in names:
            found.append({"pid": int(cmdline.parent.name), "program": program})
    return sorted(found, key=lambda p: p["pid"])


def lowstate_health(samples: list[dict[str, Any]], *, expected_joints: int) -> dict[str, Any]:
    finite = all(
        len(s["q"]) == expected_joints and all(math.isfinite(v) for v in s["q"] + list(s["imu"]))
        for s in samples
    )
    ticks = [s["tick"] for s in samples]
    advancing = len(ticks) > 1 and ticks[-1] > ticks[0] and all(b >= a for a, b in pairwise(ticks))
    return {
        "samples": len(samples),
        "joints": len(samples[-1]["q"]) if samples else 0,
        "finite": finite,
        "tick_advancing": advancing,
        "ready": bool(samples) and finite and advancing,
    }


def verdict(report: dict[str, Any]) -> list[str]:
    issues = []
    if not report["clocks"]["pinned"]:
        issues.append("clocks_not_pinned")
    if report["sonic_processes"]:
        issues.append("sonic_process_running")
    issues += [f"task_ports_in_use:{port}" for port in report["ports_in_use"]]
    issues += [f"stream_unready:{n}" for n, s in report["streams"].items() if not s["ready"]]
    if not report["lowstate"]["ready"]:
        issues.append("lowstate_unready")
    if report["lowcmd_traffic_samples"]:
        issues.append("active_motor_commands")
    if report["conversion_errors"]:
        issues.append("sensor_conversion_errors")
    if not report["extrinsic_commissioned"]:
        issues.append("extrinsic_uncommissioned")
    return issues


def host_load(proc: Path, sysfs: Path) -> dict[str, Any]:
    meminfo = dict(
        line.split(":", 1) for line in (proc / "meminfo").read_text().splitlines() if ":" in line
    )
    gpu_load = next(iter(sysfs.glob("devices/platform/**/17000000.gpu/load")), None)
    return {
        "loadavg": (proc / "loadavg").read_text().split()[:3],
        "cpus": os.cpu_count(),
        "mem_available_kb": int(meminfo.get("MemAvailable", "0 kB").split()[0]),
        "gpu_load_permille": int(gpu_load.read_text()) if gpu_load else None,
    }


@dataclass
class DdsObservation:
    streams: dict[str, list[tuple[float, float]]]
    lowstate: list[dict[str, Any]]
    lowcmd_traffic_samples: int
    command_writers: list[dict[str, Any]]
    conversion_errors: int


def observe_dds(interface: str, domain: int, seconds: float) -> DdsObservation:
    """Readers only. Lazy imports: these packages exist in the pc2 task image."""
    from cyclonedds.builtin import (  # type: ignore[import-not-found]
        BuiltinDataReader,
        BuiltinTopicDcpsPublication,
    )
    from cyclonedds.domain import DomainParticipant  # type: ignore[import-not-found]
    from cyclonedds.qos import Policy, Qos  # type: ignore[import-not-found]
    from cyclonedds.sub import DataReader  # type: ignore[import-not-found]
    from cyclonedds.topic import Topic  # type: ignore[import-not-found]
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,  # type: ignore[import-not-found]
    )
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import (
        PointCloud2_,  # type: ignore[import-not-found]
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # type: ignore[import-not-found]
        LowCmd_,
        LowState_,
    )

    from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
        encode_cloud,
        encode_imu,
        encode_waist,
        livox_imu_type,
    )

    Imu_ = livox_imu_type()

    ChannelFactoryInitialize(domain, interface)
    participant = DomainParticipant(domain)
    qos = Qos(
        Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(256)
    )
    readers = {
        name: DataReader(participant, Topic(participant, topic, kind), qos=qos)
        for name, topic, kind in (
            ("lidar_raw", "rt/utlidar/cloud_livox_mid360", PointCloud2_),
            ("imu_raw", "rt/utlidar/imu_livox_mid360", Imu_),
            ("lowstate", "rt/lowstate", LowState_),
            ("lowcmd", "rt/lowcmd", LowCmd_),
        )
    }
    publications = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
    observed = DdsObservation({n: [] for n in BUDGETS}, [], 0, [], 0)
    writers: dict[str, dict[str, Any]] = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        for sample in publications.take(256):
            if sample.sample_info.valid_data and sample.topic_name in ("rt/lowcmd", "rt/arm_sdk"):
                writers[str(sample.key)] = {
                    "topic": sample.topic_name,
                    "participant": str(sample.participant_key),
                }
        for name, reader in readers.items():
            for sample in reader.take(256):
                if not sample.sample_info.valid_data:
                    continue
                now = time.monotonic()
                if name == "lowcmd":
                    observed.lowcmd_traffic_samples += 1
                    continue
                try:
                    if name == "lowstate":
                        encode_waist(sample)
                        stamp = int(sample.tick) * 1e-3
                        imu = sample.imu_state
                        observed.lowstate.append(
                            {
                                "tick": int(sample.tick),
                                "q": [float(m.q) for m in sample.motor_state[:29]],
                                "imu": [*imu.quaternion, *imu.gyroscope, *imu.accelerometer],
                            }
                        )
                    else:
                        (encode_cloud if name == "lidar_raw" else encode_imu)(sample)
                        stamp = sample.header.stamp.sec + sample.header.stamp.nanosec * 1e-9
                except (ValueError, AttributeError, IndexError):
                    observed.conversion_errors += 1
                    continue
                observed.streams[name].append((now, stamp))
        time.sleep(0.002)
    observed.command_writers = sorted(
        writers.values(), key=lambda w: (w["topic"], w["participant"])
    )
    return observed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--proc", type=Path, default=Path("/proc"))
    parser.add_argument("--sysfs", type=Path, default=Path("/sys"))
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    started = time.time()
    dds = observe_dds(config["interface"], config["dds_domain"], args.seconds)
    ports = set(config["local_ports"].values())
    report: dict[str, Any] = {
        "started_unix": started,
        "observation_seconds": args.seconds,
        "creates_writers_or_command_sockets": False,
        "clocks": clocks_pinned(args.sysfs),
        "sonic_processes": running(args.proc, (SONIC_PROCESS,)),
        "ports_in_use": sorted(ports & listening_ports(args.proc)),
        "streams": {
            n: stream_progress(rows, BUDGETS[n], repeats_ok=n == "lowstate")
            for n, rows in dds.streams.items()
        },
        "lowstate": lowstate_health(dds.lowstate, expected_joints=config["body_joints"]),
        "lowcmd_traffic_samples": dds.lowcmd_traffic_samples,
        "command_writers_discovered": dds.command_writers,
        "conversion_errors": dds.conversion_errors,
        "extrinsic_commissioned": config.get("torso_to_sensor") is not None,
        "host_load": host_load(args.proc, args.sysfs),
    }
    report["blocking"] = verdict(report)
    report["ready"] = not report["blocking"]
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"ready": report["ready"], "blocking": report["blocking"]}))
    raise SystemExit(0 if report["ready"] else 1)


if __name__ == "__main__":
    main()
