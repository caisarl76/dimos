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

"""Simulation-only CycloneDDS reader stand-in for the two-host stage.

Provides only the reader calls ``SonicNavigationSensorSender._open_readers``
makes; there is no writer. MID360S cloud/IMU replay the pinned recorded payloads
at real-time pace with their recorded spacing, offset by one recording span per
loop. ``rt/lowstate`` is derived from MuJoCo physics (waist joints, sim-time
tick). A ``freeze-<channel>`` file in the run folder withholds that source while
time keeps passing. Never on the path of a real robot process.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace
from typing import Any

import numpy as np

_OUTPUT = Path(os.environ["SONIC_TWO_HOST_OUTPUT"])
_REPLAY = Path(os.environ["SONIC_TWO_HOST_REPLAY"])


def _register(name: str, **attrs: Any) -> None:
    module = ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module


class IdlStruct:
    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__()


def autoid(_: str) -> Any:
    return lambda cls: cls


def final(cls: Any) -> Any:
    return cls


class _Array:
    def __getitem__(self, _: Any) -> type:
        return list


class Policy:
    Reliability = SimpleNamespace(BestEffort="best_effort")
    Durability = SimpleNamespace(Volatile="volatile")
    History = SimpleNamespace(KeepLast=lambda depth: depth)


class Qos:
    def __init__(self, *policies: Any) -> None:
        self.policies = policies


class DomainParticipant:
    def __init__(self, domain: int = 0) -> None:
        self.domain = domain


class Topic:
    def __init__(self, participant: DomainParticipant, name: str, kind: Any) -> None:
        self.name = name


def _header(stamp: float) -> SimpleNamespace:
    ns = round(stamp * 1e9)
    return SimpleNamespace(stamp=SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9))


def _sample(**fields: Any) -> SimpleNamespace:
    return SimpleNamespace(sample_info=SimpleNamespace(valid_data=True), **fields)


class _Recording:
    """One shared schedule so cloud and IMU stay in one recorded clock domain."""

    def __init__(self) -> None:
        with np.load(_REPLAY) as data:
            self.imu = np.asarray(data["imu"])
            self.clouds = np.asarray(data["cloud_stamps"])
            self.points = np.asarray(data["last_cloud"])
        # Loop on whole cloud periods inside the IMU coverage, keeping only the IMU
        # samples in that window, so neither stream has an artificial gap at a wrap.
        self.clouds = self.clouds[self.clouds[:, 1] >= self.imu[0, 1]]
        period = float(np.median(np.diff(self.clouds[:, 1])))
        self.base = float(self.clouds[0, 1])
        self.span = float(self.clouds[-1, 1]) - self.base + period
        inside = (self.imu[:, 1] >= self.base) & (self.imu[:, 1] < self.base + self.span)
        self.imu = self.imu[inside]
        assert self.base + self.span - self.imu[-1, 1] < 0.02, "IMU must cover the loop"
        # One replay clock per run, shared by restarted sender processes: a real
        # sensor keeps its clock when the DimOS process restarts.
        epoch = _OUTPUT / "replay-epoch.json"
        if not epoch.exists():
            epoch.write_text(json.dumps({"monotonic": time.monotonic()}))
        self.started = float(json.loads(epoch.read_text())["monotonic"])

    def due(self, stamp: float) -> bool:
        return time.monotonic() >= self.started + stamp - self.base

    def resume_index(self, rows: Any) -> int:
        """First sample at or after the current replay time."""
        elapsed = max(0.0, time.monotonic() - self.started)
        loop, offset = divmod(elapsed, self.span)
        return int(loop) * len(rows) + int(np.searchsorted(rows[:, 1] - self.base, offset))


_RECORDING: _Recording | None = None


def _recording() -> _Recording:
    global _RECORDING
    if _RECORDING is None:
        _RECORDING = _Recording()
    return _RECORDING


class _Replay:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.recording = _recording()
        rows = self.recording.imu if channel == "imu_raw" else self.recording.clouds
        self.index = self.recording.resume_index(rows)

    def stamp(self, rows: Any) -> tuple[float, Any]:
        loop, row = divmod(self.index, len(rows))
        return float(rows[row, 1]) + loop * self.recording.span, rows[row]

    def take(self, limit: int) -> list[Any]:
        rec = self.recording
        rows = rec.imu if self.channel == "imu_raw" else rec.clouds
        frozen = (_OUTPUT / f"freeze-{self.channel}").exists()
        out: list[Any] = []
        while len(out) < limit:
            stamp, row = self.stamp(rows)
            if not rec.due(stamp):
                break
            self.index += 1
            if frozen:
                continue  # time passes; withheld samples are never replayed late
            if self.channel == "imu_raw":
                out.append(
                    _sample(
                        header=_header(stamp),
                        angular_velocity=SimpleNamespace(x=row[5], y=row[6], z=row[7]),
                        linear_acceleration=SimpleNamespace(x=row[2], y=row[3], z=row[4]),
                        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                        orientation_covariance=[0.0] * 9,
                        angular_velocity_covariance=[0.0] * 9,
                        linear_acceleration_covariance=[0.0] * 9,
                    )
                )
            else:
                points = rec.points
                out.append(
                    _sample(
                        header=_header(stamp),
                        is_bigendian=False,
                        is_dense=True,
                        height=1,
                        width=len(points),
                        point_step=points.dtype.itemsize,
                        row_step=points.nbytes,
                        data=points.tobytes(),
                        fields=[
                            SimpleNamespace(
                                name=name,
                                offset=points.dtype.fields[name][1],
                                datatype=7,
                                count=1,
                            )
                            for name in ("x", "y", "z", "time")
                        ],
                    )
                )
        return out


class _LowState:
    """Waist joints from MuJoCo qpos; tick follows the physics row's monotonic stamp.

    Simulation time restarts when the support band releases; a robot's lowstate
    tick does not, so it must not be used as the tick source.
    """

    def __init__(self) -> None:
        self.tick = -1

    def take(self, limit: int) -> list[Any]:
        if (_OUTPUT / "freeze-waist_tf").exists():
            return []
        try:
            row = json.loads((_OUTPUT / "physics-latest.json").read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return []
        tick = int(row["monotonic"] * 1000)
        if tick <= self.tick:
            return []
        self.tick = tick
        joints = row["qpos"][7:]
        return [_sample(tick=tick, motor_state=[SimpleNamespace(q=q) for q in joints])]


_SOURCES = {
    "rt/utlidar/cloud_livox_mid360": lambda: _Replay("lidar_raw"),
    "rt/utlidar/imu_livox_mid360": lambda: _Replay("imu_raw"),
    "rt/lowstate": _LowState,
}


class DataReader:
    def __init__(self, participant: DomainParticipant, topic: Topic, qos: Qos | None = None):
        self._source = _SOURCES[topic.name]()

    def take(self, limit: int) -> list[Any]:
        return self._source.take(limit)  # type: ignore[no-any-return]


_register("cyclonedds.domain", DomainParticipant=DomainParticipant)
_register("cyclonedds.idl", IdlStruct=IdlStruct)
_register("cyclonedds.idl.annotations", autoid=autoid, final=final)
_register("cyclonedds.idl.types", array=_Array(), float64=float)
_register("cyclonedds.qos", Policy=Policy, Qos=Qos)
_register("cyclonedds.sub", DataReader=DataReader)
_register("cyclonedds.topic", Topic=Topic)
