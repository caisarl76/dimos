# Copyright 2025-2026 Dimensional Inc.
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

"""Sensor-chain progress on the peer's serialized loop; no wall-clock aging."""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import math
from numbers import Real
from types import MappingProxyType

INPUT_BUDGETS: Mapping[str, float] = MappingProxyType(
    {
        "lidar_raw": 0.350,
        "imu_raw": 0.200,
        "lio_cloud": 0.350,
        "odom": 0.350,
        "base_tf": 0.350,
        "global_map": 1.000,
        "global_costmap": 1.000,
    }
)
_PARENTS: Mapping[str, tuple[str, ...]] = {
    "lidar_raw": (),
    "imu_raw": (),
    "lio_cloud": ("lidar_raw", "imu_raw"),
    "odom": ("lidar_raw", "imu_raw"),
    "base_tf": ("odom",),
    "global_map": ("lio_cloud",),
    "global_costmap": ("global_map",),
}


@dataclass(frozen=True)
class InputProgress:
    name: str
    source_stamp: float
    age: float
    upstream_lag: float
    valid: bool


@dataclass(frozen=True)
class ReadinessSnapshot:
    ready: bool
    inputs: tuple[InputProgress, ...]
    fault: str | None


@dataclass(frozen=True)
class _Source:
    stamp: float
    advanced_at: float
    valid: bool


def _finite(value: float) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _ancestors(name: str) -> set[str]:
    result = set(_PARENTS[name])
    for parent in _PARENTS[name]:
        result.update(_ancestors(parent))
    return result


class NavigationReadiness:
    """Duplicates cannot renew progress; source rollback needs a fresh instance.

    Clock domains are an explicit, verified adapter prerequisite. Unknown or
    different domains cannot be compared. Invalid/empty outputs never establish
    a source-clock high-water mark, even when they carry regenerated stamps.
    """

    def __init__(
        self, *, extrinsic_valid: bool = False, clock_domains: Mapping[str, str] | None = None
    ) -> None:
        if type(extrinsic_valid) is not bool:
            raise ValueError("explicit extrinsic validity required")
        self._extrinsic_valid = extrinsic_valid
        self._domains = dict(clock_domains or {})
        if any(
            name not in INPUT_BUDGETS or not domain or not isinstance(domain, str)
            for name, domain in self._domains.items()
        ):
            raise ValueError("known input names and nonempty clock domains required")
        self._sources: dict[str, _Source] = {}
        self._fault: str | None = None

    def observe(self, name: str, stamp: float, *, valid: bool, received_monotonic: float) -> None:
        if name not in INPUT_BUDGETS or type(valid) is not bool:
            raise ValueError("known input and explicit validity required")
        if not _finite(received_monotonic) or received_monotonic < 0:
            raise ValueError("finite nonnegative peer receipt time required")
        previous = self._sources.get(name)
        if not valid or not _finite(stamp) or stamp <= 0:
            if previous is not None:
                self._sources[name] = replace(previous, valid=False)
            return
        if previous is not None:
            if stamp < previous.stamp:
                self._fault = self._fault or f"source_stamp_rollback:{name}"
                return
            if stamp == previous.stamp or received_monotonic < previous.advanced_at:
                return
        self._sources[name] = _Source(float(stamp), float(received_monotonic), True)

    def snapshot(self, *, now: float) -> ReadinessSnapshot:
        if not _finite(now) or now < 0:
            raise ValueError("finite nonnegative peer time required")
        fault = self._fault or (None if self._extrinsic_valid else "extrinsic_unverified")
        progress = []
        for name, budget in INPUT_BUDGETS.items():
            source = self._sources.get(name)
            issue = None
            lag = 0.0
            if source is None:
                issue = f"missing:{name}"
            elif not source.valid:
                issue = f"invalid:{name}"
            elif now < source.advanced_at:
                issue = f"future_receipt:{name}"
            elif now >= source.advanced_at + budget:
                issue = f"stale:{name}"
            elif name not in self._domains:
                issue = f"clock_domain_unknown:{name}"
            else:
                for parent in _ancestors(name):
                    upstream = self._sources.get(parent)
                    if self._domains.get(parent) != self._domains[name]:
                        issue = f"clock_domain_mismatch:{name}"
                        break
                    if upstream is None or not upstream.valid:
                        issue = f"upstream_unready:{name}"
                        break
                    lag = max(lag, upstream.stamp - source.stamp)
                if issue is None and lag >= budget:
                    issue = f"upstream_lag:{name}"
            if fault is None:
                fault = issue
            progress.append(
                InputProgress(
                    name,
                    0.0 if source is None else source.stamp,
                    budget if source is None else max(0.0, now - source.advanced_at),
                    lag,
                    issue is None,
                )
            )
        return ReadinessSnapshot(fault is None, tuple(progress), fault)
