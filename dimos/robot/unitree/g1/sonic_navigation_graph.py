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

"""Exact per-host graph validation for the two independent SONIC navigation blueprints.

Each role names its allowed modules and the exact resolved producer -> consumer
stream edges, so extra modules, alternate sources, extra consumers and
remappings all fail before any worker, socket or control-mode startup. This
cannot see processes outside the blueprint; inspect runtime ownership before
launch.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from collections.abc import Mapping
from dataclasses import dataclass
import re
from typing import Any, Literal

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.go2.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
    SonicNavigationSensorReceiver,
    SonicNavigationSensorSender,
)
from dimos.robot.unitree.g1.sonic_service_module import SonicServiceModule

Role = Literal["peer", "pc2"]
Edge = tuple[type[ModuleBase], str, type[ModuleBase], str]

_ENDPOINT = re.compile(r"^tcp://[^:/\s]+:(\d{1,5})$")


@dataclass(frozen=True)
class _RoleGraph:
    required: frozenset[type[ModuleBase]]
    optional: frozenset[type[ModuleBase]]
    edges: frozenset[Edge]
    endpoints: tuple[tuple[type[ModuleBase], str], ...]


_R, _PL, _RM = SonicNavigationSensorReceiver, PointLio, RayTracingVoxelMap
_CM, _AS, _MM, _P = CostMapper, ReplanningAStarPlanner, MovementManager, SonicNavigationPeerModule

ROLES: Mapping[Role, _RoleGraph] = {
    "peer": _RoleGraph(
        required=frozenset({_R, _PL, _RM, _CM, _AS, _MM, _P}),
        optional=frozenset(),
        edges=frozenset(
            {
                (_R, "lidar_raw", _PL, "lidar_raw"),
                (_R, "lidar_raw", _P, "lidar_raw"),
                (_R, "imu_raw", _PL, "imu_raw"),
                (_R, "imu_raw", _P, "imu_raw"),
                (_PL, "odometry", _R, "lio_odometry"),
                (_PL, "lidar", _RM, "lidar"),
                (_PL, "lidar", _P, "lidar"),
                (_PL, "tf", _RM, "tf"),
                (_PL, "tf", _P, "tf"),
                (_R, "tf", _RM, "tf"),
                (_R, "tf", _P, "tf"),
                (_R, "odometry", _AS, "odometry"),
                (_R, "odometry", _P, "odometry"),
                (_RM, "global_map", _CM, "global_map"),
                (_RM, "global_map", _P, "global_map"),
                (_CM, "global_costmap", _AS, "global_costmap"),
                (_CM, "global_costmap", _P, "global_costmap"),
                (_AS, "nav_cmd_vel", _MM, "nav_cmd_vel"),
                (_MM, "stop_movement", _AS, "stop_movement"),
                (_MM, "cmd_vel", _P, "cmd_vel"),
            }
        ),
        endpoints=((_R, "cloud_endpoint"), (_R, "state_endpoint"), (_P, "link_bind")),
    ),
    "pc2": _RoleGraph(
        required=frozenset(
            {SonicNavigationSensorSender, SonicNavigationRelayModule, SonicExecutionModule}
        ),
        optional=frozenset({SonicServiceModule}),
        edges=frozenset(
            {
                (
                    SonicNavigationRelayModule,
                    "navigation_input",
                    SonicExecutionModule,
                    "navigation_input",
                )
            }
        ),
        endpoints=(
            (SonicNavigationSensorSender, "cloud_bind"),
            (SonicNavigationSensorSender, "state_bind"),
            (SonicNavigationRelayModule, "peer_endpoint"),
            (SonicExecutionModule, "reference_bind"),
            (SonicExecutionModule, "feedback_endpoint"),
        ),
    ),
}


def resolved_edges(blueprint: Blueprint) -> set[Edge]:
    producers: dict[tuple[str, type], list[tuple[type[ModuleBase], str]]] = defaultdict(list)
    consumers: dict[tuple[str, type], list[tuple[type[ModuleBase], str]]] = defaultdict(list)
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            key = (blueprint.remapping_map.get((atom.name, stream.name), stream.name), stream.type)
            if stream.direction in ("out", "inout"):
                producers[key].append((atom.module, stream.name))  # type: ignore[index]
            if stream.direction in ("in", "inout"):
                consumers[key].append((atom.module, stream.name))  # type: ignore[index]
    return {(*p, *c) for key, ps in producers.items() for p in ps for c in consumers.get(key, [])}


def validate_sonic_navigation_graph(
    blueprint: Blueprint,
    module_kwargs: Mapping[str, Mapping[str, Any]] | None = None,
    *,
    role: Role,
) -> None:
    """Raise ValueError unless this is exactly the ``role`` graph; a Blueprint validator."""
    graph = ROLES[role]
    atoms = blueprint.active_blueprints
    kwargs = {
        atom.module: {**atom.kwargs, **(module_kwargs or {}).get(atom.name, {})} for atom in atoms
    }
    counts = Counter(atom.module for atom in atoms)
    problems = []
    unknown = sorted(m.__name__ for m in counts if m not in graph.required | graph.optional)
    if unknown:
        problems.append(f"modules not allowed on {role}: {', '.join(unknown)}")
    duplicated = sorted(m.__name__ for m, n in counts.items() if n > 1)
    if duplicated:
        problems.append(f"duplicate owners: {', '.join(duplicated)}")
    missing = sorted(m.__name__ for m in graph.required if m not in counts)
    if missing:
        problems.append(f"missing required modules: {', '.join(missing)}")
    if any(not isinstance(target, str) for target in blueprint.remapping_map.values()):
        problems.append("module reference remapping is not permitted")
    if not unknown and not duplicated:
        actual = resolved_edges(blueprint)

        def names(edges: set[Edge]) -> str:
            return ", ".join(sorted(f"{a.__name__}.{b}->{c.__name__}.{d}" for a, b, c, d in edges))

        if actual - graph.edges:
            problems.append(f"unexpected connection: {names(actual - graph.edges)}")
        if graph.edges - actual:
            problems.append(f"missing connection: {names(set(graph.edges) - actual)}")
    ports: dict[str, str] = {}
    for module, field in graph.endpoints:
        if module not in kwargs:
            continue
        value = kwargs[module].get(field)
        match = _ENDPOINT.match(value) if isinstance(value, str) else None
        label = f"{module.__name__}.{field}"
        if match is None:
            problems.append(f"endpoint {label} must be an explicit tcp://host:port")
        elif match[1] in ports:
            problems.append(f"endpoint {label} reuses port {match[1]} of {ports[match[1]]}")
        else:
            ports[match[1]] = label
    if role == "peer":
        if kwargs.get(CostMapper, {}).get("initial_safe_radius_meters", 0.0) != 0:
            problems.append("CostMapper initial_safe_radius_meters must be 0")
        if kwargs.get(ReplanningAStarPlanner, {}).get("min_angular_velocity") != 0:
            problems.append("ReplanningAStarPlanner min_angular_velocity must be 0")
        # The pc2 execution rejects walking above 0.2 m/s, so an uncapped follower never moves.
        nerf = kwargs.get(ReplanningAStarPlanner, {}).get("nerf_speed", 1.0)
        if not isinstance(nerf, (int, float)) or LocalPlanner._speed * nerf > 0.2 + 1e-9:
            problems.append("ReplanningAStarPlanner nerf_speed must cap the follower at 0.2 m/s")
        claimed = kwargs.get(SonicNavigationPeerModule, {}).get("extrinsic_valid", False)
        if claimed and kwargs.get(SonicNavigationSensorReceiver, {}).get("torso_to_sensor") is None:
            problems.append("extrinsic_valid requires a commissioned torso_to_sensor extrinsic")
    elif kwargs.get(SonicExecutionModule, {}).get("navigation_enabled") is not True:
        problems.append("SonicExecutionModule navigation_enabled must be True")
    if problems:
        raise ValueError(f"SONIC navigation {role} graph rejected: " + "; ".join(problems))
