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

"""Both navigation graphs reject forbidden owners before any startup side effect."""

import importlib
import sys
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import CostMapper
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_pc2 import (
    unitree_g1_sonic_navigation_pc2,
)
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_peer import (
    unitree_g1_sonic_navigation_peer,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_navigation_graph import validate_sonic_navigation_graph
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import (
    SonicNavigationSensorReceiver,
    SonicNavigationSensorSender,
)
from dimos.robot.unitree.g1.sonic_service_module import SonicServiceModule
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

PEER_KWARGS = {
    SonicNavigationSensorReceiver.name: {
        "cloud_endpoint": "tcp://192.0.2.10:7101",
        "state_endpoint": "tcp://192.0.2.10:7102",
    },
    SonicNavigationPeerModule.name: {"link_bind": "tcp://0.0.0.0:7103"},
}
PC2_KWARGS = {
    SonicNavigationSensorSender.name: {
        "cloud_bind": "tcp://0.0.0.0:7101",
        "state_bind": "tcp://0.0.0.0:7102",
    },
    SonicNavigationRelayModule.name: {"peer_endpoint": "tcp://192.0.2.20:7103"},
    SonicExecutionModule.name: {
        "reference_bind": "tcp://127.0.0.1:5556",
        "feedback_endpoint": "tcp://127.0.0.1:5557",
        "navigation_enabled": True,
    },
}


def resolved(blueprint: Blueprint, overrides: dict[str, dict[str, Any]]) -> dict[str, Any]:
    return {
        atom.name: {**atom.kwargs, **overrides.get(atom.name, {})}
        for atom in blueprint.active_blueprints
    }


def check(blueprint: Blueprint, role: Any, overrides: dict[str, dict[str, Any]]) -> None:
    validate_sonic_navigation_graph(blueprint, resolved(blueprint, overrides), role=role)


class AlternateCloud(Module):
    lidar_raw: Out[PointCloud2]


@pytest.fixture
def stock_sdk(mocker):
    names = [
        "unitree_sdk2py",
        "unitree_sdk2py.comm",
        "unitree_sdk2py.comm.motion_switcher",
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client",
        "unitree_sdk2py.core",
        "unitree_sdk2py.core.channel",
        "unitree_sdk2py.g1",
        "unitree_sdk2py.g1.loco",
        "unitree_sdk2py.g1.loco.g1_loco_api",
        "unitree_sdk2py.g1.loco.g1_loco_client",
    ]
    mocker.patch.dict(sys.modules, {name: MagicMock() for name in names})
    module = importlib.import_module("dimos.robot.unitree.g1.effectors.high_level.dds_sdk")
    return module.G1HighLevelDdsSdk


def test_valid_graphs_pass_independently() -> None:
    check(unitree_g1_sonic_navigation_peer, "peer", PEER_KWARGS)
    check(unitree_g1_sonic_navigation_pc2, "pc2", PC2_KWARGS)
    with_service = autoconnect(unitree_g1_sonic_navigation_pc2, SonicServiceModule.blueprint())
    check(with_service, "pc2", PC2_KWARGS)


def test_graphs_are_role_specific() -> None:
    with pytest.raises(ValueError, match="not allowed"):
        check(unitree_g1_sonic_navigation_peer, "pc2", PC2_KWARGS)
    with pytest.raises(ValueError, match="not allowed"):
        check(unitree_g1_sonic_navigation_pc2, "peer", PEER_KWARGS)


@pytest.mark.parametrize(
    "extra",
    [FastLio2, G1WholeBodyConnection, AlternateCloud, MovementManager],
    ids=["fastlio2", "wholebody", "unknown-alternate-source", "second-cmd-vel-source"],
)
def test_peer_rejects_forbidden_or_duplicate_modules(extra: type[Module]) -> None:
    blueprint = autoconnect(unitree_g1_sonic_navigation_peer, extra.blueprint())
    if extra is MovementManager:
        # Same-class duplicates collapse in autoconnect; force a distinct instance.
        blueprint = autoconnect(
            unitree_g1_sonic_navigation_peer,
            MovementManager.blueprint(instance_name="second_movement"),
        )
    with pytest.raises(ValueError, match="not allowed|duplicate owners"):
        check(blueprint, "peer", PEER_KWARGS)


def test_stock_sdk_rejected_on_both_roles(stock_sdk) -> None:
    for base, role, kwargs in [
        (unitree_g1_sonic_navigation_peer, "peer", PEER_KWARGS),
        (unitree_g1_sonic_navigation_pc2, "pc2", PC2_KWARGS),
    ]:
        with pytest.raises(ValueError, match="G1HighLevelDdsSdk"):
            check(autoconnect(base, stock_sdk.blueprint()), role, kwargs)


def test_pc2_rejects_second_execution_owner() -> None:
    blueprint = autoconnect(
        unitree_g1_sonic_navigation_pc2,
        SonicExecutionModule.blueprint(instance_name="second_execution"),
    )
    with pytest.raises(ValueError, match="SonicExecutionModule"):
        check(blueprint, "pc2", PC2_KWARGS)


@pytest.mark.parametrize(
    "remapping",
    [
        (PointLio, "odometry", "odometry"),  # LIO pose bypasses base composition
        (MovementManager, "tele_cmd_vel", "cmd_vel"),  # cmd_vel gains a second consumer
        (SonicNavigationPeerModule, "cmd_vel", "nav_cmd_vel"),  # peer bypasses the manager
        (CostMapper, "merged_map", "global_map"),  # merged-map input re-enabled
    ],
    ids=["alternate-odometry", "second-consumer", "bypass-manager", "merged-map"],
)
def test_remapping_cannot_bypass_peer_edges(remapping: Any) -> None:
    with pytest.raises(ValueError, match="connection"):
        check(unitree_g1_sonic_navigation_peer.remappings([remapping]), "peer", PEER_KWARGS)


def test_pc2_has_no_cmd_vel_and_one_navigation_input_producer() -> None:
    blueprint = unitree_g1_sonic_navigation_pc2.remappings(
        [(SonicExecutionModule, "navigation_input", "cmd_vel")]
    )
    with pytest.raises(ValueError, match="connection"):
        check(blueprint, "pc2", PC2_KWARGS)


def test_disabling_a_required_owner_is_rejected() -> None:
    blueprint = unitree_g1_sonic_navigation_pc2.disabled_modules(SonicExecutionModule)
    with pytest.raises(ValueError, match="missing"):
        check(blueprint, "pc2", PC2_KWARGS)


def test_endpoint_ownership() -> None:
    shared = {
        **PC2_KWARGS,
        SonicNavigationSensorSender.name: {
            "cloud_bind": "tcp://0.0.0.0:5556",  # native command endpoint port
            "state_bind": "tcp://0.0.0.0:7102",
        },
    }
    with pytest.raises(ValueError, match="endpoint"):
        check(unitree_g1_sonic_navigation_pc2, "pc2", shared)
    missing = {**PEER_KWARGS, SonicNavigationPeerModule.name: {}}
    with pytest.raises(ValueError, match="endpoint"):
        check(unitree_g1_sonic_navigation_peer, "peer", missing)
    ephemeral = {**PEER_KWARGS, SonicNavigationPeerModule.name: {"link_bind": "tcp://0.0.0.0:*"}}
    with pytest.raises(ValueError, match="endpoint"):
        check(unitree_g1_sonic_navigation_peer, "peer", ephemeral)


def test_safety_configuration_is_enforced() -> None:
    disabled = {**PC2_KWARGS}
    disabled[SonicExecutionModule.name] = {
        **PC2_KWARGS[SonicExecutionModule.name],
        "navigation_enabled": False,
    }
    with pytest.raises(ValueError, match="navigation_enabled"):
        check(unitree_g1_sonic_navigation_pc2, "pc2", disabled)
    radius = {**PEER_KWARGS, CostMapper.name: {"initial_safe_radius_meters": 0.5}}
    with pytest.raises(ValueError, match="initial_safe_radius"):
        check(unitree_g1_sonic_navigation_peer, "peer", radius)
    turn_floor = {**PEER_KWARGS, ReplanningAStarPlanner.name: {"min_angular_velocity": 0.2}}
    with pytest.raises(ValueError, match="min_angular_velocity"):
        check(unitree_g1_sonic_navigation_peer, "peer", turn_floor)
    claimed = {
        **PEER_KWARGS,
        SonicNavigationPeerModule.name: {
            "link_bind": "tcp://0.0.0.0:7103",
            "extrinsic_valid": True,
        },
    }
    with pytest.raises(ValueError, match="extrinsic"):
        check(unitree_g1_sonic_navigation_peer, "peer", claimed)


@pytest.mark.parametrize("role", ["peer", "pc2"])
def test_build_rejects_before_configuration_sockets_or_workers(role: str, mocker) -> None:
    base = unitree_g1_sonic_navigation_peer if role == "peer" else unitree_g1_sonic_navigation_pc2
    configure = mocker.patch("dimos.core.coordination.module_coordinator._run_configurators")
    materialize = mocker.patch("dimos.core.coordination.module_coordinator._materialize_transports")
    start = mocker.patch.object(ModuleCoordinator, "start")
    deploy = mocker.patch("dimos.core.coordination.module_coordinator._deploy_all_modules")

    with pytest.raises(ValueError):
        ModuleCoordinator.build(autoconnect(base, FastLio2.blueprint()))

    for side_effect in (configure, materialize, start, deploy):
        side_effect.assert_not_called()
