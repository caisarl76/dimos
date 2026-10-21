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

import math

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.go2.replanning_a_star.controllers import PController, PdController
from dimos.navigation.go2.replanning_a_star.module import (
    ReplanningAStarPlanner,
    ReplanningAStarPlannerConfig,
)


@pytest.fixture
def make_planner():
    modules = []

    def create(**kwargs):
        module = ReplanningAStarPlanner(**kwargs)
        modules.append(module)
        return module

    try:
        yield create
    finally:
        for module in modules:
            module.stop()


@pytest.mark.parametrize("controller_type", [PController, PdController])
def test_min_angular_velocity_is_instance_scoped(controller_type):
    default = controller_type(False, 0.2, 10.0)
    smooth = controller_type(False, 0.2, 10.0, min_angular_velocity=0.0)
    # A PD sample seeded at this error isolates its proportional response.
    default.reset_yaw_error(0.01)
    smooth.reset_yaw_error(0.01)
    assert default.rotate(0.01).angular.z == pytest.approx(0.2)
    assert smooth.rotate(0.01).angular.z == pytest.approx(0.005)
    default.reset_errors()
    smooth.reset_errors()
    default.reset_yaw_error(-0.01)
    smooth.reset_yaw_error(-0.01)
    assert smooth.rotate(-0.01).angular.z == pytest.approx(-0.005)
    assert default.rotate(-0.01).angular.z == pytest.approx(-0.2)


@pytest.mark.parametrize("value", [-0.1, math.nan, math.inf, -math.inf])
def test_invalid_angular_minimum_rejected(value):
    with pytest.raises(ValidationError):
        ReplanningAStarPlannerConfig(min_angular_velocity=value)
    with pytest.raises(ValueError):
        PController(False, 0.2, 10.0, min_angular_velocity=value)


def test_sonic_follower_envelope(make_planner):
    module = make_planner(
        g=GlobalConfig(simulation=""),
        nerf_speed=0.2 / 0.55,
        min_angular_velocity=0.0,
    )
    controller = module._planner._local_planner._controller
    assert controller.rotate(0.01).angular.z == pytest.approx(0.005)
    assert controller.rotate(10).angular.z == pytest.approx(0.2)
    assert controller.rotate(-10).angular.z == pytest.approx(-0.2)
    pose = PoseStamped()
    for angle in [0.0, -0.01, 0.01, 0.8, -0.8, 2.0]:
        command = controller.advance(np.array([math.cos(angle), math.sin(angle)]), pose)
        assert 0.0 <= command.linear.x <= 0.2 + 1e-12
        assert abs(command.angular.z) <= 0.2 + 1e-12
    assert controller.rotate(0.0).angular.z == 0.0


def test_default_module_retains_turn_floor(make_planner):
    module = make_planner(g=GlobalConfig(simulation=""))
    assert module._planner._local_planner._controller.rotate(0.01).angular.z == 0.2
