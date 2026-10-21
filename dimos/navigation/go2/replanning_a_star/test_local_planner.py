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
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.go2.replanning_a_star.local_planner import LocalPlanner


def first_command(heading: float, **kwargs: float) -> Twist:
    """First motion command for a straight free path `heading` rad left of the robot."""
    free = OccupancyGrid(
        grid=np.zeros((100, 100), dtype=np.int8),
        origin=Pose(position=Vector3(-2.5, -2.5, 0.0)),
    )
    planner = LocalPlanner(
        SimpleNamespace(binary_costmap=free), goal_tolerance=0.2, robot_width=0.3, **kwargs
    )
    planner.handle_odom(PoseStamped(position=Vector3(0.0, 0.0, 0.0)))
    yaw = Quaternion.from_euler(Vector3(0.0, 0.0, heading))
    poses = [
        PoseStamped(
            position=Vector3(d * math.cos(heading), d * math.sin(heading), 0.0), orientation=yaw
        )
        for d in np.arange(0.05, 1.05, 0.05)  # a path starting on the robot is "arrived"
    ]
    commands: list[Twist] = []
    sent = threading.Event()
    moving = lambda cmd: cmd.linear.x != 0.0 or cmd.angular.z != 0.0  # noqa: E731
    # start_planning first publishes the zero stop of the reset.
    planner.cmd_vel.subscribe(lambda cmd: moving(cmd) and (commands.append(cmd), sent.set()))
    planner.start_planning(Path(poses=poses))
    assert sent.wait(2.0)
    planner.stop_planning()
    return commands[0]


def test_default_turns_in_place_beyond_twenty_degrees() -> None:
    cmd = first_command(0.6)
    assert cmd.linear.x == 0.0 and cmd.angular.z > 0.0


@pytest.mark.parametrize("heading", [0.6, -0.6])
def test_wider_initial_tolerance_walks_off_while_turning(heading: float) -> None:
    cmd = first_command(heading, initial_rotation_tolerance=math.pi / 4)
    assert cmd.linear.x > 0.0 and math.copysign(1.0, cmd.angular.z) == math.copysign(1.0, heading)


def test_heading_beyond_the_initial_tolerance_still_turns_in_place() -> None:
    cmd = first_command(1.0, initial_rotation_tolerance=math.pi / 4)
    assert cmd.linear.x == 0.0 and cmd.angular.z > 0.0
