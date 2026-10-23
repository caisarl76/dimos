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

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner


def test_find_wide_path_with_start_inside_inflation() -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = OccupancyGrid(grid=grid, resolution=resolution, origin=Pose(), frame_id="world")

    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Vector3(1.43, 1.5, 0)
    # On the other side of the wall; the path must round a wall end.
    goal = Vector3(2.75, 1.5, 0)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0


def _walled(blocked: bool) -> OccupancyGrid:
    grid = np.zeros((60, 60), dtype=np.int8)
    if blocked:
        grid[:, 30] = 100  # a wall at x=1.5 m across the whole map
    return OccupancyGrid(grid=grid, resolution=0.05, origin=Pose(), frame_id="world")


def _planner_with_goal(blocked: bool) -> tuple[GlobalPlanner, list[bool], list[int]]:
    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    reached: list[bool] = []
    paths: list[int] = []
    planner.goal_reached.subscribe(lambda b: reached.append(b.data))
    planner.path.subscribe(lambda p: paths.append(len(p.poses)))
    planner.handle_global_costmap(_walled(blocked))
    planner.handle_odom(PoseStamped(position=Vector3(0.5, 1.5, 0), frame_id="world"))
    planner.handle_goal_request(PoseStamped(position=Vector3(2.5, 1.5, 0), frame_id="world"))
    return planner, reached, paths


def test_a_goal_without_a_path_is_kept_and_planned_once_the_way_clears() -> None:
    # Session 2026-10-22-5: someone in the way when the goal was sent made the
    # planner give up at once, and the robot never moved.
    planner, reached, paths = _planner_with_goal(blocked=True)
    assert planner._current_goal is not None and reached == []
    assert not any(paths), "no path published while blocked"

    planner.handle_global_costmap(_walled(blocked=False))
    planner._plan_path()

    assert paths[-1] > 0 and reached == []
    assert planner._no_path_since is None


def test_a_goal_without_a_path_is_given_up_after_the_retry_window() -> None:
    planner, reached, _ = _planner_with_goal(blocked=True)
    assert planner._no_path_since is not None
    planner._no_path_since -= planner._no_path_retry_window

    planner._plan_path()

    assert reached == [False]
    assert planner._current_goal is None
