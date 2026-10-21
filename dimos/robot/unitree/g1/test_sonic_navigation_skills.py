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


"""Agent skills on pc2: only inside an operator-begun request, bounded, relative goals."""

import math
from types import SimpleNamespace

import pytest

from dimos.robot.unitree.g1.sonic_navigation_skills import SonicNavigationSkills


class FakeExecution:
    def __init__(self, phase: str = "active") -> None:
        self.phase, self.goals, self.ended = phase, [], 0
        self.refuse: str | None = None

    def get_navigation_status(self) -> dict:
        return {"phase": self.phase, "request_id": "req-1", "fault": None, "pose_ready": False}

    def set_navigation_goal(self, request_id: str, goal) -> bool:
        if self.refuse:
            raise RuntimeError(self.refuse)
        self.goals.append((request_id, goal))
        return True

    def end_navigation(self) -> dict:
        self.ended += 1
        self.phase = "stopping"
        return {"phase": self.phase}


@pytest.fixture
def skills():
    module = SonicNavigationSkills()
    execution = FakeExecution()
    module._execution = execution
    module._peer = SimpleNamespace(
        get_navigation_peer_status=lambda: {"state": "following_path", "fault": None}
    )
    yield module, execution
    module.stop()


def test_walk_to_sends_a_base_relative_goal_in_the_active_request(skills) -> None:
    module, execution = skills
    reply = module.walk_to(1.5, 0.5)
    ((request_id, goal),) = execution.goals
    assert request_id == "req-1" and goal.frame_id == "base_link"
    assert (goal.position.x, goal.position.y) == (1.5, 0.5)
    assert goal.orientation.euler[2] == pytest.approx(math.atan2(0.5, 1.5))
    assert "walking" in reply.lower()


def test_walk_to_needs_an_operator_begun_request(skills) -> None:
    module, execution = skills
    execution.phase = "disarmed"
    assert "operator" in module.walk_to(1.0)
    assert execution.goals == []


@pytest.mark.parametrize(("forward", "left"), [(3.5, 0.0), (0.0, 2.5), (0.1, 0.1), (math.nan, 0.0)])
def test_walk_to_refuses_out_of_bounds_targets(skills, forward, left) -> None:
    module, execution = skills
    assert "refused" in module.walk_to(forward, left).lower()
    assert execution.goals == []


def test_walk_to_reports_an_execution_refusal(skills) -> None:
    module, execution = skills
    execution.refuse = "navigation goal rejected or admission closed (peer refused)"
    assert "peer refused" in module.walk_to(1.0)


def test_stop_walking_ends_the_request(skills) -> None:
    module, execution = skills
    assert "stopping" in module.stop_walking()
    assert execution.ended == 1


def test_navigation_status_includes_the_peer_planner(skills) -> None:
    module, _ = skills
    status = module.navigation_status()
    assert "active" in status and "following_path" in status
    module._peer = SimpleNamespace(get_navigation_peer_status=lambda: 1 / 0)
    assert "peer status unavailable" in module.navigation_status()
