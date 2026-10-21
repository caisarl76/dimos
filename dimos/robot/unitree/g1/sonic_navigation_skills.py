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


"""Agent skills for SONIC navigation, beside the pc2 execution owner.

The skills only act inside a navigation request the operator began: they never
call begin_navigation (operator decision, 2026-10-21). pc2 has no position, so a
goal is sent relative to the robot (base_link) and the workstation peer turns it
into odom. An LLM agent on the workstation reaches them through McpServer.
"""

from __future__ import annotations

import math
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.spec.utils import Spec

# Bounds for one agent goal: about what an operator watches at once.
MAX_FORWARD_M = 3.0
MAX_LEFT_M = 2.0
MIN_DISTANCE_M = 0.3


class SonicNavigationExecutionSpec(Spec, Protocol):
    def get_navigation_status(self) -> dict[str, Any]: ...
    def set_navigation_goal(self, request_id: str, goal: PoseStamped) -> bool: ...
    def end_navigation(self) -> dict[str, Any]: ...


class SonicNavigationPeerStatusSpec(Spec, Protocol):
    def get_navigation_peer_status(self) -> dict[str, Any]: ...


class SonicNavigationSkills(Module):
    _execution: SonicNavigationExecutionSpec
    _peer: SonicNavigationPeerStatusSpec

    @skill
    def walk_to(self, forward_m: float, left_m: float = 0.0) -> str:
        """Walk to a point relative to where the robot stands now, avoiding obstacles.

        Works only while the operator has begun a navigation session. Returns at once;
        call navigation_status to follow progress.

        Args:
            forward_m: Metres ahead of the robot, 0 to 3.
            left_m: Metres to the robot's left (negative is right), -2 to 2.
        """
        if not (
            math.isfinite(forward_m)
            and math.isfinite(left_m)
            and 0.0 <= forward_m <= MAX_FORWARD_M
            and abs(left_m) <= MAX_LEFT_M
            and math.hypot(forward_m, left_m) >= MIN_DISTANCE_M
        ):
            return (
                f"Refused: the target must be 0-{MAX_FORWARD_M:g} m ahead, within "
                f"{MAX_LEFT_M:g} m to either side and at least {MIN_DISTANCE_M:g} m away."
            )
        status = self._execution.get_navigation_status()
        if status.get("phase") != "active" or not status.get("request_id"):
            return (
                f"Not walking: no active navigation session (phase {status.get('phase')}). "
                "The operator must begin one first."
            )
        goal = PoseStamped(
            frame_id="base_link",
            position=Vector3(forward_m, left_m, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.atan2(left_m, forward_m))),
        )
        try:
            accepted = self._execution.set_navigation_goal(status["request_id"], goal)
        except Exception as exc:
            return f"Goal refused: {exc}"
        if accepted is not True:
            return "Goal refused by the navigation stack."
        return f"Walking to {forward_m:.2f} m ahead, {left_m:.2f} m left."

    @skill
    def stop_walking(self) -> str:
        """Stop walking now and end the navigation session (the operator begins a new one)."""
        status = self._execution.end_navigation()
        return f"Stopping: navigation phase {status.get('phase')}."

    @skill
    def navigation_status(self) -> str:
        """Report the navigation session and the planner's progress toward the goal."""
        status = self._execution.get_navigation_status()
        try:
            peer = self._peer.get_navigation_peer_status()
            planner = f"planner state {peer.get('state')}, fault {peer.get('fault')}"
        except Exception as exc:
            planner = f"peer status unavailable ({type(exc).__name__})"
        return f"Navigation phase {status.get('phase')}, fault {status.get('fault')}; {planner}."


sonic_navigation_skills = SonicNavigationSkills.blueprint
