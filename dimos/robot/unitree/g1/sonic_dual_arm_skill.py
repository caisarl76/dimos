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

"""Left-arm skills for two G1s on the guarded SONIC controller, from one process.

The same skills as SonicArmSkillContainer, each taking `robot` ("222", "223" or "both"). One robot
is a single-arm move; "both" starts both arms together and retreats both if either stops (run_arms).
One motion runs at a time across both robots.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from functools import partial
from typing import Any, Literal

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.robot.unitree.g1.sonic_arm_reference import ArmCollisionChecker, SonicEncoder
from dimos.robot.unitree.g1.sonic_arm_skill import (
    _NO_PATH_CHECK,
    ArmLauncher,
    ArmMove,
    GestureName,
    LeftArmPose,
    SonicArm,
    SonicArmExecutionSpec,
    SonicArmSkillConfig,
    UpperBodyMotion,
    gesture_move,
    left_arm_joints_move,
    left_arm_pose_move,
    run_arms,
    upper_body_move,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback

Robot = Literal["222", "223", "both"]
_ROBOTS = ("222", "223")


class DualSonicArmSkillContainer(ArmLauncher, Module):
    """Arm skills for robots 222 and 223 over their own SONIC executions."""

    config: SonicArmSkillConfig
    feedback_222: In[SonicFeedback]
    feedback_223: In[SonicFeedback]
    _execution_222: SonicArmExecutionSpec
    _execution_223: SonicArmExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._by_robot: dict[str, SonicArm] = {}
        super().__init__(**kwargs)

    def _arms(self) -> Sequence[SonicArm]:
        return list(self._by_robot.values())

    @rpc
    def start(self) -> None:
        encoder = SonicEncoder(self.config.encoder_path)
        checker = None
        if self.config.collision_model_path is not None:
            checker = ArmCollisionChecker(self.config.collision_model_path)
        executions = {"222": self._execution_222, "223": self._execution_223}
        self._by_robot = {
            name: SonicArm(name, self.config, execution, encoder, checker)
            for name, execution in executions.items()
        }
        super().start()

    @rpc
    def stop(self) -> None:
        self._retreat_before_stop()
        super().stop()

    @rpc
    async def handle_feedback_222(self, feedback: SonicFeedback) -> None:
        """Track robot 222's active move from every telemetry sample."""
        if "222" in self._by_robot:
            self._by_robot["222"].handle_feedback(feedback)

    @rpc
    async def handle_feedback_223(self, feedback: SonicFeedback) -> None:
        """Track robot 223's active move from every telemetry sample."""
        if "223" in self._by_robot:
            self._by_robot["223"].handle_feedback(feedback)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_upper_body(self, robot: Robot, motion: UpperBodyMotion, angle_deg: float) -> str:
        """Move one upper-body joint of robot 222, 223 or both out by angle_deg, hold it for 1 second, move it back, then hand control back.

        Motions: "left_arm_forward" swings the left arm forward (shoulder pitch),
        "left_arm_sideways" raises the left arm out to the side (shoulder roll), and
        "left_forearm_raise" lifts the left forearm (elbow). Min-jerk, 2 s out and 2 s back (longer
        for big angles), while SONIC keeps the robot balanced; 18-24 s in all. "both" moves the two
        robots together and stops both if either stops. Requires the SONIC controller running in
        confirmed planner idle; it never starts or stops the controller. Stops early and returns
        control to the planner if any joint strays from the plan. Returns at once; a tool update
        reports the result.

        Args:
            robot: "222", "223" or "both".
            motion: One of "left_arm_forward", "left_arm_sideways", "left_forearm_raise".
            angle_deg: How far to move, in degrees: left_arm_forward 10-28, left_arm_sideways 10-40,
                left_forearm_raise 20-60.
        """
        return self._launch_move(
            "move_upper_body", robot, lambda: upper_body_move(self.config, motion, angle_deg)
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_left_arm_pose(self, robot: Robot, pose: LeftArmPose, hold_seconds: float = 1.0) -> str:
        """Move the left arm of robot 222, 223 or both to a full-range pose, hold it for hold_seconds, bring it back, then hand control back.

        Poses: "left_arm_forward_horizontal" (straight arm pointing forward at shoulder height),
        "left_arm_side_horizontal" (straight arm out to the side at shoulder height),
        "left_hand_over_head" (arm raised forward and up until the hand is over the head),
        "left_hand_over_head_sideways" (arm raised out to the side and up over the head). Slow
        (peak 0.47 rad/s): 20-40 s in all, overhead the longest. Requires the SONIC controller
        running in confirmed planner idle; it never starts or stops the controller. Stops early and
        returns control to the planner if the legs, waist or the other arm stray, or the arm
        strays far from the plan. Returns at once; a tool update reports the result.

        Args:
            robot: "222", "223" or "both".
            pose: One of "left_arm_forward_horizontal", "left_arm_side_horizontal",
                "left_hand_over_head", "left_hand_over_head_sideways".
            hold_seconds: How long to hold the pose, 1-20 s.
        """
        return self._launch_move(
            "move_left_arm_pose",
            robot,
            lambda: left_arm_pose_move(self.config, pose, hold_seconds),
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_left_arm_joints(
        self,
        robot: Robot,
        shoulder_pitch_deg: float,
        shoulder_roll_deg: float,
        shoulder_yaw_deg: float,
        elbow_deg: float,
        hold_seconds: float = 1.0,
    ) -> str:
        """Move the left arm of robot 222, 223 or both to the given joint angles, hold for hold_seconds, bring it back, then hand control back.

        Angles are absolute, in degrees. Hanging rest is about pitch 11, roll 11, yaw 0, elbow 34.
        Shoulder pitch: negative raises the arm forward (-90 forward horizontal, -160 overhead).
        Shoulder roll: positive raises it out to the side (86 side horizontal). Shoulder yaw rotates
        the upper arm. Elbow: 82 is straight, 0 is bent 90 degrees, negative bends further. The
        planned path is checked against the robot's body first and refused if the arm would come
        near it. Slow (peak 0.47 rad/s), up to about 40 s. Prefer move_left_arm_pose for the
        named poses. Returns at once; a tool update reports the result.

        Args:
            robot: "222", "223" or "both".
            shoulder_pitch_deg: -165 to 45.
            shoulder_roll_deg: -10 to 120.
            shoulder_yaw_deg: -90 to 90.
            elbow_deg: -30 to 82.
            hold_seconds: How long to hold the pose, 1-20 s.
        """
        values = (shoulder_pitch_deg, shoulder_roll_deg, shoulder_yaw_deg, elbow_deg)

        def make_move() -> ArmMove:
            if any(arm._checker is None for arm in self._select(robot)):
                raise RuntimeError(_NO_PATH_CHECK)
            return left_arm_joints_move(self.config, values, hold_seconds)

        return self._launch_move("move_left_arm_joints", robot, make_move)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def perform_gesture(self, robot: Robot, name: GestureName) -> str:
        """Perform a left-arm gesture on robot 222, 223 or both, then bring the arm back and hand control back.

        Gestures: "wave" (raises the hand and swings it side to side three times, about 30 s),
        "point_forward" (arm out in front, about 16 s), "stop_sign" (raised forward palm out, about 15 s),
        "raise_hand" (arm up, about 25 s) and "beckon" (forearm curls in and out twice, about 21 s).
        Slow (peak 0.47 rad/s). Requires the SONIC controller running in confirmed planner idle; it
        never starts or stops the controller. Stops early and returns control to the planner if the
        legs, waist or the other arm stray, or the arm strays far from the plan. Returns at once; a
        tool update reports the result.

        Args:
            robot: "222", "223" or "both".
            name: One of "wave", "point_forward", "stop_sign", "raise_hand", "beckon".
        """
        return self._launch_move("perform_gesture", robot, lambda: gesture_move(self.config, name))

    @skill
    def stop_arm(self) -> str:
        """Stop the running arm motion on every robot: each arm returns smoothly to where it started, then control goes back to SONIC's planner.

        The stopped motion's tool update reports how far each robot got.
        """
        return self._request_stop()

    def _select(self, robot: str) -> list[SonicArm]:
        """The arms a command targets; an unknown robot selects all, and validation refuses it."""
        if str(robot) in self._by_robot:
            return [self._by_robot[str(robot)]]
        return list(self._arms())

    def _launch_move(self, tool: str, robot: str, make_move: Callable[[], ArmMove]) -> str:
        robot = str(robot)  # `dimos mcp call --arg robot=223` sends the integer 223.
        selected = self._select(robot)

        def prepare() -> Callable[[], SkillResult]:
            if robot not in (*_ROBOTS, "both"):
                raise ValueError(f"robot must be one of {[*_ROBOTS, 'both']}")
            move = make_move()
            jobs = [
                (
                    arm,
                    partial(
                        arm.prepare,
                        move.label,
                        move.keyframes_of,
                        move.allowed,
                        require_check=move.require_check,
                    ),
                )
                for arm in selected
            ]
            return lambda: run_arms(jobs)

        return self._launch(tool, prepare, selected)


dual_sonic_arm_skill_container = DualSonicArmSkillContainer.blueprint
