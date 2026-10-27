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

"""Arm skills for two G1s on the guarded SONIC controller, from one process.

The same skills as SonicArmSkillContainer, each taking `robot` ("222", "223" or "both"). One robot
is a single-arm move; "both" starts both arms together and retreats both if either stops (run_arms).
One motion runs at a time across both robots. Each robot plans from its own measured body and Dex3
hands; whether its hand readings mean anything is per robot (hands_mounted_222/_223), and so are
where its voice lines play (voice_host_222/_223) and its wide-leg gestures' leg margin
(wide_leg_margin_222/_223).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from functools import partial
import sys
from typing import Any, Literal

from pydantic import Field, model_validator

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.robot.unitree.g1.sonic_arm_reference import ArmCollisionChecker, SonicEncoder
from dimos.robot.unitree.g1.sonic_arm_skill import (
    _NO_PATH_CHECK,
    FINGER_CHECK,
    LEG_MARGIN_CEILING,
    ArmLauncher,
    ArmMove,
    GestureName,
    LeftArmPose,
    SonicArm,
    SonicArmExecutionSpec,
    SonicArmSkillConfig,
    UpperBodyMotion,
    check_line,
    finger_check_move,
    gesture_move,
    left_arm_joints_move,
    left_arm_pose_move,
    run_arms,
    upper_body_move,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

Robot = Literal["222", "223", "both"]
_ROBOTS = ("222", "223")


class DualSonicArmSkillConfig(SonicArmSkillConfig):
    # Dex3 hands per robot (222 has them, 223 runs without): only these say a robot's hand
    # readings mean anything. The single-robot hands_mounted would claim it for both, so it is
    # refused here.
    hands_mounted_222: bool = False
    hands_mounted_223: bool = False
    # Voice lines per robot (perform_gesture's line): the ssh alias of each robot's PC; None keeps
    # that robot silent. voice_command, voice_dir, voice_volume and voice_lead_s are shared (each
    # robot times its line from its own plan). The single-robot voice_host would speak on both,
    # so it is refused here.
    voice_host_222: str | None = None
    voice_host_223: str | None = None
    # The wide-leg gestures' leg margin per robot (wide_leg_margin); the blueprint gives 222 the
    # ceiling. The single-robot wide_leg_margin would widen both, so it is refused here.
    wide_leg_margin_222: float | None = Field(default=None, gt=0, le=LEG_MARGIN_CEILING)
    wide_leg_margin_223: float | None = Field(default=None, gt=0, le=LEG_MARGIN_CEILING)

    @model_validator(mode="after")
    def _per_robot(self) -> Self:
        if self.hands_mounted:
            raise ValueError(
                "the dual container sets hands per robot: use hands_mounted_222 / hands_mounted_223"
            )
        if self.voice_host is not None:
            raise ValueError(
                "the dual container sets the voice host per robot: use voice_host_222 / voice_host_223"
            )
        if self.wide_leg_margin is not None:
            raise ValueError(
                "the dual container sets the wide leg margin per robot: use wide_leg_margin_222 / "
                "wide_leg_margin_223"
            )
        return self


class DualSonicArmSkillContainer(ArmLauncher, Module):
    """Arm skills for robots 222 and 223 over their own SONIC executions."""

    config: DualSonicArmSkillConfig
    feedback_222: In[SonicFeedback]
    feedback_223: In[SonicFeedback]
    _execution_222: SonicArmExecutionSpec
    _execution_223: SonicArmExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._by_robot: dict[str, SonicArm] = {}
        # Each robot's last operator motion's final report; None while one runs (or before any).
        self._operator_results: dict[str, str | None] = dict.fromkeys(_ROBOTS)
        super().__init__(**kwargs)

    def _arms(self) -> Sequence[SonicArm]:
        return list(self._by_robot.values())

    @rpc
    def start(self) -> None:
        encoder = SonicEncoder(self.config.encoder_path)
        checker = both_checker = None
        if self.config.collision_model_path is not None:
            # TODO(dex3): the checkers ignore the hand geoms (built for 223, no hands). 222's Dex3
            # hand is much bigger, so a path that clears on 223 can hit 222's body. Not critical
            # while trajectory speed is managed; add the Dex3 hand to the 222 check.
            checker = ArmCollisionChecker(self.config.collision_model_path)
            both_checker = ArmCollisionChecker(
                self.config.collision_model_path, arms=("left", "right")
            )
        executions = {"222": self._execution_222, "223": self._execution_223}
        hands = {"222": self.config.hands_mounted_222, "223": self.config.hands_mounted_223}
        voices = {"222": self.config.voice_host_222, "223": self.config.voice_host_223}
        legs = {"222": self.config.wide_leg_margin_222, "223": self.config.wide_leg_margin_223}
        self._by_robot = {
            name: SonicArm(
                name,
                self.config.model_copy(
                    update={
                        "hands_mounted": hands[name],
                        "voice_host": voices[name],
                        "wide_leg_margin": legs[name],
                    }
                ),
                execution,
                encoder,
                checker,
                both_checker,
            )
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
        for big angles), while SONIC keeps the robot balanced; 11-13 s of motion. "both" moves the two
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
            "move_upper_body", robot, lambda config: upper_body_move(config, motion, angle_deg)
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_left_arm_pose(self, robot: Robot, pose: LeftArmPose, hold_seconds: float = 1.0) -> str:
        """Move the left arm of robot 222, 223 or both to a full-range pose, hold it for hold_seconds, bring it back, then hand control back.

        Poses: "left_arm_forward_horizontal" (straight arm pointing forward at shoulder height),
        "left_arm_side_horizontal" (straight arm out to the side at shoulder height),
        "left_hand_over_head" (arm raised forward and up until the hand is over the head),
        "left_hand_over_head_sideways" (arm raised out to the side and up over the head). Slow
        (peak 0.75 rad/s): 13-21 s of motion, overhead the longest. Requires the SONIC controller
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
            lambda config: left_arm_pose_move(config, pose, hold_seconds),
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
        near it. Slow (peak 0.75 rad/s), up to about 25 s. Prefer move_left_arm_pose for the
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

        def make_move(config: SonicArmSkillConfig) -> ArmMove:
            if any(arm._checker is None for arm in self._select(robot)):
                raise RuntimeError(_NO_PATH_CHECK)
            return left_arm_joints_move(config, values, hold_seconds)

        return self._launch_move("move_left_arm_joints", robot, make_move)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def perform_gesture(self, robot: Robot, name: GestureName, line: str | None = None) -> str:
        """Perform a gesture on robot 222, 223 or both, then bring the arms back and hand control back.

        Left arm only: "wave" (raises the hand and swings it side to side three times, about 26 s),
        "point_forward" (arm out in front, about 15 s), "stop_sign" (raised forward palm out, about 14 s),
        "raise_hand" (arm up, about 21 s) and "beckon" (forearm curls in and out twice, about 20 s).
        Imported, either or both arms, with the Dex3 hands where noted: "wave_one_hand" (right
        arm waves, about 16 s), "v_sign" (right arm, V with the fingers, about 18 s), "good" (right
        arm, thumbs up, about 19-20 s), "support_person" (both arms forward, about 17 s) and
        "two_hand_hearts" (both arms and hands form a heart, about 31 s).
        Demo, arms and waist: "invite" (both hands up, palms forward, swaying once, then a waist
        turn left and right, about 17 s),
        "take_left"/"take_right" (that arm points down and out, palm forward, about 15 s),
        "entrance_left"/"entrance_right" (that arm points level out to the side, palm forward,
        about 15 s), "thanks" (hands held together at the chest with a short bow, about 17 s) and
        "bow" (arms hanging in front, a bow forward from the waist, about 15 s). Each robot
        plans from its own measured pose and hands, so with "both" an imported gesture's timing may
        differ a little between the robots. Slow (peak 0.75 rad/s, fingers too). Requires the SONIC controller
        running in confirmed planner idle; it never starts or stops the controller. Stops early and
        returns control to the planner if the legs or waist stray, or an arm strays far from the
        plan. Returns at once; a tool update reports the result.

        Args:
            robot: "222", "223" or "both".
            name: One of the gesture names above.
            line: Optional voice line (wav name, e.g. "01_intro") spoken as the motion starts, by
                each selected robot that has a voice.
        """
        return self._launch_move(
            "perform_gesture", robot, lambda config: gesture_move(config, name), line
        )

    @skill
    def stop_arm(self) -> str:
        """Stop the running arm motion on every robot: each arm returns smoothly to where it started, then control goes back to SONIC's planner.

        The stopped motion's tool update reports how far each robot got.
        """
        return self._request_stop()

    @rpc
    def check_dex3_finger_order(self, robot: str) -> str:
        """Operator only (not a skill): on robot 222 or 223, a background motion that holds the
        measured body and moves the Dex3 hands to an asymmetric pose for 3 s, then back, through
        that robot's guarded, hand-speed-limited session. Returns "started" at once; stop_arm
        stops it, and operator_result(robot) reports it. The operator confirms FINGER_CHECK_POSE
        by eye. Refused, with nothing sent, unless hands_mounted_<robot> is set.
        """
        robot = str(robot)
        if robot not in _ROBOTS:
            raise ValueError(f"robot must be one of {list(_ROBOTS)}")
        arm = self._by_robot[robot]
        move = finger_check_move(arm.config)
        with self._lifecycle:
            self._admit()
            if not arm.config.hands_mounted:
                raise RuntimeError(
                    f"no hands configured on {robot} (hands_mounted_{robot} is False); nothing sent"
                )
            self._operator_results[robot] = None

            def store(text: str) -> None:
                self._operator_results[robot] = text

            self._begin(
                FINGER_CHECK,
                lambda: run_arms(self._jobs([(arm, move)])),
                [arm],
                store,
                lambda: None,
            )
        return "started"

    @rpc
    def operator_result(self, robot: str) -> str | None:
        """Robot 222's or 223's last operator motion's final report, or None while it runs (or
        before any)."""
        robot = str(robot)
        if robot not in _ROBOTS:
            raise ValueError(f"robot must be one of {list(_ROBOTS)}")
        with self._lifecycle:
            return self._operator_results[robot]

    @staticmethod
    def _jobs(
        moves: Sequence[tuple[SonicArm, ArmMove]], line: str | None = None
    ) -> list[tuple[SonicArm, Callable[[], Any]]]:
        """run_arms jobs: each arm prepares its move from its own measured pose and hands; those
        with a voice speak `line`."""
        return [
            (
                arm,
                partial(
                    arm.prepare,
                    move.label,
                    move.keyframes_of,
                    move.allowed,
                    require_check=move.require_check,
                    move_s=move.move_s,
                    return_min_s=move.return_min_s,
                    arms=move.arms,
                    line=line if arm.speaks else None,
                ),
            )
            for arm, move in moves
        ]

    def _select(self, robot: str) -> list[SonicArm]:
        """The arms a command targets; an unknown robot selects all, and validation refuses it."""
        if str(robot) in self._by_robot:
            return [self._by_robot[str(robot)]]
        return list(self._arms())

    def _launch_move(
        self,
        tool: str,
        robot: str,
        make_move: Callable[[SonicArmSkillConfig], ArmMove],
        line: str | None = None,
    ) -> str:
        robot = str(robot)  # `dimos mcp call --arg robot=223` sends the integer 223.
        selected = self._select(robot)

        def prepare() -> Callable[[], SkillResult]:
            if robot not in (*_ROBOTS, "both"):
                raise ValueError(f"robot must be one of {[*_ROBOTS, 'both']}")
            # Each robot's move from its own config (its wide_leg_margin).
            moves = [(arm, make_move(arm.config)) for arm in selected]
            if line is not None and not any(arm.speaks for arm in selected):
                where, hosts = (
                    ("on either robot", "voice_host_222 / voice_host_223")
                    if robot == "both"
                    else (f"on {robot}", f"voice_host_{robot}")
                )
                raise ValueError(
                    f"voice is not configured {where} ({hosts}, voice_command); nothing sent"
                )
            check_line(line)
            for arm, move in moves:
                arm.require_checker(move)
            jobs = self._jobs(moves, line)
            return lambda: run_arms(jobs)

        return self._launch(tool, prepare, selected)


dual_sonic_arm_skill_container = DualSonicArmSkillContainer.blueprint
