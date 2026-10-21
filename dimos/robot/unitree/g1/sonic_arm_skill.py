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

"""Left-arm-forward skill on the guarded SONIC controller, through SonicExecutionModule.

Encodes a min-jerk left-shoulder-pitch reference from the measured pose (sonic_arm_reference),
streams it as a sonic78 source and hands control back to the native planner. The whole 11 s plan
is one open-loop chunk planned from the start pose, exactly as commissioned on a G1, so the
execution profile's max_observation_age must cover it. Resubmitting it like a policy source was
rejected: every chunk restarts the session blend, a 2.5 Hz speed ripple. observed_at uses this
host's monotonic clock: run on the same host as the execution module. Never starts or stops native
control. Cancels to the planner if a joint leaves its per-group envelope or exceeds `max_dq`.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from pathlib import Path
import threading
import time
from typing import Any, Literal, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.control.components import make_humanoid_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.robot.unitree.g1.sonic_arm_reference import (
    ARM_JOINTS,
    LEFT_SHOULDER_PITCH,
    LOOSE_JOINTS,
    RATE,
    ArmCollisionChecker,
    SonicEncoder,
    group_margins,
    pose_margins,
    stroke_plan,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.spec.utils import Spec

ArmError = Literal[
    "INVALID_ANGLE",
    "BUSY",
    "NOT_READY",
    "MOVED",
    "ENVELOPE",
    "EXECUTION_FAILED",
    "HANDBACK_UNCONFIRMED",
    "NOT_REACHED",
    "INVALID_MOTION",
    "UNSAFE_PATH",
]
UpperBodyMotion = Literal["left_arm_forward", "left_arm_sideways", "left_forearm_raise"]
# Motion -> (motor-order joint, sign of a positive angle, min deg, max deg). Directions from the G1
# model in the torso frame. Ranges from a MuJoCo floor sweep at 10-60 deg with the stroke slowed to
# PEAK_SPEED, and robot runs at 10/20/28 deg:
# - forward: from 40 deg the legs passed their envelope and a foot partly unloaded.
# - sideways: at 50-60 deg other arm joints and wrists reached 85-90% of their envelopes.
# - forearm: tracked 95% to 60 deg with little side motion, but its joint friction against the soft
#   arm gains costs 5-7 deg (robot: 48% at 10 deg, 65% at 20 deg).
# Shoulder roll only outward: inward drives the hand into the torso. Waist turns were left out:
# SONIC tracked only 18-48% of an isolated waist yaw.
MOTIONS: dict[str, tuple[int, int, float, float]] = {
    "left_arm_forward": (LEFT_SHOULDER_PITCH, -1, 10.0, 28.0),  # hand forward and up
    "left_arm_sideways": (16, +1, 10.0, 40.0),  # left shoulder roll: hand outward
    "left_forearm_raise": (18, -1, 20.0, 60.0),  # left elbow: forearm up
}
LeftArmPose = Literal[
    "left_arm_forward_horizontal",
    "left_arm_side_horizontal",
    "left_hand_over_head",
    "left_hand_over_head_sideways",
]
# Full-range left-arm poses: absolute shoulder pitch, roll, yaw and elbow (motor 15-18), rad. Elbow
# +1.43 rad is straight. MuJoCo on the floor at PEAK_SPEED: all four tracked within 0.13 rad, kept 8/8
# floor contacts (7 for forward) and never touched the body. The overhead pose needs roll 0.5:
# at 0.2 the hand crosses over the head.
POSES: dict[str, tuple[float, float, float, float]] = {
    "left_arm_forward_horizontal": (-1.50, 0.20, 0.00, 1.20),
    "left_arm_side_horizontal": (0.20, 1.50, 0.00, 1.20),
    "left_hand_over_head": (-2.80, 0.50, 0.00, 1.20),
    "left_hand_over_head_sideways": (0.00, 2.10, 0.00, 1.20),
}
# move_left_arm_joints bounds, deg, inside the joint limits. Collisions inside the box are left to the
# runtime path check (ArmCollisionChecker), which the joint-target skill requires.
JOINT_BOUNDS = {
    "shoulder_pitch": (-165.0, 45.0),
    "shoulder_roll": (-10.0, 120.0),
    "shoulder_yaw": (-90.0, 90.0),
    "elbow": (-30.0, 82.0),
}
_LEFT_ARM_TARGET = [15, 16, 17, 18]
_MOVED_LIMIT = 0.05  # rad: body motion allowed while encoding.
_MAX_CHUNK_ROWS = 1000  # SonicTaskSession accepts chunks of at most 1000 rows (20 s).
_CHUNK_OVERLAP = 25  # rows the first chunk runs past the split, so it never runs dry


def chunk_rows(plan_rows: int, hold_start: int) -> list[tuple[int, int]]:
    """(start, end) rows of each chunk to submit. One chunk when it fits the session's limit, as
    commissioned; otherwise two, split at the start of the hold at full depth. The reference is still
    there, so the session's blend restart at the second chunk cannot ripple the speed."""
    if plan_rows <= _MAX_CHUNK_ROWS:
        return [(0, plan_rows)]
    chunks = [(0, hold_start + _CHUNK_OVERLAP), (hold_start, plan_rows)]
    if any(end - start > _MAX_CHUNK_ROWS for start, end in chunks):
        raise ValueError("plan too long for two session chunks")
    return chunks


_JOINTS = [name.removeprefix("g1/") for name in make_humanoid_joints("g1")]
_GROUPS = {  # Reporting groups, matching the envelope groups.
    "legs": list(range(12)),
    "waist": [12, 13, 14],
    "arms": ARM_JOINTS,
    "wrists_shoulder_yaw": LOOSE_JOINTS,
}


def _peak(peak: tuple[float, int]) -> dict[str, Any]:
    return {"rad_s": round(peak[0], 3), "joint": _JOINTS[peak[1]] if peak[0] > 0 else None}


def _fail(code: ArmError, message: str, report: dict[str, Any]) -> SkillResult[ArmError]:
    return SkillResult[ArmError](False, message, code, metadata=report)


class SonicArmExecutionSpec(Spec, Protocol):
    def get_status(self) -> dict[str, Any]: ...
    def get_feedback(self) -> SonicFeedback | None: ...
    def start_task(self) -> dict[str, Any]: ...
    def submit_chunk(
        self, task_id: str, sequence: int, observed_at: float, actions: NDArray[Any]
    ) -> dict[str, Any]: ...
    def finish_task(self, task_id: str) -> dict[str, Any]: ...
    def cancel_task(self, task_id: str) -> dict[str, Any]: ...


class SonicArmSkillConfig(ModuleConfig):
    encoder_path: Path
    body_margin: float = Field(default=0.15, gt=0, le=0.3)  # legs and waist, rad
    arm_margin: float = Field(default=0.20, gt=0, le=0.4)  # shoulder pitch/roll and elbows, rad
    wrist_margin: float = Field(default=0.30, gt=0, le=0.5)  # wrists and shoulder yaws, rad
    max_dq: float = Field(default=3.0, gt=0, le=5.0)  # rad/s, any joint
    return_watch_seconds: float = Field(default=3.0, ge=0, le=5.0)
    # Full-range poses and joint targets: every left-arm joint may stray this far from the plan
    # (SONIC coordinates the whole arm), every right-arm joint this far from its start, rad.
    pose_arm_margin: float = Field(default=0.45, gt=0, le=0.6)
    other_arm_margin: float = Field(default=0.35, gt=0, le=0.5)
    # gear_sonic scene_43dof.xml for the runtime path check; move_left_arm_joints needs it.
    collision_model_path: Path | None = None


@dataclass
class _Watch:
    base: NDArray[np.float64]
    allowed: NDArray[np.float64]
    max_dq: float
    delta: NDArray[np.float64]  # The planned displacement of every joint at full depth.
    returning: bool = False
    reached: float = 0.0  # Furthest progress along the planned displacement, as a fraction.
    reached_q: NDArray[np.float64] = field(default_factory=lambda: np.zeros(29))
    deviation: NDArray[np.float64] = field(default_factory=lambda: np.zeros(29))
    stream_dq: tuple[float, int] = (0.0, -1)
    return_dq: tuple[float, int] = (0.0, -1)
    abort: str | None = None


class SonicArmSkillContainer(Module):
    """Arm-reference skills over the guarded SONIC controller (no hands required)."""

    config: SonicArmSkillConfig
    feedback: In[SonicFeedback]
    _execution: SonicArmExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._watch: _Watch | None = None
        self._encoder: SonicEncoder | None = None
        self._checker: ArmCollisionChecker | None = None
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        self._encoder = SonicEncoder(self.config.encoder_path)
        if self.config.collision_model_path is not None:
            self._checker = ArmCollisionChecker(self.config.collision_model_path)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    async def handle_feedback(self, feedback: SonicFeedback) -> None:
        """Track the active move's envelope, reach and speeds from every telemetry sample."""
        with self._lock:
            watch = self._watch
            if watch is None:
                return
            q = np.asarray(feedback.body_q) - watch.base
            dq = np.abs(np.asarray(feedback.body_dq))
            k = int(dq.argmax())
            if watch.returning:
                watch.return_dq = max(watch.return_dq, (float(dq[k]), k))
                return
            progress = float(q @ watch.delta / (watch.delta @ watch.delta))
            if progress > watch.reached:
                watch.reached, watch.reached_q = progress, np.asarray(feedback.body_q, np.float64)
            watch.deviation = np.maximum(watch.deviation, np.abs(q))
            watch.stream_dq = max(watch.stream_dq, (float(dq[k]), k))
            if watch.abort is None:
                outside = np.nonzero(np.abs(q) > watch.allowed)[0]
                if len(outside):
                    j = int(outside[0])
                    watch.abort = f"{_JOINTS[j]} left its envelope ({q[j]:+.3f} rad)"
                elif dq[k] > watch.max_dq:
                    watch.abort = f"{_JOINTS[k]} speed {dq[k]:.2f} rad/s > {watch.max_dq}"

    @skill(uses=[CAP_MOVEMENT])
    def move_upper_body(self, motion: UpperBodyMotion, angle_deg: float) -> SkillResult[ArmError]:
        """Move one upper-body joint out by angle_deg, hold it for 1 second, move it back, then hand control back.

        Motions: "left_arm_forward" swings the left arm forward (shoulder pitch),
        "left_arm_sideways" raises the left arm out to the side (shoulder roll), and
        "left_forearm_raise" lifts the left forearm (elbow). Min-jerk, 2 s out and 2 s back (longer
        for big angles), while SONIC keeps the robot balanced; 18-24 s in all. Requires the SONIC
        controller running in confirmed planner idle; it never starts or stops the controller. Stops
        early and returns control to the planner if any joint strays from the plan.

        Args:
            motion: One of "left_arm_forward", "left_arm_sideways", "left_forearm_raise".
            angle_deg: How far to move, in degrees: left_arm_forward 10-28, left_arm_sideways 10-40,
                left_forearm_raise 20-60.
        """
        if motion not in MOTIONS:
            return SkillResult.fail("INVALID_MOTION", f"motion must be one of {sorted(MOTIONS)}")
        joint, sign, low, high = MOTIONS[motion]
        if not low <= angle_deg <= high or not math.isfinite(angle_deg):
            return SkillResult.fail(
                "INVALID_ANGLE", f"{motion} angle_deg must be within {low:g}..{high:g}"
            )
        delta = np.zeros(29)
        delta[joint] = sign * math.radians(angle_deg)
        allowed = group_margins(
            self.config.body_margin, self.config.arm_margin, self.config.wrist_margin
        )
        return self._run(motion, lambda base: delta, allowed, require_check=False)

    @skill(uses=[CAP_MOVEMENT])
    def move_left_arm_pose(self, pose: LeftArmPose) -> SkillResult[ArmError]:
        """Move the left arm to a full-range pose, hold it for 1 second, bring it back, then hand control back.

        Poses: "left_arm_forward_horizontal" (straight arm pointing forward at shoulder height),
        "left_arm_side_horizontal" (straight arm out to the side at shoulder height),
        "left_hand_over_head" (arm raised forward and up until the hand is over the head),
        "left_hand_over_head_sideways" (arm raised out to the side and up over the head). Slow
        (peak 0.47 rad/s): 20-40 s in all, overhead the longest. Requires the SONIC controller running
        in confirmed planner idle; it never starts or stops the controller. Stops early and returns
        control to the planner if the legs, waist or the other arm stray, or the arm strays far from
        the plan.

        Args:
            pose: One of "left_arm_forward_horizontal", "left_arm_side_horizontal",
                "left_hand_over_head", "left_hand_over_head_sideways".
        """
        if pose not in POSES:
            return SkillResult.fail("INVALID_MOTION", f"pose must be one of {sorted(POSES)}")
        return self._run_targets(pose, np.asarray(POSES[pose]), require_check=False)

    @skill(uses=[CAP_MOVEMENT])
    def move_left_arm_joints(
        self,
        shoulder_pitch_deg: float,
        shoulder_roll_deg: float,
        shoulder_yaw_deg: float,
        elbow_deg: float,
    ) -> SkillResult[ArmError]:
        """Move the left arm to the given joint angles, hold for 1 second, bring it back, then hand control back.

        Angles are absolute, in degrees. Hanging rest is about pitch 11, roll 11, yaw 0, elbow 34.
        Shoulder pitch: negative raises the arm forward (-90 forward horizontal, -160 overhead).
        Shoulder roll: positive raises it out to the side (86 side horizontal). Shoulder yaw rotates
        the upper arm. Elbow: 82 is straight, 0 is bent 90 degrees, negative bends further. The
        planned path is checked against the robot's body first and refused if the arm would come
        near it. Slow (peak 0.47 rad/s), up to about 40 s. Prefer move_left_arm_pose for the
        named poses.

        Args:
            shoulder_pitch_deg: -165 to 45.
            shoulder_roll_deg: -10 to 120.
            shoulder_yaw_deg: -90 to 90.
            elbow_deg: -30 to 82.
        """
        values = (shoulder_pitch_deg, shoulder_roll_deg, shoulder_yaw_deg, elbow_deg)
        for (joint, (low, high)), value in zip(JOINT_BOUNDS.items(), values, strict=True):
            if not math.isfinite(value) or not low <= value <= high:
                return SkillResult.fail(
                    "INVALID_ANGLE", f"{joint} must be within {low:g}..{high:g} deg"
                )
        target = np.radians(np.asarray(values, np.float64))
        label = "joints(" + ", ".join(f"{v:g}" for v in values) + ")"
        return self._run_targets(label, target, require_check=True)

    def _run_targets(
        self, label: str, target: NDArray[np.float64], *, require_check: bool
    ) -> SkillResult[ArmError]:
        def delta_of(base: NDArray[np.float64]) -> NDArray[np.float64]:
            delta = np.zeros(29)
            delta[_LEFT_ARM_TARGET] = target - base[_LEFT_ARM_TARGET]
            return delta

        allowed = pose_margins(
            self.config.body_margin, self.config.pose_arm_margin, self.config.other_arm_margin
        )
        return self._run(label, delta_of, allowed, require_check=require_check)

    def _run(
        self,
        label: str,
        delta_of: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        allowed: NDArray[np.float64],
        *,
        require_check: bool,
    ) -> SkillResult[ArmError]:
        if require_check and self._checker is None:
            return SkillResult.fail(
                "NOT_READY",
                "collision_model_path is not configured; joint targets need the path check",
            )
        if not self._busy.acquire(blocking=False):
            return SkillResult.fail("BUSY", "an arm move is already running")
        try:
            return self._move(label, delta_of, allowed)
        finally:
            with self._lock:
                self._watch = None
            self._busy.release()

    def _move(
        self,
        label: str,
        delta_of: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        allowed: NDArray[np.float64],
    ) -> SkillResult[ArmError]:
        assert self._encoder is not None
        status = self._execution.get_status()
        profile = status["profile"]
        if not status["ready"]:
            return SkillResult.fail("NOT_READY", "SONIC must be running in confirmed planner idle")
        state = self._execution.get_feedback()
        if state is None or state.token_state is None:
            return SkillResult.fail("NOT_READY", "no fresh controller feedback")
        base = np.asarray(state.body_q, np.float64)
        delta = delta_of(base)
        if not np.abs(delta).max() > math.radians(1):
            return SkillResult.fail("INVALID_ANGLE", "the arm is already there; nothing to move")
        poses = stroke_plan(base, delta)
        # First row at full depth, in token-plan rows (token_schedule prepends a 2 s blend).
        hold_start = 2 * RATE + int(np.argmax(np.abs(poses - base).sum(axis=1)))
        if self._checker is not None and (contact := self._checker.first_contact(poses)):
            return SkillResult.fail(
                "UNSAFE_PATH",
                f"the planned arm path comes within 2 cm of the {contact}; nothing sent",
            )
        started = time.monotonic()
        motion_tokens = self._encoder.encode(poses)
        encode_s = time.monotonic() - started
        current = self._execution.get_feedback()
        if current is None or np.abs(np.asarray(current.body_q) - base).max() > _MOVED_LIMIT:
            return SkillResult.fail(
                "MOVED", "the body moved while the reference was encoded; nothing sent"
            )
        tokens = token_schedule(np.asarray(state.token_state), motion_tokens)
        hands = np.r_[state.left_hand_q, state.right_hand_q]
        plan = np.hstack([tokens, np.tile(hands, (len(tokens), 1))])
        if (
            profile["max_observation_age"] <= len(plan) / RATE + 0.5
            or profile["max_duration"] <= len(plan) / RATE + 5.0
        ):
            return SkillResult.fail(
                "NOT_READY",
                "the execution profile's observation age and duration must cover the plan",
            )
        allowed = allowed + np.abs(delta)
        watch = _Watch(base=base, allowed=allowed, max_dq=self.config.max_dq, delta=delta)
        with self._lock:
            self._watch = watch

        try:
            task_id = self._execution.start_task()["task_id"]
        except Exception as exc:
            return SkillResult.fail("NOT_READY", f"execution refused the task: {exc}; nothing sent")
        try:
            outcome = self._stream(task_id, plan, chunk_rows(len(plan), hold_start), profile, watch)
        except Exception as exc:
            outcome = f"{type(exc).__name__}: {exc}"
            try:
                self._execution.cancel_task(task_id)
            except Exception:
                pass  # The execution module fails the session on its own faults.
        task = self._wait_done(profile)
        with self._lock:
            watch.returning = True
        if task is not None and task["handback_confirmed"]:
            time.sleep(self.config.return_watch_seconds)

        moved = np.nonzero(delta)[0]
        report = {
            "task_id": task_id,
            "motion": label,
            "reached_fraction": round(watch.reached, 2),
            "planned_deg": {_JOINTS[j]: round(math.degrees(base[j] + delta[j]), 1) for j in moved},
            "reached_deg": {
                _JOINTS[j]: round(math.degrees(watch.reached_q[j]), 1)
                if watch.reached > 0
                else None
                for j in moved
            },
            "max_deviation_rad": {
                # The moving joints are the motion itself, not deviations.
                g: round(float(max([watch.deviation[j] for j in i if j not in moved] or [0.0])), 3)
                for g, i in _GROUPS.items()
            },
            "stream_peak_dq": _peak(watch.stream_dq),
            "return_peak_dq": _peak(watch.return_dq),
            "encode_s": round(encode_s, 2),
            "journal_path": self._execution.get_status()["journal_path"],
        }
        if len(moved) == 1:  # move_upper_body: the swing in degrees, as before.
            report["reached_deg"] = round(math.degrees(watch.reached * abs(delta[moved[0]])), 1)
            report["planned_deg"] = round(math.degrees(abs(delta[moved[0]])), 1)
        if task is None or not task["handback_confirmed"]:
            return _fail(
                "HANDBACK_UNCONFIRMED",
                f"planner handback unconfirmed ({outcome}); check the controller",
                report,
            )
        if outcome == "envelope":
            return _fail(
                "ENVELOPE", f"stopped early: {watch.abort}; control returned to the planner", report
            )
        if outcome == "finished" and task["outcome"] != "finished":
            outcome = f"session {task['outcome']}: {task['reason']}"
        if outcome != "finished":
            return _fail("EXECUTION_FAILED", f"{outcome}; control returned to the planner", report)
        if watch.reached < 0.5:  # SONIC tracked less than half the planned move.
            return _fail(
                "NOT_REACHED", "the arm did not follow the reference; planner in control", report
            )
        reached = (
            f"reached {report['reached_deg']:.1f} deg"
            if len(moved) == 1
            else f"reached {100 * watch.reached:.0f}% of the planned pose"
        )
        return SkillResult.ok(f"{label} {reached} and returned; planner in control", **report)

    def _stream(
        self,
        task_id: str,
        plan: NDArray[np.float64],
        chunks: list[tuple[int, int]],
        profile: dict[str, Any],
        watch: _Watch,
    ) -> str:
        startup = time.monotonic() + profile["seed_seconds"] + profile["arm_timeout"] + 1.0
        while (task := self._task())["phase"] != "waiting_for_chunk":
            if task["phase"] == "done" or time.monotonic() > startup:
                return f"stream did not arm ({task['reason']})"
            time.sleep(0.01)
        plan_start = next_check = time.monotonic()
        start, end = chunks[0]
        task = self._execution.submit_chunk(task_id, 0, plan_start, plan[start:end])
        pending = chunks[1:]
        while True:
            with self._lock:
                abort = watch.abort
            if abort is not None:
                self._execution.cancel_task(task_id)
                return "envelope"
            now = time.monotonic()
            if (now - plan_start) * RATE >= len(plan) - 2:
                # The plan already blended back to the planner token.
                self._execution.finish_task(task_id)
                return "finished"  # Checked against the session's own outcome afterwards.
            if pending and now >= plan_start + pending[0][0] / RATE:
                start, end = pending.pop(0)
                task = self._execution.submit_chunk(
                    task_id,
                    len(chunks) - len(pending) - 1,
                    plan_start + start / RATE,
                    plan[start:end],
                )
            elif now >= next_check:
                task, next_check = self._task(), now + 0.1
            if task["phase"] not in ("waiting_for_chunk", "running"):
                return f"session ended: {task['reason']}"
            time.sleep(0.005)

    def _task(self) -> dict[str, Any]:
        task: dict[str, Any] = self._execution.get_status()["task"]
        return task

    def _wait_done(self, profile: dict[str, Any]) -> dict[str, Any] | None:
        deadline = time.monotonic() + profile["handback_timeout"] + 1.0
        while time.monotonic() < deadline:
            task = self._task()
            if task["phase"] == "done":
                return task
            time.sleep(0.01)
        return None


sonic_arm_skill_container = SonicArmSkillContainer.blueprint
