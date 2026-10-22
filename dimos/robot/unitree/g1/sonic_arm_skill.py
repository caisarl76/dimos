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

Encodes a min-jerk keyframe reference from the measured pose (sonic_arm_reference), streams it as
a sonic78 source and hands control back to the native planner. A plan of up to 1000 rows is one
open-loop chunk planned from the start pose, exactly as commissioned on a G1; a longer one is split
only inside a hold, because every chunk restarts the session blend, a 2.5 Hz speed ripple while the
arm moves. The execution profile's max_observation_age must cover the longest chunk. A safety stop
retreats along the plan's own way home instead of handing a raised arm to the native planner.
observed_at uses this host's monotonic clock: run on the same host as the execution module. Never
starts or stops native control. Cancels to the planner if a joint leaves its per-group envelope or
exceeds `max_dq`. The motion skills run in the background and report through the tool stream;
stop_arm retreats the same way a safety stop does. run_arms starts several robots' arms together:
all-or-nothing up to the first chunk, then any one's stop or failure retreats the others.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from contextlib import suppress
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Literal, NamedTuple, Protocol

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
from dimos.models.vl.base import VlModel
from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.sonic_arm_reference import (
    _SPLIT_STILL_ROWS,
    ARM_JOINTS,
    HEAD_CAMERA_SIZE,
    JOINT_BOUNDS,
    LEFT_SHOULDER_PITCH,
    LOOSE_JOINTS,
    POINT_AZIMUTH_DEG,
    POINT_ELEVATION_DEG,
    RATE,
    ArmCollisionChecker,
    ArmKinematics,
    Keyframe,
    SonicEncoder,
    group_margins,
    keyframe_plan,
    pose_margins,
    retreat_poses,
    retreat_route,
    stroke_plan,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_session import validate_actions
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
GestureName = Literal["wave", "point_forward", "stop_sign", "raise_hand", "beckon"]
_WAVE_RAISE = (-0.3, 1.3, 0.0, 0.0)
# Left-arm gestures as absolute (shoulder pitch, roll, yaw, elbow) rad and the hold at each, s; the
# engine adds the way back to the measured start. Wave swings the shoulder yaw, beckon the elbow.
GESTURES: dict[str, list[tuple[tuple[float, float, float, float], float]]] = {
    "wave": [
        (_WAVE_RAISE, 0.0),
        *[((-0.3, 1.3, yaw, 0.0), 0.0) for yaw in (-0.35, 0.35) * 3],
        (_WAVE_RAISE, 0.5),
    ],
    "point_forward": [((-1.5, 0.25, 0.0, 1.35), 2.0)],
    "stop_sign": [((-1.3, 0.25, 0.0, 0.0), 2.0)],
    # Roll 0.6, not 0.45: in the MuJoCo stand-in 0.45 tripped the 0.15 knee envelope during the
    # hold every time (0.6: 0.12 rad).
    "raise_hand": [((-2.6, 0.6, 0.0, 1.2), 2.0)],
    "beckon": [
        ((-1.2, 0.3, 0.0, 0.6), 0.0),
        *[((-1.2, 0.3, 0.0, elbow), 0.0) for elbow in (0.0, 0.6) * 2],
    ],
}
GESTURE_MAX_S = 40.0  # longest plan a gesture may take, lead-in and way home included
_LEFT_ARM_TARGET = [15, 16, 17, 18]
_MOVED_LIMIT = 0.05  # rad: body motion allowed while encoding.
_MAX_CHUNK_ROWS = 1000  # SonicTaskSession accepts chunks of at most 1000 rows (20 s).
_CHUNK_OVERLAP = 25  # rows the first chunk runs past the split, so it never runs dry
_HOLD_BOUNDS = (1.0, 20.0)  # seconds
# Fraction of the planned displacement, measured from the pose at the chunk-0 send, that marks the
# stroke's start. Projected on the plan so secondary-joint drift in the token blend does not count.
_ONSET_PROGRESS = 0.10
_NO_PATH_CHECK = "collision_model_path is not configured; joint targets need the path check"
_NO_ARM_MODEL = "collision_model_path is not configured; point_at needs the arm model"
_FRAME_TIMEOUT_S = 5.0
# How long Module.stop() waits for a running motion to retreat and hand back: the longest retreat
# is one session chunk (20 s), then _wait_done (handback_timeout 2 s + 1 s) and at most 5 s of
# return_watch_seconds; it also covers a stop that lands during encoding (wave: ~14 s on pc2).
_STOP_JOIN_S = 30.0
BOX_PROMPT = """Find every `{query}` in this image.

Reply with only JSON, no other text: a list of boxes, one per match,
[["label", x1, y1, x2, y2], ...]
where (x1, y1) is the box's top-left corner and (x2, y2) its bottom-right corner.
If there is no match, reply [].
"""
BoxScale = Literal["pixels", "normalized_1000"]


def query_boxes(
    vl: VlModel, image: Image, query: str, box_scale: BoxScale = "pixels"
) -> list[tuple[str, float, float, float, float]]:
    """(label, x1, y1, x2, y2) pixel boxes the vision model finds for `query`; model errors propagate,
    and a reply that is not a JSON list raises ValueError.

    Coordinates are scaled to pixels first ("normalized_1000": 0-1000 on each axis), then clipped
    to the image; malformed entries and boxes with no area left are dropped.
    """
    raw = vl.query_json(image, BOX_PROMPT.format(query=query))
    if not isinstance(raw, list):
        raise ValueError("vision model reply is not a list of boxes")
    w, h = image.width, image.height
    sx, sy = (w / 1000, h / 1000) if box_scale == "normalized_1000" else (1.0, 1.0)
    boxes = []
    for entry in raw:
        try:
            label, x1, y1, x2, y2 = entry
            x1, x2 = (min(max(float(x) * sx, 0.0), w) for x in (x1, x2))
            y1, y2 = (min(max(float(y) * sy, 0.0), h) for y in (y1, y2))
        except (TypeError, ValueError):
            continue
        if x2 > x1 and y2 > y1:  # Also drops NaN and inverted corners.
            boxes.append((str(label), x1, y1, x2, y2))
    return boxes


def chunk_rows(plan_rows: int, still: NDArray[np.bool_]) -> list[tuple[int, int]]:
    """Greedy session chunks, restarting the blend only at stationary token-plan rows."""
    chunks = []
    start = 0
    while plan_rows - start > _MAX_CHUNK_ROWS:
        candidates = np.flatnonzero(still[start + 1 : start + _MAX_CHUNK_ROWS - _CHUNK_OVERLAP + 1])
        if not len(candidates):
            raise ValueError("plan cannot be split into session chunks")
        split = start + 1 + int(candidates[-1])
        chunks.append((start, split + _CHUNK_OVERLAP))
        start = split
    chunks.append((start, plan_rows))
    return chunks


def token_still(tokens: NDArray[Any]) -> NDArray[np.bool_]:
    """Rows whose next 15 tokens are all within 1e-9 of the first (including itself)."""
    still = np.zeros(len(tokens), dtype=np.bool_)
    for row in range(len(tokens) - _SPLIT_STILL_ROWS + 1):
        still[row] = np.all(np.abs(tokens[row : row + _SPLIT_STILL_ROWS] - tokens[row]) <= 1e-9)
    return still


def gesture_keyframes(name: str, base: NDArray[np.float64]) -> list[Keyframe]:
    """The gesture's keyframes: its left-arm targets over `base` (wrists and everything else kept)."""
    keyframes = []
    for arm, hold in GESTURES[name]:
        target = base.copy()
        target[_LEFT_ARM_TARGET] = arm
        keyframes.append(Keyframe(target, hold))
    return keyframes


def _check_hold(hold_seconds: float) -> None:
    low, high = _HOLD_BOUNDS
    if not math.isfinite(hold_seconds) or not low <= hold_seconds <= high:
        raise ValueError(f"hold_seconds must be within {low:g}..{high:g}")


# Extra report fields from the start pose and the body_q samples held at the deepest keyframe.
ReportExtra = Callable[[NDArray[np.float64], NDArray[np.float64] | None], dict[str, Any]]


class ArmMove(NamedTuple):
    """What a skill asks of an arm: SonicArm.prepare's label, keyframes, envelope and check mode."""

    label: str
    keyframes_of: Callable[[NDArray[np.float64]], list[Keyframe]]
    allowed: NDArray[np.float64]
    require_check: bool


def upper_body_move(config: SonicArmSkillConfig, motion: str, angle_deg: float) -> ArmMove:
    """Validate a move_upper_body call."""
    if motion not in MOTIONS:
        raise ValueError(f"motion must be one of {sorted(MOTIONS)}")
    joint, sign, low, high = MOTIONS[motion]
    if not low <= angle_deg <= high or not math.isfinite(angle_deg):
        raise ValueError(f"{motion} angle_deg must be within {low:g}..{high:g}")
    delta = np.zeros(29)
    delta[joint] = sign * math.radians(angle_deg)
    allowed = group_margins(config.body_margin, config.arm_margin, config.wrist_margin)
    return ArmMove(motion, lambda base: [Keyframe(base + delta, 1.0)], allowed, False)


def _target_move(
    config: SonicArmSkillConfig,
    label: str,
    target: NDArray[np.float64],
    hold_seconds: float,
    require_check: bool,
) -> ArmMove:
    def keyframes_of(base: NDArray[np.float64]) -> list[Keyframe]:
        goal = base.copy()
        goal[_LEFT_ARM_TARGET] = target
        return [Keyframe(goal, hold_seconds)]

    allowed = pose_margins(config.body_margin, config.pose_arm_margin, config.other_arm_margin)
    return ArmMove(label, keyframes_of, allowed, require_check)


def left_arm_pose_move(config: SonicArmSkillConfig, pose: str, hold_seconds: float) -> ArmMove:
    """Validate a move_left_arm_pose call."""
    if pose not in POSES:
        raise ValueError(f"pose must be one of {sorted(POSES)}")
    _check_hold(hold_seconds)
    return _target_move(config, pose, np.asarray(POSES[pose]), hold_seconds, False)


def left_arm_joints_move(
    config: SonicArmSkillConfig, values: tuple[float, float, float, float], hold_seconds: float
) -> ArmMove:
    """Validate a move_left_arm_joints call (the caller checks its arms have a path checker)."""
    for (joint, (low, high)), value in zip(JOINT_BOUNDS.items(), values, strict=True):
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{joint} must be within {low:g}..{high:g} deg")
    _check_hold(hold_seconds)
    target = np.radians(np.asarray(values, np.float64))
    label = "joints(" + ", ".join(f"{v:g}" for v in values) + ")"
    return _target_move(config, label, target, hold_seconds, True)


def gesture_move(config: SonicArmSkillConfig, name: str) -> ArmMove:
    """Validate a perform_gesture call."""
    if name not in GESTURES:
        raise ValueError(f"name must be one of {sorted(GESTURES)}")

    def keyframes_of(base: NDArray[np.float64]) -> list[Keyframe]:
        keyframes = gesture_keyframes(name, base)
        seconds = len(keyframe_plan(base, keyframes).poses) / RATE
        if seconds > GESTURE_MAX_S:
            raise ValueError(
                f"{name} would take {seconds:.0f} s, more than {GESTURE_MAX_S:g} s; nothing sent"
            )
        return keyframes

    allowed = pose_margins(config.body_margin, config.pose_arm_margin, config.other_arm_margin)
    return ArmMove(name, keyframes_of, allowed, False)


_JOINTS = [name.removeprefix("g1/") for name in make_humanoid_joints("g1")]
_GROUPS = {  # Reporting groups, matching the envelope groups.
    "legs": list(range(12)),
    "waist": [12, 13, 14],
    "arms": ARM_JOINTS,
    "wrists_shoulder_yaw": LOOSE_JOINTS,
}


def _peak(peak: tuple[float, int]) -> dict[str, Any]:
    return {"rad_s": round(peak[0], 3), "joint": _JOINTS[peak[1]] if peak[0] > 0 else None}


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
    # gear_sonic scene_43dof.xml for the runtime path check and point_at's kinematics;
    # move_left_arm_joints and point_at need it.
    collision_model_path: Path | None = None
    # OpenAI-compatible vision model for point_at_object (OPENAI_BASE_URL / OPENAI_API_KEY from the
    # environment); None disables the skill. vl_box_scale is the model's box coordinate convention.
    vl_model: str | None = None
    vl_box_scale: BoxScale = "pixels"


@dataclass
class _Watch:
    base: NDArray[np.float64]
    allowed: NDArray[np.float64]
    max_dq: float
    delta: NDArray[np.float64]  # The planned displacement of every joint at the deepest keyframe.
    returning: bool = False
    reached: float = 0.0  # Furthest progress along the planned displacement, as a fraction.
    reached_q: NDArray[np.float64] = field(default_factory=lambda: np.zeros(29))
    deviation: NDArray[np.float64] = field(default_factory=lambda: np.zeros(29))
    stream_dq: tuple[float, int] = (0.0, -1)
    return_dq: tuple[float, int] = (0.0, -1)
    abort: str | None = None
    overspeed_at: float = -1.0  # monotonic time of the last sample above max_dq
    hold_rows: tuple[int, int] | None = None  # token-plan rows of the deepest keyframe's hold
    plan_start: float | None = None  # monotonic time of the first chunk's row 0
    hold_q: list[NDArray[np.float64]] = field(default_factory=list)  # body_q inside the hold
    # Start timing, all on the execution host's monotonic clock: just before and just after the
    # chunk-0 submit_chunk call, and the receipt time of the first sample whose progress along delta,
    # measured from onset_q (the latest sample received before the send), reached _ONSET_PROGRESS.
    send_at: float | None = None
    ack_at: float | None = None
    onset_q: NDArray[np.float64] | None = None
    onset_at: float = -math.inf
    moved_at: float | None = None


@dataclass
class _Group:
    """What an arm started together with others shares with them."""

    barrier: threading.Barrier  # Every arm submits chunk 0 only once all are armed.
    peers: list[tuple[str, _Watch]]  # The other arms' names and watches.


@dataclass
class _Prepared:
    """A planned move, checked and encoded; nothing has been sent."""

    label: str
    base: NDArray[np.float64]
    deltas: NDArray[np.float64]
    delta: NDArray[np.float64]
    plan: NDArray[np.float64]
    chunks: list[tuple[int, int]]
    retreat: Callable[[int], NDArray[np.float64] | None]
    profile: dict[str, Any]
    watch: _Watch
    keyframes: int
    encode_s: float
    report_extra: ReportExtra | None


class SonicArm:
    """One robot's arm engine: plans, checks and streams a move through its execution module."""

    def __init__(
        self,
        name: str,
        config: SonicArmSkillConfig,
        execution: SonicArmExecutionSpec,
        encoder: SonicEncoder,
        checker: ArmCollisionChecker | None,
    ) -> None:
        self.name = name
        self.config = config
        self._execution = execution
        self._encoder = encoder
        self._checker = checker
        self._lock = threading.Lock()
        self._watch: _Watch | None = None
        self.stop = threading.Event()

    def reset(self) -> None:
        """Forget the previous motion: clear its stop request and watch."""
        self.stop.clear()
        with self._lock:
            self._watch = None

    def clear_watch(self) -> None:
        with self._lock:
            self._watch = None

    def handle_feedback(self, feedback: SonicFeedback) -> None:
        """Track the active move's envelope, reach and speeds from every telemetry sample."""
        with self._lock:
            watch = self._watch
            if watch is None:
                return
            body = np.asarray(feedback.body_q, np.float64)
            q = body - watch.base
            dq = np.abs(np.asarray(feedback.body_dq))
            k = int(dq.argmax())
            if watch.returning:
                watch.return_dq = max(watch.return_dq, (float(dq[k]), k))
                return
            # Receipt time, not this handler's: samples queue before they get here.
            received = feedback.received_monotonic
            if watch.send_at is None or received <= watch.send_at:
                if received >= watch.onset_at:
                    watch.onset_q, watch.onset_at = body, received
            elif watch.onset_q is not None and (
                watch.moved_at is None or received < watch.moved_at
            ):
                moved = (body - watch.onset_q) @ watch.delta / (watch.delta @ watch.delta)
                if moved >= _ONSET_PROGRESS:
                    watch.moved_at = received
            progress = float(q @ watch.delta / (watch.delta @ watch.delta))
            if progress > watch.reached:
                watch.reached, watch.reached_q = progress, body
            watch.deviation = np.maximum(watch.deviation, np.abs(q))
            watch.stream_dq = max(watch.stream_dq, (float(dq[k]), k))
            if dq[k] > watch.max_dq:
                watch.overspeed_at = time.monotonic()
            if watch.abort is None:
                outside = np.nonzero(np.abs(q) > watch.allowed)[0]
                if len(outside):
                    j = int(outside[0])
                    watch.abort = f"{_JOINTS[j]} left its envelope ({q[j]:+.3f} rad)"
                elif dq[k] > watch.max_dq:
                    watch.abort = f"{_JOINTS[k]} speed {dq[k]:.2f} rad/s > {watch.max_dq}"
            # After the envelope check, so the sample that trips it is not collected. A peer's
            # abort reaches watch.abort through _stream, and stops collection on this arm too.
            if (
                watch.abort is None
                and not self.stop.is_set()
                and watch.plan_start is not None
                and watch.hold_rows is not None
                and watch.hold_rows[0] <= (received - watch.plan_start) * RATE <= watch.hold_rows[1]
            ):
                watch.hold_q.append(body)

    def prepare(
        self,
        label: str,
        keyframes_of: Callable[[NDArray[np.float64]], list[Keyframe]],
        allowed: NDArray[np.float64],
        *,
        require_check: bool,
        report_extra: ReportExtra | None = None,
    ) -> _Prepared:
        """Plan, check and encode a move. Raises with nothing sent."""
        if require_check and self._checker is None:
            raise RuntimeError(_NO_PATH_CHECK)
        status = self._execution.get_status()
        profile = status["profile"]
        if not status["ready"]:
            raise RuntimeError("SONIC is not in confirmed planner idle; nothing sent")
        state = self._execution.get_feedback()
        if state is None or state.token_state is None:
            raise RuntimeError("no fresh controller feedback; nothing sent")
        base = np.asarray(state.body_q, np.float64)
        keyframes = keyframes_of(base)
        targets = np.asarray([k.target for k in keyframes], np.float64).reshape(-1, 29)
        deltas = targets - base
        if not np.abs(deltas).max() > math.radians(1):
            raise ValueError("the arm is already there; nothing to move")
        # Token-plan rows: token_schedule adds the same blend at each end of the poses.
        pad = (len(token_schedule(np.zeros(64), np.zeros((1, 64)))) - 1) // 2
        kplan = keyframe_plan(base, keyframes, split_every=_MAX_CHUNK_ROWS - _CHUNK_OVERLAP - pad)
        # Where a safety stop at each segment resumes. A unit that is the tail of the plan reuses
        # the plan's own tokens.
        units = [retreat_poses(kplan, s) for s in range(len(kplan.segments))]
        is_tail = [np.array_equal(u, kplan.poses[len(kplan.poses) - len(u) :]) for u in units]
        if any(len(u) + pad > _MAX_CHUNK_ROWS for u in units):
            raise ValueError("a retreat is longer than one session chunk")
        extra = {u.tobytes(): u for u, tail in zip(units, is_tail, strict=True) if not tail}
        if self._checker is not None:
            for path in (kplan.poses, *extra.values()):
                if contact := self._checker.first_contact(path, start=base):
                    raise RuntimeError(
                        f"the planned arm path comes within 2 cm of the {contact}; nothing sent",
                    )
        started = time.monotonic()
        motion_tokens = self._encoder.encode(kplan.poses)
        unit_tokens = {key: self._encoder.encode(u) for key, u in extra.items()}
        encode_s = time.monotonic() - started
        if self.stop.is_set():
            raise RuntimeError("stopped on request; nothing sent")
        current = self._execution.get_feedback()
        if current is None or np.abs(np.asarray(current.body_q) - base).max() > _MOVED_LIMIT:
            raise RuntimeError("the body moved while the reference was encoded; nothing sent")
        captured = np.asarray(state.token_state)
        tokens = token_schedule(captured, motion_tokens)
        hands = np.r_[state.left_hand_q, state.right_hand_q]
        plan = np.hstack([tokens, np.tile(hands, (len(tokens), 1))])
        chunks = chunk_rows(len(plan), token_still(tokens))

        def retreat_rows(segment: int, offset: int) -> NDArray[np.float64]:
            unit = units[segment]
            unit_rows = (
                motion_tokens[len(motion_tokens) - len(unit) :]
                if is_tail[segment]
                else unit_tokens[unit.tobytes()]
            )
            rows = token_schedule(captured, unit_rows)[pad + offset :]
            return np.hstack([rows, np.tile(hands, (len(rows), 1))])

        def retreat(token_row: int) -> NDArray[np.float64] | None:
            if token_row < pad:  # Still in the lead-in blend: play it backwards to the planner.
                return np.vstack([plan[token_row::-1], plan[-1]])
            route = retreat_route(kplan, token_row - pad)
            if route is None:
                return None  # Already on the way home.
            return retreat_rows(*route)

        longest_chunk = max(end - start for start, end in chunks)
        longest_retreat = max(len(u) + pad for u in units)
        if (
            profile["max_observation_age"] <= max(longest_chunk, longest_retreat) / RATE + 0.5
            or profile["max_duration"] <= len(plan) / RATE + longest_retreat / RATE + 2.0
        ):
            raise RuntimeError(
                "the execution profile's observation age and duration must cover the plan",
            )
        deepest = int(np.argmax(np.linalg.norm(deltas, axis=1)))
        delta = deltas[deepest]
        seg, nxt = kplan.segments[deepest], kplan.segments[deepest + 1]
        hold = (pad + seg.end - 1 + 25, pad + nxt.start - 25)
        allowed = allowed + np.abs(deltas).max(axis=0)
        token_bound = profile["token_bound"]
        for start, end in chunks:  # What the session would reject is refused with nothing sent.
            validate_actions(plan[start:end], token_bound)
        for segment in range(len(units)):
            validate_actions(retreat_rows(segment, 0), token_bound)
        return _Prepared(
            label=label,
            base=base,
            deltas=deltas,
            delta=delta,
            plan=plan,
            chunks=chunks,
            retreat=retreat,
            profile=profile,
            watch=_Watch(
                base=base,
                allowed=allowed,
                max_dq=self.config.max_dq,
                delta=delta,
                hold_rows=hold if hold[0] <= hold[1] else None,
            ),
            keyframes=len(keyframes),
            encode_s=encode_s,
            report_extra=report_extra,
        )

    def recheck(self, prepared: _Prepared) -> None:
        """The checks made after encoding, again just before the task starts."""
        if not self._execution.get_status()["ready"]:
            raise RuntimeError("SONIC is not in confirmed planner idle; nothing sent")
        current = self._execution.get_feedback()
        if current is None or current.token_state is None:
            raise RuntimeError("no fresh controller feedback; nothing sent")
        if np.abs(np.asarray(current.body_q) - prepared.base).max() > _MOVED_LIMIT:
            raise RuntimeError("the body moved before the task started; nothing sent")

    def execute(self, prepared: _Prepared, group: _Group | None = None) -> SkillResult:
        """Start the task, stream the prepared move and report.

        Any non-success sets this arm's watch.abort, so peers still streaming in its group retreat.
        """
        try:
            return self._execute(prepared, group)
        except BaseException as exc:
            if group is not None:
                group.barrier.abort()  # A peer still arming calls its start off at once.
            self._fail(prepared.watch, str(exc) or type(exc).__name__)
            raise

    def _fail(self, watch: _Watch, reason: str) -> None:
        with self._lock:
            if watch.abort is None:
                watch.abort = reason

    def _cancel(self, task_id: str) -> None:
        with suppress(Exception):  # The execution module fails the session on its own faults.
            self._execution.cancel_task(task_id)

    def _execute(self, prepared: _Prepared, group: _Group | None) -> SkillResult:
        self.recheck(prepared)
        label, base, deltas, delta = prepared.label, prepared.base, prepared.deltas, prepared.delta
        plan, chunks, profile, watch = (
            prepared.plan,
            prepared.chunks,
            prepared.profile,
            prepared.watch,
        )
        with self._lock:
            self._watch = watch
        try:
            task_id = self._execution.start_task()["task_id"]
        except Exception as exc:
            raise RuntimeError(f"execution refused the task: {exc}; nothing sent") from exc
        try:
            outcome = self._stream(task_id, plan, chunks, prepared.retreat, profile, watch, group)
        except Exception as exc:
            outcome = f"{type(exc).__name__}: {exc}"
            self._cancel(task_id)
        if outcome != "finished":
            self._fail(watch, outcome)
        task = self._wait_done(profile)
        if task is None or not task["handback_confirmed"]:
            self._fail(watch, "planner handback unconfirmed")
        elif task["outcome"] != "finished":
            self._fail(watch, f"session {task['outcome']}: {task['reason']}")
        with self._lock:
            watch.returning = True
        if task is not None and task["handback_confirmed"]:
            time.sleep(self.config.return_watch_seconds)

        moved = np.nonzero(np.any(deltas != 0, axis=0))[0]
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
            "keyframes": prepared.keyframes,
            "chunks": len(chunks),
            "encode_s": round(prepared.encode_s, 2),
            "journal_path": self._execution.get_status()["journal_path"],
        }
        with self._lock:
            samples = list(watch.hold_q)
        hold_q = np.asarray(samples) if samples else None
        report["hold_samples"] = len(samples)
        if prepared.report_extra is not None:
            report.update(prepared.report_extra(base, hold_q))
        if len(moved) == 1:  # move_upper_body: the swing in degrees, as before.
            report["reached_deg"] = round(math.degrees(watch.reached * abs(delta[moved[0]])), 1)
            report["planned_deg"] = round(math.degrees(abs(delta[moved[0]])), 1)
        journal = report["journal_path"]
        if task is None or not task["handback_confirmed"]:
            raise RuntimeError(
                f"planner handback unconfirmed ({outcome}); check the controller. Journal: {journal}"
            )
        if outcome == "finished" and task["outcome"] != "finished":
            outcome = f"session {task['outcome']}: {task['reason']}"
        if outcome == "stopped":
            raise RuntimeError("stopped on request; nothing sent")
        if outcome not in ("finished", "envelope"):
            raise RuntimeError(f"{outcome}; control returned to the planner. Journal: {journal}")
        if outcome == "envelope":
            message = f"{label} stopped early: {watch.abort}"
        elif watch.reached < 0.5:  # SONIC tracked less than half the planned move.
            message = f"{label} did not follow the reference: reached {100 * watch.reached:.0f}% of the plan"
        elif len(moved) == 1:
            message = f"{label} reached {report['reached_deg']:.1f} deg and returned"
        else:
            message = f"{label} reached {100 * watch.reached:.0f}% of the planned pose and returned"
        return SkillResult(f"{message}; control returned to SONIC's planner", metadata=report)

    def _stream(
        self,
        task_id: str,
        plan: NDArray[np.float64],
        chunks: list[tuple[int, int]],
        retreat: Callable[[int], NDArray[np.float64] | None],
        profile: dict[str, Any],
        watch: _Watch,
        group: _Group | None = None,
    ) -> str:
        barrier = None if group is None else group.barrier
        peers = [] if group is None else group.peers
        startup = time.monotonic() + profile["seed_seconds"] + profile["arm_timeout"] + 1.0
        while (task := self._task())["phase"] != "waiting_for_chunk":
            called_off = barrier is not None and barrier.broken
            if called_off or task["phase"] == "done" or time.monotonic() > startup:
                if barrier is not None:
                    barrier.abort()  # The peers give up at once, not at their barrier timeout.
                    if task["phase"] != "done":
                        self._cancel(task_id)  # Nothing sent: the session hands back as usual.
                if called_off and self.stop.is_set():
                    return "stopped"
                reason = "the joint start was called off" if called_off else task["reason"]
                return f"stream did not arm ({reason})"
            time.sleep(0.01)
        if self.stop.is_set():  # stop_arm while arming: hand back before any chunk.
            if barrier is not None:
                barrier.abort()  # Every peer calls its start off too: no robot gets a chunk.
            self._execution.finish_task(task_id)
            return "stopped"
        if barrier is not None:
            try:  # Released together; inside the session's first-chunk timeout.
                barrier.wait(timeout=profile["first_chunk_timeout"] / 2)
            except threading.BrokenBarrierError:
                self._cancel(task_id)
                return "stopped" if self.stop.is_set() else "another robot did not arm"
        plan_start = next_check = time.monotonic()
        with self._lock:  # Once, here: past any joint-start barrier, never on a retreat.
            watch.send_at = watch.plan_start = plan_start
        start, end = chunks[0]
        task = self._execution.submit_chunk(task_id, 0, plan_start, plan[start:end])
        watch.ack_at = time.monotonic()
        pending, sequence = chunks[1:], 0
        retreat_start: float | None = None  # set once a safety stop starts the smooth way back
        while True:
            now = time.monotonic()
            with self._lock:
                if self.stop.is_set() and watch.abort is None:
                    watch.abort = "stop requested"  # Retreats like a safety stop.
                for name, peer in peers:  # A peer's stop or failure retreats this arm too.
                    if watch.abort is None and (peer_abort := peer.abort) is not None:
                        watch.abort = f"{name}: {peer_abort}"
                abort, overspeed_at = watch.abort, watch.overspeed_at
            if abort is not None and retreat_start is None:
                if (rows := retreat(int((now - plan_start) * RATE))) is not None:
                    pending = []  # The way back replaces the rest of the plan.
                    sequence += 1
                    task = self._execution.submit_chunk(task_id, sequence, now, rows)
                    plan, plan_start = rows, now
                retreat_start = now
            elif retreat_start is not None and overspeed_at > retreat_start:
                self._execution.cancel_task(task_id)  # A joint ran away during the way back.
                return "envelope"
            if (now - plan_start) * RATE >= len(plan) - 2:
                # The plan already blended back to the planner token.
                self._execution.finish_task(task_id)
                # Checked against the session's own outcome afterwards.
                return "finished" if retreat_start is None else "envelope"
            if pending and now >= plan_start + pending[0][0] / RATE:
                start, end = pending.pop(0)
                sequence += 1
                task = self._execution.submit_chunk(
                    task_id, sequence, plan_start + start / RATE, plan[start:end]
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


def run_arms(jobs: Sequence[tuple[SonicArm, Callable[[], _Prepared]]]) -> SkillResult:
    """Run each arm's prepared move; several arms start together and stop together.

    One job is the single-arm path. With several, every prepare and then every recheck runs before
    any task starts, so a refusal sends nothing to any robot. Each arm then streams on its own
    thread; all submit chunk 0 once all are armed, and any arm's stop or failure retreats the rest.
    """
    if len(jobs) == 1:
        arm, prepare = jobs[0]
        return arm.execute(prepare())
    prepared: list[_Prepared] = []
    for arm, prepare in jobs:
        try:
            prepared.append(prepare())
        except Exception as exc:
            raise RuntimeError(f"{arm.name}: {exc}; nothing sent to any robot") from exc
    for (arm, _), move in zip(jobs, prepared, strict=True):
        try:  # Each pose may have moved while a later arm was encoding.
            arm.recheck(move)
        except Exception as exc:
            raise RuntimeError(f"{arm.name}: {exc}; nothing sent to any robot") from exc
    names = [arm.name for arm, _ in jobs]
    watches = [move.watch for move in prepared]
    barrier = threading.Barrier(len(jobs))
    outcomes: list[SkillResult | BaseException | None] = [None] * len(jobs)

    def run(i: int) -> None:
        peers = [
            (name, watch)
            for j, (name, watch) in enumerate(zip(names, watches, strict=True))
            if j != i
        ]
        try:
            outcomes[i] = jobs[i][0].execute(prepared[i], _Group(barrier, peers))
        except BaseException as exc:
            outcomes[i] = exc

    threads = [
        threading.Thread(target=run, args=(i,), name=f"arm-{name}", daemon=True)
        for i, name in enumerate(names)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    results = [o for o in outcomes if isinstance(o, SkillResult)]
    if len(results) < len(jobs):
        raise RuntimeError(
            "; ".join(
                f"{name}: {o.message if isinstance(o, SkillResult) else str(o) or type(o).__name__}"
                for name, o in zip(names, outcomes, strict=True)
            )
        )
    sends = [w.send_at for w in watches if w.send_at is not None]  # Every returned arm sent.
    moves = [w.moved_at for w in watches if w.moved_at is not None]
    first = min(sends)

    def since(t: float | None) -> float | None:
        return None if t is None else round(t - first, 3)

    submit_skew = round(max(sends) - min(sends), 3)
    start_skew = round(max(moves) - min(moves), 3) if len(moves) == len(watches) else None
    logger.info("Arm group start skew", submit_skew_s=submit_skew, start_skew_s=start_skew)
    metadata: dict[str, Any] = {
        name: result.metadata
        | {"send_at": since(w.send_at), "ack_at": since(w.ack_at), "moved_at": since(w.moved_at)}
        for name, result, w in zip(names, results, watches, strict=True)
    }
    metadata |= {"submit_skew_s": submit_skew, "start_skew_s": start_skew}
    return SkillResult(
        "; ".join(f"{name}: {r.message}" for name, r in zip(names, results, strict=True)),
        metadata=metadata,
    )


class ArmLauncher:
    """Background launch, tool-stream reporting and stop of arm motions, over the host's arms.

    The host provides `_arms()` and the Module tool-stream methods.
    """

    if TYPE_CHECKING:

        def _arms(self) -> Sequence[SonicArm]: ...
        def start_tool(self, name: str) -> None: ...
        def tool_update(self, name: str, message: str) -> None: ...
        def stop_tool(self, name: str) -> None: ...

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        # Serializes every change of the running motion: start, refusal, rebind and cleanup.
        self._lifecycle = threading.Lock()
        self._running: str | None = None  # The running motion's skill (and tool-stream) name.
        self._launched: Sequence[SonicArm] = ()  # The running motion's arms.
        self._motion: threading.Thread | None = None  # The running motion's thread.
        super().__init__(*args, **kwargs)

    def _retreat_before_stop(self) -> None:
        """For the host's Module.stop(): stop a running motion and wait for its retreat.

        Best-effort, so the native planner never takes over a raised arm; the coordinator may stop
        the execution workers concurrently.
        """
        with self._lifecycle:
            motion = self._motion if self._running is not None else None
            if motion is not None:
                for arm in self._launched:
                    arm.stop.set()
        if motion is not None:
            motion.join(timeout=_STOP_JOIN_S)

    def _request_stop(self) -> str:
        with self._lifecycle:
            if self._running is None:
                return "No arm motion is running."
            for arm in self._launched:
                arm.stop.set()
        return "Stopping: the arm returns smoothly to its start pose; a tool update reports the result."

    def _launch(
        self,
        tool: str,
        prepare: Callable[[], Callable[[], SkillResult]],
        arms: Sequence[SonicArm] | None = None,
    ) -> str:
        """Start a motion in the background; its result is the tool stream's final update.

        Checks for a running motion before validating, and never closes or releases what it holds:
        a same-tool retry through MCP has already taken over the capability hold, and start_tool
        rebinds the live stream to it, so the running motion's stream stop releases it.
        """
        selected = self._arms() if arms is None else arms
        with self._lifecycle:
            if self._running == tool:
                self.start_tool(tool)
                return (
                    f"{tool} is already running; its result will follow as a tool update. "
                    "Call stop_arm to stop it."
                )
            if self._running is not None:
                raise RuntimeError("an arm move is already running")
            work = prepare()
            self.start_tool(tool)
            self._running = tool
            for arm in selected:
                arm.reset()
            self._launched = selected
            self._motion = threading.Thread(
                target=self._finish, args=(tool, work, selected), name=f"arm-{tool}", daemon=True
            )
            try:
                self._motion.start()
            except BaseException:
                self.stop_tool(tool)
                self._running = None
                self._launched = ()
                raise
        return f"{tool} started; a tool update reports the result. Call stop_arm to stop early."

    def _finish(self, tool: str, work: Callable[[], SkillResult], arms: Sequence[SonicArm]) -> None:
        started = time.monotonic()
        try:
            result = work()
            elapsed_ms = (time.monotonic() - started) * 1000.0
            text = replace(result, duration_ms=elapsed_ms).agent_encode()[0]["text"]
        except Exception as exc:
            text = f"{tool} failed: {exc}"
        except BaseException as exc:
            text = f"{tool} failed: {type(exc).__name__}"
            raise  # After the cleanup below, so the motion never stays held.
        finally:
            logger.info("Arm skill result", tool=tool, result=text)
            # One critical section: a call arriving now either rebinds a stream that still has
            # this update to come, or finds nothing running and opens a fresh one.
            with self._lifecycle:
                try:
                    self.tool_update(tool, text)
                finally:
                    self.stop_tool(tool)
                    self._running = None
                    for arm in arms:
                        arm.clear_watch()


class SonicArmSkillContainer(ArmLauncher, Module):
    """Arm-reference skills over the guarded SONIC controller (no hands required)."""

    config: SonicArmSkillConfig
    feedback: In[SonicFeedback]
    color_image: In[Image]
    _execution: SonicArmExecutionSpec

    def __init__(self, **kwargs: Any) -> None:
        self._arm: SonicArm | None = None
        self._kinematics: ArmKinematics | None = None
        self._vl: VlModel | None = None
        super().__init__(**kwargs)

    def _arms(self) -> Sequence[SonicArm]:
        return [self._arm] if self._arm is not None else []

    @rpc
    def start(self) -> None:
        encoder = SonicEncoder(self.config.encoder_path)
        checker = None
        if self.config.collision_model_path is not None:
            checker = ArmCollisionChecker(self.config.collision_model_path)
            self._kinematics = ArmKinematics(self.config.collision_model_path)
        if self.config.vl_model is not None:
            self._vl = OpenAIVlModel(model_name=self.config.vl_model)
        self._arm = SonicArm("g1", self.config, self._execution, encoder, checker)
        super().start()

    @rpc
    def stop(self) -> None:
        self._retreat_before_stop()
        super().stop()

    @rpc
    async def handle_feedback(self, feedback: SonicFeedback) -> None:
        """Track the active move's envelope, reach and speeds from every telemetry sample."""
        if self._arm is not None:
            self._arm.handle_feedback(feedback)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_upper_body(self, motion: UpperBodyMotion, angle_deg: float) -> str:
        """Move one upper-body joint out by angle_deg, hold it for 1 second, move it back, then hand control back.

        Motions: "left_arm_forward" swings the left arm forward (shoulder pitch),
        "left_arm_sideways" raises the left arm out to the side (shoulder roll), and
        "left_forearm_raise" lifts the left forearm (elbow). Min-jerk, 2 s out and 2 s back (longer
        for big angles), while SONIC keeps the robot balanced; 18-24 s in all. Requires the SONIC
        controller running in confirmed planner idle; it never starts or stops the controller. Stops
        early and returns control to the planner if any joint strays from the plan. Returns at
        once; a tool update reports the result.

        Args:
            motion: One of "left_arm_forward", "left_arm_sideways", "left_forearm_raise".
            angle_deg: How far to move, in degrees: left_arm_forward 10-28, left_arm_sideways 10-40,
                left_forearm_raise 20-60.
        """

        def prepare() -> Callable[[], SkillResult]:
            move = upper_body_move(self.config, motion, angle_deg)
            return lambda: self._run_move(move)

        return self._launch("move_upper_body", prepare)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_left_arm_pose(self, pose: LeftArmPose, hold_seconds: float = 1.0) -> str:
        """Move the left arm to a full-range pose, hold it for hold_seconds, bring it back, then hand control back.

        Poses: "left_arm_forward_horizontal" (straight arm pointing forward at shoulder height),
        "left_arm_side_horizontal" (straight arm out to the side at shoulder height),
        "left_hand_over_head" (arm raised forward and up until the hand is over the head),
        "left_hand_over_head_sideways" (arm raised out to the side and up over the head). Slow
        (peak 0.47 rad/s): 20-40 s in all, overhead the longest. Requires the SONIC controller running
        in confirmed planner idle; it never starts or stops the controller. Stops early and returns
        control to the planner if the legs, waist or the other arm stray, or the arm strays far from
        the plan. Returns at once; a tool update reports the result.

        Args:
            pose: One of "left_arm_forward_horizontal", "left_arm_side_horizontal",
                "left_hand_over_head", "left_hand_over_head_sideways".
            hold_seconds: How long to hold the pose, 1-20 s.
        """

        def prepare() -> Callable[[], SkillResult]:
            move = left_arm_pose_move(self.config, pose, hold_seconds)
            return lambda: self._run_move(move)

        return self._launch("move_left_arm_pose", prepare)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def move_left_arm_joints(
        self,
        shoulder_pitch_deg: float,
        shoulder_roll_deg: float,
        shoulder_yaw_deg: float,
        elbow_deg: float,
        hold_seconds: float = 1.0,
    ) -> str:
        """Move the left arm to the given joint angles, hold for hold_seconds, bring it back, then hand control back.

        Angles are absolute, in degrees. Hanging rest is about pitch 11, roll 11, yaw 0, elbow 34.
        Shoulder pitch: negative raises the arm forward (-90 forward horizontal, -160 overhead).
        Shoulder roll: positive raises it out to the side (86 side horizontal). Shoulder yaw rotates
        the upper arm. Elbow: 82 is straight, 0 is bent 90 degrees, negative bends further. The
        planned path is checked against the robot's body first and refused if the arm would come
        near it. Slow (peak 0.47 rad/s), up to about 40 s. Prefer move_left_arm_pose for the
        named poses. Returns at once; a tool update reports the result.

        Args:
            shoulder_pitch_deg: -165 to 45.
            shoulder_roll_deg: -10 to 120.
            shoulder_yaw_deg: -90 to 90.
            elbow_deg: -30 to 82.
            hold_seconds: How long to hold the pose, 1-20 s.
        """
        values = (shoulder_pitch_deg, shoulder_roll_deg, shoulder_yaw_deg, elbow_deg)

        def prepare() -> Callable[[], SkillResult]:
            if self._arm is None or self._arm._checker is None:
                raise RuntimeError(_NO_PATH_CHECK)
            move = left_arm_joints_move(self.config, values, hold_seconds)
            return lambda: self._run_move(move)

        return self._launch("move_left_arm_joints", prepare)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def perform_gesture(self, name: GestureName) -> str:
        """Perform a left-arm gesture, then bring the arm back and hand control back.

        Gestures: "wave" (raises the hand and swings it side to side three times, about 30 s),
        "point_forward" (arm out in front, about 16 s), "stop_sign" (raised forward palm out, about 15 s),
        "raise_hand" (arm up, about 25 s) and "beckon" (forearm curls in and out twice, about 21 s).
        Slow (peak 0.47 rad/s). Requires the SONIC controller running in confirmed planner idle; it
        never starts or stops the controller. Stops early and returns control to the planner if the
        legs, waist or the other arm stray, or the arm strays far from the plan. Returns at once; a
        tool update reports the result.

        Args:
            name: One of "wave", "point_forward", "stop_sign", "raise_hand", "beckon".
        """

        def prepare() -> Callable[[], SkillResult]:
            move = gesture_move(self.config, name)
            return lambda: self._run_move(move)

        return self._launch("perform_gesture", prepare)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def point_at(self, azimuth_deg: float, elevation_deg: float, hold_seconds: float = 2.0) -> str:
        """Point the straight left arm in a direction, hold for hold_seconds, bring it back, then hand control back.

        Directions are relative to the torso: azimuth 0 = straight ahead, positive = the robot's
        left; elevation 0 = horizontal, positive = up. Requests must lie within azimuth -35 to 110
        and elevation -75 to 75 degrees. Pointing low and close to straight ahead, or far to the
        right, may be refused when the arm would pass near the body or cannot reach; the refusal
        says why, and nothing moves. The arm is aimed from the shoulder within 3 degrees. Slow
        (peak 0.47 rad/s), 10-30 s plus the hold. Returns at once; a tool update reports the result
        with the achieved direction.

        Args:
            azimuth_deg: -35 to 110; positive is to the robot's left.
            elevation_deg: -75 to 75; positive is up.
            hold_seconds: How long to hold the pointing pose, 1-20 s.
        """

        def prepare() -> Callable[[], SkillResult]:
            for name, value, (low, high) in (
                ("azimuth_deg", azimuth_deg, POINT_AZIMUTH_DEG),
                ("elevation_deg", elevation_deg, POINT_ELEVATION_DEG),
            ):
                if not math.isfinite(value) or not low <= value <= high:
                    raise ValueError(
                        f"{name} must be within {low:g}..{high:g}: outside the pointing cone"
                    )
            _check_hold(hold_seconds)
            if self._arm is None or self._arm._checker is None or self._kinematics is None:
                raise RuntimeError(_NO_ARM_MODEL)
            label = f"point_at({azimuth_deg:g}, {elevation_deg:g})"
            return lambda: self._point(label, azimuth_deg, elevation_deg, hold_seconds, {})

        return self._launch("point_at", prepare)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def point_at_object(self, description: str, hold_seconds: float = 2.0) -> str:
        """Point the left forearm at an object the head camera sees, hold for hold_seconds, bring it back, then hand control back.

        A vision model finds the description in the current head-camera image, and the forearm
        points at the floor under the largest match's centre, almost anywhere the head camera sees; low
        and to the right, near the hip, it may refuse. The head
        camera looks at the floor about 0.3-2 m ahead; objects above about waist height may be out
        of view, and objects on a table may be aimed slightly behind themselves. If nothing matches
        or the vision model fails, nothing moves. Refused, with the reason, if no natural pose
        points there or every such pose comes near the body. Returns at once; a tool update reports
        the result with the object's box and the planned and measured aim errors.

        Args:
            description: What to point at, e.g. "red box".
            hold_seconds: How long to hold the pointing pose, 1-20 s.
        """

        def prepare() -> Callable[[], SkillResult]:
            if self._vl is None:
                raise RuntimeError("point_at_object is disabled: no vl_model configured")
            if self._arm is None or self._arm._checker is None or self._kinematics is None:
                raise RuntimeError(_NO_ARM_MODEL)
            _check_hold(hold_seconds)
            if not description.strip():
                raise ValueError("description must not be empty")
            return lambda: self._point_at_object(description, hold_seconds)

        return self._launch("point_at_object", prepare)

    @skill
    def stop_arm(self) -> str:
        """Stop the running arm motion: the arm returns smoothly to where it started, then control goes back to SONIC's planner.

        The stopped motion's tool update reports how far it got.
        """
        return self._request_stop()

    def _point_at_object(self, description: str, hold_seconds: float) -> SkillResult:
        """Find the object, then aim the forearm at it. Runs on the motion thread: the vision
        call and the forearm solve can take seconds, and a stop_arm meanwhile is honoured after
        encoding."""
        assert self._vl is not None and self._kinematics is not None
        try:
            image = self.color_image.get_next(timeout=_FRAME_TIMEOUT_S)
        except Exception as exc:
            raise RuntimeError(
                f"no camera frame within {_FRAME_TIMEOUT_S:g} s; nothing sent"
            ) from exc
        if (image.width, image.height) != HEAD_CAMERA_SIZE:
            raise RuntimeError(
                f"camera frame is {image.width}x{image.height}, not the calibrated "
                f"{HEAD_CAMERA_SIZE[0]}x{HEAD_CAMERA_SIZE[1]}; nothing sent"
            )
        try:
            boxes = query_boxes(self._vl, image, description, self.config.vl_box_scale)
        except Exception as exc:
            raise RuntimeError(
                f"vision model {self.config.vl_model} failed: {exc}; nothing sent"
            ) from exc
        if not boxes:
            raise RuntimeError(f"found no '{description}' in the camera image; nothing sent")
        label, x1, y1, x2, y2 = max(boxes, key=lambda b: (b[3] - b[1]) * (b[4] - b[2]))
        extra = {"label": label, "box_px": [round(v) for v in (x1, y1, x2, y2)]}
        return self._point_forearm(
            f"point_at_object('{description}')", (x1 + x2) / 2, (y1 + y2) / 2, hold_seconds, extra
        )

    def _point_forearm(
        self, label: str, u: float, v: float, hold_seconds: float, extra: dict[str, Any]
    ) -> SkillResult:
        """point_at_object's motion: aim the forearm at pixel (u, v)'s floor point from the
        measured pose, run it, report the planned and held aim errors."""
        assert self._arm is not None
        kinematics, checker = self._kinematics, self._arm._checker
        assert kinematics is not None and checker is not None
        solved: list[tuple[NDArray[np.float64], float, NDArray[np.float64]]] = []

        def keyframes_of(base: NDArray[np.float64]) -> list[Keyframe]:
            point, distance = kinematics.aim_point(base, u, v)
            q4 = kinematics.solve_forearm(
                base,
                point,
                lambda pose: checker.first_contact(stroke_plan(base, pose - base), start=base),
            )
            goal = base.copy()
            goal[_LEFT_ARM_TARGET] = q4
            solved.append((point, distance, goal))
            return [Keyframe(goal, hold_seconds)]

        def report_extra(
            base: NDArray[np.float64], hold_q: NDArray[np.float64] | None
        ) -> dict[str, Any]:
            point, distance, goal = solved[-1]
            planned = round(kinematics.forearm_error(goal, point), 2)
            wrist_x = round(float(kinematics.forearm_points(goal)[1][0]), 3)
            measured = (
                None
                if hold_q is None
                else round(
                    float(np.median([kinematics.forearm_error(q, point) for q in hold_q])), 2
                )
            )
            return {
                "mode": "forearm",
                **extra,
                "aim_point_m": [round(float(c), 3) for c in point],
                "aim_distance_m": round(float(distance), 2),
                "pose_deg": [round(math.degrees(a), 1) for a in goal[_LEFT_ARM_TARGET]],
                "planned_aim_error_deg": planned,
                "planned_wrist_x_m": wrist_x,
                "measured_aim_error_deg": measured,
            }

        allowed = pose_margins(
            self.config.body_margin, self.config.pose_arm_margin, self.config.other_arm_margin
        )
        return self._run(
            label, keyframes_of, allowed, require_check=True, report_extra=report_extra
        )

    def _point(
        self,
        label: str,
        azimuth_deg: float,
        elevation_deg: float,
        hold_seconds: float,
        extra: dict[str, Any],
    ) -> SkillResult:
        """point_at's motion: solve the arm from the measured pose, run it, report the directions."""
        kinematics = self._kinematics
        assert kinematics is not None
        solved: list[NDArray[np.float64]] = []

        def keyframes_of(base: NDArray[np.float64]) -> list[Keyframe]:
            # Solved from the measured pose, so the waist and wrists are the robot's own.
            goal = base.copy()
            goal[_LEFT_ARM_TARGET] = kinematics.solve(base, azimuth_deg, elevation_deg)
            solved.append(goal)
            return [Keyframe(goal, hold_seconds)]

        def report_extra(
            base: NDArray[np.float64], hold_q: NDArray[np.float64] | None
        ) -> dict[str, Any]:
            return {
                **extra,
                "requested_direction_deg": [azimuth_deg, elevation_deg],
                "achieved_direction_deg": [round(v, 1) for v in kinematics.direction(solved[-1])],
            }

        allowed = pose_margins(
            self.config.body_margin, self.config.pose_arm_margin, self.config.other_arm_margin
        )
        return self._run(
            label, keyframes_of, allowed, require_check=True, report_extra=report_extra
        )

    def _run_move(self, move: ArmMove) -> SkillResult:
        return self._run(
            move.label, move.keyframes_of, move.allowed, require_check=move.require_check
        )

    def _run(
        self,
        label: str,
        keyframes_of: Callable[[NDArray[np.float64]], list[Keyframe]],
        allowed: NDArray[np.float64],
        *,
        require_check: bool,
        report_extra: ReportExtra | None = None,
    ) -> SkillResult:
        assert self._arm is not None
        return self._arm.execute(
            self._arm.prepare(
                label, keyframes_of, allowed, require_check=require_check, report_extra=report_extra
            )
        )


sonic_arm_skill_container = SonicArmSkillContainer.blueprint
