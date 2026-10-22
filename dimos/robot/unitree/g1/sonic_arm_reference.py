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

"""Joint-space arm reference -> SONIC v1.1 sonic78 tokens.

Encoder mode 0 input: ten future frames 5 ticks apart at 50 Hz, absolute joints and velocities in
IsaacLab order, identity anchor, terminal pose held with zero velocity. The baseline is the measured
body pose, so the reference holds the robot where it is. Commissioned on a G1 onboard PC (PC2) at up to
0.5 rad of left shoulder pitch; a planner-frame baseline was rejected (larger side motion in MuJoCo).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import hashlib
import itertools
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

ISAAC_FROM_MOTOR = np.array(
    [
        0,
        6,
        12,
        1,
        7,
        13,
        2,
        8,
        14,
        3,
        9,
        15,
        22,
        4,
        10,
        16,
        23,
        5,
        11,
        17,
        24,
        18,
        25,
        19,
        26,
        20,
        27,
        21,
        28,
    ]
)
LEFT_SHOULDER_PITCH = 15  # Motor order; negative moves the hand forward.
ENCODER_SHA256 = "fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae"  # SONIC v1.1
RATE = 50
PEAK_SPEED = 0.47  # rad/s: the commissioned 0.5 rad stroke over 2 s; bigger moves take longer.
ENCODER_LOOKAHEAD_ROWS = 45
_SPLIT_STILL_ROWS = 15
_SPLIT_POSE_ROWS = _SPLIT_STILL_ROWS + ENCODER_LOOKAHEAD_ROWS + 2
ARM_JOINTS = [15, 16, 18, 22, 23, 25]  # shoulder pitch and roll, elbows
LEFT_ARM = list(range(15, 22))
RIGHT_ARM = list(range(22, 29))
MOTOR_NAMES = (
    [
        f"{side}_{joint}"
        for side, joints in (
            ("left", ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")),
            ("right", ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")),
        )
        for joint in joints
    ]
    + ["waist_yaw", "waist_roll", "waist_pitch"]
    + [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ]
)
_LEFT_ARM_LINKS = ("left_shoulder_yaw", "left_elbow", "left_wrist")
# The left shoulder links next to the torso, absent hands, and the floor.
_IGNORED = ("left_shoulder_pitch", "left_shoulder_roll", "left_hand", "right_hand", "world")
# Wrists and shoulder yaws: SONIC's decoder moves these by itself, at low torque, when an arm moves.
LOOSE_JOINTS = [17, 19, 20, 21, 24, 26, 27, 28]
# move_left_arm_joints bounds, deg, inside the joint limits. Collisions inside the box are left to the
# runtime path check (ArmCollisionChecker), which the joint-target skill requires.
JOINT_BOUNDS = {
    "shoulder_pitch": (-165.0, 45.0),
    "shoulder_roll": (-10.0, 120.0),
    "shoulder_yaw": (-90.0, 90.0),
    "elbow": (-30.0, 82.0),
}
# point_at's coarse request bound in the torso frame, deg: azimuth 0 ahead, positive to the robot's
# left; elevation positive up. The reachable, body-clear region inside it is not a box: each request
# is decided by its own IK (3 deg) and path check. From idle, low and near-straight-ahead aims meet
# the torso (shoulder roll near 0), the lowest meet the hip, and far right or high-behind ones are
# out of the shoulder's reach.
POINT_AZIMUTH_DEG = (-35.0, 110.0)
POINT_ELEVATION_DEG = (-75.0, 75.0)
POINT_ELBOW = 1.35  # rad, near straight (1.43 is straight)
_POINT_TOLERANCE_DEG = 3.0
_COMPASS = np.array([(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)])
# The D435i head camera: factory colour intrinsics (librealsense, distortion all zero) at this size,
# and the URDF d435_joint on torso_link, pitched 0.83 rad down at the floor ahead.
HEAD_CAMERA_SIZE = (640, 480)
HEAD_CAMERA_K = (606.62, 607.12, 325.54, 245.35)  # fx, fy, cx, cy
HEAD_CAMERA_XYZ = (0.0576235, 0.01753, 0.41987)  # torso_link -> camera_link
HEAD_CAMERA_QUAT_XYZW = (0.0, 0.40355, 0.0, 0.91496)
# Forearm pointing: the elbow-to-wrist line, extended, passes through the pixel ray's floor point.
AIM_RANGE_M = (0.5, 4.0)  # clamp on the ray length to the aim point; a level ray uses the far end
FOREARM_PREF = np.radians([-45.0, 25.0, 0.0, 30.0])  # pitch, roll, yaw, elbow: a natural point
FOREARM_WEIGHTS = np.array([1.0, 1.0, 0.7, 0.7])
# Natural-pose acceptance, torso frame: hand ahead, forearm forward, moderate bend, modest roll,
# every arm joint off its JOINT_BOUNDS limit.
FOREARM_WRIST_MIN_X = 0.10  # m
FOREARM_MIN_X = 0.2  # x of the forearm's unit vector
ELBOW_STRAIGHT = 1.43  # rad
ELBOW_BEND_DEG = (5.0, 70.0)  # from straight
FOREARM_ROLL_MAX_DEG = 60.0
FOREARM_LIMIT_MARGIN_DEG = 5.0
# Search: a coarse grid (pitch, roll, yaw, elbow steps, deg; roll from 10 deg), seeds within 8 deg
# ranked by FOREARM_PREF distance, the first 40 refined by a compass search from 4 to 0.05 deg.
_FOREARM_GRID_STEPS_DEG = (10.0, 10.0, 15.0, 15.0)
_FOREARM_GRID_MIN_ROLL_DEG = 10.0
_FOREARM_SEED_DEG = 8.0
_FOREARM_SEEDS = 40
_FOREARM_STEP_DEG = (4.0, 0.05)
_COMPASS_4D = np.vstack([np.eye(4), -np.eye(4)])


class SonicEncoder:
    """Hash-checked SONIC v1.1 encoder on one CPU thread (about 1.5 s per 350 rows on the G1 PC2)."""

    def __init__(self, path: Path) -> None:
        import onnxruntime as ort

        if hashlib.sha256(path.read_bytes()).hexdigest() != ENCODER_SHA256:
            raise ValueError(f"{path} is not the SONIC v1.1 encoder")
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])

    def encode(
        self, body_motor: NDArray[Any], anchor: NDArray[Any] | None = None
    ) -> NDArray[np.float64]:
        """[N,29] motor-order poses at 50 Hz, constant root anchor [6] (default identity) -> [N,64] tokens."""
        body = np.asarray(body_motor, np.float64)[:, ISAAC_FROM_MOTOR]
        vel = np.gradient(body, 1 / RATE, axis=0) if len(body) > 1 else np.zeros_like(body)
        vel[-1] = 0
        body = np.vstack([body, np.repeat(body[-1:], 46, 0)])
        vel = np.vstack([vel, np.zeros((46, 29))])
        rows = np.arange(len(body_motor))[:, None] + np.arange(10)[None] * 5
        x = np.zeros((len(body_motor), 1751), np.float32)
        x[:, 4:294] = body[rows].reshape(-1, 290)
        x[:, 294:584] = vel[rows].reshape(-1, 290)
        x[:, 584:644] = np.tile([1, 0, 0, 1, 0, 0] if anchor is None else anchor, 10)
        tokens = [self._session.run(None, {"obs_dict": row[None]})[0] for row in x]
        return np.asarray(np.concatenate(tokens), np.float64)


def min_jerk(n: int) -> NDArray[np.float64]:
    s = np.linspace(0, 1, n)
    return np.asarray(10 * s**3 - 15 * s**4 + 6 * s**5, np.float64)


def stroke_plan(
    base: NDArray[Any],
    delta: NDArray[Any],
    *,
    lead_s: float = 1.0,
    move_s: float = 2.0,
    hold_s: float = 1.0,
    peak_speed: float = PEAK_SPEED,
) -> NDArray[np.float64]:
    """Base pose, lead-in, move every joint by delta[29] together, hold, move back, lead-out.

    [N,29] at 50 Hz. Each stroke lasts at least move_s and long enough that the largest joint's
    min-jerk peak speed (1.875 * delta / T) stays at or below peak_speed.
    """
    base = np.asarray(base, np.float64)
    return keyframe_plan(
        base,
        [Keyframe(base + np.asarray(delta, np.float64), hold_s)],
        lead_s=lead_s,
        move_s=move_s,
        peak_speed=peak_speed,
    ).poses


@dataclass(frozen=True)
class Keyframe:
    target: NDArray[np.float64]
    hold_s: float = 0.0


@dataclass(frozen=True)
class Segment:
    start: int
    end: int
    src: int
    dst: int


@dataclass(frozen=True)
class KeyframePlan:
    base: NDArray[np.float64]
    targets: NDArray[np.float64]
    poses: NDArray[np.float64]
    segments: list[Segment]
    still: NDArray[np.bool_]
    move_s: float = 2.0
    peak_speed: float = PEAK_SPEED


def _segment_poses(
    src: NDArray[np.float64], dst: NDArray[np.float64], move_s: float, peak_speed: float
) -> NDArray[np.float64]:
    duration = max(move_s, 1.875 * float(np.abs(dst - src).max()) / peak_speed)
    rows = math.ceil(duration * RATE) + 1
    # Use the same arithmetic in either direction so a return exactly mirrors the outward path.
    reverse = tuple(src) > tuple(dst)
    a, b = (dst, src) if reverse else (src, dst)
    poses = a + min_jerk(rows)[:, None] * (b - a)
    poses[0], poses[-1] = a, b
    return poses[::-1] if reverse else poses


def keyframe_plan(
    base: NDArray[Any],
    keyframes: Sequence[Keyframe],
    *,
    lead_s: float = 1.0,
    move_s: float = 2.0,
    peak_speed: float = PEAK_SPEED,
    split_every: int | None = None,
) -> KeyframePlan:
    """Min-jerk moves through absolute targets and back, with holds long enough for token splits."""
    base = np.asarray(base, np.float64)
    targets = np.asarray([keyframe.target for keyframe in keyframes], np.float64).reshape(-1, 29)
    lead = np.tile(base, (round(lead_s * RATE), 1))
    parts = [lead]
    segments: list[Segment] = []
    row = len(lead)
    last_still = max(0, row - _SPLIT_POSE_ROWS + 1)
    src = base
    for k, dst in enumerate([*targets, base]):
        move = _segment_poses(src, dst, move_s, peak_speed)
        if split_every is not None and len(move) > split_every:
            raise ValueError("segment longer than a session chunk")
        if k:
            hold = round(keyframes[k - 1].hold_s * RATE)
            # Arrival and departure also repeat the target; the window needs both neighbours.
            if hold + 2 >= _SPLIT_POSE_ROWS:
                last_still = row + hold - _SPLIT_POSE_ROWS + 2
            tail = len(lead) if k == len(targets) else 0  # the lead-out stays in the last chunk
            if split_every is not None and row + hold + len(move) + tail - last_still > split_every:
                hold = max(hold, _SPLIT_POSE_ROWS)
                last_still = row + hold - _SPLIT_POSE_ROWS + 2
            parts.append(np.tile(src, (hold, 1)))
            row += hold
        parts.append(move)
        segments.append(Segment(row, row + len(move), k - 1, k if k < len(targets) else -1))
        row += len(move)
        src = dst
    parts.append(lead)
    poses = np.vstack(parts)
    still = np.zeros(len(poses), dtype=np.bool_)
    if len(poses) >= _SPLIT_POSE_ROWS:
        unchanged = np.all(poses[1:] == poses[:-1], axis=1)
        windows = np.lib.stride_tricks.sliding_window_view(unchanged, _SPLIT_POSE_ROWS - 1)
        still[1 : len(windows) + 1] = windows.all(axis=1)
    return KeyframePlan(base, targets, poses, segments, still, move_s, peak_speed)


def retreat_poses(plan: KeyframePlan, segment: int) -> NDArray[np.float64]:
    """Reverse the active outward segment, go directly home, and keep the plan's lead-out."""
    seg = plan.segments[segment]
    if seg.dst == -1:
        return plan.poses[seg.start :]
    parts = [plan.poses[seg.start : seg.end][::-1]]
    if seg.src >= 0:
        parts.append(_segment_poses(plan.targets[seg.src], plan.base, plan.move_s, plan.peak_speed))
    parts.append(plan.poses[plan.segments[-1].end :])
    return np.vstack(parts)


def retreat_route(plan: KeyframePlan, row: int) -> tuple[int, int] | None:
    """Retreat unit and offset for a pose row; None when the plan is already returning home."""
    if row >= plan.segments[-1].start:
        return None
    for s, seg in enumerate(plan.segments):
        if row < seg.start:
            return s, 0 if seg.dst == -1 else seg.end - seg.start
        if row < seg.end:
            return s, seg.end - 1 - row
    return None


def arm_poses(
    base: NDArray[Any],
    joint: int,
    delta: float,
    *,
    lead_s: float = 1.0,
    move_s: float = 2.0,
    hold_s: float = 1.0,
    peak_speed: float = PEAK_SPEED,
) -> NDArray[np.float64]:
    """stroke_plan for one joint moved by delta."""
    vector = np.zeros(29)
    vector[joint] = delta
    return stroke_plan(
        base, vector, lead_s=lead_s, move_s=move_s, hold_s=hold_s, peak_speed=peak_speed
    )


def blend(a: NDArray[Any], b: NDArray[Any], seconds: float) -> NDArray[np.float64]:
    w = np.linspace(0, 1, round(seconds * RATE) + 1)[1:, None]
    return np.asarray((1 - w) * a + w * b, np.float64)


def token_schedule(
    captured: NDArray[Any], motion_tokens: NDArray[Any], blend_s: float = 2.0
) -> NDArray[np.float64]:
    """Planner token -> blend -> motion tokens -> blend -> planner token, at 50 Hz.

    The 2 s blends follow the lerobot finding that an instant switch from the planner token
    unloaded a foot.
    """
    captured = np.asarray(captured, np.float64)
    return np.vstack(
        [
            blend(captured, motion_tokens[0], blend_s),
            motion_tokens,
            blend(motion_tokens[-1], captured, blend_s),
        ]
    )


def group_margins(body: float, arm: float, wrist: float) -> NDArray[np.float64]:
    """Per-joint envelope, motor order: legs and waist `body`, shoulder pitch/roll and elbows `arm`,
    wrists and shoulder yaws `wrist`.

    On a G1, SONIC's decoder moved the wrists 0.05-0.16 rad and the shoulder yaws up to 0.20 rad by
    itself (about 0.2-1.4 Nm) during left-arm moves.
    """
    margins = np.full(29, body)
    margins[ARM_JOINTS] = arm
    margins[LOOSE_JOINTS] = wrist
    return margins


def pose_margins(body: float, moving_arm: float, other_arm: float) -> NDArray[np.float64]:
    """Envelope for full-range left-arm poses: legs and waist `body`, every left-arm joint
    `moving_arm` (SONIC coordinates the whole arm: in MuJoCo it added -0.31 rad of shoulder pitch to
    a side raise), every right-arm joint `other_arm`."""
    margins = np.full(29, body)
    margins[LEFT_ARM] = moving_arm
    margins[RIGHT_ARM] = other_arm
    return margins


def pixel_ray(u: float, v: float) -> NDArray[np.float64]:
    """Unit ray through head-camera pixel (u, v), torso_link frame."""
    fx, fy, cx, cy = HEAD_CAMERA_K
    x, y = (u - cx) / fx, (v - cy) / fy
    link = np.array([1.0, -x, -y])  # optical (x right, y down, z ahead) -> camera_link
    q, w = np.array(HEAD_CAMERA_QUAT_XYZW[:3]), HEAD_CAMERA_QUAT_XYZW[3]
    t = 2.0 * np.cross(q, link)
    ray = link + w * t + np.cross(q, t)  # rotated into torso_link
    return np.asarray(ray / np.linalg.norm(ray), np.float64)


class ArmCollisionChecker:
    """Kinematic check that a planned left-arm path keeps clear of the body.

    Uses the G1 SONIC MuJoCo model (gear_sonic scene_43dof.xml). Every geom gets `margin` of extra
    reach and MuJoCo adds the two, so a contact means the left upper arm, elbow or wrist links come
    within 2 * margin of the torso, pelvis, legs or right arm. The Dex3 hand geoms are ignored (no
    hands are installed). Pairs already that close at the start pose (the robot's idle left wrist
    rests about 2 cm from the hip) are allowed unless the path brings them `closer` nearer.
    Not thread-safe: every call reuses the instance's one MjData; the skill uses it only from its
    single motion thread.
    """

    def __init__(self, scene: Path, margin: float = 0.01, closer: float = 0.005) -> None:
        import mujoco

        self._mujoco = mujoco
        self._m = mujoco.MjModel.from_xml_path(str(scene))
        self._m.geom_margin[:] = margin
        self._d = mujoco.MjData(self._m)
        self._closer = closer
        body = [self._m.body(self._m.geom_bodyid[g]).name for g in range(self._m.ngeom)]
        self._body = body
        self._arm = {g for g in range(self._m.ngeom) if body[g].startswith(_LEFT_ARM_LINKS)}
        self._qadr = [self._m.jnt_qposadr[self._m.joint(n + "_joint").id] for n in MOTOR_NAMES]

    def _near(self, q: NDArray[Any]) -> dict[tuple[int, int], float]:
        """Arm-to-body geom pairs within the margin at pose q, with their distances."""
        self._mujoco.mj_resetData(self._m, self._d)  # type: ignore[attr-defined]
        self._d.qpos[2] = 0.79
        self._d.qpos[self._qadr] = q
        self._mujoco.mj_forward(self._m, self._d)
        near: dict[tuple[int, int], float] = {}
        for c in self._d.contact[: self._d.ncon]:
            a, b = int(c.geom1), int(c.geom2)
            if (a in self._arm) != (b in self._arm):
                arm, other = (a, b) if a in self._arm else (b, a)
                if not self._body[other].startswith(_IGNORED):
                    near[(arm, other)] = min(float(c.dist), near.get((arm, other), np.inf))
        return near

    def first_contact(
        self, poses: NDArray[Any], every: int = 5, start: NDArray[Any] | None = None
    ) -> str | None:
        """First body near the arm, or None. Tolerate contacts from start (default poses[0])."""
        poses = np.asarray(poses)
        baseline = self._near(poses[0] if start is None else start)
        for q in poses[::every]:
            for (arm, other), dist in self._near(q).items():
                if (arm, other) not in baseline or dist < baseline[(arm, other)] - self._closer:
                    return str(self._body[other])
        return None


def _refine_forearm(
    kin: ArmKinematics, base: NDArray[np.float64], seed: NDArray[Any], point: NDArray[Any]
) -> tuple[NDArray[np.float64], float]:
    """Compass search from seed [4] to the smallest forearm error at point: (q4, error deg)."""
    pose = base.copy()

    def cost(q4: NDArray[np.float64]) -> float:
        pose[15:19] = q4
        return kin.forearm_error(pose, point)

    low, high = kin.arm_bounds[:, 0], kin.arm_bounds[:, 1]
    best = np.asarray(seed, np.float64)
    best_cost, step = cost(best), math.radians(_FOREARM_STEP_DEG[0])
    while step > math.radians(_FOREARM_STEP_DEG[1]):
        moves = [np.clip(best + step * d, low, high) for d in _COMPASS_4D]
        costs = [cost(x) for x in moves]
        k = int(np.argmin(costs))
        if costs[k] < best_cost:
            best, best_cost = moves[k], costs[k]
        else:
            step /= 2
    return best, best_cost


class ArmKinematics:
    """Left-arm forward kinematics on scene_43dof.xml, and pointing IK for a near-straight arm
    (`solve`) or a bent arm aiming its forearm (`solve_forearm`).

    Directions are in the torso_link frame, from the left_shoulder_pitch_link origin to the
    left_wrist_yaw_link origin; the forearm runs from the left_elbow_link origin to the wrist.
    Same model and qpos layout as ArmCollisionChecker.
    Not thread-safe: every call reuses the instance's one MjData; the skill uses it only from its
    single motion thread.
    """

    def __init__(self, scene: Path) -> None:
        import mujoco

        self._mujoco = mujoco
        self._m = mujoco.MjModel.from_xml_path(str(scene))
        self._d = mujoco.MjData(self._m)
        self._d.qpos[2] = 0.79
        self._qadr = [self._m.jnt_qposadr[self._m.joint(n + "_joint").id] for n in MOTOR_NAMES]
        self._torso, self._shoulder, self._elbow, self._wrist = (
            self._m.body(name).id
            for name in (
                "torso_link",
                "left_shoulder_pitch_link",
                "left_elbow_link",
                "left_wrist_yaw_link",
            )
        )
        self._ankles = [self._m.body(f"{s}_ankle_roll_link").id for s in ("left", "right")]
        self._bounds = np.radians([JOINT_BOUNDS["shoulder_pitch"], JOINT_BOUNDS["shoulder_roll"]])
        self.arm_bounds = np.radians(
            [JOINT_BOUNDS[n] for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]
        )
        self.sole_offset = self._sole_offset()
        low, high = self.arm_bounds.T.copy()
        low[1] = math.radians(_FOREARM_GRID_MIN_ROLL_DEG)  # the grid only; refinement may go lower
        steps = np.radians(_FOREARM_GRID_STEPS_DEG)
        self._forearm_grid = np.array(
            list(itertools.product(*map(np.arange, low, high + 1e-9, steps)))
        )

    def _sole_offset(self) -> float:
        """Height of an ankle-roll origin above the lowest point of its foot's collision geometry,
        at the zero pose (the geoms' bounding boxes; exact for the G1's box soles)."""
        d = self._mujoco.MjData(self._m)
        self._mujoco.mj_kinematics(self._m, d)  # type: ignore[attr-defined]
        offset = 0.0
        for ankle in self._ankles:
            for g in np.flatnonzero(self._m.geom_bodyid == ankle):
                if self._m.geom_contype[g] or self._m.geom_conaffinity[g]:
                    rot = d.geom_xmat[g].reshape(3, 3)
                    center, half = self._m.geom_aabb[g, :3], self._m.geom_aabb[g, 3:]
                    lowest = (d.geom_xpos[g] + rot @ center)[2] - np.abs(rot[2]) @ half
                    offset = max(offset, float(d.xpos[ankle][2] - lowest))
        if offset == 0.0:
            raise ValueError("no foot collision geometry below the ankle-roll links")
        return offset

    def _frame(self, q: NDArray[Any]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Shoulder origin and shoulder-to-wrist vector at pose q, torso frame."""
        self._d.qpos[self._qadr] = q
        self._mujoco.mj_kinematics(self._m, self._d)  # type: ignore[attr-defined]
        to_torso = self._d.xmat[self._torso].reshape(3, 3).T
        shoulder = self._d.xpos[self._shoulder]
        return (
            to_torso @ (shoulder - self._d.xpos[self._torso]),
            to_torso @ (self._d.xpos[self._wrist] - shoulder),
        )

    def direction(self, q: NDArray[Any]) -> tuple[float, float]:
        """(azimuth, elevation) of the arm at pose q, deg."""
        x, y, z = self._frame(q)[1]
        return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(z, math.hypot(x, y)))

    def solve(
        self, q: NDArray[Any], azimuth_deg: float, elevation_deg: float
    ) -> NDArray[np.float64]:
        """Shoulder pitch and roll that point the arm (elbow POINT_ELBOW, yaw 0) at the direction.

        A 5 deg grid over JOINT_BOUNDS, then a compass search down to 0.01 deg. The rest of q (waist,
        wrists) stays as given. [4]: pitch, roll, yaw, elbow, rad.
        """
        az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
        want = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        pose = np.asarray(q, np.float64).copy()
        pose[17:19] = 0.0, POINT_ELBOW
        low, high = self._bounds[:, 0], self._bounds[:, 1]

        def cost(pitch_roll: NDArray[np.float64]) -> float:
            pose[15:17] = pitch_roll
            arm = self._frame(pose)[1]
            return -float(arm @ want) / float(np.linalg.norm(arm))

        axes = [np.arange(lo, hi + 1e-9, math.radians(5)) for lo, hi in self._bounds]
        grid = np.stack(np.meshgrid(*axes), -1).reshape(-1, 2)
        best = grid[int(np.argmin([cost(x) for x in grid]))]
        best_cost, step = cost(best), math.radians(2.5)
        while step > math.radians(0.01):
            moves = [np.clip(best + step * d, low, high) for d in _COMPASS]
            costs = [cost(x) for x in moves]
            k = int(np.argmin(costs))
            if costs[k] < best_cost:
                best, best_cost = moves[k], costs[k]
            else:
                step /= 2
        off = math.degrees(math.acos(min(1.0, -best_cost)))
        if off > _POINT_TOLERANCE_DEG:
            raise ValueError(
                f"the left arm cannot point at azimuth {azimuth_deg:g}, elevation "
                f"{elevation_deg:g} deg: its closest pose is {off:.0f} deg away, more than 3 deg off"
            )
        return np.array([best[0], best[1], 0.0, POINT_ELBOW])

    def forearm_points(self, q: NDArray[Any]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Elbow and wrist origins at pose q, torso frame."""
        self._d.qpos[self._qadr] = q
        self._mujoco.mj_kinematics(self._m, self._d)  # type: ignore[attr-defined]
        to_torso = self._d.xmat[self._torso].reshape(3, 3).T
        origin = self._d.xpos[self._torso]
        return (
            to_torso @ (self._d.xpos[self._elbow] - origin),
            to_torso @ (self._d.xpos[self._wrist] - origin),
        )

    def aim_point(self, q: NDArray[Any], u: float, v: float) -> tuple[NDArray[np.float64], float]:
        """Where pixel (u, v)'s ray meets the floor at measured pose q: (point in the torso frame,
        ray length from the camera in m), the length clamped to AIM_RANGE_M.

        The floor is level in the model's world (pelvis upright), at the lower ankle-roll origin
        minus the sole offset.
        """
        self._d.qpos[self._qadr] = q
        self._mujoco.mj_kinematics(self._m, self._d)  # type: ignore[attr-defined]
        up = self._d.xmat[self._torso].reshape(3, 3)[2]  # world z row: torso frame -> world height
        floor = min(self._d.xpos[a][2] for a in self._ankles) - self.sole_offset
        camera = np.array(HEAD_CAMERA_XYZ)
        ray = pixel_ray(u, v)
        height = self._d.xpos[self._torso][2] + up @ camera - floor
        drop = -float(up @ ray)
        length = height / drop if drop > 1e-6 else AIM_RANGE_M[1]
        length = min(max(length, AIM_RANGE_M[0]), AIM_RANGE_M[1])
        return camera + length * ray, length

    def forearm_error(self, q: NDArray[Any], point: NDArray[Any]) -> float:
        """Angle between the forearm (elbow to wrist) and the wrist-to-point line at pose q, deg."""
        elbow, wrist = self.forearm_points(q)
        forearm, to_point = wrist - elbow, np.asarray(point) - wrist
        cos = forearm @ to_point / (np.linalg.norm(forearm) * np.linalg.norm(to_point))
        return math.degrees(math.acos(min(1.0, max(-1.0, float(cos)))))

    def natural_pose(self, q: NDArray[Any]) -> str | None:
        """None when pose q reads as pointing, else the name of the first failed check."""
        elbow, wrist = self.forearm_points(q)
        q4 = np.asarray(q, np.float64)[15:19]
        bend = math.degrees(ELBOW_STRAIGHT - q4[3])
        margin = math.radians(FOREARM_LIMIT_MARGIN_DEG)
        if wrist[0] < FOREARM_WRIST_MIN_X:
            return "wrist_behind"
        if (wrist - elbow)[0] / np.linalg.norm(wrist - elbow) < FOREARM_MIN_X:
            return "forearm_backward"
        if not ELBOW_BEND_DEG[0] <= bend <= ELBOW_BEND_DEG[1]:
            return "elbow_bend"
        if math.degrees(q4[1]) > FOREARM_ROLL_MAX_DEG:
            return "shoulder_roll"
        if np.any(q4 < self.arm_bounds[:, 0] + margin) or np.any(
            q4 > self.arm_bounds[:, 1] - margin
        ):
            return "joint_limit"
        return None

    def solve_forearm(
        self,
        q: NDArray[Any],
        point: NDArray[Any],
        path_clear: Callable[[NDArray[np.float64]], str | None],
    ) -> NDArray[np.float64]:
        """Shoulder pitch, roll, yaw and elbow [4] that aim the forearm at point (torso frame)
        within 3 deg, in a natural pose whose path path_clear(full pose) passes.

        The rest of q stays as given. Coarse grid -> the 40 seeds within 8 deg nearest
        FOREARM_PREF -> refined -> kept when within 3 deg and natural -> re-ranked by FOREARM_PREF
        distance -> path-checked in that order; the first clear one wins. The best posture among
        the searched candidates, not a global optimum.
        """
        base = np.asarray(q, np.float64).copy()
        point = np.asarray(point, np.float64)
        where = "({:.2f}, {:.2f}, {:.2f}) m".format(*point)
        # Grid FK, recomputed per call since it depends on the rest of q (about 0.3 s for 27k poses).
        self.forearm_points(base)
        to_torso = self._d.xmat[self._torso].reshape(3, 3).T.copy()
        origin = self._d.xpos[self._torso].copy()
        arm = self._qadr[15:19]
        elbows = np.empty((len(self._forearm_grid), 3))
        wrists = np.empty_like(elbows)
        for i, q4 in enumerate(self._forearm_grid):
            self._d.qpos[arm] = q4
            self._mujoco.mj_kinematics(self._m, self._d)  # type: ignore[attr-defined]
            elbows[i], wrists[i] = self._d.xpos[self._elbow], self._d.xpos[self._wrist]
        elbows, wrists = (elbows - origin) @ to_torso.T, (wrists - origin) @ to_torso.T
        forearm = wrists - elbows
        to_point = point - wrists
        cos = np.sum(forearm * to_point, axis=1) / (
            np.linalg.norm(forearm, axis=1) * np.linalg.norm(to_point, axis=1)
        )
        error = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
        seeds = self._forearm_grid[error < _FOREARM_SEED_DEG]
        seeds = seeds[np.argsort(_pref_distance(seeds), kind="stable")][:_FOREARM_SEEDS]
        close = False
        accepted: list[tuple[float, NDArray[np.float64]]] = []
        for seed in seeds:
            q4, off = _refine_forearm(self, base, seed, point)
            if off > _POINT_TOLERANCE_DEG:
                continue
            close = True
            pose = base.copy()
            pose[15:19] = q4
            if self.natural_pose(pose) is None:
                accepted.append((float(_pref_distance(q4)), pose))
        if not close:
            raise ValueError(f"the left forearm cannot point within 3 deg at {where}")
        if not accepted:
            raise ValueError(
                f"the left forearm cannot aim at {where}: no natural pose points there"
            )
        accepted.sort(key=lambda c: c[0])
        blocked = None
        for _, pose in accepted:
            body = path_clear(pose)
            if body is None:
                return pose[15:19].copy()
            blocked = blocked or body
        raise ValueError(
            f"the left forearm cannot aim at {where}: every pose that points there comes near "
            f"the {blocked}"
        )


def _pref_distance(q4: NDArray[Any]) -> NDArray[np.float64]:
    """Weighted distance of [..., 4] arm poses from FOREARM_PREF, rad."""
    return np.asarray(
        np.linalg.norm((np.asarray(q4) - FOREARM_PREF) * FOREARM_WEIGHTS, axis=-1), np.float64
    )
