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

"""G1 gesture motions imported from the physical_ai_agent repository (see gestures/SOURCE.md).

Each ``gestures/<name>.json`` is a copy of their motion data: ``name``, ``return_s`` and
``keyframes``, each keyframe with ``t`` (s from start, ascending from 0), ``delta`` (rad added to the measured start),
``absolute`` (rad; wins over ``delta`` for the same joint) and ``hands`` (Dex3 absolute finger
commands per side; unnamed fingers take the deploy fist). Joints and fingers are mapped by NAME:
joints to ``MOTOR_NAMES`` indices, fingers to ``HAND_ORDER`` slots (left 0-6, right 7-13).
A keyframe with no ``hands`` keeps the previous keyframe's hand targets (the source player sends
the deploy fist instead); no retained motion depends on this. Unknown keys are errors.
``load_gestures`` and ``resolve`` hold no state.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sonic_arm_reference import HAND_JOINTS, MOTOR_NAMES, Keyframe

DATA_DIR = Path(__file__).parent / "gestures"

RETAINED: tuple[str, ...] = (
    "wave_one_hand",
    "v_sign",
    "good",
    "support_person",
    "two_hand_hearts",
)

# (lower, upper) rad: the G1 URDF joint limits, copied from physical_ai_agent upper_body.LIMITS.
UB_LIMITS: dict[str, tuple[float, float]] = {
    "waist_yaw": (-2.618, 2.618),
    "waist_roll": (-0.52, 0.52),
    "waist_pitch": (-0.52, 0.52),
    "left_shoulder_pitch": (-3.0892, 2.6704),
    "right_shoulder_pitch": (-3.0892, 2.6704),
    "left_shoulder_roll": (-1.5882, 2.2515),
    "right_shoulder_roll": (-2.2515, 1.5882),
    "left_shoulder_yaw": (-2.618, 2.618),
    "right_shoulder_yaw": (-2.618, 2.618),
    "left_elbow": (-1.0472, 2.0944),
    "right_elbow": (-1.0472, 2.0944),
    "left_wrist_roll": (-1.972222054, 1.972222054),
    "right_wrist_roll": (-1.972222054, 1.972222054),
    "left_wrist_pitch": (-1.614429558, 1.614429558),
    "right_wrist_pitch": (-1.614429558, 1.614429558),
    "left_wrist_yaw": (-1.614429558, 1.614429558),
    "right_wrist_yaw": (-1.614429558, 1.614429558),
}
_WAIST = ("waist_yaw", "waist_roll", "waist_pitch")

# Dex3 command order, the same on both hands (physical_ai_agent hands.ORDER, right hand confirmed
# on hardware). Our stream's hand slots use it too (right_hand_order="native").
HAND_ORDER = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
# (lower, upper) rad in command order: the Dex3 deploy command limits (MIN/MAX_LIMITS_{LEFT,RIGHT}),
# copied from physical_ai_agent hands.LIMITS.
HAND_LIMITS: dict[str, tuple[tuple[float, float], ...]] = {
    "left": (
        (-1.05, 1.05),
        (-0.724, 1.05),
        (0.0, 1.75),
        (-1.57, 0.0),
        (-1.75, 0.0),
        (-1.57, 0.0),
        (-1.75, 0.0),
    ),
    "right": (
        (-1.05, 1.05),
        (-1.05, 0.742),
        (-1.75, 0.0),
        (0.0, 1.57),
        (0.0, 1.75),
        (0.0, 1.57),
        (0.0, 1.75),
    ),
}
# The deploy's default hand (a fist), command order, copied from physical_ai_agent hands.DEFAULT.
HAND_DEFAULT: dict[str, tuple[float, ...]] = {
    "left": (0.0, 0.0, 1.75, -1.57, -1.75, -1.57, -1.75),
    "right": (0.0, 0.0, -1.75, 1.57, 1.75, 1.57, 1.75),
}
_SIDES = ("left", "right")


@dataclass(frozen=True)
class ImportedKeyframe:
    t: float
    delta: dict[int, float]  # motor index -> rad added to the start pose
    absolute: dict[int, float]  # motor index -> rad
    hands: (
        NDArray[np.float64] | None
    )  # [14], left then right in HAND_ORDER; None keeps the previous


@dataclass(frozen=True)
class ImportedGesture:
    name: str
    keyframes: tuple[ImportedKeyframe, ...]
    return_s: float


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where} = {value!r} is not a finite number")
    return float(value)


def _keys(obj: Any, allowed: tuple[str, ...], where: str) -> dict[str, Any]:
    if not isinstance(obj, dict):
        raise ValueError(f"{where} must be an object")
    unknown = sorted(set(obj) - set(allowed))
    if unknown:
        raise ValueError(f"{where}: unknown keys {unknown}; allowed {list(allowed)}")
    return obj


def _check_limit(name: str, value: float, limit: tuple[float, float], where: str) -> None:
    lo, hi = limit
    if not lo <= value <= hi:
        raise ValueError(f"{where}: {name} = {value:.6g} outside its limit [{lo}, {hi}]")


def _joints(raw: Any, absolute: bool, where: str) -> dict[int, float]:
    out: dict[int, float] = {}
    for name, v in _keys(raw, tuple(UB_LIMITS), where).items():
        value = _number(v, f"{where}.{name}")
        if name in _WAIST and (absolute or value != 0.0):
            raise ValueError(f"{where}: {name} moves the waist; waist entries must be zero deltas")
        if absolute:
            _check_limit(name, value, UB_LIMITS[name], where)
        out[MOTOR_NAMES.index(name)] = value
    return out


def _hands(raw: Any, where: str) -> NDArray[np.float64]:
    hands = np.array([*HAND_DEFAULT["left"], *HAND_DEFAULT["right"]], np.float64)
    for side, fingers in _keys(raw, _SIDES, where).items():
        offset = 7 * _SIDES.index(side)
        for name, v in _keys(fingers, HAND_ORDER, f"{where}.{side}").items():
            value = _number(v, f"{where}.{side}.{name}")
            slot = HAND_ORDER.index(name)
            _check_limit(f"{side} {name}", value, HAND_LIMITS[side][slot], where)
            hands[offset + slot] = value
    return hands


def parse_gesture(obj: Any, source: str = "") -> ImportedGesture:
    """Validate one gesture object: names, waist, absolute and hand limits, ascending ``t``."""
    g = _keys(obj, ("name", "return_s", "keyframes"), source or "gesture")
    name = g.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError(f"{source}: missing name")
    frames = g.get("keyframes")
    if not isinstance(frames, list) or len(frames) < 2:
        raise ValueError(
            f"{name}: keyframes must be a list of the start pose and at least one more"
        )
    keyframes: list[ImportedKeyframe] = []
    for i, raw in enumerate(frames):
        where = f"{name} keyframes[{i}]"
        frame = _keys(raw, ("t", "delta", "absolute", "hands"), where)
        t = _number(frame.get("t"), f"{where}.t")
        if (i == 0 and t != 0.0) or (i > 0 and t <= keyframes[-1].t):
            raise ValueError(f"{where}: t = {t} must start at 0 and ascend")
        hands = _hands(frame["hands"], f"{where}.hands") if "hands" in frame else None
        delta = _joints(frame.get("delta", {}), False, f"{where}.delta")
        absolute = _joints(frame.get("absolute", {}), True, f"{where}.absolute")
        if i == 0 and (absolute or hands is not None or any(delta.values())):
            raise ValueError(
                f"{where}: the t=0 keyframe is the start pose; only zero deltas allowed"
            )
        keyframes.append(ImportedKeyframe(t, delta, absolute, hands))
    return_s = _number(g.get("return_s"), f"{name}.return_s")
    if return_s < 0.0:
        raise ValueError(f"{name}: return_s = {return_s} must be >= 0")
    return ImportedGesture(name, tuple(keyframes), return_s)


def load_gestures() -> dict[str, ImportedGesture]:
    """The RETAINED gestures, read from the JSON files next to this module."""
    out = {}
    for name in RETAINED:
        gesture = parse_gesture(json.loads((DATA_DIR / f"{name}.json").read_text()), name)
        if gesture.name != name:
            raise ValueError(f"{name}.json names itself {gesture.name!r}")
        out[name] = gesture
    return out


def resolve(g: ImportedGesture, base: NDArray[Any], base_hands: NDArray[Any]) -> list[Keyframe]:
    """One Keyframe per imported keyframe after the t=0 start pose, against the measured base.

    Joint target: ``absolute`` if given, else ``base + delta``; unnamed joints stay at base. Hands:
    the keyframe's, else the previous ones (starting from ``base_hands``). ``min_s`` is the gap
    from the previous keyframe's ``t`` (the first gap is from 0). Raises ValueError if a joint the
    keyframe sets resolves outside UB_LIMITS; joints left at base are already held, not re-checked.
    """
    base = np.asarray(base, np.float64).reshape(29)
    hands = np.asarray(base_hands, np.float64).reshape(HAND_JOINTS)
    out: list[Keyframe] = []
    t_prev = 0.0
    for i, k in enumerate(g.keyframes[1:], start=1):
        target = base.copy()
        for idx, d in k.delta.items():
            target[idx] += d
        for idx, a in k.absolute.items():
            target[idx] = a
        for idx in k.delta.keys() | k.absolute.keys():
            name = MOTOR_NAMES[idx]
            _check_limit(name, float(target[idx]), UB_LIMITS[name], f"{g.name} keyframes[{i}]")
        if k.hands is not None:
            hands = k.hands
        out.append(Keyframe(target, hands=hands.copy(), min_s=k.t - t_prev))
        t_prev = k.t
    return out
