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

"""Offline tool behind ``gestures/clips/``: ARDY qpos CSV -> 50 Hz arm clip CSV (see gestures/SOURCE.md).

Usage: python -m dimos.robot.unitree.g1.tool_ardy_clip IN.csv SPEED OUT.csv
"""

from __future__ import annotations

import sys

import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import CubicSpline

from dimos.robot.unitree.g1.imported_gestures import CLIP_COLUMNS
from dimos.robot.unitree.g1.sonic_arm_reference import RATE

STILL_DQ = 0.05  # rad/s: an arm slower than this counts as still
SETTLE_S = 0.5  # still lead-in and tail kept around the motion
EASE_S = 0.4  # playback speed ramps 0 -> speed (and back) over this much output time at each end


def convert(qpos: NDArray[np.float64], speed: float, src_fps: float = 25.0) -> NDArray[np.float64]:
    """``[M, 14]`` arm rows (motors 15-28) at RATE Hz, played at ``speed`` x, trimmed to the motion."""
    arms = qpos[:, 7 + 15 : 7 + 29]
    t = np.arange(len(arms)) / src_fps
    spline = CubicSpline(t, arms, axis=0)
    out_t = np.arange(0.0, t[-1] / speed, 1 / RATE) * speed
    moving = np.flatnonzero(np.abs(np.diff(spline(out_t), axis=0)).max(axis=1) * RATE > STILL_DQ)
    if len(moving) == 0:
        return np.asarray(spline(out_t), np.float64)
    pad = round(SETTLE_S * RATE)
    kept = out_t[max(0, moving[0] - pad) : min(len(out_t), moving[-1] + 2 + pad)]
    start, span = kept[0], kept[-1] - kept[0]  # source seconds
    total = span / speed + EASE_S  # output seconds: the two half-ramps each cost EASE_S / 2
    if total < 2 * EASE_S:
        raise ValueError(f"clip is {total:.2f} s of output, shorter than 2 * EASE_S")
    tau = np.arange(0.0, total, 1 / RATE)
    tau = np.append(tau, total) if total - tau[-1] > 0.5 / RATE else tau
    # source time covered after tau seconds of output; the rate is linear 0 -> speed -> 0 at the ends
    up = speed * tau**2 / (2 * EASE_S)
    down = span - speed * (total - tau) ** 2 / (2 * EASE_S)
    src = np.where(
        tau < EASE_S, up, np.where(tau > total - EASE_S, down, speed * (tau - EASE_S / 2))
    )
    return np.asarray(spline(start + src), np.float64)


def main(src: str, speed: str, dst: str) -> None:
    rows = convert(np.loadtxt(src, delimiter=","), float(speed))
    np.savetxt(dst, rows, fmt="%.6f", delimiter=",", header=",".join(CLIP_COLUMNS), comments="")
    peak = np.abs(np.diff(rows, axis=0)) * RATE
    j = np.unravel_index(peak.argmax(), peak.shape)[1]
    print(f"{dst}: {len(rows) / RATE:.2f} s, peak {peak.max():.2f} rad/s ({CLIP_COLUMNS[j]})")


if __name__ == "__main__":
    main(*sys.argv[1:4])
