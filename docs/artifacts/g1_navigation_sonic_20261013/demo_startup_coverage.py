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


"""Startup LiDAR blind spots: what one scan sees, and what a waist yaw sweep would add.

Reads the recorded MID360S scan (stamp-replay/dds-inputs.npz) and startup costmap
snapshots of robot sessions. Offline only; no robot or simulator.

    python docs/artifacts/g1_navigation_sonic_20261013/demo_startup_coverage.py
"""

import json
import math
from pathlib import Path

import numpy as np

from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360

HERE = Path(__file__).resolve().parent
STARTUP_SESSIONS = ("session-20261019-11", "session-20261020-5", "session-20261021-8")


def floor_returns() -> tuple[np.ndarray, float]:
    """Floor returns of the recorded scan in the base frame, and the sensor height."""
    scan = np.load(HERE / "stamp-replay/dds-inputs.npz", allow_pickle=True)["last_cloud"]
    points = np.c_[scan["x"], scan["y"], scan["z"]].astype(float)
    points = points[np.linalg.norm(points, axis=1) > 0.05]  # zeros are no-return beams
    matrix = (base_to_torso(0.0, 0.0, 0.0) + torso_to_mid360()).to_matrix()
    base = points @ matrix[:3, :3].T + matrix[:3, 3]
    floor_z = float(np.percentile(base[:, 2], 2))
    return base[np.abs(base[:, 2] - floor_z) < 0.06], float(matrix[2, 3] - floor_z)


def swept_coverage(floor: np.ndarray, sweep_deg: float, near: float, far: float) -> float:
    """Share of 1-degree bearings with floor seen in [near, far) after a waist yaw sweep.

    The waist axis passes within 4 mm of the sensor, so a sweep rotates the visible pattern
    (occluders included) about the robot's vertical axis.
    """
    distance = np.hypot(floor[:, 0], floor[:, 1])
    bearing = np.degrees(np.arctan2(floor[:, 1], floor[:, 0]))[
        (distance >= near) & (distance < far)
    ]
    seen = np.zeros(360, dtype=bool)
    for offset in np.arange(-sweep_deg, sweep_deg + 1.0, 5.0) if sweep_deg else (0.0,):
        seen[(np.floor(bearing + offset) % 360).astype(int)] = True
    return float(seen.mean())


def startup_unknown(session: str) -> dict[str, list[int]]:
    """Unknown cell share (%) by 30-degree bearing at startup, before any walking."""
    results = json.loads((HERE / "hardware" / session / "results.json").read_text())
    first = next(c for c in results["cases"] if c.get("snapshot"))
    snap = np.load(HERE / "hardware" / session / first["snapshot"])
    grid, (ox, oy), res = snap["grid"], snap["origin"], float(snap["resolution"])
    x, y, _, heading = snap["robot"]
    rows, cols = np.indices(grid.shape)
    cx, cy = ox + (cols + 0.5) * res, oy + (rows + 0.5) * res
    distance = np.hypot(cx - x, cy - y)
    bearing = np.degrees(
        np.remainder(np.arctan2(cy - y, cx - x) - heading + math.pi, 2 * math.pi) - math.pi
    )
    out = {}
    for near, far in ((0.5, 1.0), (1.0, 2.0)):
        band = (distance >= near) & (distance < far)
        out[f"{near}-{far} m"] = [
            round(100 * float((grid[band & (bearing >= b) & (bearing < b + 30)] < 0).mean()))
            for b in range(-180, 180, 30)
        ]
    return out


def main() -> None:
    floor, height = floor_returns()
    reach = float(np.hypot(floor[:, 0], floor[:, 1]).min())
    print(f"sensor {height:.2f} m above the floor; nearest floor return {reach:.2f} m")
    for near, far in ((0.5, 1.0), (1.0, 1.5), (1.5, 2.0)):
        row = [
            f"+-{s:3.0f}: {100 * swept_coverage(floor, s, near, far):3.0f}%"
            for s in (0, 30, 60, 90, 150)
        ]
        print(f"floor bearings seen at {near}-{far} m, by waist yaw sweep: " + " | ".join(row))
    print("startup unknown % by bearing (-180..150 in 30 deg steps, 0 = forward):")
    for session in STARTUP_SESSIONS:
        print(f"  {session}: {startup_unknown(session)}")


if __name__ == "__main__":
    main()
