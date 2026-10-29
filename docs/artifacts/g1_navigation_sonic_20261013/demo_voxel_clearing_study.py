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

"""Why the voxels a person leaves in front of a standing G1 survive (sessions 2026-10-22-3..6).

The real VoxelRayMapper (peer settings: 0.05 m voxels, defaults otherwise) is fed scans
from a modelled MID360 at the G1 head mount: upside down, so its -7..52 deg field looks
down to about 52 deg below the horizon, with PointLio's 0.5 m blind range. A room with
two real boxes that must stay mapped, a person standing `distance` ahead for 5 s, then
10 s with the person gone; ranges carry the MID360's 2 cm noise. The voxels still inside
the person's footprint are the stale ones, and the costmap cells they block (above
can_climb). Variants change the clearing settings: grace_depth (no clearing this near a
ray's hit) and graze_cos (no clearing by a ray this oblique to the voxel's surface). The
floor and box cells each variant keeps show what a looser setting costs.

A robot recording (walk harness `record_lidar`, peer LidarRecording) replays the same
way: its scans under each setting, and the blocked cells within 3 m that each leaves.

Usage (needs dimos_voxel_ray_tracing):
    demo_voxel_clearing_study.py <output.json>
    demo_voxel_clearing_study.py <output.json> --replay <lidar-NN.npz>
"""

import json
import math
from pathlib import Path
import sys
from typing import Any

from dimos_voxel_ray_tracing import VoxelRayMapper
import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360

PELVIS_Z = 0.80  # standing G1, floor at z = 0
ROOM = (-3.0, 6.0, -3.0, 3.0, 2.8)  # x_min, x_max, y_min, y_max, ceiling
FOV_DEG = (-7.0, 52.0)  # MID360 elevation, its own frame
BLIND_M = 0.5  # PointLio blind
RAYS = 8000  # per 10 Hz scan, after PointLio's every-3rd-point decimation
RANGE_NOISE_M = 0.02
CELL_M, CAN_CLIMB_M = 0.05, 0.10  # the peer costmap: cells, and the height that blocks
PERSON_RADIUS, PERSON_HEIGHT = 0.18, 1.7
# Real obstacles that must stay: the harness's 40 cm box, and a 12 cm step near the floor.
BOXES = {"box_40cm": (2.0, 0.8, 0.2, 0.4), "step_12cm": (1.6, -0.9, 0.25, 0.12)}  # cx cy half h
PRESENT, ABSENT = 50, 100  # scans: 5 s standing, then 10 s gone
PEER = {"voxel_size": 0.05, "max_range": 30.0, "worker_threads": 2}
VARIANTS = {
    "peer": {},
    "grace_0.05": {"grace_depth": 0.05},
    "graze_0.5": {"graze_cos": 0.5},
    "graze_0.3": {"graze_cos": 0.3},
    "graze_0": {"graze_cos": 0.0},
    "grace_0.05_graze_0": {"grace_depth": 0.05, "graze_cos": 0.0},
}
DISTANCES = (0.6, 0.8, 1.0, 1.5)
BANDS = ((0.0, 0.1), (0.1, 0.3), (0.3, 1.0), (1.0, 2.0))


def sensor_pose() -> Transform:
    base = Transform(
        translation=Vector3(0.0, 0.0, PELVIS_Z), frame_id="odom", child_frame_id="base_link"
    )
    return base + base_to_torso(0.0, 0.0, 0.0) + torso_to_mid360()


def directions(rng: np.random.Generator) -> np.ndarray:
    """Unit rays in the sensor frame, uniform over the field (the MID360's non-repeating
    pattern covers it within a fraction of a second)."""
    azimuth = rng.uniform(0.0, 2 * math.pi, RAYS)
    low, high = (math.sin(math.radians(a)) for a in FOV_DEG)
    z = rng.uniform(low, high, RAYS)
    r = np.sqrt(1 - z**2)
    return np.stack([r * np.cos(azimuth), r * np.sin(azimuth), z], axis=1)


def cast(origin: np.ndarray, rays: np.ndarray, person: float | None) -> np.ndarray:
    """Range of each world-frame ray to the room, the boxes or the person (inf: none)."""
    x0, x1, y0, y1, ceiling = ROOM
    hits = np.full(len(rays), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for axis, planes in ((0, (x0, x1)), (1, (y0, y1)), (2, (0.0, ceiling))):
            for plane in planes:
                t = (plane - origin[axis]) / rays[:, axis]
                hits = np.where((t > 0) & (t < hits), t, hits)
        for cx, cy, half, height in BOXES.values():
            low = np.array([cx - half, cy - half, 0.0]) - origin
            high = np.array([cx + half, cy + half, height]) - origin
            t0, t1 = low / rays, high / rays
            near = np.nanmax(np.minimum(t0, t1), axis=1)
            far = np.nanmin(np.maximum(t0, t1), axis=1)
            hits = np.where((near <= far) & (near > 0) & (near < hits), near, hits)
        if person is not None:
            # Vertical cylinder at (person, 0), from the floor up.
            dx, dy = origin[0] - person, origin[1]
            a = rays[:, 0] ** 2 + rays[:, 1] ** 2
            b = 2 * (dx * rays[:, 0] + dy * rays[:, 1])
            c = dx**2 + dy**2 - PERSON_RADIUS**2
            t = (-b - np.sqrt(b**2 - 4 * a * c)) / (2 * a)
            z = origin[2] + t * rays[:, 2]
            inside = (t > 0) & (z >= 0) & (z <= PERSON_HEIGHT) & (t < hits)
            hits = np.where(inside, t, hits)
    return hits


def stale_voxels(mapper: Any, person: float) -> np.ndarray:
    voxels = mapper.global_map()
    footprint = np.hypot(voxels[:, 0] - person, voxels[:, 1]) <= PERSON_RADIUS + 0.05
    return voxels[footprint & (voxels[:, 2] > 0.05)]  # above the floor voxel layer


def cells(points: np.ndarray) -> set[tuple[int, int]]:
    return {(int(x), int(y)) for x, y in np.floor(points[:, :2] / CELL_M)}


def kept(mapper: Any) -> dict[str, float]:
    """What the costmap gets from real surfaces: the share of floor cells mapped in
    rings around the robot, and of each box's footprint cells blocked (above can_climb)."""
    voxels = mapper.global_map()
    floor = cells(voxels[voxels[:, 2] < 0.05])
    shares: dict[str, float] = {}
    for low, high in ((0.5, 1.0), (1.0, 2.0), (2.0, 3.0)):
        grid = np.mgrid[-high:high:CELL_M, -high:high:CELL_M].reshape(2, -1).T + CELL_M / 2
        ring = grid[(np.hypot(*grid.T) >= low) & (np.hypot(*grid.T) < high)]
        free = [c for c in cells(ring) if not any(_in_box(c))]
        shares[f"floor_{low}-{high}m"] = round(sum(c in floor for c in free) / len(free), 3)
    blocked = cells(voxels[voxels[:, 2] > CAN_CLIMB_M])
    for name, (cx, cy, half, _) in BOXES.items():
        grid = np.mgrid[cx - half : cx + half : CELL_M, cy - half : cy + half : CELL_M]
        footprint = cells(grid.reshape(2, -1).T + CELL_M / 2)
        shares[name] = round(len(footprint & blocked) / len(footprint), 3)
    return shares


def _in_box(cell: tuple[int, int]) -> list[bool]:
    x, y = (cell[0] + 0.5) * CELL_M, (cell[1] + 0.5) * CELL_M
    return [abs(x - cx) <= half and abs(y - cy) <= half for cx, cy, half, _ in BOXES.values()]


def run(variant: dict[str, float], distance: float) -> dict[str, Any]:
    rng = np.random.default_rng(0)
    pose = sensor_pose()
    origin = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
    rotation = pose.to_matrix()[:3, :3]
    q = pose.rotation
    mapper = VoxelRayMapper(**PEER, **variant)
    for scan in range(PRESENT + ABSENT):
        local = directions(rng)
        ranges = cast(origin, local @ rotation.T, distance if scan < PRESENT else None)
        ranges += rng.normal(0.0, RANGE_NOISE_M, len(ranges))
        keep = np.isfinite(ranges) & (ranges >= BLIND_M) & (ranges <= PEER["max_range"])
        points = (local[keep] * ranges[keep, None]).astype(np.float32)
        mapper.add_frame(points, tuple(origin), (q.x, q.y, q.z, q.w))
        if scan == PRESENT - 1:
            seen = len(stale_voxels(mapper, distance))
            surfaces_before = kept(mapper)
    stale = stale_voxels(mapper, distance)
    bands = {
        f"{lo:.1f}-{hi:.1f}": int(((stale[:, 2] >= lo) & (stale[:, 2] < hi)).sum())
        for lo, hi in BANDS
    }
    return {
        "person_voxels_seen": seen,
        "stale_after_10s": len(stale),
        "stale_blocking_cells": len(cells(stale[stale[:, 2] > CAN_CLIMB_M])),
        # On the person's flanks, which the standing sensor sees edge-on.
        "stale_on_flank": int((np.abs(stale[:, 1] - origin[1]) > PERSON_RADIUS / 2).sum()),
        "stale_by_height_m": bands,
        "surfaces_at_5s": surfaces_before,
        "surfaces_at_15s": kept(mapper),
    }


def blocked_near(voxels: np.ndarray, sensor: np.ndarray) -> set[tuple[int, int]]:
    """Cells within 3 m with a voxel above can_climb over the floor, the floor taken as
    the most populated 4 cm height bin well below the sensor."""
    near = voxels[np.hypot(*(voxels[:, :2] - sensor[:2]).T) <= 3.0]
    low = near[near[:, 2] < sensor[2] - 0.8, 2]
    counts, edges = np.histogram(low, bins=np.arange(low.min(), low.max() + 0.08, 0.04))
    floor = edges[int(np.argmax(counts))] + 0.02
    return cells(near[near[:, 2] > floor + CAN_CLIMB_M])


def replay(path: Path) -> dict[str, Any]:
    with np.load(path) as recording:
        points, offsets, stamps = (recording[k] for k in ("points", "scan_offsets", "scan_ts"))
        poses, live = recording["poses"], recording["map_after"]
    sensor = poses[-1, :3]
    results: dict[str, Any] = {
        "scans": len(stamps),
        "live_map_blocked_cells": len(blocked_near(live, sensor)),
    }
    for name in ("peer", "graze_0.3", "graze_0"):
        mapper = VoxelRayMapper(**PEER, **VARIANTS[name])
        used = 0
        for i, stamp in enumerate(stamps):
            nearest = int(np.argmin(np.abs(poses[:, 7] - stamp)))
            if abs(poses[nearest, 7] - stamp) > 0.1:  # the ray map's tf tolerance
                continue
            x, y, z, qx, qy, qz, qw, _ = poses[nearest]
            mapper.add_frame(points[offsets[i] : offsets[i + 1]], (x, y, z), (qx, qy, qz, qw))
            used += 1
        blocked = blocked_near(mapper.global_map(), sensor)
        results[name] = {"scans_used": used, "blocked_cells": len(blocked)}
        print(name, results[name], flush=True)
    return results


def main() -> None:
    if "--replay" in sys.argv:
        recording = Path(sys.argv[sys.argv.index("--replay") + 1])
        Path(sys.argv[1]).write_text(json.dumps(replay(recording), indent=1) + "\n")
        return
    pose = sensor_pose()
    results: dict[str, Any] = {
        "sensor_height_m": pose.translation.z,
        "nearest_floor_seen_m": pose.translation.z / math.tan(math.radians(FOV_DEG[1])),
        "peer": PEER,
        "boxes": BOXES,
        "runs": {},
    }
    for name, variant in VARIANTS.items():
        for distance in DISTANCES:
            row = run(variant, distance)
            results["runs"][f"{name}@{distance}"] = row
            print(name, distance, row, flush=True)
    Path(sys.argv[1]).write_text(json.dumps(results, indent=1) + "\n")


if __name__ == "__main__":
    main()
