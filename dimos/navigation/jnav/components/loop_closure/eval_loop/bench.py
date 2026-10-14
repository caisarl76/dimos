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

"""Benchmark Ivan's go2 PGO (dimos/navigation/go2/loop_closure/pgo.py) against gsc_pgo (pgo-eval).

Both get the exact same input: the recording's `lidar` stream (world-frame scans, each
stamped with its odom pose). Both run at their DEFAULT config. Each yields a keyframe
table (ts, raw pose, optimized pose); the same slerp/lerp turns that into a per-scan
correction, and the same renderer draws raw / Ivan / gsc top-downs.

usage: bench.py <dataset> [<dataset> ...]
"""

import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from paths import OUT as HERE, PGO_EVAL, REPO, import_ivan
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

PGO = import_ivan()[1].PGO

DATA = REPO / "data"
RENDER_POINTS_PER_SCAN = 4000
VOXEL_M = 0.2


def load(db: Path):
    """Scans in time order: ts, (N,7) pose rows [x,y,z,qx,qy,qz,qw], world points (subsampled)."""
    rng = np.random.default_rng(0)
    times, poses, clouds = [], [], []
    with SqliteStore(path=str(db)) as store:
        per_scan = max(
            200, min(RENDER_POINTS_PER_SCAN, int(6e6 / max(1, store.streams.lidar.count())))
        )
        for obs in store.streams.lidar.order_by("ts"):
            pose = obs.pose
            if pose is None or pose.position.is_zero() or pose.orientation.is_zero():
                continue
            pts = np.asarray(obs.data.points_f32(), np.float64)
            if len(pts) == 0:
                continue
            if len(pts) > per_scan:
                pts = pts[rng.choice(len(pts), per_scan, replace=False)]
            p, q = pose.position, pose.orientation
            times.append(obs.ts)
            poses.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            clouds.append(pts)
    return np.asarray(times), np.asarray(poses), clouds


def run_ivan(db: Path):
    with SqliteStore(path=str(db)) as store:
        started = time.perf_counter()
        graph = store.streams.lidar.transform(PGO()).last().data
        runtime = time.perf_counter() - started
    rows_raw, rows_opt = [], []
    for kf in graph.keyframes:
        for rows, tf in ((rows_raw, kf.local), (rows_opt, kf.optimized)):
            t, r = tf.translation, tf.rotation
            rows.append([kf.ts, t.x, t.y, t.z, r.x, r.y, r.z, r.w])
    return np.asarray(rows_raw), np.asarray(rows_opt), len(graph.loops), runtime


def write_gsc_input(times, poses, db: Path, out: Path):
    """Body-frame copy of the full-res lidar + an Odometry stream of the scans' own poses."""
    if out.exists():
        return
    tmp = out.with_suffix(".tmp.db")
    tmp.unlink(missing_ok=True)
    pose_by_ts = dict(zip(times.tolist(), poses, strict=True))
    with SqliteStore(path=str(db)) as src, SqliteStore(path=str(tmp)) as dst:
        odom = dst.stream("bench_odom", Odometry)
        lidar = dst.stream("bench_lidar", PointCloud2)
        for obs in src.streams.lidar.order_by("ts"):
            row = pose_by_ts.get(obs.ts)
            if row is None:
                continue
            x, y, z, qx, qy, qz, qw = row
            rotation = Rotation.from_quat([qx, qy, qz, qw])
            body = rotation.inv().apply(np.asarray(obs.data.points_f32(), np.float64) - [x, y, z])
            odom.append(
                Odometry(
                    ts=obs.ts,
                    frame_id="world",
                    child_frame_id="base_link",
                    pose=Pose(x, y, z, qx, qy, qz, qw),
                ),
                ts=obs.ts,
                pose=(x, y, z, qx, qy, qz, qw),
            )
            lidar.append(
                PointCloud2.from_numpy(
                    body.astype(np.float32), frame_id="base_link", timestamp=obs.ts
                ),
                ts=obs.ts,
                pose=(x, y, z, qx, qy, qz, qw),
            )
    tmp.rename(out)


def run_gsc(gsc_db: Path, out_json: Path):
    started = time.perf_counter()
    subprocess.run(
        [
            str(PGO_EVAL),
            "--db",
            str(gsc_db),
            "--odom-stream",
            "bench_odom",
            "--lidar-stream",
            "bench_lidar",
            "--output",
            str(out_json),
        ],
        check=True,
    )
    runtime = time.perf_counter() - started
    result = json.loads(out_json.read_text())
    odom = np.asarray(result["odom_rows"], float)
    graph = np.asarray(result["graph"], float)
    # raw pose at each keyframe = the odom row with that exact timestamp
    index = np.clip(np.searchsorted(odom[:, 0], graph[:, 0]), 0, len(odom) - 1)
    return odom[index], graph, result["closures"], runtime, result


def correction_table(raw: np.ndarray, opt: np.ndarray):
    """Per-keyframe correction C = optimized · raw^-1, as (ts, Rotation, translations)."""
    r_raw, r_opt = Rotation.from_quat(raw[:, 4:8]), Rotation.from_quat(opt[:, 4:8])
    r_c = r_opt * r_raw.inv()
    t_c = opt[:, 1:4] - r_c.apply(raw[:, 1:4])
    return raw[:, 0], r_c, t_c


def apply_correction(table, times, poses, clouds):
    ts_k, r_c, t_c = table
    order = np.argsort(ts_k)
    ts_k, r_c, t_c = ts_k[order], r_c[order], t_c[order]
    keep = np.concatenate([[True], np.diff(ts_k) > 0])
    ts_k, r_c, t_c = ts_k[keep], r_c[keep], t_c[keep]
    clipped = np.clip(times, ts_k[0], ts_k[-1])
    rotations = Slerp(ts_k, r_c)(clipped)
    translations = np.stack([np.interp(clipped, ts_k, t_c[:, i]) for i in range(3)], axis=1)
    positions = rotations.apply(poses[:, :3]) + translations
    new_clouds = [rotations[i].apply(c) + translations[i] for i, c in enumerate(clouds)]
    return positions, new_clouds


def occupied_voxels(clouds) -> int:
    allpts = np.vstack(clouds)
    return len(np.unique(np.floor(allpts / VOXEL_M).astype(np.int64), axis=0))


def main():
    summary = []
    for name in sys.argv[1:]:
        name, _, explicit = name.partition("=")
        db = Path(explicit) if explicit else HERE / f"{name}.src.db"
        if not db.exists():
            db = DATA / f"{name}.db"
        print(f"== {name}", flush=True)
        times, poses, clouds = load(db)
        ivan_error = None
        try:
            k_raw, k_opt, ivan_loops, ivan_time = run_ivan(db)
        except Exception as error:  # Ivan's PGO raising kills his whole run; show raw in his panel
            ivan_error = f"{type(error).__name__}: {str(error)[-160:]}"
            print("ivan CRASHED:", ivan_error, flush=True)
            k_raw = k_opt = np.column_stack([times, poses])
            ivan_loops, ivan_time = 0, float("nan")
        print(f"ivan: {len(k_raw)} keyframes, {ivan_loops} loops, {ivan_time:.1f}s", flush=True)
        gsc_db = HERE / f"{name}.gsc.db"
        write_gsc_input(times, poses, db, gsc_db)
        g_raw, g_opt, gsc_loops, gsc_time, gsc_result = run_gsc(gsc_db, HERE / f"{name}.gsc.json")
        print(f"gsc: {len(g_raw)} keyframes, {gsc_loops} loops, {gsc_time:.1f}s", flush=True)

        ivan_pos, ivan_clouds = apply_correction(
            correction_table(k_raw, k_opt), times, poses, clouds
        )
        gsc_pos, gsc_clouds = apply_correction(correction_table(g_raw, g_opt), times, poses, clouds)
        np.savez_compressed(
            HERE / f"{name}.maps.npz",
            raw=np.vstack(clouds),
            ivan=np.vstack(ivan_clouds),
            gsc=np.vstack(gsc_clouds),
            raw_path=poses[:, :3],
            ivan_path=ivan_pos,
            gsc_path=gsc_pos,
            times=times,
        )
        row = {
            "dataset": name,
            "scans": len(times),
            "duration_s": float(times[-1] - times[0]),
            "raw_end_gap_m": float(np.linalg.norm(poses[-1, :3] - poses[0, :3])),
            "voxels_raw": occupied_voxels(clouds),
            "ivan": {
                "error": ivan_error,
                "keyframes": len(k_raw),
                "loops": ivan_loops,
                "runtime_s": ivan_time,
                "voxels": occupied_voxels(ivan_clouds),
                "end_gap_m": float(np.linalg.norm(ivan_pos[-1] - ivan_pos[0])),
                "max_shift_m": float(np.max(np.linalg.norm(ivan_pos - poses[:, :3], axis=1))),
            },
            "gsc": {
                "keyframes": len(g_raw),
                "loops": gsc_loops,
                "runtime_s": gsc_time,
                "voxels": occupied_voxels(gsc_clouds),
                "end_gap_m": float(np.linalg.norm(gsc_pos[-1] - gsc_pos[0])),
                "max_shift_m": float(np.max(np.linalg.norm(gsc_pos - poses[:, :3], axis=1))),
                "gate_counts": gsc_result.get("gate_counts"),
            },
        }
        print(json.dumps(row), flush=True)
        (HERE / f"{name}.summary.json").write_text(json.dumps(row, indent=2))
        summary.append(row)


if __name__ == "__main__":
    main()
