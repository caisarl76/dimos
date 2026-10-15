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

"""Passive static check of the torso_link -> Mid-360 mount on pc2.

With the robot still, two independent observations must agree with the pelvis
IMU's gravity carried through the live waist joints into torso_link:
the Mid-360's own accelerometer rotated by the mount, and the normal of the
LiDAR floor plane. This verifies the mount's roll and pitch and reports the
torso height above the floor. Mount yaw is not observable statically; check it
from the first forward walking case. DDS readers only, like the readiness probe.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.unitree.g1.g1_tf_publisher import base_to_torso, torso_to_mid360

MAX_TILT_DEG = 1.0
MAX_GYRO_RMS = 0.05  # rad/s on the pelvis IMU
MIN_FLOOR_POINTS = 500
WAIST_JOINTS = (12, 13, 14)  # rt/lowstate yaw, roll, pitch

Array = NDArray[np.float64]


def rotation_translation(transform: Transform) -> tuple[Rotation, Array]:
    q, t = transform.rotation, transform.translation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])


def unit(v: Array) -> Array:
    return v / np.linalg.norm(v)  # type: ignore[no-any-return]


def angle_deg(a: Array, b: Array) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0))))


def pelvis_up_in_torso(pelvis_accel: Array, waist: Array) -> Array:
    """Mean specific force (gravity reaction, "up") from the pelvis IMU, in torso_link."""
    up = [
        rotation_translation(base_to_torso(*w))[0].inv().apply(unit(a))
        for a, w in zip(pelvis_accel, waist, strict=True)
    ]
    return unit(np.mean(up, axis=0))


def floor_plane(cloud: Array, up: Array, mount: Transform) -> dict[str, Any]:
    """Dominant plane well below the torso, fitted in torso_link."""
    rotation, translation = rotation_translation(mount)
    points = rotation.apply(cloud) + translation if len(cloud) else np.empty((0, 3))
    heights = points @ up
    below = heights < -0.5
    if below.sum() < MIN_FLOOR_POINTS:
        return {"points": int(below.sum()), "found": False}
    counts, edges = np.histogram(heights[below], bins=np.arange(-3.0, -0.5, 0.02))
    mode = edges[np.argmax(counts)] + 0.01
    inliers = points[below & (np.abs(heights - mode) < 0.03)]
    normal = up
    for tolerance in (None, 0.01):  # fit, then refit on points within 1 cm of the plane
        if tolerance is not None:
            inliers = inliers[np.abs((inliers - centroid) @ normal) < tolerance]
        centroid = inliers.mean(axis=0)
        normal = np.linalg.svd(inliers - centroid, full_matrices=False)[2][2]
        normal = normal if normal @ up > 0 else -normal
    return {
        "points": len(inliers),
        "found": len(inliers) >= MIN_FLOOR_POINTS,
        "tilt_deg": angle_deg(normal, up),
        "torso_height_m": float(-(centroid @ normal)),
        "normal_torso": normal.tolist(),
    }


def tilt_components(a: Array, b: Array) -> tuple[float, float]:
    """(roll, pitch) in degrees of up vector `a` relative to `b`, about torso x and y."""
    roll = math.atan2(-a[1], a[2]) - math.atan2(-b[1], b[2])
    pitch = math.atan2(a[0], a[2]) - math.atan2(b[0], b[2])
    return math.degrees(roll), math.degrees(pitch)


def evaluate(
    mid360_accel: Array,
    pelvis_accel: Array,
    waist: Array,
    cloud: Array,
    mount: Transform,
    *,
    pelvis_gyro_rms: float = 0.0,
) -> dict[str, Any]:
    rotation, _ = rotation_translation(mount)
    up = pelvis_up_in_torso(pelvis_accel, waist)
    mid360_up = unit(rotation.apply(unit(np.mean(mid360_accel, axis=0))))
    floor = floor_plane(cloud, up, mount)
    roll, pitch = tilt_components(mid360_up, up)
    if floor["found"]:
        floor["roll_deg"], floor["pitch_deg"] = tilt_components(np.array(floor["normal_torso"]), up)
    result = {
        "static": pelvis_gyro_rms < MAX_GYRO_RMS,
        "pelvis_gyro_rms": pelvis_gyro_rms,
        "gravity": {
            "angle_deg": angle_deg(mid360_up, up),
            "roll_deg": roll,
            "pitch_deg": pitch,
            "pelvis_up_torso": up.tolist(),
            "mid360_up_torso": mid360_up.tolist(),
            "samples": {"mid360": len(mid360_accel), "pelvis": len(pelvis_accel)},
        },
        "floor": floor,
        # Mount-independent: floor levelness plus the Mid-360's internal IMU-to-LiDAR alignment.
        "floor_vs_mid360_deg": (
            angle_deg(np.array(floor["normal_torso"]), mid360_up) if floor["found"] else None
        ),
        "max_tilt_deg": MAX_TILT_DEG,
    }
    result["passed"] = bool(
        result["static"]
        and result["gravity"]["angle_deg"] <= MAX_TILT_DEG
        and floor["found"]
        and floor["tilt_deg"] <= MAX_TILT_DEG
    )
    return result


def observe(interface: str, domain: int, seconds: float) -> dict[str, Any]:
    """Readers only. Lazy imports: these packages exist in the pc2 task image."""
    from cyclonedds.domain import DomainParticipant  # type: ignore[import-not-found]
    from cyclonedds.qos import Policy, Qos  # type: ignore[import-not-found]
    from cyclonedds.sub import DataReader  # type: ignore[import-not-found]
    from cyclonedds.topic import Topic  # type: ignore[import-not-found]
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,  # type: ignore[import-not-found]
    )
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import (
        PointCloud2_,  # type: ignore[import-not-found]
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_  # type: ignore[import-not-found]

    from dimos.robot.unitree.g1.sonic_navigation_sensor_module import livox_imu_type

    ChannelFactoryInitialize(domain, interface)
    participant = DomainParticipant(domain)
    qos = Qos(
        Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(256)
    )
    readers = {
        name: DataReader(participant, Topic(participant, topic, kind), qos=qos)
        for name, topic, kind in (
            ("lidar", "rt/utlidar/cloud_livox_mid360", PointCloud2_),
            ("imu", "rt/utlidar/imu_livox_mid360", livox_imu_type()),
            ("lowstate", "rt/lowstate", LowState_),
        )
    }
    seen: dict[str, list[Any]] = {"mid360": [], "pelvis": [], "gyro": [], "waist": [], "cloud": []}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        for sample in readers["imu"].take(256):
            if sample.sample_info.valid_data:
                a = sample.linear_acceleration
                seen["mid360"].append((a.x, a.y, a.z))
        for sample in readers["lowstate"].take(256):
            if sample.sample_info.valid_data:
                seen["pelvis"].append(tuple(sample.imu_state.accelerometer))
                seen["gyro"].append(tuple(sample.imu_state.gyroscope))
                seen["waist"].append(tuple(sample.motor_state[j].q for j in WAIST_JOINTS))
        for sample in readers["lidar"].take(16):
            if sample.sample_info.valid_data:
                fields = {f.name: f.offset for f in sample.fields}
                raw = np.frombuffer(bytes(sample.data), dtype=np.uint8).reshape(
                    -1, sample.point_step
                )
                xyz = np.column_stack(
                    [raw[:, fields[n] : fields[n] + 4].copy().view("<f4")[:, 0] for n in "xyz"]
                ).astype(np.float64)
                seen["cloud"].append(xyz[np.isfinite(xyz).all(axis=1)])
        time.sleep(0.002)
    return seen


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="enP8p1s0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    started = time.time()
    seen = observe(args.interface, args.domain, args.seconds)
    if not (seen["mid360"] and seen["pelvis"]):
        raise SystemExit(f"missing IMU samples: {[(k, len(v)) for k, v in seen.items()]}")
    gyro = np.array(seen["gyro"], dtype=np.float64)
    cloud = np.vstack(seen["cloud"]) if seen["cloud"] else np.empty((0, 3))
    result = evaluate(
        np.array(seen["mid360"], dtype=np.float64),
        np.array(seen["pelvis"], dtype=np.float64),
        np.array(seen["waist"], dtype=np.float64),
        cloud,
        torso_to_mid360(),
        pelvis_gyro_rms=float(np.sqrt((gyro**2).sum(axis=1).mean())),
    )
    mid360, pelvis = np.array(seen["mid360"]), np.array(seen["pelvis"])
    result.update(
        started_unix=started,
        observation_seconds=args.seconds,
        mount="g1_tf_publisher.torso_to_mid360 (URDF mount pitch + upside-down roll)",
        clouds=len(seen["cloud"]),
        cloud_points=len(cloud),
        mid360_accel_norm_g=float(np.linalg.norm(mid360.mean(axis=0))),
        pelvis_accel_norm=float(np.linalg.norm(pelvis.mean(axis=0))),
        waist_mean_rad=np.mean(seen["waist"], axis=0).tolist(),
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps({k: result[k] for k in ("passed", "static", "gravity", "floor_vs_mid360_deg")})
    )
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
