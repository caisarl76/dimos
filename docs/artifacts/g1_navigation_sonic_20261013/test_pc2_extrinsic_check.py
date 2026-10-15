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

"""Static Mid-360 mount check on synthetic scenes built from a known robot pose."""

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.g1_tf_publisher import torso_to_mid360
from docs.artifacts.g1_navigation_sonic_20261013 import demo_pc2_extrinsic_check as check

WAIST = np.array([0.02, -0.01, 0.03])
WORLD_BASE = Rotation.from_euler("xyz", [0.01, 0.05, 0.4])  # leaning, arbitrary heading
TORSO_HEIGHT = 0.85


def scene(mount: Transform) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Mid-360 accel (g), pelvis accel (m/s^2) and a cloud for a sensor truly at `mount`."""
    base_torso, _ = check.rotation_translation(check.base_to_torso(*WAIST))
    torso_mid, torso_mid_xyz = check.rotation_translation(mount)
    world_torso = WORLD_BASE * base_torso
    up = np.array([0.0, 0.0, 1.0])
    pelvis = WORLD_BASE.inv().apply(up * 9.80665)
    mid360 = (world_torso * torso_mid).inv().apply(up)
    rng = np.random.default_rng(0)
    xy = rng.uniform(-4, 4, (4000, 2))
    floor = np.column_stack([xy, np.full(len(xy), -TORSO_HEIGHT)])
    table = np.column_stack([rng.uniform(1, 2, (800, 2)), np.full(800, 0.75 - TORSO_HEIGHT)])
    wall = np.column_stack(
        [np.full(1500, 3.0), rng.uniform(-3, 3, 1500), rng.uniform(-TORSO_HEIGHT, 1.0, 1500)]
    )
    world = np.vstack([floor, table, wall])  # relative to the torso origin, world axes
    cloud = torso_mid.inv().apply(world_torso.inv().apply(world) - torso_mid_xyz)
    return np.tile(mid360, (50, 1)), np.tile(pelvis, (500, 1)), cloud


def test_nominal_mount_agrees_with_pelvis_gravity_and_floor() -> None:
    mid360, pelvis, cloud = scene(torso_to_mid360())
    result = check.evaluate(mid360, pelvis, np.tile(WAIST, (500, 1)), cloud, torso_to_mid360())
    assert result["gravity"]["angle_deg"] < 0.01
    assert result["floor"]["tilt_deg"] < 0.01
    assert abs(result["floor"]["torso_height_m"] - TORSO_HEIGHT) < 0.005
    assert result["passed"]


def test_two_degree_roll_error_fails_both_checks() -> None:
    nominal = torso_to_mid360()
    true_mount = Transform(
        translation=nominal.translation,
        rotation=nominal.rotation * Quaternion.from_euler(Vector3(np.radians(2.0), 0.0, 0.0)),
        frame_id="torso_link",
        child_frame_id="mid360_link",
    )
    mid360, pelvis, cloud = scene(true_mount)
    result = check.evaluate(mid360, pelvis, np.tile(WAIST, (500, 1)), cloud, nominal)
    assert abs(result["gravity"]["angle_deg"] - 2.0) < 0.05
    assert abs(result["floor"]["tilt_deg"] - 2.0) < 0.05
    assert abs(abs(result["gravity"]["roll_deg"]) - 2.0) < 0.05
    assert abs(result["gravity"]["pitch_deg"]) < 0.05
    assert result["floor_vs_mid360_deg"] < 0.01  # both seen through the same mount
    assert not result["passed"]


def test_missing_floor_or_motion_fails() -> None:
    mid360, pelvis, cloud = scene(torso_to_mid360())
    waist = np.tile(WAIST, (500, 1))
    no_floor = cloud[:0]
    assert not check.evaluate(mid360, pelvis, waist, no_floor, torso_to_mid360())["passed"]
    moving = check.evaluate(mid360, pelvis, waist, cloud, torso_to_mid360(), pelvis_gyro_rms=0.2)
    assert not moving["static"] and not moving["passed"]
