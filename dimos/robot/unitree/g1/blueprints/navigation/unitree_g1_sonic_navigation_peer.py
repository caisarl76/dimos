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

"""Workstation half of SONIC navigation: sensor receiver, PointLio, mapping, A* and peer.

Runs with its own graph validation; the pc2 half is a separate blueprint. Endpoints
and the commissioned torso->sensor extrinsic are explicit configuration.
"""

from functools import partial
import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.sonic_navigation_graph import validate_sonic_navigation_graph
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import SonicNavigationSensorReceiver

assert G1.height_clearance is not None and G1.width_clearance is not None

# Same geometry as unitree_g1_nav_simple, without importing its stock-SDK onboard stack.
voxel_resolution = 0.05
g1_overhead_clearance = G1.height_clearance + 0.2
g1_max_step_height = 0.10
g1_rotation_diameter = 0.8
# The A* follower's 0.55 m/s nominal speed becomes the 0.2 m/s SONIC walk ceiling.
sonic_follower_nerf_speed = 0.2 / 0.55
# The real G1 turns in place at about 2-3 deg/s but about 14 deg/s while walking a curve
# (session 2026-10-19-11), so start walking below 45 deg of heading error. At 45 deg
# the follower's forward speed, 0.2 * (1 - 45/90), is the 0.1 m/s minimum walk.
sonic_initial_rotation_tolerance = math.pi / 4

unitree_g1_sonic_navigation_peer = (
    autoconnect(
        SonicNavigationSensorReceiver.blueprint(),
        PointLio.blueprint(),
        RayTracingVoxelMap.blueprint(voxel_size=voxel_resolution, worker_threads=2),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=voxel_resolution,
                can_pass_under=g1_overhead_clearance,
                can_climb=g1_max_step_height,
            ),
            initial_safe_radius_meters=0.0,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=G1.width_clearance,
            robot_rotation_diameter=g1_rotation_diameter,
            min_angular_velocity=0.0,
            initial_rotation_tolerance=sonic_initial_rotation_tolerance,
        ),
        MovementManager.blueprint(),
        SonicNavigationPeerModule.blueprint(),
    )
    .remappings(
        [
            # PointLio pose is in the sensor frame; the receiver publishes base odometry.
            (PointLio, "odometry", "lio_odometry"),
            # Merged-map input is disabled; only the monitored ray map feeds the costmap.
            (CostMapper, "merged_map", "sonic_navigation_merged_map_disabled"),
        ]
    )
    .global_config(
        n_workers=5,
        robot_model="unitree_g1",
        nerf_speed=sonic_follower_nerf_speed,
        simulation="",
        viewer="none",
    )
    .validators(partial(validate_sonic_navigation_graph, role="peer"))
)
