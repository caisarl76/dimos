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

"""Live Rerun web view of the workstation peer: costmaps, voxel map, robot, path.

A separate read-only process. It joins the peer host's private zenoh router as a
client and subscribes to a few topics, so the validated peer graph is unchanged.
Start it beside a walk-harness session and open the URL it prints. A bare
http://localhost:9878 loads an unconnected viewer: rerun only passes the data
server to the page through `?url=`.

    python docs/artifacts/g1_navigation_sonic_20261013/demo_peer_rerun_view.py
"""

import argparse
import math
import signal
from typing import Any

import rerun as rr

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import g1_costmap
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEB_VIEWER_PORT

ROUTER = "tcp/127.0.0.1:17211"  # demo_pc2_navigation_walk.py peer router
TOPICS = ["global_costmap", "navigation_costmap", "global_map", "path", "odometry", "tf"]
# odom is the boot base_link (pelvis); the floor sits about one pelvis height below it.
FLOOR_Z = -0.74


def costmap(grid: Any) -> Any:
    return g1_costmap(grid, z_offset=FLOOR_Z + 0.02)


def robot_marker(odom: Odometry) -> Any:
    """The planner's footprint (G1 width clearance) and a heading arrow, on the floor."""
    assert G1.width_clearance is not None
    radius, z = G1.width_clearance / 2, FLOOR_Z + 0.05
    x, y, yaw = odom.x, odom.y, odom.yaw

    def at(r: float, a: float) -> list[float]:
        return [x + r * math.cos(a), y + r * math.sin(a), z]

    circle = [at(radius, 2 * math.pi * i / 32) for i in range(33)]
    tip = at(0.5, yaw)
    heading = [[x, y, z], tip]
    head = [at(0.38, yaw + 0.25), tip, at(0.38, yaw - 0.25)]
    return rr.LineStrips3D(
        [circle, heading, head], colors=[[0, 200, 255], [255, 200, 0], [255, 200, 0]], radii=0.015
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--router", default=ROUTER)
    parser.add_argument("--web-port", type=int, default=RERUN_WEB_VIEWER_PORT)
    args = parser.parse_args()
    bridge = RerunBridgeModule(
        pubsubs=[Zenoh(mode="client", connect=[args.router], scouting=False, multicast=False)],
        topics=TOPICS,
        visual_override={
            "world/global_costmap": costmap,
            "world/odometry": robot_marker,
            # The planner's inflated view; drawn just above the raw costmap.
            "world/navigation_costmap": lambda grid: g1_costmap(grid, z_offset=FLOOR_Z + 0.04),
        },
        max_hz={"world/global_map": 2.0},
        rerun_open="none",
        rerun_web=True,
        web_port=args.web_port,
    )
    bridge.start()
    data = f"rerun%2Bhttp://localhost:{RERUN_GRPC_PORT}/proxy"
    print(f"Rerun web viewer: http://localhost:{args.web_port}/?url={data}", flush=True)
    signal.signal(signal.SIGINT, lambda *_: None)
    signal.signal(signal.SIGTERM, lambda *_: None)
    signal.pause()
    bridge.stop()


if __name__ == "__main__":
    main()
