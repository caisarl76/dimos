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

"""Workstation host of the two-host simulation: the validated peer blueprint, unmodified.

Runs in its own process (and monotonic time namespace) with a private zenoh
router. stdin carries JSON control lines; stdout carries JSON replies only.
Teleop enters through MovementManager's existing tele_cmd_vel input. The torso
-> MID360 mount is the nominal URDF value, a simulation-only stand-in for the
commissioned extrinsic.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import numpy as np
import psutil
import zenoh

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_peer import (
    unitree_g1_sonic_navigation_peer,
)
from dimos.robot.unitree.g1.g1_tf_publisher import torso_to_mid360
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_readiness import INPUT_BUDGETS
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import SonicNavigationSensorReceiver

NATIVE_NAMES = ("pointlio_native", "voxel_ray_tracing")

if os.environ.get("SONIC_TWO_HOST_OUTPUT"):
    # Workers re-import this entry module: audit link control calls (behavior unchanged).
    import demo_navigation_audit

    demo_navigation_audit.install()


def reply(**fields: Any) -> None:
    sys.stdout.write(json.dumps(fields) + "\n")
    sys.stdout.flush()


def open_router(endpoint: str) -> Any:
    config = zenoh.Config()
    for key, value in {
        "mode": "router",
        "listen/endpoints": [endpoint],
        "connect/endpoints": [],
        "scouting/multicast/enabled": False,
        "scouting/gossip/enabled": True,
    }.items():
        config.insert_json5(key, json.dumps(value))
    return zenoh.open(config)


def costmap_summary(
    grid: OccupancyGrid, odom: Odometry | None, points: list[list[float]]
) -> dict[str, Any]:
    cells = np.asarray(grid.grid)
    summary: dict[str, Any] = {
        "stamp": grid.ts,
        "shape": list(cells.shape),
        "resolution": grid.resolution,
        "sha256": hashlib.sha256(cells.tobytes()).hexdigest(),
        "unknown": int((cells < 0).sum()),
        "free": int((cells == 0).sum()),
        "occupied": int((cells >= 100).sum()),
    }
    res, ox, oy = grid.resolution, grid.origin.position.x, grid.origin.position.y
    values = []
    for x, y in points:
        col, row = int((x - ox) // res), int((y - oy) // res)
        inside = 0 <= row < cells.shape[0] and 0 <= col < cells.shape[1]
        values.append(int(cells[row, col]) if inside else None)
    summary["values"] = values
    if odom is None:
        return summary
    rows, cols = np.indices(cells.shape)
    wx, wy = ox + (cols + 0.5) * res, oy + (rows + 0.5) * res
    dist = np.hypot(wx - odom.position.x, wy - odom.position.y)
    near = dist <= 1.2
    summary.update(
        robot_xy=[odom.position.x, odom.position.y],
        robot_yaw=odom.yaw,
        robot_cell=int(cells[np.unravel_index(np.argmin(dist), cells.shape)]),
        startup_disk={
            "radius_m": 1.2,
            "cells": int(near.sum()),
            "unknown": int((cells[near] < 0).sum()),
            "free": int((cells[near] == 0).sum()),
            "occupied": int((cells[near] >= 100).sum()),
        },
    )
    margin = math.ceil(0.35 / res)
    free = cells == 0
    padded = np.pad(free, margin, constant_values=False)
    clear = np.ones_like(free)
    for dr in range(-margin, margin + 1):
        for dc in range(-margin, margin + 1):
            clear &= padded[
                margin + dr : margin + dr + free.shape[0], margin + dc : margin + dc + free.shape[1]
            ]
    band = (dist >= 1.0) & (dist <= 2.5)
    for name, mask in (
        ("clear_goal", clear & band),
        ("unknown_goal", (cells < 0) & band),
        ("occupied_goal", (cells >= 100) & band),
    ):
        picks = np.argwhere(mask)
        if len(picks):
            r, c = picks[np.argmin(dist[mask])]
            summary[name] = [float(wx[r, c]), float(wy[r, c])]
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--router", required=True)
    parser.add_argument("--link-bind", required=True)
    parser.add_argument("--cloud", required=True)
    parser.add_argument("--state", required=True)
    args = parser.parse_args()
    router = open_router(args.router)
    mount = torso_to_mid360()
    t, q = mount.translation, mount.rotation
    blueprint = unitree_g1_sonic_navigation_peer
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={
            SonicNavigationSensorReceiver.name: {
                "cloud_endpoint": args.cloud,
                "state_endpoint": args.state,
                "torso_to_sensor": [t.x, t.y, t.z, q.x, q.y, q.z, q.w],
            },
            SonicNavigationPeerModule.name: {
                "link_bind": args.link_bind,
                "extrinsic_valid": True,
                "clock_domains": {name: "mid360" for name in INPUT_BUDGETS},
            },
            "g": {
                "transport": "zenoh",
                "viewer": "none",
                "robot_ip": None,
                "robot_ips": None,
                "zenoh_mode": "client",
                "zenoh_connect": args.router,
                "zenoh_multicast": False,
                "zenoh_scouting": False,
                "zenoh_gossip": True,
                "serve_coordinator_rpc": False,
            },
        },
    )
    lock = threading.Lock()
    latest: dict[str, Any] = {}
    counts: dict[str, int] = {}
    tele: list[Twist | None] = [None]
    telemetry = (args.output / "peer-telemetry.jsonl").open("a")
    subscriptions: list[Any] = []
    coordinator = None

    def receive(name: str, msg: Any) -> None:
        now = time.monotonic()
        with lock:
            latest[name] = msg
            counts[name] = counts.get(name, 0) + 1
            if name == "cmd_vel":
                telemetry.write(
                    json.dumps({"peer_monotonic": now, "cmd_vel": [msg.linear.x, msg.angular.z]})
                    + "\n"
                )
            elif name == "path":
                # An empty Path is the planner clearing after a rejected/cancelled goal.
                if msg.poses:
                    counts["plan"] = counts.get("plan", 0) + 1
                telemetry.write(
                    json.dumps({"peer_monotonic": now, "path_poses": len(msg.poses)}) + "\n"
                )

    try:
        coordinator = ModuleCoordinator.build(blueprint, parsed)
        for (name, kind), transport in coordinator.transports.items():
            if (name, kind) in {
                ("cmd_vel", Twist),
                ("global_costmap", OccupancyGrid),
                ("odometry", Odometry),
                ("path", NavPath),
                ("global_map", PointCloud2),
            }:
                subscriptions.append(transport.subscribe(lambda m, n=name: receive(n, m)))
        teleop = coordinator.transports[("tele_cmd_vel", Twist)]
        costmap_out = coordinator.transports[("global_costmap", OccupancyGrid)]
        stop = threading.Event()
        synthetic = threading.Event()

        def publish_synthetic() -> None:
            # Simulation-only stand-in for a frozen CostMapper: an all-free 6 m square
            # around the robot, stamped with the latest real map stamp so its progress
            # still tracks the real ray map. Production never clears cells.
            while synthetic.is_set() and not stop.wait(0.2):
                with lock:
                    cloud, odom = latest.get("global_map"), latest.get("odometry")
                if cloud is None or odom is None:
                    continue
                origin = Pose(position=Vector3(odom.position.x - 3.0, odom.position.y - 3.0, 0.0))
                costmap_out.broadcast(
                    None,
                    OccupancyGrid(
                        grid=np.zeros((120, 120), dtype=np.int8),
                        resolution=0.05,
                        origin=origin,
                        frame_id="odom",
                        ts=cloud.ts,
                    ),
                )

        def publish_teleop() -> None:
            while not stop.wait(0.1):
                value = tele[0]
                if value is not None:
                    teleop.broadcast(None, value)

        threading.Thread(target=publish_teleop, daemon=True).start()
        reply(op="ready", pid=psutil.Process().pid, monotonic=time.monotonic())
        for line in sys.stdin:
            command = json.loads(line)
            op = command["op"]
            if op == "tele":
                tele[0] = Twist(
                    linear=Vector3(command["vx"], 0, 0), angular=Vector3(0, 0, command["wz"])
                )
                reply(op=op)
            elif op == "tele_stop":
                tele[0] = None
                reply(op=op)
            elif op == "pids":
                manager = coordinator._managers["python"]
                natives = {}
                for child in psutil.Process().children(recursive=True):
                    try:
                        name = Path(child.cmdline()[0]).name
                    except (psutil.Error, IndexError):
                        continue
                    if name in NATIVE_NAMES:
                        natives[name] = child.pid
                reply(
                    op=op,
                    workers=[{"pid": w.pid, "modules": w.module_names} for w in manager.workers],
                    natives=natives,
                )
            elif op == "costmap":
                with lock:
                    grid, odom = latest.get("global_costmap"), latest.get("odometry")
                    seen = dict(counts)
                summary = (
                    None if grid is None else costmap_summary(grid, odom, command.get("points", []))
                )
                reply(op=op, counts=seen, costmap=summary)
            elif op == "synthetic_costmap":
                synthetic.set()
                threading.Thread(target=publish_synthetic, daemon=True).start()
                reply(op=op)
            elif op == "synthetic_stop":
                synthetic.clear()
                reply(op=op)
            elif op == "stop":
                reply(op=op)
                break
        stop.set()
    finally:
        for unsubscribe in subscriptions:
            unsubscribe()
        if coordinator is not None:
            coordinator.stop()
        telemetry.close()
        router.close()


if __name__ == "__main__":
    main()
