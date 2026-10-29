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

"""Whole goal cases through run_session, with the peer host and walk agent faked.

The fakes answer the same JSON ops as peer_host.py and walk_agent.py and write the
peer's telemetry file; the operator's answers are scripted.
"""

import builtins
from datetime import date
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.robot.unitree.g1.deploy.sonic_navigation.workstation import walk_harness as walk

TOKEN = f"operator-ready-{date.today():%Y%m%d}"
RES, ORIGIN, SIZE = 0.05, (-3.0, -3.0), 120


class World:
    """Shared fake state: the costmap, the robot pose and the navigation phase."""

    def __init__(self, out: Path, *, plans: bool, waypoints: list[tuple[float, float]]) -> None:
        self.out = out
        self.plans = plans
        self.waypoints = waypoints
        self.grid = np.zeros((SIZE, SIZE), dtype=np.int8)
        self.pose = [0.0, 0.0, 0.0]
        self.phase = "idle"
        self.ops: list[str] = []
        self.walking = False
        self.stop = threading.Event()
        threading.Thread(target=self._telemetry, daemon=True).start()

    def box(self, ahead: tuple[float, float], left: tuple[float, float]) -> None:
        rows, cols = np.indices(self.grid.shape)
        x = ORIGIN[0] + (cols + 0.5) * RES
        y = ORIGIN[1] + (rows + 0.5) * RES
        inside = (x > ahead[0]) & (x < ahead[1]) & (y > left[0]) & (y < left[1])
        self.grid[inside] = 100

    def value(self, x: float, y: float) -> int:
        return int(self.grid[int((y - ORIGIN[1]) // RES), int((x - ORIGIN[0]) // RES)])

    def _telemetry(self) -> None:
        with (self.out / "peer-telemetry.jsonl").open("a", buffering=1) as stream:
            while not self.stop.wait(0.05):
                now = time.monotonic()
                stream.write(json.dumps({"peer_monotonic": now, "odom": [*self.pose, now]}) + "\n")

    def walk_to_goal(self) -> None:
        if not self.plans:
            return  # the planner finds no path: the robot stands
        with (self.out / "peer-telemetry.jsonl").open("a", buffering=1) as stream:
            stream.write(json.dumps({"peer_monotonic": time.monotonic(), "path_poses": 20}) + "\n")
        self.walking = True

        def run() -> None:
            for wx, wy in self.waypoints:
                while self.walking and math.dist(self.pose[:2], (wx, wy)) > 0.02:
                    x, y = self.pose[:2]
                    d = math.dist((x, y), (wx, wy))
                    step = min(0.05, d)
                    self.pose[0] += step * (wx - x) / d
                    self.pose[1] += step * (wy - y) / d
                    time.sleep(0.05)

        threading.Thread(target=run, daemon=True).start()


class FakePeer:
    def __init__(self, world: World) -> None:
        self.world = world

    def read(self, timeout: float) -> dict[str, Any]:
        return {"op": "ready"}

    def call(self, op: str, timeout: float = 10.0, **fields: Any) -> dict[str, Any]:
        w = self.world
        w.ops.append(op)
        if op == "costmap":
            values = [w.value(x, y) for x, y in fields["points"]]
            summary = {"robot_cell": 0, "values": values, "occupied_within_0.5m": 0}
            return {"op": op, "costmap": summary}
        if op == "snapshot":
            np.savez(
                fields["path"], grid=w.grid, origin=np.array(ORIGIN), resolution=RES,
                robot=np.array([*w.pose[:2], 0.0, w.pose[2]]),
            )  # fmt: skip
            return {"op": op, "saved": True}
        if op == "refresh_map":
            return {"op": op, "region": {"x": w.pose[0], "y": w.pose[1], "radius": 2.5}}
        if op == "record_lidar":
            Path(fields["path"]).write_bytes(b"npz")
            return {"op": op, "scans": 10 * fields["seconds"], "points": 0, "poses": 0}
        return {"op": op}

    def close(self, op: str) -> None:
        self.world.stop.set()


class FakeAgent:
    def __init__(self, world: World) -> None:
        self.world = world

    def read(self, timeout: float) -> dict[str, Any]:
        return {"op": "attach", "result": True}

    def call(self, op: str, timeout: float = 10.0, **fields: Any) -> dict[str, Any]:
        w = self.world
        w.ops.append(op)
        now = time.monotonic()
        if op == "status":
            navigation = {"phase": w.phase, "pose_ready": w.phase == "idle"}
            return {"op": op, "ready": True, "feedback_fresh": True, "navigation": navigation}
        if op == "begin":
            w.phase = "active"
            return {"op": op, "result": {"phase": "active"}}
        if op == "goal":
            w.walk_to_goal()
            return {"op": op, "result": True}
        if op == "end":
            w.walking, w.phase = False, "idle"
            return {"op": op, "monotonic": now}
        if op == "since":
            rows = [
                {"monotonic": now, "feedback": True, "mode": mode, "latched": False,
                 "applied_sequence": i, "phase": "active"}
                for i, mode in enumerate((0, 1, 0))
            ]  # fmt: skip
            return {"op": op, "rows": rows}
        return {"op": op, "monotonic": now}

    def close(self, op: str) -> None:
        pass


def run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    answers: list[str],
    *,
    prior: tuple[str, ...],
    plans: bool = True,
    waypoints: list[tuple[float, float]] | None = None,
    box: bool = False,
) -> tuple[dict[str, Any], World]:
    earlier = tmp_path / "earlier"
    earlier.mkdir()
    cases = [{"name": n, "passed": True} for n in prior]
    (earlier / "results.json").write_text(json.dumps({"operator_ready": TOKEN, "cases": cases}))
    out = tmp_path / "session"
    worlds: list[World] = []

    def child(command: list[str], *args: Any, **kwargs: Any) -> Any:
        if command[0] == "peer":
            world = World(Path(command[1]), plans=plans, waypoints=waypoints or [(1.0, 0.0)])
            if box:
                world.box((1.1, 1.3), (-0.3, 0.3))
            worlds.append(world)
            time.sleep(0.15)  # a few odometry rows before the first case
            return FakePeer(world)
        return FakeAgent(worlds[0])

    def observe_link(path: Path, stop: threading.Event, latest: dict[str, Any]) -> None:
        while not stop.wait(0.05):
            latest.update(ready=True, monotonic=time.monotonic(), settled_for_s=1.0)

    script = iter(answers)

    def scripted(prompt: str = "") -> str:
        return next(script)  # StopIteration: the harness asked more than the script holds

    monkeypatch.setattr(walk, "peer_command", lambda out: ["peer", str(out)])
    monkeypatch.setattr(walk, "agent_command", lambda out: ["agent"])
    monkeypatch.setattr(walk, "JsonLines", child)
    monkeypatch.setattr(walk, "observe_link", observe_link)
    monkeypatch.setattr(walk, "REFRESH_SETTLE_S", 0.0)
    monkeypatch.setattr(walk, "PLAN_WAIT_S", 1.5)
    monkeypatch.setattr(builtins, "input", scripted)
    walk.run_session(
        SimpleNamespace(
            operator_ready=TOKEN,
            extrinsic_acceptance=walk.HERE / "extrinsic-acceptance.json",
            output=out,
            prior_session=[earlier],
            ignore_rear_occupancy=False,
        )
    )
    return json.loads((out / "results.json").read_text()), worlds[0]


CURVES = ("idle", "operator_stop", "forward_020", "curve_left_020", "curve_right_020")


def test_goal_1m_refreshes_the_map_before_the_pre_check_and_walks(tmp_path, monkeypatch) -> None:
    results, world = run(
        tmp_path, monkeypatch, ["goal_1m", "y", "walked 1 m", "y", "quit"], prior=CURVES
    )
    (record,) = results["cases"]
    assert record["passed"], record
    assert record["refresh"]["region"] is not None and record["reached_s"] is not None
    # Refresh, then the pre-check costmap, then begin and the goal.
    ops = [op for op in world.ops if op in ("refresh_map", "costmap", "begin", "goal")]
    assert ops == ["refresh_map", "costmap", "begin", "goal"]


def test_goal_without_clearance_is_refused_before_begin(tmp_path, monkeypatch) -> None:
    results, world = run(tmp_path, monkeypatch, ["goal_1m", "n", "quit"], prior=CURVES)
    (record,) = results["cases"]
    assert not record["passed"] and record["refresh"] is None
    assert record["error"] == "goal refused before begin: operator did not confirm clearance"
    assert "refresh_map" not in world.ops and "begin" not in world.ops


def test_goal_with_no_path_ends_after_the_plan_wait(tmp_path, monkeypatch) -> None:
    results, world = run(
        tmp_path,
        monkeypatch,
        ["goal_1m", "y", "stood still", "n", "quit"],
        prior=CURVES,
        plans=False,
    )
    (record,) = results["cases"]
    assert not record["passed"] and record["error"] == "the planner found no path to the goal"
    assert record["events"]["no_plan"] >= walk.PLAN_WAIT_S and record["reached_s"] is None
    assert "end" in world.ops  # the request was ended, the robot handed back


def test_goal_detour_sees_the_box_in_the_corridor_and_passes_clear(tmp_path, monkeypatch) -> None:
    results, _ = run(
        tmp_path,
        monkeypatch,
        ["goal_detour", "", "y", "went round the box", "y", "quit"],
        prior=(*CURVES, "goal_1m"),
        waypoints=[(1.2, 0.75), (2.0, 0.0)],
        box=True,
    )
    (record,) = results["cases"]
    # The fake peer publishes its plan before the goal's reply returns, as a real peer
    # may: the plan still counts (it was missed when timed from the reply).
    assert record["passed"], record
    assert "planned" in record["events"] and record["clearance_m"] >= walk.MIN_CLEARANCE_M


def test_record_lidar_records_a_window_without_moving_the_robot(tmp_path, monkeypatch) -> None:
    results, world = run(
        tmp_path, monkeypatch, ["record_lidar", "y", "stood 1 m ahead, then left", "quit"], prior=()
    )
    (record,) = results["cases"]
    assert record["passed"] and record["scans"] == 10 * walk.CASES["record_lidar"].seconds
    assert (tmp_path / "session" / record["recording"]).exists()
    assert record["note"] == "stood 1 m ahead, then left"
    assert "begin" not in world.ops and "goal" not in world.ops
