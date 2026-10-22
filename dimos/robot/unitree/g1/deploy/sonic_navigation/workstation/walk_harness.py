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

"""Supervised pc2 walk harness (Task 11), run on the workstation by the operator.

Preconditions, all outside this script: fresh operator readiness (standing, clear area,
tested native stop, the manipulation session released the robot), the task's native
controller and graph started with `launch.sh --commission`, and an accepted extrinsic.

It starts the validated workstation peer graph (the Task 9 peer host with the real
pc2 endpoints) and attaches the pc2 agent to the graph container. The operator picks
each bounded named case; a case unlocks only after its prerequisites passed in this
session. Teleop enters through MovementManager's tele_cmd_vel; the harness never
raises a speed or changes controller ownership, and retries a refused begin only once,
after a readiness lapse. Every case
records the commanded twist, the native guard state, the peer's LIO base odometry
and the operator's own observation.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import asdict, dataclass
from datetime import date
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from dimos.robot.unitree.g1.sonic_navigation_link import SonicNavigationLink

HERE = Path(__file__).resolve().parent
PC2 = "pc2"
CONTAINER = "dimos-g1-sonic-navigation-graph-20261013"
AGENT = "/opt/sonic-navigation/package/walk_agent.py"
ENDPOINTS = {
    "router": "tcp/127.0.0.1:17211",
    "link_bind": "tcp://192.168.0.62:17103",
    "cloud": "tcp://192.168.0.223:17101",
    "state": "tcp://192.168.0.223:17102",
}
MOUNT = "g1_tf_publisher.torso_to_mid360"  # what the peer host configures
MIN_MOTION = 0.02  # m or rad: below this the robot "stayed in place"
IDLE_POSITION_M = 0.02  # operator decision 2026-10-15: a one-time stance settle is IDLE
IDLE_YAW_RAD = math.radians(3.0)
SETTLE_DWELL_S = 0.5
SETTLE_TIMEOUT_S = 3.0
CANCEL_IDLE_S = 0.5
PAUSE_IDLE_S = 0.6
STOP_WAIT_S = 30.0
GOAL_M = 1.0
GOAL_TOLERANCE_M = 0.3


@dataclass(frozen=True)
class Case:
    kind: str
    vx: float = 0.0
    wz: float = 0.0
    seconds: float = 0.0
    requires: tuple[str, ...] = ()
    bearing: float = 0.0  # goal direction, rad left of the robot heading
    distance: float = GOAL_M  # goal distance along the bearing
    # Goal variants: "placed" (a box already across the path), "drop" (the operator
    # places it mid-walk), "teleop" (a teleop turn mid-goal).
    event: str = ""


# Startup scan (startup-blind-spot.md): the head frame and cables hide fixed wedges of
# the scan; turning the whole robot in place sweeps them over the surroundings.
SCAN_LEGS_DEG = (60.0, -60.0, 0.0)  # headings relative to the start, in order
SCAN_TOLERANCE_DEG = 5.0
SCAN_LEG_TIMEOUT_S = 75.0  # the robot turns in place at about 2-3 deg/s

CASES = {
    "idle": Case("idle", seconds=3.0),
    "operator_stop": Case("operator_stop", requires=("idle",)),
    "forward_010": Case("motion", vx=0.10, seconds=1.0, requires=("idle", "operator_stop")),
    # 0.10 m/s for 1 s applied but never stepped on pc2 (2026-10-15); the operator made the
    # simulation-verified 0.20 m/s for 2 s the first walking case.
    "forward_020": Case("motion", vx=0.20, seconds=2.0, requires=("idle", "operator_stop")),
    # 0.10 rad/s for 1 s barely turned (1.6 deg); the operator doubled the rate to the
    # 0.20 rad/s cap, for the 2 s simulation ran.
    "turn_left_020": Case("motion", wz=0.20, seconds=2.0, requires=("forward_020",)),
    "turn_right_020": Case("motion", wz=-0.20, seconds=2.0, requires=("forward_020",)),
    # 4 s so the gait is past start-up; right/left compare the simulation's right slowdown.
    "curve_left_020": Case("motion", vx=0.20, wz=0.20, seconds=4.0, requires=("forward_020",)),
    "curve_right_020": Case("motion", vx=0.20, wz=-0.20, seconds=4.0, requires=("forward_020",)),
    "cancel": Case("cancel", vx=0.10, seconds=0.6, requires=("forward_020",)),
    "source_pause": Case("source_pause", vx=0.10, seconds=0.5, requires=("forward_020",)),
    # Read-only: the live costmap around the robot, before any physical goal is designed.
    "costmap_check": Case("costmap"),
    # One short physical goal (plan Task 11), only after the robot has walked both curves
    # so the near field is observed; no cell is ever cleared to admit it.
    "goal_1m": Case("goal", seconds=20.0, requires=("curve_left_020", "curve_right_020")),
    # Starts 30 deg off the path: past the planner's default 20 deg in-place turn, inside
    # the SONIC peer's 45 deg walk-off tolerance.
    "goal_1m_left30": Case(
        "goal",
        seconds=20.0,
        requires=("curve_left_020", "curve_right_020"),
        bearing=math.radians(30),
    ),
    # Obstacle cases, after the simulation ones (two-host-sim/obstacles-4). The box must
    # be tall and wide enough to show in the costmap: about 40 cm tall, 40-60 cm wide.
    "startup_scan": Case("scan", wz=0.20, seconds=SCAN_LEG_TIMEOUT_S, requires=("operator_stop",)),
    "goal_detour": Case("goal", seconds=60.0, requires=("goal_1m",), distance=2.0, event="placed"),
    "goal_obstacle_appears": Case(
        "goal", seconds=60.0, requires=("goal_1m",), distance=2.0, event="drop"
    ),
    # MovementManager cancels the goal on any teleop message; the robot must stay put.
    "goal_teleop_cancel": Case(
        "goal", seconds=20.0, requires=("goal_1m",), distance=2.0, event="teleop"
    ),
}
# Base centre to the nearest box cell, about the G1 shoulder half-width (operator
# decision 2026-10-21; session 6 passed a box at 0.247 m without contact).
MIN_CLEARANCE_M = 0.20
# Probes along the path for a placed obstacle: 0.6-1.8 m, then the goal cell.
OBSTACLE_PROBES = [0.6 + 0.1 * i for i in range(13)]


def goal_pose(
    x: float, y: float, h: float, bearing: float, *, distance: float = GOAL_M
) -> tuple[tuple[float, float], float]:
    """The point `distance` away along `bearing` from heading `h`, and that direction."""
    yaw = h + bearing
    return (x + distance * math.cos(yaw), y + distance * math.sin(yaw)), yaw


def check_token(token: str) -> None:
    if token != f"operator-ready-{date.today():%Y%m%d}":
        raise SystemExit(f"needs today's operator token: operator-ready-{date.today():%Y%m%d}")


def blocked(name: str, results: list[dict[str, Any]]) -> str | None:
    """Why `name` may not run yet, or None. Physical goals are not harness cases."""
    if name not in CASES:
        return f"unknown case {name!r}"
    done = {r["name"] for r in results if r["passed"]}
    missing = [r for r in CASES[name].requires if r not in done]
    return f"requires {', '.join(missing)}" if missing else None


def prior_passed(sessions: list[Path], token: str) -> list[dict[str, Any]]:
    """Passed cases of earlier sessions under the same operator token (same day)."""
    cases = []
    for session in sessions:
        result = json.loads((session / "results.json").read_text())
        if result["operator_ready"] != token:
            raise SystemExit(f"{session} was run under {result['operator_ready']}, not {token}")
        cases += [{**c, "session": session.name} for c in result["cases"] if c["passed"]]
    return cases


BEGIN_RETRY_S = 0.3
# A Wi-Fi round-trip burst can leave pc2's readiness lease lapsed for a moment
# (2026-10-20 source_pause); the execution then refuses begin with this message.
TRANSIENT_BEGIN = "fresh navigation readiness required"


def begin_with_retry(
    call: Callable[[], dict[str, Any]], sleep: Callable[[float], None]
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """begin, once more after BEGIN_RETRY_S if refused for a readiness lapse only.

    Returns the reply and the refused first reply when it retried.
    """
    reply = call()
    if TRANSIENT_BEGIN not in str(reply.get("error", "")):
        return reply, None
    sleep(BEGIN_RETRY_S)
    return call(), reply


def goal_admissible(
    summary: dict[str, Any] | None, ignore_rear: bool = False, *, obstacle: bool = False
) -> str | None:
    """Why a goal along the probed cells may not be sent now, or None.

    ignore_rear (opt-in, for the support crane behind the robot) leaves occupied
    cells in the rear half of the 0.5 m disk out of the near-field check. With
    obstacle, the probes before the last must show the placed box and the last
    (the goal cell) must be free; otherwise every probe must be free.
    """
    if summary is None:
        return "no costmap yet"
    if summary.get("robot_cell") != 0:
        return f"robot cell not known free ({summary.get('robot_cell')})"
    values = summary.get("values", [None])
    if obstacle:
        if not any(v is not None and v >= 100 for v in values[:-1]):
            return f"placed obstacle not seen on the path ({values[:-1]})"
        if values[-1] != 0:
            return f"goal cell not known free ({values[-1]})"
    elif any(value != 0 for value in values):
        return f"cells 0.25-1.0 m ahead not all free ({values})"
    near = summary.get("occupied_within_0.5m", 1)
    if ignore_rear and near:
        ahead = near - summary.get("occupied_within_0.5m_behind", 0)
        if ahead:
            return f"occupied cells within 0.5 m ({ahead} ahead, rear ignored)"
        return None
    if near:
        return f"occupied cells within 0.5 m ({near})"
    return None


def unknown_by_bearing(
    grid: Any, origin: tuple[float, float], resolution: float, pose: tuple[float, float, float]
) -> dict[str, Any]:
    """Unknown cell share (%) by 30-degree bearing from the heading (from -180), per band."""
    cells = np.asarray(grid)
    rows, cols = np.indices(cells.shape)
    x, y, h = pose
    cx = origin[0] + (cols + 0.5) * resolution - x
    cy = origin[1] + (rows + 0.5) * resolution - y
    distance = np.hypot(cx, cy)
    bearing = np.degrees(np.remainder(np.arctan2(cy, cx) - h + math.pi, 2 * math.pi) - math.pi)
    out: dict[str, Any] = {"total": {}}
    for near, far in ((0.5, 1.0), (1.0, 2.0)):
        band = (distance >= near) & (distance < far)
        name = f"{near}-{far} m"
        out[name] = [
            round(100 * float((cells[mask] < 0).mean()))
            if (mask := band & (bearing >= b) & (bearing < b + 30)).any()
            else None
            for b in range(-180, 180, 30)
        ]
        out["total"][name] = round(100 * float((cells[band] < 0).mean()))
    return out


def occupied_ahead(
    grid: Any,
    origin: tuple[float, float],
    resolution: float,
    pose: tuple[float, float, float],
    *,
    ahead: tuple[float, float] = (0.3, 3.0),
    lateral: float = 1.5,
) -> Any:
    """Occupied cell centres in the region ahead of pose (x, y, heading).

    The support crane follows the robot, so it stays behind the start and its trail
    is mapped along the walked path afterwards: measure the box in a map taken
    before that, ahead of the robot.
    """
    rows, cols = np.nonzero(np.asarray(grid) >= 100)
    cells = np.stack(
        [origin[0] + (cols + 0.5) * resolution, origin[1] + (rows + 0.5) * resolution], axis=1
    )
    x, y, h = pose
    along = (cells[:, 0] - x) * math.cos(h) + (cells[:, 1] - y) * math.sin(h)
    across = -(cells[:, 0] - x) * math.sin(h) + (cells[:, 1] - y) * math.cos(h)
    return cells[(along >= ahead[0]) & (along <= ahead[1]) & (np.abs(across) <= lateral)]


def track_clearance(track: list[tuple[float, float]], cells: Any) -> float | None:
    """Closest approach of the base track to the given cell centres."""
    if not len(cells) or not track:
        return None
    distances, _ = cKDTree(cells).query(np.asarray(track, dtype=float))
    return float(np.min(distances))


def motion(rows: list[dict[str, Any]], t0: float, t1: float) -> dict[str, float] | None:
    """LIO base displacement over (t0, t1], expressed in the starting heading frame."""
    odom = [r for r in rows if "odom" in r]
    inside = [r for r in odom if t0 < r["peer_monotonic"] <= t1]
    if not inside:
        return None
    before = [r for r in odom if r["peer_monotonic"] <= t0]
    x0, y0, h0, _ = (before[-1] if before else inside[0])["odom"]
    x1, y1, h1, _ = inside[-1]["odom"]
    dx, dy = x1 - x0, y1 - y0
    forward = dx * math.cos(h0) + dy * math.sin(h0)
    lateral = -dx * math.sin(h0) + dy * math.cos(h0)
    return {
        "forward_m": forward,
        "lateral_m": lateral,
        "yaw_rad": math.remainder(h1 - h0, 2 * math.pi),
        # A forward command's sideways drift: the mount-yaw check statics cannot make.
        "heading_offset_deg": math.degrees(math.atan2(lateral, forward)),
    }


def motion_ok(case: Case, moved: dict[str, float] | None) -> bool:
    if moved is None:
        return False

    def within(command: float, measured: float) -> bool:
        signed = measured if command > 0 else -measured
        return MIN_MOTION <= signed <= 2 * abs(command) * case.seconds

    linear = not case.vx or within(case.vx, moved["forward_m"])
    turn = not case.wz or within(case.wz, moved["yaw_rad"])
    return linear and turn


def idle_ok(native: dict[str, Any], moved: dict[str, float] | None) -> bool:
    return (
        not native["walked"]
        and moved is not None
        and math.hypot(moved["forward_m"], moved["lateral_m"]) < IDLE_POSITION_M
        and abs(moved["yaw_rad"]) < IDLE_YAW_RAD
    )


def motion_case_ok(case: Case, native: dict[str, Any], moved: dict[str, float] | None) -> bool:
    """A pure turn is an IDLE facing change by design; only forward motion must walk."""
    return (
        (native["walked"] or not case.vx)
        and native["ended_idle"]
        and not native["latched"]
        and motion_ok(case, moved)
    )


def native_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    present = [r for r in rows if r["feedback"]]
    sequences = [r["applied_sequence"] for r in present]
    return {
        "samples": len(rows),
        "walked": any(r["mode"] == 1 for r in present),
        "ended_idle": bool(rows) and rows[-1]["feedback"] and rows[-1]["mode"] == 0,
        "latched": any(r["latched"] for r in present),
        "feedback_gaps": len(rows) - len(present),
        "applied_sequence_advance": max(sequences) - min(sequences) if sequences else 0,
    }


class JsonLines:
    """A child process speaking JSON lines: commands on stdin, replies on stdout.

    stdout is drained continuously: DimOS logs to stdout too (also from forked
    workers), and an undrained pipe blocked the peer host's replies (2026-10-19-5).
    """

    def __init__(
        self,
        command: list[str],
        log: Any,
        env: dict[str, str] | None = None,
        *,
        stdout_log: Path | None = None,
    ) -> None:
        self._name = Path(command[0]).name
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True, env=env
        )
        self._lock = threading.Lock()
        self._replies: queue.Queue[dict[str, Any] | None] = queue.Queue()
        threading.Thread(target=self._drain, args=(stdout_log,), daemon=True).start()

    def _drain(self, stdout_log: Path | None) -> None:
        assert self.process.stdout is not None
        sink = stdout_log.open("a", buffering=1) if stdout_log else None
        try:
            for line in self.process.stdout:
                if line.startswith("{"):
                    self._replies.put(json.loads(line))
                elif sink is not None:
                    sink.write(line)
        finally:
            self._replies.put(None)  # end of output: the child exited
            if sink is not None:
                sink.close()

    def read(self, timeout: float) -> dict[str, Any]:
        try:
            reply = self._replies.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"{self._name} reply") from None
        if reply is None:
            self._replies.put(None)
            raise RuntimeError(f"{self._name} exited")
        return reply

    def call(self, op: str, timeout: float = 10.0, **fields: Any) -> dict[str, Any]:
        with self._lock:
            assert self.process.stdin is not None
            self.process.stdin.write(json.dumps({"op": op, **fields}) + "\n")
            self.process.stdin.flush()
            reply = self.read(timeout)
        if reply.get("op") != op:
            raise RuntimeError(f"unexpected reply to {op}: {reply}")
        return reply

    def close(self, op: str) -> None:
        if self.process.poll() is None:
            try:
                assert self.process.stdin is not None
                self.process.stdin.write(json.dumps({"op": op}) + "\n")
                self.process.stdin.close()
                self.process.wait(timeout=20.0)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()


def observe_link(path: Path, stop: threading.Event, latest: dict[str, Any]) -> None:
    """Read-only status client on the peer's link: readiness, faults and settling."""
    with path.open("a", buffering=1) as log:
        while not stop.wait(0.2):
            try:
                with SonicNavigationLink(ENDPOINTS["link_bind"]) as link:
                    while not stop.wait(0.1):
                        result, t0, t1 = link.control("status", None)
                        snapshot = result["readiness"]
                        row = {
                            "monotonic": t1,
                            "rtt_s": t1 - t0,
                            "ready": snapshot.ready,
                            "fault": snapshot.fault,
                            "linear_speed_mps": result["linear_speed_mps"],
                            "yaw_rate_radps": result["yaw_rate_radps"],
                            "settled_for_s": result["settled_for_s"],
                            "inputs": {
                                p.name: [p.age, p.upstream_lag, p.valid] for p in snapshot.inputs
                            },
                        }
                        latest.update(row)
                        log.write(json.dumps(row) + "\n")
            except Exception as exc:  # peer restarting: record and retry
                latest.clear()
                log.write(json.dumps({"monotonic": time.monotonic(), "error": repr(exc)}) + "\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def peer_command(out: Path) -> list[str]:
    endpoints = (f"--{k.replace('_', '-')}={v}" for k, v in ENDPOINTS.items())
    script = HERE / "peer_host.py"
    return [sys.executable, "-u", str(script), "--output", str(out), *endpoints]


def agent_command(out: Path) -> list[str]:
    log = f"/verification/walk-{out.name}.jsonl"
    return ["ssh", "-o", "BatchMode=yes", PC2, "docker", "exec", "-i", CONTAINER, "python", AGENT,
            "--log", log]  # fmt: skip


def run_session(args: Any) -> None:
    check_token(args.operator_ready)
    acceptance = json.loads(args.extrinsic_acceptance.read_text())
    if acceptance.get("accepted") is not True or acceptance.get("mount") != MOUNT:
        raise SystemExit(f"the operator has not accepted the mount the peer uses ({MOUNT})")
    prior = prior_passed(args.prior_session, args.operator_ready)
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    root = HERE.parents[6]
    results: dict[str, Any] = {
        "operator_ready": args.operator_ready,
        "ignore_rear_occupancy": args.ignore_rear_occupancy,
        "extrinsic_acceptance": acceptance,
        "prior_passed": [{"session": c["session"], "name": c["name"]} for c in prior],
        "endpoints": ENDPOINTS,
        "cases_offered": {name: asdict(case) for name, case in CASES.items()},
        "harness_sha256": {
            p.name: sha(p)
            for p in (
                HERE / "walk_harness.py",
                HERE.parent / "pc2_tools/walk_agent.py",
                HERE / "peer_host.py",
            )
        },
        "dimos_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True
        ).stdout.strip(),
        "cases": [],
    }

    def save() -> None:
        (out / "results.json").write_text(json.dumps(results, indent=2, default=str) + "\n")

    save()
    log = (out / "harness.log").open("a")
    env = {**os.environ, "PYTHONPATH": f"{root}:{HERE}"}
    peer = JsonLines(peer_command(out), log, env, stdout_log=out / "peer-stdout.log")
    ready = peer.read(120.0)
    assert ready["op"] == "ready", ready
    agent = JsonLines(agent_command(out), log, stdout_log=out / "agent-stdout.log")
    results["agent_attach"] = agent.read(30.0)
    save()
    stop = threading.Event()
    link: dict[str, Any] = {}
    threading.Thread(
        target=observe_link, args=(out / "peer-readiness.jsonl", stop, link), daemon=True
    ).start()

    def status() -> dict[str, Any]:
        return agent.call("status")

    def wait(condition: Callable[[], bool], timeout: float) -> float | None:
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            if condition():
                return time.monotonic() - started
            time.sleep(0.05)
        return None

    def link_ready() -> bool:
        return link.get("ready") is True and time.monotonic() - link["monotonic"] < 0.5

    def settled() -> float | None:
        return wait(
            lambda: link_ready() and link["settled_for_s"] >= SETTLE_DWELL_S, SETTLE_TIMEOUT_S
        )

    def odometry(since: float) -> list[dict[str, Any]]:
        rows = []
        with (out / "peer-telemetry.jsonl").open() as stream:
            for line in stream:
                row = json.loads(line)
                if "odom" in row and row["peer_monotonic"] > since:
                    rows.append(row)
        return rows

    def latest_odom() -> list[float]:
        """Newest odometry row, read from the end of the telemetry file."""
        with (out / "peer-telemetry.jsonl").open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 65536))
            lines = stream.read().decode(errors="replace").splitlines()
        for line in reversed(lines[1:]):
            row = json.loads(line)
            if "odom" in row:
                return row["odom"]  # type: ignore[no-any-return]
        raise RuntimeError("no recent odometry")

    def costmap_record(
        name: str, bearing: float = 0.0, probes: list[float] | None = None, suffix: str = ""
    ) -> dict[str, Any]:
        """Read-only: no begin, no motion. Cells along `bearing` plus the peer summary."""
        x, y, h, _ = latest_odom()
        h += bearing
        ahead = [
            [x + d * math.cos(h), y + d * math.sin(h)] for d in probes or (0.25, 0.5, 0.75, 1.0)
        ]
        reply = peer.call("costmap", points=ahead)
        summary = reply["costmap"]
        snapshot = out / f"costmap-{len(results['cases']) + 1:02d}{suffix}.npz"
        saved = peer.call("snapshot", path=str(snapshot)).get("saved")
        record = {"name": name, "started_unix": time.time(), "ahead_points": ahead, **reply}
        record["snapshot"] = saved and snapshot.name
        record["passed"] = record["measured_ok"] = summary is not None
        if summary is not None:
            disk = summary.get("startup_disk", {})
            print(
                json.dumps(
                    {
                        "robot_cell": summary.get("robot_cell"),
                        "within_1.2m": {k: disk.get(k) for k in ("unknown", "free", "occupied")},
                        "probes_ahead": summary.get("values"),
                        "clear_goal": summary.get("clear_goal"),
                    }
                )
            )
        return record

    def goal_record(name: str, case: Case) -> dict[str, Any]:
        """Pre-checks, then one goal `case.distance` along the case bearing through
        set_navigation_goal, with the case's mid-goal event; cancel after."""
        if case.event == "placed":
            input(
                "Place a box (about 40 cm tall, 40-60 cm wide) about 1.2 m ahead, across "
                "the straight path, then press Enter. "
            )
        probes = [*OBSTACLE_PROBES, case.distance] if case.event == "placed" else None
        record = costmap_record(name, case.bearing, probes)
        record["passed"] = record["measured_ok"] = False
        reason = goal_admissible(
            record.get("costmap"),
            ignore_rear=args.ignore_rear_occupancy,
            obstacle=case.event == "placed",
        )
        clear = {
            "placed": f"Apart from the box, is the area {case.distance:.0f} m ahead clear?",
            "drop": f"Is the area {case.distance:.0f} m ahead clear, with a box in hand to "
            "place about 1 m ahead of the robot when prompted?",
        }.get(
            case.event,
            f"Is the area {case.distance:.0f} m ahead, {math.degrees(case.bearing):.0f} deg "
            "left, physically clear?",
        )
        if reason is None and input(f"{clear} [y/n] ").strip().lower() != "y":
            reason = "operator did not confirm clearance"
        if reason is not None:
            record["error"] = f"goal refused before begin: {reason}"
            record["begin"] = {"error": record["error"]}
            return record
        started, mark = time.monotonic(), agent.call("mark")["monotonic"]
        record["begin"], record["begin_refused_first"] = begin_with_retry(
            lambda: agent.call("begin"), time.sleep
        )
        if (
            "error" in record["begin"]
            or wait(lambda: status()["navigation"]["phase"] == "active", 1.5) is None
        ):
            tele(0.0, 0.0)
            record["error"] = "begin refused or no active phase; no goal was sent"
            return record
        tele(0.0, 0.0)
        time.sleep(1.0)
        x, y, h, _ = latest_odom()
        goal, goal_yaw = goal_pose(x, y, h, case.bearing, distance=case.distance)
        record["goal_xy"], record["start_xy"] = list(goal), [x, y]
        # A* commands reach cmd_vel only once teleop stops (MovementManager cooldown).
        peer.call("tele_stop")
        record["goal"] = agent.call("goal", x=goal[0], y=goal[1], yaw=goal_yaw)
        reached, events, goal_started = None, {}, time.monotonic()
        drop_pose: list[float] = []
        while record["goal"].get("result") is True:
            now = time.monotonic()
            if now - goal_started >= case.seconds:
                break
            position = latest_odom()[:2]
            if math.dist(position, goal) < GOAL_TOLERANCE_M:
                reached = now - goal_started
                break
            walked = math.dist(position, (x, y))
            if case.event == "drop" and "drop" not in events and walked >= 0.5:
                events["drop"] = now - goal_started
                drop_pose = latest_odom()[:3]
                print("\a>>> PLACE THE BOX NOW, about 1 m ahead of the robot, across its path <<<")
            if "drop" in events and "box_snapshot" not in events:
                # Once placed and mapped, before the crane trail reaches that area.
                if now - goal_started - events["drop"] >= 2.5:
                    events["box_snapshot"] = now - goal_started
                    box_map = out / f"costmap-{len(results['cases']) + 1:02d}-box.npz"
                    if peer.call("snapshot", path=str(box_map)).get("saved"):
                        record["snapshot_box"] = box_map.name
            if case.event == "teleop":
                # Teleop cancels the goal (MovementManager); the robot must then stay put.
                if "teleop_on" not in events and walked >= 0.3:
                    events["teleop_on"] = now - goal_started
                    tele(0.0, 0.2)
                elif "teleop_off" not in events and "teleop_on" in events:
                    if now - goal_started - events["teleop_on"] >= 2.0:
                        events["teleop_off"] = now - goal_started
                        peer.call("tele_stop")
                elif "teleop_off" in events and now - goal_started - events["teleop_off"] >= 3.0:
                    break
            time.sleep(0.05)
        record["reached_s"], record["events"] = reached, events
        record["cancel"] = agent.call("end")
        tele(0.0, 0.0)
        record["settle_s"] = settled()
        record["handback_s"] = wait(lambda: status()["navigation"]["pose_ready"], 6.0)
        record["native"] = native_summary(agent.call("since", monotonic=mark)["rows"])
        end = latest_odom()
        record["final_error_m"] = math.dist(end[:2], goal)
        unit = ((goal[0] - x) / case.distance, (goal[1] - y) / case.distance)
        rows = odometry(started)
        record["max_cross_track_m"] = max(
            (abs(unit[0] * (r["odom"][1] - y) - unit[1] * (r["odom"][0] - x)) for r in rows),
            default=0.0,
        )
        sound = not record["native"]["latched"] and record["settle_s"] is not None
        if case.event in ("placed", "drop"):
            # The box measured in a map from before the crane trail covers the path:
            # the pre-goal map for a placed box, the post-drop map for a dropped one.
            pose: tuple[float, float, float] | None
            if case.event == "placed":
                map_name, pose = record.get("snapshot"), (x, y, h)
            else:
                map_name = record.get("snapshot_box")
                pose = (drop_pose[0], drop_pose[1], drop_pose[2]) if drop_pose else None
            if map_name and pose is not None:
                with np.load(out / map_name) as snap:
                    cells = occupied_ahead(
                        snap["grid"],
                        (float(snap["origin"][0]), float(snap["origin"][1])),
                        float(snap["resolution"]),
                        pose,
                    )
                record["clearance_m"] = track_clearance(
                    [(r["odom"][0], r["odom"][1]) for r in rows], cells
                )
            clear_ok = (record.get("clearance_m") or 0.0) >= MIN_CLEARANCE_M
            if case.event == "placed":
                ok = reached is not None and clear_ok
            else:  # replanned and arrived, or stopped short: either way clear of the box
                ok = "drop" in events and clear_ok
            question = "Did the robot avoid the box without touching it?"
        elif case.event == "teleop":
            off = goal_started + events.get("teleop_off", math.inf)
            late = [r for r in rows if r["peer_monotonic"] >= off + 0.5]
            drift = math.dist(late[0]["odom"][:2], late[-1]["odom"][:2]) if late else math.inf
            record["moved_after_teleop_m"] = drift
            ok = "teleop_off" in events and reached is None and drift < 0.05
            question = "Did the robot turn under teleop, then stand still (goal cancelled)?"
        else:
            ok = reached is not None
            question = "Did the robot walk to the goal and stop there?"
        record["observation"] = observe(question)
        ok = ok and sound
        record["measured_ok"] = bool(ok)
        record["passed"] = bool(ok) and record["observation"]["confirmed"]
        return record

    def coverage(snapshot: str | None) -> dict[str, Any] | None:
        if not snapshot:
            return None
        with np.load(out / snapshot) as snap:
            x, y, _, h = (float(v) for v in snap["robot"])
            return unknown_by_bearing(
                snap["grid"],
                (float(snap["origin"][0]), float(snap["origin"][1])),
                float(snap["resolution"]),
                (x, y, h),
            )

    def scan_record(name: str, case: Case) -> dict[str, Any]:
        """Turn in place through SCAN_LEGS_DEG, closed on LIO yaw, then compare the map."""
        record = costmap_record(name)
        record["coverage_before"] = coverage(record.get("snapshot"))
        record["passed"] = record["measured_ok"] = False
        mark = agent.call("mark")["monotonic"]
        record["begin"], record["begin_refused_first"] = begin_with_retry(
            lambda: agent.call("begin"), time.sleep
        )
        if (
            "error" in record["begin"]
            or wait(lambda: status()["navigation"]["phase"] == "active", 1.5) is None
        ):
            tele(0.0, 0.0)
            record["error"] = "begin refused or no active phase; no motion was commanded"
            return record
        tele(0.0, 0.0)
        time.sleep(1.0)
        heading = latest_odom()[2]
        legs = []
        for target_deg in SCAN_LEGS_DEG:
            target = heading + math.radians(target_deg)
            yaw0 = latest_odom()[2]
            sign = 1.0 if math.remainder(target - yaw0, 2 * math.pi) > 0 else -1.0
            leg_start = tele(0.0, sign * case.wz)
            reached = None
            while time.monotonic() - leg_start < case.seconds:
                error = math.remainder(target - latest_odom()[2], 2 * math.pi)
                if sign * error <= math.radians(SCAN_TOLERANCE_DEG):
                    reached = time.monotonic() - leg_start
                    break
                time.sleep(0.05)
            tele(0.0, 0.0)
            time.sleep(1.5)  # the peer ramps the turn down; let the heading settle
            turned = math.degrees(math.remainder(latest_odom()[2] - yaw0, 2 * math.pi))
            legs.append(
                {
                    "target_deg": target_deg,
                    "reached_s": reached,
                    "turned_deg": turned,
                    "rate_dps": abs(turned) / reached if reached else None,
                }
            )
            print(json.dumps(legs[-1]))
            if reached is None:
                break
        record["legs"] = legs
        record["settle_s"] = settled()
        record["end"] = agent.call("end")
        record["handback_s"] = wait(lambda: status()["navigation"]["pose_ready"], 6.0)
        record["native"] = native_summary(agent.call("since", monotonic=mark)["rows"])
        after = costmap_record(name, suffix="-after")
        record["snapshot_after"] = after.get("snapshot")
        record["coverage_after"] = coverage(after.get("snapshot"))
        print(
            json.dumps(
                {
                    "unknown_before": (record["coverage_before"] or {}).get("total"),
                    "unknown_after": (record["coverage_after"] or {}).get("total"),
                }
            )
        )
        record["observation"] = observe("Did the robot turn left, right and back in place?")
        ok = (
            len(legs) == len(SCAN_LEGS_DEG)
            and all(leg["reached_s"] is not None for leg in legs)
            and not record["native"]["latched"]
            and record["settle_s"] is not None
        )
        record["measured_ok"] = bool(ok)
        record["passed"] = bool(ok) and record["observation"]["confirmed"]
        return record

    def tele(vx: float, wz: float) -> float:
        peer.call("tele", vx=vx, wz=wz)
        return time.monotonic()

    def observe(question: str) -> dict[str, Any]:
        note = input("Operator: describe what the robot did (independent observation): ")
        return {"note": note, "confirmed": input(f"{question} [y/n] ").strip().lower() == "y"}

    def run_case(name: str, case: Case) -> dict[str, Any]:
        started, mark = time.monotonic(), agent.call("mark")["monotonic"]
        record: dict[str, Any] = {"name": name, "case": asdict(case), "started_unix": time.time()}
        record["begin"], record["begin_refused_first"] = begin_with_retry(
            lambda: agent.call("begin"), time.sleep
        )
        if (
            "error" in record["begin"]
            or wait(lambda: status()["navigation"]["phase"] == "active", 1.5) is None
        ):
            tele(0.0, 0.0)
            record["error"] = "begin refused or no active phase; no motion was commanded"
            record["measured_ok"] = record["passed"] = False
            return record
        tele(0.0, 0.0)
        time.sleep(1.0)
        command_start = tele(case.vx, case.wz)
        if case.kind == "operator_stop":
            # Wait for the stop itself (session 2026-10-16-3 pressed it after a fixed window).
            print(f"Press the native operator stop (o) now; waiting up to {STOP_WAIT_S:.0f} s ...")
            command_start = time.monotonic()
            record["stop_observed_s"] = wait(
                lambda: not status().get("feedback_fresh", True), STOP_WAIT_S
            )
            time.sleep(1.0)
        elif case.kind == "cancel":
            time.sleep(case.seconds)
            record["cancel"] = agent.call("end")
            tele(0.0, 0.0)
        elif case.kind == "source_pause":
            time.sleep(case.seconds)
            peer.call("tele_stop")
            record["pause_mark"] = agent.call("mark")["monotonic"]
            time.sleep(1.0)
            tele(0.0, 0.0)
        else:
            time.sleep(case.seconds)
            tele(0.0, 0.0)
        command_end = time.monotonic()
        record["settle_s"] = settled()
        if case.kind != "cancel":
            record["end"] = agent.call("end")
        record["handback_s"] = wait(lambda: status()["navigation"]["pose_ready"], 6.0)
        native = agent.call("since", monotonic=mark)["rows"]
        record["native"] = native_summary(native)
        record["motion"] = motion(odometry(started), command_start - 1.0, time.monotonic())
        record["command_window_s"] = command_end - command_start
        evidence = record["native"]
        if case.kind == "idle":
            ok = idle_ok(evidence, record["motion"])
            question = "Did the robot stand still in IDLE?"
        elif case.kind == "operator_stop":
            ok = evidence["latched"] or evidence["feedback_gaps"] > 0
            question = "Did the native operator stop act, leaving the robot safe?"
        elif case.kind == "cancel":
            after = [r for r in native if r["monotonic"] >= record["cancel"]["monotonic"]]
            idle_at = next((r["monotonic"] for r in after if r["mode"] == 0), None)
            record["cancel_idle_s"] = (
                None if idle_at is None else idle_at - record["cancel"]["monotonic"]
            )
            ok = (
                "error" not in record["cancel"]
                and record["cancel_idle_s"] is not None
                and record["cancel_idle_s"] <= CANCEL_IDLE_S
                and not evidence["latched"]
            )
            question = "Did the robot stop walking at the cancel?"
        elif case.kind == "source_pause":
            after = [r for r in native if r["monotonic"] >= record["pause_mark"]]
            idle = next((r for r in after if r["mode"] == 0), None)
            record["pause_idle_s"] = (
                None if idle is None else idle["monotonic"] - record["pause_mark"]
            )
            ok = (
                idle is not None
                and record["pause_idle_s"] <= PAUSE_IDLE_S
                and idle["phase"] == "active"
                and not evidence["latched"]
            )
            question = "Did the robot stop walking when the source paused?"
        else:
            ok = motion_case_ok(case, evidence, record["motion"])
            question = "Did it move as commanded, then stop and stand?"
        if case.kind != "operator_stop":
            ok = ok and record["settle_s"] is not None and record["handback_s"] is not None
        record["observation"] = observe(question)
        record["measured_ok"] = bool(ok)
        record["passed"] = bool(ok) and record["observation"]["confirmed"]
        return record

    results["link_ready_after_s"] = wait(link_ready, 90.0)
    save()
    if results["link_ready_after_s"] is None:
        print("The peer link did not report ready within 90 s; see peer-readiness.jsonl.")
    try:
        while True:
            state = status()
            print(
                f"\nlink ready={link_ready()} execution ready={state.get('ready')} "
                f"phase={state.get('navigation', {}).get('phase')}"
            )
            for name in CASES:
                reason = blocked(name, prior + results["cases"])
                print(f"  {name:15s} {'available' if reason is None else reason}")
            name = input("case (or quit)> ").strip()
            if name == "quit":
                break
            reason = blocked(name, prior + results["cases"])
            if reason is not None:
                print(f"{name}: {reason}")
                continue
            if not (link_ready() and state.get("ready")):
                print("Not ready: the peer link and the execution owner must both be ready.")
                continue
            if CASES[name].kind == "costmap":
                record = costmap_record(name)
            elif CASES[name].kind == "goal":
                record = goal_record(name, CASES[name])
            elif CASES[name].kind == "scan":
                record = scan_record(name, CASES[name])
            else:
                record = run_case(name, CASES[name])
            results["cases"].append(record)
            save()
            summary = {k: record.get(k) for k in ("name", "passed", "measured_ok", "motion")}
            if "error" in record:
                summary["refused"] = record["begin"].get("error", record["error"])
            print(json.dumps(summary))
    finally:
        try:
            peer.call("tele", vx=0.0, wz=0.0)
            if status()["navigation"]["phase"] in ("active", "rearming"):
                results["teardown_end"] = agent.call("end")
                results["teardown_handback_s"] = wait(
                    lambda: status()["navigation"]["pose_ready"], 6.0
                )
        finally:
            stop.set()
            agent.close("quit")
            peer.close("stop")
            save()
            log.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-ready", required=True, help="operator-ready-YYYYMMDD (today)")
    parser.add_argument(
        "--extrinsic-acceptance", type=Path, default=HERE / "extrinsic-acceptance.json"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--prior-session",
        type=Path,
        action="append",
        default=[],
        help="earlier session today whose passed cases count as prerequisites",
    )
    parser.add_argument(
        "--ignore-rear-occupancy",
        action="store_true",
        help="goal_1m: ignore occupied cells behind the robot within 0.5 m (support crane)",
    )
    run_session(parser.parse_args())


if __name__ == "__main__":
    main()
