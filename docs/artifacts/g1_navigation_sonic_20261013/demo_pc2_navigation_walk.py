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
raises a speed, retries a refused begin or changes controller ownership. Every case
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
import select
import subprocess
import sys
import threading
import time
from typing import Any

from dimos.robot.unitree.g1.sonic_navigation_link import SonicNavigationLink

HERE = Path(__file__).resolve().parent
PC2 = "pc2"
CONTAINER = "dimos-g1-sonic-navigation-graph-20261013"
AGENT = "/opt/sonic-navigation/package/demo_pc2_walk_agent.py"
ENDPOINTS = {
    "router": "tcp/127.0.0.1:17211",
    "link_bind": "tcp://192.168.0.62:17103",
    "cloud": "tcp://192.168.0.223:17101",
    "state": "tcp://192.168.0.223:17102",
}
MIN_MOTION = 0.02  # m or rad: below this the robot "stayed in place"
IDLE_MOTION = 0.02
SETTLE_DWELL_S = 0.5
SETTLE_TIMEOUT_S = 3.0
CANCEL_IDLE_S = 0.5
PAUSE_IDLE_S = 0.6


@dataclass(frozen=True)
class Case:
    kind: str
    vx: float = 0.0
    wz: float = 0.0
    seconds: float = 0.0
    requires: tuple[str, ...] = ()


CASES = {
    "idle": Case("idle", seconds=3.0),
    "operator_stop": Case("operator_stop", requires=("idle",)),
    "forward_010": Case("motion", vx=0.10, seconds=1.0, requires=("idle", "operator_stop")),
    "forward_020": Case("motion", vx=0.20, seconds=2.0, requires=("forward_010",)),
    "turn_left_010": Case("motion", wz=0.10, seconds=1.0, requires=("forward_010",)),
    "turn_right_010": Case("motion", wz=-0.10, seconds=1.0, requires=("forward_010",)),
    "cancel": Case("cancel", vx=0.10, seconds=0.6, requires=("forward_010",)),
    "source_pause": Case("source_pause", vx=0.10, seconds=0.5, requires=("forward_010",)),
}


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
    command, measured = (case.vx, moved["forward_m"]) if case.vx else (case.wz, moved["yaw_rad"])
    signed = measured if command > 0 else -measured
    return MIN_MOTION <= signed <= 2 * abs(command) * case.seconds


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
    """A child process speaking JSON lines: commands on stdin, replies on stdout."""

    def __init__(self, command: list[str], log: Any, env: dict[str, str] | None = None) -> None:
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True, env=env
        )
        self._lock = threading.Lock()

    def read(self, timeout: float) -> dict[str, Any]:
        assert self.process.stdout is not None
        poller = select.poll()
        poller.register(self.process.stdout, select.POLLIN)
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if poller.poll(100):
                line = self.process.stdout.readline()
                if not line:
                    raise RuntimeError(f"{self.process.args[0]} exited")
                if line.startswith("{"):
                    return json.loads(line)  # type: ignore[no-any-return]
        raise TimeoutError(f"{self.process.args[0]} reply")

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
    script = HERE / "demo_navigation_two_host_peer.py"
    return [sys.executable, "-u", str(script), "--output", str(out), *endpoints]


def agent_command(out: Path) -> list[str]:
    log = f"/verification/walk-{out.name}.jsonl"
    return ["ssh", "-o", "BatchMode=yes", PC2, "docker", "exec", "-i", CONTAINER, "python", AGENT,
            "--log", log]  # fmt: skip


def run_session(args: Any) -> None:
    check_token(args.operator_ready)
    acceptance = json.loads(args.extrinsic_acceptance.read_text())
    if acceptance.get("accepted") is not True or acceptance.get("date") != f"{date.today()}":
        raise SystemExit("extrinsic acceptance must be accepted today by the operator")
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    root = HERE.parents[2]
    results: dict[str, Any] = {
        "operator_ready": args.operator_ready,
        "extrinsic_acceptance": acceptance,
        "endpoints": ENDPOINTS,
        "cases_offered": {name: asdict(case) for name, case in CASES.items()},
        "harness_sha256": {
            p.name: sha(p)
            for p in (
                HERE / "demo_pc2_navigation_walk.py",
                HERE / "demo_pc2_walk_agent.py",
                HERE / "demo_navigation_two_host_peer.py",
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
    peer = JsonLines(peer_command(out), log, env)
    ready = peer.read(120.0)
    assert ready["op"] == "ready", ready
    agent = JsonLines(agent_command(out), log)
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

    def tele(vx: float, wz: float) -> float:
        peer.call("tele", vx=vx, wz=wz)
        return time.monotonic()

    def observe(question: str) -> dict[str, Any]:
        note = input("Operator: describe what the robot did (independent observation): ")
        return {"note": note, "confirmed": input(f"{question} [y/n] ").strip().lower() == "y"}

    def run_case(name: str, case: Case) -> dict[str, Any]:
        started, mark = time.monotonic(), agent.call("mark")["monotonic"]
        record: dict[str, Any] = {"name": name, "case": asdict(case), "started_unix": time.time()}
        record["begin"] = agent.call("begin")
        if (
            "error" in record["begin"]
            or wait(lambda: status()["navigation"]["phase"] == "active", 1.5) is None
        ):
            tele(0.0, 0.0)
            record["error"] = "begin refused or no active phase; no motion was commanded"
            return record
        tele(0.0, 0.0)
        time.sleep(1.0)
        command_start = tele(case.vx, case.wz)
        if case.kind == "operator_stop":
            input("Press the native operator stop now, then Enter here: ")
            command_start = time.monotonic()
            time.sleep(3.0)
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
            moved = record["motion"]
            ok = (
                not evidence["walked"]
                and moved is not None
                and abs(moved["forward_m"]) < IDLE_MOTION
                and abs(moved["yaw_rad"]) < IDLE_MOTION
            )
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
            ok = (
                evidence["walked"]
                and evidence["ended_idle"]
                and not evidence["latched"]
                and motion_ok(case, record["motion"])
            )
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
                reason = blocked(name, results["cases"])
                print(f"  {name:15s} {'available' if reason is None else reason}")
            name = input("case (or quit)> ").strip()
            if name == "quit":
                break
            reason = blocked(name, results["cases"])
            if reason is not None:
                print(f"{name}: {reason}")
                continue
            if not (link_ready() and state.get("ready")):
                print("Not ready: the peer link and the execution owner must both be ready.")
                continue
            record = run_case(name, CASES[name])
            results["cases"].append(record)
            save()
            print(
                json.dumps({k: record.get(k) for k in ("name", "passed", "measured_ok", "motion")})
            )
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
    parser.add_argument("--extrinsic-acceptance", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run_session(parser.parse_args())


if __name__ == "__main__":
    main()
