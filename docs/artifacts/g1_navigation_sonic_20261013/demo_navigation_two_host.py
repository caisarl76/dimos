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

"""Two-host stage: both validated blueprints, the real link and native SONIC physics.

Everything runs in one loopback-only namespace with a separate zenoh router per
"host". The pc2 graph (read-only sensor sender, relay, the one execution owner)
runs in this process's coordinator; the workstation peer graph runs in its own
process and monotonic time namespace. Only the private ZMQ link, the sensor
relay and the native endpoints cross between them, and the link, native
reference and native feedback paths pass through byte-level fault proxies.

Recorded MID360S payloads replay through the production sender's DDS reader
calls; they do not move with the simulated robot, so peer odometry/settling is
static. Physics settling is therefore measured independently here, and goal
tests prove acceptance/rejection, not goal reaching.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any

from demo_navigation_sim import PhysicsProof, native_command, native_env, yaw
import numpy as np
import psutil
import zmq

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_pc2 import (
    unitree_g1_sonic_navigation_pc2,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_link import SonicNavigationLink
from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import SonicNavigationSensorSender
from dimos.robot.unitree.g1.sonic_reference import encode_command

HERE = Path(__file__).resolve().parent
PORTS = {
    "reference": 18556,  # execution binds
    "reference_proxy": 18559,  # native connects here
    "feedback": 18557,  # native binds
    "feedback_proxy": 18558,  # execution connects here
    "cloud": 18601,
    "state": 18602,
    "link": 18603,  # peer binds
    "link_proxy": 18604,  # relay connects here
    "router_pc2": 18611,
    "router_peer": 18612,
}
PEER_CLOCK_OFFSETS = (10_000, 20_000, 30_000, 40_000)


def tcp(port: int) -> str:
    return f"tcp://127.0.0.1:{port}"


class FaultProxy:
    """Byte-level TCP forwarder: pass, fixed added delay, or hold (no delivery)."""

    def __init__(self, listen: int, target: int) -> None:
        self.target = target
        self.delay = 0.0
        self.hold = False
        self._stop = threading.Event()
        self._server = socket.create_server(("127.0.0.1", listen))
        self._server.settimeout(0.1)
        self._sockets: list[socket.socket] = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                client, _ = self._server.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            try:
                upstream = socket.create_connection(("127.0.0.1", self.target), timeout=1.0)
            except OSError:
                client.close()
                continue
            pair = (client, upstream)
            for s in pair:
                s.settimeout(None)  # connect timeout only; idle directions must not expire
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sockets.append(s)
            # The link opens one connection per request: release both ends once both
            # directions finish, or the forwarder exhausts descriptors within minutes.
            remaining = [2]
            lock = threading.Lock()

            def finished(
                pair: tuple[socket.socket, socket.socket] = pair,
                remaining: list[int] = remaining,
                lock: threading.Lock = lock,
            ) -> None:
                with lock:
                    remaining[0] -= 1
                    if remaining[0]:
                        return
                for s in pair:
                    s.close()
                    if s in self._sockets:
                        self._sockets.remove(s)

            for src, dst in pair, pair[::-1]:
                queue: deque[tuple[float, bytes]] = deque()
                ready = threading.Condition()
                threading.Thread(
                    target=self._read, args=(src, dst, queue, ready), daemon=True
                ).start()
                threading.Thread(
                    target=self._write, args=(dst, queue, ready, finished), daemon=True
                ).start()

    def _read(
        self, src: socket.socket, dst: socket.socket, queue: deque, ready: threading.Condition
    ) -> None:
        try:
            while data := src.recv(65536):
                with ready:
                    queue.append((time.monotonic(), data))
                    ready.notify()
        except OSError:
            pass
        with ready:
            queue.append((time.monotonic(), b""))
            ready.notify()

    def _write(
        self, dst: socket.socket, queue: deque, ready: threading.Condition, finished: Any
    ) -> None:
        try:
            self._forward(dst, queue, ready)
        finally:
            finished()

    def _forward(self, dst: socket.socket, queue: deque, ready: threading.Condition) -> None:
        while not self._stop.is_set():
            with ready:
                while not queue:
                    ready.wait(0.05)
                    if self._stop.is_set():
                        return
                received, data = queue[0]
                due = received + self.delay
                if self.hold or time.monotonic() < due:
                    ready.wait(0.005)
                    continue
                queue.popleft()
            try:
                if not data:
                    dst.shutdown(socket.SHUT_WR)
                    return
                dst.sendall(data)
            except OSError:
                return

    def close(self) -> None:
        self._stop.set()
        self._server.close()
        for s in self._sockets:
            s.close()


def open_router(port: int) -> Any:
    from demo_navigation_two_host_peer import open_router as peer_router

    return peer_router(f"tcp/127.0.0.1:{port}")


class PeerHost:
    """The workstation process, in its own monotonic clock namespace."""

    def __init__(self, output: Path, offset: int, log: Any) -> None:
        self.offset = offset
        self.process = subprocess.Popen(
            [
                "unshare",
                "--time",
                "--monotonic",
                str(offset),
                "--fork",
                "--kill-child",
                sys.executable,
                "-u",
                str(HERE / "demo_navigation_two_host_peer.py"),
                "--output",
                str(output),
                "--router",
                f"tcp/127.0.0.1:{PORTS['router_peer']}",
                "--link-bind",
                tcp(PORTS["link"]),
                "--cloud",
                tcp(PORTS["cloud"]),
                "--state",
                tcp(PORTS["state"]),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=log,
            text=True,
        )
        self.ready = self.read(120.0)
        assert self.ready["op"] == "ready", self.ready

    def read(self, timeout: float) -> dict[str, Any]:
        assert self.process.stdout is not None
        until = time.monotonic() + timeout
        poller = select.poll()
        poller.register(self.process.stdout, select.POLLIN)
        while time.monotonic() < until:
            if poller.poll(100):
                line = self.process.stdout.readline()
                if not line:
                    raise RuntimeError("peer host exited")
                if line.startswith("{"):
                    return json.loads(line)  # type: ignore[no-any-return]
        raise TimeoutError("peer host reply")

    def call(self, op: str, **fields: Any) -> dict[str, Any]:
        assert self.process.stdin is not None
        self.process.stdin.write(json.dumps({"op": op, **fields}) + "\n")
        self.process.stdin.flush()
        reply = self.read(10.0)
        assert reply["op"] == op, reply
        return reply

    def stop(self, *, kill: bool = False) -> None:
        if self.process.poll() is None:
            if kill:
                # The whole host dies: workers and native children too, not just the wrapper.
                tree = psutil.Process(self.process.pid).children(recursive=True)
                for proc in [psutil.Process(self.process.pid), *tree]:
                    try:
                        proc.kill()
                    except psutil.Error:
                        pass
                psutil.wait_procs(tree, timeout=5.0)
            else:
                try:
                    self.call("stop")
                except (OSError, RuntimeError, TimeoutError):
                    self.process.terminate()
            try:
                self.process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)


def observe_peer(
    output: Path, stop: threading.Event, scenario: list[str], latest: dict[str, Any]
) -> None:
    """Independent read-only status client (not via the fault proxy): per-input ages/lags."""
    with (output / "peer-readiness.jsonl").open("w") as log:
        while not stop.wait(0.2):
            try:
                with SonicNavigationLink(tcp(PORTS["link"])) as link:
                    while not stop.wait(0.2):
                        result, t0, t1 = link.control("status", None)
                        snapshot = result["readiness"]
                        latest.update(monotonic=t1, ready=snapshot.ready, fault=snapshot.fault)
                        log.write(
                            json.dumps(
                                {
                                    "monotonic": t1,
                                    "scenario": scenario[0],
                                    "rtt_s": t1 - t0,
                                    "ready": snapshot.ready,
                                    "fault": snapshot.fault,
                                    "peer_request": [
                                        result["request_id"],
                                        result["accepted"],
                                        result["cancelled"],
                                        result["state"],
                                    ],
                                    "settling": [
                                        result["linear_speed_mps"],
                                        result["yaw_rate_radps"],
                                        result["settled_for_s"],
                                    ],
                                    "inputs": {
                                        p.name: [round(p.age, 4), round(p.upstream_lag, 4), p.valid]
                                        for p in snapshot.inputs
                                    },
                                }
                            )
                            + "\n"
                        )
                        log.flush()
            except Exception as exc:  # peer down/restarting: record and retry
                log.write(json.dumps({"monotonic": time.monotonic(), "error": repr(exc)}) + "\n")


def build_pc2(output: Path) -> ModuleCoordinator:
    blueprint = unitree_g1_sonic_navigation_pc2
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={
            SonicNavigationSensorSender.name: {
                "cloud_bind": tcp(PORTS["cloud"]),
                "state_bind": tcp(PORTS["state"]),
            },
            SonicNavigationRelayModule.name: {"peer_endpoint": tcp(PORTS["link_proxy"])},
            SonicExecutionModule.name: {
                "reference_bind": tcp(PORTS["reference"]),
                "feedback_endpoint": tcp(PORTS["feedback_proxy"]),
                "profile": {
                    "name": "two-host-native",
                    "source_fps": 50,
                    "right_hand_order": "native",
                    "handback_policy": "native_planner",
                },
                "journal_dir": str(output / "sessions"),
                "navigation_enabled": True,
            },
            "g": {
                "transport": "zenoh",
                "viewer": "none",
                "robot_ip": None,
                "robot_ips": None,
                "zenoh_mode": "client",
                "zenoh_connect": f"tcp/127.0.0.1:{PORTS['router_pc2']}",
                "zenoh_multicast": False,
                "zenoh_scouting": False,
                "zenoh_gossip": True,
                "serve_coordinator_rpc": False,
            },
        },
    )
    return ModuleCoordinator.build(blueprint, parsed)


def worker_pid(coordinator: ModuleCoordinator, module: str) -> int:
    for worker in coordinator._managers["python"].workers:
        if module in worker.module_names:
            assert worker.pid is not None
            return worker.pid
    raise LookupError(module)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_two_host(args: Any) -> None:
    root = HERE.parents[2]
    PORTS.update(reference=args.reference_port, feedback=args.feedback_port)
    args.output.mkdir(parents=True, exist_ok=False)
    runtime = root / "build/sonic-navigation-runtime"
    binary = args.binary or runtime / "source/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    if not binary.resolve().is_relative_to(runtime.resolve()):
        raise ValueError("binary must belong to this private runtime")
    assets = runtime / "assets"
    expected = json.loads((HERE / "simulation-assets.json").read_text())
    for name, digest in expected["sha256"].items():
        assert sha(assets / name) == digest, name
    replay = HERE / "stamp-replay/dds-inputs.npz"
    os.environ["SONIC_TWO_HOST_OUTPUT"] = str(args.output)
    os.environ["SONIC_TWO_HOST_REPLAY"] = str(replay)
    # Workers re-import __main__, which installs the scoped finder from this variable.
    import demo_navigation_dds_finder

    demo_navigation_dds_finder.install()
    command = native_command(args, binary, assets, PORTS["feedback"])
    command[command.index("--zmq-port") + 1] = str(PORTS["reference_proxy"])
    env = native_env(args)
    natives = {
        name: root / "target/release" / name for name in ("pointlio_native", "voxel_ray_tracing")
    }
    shim_files = sorted((HERE / "two-host-dds-shim").rglob("*.py"))
    manifest = {
        "stage": args.stage,
        "command": command,
        "netns": os.readlink("/proc/self/ns/net"),
        "host_netns": args.host_netns,
        "interfaces": socket.if_nameindex(),
        "gpu": args.gpu,
        "ports": PORTS,
        "peer_clock_offsets_s": PEER_CLOCK_OFFSETS,
        "binary_sha256": sha(binary),
        "peer_native_sha256": {name: sha(path) for name, path in natives.items()},
        "replay_sha256": sha(replay),
        "harness_sha256": {
            name: sha(HERE / name)
            for name in (
                "demo_navigation_sim.py",
                "demo_navigation_two_host.py",
                "demo_navigation_two_host_peer.py",
                "demo_navigation_dds_finder.py",
                "demo_navigation_audit.py",
                "demo_analyze_navigation.py",
                "demo_navigation_sim_worker.py",
                "demo_navigation_timing.py",
            )
        },
        "dds_shim_sha256": {str(p.relative_to(HERE)): sha(p) for p in shim_files},
        "production_source_sha256": {
            name: sha(root / name)
            for name in (
                "dimos/robot/unitree/g1/sonic_navigation.py",
                "dimos/robot/unitree/g1/sonic_navigation_session.py",
                "dimos/robot/unitree/g1/sonic_execution_module.py",
                "dimos/robot/unitree/g1/sonic_navigation_link.py",
                "dimos/robot/unitree/g1/sonic_navigation_peer_module.py",
                "dimos/robot/unitree/g1/sonic_navigation_relay_module.py",
                "dimos/robot/unitree/g1/sonic_navigation_readiness.py",
                "dimos/robot/unitree/g1/sonic_navigation_sensor_module.py",
                "dimos/robot/unitree/g1/sonic_navigation_graph.py",
                "dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_sonic_navigation_peer.py",
                "dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_sonic_navigation_pc2.py",
                "dimos/navigation/go2/replanning_a_star/controllers.py",
            )
        },
        "asset_sha256": expected["sha256"],
        "scope": (
            "Both validated blueprints, the real link and sensor relay, one native controller and "
            "headless physics in one loopback namespace; recorded sensors do not follow the robot"
        ),
        "simulation_only": [
            "DDS reader stand-in replays recorded MID360S payloads; lowstate from physics",
            "nominal URDF torso->MID360 mount stands in for the commissioned extrinsic",
            "operator start command sent by a short-lived harness publisher before pc2 binds",
        ],
        "thresholds": {
            "linear_speed_mps": 0.03,
            "yaw_rate_radps": 0.03,
            "settle_seconds": 0.5,
            "settle_timeout": 3.0,
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    results: dict[str, Any] = {"status": "incomplete", "cases": [], "error": None, "load": {}}
    processes: list[subprocess.Popen[Any]] = []
    proof = PhysicsProof(args.output)
    proxies: dict[str, FaultProxy] = {}
    routers: list[Any] = []
    peer: PeerHost | None = None
    pc2: ModuleCoordinator | None = None
    execution: Any = None
    inputs_seen: list[dict[str, Any]] = []
    unsubscribe: list[Any] = []
    stopped: list[int] = []
    scenario = "startup"
    current = [scenario]  # shared with the read-only peer observer thread
    observer_stop = threading.Event()
    peer_seen: dict[str, Any] = {}  # latest read-only peer status; never the relay's control slot
    telemetry = (args.output / "telemetry.jsonl").open("w")
    refs = (args.output / "references.jsonl").open("w")
    inputs_log = (args.output / "inputs.jsonl").open("w")
    peer_log = (args.output / "peer.log").open("w")
    context = zmq.Context()
    observer = context.socket(zmq.SUB)
    observer.setsockopt(zmq.LINGER, 0)
    observer.setsockopt(zmq.SUBSCRIBE, b"")
    observer.connect(tcp(PORTS["reference"]))

    def case(name: str, passed: bool, **data: Any) -> None:
        results["cases"].append({"name": name, "passed": bool(passed), **data})
        assert passed, f"{name}: {data}"

    def nav() -> dict[str, Any]:
        return execution.get_navigation_status()  # type: ignore[no-any-return]

    def feedback() -> Any:
        return execution.get_feedback()

    last_telemetry = [0.0]

    def pump(seconds: float) -> None:
        current[0] = scenario
        until = time.monotonic() + seconds
        while True:
            now = time.monotonic()
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("owned simulator/controller exited; inspect logs")
            if (args.output / "physics-latest.json").exists():
                proof.update()
            if execution is not None and now - last_telemetry[0] >= 0.05:
                last_telemetry[0] = now
                state = feedback()
                telemetry.write(
                    json.dumps(
                        {
                            "monotonic": now,
                            "scenario": scenario,
                            "execution": execution.get_status(),
                            "feedback": None if state is None else asdict(state),
                        },
                        default=str,
                    )
                    + "\n"
                )
            for _ in range(64):
                if not observer.poll(0):
                    break
                packet = observer.recv()
                refs.write(
                    json.dumps(
                        {
                            "monotonic": now,
                            "topic": packet[:7].decode(errors="replace"),
                            "packet_hex": packet.hex(),
                        }
                    )
                    + "\n"
                )
            if now >= until:
                break
            time.sleep(0.01)

    def wait(condition: Any, timeout: float, name: str) -> float:
        started = time.monotonic()
        while not condition():
            if time.monotonic() - started >= timeout:
                raise TimeoutError(name)
            pump(0.02)
        return time.monotonic() - started

    def envelope() -> dict[str, Any]:
        row = proof.latest
        assert row and time.monotonic() - row["monotonic"] < 0.2, "fresh physics required"
        assert not row["band"] and row["automatic_resets"] == 0, "unsupported/no resets required"
        assert row["pelvis"][2] >= 0.5 and row["tilt_deg"] <= 10, "stable standing required"
        return row

    def settle(name: str) -> None:
        """Original gate (0.03 m/s, 0.03 rad/s, 0.5 s dwell, 3 s). A miss stays a failed
        case; the run then waits for real stillness so later cases start from rest."""
        started = time.monotonic()
        proof.settled_start = None

        def still() -> bool:
            return (
                proof.settled_start is not None
                and proof.latest["monotonic"] - proof.settled_start >= 0.5
            )

        try:
            wait(still, 3.0, name + " measured physics settling")
            passed = True
        except TimeoutError:
            passed = False
            wait(still, 6.0, name + " diagnostic stillness after a missed settle gate")
        envelope()
        results["cases"].append(
            {"name": name + "_settled", "passed": passed, "elapsed": time.monotonic() - started}
        )

    def tele(vx: float = 0.0, wz: float = 0.0) -> None:
        assert peer is not None
        peer.call("tele", vx=vx, wz=wz)

    def tele_stop() -> None:
        assert peer is not None
        peer.call("tele_stop")

    def recover(name: str) -> None:
        """Explicit operator recovery: stop input, settle, confirmed handback, new begin."""
        if nav()["phase"] in ("active", "rearming"):
            execution.end_navigation()
        tele(0.0, 0.0)
        wait(lambda: nav()["pose_ready"], 6.0, name + " confirmed cancel and handback")
        settle(name)
        wait(link_ready, 6.0, name + " link ready")
        execution.begin_navigation()
        wait(lambda: nav()["phase"] == "active", 1.5, name + " explicit rearm")

    def link_ready() -> bool:
        """The peer's own readiness over the link and a fresh ready atomic input on pc2."""
        latest = inputs_seen[-1] if inputs_seen else None
        return (
            latest is not None
            and latest["ready"]
            and time.monotonic() - latest["monotonic"] < 0.3
            and peer_seen.get("ready") is True
            and time.monotonic() - peer_seen["monotonic"] < 0.5
        )

    def relayed_ready() -> bool:
        # Execution readiness also requires an unlatched native guard; after a latch
        # only the explicit begin_navigation rearm clears it.
        return link_ready() and execution.get_status()["ready"]

    def moving() -> bool:
        state = feedback()
        return (
            state is not None and state.planner_guard is not None and state.planner_guard.mode == 1
        )

    def idle_unlatched() -> bool:
        state = feedback()
        return (
            state is not None
            and state.planner_guard is not None
            and state.planner_guard.mode == 0
            and not state.planner_guard.latched
        )

    def stop_pid(pid: int) -> None:
        os.kill(pid, signal.SIGSTOP)
        stopped.append(pid)

    def cont_pid(pid: int) -> None:
        os.kill(pid, signal.SIGCONT)
        stopped.remove(pid)

    def record_input(value: NavigationInput) -> None:
        now = time.monotonic()
        row = {
            "monotonic": now,
            "scenario": scenario,
            "source_sequence": value.source_sequence,
            "source_age_bound_s": now + 0.350 - value.velocity_deadline,
            "readiness_margin_s": value.readiness_deadline - now,
            "ready": value.ready,
            "fault": value.fault,
        }
        inputs_seen.append(row)
        inputs_log.write(json.dumps(row) + "\n")

    def attach_pc2() -> None:
        nonlocal pc2, execution
        pc2 = build_pc2(args.output)
        execution = pc2.get_instance(SonicExecutionModule)
        unsubscribe.append(
            pc2.transports[("navigation_input", NavigationInput)].subscribe(record_input)
        )

    def detach_pc2() -> None:
        nonlocal pc2, execution
        for undo in unsubscribe:
            undo()
        unsubscribe.clear()
        if pc2 is not None:
            pc2.stop()
        pc2 = execution = None

    def cpu_baseline() -> dict[str, list[psutil.Process]]:
        assert peer is not None
        me = psutil.Process()
        groups = {
            "sim": [psutil.Process(processes[0].pid)],
            "native": [psutil.Process(processes[1].pid)],
            "peer_host": [
                psutil.Process(peer.process.pid),
                *psutil.Process(peer.process.pid).children(recursive=True),
            ],
        }
        others = {p.pid for procs in groups.values() for p in procs}
        groups["pc2_host"] = [me] + [p for p in me.children(recursive=True) if p.pid not in others]
        for procs in groups.values():
            for p in procs:
                p.cpu_percent(None)
        return groups

    def cpu_read(groups: dict[str, list[psutil.Process]]) -> dict[str, float]:
        """Average CPU percent per host/process group since cpu_baseline()."""
        load = {}
        for key, procs in groups.items():
            total = 0.0
            for p in procs:
                try:
                    total += p.cpu_percent(None)
                except psutil.Error:
                    pass
            load[key] = round(total, 1)
        return load

    def deploy_offset() -> int:
        return (args.output / "deploy.log").stat().st_size

    def loop_timing(start: int) -> dict[str, Any]:
        text = (args.output / "deploy.log").read_bytes()[start:].decode(errors="replace")
        ages = {"lowstate_ms": [], "imu_ms": []}
        for line in text.splitlines():
            if line.startswith("Loop timing"):
                fields = dict(
                    part.strip().split(": ", 1)
                    for part in line[len("Loop timing - ") :].split(",")
                    if ": " in part
                )
                ages["lowstate_ms"].append(float(fields["LowState age"].rstrip("ms")))
                ages["imu_ms"].append(float(fields["IMU age"].rstrip("ms")))
        return {
            key: {
                "samples": len(values),
                "max": max(values) if values else None,
                "p95": float(np.percentile(values, 95)) if values else None,
                "over_50ms": sum(v > 50 for v in values),
            }
            for key, values in ages.items()
        }

    def command_ages(name: str) -> dict[str, Any]:
        """Conservative source age at the first pc2 receipt of each new emission."""
        ages, last = [], None
        for r in inputs_seen:
            if (
                r["scenario"] == name
                and r["ready"]
                and (last is None or r["source_sequence"] > last)
            ):
                ages.append(r["source_age_bound_s"])
            if r["scenario"] == name:
                last = r["source_sequence"]
        return {
            "samples": len(ages),
            "max_s": max(ages) if ages else None,
            "p95_s": float(np.percentile(ages, 95)) if ages else None,
        }

    try:
        with (
            (args.output / "sim.log").open("wb") as simlog,
            (args.output / "deploy.log").open("wb") as deploylog,
        ):
            sim_env = {k: v for k, v in env.items() if not k.startswith("SONIC_TWO_HOST")}
            processes.append(
                subprocess.Popen(
                    [
                        str(args.groot / ".venv_sim/bin/python"),
                        "-u",
                        str(HERE / "demo_navigation_sim_worker.py"),
                        "--output",
                        str(args.output),
                        "--host-netns",
                        args.host_netns,
                        "--pacing",
                        args.pacing,
                        "--max-seconds",
                        "600",
                    ],
                    cwd=args.groot,
                    env=sim_env,
                    stdout=simlog,
                    stderr=simlog,
                )
            )
            wait(lambda: (args.output / "sim-ready").exists(), 30.0, "simulator readiness")
            proxies["reference"] = FaultProxy(PORTS["reference_proxy"], PORTS["reference"])
            proxies["feedback"] = FaultProxy(PORTS["feedback_proxy"], PORTS["feedback"])
            proxies["link"] = FaultProxy(PORTS["link_proxy"], PORTS["link"])
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=args.output,
                    env=sim_env,
                    stdin=subprocess.PIPE,
                    stdout=deploylog,
                    stderr=deploylog,
                )
            )
            wait(
                lambda: "Init Done" in (args.output / "deploy.log").read_text(errors="replace"),
                120.0,
                "native initialization",
            )
            # Simulated operator start: a short-lived publisher, closed before pc2 binds.
            operator = context.socket(zmq.PUB)
            operator.setsockopt(zmq.LINGER, 0)
            operator.bind(tcp(PORTS["reference"]))
            started = time.monotonic()
            while "transitioning to CONTROL state" not in (args.output / "deploy.log").read_text(
                errors="replace"
            ):
                if time.monotonic() - started > 10.0:
                    raise TimeoutError("control initialization")
                operator.send(encode_command(start=True, stop=False, planner=True))
                pump(0.02)
            until = time.monotonic() + 4.0
            while time.monotonic() < until:
                operator.send(encode_command(start=False, stop=False, planner=True))
                pump(0.02)
            operator.close()
            # Start both hosts while the support band still holds the robot: process
            # startup on this shared workstation stalls physics steps (observed 130-270 ms).
            routers.append(open_router(PORTS["router_pc2"]))
            attach_pc2()
            assert pc2 is not None
            case(
                "pc2_graph_validated_before_startup",
                bool(unitree_g1_sonic_navigation_pc2.validator_checks),
                modules=sorted(
                    a.module.__name__ for a in unitree_g1_sonic_navigation_pc2.active_blueprints
                ),
            )
            peer = PeerHost(args.output, PEER_CLOCK_OFFSETS[0], peer_log)
            threading.Thread(
                target=observe_peer,
                args=(args.output, observer_stop, current, peer_seen),
                daemon=True,
            ).start()
            case(
                "peer_graph_validated_in_separate_clock",
                abs(peer.ready["monotonic"] - time.monotonic() - PEER_CLOCK_OFFSETS[0]) < 5.0,
                peer_monotonic=peer.ready["monotonic"],
                pc2_monotonic=time.monotonic(),
            )
            tele(0.0, 0.0)
            wait(
                lambda: execution.get_status()["subscriptions_ready"],
                10.0,
                "three native subscriptions",
            )
            ready_after = wait(relayed_ready, 90.0, "relayed sensor/map chain readiness")
            (args.output / "release").touch()
            pump(6.0)
            envelope()
            wait(relayed_ready, 6.0, "readiness after release")
            case(
                "relayed_chain_ready",
                execution.get_status()["ready"],
                elapsed=ready_after,
                peer_status=dict(peer_seen),
            )
            pids = peer.call("pids")
            (args.output / "peer-pids.json").write_text(json.dumps(pids, indent=2) + "\n")
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "native rearm")
            scenario = "initial_idle"
            pump(3.0)
            settle("initial_idle")

            for name, vx, wz in (
                ("forward", 0.2, 0.0),
                ("backward", -0.2, 0.0),
                ("left_turn", 0.0, 0.2),
                ("right_turn", 0.0, -0.2),
            ):
                before = envelope()
                scenario = name
                tele(vx, wz)
                pump(2.0)
                after = envelope()
                tele(0.0, 0.0)
                heading = yaw(before["qpos"][3:7])
                dx = after["pelvis"][0] - before["pelvis"][0]
                dy = after["pelvis"][1] - before["pelvis"][1]
                forward = dx * math.cos(heading) + dy * math.sin(heading)
                angle = math.remainder(yaw(after["qpos"][3:7]) - heading, 2 * math.pi)
                case(
                    name,
                    forward * vx > 0.001 if vx else angle * wz > 0.001,
                    forward_displacement_m=forward,
                    yaw_change_rad=angle,
                    command_age=command_ages(name),
                )
                settle(name)

            scenario = "source_pause"
            tele(0.2, 0.0)
            pump(0.5)
            tele_stop()
            pump(0.25)
            case("source_pause_250ms", nav()["phase"] == "active", phase=nav()["phase"])
            elapsed = wait(idle_unlatched, 0.6, "source expiry IDLE")
            case(
                "source_expiry_idle_without_latch",
                nav()["phase"] == "active" and feedback().planner_guard.lease_armed,
                after_pause_s=0.25 + elapsed,
            )
            tele(0.0, 0.0)
            settle("source_expiry")
            scenario = "source_outage"
            tele_stop()
            pump(2.7)
            case(
                "source_outage_ends",
                nav()["phase"] in ("stopping", "disarmed"),
                phase=nav()["phase"],
            )
            recover("source_outage")

            # The link opens a fresh DEALER connection per request, so its application
            # round trip includes connection setup: a multiple of the network RTT.
            scenario = "link_delay_20ms"
            tele(0.2, 0.0)
            pump(0.5)
            proxies["link"].delay = 0.010  # each direction
            pump(1.5)
            case(
                "link_delay_20ms_rtt_moving",
                moving() and nav()["phase"] == "active",
                command_age=command_ages(scenario),
            )
            scenario = "link_delay_150ms"
            proxies["link"].delay = 0.075
            elapsed = wait(lambda: not moving(), 1.0, "150 ms network RTT stops motion")
            case(
                "link_delay_150ms_rtt_stops_motion",
                elapsed <= 0.45,
                elapsed=elapsed,
                phase=nav()["phase"],
                fault=nav()["fault"],
            )
            proxies["link"].delay = 0.0
            tele(0.0, 0.0)
            settle("link_delay")
            recover("link_delay")
            scenario = "link_hold_300ms"
            tele(0.2, 0.0)
            pump(0.5)
            proxies["link"].hold = True
            elapsed = wait(idle_unlatched, 0.6, "blocked replies reach IDLE")
            pump(max(0.0, 0.3 - elapsed))
            proxies["link"].hold = False
            case(
                "link_hold_300ms_idle_without_latch",
                not feedback().planner_guard.latched,
                elapsed=elapsed,
            )
            tele(0.0, 0.0)
            settle("link_hold")
            recover("link_hold")
            scenario = "link_hold_2500ms"
            tele(0.2, 0.0)
            pump(0.5)
            proxies["link"].hold = True
            pump(2.5)
            case(
                "link_hold_2500ms_ends",
                nav()["phase"] in ("stopping", "disarmed", "fault"),
                phase=nav()["phase"],
            )
            proxies["link"].hold = False
            recover("link_hold_2500ms")

            peer_pids = json.loads((args.output / "peer-pids.json").read_text())
            costmap_worker = next(
                w["pid"] for w in peer_pids["workers"] if "CostMapper" in w["modules"]
            )
            freezes = {
                "lidar_raw": ("file", "lidar_raw"),
                "imu_raw": ("file", "imu_raw"),
                "waist_tf": ("file", "waist_tf"),
                "lio": ("pid", peer_pids["natives"]["pointlio_native"]),
                "map": ("pid", peer_pids["natives"]["voxel_ray_tracing"]),
                "costmap_worker": ("pid", costmap_worker),
            }
            for stage, (kind, target) in freezes.items():
                scenario = f"freeze_{stage}"
                tele(0.2, 0.0)
                pump(0.5)
                assert moving(), stage
                frozen_at = time.monotonic()
                if kind == "file":
                    (args.output / f"freeze-{target}").touch()
                else:
                    stop_pid(target)
                try:
                    elapsed = wait(
                        lambda: nav()["phase"] != "active" and idle_unlatched(),
                        2.5,
                        f"{stage} freeze closes admission",
                    )
                    case(
                        f"freeze_{stage}_closes_admission",
                        True,
                        elapsed=elapsed,
                        phase=nav()["phase"],
                        fresh_teleop=True,
                        co_located=[
                            w["modules"] for w in peer_pids["workers"] if w["pid"] == target
                        ],
                    )
                finally:
                    if kind == "file":
                        (args.output / f"freeze-{target}").unlink()
                    else:
                        cont_pid(target)
                case(
                    f"freeze_{stage}_stays_closed_while_teleop_fresh",
                    nav()["phase"] != "active",
                    after_s=time.monotonic() - frozen_at,
                )
                recover(f"freeze_{stage}")

            scenario = "goals"
            before = peer.call("costmap")["costmap"]
            (args.output / "costmap-before.json").write_text(json.dumps(before, indent=2) + "\n")
            for kind in ("unknown_goal", "occupied_goal", "clear_goal"):
                if kind not in before:
                    case(f"{kind}_available", False, costmap=before)
                counts = peer.call("costmap")["counts"]
                request_id = nav()["request_id"]
                x, y = before[kind]
                goal = PoseStamped(
                    position=Vector3(x, y, 0.0), orientation=Quaternion(w=1.0), frame_id="odom"
                )
                tele_stop()
                try:
                    accepted = execution.set_navigation_goal(request_id, goal)
                except Exception as exc:
                    accepted = f"{type(exc).__name__}: {exc}"
                pump(3.0)
                after = peer.call("costmap", points=[[x, y]])
                gained = {
                    key: after["counts"].get(key, 0) - counts.get(key, 0)
                    for key in ("cmd_vel", "plan")
                }
                if kind == "clear_goal":
                    # Recorded near-field cells around the robot stay unknown (no startup
                    # disk clearing), so a free goal is still unplannable from here.
                    case(
                        "clear_goal_on_recorded_map_rejected_from_unknown_start",
                        gained["plan"] == 0 and not moving() and before["robot_cell"] < 0,
                        goal=[x, y],
                        accepted=accepted,
                        gained=gained,
                        robot_cell=before["robot_cell"],
                    )
                else:
                    value = after["costmap"]["values"][0]
                    disk = after["costmap"]["startup_disk"]
                    case(
                        f"{kind}_rejected_retains_cells",
                        gained["plan"] == 0
                        and not moving()
                        and value is not None
                        and (value < 0 if kind == "unknown_goal" else value >= 100)
                        and disk["unknown"] + disk["occupied"] > 0,
                        goal=[x, y],
                        accepted=accepted,
                        gained=gained,
                        goal_cell_after=value,
                        startup_disk_before=before.get("startup_disk"),
                        startup_disk_after=disk,
                        robot_cell=before.get("robot_cell"),
                    )
                recover(kind)
            scenario = "synthetic_clear_goal"
            stop_pid(costmap_worker)
            try:
                peer.call("synthetic_costmap")
                pump(0.6)  # zero teleop keeps the source fresh while the stand-in map takes over
                counts = peer.call("costmap")["counts"]
                robot = before["robot_xy"]
                goal = PoseStamped(
                    position=Vector3(
                        robot[0] + 1.5 * math.cos(before["robot_yaw"]),
                        robot[1] + 1.5 * math.sin(before["robot_yaw"]),
                        0.0,
                    ),
                    orientation=Quaternion(w=1.0),
                    frame_id="odom",
                )
                start_pose = envelope()
                # Goal immediately after the last teleop: A* commands pass once the 1 s
                # MovementManager teleop cooldown ends, inside the 2 s outage bound.
                tele_stop()
                accepted = execution.set_navigation_goal(nav()["request_id"], goal)
                elapsed = wait(moving, 3.0, "A* motion through the link")
                pump(1.5)
                after = peer.call("costmap")
                gained = {
                    k: after["counts"].get(k, 0) - counts.get(k, 0) for k in ("cmd_vel", "plan")
                }
                end_pose = envelope()
                heading = yaw(start_pose["qpos"][3:7])
                forward = (end_pose["pelvis"][0] - start_pose["pelvis"][0]) * math.cos(heading) + (
                    end_pose["pelvis"][1] - start_pose["pelvis"][1]
                ) * math.sin(heading)
                case(
                    "synthetic_clear_goal_followed_through_link",
                    accepted is True
                    and gained["plan"] > 0
                    and gained["cmd_vel"] > 0
                    and forward > 0.001,
                    elapsed_to_motion=elapsed,
                    gained=gained,
                    forward_displacement_m=forward,
                    command_age=command_ages(scenario),
                    note="sim-only synthetic all-free costmap replaces frozen CostMapper output",
                )
                cancel_started = time.monotonic()
                execution.end_navigation()
                cancelled = wait(lambda: nav()["pose_ready"] and not moving(), 6.0, "goal cancel")
                case(
                    "goal_cancel_reaches_public_planner",
                    True,
                    elapsed=cancelled,
                    since_request=time.monotonic() - cancel_started,
                )
            finally:
                peer.call("synthetic_stop")
                cont_pid(costmap_worker)
            settle("synthetic_clear_goal")
            # The resumed CostMapper republishes an older stamp than the synthetic maps:
            # the peer must latch the rollback and stay unready until a fresh instance.
            scenario = "costmap_rollback"
            latched = wait(
                lambda: str(peer_seen.get("fault", "")).startswith("source_stamp_rollback"),
                5.0,
                "peer rollback latch",
            )
            pump(1.0)
            try:
                execution.begin_navigation()
                refused = None
            except Exception as exc:
                refused = f"{type(exc).__name__}: {exc}"
            case(
                "stamp_rollback_latches_peer_until_fresh_instance",
                refused is not None and nav()["phase"] != "active",
                fault=peer_seen.get("fault"),
                latched_after_s=latched,
                begin_refused=refused,
            )
            peer.stop(kill=True)
            peer = PeerHost(args.output, PEER_CLOCK_OFFSETS[1], peer_log)
            peer.call("tele", vx=0.0, wz=0.0)
            elapsed = wait(relayed_ready, 90.0, "fresh peer commissioning")
            case("fresh_peer_instance_recovers_readiness", True, elapsed=elapsed)
            peer_pids = peer.call("pids")  # never signal a previous instance's PIDs
            (args.output / "peer-pids-fresh.json").write_text(
                json.dumps(peer_pids, indent=2) + "\n"
            )
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "fresh peer rearm")
            case(
                "no_startup_disk_clearing",
                before["startup_disk"]["unknown"] + before["startup_disk"]["occupied"] > 0,
                startup_disk=before["startup_disk"],
            )

            scenario = "cancel_stall"
            planner_worker = next(
                w["pid"] for w in peer_pids["workers"] if "ReplanningAStarPlanner" in w["modules"]
            )
            tele(0.2, 0.0)
            pump(0.5)
            stop_pid(planner_worker)
            try:
                execution.end_navigation()
                closed = nav()["phase"] != "active"
                pump(0.3)
                try:
                    execution.start_task()
                    pose_refused = False
                except Exception as exc:
                    pose_refused = True
                    refusal = f"{type(exc).__name__}: {exc}"
                case(
                    "cancel_stall_blocks_pose",
                    closed and pose_refused and not nav()["pose_ready"],
                    refusal=refusal if pose_refused else None,
                    stalled_worker_modules=[
                        w["modules"] for w in peer_pids["workers"] if w["pid"] == planner_worker
                    ],
                )
            finally:
                cont_pid(planner_worker)
            tele(0.0, 0.0)
            elapsed = wait(lambda: nav()["pose_ready"], 6.0, "stalled cancel resolves")
            case("cancel_resolves_after_stall", True, elapsed=elapsed)
            settle("cancel_stall")
            wait(relayed_ready, 6.0, "ready after stall")
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "stall rearm")

            scenario = "native_watchdog"
            tele(0.2, 0.0)
            pump(0.5)
            proxies["reference"].hold = True
            elapsed = wait(lambda: feedback().planner_guard.latched, 1.0, "native watchdog latch")
            proxies["reference"].hold = False
            case("native_watchdog_250ms", feedback().planner_guard.mode == 0, elapsed=elapsed)
            recover("native_watchdog")

            scenario = "feedback_loss"
            proxies["feedback"].hold = True
            pump(0.35)
            fault = nav()["fault"]
            proxies["feedback"].hold = False
            case("feedback_loss_200ms", fault == "feedback_lost", fault=fault)
            recover("feedback_loss")

            scenario = "seeded_pose"
            tele(0.0, 0.0)
            execution.end_navigation()
            wait(lambda: nav()["pose_ready"], 6.0, "cancel before pose")
            settle("before_pose")
            wait(relayed_ready, 6.0, "ready before pose")
            log_start = deploy_offset()
            task = execution.start_task()
            wait(
                lambda: execution.get_status()["task"]["phase"] == "waiting_for_chunk",
                2.0,
                "pose activation",
            )
            began = time.monotonic()
            chunk = 0
            groups = cpu_baseline()
            while time.monotonic() - began < 1.5:
                state = feedback()
                action = np.asarray(state.token_state + state.left_hand_q + state.right_hand_q)
                execution.submit_chunk(
                    task["task_id"], chunk, state.received_monotonic, np.tile(action, (50, 1))
                )
                chunk += 1
                pump(0.15)
                assert not feedback().planner_guard.latched
            load = cpu_read(groups)
            case(
                "seeded_pose_suspends_planner_watchdog",
                nav()["phase"] == "pose",
                duration=time.monotonic() - began,
            )
            results["load"]["vla_phase"] = {
                "cpu_percent": load,
                "native_loop": loop_timing(log_start),
                "sensor_relay_active": True,
            }
            execution.cancel_task(task["task_id"])
            wait(lambda: nav()["phase"] == "disarmed", 4.0, "confirmed pose handback")
            settle("pose_handback")
            wait(relayed_ready, 6.0, "ready after handback")
            tele_stop()
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "post-handback rearm")
            pump(1.0)
            case(
                "no_old_target_after_handback",
                not moving() and proof.latest["linear_speed_mps"] <= 0.03,
                speed=proof.latest["linear_speed_mps"],
            )

            scenario = "pose_expiry"
            execution.end_navigation()
            wait(lambda: nav()["pose_ready"], 6.0, "cancel before pose expiry")
            wait(relayed_ready, 6.0, "ready before pose expiry")
            task = execution.start_task()
            wait(
                lambda: execution.get_status()["task"]["phase"] == "waiting_for_chunk",
                2.0,
                "expiry pose activation",
            )
            state = feedback()
            action = np.asarray(state.token_state + state.left_hand_q + state.right_hand_q)
            execution.submit_chunk(
                task["task_id"], 0, state.received_monotonic, np.tile(action, (50, 1))
            )
            pump(0.1)
            proxies["reference"].hold = True
            elapsed = wait(
                lambda: feedback().reference_guard.expired, 0.6, "native 200ms pose expiry"
            )
            case(
                "pose_source_expiry_200ms",
                not feedback().planner_guard.latched,
                elapsed=elapsed,
                native_age_ms=feedback().reference_guard.age_ms,
            )
            proxies["reference"].hold = False
            wait(
                lambda: execution.get_status()["task"]["phase"] == "done"
                and execution.get_status()["task"]["handback_confirmed"],
                4.0,
                "expired pose confirmed handback",
            )
            settle("pose_expiry")

            scenario = "peer_restart"
            wait(relayed_ready, 6.0, "ready before peer restart")
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "peer restart rearm")
            tele(0.2, 0.0)
            pump(0.5)
            peer.stop(kill=True)
            elapsed = wait(idle_unlatched, 0.8, "peer loss reaches IDLE")
            pump(2.5)
            case(
                "peer_loss_ends_navigation",
                nav()["phase"] != "active",
                idle_after_s=elapsed,
                phase=nav()["phase"],
            )
            settle("peer_loss")
            peer = PeerHost(args.output, PEER_CLOCK_OFFSETS[2], peer_log)
            tele(0.2, 0.0)  # a fresh peer emitting motion still cannot resume the old session
            pump(1.0)
            case(
                "restarted_peer_requires_explicit_begin",
                nav()["phase"] != "active" and not moving(),
            )
            tele(0.0, 0.0)
            wait(lambda: nav()["pose_ready"], 6.0, "peer restart handback")
            wait(relayed_ready, 90.0, "restarted peer readiness")
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "restarted peer rearm")
            scenario = "after_peer_restart"
            before = envelope()
            tele(0.2, 0.0)
            pump(2.0)
            after = envelope()
            tele(0.0, 0.0)
            heading = yaw(before["qpos"][3:7])
            forward = (after["pelvis"][0] - before["pelvis"][0]) * math.cos(heading) + (
                after["pelvis"][1] - before["pelvis"][1]
            ) * math.sin(heading)
            case("forward_after_peer_restart", forward > 0.001, forward_displacement_m=forward)
            settle("after_peer_restart")

            scenario = "publisher_restart"
            tele(0.2, 0.0)
            pump(0.5)
            owner = worker_pid(pc2, "SonicExecutionModule")
            stalled, execution = execution, None  # no RPCs into the frozen owner
            stop_pid(owner)
            proof.settled_start = None
            pump(0.6)  # the native watchdog must act with no publisher at all
            execution = stalled
            os.kill(owner, signal.SIGKILL)
            stopped.remove(owner)
            detach_pc2()
            tele(0.0, 0.0)
            settle("publisher_loss")
            attach_pc2()
            wait(
                lambda: execution.get_status()["subscriptions_ready"],
                10.0,
                "restarted subscriptions",
            )
            wait(lambda: feedback() is not None, 2.0, "restarted feedback")
            pump(1.0)
            case(
                "restarted_publisher_starts_disarmed",
                nav()["phase"] == "disarmed" and not moving(),
                phase=nav()["phase"],
                native_latched=feedback().planner_guard.latched,
            )
            pump(1.0)
            latest = inputs_seen[-1]
            # The new relay owns no request but the peer still reports the previous
            # session's: it must stay closed rather than adopt it (no old target).
            case(
                "restarted_relay_refuses_unknown_peer_request",
                not latest["ready"] and latest["fault"] == "peer_request_changed" and not moving(),
                fault=latest["fault"],
            )
            # Explicit recovery: commission a fresh peer, which owns no request.
            peer.stop(kill=True)
            peer = PeerHost(args.output, PEER_CLOCK_OFFSETS[3], peer_log)
            peer.call("tele", vx=0.0, wz=0.0)
            wait(link_ready, 90.0, "restarted publisher link readiness")
            execution.begin_navigation()
            wait(lambda: nav()["phase"] == "active", 1.5, "restarted publisher rearm")
            pump(1.0)
            case("no_old_target_after_publisher_restart", not moving())
            execution.end_navigation()
            wait(lambda: nav()["pose_ready"], 6.0, "final cancel")
            settle("final")
            missed = [c["name"] for c in results["cases"] if not c["passed"]]
            results["status"] = "failed" if missed else "passed"
            results["failed_cases"] = missed
    except Exception as exc:
        results["status"] = "failed"
        results["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        observer_stop.set()
        for pid in stopped:
            os.kill(pid, signal.SIGCONT)
        if peer is not None:
            peer.stop()
        detach_pc2()
        for proxy in proxies.values():
            proxy.close()
        for router in routers:
            router.close()
        (args.output / "sim-stop").touch()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)
        observer.close()
        context.term()
        for stream in (telemetry, refs, inputs_log, peer_log):
            stream.close()
        try:
            rows = [json.loads(line) for line in (args.output / "physics.jsonl").open()]
            released = [r for r in rows if not r["band"]]
            if released:
                lateness = [r["scheduler_lateness_s"] for r in released]
                results["load"]["physics_after_release"] = {
                    "max_scheduler_lateness_s": max(lateness),
                    "p99_scheduler_lateness_s": float(np.percentile(lateness, 99)),
                    "missed_step_deadlines": released[-1]["missed_step_deadlines"]
                    - released[0]["missed_step_deadlines"],
                    "automatic_resets": released[-1]["automatic_resets"],
                }
        except (OSError, ValueError, KeyError) as exc:
            results["load"]["physics_after_release"] = {"error": repr(exc)}
        (args.output / "results.json").write_text(json.dumps(results, indent=2, default=str) + "\n")
