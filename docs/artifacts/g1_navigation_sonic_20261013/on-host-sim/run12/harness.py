"""Native navigation commissioning in a private loopback-only namespace.

Uses one execution owner/publisher and only its own native/simulator children.
The on-host source and cancel/status fixture replace the Stage B peer, not the
native controller or physics. Requires existing private assets; never installs.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
from typing import Any

from demo_navigation_noise import (
    ConversionTrace,
    FixedHeadingFixture,
)
import numpy as np
import zmq

from dimos.core.core import rpc
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.go2.replanning_a_star.controllers import PController
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import TOPIC
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_reference import encode_command
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile


def isolated(host_netns: str) -> None:
    if [name for _, name in socket.if_nameindex()] != ["lo"] or os.readlink(
        "/proc/self/ns/net"
    ) == host_netns:
        raise RuntimeError("requires a separate loopback-only namespace")


class SimulationExecution(SonicExecutionModule):
    """Simulation lifecycle only; navigation still uses the production owner."""

    @rpc
    async def start_simulated_controller(self, host_netns: str) -> None:
        isolated(host_netns)
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            self._send(encode_command(start=True, stop=False, planner=True))
            await asyncio.sleep(0.02)

    drop_planner: bool = False
    drop_pose: bool = False
    fixed_fixture: FixedHeadingFixture | None = None
    noise_timing: dict[str, Any] | None = None

    def _send(self, packet: bytes) -> None:
        fixture_record = None
        if (
            self.fixed_fixture is not None
            and packet.startswith(b"planner")
            and (
                struct.unpack("<i", packet[1287:1291])[0] == 1
                or self.fixed_fixture.source_sequence >= 0
            )
        ):
            value, state = self._navigation._input, self.get_feedback()
            if value is not None and state is not None:
                packet, fixture_record = self.fixed_fixture.convert(
                    packet,
                    value.twist,
                    state,
                    source_sequence=self._navigation.status.source_sequence,
                )
        if not (
            (self.drop_planner and packet.startswith(b"planner"))
            or (self.drop_pose and packet.startswith(b"pose"))
        ):
            super()._send(packet)
            sent_at = time.monotonic()
            if (
                self.noise_timing is not None
                and packet.startswith(b"planner")
                and struct.unpack("<i", packet[1287:1291])[0] == 1
                and "first_moving" not in self.noise_timing
            ):
                self.noise_timing.update(
                    first_moving=sent_at, first_sequence=struct.unpack("<q", packet[1323:1331])[0]
                )
            if getattr(self, "sent_trace", None) is not None:
                self.sent_trace.write(
                    json.dumps(
                        {
                            "monotonic": sent_at,
                            "source_sequence": self._navigation.status.source_sequence,
                            "packet_hex": packet.hex(),
                            "fixture": fixture_record,
                        }
                    )
                    + "\n"
                )
                self.sent_trace.flush()

    @rpc
    async def drop_native_feedback(self, drop: bool, host_netns: str) -> None:
        isolated(host_netns)
        assert self._subscriber is not None
        self._subscriber.setsockopt(zmq.UNSUBSCRIBE if drop else zmq.SUBSCRIBE, TOPIC)

    @rpc
    async def disconnect_simulated_publisher(self, host_netns: str) -> None:
        isolated(host_netns)
        assert self._publisher is not None
        # A real endpoint removal closes all connected subscriber pipes. Keep
        # the socket object valid so the production loop can receive notices.
        self._publisher.unbind(self.config.reference_bind)


def yaw(q: list[float]) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PhysicsProof:
    """Actual physics progress; never infer stationary state from zero commands."""

    def __init__(self, folder: Path) -> None:
        self.folder = folder
        self.request: str | None = None
        self.previous: dict[str, Any] | None = None
        self.settled_start: float | None = None
        self.latest: dict[str, Any] = {}

    def update(self) -> dict[str, Any]:
        row = json.loads((self.folder / "physics-latest.json").read_text())
        previous = self.previous
        if previous is not None and row["monotonic"] > previous["monotonic"]:
            dt = row["sim_time"] - previous["sim_time"]
            delta = math.remainder(yaw(row["qpos"][3:7]) - yaw(previous["qpos"][3:7]), 2 * math.pi)
            row["yaw_rate_radps"] = delta / dt if dt > 0 else math.inf
            gap = row["monotonic"] - previous["monotonic"]
            stationary = (
                gap <= 0.12
                and row["linear_speed_mps"] <= 0.03
                and abs(row["yaw_rate_radps"]) <= 0.03
            )
            self.settled_start = (
                (previous["monotonic"] if self.settled_start is None else self.settled_start)
                if stationary
                else None
            )
            self.previous = row
            self.latest = row
        elif previous is None:
            self.previous = row
            self.latest = {**row, "yaw_rate_radps": math.inf}
        return self.latest

    def cancel_navigation(self, request_id: str) -> dict[str, Any]:
        # The test source has no A* goal. Real public cancel/state RPCs belong
        # to Task 7; this fixture confirms its local source's request only.
        self.request = request_id
        return {"request_id": request_id, "cancelled": True, "state": "idle"}

    def get_navigation_peer_status(self) -> dict[str, Any]:
        row = self.latest
        stamp = row.get("monotonic", -math.inf)
        return {
            "request_id": self.request,
            "cancelled": True,
            "state": "idle",
            "ready": time.monotonic() - stamp < 0.2,
            "readiness_deadline": stamp + 0.2,
            "linear_speed_mps": row.get("linear_speed_mps", math.inf),
            "yaw_rate_radps": row.get("yaw_rate_radps", math.inf),
            "settled_for_s": 0.0 if self.settled_start is None else stamp - self.settled_start,
        }

    def set_navigation_goal(self, request_id: str, goal: Any) -> bool:
        raise RuntimeError("no off-board goal planner in Stage A")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["on-host"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--groot", type=Path, required=True)
    parser.add_argument("--host-netns", required=True)
    parser.add_argument("--gpu", required=True)
    parser.add_argument("--reference-port", type=int, required=True)
    parser.add_argument("--feedback-port", type=int, required=True)
    parser.add_argument(
        "--diagnose-forward",
        action="store_true",
        help="Keep the original two-second verdict, then record four more seconds for diagnosis only",
    )
    parser.add_argument(
        "--diagnose-transitions",
        type=int,
        choices=range(1, 5),
        help="Record repeated two-second forward transitions; failures stay recorded",
    )
    parser.add_argument(
        "--diagnose-noisy-follower",
        action="store_true",
        help="Isolate the translating trace; never report the full commissioning matrix passed",
    )
    parser.add_argument(
        "--noise-proof",
        choices=["fixed", "live"],
        help="Run only the labelled constant-heading fixture or live-heading attribution proof",
    )
    args = parser.parse_args()
    isolated(args.host_netns)
    root = Path(__file__).resolve().parents[3]
    args.output.mkdir(parents=True, exist_ok=False)
    runtime = root / "build/sonic-navigation-runtime"
    binary = runtime / "source/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    assets = runtime / "assets"
    expected = json.loads((Path(__file__).with_name("simulation-assets.json")).read_text())
    for name, digest in expected["sha256"].items():
        assert hashlib.sha256((assets / name).read_bytes()).hexdigest() == digest, name
    command = [
        str(binary),
        "lo",
        str(assets / "model_decoder.onnx"),
        str(args.groot / "gear_sonic_deploy/reference/example"),
        "--obs-config",
        str(assets / "observation_config.yaml"),
        "--encoder-file",
        str(assets / "model_encoder.onnx"),
        "--planner-file",
        str(assets / "planner/target_vel/V2/planner_sonic.onnx"),
        "--input-type",
        "zmq_manager",
        "--output-type",
        "zmq",
        "--zmq-host",
        "127.0.0.1",
        "--zmq-port",
        str(args.reference_port),
        "--zmq-out-port",
        str(args.feedback_port),
        "--disable-crc-check",
        "--reference-timeout-ms",
        "200",
        "--guarded-planner",
        "--planner-motion-logfile",
        str(args.output / "planner-motion.csv"),
        "--target-motion-logfile",
        str(args.output / "target-motion.csv"),
    ]
    env = {
        **os.environ,
        "PYTHONDONTWRITEBYTECODE": "1",
        "CUDA_VISIBLE_DEVICES": args.gpu,
        "HF_HUB_OFFLINE": "1",
        "TRANSFORMERS_OFFLINE": "1",
        "MUJOCO_GL": "egl",
    }
    sdk = args.groot / "gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64"
    env["LD_LIBRARY_PATH"] = str(sdk) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    manifest = {
        "stage": args.stage,
        "command": command,
        "netns": os.readlink("/proc/self/ns/net"),
        "host_netns": args.host_netns,
        "interfaces": socket.if_nameindex(),
        "gpu": args.gpu,
        "dds": "domain 0 isolated in this namespace",
        "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "harness_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "worker_sha256": hashlib.sha256(
            Path(__file__).with_name("demo_navigation_sim_worker.py").read_bytes()
        ).hexdigest(),
        "noise_helper_sha256": hashlib.sha256(
            Path(__file__).with_name("demo_navigation_noise.py").read_bytes()
        ).hexdigest(),
        "production_source_sha256": {
            name: hashlib.sha256((root / name).read_bytes()).hexdigest()
            for name in (
                "dimos/robot/unitree/g1/sonic_navigation.py",
                "dimos/robot/unitree/g1/sonic_navigation_session.py",
                "dimos/robot/unitree/g1/sonic_execution_module.py",
                "dimos/navigation/go2/replanning_a_star/controllers.py",
            )
        },
        "asset_sha256": expected["sha256"],
        "diagnose_forward": args.diagnose_forward,
        "diagnose_transitions": args.diagnose_transitions,
        "diagnose_noisy_follower": args.diagnose_noisy_follower,
        "noise_proof": args.noise_proof,
        "scope": "One native controller, one production execution publisher, headless physics; on-host input/control fixture",
        "thresholds": {
            "linear_speed_mps": 0.03,
            "yaw_rate_radps": 0.03,
            "settle_seconds": 0.5,
            "settle_timeout": 3.0,
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "harness.py").write_bytes(Path(__file__).read_bytes())
    (args.output / "noise-helper.py").write_bytes(
        Path(__file__).with_name("demo_navigation_noise.py").read_bytes()
    )
    results: dict[str, Any] = {"status": "incomplete", "cases": [], "error": None}
    processes: list[subprocess.Popen[Any]] = []
    owner: SimulationExecution | None = None
    proof = PhysicsProof(args.output)
    source = make_transport(f"/navigation-sim/{args.output.name}", NavigationInput)
    sequence = 0
    next_source = 0.0
    velocity = Twist()
    noisy_follower: PController | None = None
    # Quaternion.euler lazily imports SciPy. Exercise the actual advance path
    # before any live readiness lease starts; cold import is not a sensor pause.
    follower = PController(False, 0.2, 10, min_angular_velocity=0.0)
    follower.advance(
        np.array([1.0, 0.01]),
        PoseStamped(position=Vector3(), orientation=Quaternion(w=1.0)),
    )
    noisy_tick = 0
    source_paused = False
    held_input: NavigationInput | None = None
    scenario = "startup"
    telemetry = (args.output / "telemetry.jsonl").open("w")
    refs = (args.output / "references.jsonl").open("w")
    sent = (args.output / "sent.jsonl").open("w")
    inputs = (args.output / "inputs.jsonl").open("w")
    conversions = (args.output / "conversions.jsonl").open("w")

    def record_conversion(record: dict[str, Any]) -> None:
        conversions.write(json.dumps(record) + "\n")
        conversions.flush()

    context = zmq.Context()
    observer = context.socket(zmq.SUB)
    observer.setsockopt(zmq.LINGER, 0)
    observer.setsockopt(zmq.SUBSCRIBE, b"")
    observer.connect(f"tcp://127.0.0.1:{args.reference_port}")

    def pump(seconds: float) -> None:
        nonlocal next_source, sequence, held_input, velocity, noisy_tick
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            now = time.monotonic()
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("owned simulator/controller exited; inspect logs")
            if (args.output / "physics-latest.json").exists():
                row = proof.update()
                if owner is not None and now >= next_source:
                    next_source = now + 0.1
                    if noisy_follower is not None:
                        noisy_tick += 1
                        measured_yaw = 0.0 if args.noise_proof == "fixed" else yaw(row["qpos"][3:7])
                        error = 0.01 if noisy_tick % 2 else -0.01
                        w, x, y, z = (
                            (1.0, 0.0, 0.0, 0.0)
                            if args.noise_proof == "fixed"
                            else row["qpos"][3:7]
                        )
                        pose = PoseStamped(
                            position=Vector3(*row["pelvis"]),
                            orientation=Quaternion(x=x, y=y, z=z, w=w),
                        )
                        target = np.array(
                            [
                                row["pelvis"][0] + math.cos(measured_yaw + error),
                                row["pelvis"][1] + math.sin(measured_yaw + error),
                            ]
                        )
                        velocity = noisy_follower.advance(target, pose)
                        assert velocity.linear.x == 0.2 and abs(velocity.angular.z) <= 0.2
                    ready = 0 <= now - row["monotonic"] < 0.2
                    if not source_paused or held_input is None:
                        sequence += 1
                        held_input = NavigationInput(
                            velocity, sequence, now + 0.35, row["monotonic"] + 0.2, ready, None
                        )
                        inputs.write(
                            json.dumps(
                                {
                                    "monotonic": now,
                                    "source_sequence": sequence,
                                    "velocity": list(held_input._velocity),
                                    "velocity_deadline": held_input.velocity_deadline,
                                    "readiness_deadline": held_input.readiness_deadline,
                                    "physics_stamp": row["monotonic"],
                                    "kind": "fixed_heading_fixture"
                                    if args.noise_proof == "fixed"
                                    else "live",
                                }
                            )
                            + "\n"
                        )
                        inputs.flush()
                    source.publish(
                        NavigationInput(
                            held_input.twist,
                            held_input.source_sequence,
                            held_input.velocity_deadline,
                            row["monotonic"] + 0.2,
                            ready,
                            None if ready else "physics_stale",
                        )
                    )
            if owner is not None:
                state = owner.get_feedback()
                telemetry.write(
                    json.dumps(
                        {
                            "monotonic": now,
                            "scenario": scenario,
                            "execution": owner.get_status(),
                            "feedback": None if state is None else asdict(state),
                        }
                    )
                    + "\n"
                )
                telemetry.flush()
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
            refs.flush()
            time.sleep(0.01)

    def wait(condition: Any, timeout: float, name: str) -> None:
        until = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= until:
                raise TimeoutError(name)
            pump(0.02)

    def envelope() -> dict[str, Any]:
        row = proof.latest
        assert row and time.monotonic() - row["monotonic"] < 0.2, "fresh physics required"
        assert not row["band"] and row["automatic_resets"] == 0, "unsupported/no resets required"
        assert row["pelvis"][2] >= 0.5 and row["tilt_deg"] <= 10, "stable standing required"
        assert np.all(np.isfinite(row["qpos"])), "finite physics required"
        return row

    def settle(name: str) -> None:
        nonlocal velocity
        velocity = Twist()
        started = time.monotonic()
        # A previous stationary interval cannot prove the current stop. Only
        # advancing physics after this zero command can qualify.
        proof.settled_start = None
        wait(
            lambda: (
                proof.settled_start is not None
                and proof.latest["monotonic"] - proof.settled_start >= 0.5
            ),
            3.0,
            name + " measured settling",
        )
        envelope()
        results["cases"].append(
            {
                "name": name + "_settled",
                "passed": True,
                "elapsed": time.monotonic() - started,
                "speed": proof.latest["linear_speed_mps"],
                "yaw_rate": proof.latest["yaw_rate_radps"],
            }
        )

    try:
        with (
            (args.output / "sim.log").open("wb") as simlog,
            (args.output / "deploy.log").open("wb") as deploylog,
        ):
            worker = Path(__file__).with_name("demo_navigation_sim_worker.py")
            processes.append(
                subprocess.Popen(
                    [
                        str(args.groot / ".venv_sim/bin/python"),
                        "-u",
                        str(worker),
                        "--output",
                        str(args.output),
                        "--host-netns",
                        args.host_netns,
                    ],
                    cwd=args.groot,
                    env=env,
                    stdout=simlog,
                    stderr=simlog,
                )
            )
            wait(lambda: (args.output / "sim-ready").exists(), 30.0, "simulator readiness")
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=args.output,
                    env=env,
                    stdin=subprocess.PIPE,
                    stdout=deploylog,
                    stderr=deploylog,
                )
            )
            source.start()
            owner = SimulationExecution(
                reference_bind=f"tcp://127.0.0.1:{args.reference_port}",
                feedback_endpoint=f"tcp://127.0.0.1:{args.feedback_port}",
                profile=SonicModelProfile(
                    name="on-host-native",
                    source_fps=50,
                    right_hand_order="native",
                    handback_policy="native_planner",
                ),
                journal_dir=args.output / "sessions",
                navigation_enabled=True,
            )
            owner._navigation_control = proof
            owner.sent_trace = sent
            owner.set_transport("navigation_input", source)
            owner.start()
            wait(
                lambda: "Init Done" in (args.output / "deploy.log").read_text(errors="replace"),
                120.0,
                "native initialization",
            )
            wait(
                lambda: owner.get_status()["subscriptions_ready"], 5.0, "three native subscriptions"
            )
            owner.start_simulated_controller(args.host_netns)
            wait(
                lambda: (
                    "transitioning to CONTROL state"
                    in (args.output / "deploy.log").read_text(errors="replace")
                ),
                10.0,
                "control initialization",
            )
            pump(4.0)
            (args.output / "release").touch()
            pump(6.0)
            envelope()
            wait(lambda: owner.get_status()["ready"], 2.0, "guarded native readiness")
            owner.begin_navigation()
            wait(lambda: owner.get_navigation_status()["phase"] == "active", 1.0, "native rearm")
            scenario = "initial_idle"
            pump(5.0)
            settle("initial_idle")
            motions = [
                ("forward", 0.2, 0.0),
                ("backward", -0.2, 0.0),
                ("left_turn", 0.0, 0.2),
                ("right_turn", 0.0, -0.2),
            ]
            if args.diagnose_noisy_follower or args.noise_proof:
                motions = []
            if args.diagnose_transitions:
                motions = [
                    (f"forward_transition_{i}", 0.2, 0.0) for i in range(args.diagnose_transitions)
                ]
            for name, vx, wz in motions:
                before = envelope()
                scenario = name
                velocity = Twist(linear=Vector3(vx, 0, 0), angular=Vector3(0, 0, wz))
                pump(2.0)
                after = envelope()
                heading = yaw(before["qpos"][3:7])
                dx, dy = (
                    after["pelvis"][0] - before["pelvis"][0],
                    after["pelvis"][1] - before["pelvis"][1],
                )
                forward = dx * math.cos(heading) + dy * math.sin(heading)
                angle = math.remainder(yaw(after["qpos"][3:7]) - heading, 2 * math.pi)
                passed = forward * vx > 0.001 if vx else angle * wz > 0.001
                results["cases"].append(
                    {
                        "name": name,
                        "passed": passed,
                        "start": before["monotonic"],
                        "end": after["monotonic"],
                        "forward_displacement_m": forward,
                        "yaw_change_rad": angle,
                    }
                )
                if args.diagnose_forward and name == "forward":
                    pump(4.0)
                    after_six = envelope()
                    dx6, dy6 = (
                        after_six["pelvis"][0] - before["pelvis"][0],
                        after_six["pelvis"][1] - before["pelvis"][1],
                    )
                    results["diagnostic_forward_6s"] = {
                        "start": before["monotonic"],
                        "end": after_six["monotonic"],
                        "forward_displacement_m": dx6 * math.cos(heading) + dy6 * math.sin(heading),
                        "final_speed_mps": after_six["linear_speed_mps"],
                        "note": "Additional diagnostic window; original 2s criterion retained unchanged",
                    }
                    settle("diagnostic_forward")
                    owner.end_navigation()
                    wait(
                        lambda: owner.get_navigation_status()["pose_ready"], 3.0, "diagnostic stop"
                    )
                    results["status"] = "diagnostic_only"
                    return
                if not args.diagnose_transitions:
                    assert passed, name + " direction mismatch"
                settle(name)
                if args.diagnose_transitions:
                    pump(5.0)
            if args.diagnose_transitions:
                owner.end_navigation()
                wait(lambda: owner.get_navigation_status()["pose_ready"], 3.0, "diagnostic stop")
                results["status"] = "diagnostic_only"
                return
            scenario = "noisy_follower"
            settle("before_noisy_trace")
            owner._navigation._tracker = ConversionTrace(
                owner._navigation._tracker,
                lambda: owner._navigation.status.source_sequence,
                record_conversion,
            )
            if args.noise_proof == "fixed":
                owner.fixed_fixture = FixedHeadingFixture()
            noisy_follower = follower
            request_time = time.monotonic()
            owner.noise_timing = {"request": request_time}
            (args.output / "imu-noise").touch()
            wait(
                lambda: (
                    "first_sequence" in owner.noise_timing
                    and owner.get_feedback().planner_guard.mode == 1
                    and owner.get_feedback().planner_guard.applied_sequence
                    >= owner.noise_timing["first_sequence"]
                ),
                0.2,
                "first moving native acknowledgment",
            )
            noise_started = owner.get_feedback().received_monotonic
            pump(5.0)
            noise_ended = time.monotonic()
            noisy_follower = None
            (args.output / "imu-noise").unlink()
            navigation_active = (
                owner.get_navigation_status()["phase"] == "active"
                and owner.get_feedback().planner_guard.lease_armed
                and not owner.get_feedback().planner_guard.latched
            )
            # Native reasons, rather than wire updates or scheduled inference.
            with (args.output / "planner-motion.csv.replans.csv").open() as stream:
                changes = [
                    row
                    for row in csv.DictReader(stream)
                    if request_time <= float(row["started_monotonic"]) < noise_ended
                    and any(
                        row[key] == "1"
                        for key in (
                            "mode_changed",
                            "facing_changed",
                            "height_changed",
                            "speed_changed",
                            "direction_changed",
                        )
                    )
                ]
            results["cases"].append(
                {
                    "name": "noisy_trace_replan_limit",
                    "passed": navigation_active
                    and (args.noise_proof != "fixed" or len(changes) <= 1),
                    "start": noise_started,
                    "end": noise_ended,
                    "request": request_time,
                    "first_moving": owner.noise_timing["first_moving"],
                    "first_applied": noise_started,
                    "proof": "fixed" if args.noise_proof == "fixed" else "live",
                    "native_state_replans": len(changes),
                    "navigation_active": navigation_active,
                    "yaw_noise_rad": 0.005,
                    "source": "PController.advance, 0.2 m/s, alternating +/-0.01 rad lookahead error",
                }
            )
            if args.noise_proof == "fixed":
                assert len(changes) <= 1, "fixed-heading fixture triggered excess native replans"
            assert navigation_active, "noisy trace lost navigation admission"
            settle("noisy_trace")
            if args.diagnose_noisy_follower or args.noise_proof:
                owner.end_navigation()
                wait(lambda: owner.get_navigation_status()["pose_ready"], 3.0, "diagnostic stop")
                results["status"] = "diagnostic_only"
                return
            scenario = "source_pause"
            velocity = Twist(linear=Vector3(0.2, 0, 0))
            pump(0.5)
            source_paused = True
            pump(0.25)
            assert owner.get_navigation_status()["phase"] == "active"
            results["cases"].append({"name": "source_pause_250ms", "passed": True})
            pump(0.35)
            assert owner.get_feedback().planner_guard.mode == 0
            assert (
                owner.get_feedback().planner_guard.lease_armed
                and not owner.get_feedback().planner_guard.latched
            )
            results["cases"].append({"name": "source_expiry_idle_without_latch", "passed": True})
            source_paused = False
            velocity = Twist()
            pump(0.2)
            settle("source_expiry")
            source_paused = True
            scenario = "source_outage"
            pump(2.7)
            assert owner.get_navigation_status()["phase"] in ("stopping", "disarmed")
            assert not owner.get_feedback().planner_guard.lease_armed
            results["cases"].append({"name": "source_outage_ends", "passed": True})
            source_paused = False
            pump(0.2)
            wait(
                lambda: owner.get_navigation_status()["pose_ready"],
                3.0,
                "outage cancellation and settling",
            )
            owner.begin_navigation()
            wait(lambda: owner.get_navigation_status()["phase"] == "active", 1.0, "outage rearm")

            scenario = "native_watchdog"
            velocity = Twist(linear=Vector3(0.2, 0, 0))
            pump(0.5)
            owner.drop_planner = True
            wait(lambda: owner.get_feedback().planner_guard.latched, 1.0, "native watchdog")
            assert owner.get_feedback().planner_guard.mode == 0
            assert owner.get_navigation_status()["phase"] == "fault"
            results["cases"].append({"name": "native_watchdog_250ms", "passed": True})
            owner.drop_planner = False
            settle("watchdog")
            wait(lambda: owner.get_navigation_status()["pose_ready"], 3.0, "watchdog cancel")
            owner.begin_navigation()
            wait(
                lambda: owner.get_navigation_status()["phase"] == "active",
                1.0,
                "watchdog explicit rearm",
            )
            assert not owner.get_feedback().planner_guard.latched

            scenario = "feedback_loss"
            owner.drop_native_feedback(True, args.host_netns)
            pump(0.35)
            assert owner.get_navigation_status()["fault"] == "feedback_lost"
            results["cases"].append({"name": "feedback_loss_200ms", "passed": True})
            owner.drop_native_feedback(False, args.host_netns)
            wait(
                lambda: owner.get_navigation_status()["pose_ready"],
                3.0,
                "feedback recovery settles",
            )
            owner.begin_navigation()
            wait(
                lambda: owner.get_navigation_status()["phase"] == "active",
                1.0,
                "feedback explicit rearm",
            )

            scenario = "seeded_pose"
            owner.end_navigation()
            wait(
                lambda: owner.get_navigation_status()["pose_ready"],
                3.0,
                "cancel and physical settling",
            )
            task = owner.start_task()
            wait(
                lambda: owner.get_status()["task"]["phase"] == "waiting_for_chunk",
                2.0,
                "pose activation",
            )
            began = time.monotonic()
            chunk_seq = 0
            while time.monotonic() - began < 1.2:
                state = owner.get_feedback()
                assert state is not None and state.token_state is not None
                action = np.asarray(state.token_state + state.left_hand_q + state.right_hand_q)
                owner.submit_chunk(
                    task["task_id"], chunk_seq, state.received_monotonic, np.tile(action, (50, 1))
                )
                chunk_seq += 1
                pump(0.15)
                assert not owner.get_feedback().planner_guard.latched
            assert owner.get_navigation_status()["phase"] == "pose"
            results["cases"].append(
                {
                    "name": "seeded_pose_suspends_planner_watchdog",
                    "passed": True,
                    "duration": time.monotonic() - began,
                }
            )
            owner.cancel_task(task["task_id"])
            wait(
                lambda: owner.get_navigation_status()["phase"] == "disarmed",
                3.0,
                "confirmed pose handback",
            )
            settle("pose_handback")
            assert not owner.get_feedback().planner_guard.lease_armed
            scenario = "pose_expiry"
            task = owner.start_task()
            wait(
                lambda: owner.get_status()["task"]["phase"] == "waiting_for_chunk",
                2.0,
                "expiry pose activation",
            )
            state = owner.get_feedback()
            action = np.asarray(state.token_state + state.left_hand_q + state.right_hand_q)
            owner.submit_chunk(
                task["task_id"], 0, state.received_monotonic, np.tile(action, (50, 1))
            )
            pump(0.1)
            owner.drop_pose = True
            expiry_started = time.monotonic()
            wait(
                lambda: owner.get_feedback().reference_guard.expired,
                0.5,
                "native 200ms pose expiry",
            )
            expired = owner.get_feedback()
            assert not expired.planner_guard.latched
            results["cases"].append(
                {
                    "name": "pose_source_expiry_200ms",
                    "passed": True,
                    "elapsed": time.monotonic() - expiry_started,
                    "native_age_ms": expired.reference_guard.age_ms,
                }
            )
            owner.drop_pose = False
            wait(
                lambda: (
                    owner.get_status()["task"]["phase"] == "done"
                    and owner.get_status()["task"]["handback_confirmed"]
                ),
                3.0,
                "expired pose confirmed handback",
            )
            settle("pose_expiry")
            wait(lambda: owner.get_status()["ready"], 3.0, "post-pose ready")
            owner.begin_navigation()
            wait(lambda: owner.get_navigation_status()["phase"] == "active", 1.0, "final rearm")
            scenario = "publisher_disconnect"
            velocity = Twist(linear=Vector3(0.2, 0, 0))
            pump(0.5)
            disconnected = time.monotonic()
            owner.disconnect_simulated_publisher(args.host_netns)
            wait(
                lambda: not owner.get_status()["subscriptions_ready"],
                0.5,
                "actual native subscription disconnect",
            )
            assert owner.get_navigation_status()["fault"] == "subscriptions_lost"
            results["cases"].append({"name": "subscriptions_loss", "passed": True})
            wait(
                lambda: owner.get_feedback().planner_guard.latched,
                0.75,
                "disconnected publisher native watchdog",
            )
            assert owner.get_feedback().planner_guard.mode == 0
            results["cases"].append(
                {
                    "name": "stopped_publisher_watchdog",
                    "passed": True,
                    "elapsed": time.monotonic() - disconnected,
                    "injection": "unbind the sole publisher endpoint; actual subscriber pipes close",
                }
            )
            settle("publisher_disconnect")
            results["status"] = "passed"
    except Exception as exc:
        results["status"] = "failed"
        results["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if owner is not None:
            owner.stop()
        source.stop()
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
        telemetry.close()
        refs.close()
        sent.close()
        inputs.close()
        conversions.close()
        (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
