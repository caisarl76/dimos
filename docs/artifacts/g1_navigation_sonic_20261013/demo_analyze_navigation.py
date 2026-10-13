"""Reject incomplete native navigation evidence; never relax run thresholds."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import struct

REQUIRED = {
    "initial_idle_settled",
    "forward",
    "backward",
    "left_turn",
    "right_turn",
    "forward_settled",
    "backward_settled",
    "left_turn_settled",
    "right_turn_settled",
    "source_pause_250ms",
    "source_expiry_idle_without_latch",
    "source_outage_ends",
    "native_watchdog_250ms",
    "feedback_loss_200ms",
    "seeded_pose_suspends_planner_watchdog",
    "pose_source_expiry_200ms",
    "subscriptions_loss",
    "stopped_publisher_watchdog",
    "noisy_trace_replan_limit",
}


def translating_noise_checks(folder: Path, case: dict) -> dict:
    """Recompute the translating trace verdict from independent raw records."""
    start, end = case["start"], case["end"]
    physics = [
        row
        for line in (folder / "physics.jsonl").open()
        if start <= (row := json.loads(line))["monotonic"] <= end
    ]
    telemetry = [
        row
        for line in (folder / "telemetry.jsonl").open()
        if start <= (row := json.loads(line))["monotonic"] <= end
    ]
    sent = folder / "sent.jsonl"
    packets = []
    if sent.exists():
        for line in sent.open():
            row = json.loads(line)
            packet = bytes.fromhex(row["packet_hex"])
            if start <= row["monotonic"] <= end and packet.startswith(b"planner"):
                packets.append((row["source_sequence"], packet[1287:1323]))
    heartbeat_payloads: dict[int, set[bytes]] = {}
    for sequence, payload in packets:
        heartbeat_payloads.setdefault(sequence, set()).add(payload)
    modes = [struct.unpack("<i", payload[:4])[0] for _, payload in packets]
    first_moving = modes.index(1) if 1 in modes else len(modes)
    with (folder / "planner-motion.csv.replans.csv").open() as stream:
        replans = sum(
            start <= float(row["started_monotonic"]) < end
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
            for row in csv.DictReader(stream)
        )
    displacement = None
    if len(physics) >= 2:
        first, last = physics[0], physics[-1]
        w, x, y, z = first["qpos"][3:7]
        heading = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        displacement = sum(
            (last["pelvis"][i] - first["pelvis"][i]) * direction
            for i, direction in enumerate((math.cos(heading), math.sin(heading)))
        )
    checks = {
        "five_second_trace": end - start >= 5.0,
        "advancing_physics": len(physics) >= 200,
        "both_sensor_noise_signs": {row["imu_noise_rad"] for row in physics} >= {-0.005, 0.005},
        # The source begins on its next 10Hz tick, so initial IDLE receipts are
        # expected. After admission, any return to IDLE invalidates the trace.
        "moving_wire": modes.count(1) >= 200
        and first_moving <= 10
        and all(mode == 1 for mode in modes[first_moving:]),
        "advancing_source": len(heartbeat_payloads) >= 40,
        "literal_heartbeats": bool(heartbeat_payloads)
        and all(len(payloads) == 1 for payloads in heartbeat_payloads.values()),
        "active_throughout": bool(telemetry)
        and all(row["execution"]["navigation"]["phase"] == "active" for row in telemetry),
        "native_replans": replans <= 1 and replans == case["native_state_replans"],
        "forward_physics": displacement is not None and displacement > 0,
        "standing_envelope": bool(physics)
        and all(
            not row["band"]
            and row["automatic_resets"] == 0
            and row["pelvis"][2] >= 0.5
            and row["tilt_deg"] <= 10
            for row in physics
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "forward_displacement_m": displacement,
        "native_state_replans": replans,
        "planner_packets": len(packets),
        "source_sequences": len(heartbeat_payloads),
    }


def analyze(folder: Path) -> dict:
    manifest = json.loads((folder / "manifest.json").read_text())
    result = json.loads((folder / "results.json").read_text())
    cases = {case["name"]: case for case in result["cases"]}
    failed = [name for name, case in cases.items() if case["passed"] is not True]
    missing = sorted(REQUIRED - cases.keys())
    assert manifest["interfaces"] == [[1, "lo"]]
    assert manifest["netns"] != manifest["host_netns"]
    assert manifest["thresholds"] == {
        "linear_speed_mps": 0.03,
        "yaw_rate_radps": 0.03,
        "settle_seconds": 0.5,
        "settle_timeout": 3.0,
    }
    last_sequence = 0
    moving = idle = pose = 0
    for line in (folder / "references.jsonl").read_text().splitlines():
        packet = bytes.fromhex(json.loads(line)["packet_hex"])
        if packet.startswith(b"planner"):
            assert len(packet) == 1339
            mode, *values, seq, deadline = struct.unpack("<i8fqd", packet[1287:])
            assert seq > last_sequence and deadline > 0
            last_sequence = seq
            assert mode in (0, 1)
            if mode == 0:
                assert values[:3] == [0.0, 0.0, 0.0] and values[6] == 0.0
                idle += 1
            else:
                assert 0.09999999 <= values[6] <= 0.20000001
                moving += 1
        elif packet.startswith(b"pose"):
            pose += 1
    noise = (
        translating_noise_checks(folder, cases["noisy_trace_replan_limit"])
        if "noisy_trace_replan_limit" in cases
        else {"passed": False, "reason": "missing trace"}
    )
    passed = (
        not missing
        and not failed
        and result["status"] == "passed"
        and moving > 0
        and idle > 0
        and pose > 0
        and noise["passed"]
    )
    return {
        "passed": passed,
        "completed": sorted(cases),
        "missing": missing,
        "failed": failed,
        "run_status": result["status"],
        "run_error": result["error"],
        "wire_samples": {"moving": moving, "idle": idle, "pose": pose},
        "translating_noise": noise,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    report = analyze(args.folder)
    (args.folder / "analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
