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


REASONS = ("mode_changed", "facing_changed", "height_changed", "speed_changed", "direction_changed")


def conversion_allowed(row: dict) -> bool:
    """Changed fields need a fresh intent threshold crossing or frame transition."""
    before = struct.unpack("<i8f", bytes.fromhex(row["previous_payload"]))
    after = struct.unpack("<i8f", bytes.fromhex(row["payload_hex"]))
    vx, vy, vz, wx, wy, wz = row["velocity"]

    def yaw(q: list) -> float:
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def difference(a: float, b: float) -> float:
        return abs(math.remainder(a - b, 2 * math.pi))

    reference = yaw(row["reference_quat"])
    reset = (
        row["reference_before"] is None or difference(reference, row["reference_before"]) >= 0.02
    )
    transition = before[0] != after[0] or row["stopped_before"] or reset
    if any((vy, vz, wx, wy)) or abs(vx) > 0.2 or abs(wz) > 0.2:
        return False
    if before[1:4] != after[1:4]:
        expected = yaw(row["base_quat"]) + (math.pi if vx < 0 else 0) - reference
        measured = math.atan2(after[2], after[1])
        previous = math.atan2(before[2], before[1])
        if not (transition or difference(measured, previous) >= 0.02 - 1e-6):
            return False
        if after[0] == 1 and difference(measured, expected) > 1e-6:
            return False
    if before[4:7] != after[4:7] and not transition:
        if abs(wz) <= 0.01:
            return False
        if (
            difference(math.atan2(after[5], after[4]), math.atan2(before[5], before[4]))
            < 0.02 - 1e-6
        ):
            return False
    if before[7] != after[7] and not transition and abs(after[7] - before[7]) < 0.01 - 1e-6:
        return False
    return True


def attribute_replans(*, sent: list[dict], acks: list[dict], native: list[dict]) -> dict:
    """A state replan must match recent wire intent and an actual applied ACK.

    All stamps share one host clock. The .2s ACK window is the existing feedback
    freshness budget, not a relaxation of command admission or lease deadlines.
    """
    packets: dict[int, dict] = {}
    held: dict[int, bytes] = {}
    errors: list[str] = []
    for row in sent:
        packet = bytes.fromhex(row["packet_hex"])
        if not packet.startswith(b"planner"):
            continue
        mode, *values, sequence, deadline = struct.unpack("<i8fqd", packet[1287:])
        payload = packet[1287:1323]
        source = row["source_sequence"]
        if source in held and held[source] != payload:
            errors.append(f"source {source}: heartbeat changed motion")
        held[source] = payload
        packets[sequence] = {**row, "payload": payload, "mode": mode, "deadline": deadline}
    attributed = []
    previous_native = None
    for row in native:
        start, finish = float(row["started_monotonic"]), float(row["finished_monotonic"])
        mode = int(row["mode"])
        payload = struct.pack(
            "<i8f",
            mode,
            float(row["movement_x"]),
            float(row["movement_y"]),
            0,
            float(row["facing_x"]),
            float(row["facing_y"]),
            0,
            float(row["speed"]) if mode else 0,
            -1,
        )
        changed = any(row[key] == "1" for key in REASONS)
        horizon = row.get("horizon_refresh", "0") == "1"
        if horizon:
            try:
                # This proof is pinned to the shipped 10-frame/stride-5
                # encoder and .1s planner period with two-frame lookahead.
                remaining = int(row["horizon_remaining"])
                window = int(row["horizon_window"])
                margin = int(row["horizon_margin"])
                valid_horizon = (
                    mode == 1
                    and float(row["speed"]) > 0
                    and window == 46
                    and margin == 8
                    and 0 < remaining <= window + margin
                )
            except (KeyError, ValueError):
                valid_horizon = False
            if not valid_horizon:
                errors.append(f"native {start}: invalid horizon refresh reason")
        if changed and previous_native == payload:
            errors.append(f"native {start}: state replan repeated unchanged motion")
        # Periodic inference establishes the preceding state too; skipping it
        # could let a later heartbeat-only replan masquerade as a transition.
        previous_native = payload
        if not changed:
            if row.get("periodic") != "1" and not horizon:
                errors.append(f"native {start}: inference has no recorded maintenance reason")
            continue
        matches = []
        for ack in acks:
            packet = packets.get(ack["sequence"])
            if (
                packet
                and packet["payload"] == payload
                and start - 0.25 <= packet["monotonic"] <= start
                and start - 0.2 <= ack["monotonic"] <= finish + 0.2
                and packet["monotonic"] <= ack["monotonic"]
                and packet["deadline"] > start
            ):
                matches.append(ack["sequence"])
        if not matches:
            errors.append(f"native {start}: no acknowledged matching intent")
        attributed.append(
            {
                "monotonic": start,
                "acknowledged_sequences": sorted(set(matches)),
                "reasons": [key for key in REASONS if row[key] == "1"],
            }
        )
    return {
        "passed": not errors and bool(attributed),
        "errors": errors,
        "native_state_replans": len(attributed),
        "attributed": attributed,
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
    sent_rows = []
    if sent.exists():
        for line in sent.open():
            row = json.loads(line)
            packet = bytes.fromhex(row["packet_hex"])
            if case.get("request", start) <= row["monotonic"] <= end:
                sent_rows.append(row)
            if start <= row["monotonic"] <= end and packet.startswith(b"planner"):
                packets.append((row["source_sequence"], packet[1287:1323]))
    heartbeat_payloads: dict[int, set[bytes]] = {}
    for sequence, payload in packets:
        heartbeat_payloads.setdefault(sequence, set()).add(payload)
    modes = [struct.unpack("<i", payload[:4])[0] for _, payload in packets]
    first_moving = modes.index(1) if 1 in modes else len(modes)
    with (folder / "planner-motion.csv.replans.csv").open() as stream:
        native_rows = [
            row
            for row in csv.DictReader(stream)
            if case.get("request", start) <= float(row["started_monotonic"]) < end
        ]
    ack_rows = []
    for line in (folder / "telemetry.jsonl").open():
        row = json.loads(line)
        guard = (row.get("feedback") or {}).get("planner_guard")
        if guard and case.get("request", start) - 0.2 <= row["monotonic"] <= end + 0.2:
            ack_rows.append({"monotonic": row["monotonic"], "sequence": guard["applied_sequence"]})
    attribution = attribute_replans(sent=sent_rows, acks=ack_rows, native=native_rows)
    replans = attribution["native_state_replans"]
    conversion_errors = []
    if case.get("proof") == "live" and (folder / "conversions.jsonl").exists():
        inputs = {
            row["source_sequence"]: row
            for line in (folder / "inputs.jsonl").open()
            if (row := json.loads(line))
        }
        converted = {}
        for line in (folder / "conversions.jsonl").open():
            row = json.loads(line)
            if not case["request"] <= row["monotonic"] <= end:
                continue
            source = inputs.get(row["source_sequence"])
            if (
                source is None
                or source["velocity"] != row["velocity"]
                or source["velocity_deadline"] <= row["monotonic"]
                or not conversion_allowed(row)
            ):
                conversion_errors.append(f"unexplained conversion: {row['source_sequence']}")
            converted[row["source_sequence"]] = row["payload_hex"]
        for row in sent_rows:
            packet = bytes.fromhex(row["packet_hex"])
            if packet.startswith(b"planner") and struct.unpack("<i", packet[1287:1291])[0] == 1:
                if converted.get(row["source_sequence"]) != packet[1287:1323].hex():
                    conversion_errors.append(f"wire has no conversion: {row['source_sequence']}")
    elif case.get("proof") == "fixed":
        for row in sent_rows:
            packet = bytes.fromhex(row["packet_hex"])
            if packet.startswith(b"planner") and struct.unpack("<i", packet[1287:1291])[0] == 1:
                fixture = row.get("fixture") or {}
                if (
                    fixture.get("kind") != "fixed_heading_fixture"
                    or fixture.get("nominal_yaw") != 0
                    or abs(fixture.get("measurement_noise", 0)) != 0.005
                    or fixture.get("payload_hex") != packet[1287:1323].hex()
                ):
                    conversion_errors.append("missing fixed-heading provenance")
    else:
        conversion_errors.append("missing labelled conversion proof")
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
        "native_replans": attribution["passed"]
        and replans == case["native_state_replans"]
        and (case.get("proof") != "fixed" or replans <= 1),
        "conversion_provenance": not conversion_errors,
        "held_facing": len({payload[16:28] for _, payload in packets}) == 1,
        "fixed_movement": case.get("proof") != "fixed"
        or len({payload for _, payload in packets}) == 1,
        "bounded_applied_latency": "first_applied" in case
        and 0 <= case["first_moving"] - case["request"] <= 0.2
        and 0 <= case["first_applied"] - case["first_moving"] <= 0.2,
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
        "attribution": attribution,
        "conversion_errors": conversion_errors,
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
    proof_only = manifest.get("noise_proof") in ("fixed", "live")
    return {
        "passed": passed,
        "completed": sorted(cases),
        "missing": missing,
        "failed": failed,
        "run_status": result["status"],
        "run_error": result["error"],
        "wire_samples": {"moving": moving, "idle": idle, "pose": pose},
        "translating_noise": noise,
        "noise_proof_only_passed": proof_only
        and noise["passed"]
        and result["status"] == "diagnostic_only"
        and not failed
        and result["error"] is None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument(
        "--proof-only",
        action="store_true",
        help="Validate a labelled noise proof; never imply the full matrix passed",
    )
    args = parser.parse_args()
    report = analyze(args.folder)
    (args.folder / "analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    complete = report["noise_proof_only_passed"] if args.proof_only else report["passed"]
    if not complete:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
