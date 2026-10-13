"""Explicit fixed-frame fixture and raw native evidence attribution; simulation only."""

from dataclasses import replace
import math
import struct
import time
from typing import Any

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_navigation import (
    NavigationLimits,
    PlannerIntentTracker,
    encode_planner,
)

REASONS = ("mode_changed", "facing_changed", "height_changed", "speed_changed", "direction_changed")


def motion_bytes(intent: Any) -> bytes:
    return encode_planner(intent, sequence=1, deadline=1.0)[1287:1323]


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


class FixedHeadingFixture:
    """Replace only the planner motion payload; real native guard fields survive."""

    def __init__(self) -> None:
        self.tracker = PlannerIntentTracker(NavigationLimits())
        self.source_sequence = -1
        self.mode = -1
        self.last_record: dict[str, Any] = {}

    def convert(
        self, packet: bytes, twist: Twist, feedback: SonicFeedback, *, source_sequence: int
    ) -> tuple[bytes, dict[str, Any]]:
        mode = struct.unpack("<i", packet[1287:1291])[0]
        if source_sequence != self.source_sequence or mode != self.mode:
            noise = 0.005 if source_sequence % 2 else -0.005
            fixed = replace(
                feedback,
                base_quat_measured=(math.cos(noise / 2), 0.0, 0.0, math.sin(noise / 2)),
                reference_heading_quat=(1.0, 0.0, 0.0, 0.0),
                planner_reference_active=True,
            )
            intent = self.tracker.update(twist, fixed) if mode == 1 else self.tracker.stop(fixed)
            self.last_record = {
                "kind": "fixed_heading_fixture",
                "source_sequence": source_sequence,
                "nominal_yaw": 0.0,
                "measurement_noise": noise,
                "payload_hex": motion_bytes(intent).hex(),
            }
            self.source_sequence, self.mode = source_sequence, mode
        return packet[:1287] + motion_bytes(self.tracker.heartbeat()) + packet[
            1323:
        ], self.last_record


class ConversionTrace:
    """Delegate to the existing tracker and record the exact conversion operands."""

    def __init__(self, tracker: PlannerIntentTracker, source_sequence: Any, record: Any) -> None:
        self.tracker, self.source_sequence, self.record = tracker, source_sequence, record

    def heartbeat(self) -> Any:
        return self.tracker.heartbeat()

    def update(self, twist: Twist, feedback: SonicFeedback, *, stopping: bool = False) -> Any:
        previous = self.tracker.heartbeat()
        reference_before = self.tracker._reference_yaw
        stopped_before = self.tracker._stopped
        intent = self.tracker.update(twist, feedback, stopping=stopping)
        self.record(
            {
                "monotonic": time.monotonic(),
                "source_sequence": self.source_sequence(),
                "previous_payload": motion_bytes(previous).hex(),
                "payload_hex": motion_bytes(intent).hex(),
                "base_quat": feedback.base_quat_measured,
                "reference_quat": feedback.reference_heading_quat,
                "reference_before": reference_before,
                "stopped_before": stopped_before,
                "velocity": [
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z,
                ],
            }
        )
        return intent

    def stop(self, feedback: SonicFeedback | None) -> Any:
        return self.tracker.stop(feedback)


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
    for row in native:
        if not any(row[key] == "1" for key in REASONS):
            continue
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
        matches = []
        for ack in acks:
            packet = packets.get(ack["sequence"])
            if (
                packet
                and packet["payload"] == payload
                and start - 0.25 <= packet["monotonic"] <= finish
                and start - 0.2 <= ack["monotonic"] <= finish + 0.2
                and packet["deadline"] > packet["monotonic"]
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
