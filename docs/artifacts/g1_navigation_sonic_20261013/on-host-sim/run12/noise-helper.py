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
