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

"""Test-only native PlannerLease stand-in, pinned by the shared JSON oracle.

Never import this from an execution module or use it as a robot controller.
"""

from dataclasses import dataclass, replace
import math
import struct


@dataclass(frozen=True)
class PlannerPacket:
    mode: int = 0
    movement: tuple[float, float, float] = (0.0, 0.0, 0.0)
    facing: tuple[float, float, float] = (1.0, 0.0, 0.0)
    speed: float = 0.0
    height: float = -1.0
    sequence: int = -1
    deadline: float = 0.0


@dataclass
class LeaseStatus:
    mode: int = 0
    high_water: int = 0
    applied_sequence: int = 0
    rearm_barrier: int = 0
    lease_armed: bool = False
    latched: bool = False
    idle_ack_sequence: int = 0
    fault: str = "disarmed"


class PlannerLease:
    def __init__(self) -> None:
        self.status = LeaseStatus()
        self.active = PlannerPacket()
        self.pending: PlannerPacket | None = None
        self.end_ack: int | None = None
        self.last_receipt = 0.0
        self.rearm_deadline = 0.0
        self.awaiting_idle = False
        self.pose = False

    def fresh(self, sequence: int, deadline: float, now: float) -> bool:
        return (
            sequence > self.status.high_water
            and math.isfinite(now)
            and math.isfinite(deadline)
            and deadline > now
        )

    @staticmethod
    def valid(packet: PlannerPacket) -> bool:
        def horizontal_unit(vector: tuple[float, float, float]) -> bool:
            return (
                all(math.isfinite(v) for v in vector)
                and vector[2] == 0
                and abs(vector[0] ** 2 + vector[1] ** 2 - 1) <= 0.0001
            )

        if (
            packet.mode not in (0, 1)
            or not math.isfinite(packet.speed)
            or packet.height != -1
            or (
                not all(math.isfinite(v) for v in packet.movement)
                or not horizontal_unit(packet.facing)
            )
        ):
            return False
        if packet.mode == 0:
            return packet.speed == 0 and packet.movement == (0, 0, 0)
        # Native wire values and its speed caps are float32, including oracle
        # literals that Python initially parses as doubles.
        speed = struct.unpack("<f", struct.pack("<f", packet.speed))[0]
        return 0.10000000149011612 <= speed <= 0.20000000298023224 and horizontal_unit(
            packet.movement
        )

    def force_idle(self) -> None:
        self.active = replace(
            self.active, mode=0, movement=(0.0, 0.0, 0.0), speed=0.0, deadline=0.0
        )
        self.status.mode = 0

    def close(self) -> None:
        self.status.lease_armed = False
        self.awaiting_idle = False
        self.pending = None
        self.end_ack = None
        self.force_idle()

    def rearm(self, sequence: int, deadline: float, now: float) -> bool:
        self.tick(now)
        if self.pose or not self.fresh(sequence, deadline, now):
            return False
        self.status.high_water = self.status.rearm_barrier = sequence
        self.status.lease_armed = False
        self.awaiting_idle = True
        self.rearm_deadline = min(deadline, now + 0.25)
        self.last_receipt = now
        self.force_idle()
        self.pending = None
        if not self.status.latched:
            self.status.fault = "awaiting_idle"
        return True

    def accept(self, packet: PlannerPacket, now: float) -> bool:
        self.tick(now)
        if (
            self.pose
            or not self.fresh(packet.sequence, packet.deadline, now)
            or not self.valid(packet)
        ):
            return False
        if self.awaiting_idle:
            if now >= self.rearm_deadline or packet.mode != 0:
                return False
        elif not self.status.lease_armed or self.status.latched:
            return False
        self.status.high_water = packet.sequence
        self.last_receipt = now
        self.pending = replace(packet, deadline=min(packet.deadline, now + 0.25))
        return True

    def install(self, now: float) -> bool:
        self.tick(now)
        packet, self.pending = self.pending, None
        if (
            packet is None
            or self.pose
            or packet.deadline <= now
            or (not self.awaiting_idle and not self.status.lease_armed)
        ):
            return False
        if self.awaiting_idle:
            if now >= self.rearm_deadline or packet.mode != 0:
                return False
            self.awaiting_idle = False
            self.status.lease_armed = True
            self.status.latched = False
            self.status.fault = ""
        self.active = packet
        self.status.mode = packet.mode
        self.status.applied_sequence = packet.sequence
        if packet.mode == 0:
            self.status.idle_ack_sequence = packet.sequence
        return True

    def end(self, sequence: int, deadline: float, now: float) -> bool:
        self.tick(now)
        if self.pose or not self.fresh(sequence, deadline, now):
            return False
        self.status.high_water = sequence
        self.close()
        if not self.status.latched:
            self.status.fault = "ended"
        self.end_ack = sequence
        return True

    def acknowledge_end(self) -> None:
        if self.end_ack is not None:
            self.status.applied_sequence = self.status.idle_ack_sequence = self.end_ack
            self.end_ack = None

    def enter_pose(self) -> None:
        self.close()
        self.pose = True
        if not self.status.latched:
            self.status.fault = "pose"

    def handback(self, now: float) -> None:
        self.close()
        self.pose = False
        if not self.status.latched:
            self.status.fault = "handback"

    def tick(self, now: float) -> None:
        if not math.isfinite(now) or self.pose:
            return
        if (self.status.lease_armed and now - self.last_receipt >= 0.25) or (
            self.awaiting_idle and now >= self.rearm_deadline
        ):
            self.close()
            self.status.latched = True
            self.status.fault = "planner_watchdog"
        elif self.active.deadline > 0 and now >= self.active.deadline:
            self.force_idle()
