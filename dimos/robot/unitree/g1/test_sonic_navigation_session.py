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

from dataclasses import replace
import json
import math
import struct
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_feedback import PlannerGuardStatus, SonicFeedback
from dimos.robot.unitree.g1.sonic_navigation import NavigationInput
from dimos.robot.unitree.g1.sonic_navigation_session import NavigationPhase, SonicNavigationSession


def state(now: float = 10, **guard: Any) -> SonicFeedback:
    values = dict(
        mode=0,
        high_water=0,
        applied_sequence=0,
        rearm_barrier=0,
        lease_armed=True,
        latched=False,
        idle_ack_sequence=0,
        fault=None,
    )
    values.update(guard)
    return SonicFeedback(
        index=1,
        source_timestamp=None,
        received_at=now,
        received_monotonic=now,
        body_q=(0.0,) * 29,
        body_dq=(0.0,) * 29,
        left_hand_q=(0.0,) * 7,
        left_hand_dq=(0.0,) * 7,
        right_hand_q=(0.0,) * 7,
        right_hand_dq=(0.0,) * 7,
        base_quat_measured=(1.0, 0.0, 0.0, 0.0),
        reference_heading_quat=(1.0, 0.0, 0.0, 0.0),
        planner_reference_active=True,
        planner_guard=PlannerGuardStatus(**values),
    )


def sample(seq: int, now: float = 10, speed: float = 0.2) -> NavigationInput:
    return NavigationInput(
        Twist(linear=Vector3(speed, 0, 0)), seq, now + 0.35, now + 0.35, True, None
    )


def motion(packet: bytes) -> tuple[int | float, ...]:
    assert packet.startswith(b"planner")
    return struct.unpack("<i8f", packet[1287:1323])


def arm(session: SonicNavigationSession, packets: list[bytes], now: float = 10) -> None:
    session.publisher_started(now=now - 0.25)
    session.begin(state(now), sample(1, now, speed=0), now=now)
    session.tick(state(now), subscriptions_ready=True, now=now)
    seq = struct.unpack("<q", packets[-1][-16:-8])[0]
    session.observe(
        state(
            now,
            high_water=seq,
            applied_sequence=seq,
            idle_ack_sequence=seq,
            lease_armed=True,
            rearm_barrier=seq - 1,
            fault=None,
        ),
        now=now,
    )
    assert session.status.phase == NavigationPhase.ACTIVE


def test_atomic_input_owns_immutable_twist_snapshot() -> None:
    twist = Twist(linear=Vector3(0.2, 0, 0))
    value = NavigationInput(twist, 1, 10.35, 10.35, True, None)
    twist.linear.x = 0
    value.twist.linear.x = -0.2
    assert value.twist.linear.x == 0.2


def test_atomic_input_requires_explicit_begin() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    assert not session.receive(sample(1), now=10)
    session.tick(state(), subscriptions_ready=True, now=10)
    assert packets == []
    arm(session, packets)
    assert not session.receive(sample(1), now=10)
    session.tick(state(), subscriptions_ready=True, now=10)
    assert motion(packets[-1])[0] == 0
    assert session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    assert motion(packets[-1])[0] == 1
    assert not session.receive(sample(2, speed=-0.2), now=10.03)
    session.tick(state(10.04), subscriptions_ready=True, now=10.04)
    assert packets[-1][1287:1323] == packets[-2][1287:1323]
    assert packets[-1][-16:] != packets[-2][-16:]


def test_source_expiry_is_idle_without_latch_then_new_source_resumes() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    assert motion(packets[-1])[0] == 1
    # Sensor readiness may advance while the velocity source is frozen. A
    # repeated velocity sequence cannot extend its original velocity deadline.
    frozen = NavigationInput(sample(2).twist, 2, 10.35, 10.7, True, None)
    assert not session.receive(frozen, now=10.35)
    session.tick(state(10.35), subscriptions_ready=True, now=10.35)
    assert motion(packets[-1])[0] == 0
    assert session.status.phase == NavigationPhase.ACTIVE
    assert session.status.fault is None
    session.receive(sample(3, 10.4), now=10.4)
    session.tick(state(10.4), subscriptions_ready=True, now=10.4)
    assert motion(packets[-1])[0] == 1


@pytest.mark.parametrize(
    "invalid",
    [Twist(linear=Vector3(0.21, 0, 0)), Twist(linear=Vector3(0, 0.1, 0))],
)
def test_rejected_velocity_commands_idle_and_next_valid_source_recovers(invalid: Twist) -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    cut = len(packets)
    session.receive(NavigationInput(invalid, 3, 10.4, 10.4, True, None), now=10.05)
    session.tick(state(10.06), subscriptions_ready=True, now=10.06)
    assert session.status.phase == NavigationPhase.ACTIVE
    assert session.status.rejected_input is not None
    assert session.status.fault is None and not session.status.cancellation_pending
    assert motion(packets[-1])[0] == 0
    session.tick(state(10.08), subscriptions_ready=True, now=10.08)
    assert motion(packets[-1])[0] == 0
    session.receive(sample(4, 10.1), now=10.1)
    session.tick(state(10.1), subscriptions_ready=True, now=10.1)
    assert motion(packets[-1])[0] == 1
    assert session.status.rejected_input is None
    assert all(packet.startswith(b"planner") for packet in packets[cut:])


def test_missing_native_frame_feedback_remains_a_hard_fault() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(
        replace(state(10.02), base_quat_measured=None), subscriptions_ready=True, now=10.02
    )
    assert session.status.phase == NavigationPhase.FAULT
    assert session.status.fault == "invalid_navigation_feedback"
    assert packets[-1].startswith(b"command")


@pytest.mark.parametrize("failure", ["feedback", "subscriptions", "native_latch"])
def test_hard_fault_closes_motion_before_cancellation(failure: str) -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    assert motion(packets[-1])[0] == 1
    cut = len(packets)
    latest = state(10.2)
    now = 10.2
    if failure == "feedback":
        latest = state(10)
    if failure == "native_latch":
        latest = state(10.2, latched=True, fault="planner_watchdog")
    session.tick(latest, subscriptions_ready=failure != "subscriptions", now=now)
    assert session.status.phase == NavigationPhase.FAULT
    assert session.status.cancellation_pending
    assert packets[-1].startswith(b"command")
    # A fresh nonzero input never rearms a hard fault.
    session.receive(sample(3, now), now=now)
    session.tick(state(now), subscriptions_ready=True, now=now)
    assert all(not p.startswith(b"planner") or motion(p)[0] == 0 for p in packets[cut:])


def walking(session: SonicNavigationSession, packets: list[bytes]) -> None:
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    assert motion(packets[-1])[0] == 1


def idle_since(packets: list[bytes], cut: int) -> bool:
    planner = [p for p in packets[cut:] if p.startswith(b"planner")]
    return bool(planner) and all(motion(p)[0] == 0 for p in planner)


def test_readiness_lapse_from_relay_lag_is_idle_then_fresh_proof_resumes() -> None:
    # pc2, 2026-10-15: one relay reply over Wi-Fi took >100 ms, readiness lapsed and
    # the session faulted. A timing lapse now stops walking at the same deadline but
    # resumes on fresh proof without an operator rearm.
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    walking(session, packets)
    cut = len(packets)
    session.tick(state(10.36), subscriptions_ready=True, now=10.36)  # no reply arrived
    assert idle_since(packets, cut)
    assert session.status.phase == NavigationPhase.ACTIVE and session.status.fault is None
    assert not any(p.startswith(b"command") for p in packets[cut:])
    late = NavigationInput(sample(3).twist, 3, 10.5, 10.35, True, None)  # expired in transit
    assert not session.receive(late, now=10.4)
    session.tick(state(10.42), subscriptions_ready=True, now=10.42)
    assert idle_since(packets, cut) and session.status.phase == NavigationPhase.ACTIVE
    session.receive(sample(4, 10.5), now=10.5)
    session.tick(state(10.5), subscriptions_ready=True, now=10.5)
    assert motion(packets[-1])[0] == 1


@pytest.mark.parametrize(
    "fault",
    ["navigation_inputs_unready", "stale:base_tf", "upstream_lag:odom", "upstream_unready:odom"],
)
def test_peer_timing_unready_input_is_idle_not_fault(fault: str) -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    walking(session, packets)
    cut = len(packets)
    unready = NavigationInput(sample(3).twist, 3, 10.4, 10.4, False, fault)
    assert not session.receive(unready, now=10.05)
    session.tick(state(10.36), subscriptions_ready=True, now=10.36)
    assert idle_since(packets, cut)
    assert session.status.phase == NavigationPhase.ACTIVE and session.status.fault is None
    session.receive(sample(4, 10.4), now=10.4)
    session.tick(state(10.4), subscriptions_ready=True, now=10.4)
    assert motion(packets[-1])[0] == 1


@pytest.mark.parametrize(
    "fault",
    [
        "peer_source_rollback",
        "peer_request_changed",
        "cached_velocity_changed",
        "source_stamp_rollback:odom",
        "future_receipt:lidar_raw",
        "clock_domain_mismatch:odom",
        "extrinsic_unverified",
    ],
)
def test_integrity_unready_input_stays_terminal(fault: str) -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    walking(session, packets)
    session.receive(NavigationInput(sample(3).twist, 3, 10.4, 10.4, False, fault), now=10.05)
    assert session.status.phase == NavigationPhase.FAULT and session.status.fault == fault


def test_lapsed_readiness_never_walks_on_a_fresh_velocity() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    walking(session, packets)
    # Velocity proof good until 10.45; sensor readiness only until 10.2.
    assert session.receive(NavigationInput(sample(3).twist, 3, 10.45, 10.2, True, None), now=10.1)
    session.tick(state(10.12), subscriptions_ready=True, now=10.12)
    assert motion(packets[-1])[0] == 1
    cut = len(packets)
    session.tick(state(10.25), subscriptions_ready=True, now=10.25)
    assert idle_since(packets, cut) and session.status.phase == NavigationPhase.ACTIVE


def test_two_seconds_without_readiness_faults_and_never_walks() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    walking(session, packets)
    cut = len(packets)
    # The velocity source stays fresh; sensor readiness never returns.
    for i in range(1, 12):
        now = 10.2 + 0.2 * i
        fresh_velocity = NavigationInput(
            sample(2 + i).twist, 2 + i, now + 0.35, now - 0.01, True, None
        )
        session.receive(fresh_velocity, now=now)
        session.tick(state(now), subscriptions_ready=True, now=now)
    assert idle_since(packets, cut)
    assert session.status.phase == NavigationPhase.FAULT
    assert session.status.fault == "readiness_expired"
    assert session.status.cancellation_pending and packets[-1].startswith(b"command")


def test_concurrent_begin_and_expired_prebegin_context_rejected() -> None:
    session = SonicNavigationSession(send=lambda packet: None)
    session.publisher_started(now=9.75)
    with pytest.raises(ValueError, match="readiness"):
        session.begin(state(10.4), sample(1), now=10.4)
    session.begin(state(), sample(1), now=10)
    with pytest.raises(RuntimeError, match="active"):
        session.begin(state(), sample(2), now=10)


def test_peer_cancel_and_native_idle_are_both_required_for_pose() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    request = session.status.request_id
    session.end(now=10.01)
    seq = struct.unpack("<q", packets[-1][-16:-8])[0]
    assert not session.confirm_cancel(
        {"request_id": "wrong", "cancelled": True, "state": "idle"}, now=10.02
    )
    assert not session.status.pose_ready
    assert session.confirm_cancel(
        {"request_id": request, "cancelled": True, "state": "idle"}, now=10.02
    )
    session.observe(
        state(
            10.02, high_water=seq, applied_sequence=seq, idle_ack_sequence=seq, lease_armed=False
        ),
        now=10.02,
    )
    session.tick(
        state(
            10.52, high_water=seq, applied_sequence=seq, idle_ack_sequence=seq, lease_armed=False
        ),
        subscriptions_ready=True,
        now=10.52,
    )
    assert not session.status.pose_ready
    session.confirm_peer_status(
        {
            "request_id": request,
            "cancelled": True,
            "state": "idle",
            "ready": True,
            "readiness_deadline": 10.87,
            "linear_speed_mps": 0.01,
            "yaw_rate_radps": 0.01,
            "settled_for_s": 0.5,
        },
        now=10.52,
    )
    session.tick(
        state(
            10.53, high_water=seq, applied_sequence=seq, idle_ack_sequence=seq, lease_armed=False
        ),
        subscriptions_ready=True,
        now=10.53,
    )
    assert session.status.pose_ready
    assert session.status.phase == NavigationPhase.DISARMED
    assert session.pose_ready(now=10.54)
    assert not session.pose_ready(now=10.87)
    with pytest.raises(RuntimeError, match="settling"):
        session.enter_pose(now=10.87)


def test_pose_has_no_planner_heartbeat_and_handback_requires_new_begin() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    session.enter_pose(now=20)
    session.tick(state(20), subscriptions_ready=True, now=20)
    assert packets == []
    session.handback()
    assert not session.receive(sample(5, 20), now=20)
    session.tick(state(20), subscriptions_ready=True, now=20)
    assert packets == []


@pytest.mark.parametrize(
    "negative",
    [
        {"ready": False},
        {"linear_speed_mps": 0.2},
        {"yaw_rate_radps": 0.2},
        {"settled_for_s": 0.0},
        {"readiness_deadline": 10.55},
    ],
)
def test_new_matching_negative_settling_proof_revokes_pose_admission(
    negative: dict[str, Any],
) -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.end(now=10.01)
    sequence = struct.unpack("<q", packets[-1][-16:-8])[0]
    session.confirm_cancel(
        {"request_id": session.status.request_id, "cancelled": True, "state": "idle"}, now=10.02
    )
    for now in (10.02, 10.52):
        session.tick(
            state(now, idle_ack_sequence=sequence, high_water=sequence, lease_armed=False),
            subscriptions_ready=True,
            now=now,
        )
    proof = {
        "request_id": session.status.request_id,
        "cancelled": True,
        "state": "idle",
        "ready": True,
        "readiness_deadline": 10.87,
        "linear_speed_mps": 0.01,
        "yaw_rate_radps": 0.01,
        "settled_for_s": 0.5,
    }
    assert session.confirm_peer_status(proof, now=10.52)
    session.tick(
        state(10.53, idle_ack_sequence=sequence, high_water=sequence, lease_armed=False),
        subscriptions_ready=True,
        now=10.53,
    )
    assert session.pose_ready(now=10.54)
    assert not session.confirm_peer_status({**proof, "request_id": "wrong"}, now=10.55)
    assert session.pose_ready(now=10.56)
    assert not session.confirm_peer_status({**proof, **negative}, now=10.60)
    assert not session.pose_ready(now=10.61)
    with pytest.raises(RuntimeError, match="settling"):
        session.enter_pose(now=10.61)


def test_publisher_restart_requires_quiet_interval_before_rearm() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    session.publisher_started(now=10)
    for now in (10, 10.249):
        assert not session.publisher_ready(now=now)
        with pytest.raises(RuntimeError, match="quiet interval"):
            session.begin(state(now), sample(1, now), now=now)
    assert packets == []
    assert session.publisher_ready(now=10.25)
    session.begin(state(10.25, high_water=999), sample(1, 10.25), now=10.25)
    assert len(packets) == 1
    assert struct.unpack("<q", packets[0][-16:-8])[0] > 999


def test_publisher_restart_handshake_is_above_native_high_water() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    session.publisher_started(now=9.75)
    session.begin(state(high_water=100_000_000_000), sample(1), now=10)
    header = json.loads(packets[0][7:1287].rstrip(b"\0"))
    assert header["fields"][-2]["name"] == "planner_sequence"
    assert struct.unpack("<q", packets[0][-16:-8])[0] > 100_000_000_000


def test_rearm_accepts_applied_idle_while_newer_heartbeat_is_pending() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    session.publisher_started(now=9.75)
    session.begin(state(), sample(1), now=10)
    barrier = struct.unpack("<q", packets[0][-16:-8])[0]
    session.tick(state(), subscriptions_ready=True, now=10)
    applied = struct.unpack("<q", packets[-1][-16:-8])[0]
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    assert struct.unpack("<q", packets[-1][-16:-8])[0] > applied
    session.observe(state(10.03, rearm_barrier=barrier, idle_ack_sequence=applied), now=10.03)
    assert session.status.phase == NavigationPhase.ACTIVE


def test_two_seconds_of_source_outage_closes_navigation() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    # Sensor readiness continues independently; velocity source does not.
    for now in (10.35, 10.65, 10.95, 11.25, 11.55, 11.85, 12.15, 12.35):
        frozen = NavigationInput(sample(2).twist, 2, 10.35, now + 0.35, True, None)
        session.receive(frozen, now=now)
        session.tick(state(now), subscriptions_ready=True, now=now)
    assert session.status.phase == NavigationPhase.STOPPING
    assert session.status.cancellation_pending
    assert packets[-1].startswith(b"command")


def test_heartbeats_hold_motion_despite_measured_yaw_noise() -> None:
    packets: list[bytes] = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    session.receive(sample(2), now=10)
    session.tick(state(10.02), subscriptions_ready=True, now=10.02)
    held = packets[-1][1287:1323]
    for i, yaw in enumerate((0.005, -0.005, 0.004, -0.004), start=2):
        now = 10 + i * 0.021
        feedback = replace(
            state(now), base_quat_measured=(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        )
        session.tick(feedback, subscriptions_ready=True, now=now)
        assert packets[-1][1287:1323] == held
