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

"""Peer/relay checks use real JSON sockets and measured pose progression."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import replace
import struct
import threading
from unittest.mock import call

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.g1.sonic_navigation_link import (
    SonicNavigationLink,
    bound_navigation_input,
    goal_payload,
)
from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
from dimos.robot.unitree.g1.sonic_navigation_readiness import INPUT_BUDGETS
from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule
from dimos.robot.unitree.g1.sonic_navigation_session import NavigationPhase, SonicNavigationSession
from dimos.robot.unitree.g1.test_sonic_navigation_session import arm, motion, state


@pytest.fixture
def clock(mocker):
    value = [10.0]
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_navigation_peer_module.monotonic",
        side_effect=lambda: value[0],
    )
    return value


@pytest.fixture
def peers(clock, mocker):
    with ExitStack() as cleanup:

        def create(**overrides):
            peer = SonicNavigationPeerModule(
                link_bind="tcp://127.0.0.1:*",
                extrinsic_valid=True,
                clock_domains={n: "sensor" for n in INPUT_BUDGETS},
                **overrides,
            )
            cleanup.callback(peer.stop)
            planner = mocker.Mock()
            planner.cancel_goal.return_value = True
            planner.get_state.return_value = NavigationState.IDLE
            planner.set_goal.return_value = True
            mocker.patch.object(peer, "_planner_spec", planner)
            return peer, planner

        yield create


@pytest.fixture
def peer(peers):
    return peers()[0]


def feed_inputs(peer, stamp=100.0, *, frozen=None, x=0.0, yaw=0.0):
    cloud = PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32), frame_id="odom")
    cloud.ts = 100.0 if frozen == "global_map" else stamp
    peer.observe_global_map(cloud)
    for name, handler in [("lidar_raw", peer.observe_lidar_raw), ("lio_cloud", peer.observe_lidar)]:
        sample = PointCloud2.from_numpy(np.array([[1.0, 0.0, 0.0]], dtype=np.float32))
        sample.ts = 100.0 if frozen == name else stamp
        handler(sample)
    peer.observe_imu_raw(
        Imu(ts=100.0 if frozen == "imu_raw" else stamp, linear_acceleration=Vector3(0, 0, 9.80665))
    )
    pose = Pose(position=Vector3(x, 0, 0), orientation=Quaternion.from_euler(Vector3(0, 0, yaw)))
    peer.observe_odometry(
        Odometry(
            ts=100.0 if frozen == "odom" else stamp,
            frame_id="odom",
            child_frame_id="base_link",
            pose=pose,
        )
    )
    peer.observe_tf(
        TFMessage(
            Transform(
                translation=Vector3(x, 0, 0),
                rotation=pose.orientation,
                frame_id="odom",
                child_frame_id="base_link",
                ts=100.0 if frozen == "base_tf" else stamp,
            )
        )
    )
    peer.observe_global_costmap(
        OccupancyGrid(
            grid=np.array([[0, -1], [100, 0]], dtype=np.int8),
            resolution=0.05,
            ts=100.0 if frozen == "global_costmap" else stamp,
        )
    )


def test_poll_age_comes_from_original_emission_not_poll_time(peer, clock):
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    clock[0] = 10.1
    feed_inputs(peer, 100.1)
    first = peer.poll_reply("p1")
    second = peer.poll_reply("p2")
    assert first.source_sequence == second.source_sequence == 1
    assert first.source_age == second.source_age == pytest.approx(0.1)
    assert first.twist.linear.x == 0.2


def test_first_goal_can_start_from_sensor_readiness_without_a_velocity(peer):
    feed_inputs(peer)
    readiness = bound_navigation_input(peer.poll_reply("startup"), t0=10, t1=10.01)
    assert readiness.ready and readiness.velocity_deadline <= 10.01
    packets = []
    session = SonicNavigationSession(send=packets.append)
    session.publisher_started(now=9.75)
    session.begin(state(10.01), readiness, now=10.01)
    session.tick(state(10.01), subscriptions_ready=True, now=10.01)
    sequence = struct.unpack("<q", packets[-1][-16:-8])[0]
    session.observe(
        state(
            10.01,
            high_water=sequence,
            applied_sequence=sequence,
            idle_ack_sequence=sequence,
            rearm_barrier=sequence - 1,
        ),
        now=10.01,
    )
    assert session.status.phase == NavigationPhase.ACTIVE
    assert motion(packets[-1])[0] == 0
    for now in (10.12, 10.22, 10.32, 10.42):
        readiness = bound_navigation_input(peer.poll_reply("still-idle"), t0=now, t1=now + 0.01)
        assert not session.receive(readiness, now=now + 0.01)
        session.tick(state(now + 0.01), subscriptions_ready=True, now=now + 0.01)
        assert session.status.phase == NavigationPhase.ACTIVE
        assert motion(packets[-1])[0] == 0
    assert peer.set_goal(session.status.request_id, PoseStamped(frame_id="odom"))
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    value = bound_navigation_input(peer.poll_reply("first-motion"), t0=10.44, t1=10.45)
    assert session.receive(value, now=10.45)
    session.tick(state(10.45), subscriptions_ready=True, now=10.45)
    assert motion(packets[-1])[0] == 1


@pytest.mark.parametrize("frozen,budget", INPUT_BUDGETS.items())
def test_freeze_inputs_while_twists_continue(peer, clock, frozen, budget):
    feed_inputs(peer)
    for i in range(1, 12):
        clock[0] = 10.0 + i * 0.1
        feed_inputs(peer, 100 + i * 0.1, frozen=frozen)
        peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
        reply = peer.poll_reply("poll")
        if i * 0.1 >= budget:
            assert not reply.readiness.ready
            value = bound_navigation_input(reply, t0=10, t1=10.01)
            assert not value.ready
    assert reply.source_sequence == 11


def test_stamp_rollback_cannot_recover_on_later_valid_twists(peer, clock):
    feed_inputs(peer)
    feed_inputs(peer, 99.0)
    clock[0] = 10.1
    feed_inputs(peer, 100.1)
    peer.observe_cmd_vel(Twist())
    assert peer.poll_reply("poll").readiness.fault.startswith("source_stamp_rollback:")


def test_measured_settling_uses_pose_differences_not_default_zero_twist(peer, clock):
    feed_inputs(peer)
    for i in range(1, 7):
        clock[0] = 10 + i * 0.1
        feed_inputs(peer, 100 + i * 0.1, x=i * 0.01)
    status = peer.peer_status()
    assert status["linear_speed_mps"] == pytest.approx(0.1)
    assert status["settled_for_s"] == 0
    # Speed spans the last 0.5 s of poses, so a stop is recognised up to 0.5 s late.
    for i in range(7, 18):
        clock[0] = 10 + i * 0.1
        feed_inputs(peer, 100 + i * 0.1, x=0.06)
    assert peer.peer_status()["settled_for_s"] >= 0.5


def test_standing_robot_with_lio_jitter_settles(peer, clock):
    # pc2, 2026-10-15: a standing G1's LIO pose jitters ~5 mm per 0.1 s sample, which
    # consecutive differences read as 3-5 cm/s; idle and handback never settled.
    feed_inputs(peer)
    for i in range(1, 13):
        clock[0] = 10 + i * 0.1
        feed_inputs(peer, 100 + i * 0.1, x=0.005 * (-1) ** i, yaw=0.002 * (-1) ** i)
        if i == 5:  # the first full 0.5 s window already shows 0.5 s of stillness
            assert peer.peer_status()["settled_for_s"] >= 0.5 - 1e-9
    status = peer.peer_status()
    assert status["linear_speed_mps"] <= 0.03 and abs(status["yaw_rate_radps"]) <= 0.03
    assert status["settled_for_s"] >= 0.5


def test_walking_robot_never_reads_as_settled(peer, clock):
    feed_inputs(peer)
    for i in range(1, 20):
        clock[0] = 10 + i * 0.1
        feed_inputs(peer, 100 + i * 0.1, x=0.05 * i * 0.1)  # 0.05 m/s
        assert peer.peer_status()["settled_for_s"] == 0


def test_duplicate_odometry_does_not_extend_settling_interval(peer, clock):
    feed_inputs(peer)
    clock[0] = 10.1
    feed_inputs(peer, 100.1)
    duration = peer.peer_status()["settled_for_s"]
    clock[0] = 11.0
    feed_inputs(peer, 100.1)
    status = peer.peer_status()
    assert status["settled_for_s"] == duration
    assert not status["readiness"].ready


def test_cancel_requires_completed_public_rpc_and_fresh_idle(peers, clock):
    peer, planner = peers()
    feed_inputs(peer)
    assert peer.set_goal("request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0)))
    planner.reset_mock()
    planner.get_state.return_value = NavigationState.FOLLOWING_PATH
    assert not peer.cancel("request-1")["cancelled"]
    assert planner.mock_calls == [call.cancel_goal(), call.get_state()]
    planner.get_state.return_value = NavigationState.IDLE
    assert peer.cancel("request-1")["cancelled"]
    assert not peer.cancel("request-old")["cancelled"]


def test_real_link_and_correlated_control(peers):
    peer, planner = peers()
    peer.start()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    with SonicNavigationLink(peer.bound_endpoint) as link:
        poll, t0, t1 = link.poll(None)
        assert poll.source_sequence == 1 and t1 >= t0
        result, _, _ = link.control(
            "goal", "request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0))
        )
        assert result["accepted"]
        result, _, _ = link.control("cancel", "request-1")
        assert result["cancelled"] and result["state"] == "idle"


def test_cancel_preempts_blocked_poll(peers, mocker):
    peer, planner = peers()
    peer.start()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    blocked = threading.Event()
    release = threading.Event()
    relay = SonicNavigationRelayModule(peer_endpoint=peer.bound_endpoint)
    mocker.patch.object(relay, "_publish_input")
    original = relay._link.poll

    def slow_poll(request_id):
        blocked.set()
        assert release.wait(2)
        return original(request_id)

    mocker.patch.object(relay._link, "poll", side_effect=slow_poll)
    with ExitStack() as cleanup:
        cleanup.callback(relay.stop)
        cleanup.callback(release.set)
        relay.start()
        assert blocked.wait(2)
        assert relay.set_navigation_goal(
            "request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0))
        )
        result = relay.cancel_navigation("request-1")
        assert result["cancelled"]
        assert relay._publish_input.call_args_list[0].args[0].ready is False


@pytest.fixture
def relays():
    with ExitStack() as cleanup:

        def create():
            relay = SonicNavigationRelayModule(peer_endpoint="tcp://127.0.0.1:1")
            cleanup.callback(relay.stop)
            return relay

        yield create


def test_lost_goal_reply_closes_admission(relays, mocker):
    relay = relays()
    published = mocker.patch.object(relay, "_publish_input")
    mocker.patch.object(relay._link, "control", side_effect=TimeoutError("lost goal reply"))
    with pytest.raises(TimeoutError):
        relay.set_navigation_goal(
            "request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0))
        )
    assert published.call_count == 1
    assert published.call_args.args[0].ready is False


def test_poll_during_goal_dispatch_cannot_manufacture_peer_restart(relays, peer, mocker):
    relay = relays()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    original_reply = peer.poll_reply("poll-1")
    mocker.patch.object(relay, "_publish_input")

    def acknowledge_goal(*args):
        # Independent poll can arrive before the goal's request binds remotely.
        relay.accept_poll(original_reply, t0=10, t1=10.01)
        return {"request_id": "request-1", "accepted": True, "fault": None}, 10, 10.01

    mocker.patch.object(relay._link, "control", side_effect=acknowledge_goal)
    assert relay.set_navigation_goal(
        "request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0))
    )
    value = relay.accept_poll(replace(original_reply, request_id="request-1"), t0=10, t1=10.01)
    assert value.ready


def test_empty_cloud_invalidates_previously_valid_map(peer):
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    empty = PointCloud2(ts=100.1)
    peer.observe_global_map(empty)
    assert not peer.poll_reply("poll").readiness.ready


def test_cached_sequence_cannot_extend_motion_deadline(relays, peer):
    relay = relays()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    reply = peer.poll_reply("poll")
    first = relay.accept_poll(reply, t0=10, t1=10.01)
    repeated = relay.accept_poll(replace(reply, source_age=0), t0=10.1, t1=10.11)
    assert repeated.velocity_deadline == first.velocity_deadline == pytest.approx(10.35)
    assert repeated.readiness_deadline > first.readiness_deadline


def test_cached_reply_keeps_native_heartbeat_within_original_source_deadline(relays, peer):
    relay = relays()
    packets = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    reply = peer.poll_reply("poll")
    first = relay.accept_poll(reply, t0=10, t1=10.01)
    assert session.receive(first, now=10.01)
    for tick in (10.02, 10.12, 10.22, 10.32):
        cached = relay.accept_poll(replace(reply, source_age=0), t0=tick, t1=tick + 0.01)
        assert not session.receive(cached, now=tick + 0.01)
        session.tick(state(tick + 0.01), subscriptions_ready=True, now=tick + 0.01)
        assert motion(packets[-1])[0] == 1
    session.tick(state(10.35), subscriptions_ready=True, now=10.35)
    assert motion(packets[-1])[0] == 0
    assert session.status.phase == NavigationPhase.ACTIVE
    assert session.status.fault is None


def test_source_rollback_is_delivered_as_unready_at_existing_sequence(relays, peer, mocker):
    # During a live session; without one, a restarted peer starts a new epoch instead.
    relay = relays()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    reply = replace(peer.poll_reply("poll"), request_id="request-1")
    accept_goal(relay, mocker)
    relay.accept_poll(reply, t0=10, t1=10.01)
    reset = relay.accept_poll(replace(reply, source_sequence=0), t0=10.01, t1=10.02)
    assert not reset.ready and reset.source_sequence == 1
    assert reset.fault == "peer_source_rollback"
    assert not relay.accept_poll(reply, t0=10.02, t1=10.03).ready


def test_rejected_goal_cannot_reuse_previous_acceptance(peer):
    feed_inputs(peer)
    assert peer.set_goal("request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0)))
    result = peer._control(
        {
            "operation": "goal",
            "request_id": "request-1",
            "payload": goal_payload(PoseStamped(frame_id="wrong-frame")),
        }
    )
    assert result["accepted"] is False


@pytest.mark.parametrize("frozen", INPUT_BUDGETS)
def test_frozen_sensor_ends_real_admission_even_with_new_velocities(peer, clock, frozen):
    packets = []
    session = SonicNavigationSession(send=packets.append)
    arm(session, packets)
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    clock[0] = 10.01
    feed_inputs(peer, 100.01)
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    value = bound_navigation_input(peer.poll_reply("poll"), t0=10.01, t1=10.02)
    assert session.receive(value, now=10.02)
    clock[0] = 10.01 + INPUT_BUDGETS[frozen] + 0.001
    feed_inputs(peer, 100 + clock[0] - 10, frozen=frozen)
    peer.observe_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    stopped = bound_navigation_input(peer.poll_reply("poll"), t0=clock[0], t1=clock[0] + 0.01)
    assert not session.receive(stopped, now=clock[0] + 0.01)
    assert session.status.phase == NavigationPhase.FAULT and session.status.cancellation_pending


def test_link_cancel_slot_runs_while_real_poll_has_no_reply(peers, mocker):
    peer, planner = peers()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    assert peer.set_goal("request-1", PoseStamped(frame_id="odom", position=Vector3(0.5, 0, 0)))
    missing_reply = threading.Event()
    original = peer._reply

    def drop_poll(sock, identity, request, payload):
        if request["operation"] == "poll":
            missing_reply.set()
        else:
            original(sock, identity, request, payload)

    mocker.patch.object(peer, "_reply", side_effect=drop_poll)
    peer.start()
    with (
        SonicNavigationLink(peer.bound_endpoint) as link,
        ThreadPoolExecutor(max_workers=1) as worker,
    ):
        pending = worker.submit(link.poll, "request-1")
        assert missing_reply.wait(2)
        result, _, _ = link.control("cancel", "request-1")
        assert result["cancelled"]
        with pytest.raises(TimeoutError):
            pending.result(timeout=1)


def test_old_cancel_ack_cannot_authorize_current_session(relays, mocker):
    relay = relays()
    published = mocker.patch.object(relay, "_publish_input")
    mocker.patch.object(
        relay._link,
        "control",
        return_value=(
            {"request_id": "request-old", "cancelled": True, "state": "idle", "fault": None},
            10,
            10.01,
        ),
    )
    result = relay.cancel_navigation("request-current")
    assert not result["cancelled"] and published.call_args.args[0].ready is False


def test_new_session_can_end_before_first_goal_after_previous_cancel(peers, mocker):
    peer, planner = peers()
    feed_inputs(peer)
    peer.start()
    relay = SonicNavigationRelayModule(peer_endpoint=peer.bound_endpoint)
    mocker.patch.object(relay, "_publish_input")
    with ExitStack() as cleanup:
        cleanup.callback(relay.stop)
        assert relay.set_navigation_goal("request-a", PoseStamped(frame_id="odom"))
        assert relay.cancel_navigation("request-a")["cancelled"]
        result = relay.cancel_navigation("request-b")
        assert result["cancelled"] and result["request_id"] == "request-b"
        assert not relay.cancel_navigation("request-a")["cancelled"]


def test_new_session_cancel_binding_survives_lost_first_request(peers, mocker):
    peer, planner = peers()
    feed_inputs(peer)
    peer.start()
    relay = SonicNavigationRelayModule(peer_endpoint=peer.bound_endpoint)
    mocker.patch.object(relay, "_publish_input")
    with ExitStack() as cleanup:
        cleanup.callback(relay.stop)
        assert relay.set_navigation_goal("request-a", PoseStamped(frame_id="odom"))
        assert relay.cancel_navigation("request-a")["cancelled"]
        original = relay._link.control
        attempts = []

        def lose_first(*args, **kwargs):
            attempts.append(kwargs)
            if len(attempts) == 1:
                # A concurrent poll is expected to report the previous ID until
                # the independent control request binds the next session.
                value = relay.accept_poll(peer.poll_reply("during-cancel"), t0=10, t1=10.01)
                assert not value.ready
                raise TimeoutError("first cancel request lost")
            return original(*args, **kwargs)

        mocker.patch.object(relay._link, "control", side_effect=lose_first)
        with pytest.raises(TimeoutError):
            relay.cancel_navigation("request-b")
        assert relay.cancel_navigation("request-b")["cancelled"]
        assert relay._reset_fault is None


def test_new_goal_lost_before_binding_can_still_be_cancelled(peers, mocker):
    peer, planner = peers()
    feed_inputs(peer)
    peer.start()
    relay = SonicNavigationRelayModule(peer_endpoint=peer.bound_endpoint)
    mocker.patch.object(relay, "_publish_input")
    with ExitStack() as cleanup:
        cleanup.callback(relay.stop)
        assert relay.set_navigation_goal("request-a", PoseStamped(frame_id="odom"))
        assert relay.cancel_navigation("request-a")["cancelled"]
        original = relay._link.control

        def lose_goal(operation, *args, **kwargs):
            if operation == "goal":
                raise TimeoutError("new goal never reached the peer")
            return original(operation, *args, **kwargs)

        mocker.patch.object(relay._link, "control", side_effect=lose_goal)
        with pytest.raises(TimeoutError):
            relay.set_navigation_goal("request-b", PoseStamped(frame_id="odom"))
        assert relay.cancel_navigation("request-b")["cancelled"]


def test_delayed_poll_from_goal_dispatch_is_discarded_after_ack(peers, mocker):
    peer, planner = peers()
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    peer.start()
    relay = SonicNavigationRelayModule(peer_endpoint=peer.bound_endpoint)
    mocker.patch.object(relay, "_publish_input")
    received = threading.Event()
    release = threading.Event()
    original_poll = relay._link.poll

    def delayed_poll(request):
        value = original_poll(request)
        received.set()
        assert release.wait(2)
        relay._stopping.set()
        return value

    def goal_ack(*args):
        relay.start()
        assert received.wait(2)
        return {"request_id": "request-a", "accepted": True, "fault": None}, 10, 10.01

    mocker.patch.object(relay._link, "poll", side_effect=delayed_poll)
    mocker.patch.object(relay._link, "control", side_effect=goal_ack)
    with ExitStack() as cleanup:
        cleanup.callback(relay.stop)
        cleanup.callback(release.set)
        assert relay.set_navigation_goal("request-a", PoseStamped(frame_id="odom"))
        release.set()
        relay._thread.join(timeout=2)
        assert not relay._thread.is_alive()
        assert relay._reset_fault is None


def test_settling_proof_rejects_status_round_trip_over_100ms(relays, peer, mocker):
    relay = relays()
    feed_inputs(peer)
    result = peer.peer_status()
    mocker.patch.object(relay._link, "control", return_value=(result, 10, 10.15))
    with pytest.raises(ValueError, match="100 ms"):
        relay.get_navigation_peer_status()


def accept_goal(relay, mocker, request_id="request-1"):
    mocker.patch.object(
        relay._link,
        "control",
        return_value=({"request_id": request_id, "accepted": True, "fault": None}, 10, 10.01),
    )
    assert relay.set_navigation_goal(request_id, PoseStamped(frame_id="odom"))


def restarted_peer_replies(peer):
    feed_inputs(peer)
    peer.observe_cmd_vel(Twist())
    return peer.poll_reply("poll")


@pytest.mark.parametrize("confirmed", [True, False])
def test_restarted_peer_needs_confirmed_cancel_for_a_new_epoch(relays, peer, mocker, confirmed):
    relay = relays()
    mocker.patch.object(relay, "_publish_input")
    reply = restarted_peer_replies(peer)
    accept_goal(relay, mocker)
    old = relay.accept_poll(
        replace(reply, request_id="request-1", source_sequence=5), t0=10, t1=10.01
    )
    assert old.ready and old.source_sequence == 5
    # The restarted peer counts from 1 again: the live session latches closed.
    restarted = relay.accept_poll(replace(reply, source_sequence=1), t0=10.01, t1=10.02)
    assert not restarted.ready and restarted.fault == "peer_source_rollback"
    mocker.patch.object(
        relay._link,
        "control",
        return_value=(
            {"request_id": "request-1", "cancelled": confirmed, "state": "idle", "fault": None},
            10.02,
            10.03,
        ),
    )
    assert relay.cancel_navigation("request-1")["cancelled"] is confirmed
    fresh = relay.accept_poll(
        replace(reply, request_id="request-1", source_sequence=2), t0=10.03, t1=10.04
    )
    if not confirmed:
        assert not fresh.ready and fresh.fault == "peer_source_rollback"
        return
    assert fresh.ready and fresh.source_sequence > old.source_sequence
    repeated = relay.accept_poll(
        replace(reply, request_id="request-1", source_sequence=2), t0=10.04, t1=10.05
    )
    assert repeated.source_sequence == fresh.source_sequence  # a cached emission stays cached


def test_restarted_peer_without_live_session_starts_new_epoch(relays, peer, mocker):
    relay = relays()
    mocker.patch.object(relay, "_publish_input")
    reply = restarted_peer_replies(peer)
    old = relay.accept_poll(replace(reply, source_sequence=5), t0=10, t1=10.01)
    restarted = relay.accept_poll(replace(reply, source_sequence=1), t0=10.01, t1=10.02)
    assert restarted.ready and restarted.fault is None
    assert restarted.source_sequence > old.source_sequence


def test_pending_goal_refreshes_readiness_but_holds_motion(relays, peer, mocker):
    relay = relays()
    published = mocker.patch.object(relay, "_publish_input")
    reply = replace(restarted_peer_replies(peer), request_id="request-1")
    accept_goal(relay, mocker)
    walking = replace(reply, source_sequence=5, twist=Twist(linear=Vector3(0.2, 0, 0)))
    held = relay.accept_poll(walking, t0=10, t1=10.01)
    during: list = []

    def goal_in_flight(*args, **kwargs):
        newer = replace(reply, source_sequence=7, twist=Twist(linear=Vector3(0.1, 0, 0)))
        during.append(relay.accept_poll(newer, t0=10.15, t1=10.16))
        during.append(relay.accept_poll(replace(reply, source_sequence=1), t0=10.17, t1=10.18))
        return {"request_id": "request-1", "accepted": True, "fault": None}, 10.2, 10.21

    mocker.patch.object(relay._link, "control", side_effect=goal_in_flight)
    assert relay.set_navigation_goal("request-1", PoseStamped(frame_id="odom"))
    refreshed, rollback = during
    # Readiness keeps flowing while the goal is pending; motion stays the held proof.
    assert refreshed.ready and refreshed.readiness_deadline > held.readiness_deadline
    assert refreshed.source_sequence == held.source_sequence
    assert refreshed._velocity == held._velocity
    assert refreshed.velocity_deadline == held.velocity_deadline
    assert published.call_args_list[-1].args[0] is refreshed
    assert not rollback.ready


def test_new_relay_clears_foreign_request_latch_only_for_a_peer_without_one(relays, peer, mocker):
    # A restarted pc2 relay owns no session; the peer may still hold an earlier one.
    relay = relays()
    mocker.patch.object(relay, "_publish_input")
    reply = restarted_peer_replies(peer)
    foreign = relay.accept_poll(
        replace(reply, request_id="old", source_sequence=9), t0=10, t1=10.01
    )
    assert not foreign.ready and foreign.fault == "peer_request_changed"
    still = relay.accept_poll(
        replace(reply, request_id="old", source_sequence=10), t0=10.01, t1=10.02
    )
    assert not still.ready  # the earlier session's commands can never be adopted
    fresh = relay.accept_poll(
        replace(reply, request_id=None, source_sequence=1), t0=10.02, t1=10.03
    )
    assert fresh.ready and fresh.fault is None
