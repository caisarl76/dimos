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

"""Two robots' arms started together by run_arms, each against its own scripted execution.

One feeder per arm plays body telemetry as a function of the sample's receipt time after the
arm's chunk-0 submission (s), so both robots "move" relative to their own start.
"""

from dataclasses import replace
from functools import partial
import math
import threading
import time

import numpy as np
import pytest

from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, RPCClient as RPCTransport
from dimos.robot.unitree.g1 import sonic_arm_skill
from dimos.robot.unitree.g1.sonic_arm_reference import (
    Keyframe,
    group_margins,
    pose_margins,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    POSES,
    SonicArm,
    SonicArmSkillConfig,
    _Watch,
    run_arms,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_test_support import (
    PEER_TOKEN,
    FakeEncoder,
    fast_keyframe_plan,
    feedback,
    feeding_arms,
    no_pose,
    real_hold_plan,
    task,
    tokens,
    wait_until,
)


@pytest.fixture
def arms(dual_executions, mocker):
    """Robot 222's and 223's SonicArm, each on its own scripted execution, with ~1 s plans."""
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", fast_keyframe_plan)
    mocker.patch.object(sonic_arm_skill, "token_schedule", partial(token_schedule, blend_s=0.2))
    config = SonicArmSkillConfig(encoder_path="unused.onnx", return_watch_seconds=0.0)
    return [
        SonicArm(name, config, execution, FakeEncoder(None), None)
        for name, (execution, _) in zip(("222", "223"), dual_executions, strict=True)
    ]


class InProcessRPC(RPCTransport):
    """An RPC transport to an in-process module, with call_sync's own per-method timeouts. Once
    `dead` is set nothing answers, as with a dead or wedged execution worker."""

    def __init__(self, module):
        self.rpc_timeouts = {}
        self.default_rpc_timeout = DEFAULT_RPC_TIMEOUT
        self.module = module
        self.dead = threading.Event()

    def call(self, name, arguments, cb):
        args, kwargs = arguments

        def serve():
            try:
                result = getattr(self.module, name.rsplit("/", 1)[-1])(*args, **kwargs)
            except Exception as exc:
                result = exc
            if not self.dead.is_set():
                cb(result)

        if not self.dead.is_set():
            threading.Thread(target=serve, daemon=True).start()
        return lambda: None


@pytest.fixture
def proxied(arms, dual_executions):
    """223's arm over a worker-style RPC proxy to its execution; returns the proxy's transport."""
    transport = InProcessRPC(dual_executions[1][0])
    proxy = RPCClient(None, SonicExecutionModule, "g1_223", rpc=transport)
    arms[1] = SonicArm("223", arms[1].config, proxy, arms[1]._encoder, None)
    return transport


def ramp(s):
    """Out from 0.3 s, back from 0.7 s, both over 0.3 s: 10% progress at 0.33 s for any stroke."""
    return float(np.clip(min((s - 0.3) / 0.3, (1.0 - s) / 0.3), 0.0, 1.0))


def swing(arm, degrees=20.0, hold=1.0):
    """A job: arm's prepare for a left-shoulder-pitch swing forward."""
    delta = np.zeros(29)
    delta[15] = -math.radians(degrees)
    return arm, partial(
        arm.prepare,
        f"swing{degrees:g}",
        lambda base, hands: [Keyframe(base + delta, hold)],
        group_margins(0.15, 0.2, 0.3),
        require_check=False,
    )


def pose(arm, name="left_arm_forward_horizontal"):
    """A job: arm's prepare for a one-keyframe full-range left-arm pose."""

    def keyframes_of(base, hands):
        goal = base.copy()
        goal[15:19] = POSES[name]
        return [Keyframe(goal, 1.0)]

    return arm, partial(
        arm.prepare, name, keyframes_of, pose_margins(0.15, 0.45, 0.35), require_check=False
    )


def peers(dual_executions):
    return [peer for _, peer in dual_executions]


def planner_token_only(peer):
    streamed = tokens(peer.snapshot())
    return not len(streamed) or np.allclose(streamed, PEER_TOKEN)


def ends_at_the_planner_smoothly(peer):
    streamed = tokens(peer.snapshot())[:, 0]
    return streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3) and (
        np.abs(np.diff(streamed)).max() <= 0.05 + 1e-6
    )


def test_both_start_together_and_hand_back(arms):
    with feeding_arms(arms):
        result = run_arms([swing(arm) for arm in arms])
    assert result.message.startswith("222: swing20 reached"), result
    assert "; 223: swing20 reached" in result.message, result
    for arm in arms:
        status = task(arm)
        assert status["outcome"] == "finished" and status["handback_confirmed"], status
        assert result.metadata[arm.name]["reached_fraction"] == pytest.approx(1.0, abs=0.05)
    assert result.metadata["start_skew_s"] < 0.3, result.metadata


def test_both_peer_never_arms_moves_neither(arms, dual_executions, mocker):
    mocker.patch.object(arms[1]._execution, "start_task", side_effect=RuntimeError("busy"))
    with feeding_arms(arms), pytest.raises(RuntimeError, match="223: execution refused the task"):
        run_arms([swing(arm) for arm in arms])
    assert all(planner_token_only(peer) for peer in peers(dual_executions))
    status = task(arms[0])
    assert status is None or status["handback_confirmed"], status


def test_both_peer_arming_times_out(arms, dual_executions, mocker):
    waits = []

    class RecordingBarrier(threading.Barrier):
        def wait(self, timeout=None):
            started = time.monotonic()
            try:
                return super().wait(timeout)
            except threading.BrokenBarrierError:
                waits.append(time.monotonic() - started)
                raise

    mocker.patch.object(threading, "Barrier", RecordingBarrier)
    real = arms[1]._task

    def never_waiting():
        status = real()
        return status | {"phase": "arming"} if status["phase"] == "waiting_for_chunk" else status

    mocker.patch.object(arms[1], "_task", never_waiting)
    with feeding_arms(arms), pytest.raises(RuntimeError, match="223") as raised:
        run_arms([swing(arm) for arm in arms])
    assert "222: another robot did not arm" in str(raised.value)
    assert len(waits) == 1 and waits[0] < 1.0 / 2 + 0.5, waits  # first_chunk_timeout / 2
    assert all(planner_token_only(peer) for peer in peers(dual_executions))
    assert task(arms[0])["handback_confirmed"], task(arms[0])
    assert task(arms[1])["outcome"] == "cancelled", task(arms[1])


def test_both_stale_feedback_refuses_before_sending(arms, dual_executions):
    peer = dual_executions[1][1]
    peer.pause_feedback.set()
    try:
        wait_until(lambda: arms[1]._execution.get_feedback() is None)
        with pytest.raises(RuntimeError, match="223: .*nothing sent to any robot"):
            run_arms([swing(arm) for arm in arms])
    finally:
        peer.pause_feedback.clear()
    assert all(no_pose(peer) for peer in peers(dual_executions))


def test_both_recheck_after_all_prepared(arms, dual_executions, mocker):
    moved = threading.Event()
    encode = arms[1]._encoder.encode

    def slow_encode(poses):
        time.sleep(0.3)
        moved.set()  # 222's knee moves while 223 encodes.
        return encode(poses)

    mocker.patch.object(arms[1]._encoder, "encode", side_effect=slow_encode)
    real = arms[0]._execution.get_feedback

    def get_feedback():
        state = real()
        if state is None or not moved.is_set():
            return state
        q = list(state.body_q)
        q[3] += 0.1
        return replace(state, body_q=tuple(q))

    mocker.patch.object(arms[0]._execution, "get_feedback", get_feedback)
    with pytest.raises(RuntimeError, match="222: .*nothing sent"):
        run_arms([swing(arm) for arm in arms])
    assert all(no_pose(peer) for peer in peers(dual_executions))


def test_both_invalid_chunk_refused_before_sending(arms, dual_executions, mocker):
    encode = arms[1]._encoder.encode
    mocker.patch.object(arms[1]._encoder, "encode", side_effect=lambda poses: 100 * encode(poses))
    with pytest.raises(RuntimeError, match="223: token bound exceeded"):
        run_arms([swing(arm) for arm in arms])
    assert all(no_pose(peer) for peer in peers(dual_executions))


def test_onset_ignores_idle_drift_and_reports_unequal_strokes(arms):
    with feeding_arms(arms, shape=ramp, noise=(0.01, 0.0)):
        result = run_arms([swing(arms[0], 10.0), swing(arms[1], 30.0)])
    meta = result.metadata
    assert math.isfinite(meta["start_skew_s"]) and meta["start_skew_s"] < 0.3, meta
    assert meta["submit_skew_s"] < 0.05, meta
    for name in ("222", "223"):
        report = meta[name]
        assert report["moved_at"] > report["send_at"], report
        assert report["ack_at"] >= report["send_at"], report
    assert min(meta[name]["send_at"] for name in ("222", "223")) == 0.0


def test_onset_uses_receipt_time_not_handler_time():
    arm = SonicArm("222", SonicArmSkillConfig(encoder_path="unused.onnx"), None, None, None)
    delta = np.zeros(29)
    delta[15] = -0.3
    watch = _Watch(base=np.zeros(29), allowed=np.ones(29), max_dq=3.0, delta=delta)
    arm._watch = watch

    def sample(received, joint15):
        q = np.zeros(29)
        q[15] = joint15
        return replace(feedback(received), body_q=tuple(q))

    arm.handle_feedback(sample(9.99, 0.0))
    watch.send_at = 10.0
    arm.handle_feedback(sample(9.98, -0.1))  # Received before the send, handled after it.
    assert watch.moved_at is None
    arm.handle_feedback(sample(10.01, -0.029))  # Under 10% of the stroke.
    assert watch.moved_at is None
    arm.handle_feedback(sample(10.02, -0.1))
    assert watch.moved_at == 10.02


def test_onset_ignores_secondary_joint_drift_in_the_blend(arms):
    """On 222 the shoulder roll drifts 0.05 rad in the token blend; both strokes start at 0.3 s."""

    def drift(s):
        return {16: 0.05 if s > 0.05 else 0.0}

    with feeding_arms(arms, shape=ramp, extra=(drift, None)):
        result = run_arms([pose(arm) for arm in arms])
    assert result.metadata["start_skew_s"] < 0.1, result.metadata


def test_both_envelope_on_one_retreats_both(arms, dual_executions):
    def knee(s):
        return {3: 0.2 if s > 0.4 else 0.0}

    with feeding_arms(arms, extra=(knee, None)):
        result = run_arms([swing(arm) for arm in arms])
    first, second = result.message.split("; 223: ")
    assert "stopped early" in first and "left_knee" in first, result
    assert "stopped early: 222: left_knee" in second, result
    for (_, peer), arm in zip(dual_executions, arms, strict=True):
        assert ends_at_the_planner_smoothly(peer)
        assert task(arm)["handback_confirmed"], task(arm)


def test_both_stop_request_retreats_both(arms, dual_executions):
    def stop_mid_stream():
        wait_until(lambda: all(a._watch is not None and a._watch.send_at for a in arms))
        time.sleep(0.3)
        for arm in arms:
            arm.stop.set()

    stopper = threading.Thread(target=stop_mid_stream, daemon=True)
    stopper.start()
    with feeding_arms(arms):
        result = run_arms([swing(arm) for arm in arms])
    stopper.join(2)
    first, second = result.message.split("; 223: ")
    assert "stop requested" in first and "stop requested" in second, result
    for (_, peer), arm in zip(dual_executions, arms, strict=True):
        assert ends_at_the_planner_smoothly(peer)
        assert task(arm)["handback_confirmed"], task(arm)


def test_both_with_different_start_poses_reports_both(arms, mocker):
    real = arms[0]._execution.get_feedback

    def raised():
        state = real()
        if state is None:
            return None
        q = list(state.body_q)
        q[15] += 0.2
        return replace(state, body_q=tuple(q))

    mocker.patch.object(arms[0]._execution, "get_feedback", raised)
    with feeding_arms(arms):
        result = run_arms([pose(arm) for arm in arms])
    for arm in arms:
        report = result.metadata[arm.name]
        assert report["motion"] == "left_arm_forward_horizontal", report
        assert task(arm)["outcome"] == "finished" and task(arm)["handback_confirmed"]
    assert result.message.count("pose and returned") == 2, result
    planned = [result.metadata[name]["planned_deg"] for name in ("222", "223")]
    assert planned[0]["left_shoulder_pitch"] == planned[1]["left_shoulder_pitch"]  # absolute


def test_both_mixed_outcome_retreats_peer_and_fails_naming_both(arms, dual_executions, mocker):
    def plan(base, keyframes, **kwargs):  # Holds up to 1 s: 222 holds 1 s, 223 0.1 s.
        capped = [replace(k, hold_s=min(k.hold_s, 1.0)) for k in keyframes]
        return real_hold_plan(base, capped, **kwargs)

    mocker.patch.object(sonic_arm_skill, "keyframe_plan", plan)
    mocker.patch.object(arms[1], "_wait_done", return_value=None)
    with feeding_arms(arms), pytest.raises(RuntimeError) as raised:
        run_arms([swing(arms[0], hold=1.0), swing(arms[1], hold=0.1)])
    first, second = str(raised.value).split("; 223: ")
    assert first.startswith("222: ") and "stopped early" in first and "223" in first, first
    assert "planner handback unconfirmed" in second, second
    assert ends_at_the_planner_smoothly(dual_executions[0][1])


def test_both_feedback_lost_mid_stream_retreats_the_peer(arms, dual_executions):
    def drop_223_feedback():
        wait_until(lambda: all(a._watch is not None and a._watch.send_at for a in arms))
        time.sleep(0.3)
        dual_executions[1][1].pause_feedback.set()

    dropper = threading.Thread(target=drop_223_feedback, daemon=True)
    dropper.start()
    try:
        with feeding_arms(arms):
            try:
                result = run_arms([swing(arm, hold=3.0) for arm in arms])
                message = result.message
            except RuntimeError as error:
                message = str(error)
    finally:
        dropper.join(2)
        dual_executions[1][1].pause_feedback.clear()
    first = message.split("; 223: ")[0]
    assert first.startswith("222: ") and "stopped early" in first and "223" in first, message
    assert "223: session ended: feedback stale" in first, message
    assert ends_at_the_planner_smoothly(dual_executions[0][1])
    assert task(arms[0])["handback_confirmed"], task(arms[0])


def test_one_job_is_the_single_arm_path(arms, dual_executions, mocker):
    barrier = mocker.spy(threading, "Barrier")
    with feeding_arms(arms):
        result = run_arms([swing(arms[0])])
    assert result.message.startswith("swing20 reached"), result  # No name prefix.
    assert "start_skew_s" not in result.metadata and barrier.call_count == 0
    assert no_pose(dual_executions[1][1])
    with pytest.raises(ValueError, match="the arm is already there"):  # Raised as it is.
        run_arms([swing(arms[0], degrees=0.0)])


@pytest.mark.parametrize("late", [0, 1])
def test_both_stop_while_one_arms_sends_no_chunk_to_either(arms, dual_executions, mocker, late):
    """A stop while the late arm arms, the other already armed and waiting: no robot gets a chunk."""
    early = arms[1 - late]
    real = arms[late]._execution.start_task
    submits = [mocker.spy(arm._execution, "submit_chunk") for arm in arms]

    def start_then_stop():
        wait_until(lambda: (t := task(early)) is not None and t["phase"] == "waiting_for_chunk")
        started = real()
        for arm in arms:  # What stop_arm does to a joint start's arms.
            arm.stop.set()
        return started

    mocker.patch.object(arms[late]._execution, "start_task", side_effect=start_then_stop)
    with feeding_arms(arms), pytest.raises(RuntimeError) as raised:
        run_arms([swing(arm) for arm in arms])
    assert str(raised.value) == (
        "222: stopped on request; nothing sent; 223: stopped on request; nothing sent"
    )
    assert [s.call_count for s in submits] == [0, 0]
    assert all(planner_token_only(peer) for peer in peers(dual_executions))
    for arm in arms:
        assert task(arm)["handback_confirmed"], task(arm)


def test_each_arm_samples_its_own_hold_until_a_peer_abort(arms, mocker):
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", real_hold_plan)  # real holds

    def stamped(s):  # The feeder's time since this arm's own chunk 0, in the left wrist pitch.
        return {20: 0.01 * s}

    def knee(s):  # 222 leaves its envelope mid-hold.
        return {20: 0.01 * s, 3: 0.2 if s > 1.6 else 0.0}

    with feeding_arms(arms, extra=(knee, stamped)):
        result = run_arms([swing(arm, hold=2.0) for arm in arms])
    assert "stopped early: 222: left_knee" in result.message, result
    for arm in arms:
        watch = arm._watch
        assert watch is not None and watch.hold_rows is not None
        assert watch.plan_start == watch.send_at
        times = [(q[20] - watch.base[20]) / 0.01 for q in watch.hold_q]
        assert result.metadata[arm.name]["hold_samples"] == len(times) > 0, arm.name
        assert watch.hold_rows[0] / 50 - 0.03 <= min(times), arm.name
        assert max(times) <= 1.6 + 0.1, arm.name  # Neither collects after the abort.


def real_holds(mocker):
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", real_hold_plan)


def both_sent(arms):
    return all(a._watch is not None and a._watch.send_at for a in arms)


def timed_submits(arm, mocker):
    """(sequence, monotonic time) of each submit_chunk call on arm's in-process execution."""
    submits = []
    real = arm._execution.submit_chunk

    def submit(task_id, sequence, *args):
        submits.append((sequence, time.monotonic()))
        return real(task_id, sequence, *args)

    mocker.patch.object(arm._execution, "submit_chunk", side_effect=submit)
    return submits


def test_both_peer_rpc_error_mid_stream_retreats_the_healthy_arm(arms, dual_executions, mocker):
    """223's status RPC fails and its cancel blocks until 222's task is done: 222 still retreats."""
    broken = threading.Event()
    status, cancel = arms[1]._execution.get_status, arms[1]._execution.cancel_task

    def failing_status():
        if broken.is_set():
            raise RuntimeError("execution worker lost")
        return status()

    def blocking_cancel(task_id):  # An RPC that only returns once 222 is done either way.
        wait_until(lambda: task(arms[0])["phase"] == "done")
        return cancel(task_id)

    real_holds(mocker)
    mocker.patch.object(arms[1]._execution, "get_status", side_effect=failing_status)
    mocker.patch.object(arms[1]._execution, "cancel_task", side_effect=blocking_cancel)

    def break_223():
        wait_until(lambda: both_sent(arms))
        time.sleep(0.3)
        broken.set()

    breaker = threading.Thread(target=break_223, daemon=True)
    breaker.start()
    with feeding_arms(arms), pytest.raises(RuntimeError) as raised:
        run_arms([swing(arm, hold=2.0) for arm in arms])
    breaker.join(2)
    first, second = str(raised.value).split("; 223: ")
    assert first.startswith("222: ") and "stopped early: 223: " in first, first
    assert "execution worker lost" in first and "execution worker lost" in second, raised.value
    assert ends_at_the_planner_smoothly(dual_executions[0][1])
    assert task(arms[0])["handback_confirmed"], task(arms[0])


def test_both_peer_rpc_hang_mid_stream_retreats_the_healthy_arm_promptly(
    arms, dual_executions, proxied, mocker
):
    """223's worker stops answering mid-stream: its bounded RPC fails and 222 retreats at once."""
    real_holds(mocker)
    submits = timed_submits(arms[0], mocker)
    hung: list[float] = []

    def hang_223():
        wait_until(lambda: both_sent(arms))
        time.sleep(0.3)
        hung.append(time.monotonic())
        proxied.dead.set()

    hanger = threading.Thread(target=hang_223, daemon=True)
    hanger.start()
    started = time.monotonic()
    with feeding_arms(arms), pytest.raises(RuntimeError) as raised:
        run_arms([swing(arm, hold=2.0) for arm in arms])
    elapsed = time.monotonic() - started
    hanger.join(2)
    first, second = str(raised.value).split("; 223: ")
    assert first.startswith("222: ") and "stopped early: 223: " in first, first
    assert "timed out" in first, raised.value
    # The operator's cue to check 223's controller, not a bare RPC error an agent might retry.
    assert second.startswith(
        "planner handback unconfirmed (TimeoutError: RPC call to 'g1_223/"
    ) and second.endswith("check the controller. Journal: unavailable"), second
    retreats = [t for sequence, t in submits if sequence > 0]
    assert retreats and retreats[0] - hung[0] < 1.5, (retreats, hung)
    assert elapsed < 15, elapsed  # Not the transport's 120 s default.
    assert ends_at_the_planner_smoothly(dual_executions[0][1])
    assert task(arms[0])["handback_confirmed"], task(arms[0])
