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

"""The arm skill against the real SonicExecutionModule and its scripted native peer.

A fake encoder writes the planned left-shoulder-pitch offset into token value 0; a feeder
thread plays body telemetry into the skill's watch. The plan is shortened to about 1.2 s.
"""

from contextlib import contextmanager
from dataclasses import replace
from functools import partial
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest

from dimos.agents.mcp.tool_stream import NOTIFICATIONS_MESSAGE_METHOD, TOOL_STREAM_STOPPED_METHOD
from dimos.agents.skill_result import SkillResult
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.coordination.blueprint_config.sources import validate_global_values
from dimos.core.module import Module
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_reference, sonic_arm_skill, sonic_test_support
from dimos.robot.unitree.g1.blueprints.agentic.unitree_g1_sonic_arm_agentic import (
    unitree_g1_sonic_arm_agentic,
)
from dimos.robot.unitree.g1.imported_gestures import AUTHORED, HAND_LIMITS, HAND_ORDER, RETAINED
from dimos.robot.unitree.g1.sonic_arm_reference import (
    PEAK_SPEED,
    RATE,
    Keyframe,
    arm_poses,
    group_margins,
    keyframe_plan,
    min_jerk,
    pose_margins,
    stroke_plan,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    ALL_LEG_JOINTS,
    GESTURES,
    WIDE_LEG_JOINTS,
    SonicArmSkillContainer,
    gesture_margins,
    query_boxes,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_session import SessionPhase, SonicModelProfile, SonicTaskSession
from dimos.robot.unitree.g1.sonic_test_support import (
    FAKE_GAIN,
    LEASE_STALL_RERUN,
    PEER_TOKEN,
    ROBOT_IDLE,
    SCENE,
    WAIT_S,
    FakeEncoder,
    FakeVl,
    IMAGE_640x480,
    fast_keyframe_plan,
    feedback,
    frames,
    gate_start,
    needs_scene,
    pose_values,
    real_hold_plan,
    stop_in_background,
    tokens,
    unheld,
    wait_until,
)
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_ARM_SYSTEM_PROMPT

pytestmark = LEASE_STALL_RERUN

LEAD_IN_ROWS = 10  # The fixture's 0.2 s token blend at 50 Hz.


@pytest.fixture
def execution_config(peer, tmp_path):
    """test_sonic_execution_module's scripted peer, with an observation age covering the plan."""
    return dict(
        reference_bind="tcp://127.0.0.1:*",
        feedback_endpoint=peer.feedback_endpoint,
        journal_dir=tmp_path,
        profile=SonicModelProfile(
            name="scripted-peer",
            source_fps=50,
            right_hand_order="native",
            handback_policy="native_planner",
            seed_seconds=0.06,
            blend_seconds=0.04,
            first_chunk_timeout=1.0,
            handback_timeout=0.4,
            max_observation_age=6.0,
            # Execution module, skill, scripted peer and feeders share one process here, and the
            # event loop occasionally stalls ~170 ms (about 1 run in 30): allow up to the 200 ms lease.
            max_tick_gap=0.19,
            hand_step_limit=PEAK_SPEED / RATE,  # as SONIC_ARM_PROFILE
        ),
    )


@pytest.fixture
def skill(real_launch_skill, mocker):
    """The skill with each motion run on the calling thread: returns its SkillResult or raises."""
    mocker.patch.object(real_launch_skill[0], "_launch", lambda tool, prepare: prepare()())
    return real_launch_skill


@pytest.fixture
def real_launch_skill(execution, tool_transport, mocker):
    mocker.patch.object(sonic_arm_skill, "SonicEncoder", FakeEncoder)
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", fast_keyframe_plan)
    mocker.patch.object(sonic_arm_skill, "token_schedule", partial(token_schedule, blend_s=0.2))
    module = SonicArmSkillContainer(encoder_path="unused.onnx", return_watch_seconds=0.0)
    mocker.patch.object(module, "_execution", execution[0], create=True)
    transport = make_transport(f"/arm-skill-test/{uuid4().hex}", SonicFeedback)
    module.set_transport("feedback", transport)
    try:
        transport.start()
        module.start()
        yield module, execution[1]
    finally:
        module.stop()
        transport.stop()


@contextmanager
def feeding(module, body_q=lambda t: np.zeros(29), hands=None):
    """A feeder plays body telemetry into the skill (t = seconds since entering).

    `hands(t)`, if given, plays the 14 Dex3 hand readings too (left then right). Yields a
    function that stops the feeder early.
    """
    stopped = threading.Event()

    def feed():
        start = time.monotonic()
        while not stopped.is_set():
            state = module._execution.get_feedback()
            if state is not None:
                t = time.monotonic() - start
                q = tuple(float(v) for v in body_q(t))
                state = replace(state, body_q=q)
                if hands is not None:
                    h = [float(v) for v in hands(t)]
                    state = replace(state, left_hand_q=tuple(h[:7]), right_hand_q=tuple(h[7:]))
                module.handle_feedback(state)
            time.sleep(0.01)

    def stop():
        stopped.set()
        feeder.join(2)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        yield stop
    finally:
        stop()


def run(module, angle_deg, body_q=lambda t: np.zeros(29), motion="left_arm_forward"):
    """Call the skill while a feeder plays body telemetry (t = seconds since the call)."""
    with feeding(module, body_q):
        return module.move_upper_body(motion, angle_deg)


def follows(t, delta=-np.radians(20), joint=15):  # The moving joint at the plan's depth mid-run.
    q = np.zeros(29)
    q[joint] = delta if 0.5 < t < 0.9 else 0.0
    return q


def test_pure_reference_parts():
    poses = arm_poses(np.zeros(29), 15, -0.3)
    move_rows = math.ceil(max(2.0, 1.875 * 0.3 / PEAK_SPEED) * RATE) + 1
    assert poses.shape == (150 + 2 * move_rows, 29) and np.allclose(np.delete(poses, 15, 1), 0)
    assert poses[0, 15] == poses[-1, 15] == 0 and np.isclose(poses[:, 15].min(), -0.3)
    assert np.abs(np.diff(min_jerk(100))).max() * 50 < 1.875 / (99 / 50) + 1e-3  # min-jerk peak
    schedule = token_schedule(np.zeros(64), np.ones((10, 64)))
    assert len(schedule) == 210 and np.allclose(schedule[[0, -1], 0], [0.01, 0.0])
    margins = group_margins(0.15, 0.2, 0.3)
    assert margins[3] == margins[13] == 0.15 and margins[16] == 0.2 and margins[26] == 0.3


def test_swing_streams_one_chunk_and_hands_back(skill):
    module, peer = skill
    result = run(module, 20.0, follows)
    assert "returned" in result.message, result
    assert result.metadata["reached_deg"] == pytest.approx(20.0, abs=0.1)
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "finished" and status["handback_confirmed"]
    assert status["last_chunk_sequence"] == 0  # One open-loop chunk, as commissioned.
    packets = peer.snapshot()
    assert all(
        p[-3:-1] == b"\0\0" for p in packets if p.startswith(b"command")
    )  # Never start/stop.
    streamed = tokens(packets)[:, 0]
    assert streamed.min() == pytest.approx(PEER_TOKEN - FAKE_GAIN * np.radians(20), abs=0.01)
    assert np.abs(np.diff(streamed)).max() <= 0.05 + 1e-6
    assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)  # Blended back before handback.


def test_wrist_inside_its_envelope_passes(skill):
    def wrist(t):
        q = follows(t)
        q[26] = 0.25
        return q

    result = run(skill[0], 20.0, wrist)
    assert "returned" in result.message, result
    assert result.metadata["max_deviation_rad"]["wrists_shoulder_yaw"] == 0.25


def test_leg_outside_its_envelope_returns_smoothly_then_hands_back(skill):
    module, peer = skill

    def knee(t):
        q = follows(t)
        q[3] = 0.2 if t > 0.4 else 0.0
        return q

    result = run(module, 20.0, knee)
    assert "stopped early" in result.message and "left_knee" in result.message, result
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "finished" and status["handback_confirmed"], status
    # The stream itself returned to the planner token: no mid-pose snap by the native planner.
    streamed = tokens(peer.snapshot())[:, 0]
    assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)
    assert np.abs(np.diff(streamed)).max() <= 0.05 + 1e-6


def test_overspeed_during_the_way_back_hands_back_at_once(skill):
    module, _ = skill
    stopped = threading.Event()

    def feed():
        start = time.monotonic()
        while not stopped.is_set():
            state = module._execution.get_feedback()
            if state is not None:
                t = time.monotonic() - start
                q = np.zeros(29)
                q[3] = 0.2 if t > 0.4 else 0.0  # leaves the envelope: the way back starts
                dq = np.zeros(29)
                dq[5] = 5.0 if t > 0.55 else 0.0  # then a joint runs away
                module.handle_feedback(replace(state, body_q=tuple(q), body_dq=tuple(dq)))
            time.sleep(0.01)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        result = module.move_upper_body("left_arm_forward", 20.0)
    finally:
        stopped.set()
        feeder.join(2)
    assert "stopped early" in result.message, result
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "cancelled" and status["handback_confirmed"], status


def test_retreat_resumes_at_the_matching_point_of_the_way_back(skill, mocker):
    module, _ = skill
    submit = mocker.spy(module._execution, "submit_chunk")

    def knee(t):
        q = follows(t)
        q[3] = 0.2 if t > 0.4 else 0.0
        return q

    result = run(module, 20.0, knee)
    assert "stopped early" in result.message, result
    first, retreat = submit.call_args_list[0].args[3], submit.call_args_list[1].args
    assert retreat[1] == 1  # the way back is its own chunk
    # The retreat is the tail of the plan's own way home, token for token, blend included.
    rows = retreat[3]
    assert np.array_equal(rows, first[len(first) - len(rows) :])
    kplan = sonic_arm_skill.keyframe_plan(np.zeros(29), [Keyframe(np.full(29, 0.5), 1.0)])
    pad = (len(unheld(first)) - len(kplan.poses)) // 2  # The blends, not the finish hold.
    assert len(first) - len(rows) >= pad + kplan.segments[-1].start


def test_one_keyframe_moves_encode_the_plan_once(skill, mocker):
    module, _ = skill
    spy = mocker.spy(module._arm._encoder, "encode")
    run(module, 20.0, follows)
    assert spy.call_count == 1  # the retreat reuses the plan's own way back


def test_long_hold_splits_inside_the_hold_and_passes_the_age_check(skill, mocker):
    module, _ = skill
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    mocker.patch.object(sonic_arm_skill, "_MAX_CHUNK_ROWS", 120)
    submit = mocker.spy(module._execution, "submit_chunk")
    target = sonic_arm_skill.POSES["left_arm_side_horizontal"]
    result = call_with_feed(
        module,
        lambda: module.move_left_arm_pose("left_arm_side_horizontal", hold_seconds=4.0),
        reaching(target),
    )
    assert "returned" in result.message, result
    assert result.metadata["chunks"] >= 2
    pad = 10  # the 0.2 s blends of the fixture
    hold = range(pad + 5 + 16, pad + 5 + 16 + 200)  # lead, outward move, then the 4 s hold
    chunks = [call.args[3] for call in submit.call_args_list]
    assert max(len(c) for c in chunks) <= 125  # inside the 3.0 s observation age less 0.5 s
    start = 0
    for rows in chunks[:-1]:
        start += len(rows) - 25  # chunk overlap
        assert start in hold


@pytest.mark.parametrize("hold", [0.5, 21.0, float("nan")])
def test_hold_seconds_outside_1_to_20_sends_nothing(skill, hold):
    module, peer = skill
    with pytest.raises(ValueError, match="hold_seconds"):
        module.move_left_arm_pose("left_arm_side_horizontal", hold_seconds=hold)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_profile_without_room_for_the_retreat_sends_nothing(skill, mocker):
    module, peer = skill
    status = module._execution.get_status()
    short = status | {"profile": status["profile"] | {"max_duration": 2.0}}
    mocker.patch.object(module._execution, "get_status", return_value=short)
    with pytest.raises(RuntimeError, match="must cover the plan"):
        module.move_upper_body("left_arm_forward", 20.0)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_arm_that_does_not_follow_is_reported(skill):
    result = run(skill[0], 20.0)
    assert "did not follow the reference" in result.message, result


@pytest.mark.parametrize("angle", [0.0, 29.0, float("nan")])
def test_invalid_angle_sends_nothing(skill, angle):
    module, peer = skill
    before = len(peer.snapshot())
    with pytest.raises(ValueError, match="angle_deg"):
        module.move_upper_body("left_arm_forward", angle)
    assert not any(p.startswith(b"pose") for p in peer.snapshot()[before:])
    assert module._execution.get_status()["task"] is None


def test_movement_during_encoding_sends_nothing(skill, mocker):
    module, _ = skill
    state = module._execution.get_feedback()
    moved = replace(state, body_q=(0.1,) * 29)
    mocker.patch.object(module._execution, "get_feedback", side_effect=[state, moved])
    with pytest.raises(RuntimeError, match="moved"):
        module.move_upper_body("left_arm_forward", 20.0)
    assert module._execution.get_status()["task"] is None


def test_not_ready_execution_sends_nothing(skill, mocker):
    module, _ = skill
    status = module._execution.get_status()
    mocker.patch.object(module._execution, "get_status", return_value=status | {"ready": False})
    with pytest.raises(RuntimeError, match="planner idle|observation age"):
        module.move_upper_body("left_arm_forward", 20.0)


def test_session_failure_is_not_reported_as_success(skill, mocker):
    module, peer = skill
    submit = mocker.spy(module._execution, "submit_chunk")

    def drop_feedback():
        wait_until(lambda: submit.call_count == 1)  # The motion is streaming.
        peer.pause_feedback.set()  # The execution session fails on stale feedback...
        wait_until(lambda: module._arm._task()["phase"] == SessionPhase.HANDBACK)
        peer.pause_feedback.clear()  # ...and confirms the planner once feedback returns.

    dropper = threading.Thread(target=drop_feedback, daemon=True)
    dropper.start()
    with pytest.raises(RuntimeError, match="control returned to the planner"):
        run(module, 20.0, follows)
    dropper.join(2)
    assert not dropper.is_alive()
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "failed" and status["handback_confirmed"]


def test_one_chunk_streams_the_plan_without_speed_ripple():
    """The skill's single 11 s chunk through the session's default 0.3 s blend."""
    packets: list[bytes] = []
    profile = SonicModelProfile(
        name="arm",
        source_fps=50,
        right_hand_order="native",
        handback_policy="native_planner",
        max_observation_age=12.0,
    )
    session = SonicTaskSession("task", profile, send=packets.append, journal=lambda event: None)
    seed = feedback(10.0)
    poses = arm_poses(np.zeros(29), 15, -0.5)
    move_rows = math.ceil(max(2.0, 1.875 * 0.5 / PEAK_SPEED) * RATE) + 1
    assert len(poses) == 150 + 2 * move_rows
    motion = np.full((len(poses), 64), 0.125)
    motion[:, 0] += poses[:, 15]
    tokens = token_schedule(np.asarray(seed.token_state), motion)
    plan = np.hstack(
        [tokens, np.tile(np.r_[seed.left_hand_q, seed.right_hand_q], (len(tokens), 1))]
    )
    session.start(seed, now=10.0)
    now, index, plan_start, emitted = 10.0, 100, None, []
    while session.status.phase != SessionPhase.HANDBACK:
        now, index = round(now + 0.02, 2), index + 1
        planner = session.status.phase == SessionPhase.SEEDING
        session.observe(
            feedback(
                now,
                index,
                planner=planner,
                epoch=7 if planner else 8,
                accepted=session.status.last_frame_index,
            ),
            now=now,
        )
        if session.status.phase == SessionPhase.WAITING:
            plan_start = now
            session.submit_chunk(task_id="task", sequence=0, observed_at=now, actions=plan, now=now)
        elif plan_start is not None and (now - plan_start) * 50 >= len(plan) - 2:
            session.finish(now=now)
        before = len(packets)
        session.tick(now=now)
        if plan_start is not None and len(packets) > before:
            emitted.append((round((now - plan_start) * 50), pose_values(packets[-1])[0]))
    assert session.status.outcome == "finished"
    k, value = np.array(emitted).T
    assert np.abs(value - plan[k.astype(int), 0]).max() < 0.005
    assert np.abs(np.diff(value)).max() <= np.abs(np.diff(plan[:, 0])).max() + 1e-4  # No ripple.
    assert value.min() == pytest.approx(0.125 - 0.5, abs=0.005)


def test_observation_age_must_cover_the_longest_retreat(skill, mocker):
    module, peer = skill
    # Tiny chunks: only the retreat (21 rows + the 10-row blend = 0.62 s) is longer than 0.5 s.
    mocker.patch.object(
        sonic_arm_skill, "chunk_rows", lambda n, still: [(i, min(i + 5, n)) for i in range(0, n, 5)]
    )
    status = module._execution.get_status()
    short = status | {"profile": status["profile"] | {"max_observation_age": 1.0}}
    mocker.patch.object(module._execution, "get_status", return_value=short)
    with pytest.raises(RuntimeError, match="must cover the plan"):
        module.move_upper_body("left_arm_forward", 20.0)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_profile_too_short_for_the_plan_sends_nothing(skill, mocker):
    module, peer = skill
    status = module._execution.get_status()
    short = status | {"profile": status["profile"] | {"max_observation_age": 0.5}}
    mocker.patch.object(module._execution, "get_status", return_value=short)
    with pytest.raises(RuntimeError, match="planner idle|observation age"):
        module.move_upper_body("left_arm_forward", 20.0)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_failed_session_at_finish_is_not_reported_as_success(skill, mocker):
    module, _ = skill
    execution = module._execution
    fail = execution.fail_task
    mocker.patch.object(
        execution, "finish_task", side_effect=lambda task_id: fail(task_id, "injected")
    )
    with pytest.raises(RuntimeError, match="injected"):
        run(module, 20.0, follows)


def test_motion_skill_returns_at_once_and_reports_through_the_tool_stream(
    real_launch_skill, mocker
):
    module, _ = real_launch_skill
    updates = mocker.spy(module, "tool_update")
    with feeding(module, follows):
        text = module.move_upper_body("left_arm_forward", 20.0)
        assert "started" in text and updates.call_count == 0
        wait_until(lambda: updates.call_count == 1)
    assert updates.call_args.args[0] == "move_upper_body"
    assert json.loads(updates.call_args.args[1])["message"].startswith("left_arm_forward reached")
    wait_until(lambda: module._running is None)
    assert module.stop_arm() == "No arm motion is running."


def test_stop_arm_with_nothing_running_says_so(skill):
    assert skill[0].stop_arm() == "No arm motion is running."


def test_stop_mid_motion_retreats_smoothly_and_hands_back(real_launch_skill, mocker):
    module, _ = real_launch_skill
    submit = mocker.spy(module._execution, "submit_chunk")
    updates = mocker.spy(module, "tool_update")
    with feeding(module, follows):
        module.move_upper_body("left_arm_forward", 20.0)
        wait_until(lambda: submit.call_count == 1)  # The stream is armed.
        assert module.stop_arm() == (
            "Stopping: the arm returns smoothly to its start pose; a tool update reports the result."
        )
        wait_until(lambda: updates.call_count == 1)
    message = json.loads(updates.call_args.args[1])["message"]
    assert "stopped early: stop requested" in message, message
    assert submit.call_args_list[1].args[1] == 1  # The way back, as its own chunk.
    status = module._execution.get_status()["task"]
    assert status["handback_confirmed"], status


def test_stop_during_encoding_sends_nothing(real_launch_skill, mocker):
    module, peer = real_launch_skill
    updates = mocker.spy(module, "tool_update")
    encode = module._arm._encoder.encode

    def stop_then_encode(poses):
        assert module.stop_arm().startswith("Stopping")
        return encode(poses)

    mocker.patch.object(module._arm._encoder, "encode", side_effect=stop_then_encode)
    module.move_upper_body("left_arm_forward", 20.0)
    wait_until(lambda: updates.call_count == 1)
    assert updates.call_args.args[1] == "move_upper_body failed: stopped on request; nothing sent"
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    assert module._execution.get_status()["task"] is None


def stop_at_row(module, mocker, row, on_row=lambda: None):
    """Records the stream's stop row against sonic_arm_skill's own (real) clock.

    The first time the stream reads its clock at `row` token rows or more into the plan, it calls
    `on_row` on the stream thread and sets the returned event. Production computes the retreat row
    and the stream gets it unchanged. Each stop is recorded as (the row the stream passed to
    retreat(), the row its clock implied there: int((now - plan_start) * RATE), the retreat built).
    """
    last = threading.local()  # Each thread's latest reading of the module's clock.
    stops = []
    reached = threading.Event()

    def monotonic():
        last.now = time.monotonic()
        start = getattr(last, "watch", None) and last.watch.plan_start  # The stream thread's.
        if start is not None and not reached.is_set() and (last.now - start) * RATE >= row:
            on_row()  # Seen by this very loop pass, which retreats from this reading.
            reached.set()
        return last.now

    mocker.patch.object(
        sonic_arm_skill, "time", SimpleNamespace(monotonic=monotonic, sleep=time.sleep)
    )
    real_stream = module._arm._stream

    def stream(task_id, plan, chunks, retreat, profile, watch, group=None):
        last.watch = watch  # Runs on the stream thread: its clock readings watch for the row.

        def recorded(token_row):
            # Nothing reads the clock between the stream's stop reading and this call.
            implied = int((last.now - watch.plan_start) * RATE)
            stops.append((token_row, implied, retreat(token_row)))
            return stops[-1][2]

        return real_stream(task_id, plan, chunks, recorded, profile, watch, group)

    mocker.patch.object(module._arm, "_stream", side_effect=stream)
    return reached, stops


def test_module_stop_mid_motion_waits_for_the_retreat_and_the_final_update(
    real_launch_skill, mocker
):
    module, _ = real_launch_skill
    submit = mocker.spy(module._execution, "submit_chunk")
    updates = mocker.spy(module, "tool_update")
    reached, stops = stop_at_row(module, mocker, LEAD_IN_ROWS + 5)  # Mid-motion, past the blend.
    teardown = Module.stop
    with feeding(module, follows) as stop_feeding:

        def quiet_teardown(self):  # A feeder call into the stopping loop would wait forever.
            stop_feeding()
            teardown(self)

        mocker.patch.object(Module, "stop", quiet_teardown)
        module.move_upper_body("left_arm_forward", 20.0)
        assert reached.wait(WAIT_S)
        module.stop()
        # stop() returned only after the retreat and the motion's own final update.
        assert module._arm.stop.is_set()
        assert [c.args[1] for c in submit.call_args_list] == [0, 1]
        assert updates.call_count == 1
    assert "stopped early: stop requested" in json.loads(updates.call_args.args[1])["message"]
    [(row, implied, built)] = stops
    # The row the stream's clock implies at the stop, not one a chunk or the blend pad off.
    assert row == implied >= LEAD_IN_ROWS + 5, stops
    assert np.array_equal(unheld(submit.call_args_list[1].args[3]), built)
    assert module._execution.get_status()["task"]["handback_confirmed"]


def test_motion_racing_module_stop_is_refused_and_nothing_outlives_stop(real_launch_skill, mocker):
    module, _ = real_launch_skill
    gate, start_task = gate_start(module, mocker)
    stopper, release = stop_in_background(module, mocker)
    try:
        with pytest.raises(RuntimeError, match="the arm skill is shutting down"):
            module.move_upper_body("left_arm_forward", 20.0)
        assert module.stop_arm() == "No arm motion is running."
    finally:
        release.set()
        stopper.join(10)
    alive = module._motion is not None and module._motion.is_alive()
    assert not stopper.is_alive() and not alive
    assert all(arm.stop.is_set() for arm in module._launched)
    assert module._running is None and not module._tools
    gate.set()
    assert start_task.call_count == 0


def test_same_skill_retry_during_module_stop_keeps_the_hold_until_the_retreat_ends(
    real_launch_skill, mcp, tool_transport, mocker
):
    module, _ = real_launch_skill
    call, registry = mcp
    gate, start_task = gate_start(module, mocker)
    assert "started" in call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)[0]
    wait_until(lambda: start_task.call_count == 1)  # Parked at the gate, past the stop checks.
    stopper = threading.Thread(target=module.stop)
    stopper.start()
    try:
        wait_until(module._arm.stop.is_set)  # stop() has begun and waits for the motion.
        assert module.stop_arm().startswith("Stopping")
        text, error = call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)
        assert "already running" in text and not error  # Admits nothing; rebinds the hold.
        assert registry.snapshot() == {"movement": "move_upper_body"}
        assert not frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)
    finally:
        gate.set()
        stopper.join(10)
    assert not stopper.is_alive() and not module._motion.is_alive() and module._arm.stop.is_set()
    assert registry.snapshot() == {} and not module._tools
    assert len(frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)) == 1
    text, error = call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)
    assert error and "the arm skill is shutting down" in text, text
    assert registry.snapshot() == {} and not module._tools


def test_motion_called_during_module_stop_is_refused_and_nothing_outlives_stop(
    real_launch_skill, mocker
):
    module, _ = real_launch_skill
    in_teardown, finish_teardown = threading.Event(), threading.Event()
    teardown = Module.stop

    def gated_teardown(self):  # The base teardown, held open until the call below is answered.
        in_teardown.set()
        assert finish_teardown.wait(10)
        teardown(self)

    mocker.patch.object(Module, "stop", gated_teardown)
    updates = mocker.spy(module, "tool_update")
    stopper = threading.Thread(target=module.stop)
    stopper.start()
    try:
        assert in_teardown.wait(10)
        with pytest.raises(RuntimeError, match="shutting down; nothing started"):
            module.move_upper_body("left_arm_forward", 20.0)
    finally:
        finish_teardown.set()
        stopper.join(10)
    assert not stopper.is_alive()
    assert module._motion is None and module._running is None and not module._tools
    assert updates.call_count == 0


def test_stop_in_the_lead_in_blend_plays_the_blend_backwards(skill, mocker):
    module, _ = skill
    spy = mocker.spy(module._execution, "submit_chunk")
    # Stopped a few rows into the fixture's lead-in blend, seen by that same loop pass.
    _, stops = stop_at_row(module, mocker, 3, on_row=module._arm.stop.set)
    result = run(module, 20.0, follows)
    assert "stopped early: stop requested" in result.message, result
    [(row, implied, _)] = stops
    # The row the stream's clock implies at the stop, not one a chunk or the blend pad off.
    assert row == implied and 3 <= row < LEAD_IN_ROWS, stops
    assert len(spy.call_args_list) == 2
    first, rows = unheld(spy.call_args_list[0].args[3]), unheld(spy.call_args_list[1].args[3])
    assert len(rows) == row + 2  # The blend rows sent so far, reversed, then the planner token.
    assert np.array_equal(rows[:-1], first[row::-1])
    assert np.array_equal(rows[-1], first[-1])  # The captured planner token (and hands).
    assert rows[-1][0] == pytest.approx(PEER_TOKEN)
    assert module._execution.get_status()["task"]["handback_confirmed"]


def test_stop_during_arming_sends_no_chunk(real_launch_skill, mocker):
    module, peer = real_launch_skill
    submit = mocker.spy(module._execution, "submit_chunk")
    updates = mocker.spy(module, "tool_update")
    real = module._execution.start_task

    def start_then_stop():
        task = real()
        assert module.stop_arm().startswith("Stopping")  # While the stream arms.
        return task

    mocker.patch.object(module._execution, "start_task", side_effect=start_then_stop)
    with feeding(module, follows):
        module.move_upper_body("left_arm_forward", 20.0)
        wait_until(lambda: updates.call_count == 1)
    assert updates.call_args.args[1] == "move_upper_body failed: stopped on request; nothing sent"
    assert submit.call_count == 0
    # Only the session's own arming packets, holding the planner token: no motion was sent.
    assert np.allclose(tokens(peer.snapshot())[:, 0], PEER_TOKEN)
    assert module._execution.get_status()["task"]["handback_confirmed"]


def test_thread_start_failure_leaves_nothing_running(real_launch_skill, mocker):
    module, _ = real_launch_skill
    real_start = threading.Thread.start

    def start(thread):
        if thread.name.startswith("arm-"):
            raise RuntimeError("can't start new thread")
        real_start(thread)

    mocker.patch.object(threading.Thread, "start", start)
    with pytest.raises(RuntimeError, match="can't start new thread"):
        module.move_upper_body("left_arm_forward", 20.0)
    assert module._running is None and not module._tools
    assert module.stop_arm() == "No arm motion is running."


def test_second_stop_during_the_retreat_changes_nothing(real_launch_skill, mocker):
    module, _ = real_launch_skill
    submit = mocker.spy(module._execution, "submit_chunk")
    updates = mocker.spy(module, "tool_update")
    with feeding(module, follows):
        module.move_upper_body("left_arm_forward", 20.0)
        wait_until(lambda: submit.call_count == 1)
        module.stop_arm()
        wait_until(lambda: submit.call_count == 2)  # The retreat chunk.
        module.stop_arm()
        wait_until(lambda: updates.call_count == 1)
    assert [c.args[1] for c in submit.call_args_list] == [0, 1]
    assert "stopped early: stop requested" in json.loads(updates.call_args.args[1])["message"]


def test_second_motion_while_one_runs_is_refused_and_the_first_completes(real_launch_skill, mocker):
    module, _ = real_launch_skill
    updates = mocker.spy(module, "tool_update")
    with feeding(module, follows):
        assert "started" in module.move_upper_body("left_arm_forward", 20.0)
        with pytest.raises(RuntimeError, match="an arm move is already running"):
            module.move_left_arm_pose("left_arm_side_horizontal")
        wait_until(lambda: updates.call_count == 1)
    assert json.loads(updates.call_args.args[1])["message"].startswith("left_arm_forward reached")


def test_same_skill_retry_through_mcp_keeps_the_running_stream_and_capability(
    real_launch_skill, mcp, tool_transport, mocker
):
    module, _ = real_launch_skill
    call, registry = mcp
    gate, _ = gate_start(module, mocker)
    text, error = call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)
    assert "started" in text and not error
    text, error = call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)
    assert not error and text == (
        "move_upper_body is already running; its result will follow as a tool update. "
        "Call stop_arm to stop it."
    )
    assert registry.snapshot() == {"movement": "move_upper_body"}
    assert "move_upper_body" in module._tools
    assert not frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)
    text, _ = call("move_left_arm_pose", pose="left_arm_side_horizontal")
    assert "held by 'move_upper_body'" in text
    assert "move_upper_body" in module._tools
    gate.set()
    wait_until(lambda: registry.snapshot() == {})
    assert len(frames(tool_transport, NOTIFICATIONS_MESSAGE_METHOD)) == 1  # The final update.
    assert len(frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)) == 1
    assert not module._tools


def test_invalid_same_skill_retry_through_mcp_keeps_the_capability(
    real_launch_skill, mcp, tool_transport, mocker
):
    module, _ = real_launch_skill
    call, registry = mcp
    gate, _ = gate_start(module, mocker)
    assert "started" in call("move_upper_body", motion="left_arm_forward", angle_deg=20.0)[0]
    text, error = call("move_upper_body", motion="left_arm_forward", angle_deg=0.0)
    assert "already running" in text and not error  # Not even validated.
    assert registry.snapshot() == {"movement": "move_upper_body"}
    gate.set()
    wait_until(lambda: not module._tools)
    assert registry.snapshot() == {}
    assert len(frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)) == 1


def test_call_arriving_during_completion_gets_a_fresh_stream(real_launch_skill, mocker):
    module, _ = real_launch_skill
    real_update = module.tool_update
    release = threading.Event()

    def blocking_first_update(name, message):
        if updates.call_count == 1:
            release.wait(WAIT_S)
        real_update(name, message)

    updates = mocker.patch.object(module, "tool_update", side_effect=blocking_first_update)
    start_tool = mocker.spy(module, "start_tool")
    stop_tool = mocker.spy(module, "stop_tool")
    module.move_upper_body("left_arm_forward", 20.0)
    first = module._tools["move_upper_body"]
    wait_until(lambda: updates.call_count == 1)  # The first run's cleanup holds the lifecycle.
    second: list[str] = []
    caller = threading.Thread(
        target=lambda: second.append(module.move_upper_body("left_arm_forward", 20.0))
    )
    caller.start()
    time.sleep(0.2)
    assert not second  # Waits for the cleanup instead of rebinding a closing stream.
    release.set()
    caller.join(5)
    assert second and "started" in second[0], second
    assert start_tool.call_count == 2 and stop_tool.call_count == 1
    assert module._tools["move_upper_body"] is not first
    wait_until(lambda: stop_tool.call_count == 2)
    assert updates.call_count == 2 and not module._tools


def test_concurrent_starts_start_exactly_one_motion(real_launch_skill, mocker):
    module, _ = real_launch_skill
    gate, start_task = gate_start(module, mocker)
    stop_tool = mocker.spy(module, "stop_tool")
    barrier = threading.Barrier(8)
    results: list[str] = []

    def start():
        barrier.wait()
        results.append(module.move_upper_body("left_arm_forward", 20.0))

    threads = [threading.Thread(target=start) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(5)
    assert sum("started" in r for r in results) == 1, results
    assert sum("already running" in r for r in results) == 7, results
    gate.set()
    wait_until(lambda: stop_tool.call_count == 1)
    assert start_task.call_count == 1


def test_base_exception_in_the_motion_still_closes_the_stream(real_launch_skill, mocker):
    class Interrupted(BaseException):
        pass

    module, _ = real_launch_skill
    mocker.patch.object(module, "_run", side_effect=Interrupted())
    excepthook = mocker.patch.object(threading, "excepthook")  # The worker re-raises it.
    updates = mocker.spy(module, "tool_update")
    stop_tool = mocker.spy(module, "stop_tool")
    assert "started" in module.move_upper_body("left_arm_forward", 20.0)
    wait_until(lambda: excepthook.call_count == 1)
    assert excepthook.call_args.args[0].exc_type is Interrupted
    assert updates.call_args.args[1] == "move_upper_body failed: Interrupted"
    assert stop_tool.call_count == 1 and not module._tools and module._running is None
    assert module._arm._watch is None


def test_agent_prompt_names_only_skills_the_blueprint_has():
    named = set(re.findall(r"`(\w+)\(", G1_SONIC_ARM_SYSTEM_PROMPT))
    skills = {
        k
        for container in (SonicArmSkillContainer, ObserveSkill)
        for k, v in vars(container).items()
        if getattr(v, "__skill__", False)
    }
    assert named and named <= skills, named - skills


@pytest.mark.parametrize(
    ("motion", "joint", "sign"), [("left_arm_sideways", 16, 1), ("left_forearm_raise", 18, -1)]
)
def test_other_motions_move_their_own_joint_with_its_widened_envelope(skill, motion, joint, sign):
    module, peer = skill
    # 0.33 rad is past the 0.20 arm margin alone: only the moving joint's widened envelope allows it.
    result = run(module, 22.0, partial(follows, delta=sign * 0.33, joint=joint), motion=motion)
    assert "returned" in result.message, result
    assert (
        result.metadata["max_deviation_rad"]["arms"] == 0.0
    )  # The moving joint is not a deviation.
    assert result.metadata["motion"] == motion and result.metadata["reached_deg"] == pytest.approx(
        18.9, abs=0.1
    )
    streamed = tokens(peer.snapshot())[:, 0]
    assert streamed.max() - streamed.min() == pytest.approx(FAKE_GAIN * np.radians(22), abs=0.01)


def test_unknown_motion_sends_nothing(skill):
    module, peer = skill
    with pytest.raises(ValueError, match="motion must be one of"):
        module.move_upper_body("waist_turn_left", 10.0)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_motion_enum_matches_the_motion_table():
    assert set(sonic_arm_skill.UpperBodyMotion.__args__) == set(sonic_arm_skill.MOTIONS)


@pytest.mark.parametrize(
    ("motion", "angle"),
    [
        ("left_arm_forward", 29.0),
        ("left_arm_sideways", 9.0),
        ("left_arm_sideways", 41.0),
        ("left_forearm_raise", 19.0),
        ("left_forearm_raise", 61.0),
    ],
)
def test_each_motion_keeps_its_own_angle_range(skill, motion, angle):
    module, peer = skill
    with pytest.raises(ValueError, match="angle_deg"):
        module.move_upper_body(motion, angle)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_big_strokes_slow_down_to_the_commissioned_peak_speed():
    commissioned = arm_poses(np.zeros(29), 15, -0.5)  # 0.5 rad over 2 s: the speed cap's reference
    move_rows = math.ceil(max(2.0, 1.875 * 0.5 / PEAK_SPEED) * RATE) + 1
    assert len(commissioned) == 150 + 2 * move_rows
    assert np.abs(np.diff(commissioned, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    big = arm_poses(np.zeros(29), 18, -np.radians(60))
    speed = np.abs(np.diff(big[:, 18])).max() * RATE
    assert speed <= PEAK_SPEED + 1e-9
    move_rows = math.ceil(max(2.0, 1.875 * np.radians(60) / PEAK_SPEED) * RATE) + 1
    assert len(big) == 150 + 2 * move_rows


@pytest.mark.parametrize(("joint", "passes"), [(24, True), (17, True), (22, False), (25, False)])
def test_shoulder_yaw_shares_the_loose_envelope(skill, joint, passes):
    """Shoulder yaws (17, 24) get the 0.30 rad wrist envelope; shoulder pitch/roll and elbows keep 0.20."""
    module, _ = skill

    def drift(t):
        q = follows(t)
        q[joint] = 0.25 if t > 0.4 else 0.0
        return q

    result = run(module, 20.0, drift)
    if passes:
        assert "returned" in result.message, result
    else:
        assert "stopped early" in result.message, result
        assert sonic_arm_skill._JOINTS[joint] in result.message, result


class FakeChecker:
    def __init__(self, contact=None):
        self.contact, self.calls = contact, 0

    def first_contact(self, poses, every=5, start=None):
        self.calls += 1
        return self.contact


def reaching(target, fraction=0.8, extra=None):
    """Telemetry that moves the left arm to `fraction` of an absolute target mid-run."""

    def body_q(t):
        q = np.zeros(29)
        if 0.5 < t < 0.9:
            q[15:19] = fraction * np.asarray(target)
        for joint, value in (extra or {}).items():
            q[joint] = value if t > 0.4 else 0.0
        return q

    return body_q


def call_with_feed(module, call, body_q):
    with feeding(module, body_q):
        return call()


def test_full_range_pose_moves_every_left_arm_joint_together(skill):
    module, peer = skill
    target = sonic_arm_skill.POSES["left_hand_over_head"]
    result = call_with_feed(
        module, lambda: module.move_left_arm_pose("left_hand_over_head"), reaching(target)
    )
    assert "returned" in result.message, result
    assert result.metadata["reached_fraction"] == pytest.approx(0.8, abs=0.01)
    assert result.metadata["planned_deg"]["left_shoulder_pitch"] == pytest.approx(
        np.degrees(-2.8), abs=0.1
    )
    streamed = tokens(peer.snapshot())[:, 0]  # FakeEncoder: summed offsets of the moved joints
    assert streamed.min() - PEER_TOKEN == pytest.approx(FAKE_GAIN * sum(target), abs=0.02)


def test_pose_envelope_lets_the_arm_and_other_arm_coordinate(skill):
    module, _ = skill
    target = sonic_arm_skill.POSES["left_arm_side_horizontal"]
    # SONIC adds shoulder pitch (0.35 off plan) and moves the right arm (0.30): both allowed.
    result = call_with_feed(
        module,
        lambda: module.move_left_arm_pose("left_arm_side_horizontal"),
        reaching(target, extra={15: -0.35, 23: 0.30}),
    )
    assert "returned" in result.message, result


def test_pose_envelope_still_guards_the_legs(skill):
    module, _ = skill
    target = sonic_arm_skill.POSES["left_arm_side_horizontal"]
    result = call_with_feed(
        module,
        lambda: module.move_left_arm_pose("left_arm_side_horizontal"),
        reaching(target, extra={3: 0.2}),
    )
    assert "stopped early" in result.message and "left_knee" in result.message, result


def test_joint_targets_need_the_path_check(real_launch_skill, mocker):
    module, peer = real_launch_skill
    start_tool = mocker.spy(module, "start_tool")
    with pytest.raises(RuntimeError, match="collision_model_path is not configured"):
        module.move_left_arm_joints(-90.0, 11.0, 0.0, 70.0)
    assert start_tool.call_count == 0 and module._running is None  # Refused before starting.
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


@pytest.mark.parametrize(
    "angles", [(-170.0, 11.0, 0.0, 70.0), (-90.0, -20.0, 0.0, 70.0), (-90.0, 11.0, 0.0, 90.0)]
)
def test_joint_targets_outside_the_box_send_nothing(skill, angles):
    module, peer = skill
    module._arm._checker = FakeChecker()
    with pytest.raises(ValueError, match="must be within"):
        module.move_left_arm_joints(*angles)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_unsafe_joint_target_path_is_refused_before_sending(skill):
    module, peer = skill
    module._arm._checker = FakeChecker("torso_link")
    with pytest.raises(RuntimeError, match="torso_link; nothing sent"):
        module.move_left_arm_joints(11.0, -10.0, 0.0, 34.0)
    assert module._arm._checker.calls == 1
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_checked_joint_target_runs(skill):
    module, _ = skill
    module._arm._checker = FakeChecker()
    target = np.radians([-90.0, 11.0, 0.0, 70.0])
    result = call_with_feed(
        module, lambda: module.move_left_arm_joints(-90.0, 11.0, 0.0, 70.0), reaching(target)
    )
    assert "returned" in result.message, result
    assert module._arm._checker.calls == 1


@needs_scene
def test_real_collision_model_clears_the_poses_and_catches_the_torso():
    checker = sonic_arm_reference.ArmCollisionChecker(SCENE)
    legs_waist = [-0.312, 0, 0, 0.669, -0.363, 0, -0.312, 0, 0, 0.669, -0.363, 0, 0, 0, 0]
    rest = np.array([*legs_waist, 0.2, 0.2, 0, 0.6, 0, 0, 0, 0.2, -0.2, 0, 0.6, 0, 0, 0])
    for pose, target in sonic_arm_skill.POSES.items():
        delta = np.zeros(29)
        delta[15:19] = np.asarray(target) - rest[15:19]
        assert checker.first_contact(stroke_plan(rest, delta)) is None, pose
    inward = np.zeros(29)
    inward[16] = -0.7  # shoulder roll into the torso
    assert checker.first_contact(stroke_plan(rest, inward)) == "torso_link"


@needs_scene
def test_robot_idle_pose_near_the_hip_does_not_block_moves_away():
    """The G1's idle left wrist rests about 2 cm from the hip (pc2 log 2026-10-19 16:22)."""
    checker = sonic_arm_reference.ArmCollisionChecker(SCENE)
    idle = ROBOT_IDLE
    assert checker.first_contact(idle[None]) is None  # Already near at the start: allowed.
    for pose, target in sonic_arm_skill.POSES.items():
        delta = np.zeros(29)
        delta[15:19] = np.asarray(target) - idle[15:19]
        assert checker.first_contact(stroke_plan(idle, delta)) is None, pose
    inward = np.zeros(29)
    inward[16] = -0.25  # Closing in on the hip is still refused.
    assert checker.first_contact(stroke_plan(idle, inward)) == "left_hip_roll_link"


def raised_start(module, mocker, pitch=-1.0):
    """The execution module's feedback with the left shoulder pitch already raised."""
    real = module._execution.get_feedback

    def get_feedback():
        state = real()
        q = list(state.body_q)
        q[15] = pitch
        return replace(state, body_q=tuple(q))

    mocker.patch.object(module._execution, "get_feedback", get_feedback)
    q = np.zeros(29)
    q[15] = pitch
    return q


def test_gesture_from_a_raised_arm_returns_to_where_it_started(skill, mocker):
    module, peer = skill
    raised = raised_start(module, mocker)
    encode = mocker.spy(module._arm._encoder, "encode")
    result = call_with_feed(module, lambda: module.perform_gesture("beckon"), lambda t: raised)
    assert result.metadata["motion"] == "beckon" and result.metadata["keyframes"] == 5
    poses = encode.call_args_list[0].args[0]  # the plan; later calls are retreat units
    assert poses[0, 15] == poses[-1, 15] == -1.0  # The measured start, not the nominal rest.
    assert np.array_equal(poses[0], poses[-1]) and np.array_equal(poses[0], raised)
    assert np.isclose(poses[:, 15].min(), -1.2)  # beckon's own pitch, an absolute target
    streamed = tokens(peer.snapshot())[:, 0]
    assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)


def test_gesture_line_plays_on_the_robot_at_the_first_chunk_and_stop_cuts_it(skill, mocker):
    module, peer = skill
    raised = raised_start(module, mocker)
    mocker.patch.object(module.config, "voice_host", "pc2")
    shell = mocker.patch.object(sonic_arm_skill, "subprocess")
    shell.Popen.return_value.poll.return_value = 0
    with pytest.raises(ValueError, match="voice_command"):
        module.perform_gesture("beckon", line="01_intro")
    mocker.patch.object(module.config, "voice_command", "python3 ~/g1_audio.py")
    with pytest.raises(ValueError, match="wav name"):
        module.perform_gesture("beckon", line="../x; reboot")
    assert not shell.Popen.called and not peer.snapshot()
    mocker.patch.object(module.config, "voice_lead_s", 0.0)
    submit, launched = mocker.spy(module._execution, "submit_chunk"), []
    shell.Popen.side_effect = lambda *a, **k: launched.append(time.monotonic()) or mocker.DEFAULT
    result = call_with_feed(
        module, lambda: module.perform_gesture("beckon", line="01_intro"), lambda t: raised
    )
    # Launched as the arm starts moving: after fast_keyframe_plan's 0.1 s lead-in and the 0.2 s test
    # blend, not at chunk 0.
    assert launched[0] - submit.call_args_list[0].args[2] == pytest.approx(0.3, abs=0.05)
    shell.Popen.side_effect = None
    (argv,), _ = shell.Popen.call_args
    assert argv[-2:] == ["pc2", "python3 ~/g1_audio.py play ~/lines/01_intro.wav 60"]
    assert result.metadata["voice"] == "01_intro: played"
    assert argv[:3] == ["ssh", "-n", "-o"] and argv[-3] == "--"
    shell.Popen.return_value.poll.return_value = None  # still playing
    module._arm.hush()
    shell.Popen.return_value.kill.assert_called_once()
    cut = shell.Popen.call_args.args[0][-1]
    assert cut == "pkill -f 'g1_audio\\.py play'; python3 ~/g1_audio.py stop"
    assert not re.search(r"g1_audio\.py play", cut)  # the cut's own shell never matches
    assert re.search(r"g1_audio\.py play", argv[-1])  # the player does
    shell.Popen.side_effect = OSError("fork failed")  # a failed cut is logged, never raised
    module._arm.hush()
    shell.Popen.side_effect, calls = None, shell.Popen.call_count
    module._arm._voice_line = "01_intro"  # a safety abort before the first chunk: nothing is spoken
    module._arm._speak(
        sonic_arm_skill._Watch(np.zeros(29), np.zeros(29), 3.0, np.zeros(29), abort="x")
    )
    assert shell.Popen.call_count == calls and module._arm._voice_line is None


def test_a_late_finish_lands_in_the_final_hold_and_still_finishes(skill, mocker):
    """finish_task comes 2 rows (40 ms) before the plan ends; a stall past that is not a failure."""
    module, peer = skill
    real = module._execution.finish_task

    def late(task_id):
        time.sleep(0.15)  # A stalled stream thread: a few ticks.
        return real(task_id)

    mocker.patch.object(module._execution, "finish_task", side_effect=late)
    submit = mocker.spy(module._execution, "submit_chunk")
    result = run(module, 20.0, follows)
    assert "returned" in result.message, result
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "finished" and status["handback_confirmed"], status
    # The frames after the plan's end repeat its final row: still tokens and hands.
    final = unheld(submit.call_args_list[-1].args[3])[-1]
    held = [pose_values(p) for p in peer.snapshot() if p.startswith(b"pose")][-2:]
    for frame in held:
        assert np.allclose(frame, final, atol=1e-6)  # float32 on the wire


def test_stop_mid_wave_reverses_one_swing_then_goes_home(skill, mocker):
    module, _ = skill
    submit = mocker.spy(module._execution, "submit_chunk")

    def knee(t):
        q = np.zeros(29)
        q[3] = 0.2 if t > 1.5 else 0.0  # During the fourth swing of the 3 s fast plan.
        return q

    result = call_with_feed(module, lambda: module.perform_gesture("wave"), knee)
    assert "stopped early" in result.message and "left_knee" in result.message, result
    assert [c.args[1] for c in submit.call_args_list] == [0, 1]  # The retreat is its own chunk.
    first, retreat = submit.call_args_list[0].args[3], submit.call_args_list[1].args[3]
    assert len(retreat) < len(first) / 2  # One swing and the way home, never the gesture again.
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "finished" and status["handback_confirmed"], status


def test_a_session_failure_is_not_hidden_behind_stopped_early(skill, mocker):
    module, _ = skill
    fail = module._execution.fail_task
    mocker.patch.object(
        module._execution, "finish_task", side_effect=lambda task_id: fail(task_id, "injected")
    )

    def knee(t):
        q = np.zeros(29)
        q[3] = 0.2 if t > 1.5 else 0.0
        return q

    result = call_with_feed(module, lambda: module.perform_gesture("wave"), knee)
    assert "stopped early" in result.message, result
    assert "session failed: injected" in result.message, result


def test_gesture_longer_than_the_limit_sends_nothing(skill, mocker):
    module, peer = skill
    mocker.patch.object(sonic_arm_skill, "GESTURE_MAX_S", 1.0)
    with pytest.raises(ValueError, match=r"wave would take \d+ s, more than 1 s; nothing sent"):
        module.perform_gesture("wave")
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    assert module._execution.get_status()["task"] is None


def test_unknown_gesture_sends_nothing(skill):
    module, peer = skill
    with pytest.raises(ValueError, match="name must be one of"):
        module.perform_gesture("dance")
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_only_the_waist_gestures_widen_the_legs():
    for name, spec in GESTURES.items():
        assert (spec.leg_margin is None) == (name not in ("invite", "thanks", "bow")), name


def test_wide_leg_margin_skips_hip_yaw():
    margins = gesture_margins("two_arm", 0.15, 0.45, 0.35, leg_margin=0.23)
    assert margins[2] == margins[8] == 0.15
    assert all(margins[j] == 0.23 for j in WIDE_LEG_JOINTS)


def test_invite_widens_hip_yaw_too():
    margins = {
        name: gesture_margins(
            "two_arm", 0.15, 0.45, 0.35, GESTURES[name].leg_margin, GESTURES[name].leg_joints
        )
        for name in ("invite", "thanks")
    }
    invite, thanks = margins["invite"], margins["thanks"]
    assert all(invite[j] == 0.23 for j in range(12))
    assert thanks[2] == thanks[8] == 0.15 and thanks[0] == 0.23


def test_leg_margin_ceiling_is_enforced(mocker):
    mocker.patch.dict(sonic_arm_skill.GESTURE_LEG_MARGINS, {"invite": (0.26, ALL_LEG_JOINTS)})
    with pytest.raises(ValueError, match="over"):
        sonic_arm_skill._gesture_table()
    mocker.patch.object(sonic_arm_skill, "GESTURE_LEG_MARGINS", {"nope": (0.2, ALL_LEG_JOINTS)})
    with pytest.raises(ValueError, match="not a gesture"):
        sonic_arm_skill._gesture_table()
    mocker.patch.object(sonic_arm_skill, "GESTURE_LEG_MARGINS", {"bow": (0.2, (0, 15))})
    with pytest.raises(ValueError, match="outside the legs"):
        sonic_arm_skill._gesture_table()


def test_waist_envelope_includes_the_planned_waist(skill, mocker):
    module, _ = skill
    module._arm._both_checker = FakeChecker()
    invite = GESTURES["invite"]

    def with_waist(base, hands):
        keyframes = invite.keyframes_of(base, hands)
        target = keyframes[0].target.copy()
        target[12] += 0.40  # waist_yaw
        return [replace(keyframes[0], target=target), *keyframes[1:]]

    mocker.patch.dict(GESTURES, {"invite": replace(invite, keyframes_of=with_waist)})
    watch = mocker.spy(sonic_arm_skill, "_Watch")
    call_with_feed(module, lambda: module.perform_gesture("invite"), lambda t: np.zeros(29))
    allowed = watch.call_args.kwargs["allowed"]
    assert allowed[12] >= 0.40 + module.config.body_margin - 1e-9
    assert all(allowed[j] >= 0.23 for j in range(12))


def test_gesture_checks_the_path_when_configured_and_runs_without(skill):
    module, _ = skill
    module._arm._checker = FakeChecker("torso_link")
    with pytest.raises(RuntimeError, match="torso_link; nothing sent"):
        module.perform_gesture("stop_sign")
    module._arm._checker = None
    result = call_with_feed(
        module,
        lambda: module.perform_gesture("stop_sign"),
        reaching(sonic_arm_skill.LEFT_ARM_GESTURES["stop_sign"][0][0]),
    )
    assert "returned" in result.message, result


def test_report_extra_is_merged_into_the_metadata(skill):
    module, _ = skill
    allowed = pose_margins(0.15, 0.45, 0.35)
    seen = []

    def extra(base, hold_q):
        seen.append(base.copy())
        return {"direction": "left"}

    result = call_with_feed(
        module,
        lambda: module._run(
            "probe",
            lambda base, hands: [Keyframe(base + 0.3 * np.eye(29)[15], 0.1)],
            allowed,
            require_check=False,
            report_extra=extra,
        ),
        follows,
    )
    assert result.metadata["direction"] == "left" and result.metadata["motion"] == "probe"
    assert len(seen) == 1 and seen[0].shape == (29,)


def _stamped(t):
    """Telemetry that carries its own feeder time (in 0.1 s units) in the moving elbow joint."""
    q = np.zeros(29)
    q[15] = 0.1 * t
    return q


def _hold_run(module, mocker, body_q, report_extra):
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    submit = mocker.spy(module._execution, "submit_chunk")
    t0 = time.monotonic()
    result = call_with_feed(
        module,
        lambda: module._run(
            "probe",
            lambda base, hands: [Keyframe(base + 0.3 * np.eye(29)[15], 2.0)],
            pose_margins(0.15, 0.45, 0.35),
            require_check=False,
            report_extra=report_extra,
        ),
        body_q,
    )
    return result, t0, submit.call_args_list[0].args[2]


def test_hold_samples_come_only_from_the_hold_window(skill, mocker):
    module, _ = skill
    seen = []
    result, t0, plan_start = _hold_run(
        module, mocker, _stamped, lambda base, hold_q: seen.append(hold_q) or {}
    )
    first, last = module._arm._watch.hold_rows
    times = [t0 + 10 * q[15] for q in seen[0]]
    assert result.metadata["hold_samples"] == len(times) > 0
    assert plan_start + first / 50 - 0.03 <= min(times)
    assert max(times) <= plan_start + last / 50 + 0.03


def test_no_hold_samples_after_a_stop(skill, mocker):
    module, _ = skill
    seen = []

    def knee(t):
        q = _stamped(t)
        q[3] = 0.2 if t > 0.2 else 0.0
        return q

    result, _, _ = _hold_run(module, mocker, knee, lambda base, hold_q: seen.append(hold_q) or {})
    assert "stopped early" in result.message, result
    assert result.metadata["hold_samples"] == 0 and seen == [None]


def test_samples_stop_at_a_mid_hold_stop(real_launch_skill, mocker):
    module, _ = real_launch_skill
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    updates = mocker.spy(module, "tool_update")
    t0 = []

    def stamped(t):
        t0 or t0.append(time.monotonic() - t)  # feeder start, in wall time
        return _stamped(t)

    with feeding(module, stamped):
        module.move_left_arm_pose("left_arm_side_horizontal", hold_seconds=2.0)
        wait_until(lambda: module._arm._watch is not None)
        watch = module._arm._watch  # The motion clears it when done.
        wait_until(lambda: len(watch.hold_q) >= 5)
        stopped = time.monotonic()
        module.stop_arm()
        wait_until(lambda: updates.call_count == 1)
    arrivals = [t0[0] + 10 * q[15] for q in watch.hold_q]
    assert len(arrivals) >= 5 and max(arrivals) <= stopped


def test_long_plans_split_into_two_chunks_at_the_hold():
    still = np.zeros(800, bool)
    still[400] = True
    assert sonic_arm_skill.chunk_rows(800, still) == [(0, 800)]  # As commissioned: one chunk.
    still = np.zeros(1545, bool)
    still[705] = True
    chunks = sonic_arm_skill.chunk_rows(1545, still)
    assert chunks == [(0, 730), (705, 1545)]
    assert all(end - start <= 1000 for start, end in chunks)
    still = np.zeros(2500, bool)
    still[200] = True
    with pytest.raises(ValueError):
        sonic_arm_skill.chunk_rows(2500, still)


def test_a_last_chunk_that_filled_the_old_boundary_still_gets_a_full_hold():
    still = np.zeros(1500, bool)
    still[[510, 1000]] = True  # Old boundary: split at 510, leaving a 990-row last chunk.
    chunks = sonic_arm_skill.chunk_rows(1500, still)
    assert chunks == [(0, 535), (510, 1025), (1000, 1500)]
    rows = np.zeros((chunks[-1][1] - chunks[-1][0], 3))
    rows[-1] = 1.0
    held = sonic_arm_skill.with_finish_hold(rows)
    assert len(held) == len(rows) + sonic_arm_skill._FINISH_HOLD_ROWS == 525
    assert np.all(held[len(rows) - 1 :] == 1.0)
    full = sonic_arm_skill._MAX_CHUNK_ROWS - sonic_arm_skill._FINISH_HOLD_ROWS
    assert len(sonic_arm_skill.with_finish_hold(np.zeros((full, 3)))) == 1000
    with pytest.raises(RuntimeError, match="finish hold"):
        sonic_arm_skill.with_finish_hold(np.zeros((full + 1, 3)))
    assert all(end - start + 25 <= 1000 for start, end in chunks[-1:])


def test_split_overhead_plan_streams_without_a_ripple_at_the_second_chunk():
    """The slowest pose (overhead at 0.75 rad/s, 22 s) through the real session on a simulated clock."""
    packets: list[bytes] = []
    profile = SonicModelProfile(
        name="arm",
        source_fps=50,
        right_hand_order="native",
        handback_policy="native_planner",
        max_observation_age=35.0,
        max_duration=45.0,
    )
    session = SonicTaskSession("task", profile, send=packets.append, journal=lambda event: None)
    seed = feedback(10.0)
    rest = np.zeros(29)
    delta = np.zeros(29)
    delta[15:19] = np.asarray(sonic_arm_skill.POSES["left_hand_over_head"]) - [0.2, 0.2, 0.0, 0.6]
    poses = stroke_plan(rest, delta)
    motion = np.full((len(poses), 64), 0.125)
    motion[:, 0] += FAKE_GAIN * (poses - poses[0]).sum(axis=1)
    tokens = token_schedule(np.asarray(seed.token_state), motion)
    plan = np.hstack(
        [tokens, np.tile(np.r_[seed.left_hand_q, seed.right_hand_q], (len(tokens), 1))]
    )
    hold_start = 100 + int(np.argmax(np.abs(poses - rest).sum(axis=1)))
    move_rows = math.ceil(max(2.0, 1.875 * np.abs(delta).max() / PEAK_SPEED) * RATE) + 1
    assert len(plan) == 350 + 2 * move_rows
    assert hold_start == 100 + 50 + move_rows - 1
    still = np.zeros(len(plan), bool)
    still[hold_start] = True
    chunks = sonic_arm_skill.chunk_rows(len(plan), still)
    assert len(plan) > 1000 and len(chunks) == 2
    session.start(seed, now=10.0)
    now, index, plan_start, emitted, sent = 10.0, 100, None, [], 0
    while session.status.phase != SessionPhase.HANDBACK:
        now, index = round(now + 0.02, 2), index + 1
        planner = session.status.phase == SessionPhase.SEEDING
        session.observe(
            feedback(
                now,
                index,
                planner=planner,
                epoch=7 if planner else 8,
                accepted=session.status.last_frame_index,
            ),
            now=now,
        )
        if session.status.phase in (SessionPhase.WAITING, SessionPhase.RUNNING):
            plan_start = plan_start or now
            k = round((now - plan_start) * 50)
            if sent < len(chunks) and k >= chunks[sent][0]:
                start, end = chunks[sent]
                session.submit_chunk(
                    task_id="task",
                    sequence=sent,
                    observed_at=plan_start + start / 50,
                    actions=plan[start:end],
                    now=now,
                )
                sent += 1
            elif k >= len(plan) - 2:
                session.finish(now=now)
        before = len(packets)
        session.tick(now=now)
        if plan_start is not None and len(packets) > before:
            emitted.append((round((now - plan_start) * 50), pose_values(packets[-1])[0]))
    assert session.status.outcome == "finished" and session.status.last_chunk_sequence == 1
    k, value = np.array(emitted).T
    assert np.abs(value - plan[k.astype(int), 0]).max() < 0.005
    assert np.abs(np.diff(value)).max() <= np.abs(np.diff(plan[:, 0])).max() + 1e-4  # No ripple.


class FakeKinematics:
    """solve and solve_forearm return a fixed reachable pose, or raise what the real ones raise.

    forearm_error reads 100 * the left wrist pitch (motor 20) of the pose, so telemetry sets it.
    """

    POSE = (-1.2, 0.5, 0.0, 1.35)
    AIM = np.array([1.2, 0.1, -0.9])

    def __init__(self, reachable=True):
        self.reachable, self.bases, self.pixels, self.errors = reachable, [], [], []

    def aim_point(self, q, u, v):
        self.bases.append(np.array(q))
        self.pixels.append((u, v))
        return self.AIM.copy(), 1.68

    def solve_forearm(self, q, point, path_clear):
        assert np.array_equal(point, self.AIM)
        if not self.reachable:
            raise ValueError(
                "the left forearm cannot aim at (1.20, 0.10, -0.90) m: no natural pose"
            )
        pose = np.array(q, np.float64)
        pose[15:19] = self.POSE
        assert path_clear(pose) is None
        return np.array(self.POSE)

    def forearm_points(self, q):
        return np.zeros(3), np.array([0.12345, 0.2, -0.1])

    def forearm_error(self, q, point):
        assert np.array_equal(point, self.AIM)
        self.errors.append(100 * float(q[20]))
        return self.errors[-1]

    def solve(self, q, azimuth_deg, elevation_deg):
        self.bases.append(np.array(q))
        if not self.reachable:
            raise ValueError(f"({azimuth_deg:g}, {elevation_deg:g}) is more than 3 deg off")
        return np.array(self.POSE)

    def direction(self, q):
        assert np.allclose(q[15:19], self.POSE)
        return (31.04, -19.96)


@pytest.mark.parametrize("az,el", [(-40.0, 0.0), (0.0, 80.0), (120.0, 0.0), (math.nan, 0.0)])
def test_point_at_outside_the_cone_sends_nothing(skill, az, el):
    module, peer = skill
    with pytest.raises(ValueError, match="cone"):
        module.point_at(az, el)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_point_at_needs_the_arm_model(real_launch_skill, mocker):
    module, peer = real_launch_skill
    start_tool = mocker.spy(module, "start_tool")
    with pytest.raises(RuntimeError, match="collision_model_path is not configured"):
        module.point_at(30.0, -20.0)
    assert start_tool.call_count == 0 and module._running is None
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


@pytest.mark.parametrize("hold", [0.5, 21.0])
def test_point_at_hold_outside_1_to_20_sends_nothing(skill, hold):
    module, peer = skill
    module._arm._checker, module._kinematics = FakeChecker(), FakeKinematics()
    with pytest.raises(ValueError, match="hold_seconds"):
        module.point_at(30.0, -20.0, hold)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_unreachable_direction_sends_nothing(skill):
    module, peer = skill
    module._arm._checker, module._kinematics = FakeChecker(), FakeKinematics(reachable=False)
    with pytest.raises(ValueError, match="more than 3 deg off"):
        module.point_at(30.0, -20.0)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    assert module._execution.get_status()["task"] is None


def test_point_at_solves_from_the_measured_base_and_reports_the_direction(skill, mocker):
    module, _ = skill
    raised = raised_start(module, mocker, pitch=-0.3)
    module._arm._checker, module._kinematics = FakeChecker(), FakeKinematics()
    result = call_with_feed(
        module, lambda: module.point_at(30.0, -20.0), reaching(FakeKinematics.POSE)
    )
    assert "returned" in result.message, result
    assert np.array_equal(module._kinematics.bases[0], raised)  # The real start, not a rest pose.
    assert module._arm._checker.calls == 1
    assert result.metadata["requested_direction_deg"] == [30.0, -20.0]
    assert result.metadata["achieved_direction_deg"] == [31.0, -20.0]
    assert result.metadata["planned_deg"]["left_elbow"] == pytest.approx(
        math.degrees(1.35), abs=0.1
    )


@needs_scene
def test_point_at_with_the_real_arm_model(skill):
    module, _ = skill
    module._arm._checker = FakeChecker()
    module._kinematics = sonic_arm_reference.ArmKinematics(SCENE)
    target = module._kinematics.solve(np.zeros(29), 45.0, -30.0)
    result = call_with_feed(module, lambda: module.point_at(45.0, -30.0), reaching(target))
    assert "returned" in result.message, result
    az, el = result.metadata["achieved_direction_deg"]
    assert abs(az - 45.0) < 3.0 and abs(el + 30.0) < 3.0


@needs_scene
def test_point_at_near_the_body_is_refused_by_the_path_check(skill, mocker):
    module, peer = skill
    real = module._execution.get_feedback
    mocker.patch.object(
        module._execution,
        "get_feedback",
        lambda: replace(real(), body_q=tuple(float(v) for v in ROBOT_IDLE)),
    )
    module._arm._checker = sonic_arm_reference.ArmCollisionChecker(SCENE)
    module._kinematics = sonic_arm_reference.ArmKinematics(SCENE)
    with pytest.raises(RuntimeError, match="comes within 2 cm of the torso_link; nothing sent"):
        module.point_at(0.0, -60.0)  # low and straight ahead: the upper arm meets the torso
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_prompt_states_the_point_at_bound():
    (az_low, az_high), (el_low, el_high) = (
        sonic_arm_reference.POINT_AZIMUTH_DEG,
        sonic_arm_reference.POINT_ELEVATION_DEG,
    )
    assert f"azimuth {az_low:g} to {az_high:g}" in G1_SONIC_ARM_SYSTEM_PROMPT
    assert f"elevation {el_low:g} to {el_high:g}" in G1_SONIC_ARM_SYSTEM_PROMPT
    assert "may be refused" in G1_SONIC_ARM_SYSTEM_PROMPT


def seeing(module, mocker, payload, kinematics=None):
    """The skill with a fake vision model, arm model and head-camera frame."""
    mocker.patch.object(module.config, "vl_model", "fake-vl")
    module._vl = FakeVl(payload)
    module._arm._checker, module._kinematics = FakeChecker(), kinematics or FakeKinematics()
    mocker.patch.object(module.color_image, "get_next", return_value=IMAGE_640x480)


def test_point_at_object_with_no_match_says_so_and_sends_nothing(skill, mocker):
    module, peer = skill
    seeing(module, mocker, [])
    with pytest.raises(RuntimeError, match="found no 'red box' in the camera image; nothing sent"):
        module.point_at_object("red box")
    assert "red box" in module._vl.prompts[0]
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_point_at_object_vision_failure_is_not_reported_as_not_found(skill, mocker):
    module, peer = skill
    seeing(module, mocker, ConnectionError("gateway down"))
    with pytest.raises(
        RuntimeError, match="vision model fake-vl failed: gateway down; nothing sent"
    ):
        module.point_at_object("red box")
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_point_at_object_reply_that_is_not_a_list_is_a_vision_failure(skill, mocker):
    module, peer = skill
    seeing(module, mocker, {"boxes": [["box", 1, 1, 9, 9]]})
    with pytest.raises(
        RuntimeError,
        match="vision model fake-vl failed: vision model reply is not a list of boxes; nothing sent",
    ):
        module.point_at_object("red box")
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    with pytest.raises(ValueError, match="not a list of boxes"):
        query_boxes(FakeVl("[]"), IMAGE_640x480, "box")  # A string, not a parsed list.
    assert query_boxes(FakeVl([]), IMAGE_640x480, "box") == []


def test_point_at_object_without_a_camera_frame_sends_nothing(skill, mocker):
    module, peer = skill
    seeing(module, mocker, [["box", 1, 1, 2, 2]])
    mocker.patch.object(module.color_image, "get_next", side_effect=Exception("timeout"))
    with pytest.raises(RuntimeError, match="no camera frame.*; nothing sent"):
        module.point_at_object("red box")
    assert module._vl.prompts == []
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_normalized_boxes_convert_before_bounds_checks():
    vl = FakeVl([["box", 700, 600, 900, 900]])
    ((label, x1, y1, x2, y2),) = query_boxes(vl, IMAGE_640x480, "box", "normalized_1000")
    assert label == "box" and (x1, y1, x2, y2) == pytest.approx((448, 288, 576, 432))
    # Read as pixels it lies off the image: zero area after clipping.
    assert query_boxes(vl, IMAGE_640x480, "box", "pixels") == []


def test_malformed_boxes_are_dropped_and_partial_ones_clipped():
    vl = FakeVl(
        [["a", 1, 2, 3], "junk", ["b", "x", 0, 9, 9], ["c", 9, 9, 1, 1], ["d", -5, 10, 700, 20]]
    )
    assert query_boxes(vl, IMAGE_640x480, "box") == [("d", 0.0, 10.0, 640.0, 20.0)]


def test_point_at_object_aims_the_forearm_at_the_largest_box(skill, mocker):
    module, _ = skill
    raised = raised_start(module, mocker, pitch=-0.3)
    seeing(module, mocker, [["small", 300, 200, 320, 220], ["big box", 100.4, 300, 260, 420]])
    result = call_with_feed(
        module, lambda: module.point_at_object("box"), reaching(FakeKinematics.POSE)
    )
    assert "returned" in result.message, result
    kin = module._kinematics
    assert kin.pixels == [(pytest.approx(180.2), 360.0)]
    assert np.array_equal(kin.bases[0], raised)  # The measured start, not a rest pose.
    assert module._arm._checker.calls == 2  # The solver's path check, then the plan's own.
    meta = result.metadata
    assert meta["mode"] == "forearm" and meta["label"] == "big box"
    assert meta["box_px"] == [100, 300, 260, 420]
    assert meta["aim_point_m"] == [1.2, 0.1, -0.9] and meta["aim_distance_m"] == 1.68
    assert meta["pose_deg"] == [-68.8, 28.6, 0.0, 77.3]
    assert meta["planned_aim_error_deg"] == 0.0
    assert meta["planned_wrist_x_m"] == 0.123
    assert meta["measured_aim_error_deg"] is None  # The fast plan's hold is too short to sample.
    assert "direction_deg" not in meta and "requested_direction_deg" not in meta


def test_point_at_object_refusal_from_the_solver_sends_nothing(skill, mocker):
    module, peer = skill
    seeing(module, mocker, [["box", 300, 200, 340, 240]], FakeKinematics(reachable=False))
    with pytest.raises(ValueError, match="no natural pose"):
        module.point_at_object("box")
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    assert module._execution.get_status()["task"] is None


def test_point_at_object_reports_measured_error_from_hold_samples(skill, mocker):
    module, _ = skill
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    seeing(module, mocker, [["box", 300, 200, 340, 240]])

    def body_q(t):  # Every fourth 10 ms tick an outlier: 20 deg; otherwise 1 deg.
        q = np.zeros(29)
        q[15:19] = 0.8 * np.asarray(FakeKinematics.POSE)
        q[20] = 0.2 if int(t * 100) % 4 == 0 else 0.01
        return q

    result = call_with_feed(module, lambda: module.point_at_object("box", 2.0), body_q)
    meta = result.metadata
    planned, *samples = module._kinematics.errors  # The planned pose's error comes first.
    assert planned == 0.0
    assert meta["hold_samples"] == len(samples) > 10
    assert 20.0 in samples and 1.0 in samples
    assert meta["planned_aim_error_deg"] == 0.0
    assert meta["measured_aim_error_deg"] == 1.0  # The median, not the mean.


def at_idle(module, mocker):
    """The measured pose is the robot's idle, with the real arm model and path check."""
    real = module._execution.get_feedback
    mocker.patch.object(
        module._execution,
        "get_feedback",
        lambda: replace(real(), body_q=tuple(float(v) for v in ROBOT_IDLE)),
    )
    module._arm._checker = sonic_arm_reference.ArmCollisionChecker(SCENE)
    module._kinematics = sonic_arm_reference.ArmKinematics(SCENE)


def idle_reaching(module, u, v):
    """Telemetry at the robot's idle with the left arm 80% of the way to the solved pose."""
    kin, checker = module._kinematics, module._arm._checker
    point, _ = kin.aim_point(ROBOT_IDLE, u, v)
    q4 = kin.solve_forearm(
        ROBOT_IDLE,
        point,
        lambda pose: checker.first_contact(
            stroke_plan(ROBOT_IDLE, pose - ROBOT_IDLE), start=ROBOT_IDLE
        ),
    )
    q = ROBOT_IDLE.copy()
    q[15:19] += 0.8 * (q4 - ROBOT_IDLE[15:19])
    return lambda t: q


@needs_scene
def test_point_at_object_aims_the_forearm_at_the_floor_point(skill, mocker):
    module, peer = skill
    seeing(module, mocker, [["box", 300, 220, 340, 260]])
    at_idle(module, mocker)
    result = call_with_feed(
        module, lambda: module.point_at_object("box"), idle_reaching(module, 320, 240)
    )
    assert "returned" in result.message, result
    meta = result.metadata
    print(meta)
    assert meta["mode"] == "forearm"
    assert meta["planned_aim_error_deg"] <= 3.0
    assert meta["aim_distance_m"] == pytest.approx(1.68, abs=0.05)
    assert len(meta["aim_point_m"]) == 3 and len(meta["pose_deg"]) == 4
    assert any(p.startswith(b"pose") for p in peer.snapshot())


@needs_scene
def test_point_at_object_right_edge_is_pointable(skill, mocker):
    module, _ = skill
    seeing(module, mocker, [["box", 600, 220, 640, 260]])  # Outside the old straight-arm cone.
    at_idle(module, mocker)
    result = call_with_feed(
        module, lambda: module.point_at_object("box"), idle_reaching(module, 620, 240)
    )
    assert "returned" in result.message, result
    assert result.metadata["planned_aim_error_deg"] <= 3.0


def test_point_at_object_without_a_model_is_disabled(skill):
    module, _ = skill
    with pytest.raises(RuntimeError, match="disabled: no vl_model configured"):
        module.point_at_object("red box")


@pytest.mark.parametrize(
    ("description", "hold", "error"),
    [("", 2.0, "description"), ("box", 0.5, "hold_seconds")],
)
def test_point_at_object_rejects_bad_arguments_before_looking(
    skill, mocker, description, hold, error
):
    module, _ = skill
    seeing(module, mocker, [])
    with pytest.raises(ValueError, match=error):
        module.point_at_object(description, hold)
    assert module._vl.prompts == []


def test_agentic_blueprint_global_config_is_valid():
    validate_global_values(unitree_g1_sonic_arm_agentic.global_config_overrides)


def one_joint_prepare(module, mocker=None):
    allowed = group_margins(0.15, 0.2, 0.3)
    delta = np.zeros(29)
    delta[15] = -math.radians(20)
    return module._arm.prepare(
        "p", lambda base, hands: [Keyframe(base + delta, 1.0)], allowed, require_check=False
    )


def test_prepare_sends_nothing_and_execute_runs_it(skill):
    module, peer = skill
    with feeding(module, follows):
        prepared = one_joint_prepare(module)
        assert not any(p.startswith(b"pose") for p in peer.snapshot())
        result = module._arm.execute(prepared)
    assert "returned" in result.message, result
    assert module._execution.get_status()["task"]["handback_confirmed"]


def test_prepare_refuses_a_chunk_the_session_would_reject(skill, mocker):
    module, peer = skill
    mocker.patch.object(sonic_test_support, "FAKE_GAIN", 50.0)
    with feeding(module, follows), pytest.raises(ValueError, match="token bound exceeded"):
        one_joint_prepare(module)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_move_stop_move_again_with_the_real_launcher(real_launch_skill, mocker):
    module, _ = real_launch_skill
    submit = mocker.spy(module._execution, "submit_chunk")
    updates = mocker.spy(module, "tool_update")
    with feeding(module, follows):
        module.move_upper_body("left_arm_forward", 20.0)
        wait_until(lambda: submit.call_count == 1)
        module.stop_arm()
        wait_until(lambda: updates.call_count == 1)
        assert "stop requested" in updates.call_args_list[0].args[1]
        wait_until(lambda: module._running is None)
        module.move_upper_body("left_arm_forward", 20.0)
        wait_until(lambda: updates.call_count == 2)
    second = updates.call_args_list[1].args[1]
    assert "returned" in second and "stop requested" not in second, second


def test_execute_rechecks_the_pose(skill, mocker):
    module, peer = skill
    with feeding(module, follows):
        prepared = one_joint_prepare(module)
        state = module._execution.get_feedback()
        q = list(state.body_q)
        q[3] += 0.1
        mocker.patch.object(
            module._execution, "get_feedback", return_value=replace(state, body_q=tuple(q))
        )
        with pytest.raises(RuntimeError, match="the body moved"):
            module._arm.execute(prepared)
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


# Imported gestures, per-row Dex3 hands and the operator-only finger-order check.

PEER_HANDS = np.r_[[0.1] * 7, [0.2] * 7]  # ScriptedPeer's left_hand_q, then right_hand_q
# Command order (thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1), left then right:
# LEFT index open, middle closed, thumb half; RIGHT middle open, index closed, thumb rotated.
FINGER_POSE = np.array(
    [0.0, 0.0, 0.875, -1.57, -1.75, 0.0, 0.0, 0.6, 0.0, -1.75, 0.0, 0.0, 1.57, 1.75]
)
ENCODER = Path(os.environ.get("GEAR_SONIC_ENCODER", "/nonexistent"))


def hands_of(chunk):
    return np.asarray(chunk)[:, 64:]


def padded(rows, pad=LEAD_IN_ROWS):
    """Pose-row hands [N,14] -> token rows: the token schedule's blends hold the end rows."""
    return np.vstack([np.repeat(rows[:1], pad, 0), rows, np.repeat(rows[-1:], pad, 0)])


def test_gesture_names_match_the_gesture_table():
    names = set(sonic_arm_skill.GestureName.__args__)
    assert names == set(GESTURES)
    assert len(names) == len(sonic_arm_skill.LEFT_ARM_GESTURES) + len(RETAINED) + len(AUTHORED)
    assert set(RETAINED + AUTHORED) <= names
    for name in names:
        assert f"`{name}`" in G1_SONIC_ARM_SYSTEM_PROMPT, name
    _, move_s, return_min_s, arms, envelope = sonic_arm_skill.gesture_plan_inputs(
        "wave", np.zeros(29), PEER_HANDS
    )
    assert (move_s, return_min_s, arms, envelope) == (2.0, 0.0, ("left",), "pose")
    _, move_s, return_min_s, arms, envelope = sonic_arm_skill.gesture_plan_inputs(
        "two_hand_hearts", np.zeros(29), PEER_HANDS
    )
    assert (move_s, return_min_s, arms, envelope) == (0.5, 5.0, ("left", "right"), "two_arm")


def test_every_gesture_fits_the_time_limit_from_the_robot_idle():
    for name in GESTURES:
        keyframes, move_s, return_min_s, _, _ = sonic_arm_skill.gesture_plan_inputs(
            name, ROBOT_IDLE, np.zeros(14)
        )
        plan = keyframe_plan(
            ROBOT_IDLE, keyframes, base_hands=np.zeros(14), move_s=move_s, return_min_s=return_min_s
        )
        assert len(plan.poses) / RATE <= sonic_arm_skill.GESTURE_MAX_S == 45.0, name


@pytest.mark.parametrize("name", sorted(GESTURES))
def test_every_gesture_from_the_robot_idle_splits_into_session_chunks(name):
    """_move's plan, real 2 s token blends and split mask: every chunk fits a session chunk, and
    each split row has still tokens, poses and hands. beckon (802 pose rows, no hold) once had no
    still row and did not fit one chunk with both blends (1002 rows)."""
    pad = (len(token_schedule(np.zeros(64), np.zeros((1, 64)))) - 1) // 2
    assert pad == 100
    keyframes, move_s, return_min_s, _, _ = sonic_arm_skill.gesture_plan_inputs(
        name, ROBOT_IDLE, np.zeros(14)
    )
    kplan = keyframe_plan(
        ROBOT_IDLE,
        keyframes,
        base_hands=np.zeros(14),
        move_s=move_s,
        return_min_s=return_min_s,
        split_every=sonic_arm_skill._MAX_CHUNK_ROWS - sonic_arm_skill._CHUNK_OVERLAP - pad,
        one_chunk=sonic_arm_skill._MAX_CHUNK_ROWS - sonic_arm_skill._FINISH_HOLD_ROWS - 2 * pad,
    )
    tokens = token_schedule(np.full(64, PEER_TOKEN), FakeEncoder("unused").encode(kplan.poses))
    token_still = sonic_arm_skill.token_still(tokens)
    plan_still = np.r_[np.zeros(pad, bool), kplan.still, np.zeros(pad, bool)]
    chunks = sonic_arm_skill.chunk_rows(len(tokens), token_still & plan_still)
    assert all(end - start <= sonic_arm_skill._MAX_CHUNK_ROWS for start, end in chunks), chunks
    assert chunks[-1][1] - chunks[-1][0] <= sonic_arm_skill._MAX_CHUNK_ROWS - 25  # Room to hold.
    assert chunks[-1][1] == len(tokens)
    hands = np.vstack(
        [np.repeat(kplan.hands[:1], pad, 0), kplan.hands, np.repeat(kplan.hands[-1:], pad, 0)]
    )
    for start, _ in chunks[1:]:
        assert token_still[start] and plan_still[start], (name, start)
        assert np.ptp(hands[start : start + 15], axis=0).max() == 0.0, (name, start)
    if name == "beckon":
        assert len(chunks) == 2  # 864 pose rows with the split hold: too long for one chunk.


def two_hand_hearts_plan():
    """The plan _move builds for two_hand_hearts from the peer's measured pose (fast engine)."""
    base = np.zeros(29)
    keyframes, move_s, return_min_s, _, _ = sonic_arm_skill.gesture_plan_inputs(
        "two_hand_hearts", base, PEER_HANDS
    )
    return sonic_arm_skill.keyframe_plan(
        base, keyframes, base_hands=PEER_HANDS, move_s=move_s, return_min_s=return_min_s
    )


def test_two_hand_hearts_submits_the_plan_hands_row_for_row(skill, mocker):
    module, _ = skill
    module._arm._both_checker = FakeChecker()
    submit = mocker.spy(module._execution, "submit_chunk")
    result = call_with_feed(
        module, lambda: module.perform_gesture("two_hand_hearts"), lambda t: np.zeros(29)
    )
    assert result.metadata["motion"] == "two_hand_hearts" and module._arm._both_checker.calls >= 1
    kplan = two_hand_hearts_plan()
    assert np.ptp(kplan.hands, axis=0).max() > 0.1  # The fingers move: not the measured hands.
    [first] = [unheld(c.args[3]) for c in submit.call_args_list]
    assert len(first) == len(kplan.poses) + 2 * LEAD_IN_ROWS
    rows = np.linspace(0, len(kplan.poses) - 1, 25).astype(int)
    assert np.array_equal(hands_of(first)[LEAD_IN_ROWS + rows], kplan.hands[rows])
    assert np.array_equal(hands_of(first), padded(kplan.hands))


def test_hand_steps_stay_limited_through_a_stop_at_peak_finger_velocity(skill, mocker):
    module, peer = skill
    module._arm._both_checker = FakeChecker()
    kplan = two_hand_hearts_plan()
    steps = np.abs(np.diff(kplan.hands, axis=0)).max(axis=1)
    peak = int(np.argmax(steps)) + 1
    assert steps.max() > PEAK_SPEED / RATE  # Unclamped, the plan would step past the limit.
    submit = mocker.spy(module._execution, "submit_chunk")
    _, stops = stop_at_row(module, mocker, LEAD_IN_ROWS + peak, on_row=module._arm.stop.set)
    result = call_with_feed(
        module, lambda: module.perform_gesture("two_hand_hearts"), lambda t: np.zeros(29)
    )
    assert "stopped early: stop requested" in result.message, result
    [(row, _, _)] = stops
    # The retreat's hand columns are the plan's own way back (retreat_hands), blend rows included.
    segment, offset = sonic_arm_reference.retreat_route(kplan, row - LEAD_IN_ROWS)
    unit = sonic_arm_reference.retreat_hands(kplan, segment)
    retreat = unheld(submit.call_args_list[1].args[3])
    assert np.array_equal(hands_of(retreat), padded(unit)[LEAD_IN_ROWS + offset :])
    sent = np.array(
        [pose_values(p)[64:] for p in peer.snapshot() if p.startswith(b"pose")], np.float64
    )
    sent_steps = np.abs(np.diff(sent, axis=0)).max(axis=1)
    assert sent_steps.max() <= PEAK_SPEED / RATE + 1e-6
    assert sent_steps.max() > 0.5 * PEAK_SPEED / RATE  # The hands did move on the wire.
    assert module._execution.get_status()["task"]["handback_confirmed"]


def test_support_person_from_a_bent_elbow_is_refused_before_sending(skill, mocker):
    module, peer = skill
    module._arm._both_checker = FakeChecker()
    real = module._execution.get_feedback

    def bent():
        state = real()
        q = list(state.body_q)
        q[18] = -0.8  # left elbow; support_person adds -0.5, past the -1.0472 limit
        return replace(state, body_q=tuple(q))

    mocker.patch.object(module._execution, "get_feedback", bent)
    with pytest.raises(
        ValueError, match=r"support_person keyframes\[1\]: left_elbow = -1\.3 outside its limit"
    ):
        module.perform_gesture("support_person")
    assert module._arm._both_checker.calls == 0
    assert not any(p.startswith(b"pose") for p in peer.snapshot())
    assert module._execution.get_status()["task"] is None


def test_imported_gestures_need_the_both_arm_path_check(real_launch_skill, mocker):
    module, peer = real_launch_skill
    start_tool = mocker.spy(module, "start_tool")
    with pytest.raises(RuntimeError, match="both-arm path check"):
        module.perform_gesture("two_hand_hearts")
    assert start_tool.call_count == 0 and module._running is None  # Refused before starting.
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_imported_gesture_paths_go_through_the_both_arm_checker(skill):
    module, peer = skill
    module._arm._checker = FakeChecker()
    module._arm._both_checker = FakeChecker("left_elbow_link (other arm)")
    with pytest.raises(RuntimeError, match=r"left_elbow_link \(other arm\); nothing sent"):
        module.perform_gesture("two_hand_hearts")
    assert module._arm._checker.calls == 0 and module._arm._both_checker.calls >= 1
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_start_builds_the_both_arm_checker_from_the_collision_model(mocker):
    checker = mocker.patch.object(sonic_arm_skill, "ArmCollisionChecker")
    mocker.patch.object(sonic_arm_skill, "ArmKinematics")
    mocker.patch.object(sonic_arm_skill, "SonicEncoder", FakeEncoder)
    module = SonicArmSkillContainer(encoder_path="unused.onnx", collision_model_path="scene.xml")
    # start() builds the SonicArm over the wired execution; this test never moves it.
    mocker.patch.object(module, "_execution", mocker.MagicMock(), create=True)
    transport = make_transport(f"/arm-skill-test/{uuid4().hex}", SonicFeedback)
    module.set_transport("feedback", transport)
    try:
        transport.start()
        module.start()
        scene = Path("scene.xml")
        assert checker.call_args_list == [
            mocker.call(scene),
            mocker.call(scene, arms=("left", "right")),
        ]
    finally:
        module.stop()
        transport.stop()


def test_finger_check_submits_the_asymmetric_pose_with_the_body_at_its_start(
    real_launch_skill, mocker
):
    module, _ = real_launch_skill
    mocker.patch.object(module.config, "hands_mounted", True)
    submit = mocker.spy(module._execution, "submit_chunk")
    encode = mocker.spy(module._arm._encoder, "encode")
    with feeding(module):
        assert module.check_dex3_finger_order() == "started"
        assert module.operator_result() is None
        wait_until(lambda: module.operator_result() is not None)
    hands = hands_of(submit.call_args_list[0].args[3])
    assert any(np.array_equal(h, FINGER_POSE) for h in hands)
    assert np.array_equal(hands[0], PEER_HANDS) and np.array_equal(hands[-1], PEER_HANDS)
    poses = encode.call_args_list[0].args[0]
    assert np.all(poses == poses[0])  # Arms and body stay at the measured start.
    result = module.operator_result()
    assert "LEFT index open, middle closed, thumb half" in result, result
    assert "RIGHT middle open, index closed, thumb rotated" in result, result
    assert module._running is None and module._arm._watch is None


def test_failed_finger_check_delivers_the_hand_diagnostics(real_launch_skill, mocker):
    module, _ = real_launch_skill
    mocker.patch.object(module.config, "hands_mounted", True)
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", real_hold_plan)  # real holds
    with feeding(module, hands=lambda t: PEER_HANDS):  # Constant readings: they do not follow.
        assert module.check_dex3_finger_order() == "started"
        wait_until(lambda: module.operator_result() is not None, 30)
    result = module.operator_result()
    assert f'"message": "{sonic_arm_skill.FINGER_CHECK} failed: ' in result, result
    assert "hand did not follow" in result, result
    assert "hand_held" in result and "hand_target" in result, result


def test_finger_check_refuses_while_a_motion_runs(real_launch_skill, mocker):
    module, _ = real_launch_skill
    mocker.patch.object(module.config, "hands_mounted", True)
    gate, _ = gate_start(module, mocker)
    assert "started" in module.move_upper_body("left_arm_forward", 20.0)
    with pytest.raises(RuntimeError, match="an arm move is already running"):
        module.check_dex3_finger_order()
    gate.set()
    wait_until(lambda: module._running is None)
    assert module.operator_result() is None


def test_finger_check_without_hands_configured_is_refused(real_launch_skill):
    module, peer = real_launch_skill
    with pytest.raises(RuntimeError, match="no hands configured"):
        module.check_dex3_finger_order()
    assert module._running is None and module.operator_result() is None
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_finger_check_is_an_operator_rpc_not_a_skill(real_launch_skill):
    module, _ = real_launch_skill
    names = {s.func_name for s in module.get_skills()}
    assert "perform_gesture" in names and "stop_arm" in names
    assert not {"check_dex3_finger_order", "operator_result"} & names
    for rpc in (
        SonicArmSkillContainer.check_dex3_finger_order,
        SonicArmSkillContainer.operator_result,
    ):
        assert getattr(rpc, "__rpc__", False) and not getattr(rpc, "__skill__", False)


def hand_only_run(module, mocker, hands=None):
    """A motion that moves only the hands, with a real 2 s hold, while hand telemetry plays."""
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    with feeding(module, hands=hands):
        return module._run(
            "fingers",
            lambda base, base_hands: [Keyframe(base.copy(), 2.0, hands=FINGER_POSE)],
            group_margins(0.15, 0.2, 0.3),
            require_check=False,
        )


def test_hand_only_motion_with_following_hands_reports_the_pose_held(skill, mocker):
    module, _ = skill
    mocker.patch.object(module.config, "hands_mounted", True)
    result = hand_only_run(module, mocker, hands=lambda t: FINGER_POSE + 0.05)
    assert "hand pose held" in result.message, result
    text = result.agent_encode()[0]["text"]
    assert "NaN" not in text and "did not follow" not in text, text
    assert result.metadata["hand_hold_samples"] > 10
    assert result.metadata["hand_worst_error_rad"] == pytest.approx(0.05, abs=1e-6)
    assert len(result.metadata["hand_held"]) == 14 and len(result.metadata["hand_target"]) == 14
    assert result.metadata["hand_held"] == np.round(FINGER_POSE + 0.05, 3).tolist()


def test_finger_check_pose_is_within_the_hand_limits():
    for side, offset in (("left", 0), ("right", 7)):
        for slot, (lo, hi) in enumerate(HAND_LIMITS[side]):
            value = sonic_arm_skill.FINGER_CHECK_HANDS[offset + slot]
            assert lo <= value <= hi, (side, HAND_ORDER[slot], value)


@pytest.mark.parametrize(
    ("hands", "worst"),
    [
        (lambda t: np.zeros(14), "left middle_1 off by 1.75 rad"),  # all-zero readings
        (None, "right thumb_2 off by 1.95 rad"),  # the peer's unchanged readings
    ],
)
def test_hand_only_motion_with_hands_that_do_not_move_fails(skill, mocker, hands, worst):
    module, _ = skill
    mocker.patch.object(module.config, "hands_mounted", True)
    with pytest.raises(RuntimeError, match=f"hand did not follow: {worst}") as error:
        hand_only_run(module, mocker, hands)
    assert "unavailable" not in str(error.value) and "NaN" not in str(error.value)
    # What a failed run delivers: the same text and metadata as a successful one.
    text = SkillResult(f"fingers failed: {error.value}", metadata=error.value.metadata)
    text = text.agent_encode()[0]["text"]
    assert '"message": "fingers failed: ' in text and "hand did not follow" in text, text
    for key in ("hand_base", "hand_target", "hand_held", "hand_worst_finger"):
        assert key in text, (key, text)


def test_hand_only_motion_without_hands_configured_claims_nothing_about_the_pose(skill, mocker):
    module, _ = skill
    result = hand_only_run(module, mocker, hands=lambda t: FINGER_POSE)
    assert "hand feedback unavailable (no hands configured)" in result.message, result
    text = result.agent_encode()[0]["text"]
    assert "held" not in text and "did not follow" not in text and "NaN" not in text, text


def test_a_finger_only_move_mid_plan_is_never_a_split_point(skill, mocker):
    module, _ = skill
    mocker.patch.object(
        sonic_arm_skill,
        "keyframe_plan",
        real_hold_plan,  # real holds
    )
    # The latest split the first chunk allows (row 115) lies inside the finger move (rows 106-122):
    # the tokens stay still there, since they do not encode the hands.
    mocker.patch.object(sonic_arm_skill, "_MAX_CHUNK_ROWS", 140)
    submit = mocker.spy(module._execution, "submit_chunk")
    arm = 0.3 * np.eye(29)[15]
    call_with_feed(
        module,
        lambda: module._run(
            "fingers",
            lambda base, hands: [
                Keyframe(base + arm, 1.5),
                Keyframe(base + arm, 1.5, hands=FINGER_POSE),
            ],
            pose_margins(0.15, 0.45, 0.35),
            require_check=False,
        ),
        lambda t: np.zeros(29),
    )
    chunks = [c.args[3] for c in submit.call_args_list]
    assert len(chunks) >= 2
    starts = np.cumsum([0] + [len(c) - 25 for c in chunks[:-1]])
    plan = np.zeros((starts[-1] + len(chunks[-1]), 78))
    for start, rows in zip(starts, chunks, strict=True):
        plan[start : start + len(rows)] = rows
    moving = np.flatnonzero(np.any(np.diff(hands_of(plan), axis=0) != 0, axis=1))
    assert len(moving) and moving.min() > LEAD_IN_ROWS  # The fingers move mid-plan.
    for split in starts[1:]:
        window = plan[split - 1 : split + sonic_arm_reference._SPLIT_POSE_ROWS - 1]
        assert np.all(window == window[0]), (split, moving.min(), moving.max())


@pytest.fixture
def worker_arm(peer, execution_config, dimos_cluster):
    """The execution module and the arm container each in a worker, wired as the blueprint does."""
    profile = replace(execution_config["profile"], max_observation_age=25.0, max_duration=60.0)
    execution = dimos_cluster.deploy(
        SonicExecutionModule, **(execution_config | {"profile": profile})
    )
    arm = dimos_cluster.deploy(
        SonicArmSkillContainer, encoder_path=ENCODER, hands_mounted=True, return_watch_seconds=0.0
    )
    transport = make_transport(f"/arm-skill-worker-test/{uuid4().hex}", SonicFeedback)
    try:
        execution.set_transport("feedback", transport)
        arm.set_transport("feedback", transport)
        arm.set_module_ref("_execution", execution)
        execution.start()
        peer.reference_endpoint = execution.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: execution.get_status()["ready"], timeout=WAIT_S)
        arm.start()
        yield arm, execution
    finally:
        arm.stop()
        execution.stop()


@pytest.mark.skipif(not ENCODER.is_file(), reason="GEAR_SONIC_ENCODER is unset or missing")
def test_finger_check_through_the_worker_returns_at_once_and_stop_arm_retreats(worker_arm, peer):
    arm, execution = worker_arm
    called = time.monotonic()
    assert arm.check_dex3_finger_order() == "started"
    assert time.monotonic() - called < 1.0
    assert arm.operator_result() is None

    # The transmitted hands reached the finger pose: the plan is in the hold. Then a few more.
    def sent_hands():
        return [pose_values(p)[64:] for p in peer.snapshot() if p.startswith(b"pose")]

    wait_until(lambda: any(np.allclose(h, FINGER_POSE, atol=1e-3) for h in sent_hands()), 30)
    arrived = len(sent_hands())
    wait_until(lambda: len(sent_hands()) >= arrived + 10, 30)
    called = time.monotonic()
    assert arm.stop_arm().startswith("Stopping")
    assert time.monotonic() - called < 1.0
    wait_until(lambda: arm.operator_result() is not None, 30)
    result = arm.operator_result()
    assert "stopped early: stop requested" in result, result
    task = execution.get_status()["task"]
    assert task["last_chunk_sequence"] == 1 and task["handback_confirmed"], task  # The retreat.
