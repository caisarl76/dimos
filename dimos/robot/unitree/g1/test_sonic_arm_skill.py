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

import asyncio
from contextlib import contextmanager
from dataclasses import replace
from functools import partial
import json
import math
from pathlib import Path
import re
import sys
import threading
import time
from uuid import uuid4

import numpy as np
import pytest

from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp import mcp_server
from dimos.agents.mcp.tool_stream import NOTIFICATIONS_MESSAGE_METHOD, TOOL_STREAM_STOPPED_METHOD
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_reference, sonic_arm_skill
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
from dimos.robot.unitree.g1.sonic_arm_skill import GESTURES, SonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_session import SessionPhase, SonicModelProfile, SonicTaskSession
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_ARM_SYSTEM_PROMPT
from dimos.robot.unitree.g1.test_sonic_execution_module import (
    execution as execution,
    peer as peer,
)
from dimos.robot.unitree.g1.test_sonic_session import feedback, pose_values

PEER_TOKEN = 0.125  # ScriptedPeer token_state
FAKE_GAIN = 0.25  # Fake token value 0 per rad of summed joint offset; keeps big poses in the bound.


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
        ),
    )


class FakeEncoder:
    def __init__(self, path):
        pass

    def encode(self, poses):
        tokens = np.full((len(poses), 64), PEER_TOKEN)
        tokens[:, 0] += FAKE_GAIN * (poses - poses[0]).sum(axis=1)  # The planned offsets.
        return tokens


def fast_keyframe_plan(base, keyframes, **kwargs):
    """The real engine with short leads, moves and holds, so a run takes about a second."""
    capped = [replace(k, hold_s=min(k.hold_s, 0.1)) for k in keyframes]
    return keyframe_plan(base, capped, **kwargs, lead_s=0.1, move_s=0.3, peak_speed=100.0)


@pytest.fixture
def tool_transport(mocker):
    """The transport every ToolStream publishes on, mocked as in test_tool_stream."""
    transport = mocker.MagicMock()
    mocker.patch("dimos.agents.mcp.tool_stream.make_transport", return_value=transport)
    return transport


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
def feeding(module, body_q=lambda t: np.zeros(29)):
    """A feeder plays body telemetry into the skill (t = seconds since entering)."""
    stopped = threading.Event()

    def feed():
        start = time.monotonic()
        while not stopped.is_set():
            state = module._execution.get_feedback()
            if state is not None:
                q = tuple(float(v) for v in body_q(time.monotonic() - start))
                module.handle_feedback(replace(state, body_q=q))
            time.sleep(0.01)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        yield
    finally:
        stopped.set()
        feeder.join(2)


def run(module, angle_deg, body_q=lambda t: np.zeros(29), motion="left_arm_forward"):
    """Call the skill while a feeder plays body telemetry (t = seconds since the call)."""
    with feeding(module, body_q):
        return module.move_upper_body(motion, angle_deg)


def wait_until(condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.01)


def follows(t, delta=-np.radians(20), joint=15):  # The moving joint at the plan's depth mid-run.
    q = np.zeros(29)
    q[joint] = delta if 0.5 < t < 0.9 else 0.0
    return q


def tokens(packets):
    return np.array([np.frombuffer(p[1284:1540], "<f4") for p in packets if p.startswith(b"pose")])


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
    pad = (len(first) - len(kplan.poses)) // 2
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
        partial(keyframe_plan, lead_s=0.1, move_s=0.3, peak_speed=100.0),  # real holds
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


def test_session_failure_is_not_reported_as_success(skill):
    module, peer = skill

    def drop_feedback():
        time.sleep(0.5)
        peer.pause_feedback.set()  # The execution session fails on stale feedback...
        time.sleep(0.3)
        peer.pause_feedback.clear()  # ...and confirms the planner once feedback returns.

    dropper = threading.Thread(target=drop_feedback, daemon=True)
    dropper.start()
    with pytest.raises(RuntimeError, match="control returned to the planner"):
        run(module, 20.0, follows)
    dropper.join(2)
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


def frames(transport, method):
    return [c.args[0] for c in transport.publish.call_args_list if c.args[0]["method"] == method]


def gate_start(module, mocker):
    """Holds the motion before its execution task starts until the returned event is set."""
    gate = threading.Event()
    real = module._execution.start_task
    start = mocker.patch.object(
        module._execution, "start_task", side_effect=lambda: gate.wait(10) and real()
    )
    return gate, start


@pytest.fixture
def mcp(real_launch_skill, tool_transport, monkeypatch):
    """McpServer's tools/call and tool-stream handlers over the arm module, with a fresh registry."""
    module, _ = real_launch_skill
    skills = {s.func_name: s for s in module.get_skills()}
    registry = CapabilityRegistry()
    monkeypatch.setattr(mcp_server.app.state, "skills_by_name", skills)
    monkeypatch.setattr(mcp_server.app.state, "cap_registry", registry)
    monkeypatch.setattr(mcp_server.app.state, "event_loop", None)
    # Every stop frame reaches the server's handler, which releases the hold it carries.
    tool_transport.publish.side_effect = mcp_server._fan_out_to_sse_queues
    rpc_calls = {name: getattr(module, name) for name in skills}

    def call(name, **arguments):
        params = {"name": name, "arguments": arguments}
        request = {"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": params}
        response = asyncio.run(mcp_server.handle_request(request, list(skills.values()), rpc_calls))
        result = response["result"]
        return result["content"][0]["text"], result.get("isError", False)

    return call, registry


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
            release.wait(10)
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


_MODEL = "gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
SCENE = next(
    (
        root / _MODEL
        for root in (
            Path.home() / "work/GR00T-WholeBodyControl-hand-tracking-watchdog",
            Path.home() / "GR00T-WholeBodyControl",  # the G1 PC2
        )
        if (root / _MODEL).exists()
    ),
    Path("/nonexistent"),
)
needs_scene = pytest.mark.skipif(
    not SCENE.exists(), reason="gear_sonic G1 MuJoCo model not on this host"
)
ROBOT_IDLE = np.array(
    [
        -0.312,
        0.044,
        0,
        0.669,
        -0.363,
        0,
        -0.312,
        0,
        0,
        0.669,
        -0.363,
        0,
        0,
        0,
        0,
        -0.02,
        0.284,
        -0.651,
        0.984,
        -0.152,
        -0.06,
        0.011,
        0.2,
        -0.2,
        0,
        0.6,
        0,
        0,
        0,
    ]
)


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


def test_gesture_checks_the_path_when_configured_and_runs_without(skill):
    module, _ = skill
    module._arm._checker = FakeChecker("torso_link")
    with pytest.raises(RuntimeError, match="torso_link; nothing sent"):
        module.perform_gesture("stop_sign")
    module._arm._checker = None
    result = call_with_feed(
        module, lambda: module.perform_gesture("stop_sign"), reaching(GESTURES["stop_sign"][0][0])
    )
    assert "returned" in result.message, result


def test_report_extra_is_merged_into_the_metadata(skill):
    module, _ = skill
    allowed = pose_margins(0.15, 0.45, 0.35)
    seen = []

    def extra(base):
        seen.append(base.copy())
        return {"direction": "left"}

    result = call_with_feed(
        module,
        lambda: module._run(
            "probe",
            lambda base: [Keyframe(base + 0.3 * np.eye(29)[15], 0.1)],
            allowed,
            require_check=False,
            report_extra=extra,
        ),
        follows,
    )
    assert result.metadata["direction"] == "left" and result.metadata["motion"] == "probe"
    assert len(seen) == 1 and seen[0].shape == (29,)


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


def test_split_overhead_plan_streams_without_a_ripple_at_the_second_chunk():
    """The slowest pose (overhead at 0.47 rad/s, 31 s) through the real session on a simulated clock."""
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


def one_joint_prepare(module, mocker=None):
    allowed = group_margins(0.15, 0.2, 0.3)
    delta = np.zeros(29)
    delta[15] = -math.radians(20)
    return module._arm.prepare(
        "p", lambda base: [Keyframe(base + delta, 1.0)], allowed, require_check=False
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
    mocker.patch.object(sys.modules[__name__], "FAKE_GAIN", 50.0)
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
