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

from dataclasses import replace
from functools import partial
import re
import threading
import time
from uuid import uuid4

import numpy as np
import pytest

from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_skill
from dimos.robot.unitree.g1.sonic_arm_reference import (
    PEAK_SPEED,
    arm_poses,
    group_margins,
    min_jerk,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import SonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_session import SessionPhase, SonicModelProfile, SonicTaskSession
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_ARM_SYSTEM_PROMPT
from dimos.robot.unitree.g1.test_sonic_execution_module import (
    execution as execution,
    peer as peer,
)
from dimos.robot.unitree.g1.test_sonic_session import feedback, pose_values

PEER_TOKEN = 0.125  # ScriptedPeer token_state


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
            max_observation_age=3.0,
        ),
    )


class FakeEncoder:
    def __init__(self, path):
        pass

    def encode(self, poses):
        tokens = np.full((len(poses), 64), PEER_TOKEN)
        tokens[:, 0] += (poses - poses[0]).sum(
            axis=1
        )  # The planned offset of whichever joint moves.
        return tokens


@pytest.fixture
def skill(execution, mocker):
    mocker.patch.object(sonic_arm_skill, "SonicEncoder", FakeEncoder)
    mocker.patch.object(
        sonic_arm_skill,
        "arm_poses",
        partial(arm_poses, lead_s=0.1, move_s=0.3, hold_s=0.1, peak_speed=100.0),
    )
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


def run(module, angle_deg, body_q=lambda t: np.zeros(29), motion="left_arm_forward"):
    """Call the skill while a feeder plays body telemetry (t = seconds since the call)."""
    module_execution = module._execution
    stopped = threading.Event()

    def feed():
        start = time.monotonic()
        while not stopped.is_set():
            state = module_execution.get_feedback()
            if state is not None:
                q = tuple(float(v) for v in body_q(time.monotonic() - start))
                module.handle_feedback(replace(state, body_q=q))
            time.sleep(0.01)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        return module.move_upper_body(motion, angle_deg)
    finally:
        stopped.set()
        feeder.join(2)


def follows(t, delta=-np.radians(20), joint=15):  # The moving joint at the plan's depth mid-run.
    q = np.zeros(29)
    q[joint] = delta if 0.5 < t < 0.9 else 0.0
    return q


def tokens(packets):
    return np.array([np.frombuffer(p[1284:1540], "<f4") for p in packets if p.startswith(b"pose")])


def test_pure_reference_parts():
    poses = arm_poses(np.zeros(29), 15, -0.3)
    assert poses.shape == (350, 29) and np.allclose(np.delete(poses, 15, 1), 0)
    assert poses[0, 15] == poses[-1, 15] == 0 and np.isclose(poses[:, 15].min(), -0.3)
    assert np.abs(np.diff(min_jerk(100))).max() * 50 < 1.875 / (99 / 50) + 1e-3  # min-jerk peak
    schedule = token_schedule(np.zeros(64), np.ones((10, 64)))
    assert len(schedule) == 210 and np.allclose(schedule[[0, -1], 0], [0.01, 0.0])
    margins = group_margins(0.15, 0.2, 0.3)
    assert margins[3] == margins[13] == 0.15 and margins[16] == 0.2 and margins[26] == 0.3


def test_swing_streams_one_chunk_and_hands_back(skill):
    module, peer = skill
    result = run(module, 20.0, follows)
    assert result.success, result
    assert result.metadata["reached_deg"] == pytest.approx(20.0, abs=0.1)
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "finished" and status["handback_confirmed"]
    assert status["last_chunk_sequence"] == 0  # One open-loop chunk, as commissioned.
    packets = peer.snapshot()
    assert all(
        p[-3:-1] == b"\0\0" for p in packets if p.startswith(b"command")
    )  # Never start/stop.
    streamed = tokens(packets)[:, 0]
    assert streamed.min() == pytest.approx(PEER_TOKEN - np.radians(20), abs=0.01)  # Full depth.
    assert np.abs(np.diff(streamed)).max() <= 0.05 + 1e-6
    assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)  # Blended back before handback.


def test_wrist_inside_its_envelope_passes(skill):
    def wrist(t):
        q = follows(t)
        q[26] = 0.25
        return q

    result = run(skill[0], 20.0, wrist)
    assert result.success, result
    assert result.metadata["max_deviation_rad"]["wrists"] == 0.25


def test_leg_outside_its_envelope_cancels_to_planner(skill):
    module, _ = skill

    def knee(t):
        q = follows(t)
        q[3] = 0.2 if t > 0.4 else 0.0
        return q

    started = time.monotonic()
    result = run(module, 20.0, knee)
    assert result.error_code == "ENVELOPE" and "left_knee" in result.message, result
    status = module._execution.get_status()["task"]
    assert status["outcome"] == "cancelled" and status["handback_confirmed"]
    assert time.monotonic() - started < 0.4 + 1.2  # Cancelled before the plan ended.


def test_arm_that_does_not_follow_is_reported(skill):
    result = run(skill[0], 20.0)
    assert result.error_code == "NOT_REACHED", result


@pytest.mark.parametrize("angle", [0.0, 29.0, float("nan")])
def test_invalid_angle_sends_nothing(skill, angle):
    module, peer = skill
    before = len(peer.snapshot())
    assert module.move_upper_body("left_arm_forward", angle).error_code == "INVALID_ANGLE"
    assert not any(p.startswith(b"pose") for p in peer.snapshot()[before:])
    assert module._execution.get_status()["task"] is None


def test_movement_during_encoding_sends_nothing(skill, mocker):
    module, _ = skill
    state = module._execution.get_feedback()
    moved = replace(state, body_q=(0.1,) * 29)
    mocker.patch.object(module._execution, "get_feedback", side_effect=[state, moved])
    assert module.move_upper_body("left_arm_forward", 20.0).error_code == "MOVED"
    assert module._execution.get_status()["task"] is None


def test_not_ready_execution_sends_nothing(skill, mocker):
    module, _ = skill
    status = module._execution.get_status()
    mocker.patch.object(module._execution, "get_status", return_value=status | {"ready": False})
    assert module.move_upper_body("left_arm_forward", 20.0).error_code == "NOT_READY"


def test_session_failure_is_not_reported_as_success(skill):
    module, peer = skill

    def drop_feedback():
        time.sleep(0.5)
        peer.pause_feedback.set()  # The execution session fails on stale feedback...
        time.sleep(0.3)
        peer.pause_feedback.clear()  # ...and confirms the planner once feedback returns.

    dropper = threading.Thread(target=drop_feedback, daemon=True)
    dropper.start()
    result = run(module, 20.0, follows)
    dropper.join(2)
    assert result.error_code == "EXECUTION_FAILED", result
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
    motion = np.full((350, 64), 0.125)
    motion[:, 0] += arm_poses(np.zeros(29), 15, -0.5)[:, 15]
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
    assert module.move_upper_body("left_arm_forward", 20.0).error_code == "NOT_READY"
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_failed_session_at_finish_is_not_reported_as_success(skill, mocker):
    module, _ = skill
    execution = module._execution
    fail = execution.fail_task
    mocker.patch.object(
        execution, "finish_task", side_effect=lambda task_id: fail(task_id, "injected")
    )
    result = run(module, 20.0, follows)
    assert result.error_code == "EXECUTION_FAILED" and "injected" in result.message, result


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
    assert result.success, result
    assert result.metadata["motion"] == motion and result.metadata["reached_deg"] == pytest.approx(
        18.9, abs=0.1
    )
    streamed = tokens(peer.snapshot())[:, 0]
    assert streamed.max() - streamed.min() == pytest.approx(np.radians(22), abs=0.01)


def test_unknown_motion_sends_nothing(skill):
    module, peer = skill
    assert module.move_upper_body("waist_turn_left", 10.0).error_code == "INVALID_MOTION"
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
    assert module.move_upper_body(motion, angle).error_code == "INVALID_ANGLE"
    assert not any(p.startswith(b"pose") for p in peer.snapshot())


def test_big_strokes_slow_down_to_the_commissioned_peak_speed():
    commissioned = arm_poses(np.zeros(29), 15, -0.5)  # 0.5 rad over 2 s: the speed cap's reference
    assert len(commissioned) == 350
    big = arm_poses(np.zeros(29), 18, -np.radians(60))
    speed = np.abs(np.diff(big[:, 18])).max() * 50
    assert speed <= PEAK_SPEED + 0.01
    assert len(big) == 50 + 2 * round(1.875 * np.radians(60) / 0.47 * 50) + 50 + 50
