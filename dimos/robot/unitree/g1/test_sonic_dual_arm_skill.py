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

"""The dual-robot arm container and blueprints, over two scripted SONIC executions.

Each robot has its own scripted peer; the lifecycle tests reuse test_sonic_arm_skill's MCP and
tool-stream fixtures with the dual container in place of the single one.
"""

from dataclasses import replace
from functools import partial
import json
import os
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
from typing import get_args
from uuid import uuid4

import numpy as np
import pytest

from dimos.agents.mcp.tool_stream import TOOL_STREAM_STOPPED_METHOD
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import GlobalConfig
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_skill, sonic_dual_arm_skill
from dimos.robot.unitree.g1.blueprints.agentic.unitree_g1_sonic_arm_dual_agentic import (
    unitree_g1_sonic_arm_dual_agentic,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm_dual import (
    unitree_g1_sonic_arm_dual,
)
from dimos.robot.unitree.g1.imported_gestures import HAND_DEFAULT
from dimos.robot.unitree.g1.sonic_arm_reference import (
    PEAK_SPEED,
    RATE,
    keyframe_plan,
    retreat_hands,
    retreat_poses,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    FINGER_CHECK_HANDS,
    GESTURES,
    IMPORTED_MOVE_S,
    gesture_keyframes,
    gesture_plan_inputs,
)
from dimos.robot.unitree.g1.sonic_dual_arm_skill import (
    _ROBOTS,
    DualSonicArmSkillConfig,
    DualSonicArmSkillContainer,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_test_support import (
    ARM_PROFILE,
    FAKE_GAIN,
    PEER_TOKEN,
    FakeEncoder,
    fast_keyframe_plan,
    feeding_arms,
    frames,
    gate_start,
    no_pose,
    pose_values,
    real_hold_plan,
    scripted_peer,
    stop_in_background,
    task,
    tokens,
    unheld,
    wait_until,
)
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_DUAL_ARM_SYSTEM_PROMPT


class AbsoluteEncoder(FakeEncoder):
    """Tokens that depend on the pose alone, so a retreat unit starting mid-motion joins the plan."""

    def encode(self, poses):
        tokens = np.full((len(poses), 64), PEER_TOKEN)
        tokens[:, 0] += FAKE_GAIN / 2 * poses.sum(axis=1)
        return tokens


@pytest.fixture
def real_launch_skill(request, dual_executions, tool_transport, mocker):
    """The dual container started on both executions; yields (module, [(execution, peer)]).

    Indirect parametrization adds config fields (the voice tests' hosts and command).
    """
    mocker.patch.object(sonic_dual_arm_skill, "SonicEncoder", AbsoluteEncoder)
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", fast_keyframe_plan)
    mocker.patch.object(sonic_arm_skill, "token_schedule", partial(token_schedule, blend_s=0.2))
    module = DualSonicArmSkillContainer(
        encoder_path="unused.onnx",
        return_watch_seconds=0.0,
        hands_mounted_222=True,
        **getattr(request, "param", {}),
    )
    (first, _), (second, _) = dual_executions
    mocker.patch.object(module, "_execution_222", first, create=True)
    mocker.patch.object(module, "_execution_223", second, create=True)
    transports = [make_transport(f"/dual-arm-test/{uuid4().hex}", SonicFeedback) for _ in range(2)]
    for name, transport in zip(("feedback_222", "feedback_223"), transports, strict=True):
        module.set_transport(name, transport)
    try:
        for transport in transports:
            transport.start()
        module.start()
        yield module, dual_executions
    finally:
        module.stop()
        for transport in transports:
            transport.stop()


@pytest.fixture
def inline(real_launch_skill, mocker):
    """The container with each motion run on the calling thread: its SkillResult or a raise."""
    module, executions = real_launch_skill
    mocker.patch.object(module, "_launch", lambda tool, prepare, arms=None: prepare()())
    return module, executions


def gate_starts(module, mocker):
    """Holds both robots' motions before their tasks start until the returned events are set."""
    gates = [
        gate_start(SimpleNamespace(_execution=execution), mocker)
        for execution in (module._execution_222, module._execution_223)
    ]
    return [g for g, _ in gates], [s for _, s in gates]


def zero(s):
    return 0.0


def test_dual_blueprint_wires_each_robot_to_its_own_execution():
    atoms = {a.name: a for a in unitree_g1_sonic_arm_dual.blueprints}
    assert atoms["g1_222"].module is atoms["g1_223"].module is SonicExecutionModule
    remap = unitree_g1_sonic_arm_dual.remapping_map
    for robot in ("222", "223"):
        assert remap[f"g1_{robot}", "feedback"] == f"feedback_{robot}"
        assert remap[f"g1_{robot}", "execution_status"] == f"execution_status_{robot}"
    container = DualSonicArmSkillContainer.name
    assert remap[container, "_execution_222"] == "g1_222"
    assert remap[container, "_execution_223"] == "g1_223"
    ports = {"222": ("15556", "15557"), "223": ("16556", "16557")}
    for robot, (reference, feedback) in ports.items():
        kwargs = atoms[f"g1_{robot}"].kwargs
        assert kwargs["reference_bind"] == f"tcp://127.0.0.1:{reference}"
        assert kwargs["feedback_endpoint"] == f"tcp://127.0.0.1:{feedback}"
        assert kwargs["journal_dir"].parts[-2:] == ("sonic-arm", robot)


def test_dual_blueprint_namespaced_keeps_each_robots_execution():
    remap = unitree_g1_sonic_arm_dual.namespace("pair").remapping_map
    container = f"pair/{DualSonicArmSkillContainer.name}"
    assert remap[container, "_execution_222"] == "pair/g1_222"
    assert remap[container, "_execution_223"] == "pair/g1_223"


def test_dual_prompt_names_only_dual_skills():
    named = set(re.findall(r"`(\w+)\(", G1_SONIC_DUAL_ARM_SYSTEM_PROMPT))
    skills = {
        k for k, v in vars(DualSonicArmSkillContainer).items() if getattr(v, "__skill__", False)
    }
    assert named and named <= skills, named - skills


def test_dual_prompt_names_every_gesture():
    for name in get_args(sonic_arm_skill.GestureName):
        assert f"`{name}`" in G1_SONIC_DUAL_ARM_SYSTEM_PROMPT, name


@pytest.mark.parametrize("robot", ["222", 222])  # `dimos mcp call --arg robot=222` sends an int.
def test_single_robot_command_ignores_the_other_robot(inline, robot):
    module, executions = inline
    peer = executions[1][1]
    peer.pause_feedback.set()
    try:
        wait_until(lambda: module._execution_223.get_feedback() is None)
        with feeding_arms(module._arms()):
            result = module.move_upper_body(robot, "left_arm_forward", 20.0)
    finally:
        peer.pause_feedback.clear()
    assert result.message.startswith("left_arm_forward reached"), result
    assert "returned" in result.message and "223" not in result.metadata
    assert no_pose(peer)


def test_unknown_robot_refuses_before_sending(inline):
    module, executions = inline
    with pytest.raises(ValueError, match=re.escape("robot must be one of ['222', '223', 'both']")):
        module.move_upper_body(224, "left_arm_forward", 20.0)
    assert all(no_pose(peer) for _, peer in executions)


def test_gesture_both_through_the_container(inline):
    module, _ = inline
    with feeding_arms(module._arms()):
        result = module.perform_gesture("both", "wave")
    assert "222: wave reached" in result.message and "; 223: wave reached" in result.message
    assert result.metadata["start_skew_s"] < 0.5, result.metadata


def test_stop_arm_with_nothing_running_says_so(real_launch_skill):
    module, _ = real_launch_skill
    assert module.stop_arm() == "No arm motion is running."


def _wave_extremes():
    """Seconds after chunk 0 at which each swing of the fast wave plan reaches its extreme."""
    base = np.zeros(29)
    plan = fast_keyframe_plan(base, gesture_keyframes("wave", base))
    return [seg.end / RATE for seg in plan.segments[1:7]]


@pytest.mark.parametrize("stop_s", _wave_extremes())
def test_peer_stop_during_wave_retreats_continuously(inline, stop_s, mocker):
    module, executions = inline
    base = np.zeros(29)
    plan = fast_keyframe_plan(base, gesture_keyframes("wave", base))
    # One row's worth of motion in the fake token: the largest step of the plan or of any retreat
    # unit (the way home from a swing extreme is longer than a swing).
    encoder = AbsoluteEncoder(None)
    units = [retreat_poses(plan, k) for k in range(len(plan.segments))]
    row = max(np.abs(np.diff(encoder.encode(p)[:, 0])).max() for p in (plan.poses, *units))
    submit = mocker.spy(module._execution_223, "submit_chunk")
    # The session samples the chunk on its tick clock, so a packet moves at most one row step per
    # 1/RATE of that clock since the previous packet. Emits are 20-30 ms apart, but 50 ms and more
    # on a loaded host, so the step is bounded per elapsed tick time, not per packet.
    sent = []  # Per robot: the tick time and bytes of every pose packet sent.
    for execution, _ in executions:
        sent.append(poses := [])

        def record(packet, execution=execution, send=execution._send, poses=poses):
            if packet.startswith(b"pose"):
                poses.append((execution._session._now, packet))
            send(packet)

        mocker.patch.object(execution, "_send", record)

    def knee(s):
        return {3: 0.2 if s > stop_s else 0.0}

    with feeding_arms(module._arms(), shape=zero, extra=(knee, None)):
        result = module.perform_gesture("both", "wave")
    assert "stopped early" in result.message and "left_knee" in result.message, result
    assert "stopped early: 222: left_knee" in result.message.split("; 223: ")[1], result
    calls = submit.call_args_list
    first, retreat = unheld(calls[0].args[3]), unheld(calls[-1].args[3])
    assert len(calls) == 2 and len(retreat) < len(first) / 2
    for _, peer in executions:
        wait_until(
            lambda p=peer: tokens(p.snapshot())[-1, 0] == pytest.approx(PEER_TOKEN, abs=1e-3)
        )
        streamed = tokens(peer.snapshot())[:, 0]
        assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)
    for poses in sent:
        ticks = np.diff([t for t, _ in poses]) * RATE
        rows = np.abs(np.diff(tokens([p for _, p in poses])[:, 0])) / ticks / row
        assert rows.max() <= 1.25, rows.max()  # A skipped reverse segment steps over 2 rows.
    for arm in module._arms():
        assert task(arm)["handback_confirmed"], task(arm)


def test_invalid_same_skill_retry_keeps_the_capability(
    real_launch_skill, mcp, tool_transport, mocker
):
    module, _ = real_launch_skill
    call, registry = mcp
    gates, _ = gate_starts(module, mocker)
    assert "started" in call("perform_gesture", robot="both", name="wave")[0]
    text, error = call("perform_gesture", robot="both", name="nope")
    assert "already running" in text and not error  # Not even validated.
    assert registry.snapshot() == {"movement": "perform_gesture"}
    for gate in gates:
        gate.set()
    wait_until(lambda: not module._tools)
    assert registry.snapshot() == {}
    assert len(frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)) == 1


def test_call_during_completion_gets_a_fresh_stream(real_launch_skill, mocker):
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
    module.move_upper_body("both", "left_arm_forward", 20.0)
    first = module._tools["move_upper_body"]
    wait_until(lambda: updates.call_count == 1)  # The first run's cleanup holds the lifecycle.
    second: list[str] = []
    caller = threading.Thread(
        target=lambda: second.append(module.move_upper_body("both", "left_arm_forward", 20.0))
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
    gates, starts = gate_starts(module, mocker)
    stop_tool = mocker.spy(module, "stop_tool")
    barrier = threading.Barrier(8)
    results: list[str] = []

    def start():
        barrier.wait()
        results.append(module.move_upper_body("both", "left_arm_forward", 20.0))

    threads = [threading.Thread(target=start) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(5)
    assert sum("started" in r for r in results) == 1, results
    assert sum("already running" in r for r in results) == 7, results
    for gate in gates:
        gate.set()
    wait_until(lambda: stop_tool.call_count == 1)
    assert [s.call_count for s in starts] == [1, 1]


def test_refusal_never_releases_the_running_motion(real_launch_skill, mocker):
    module, _ = real_launch_skill
    updates = mocker.spy(module, "tool_update")
    stop_tool = mocker.spy(module, "stop_tool")
    with feeding_arms(module._arms()):
        assert "started" in module.perform_gesture("both", "wave")
        assert "already running" in module.perform_gesture("both", "wave")
        with pytest.raises(RuntimeError, match="an arm move is already running"):
            module.move_upper_body("222", "left_arm_forward", 20.0)
        assert stop_tool.call_count == 0 and module._running == "perform_gesture"
        assert not any(arm.stop.is_set() for arm in module._arms())
        wait_until(lambda: updates.call_count == 1)
    message = json.loads(updates.call_args.args[1])["message"]
    assert "222: wave reached" in message and "; 223: wave reached" in message
    assert message.count("and returned") == 2, message
    wait_until(lambda: module._running is None)
    assert stop_tool.call_count == 1


def test_module_stop_mid_motion_retreats_both_and_waits_for_the_update(real_launch_skill, mocker):
    module, executions = real_launch_skill
    submits = [mocker.spy(execution, "submit_chunk") for execution, _ in executions]
    updates = mocker.spy(module, "tool_update")
    with feeding_arms(module._arms()):
        module.move_upper_body("both", "left_arm_forward", 20.0)
        wait_until(lambda: all(s.call_count == 1 for s in submits))  # Both streams armed.
        time.sleep(0.4)  # Mid-motion, past the lead-in blend.
        module.stop()
        # stop() returned only after both retreats and the motion's own final update.
        assert all(arm.stop.is_set() for arm in module._arms())
        assert [[c.args[1] for c in s.call_args_list] for s in submits] == [[0, 1], [0, 1]]
        assert updates.call_count == 1
    first, second = json.loads(updates.call_args.args[1])["message"].split("; 223: ")
    assert "stopped early" in first and "stop requested" in first, first
    assert "stopped early" in second and "stop requested" in second, second
    for arm in module._arms():
        assert task(arm)["handback_confirmed"], task(arm)


def test_motion_racing_module_stop_is_refused_and_nothing_outlives_stop(real_launch_skill, mocker):
    module, _ = real_launch_skill
    gates, starts = gate_starts(module, mocker)
    stopper, release = stop_in_background(module, mocker)
    try:
        with pytest.raises(RuntimeError, match="the arm skill is shutting down"):
            module.perform_gesture("both", "wave")
        assert module.stop_arm() == "No arm motion is running."
    finally:
        release.set()
        stopper.join(10)
    alive = module._motion is not None and module._motion.is_alive()
    assert not stopper.is_alive() and not alive
    assert all(arm.stop.is_set() for arm in module._launched)
    assert module._running is None and not module._tools
    for gate in gates:
        gate.set()
    assert [s.call_count for s in starts] == [0, 0]


def test_same_skill_retry_during_module_stop_keeps_the_hold_until_the_retreat_ends(
    real_launch_skill, mcp, tool_transport, mocker
):
    module, _ = real_launch_skill
    call, registry = mcp
    gates, starts = gate_starts(module, mocker)
    assert "started" in call("perform_gesture", robot="both", name="wave")[0]
    wait_until(lambda: all(s.call_count == 1 for s in starts))  # Both parked at their gates.
    stopper = threading.Thread(target=module.stop)
    stopper.start()
    try:
        wait_until(lambda: all(arm.stop.is_set() for arm in module._arms()))  # stop() has begun.
        text, error = call("perform_gesture", robot="both", name="wave")
        assert "already running" in text and not error  # Admits nothing; rebinds the hold.
        assert registry.snapshot() == {"movement": "perform_gesture"}
        text, _ = call("move_upper_body", robot="222", motion="left_arm_forward", angle_deg=20.0)
        assert "held by 'perform_gesture'" in text
    finally:
        for gate in gates:
            gate.set()
        stopper.join(10)
    assert not stopper.is_alive() and not module._motion.is_alive()
    assert registry.snapshot() == {} and not module._tools
    assert len(frames(tool_transport, TOOL_STREAM_STOPPED_METHOD)) == 1
    text, error = call("perform_gesture", robot="223", name="wave")
    assert error and "the arm skill is shutting down" in text, text
    assert registry.snapshot() == {} and not module._tools


def test_dual_blueprints_set_only_known_global_config_fields() -> None:
    for bp in (unitree_g1_sonic_arm_dual, unitree_g1_sonic_arm_dual_agentic):
        assert set(bp.global_config_overrides) <= set(GlobalConfig.model_fields)


# Imported gestures, per-row Dex3 hands per robot, and the operator finger check.

FIST = np.r_[HAND_DEFAULT["left"], HAND_DEFAULT["right"]]  # 222's Dex3 at the deploy default
UNWIRED = np.zeros(14)  # 223: no hands, so no hand feedback
PAD = 10  # The fixture's 0.2 s token blend at 50 Hz.
ENCODER = Path(os.environ.get("GEAR_SONIC_ENCODER", "/nonexistent"))


class FakeChecker:
    def __init__(self, contact=None):
        self.contact, self.calls = contact, 0

    def first_contact(self, poses, every=5, start=None):
        self.calls += 1
        return self.contact


def hand_timed_plan(base, keyframes, **kwargs):
    """fast_keyframe_plan's short leads, moves and holds at a 3 rad/s peak, where the fingers'
    travel sets the segments that move them: each robot's measured hands change its timing."""
    capped = [replace(k, hold_s=min(k.hold_s, 0.1), min_s=min(k.min_s, 0.1)) for k in keyframes]
    fast = dict(lead_s=0.1, move_s=0.3, peak_speed=3.0)
    fast["return_min_s"] = min(kwargs.get("return_min_s", 0.0), 0.1)
    return keyframe_plan(base, capped, **(kwargs | fast))


def planned(name, hands):
    """The plan a robot at the scripted body pose and these hands builds for an imported gesture."""
    base = np.zeros(29)
    keyframes, move_s, return_min_s, _, _ = gesture_plan_inputs(name, base, hands)
    return hand_timed_plan(
        base, keyframes, base_hands=hands, move_s=move_s, return_min_s=return_min_s
    )


def padded(rows):
    """Pose-row hands [N,14] -> token rows: the token schedule's blends hold the end rows."""
    return np.vstack([np.repeat(rows[:1], PAD, 0), rows, np.repeat(rows[-1:], PAD, 0)])


def hands_of(chunk):
    return np.asarray(chunk)[:, 64:]


def sent_hands(peer):
    return np.array([pose_values(p)[64:] for p in peer.snapshot() if p.startswith(b"pose")])


@pytest.fixture
def hands(inline, mocker):
    """222 reads its Dex3 fist, 223 (handless) zeros; both arms have fake path checks, and plans
    are hand-timed."""
    module, executions = inline
    for (execution, peer), measured in zip(executions, (FIST, UNWIRED), strict=True):
        peer.left_hand_q, peer.right_hand_q = list(measured[:7]), list(measured[7:])
        wait_until(
            lambda e=execution, m=measured: np.array_equal(
                np.r_[e.get_feedback().left_hand_q, e.get_feedback().right_hand_q], m
            )
        )
    for arm in module._arms():
        arm._checker, arm._both_checker = FakeChecker(), FakeChecker()
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", side_effect=hand_timed_plan)
    return module, executions


def plan_calls():
    """(base_hands, move_s, return_min_s) of every keyframe_plan call the skill made."""
    return [
        (c.kwargs["base_hands"], c.kwargs["move_s"], c.kwargs["return_min_s"])
        for c in sonic_arm_skill.keyframe_plan.call_args_list
    ]


def test_both_two_hand_hearts_streams_each_robots_own_moving_hands_step_limited(hands, mocker):
    module, executions = hands
    submits = [mocker.spy(execution, "submit_chunk") for execution, _ in executions]
    with feeding_arms(module._arms()):
        result = module.perform_gesture("both", "two_hand_hearts")
    assert "222: two_hand_hearts reached 100%" in result.message, result
    assert "; 223: two_hand_hearts reached 100%" in result.message, result
    assert result.metadata["start_skew_s"] is not None, result.metadata  # Recorded for both.
    assert result.metadata["start_skew_s"] < 0.5, result.metadata
    lengths = []
    for (_, peer), submit, measured in zip(executions, submits, (FIST, UNWIRED), strict=True):
        kplan = planned("two_hand_hearts", measured)
        assert np.ptp(kplan.hands, axis=0).max() > 0.1  # The fingers move: not the measured hands.
        [first] = [unheld(c.args[3]) for c in submit.call_args_list]
        assert np.array_equal(hands_of(first), padded(kplan.hands))  # Its own hands, row for row.
        lengths.append(len(first))
        steps = np.abs(np.diff(sent_hands(peer), axis=0)).max(axis=1)
        assert steps.max() <= PEAK_SPEED / RATE + 1e-6  # The session's hand_step_limit.
        assert steps.max() > 0.5 * PEAK_SPEED / RATE  # The hands did move on the wire.
    # The fist's longer finger travel to the heart stretches 222's finger segments.
    assert lengths[0] > lengths[1], lengths
    # Each robot planned with its own measured hands and the imported timing.
    calls = plan_calls()
    spec = GESTURES["two_hand_hearts"]
    assert {(m, r) for _, m, r in calls} == {(IMPORTED_MOVE_S, spec.return_min_s)}, calls
    assert spec.return_min_s > 0.1  # Not the default way home.
    for measured in (FIST, UNWIRED):
        assert any(np.array_equal(h, measured) for h, _, _ in calls), measured
    for arm in module._arms():
        assert task(arm)["handback_confirmed"], task(arm)


def test_both_imported_gesture_refuses_an_arm_against_arm_contact(hands):
    module, executions = hands
    first, second = module._arms()
    second._both_checker = FakeChecker("left_elbow_link (other arm)")
    with pytest.raises(
        RuntimeError,
        match=r"^223: the planned arm path comes within 2 cm of the left_elbow_link \(other arm\); "
        r"nothing sent; nothing sent to any robot$",
    ):
        module.perform_gesture("both", "two_hand_hearts")
    assert first._both_checker.calls >= 1 and second._both_checker.calls >= 1
    assert first._checker.calls == second._checker.calls == 0  # Not the left-arm check.
    assert all(no_pose(peer) for _, peer in executions)
    assert all(task(arm) is None for arm in module._arms())


def test_dual_start_builds_the_both_arm_checker_for_each_robot(dual_executions, mocker):
    checker = mocker.patch.object(sonic_dual_arm_skill, "ArmCollisionChecker")
    mocker.patch.object(sonic_dual_arm_skill, "SonicEncoder", FakeEncoder)
    module = DualSonicArmSkillContainer(
        encoder_path="unused.onnx", collision_model_path="scene.xml"
    )
    (first, _), (second, _) = dual_executions
    mocker.patch.object(module, "_execution_222", first, create=True)
    mocker.patch.object(module, "_execution_223", second, create=True)
    transports = [make_transport(f"/dual-arm-test/{uuid4().hex}", SonicFeedback) for _ in range(2)]
    for name, transport in zip(("feedback_222", "feedback_223"), transports, strict=True):
        module.set_transport(name, transport)
        transport.start()
    try:
        module.start()
        scene = Path("scene.xml")
        assert checker.call_args_list == [
            mocker.call(scene),
            mocker.call(scene, arms=("left", "right")),
        ]
        for arm in module._arms():
            assert arm._both_checker is checker.return_value
    finally:
        module.stop()
        for transport in transports:
            transport.stop()


def test_imported_gestures_without_the_both_arm_check_are_refused(real_launch_skill, mocker):
    module, executions = real_launch_skill
    start_tool = mocker.spy(module, "start_tool")
    with pytest.raises(RuntimeError, match="both-arm path check"):
        module.perform_gesture("both", "support_person")
    assert start_tool.call_count == 0 and module._running is None
    assert all(no_pose(peer) for _, peer in executions)


@pytest.mark.parametrize("refusing", ["222", "223"])
def test_a_resolved_limit_refusal_on_one_robot_starts_neither(hands, mocker, refusing):
    module, executions = hands
    execution = module._by_robot[refusing]._execution
    real = execution.get_feedback

    def bent():
        state = real()
        q = list(state.body_q)
        q[18] = -0.8  # left elbow; support_person adds -0.5, past the -1.0472 limit
        return replace(state, body_q=tuple(q))

    mocker.patch.object(execution, "get_feedback", bent)
    with pytest.raises(
        RuntimeError,
        match=rf"^{refusing}: support_person keyframes\[1\]: left_elbow = -1\.3 outside its "
        r"limit .*; nothing sent to any robot$",
    ):
        module.perform_gesture("both", "support_person")
    assert module._by_robot[refusing]._both_checker.calls == 0  # Refused before the path check.
    assert all(no_pose(peer) for _, peer in executions)
    assert all(task(arm) is None for arm in module._arms())


@pytest.mark.parametrize("how", ["fault on 222", "stop on 223"])
def test_one_robots_fault_or_stop_retreats_both_with_their_hands(hands, mocker, how):
    module, executions = hands
    submits = [mocker.spy(execution, "submit_chunk") for execution, _ in executions]
    arm_223 = module._by_robot["223"]

    def knee(s):  # Mid first segment on both robots, while their fingers move.
        return {3: 0.2 if s > 0.6 else 0.0}

    def stop(s):
        if s > 0.6:
            arm_223.stop.set()
        return {}

    extra = (knee, None) if how == "fault on 222" else (None, stop)
    with feeding_arms(module._arms(), shape=zero, extra=extra):
        result = module.perform_gesture("both", "two_hand_hearts")
    first, second = result.message.split("; 223: ")
    if how == "fault on 222":
        assert "stopped early: left_knee" in first and "stopped early: 222: left_knee" in second
    else:
        assert "stopped early: 223: stop requested" in first, first
        assert "stopped early: stop requested" in second, second
    for (_, peer), submit, measured in zip(executions, submits, (FIST, UNWIRED), strict=True):
        kplan = planned("two_hand_hearts", measured)
        plan, retreat = (unheld(c.args[3]) for c in submit.call_args_list)
        moving = np.any(np.diff(hands_of(plan), axis=0) != 0, axis=1)
        # The way back starts at a plan row where the fingers were moving, not at a jump...
        starts = np.flatnonzero(np.all(hands_of(plan)[1:] == hands_of(retreat)[0], axis=1))
        assert len(starts) and moving[starts].any(), starts
        # ...and is the plan's own way home for the hands (retreat_hands), ending at the start.
        units = [padded(retreat_hands(kplan, k)) for k in range(len(kplan.segments))]
        assert any(
            len(retreat) <= len(u) and np.array_equal(hands_of(retreat), u[len(u) - len(retreat) :])
            for u in units
        )
        assert np.array_equal(hands_of(retreat)[-1], measured)
        steps = np.abs(np.diff(sent_hands(peer), axis=0)).max(axis=1)
        assert steps.max() <= PEAK_SPEED / RATE + 1e-6
    for arm in module._arms():
        assert task(arm)["handback_confirmed"], task(arm)


def test_both_execution_profiles_limit_hand_steps():
    atoms = {a.name: a for a in unitree_g1_sonic_arm_dual.blueprints}
    for robot in ("222", "223"):
        profile = atoms[f"g1_{robot}"].kwargs["profile"]
        assert profile.hand_step_limit == pytest.approx(PEAK_SPEED / RATE), robot


def test_hands_are_configured_per_robot(real_launch_skill):
    parsed = BlueprintConfigParser(unitree_g1_sonic_arm_dual).parse(
        [
            "--dualsonicarmskillcontainer.encoder-path=unused.onnx",
            "--dualsonicarmskillcontainer.hands-mounted-222=true",
        ],
        environ={},
    )
    config = DualSonicArmSkillConfig(**parsed.module_kwargs("dualsonicarmskillcontainer"))
    assert (config.hands_mounted_222, config.hands_mounted_223) == (True, False)
    with pytest.raises(ValueError, match="hands_mounted_222 / hands_mounted_223"):
        DualSonicArmSkillConfig(encoder_path="unused.onnx", hands_mounted=True)
    module, _ = real_launch_skill  # Configured with hands_mounted_222=True.
    assert module._by_robot["222"].config.hands_mounted is True
    assert module._by_robot["223"].config.hands_mounted is False
    assert module.config.hands_mounted is False


def test_finger_check_runs_on_its_robot_only_and_delivers_the_hand_diagnostics(
    real_launch_skill, mocker
):
    module, executions = real_launch_skill
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", real_hold_plan)  # The real 3 s hold.
    submit = mocker.spy(module._execution_222, "submit_chunk")
    encode = mocker.spy(module._by_robot["222"]._encoder, "encode")
    with feeding_arms(module._arms(), shape=zero):
        called = time.monotonic()
        assert module.check_dex3_finger_order(222) == "started"  # The CLI's integer robot.
        assert time.monotonic() - called < 1.0
        assert module.operator_result("222") is None
        wait_until(lambda: module.operator_result("222") is not None, 30)
    hands = hands_of(submit.call_args_list[0].args[3])
    assert any(np.array_equal(h, FINGER_CHECK_HANDS) for h in hands)
    poses = encode.call_args_list[0].args[0]
    assert np.all(poses == poses[0])  # The body stays at its measured start.
    # The scripted peer's hands never move: the hold evidence says so, with the diagnostics.
    result = module.operator_result("222")
    assert '"message": "check_dex3_finger_order failed: ' in result, result
    assert "hand did not follow" in result, result
    for key in ("hand_base", "hand_target", "hand_held"):
        assert key in result, (key, result)
    assert module.operator_result("223") is None and no_pose(executions[1][1])
    assert module._running is None


def test_finger_check_stops_through_stop_arm(real_launch_skill, mocker):
    module, executions = real_launch_skill
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", real_hold_plan)
    submit = mocker.spy(module._execution_222, "submit_chunk")
    with feeding_arms(module._arms(), shape=zero):
        assert module.check_dex3_finger_order("222") == "started"
        wait_until(lambda: submit.call_count == 1)
        time.sleep(0.5)
        assert module.stop_arm().startswith("Stopping")
        wait_until(lambda: module.operator_result("222") is not None, 30)
    assert "stopped early: stop requested" in module.operator_result("222")
    assert [c.args[1] for c in submit.call_args_list] == [0, 1]  # The plan, then the retreat.
    assert task(module._by_robot["222"])["handback_confirmed"]
    assert no_pose(executions[1][1])


def test_finger_check_refuses_a_robot_without_hands_and_while_a_motion_runs(
    real_launch_skill, mocker
):
    module, executions = real_launch_skill
    with pytest.raises(
        RuntimeError, match=r"no hands configured on 223 \(hands_mounted_223 is False\)"
    ):
        module.check_dex3_finger_order("223")
    for robot in ("both", 224):
        with pytest.raises(ValueError, match="robot must be one of"):
            module.check_dex3_finger_order(robot)
    assert module._running is None and all(no_pose(peer) for _, peer in executions)
    gates, _ = gate_starts(module, mocker)
    assert "started" in module.move_upper_body("both", "left_arm_forward", 20.0)
    with pytest.raises(RuntimeError, match="an arm move is already running"):
        module.check_dex3_finger_order("222")
    for gate in gates:
        gate.set()
    wait_until(lambda: module._running is None)
    assert module.operator_result("222") is None and module.operator_result("223") is None


def test_finger_check_is_an_operator_rpc_not_a_skill(real_launch_skill):
    module, _ = real_launch_skill
    names = {s.func_name for s in module.get_skills()}
    assert "perform_gesture" in names and "stop_arm" in names
    assert not {"check_dex3_finger_order", "operator_result"} & names
    for rpc in (
        DualSonicArmSkillContainer.check_dex3_finger_order,
        DualSonicArmSkillContainer.operator_result,
    ):
        assert getattr(rpc, "__rpc__", False) and not getattr(rpc, "__skill__", False)


@pytest.fixture
def worker_dual(dimos_cluster, tmp_path):
    """Both execution modules and the dual container each in a worker, wired as the blueprint."""
    profile = replace(ARM_PROFILE, max_observation_age=25.0, max_duration=60.0)
    with scripted_peer() as first, scripted_peer() as second:
        peers = {"222": first, "223": second}
        executions = {}
        container = None
        try:
            for robot, peer in peers.items():
                executions[robot] = dimos_cluster.deploy(
                    SonicExecutionModule,
                    instance_name=f"g1_{robot}",
                    reference_bind="tcp://127.0.0.1:*",
                    feedback_endpoint=peer.feedback_endpoint,
                    journal_dir=tmp_path / robot,
                    profile=profile,
                )
            container = dimos_cluster.deploy(
                DualSonicArmSkillContainer,
                encoder_path=ENCODER,
                hands_mounted_222=True,
                return_watch_seconds=0.0,
            )
            for robot, execution in executions.items():
                transport = make_transport(f"/dual-arm-worker-test/{uuid4().hex}", SonicFeedback)
                execution.set_transport("feedback", transport)
                container.set_transport(f"feedback_{robot}", transport)
                container.set_module_ref(f"_execution_{robot}", execution)
                execution.start()
                peers[robot].reference_endpoint = execution.get_status()["reference_endpoint"]
                peers[robot].connected.set()
                wait_until(lambda e=execution: e.get_status()["ready"], timeout=3)
            container.start()
            yield container, executions, peers
        finally:
            if container is not None:
                container.stop()
            for execution in executions.values():
                execution.stop()


@pytest.mark.skipif(not ENCODER.is_file(), reason="GEAR_SONIC_ENCODER is unset or missing")
def test_finger_check_through_the_worker_returns_at_once_and_stop_arm_retreats(worker_dual):
    container, executions, peers = worker_dual
    called = time.monotonic()
    assert container.check_dex3_finger_order("222") == "started"
    assert time.monotonic() - called < 1.0
    assert container.operator_result("222") is None
    # The transmitted hands reached the finger pose: the plan is in the hold. Then a few more.
    wait_until(
        lambda: any(
            np.allclose(h, FINGER_CHECK_HANDS, atol=1e-3) for h in sent_hands(peers["222"])
        ),
        30,
    )
    arrived = len(sent_hands(peers["222"]))
    wait_until(lambda: len(sent_hands(peers["222"])) >= arrived + 10, 30)
    called = time.monotonic()
    assert container.stop_arm().startswith("Stopping")
    assert time.monotonic() - called < 1.0
    wait_until(lambda: container.operator_result("222") is not None, 30)
    result = container.operator_result("222")
    assert "stopped early: stop requested" in result, result
    status = executions["222"].get_status()["task"]
    assert status["last_chunk_sequence"] == 1 and status["handback_confirmed"], status
    assert no_pose(peers["223"]) and container.operator_result("223") is None


# Voice lines: perform_gesture's line plays over ssh on each robot with a voice host.

COMMAND = "python3 ~/g1_audio.py"
ONLY_223 = {"voice_host_223": "pc2", "voice_command": COMMAND}  # The demo: 222 has no wavs yet.
BOTH_VOICES = {"voice_host_222": "pc2_222", **ONLY_223}


def voices(module, mocker, code=0):
    """Scripts sonic_arm_skill's ssh; returns one log of the ssh calls and the robots' task calls.

    Entries: ("play" | "stop", host, remote command), ("kill", host), ("submit_chunk", robot,
    sequence) once the chunk is sent, ("finish_task", robot, None). Each play's process polls as
    `code` (None: still playing) until killed; an exception `code` is raised by Popen instead.
    """
    log = []
    shell = mocker.patch.object(sonic_arm_skill, "subprocess")

    def popen(argv, **kwargs):
        if isinstance(code, BaseException):
            raise code
        assert argv[0] == "ssh" and "BatchMode=yes" in argv
        host, command = argv[-2:]
        process = mocker.MagicMock()
        process.poll.return_value = code
        process.stderr.read.return_value = b"aplay: no such file"

        def kill():
            log.append(("kill", host))
            process.poll.return_value = -9

        process.kill.side_effect = kill
        log.append((command.removeprefix(f"{COMMAND} ").split()[0], host, command))
        return process

    shell.Popen.side_effect = popen
    for robot, arm in module._by_robot.items():
        for method in ("submit_chunk", "finish_task"):
            real = getattr(arm._execution, method)

            def record(*args, robot=robot, method=method, real=real):
                reply = real(*args)
                log.append((method, robot, args[1] if method == "submit_chunk" else None))
                return reply

            mocker.patch.object(arm._execution, method, side_effect=record)
    return log


def played(log):
    return [entry for entry in log if entry[0] == "play"]


@pytest.mark.parametrize("real_launch_skill", [ONLY_223], indirect=True)
def test_a_line_on_223_plays_there_once_its_first_chunk_is_sent(inline, mocker):
    module, executions = inline
    log = voices(module, mocker)
    with feeding_arms(module._arms()):
        result = module.perform_gesture("223", "wave", line="01_intro")
    assert "wave reached" in result.message, result
    assert played(log) == [("play", "pc2", f"{COMMAND} play ~/lines/01_intro.wav 60")]
    assert log.index(played(log)[0]) == log.index(("submit_chunk", "223", 0)) + 1
    assert result.metadata["voice"] == "01_intro: played"
    assert no_pose(executions[0][1])


@pytest.mark.parametrize(
    ("real_launch_skill", "hosts"),
    [(ONLY_223, {"223": "pc2"}), (BOTH_VOICES, {"222": "pc2_222", "223": "pc2"})],
    indirect=["real_launch_skill"],
)
def test_both_with_a_line_speaks_on_each_robot_with_a_voice_after_its_own_first_chunk(
    inline, mocker, hosts
):
    module, _ = inline
    log = voices(module, mocker)
    with feeding_arms(module._arms()):
        result = module.perform_gesture("both", "wave", line="01_intro")
    assert "222: wave reached" in result.message and "; 223: wave reached" in result.message
    assert sorted(host for _, host, _ in played(log)) == sorted(hosts.values())
    for robot in _ROBOTS:
        metadata = result.metadata[robot]
        if robot not in hosts:
            assert "voice" not in metadata, metadata
            continue
        play = next(entry for entry in played(log) if entry[1] == hosts[robot])
        assert log.index(play) > log.index(("submit_chunk", robot, 0)), log
        assert metadata["voice"] == "01_intro: played"


@pytest.mark.parametrize(
    ("real_launch_skill", "robot", "refusal"),
    [
        (ONLY_223, "222", "voice is not configured on 222 (voice_host_222, voice_command)"),
        ({"voice_host_223": "pc2"}, "223", "on 223 (voice_host_223, voice_command)"),
        ({}, "both", "on either robot (voice_host_222 / voice_host_223, voice_command)"),
        ({"voice_host_222": "pc2_222", "voice_host_223": "pc2"}, "both", "on either robot"),
        (ONLY_223, "223", "line must be a wav name"),
    ],
    indirect=["real_launch_skill"],
)
def test_a_line_no_selected_robot_can_speak_is_refused_with_nothing_sent(
    inline, mocker, robot, refusal
):
    module, executions = inline
    log = voices(module, mocker)
    line = "../x; reboot" if "wav name" in refusal else "01_intro"
    with pytest.raises(ValueError, match=re.escape(refusal)):
        module.perform_gesture(robot, "wave", line=line)
    assert log == [] and all(no_pose(peer) for _, peer in executions)
    assert all(task(arm) is None for arm in module._arms())


@pytest.mark.parametrize("real_launch_skill", [BOTH_VOICES], indirect=True)
def test_stop_arm_cuts_every_robots_line_at_once(real_launch_skill, mocker):
    module, _ = real_launch_skill
    log = voices(module, mocker, code=None)
    updates = mocker.spy(module, "tool_update")
    with feeding_arms(module._arms()):
        module.perform_gesture("both", "wave", line="01_intro")
        wait_until(lambda: len(played(log)) == 2)
        assert module.stop_arm().startswith("Stopping")
        at_stop = list(log)  # stop_arm itself cut both lines, not each robot's retreat later.
        wait_until(lambda: updates.call_count == 1)
    for host in ("pc2_222", "pc2"):
        assert ("kill", host) in at_stop and ("stop", host, f"{COMMAND} stop") in at_stop
        assert log.count(("kill", host)) == 1
    update = json.loads(updates.call_args.args[1])
    assert update["message"].count("stop requested") == 2, update


@pytest.mark.parametrize("real_launch_skill", [ONLY_223], indirect=True)
def test_the_other_robots_fault_cuts_the_line_as_this_robot_retreats(inline, mocker):
    module, _ = inline
    log = voices(module, mocker, code=None)

    def knee(s):
        return {3: 0.2 if s > 0.6 else 0.0}

    with feeding_arms(module._arms(), shape=zero, extra=(knee, None)):
        result = module.perform_gesture("both", "wave", line="01_intro")
    assert "stopped early: 222: left_knee" in result.message.split("; 223: ")[1], result
    # Cut right after 223's way back is sent, not only once its motion ends.
    retreat, kill = log.index(("submit_chunk", "223", 1)), log.index(("kill", "pc2"))
    assert retreat < kill < log.index(("finish_task", "223", None)), log
    assert ("stop", "pc2", f"{COMMAND} stop") in log
    assert result.metadata["223"]["voice"] == "01_intro: cut off"


@pytest.mark.parametrize("real_launch_skill", [ONLY_223], indirect=True)
def test_module_stop_cuts_the_line(real_launch_skill, mocker):
    module, _ = real_launch_skill
    log = voices(module, mocker, code=None)
    with feeding_arms(module._arms()):
        module.perform_gesture("223", "wave", line="01_intro")
        wait_until(lambda: played(log))
        module.stop()
    assert log.count(("kill", "pc2")) == 1 and ("stop", "pc2", f"{COMMAND} stop") in log


@pytest.mark.parametrize(
    ("real_launch_skill", "code", "report"),
    [
        (ONLY_223, 1, "01_intro: failed (exit 1) aplay: no such file"),
        (ONLY_223, OSError("no ssh"), "01_intro: not played"),
    ],
    indirect=["real_launch_skill"],
)
def test_a_failed_line_is_reported_and_never_fails_the_motion(inline, mocker, code, report):
    module, _ = inline
    voices(module, mocker, code=code)
    with feeding_arms(module._arms()):
        result = module.perform_gesture("both", "wave", line="01_intro")
    assert "222: wave reached" in result.message and "; 223: wave reached" in result.message
    assert result.metadata["223"]["voice"] == report


@pytest.mark.parametrize("real_launch_skill", [ONLY_223], indirect=True)
def test_voice_hosts_are_configured_per_robot(real_launch_skill):
    parsed = BlueprintConfigParser(unitree_g1_sonic_arm_dual).parse(
        [
            "--dualsonicarmskillcontainer.encoder-path=unused.onnx",
            "--dualsonicarmskillcontainer.voice-host-223=pc2",
            "--dualsonicarmskillcontainer.voice-command=~/venv/bin/python ~/g1_audio.py",
            "--dualsonicarmskillcontainer.voice-volume=80",
        ],
        environ={},
    )
    config = DualSonicArmSkillConfig(**parsed.module_kwargs("dualsonicarmskillcontainer"))
    assert (config.voice_host_222, config.voice_host_223) == (None, "pc2")
    assert (config.voice_command, config.voice_volume) == ("~/venv/bin/python ~/g1_audio.py", 80)
    with pytest.raises(ValueError, match="voice_host_222 / voice_host_223"):
        DualSonicArmSkillConfig(encoder_path="unused.onnx", voice_host="pc2")
    module, _ = real_launch_skill  # Configured with ONLY_223.
    arms = module._by_robot
    assert (arms["222"].config.voice_host, arms["223"].config.voice_host) == (None, "pc2")
    assert (arms["222"].speaks, arms["223"].speaks) == (False, True)
    assert module.config.voice_host is None
