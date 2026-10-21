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

from functools import partial
import json
import re
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest

from dimos.agents.mcp.tool_stream import TOOL_STREAM_STOPPED_METHOD
from dimos.core.global_config import GlobalConfig
from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_skill, sonic_dual_arm_skill
from dimos.robot.unitree.g1.blueprints.agentic.unitree_g1_sonic_arm_dual_agentic import (
    unitree_g1_sonic_arm_dual_agentic,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm_dual import (
    unitree_g1_sonic_arm_dual,
)
from dimos.robot.unitree.g1.sonic_arm_reference import RATE, retreat_poses, token_schedule
from dimos.robot.unitree.g1.sonic_arm_skill import gesture_keyframes
from dimos.robot.unitree.g1.sonic_dual_arm_skill import DualSonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_DUAL_ARM_SYSTEM_PROMPT
from dimos.robot.unitree.g1.test_sonic_arm_group import (
    feeding,
    no_pose,
    task,
)
from dimos.robot.unitree.g1.test_sonic_arm_skill import (
    FAKE_GAIN,
    PEER_TOKEN,
    FakeEncoder,
    fast_keyframe_plan,
    frames,
    gate_start,
    mcp as mcp,
    tokens,
    tool_transport as tool_transport,
    wait_until,
)


class AbsoluteEncoder(FakeEncoder):
    """Tokens that depend on the pose alone, so a retreat unit starting mid-motion joins the plan."""

    def encode(self, poses):
        tokens = np.full((len(poses), 64), PEER_TOKEN)
        tokens[:, 0] += FAKE_GAIN * poses.sum(axis=1)
        return tokens


@pytest.fixture
def real_launch_skill(dual_executions, tool_transport, mocker):
    """The dual container started on both executions; yields (module, [(execution, peer)])."""
    mocker.patch.object(sonic_dual_arm_skill, "SonicEncoder", AbsoluteEncoder)
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", fast_keyframe_plan)
    mocker.patch.object(sonic_arm_skill, "token_schedule", partial(token_schedule, blend_s=0.2))
    module = DualSonicArmSkillContainer(encoder_path="unused.onnx", return_watch_seconds=0.0)
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


def test_dual_prompt_names_only_dual_skills():
    named = set(re.findall(r"`(\w+)\(", G1_SONIC_DUAL_ARM_SYSTEM_PROMPT))
    skills = {
        k for k, v in vars(DualSonicArmSkillContainer).items() if getattr(v, "__skill__", False)
    }
    assert named and named <= skills, named - skills


def test_single_robot_command_ignores_the_other_robot(inline):
    module, executions = inline
    peer = executions[1][1]
    peer.pause_feedback.set()
    try:
        wait_until(lambda: module._execution_223.get_feedback() is None)
        with feeding(module._arms()):
            result = module.move_upper_body("222", "left_arm_forward", 20.0)
    finally:
        peer.pause_feedback.clear()
    assert result.message.startswith("left_arm_forward reached"), result
    assert "returned" in result.message and "223" not in result.metadata
    assert no_pose(peer)


def test_gesture_both_through_the_container(inline):
    module, _ = inline
    with feeding(module._arms()):
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
    row *= 1.1  # A blend with the stream already in flight adds a few percent.
    submit = mocker.spy(module._execution_223, "submit_chunk")

    def knee(s):
        return {3: 0.2 if s > stop_s else 0.0}

    with feeding(module._arms(), shape=zero, extra=(knee, None)):
        result = module.perform_gesture("both", "wave")
    assert "stopped early" in result.message and "left_knee" in result.message, result
    assert "stopped early: 222: left_knee" in result.message.split("; 223: ")[1], result
    first, retreat = submit.call_args_list[0].args[3], submit.call_args_list[-1].args[3]
    assert submit.call_count == 2 and len(retreat) < len(first) / 2
    for _, peer in executions:
        wait_until(
            lambda p=peer: tokens(p.snapshot())[-1, 0] == pytest.approx(PEER_TOKEN, abs=1e-3)
        )
        streamed = tokens(peer.snapshot())[:, 0]
        assert streamed[-1] == pytest.approx(PEER_TOKEN, abs=1e-3)
        assert np.abs(np.diff(streamed)).max() <= row
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
    with feeding(module._arms()):
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


def test_dual_blueprints_set_only_known_global_config_fields() -> None:
    for bp in (unitree_g1_sonic_arm_dual, unitree_g1_sonic_arm_dual_agentic):
        assert set(bp.global_config_overrides) <= set(GlobalConfig.model_fields)
