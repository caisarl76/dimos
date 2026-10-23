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

"""The arm skills end to end against the MuJoCo stand-in for the guarded SONIC controller.

The stand-in (sonic_sim_peer) runs as a subprocess in real time. The execution module and the skill
container run in-process with the robot blueprint's profile, the real encoder, the real background
`_launch` and max_dq 2.0. Each case's physics is checked from the stand-in's npz once it exits.

    GEAR_SONIC_MODEL_DIR=<HF GEAR-SONIC snapshot> GEAR_SONIC_SCENE=<gear_sonic>/.../scene_43dof.xml \\
        pytest -m native_e2e dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py -v -s
"""

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any
from uuid import uuid4

import numpy as np
import pytest

from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1 import sonic_arm_skill
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm import SONIC_ARM_PROFILE
from dimos.robot.unitree.g1.sonic_arm_reference import (
    LEFT_ARM,
    RATE,
    keyframe_plan,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    _CHUNK_OVERLAP,
    _MAX_CHUNK_ROWS,
    GESTURES,
    SonicArmSkillContainer,
    gesture_keyframes,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_test_support import FakeVl, IMAGE_640x480
from dimos.utils.testing.waiting import wait_until

MODEL_DIR = Path(os.environ.get("GEAR_SONIC_MODEL_DIR", "/nonexistent"))
SCENE = Path(os.environ.get("GEAR_SONIC_SCENE", "/nonexistent"))
ENCODER = MODEL_DIR / "sonic_v1_1/model_encoder.onnx"

pytestmark = [
    pytest.mark.native_e2e,
    pytest.mark.skipif(
        not (ENCODER.is_file() and SCENE.is_file()),
        reason="GEAR_SONIC_MODEL_DIR / GEAR_SONIC_SCENE unset or missing",
    ),
]

MAX_DQ = 2.0
MAX_TILT_DEG = 3.0
# Ruling R12: straight-ahead pointing is R6's must-succeed direction, and its peak tilt measured
# 2.75-3.0 deg over 7 fresh runs (one at 3.003). Every other case keeps 3 deg.
CASE_MAX_TILT_DEG = {"point_at(0, 0)": 3.5}
MIN_CONTACTS = 7
ARM_RETURN_RAD = 0.05
RESULT_TIMEOUT_S = 120.0
STOP_FRACTION = 0.6
HANDBACK = "control returned to SONIC's planner"

CASES: list[tuple[str, str, dict[str, Any]]] = [
    *[(f"gesture-{name}", "perform_gesture", {"name": name}) for name in GESTURES],
    *[
        (f"point_at({az}, {el})", "point_at", {"azimuth_deg": az, "elevation_deg": el})
        # (85, 20), not (90, 20): that is near the reach limit (22.5 deg achieved, (90, 15) is
        # refused) and the right knee reached the 0.15 envelope in all 4 runs from a fresh start;
        # (80, 20), (85, 20) and (90, 30) stayed clear.
        for az, el in ((0, 0), (45, -30), (85, 20))
    ],
    # Side pose, not overhead: in this sim overhead plus a long hold sits at the 0.15 leg envelope.
    (
        "pose-side-hold-10",
        "move_left_arm_pose",
        {"pose": "left_arm_side_horizontal", "hold_seconds": 10.0},
    ),
]
assert set(CASE_MAX_TILT_DEG) <= {name for name, _, _ in CASES}

# point_at_object with a fake vision model: 40x40 px boxes centred at these head-camera pixels.
OBJECT_CENTRES = {"centre": (320, 240), "right-edge": (620, 240)}
# Bottom right: every pose that aims at its floor point comes near the left hip from the
# stand-in's stance, so the skill must refuse it without moving.
REFUSED_CENTRE = (560, 450)
REFUSED_ARM_RAD = 0.02
OBJECT_HOLD_S = 3.0
MAX_PLANNED_AIM_DEG = 3.0
MAX_HELD_AIM_DEG = 8.0  # Provisional MuJoCo bar from the design's Validation section.
AIM_AGREEMENT_DEG = 2.0
HOLD_MARGIN_ROWS = 25  # The skill's hold window: 0.5 s after arrival to 0.5 s before departure.


@dataclass
class Window:
    name: str
    start: float
    end: float
    result: dict[str, Any]
    stop_at: float | None = None
    planned_end: float | None = None
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class Sim:
    process: subprocess.Popen[str]
    log: Path
    lines: list[str]
    execution: SonicExecutionModule
    skill: SonicArmSkillContainer


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port: int = s.getsockname()[1]
        return port


def _stop_peer(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=10)


@contextmanager
def stand_in(tmp: Path) -> Iterator[Sim]:
    """A fresh stand-in standing from its deterministic startup, with the execution module and the
    skill built against it. The stand-in has exited (and written its npz) when this returns.

    One stand-in per case: after each motion the planner-idle stance settles somewhere slightly
    different (hip pitch up to ~0.08 rad), and the leg envelope is measured from the stance at the
    call, so in one long session each case's outcome would depend on the cases before it.
    """
    feedback, reference = (f"tcp://127.0.0.1:{_free_port()}" for _ in range(2))
    log = tmp / "sim.npz"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "dimos.robot.unitree.g1.sonic_sim_peer",
            "--feedback",
            feedback,
            "--reference",
            reference,
            "--model-dir",
            str(MODEL_DIR),
            "--scene",
            str(SCENE),
            "--name",
            "e2e",
            "--log",
            str(log),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    lines: list[str] = []
    standing = threading.Event()

    def read() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            lines.append(line.rstrip())
            if "standing:" in line:
                standing.set()

    threading.Thread(target=read, daemon=True).start()
    execution = skill = transport = None
    try:
        deadline = time.monotonic() + 120
        while not standing.wait(0.2):
            assert process.poll() is None, "\n".join(lines)
            assert time.monotonic() < deadline, "the stand-in never stood:\n" + "\n".join(lines)
        execution = SonicExecutionModule(
            reference_bind=reference,
            feedback_endpoint=feedback,
            journal_dir=tmp / "journal",
            profile=SONIC_ARM_PROFILE,
        )
        skill = SonicArmSkillContainer(
            encoder_path=ENCODER, max_dq=MAX_DQ, collision_model_path=SCENE
        )
        skill._execution = execution  # type: ignore[assignment]  # what the blueprint injects
        transport = make_transport(f"/arm-skill-e2e/{uuid4().hex}", SonicFeedback)
        execution.set_transport("feedback", transport)
        skill.set_transport("feedback", transport)
        transport.start()
        execution.start()
        skill.start()
        wait_until(lambda: execution.get_status()["ready"], timeout=30)
        yield Sim(process, log, lines, execution, skill)
    finally:
        for module in (skill, execution):
            if module is not None:
                module.stop()
        if transport is not None:
            transport.stop()
        _stop_peer(process)
    assert process.returncode == 0, "\n".join(lines)


def _run(
    sim: Sim, mocker: Any, name: str, tool: str, kwargs: dict[str, Any], stop: bool = False
) -> Window:
    """Call the skill, optionally stop it STOP_FRACTION through its plan, and wait for its result."""
    skill, execution = sim.skill, sim.execution
    updates = mocker.spy(skill, "tool_update")
    submit = mocker.spy(execution, "submit_chunk")
    feedback = execution.get_feedback()
    assert feedback is not None
    base = np.asarray(feedback.body_q, np.float64)
    window = Window(name, time.monotonic(), 0.0, {})
    assert "started" in getattr(skill, tool)(**kwargs)
    if stop:
        wait_until(lambda: submit.call_count >= 1, timeout=60)
        plan_start = submit.call_args_list[0].args[2]
        pad = 100  # token_schedule's 2 s blend at each end
        kplan = keyframe_plan(
            base,
            gesture_keyframes(kwargs["name"], base),
            split_every=_MAX_CHUNK_ROWS - _CHUNK_OVERLAP - pad,
        )
        seconds = (len(kplan.poses) + 2 * pad) / RATE
        window.planned_end = plan_start + seconds
        time.sleep(max(0.0, plan_start + STOP_FRACTION * seconds - time.monotonic()))
        window.stop_at = time.monotonic()
        window.extra["stop_reply"] = skill.stop_arm()
    wait_until(lambda: updates.call_count >= 1, timeout=RESULT_TIMEOUT_S)
    wait_until(lambda: skill._running is None, timeout=10)
    window.end = time.monotonic()
    tool_name, text = updates.call_args.args
    assert tool_name == tool
    try:
        window.result = json.loads(text)
    except json.JSONDecodeError:
        window.result = {"message": text}
    return window


def _physics(sim: Sim, w: Window) -> list[str]:
    """Print the case's row and return its physics failures: upright, feet down, no fast joint,
    and the left arm back where it started (before the planned end, for a stopped case)."""
    log = np.load(sim.log)
    t, q, dq = log["t"], log["q"], np.abs(log["dq"])
    rows = np.flatnonzero((t >= w.start) & (t <= w.end))
    assert len(rows) > 10, w.name
    arm = np.abs(q[rows][:, LEFT_ARM] - q[rows[0], LEFT_ARM]).max(axis=1)
    meta = w.result.get("metadata", {})
    message = w.result["message"]
    outcome = (
        "stopped"
        if "stopped early" in message
        else "ok"
        if message.endswith(HANDBACK)
        else "failed"
    )
    peak, tilt = float(dq[rows].max()), float(log["tilt"][rows].max())
    contacts = int(log["contacts"][rows].min())
    legs = meta.get("max_deviation_rad", {}).get("legs", float("nan"))
    print(
        f"\n{'case':<20}{'outcome':>9}{'reached':>9}{'peak dq':>9}{'tilt':>7}{'contacts':>9}"
        f"{'legs':>7}{'s':>7}\n{w.name:<20}{outcome:>9}{meta.get('reached_fraction', float('nan')):>9.2f}"
        f"{peak:>9.2f}{tilt:>7.2f}{contacts:>9d}{legs:>7.3f}{w.end - w.start:>7.1f}\n{message}"
    )
    checks = {
        f"tilt {tilt:.2f} deg": tilt <= CASE_MAX_TILT_DEG.get(w.name, MAX_TILT_DEG),
        f"floor contacts {contacts}": contacts >= MIN_CONTACTS,
        f"peak dq {peak:.2f} rad/s": peak <= MAX_DQ,
        f"left arm ends {arm[-1]:.3f} rad from its start": arm[-1] <= ARM_RETURN_RAD,
    }
    if w.stop_at is not None:
        assert w.planned_end is not None
        away = np.flatnonzero(arm > ARM_RETURN_RAD)
        back = t[rows[away[-1] + 1]] if len(away) and away[-1] + 1 < len(rows) else t[rows[0]]
        checks[f"arm back {back - w.planned_end:+.1f} s from the planned end"] = (
            back < w.planned_end
        )
    return [k for k, ok in checks.items() if not ok]


@pytest.mark.parametrize("name,tool,kwargs", CASES, ids=[c[0] for c in CASES])
def test_skill_completes_and_hands_back(
    tmp_path: Path, tool_transport: Any, mocker: Any, name: str, tool: str, kwargs: dict[str, Any]
) -> None:
    with stand_in(tmp_path) as sim:
        window = _run(sim, mocker, name, tool, kwargs)
    failures = _physics(sim, window)
    message = window.result["message"]
    assert "stopped early" not in message and "failed" not in message, message
    assert message.endswith(HANDBACK), message
    assert window.result["metadata"]["reached_fraction"] >= 0.8, window.result
    assert not failures, failures


def test_stop_arm_mid_wave_retreats_before_the_planned_end(
    tmp_path: Path, tool_transport: Any, mocker: Any
) -> None:
    with stand_in(tmp_path) as sim:
        window = _run(sim, mocker, "stop-mid-wave", "perform_gesture", {"name": "wave"}, stop=True)
    failures = _physics(sim, window)
    assert window.extra["stop_reply"].startswith("Stopping"), window.extra["stop_reply"]
    message = window.result["message"]
    assert "stopped early: stop requested" in message and message.endswith(HANDBACK), message
    assert window.stop_at is not None
    journal = Path(window.result["metadata"]["journal_path"]).read_text().splitlines()
    chunks = [e for e in map(json.loads, journal) if e["event"] == "chunk_accepted"]
    retreats = [e for e in chunks if e["sequence"] >= 1 and e["monotonic"] >= window.stop_at]
    assert len(retreats) == 1, [(e["sequence"], e["monotonic"]) for e in chunks]
    assert not failures, failures


@pytest.mark.parametrize("where", OBJECT_CENTRES)
def test_point_at_object_holds_the_forearm_on_target(
    tmp_path: Path, tool_transport: Any, mocker: Any, where: str
) -> None:
    u, v = OBJECT_CENTRES[where]
    with stand_in(tmp_path) as sim:
        sim.skill._vl = FakeVl([["box", u - 20, v - 20, u + 20, v + 20]])  # type: ignore[assignment]
        mocker.patch.object(sim.skill.color_image, "get_next", return_value=IMAGE_640x480)
        plans = mocker.spy(sonic_arm_skill, "keyframe_plan")
        window = _run(
            sim,
            mocker,
            f"object-{where}",
            "point_at_object",
            {"description": "box", "hold_seconds": OBJECT_HOLD_S},
        )
    failures = _physics(sim, window)
    message = window.result["message"]
    assert "stopped early" not in message and "failed" not in message, message
    assert "did not follow" not in message, message
    meta = window.result["metadata"]

    # The aim metric first, so it is recorded even when a standard assert below fails.
    # The hold window in the stand-in's clock: the same rows as the skill's own window, from the
    # same plan (the one keyframe_plan call _move makes) and the same pad, offset by the first
    # chunk's row-0 time. That is the observed_at _stream submits, on this host's monotonic clock,
    # which the stand-in's npz t also uses.
    assert plans.call_count == 1, plans.call_args_list
    kplan = plans.spy_return
    pad = (len(token_schedule(np.zeros(64), np.zeros((1, 64)))) - 1) // 2
    seg, nxt = kplan.segments[0], kplan.segments[1]
    hold = (pad + seg.end - 1 + HOLD_MARGIN_ROWS, pad + nxt.start - HOLD_MARGIN_ROWS)
    journal = Path(meta["journal_path"]).read_text().splitlines()
    first = next(e for e in map(json.loads, journal) if e["event"] == "chunk_accepted")
    assert first["sequence"] == 0, first
    plan_start = float(first["observed_at"])
    log = np.load(sim.log)
    t, q = log["t"], log["q"]
    held = q[(t >= plan_start + hold[0] / RATE) & (t <= plan_start + hold[1] / RATE)]
    assert sim.skill._kinematics is not None
    npz_error = float(
        np.median([sim.skill._kinematics.forearm_error(s, meta["aim_point_m"]) for s in held])
    )
    print(
        f"{where}: box centre ({u}, {v}), aim point {meta['aim_point_m']} m, pose {meta['pose_deg']}"
        f" deg; aim error planned {meta['planned_aim_error_deg']} deg, measured (report) "
        f"{meta['measured_aim_error_deg']} deg over {meta['hold_samples']} samples, measured (npz) "
        f"{npz_error:.2f} deg over {len(held)} samples; row 0 accepted "
        f"{(first['monotonic'] - plan_start) * 1000:.1f} ms after observed_at"
    )
    assert meta["planned_aim_error_deg"] <= MAX_PLANNED_AIM_DEG, meta
    assert meta["hold_samples"] > 0, meta
    assert len(held) > 0, (plan_start, hold)
    assert npz_error <= MAX_HELD_AIM_DEG, (npz_error, meta)
    assert abs(npz_error - meta["measured_aim_error_deg"]) <= AIM_AGREEMENT_DEG, (npz_error, meta)
    assert message.endswith(HANDBACK), message
    # Ruling R1: no reached >= 0.8 bar in forearm mode; SONIC sags on the elbow (centre: planned
    # 60.4 deg, reached 50.0 deg) while the aim held at 6.4 deg. Reached is printed, not asserted.
    print(f"{where}: reached {meta['reached_fraction']:.2f} (recorded, not asserted)")
    assert not failures, failures


def test_point_at_object_refused_near_hip_sends_nothing(
    tmp_path: Path, tool_transport: Any, mocker: Any
) -> None:
    u, v = REFUSED_CENTRE
    with stand_in(tmp_path) as sim:
        sim.skill._vl = FakeVl([["box", u - 20, v - 20, u + 20, v + 20]])  # type: ignore[assignment]
        mocker.patch.object(sim.skill.color_image, "get_next", return_value=IMAGE_640x480)
        # No task means no chunk: submit_chunk needs a task id. (_run already spies submit_chunk,
        # and an RPC method cannot be spied twice.)
        tasks = mocker.spy(sim.execution, "start_task")
        window = _run(
            sim,
            mocker,
            "refused-near-hip",
            "point_at_object",
            {"description": "box", "hold_seconds": OBJECT_HOLD_S},
        )
    failures = _physics(sim, window)
    message = window.result["message"]
    assert message.startswith("point_at_object failed") and "comes near the" in message, message
    assert tasks.call_count == 0, tasks.call_args_list
    log = np.load(sim.log)
    q = log["q"][(log["t"] >= window.start) & (log["t"] <= window.end)][:, LEFT_ARM]
    drift = float(np.abs(q - q[0]).max())
    print(f"refused-near-hip: left arm drift {drift:.4f} rad over {len(q)} samples")
    assert drift <= REFUSED_ARM_RAD, drift
    assert not failures, failures
