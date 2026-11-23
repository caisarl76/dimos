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

from collections.abc import Callable, Iterator
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
from dimos.robot.unitree.g1.imported_gestures import (
    AUTHORED,
    CLIP_GESTURES,
    RETAINED,
    WAIST_GESTURES,
)
from dimos.robot.unitree.g1.sonic_arm_reference import (
    LEFT_ARM,
    MOTOR_NAMES,
    RATE,
    RIGHT_ARM,
    group_margins,
    token_schedule,
)
from dimos.robot.unitree.g1.sonic_arm_skill import (
    ARM_MOTORS,
    CLIP_DQ_CEILING,
    CLIP_DQ_FACTOR,
    FINGER_CHECK,
    LEFT_ARM_GESTURES,
    SonicArmSkillContainer,
    gesture_plan_inputs,
)
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_test_support import FakeVl, IMAGE_640x480, plan_as_prepared
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
# User, 2026-10-27: entrance_right's right-arm sweep turns the stand-in about 2.5 deg right, and
# for 20-160 ms near 5.7 s after arming the idle left wrist touches left_hip_roll_link while the
# left heel lifts two of its four corners (tilt under 0.7 deg). That gives 6 contacts in every quiet
# run since the gesture landed, and 7 only under CPU contention. entrance_left keeps all 8. The
# gesture runs in 223's demo loop, so its left heel is checked on the robot.
# User, 2026-10-28: entrance_right_natural's turn (waist_yaw to -0.17, right hip yaw 0.143) lifts
# two corners of one sole for ~2.2 s from ~5.3 s after arming (tilt under 1 deg, no body contact),
# in both MuJoCo runs. Its feet and hip yaw are checked on 223.
CASE_MIN_CONTACTS = {"gesture-entrance_right": 6, "gesture-entrance_right_natural": 6}
ARM_RETURN_RAD = 0.05
# After the wave hands back, SONIC's planner moves the right forearm off its start in MuJoCo (the
# plan itself returns it): wrist roll 0.36-0.40, pitch 0.14-0.16, yaw 0.08, elbow 0.06 rad over two
# runs. User accepted 2026-10-27: they ignore 223's right-wrist drift. Motor -> rad.
WAVE_FOREARM_RETURN_RAD = {25: 0.10, 26: 0.45, 27: 0.20, 28: 0.12}
CASE_ARM_RETURN_RAD = {
    "gesture-wave_natural": WAVE_FOREARM_RETURN_RAD,
    "stop-mid-wave_natural": WAVE_FOREARM_RETURN_RAD,
}
RESULT_TIMEOUT_S = 120.0
STOP_FRACTION = 0.6
HANDBACK = "control returned to SONIC's planner"
BOTH_ARMS = LEFT_ARM + RIGHT_ARM
# Imported gestures (design, Validation): every joint a keyframe commands (> COMMANDED_RAD from the
# measured base, wrists included) comes within TRACK_RAD of its target at least once in the
# keyframe's window: arrival + TRACK_OFFSET_S to departure + TRACK_OFFSET_S, or +-TRACK_OFFSET_S
# around the arrival for a keyframe without a hold.
TRACK_RAD = 0.25
TRACK_OFFSET_S = 0.3
COMMANDED_RAD = 0.01
# Waist gestures (invite turns waist_yaw, thanks and bow bow waist_pitch): over motion, retreat and settling
# to handback + SETTLE_S, no torso-side x pelvis/leg contact pair is new against the pre-arming
# stance, and the waist returns like the arms. The waist's tracking is printed, not asserted.
# Feet, pelvis drift and heading are printed only: the stand-in's feet slide and pivot in every
# gesture, waist or not (6.9-16.7 mm, 2.8-9.4 deg), so foot placement is checked on the robots
# (user, 2026-10-27).
WAIST = (12, 13, 14)
# invite's waist turn drags the shoulders off their targets (yaw up to 0.360, pitch up to 0.320 rad in
# MuJoCo). The user accepted that (2026-10-27, option A2): on invite's keyframes that turn the waist,
# every joint is printed, not asserted; the raise and the return to centre keep TRACK_RAD.
LOOSE_TURNS = ("invite",)
SETTLE_S = 0.5
# Task 1's MuJoCo-measured real waist change from the start at the 0.40 rad command (invite yaw +
# first, thanks raised-hands bow, bow hanging-arms bow): the stop-mid-waist trigger waits for STOP_WAIST_FRACTION of it.
WAIST_PEAK_CHANGE = {"invite": (12, 0.153), "thanks": (14, 0.216), "bow": (14, 0.213)}
STOP_WAIST_FRACTION = 0.5
STOP_TRIGGER_TIMEOUT_S = 30.0
CLIP_MAX_LAG_S = 1.0  # the lag search: measured shifted 0..CLIP_MAX_LAG_S s against the plan

CASES: list[tuple[str, str, dict[str, Any]]] = [
    *[(f"gesture-{name}", "perform_gesture", {"name": name}) for name in LEFT_ARM_GESTURES],
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
assert set(CASE_MIN_CONTACTS) <= {
    f"gesture-{name}" for name in (*RETAINED, *AUTHORED, *CLIP_GESTURES)
}

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
def stand_in(tmp: Path, hands_mounted: bool = False) -> Iterator[Sim]:
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
            encoder_path=ENCODER,
            max_dq=MAX_DQ,
            collision_model_path=SCENE,
            hands_mounted=hands_mounted,
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
    sim: Sim,
    mocker: Any,
    name: str,
    tool: str,
    kwargs: dict[str, Any],
    stop: bool = False,
    stop_when: Callable[[float], bool] | None = None,
) -> Window:
    """Call the skill, optionally stop it STOP_FRACTION through its plan (or once stop_when(the
    first chunk's observed_at) holds), and wait for its result."""
    skill, execution = sim.skill, sim.execution
    updates = mocker.spy(skill, "tool_update")
    submit = mocker.spy(execution, "submit_chunk")
    feedback = execution.get_feedback()
    assert feedback is not None
    base = np.asarray(feedback.body_q, np.float64)
    base_hands = np.r_[feedback.left_hand_q, feedback.right_hand_q].astype(np.float64)
    window = Window(name, time.monotonic(), 0.0, {})
    assert "started" in getattr(skill, tool)(**kwargs)
    if stop:
        wait_until(lambda: submit.call_count >= 1, timeout=60)
        plan_start = submit.call_args_list[0].args[2]
        keyframes, move_s, return_min_s, _, _ = gesture_plan_inputs(
            kwargs["name"], base, base_hands
        )
        kplan, pad_in, pad_out = plan_as_prepared(base, base_hands, keyframes, move_s, return_min_s)
        seconds = (len(kplan.poses) + pad_in + pad_out) / RATE
        window.planned_end = plan_start + seconds
        if stop_when is None:
            time.sleep(max(0.0, plan_start + STOP_FRACTION * seconds - time.monotonic()))
        else:
            wait_until(lambda: stop_when(plan_start), timeout=STOP_TRIGGER_TIMEOUT_S, interval=0.01)
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


def _physics(
    sim: Sim, w: Window, arms: list[int] = LEFT_ARM, before_planned_end: bool = True
) -> list[str]:
    """Print the case's row and return its physics failures: upright, feet down, no fast joint,
    and the `arms` joints back where they started (before the planned end, for a stopped case,
    when `before_planned_end`; otherwise that timing is only printed)."""
    log = np.load(sim.log)
    t, q, dq = log["t"], log["q"], np.abs(log["dq"])
    rows = np.flatnonzero((t >= w.start) & (t <= w.end))
    assert len(rows) > 10, w.name
    away_rad = np.abs(q[rows][:, arms] - q[rows[0], arms])
    arm = away_rad.max(axis=1)
    tolerance = np.full(len(arms), ARM_RETURN_RAD)
    for motor, rad in CASE_ARM_RETURN_RAD.get(w.name, {}).items():
        if motor in arms:
            tolerance[arms.index(motor)] = rad
    off = (away_rad > tolerance).any(axis=1)  # a joint past its own return tolerance
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
        f"floor contacts {contacts}": contacts >= CASE_MIN_CONTACTS.get(w.name, MIN_CONTACTS),
        f"peak dq {peak:.2f} rad/s": peak <= MAX_DQ,
        f"{'left arm' if arms == LEFT_ARM else 'arms'} end {arm[-1]:.3f} rad from the start": not off[
            -1
        ],
    }
    if w.stop_at is not None:
        assert w.planned_end is not None
        away = np.flatnonzero(off)
        back = t[rows[away[-1] + 1]] if len(away) and away[-1] + 1 < len(rows) else t[rows[0]]
        print(f"{w.name}: arms back {back - w.planned_end:+.1f} s from the planned end")
        if before_planned_end:
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


@pytest.mark.parametrize(
    "gesture,arms,before_planned_end",
    # two_hand_hearts: a retreat from 60% is planned to end ~3 s after the plan's own end, so
    # that timing is printed, not asserted; both arms must still be back home.
    [("wave", LEFT_ARM, True), ("two_hand_hearts", BOTH_ARMS, False)],
    ids=["wave", "two_hand_hearts"],
)
def test_stop_arm_mid_gesture_retreats(
    tmp_path: Path,
    tool_transport: Any,
    mocker: Any,
    gesture: str,
    arms: list[int],
    before_planned_end: bool,
) -> None:
    with stand_in(tmp_path) as sim:
        window = _run(
            sim, mocker, f"stop-mid-{gesture}", "perform_gesture", {"name": gesture}, stop=True
        )
    failures = _physics(sim, window, arms, before_planned_end)
    assert window.extra["stop_reply"].startswith("Stopping"), window.extra["stop_reply"]
    message = window.result["message"]
    assert "stopped early: stop requested" in message and message.endswith(HANDBACK), message
    assert window.stop_at is not None
    journal = Path(window.result["metadata"]["journal_path"]).read_text().splitlines()
    chunks = [e for e in map(json.loads, journal) if e["event"] == "chunk_accepted"]
    retreats = [e for e in chunks if e["sequence"] >= 1 and e["monotonic"] >= window.stop_at]
    assert len(retreats) == 1, [(e["sequence"], e["monotonic"]) for e in chunks]
    assert not failures, failures


def _rebuilt_plan(name: str, plans: Any) -> tuple[list[Any], Any, int]:
    """(keyframes, plan, pad): the plan rebuilt as _move builds it, from the base and hands _move
    measured (the arguments of its keyframe_plan call, the one with split_every); it must equal the
    plan it streamed. Plan row r runs pad (the first blend) + r rows after the first chunk's
    observed_at."""
    call = plans.call_args_list[-1]
    assert "split_every" in call.kwargs, plans.call_args_list
    base = np.asarray(call.args[0], np.float64)
    base_hands = np.asarray(call.kwargs["base_hands"], np.float64)
    keyframes, move_s, return_min_s, _, _ = gesture_plan_inputs(name, base, base_hands)
    kplan, pad, _ = plan_as_prepared(base, base_hands, keyframes, move_s, return_min_s)
    assert np.array_equal(kplan.poses, plans.spy_return.poses), "rebuilt plan != streamed plan"
    return keyframes, kplan, pad


def _plan_start(w: Window) -> float:
    """The first chunk's observed_at (journal), on the npz's monotonic clock."""
    journal = Path(w.result["metadata"]["journal_path"]).read_text().splitlines()
    first = next(e for e in map(json.loads, journal) if e["event"] == "chunk_accepted")
    assert first["sequence"] == 0, first
    return float(first["observed_at"])


def _tracking(
    sim: Sim,
    w: Window,
    name: str,
    plans: Any,
    skip: tuple[int, ...] = (),
    loose_turns: bool = False,
) -> list[str]:
    """Print the worst commanded joint per keyframe and return the keyframes that missed TRACK_RAD.
    Joints in `skip`, and with `loose_turns` every joint of a keyframe that commands the waist, are
    printed, not asserted. The plan is _rebuilt_plan's; its rows start at _plan_start."""
    keyframes, kplan, pad = _rebuilt_plan(name, plans)
    base = kplan.base
    plan_start = _plan_start(w)
    log = np.load(sim.log)
    t, q = log["t"], log["q"]
    print(
        f"\n{'case':<26}{'kf':>3}{'from s':>8}{'to s':>8}{'cmd':>5}  {'worst joint':<24}{'err':>7}"
    )
    failures = []
    for k, key in enumerate(keyframes):
        seg, nxt = kplan.segments[k], kplan.segments[k + 1]
        arrival = plan_start + (pad + seg.end - 1) / RATE
        departure = plan_start + (pad + nxt.start) / RATE
        lo, hi = (
            (arrival + TRACK_OFFSET_S, departure + TRACK_OFFSET_S)
            if nxt.start > seg.end
            else (arrival - TRACK_OFFSET_S, arrival + TRACK_OFFSET_S)
        )
        rows = (t >= lo) & (t <= hi)
        joints = np.flatnonzero(np.abs(key.target - base) > COMMANDED_RAD)
        turn = loose_turns and np.isin(joints, WAIST).any()
        checked = ~np.isin(joints, joints if turn else skip)
        assert rows.any() and len(joints), (k, lo - plan_start, hi - plan_start, joints)
        error = np.abs(q[rows][:, joints] - key.target[joints]).min(axis=0)
        worst = int(np.argmax(np.where(checked | turn, error, -1.0)))
        joint, err = MOTOR_NAMES[joints[worst]], float(error[worst])
        missed = checked.any() and err > TRACK_RAD
        loose = ", ".join(
            f"{MOTOR_NAMES[j]} {e:.3f}"
            for j, e in zip(joints[~checked], error[~checked], strict=True)
        )
        print(
            f"{w.name:<26}{k:>3}{lo - plan_start:>8.2f}{hi - plan_start:>8.2f}{checked.sum():>5}  "
            f"{joint:<24}{err:>7.3f}{'  MISSED' if missed else ''}"
            + (f"  (printed only: {loose})" if loose else "")
        )
        if missed:
            failures.append(f"keyframe {k}: {joint} off by {err:.3f} rad")
    return failures


def _settle(sim: Sim, w: Window) -> None:
    """Wait until the simulator has stepped past handback + SETTLE_S, so its log covers settling.
    A step's feedback arrives after the step's logged time, hence the two-tick margin."""
    target = w.end + SETTLE_S + 2 / RATE
    wait_until(
        lambda: (f := sim.execution.get_feedback()) is not None and f.received_monotonic >= target,
        timeout=SETTLE_S + 10.0,
        interval=0.01,
        message="the simulator stopped stepping before the settling window ended",
    )


def _wrap_deg(a: Any) -> Any:
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def _body(sim: Sim, w: Window, asserted: bool) -> list[str]:
    """Print the feet, pelvis, waist, legs and new torso-side x pelvis/leg contacts over the motion,
    any retreat and the settling (window start to end + SETTLE_S), all from the window start.
    Return the contact and waist-return failures when `asserted`; otherwise only print them.
    The contact baseline is every pair seen before the first armed sample (the pre-arming stance).
    """
    log = np.load(sim.log)
    t, q = log["t"], log["q"]
    assert t[-1] >= w.end + SETTLE_S, f"the log ends {t[-1] - w.end:.2f} s after handback"
    rows = np.flatnonzero((t >= w.start) & (t <= w.end + SETTLE_S))
    end = rows[t[rows] <= w.end][-1]  # handback, where _physics measures the arms' return
    feet = log["feet_xy_yaw"][rows].reshape(len(rows), 2, 3)  # left, right: x, y, yaw (rad)
    foot_move = np.linalg.norm(feet[:, :, :2] - feet[0, :, :2], axis=2).max(axis=0)
    foot_turn = np.abs(_wrap_deg(np.degrees(feet[:, :, 2] - feet[0, :, 2]))).max(axis=0)
    pelvis = log["pelvis_xy"][rows]
    drift = float(np.linalg.norm(pelvis - pelvis[0], axis=1).max())
    heading = float(np.abs(_wrap_deg(log["heading_deg"][rows] - log["heading_deg"][rows[0]])).max())
    q0, waist = q[rows[0]], list(WAIST)
    change = q[rows][:, waist] - q0[waist]
    waist_back = np.abs(q[end, waist] - q0[waist])
    legs = np.abs(q[rows][:, :12] - q0[:12]).max(axis=0)
    hip_yaw = float(legs[[2, 8]].max())
    leg = int(np.argmax(np.where(np.isin(np.arange(12), [2, 8]), -1.0, legs)))
    armed = np.flatnonzero(log["armed"])
    stance = log["self_contacts"][: armed[0]] if len(armed) else log["self_contacts"][:0]
    baseline = {tuple(p) for s in stance for p in json.loads(s)}
    new: dict[tuple[str, ...], int] = {}
    for s in log["self_contacts"][rows]:
        for p in map(tuple, json.loads(s)):
            if p not in baseline:
                new[p] = new.get(p, 0) + 1
    print(
        f"{w.name}: feet move L {foot_move[0] * 1000:.1f} R {foot_move[1] * 1000:.1f} mm, turn "
        f"L {foot_turn[0]:.2f} R {foot_turn[1]:.2f} deg, pelvis drift {drift * 1000:.1f} mm, "
        f"heading {heading:.2f} deg (printed only); legs peak {legs[leg]:.3f} "
        f"({MOTOR_NAMES[leg]}), hip yaw {hip_yaw:.3f}; contacts baseline {sorted(baseline)}, "
        f"new {new}\n{w.name}: waist "
        + ", ".join(
            f"{MOTOR_NAMES[j]} abs {q[rows, j].min():+.3f}..{q[rows, j].max():+.3f} change "
            f"{change[:, i].min():+.3f}/{change[:, i].max():+.3f} end {waist_back[i]:.3f}"
            for i, j in enumerate(WAIST)
        )
        + f" rad ({'asserted' if asserted else 'printed only'})"
    )
    checks = {
        f"new torso contacts {new}": not new,
        f"waist end {waist_back.max():.3f} rad from the start": waist_back.max() <= ARM_RETURN_RAD,
    }
    return [k for k, ok in checks.items() if not ok] if asserted else []


@pytest.mark.parametrize("name", RETAINED + AUTHORED)
def test_imported_gesture_tracks_and_hands_back(
    tmp_path: Path, tool_transport: Any, mocker: Any, name: str
) -> None:
    plans = mocker.spy(sonic_arm_skill, "keyframe_plan")
    with stand_in(tmp_path) as sim:
        window = _run(sim, mocker, f"gesture-{name}", "perform_gesture", {"name": name})
        _settle(sim, window)
    message = window.result["message"]
    # A refusal sends nothing, so there is no motion to check: report the refusal itself.
    assert "nothing sent" not in message, message
    failures = _physics(sim, window, BOTH_ARMS)
    waist = name in WAIST_GESTURES
    # Tracking and the body first, so they are printed even when a standard assert below fails.
    # The waist's tracking (also printed by _body) and LOOSE_TURNS are printed, not asserted.
    tracking = (
        _tracking(sim, window, name, plans, WAIST if waist else (), name in LOOSE_TURNS)
        if "metadata" in window.result
        else ["no report"]
    )
    body = _body(sim, window, asserted=waist)
    reached = window.result.get("metadata", {}).get("reached_fraction")
    print(f"{window.name}: reached {reached} (recorded, not asserted)")
    assert "stopped early" not in message and "failed" not in message, message
    assert f"of the planned pose and returned; {HANDBACK}" in message, message
    assert message.endswith(HANDBACK), message
    assert not failures, failures
    assert not tracking, tracking
    assert not body, body


@pytest.mark.parametrize("gesture", WAIST_GESTURES)
def test_stop_mid_waist_retreats_home(
    tmp_path: Path, tool_transport: Any, mocker: Any, gesture: str
) -> None:
    """stop_arm mid-turn (invite) or mid-bow (thanks, bow) must bring the waist home with the arms.

    The stop is triggered from live feedback, not plan timing: once the plan clock is inside the
    waist segment and the live waist joint has moved STOP_WAIST_FRACTION of Task 1's measured real
    change in the commanded direction. The waist segment runs from the start of the motion to the
    first keyframe that moves the waist joint (its target sets the direction) until the plan
    commands the joint back to its start: the motion, any hold, and, since SONIC's waist lags its
    reference, the way back to the start side (invite: the swing to the other side). The npz
    confirms the stop afterwards.
    """
    plans = mocker.spy(sonic_arm_skill, "keyframe_plan")
    joint, peak = WAIST_PEAK_CHANGE[gesture]
    seg: dict[str, Any] = {}

    def locate() -> None:
        _, kplan, pad = _rebuilt_plan(gesture, plans)
        start = kplan.base[joint]
        first = next(
            s
            for s in kplan.segments
            if s.dst >= 0 and abs(kplan.targets[s.dst, joint] - start) > COMMANDED_RAD
        )
        sign = np.sign(kplan.targets[first.dst, joint] - start)
        side = sign * (kplan.poses[first.start :, joint] - start)  # commanded displacement
        top = int(np.argmax(side))
        hi = first.start + top + int(np.argmax(side[top:] <= 0))
        seg.update(base=start, pad=pad, lo=first.start, hi=hi, sign=sign, segments=kplan.segments)

    def ready(plan_start: float) -> bool:
        if not seg:
            locate()
        row = (time.monotonic() - plan_start) * RATE - seg["pad"]
        assert row < seg["hi"], f"the waist segment ended before {STOP_WAIST_FRACTION:.0%} of it"
        feedback = sim.execution.get_feedback()
        return (
            feedback is not None
            and row >= seg["lo"]
            and seg["sign"] * (feedback.body_q[joint] - seg["base"]) >= STOP_WAIST_FRACTION * peak
        )

    with stand_in(tmp_path) as sim:
        window = _run(
            sim,
            mocker,
            f"stop-mid-{gesture}",
            "perform_gesture",
            {"name": gesture},
            stop=True,
            stop_when=ready,
        )
        _settle(sim, window)
    failures = _physics(sim, window, BOTH_ARMS, before_planned_end=False)
    body = _body(sim, window, asserted=True)
    assert window.stop_at is not None
    log = np.load(sim.log)
    t, q = log["t"], log["q"]
    # The feedback the trigger read is the sample at or just after the stop's last logged tick.
    i = int(np.searchsorted(t, window.stop_at, side="right"))
    row = (t[i - 1] - _plan_start(window)) * RATE - seg["pad"]
    moved = float(seg["sign"] * (q[i - 1 : i + 1, joint] - seg["base"]).max())
    inside = next(k for k, s in enumerate(seg["segments"]) if row < s.end)
    print(
        f"{window.name}: stop at plan row {row:.0f}, plan segment {inside} "
        f"({(row - seg['lo']) / RATE:.2f} s into the waist segment, rows {seg['lo']}-{seg['hi']}), "
        f"{MOTOR_NAMES[joint]} moved {moved:+.3f} rad ({moved / peak:.0%} of Task 1's {peak})"
    )
    assert seg["lo"] <= row < seg["hi"], (row, seg)
    assert moved >= STOP_WAIST_FRACTION * peak, (moved, peak)
    assert window.extra["stop_reply"].startswith("Stopping"), window.extra["stop_reply"]
    message = window.result["message"]
    assert "stopped early: stop requested" in message and message.endswith(HANDBACK), message
    journal = Path(window.result["metadata"]["journal_path"]).read_text().splitlines()
    chunks = [e for e in map(json.loads, journal) if e["event"] == "chunk_accepted"]
    retreats = [e for e in chunks if e["sequence"] >= 1 and e["monotonic"] >= window.stop_at]
    assert len(retreats) == 1, [(e["sequence"], e["monotonic"]) for e in chunks]
    assert not failures, failures
    assert not body, body


def _clip_tracking(sim: Sim, w: Window, name: str, plans: Any) -> float:
    """Print how SONIC played the clip and return its arm speed limit (fast_max_dq): the busiest
    arm joint's measured over planned peak-to-peak, the arm lag from the best cross-correlation
    shift, stream_peak_dq against the limit and the legs. Printed, not asserted."""
    _, kplan, pad = _rebuilt_plan(name, plans)
    clip = next(s for s in kplan.segments if s.clip)
    planned = kplan.poses[clip.start : clip.end]
    peak = np.abs(np.diff(planned[:, ARM_MOTORS], axis=0)).max() * RATE
    fast_max_dq = max(MAX_DQ, min(CLIP_DQ_CEILING, CLIP_DQ_FACTOR * peak))
    j = ARM_MOTORS[int(np.argmax(np.ptp(planned[:, ARM_MOTORS], axis=0)))]
    when = _plan_start(w) + (pad + np.arange(clip.start, clip.end)) / RATE
    log = np.load(sim.log)
    t, q = log["t"], log["q"]
    ref = planned[:, j] - planned[:, j].mean()

    def score(shift: float) -> float:
        got = np.interp(when + shift, t, q[:, j])
        got = got - got.mean()
        return float(ref @ got / (np.linalg.norm(ref) * np.linalg.norm(got) + 1e-12))

    shifts = np.arange(0.0, CLIP_MAX_LAG_S + 1e-9, 1 / RATE)
    lag = float(shifts[int(np.argmax([score(float(s)) for s in shifts]))])
    measured = np.interp(when + lag, t, q[:, j])
    ratio = float(np.ptp(measured) / np.ptp(planned[:, j]))
    meta = w.result.get("metadata", {})
    print(
        f"{w.name}: clip {(clip.end - clip.start) / RATE:.2f} s, busiest {MOTOR_NAMES[j]} planned "
        f"p-p {np.ptp(planned[:, j]):.3f} rad, measured {np.ptp(measured):.3f} (amplitude ratio "
        f"{ratio:.2f}), lag {lag * 1000:.0f} ms (corr {score(lag):.3f}); planned arm peak "
        f"{peak:.2f} rad/s, stream_peak_dq {meta.get('stream_peak_dq')} vs fast_max_dq "
        f"{fast_max_dq:.2f}; legs {meta.get('max_deviation_rad', {}).get('legs')}, closest leg "
        f"{meta.get('closest_leg')} (printed only)"
    )
    return float(fast_max_dq)


def _clip_speed(sim: Sim, w: Window, fast_max_dq: float, failures: list[str]) -> list[str]:
    """_physics' failures with its all-joint MAX_DQ check swapped for the clip's: legs and waist at
    MAX_DQ, arms at fast_max_dq (the skill's own watch applies it only inside the clip window)."""
    log = np.load(sim.log)
    rows = (log["t"] >= w.start) & (log["t"] <= w.end)
    dq = np.abs(log["dq"][rows])
    body, arms = float(dq[:, :15].max()), float(dq[:, ARM_MOTORS].max())
    print(f"{w.name}: npz peak dq legs+waist {body:.2f} (<= {MAX_DQ}), arms {arms:.2f} rad/s")
    checks = {
        f"legs/waist peak dq {body:.2f} rad/s": body <= MAX_DQ,
        f"arm peak dq {arms:.2f} rad/s over {fast_max_dq:.2f}": arms <= fast_max_dq,
    }
    return [f for f in failures if not f.startswith("peak dq")] + [
        k for k, ok in checks.items() if not ok
    ]


@pytest.mark.parametrize("name", CLIP_GESTURES)
def test_clip_gesture_tracks_and_hands_back(
    tmp_path: Path, tool_transport: Any, mocker: Any, name: str
) -> None:
    plans = mocker.spy(sonic_arm_skill, "keyframe_plan")
    with stand_in(tmp_path) as sim:
        window = _run(sim, mocker, f"gesture-{name}", "perform_gesture", {"name": name})
        _settle(sim, window)
    message = window.result["message"]
    assert "nothing sent" not in message, message
    failures = _physics(sim, window, BOTH_ARMS)
    fast_max_dq = _clip_tracking(sim, window, name, plans) if "metadata" in window.result else 0.0
    failures = _clip_speed(sim, window, fast_max_dq, failures)
    body = _body(sim, window, asserted=True)
    assert "stopped early" not in message and "failed" not in message, message
    assert f"of the planned pose and returned; {HANDBACK}" in message, message
    assert not failures, failures
    assert not body, body


def test_stop_mid_clip_finishes_then_returns_in_sim(
    tmp_path: Path, tool_transport: Any, mocker: Any
) -> None:
    """stop_arm STOP_FRACTION through the wave's plan lands inside the clip: the clip plays to its
    end, then the plan's own way home, and the arms are back."""
    with stand_in(tmp_path) as sim:
        window = _run(
            sim, mocker, "stop-mid-wave_natural", "perform_gesture", {"name": "wave_natural"}, True
        )
        _settle(sim, window)
    failures = _physics(sim, window, BOTH_ARMS, before_planned_end=False)
    failures = [f for f in failures if not f.startswith("peak dq")]  # the clip's own speed
    body = _body(sim, window, asserted=True)
    assert window.extra["stop_reply"].startswith("Stopping"), window.extra["stop_reply"]
    message = window.result["message"]
    assert "stopped early: stop requested (finished the clip, then returned)" in message, message
    assert message.endswith(HANDBACK), message
    assert not failures, failures
    assert not body, body


def test_finger_check_without_hands_reports_did_not_follow(
    tmp_path: Path, tool_transport: Any, mocker: Any
) -> None:
    """The stand-in has no hands and reports zero hand positions. Configured with hands, the
    finger check must read those zeros as a hand that did not follow, not as unavailable."""
    with stand_in(tmp_path, hands_mounted=True) as sim:
        skill = sim.skill
        window = Window("finger-check", time.monotonic(), 0.0, {})
        assert skill.check_dex3_finger_order() == "started"
        wait_until(lambda: skill.operator_result() is not None, timeout=RESULT_TIMEOUT_S)
        wait_until(lambda: skill._running is None, timeout=10)
        window.end = time.monotonic()
        text = skill.operator_result()
        assert text is not None
        window.result = {"message": text}
    failures = _physics(sim, window, BOTH_ARMS)
    log = np.load(sim.log)
    q = log["q"][(log["t"] >= window.start) & (log["t"] <= window.end)]
    # While SONIC follows the still reference the body settles away from the planner stance: the
    # stand-in measured 0.083 rad (left shoulder yaw), legs 0.072. So the body must stay inside the
    # finger check's own envelope while it runs, and be back within ARM_RETURN_RAD after handback.
    config = sim.skill.config
    envelope = group_margins(config.body_margin, config.arm_margin, config.wrist_margin)
    drift = np.abs(q - q[0]).max(axis=0)
    worst = int(np.argmax(drift / envelope))
    end = np.abs(q[-1] - q[0])
    print(
        f"finger-check: body drift {drift[worst]:.4f} rad ({MOTOR_NAMES[worst]}, envelope "
        f"{envelope[worst]:.2f}) over {len(q)} samples; legs {drift[:12].max():.4f}, waist "
        f"{drift[12:15].max():.4f}, arms {drift[BOTH_ARMS].max():.4f}; at the end "
        f"{end.max():.4f} ({MOTOR_NAMES[int(np.argmax(end))]})"
    )
    result = json.loads(text)  # A failed hand result keeps its diagnostics as SkillResult JSON.
    message = result["message"]
    assert message.startswith(f"{FINGER_CHECK} failed: "), message
    assert "hand did not follow" in message and "hold median" in message, message
    assert "unavailable" not in text and "nan" not in text.lower(), text
    assert message.endswith(HANDBACK), message
    assert {"hand_base", "hand_target", "hand_held"} <= set(result["metadata"]), result
    assert np.all(drift <= envelope), (MOTOR_NAMES[worst], drift[worst], envelope[worst])
    assert end.max() <= ARM_RETURN_RAD, (MOTOR_NAMES[int(np.argmax(end))], end.max())
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
