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

"""Supervised walk harness decisions: case gating, measured motion and native evidence."""

from datetime import date
import json
import math

import pytest

from docs.artifacts.g1_navigation_sonic_20261013 import demo_pc2_navigation_walk as walk


def passed(*names: str) -> list[dict]:
    return [{"name": n, "passed": True} for n in names]


def test_cases_unlock_only_after_their_measured_prerequisites() -> None:
    assert walk.blocked("forward_010", []) == "requires idle, operator_stop"
    assert walk.blocked("forward_010", passed("idle")) == "requires operator_stop"
    assert walk.blocked("forward_010", passed("idle", "operator_stop")) is None
    # Operator decision 2026-10-15: 0.10 m/s for 1 s applied but never stepped on pc2, so
    # the first walking case is the simulation-verified 0.20 m/s for 2 s.
    failed = [*passed("idle", "operator_stop"), {"name": "forward_010", "passed": False}]
    assert walk.blocked("forward_020", failed) is None
    for name in ("turn_left_010", "turn_right_010", "cancel", "source_pause"):
        assert walk.blocked(name, failed) == "requires forward_020"
        assert walk.blocked(name, [*failed, *passed("forward_020")]) is None
    assert walk.blocked("goal", passed(*walk.CASES)) is not None  # never offered here
    assert set(walk.CASES) >= {"idle", "operator_stop", "forward_010", "forward_020"}
    assert max(abs(c.vx) for c in walk.CASES.values()) <= 0.20
    assert max(abs(c.wz) for c in walk.CASES.values()) <= 0.10


def test_operator_token_must_be_today() -> None:
    walk.check_token(f"operator-ready-{date.today():%Y%m%d}")
    with pytest.raises(SystemExit):
        walk.check_token("operator-ready-20000101")


def odom(t: float, x: float, y: float, yaw: float) -> dict:
    return {"peer_monotonic": t, "odom": [x, y, yaw, t]}


def test_motion_is_measured_in_the_start_heading_frame() -> None:
    heading = math.radians(30)
    # Already 2 cm along by the first sample inside the window: the start is the pose at t0.
    early = (1.0 + 0.02 * math.cos(heading), 2.0 + 0.02 * math.sin(heading))
    rows = [odom(0.0, 1.0, 2.0, heading), odom(0.5, *early, heading)]
    end = (
        1.0 + 0.10 * math.cos(heading) - 0.01 * math.sin(heading),
        2.0 + 0.10 * math.sin(heading) + 0.01 * math.cos(heading),
    )
    rows.append(odom(2.0, *end, heading + 0.05))
    rows.append(odom(9.0, 5.0, 5.0, 0.0))  # after the window
    moved = walk.motion(rows, 0.4, 3.0)
    assert abs(moved["forward_m"] - 0.10) < 1e-9 and abs(moved["lateral_m"] - 0.01) < 1e-9
    assert abs(moved["yaw_rad"] - 0.05) < 1e-9
    assert abs(moved["heading_offset_deg"] - math.degrees(math.atan2(0.01, 0.10))) < 1e-9
    assert walk.motion(rows[:1], 0.4, 3.0) is None  # no sample inside the window


def test_motion_verdict_needs_correct_sign_and_bounded_magnitude() -> None:
    case = walk.CASES["forward_010"]
    good = {"forward_m": 0.07, "lateral_m": 0.004, "yaw_rad": 0.01, "heading_offset_deg": 3.3}
    assert walk.motion_ok(case, good)
    assert not walk.motion_ok(case, {**good, "forward_m": 0.005})  # stayed in place
    assert not walk.motion_ok(case, {**good, "forward_m": -0.05})
    assert not walk.motion_ok(case, {**good, "forward_m": 0.25})  # > 2x commanded
    turn = walk.CASES["turn_right_010"]
    assert walk.motion_ok(turn, {**good, "forward_m": 0.0, "yaw_rad": -0.08})
    assert not walk.motion_ok(turn, {**good, "forward_m": 0.0, "yaw_rad": 0.08})
    assert not walk.motion_ok(case, None)


def native(t: float, mode: int | None, *, latched: bool = False, seq: int = 0) -> dict:
    return {
        "monotonic": t,
        "feedback": mode is not None,
        "mode": mode,
        "latched": latched if mode is not None else None,
        "applied_sequence": seq if mode is not None else None,
        "phase": "active",
    }


def test_native_summary_reports_walk_then_idle_without_latch() -> None:
    rows = [
        native(0.0, 0, seq=1),
        native(0.5, 1, seq=5),
        native(1.5, 1, seq=40),
        native(2.0, 0, seq=41),
    ]
    summary = walk.native_summary(rows)
    assert summary == {
        "samples": 4,
        "walked": True,
        "ended_idle": True,
        "latched": False,
        "feedback_gaps": 0,
        "applied_sequence_advance": 40,
    }
    latched = walk.native_summary([*rows[:-1], native(2.0, 0, latched=True, seq=41)])
    assert latched["latched"] and latched["ended_idle"]
    lost = walk.native_summary([*rows, native(2.5, None)])
    assert lost["feedback_gaps"] == 1 and not lost["ended_idle"]


def test_prior_session_counts_only_its_passed_cases_with_the_same_token(tmp_path) -> None:
    # The native operator stop exits the controller, so later cases run in a new session.
    token = f"operator-ready-{date.today():%Y%m%d}"
    old = tmp_path / "s1"
    old.mkdir()
    cases = [*passed("idle", "operator_stop"), {"name": "forward_010", "passed": False}]
    (old / "results.json").write_text(json.dumps({"operator_ready": token, "cases": cases}))
    prior = walk.prior_passed([old], token)
    assert [c["name"] for c in prior] == ["idle", "operator_stop"]
    assert walk.blocked("forward_010", prior) is None
    assert walk.blocked("turn_left_010", prior) == "requires forward_020"
    with pytest.raises(SystemExit):
        walk.prior_passed([old], "operator-ready-20000101")  # another day's readiness


def test_idle_accepts_a_one_time_stance_settle_but_not_walking() -> None:
    # pc2 session 2: a ~1.2 deg heading adjustment as IDLE began, then still (operator: y).
    settle = {"forward_m": 0.0037, "lateral_m": -0.0065, "yaw_rad": 0.024}
    assert walk.idle_ok({"walked": False}, settle)
    assert not walk.idle_ok({"walked": True}, settle)
    assert not walk.idle_ok({"walked": False}, {**settle, "forward_m": 0.025})
    assert not walk.idle_ok({"walked": False}, {**settle, "yaw_rad": math.radians(3.5)})
    assert not walk.idle_ok({"walked": False}, None)
