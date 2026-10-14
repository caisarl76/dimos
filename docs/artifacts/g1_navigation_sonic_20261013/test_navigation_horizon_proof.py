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

import csv
import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).with_name("demo_navigation_horizon_proof.py")
SPEC = importlib.util.spec_from_file_location("navigation_horizon_proof", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
proof = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(proof)


@pytest.fixture
def trace(tmp_path):
    # Two real CSV buffers: initialization and one translating replan.
    blocks = [
        [[float(block * 100 + frame + joint / 100) for joint in range(36)] for frame in range(62)]
        for block in range(2)
    ]
    with (tmp_path / "planner-motion.csv").open("w") as stream:
        writer = csv.writer(stream)
        for block in blocks:
            writer.writerows([[*row, ""] for row in block])
            writer.writerow([])
    keys = ["reference_x", "reference_y", "reference_z"]
    keys += [f"reference_quat_{i}" for i in range(4)]
    keys += [f"reference_motor_q_{i}" for i in range(29)]
    with (tmp_path / "target-motion.csv.frames.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["monotonic", "frame", *keys])
        writer.writerow([1.2, 0, *blocks[1][0]])
        writer.writerow([1.9, 35, *blocks[1][35]])
    (tmp_path / "planner-motion.csv.replans.csv").write_text(
        "started_monotonic,finished_monotonic,mode,speed,mode_changed,periodic\n"
        "1.15,1.16,1,0.2,1,0\n"
    )
    (tmp_path / "results.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "cases": [{"name": "forward", "start": 1.0, "end": 3.0, "passed": True}],
            }
        )
    )
    (tmp_path / "telemetry.jsonl").write_text(
        "".join(
            json.dumps(
                {
                    "monotonic": t,
                    "execution": {"feedback_fresh": True, "navigation": {"phase": "active"}},
                    "feedback": {
                        "planner_reference_active": True,
                        "planner_guard": {
                            "mode": 1,
                            "lease_armed": True,
                            "latched": False,
                            "fault": None,
                        },
                    },
                }
            )
            + "\n"
            for t in [1.19, 1.89]
        )
    )
    return tmp_path


def test_buffer_replay_reports_padding_even_when_physical_case_passed(trace):
    result = proof.analyze_run(trace)
    case = result["cases"][0]
    assert case["max_terminal_slots"] == 4
    assert case["max_clamped_slots"] == 4
    assert case["terminal_windows_ge3"] == 1
    assert case["initial_transition_excluded_s"] == pytest.approx(0.2)
    assert case["physical_passed"] is True
    assert case["coverage_passed"] is False


def test_contradictory_last_joint_cannot_be_matched_using_root_or_timestamps(trace):
    path = trace / "target-motion.csv.frames.csv"
    rows = list(csv.reader(path.open()))
    rows[-1][-1] = str(float(rows[-1][-1]) + 0.1)
    with path.open("w") as stream:
        csv.writer(stream).writerows(rows)
    case = proof.analyze_run(trace)["cases"][0]
    assert case["unmatched_samples"] == 1
    assert case["coverage_passed"] is False


def test_equal_buffers_remain_ambiguous_instead_of_selecting_favorable_candidate(trace):
    blocks = proof.read_blocks(trace / "planner-motion.csv")
    with (trace / "planner-motion.csv").open("a") as stream:
        writer = csv.writer(stream)
        writer.writerows([*row, ""] for row in blocks[1])
        writer.writerow([])
    case = proof.analyze_run(trace)["cases"][0]
    assert case["ambiguous_samples"] == 2
    assert case["coverage_passed"] is False


@pytest.fixture
def complete_trace(trace):
    blocks = [
        [[float(block * 1000 + frame + joint / 100) for joint in range(36)] for frame in range(150)]
        for block in range(2)
    ]
    with (trace / "planner-motion.csv").open("w") as stream:
        writer = csv.writer(stream)
        for block in blocks:
            writer.writerows([*row, ""] for row in block)
            writer.writerow([])
    with (trace / "target-motion.csv.frames.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["monotonic", "frame", *proof.REFERENCE_KEYS])
        for tick in range(100):
            block = int(tick >= 10)
            frame = tick - 10 if block else tick
            writer.writerow([1.0 + tick * 0.02, frame, *blocks[block][frame]])
    feedback = json.loads((trace / "telemetry.jsonl").read_text().splitlines()[0])
    with (trace / "telemetry.jsonl").open("w") as stream:
        for tick in range(100):
            feedback["monotonic"] = 0.99 + tick * 0.02
            stream.write(json.dumps(feedback) + "\n")
    return trace


@pytest.mark.parametrize("missing", ["start", "middle", "end"])
def test_missing_target_trace_cannot_prove_full_window(complete_trace, missing):
    assert proof.analyze_run(complete_trace)["cases"][0]["coverage_passed"] is True
    path = complete_trace / "target-motion.csv.frames.csv"
    with path.open() as stream:
        rows = list(csv.reader(stream))
    keep = {
        "start": lambda t: t >= 1.2,
        "middle": lambda t: not 1.5 < t < 1.8,
        "end": lambda t: t <= 1.4,
    }[missing]
    with path.open("w") as stream:
        csv.writer(stream).writerows([rows[0], *(row for row in rows[1:] if keep(float(row[0])))])
    case = proof.analyze_run(complete_trace)["cases"][0]
    assert case["coverage_passed"] is False
    assert f"target_{missing}_gap" in case["proof_failure_reasons"]


def test_large_initial_exclusion_cannot_hide_most_of_command_window(complete_trace):
    blocks = proof.read_blocks(complete_trace / "planner-motion.csv")
    path = complete_trace / "target-motion.csv.frames.csv"
    with path.open() as stream:
        header = next(csv.reader(stream))
    with path.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for tick in range(100):
            block = int(tick >= 95)
            frame = tick - 95 if block else tick
            writer.writerow([1.0 + tick * 0.02, frame, *blocks[block][frame]])
    (complete_trace / "planner-motion.csv.replans.csv").write_text(
        "started_monotonic,finished_monotonic,mode,speed,mode_changed,periodic\n"
        "2.85,2.86,1,0.2,1,0\n"
    )
    case = proof.analyze_run(complete_trace)["cases"][0]
    assert case["coverage_passed"] is False
    assert case["initial_transition_excluded_s"] == pytest.approx(1.9)
    assert "initial_transition_exceeds_bound" in case["proof_failure_reasons"]
