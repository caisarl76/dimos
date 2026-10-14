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

"""Independently reconstruct native future-window coverage from saved CSV buffers.

Usage: python demo_navigation_horizon_proof.py RUN_DIR... --output report.json
Exit 0 means every selected window has proved coverage; 1 means padding or
incomplete/contradictory evidence. Physical case verdicts are reported separately.
"""

import argparse
from bisect import bisect_right
import csv
import hashlib
from itertools import pairwise
import json
import math
from pathlib import Path
from typing import Any

REFERENCE_KEYS = ["reference_x", "reference_y", "reference_z"]
REFERENCE_KEYS += [f"reference_quat_{i}" for i in range(4)]
REFERENCE_KEYS += [f"reference_motor_q_{i}" for i in range(29)]
OFFSETS = tuple(range(0, 46, 5))  # Enabled encoder: 10 samples, stride 5.
MATCH_ATOL = 1e-5  # Planner CSV uses six significant digits.
# Evidence bounds taken from existing measurement/source freshness budgets.
# They constrain the proof; they do not alter physical command windows.
TARGET_GAP_BOUND_S = 0.12
INITIAL_TRANSITION_BOUND_S = 0.350
FEEDBACK_AGE_BOUND_S = 0.2
REASONS = (
    "periodic",
    "mode_changed",
    "facing_changed",
    "height_changed",
    "speed_changed",
    "direction_changed",
    "horizon_refresh",
)


def read_blocks(path: Path) -> list[list[list[float]]]:
    blocks: list[list[list[float]]] = []
    current: list[list[float]] = []
    with path.open() as stream:
        for row in csv.reader(stream):
            if not row:
                if current:
                    blocks.append(current)
                    current = []
                continue
            values = [float(value) for value in row if value != ""]
            if len(values) != 36 or not all(math.isfinite(x) for x in values):
                raise ValueError("planner buffer row must contain 36 finite values")
            current.append(values)
    if current:
        blocks.append(current)
    return blocks


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open() as stream:
        return list(csv.DictReader(stream))


def matches(row: dict[str, str], blocks: list[list[list[float]]]) -> list[int]:
    frame = int(row["frame"])
    values = [float(row[key]) for key in REFERENCE_KEYS]
    if not all(math.isfinite(x) for x in values):
        return []
    return [
        index
        for index, block in enumerate(blocks)
        if 0 <= frame < len(block)
        and all(abs(a - b) <= MATCH_ATOL for a, b in zip(values, block[frame], strict=True))
    ]


def healthy_mode1(sample: dict[str, Any] | None, timestamp: float) -> bool:
    if sample is None or not 0 <= timestamp - sample["monotonic"] < FEEDBACK_AGE_BOUND_S:
        return False
    execution = sample.get("execution") or {}
    feedback = sample.get("feedback") or {}
    guard = feedback.get("planner_guard") or {}
    return bool(
        execution.get("feedback_fresh")
        and execution.get("navigation", {}).get("phase") == "active"
        and feedback.get("planner_reference_active")
        and guard.get("mode") == 1
        and guard.get("lease_armed")
        and not guard.get("latched")
        and not guard.get("fault")
    )


def analyze_run(path: Path) -> dict[str, Any]:
    blocks = read_blocks(path / "planner-motion.csv")
    replans = read_csv(path / "planner-motion.csv.replans.csv")
    # Each initialization contributes an extra buffer (pose handbacks can add
    # more later). For the translating prefix, validate every used buffer's
    # ordinal association against its first content-matched realization below.
    targets = read_csv(path / "target-motion.csv.frames.csv")
    with (path / "telemetry.jsonl").open() as stream:
        telemetry = [json.loads(line) for line in stream]
    times = [float(row["monotonic"]) for row in telemetry]
    results = json.loads((path / "results.json").read_text())
    selected = [
        case
        for case in results["cases"]
        if "start" in case
        and "end" in case
        and (
            case["name"].startswith(("forward", "backward"))
            or case["name"] == "noisy_trace_replan_limit"
        )
    ]
    reports = []
    for case in selected:
        start, end = float(case["start"]), float(case["end"])
        rows = [row for row in targets if start <= float(row["monotonic"]) <= end]
        target_times = [float(row["monotonic"]) for row in rows]
        target_gaps = [b - a for a, b in pairwise(target_times)]
        start_gap = target_times[0] - start if target_times else None
        end_gap = end - target_times[-1] if target_times else None
        max_gap = max(target_gaps, default=0.0) if target_times else None
        proof_failures = []
        if not math.isfinite(end - start) or end <= start:
            proof_failures.append("invalid_measured_window")
        if not target_times:
            proof_failures.append("no_target_samples")
        else:
            if any(gap <= 0 for gap in target_gaps):
                proof_failures.append("target_times_not_increasing")
            for name, gap in (("start", start_gap), ("middle", max_gap), ("end", end_gap)):
                if gap > TARGET_GAP_BOUND_S:
                    proof_failures.append(f"target_{name}_gap")
        matched = [(row, matches(row, blocks)) for row in rows]
        # Establish the first observed merge by buffer content, not a timestamp
        # estimate of where a trajectory starts or how long it is.
        first_seen: dict[int, float] = {}
        for row, candidates in matched:
            if len(candidates) == 1:
                first_seen.setdefault(candidates[0], float(row["monotonic"]))
        validated: set[int] = set()
        for index, timestamp in first_seen.items():
            if 0 < index <= len(replans):
                finished = float(replans[index - 1]["finished_monotonic"])
                next_finished = (
                    float(replans[index]["finished_monotonic"])
                    if index < len(replans)
                    else math.inf
                )
                if finished <= timestamp < next_finished:
                    validated.add(index)
        first_merge = None
        for row, candidates in matched:
            if len(candidates) != 1 or candidates[0] not in validated:
                continue
            replan = replans[candidates[0] - 1]
            timestamp = float(row["monotonic"])
            if (
                int(replan["mode"]) == 1
                and float(replan["speed"]) > 0
                and float(replan["finished_monotonic"]) <= timestamp
            ):
                first_merge = timestamp
                break
        if first_merge is None:
            proof_failures.append("no_translating_merge")
        elif first_merge - start > INITIAL_TRANSITION_BOUND_S:
            proof_failures.append("initial_transition_exceeds_bound")
        report: dict[str, Any] = {
            "name": case["name"],
            "start": start,
            "end": end,
            "physical_passed": case.get("passed"),
            "forward_displacement_m": case.get("forward_displacement_m"),
            "initial_transition_excluded_s": None if first_merge is None else first_merge - start,
            "proof_bounds": {
                "target_gap_s": TARGET_GAP_BOUND_S,
                "initial_transition_s": INITIAL_TRANSITION_BOUND_S,
                "feedback_age_s": FEEDBACK_AGE_BOUND_S,
            },
            "proof_failure_reasons": proof_failures,
            "target_start_gap_s": start_gap,
            "target_end_gap_s": end_gap,
            "target_max_gap_s": max_gap,
            "target_samples": len(rows),
            "initial_transition_excluded_samples": 0,
            "eligible_samples": 0,
            "unhealthy_or_non_mode1_samples": 0,
            "unmatched_samples": 0,
            "ambiguous_samples": 0,
            "contradictory_samples": 0,
            "terminal_windows": 0,
            "terminal_windows_ge3": 0,
            "clamped_windows": 0,
            "max_terminal_slots": 0,
            "max_clamped_slots": 0,
            "samples": [],
            "replans": [],
            "coverage_passed": False,
        }
        for row, candidates in matched:
            timestamp, frame = float(row["monotonic"]), int(row["frame"])
            initial_idle = bool(candidates) and all(
                index == 0 or (index <= len(replans) and int(replans[index - 1]["mode"]) != 1)
                for index in candidates
            )
            if first_merge is not None and timestamp < first_merge and initial_idle:
                report["initial_transition_excluded_samples"] += 1
                continue
            if len(candidates) != 1:
                report["unmatched_samples" if not candidates else "ambiguous_samples"] += 1
                report["samples"].append(
                    {"monotonic": timestamp, "frame": frame, "candidate_buffers": candidates}
                )
                continue
            index = candidates[0]
            replan = replans[index - 1] if 0 < index <= len(replans) else None
            if (
                index not in validated
                or replan is None
                or int(replan["mode"]) != 1
                or float(replan["speed"]) <= 0
                or float(replan["finished_monotonic"]) > timestamp
            ):
                report["contradictory_samples"] += 1
                continue
            telemetry_index = bisect_right(times, timestamp) - 1
            feedback = telemetry[telemetry_index] if telemetry_index >= 0 else None
            healthy = healthy_mode1(feedback, timestamp)
            if not healthy:
                report["unhealthy_or_non_mode1_samples"] += 1
            count = len(blocks[index])
            terminal = sum(frame + offset >= count - 1 for offset in OFFSETS)
            clamped = sum(frame + offset > count - 1 for offset in OFFSETS)
            report["samples"].append(
                {
                    "monotonic": timestamp,
                    "frame": frame,
                    "buffer": index,
                    "timesteps": count,
                    "healthy_mode1": healthy,
                    "terminal_slots": terminal,
                    "clamped_slots": clamped,
                }
            )
            if healthy:
                report["eligible_samples"] += 1
                report["terminal_windows"] += terminal > 0
                report["terminal_windows_ge3"] += terminal >= 3
                report["clamped_windows"] += clamped > 0
                report["max_terminal_slots"] = max(report["max_terminal_slots"], terminal)
                report["max_clamped_slots"] = max(report["max_clamped_slots"], clamped)
        for index, replan in enumerate(replans):
            began = float(replan["started_monotonic"])
            if not start <= began <= end:
                continue
            merged = [
                float(row["monotonic"]) for row, candidates in matched if candidates == [index + 1]
            ]
            report["replans"].append(
                {
                    "buffer": index + 1,
                    "started_monotonic": began,
                    "finished_monotonic": float(replan["finished_monotonic"]),
                    "inference_s": float(replan["finished_monotonic"]) - began,
                    "first_observed_merge_latency_s": min(merged) - began if merged else None,
                    "reasons": [reason for reason in REASONS if replan.get(reason) == "1"],
                    "horizon_remaining": replan.get("horizon_remaining"),
                    "horizon_window": replan.get("horizon_window"),
                    "horizon_margin": replan.get("horizon_margin"),
                }
            )
        for key in (
            "unhealthy_or_non_mode1_samples",
            "unmatched_samples",
            "ambiguous_samples",
            "contradictory_samples",
            "terminal_windows",
        ):
            if report[key]:
                proof_failures.append(key)
        if not report["eligible_samples"]:
            proof_failures.append("no_eligible_samples")
        report["coverage_passed"] = not proof_failures
        reports.append(report)
    names = (
        "results.json",
        "planner-motion.csv",
        "planner-motion.csv.replans.csv",
        "target-motion.csv.frames.csv",
        "telemetry.jsonl",
    )
    return {
        "run": str(path),
        "result_status": results.get("status"),
        "input_sha256": {
            name: hashlib.sha256((path / name).read_bytes()).hexdigest() for name in names
        },
        "buffer_count": len(blocks),
        "replan_count": len(replans),
        "initialization_buffer_surplus": len(blocks) - len(replans),
        "association": "Unique 36-value match; used buffer ordinal validated against consecutive replan completions. Later lifecycle buffers outside selected windows are not paired.",
        "match_atol": MATCH_ATOL,
        "matched_components": 36,
        "future_offsets": OFFSETS,
        "cases": reports,
        "coverage_passed": bool(reports) and all(row["coverage_passed"] for row in reports),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    reports = []
    for path in args.runs:
        try:
            reports.append(analyze_run(path))
        except (OSError, ValueError, KeyError) as error:
            reports.append({"run": str(path), "coverage_passed": False, "error": str(error)})
    args.output.write_text(json.dumps({"runs": reports}, indent=2) + "\n")
    for report in reports:
        print(report["run"], "coverage_passed=", report["coverage_passed"])
        for case in report.get("cases", []):
            print(
                case["name"],
                {
                    key: case[key]
                    for key in (
                        "eligible_samples",
                        "terminal_windows_ge3",
                        "max_terminal_slots",
                        "max_clamped_slots",
                        "unmatched_samples",
                        "coverage_passed",
                    )
                },
            )
    return 0 if all(report["coverage_passed"] for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
