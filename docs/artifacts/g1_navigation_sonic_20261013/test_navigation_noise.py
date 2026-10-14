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

"""Noise proof must reject unaccounted motion changes, even with healthy leases."""

from dataclasses import replace
import math
import struct

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_navigation import PlannerIntent, encode_planner
from docs.artifacts.g1_navigation_sonic_20261013.demo_analyze_navigation import (
    attribute_replans,
    conversion_allowed,
)
from docs.artifacts.g1_navigation_sonic_20261013.demo_navigation_noise import (
    FixedHeadingFixture,
)


@pytest.fixture
def feedback():
    return SonicFeedback(
        1,
        None,
        10.0,
        10.0,
        (0.0,) * 29,
        (0.0,) * 29,
        (0.0,) * 7,
        (0.0,) * 7,
        (0.0,) * 7,
        (0.0,) * 7,
        base_quat_measured=(1.0, 0.0, 0.0, 0.0),
        reference_heading_quat=(1.0, 0.0, 0.0, 0.0),
        planner_reference_active=True,
    )


def test_fixed_heading_fixture_preserves_native_lease_fields(feedback):
    fixture = FixedHeadingFixture()
    expected = None
    for source in range(50):
        yaw = source * 0.02
        actual = replace(feedback, base_quat_measured=(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        velocity = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, 0.005 * (-1) ** source))
        for heartbeat in range(5):
            original = encode_planner(
                PlannerIntent(1, (1, 0, 0), (1, 0, 0), 0.2),
                sequence=source * 5 + heartbeat + 1,
                deadline=10.2,
            )
            converted, record = fixture.convert(original, velocity, actual, source_sequence=source)
            assert converted[1323:] == original[1323:]
            assert record["kind"] == "fixed_heading_fixture"
            if expected is None:
                expected = converted[1287:1323]
            assert converted[1287:1323] == expected


@pytest.fixture
def evidence():
    packet = encode_planner(PlannerIntent(1, (1, 0, 0), (1, 0, 0), 0.2), sequence=1, deadline=10.25)
    return {
        "sent": [{"monotonic": 10.01, "source_sequence": 1, "packet_hex": packet.hex()}],
        "acks": [{"monotonic": 10.03, "sequence": 1}],
        "native": [
            {
                "started_monotonic": "10.04",
                "finished_monotonic": "10.045",
                "mode": "1",
                "speed": str(struct.unpack("<f", struct.pack("<f", 0.2))[0]),
                "movement_x": "1",
                "movement_y": "0",
                "facing_x": "1",
                "facing_y": "0",
                "mode_changed": "1",
                "direction_changed": "1",
                "speed_changed": "1",
                "facing_changed": "0",
                "height_changed": "0",
            }
        ],
    }


def test_native_replan_requires_an_acknowledged_intent(evidence):
    assert attribute_replans(**evidence)["passed"]
    evidence["acks"].clear()
    assert not attribute_replans(**evidence)["passed"]


def test_native_replan_cannot_match_a_different_motion(evidence):
    evidence["native"][0]["movement_x"] = "0"
    evidence["native"][0]["movement_y"] = "1"
    assert not attribute_replans(**evidence)["passed"]


def test_native_replan_cannot_use_an_expired_intent(evidence):
    original = bytes.fromhex(evidence["sent"][0]["packet_hex"])
    evidence["sent"][0]["packet_hex"] = (original[:-8] + struct.pack("<d", 10.02)).hex()
    assert not attribute_replans(**evidence)["passed"]


def test_unchanged_state_cannot_explain_a_second_native_replan(evidence):
    first = evidence["native"][0]
    periodic = {
        **first,
        "started_monotonic": "10.05",
        "finished_monotonic": "10.055",
        "mode_changed": "0",
        "speed_changed": "0",
        "direction_changed": "0",
    }
    duplicate = {
        **periodic,
        "started_monotonic": "10.06",
        "finished_monotonic": "10.065",
        "direction_changed": "1",
    }
    evidence["native"].extend([periodic, duplicate])
    assert not attribute_replans(**evidence)["passed"]


def test_native_replan_cannot_be_explained_by_a_later_send(evidence):
    evidence["sent"][0]["monotonic"] = 10.043
    evidence["acks"][0]["monotonic"] = 10.044
    assert not attribute_replans(**evidence)["passed"]


def horizon_row(evidence):
    return {
        **evidence["native"][0],
        "started_monotonic": "10.05",
        "finished_monotonic": "10.055",
        "mode_changed": "0",
        "speed_changed": "0",
        "direction_changed": "0",
        "periodic": "0",
        "horizon_refresh": "1",
        "horizon_remaining": "54",
        "horizon_window": "46",
        "horizon_margin": "8",
    }


def test_horizon_only_refresh_has_its_own_validated_reason(evidence):
    evidence["native"].append(horizon_row(evidence))
    assert attribute_replans(**evidence)["passed"]


@pytest.mark.parametrize(
    "field,value",
    [
        ("horizon_remaining", "55"),
        ("horizon_window", "0"),
        ("horizon_margin", "1000"),
        ("mode", "0"),
        ("speed", "0"),
    ],
)
def test_invalid_horizon_claim_cannot_explain_extra_inference(evidence, field, value):
    row = horizon_row(evidence)
    row[field] = value
    evidence["native"].append(row)
    assert not attribute_replans(**evidence)["passed"]


def test_horizon_reason_cannot_hide_an_unchanged_state_replan(evidence):
    row = horizon_row(evidence)
    row["direction_changed"] = "1"
    evidence["native"].append(row)
    assert not attribute_replans(**evidence)["passed"]


def test_native_inference_without_any_recorded_reason_is_rejected(evidence):
    row = horizon_row(evidence)
    row["horizon_refresh"] = "0"
    evidence["native"].append(row)
    assert not attribute_replans(**evidence)["passed"]


def test_same_source_heartbeat_cannot_change_intent(evidence):
    packet = encode_planner(PlannerIntent(1, (0, 1, 0), (1, 0, 0), 0.2), sequence=2, deadline=10.25)
    evidence["sent"].append({"monotonic": 10.02, "source_sequence": 1, "packet_hex": packet.hex()})
    assert not attribute_replans(**evidence)["passed"]


def test_direction_change_below_threshold_is_rejected():
    before = PlannerIntent(1, (1, 0, 0), (1, 0, 0), 0.2)
    after = PlannerIntent(1, (math.cos(0.01), math.sin(0.01), 0), (1, 0, 0), 0.2)
    row = {
        "previous_payload": encode_planner(before)[1287:1323].hex(),
        "payload_hex": encode_planner(after)[1287:1323].hex(),
        "base_quat": [math.cos(0.005), 0, 0, math.sin(0.005)],
        "reference_quat": [1, 0, 0, 0],
        "reference_before": 0,
        "stopped_before": False,
        "velocity": [0.2, 0, 0, 0, 0, 0.005],
    }
    assert not conversion_allowed(row)
    row["stopped_before"] = True
    assert conversion_allowed(row)
