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

"""The private link bounds ages without comparing remote absolute clocks."""

from dataclasses import replace
import json
import math

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.sonic_navigation_link import (
    NavigationPollReply,
    bound_navigation_input,
    decode_navigation_message,
    decode_poll_payload,
    encode_navigation_message,
    encode_poll_payload,
)
from dimos.robot.unitree.g1.sonic_navigation_readiness import (
    INPUT_BUDGETS,
    InputProgress,
    ReadinessSnapshot,
)


@pytest.fixture
def readiness():
    return ReadinessSnapshot(
        True, tuple(InputProgress(n, 10000.0, 0.0, 0.0, True) for n in INPUT_BUDGETS), None
    )


@pytest.fixture
def reply(readiness):
    return NavigationPollReply(
        "poll-1", "request-1", 7, Twist(linear=Vector3(0.2, 0, 0)), 0.12, readiness
    )


def test_unsynchronized_clock_deadline(reply):
    value = bound_navigation_input(reply, t0=10.0, t1=10.08)
    assert value.velocity_deadline == pytest.approx(10.23)
    assert value.readiness_deadline == pytest.approx(10.2)
    assert value.ready and value.twist.linear.x == 0.2 and value.source_sequence == 7


@pytest.mark.parametrize("age", [-1, math.nan, math.inf, True, 10**1000])
def test_invalid_source_age_is_rejected(reply, age):
    with pytest.raises(ValueError):
        bound_navigation_input(replace(reply, source_age=age), t0=10.0, t1=10.08)


@pytest.mark.parametrize("t0,t1", [(10.0, 10.101), (10.0, 9.99), (math.nan, 10.0)])
def test_invalid_round_trip_is_rejected(reply, t0, t1):
    with pytest.raises(ValueError):
        bound_navigation_input(reply, t0=t0, t1=t1)


def test_expired_velocity_keeps_sensor_readiness_for_recoverable_idle(reply):
    value = bound_navigation_input(replace(reply, source_age=0.3), t0=10.0, t1=10.08)
    assert value.velocity_deadline == pytest.approx(10.05)
    assert value.ready


def test_incomplete_or_expired_readiness_never_opens_admission(reply, readiness):
    incomplete = replace(readiness, inputs=readiness.inputs[:-1])
    assert not bound_navigation_input(replace(reply, readiness=incomplete), t0=10, t1=10.08).ready
    expired = replace(readiness, inputs=tuple(replace(p, age=0.19) for p in readiness.inputs))
    value = bound_navigation_input(replace(reply, readiness=expired), t0=10, t1=10.08)
    assert not value.ready and value.readiness_deadline <= 10.08


def test_atomic_bound_owns_velocity(reply):
    value = bound_navigation_input(reply, t0=10, t1=10.08)
    reply.twist.linear.x = -0.2
    assert value.twist.linear.x == 0.2


def test_wire_reply_preserves_source_age_and_exact_velocity(reply):
    payload = encode_poll_payload(reply)
    restored = decode_poll_payload(payload, correlation_id="poll-1")
    assert restored.request_id == "request-1" and restored.source_sequence == 7
    assert restored.source_age == 0.12 and restored.twist.linear.x == 0.2
    assert restored.readiness.inputs[0].source_stamp == 10000.0


@pytest.fixture
def poll_message():
    return {
        "version": 1,
        "operation": "poll",
        "correlation_id": "poll-1",
        "request_id": None,
        "reply": False,
        "payload": {},
    }


def test_single_bounded_json_message(poll_message):
    packet = encode_navigation_message(poll_message)
    assert decode_navigation_message(packet) == poll_message
    with pytest.raises(ValueError):
        decode_navigation_message(b" " * 65537)


@pytest.mark.parametrize(
    "field,bad",
    [
        ("version", 2),
        ("version", True),
        ("operation", "move"),
        ("reply", 1),
        ("correlation_id", ""),
        ("request_id", 3),
    ],
)
def test_invalid_envelope_cannot_be_a_reply(poll_message, field, bad):
    poll_message[field] = bad
    with pytest.raises(ValueError):
        decode_navigation_message(json.dumps(poll_message).encode())


def test_duplicate_json_keys_and_unknown_fields_are_rejected(poll_message):
    packet = json.dumps(poll_message).replace('"version": 1', '"version": 1, "version": 2')
    with pytest.raises(ValueError):
        decode_navigation_message(packet.encode())
    poll_message["deadline"] = 10000.0
    with pytest.raises(ValueError):
        encode_navigation_message(poll_message)


@pytest.mark.parametrize(
    "velocity", [[0.2] * 5, [0.2] * 7, [0.2, 0, 0, 0, 0, math.nan], [True, 0, 0, 0, 0, 0]]
)
def test_nonfinite_or_malformed_velocity_is_rejected(reply, velocity):
    payload = encode_poll_payload(reply)
    payload["twist"] = velocity
    with pytest.raises(ValueError):
        decode_poll_payload(payload, correlation_id="poll-1")
