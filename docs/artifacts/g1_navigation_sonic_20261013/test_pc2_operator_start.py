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

"""Operator start helper against an in-process fake native controller (no robot)."""

import socket
import threading
import time

import msgpack
import pytest
import zmq

from dimos.robot.unitree.g1.sonic_reference import encode_command
from docs.artifacts.g1_navigation_sonic_20261013 import demo_pc2_operator_start as start


def free_endpoint() -> str:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return f"tcp://127.0.0.1:{s.getsockname()[1]}"


def feedback_packet(index: int, *, confirmed: bool) -> bytes:
    return b"g1_debug" + msgpack.packb(
        {
            "index": index,
            "ros_timestamp": 0.0,
            "body_q": [0.0] * 29,
            "body_dq": [0.0] * 29,
            "left_hand_q": [0.0] * 7,
            "left_hand_dq": [0.0] * 7,
            "right_hand_q": [0.0] * 7,
            "right_hand_dq": [0.0] * 7,
            "planner_guard": {
                "mode": 0,
                "high_water": 0,
                "applied_sequence": 0,
                "rearm_barrier": 0,
                "lease_armed": False,
                "latched": False,
                "idle_ack_sequence": 0,
                "fault": None,
            },
            "reference_guard": {
                "armed": not confirmed,
                "expired": False,
                "planner_selected": confirmed,
                "planner_confirmed": confirmed,
                "rearm_requires_planner": False,
                "accepted_index": 0,
                "applied_index": 0,
                "rejected": 0,
                "epoch": 1,
                "age_ms": 0.0,
            },
        }
    )


class FakeNative(threading.Thread):
    """Subscribes to commands; after a start, publishes planner feedback."""

    def __init__(self, reference: str, feedback: str, *, confirms: bool) -> None:
        super().__init__(daemon=True)
        self.reference, self.feedback, self.confirms = reference, feedback, confirms
        self.commands: list[bytes] = []
        self.stop = threading.Event()
        self.bound = threading.Event()

    def run(self) -> None:
        with zmq.Context() as ctx, ctx.socket(zmq.SUB) as sub, ctx.socket(zmq.PUB) as pub:
            pub.bind(self.feedback)
            self.bound.set()
            sub.setsockopt(zmq.SUBSCRIBE, b"command")
            sub.connect(self.reference)
            index = 0
            while not self.stop.is_set():
                if sub.poll(10):
                    self.commands.append(sub.recv())
                index += 1
                pub.send(feedback_packet(index, confirmed=self.confirms and bool(self.commands)))


def run_native(confirms: bool) -> tuple[FakeNative, str, str]:
    reference, feedback = free_endpoint(), free_endpoint()
    native = FakeNative(reference, feedback, confirms=confirms)
    native.start()
    native.bound.wait(2)
    return native, reference, feedback


def test_sends_one_start_and_returns_after_three_confirmed_samples() -> None:
    native, reference, feedback = run_native(confirms=True)
    try:
        result = start.start_idle_planner(reference, feedback, timeout=5.0)
    finally:
        native.stop.set()
    assert native.commands == [encode_command(start=True, stop=False, planner=True)]
    assert result["reference_guard"]["planner_confirmed"] is True
    assert result["planner_guard"]["mode"] == 0 and not result["planner_guard"]["latched"]


def test_unconfirmed_planner_times_out_after_the_single_start() -> None:
    native, reference, feedback = run_native(confirms=False)
    try:
        with pytest.raises(TimeoutError, match="planner confirmation"):
            start.start_idle_planner(reference, feedback, timeout=0.5)
    finally:
        native.stop.set()
    assert len(native.commands) == 1


def test_no_subscriber_means_no_start() -> None:
    started = time.monotonic()
    with pytest.raises(TimeoutError, match="no start sent"):
        start.start_idle_planner(free_endpoint(), free_endpoint(), timeout=0.3)
    assert time.monotonic() - started < 2


def test_preview_opens_no_sockets(monkeypatch, capsys) -> None:
    monkeypatch.setattr(start.zmq, "Context", None)  # any socket use would fail
    assert start.main([]) == 0
    assert "Preview only" in capsys.readouterr().out
