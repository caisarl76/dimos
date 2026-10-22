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

"""Operator-only native planner start for the navigation controller; preview by default.

Same protocol as the pc2-proven manipulation helper (start-idle-planner.py): bind the
reference endpoint before the graph does, wait for the native command subscription,
send one {start, planner} command, then require three increasing feedback samples
with the planner confirmed and the navigation guard in IDLE, unlatched. It exits;
the native stays in planner IDLE. Run it only through `launch.sh --commission ...
start`, which verifies the controller is this task's binary and the graph is down.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import time
from typing import Any

import zmq

from dimos.robot.unitree.g1.sonic_feedback import TOPIC, SonicFeedback, decode_feedback
from dimos.robot.unitree.g1.sonic_reference import encode_command

REFERENCE = "tcp://127.0.0.1:5556"
FEEDBACK = "tcp://127.0.0.1:5557"


def confirmed(state: SonicFeedback) -> bool:
    guard, planner = state.reference_guard, state.planner_guard
    return (
        guard is not None
        and guard.planner_selected
        and guard.planner_confirmed
        and not guard.armed
        and not guard.expired
        and not guard.rearm_requires_planner
        and planner is not None
        and planner.mode == 0
        and not planner.latched
    )


def start_idle_planner(reference: str, feedback: str, timeout: float = 10.0) -> dict[str, Any]:
    context = zmq.Context()
    publisher, subscriber = context.socket(zmq.XPUB), context.socket(zmq.SUB)
    try:
        for sock in (publisher, subscriber):
            sock.setsockopt(zmq.LINGER, 0)
        subscriber.setsockopt(zmq.RCVHWM, 20)
        subscriber.setsockopt(zmq.SUBSCRIBE, TOPIC)
        subscriber.connect(feedback)
        publisher.bind(reference)  # refuses while another reference publisher holds it
        deadline = time.monotonic() + timeout
        while not (publisher.poll(100) and publisher.recv() == b"\x01command"):
            if time.monotonic() >= deadline:
                raise TimeoutError("no native command subscriber; no start sent")
        for _ in range(100):  # feedback queued before the request says nothing about it
            if not subscriber.poll(0):
                break
            subscriber.recv()
        else:
            raise RuntimeError("feedback backlog did not drain; no start sent")
        publisher.send(encode_command(start=True, stop=False, planner=True))
        confirmations, last_index = 0, -1
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not subscriber.poll(100):
                continue
            state = decode_feedback(
                subscriber.recv(), received_at=time.time(), received_monotonic=time.monotonic()
            )
            if state.index <= last_index:
                raise RuntimeError("feedback index did not advance")
            last_index = state.index
            confirmations = confirmations + 1 if confirmed(state) else 0
            if confirmations == 3:
                assert state.reference_guard is not None and state.planner_guard is not None
                return {
                    "feedback_index": state.index,
                    "reference_guard": asdict(state.reference_guard),
                    "planner_guard": asdict(state.planner_guard),
                }
        raise TimeoutError("start sent, but planner confirmation timed out; inspect SONIC")
    finally:
        publisher.close()
        subscriber.close()
        context.term()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--start-idle-planner",
        action="store_true",
        help="Operator action: start native policy/planner control; the robot can move",
    )
    args = parser.parse_args(argv)
    print('Command: {"start": true, "stop": false, "planner": true}')
    if not args.start_idle_planner:
        print("Preview only; no sockets opened. --start-idle-planner starts native control.")
        return 0
    print(json.dumps(start_idle_planner(REFERENCE, FEEDBACK), indent=2))
    print("Planner confirmed in IDLE. SONIC stays running; start the graph next.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, TimeoutError, ValueError, zmq.ZMQError) as exc:
        raise SystemExit(
            f"Start failed: {exc}. No stop is sent automatically; the operator owns shutdown."
        ) from exc
