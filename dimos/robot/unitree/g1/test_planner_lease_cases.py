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

import json
from pathlib import Path

import pytest

from dimos.robot.unitree.g1.test_utils.sonic_planner_lease import PlannerLease, PlannerPacket

_CASES = json.loads(
    (Path(__file__).parent / "test_data/sonic_planner_lease_cases.json").read_text()
)


@pytest.mark.parametrize("case", _CASES["cases"], ids=lambda case: case["name"])
def test_shared_native_deadline_sequence_oracle(case):
    lease = PlannerLease()
    for step in case["steps"]:
        action = step["action"]
        now, seq, deadline = (
            step.get("now", 0.0),
            step.get("sequence", -1),
            step.get("deadline", 0.0),
        )
        accepted = False
        if action == "rearm":
            accepted = lease.rearm(seq, deadline, now)
        elif action == "end":
            accepted = lease.end(seq, deadline, now)
        elif action == "install":
            accepted = lease.install(now)
        elif action == "tick":
            lease.tick(now)
        elif action == "pose":
            lease.enter_pose()
        elif action == "handback":
            lease.handback(now)
        elif action == "ack_end":
            lease.acknowledge_end()
        elif action in {"idle", "walk", "legacy"}:
            packet = (
                PlannerPacket()
                if action == "legacy"
                else PlannerPacket(
                    sequence=seq,
                    deadline=deadline,
                    mode=1 if action == "walk" else 0,
                    movement=(1.0, 0.0, 0.0) if action == "walk" else (0.0, 0.0, 0.0),
                    speed=0.2 if action == "walk" else 0.0,
                )
            )
            accepted = lease.accept(packet, now)
        else:
            raise AssertionError(f"unknown action: {action}")
        if "accepted" in step:
            assert accepted == step["accepted"], step
        for name, value in step["expect"].items():
            assert getattr(lease.status, name) == value, step
