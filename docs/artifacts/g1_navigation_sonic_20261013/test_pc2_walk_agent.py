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

"""pc2 walk agent: explicit begin/end only, sampled native state, errors as replies."""

import math
from types import SimpleNamespace

from docs.artifacts.g1_navigation_sonic_20261013 import demo_pc2_walk_agent as agent


class FakeExecution:
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.phase = "disarmed"
        self.feedback: object | None = SimpleNamespace(
            index=7,
            received_monotonic=99.0,
            base_quat_measured=(math.cos(0.25), 0.0, 0.0, math.sin(0.25)),  # yaw 0.5
            planner_guard=SimpleNamespace(
                mode=1,
                applied_sequence=12,
                high_water=12,
                latched=False,
                lease_armed=True,
                fault=None,
            ),
        )

    def begin_navigation(self) -> dict:
        self.calls.append("begin")
        if self.phase != "disarmed":
            raise RuntimeError("fresh navigation input and all native subscriptions required")
        self.phase = "rearming"
        return {"phase": self.phase}

    def end_navigation(self) -> dict:
        self.calls.append("end")
        self.phase = "stopping"
        return {"phase": self.phase}

    def get_navigation_status(self) -> dict:
        return {"phase": self.phase, "fault": None, "pose_ready": True}

    def get_status(self) -> dict:
        return {"ready": True, "subscriptions_ready": True, "feedback_fresh": True}

    def get_feedback(self) -> object | None:
        return self.feedback


def test_begin_end_and_rpc_errors_become_replies() -> None:
    execution = FakeExecution()
    walk = agent.Agent(execution, clock=lambda: 5.0)
    assert walk.handle({"op": "begin"})["result"] == {"phase": "rearming"}
    failed = walk.handle({"op": "begin"})
    assert "subscriptions required" in failed["error"]
    assert walk.handle({"op": "end"})["result"] == {"phase": "stopping"}
    assert execution.calls == ["begin", "begin", "end"]
    assert walk.handle({"op": "move"})["error"] == "unknown op: move"


def test_samples_native_guard_and_measured_yaw_and_filters_by_mark() -> None:
    now = [10.0]
    walk = agent.Agent(FakeExecution(), clock=lambda: now[0])
    walk.sample()
    mark = walk.handle({"op": "mark"})["monotonic"]
    now[0] = 10.04
    row = walk.sample()
    assert row["mode"] == 1 and row["lease_armed"] and row["applied_sequence"] == 12
    assert abs(row["yaw"] - 0.5) < 1e-9 and row["phase"] == "disarmed"
    assert [r["monotonic"] for r in walk.handle({"op": "since", "monotonic": mark})["rows"]] == [
        10.04
    ]


def test_missing_feedback_is_recorded_not_invented() -> None:
    execution = FakeExecution()
    execution.feedback = None
    row = agent.Agent(execution, clock=lambda: 1.0).sample()
    assert row["feedback"] is False and row["mode"] is None and row["yaw"] is None
