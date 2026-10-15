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

"""pc2 side of the supervised walk harness, attached to the running graph container.

Run with `docker exec -i <graph container> python .../demo_pc2_walk_agent.py`. It joins
the graph's private zenoh router as a client and reaches the one execution owner by RPC.
It only calls the explicit begin/end navigation RPCs and read-only status; motion
commands enter through the workstation peer. JSON lines: commands on stdin, replies
on stdout. Native state is sampled at 25 Hz into memory and `--log`.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable
import json
from pathlib import Path
import sys
import threading
import time
from typing import Any

from dimos.robot.unitree.g1.sonic_navigation import _yaw

SAMPLE_PERIOD_S = 0.04
ROUTER = "tcp/127.0.0.1:17111"


class Agent:
    def __init__(self, execution: Any, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._execution = execution
        self._clock = clock
        self._rows: deque[dict[str, Any]] = deque(maxlen=int(900 / SAMPLE_PERIOD_S))
        self._lock = threading.Lock()

    def sample(self) -> dict[str, Any]:
        feedback = self._execution.get_feedback()
        guard = None if feedback is None else feedback.planner_guard
        nav = self._execution.get_navigation_status()
        row = {
            "monotonic": self._clock(),
            "feedback": feedback is not None,
            "index": None if feedback is None else feedback.index,
            "feedback_monotonic": None if feedback is None else feedback.received_monotonic,
            "yaw": None if feedback is None else _yaw(feedback.base_quat_measured),
            "mode": None if guard is None else guard.mode,
            "applied_sequence": None if guard is None else guard.applied_sequence,
            "high_water": None if guard is None else guard.high_water,
            "latched": None if guard is None else guard.latched,
            "lease_armed": None if guard is None else guard.lease_armed,
            "guard_fault": None if guard is None else guard.fault,
            "phase": str(nav["phase"]),
            "nav_fault": nav["fault"],
            "pose_ready": nav["pose_ready"],
        }
        with self._lock:
            self._rows.append(row)
        return row

    def handle(self, command: dict[str, Any]) -> dict[str, Any]:
        op = command.get("op")
        reply: dict[str, Any] = {"op": op, "monotonic": self._clock()}
        try:
            if op == "begin":
                reply["result"] = self._execution.begin_navigation()
            elif op == "end":
                reply["result"] = self._execution.end_navigation()
            elif op == "status":
                reply["navigation"] = self._execution.get_navigation_status()
                status = self._execution.get_status()
                reply.update(
                    {k: status[k] for k in ("ready", "subscriptions_ready", "feedback_fresh")}
                )
            elif op == "mark":
                pass
            elif op == "since":
                with self._lock:
                    reply["rows"] = [r for r in self._rows if r["monotonic"] > command["monotonic"]]
            else:
                reply["error"] = f"unknown op: {op}"
        except Exception as error:  # RPC refusals are results the harness records
            reply["error"] = f"{type(error).__name__}: {error}"
        return reply


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    from dimos.core.global_config import global_config
    from dimos.core.rpc_client import RPCClient
    from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule

    global_config.update(
        transport="zenoh",
        zenoh_mode="client",
        zenoh_connect=ROUTER,
        zenoh_multicast=False,
        zenoh_scouting=False,
        zenoh_gossip=True,
        serve_coordinator_rpc=False,
    )
    execution = RPCClient.remote(SonicExecutionModule)
    agent = Agent(execution)
    stop = threading.Event()

    def sampler() -> None:
        with args.log.open("a") as log:
            while not stop.wait(SAMPLE_PERIOD_S):
                try:
                    log.write(json.dumps(agent.sample()) + "\n")
                except Exception as error:
                    log.write(
                        json.dumps({"monotonic": time.monotonic(), "error": str(error)}) + "\n"
                    )

    threading.Thread(target=sampler, daemon=True).start()
    print(json.dumps(agent.handle({"op": "status"})), flush=True)
    try:
        for line in sys.stdin:
            command = json.loads(line)
            if command.get("op") == "quit":
                break
            print(json.dumps(agent.handle(command), default=str), flush=True)
    finally:
        stop.set()
        execution.stop_rpc_client()


if __name__ == "__main__":
    main()
