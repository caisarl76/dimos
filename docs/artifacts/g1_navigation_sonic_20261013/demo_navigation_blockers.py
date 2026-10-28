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

"""Closed-loop cases for the planner's no-path retry (two-host simulation).

A wall across the corridor when the goal is sent, lifted after `blocked_s` (session
2026-10-22-5: a transient blocker at goal time). The planner keeps the goal for 3 s,
replanning, with the robot standing (GlobalPlanner retry window); after that it gives
up and the robot must not walk off once the wall is gone.
"""

from collections.abc import Callable
import math
import time
from typing import Any


def blocker_cases(
    goal_run: Callable[..., dict[str, Any]],
    peer: Any,
    corridor: Callable[[tuple[float, float, float]], list[list[float]]],
    rect: Callable[..., list[float]],
    inputs_seen: list[dict[str, Any]],
    case: Callable[..., None],
) -> None:
    def transient_wall(name: str, blocked_s: float, end_s: float | None) -> Any:
        window: dict[str, float] = {}

        def during(track: Any, start: tuple[float, float, float]) -> bool:
            if track is None:
                # Across the whole synthetic map: the corridor is open at both ends, and
                # a planner goes round a wall that ends at its walls (blocker-retry-1).
                wall = rect(start, ahead=(1.0, 1.2), left=(-50.0, 50.0))
                peer.call("synthetic_obstacles", obstacles=[*corridor(start), wall])
                return False
            now = time.monotonic()
            window.setdefault("sent", now)  # first poll, 0.1 s after the goal
            if "lifted" not in window and now - window["sent"] >= blocked_s:
                window["lifted"] = now
                peer.call("synthetic_obstacles", obstacles=corridor(start))
            return end_s is not None and now - window["sent"] >= end_s

        run = goal_run(name, 3.5, during)
        commanded = [
            r["monotonic"] - window["sent"]
            for r in inputs_seen
            if r["scenario"] == name
            and r["monotonic"] >= window.get("sent", math.inf)
            and (r["vx"] != 0.0 or r["wz"] != 0.0)
        ]
        blocked = window.get("lifted", math.inf) - window.get("sent", 0.0)
        return run, [t for t in commanded if t < blocked], commanded

    run, while_blocked, commanded = transient_wall("transient_blocker_waits", 2.0, None)
    case(
        "transient_blocker_waits_then_walks",
        run["accepted"] is True
        and not while_blocked
        and bool(commanded)
        and run["reached_s"] is not None,
        commanded_while_blocked=len(while_blocked),
        first_command_s=commanded[0] if commanded else None,
        reached_s=run["reached_s"],
        plans=run["plans"],
    )
    run, _, commanded = transient_wall("long_blocker_gives_up", 5.0, 9.0)
    walked = max((math.dist(p, run["start"][:2]) for p in run["track"]), default=0.0)
    case(
        "long_blocker_gives_up_and_stays",
        run["accepted"] is True
        and not commanded
        and walked < 0.05
        and run["reached_s"] is None
        and run["stopped_at_end"],
        commanded=len(commanded),
        walked_m=walked,
        plans=run["plans"],
    )
