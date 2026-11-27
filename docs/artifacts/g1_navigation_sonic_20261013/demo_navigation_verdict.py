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

"""Score every original transition case, including diagnostic-only traces."""

import math
from typing import Any


def transition_verdict(cases: list[dict[str, Any]], cycles: int) -> bool:
    expected = {"initial_idle_settled"}
    for index in range(cycles):
        expected.update({f"forward_transition_{index}", f"forward_transition_{index}_settled"})
    if len(cases) != len(expected) or {case.get("name") for case in cases} != expected:
        return False
    for case in cases:
        if case.get("passed") is not True:
            return False
        if case["name"].endswith("_settled"):
            numbers = [case.get(key) for key in ("elapsed", "speed", "yaw_rate")]
            if not all(type(value) in (int, float) and math.isfinite(value) for value in numbers):
                return False
            elapsed, speed, yaw_rate = numbers
            if not (0 <= elapsed <= 3.0 and 0 <= speed < 0.03 and abs(yaw_rate) < 0.03):
                return False
        else:
            value = case.get("forward_displacement_m")
            if type(value) not in (int, float) or not math.isfinite(value) or value * 0.2 <= 0.001:
                return False
    return True
