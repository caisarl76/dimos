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

"""Deterministic checks for the experiment's fixed physics-step schedule."""

import unittest

from demo_navigation_timing import FixedStepSchedule


class FixedStepScheduleChecks(unittest.TestCase):
    def test_sleep_overshoot_does_not_accumulate_simulation_lag(self):
        schedule = FixedStepSchedule(0.0, 0.005)
        wall_time = 0.0
        for _ in range(200):
            wall_time += 0.0002  # Physics and recording work.
            delay = schedule.delay_after_step(wall_time)
            wall_time += delay + 0.0003  # Repeatable OS sleep overshoot.
        self.assertAlmostEqual(wall_time, 1.0003)

    def test_slow_step_catches_up_without_changing_physics_step(self):
        schedule = FixedStepSchedule(4.0, 0.005)
        self.assertEqual(schedule.delay_after_step(4.012), 0.0)
        self.assertEqual(schedule.delay_after_step(4.012), 0.0)
        self.assertAlmostEqual(schedule.delay_after_step(4.012), 0.003)
        self.assertAlmostEqual(schedule.scheduled_at, 4.015)


if __name__ == "__main__":
    unittest.main()
