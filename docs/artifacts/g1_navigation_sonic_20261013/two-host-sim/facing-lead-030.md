# Facing-lead cap 0.3 rad against 0.2 rad (two-host simulation, 2026-10-29)

`SONIC_STUDY_FACING_CAP=0.3 SONIC_STUDY_FACING_LOOKAHEAD=1.5 run_two_host.sh <name>`.
The 1.5 s lookahead lets the lead reach 0.3 rad at the 0.2 rad/s yaw cap. The baseline
is the production 0.2 rad cap at the same code: `regress-54df8667d`, `blocker-retry-3`,
`blocker-retry-4`.

| | 0.2 rad (3 runs) | 0.3 rad (3 runs) |
|---|---|---|
| Yaw after 2 s at +0.2 rad/s (0.40 commanded) | 0.39-0.44 | 0.56-0.63 |
| Yaw after 2 s at -0.2 rad/s | -0.29 | -0.52 to -0.59 |
| `left_turn_settled` (gate 4.5 s) | 2.6, 4.3, 7.7 | 3.2, 7.9, 2.8 |
| `corridor_detour` reached | 22.3-25.0 s, 3/3 | 21.4 s, 1/3 |
| `agent_walk_to` reached | 11.8-12.1 s | 10.3 s (1 run got there) |
| Falls (automatic resets) | 0 | 14 in run 1 |

- **Run 1** fell in `corridor_detour` and aborted after 85 cases. VS Code started on the
  host at the same moment, and missed physics deadlines jumped from 731 to 1010 in the
  3 s before the fall. A quiet run misses about 500 in total.
- **Run 2:** 100/101; only `left_turn_settled` failed (7.85 s), the same noisy gate
  that failed at 0.2 rad in `blocker-retry-4`.
- **Run 3** stopped in `corridor_detour` with `readiness_expired`. The replayed IMU and
  lidar from the pc2 sender went stale for 2 s, upstream of anything the cap changes,
  and the robot stopped as designed.

At 0.3 rad the robot turns about 45% (left) to 90% (right) further than commanded in
2 s; at 0.2 rad it follows the command. Goals are not reached meaningfully faster, and
the one fall, though under host load, is not seen at 0.2 rad. The simulation gives no
reason to raise the cap. The real G1 turns in place far slower than the simulation
(2-3 deg/s), so whether more lead helps there is a robot question.
