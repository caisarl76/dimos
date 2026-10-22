# Two-G1 arm sync: first hardware run (2026-10-22)

pc2_222 (192.168.0.222, Dex3 hands active) and pc2_223 (192.168.0.223, no hands), both on stands, driven
from one workstation DimOS process over SSH tunnels. Branch `jihun/feat/g1-dual-sonic-arm` at `48fdfeabd`:
Tier-1 `aa798e221` and forearm `0d664204e` merged in. Every result line is in `results.jsonl`, extracted
from the run logs.

## Setup used

- Controllers, started by the operator: guarded builds on each PC2.
  - 223: `~/dimos-sonic-guard-pc2-223-20261014/launch-guarded.sh --run`, which checks that the clocks are pinned.
  - 222: `sudo jetson_clocks`, then `bash ~/dimos-sonic-guard-20261014/launch-guarded.sh --run`. This was the
    first physical run of 222's guarded build.
  - On both: `start-idle-planner.py --start-idle-planner`, which reported "Planner confirmed".
- Tunnels, opened after planner idle: `ssh -N -o ExitOnForwardFailure=yes … -R 5556:127.0.0.1:1x556
  -L 1x557:127.0.0.1:5557`, to `pc2_222` and to `pc2` (223's ssh alias).
- DimOS: `unitree-g1-sonic-arm-dual` (MCP), then `unitree-g1-sonic-arm-dual-agentic` (gpt-5.6-luna via
  Genon), with `--dualsonicarmskillcontainer.max-dq=2.0`. The readiness check showed both robots ready, with
  feedback ages of 4–22 ms.

## Results

| # | Command | Result | submit / start skew |
|---|---|---|---|
| 1 | `move_left_arm_pose(both)` with 222 not yet up | "222: SONIC is not in confirmed planner idle; nothing sent; nothing sent to any robot" | – |
| 2 | 223 alone: `move_upper_body` 10°, side pose, wave | 7.0°; 93 %; 96 %; all returned | – |
| 3 | `move_upper_body(222, 10°)` | 6.9°, returned (222's first guarded motion) | – |
| 4 | `move_upper_body(both, 10°)` | 5.3° / 7.9°, returned | 0.000 / **0.045 s** |
| 5 | `move_upper_body(both, 28°)` | 24.8° / 26.4°, returned | 0.000 / **0.056 s** |
| 6 | `move_left_arm_pose(both, side)` ×2 | 87–88 % / 91 %, returned | 0.001 / 2.271 s and 0.000 / 1.260 s ⚠ |
| 7 | `perform_gesture(both, wave)` | 94 % / 94 %, returned | 0.000 / 2.496 s ⚠ |
| 8 | `move_upper_body(both, 20°)`, then `stop_arm` | both "stopped early: stop requested", returned | 0.001 / **0.017 s** |
| 9 | `move_left_arm_pose(both, side, hold 2 s)` | 87 % / 93 %, returned; 49 / 50 hold samples | 0.000 / 0.188 s |
| 10 | `perform_gesture(both, wave)`, then `stop_arm` | both "stopped early: stop requested", returned | 0.000 / 1.754 s ⚠ |
| 11 | `perform_gesture(222, raise_hand)` | "stopped early: left_knee left its envelope (+0.152 rad)" at 71 % (pitch −107.9° of −149°); retreated, handed back | – |
| 12 | agent: `dimos agent-send "both robots wave"` | the agent called `perform_gesture(robot="both", name="wave")`; chunk 0 accepted 5 ms apart on the two robots (journals); `stop_arm` at +10.4 s retreated both within 6 ms; both handed back at 18:28:57.2. `dimos stop` 2.6 s later fell inside the 3 s return watch, so no result line was logged | journals: 0.005 s |

⚠ The logged `start_skew_s` takes the first planned joint to move 0.03 rad, often a secondary joint drifting
during the 2 s token blend before the stroke. The robot that "moved first" switches between runs, and the
operator saw the arms move together. For single-joint strokes (rows 4, 5, 8) the number is meaningful:
17–56 ms. Commands reached both controllers within 0–6 ms in every run (submit skew and journal chunk times).
A progress-based onset replaces this metric (fix in progress).

## Notes

- **222 reach:** 222 consistently reaches less than 223 (53–89 % vs 79–94 %). It has heavier Dex3 hands; the
  envelopes held.
- **222 right wrist:** 222's right wrist roll reached 2.05 rad/s once during a return. That was the hand
  touching the leg; the operator judged it not critical.
- **raise_hand on 222:** it trips the 0.15 rad knee envelope while SONIC balances the hand-equipped arm by
  sitting into the left leg (Tier-1 analysis). The next check is one `raise_hand` on 223, to separate the
  gesture from the hands.
- **Hand collisions unchecked on 222:** `ArmCollisionChecker` ignores the hand geoms (it was built for 223
  without hands). On 222, `move_left_arm_joints`, `point_at` and forearm pointing are unchecked for the Dex3
  hand until the checker includes it.
- **CLI robot argument:** `dimos mcp call … --arg robot=223` arrives as an integer and was refused before
  anything was sent. `--json-args` works (fix in progress).
- **Bring-up order:** planner idle must run before the tunnel. A tunnel already holding 127.0.0.1:5556 makes
  `start-idle-planner.py` fail with "Address already in use".
- **222's camera publisher:** `/home/unitree/gumin/d435_color_publisher.py` on 222 binds 0.0.0.0:5556 and
  must be stopped for a dual session.
