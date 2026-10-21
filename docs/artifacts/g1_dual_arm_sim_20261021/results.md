# Two-G1 arm sync: sim acceptance (2026-10-22)

Two real-time MuJoCo stand-ins for the guarded SONIC controller drove `unitree-g1-sonic-arm-dual-agentic`
through MCP and one agent sentence. No robot was involved.

## Setup

- Branch `jihun/feat/g1-dual-sonic-arm`, Tier-1 engine snapshot `fddbf31fe` (uncommitted Tier-1
  task 4), dual work through `67f9649c0`.
- Stand-in: frozen snapshot in `standin/` (`SHA256SUMS`). It contains `startup_on_floor`, the full
  simulator rather than the later wrapper. Still external: the lerobot harness at
  `~/work/lerobot/examples/g1_dex3_training` and the HF GEAR-SONIC snapshot `9c0ff22b`.
- Ports: 222 on 15556/15557 and 223 on 16556/16557, the blueprint's own.
- Encoder: `GR00T-WholeBodyControl/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx`.
  Collision model: `GR00T-WholeBodyControl/.../scene_43dof.xml`. `max_dq` 2.0.
- Agent: `openai:openai/gpt-5.6-luna` through the Genon gateway (key from `dimos_dev/.env.genon`).
- Driver: `demo_run_dual_sim.py <out>`. Per-run `results.json`, stand-in logs and `dimos_run.log`.
  The `sim_*.npz` state logs stay local and are not committed (10 MB per run).
- Load: no other MuJoCo sim ran, by turn-taking with the Tier-1 session. An unrelated native
  `g1_deploy_onnx_ref` (zmq 5566/5567, started from a tmux pane) was running throughout.

## Results

Start skew is the gap between the two robots' first feedback samples that moved 0.03 rad. The target
is < 0.5 s. Submit skew is the gap between the two chunk-0 sends.

| Check | run01 | run02 |
|---|---|---|
| `move_left_arm_pose(both, left_arm_side_horizontal)` | start 0.000 s / submit 0.001 s; 91 % / 92 %, returned | start 0.017 s / submit 0.001 s; 91 % / 91 %, returned |
| `perform_gesture(both, wave)` | start 0.252 s / submit 0.001 s; 98 % / 99 %, returned | start 0.249 s / submit 0.001 s; 98 % / 99 %, returned |
| `move_upper_body(223, left_arm_forward, 20)` | 223 reached 16.7°; 222 never armed | same; 222 never armed |
| `perform_gesture(both, wave)` + `stop_arm` | invalid: stop sent 8 s after the call, during encoding, so "nothing sent" (correct, but not the check) | stop 6 s after both armed: both "stopped early: stop requested", returned; peak 0.90 / 0.85 rad/s, tilt ≤ 1.55°, 8 contacts (these three figures are from the local sim_*.npz, not committed) |
| agent-send "both robots wave" | `perform_gesture(both, wave)`, start 0.015 s; 99 % / 99 % | start 0.329 s; 98 % / 98 % |
| 223 stand-in killed, then `move_left_arm_pose(both, …)` | "223: SONIC is not in confirmed planner idle; nothing sent; nothing sent to any robot"; 222 moved 0.008 rad | same; 222 moved 0.012 rad |

Stand-in totals over each whole run (both runs):
- peak left-arm speed ≤ 1.05 rad/s;
- tilt ≤ 2.0°;
- floor contacts never below 8;
- no reference expiry;
- tick time p99 16–17 ms, worst case 25 ms.

## Notes

- **Idle robot during a single-robot command.** 222's left arm moved 0.021 rad during the 223-only
  move, all of it shoulder yaw. 222 never armed (its stand-in log shows no arming), and an idle
  stand-in drifts 0.012–0.024 rad on its own after a motion. That is settling, not a reference.
- **Wave start skew.** Wave skew is 0.25–0.33 s and is unexplained so far. Submit skew is 1 ms, so
  it is the physical onset, not DimOS timing. The pose skew is ≤ 0.017 s. It is within the 0.5 s
  target, and it is the margin to watch on hardware.
- **Encoding time.** Encoding both waves takes longer than 8 s on this workstation. A `stop_arm`
  in that window refuses before anything is sent.
- **Bugs found and fixed during this run.** The dual agentic blueprint set the removed GlobalConfig
  field `mcp_timeout`, and `dimos run` refused it. Fixed in `67f9649c0` with a test. Tier-1's
  `unitree-g1-sonic-arm-agentic` has the same line, reported to that session. The driver's
  `dimos run --daemon` hung on a captured stdout pipe; it now writes to `dimos_run.log`.

## Not covered (hardware, next)

This doesn't cover the real guarded controllers, the SSH tunnels, a dropped tunnel (which still
snaps a raised arm), or pc2_222 with its hands. The runbook is in the
`unitree_g1_sonic_arm_dual` docstring.
