# Native SONIC motion failures: preserved evidence and correction plan

Status: the historical run02/run08 failures are preserved. Follow-up isolated headless trials now pass the original movement and stopping criteria after terminal consistency, absolute physics pacing, and guarded future-window maintenance. See [motion-resolution/README.md](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/README.md) for all failed and passing comparisons. The independent realized-window audit remains separately recorded. Hardware commissioning stays blocked by the exact FK fixture and later calibration/robot checks. No real robot was controlled; original source and environment were preserved. The initial diagnosis below records what was known before these follow-ups.

The strongest concrete correction candidate is an inconsistent stationary reference in the native encoder: final planner positions are held indefinitely while nonzero terminal joint velocities continue to be supplied. A second confirmed confound is physics advancing slower than the native controller's wall clock. Neither has yet been isolated experimentally as the cause of the physical failures. The preserved data establishes the failure boundary and rules out several alternatives; it does not justify saying that either physical blocker is fixed.

## Evidence location and pairing

All run paths below are relative to `docs/artifacts/g1_navigation_sonic_20261013/on-host-sim/`. The private checkout used here is `/tmp/dimos-g1-sonic-navigation-20261013`.

- `run02/results.json`: forward command +0.2 m/s for nominal 2 s produces **−0.0379703127 m** projected along the initial physical heading; initial idle settling passed.
- `run03/results.json`: same original native binary, +0.0284115632 m in the original 2 s, +0.7455301533 m over the separate 6 s diagnostic. The latter is not a replacement acceptance window.
- `run04/results.json`: three transitions produce +0.1187477513, +0.1114167163, +0.2915884420 m in the original windows; settling takes 2.6276, 2.0782, 2.1005 s. This demonstrates substantial state sensitivity, not a correction of run02.
- `run08/results.json`: forward +0.0191663300 m; `TimeoutError: forward measured settling` under the unchanged 3 s deadline.
- `run12/results.json`: 26-case matrix passes; forward +0.0946501339 m and forward settling 2.2052561441 s.
- `run02/manifest.json` native SHA256 begins `1e0185d6c9`; run03 uses the same binary. Run04 begins `fb08204d8a`. Run08 and run12 both use **`ce396b2b005264a1dc13f282bc032d0baf56be14d0a3456c815d42611802648b`**. Run12 is not evidence of a native correction after run08.
- Run02 does not contain target/planner CSV, frame/replan CSV, sent trace, or a preserved harness copy. Run08 contains both motion CSVs and frame/replan sidecars, but no sent trace or preserved harness copy. Observer timestamps in `references.jsonl` are reception/poll timestamps, not send timestamps.
- Run12 has a saved `harness.py`, `sent.jsonl`, conversion/input traces, and manifest hashes. Do not assume its modified noise test harness is byte-identical to the earlier harness.
- Launch scripts `/tmp/run_g1_navigation_on_host_20261013_run02.sh` and `..._run08.sh` use the same isolated loopback setup, GPU 1, the private native binary/assets, and `/home/jihun/work/GR00T-WholeBodyControl` for simulator and reference assets. Manifests and `sim-config.json` establish a 5 ms MuJoCo step, real native planner/encoder/decoder, and guarded planner mode. No stubbed locomotion controller is involved.

## Run02: failure is below command admission

From `run02/physics.jsonl`, `telemetry.jsonl`, and decoded `references.jsonl`:

- Initial heading is about +0.0340 rad. The forward wire intent has movement x >0.998, speed +0.200000003, and facing near +0.02545 rad. No negative-speed/backward command appears in this interval. Its few direction updates are small positive-x heading adjustments.
- Native feedback first reports SLOW_WALK about **147 ms** after the recorded initial physics sample. Mode/lease admission succeeds; native `deploy.log` also records SLOW_WALK.
- Projected physical displacement at approximately +0.5/+1/+1.5/+2 s is −0.0021/+0.0057/+0.0058/−0.0380 m. At the end, speed is about 0.1022 m/s. This is actual reverse motion, not a sign inversion introduced by the final projection.
- The physical heading changes only −0.0013937 rad across the window. A 180-degree frame error cannot explain the result.
- Initial measured stationarity was established, but that does not establish a consistent encoder reference or a repeatable joint/contact state. The test enters forward only after supported startup, release, an unsupported wait, navigation rearm, and another idle interval. Calling this simply an uninitialized controller is unsupported.
- No planner-frame/replan trace was recorded. Therefore the exact generated motion, its context, merge timing, and final reference velocities in **run02 itself** cannot be reconstructed. The later traces illuminate the same implementation path but must not be attributed to run02 as direct measurements.

The likely boundary is initial standing/reference state → generated transition → policy tracking/contact dynamics. Run03 shows that sustained positive intent eventually produces substantial positive displacement; it does not resolve failure of the original 2 s contract.

## Run08: IDLE is delivered; physical yaw prevents settling

Let `t_end = 1066470.657740966`, the recorded final forward physics timestamp. This is a close proxy for stop entry; the earlier harness did not record the exact `settle()` entry time. Offsets below use that timestamp and are not claimed as exact end-to-end stop latency.

- First observer IDLE packet is about **53 ms** after `t_end`; first telemetry IDLE ACK about **74 ms** after it.
- One native IDLE replan starts at `1066470.79936` (about **142 ms** after `t_end`) and finishes about **4.85 ms** later. Replan mode is 0, movement is zero, facing is `(0.9985110164, -0.0545509607)`. The native mode-specific default speed is `-1`; the actual wire IDLE speed is zero. This conversion is expected native behavior and is not evidence that walking remained commanded.
- The stop-facing packet remains unchanged thereafter. There is no stop-facing chase or repeated IDLE state replan in the remaining trace.
- Playback reaches final frame 74 at roughly 1.6 s after stop and holds its root pose. Native inference/command delivery latency cannot account for the remaining physical yaw oscillation.
- The reference yaw moves during the generated IDLE transition, then holds at approximately **−0.07172 rad**. Measured yaw is approximately −0.02175 at +1.5 s, −0.03149 at +2 s, −0.05029 at +2.5 s, and −0.06289 at +3 s. The snapshot facing command is near −0.05458 rad in the planner frame. Frozen *command facing* does not make every generated reference frame immediately stationary or exactly equal to the command yaw.
- Translational speed is already below 0.03 m/s during the late failure. Physical yaw-rate bursts still reach roughly **−0.151 rad/s** around +2.19 s; this resets the stationarity interval.
- Offline recomputation using consecutive preserved physics rows, finite-difference quaternion yaw over simulation time, the original speed/yaw thresholds and 0.12 s freshness gap yields qualifying intervals about [1.588, 2.005] s (**0.417 s**) and [2.525, 3.020] s (**0.495 s**). The latter extends slightly past a nominal 3 s boundary and still does not prove the required half-second inside that boundary. The online sampler can skip a logged row, and exact stop entry was not logged, so these are diagnostic reconstruction intervals; `results.json` remains the authoritative observed failure.

This is a real measured-settling miss. A small extension of the deadline might make this particular sample pass, but would hide the reason for the yaw-rate resets and is not a correction.

## Concrete native reference inconsistency

Read-only source root: `build/sonic-navigation-runtime/source/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`.

1. `include/localmotion_kplanner.hpp`, `ResampleGeneratedSequence50Hz()` (around lines 503–519), computes forward finite-difference joint velocities then **copies the penultimate velocity into the last frame**.
2. `src/g1_deploy_onnx_ref.cpp`, `CurrentFrameAdvancement()` (around lines 3430–3500), clamps an exhausted IDLE motion to its last frame. `operator_state.play` stays true. The lower-body readaptation can alter positions separately; it does not make these terminal velocities consistent.
3. `GatherMotionJointPositionsMultiFrame()` clamps all future samples to the final position.
4. `GatherMotionJointVelocitiesMultiFrame()` (around lines 842–934) also clamps future indices to the last frame, but still copies its stored nonzero velocity whenever `play` is true. This affects full-body and lower-body gathers, and the encoder parity source logging.
5. `assets/observation_config.yaml` enables `motion_joint_velocities_10frame_step5` and the corresponding lower-body feature. G1 encoder mode 0 consumes the full-body velocity feature. The discrepancy reaches inference; it is not merely a CSV logging issue.

Preserved run08 `planner-motion.csv` has eight blocks (initial motion + seven replans). The last block is the one IDLE stop motion. Its final two motor-position rows imply terminal velocities, using the native `50 * (last - previous)` rule, up to approximately **0.14415 rad/s in the legs** and **0.22497 rad/s across all joints**. Its pre-forward IDLE block has approximately **0.13456 rad/s** maximum leg velocity. CSV rounding limits precision, not the conclusion that these are nonzero. The target-frame sidecar shows stationary final positions.

Run12 contains this same inconsistency: the corresponding stop block has about 0.12370 rad/s maximum leg velocity and 0.20773 rad/s maximum overall velocity. Passing run12 therefore does not disprove the mechanism or clear it. Original local official source also copies the terminal velocity; this is not a guard-extension regression.

**Proposed minimal correction to test first:** for guarded planner playback, represent held/padded terminal samples consistently as a constant pose with zero reference joint velocity. Apply the same predicate to every affected velocity gather and parity log. Preserve genuine pre-terminal velocities and live motion samples. Do not pause the entire planner, replace leases, chase measured yaw with repeated replans, or alter the command envelope. Zeroing the stored terminal velocity alone is a smaller possible implementation; the observation-level rule should still explicitly cover terminal padding and keep position/velocity semantics aligned.

This source inconsistency is demonstrated. Its **causal contribution to run02 reverse displacement and run08 settling is a hypothesis requiring an A/B experiment**, not a proven complete explanation. If fixing it does not correct the original physical windows, retain the failure and investigate the next boundary rather than stacking speculative fixes.

## Timing and state confounds to control

The worker `demo_navigation_sim_worker.py` advances a fixed 0.005 s physics step, then uses a relative `sleep(max(0, dt - elapsed))`. Scheduler oversleep accumulates. Native planning/control proceeds at wall-clock 10/50 Hz; it does not wait for physics simulation time.

| Run | Nominal forward wall interval | Physics elapsed (approximately) | Physical/control ratio |
| --- | ---: | ---: | ---: |
| run02 | 2.002 s | 1.875 s | 0.937 |
| run03 | 2.014 s | about 1.885 s | 0.936 |
| run08 | 2.015 s | about 1.960 s | 0.973 |
| run12 | 2.013 s | about 1.965 s | 0.976 |

Exact endpoint differences can be computed from the matching `physics.jsonl` rows; nearest samples at +2.0 s differ by a few hundredths. The material fact is different effective controller/physics rates. This can affect transition/contact dynamics, not merely shorten a displacement interval.

A separate minimal harness correction is absolute monotonic step deadlines with overrun/missed-deadline logging, keeping dt, command duration, lease deadlines, and settling thresholds unchanged. Do not add simulation seconds to compensate or silently remove an overloaded trial. For deterministic replay, couple control application and physics stepping at explicitly recorded step boundaries; absolute wall pacing alone improves drift but does not produce exact closed-loop replay.

`deploy.log` reports seed 1234; source keeps it unless a replacement is supplied. There is no evidence of deliberate seed changes or the later injected ±0.005 rad IMU noise during these initial forward cases. Do not describe these failures as random seed noise without recording model inputs/outputs and showing differing results for identical inputs. Launch qpos is identical in the inspected runs; post-release settled poses are not. Run08's pre-forward lower-body mean absolute reference/measured joint error is about 0.056 rad versus run12's about 0.045 rad. At the stop replan, first-context-frame/measured error is about 0.101 rad versus 0.069 rad. Those comparisons are reference-versus-current-state diagnostics (the context has lookahead), not instantaneous error bounds or proof of a new threshold requirement.

## Regression and controlled native verification

1. **Failing deterministic native regression before implementation.** Exercise the actual terminal observation gather with a short planner motion whose last two positions differ, `play=true`, and current frame at/near the terminal frame. Assert: valid in-range samples retain their recorded velocities; held terminal/padded future samples have identical terminal positions and zero velocity; full-body and lower-body gathers and parity capture agree; default non-guarded behavior is preserved if this is scoped to the private guarded runtime. The current implementation fails this behavior because it returns the copied nonzero terminal velocities. Use actual native observation/gather code, not a test that restates a new helper in isolation.
2. **Preserved physical-failure fixtures.** Keep run02's exact two-second signed-displacement assertion and run08's exact 0.03 m/s, 0.03 rad/s, 0.5 s dwell, 3.0 s timeout verdict as failing raw-trace regressions. They protect against accidental tolerance/window changes; they do not become passing regression tests for the physical correction simply because source changes.
3. **Baseline then one-variable A/B.** Fix the manifests/assets/seed, simulator model, GPU, release protocol and initial schedule. Record native command receipt/applied/replan/merge times, simulation step, control step, entire encoder reference positions *and velocities*, context, qpos/qvel, native policy history, applied motor targets, and stop-entry time. First compare original versus terminal-reference correction with the same scheduling setup. Test the pacing change separately before combining changes. Include both starting from fresh startup and repeated forward→zero transitions within each process; do not preselect a favorable stance or add an uncounted moving warmup.
4. **Original envelopes in every trial.** At least three independent startup processes with three 2 s +0.2 m/s forward→zero cycles each provide an initial reproducibility check; retain every trial. Require the original positive signed-displacement inequality and measured settling in 3 s for every cycle, plus existing unsupported/no-resets/upright checks. Then rerun the full 26-case matrix, noise attribution and handback/expiry checks because terminal semantics also affect pose/planner transitions. This is commissioning evidence, not a statistical reliability guarantee.
5. **If exact failure replay is required**, qpos/qvel alone is insufficient: preserve MuJoCo complete integration state/contact warm-start state, controller history, planner/reference buffers, frame counters, seed and step schedule. Run02 did not record all of these; exact deterministic regeneration of its historical closed-loop failure cannot honestly be promised. A captured new failing controlled state can become the reproducible before/after fixture.
6. **If the first correction does not resolve both failures**, investigate context/physical-state mismatch next with identical-context planner output replay and controlled physics/contact-state replay. Run08's reference translates roughly +0.325 m by +2 s while actual translation is only +0.019 m; command parsing is already ruled out as that loss. Prove whether divergence enters at reference generation, encoder/decoder tracking, or asynchronous action/physics timing before changing gains, selecting a different seed, or changing the motion policy.

Completion requires a passing deterministic semantic regression plus native evidence correcting both original envelopes across controlled repetitions. Later passing samples alone, longer movement windows, longer settling timeouts, or relabeling either preserved failure are insufficient.
