# pc2 (.223) incident: offline comparison of original vs guarded executables

Date: 2026-10-14. Read-only on robot control: no SONIC start, no DDS writers, no
motor commands. One compile-only scratch build on pc2 (see §4); never executed.

## 1. Compiled control constants: identical

Extracted from `.rodata` of both ELF files (`nm -SC` + segment mapping):
`kps`, `kds` (float32), `default_angles`, `g1_action_scale` (float64),
`isaaclab_to_mujoco`, `mujoco_to_isaaclab` (int32). Byte-identical.

Waist (MuJoCo idx 12/13/14): kp 40.18/28.50/28.50, kd 2.558/1.814/1.814,
default 0/0/0, action scale 0.5475/0.4386/0.4386.

## 2. Build and link: equivalent

- Both: GCC 11.4.0, `CMAKE_BUILD_TYPE=Release`, `-O3 -ffast-math -O3 -DNDEBUG -std=c++20`
  (crtfastmath linked in both).
- Same DT_NEEDED set. RUNPATH differs (`/opt/onnxruntime/lib` → `/usr/local/lib`),
  but `/usr/local/lib/libonnxruntime.so` is a symlink to the same
  `/opt/onnxruntime/lib/libonnxruntime.so.1.16.3` (same SHA-256). Moot.
- TensorRT engine caches used by the staged run (`policy_model_decoder.trt`,
  `encoder_model_encoder.trt`, `planner_planner_sonic.trt`) are byte-identical
  to the original's and were not rebuilt (mtimes Aug 7 / Jun 10).

## 3. Code: control path differs only as the patch predicts

Normalized aarch64 disassembly (PLT/GOT/adrp resolved to symbols) of
`G1Deploy::*`, planner, motion reader, merger, observation parser:

- `LowCommandWriter`: only the `enable_dex3_hands_` gate around `writeOnce()`.
- `CheckSafety`, `CreateDampingCommand`, `LowStateHandler`, `imuTorsoHandler`,
  VR gatherers: only +4/+8 byte member-offset shifts (new `enable_dex3_hands_`).
- `Control`/`InitControl` shrink because `Dex3Hands::close`/`setAllJointsCommand`
  became out-of-line calls; `Planner`/`CurrentFrameAdvancement` gain `ldarb`/`stlrb`
  from `PlannerState` bool → `std::atomic<bool>`.
- `GatherHeadingDiffRobotRef` (policy observation): same FP operations,
  rescheduled. No semantic change found.
- Untouched functions with large diffs (`MotionDataReader::ReadFromCSV`,
  `StreamedMotionMerger::MergeIncomingData`, `create_output_data_map`) are
  load-time / output-only; see §4 for the source-equivalence rebuild.

## 4. Source-equivalence rebuild

The captured pre-port source (`/tmp/dimos-pc2-223-native-20261014`) was rebuilt on
pc2 in `/home/unitree/dimos-sonic-equiv-20261014` (thirdparty symlinked to the
original, Release, onnxruntime from `/opt`, target `g1_deploy_onnx_ref` only;
compile only, never executed).

- Size 5,588,736 bytes, identical to the original.
- 10,113 functions in both, same names, **zero size changes**.
- Normalized instruction diff: 20 functions differ, all only by string-literal
  `lo12` offsets (+8) or a string length (`#91`→`#95`). Both come from the build
  path being 4 characters longer (`dimos-sonic-equiv-20261014` vs
  `GR00T-WholeBodyControl`) in `__FILE__`/RUNPATH strings.

**The original executable is equivalent to the captured source.** The port patch
is therefore the complete code delta. The large non-patch diffs in §3 are GCC
codegen ripple from the patched translation unit, not hidden source changes.

## 5. Runtime environment: the one measured difference

Original-binary runs on this robot (`~/minji/logs/*/T5_deploy.log`, Sep 29 to Oct 1,
same model, planner, `zmq_manager`, `--output-type all`) vs the incident paste:

| Loop timing | original, 3148 samples: p50 / p95 | incident (2 samples) |
|---|---|---|
| Policy | 1.77 / 2.29 ms (≥5 ms in 0.29%) | 5.05, 5.06 ms |
| Obs | 1.75 / 2.90 ms | 2.74, 2.75 ms |
| Obs → motor command | 3.53 / 5.36 ms (≥7 ms in 2.3%) | 7.79, 7.81 ms |
| LowState age | 4.2 / 6.9 ms (≥8 ms in 3.0%) | 8.5, 8.9 ms |
| IMU age | 4.2 / 6.5 ms | 8.0, 8.7 ms |

The incident's policy time is a steady ~2.9× shift, not an outlier spike.

Cause of the slowdown, from `journalctl -b` on pc2:

- Boot 14:41 (nvpmodel MAXN, but CPU `schedutil` / GPU `nvhost_podgov` free to
  down-clock; GPU railgating enabled).
- No `jetson_clocks` until **16:31:30**, run via sudo from the *original*
  `gear_sonic_deploy` (pts/0), i.e. after the incident and after the 16:30:28
  post-stop DDS probe.
- Every original launch on record sources `scripts/setup_env.sh`, which runs
  `sudo jetson_clocks` (deploy.sh:489, and each manual `just run` in history).
  `launch-guarded.sh` bypasses `setup_env.sh`: the guarded run is the only launch
  on record at default DVFS clocks.

So sense→act latency during the incident was roughly 2× the original
(~17 ms vs ~8 ms from LowState sample to command), with the policy on a
down-clocked, railgated GPU.

## 6. Assessment

- Compiled behavior is equivalent on the motor path; the guard / no-hands patch is
  not implicated by the binary comparison.
- The only measured difference between the smooth original runs and the incident is
  the missing `jetson_clocks` (≈2× control latency). Latency-induced oscillation of
  the policy loop is a plausible mechanism for the waist "two commands fighting"
  noise. **Not proven**: no motor trace exists for the incident.
- **Operator confirmed (2026-10-14):** after the grinding, they deployed the original
  SONIC (the 16:31 `deploy.sh` session) and no grinding occurred. That run is the
  matched control: same boot, same robot and support, clocks pinned by
  `setup_env.sh`. No log of it survives (deploy.sh output was not captured), so its
  loop timing was not measured.
- The pair still varies three things together: clocks, binary (guard + no-hands),
  and launcher environment. The executable comparison makes the binary the least
  likely of the three on the motor path, but this pair alone does not separate them.
- As of 16:50 the clocks are still pinned in this boot (CPU min=max=1.984 GHz,
  GPU min=max=918 MHz). Any guarded retest before a reboot would therefore run
  with pinned clocks.

Tools and raw outputs: `binary-compare/` (`elf_arrays.py`, `fdis.py`, array dump,
link comparison, function-size diff, original-vs-rebuilt diff list, hashes).

## 7. Launcher change (prepared locally, NOT deployed to pc2)

In `../g1_pc2_223_sync_20261014/`:

- `launch-guarded.incident-v1.sh`: the launcher used in the incident. Its SHA-256
  `18ad54a9…` matches the copy deployed on pc2 and `final-verification.json`.
- `launch-guarded.sh`: the new version, SHA-256 `456bb1fd…`. Diff in
  `launch-guarded.clock-preflight.patch`. Changes:
  1. `--run` refuses to start unless every CPU's `scaling_min_freq` equals
     `cpuinfo_max_freq` and the GPU devfreq `min_freq` equals `max_freq`, which is
     the state `jetson_clocks` produces. It prints `sudo jetson_clocks` as the
     remedy and does not call sudo itself.
  2. `--check-clocks` runs only that preflight, for operator use.
  3. Adds `--enable-csv-logs --logs-dir <stage>/logs/<timestamp>` to the native
     command. The Sep 29 to Oct 1 smooth original runs also had CSV logging on.
  Everything else is unchanged: binary, flags, environment, hash check, and
  single-instance check.
- `demo_test_launch_clocks.sh`: offline test with fake sysfs trees covering
  pinned, CPU-unpinned, GPU-unpinned and no-controls cases, plus a dry-run check.
  It never passes `--run`. Result: 5/5 ok.
- Read-only `--check-clocks` on pc2 at 18:20 reports pinned, as expected after
  16:31 in the same boot.

Retest (operator-controlled, needs explicit authorization): copy the new
`launch-guarded.sh` into the staged root, then run `--check-clocks`, then one
supported `--run` with the planner helper as before. Smooth → clocks were the
cause. Grinding → the clock hypothesis is falsified, and the CSV in `logs/` holds
the waist q/action/torque trace. Stop immediately in either case of grinding.

## 8. Retest result (2026-10-14 17:27, operator-run): smooth → root cause confirmed

The operator deployed the new `launch-guarded.sh` (`456bb1fd…`), saw
`--check-clocks` report pinned, then ran `--run` plus the same planner helper on
the same support setup. **No grinding.** The startup sequence was identical to the
incident (example `neutral_kick_R_001__A543` → IDLE planner → `planner_motion`).

| Loop timing (26 samples) | incident | retest | original runs p50 |
|---|---|---|---|
| Policy | 5.06 ms | 1.71–1.72 ms | 1.77 ms |
| Obs | 2.75 ms | 1.34–1.44 ms | 1.75 ms |
| Obs → motor command | 7.8 ms | 3.06–3.16 ms | 3.53 ms |
| LowState age | 8.5–8.9 ms | 3.1–4.0 ms | 4.2 ms |

Planner init: 13.6 ms (incident 36.4 ms).

Waist CSV (`retest-20261014-172735/`, 45.7 s of CONTROL; full numbers in
`waist-metrics.txt`, produced by `binary-compare/waist_metrics.py`) is equal to or
smoother than the first 30 s of a smooth original run (0930c) on every metric.
Values are retest vs original:

- dq rms (yaw/roll/pitch): 0.051/0.063/0.053 vs 0.071/0.082/0.073 rad/s
- tau std: 0.29/0.50/0.55 vs 0.47/1.01/2.57 Nm
- tau second-difference rms (chatter): 0.30/0.34/0.43 vs 0.38/0.69/0.60
- First-second startup transient, max |dq|: 1.5/1.26/0.98 vs 1.7/2.04/1.24 rad/s

**Conclusion.** The grinding was caused by running SONIC with unpinned Jetson
clocks. `launch-guarded.sh` bypassed `setup_env.sh` and so skipped
`jetson_clocks`, which roughly tripled policy time and doubled sense-to-command
latency. The same guarded binary is smooth once the clocks are pinned. The guard
and no-hands patch is not implicated.

The retest also added CSV logging. Logging adds load rather than removing it, so
it cannot explain the improvement.

The inherited startup-ordering hazard (§ "Startup source review" in the handover)
still exists. However, its first-second waist transient is the same size in smooth
original runs, so it is not this incident. Treat it as a separate, lower-priority
item.

## 9. Body telemetry review of the retest (17:27, 45.7 s CONTROL, IDLE planner)

Source: `retest-20261014-172735/`. Script: `binary-compare/body_telemetry.py`
(full output in `retest-20261014-172735/body-telemetry.txt`).

**Healthy**

- **Loop integrity:** 2,286 rows at dt 20.000 ms (p99 20.008, max 20.014 ms).
  No missed ticks.
- **Modes:** the example motion ran for 2 rows (0.04 s), then `planner_motion`.
  Encoder mode was 0 throughout, and playback started at 0.08 s.
- **Motor errors:** none on any of the 29 motors in any row.
- **Command integrity:** PD torque reconstructed from the logged actions
  (`default + raw[isaaclab_to_mujoco]·scale`, the code's kp/kd) matches measured
  torque at r = 1.000 with 0.23 Nm rms residual. The robot executed exactly this
  controller's commands, with no sign of a second command source.
- **Startup:** the first commanded step is ≤ 0.11 rad (elbows), which is identical
  to the original run (0930c). The posture transition finishes in 1.88 s. Peak
  speeds: arms 5.2 rad/s (left shoulder yaw), legs 3.4 rad/s (left knee), waist
  1.5 rad/s. Peak torques: left shoulder roll 10.9 Nm (44 % of 25 Nm), waist
  pitch 16.9 Nm (34 % of 50 Nm). Effort limits are derived from
  `action_scale = 0.25·effort/kp`.
- **Steady state (t > 2 s):** every joint's dq rms is ≤ 0.023 rad/s, which is
  encoder-noise level, so there is no vibration. Pelvis roll 0.50 ± 0.02°,
  pitch −1.44 ± 0.03°. The only later event is a single tick at 35.1 s:
  0.11 rad/s on left hip yaw, with no target change and no base motion. That
  looks like a sensor blip or a light bump.

**Static loads (expected for this support setup, not a fault)**

The knees are nearly straight (0.20 rad, against a 0.67 rad default) at only
~5 Nm, so body weight is carried by the support. The legs nonetheless hold steady
pushes toward targets they cannot reach:

- Hip roll: −25 / +23 Nm, with the target 0.24–0.25 rad inward of measured.
- Ankle pitch: +10 / +5 Nm, with the target 0.36 / 0.18 rad away.
- Hip yaw: +6 / −9 Nm.

This indicates the feet are constrained (in contact) while the body hangs. The
original 0930c run shows the same pattern with a different split (ankle pitch
22 / 16 Nm, hip roll 8 / 5 Nm), so it is characteristic of supported IDLE
operation.

**Watch item: driver temperatures.** Windings stay ≤ 46 °C (the shoulder
pitches, which started at that level). Several driver temperatures climbed
steadily, at a decelerating rate:

- Left hip roll: 29 → 48 °C.
- Right hip roll: 30 → 46 °C.
- Left ankle roll: 35 → 47 °C, despite only ~2.8 Nm.

These rises have not been compared with the original run's temperature log,
because pc2 became unreachable during this review. For longer supported idle
sessions, monitor the hip-roll and left ankle-roll driver temperatures. Compare
with `~/minji/logs/0930c_carryon/deploy_csv/motor_temperature.csv` when pc2 is
back.

**Not covered:** arm reference streaming, guard expiry, walking or balance on
the floor.

## 10. Inherited startup-ordering hazard: measured, no fix warranted

The handover (§ "Startup source review") flagged that the first policy ticks
use the paused example reference (`neutral_kick_R_001__A543`, frame 0) before
the planner motion is selected. The handover's recommended discriminator was an
offline harness with fake engines. The CSV logs answer the question directly
with real data, so no harness was built. Target per tick =
`default + raw_action[isaaclab_to_mujoco] * scale`. Step = change from the
previous tick's target; row 0 is compared against measured `q`.

| Run | Example-conditioned ticks | Max step, example ticks | Step at planner switch | Max step, next 10 planner ticks | Steps < 0.05 rad from |
|---|---|---|---|---|---|
| Retest 17:27 (guarded) | 2 | 0.474 rad | 0.205 rad | 0.504 rad | 0.44 s |
| Original 0930c (smooth) | 3 | 0.476 rad | 0.287 rad | 0.523 rad | 1.44 s |

- The hazard is real but behaves the same in both builds. The original, smooth
  build spends one tick longer on the example reference.
- The step at the switch is smaller than the swings that the
  planner-conditioned policy produces over the next 10 ticks. The 0.2–0.5 s
  transient is the policy settling from the INIT posture, not an
  example→planner jump. A planner-readiness gate would remove at most 2–3 ticks
  and leave the transient as it is.
- Decision: no native change. The gate would alter upstream startup behavior
  without a measured benefit. Revisit only if a future log shows a large step
  at the switch row. `body_telemetry.py` now prints this section ("Startup
  target steps") for every log it reviews.

Also checked offline, for milestone 3. The DimOS SONIC session does not gate on
hand feedback validity. The no-hands controller still publishes 7+7 hand
arrays, so `decode_feedback` and the session seed are compatible with
`--disable-dex3-hands`. The echoed hand values are ignored natively in that
mode. Local suite: `pytest dimos/robot/unitree/g1 -k sonic`, 366 passed
(2026-10-14).

## 11. Follow-ups after pc2 returned (2026-10-15, read-only)

**Driver temperatures: watch item closed.** These are driver temperatures in
°C from the original 0930c `motor_temperature.csv` (317 s run), compared with
the retest:

| Joint | Retest start → 46 s | Original start → 46 s | Original max |
|---|---|---|---|
| L hip roll | 29 → 48 | 40 → 42 | 48 |
| R hip roll | 30 → 46 | 40 → 41 | 46 |
| L ankle roll | 35 → 47 | 43 → 85 | 90 |
| R ankle roll | 34 → 38 | 43 → 62 | 66 |
| L ankle pitch | 34 → 36 | 42 → 71 | 74 |

The original run reached the same hip-roll peaks and much hotter ankles: the
driver maximum was 90 °C and the winding maximum 55 °C. The retest's rises are
within the range already seen in the smooth original, so they are not an
anomaly. The two runs had different starting temperatures and durations, so
the table shows range, not a controlled comparison.

**`token_state.csv` copied** into `retest-20261014-172735/`. After 2 s of
planner IDLE, all 64 token dimensions are exactly constant: the range is 0.0,
and the largest step is 0.0. The largest absolute value is 0.438, below the
session's 1.25 bound. This supports the zero-motion hold test's premise: once
IDLE has settled, the planner feeds the policy a constant token, so echoing
the captured token gives the policy the same input.

## 12. First zero-motion hold runs (2026-10-15, operator-run)

Logs are in `hold-20261015/`: native CSVs from `20261015-085417` (a snapshot
taken while SONIC was still running) and three session journals. Journal
times and CSV `time_monotonic_ms` share CLOCK_MONOTONIC. Peak joint speed
(|dq|) per phase, across all three runs:

| Phase | Peak \|dq\| | Token vs captured |
|---|---|---|
| Idle before the test | 0.06 rad/s | equal |
| Seed (1 s, planner) | ≤0.11 | equal |
| Arm switch | 0.21–0.24 (wrists) | one-tick 0.062 blip |
| Hold (2 s, streamed) | ≤0.11 (once 0.53, run 1) | exactly equal |
| **Planner return, first ~1.6 s** | **arms 0.62–1.01 (wrist roll), waist ≤0.19, legs ≤0.21** | 0.062 off, then exactly equal again |
| After that | baseline | equal |

- Guard: expiry was detected 0.198 s after the stream stopped, and the planner
  was confirmed at 0.299 s. Handback was confirmed in 0.12 s.
- `rejected` = 50 per run. These are the pose frames that the session emits
  while SEEDING, before the stream is armed. The native side rejects them
  before storing anything, so they are harmless and by design.
- The fidget is the planner restart. On the return to PLANNER the manager
  calls `TriggerSafetyReset`, the guard forces IDLE `{0,0,0}` with speed and
  height at their −1 defaults, and the planner replans from its restart. This
  is inherited planner behavior; no velocity is commanded. Hip and knee
  positions changed by at most 0.02 rad (no stepping), and the posture after
  was within 0.022 rad of the posture before.
- Tooling gap, now fixed: v1 of `hold-reference-test.py` stopped measuring at
  planner confirmation and reported a peak of 0.24 rad/s and PASS. The current
  version watches 3 s after the return and fails above `--max-return-dq`
  (default 0.5 rad/s).

### 12a. Cause of the return fidget (offline, planner ONNX)

In IDLE the planner never replans periodically; static modes replan only on a
mode, facing or height change. A cold start (`InitializeContext`: the measured
pose held still for 4 frames, seed 1234) generates one 1.47 s "settle"
clip. Playback then clamps to its last frame. Every guard return cold-starts
the planner, so it replays that clip. Running `planner_sonic.onnx` offline
from the logged return pose (`binary-compare/planner_cold_start.py`)
reproduces the fidget. The reference travels up to 0.27 rad: left wrist roll
0.24, left wrist pitch 0.13, left shoulder yaw 0.13, right knee −0.11. The
peak reference speed is 0.77 rad/s (left wrist roll). The clip starts at the
measured pose (frame 0 within 0.026 rad), so there is no jump to blend.

On the support, the left wrist roll stays about 0.24 rad short of the planner
pose, so each restart replans the same move. The operator's hypothesis is that
the left hand touches the pelvis, which is natural behavior. The return is
affordable:

- Speed: about 3–4% of the URDF wrist and shoulder speed limits (22–37 rad/s).
- Torque: peak 25% of the limit (left ankle pitch, already at that level in
  idle). The largest rise over idle is 2.5 Nm.
- For comparison, startup reaches 5–10 rad/s.

Correction: an earlier suspicion that `InitializeContext` used the wrong joint
order was wrong. The model input is `context_mujoco_qpos`, and `MotionSequence`
joints are stored in IsaacLab order, so only the motion-to-context path needs
the remap.

**TODO (deferred, not a priority):** on guard returns only, play the first
planner clip 2.5× slower, resampling positions and velocities and leaving
program startup unchanged. This would cut the peak to about 0.3 rad/s, and
the return would take about 3.7 s. Add a native unit test.

## 13. Left-arm-forward reference: design and offline evidence (2026-10-15)

**Approach,** taken from `~/work/lerobot` (`examples/g1_dex3_training`, roundtrip audit and first real
runs). SONIC streams 64-value latent tokens. A joint-space reference becomes tokens through the v1.1
encoder in mode 0: ten future frames 5 ticks apart at 50 Hz, absolute joints and velocities in IsaacLab
order, identity heading. lerobot also found that switching from the planner token to stream tokens at
once unloaded a foot, while a 2 s token blend did not.

**Models.** The local Hugging Face cache `nvidia/GEAR-SONIC@9c0ff22b` matches the robot: encoder
`fb97de22…`, decoder `34bae857…`, config `4a67713b…`, and the planner is identical to the pc2 copy.
pc2's `assets-manifest.json` records the same encoder hash. The encoder in the Psi0 checkout
(`013ab028…`) is **not** v1.1.

**Pipeline check** (`binary-compare/encode_check.py`, `base_choice.py`). Each baseline was encoded and
compared with the robot's logged idle token:

| Baseline encoded | Values differing from logged idle token |
|---|---|
| Planner model's final idle frame | 9 / 64, each by one step (0.0625) |
| Measured pose | 23 / 64, one step |
| Nominal legs, measured arms | 43–50 / 64, up to three steps |

The planner's idle legs are close to the measured hanging legs. The reference baseline is therefore the
measured pose, entered and left through 2 s blends from the planner's live token.

**Plan** (`arm_reference.py`):
1. Hold the measured pose for 1 s.
2. Move left shoulder pitch −0.3 rad forward over 2 s (min-jerk, peak 0.28 rad/s). The URDF axis is +y,
   so negative is forward.
3. Hold for 1 s.
4. Return over 2 s.
5. Hold for 1 s.

Tokens follow `token_schedule`: planner token → 2 s blend → motion tokens → 2 s blend → planner token.

**MuJoCo closed loop** (`demo_sim_arm_reference.py`). It uses lerobot's plant and decoder harness with
the session's 0.05/tick token limit. Output is in `hold-20261015/sim-arm-forward.txt`.

| Floor | Left shoulder pitch | Largest other joint | Peak speed | Tilt | Contacts | Saturation |
|---|---|---|---|---|---|---|
| No arm motion (control) | +0.005 | 0.112 (R hip pitch) | 0.07 | 1.8° | 8/8 | 0 |
| −0.3 rad forward | reached −0.269, ended −0.002 | 0.102 (R wrist roll) | 0.43 (L shoulder) | 2.2° | 8/8 | 0 |

The harness's band case, with the pelvis held 1 m up and the legs free, flails even with no arm motion
(knee 0.9 rad, 29 rad/s). It does not represent pc2's support, where the feet touch the ground (§9),
so it gives no evidence either way.

**Robot script** `arm-forward-test.py`:
- It encodes on pc2 (one CPU thread) before sending anything.
- It re-checks that the pose hasn't moved more than 0.05 rad, then drives the reviewed
  `SonicTaskSession` with one open-loop chunk. Its profile sets `max_observation_age` to the chunk
  length, because the chunk is planned from the start pose.
- It finishes with an explicit handback, then watches the return for 3 s.
- It cancels to the planner if any joint leaves its plan ± 0.15 rad, or exceeds 3 rad/s.

`demo_check_arm_forward.py` passes 6/6, and a mutation that disables the cancel is caught by its
prompt-handback assertions.

**Unverified:**
- behavior on the real support;
- encode time on pc2;
- whether pc2's `/home/unitree/dimos/.venv` has `onnxruntime`.

The last two are now verified on pc2: encoding takes 1.44 s, and the venv has `onnxruntime` 1.24.1. The
first is answered in §14.

## 14. First left-arm-forward run (2026-10-15 11:56, operator-run)

The operator ran `arm-forward-test.py --run` once from planner IDLE. The controller had been launched at
11:56:02, and the idle token was constant for 25.7 s before the test. The run **passed**. The session
finished on its own with no envelope or speed abort, and the handback was confirmed 0.12 s later.

Evidence in `arm-20261015/`:
- the session journal (`arm-forward-20261015-115655.jsonl`, SHA `1a8487cc…`);
- the native CSV logs (`20261015-115602/`);
- a journal excerpt (only ssh logins, no controller faults);
- `review.txt`, the output of `binary-compare/arm_run_review.py` (SHA `6d1f8f16…`).

| Phase | L shoulder pitch − base | Largest other joint | Peak speed | Torque / effort | Tilt (p-p) |
|---|---|---|---|---|---|
| Idle, 5 s before | 0.000 | 0.000 | 0.07 | 0.22 (L ankle pitch) | 0.05° |
| Seed + arm (1.08 s) | 0.000 | 0.014 (R wrist roll) | 0.60 (R wrist roll) | 0.22 | 0.02° |
| Blend in (2 s) | +0.001 | 0.051 (R wrist pitch) | 0.24 | 0.23 | 0.22° |
| Motion (7 s) | **−0.296** | 0.112 (R wrist roll) | 0.70 (L elbow) | 0.24 | 1.02° |
| Blend out (2 s) | +0.017 | 0.064 (L wrist pitch) | 0.27 | 0.21 | 0.04° |
| Planner return (3 s) | −0.002…+0.019 | 0.104 (R wrist pitch) | 0.47 (L wrist roll) | 0.24 | 0.91° |

- **Tracking and timing.** The shoulder reached −0.296 of the −0.30 plan. MuJoCo predicted −0.269.
  The shoulder was past half depth from +4.14 to +7.00 s after the chunk started; the plan was
  +4.0 to +7.0 s. The deepest point came at +4.86 s.
- **Rest of the body.** The largest other joint moved 0.112 rad (right wrist roll); MuJoCo predicted
  0.102. The cancel margin is 0.15 rad, so the run had 0.04 rad of headroom. A larger `--delta` may need
  a larger margin.
- **No faults.** No motor errors, the hottest driver was 48 °C, and torque never exceeded 24% of effort.
- **Planner return.** The return peaked at 0.47 rad/s, below the 0.77–1.0 rad/s of the hold runs. Its
  cause is the same idle-settle clip (§12a, sync TODO 4).
- **End pose.** The final pose was within 0.047 rad of the start (left elbow).
- **Handoff blend.** The 23-value handoff (§13) was smooth: the first blend tick moved each value by at
  most 0.001.
- **Session step limit.** It engaged on 79 of 550 ticks. Encoder tokens jump by 1/16, which the
  0.05/tick limit spreads over two ticks.
- **`rejected: 51`.** These are the session's seeding frames, sent before arming. The hold runs show 50
  per session.
- **Arming blip.** This is pre-existing and not caused by the arm test. On the 1–2 ticks after
  `planner=false`, the applied token differs from the seed token in 7–8 values by one step. It then
  returns to the seed token before any stream change. All three hold runs show it too. The right
  wrist roll moved at 0.21–0.60 rad/s for about 0.1 s and stayed within 0.014 rad. It is not fixed;
  the likely origin is the controller's planner-to-stream switch, which hasn't been investigated. Timing rules out the SONIC start and the normal-to-planner switch:
  - `start-idle-planner.py` sends start and planner together. The logs show planner motion from row 1
    in this run and row 4 in the hold session, i.e. 20–80 ms after the log starts.
  - Each blip came 11.8–78.9 s after that, on the tick after `planner=false`.
  - The planner token had not changed for 10.0–35.1 s before each blip.

**Offline check of the arming blip's source (`binary-compare/blip_source.py`, output in
`arm-20261015/blip-source.txt`).** I encoded candidate poses offline with the v1.1 encoder and compared
them with the token logged on the first tick after `planner=false`. Counts are values that differ by one
step out of 64:

| Encoded pose | vs blip token | vs seed (idle) token |
|---|---|---|
| Planner final frame (seed 1234) | 6–8 | 9–11 |
| Measured pose at arming | 24–28 | 22–24 |
| Nominal stance | 53–54 | 48–51 |

- **No pose reproduces the blip exactly.** The planner's final frame is the closest candidate, and in
  all four runs the blip is closer to it than the seed token is.
- **Offline encoding can't settle the cause.** It doesn't reproduce the seed token exactly either; there
  are already 9–11 one-step differences there. So the planner-final hypothesis is only plausible, not
  proven.
- **The blip is systematic.** Values 25, 35 and 42 flip in all four runs, and run 3's blip token is
  identical to run 2's. That points to a fixed change in the encoder input on the switch tick, not to
  noise.
- **Confirming it needs the controller source.** The next step would be to read how the controller
  builds the encoder input on its planner-to-stream switch.

**Arming blip: root cause from the controller source.** I read the source in `pc2-native-source.tar.gz`
with `pc2-223-native.patch` (SHA `c945aa9b…`) applied.

How the switch works:
1. `planner=false` makes `ZMQManager::update` (`zmq_manager.hpp`) switch to `STREAMED_MOTION`. It calls
   `TriggerSafetyReset` and then `TriggerZMQToggle`.
2. On the next input tick, `ZMQEndpointInterface::handle_input` (`zmq_endpoint_interface.hpp`) handles
   the safety reset:
   - it clears the external token (`has_external_token_state_ = false`);
   - it freezes a copy of the planner motion (`temporary_motion`, `play = false`);
   - it sets `reinitialize_heading = true`.
3. The toggle then arms the lease and drops any pending packet (`has_new_data_ = false`).

Until the next stream packet is decoded, `GatherTokenState` (`g1_deploy_onnx_ref.cpp`) runs the local
encoder on that frozen planner frame. `UpdateHeadingState` re-anchors the heading to the robot's current
base yaw. Encoder mode 0 uses `motion_anchor_orientation_heading_10frame_step5`, which is orientation
mode 1, so the anchor's yaw error drops to 0 while the planner root's roll and pitch are kept.

Offline reproduction (`binary-compare/blip_heading.py`, SHA `e8da99e2…`; output in
`arm-20261015/blip-heading.txt`). I encoded the planner final frame with its own root roll and pitch
(about 2.5° of pitch) and zero yaw:
- It matches the blip token exactly in hold runs 2 and 3 (0/64 values differ), and within 2/64 in hold
  run 1 and the arm run.
- The idle (seed) token is matched best at a yaw error of −1.25° to −2° in the hold runs (1/64), and at
  −0.5° in the arm run (6/64).

So the blip is the controller re-zeroing a 0.5–2° heading mismatch between the robot and the planner
for the 1–2 ticks before the first stream frame is applied. It happens inside the controller, after the
switch command, so DimOS cannot avoid it. A fix would be a controller change: keep the last token, or
pre-load the session's seed, until the first stream frame. That would need review, a rebuild and
operator authorization, which isn't warranted for a ≤0.014 rad wrist motion.

**Side finding: possible improvement, not made.** `arm_reference.py` encodes with an identity anchor,
which means an upright root. The planner's idle frame has the root pitched about 2.5°. Using that root
roll and pitch in the anchor reproduces the controller's encoding much more closely (0–2/64 against
6–10/64 for the identity anchor). It would likely shrink the 23-value handoff difference (§13).

**Planner-frame baseline: tried, rejected (2026-10-15).** The user accepted the ~2° heading error and asked
to proceed. `arm_reference.py` gained `IdlePlanner` (the planner's idle frame, hash-checked
`39b553e1…`) and an `anchor` argument to `Encoder.encode`; the default is the identity anchor, so v1
behaviour is unchanged. The new `arm_reference.py` has SHA `e332683…` and its self-check passes.

On the logged robot data, the planner-frame start cut the handoff difference from 22–24/64 to 7–8/64. It
was briefly installed on pc2 and verified there: planner 0.28 s, encoding 1.46 s. The MuJoCo floor matrix
then showed it makes the arm motion worse. The script is `binary-compare/sim_anchor_matrix.py`, and its
output is in `hold-20261015/sim-anchor-matrix.txt`. The planner token stand-in is the planner frame with
its root anchor.

| Arm reference baseline | Handoff | Other joint, −0.3 rad move | Tilt | Other joint, no move |
|---|---|---|---|---|
| Measured joints, identity anchor (v1, as run) | 37/64 | 0.106 (R wrist roll) | 2.8° | 0.096 |
| Measured joints, planner anchor | 33/64 | 0.114 | 3.5° | 0.110 |
| Planner joints, identity anchor | 5/64 | 0.191 | 3.4° | 0.147 |
| Planner joints, planner anchor | 0/64 | **0.204** (R wrist roll) | 4.4° | 0.064 |

- The v1 row matches the robot run, where the other joint reached 0.112. So 0.204 would likely trip the
  0.15 rad envelope.
- The robot's real handoff under v1 was already smooth: 0.05 rad and 0.24 rad/s during the blend-in (§14).
- So the arm test was reverted to v1 locally and on pc2: `arm-forward-test.py` `3cc802b2…` and
  `demo_check_arm_forward.py` `c658887e…`. `demo_sim_arm_reference.py` was also reverted locally
  (`dcd35c89…`). Checks pass on both.
- The new `arm_reference.py` stays installed; with its default identity anchor the v1 tokens are
  unchanged.

**Larger move, offline (`hold-20261015/sim-arm-delta.txt`).** With the v1 reference on the floor, `--delta`
of −0.4 and −0.5 reach −0.373 and −0.494 rad. Side motion stays at 0.100–0.102 rad, against 0.106 at −0.3,
and tilt stays at 2.7°. The script's limit is |delta| ≤ 0.5. The planned peak speed is 0.47 rad/s
(min-jerk, 1.875·0.5/2 s). The left shoulder pitch envelope becomes 0.65 rad. The other joints keep the
0.15 rad margin, so the robot's 0.112 at −0.3 leaves the same headroom if side motion doesn't grow.

## 15. Left-arm-forward at −0.5 rad (2026-10-15 13:06, operator-run): envelope cancel

`arm-forward-test.py --delta -0.5 --run` was cancelled at +7.49 s into the chunk, on the return stroke,
with **"right_wrist_roll left its envelope (+0.155 rad)"**. The handback was confirmed, the robot
returned to the planner, and there were no motor errors. The hottest driver was 43 °C and torque stayed
at or below 20% of effort.

Evidence in `arm-20261015/`:
- the session journal (`arm-forward-20261015-130605.jsonl`);
- the native CSV logs (`20261015-130516/`);
- `review-1306.txt`;
- a journal excerpt (no controller faults; two unrelated docker containers ran at 13:06:33 and 13:06:54,
  after the run).

**Tracking.** The left shoulder pitch reached −0.463 of the −0.5 plan (93%; 99% at −0.3). The decoder's
PD target went to −0.770, and the shoulder lagged it by up to 0.35 rad.

**Right wrist roll.** The decoder commanded this motion; the hand did not hit anything.
- Its PD target follows q within about 0.03 rad, and its torque stays at or below 0.31 Nm.
- It rose during the blend-in and outward stroke: +0.09 at +2.5 s and +0.12 at +3.5 s. It then rose
  again on the return stroke: +0.158 at +7.5 s.
- The −0.3 run shows the same pattern but smaller: a +0.110 peak at +4.0 s.
- The MuJoCo floor model did not predict this; it gave 0.100 rad (right hip pitch) at −0.5 (§14).

**Largest deviation from the start pose, by joint group, before the handback:**

| Run | Legs | Waist | L arm (sh/elbow) | L wrist | R arm (sh/elbow) | R wrist |
|---|---|---|---|---|---|---|
| −0.3 (11:56) | 0.020 | 0.024 | 0.037 | 0.091 | 0.039 | 0.112 |
| −0.5 (13:06) | 0.039 | 0.033 | 0.086 | 0.075 | 0.045 | **0.155** |

The 0.15 rad envelope was a first-commissioning choice and isn't derived from any hardware limit. Only
the wrists come near it. The legs and waist, which matter for balance, used a quarter of it or less.

**Per-group envelopes (user decision, 2026-10-15).** The user noted that SONIC generates G1 motion by its
own logic, so whole-body side motion is expected. `arm-forward-test.py` (`c4b50201…`) now cancels on
these limits:

| Joints | Limit beyond the plan |
|---|---|
| Legs and waist (`--margin`) | 0.15 rad |
| Shoulders and elbows (`--arm-margin`) | 0.20 rad |
| Wrists (`--wrist-margin`) | 0.30 rad |

The moving left shoulder pitch gets |delta| on top of its group limit. The 3 rad/s speed stop is
unchanged.

`demo_check_arm_forward.py` (`826c65d0…`) passes 8/8. Its two new checks are:
- a 0.2 rad right-wrist step passes;
- a 0.2 rad left-knee step cancels within 3 s.

A mutation that loosens every group to the wrist limit is caught. Both files were installed on pc2,
their hashes match, and the checks and preview pass there. The previous files are kept as `.v1`.

Against the −0.5 run, the worst values were 0.155 (wrists, limit 0.30), 0.086 (arms, 0.20) and 0.039
(legs and waist, 0.15).

**−0.5 rad retest with per-group envelopes (13:20, operator-run): PASS.** The controller was relaunched
(PID 24888, logs `20261015-131947/`) and `start-idle-planner.py` was run first. An earlier attempt at 13:16
on a relaunched controller had been refused with "No controller feedback; nothing sent", because
`start-idle-planner.py` hadn't been run after that launch.

Evidence in `arm-20261015/`: the session journal (`arm-forward-20261015-132021.jsonl`) and
`review-1320.txt`. No docker containers ran during the run.

- **Session.** It finished on its own with no abort, and the handback was confirmed 0.22 s later.
- **Shoulder.** It reached −0.476 of −0.5 (95%), on schedule: past half depth from +3.90 to +7.16 s
  against the planned +4.0 to +7.0 s.
- **No faults.** There were no motor errors, the hottest driver was 43 °C, torque stayed at or below 18%
  of effort, and tilt stayed at or below 1.0°.
- **Return and end pose.** The planner return peaked at 0.31 rad/s, and the end pose was within 0.061 rad
  of the start.

| Group | Worst deviation | Limit | Used |
|---|---|---|---|
| Legs | 0.054 (L hip yaw) | 0.15 | 36% |
| Waist | 0.031 | 0.15 | 20% |
| Arms | 0.083 (L shoulder yaw) | 0.20 | 41% |
| Wrists | 0.103 (R wrist pitch) | 0.30 | 34% |

**Run-to-run variation.** The right wrist roll peaked at only 0.049 this time; at 13:06 it reached 0.158.
So decoder side motion varies between runs from the same command, and the 13:06 cancel was the high end
of that spread. The legs' 0.054 is the highest of the three arm runs (0.020, 0.039, 0.054) and is
worth watching.

## 16. Agent-driven left-arm move (2026-10-15 16:59, operator-run): PASS

The operator ran `unitree-g1-sonic-arm-agentic` on pc2 through `run-arm-agent.sh`, using the Genon
gateway with `openai:openai/gpt-5.6-luna`. The controller was launched at 16:57, with logs in
`20261015-165714/`, followed by `start-idle-planner.py`.

Evidence in `agent-20261015/`:
- the session journal (`db821bd4….jsonl`);
- the native CSV logs;
- the DimOS run log (`dimos-main.jsonl`);
- `review.txt`.

No docker containers ran during the run. Another session's navigation graph log on pc2 last wrote at
16:52, before this run.

**Conversation:**
1. "What can you do?": the agent described the arm skill and its 1–28° range. It also mentioned
   McpServer's built-in non-motion tools (`agent_send`, `server_status`, `list_modules`), and said it
   cannot walk, turn, change modes, start or stop the controller, or use cameras or speech.
2. "swing your left arm forward 17 degrees": one tool call, `move_left_arm_forward(17)`. It returned
   OK in 16.7 s, and the agent reported "15.9°" with control handed back to the planner.

**Motion** (the same phase review as §14 and §15):
- The left shoulder pitch reached −0.277 of −0.297 rad (93%), on schedule: past half depth from +4.16 to
  +6.98 s, against the planned +4.0 to +7.0 s. The 11:56 script run at the same angle reached 99%.
- Worst deviation by group: legs 0.026, waist 0.031, arms 0.045, wrists 0.125 rad. Those are 17%,
  21%, 23% and 42% of the group limits.
- The peak stream speed was 0.66 rad/s (left wrist roll), and tilt stayed at or below 0.5° during the
  stream.
- There were no motor errors. The hottest driver was 44 °C, and torque stayed at or below 12% of
  effort.
- The planner return peaked at 0.56 rad/s, and the end pose was within 0.044 rad of the start.

## 17. More upper-body motions: offline design and evidence (2026-10-15)

**Directions** come from the SONIC MuJoCo model (`scene_43dof.xml`): the left hand's displacement in the
torso frame (x forward, y left) for a ±0.3 rad change from the nominal stance.

| Joint | Positive change | Range (rad) | Nominal |
|---|---|---|---|
| L shoulder pitch | hand back 0.10, so negative is forward | −3.09…+2.67 | 0.2 |
| L shoulder roll | hand outward 0.08 and up; negative goes into the torso | −1.59…+2.25 | 0.2 |
| L shoulder yaw | hand outward 0.05; unintuitive with the elbow bent, so not used | −2.62…+2.62 | 0 |
| L elbow | forearm down and back, so negative raises it | −1.05…+2.09 | 0.6 |

**MuJoCo floor check at each motion's maximum.** The script is `binary-compare/sim_motions.py`, with
output in `hold-20261015/sim-motions.txt`. It uses the v1 reference (measured joints, identity anchor)
and the planner-frame stand-in token. The control run, with no motion, gives side motion 0.096 and
tilt 2.8°.

| Motion | Planned (rad) | Reached | Largest other joint | Peak dq | Tilt | Contacts |
|---|---|---|---|---|---|---|
| left_arm_forward (robot-validated) | −0.489 | −0.494 | 0.103 (R hip pitch) | 0.69 | 2.7° | 8/8 |
| left_arm_sideways | +0.489 | +0.465 (95%) | 0.195 (R wrist roll) | 0.72 | 2.7° | 8/8 |
| left_forearm_raise | −0.489 | −0.428 (88%) | 0.115 (R elbow) | 1.04 | 2.8° | 8/8 |
| waist_turn_left | +0.262 | +0.125 (48%) | 0.195 (L wrist roll) | 0.44 | 2.7° | 8/8 |
| waist_turn_right | −0.262 | −0.047 (18%) | **0.212 (R shoulder pitch)** | 0.42 | 2.5° | 8/8 |

SONIC barely tracks an isolated waist yaw, and the right turn's side motion would exceed the 0.20 rad
arm envelope. Waist turns are therefore excluded.

**Skill change.** `move_left_arm_forward(angle_deg)` became `move_upper_body(motion, angle_deg)`, with
the motions `left_arm_forward`, `left_arm_sideways` and `left_forearm_raise`, each 1–28°.
- The moving joint gets |delta| on top of its group envelope. Reach is measured on that joint, in the
  motion's direction.
- `sonic_arm_skill.py` (`885beec1`) and `test_sonic_arm_skill.py` (`a9432257`) pass 19/19 locally and on
  pc2.
- Two deliberate breakages are caught: removing the moving joint's widened envelope (6 tests fail) and
  flipping the sideways sign.
- One unexplained single failure appeared in 21 pc2 runs; it didn't reproduce in 20 more runs.
- **Local agent run** (`gpt-5.6-luna` through Genon, fake controller):
  - "Raise your left arm out to the side about 15 degrees" called `left_arm_sideways` 15.
  - "Lift your left forearm by bending the elbow, 20 degrees" called `left_forearm_raise` 20.
  - "Turn your waist left" was declined.
  - The two calls ended in correct envelope cancels: the fake controller always moves shoulder pitch,
    whatever the commanded joint.
- **Not yet run on the robot:** `left_arm_sideways` and `left_forearm_raise`.

## 18. First robot runs of the new motions (2026-10-19 09:24–09:26, operator-run)

The operator ran `unitree-g1-sonic-arm` (MCP only) on pc2 with `move_upper_body` sideways and forearm raise at
10, 20 and 28°. Evidence in `motions-20261019/`:
- the six session journals;
- the native CSV logs (`20261019-092240/`);
- the DimOS run log;
- `review.txt`, the output of `binary-compare/motions_review.py`.

Every session finished with the handback confirmed and no motor errors. The skill results:

| Motion | Planned | Reached | Skill result |
|---|---|---|---|
| left_arm_sideways | 10° | 7.7° (77%) | OK |
| left_forearm_raise | 10° | 4.8° (48%) | **NOT_REACHED** (a returned result, not an error) |
| left_arm_sideways | 20° | 18.8° (94%) | OK |
| left_forearm_raise | 20° | 12.9° (65%) | OK |
| left_arm_sideways | 28° | 28.2° (101%) | OK |
| left_forearm_raise | 28° | 26.0° (93%) | OK |

**Small moves track poorly.** The shortfall is roughly constant: sideways 2.3–1.2° and forearm 5–7°, so
it dominates small angles. That pattern points to a deadband, not a gain error. Sideways at 28° and
forearm at 28° match MuJoCo (95% and 88%).

**Group envelope use** (worst sample per run):

| Group | Range across the six runs |
|---|---|
| Legs | 12–60% |
| Waist | 12–21% |
| Arms | 23–60% |
| Wrists | 35–59% |

The 60% values are sideways at 20°: legs 0.090 rad and arms 0.120 rad. Peak speed was 0.51–0.99 rad/s
(left wrist roll or elbow), tilt 0.7–2.0° peak to peak, and the end pose within 0.05 rad.

**Hardware warning: a loaded left ankle and fast-rising ankle drivers.** This comes from the stance
and support, not from the moves.
- **Load.** The left ankle pitch carried 39% of its torque limit on average throughout: idle before
  the runs, during them and after. On 10-15 the highest torque anywhere was 12–24%.
- **Lopsided stance.** Left ankle pitch was −0.155 rad against right −0.221, so the robot was leaning
  on its left foot.
- **Heating.** Driver temperatures, left ankle roll / left ankle pitch:

| Time into the log | 0 s | 35 s | 161 s (end) |
|---|---|---|---|
| Left ankle roll driver | 35 °C | 66 °C | **80 °C** |
| Left ankle pitch driver | 35 °C | 51 °C | 58 °C |

  Both were still rising at about 10–11 °C/min when the log ended. The right ankle drivers stayed at
  45–48 °C. The ankle's two motors share the load through its linkage, which is consistent with both
  drivers heating.
- **Before the next session,** check the support height and foot contact, and keep sessions short
  until the left ankle load is back near the 10-15 levels.

**Correction to earlier reviews.** `motor_temperature.csv` holds 58 columns: winding and driver
temperature for each of 29 motors, interleaved. `arm_run_review.py` (§14–§16) used the first 29
columns, so the hottest-motor figures there were mislabeled and incomplete. Recomputed:

| Run | Winding max | Driver max | Reported earlier |
|---|---|---|---|
| 11:56 | 47 °C (L wrist pitch) | 48 °C (L ankle roll) | 48 °C, R ankle roll |
| 13:06 | 50 °C (R shoulder pitch) | 49 °C (L shoulder pitch) | n/a |
| 13:20 | 50 °C (R shoulder pitch) | 49 °C (L shoulder pitch) | 43 °C |
| 16:59 | 48 °C (R shoulder pitch) | 49 °C (waist pitch) | 44 °C, L knee |

The conclusions there stand: nothing was near a thermal concern. `motions_review.py` reads the columns
correctly.

## 19. Per-motion angle limits and a speed cap (2026-10-19, offline)

**Model path.** `~/work/Psi0` has been removed, so the sim scripts now load the identical
`scene_43dof.xml` (SHA `274e7b37…`) from `~/work/GR00T-WholeBodyControl-hand-tracking-watchdog`.
This reproduces the §17 result exactly: forward 28° reaches −0.494.

**MuJoCo floor sweep** (`binary-compare/sim_sweep.py`; output in `hold-20261015/sim-sweep.txt`). Each
stroke is slowed so the min-jerk peak speed stays at 0.47 rad/s (the commissioned 0.5 rad over 2 s):
T = max(2 s, 1.875·|Δ|/0.47). The sim's legs sit about 0.07 rad above the robot's even with no motion.

| Motion | Reach at 10 / 20 / 28 / 40 / 50 / 60° | What limits it |
|---|---|---|
| Forward | 108 / 93 / 101 / 99 / 102 / 104% | at ≥40° legs 0.154–0.163 rad (limit 0.15) and floor contacts drop 8 → 6–7 |
| Sideways | 93 / 92 / 95 / 93 / 98 / 95% | at 40–60° other arm joints 0.161–0.179 (limit 0.20) and wrists 0.234–0.271 (limit 0.30) |
| Forearm raise | 56 / 98 / 88 / 96 / 96 / 95% | none: legs ≤ 0.110, arms ≤ 0.116, wrists ≤ 0.094, 8/8 contacts |

**Small-move shortfall on the robot.** Sideways at 10° reached 77% on the robot against 93% in the sim.
The arm joints run on soft gains (kp ≈ 14), so the robot's joint friction leaves a few-degree static
offset. The decoder already commands past the target to partly make up for it. The elbow shows the
effect in the sim as well (56% at 10°).

**Clearance** (`binary-compare/arm_clearance.py`; output in `hold-20261015/arm-clearance.txt`). This
measures the kinematic distance from the left upper arm, elbow and wrist collision geoms to the rest
of the body, excluding the absent Dex3 hands and the shoulder-roll link next to the torso.
- Sideways never comes closer to the body than at the start up to 60°.
- Forward's closest point is upper arm to torso, 12.5 → 4.5 cm from 28° to 90°.
- Forearm's closest point is elbow to torso, 7.5 → 4.6 cm.
- A few isolated 0.0 cm readings come from the mesh distance solver. Their neighbors are 5–9 cm and
  MuJoCo finds no contacts at those angles, so they are not real. Treat this check as approximate.

**Change.**
- `MOTIONS` carries per-motion ranges: forward 10–28° (unchanged maximum), sideways 10–40°, forearm
  raise 20–60°.
- `arm_poses` stretches each stroke to keep peak speed at or below `PEAK_SPEED` = 0.47 rad/s.
- The blueprint's `max_observation_age` goes from 12 to 17 s; the longest plan is 15.4 s.
- The agent prompt lists the ranges.

**Checks.**
- 25/25 tests pass locally and on pc2. New tests cover each motion's range and the speed cap; removing
  the cap fails a test.
- With the real encoder on pc2:

| Motion at its maximum | Plan length | Encode time | Max \|token\| |
|---|---|---|---|
| Forward 28° | 11.0 s | 1.5 s | 0.375 |
| Sideways 40° | 12.6 s | 1.8 s | 0.312 |
| Forearm raise 60° | 15.4 s | 2.4 s | 0.250 (bound 1.25) |

  A forearm-60° call takes about 22 s in all, under the 30 s MCP timeout.
- Untested on the robot: sideways above 28° and forearm raise above 28°.

## 20. Step-up runs above 28° (2026-10-19 09:57–09:59, operator-run)

The operator ran sideways at 34° and 40° and forearm raise at 40° and 60°; 50° was skipped. Evidence in
`motions-20261019b/`: the journals, the native CSV logs (`20261019-095617/`), the DimOS run log and
`review.txt`. All four finished with the handback confirmed and no motor errors. The 60° call took
22.0 s.

| Motion | Planned | Reached | Legs | Arms (other joints) | Wrists | Peak dq | Tilt p-p |
|---|---|---|---|---|---|---|---|
| Sideways | 34° | 34.7° | 43% | 66% | 62% | 0.92 | 1.3° |
| Sideways | 40° | 37.5° | 28% | **94%** (L shoulder yaw +0.188) | 59% | 0.95 | 1.5° |
| Forearm raise | 40° | 34.8° | 14% | **88%** (R shoulder yaw −0.175) | 66% | 0.98 | 2.0° |
| Forearm raise | 60° | 58.6° | 12% | **99%** (R shoulder yaw −0.198) | 53% | 1.42 | 1.6° |

**Where the arm-group peaks come from.** SONIC's decoder commanded them; nothing failed to track and
nothing collided. The same class as the wrist motion in §15:
- **Sideways:** the left shoulder yaw rotates with the abduction. Its PD target leads q, at about
  1.4 Nm.
- **Forearm raise:** the right shoulder yaw follows its own target (target − q ≈ 0) at about 0.2 Nm.

MuJoCo under-predicted the forearm case: other arm joints at ≤ 0.116 there, against 0.175–0.198 on the
robot. At 99% the next run at these angles may cancel. That is safe, since it hands back to the
planner, but it would be a nuisance.

**Hardware warning, worse than §18.** The left ankle pitch carried 42% of its torque limit on average,
and the stance was more lopsided: left ankle pitch −0.147 against right −0.247.

| Time into the 133 s log | 0 s | 30 s | 60 s | 120 s | 133 s |
|---|---|---|---|---|---|
| Left ankle roll driver | 43 °C | 75 °C | 81 °C | 86 °C | **91 °C** |

It was still rising at +15 °C/min. The left ankle pitch driver peaked at 70 °C. The original 0930c run
reached 90 °C, but over 317 s. **Stop moving tests until the support and foot contact are fixed.**

After this session, SONIC was running again on pc2 from `just run g1_deploy_onnx_ref …`, not the
guarded launcher.

## 21. Shoulder yaw joins the loose envelope (2026-10-19, user decision: option A)

`group_margins` now uses three groups:

| Group | Joints | Limit |
|---|---|---|
| Legs and waist | 0–14 | 0.15 rad |
| Arms | shoulder pitch and roll, elbows (15, 16, 18, 22, 23, 25) | 0.20 rad |
| Loose | wrists and shoulder yaws (17, 19–21, 24, 26–28) | 0.30 rad |

The reason is §20: SONIC's decoder rotates the shoulder yaws by itself, at about 0.2–1.4 Nm, by up to
0.198 rad during left-arm moves.

The skill's reported groups now match the envelope groups (`arms`, `wrists_shoulder_yaw`) and leave out
the moving joint. Before, a sideways or forearm move reported its own swing as arm deviation, and the
arm group omitted the left shoulder pitch.

Re-reviewed with these groups, the §20 step-up runs peak at:

| Motion | Arms | Wrists and shoulder yaw |
|---|---|---|
| Sideways 34° | 65% | 62% |
| Sideways 40° | 58% | 63% |
| Forearm raise 40° | 49% | 66% |
| Forearm raise 60° | 44% | 66% |

**Checks.**
- 29/29 tests pass locally. New tests: shoulder yaw at 0.25 rad passes, while shoulder pitch and elbow
  at 0.25 cancel; the moving joint is not reported as a deviation.
- Two deliberate breakages are caught: shoulder yaw back in the arm group, and the report including the
  moving joint.
- On pc2: `sonic_arm_reference.py` `cb0bebde`, `sonic_arm_skill.py` `1c26f75e`, tests `9a74931e`.
  Earlier copies are kept as `*.before-yaw.py`.

**Intermittent test failure.** It appears about once in 30 runs on pc2 and has not reproduced in 20+
local runs, including under CPU load. A capture loop is recording it.

## 22. Robot retest with the shoulder-yaw envelope (2026-10-19 12:5x, operator-run)

The operator restarted SONIC (guarded launcher, 12:52) and the blueprint (12:53, running the §21 code),
then ran forearm raise 60°, sideways 40° and forearm raise 50°. All three succeeded with the handback
confirmed and no motor errors. Evidence in `motions-20261019c/` (journals, CSV logs, `review.txt`).

| Motion | Reached | Legs | Arms | Wrists and shoulder yaw (worst joint) |
|---|---|---|---|---|
| Forearm raise 60° | 56.4° (94%) | 29% | 48% | 55% (R shoulder yaw −0.164) |
| Sideways 40° | 37.4° (94%) | 20% | 39% | **94% (L shoulder yaw +0.283)** |
| Forearm raise 50° | 47.5° (95%) | 22% | 49% | 61% (R shoulder yaw −0.183) |

The forearm-60° case that used 99% under the old groups now uses 55%.

Sideways 40° is the new tight spot. The decoder rotates the moving arm's own shoulder yaw with the
abduction: its PD target leads q by +0.09 rad, at 1.38 Nm. That joint reached +0.188 at this angle in
§20 and +0.283 now, so it varies about 1.5× between runs.

The user asked not to track ankle temperature from here on.

## 23. Full-range left-arm poses and joint targets (2026-10-19, offline)

The user wants the arm's full range, the way teleop drives SONIC, with both named poses and joint
targets, starting at 0.47 rad/s and raising the speed after verification.

**Kinematics** (G1 SONIC MuJoCo model).
- The elbow's zero is about 90° bent; +1.43 rad is straight.
- An overhead raise with shoulder roll 0.2 swings the hand inward across the head's centerline, so the
  overhead pose uses roll 0.5.

**MuJoCo floor check of the poses** (`binary-compare/sim_poses.py`; output in
`hold-20261015/sim-poses.txt`). All four left-arm joints move together on min-jerk strokes at
0.47 rad/s. The model does detect arm-versus-torso and arm-versus-pelvis contacts (30 collidable
pairs), so "no contacts" is meaningful.

| Pose (pitch, roll, yaw, elbow, rad) | Stroke | Reached | Side motion | Floor contacts / body contacts |
|---|---|---|---|---|
| forward_horizontal (−1.5, 0.2, 0, 1.2) | 5.9 s | −1.51, 0.25, −0.13, 1.21 | legs 0.121, R wrist+yaw 0.179 | 7 / none |
| side_horizontal (0.2, 1.5, 0, 1.2) | 5.0 s | −0.11, 1.48, −0.10, 1.30 | legs 0.118, R arm 0.120 | 8 / none |
| over_head (−2.8, 0.5, 0, 1.2) | 11.1 s | −2.80, 0.37, −0.13, 1.27 | legs 0.229, R wrist+yaw 0.281 | 8 / none |
| over_head_side (0, 2.1, 0, 1.2) | 7.4 s | −0.32, 1.98, −0.06, 1.06 | legs 0.138, R arm 0.243 | 8 / none |

SONIC coordinates the whole arm. For example, it adds −0.31 rad of shoulder pitch to the side raise.
Under the old per-joint envelope that would have cancelled. The sim's legs read about 0.07 rad high
(§19).

**The joint box can't be cleared offline** (`binary-compare/arm_box_check.py`; output in
`hold-20261015/arm-box-check.txt`). Of 8820 configurations, 2480 come within 2 cm of the body, mostly
at shoulder roll ≤ 20°, which includes the hanging rest pose 2.6 cm from the torso. The skill
therefore checks each planned path at runtime instead.

**Implementation.**
- `sonic_arm_reference.stroke_plan` moves several joints together, and `arm_poses` now calls it.
- `pose_margins`: legs and waist 0.15 rad; every left-arm joint 0.45 + its planned move; every
  right-arm joint 0.35.
- `ArmCollisionChecker` samples the planned path every 0.1 s, with a 1 cm margin on every geom. It
  checks the left upper arm, elbow and wrist against the torso, pelvis, legs and right arm, and
  ignores the absent hands.
- **New skills:**
  - `move_left_arm_pose(pose)`, with four named poses.
  - `move_left_arm_joints(pitch, roll, yaw, elbow)`, in degrees within pitch −165…45, roll −10…120,
    yaw −90…90 and elbow −30…82. It requires `collision_model_path`, and refuses an unsafe path with
    `UNSAFE_PATH` before sending anything.
  - Both report `reached_fraction`, the progress along the planned displacement, with `NOT_REACHED`
    below 0.5, plus planned and reached degrees per joint. The moving joints are left out of the
    deviation groups.
- `move_upper_body` keeps its tested envelope.
- **Blueprint and agent:**
  - The execution profile allows `max_observation_age` 35 s and `max_duration` 45 s.
  - The agentic blueprint sets `mcp_timeout` to 60 s.
  - The prompt describes both new skills.
  - `run-arm-agent.sh` passes `--collision-model-path` with pc2's `scene_43dof.xml` (same SHA
    `274e7b37…`).

**Checks.**
- 39/39 tests pass locally and on pc2, including the real collision model on both: the four poses are
  clear, and roll −0.7 hits `torso_link`.
- Three deliberate breakages are each caught: skipping the collision refusal, dropping the path-check
  requirement, and tightening the pose envelope back to 0.20.
- The real encoder's tokens for the poses stay at or below 0.625 (bound 1.25).
- Timing on pc2:

| Pose | Collision check | Encode | Plan | Whole call |
|---|---|---|---|---|
| Forward horizontal | 0.18 s | 3.4 s | 20.6 s | about 28 s |
| Side horizontal | 0.14 s | 2.8 s | 17.4 s | about 25 s |
| Overhead | 0.28 s | 5.6 s | 30.9 s | about 41 s |
| Overhead via the side | 0.19 s | 3.8 s | 22.2 s | about 30 s |

  The collision model loads in 0.66 s.
- Previous pc2 files are kept in `before-poses/`.
- **Untested on the robot.**

**First robot call refused (2026-10-19 16:2x): fix.** `move_left_arm_pose left_arm_side_horizontal`
returned `UNSAFE_PATH` for `left_hip_roll_link`, and nothing was sent.

**Cause.** The robot's idle left arm is not the nominal stance: pitch −0.02, roll 0.284, yaw −0.651,
elbow 0.984 (log `20261019-162228`). In that pose the wrist-yaw link already sits 1.87 cm from the
hip-roll link. The checker counted that from the very first sample, even though the side raise clears
the body from row 70 onward.

**Measured threshold.** MuJoCo adds the two geoms' margins, so 1 cm per geom means a 2 cm threshold.
The largest reported distance over 400 random poses was 1.99 cm. The wording now says 2 cm.

**Change.** Arm-to-body pairs already within the threshold at the start pose are allowed, unless the
path brings them more than 5 mm closer. Any new pair still refuses the move.

**Checks.** From the logged robot poses, at the start and end of the 16:22 session, all four poses are
clear. Rolling the idle arm 0.25 rad further in is still refused on `left_hip_roll_link`. A regression
test uses the robot's idle pose. 40/40 tests pass locally and on pc2: reference `f8fd61a0`, skill
`46725174`, tests `fdc966c4`.

## 24. First full-range runs (2026-10-21, operator-run, `--max-dq 2.0`) and the chunk-length fix

| Call | Result | Reached | Legs | Arms | Wrists and shoulder yaw | Peak dq |
|---|---|---|---|---|---|---|
| left_arm_side_horizontal | OK | 92% | 0.076 | 0.131 | 0.172 | 1.35 (L shoulder roll) |
| left_arm_forward_horizontal | OK | 94% | 0.086 | 0.060 | 0.176 | 1.08 (L shoulder pitch) |
| joints(−60, 30, 0, 50) | OK | 96% | 0.054 | 0.060 | 0.279 | 1.22 (L shoulder pitch) |
| left_hand_over_head_sideways | EXECUTION_FAILED before moving | 0% | n/a | n/a | n/a | n/a |
| left_hand_over_head | EXECUTION_FAILED before moving | 0% | n/a | n/a | n/a | n/a |
| joints(11, −10, 0, 34) ×2 | UNSAFE_PATH (left_hip_roll_link), nothing sent | n/a | n/a | n/a | n/a | n/a |

- **Coordination as in MuJoCo.** In the side raise, SONIC moved the shoulder pitch from the planned
  11.5° to −4.6°. The legs stayed at or below 57% of their limit.
- **Speed.** The peak was 1.35 rad/s, under the 2.0 rad/s stop.

**Cause of the two overhead failures.** `SonicTaskSession` accepts at most 1000 rows (20 s) per
chunk. Those plans are 1074 and 1460 rows (pc2 timings in §23 used a slightly different base), so
the first `submit_chunk` raised. The session then handed back to the planner and the robot did not
move.

**Fix** (`chunk_rows` in the skill).
- A plan of 1000 rows or fewer is still one chunk, as commissioned.
- A longer plan becomes two chunks: rows [0, hold+25) and [hold, end), where hold is the first row at
  full depth. The second chunk is submitted at its scheduled time.
- With the real encoder, the tokens change by 0.0000 per tick around the split. The session's blend
  restart therefore cannot ripple there.

**Tests.**
- The chunk boundaries are tested directly.
- A simulated-clock session test streams the 1545-row overhead plan through two chunks: both are
  accepted, the output stays within 0.005 of the plan, and nothing exceeds the plan's own per-tick
  change.
- Moving the split 150 rows into the stroke fails that test.
- 42/42 pass locally and on pc2: skill `b979dfb9`, tests `38fb184d`.

## 25. Overhead retest in MuJoCo, the mid-pose stop hazard and the smooth retreat (2026-10-21)

**A real-time MuJoCo stand-in for the guarded controller** (`sonic_sim_peer.py` in the sync
artifacts).
- It binds `g1_debug` feedback and subscribes to the execution module's command and pose references,
  with the guard semantics: planner idle, `planner=false` arms a new epoch, accepted and applied
  indices, and `planner=true` or a 200 ms stream gap returns to the planner.
- Tokens drive the SONIC v1.1 decoder and a floor-standing G1 from the lerobot harness: 50 Hz policy,
  200 Hz physics. The planner idle token is the planner model's final idle frame.
- It keeps real time: compute is 7.7 ms per tick typically, 17.7 ms at the 99th percentile, 24 ms at
  worst. The unchanged `unitree-g1-sonic-arm` blueprint ran against it over MCP, with the collision
  model, `--max-dq 2.0`, spare ports and a private Zenoh group.

**First retest** (`hold-20261015/calls.txt`, `sim_overhead.npz`). Side 93%, forward 95% and
joints(−60, 30, 0, 50) 98% completed. The chunk split worked: both overhead plans streamed. Both then
stopped early on the leg envelope (right hip pitch −0.150, left knee +0.152; the sim's legs read
about 0.07 rad high).

**The hazard.** The stop cancelled the session, so the native planner took over at once and snapped
the raised arm home within about 0.1 s:

| Stopped move | Arm at handback | Speed after handback | Tilt | Floor contacts |
|---|---|---|---|---|
| Overhead via the side | roll 1.95 | 17.2 rad/s (L shoulder roll) | 5.4° | 8 → 6 |
| Overhead | pitch −2.45 | 22.9 rad/s (L shoulder pitch) | 11.5° | **0, pelvis dropping** |

During streaming the peak speed was at most 1.05 rad/s.

**Fix: a smooth retreat** (`retreat_row` and `_stream` in the skill).
- On an envelope or speed stop, the skill resumes at the first row of the plan's way back whose
  progress is no further than the arm's. It submits the rest of the plan from there, which takes the
  arm back to the start and blends to the planner token, and only then hands back.
- Every way-back token encodes "from here, return to the start", so the stream stays continuous.
- An overspeed during the retreat still cancels at once.
- Tests: `retreat_row` unit cases; a leg stop now ends with the stream back at the planner token, the
  session outcome `finished` and no step above the limit; an overspeed during the retreat cancels.
  Going back to cancel-on-stop fails the smooth-return test. 44/44 pass.

**Second retest** (`calls2.txt`, `sim_overhead2.npz`).

| Move | Result | Reached | Peak speed | Tilt | Floor contacts |
|---|---|---|---|---|---|
| Overhead via the side | **completed**, two chunks | 93% | 0.80 rad/s | 2.7° | 8 |
| Overhead | stopped on right hip pitch (−0.153), then retreated | 49% | 1.24 rad/s (0.05 after handback) | 2.4° | ≥ 7 |

In the stopped overhead run, the arm was back at rest at handback.

**Remaining hazard (native).** If the stream is lost mid-pose, for example DimOS crashing or a
network gap, the guard's 200 ms expiry still hands straight to the planner and would snap the arm the
same way. Only the native side can fix that, by blending the planner's return (an extension of the
deferred sync-guide TODO 4). Until then, keep full-range poses supervised.

**pc2.** Unreachable ("No route to host"). pc2 runs the pre-#4404 code, so the retreat is backported
onto its exact file versions (`2aeb5ea04`). The backport is in `pc2-backport-retreat/`:
`sonic_arm_skill.py` `5b24e8c5` and tests `59aa0d56`, 44/44 on the old-API base. Install it before
any overhead run on the robot.

**Sim startup: the robot was hanging when SONIC started (user check, 2026-10-21).** The stand-in
first used the lerobot harness's `startup_hang`.

| Harness startup phase | Duration | Peak speed | Torque saturation | Tilt | Contacts |
|---|---|---|---|---|---|
| SONIC on the band, 1 m up | 1.0 s | 26 rad/s | 100% | 8.3° | 0 |
| Band released, free fall | 0.5 s | 21 rad/s | 100% | 10.5° | 0–6 |
| Reset onto the floor, decoder history kept | 3 s | 7.2 rad/s | 0% | 3.5° | 4–8 |
| Instant stand-to-planner token switch | 3 s | 2.6 rad/s | 0% | 5.4° | 7–8 |

Clearing the history at the reset only gives 6.4 rad/s, with contacts briefly at 0.

**New startup** (`startup_on_floor` in `sonic_sim_peer.py`). The model's start pose already stands on
the floor. A band holds the pelvis at that height through the 2 s INIT ramp and the start of SONIC,
which goes straight into planner idle as `start-idle-planner` does. The band then fades out over 2 s.

| New startup phase | Duration | Peak speed | Tilt | Contacts |
|---|---|---|---|---|
| SONIC starts, held (saturation only in the first ticks, empty history) | 1 s | 4.8 rad/s | 5.2° | 3–8 |
| Band fades out | 2 s | 2.3 rad/s | 2.8° | 7–8 |
| Free standing | 3 s | 0.20 rad/s | 0.7° | 8 |
| Free standing | next 10 s | 0.00 rad/s | 0.5° | 8 |

While serving: 0.004 rad/s peak and 0.47° tilt.

**Retest on the cleanly started robot** (`calls3.txt`, `sim_overhead3.npz`). All four poses completed;
compute 11.5 ms at the 99th percentile.

| Pose | Reached | Legs (limit 0.15) | Peak speed | Tilt | Contacts |
|---|---|---|---|---|---|
| Side horizontal | 92% | 0.083 | 0.82 rad/s | 1.6° | 8 |
| Forward horizontal | 89% | 0.109 | 0.87 rad/s | 2.1° | 8 |
| Overhead via the side | 93% | 0.139 | 0.87 rad/s | 1.8° | 8 |
| Overhead | **99%** | 0.145 | 0.80 rad/s | 2.3° | ≥ 7 |

The earlier "sim legs read about 0.07 rad high" came largely from the hanging startup. The offline
scripts `sim_motions.py`, `sim_sweep.py` and `sim_poses.py` (§17–§23) still use `startup_hang`, so
their leg numbers are pessimistic. Overhead legs at 0.145 is 97% of the 0.15 limit even in a clean sim.
The robot's legs have run lower so far (0.054–0.086 on the 10-21 runs), but overhead hasn't run there.
