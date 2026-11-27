# Bounded SONIC elbow reach using measured feedback

On 2026-10-07, an experimental DimOS-side reference controller reached a measured **+0.1 rad left-elbow goal** in isolated SONIC MuJoCo simulation. Two runs of the final runner passed the declared position and settling criteria. An earlier exploratory run also passed. The C++ SONIC runtime remained the only body/hand motor-command owner.

This is a single-joint, unloaded experiment. No PC2 connection, hardware command, package update or robot deployment occurred. The production DimOS modules were unchanged in this step.

## What changed

The preceding [encoder-parity investigation](/docs/platforms/humanoid/g1/sonic-encoder-parity.md) established input/token agreement but found poor open-loop tracking. This experiment sets the goal from the **measured starting elbow position**, then adjusts the encoder reference using subsequent measured error:

```text
measured goal = initial measured elbow + 0.1 rad
reference velocity = clip(0.8 × measured error, -0.05, +0.05) rad/s
reference += reference velocity × min(feedback interval, 0.05 s)
```

A 0.01 rad deadband stops further integration. The reference is clipped to ±0.25 rad around the captured planner reference, within an additional 0–2 rad experimental envelope. These are experiment limits, not certified G1 joint limits. Each token represents ten future samples spaced 0.1 seconds apart, with bounded elbow positions and matching velocities. Other body reference positions and the captured hand references remain fixed; SONIC still determines the actual whole-body response.

The pure correction helper (`/docs/artifacts/g1_sonic_feedback_reach_20261007/demo_elbow_servo.py`) has no transport or motor interface. The simulation runner (`/docs/artifacts/g1_sonic_feedback_reach_20261007/demo_feedback_reach_sim.py`) sends its encoded references through the previously tested ZeroMQ contract and consumes typed `SonicFeedback` with each packet's original local receipt timestamp.

## Criteria declared before the runs

| Condition | Required behavior |
|---|---|
| Reach completion | Every received sample across at least 0.5 s has absolute goal error ≤0.02 rad, elbow speed ≤0.1 rad/s, maximum arm speed ≤0.2 rad/s, maximum body speed ≤0.3 rad/s. |
| Completion coverage | Repeated samples do not advance completion. A gap over 0.08 s restarts the interval. |
| Stale feedback | At 0.15 s without a new consumed sample, end the attempt. Check this on scheduled sends even without new packets. |
| Deadline | End the attempt after 10 s without completion. |
| Reference saturation | End after 1 s of continuing to push outward at a reference bound. |
| Cancel | Latch cancellation; publish no further reach references. |
| Posture during reference publishing | Fresh sampled physics, support disabled, no automatic resets, finite state, pelvis height ≥0.5 m, pelvis tilt ≤10°. |
| End of attempt | Request idle-planner handback; require fresh confirmation and disarmed stream within 2 s. Then observe an overlapping feedback/physics settling interval of at least 0.5 s. |

The publisher checks the deadline and fresh armed-controller feedback again **after encoding**, immediately before returning a packet for sending. Faults during initial hold/setup also attempt bounded planner handback. These are application checks on a best-effort Python loop, not a hard real-time guarantee. The separate native 200 ms receiver lease remains active.

## Reach results

All goals were +0.1 rad from each run's measured start. Starting poses varied between independent simulator launches; this is not a deterministic paired comparison with the prior open-loop runs.

| Run | Runner | Time to completion | Measured change | Absolute final error | Reference correction from captured planner pose |
|---|---|---:|---:|---:|---:|
| reach01 | Exploratory | 6.45 s | +0.08844 rad | 0.01156 rad | +0.22612 rad |
| reach02 | Final | 5.10 s | +0.11540 rad | 0.01540 rad | +0.16680 rad |
| reach03 | Final | 7.40 s | +0.09819 rad | 0.00181 rad | +0.18424 rad |

The exploratory runner preceded the post-encoding revalidation and broader fault-handback fixes. Its exact source snapshot is retained in `reach01/runner-exploratory.py.txt`; it is supplementary evidence. The final-runner manifests record runner/helper hashes.

An independent offline audit (`/docs/artifacts/g1_sonic_feedback_reach_20261007/demo_audit_reach.py`) recomputes the completion interval from raw telemetry, checks the actual encoded future positions/velocities against the bounds, checks reference slew against original feedback receipt intervals, and checks recorded physics. It also validates the runner's reported handback, post-handback settling and cleanup results. The final error alone is insufficient to pass.

Measured elbow response, corrected encoder reference and planner handback (`/docs/artifacts/g1_sonic_feedback_reach_20261007/audit.png`)

The orange reference is offset from the measured starting elbow for display; its correction bound is relative to the captured planner reference. A larger reference change than measured movement was needed in these trials. This shows useful compensation in these cases, not a general inverse model of SONIC or proof that token quantization caused all previous errors.

**Planner handback moves the arm away from the reached pose.** The completion interval occurs before handback. This experiment does not hold a grasp pose after reporting reach completion and does not test a commanded return to the captured start.

## Fault trials and verification

Fault-trial results and final artifact audit are recorded in the accompanying [audit summary](/docs/artifacts/g1_sonic_feedback_reach_20261007/audit.json). Cancellation is injected after one second. The stale scenario stops delivery to the task's correction consumer after one second while the actual telemetry receiver and native controller continue operating. It tests consumer freshness handling, not physical sensor loss or a DDS outage.

| Trial | Observed outcome | Attempt ended | Observed planner confirmation after request |
|---|---|---:|---:|
| cancel01 | `CANCELLED` | 1.003 s after measured start | 0.123 s |
| stale01 | `STALE_FEEDBACK` | 1.143 s after measured start; 0.162 s after last consumed sample | 0.122 s |

All five trials passed the artifact checks and reported a sampled post-handback settling interval of 0.566–0.617 s. From initial arm hold through handback/settling, recorded support was off, automatic resets remained zero, and maximum pelvis tilt was below 4.58°. The reference rate averaged 49.75–50.01 Hz across individual runs. Each native controller exited with code 0; no cleanup errors or remaining probe processes were found.

The 28 unit tests (`/docs/artifacts/g1_sonic_feedback_reach_20261007/test_elbow_servo.py`) passed. They cover reference limits, timeout, cancellation, invalid samples, changed duplicates, stale boundaries, completion coverage, speed spikes and bounded future references. Timeout and reference-saturation termination were tested in the pure helper; those outcomes were not deliberately forced in the native simulation runs. Test evidence and source/model hashes are in [verification.json](/docs/artifacts/g1_sonic_feedback_reach_20261007/verification.json).

Reported settling and posture refer to received samples before deliberate controller shutdown. Native `stop` subsequently applies damping and can cause a simulated fall; shutdown is excluded from the standing/settling claim. The measurements use local monotonic receipt times and do not establish sensor acquisition freshness, continuous-time bounds, network latency or physical hardware accuracy.

## Reproduce on this workstation

Use the same loopback-only private namespace procedure and assets as the [expiry pilot](/docs/platforms/humanoid/g1/sonic-reference-expiry.md#reproduce-on-this-workstation), substituting:

- Probe: `docs/artifacts/g1_sonic_feedback_reach_20261007/demo_feedback_reach_sim.py`.
- Binary: `/tmp/dimos-sonic-expiry-20261006/gear_sonic_deploy/target/release/g1_deploy_onnx_ref`, SHA-256 `b9a95ae89b4ce328526715b0cee30689d06dbf5c23f23479dc64ddbfce19750e`.
- Scenario: `--scenario reach`, `--scenario cancel` or `--scenario stale`.
- A new output directory for each run. Existing directories are rejected.

The runner refuses the host namespace and any interface inventory other than loopback. It uses ports 15556/15557 inside the private namespace. Do not substitute a physical robot interface.

The audit operates only on saved files:

```bash skip
MPLCONFIGDIR=/tmp/dimos-reach-matplotlib .venv/bin/python \
  docs/artifacts/g1_sonic_feedback_reach_20261007/demo_audit_reach.py \
  docs/artifacts/g1_sonic_feedback_reach_20261007/reach01 \
  docs/artifacts/g1_sonic_feedback_reach_20261007/reach02 \
  docs/artifacts/g1_sonic_feedback_reach_20261007/reach03 \
  docs/artifacts/g1_sonic_feedback_reach_20261007/cancel01 \
  docs/artifacts/g1_sonic_feedback_reach_20261007/stale01 \
  --output docs/artifacts/g1_sonic_feedback_reach_20261007/audit.json
```

## Next service milestone

Test a bounded **reach → measured hold → controlled return** in simulation. A grasp needs the arm to remain near its target while a hand action executes; immediate planner handback does not provide that behavior. Measure drift, whole-body motion and cancellation throughout the hold and return before expanding to multiple arm joints, collision-aware Cartesian targets and Dex3 grasp/release.

The production command-session adapter, session identity, calibrated camera/robot geometry, Dex3 mapping/contact behavior, and supervised hardware commissioning remain open in the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md). These few successful unloaded elbow reaches are an implementation milestone, not readiness for the object-to-tray task.
