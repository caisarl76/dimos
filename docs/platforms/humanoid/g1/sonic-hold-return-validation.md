# SONIC elbow reach, measured hold and controlled return

On 2026-10-07, two isolated MuJoCo trials completed **reach → three-second measured elbow hold → return to the captured starting elbow position → planner handback**. The return endpoint errors were 0.00720 and 0.00706 rad. This extends the [bounded feedback-reach experiment](/docs/platforms/humanoid/g1/sonic-feedback-reach.md), which handed back immediately after reaching its goal.

One successful cycle recorded an arm-joint peak of 1.884 rad/s during motion. After reviewing the MuJoCo replay, the user identified hand-to-leg contact followed by the hand slipping free as the cause. We will set this event aside for free-space arm-speed tuning and proceed with a starting pose and motion that provide hand-to-thigh clearance. Full hand-pose stability remains to be measured.

An additional attempt failed the hold criterion after 0.259 s because another arm joint reached 0.218 rad/s. The runner ended that attempt and handed control back to the planner. The two successful cycles therefore do not establish consistent holding across starting conditions.

All work used the workstation's private, loopback-only simulation namespace. No PC2 connection, real robot command, deployment or production DimOS module change occurred. The C++ SONIC controller remained the only body/hand motor-command owner.

## Behavior and limits

The experimental cycle helper (`/docs/artifacts/g1_sonic_hold_return_20261007/demo_elbow_cycle.py`) extends the preceding measured-error correction with three active states. A single reference integrator and its original bounds persist through all states; returning does not recenter the reference envelope or reset measurement history.

| State | Goal and exit condition |
|---|---|
| `REACHING` | Initial measured elbow +0.1 rad. Require at least 0.5 s of qualifying feedback, within a 10 s deadline. |
| `HOLDING` | Keep the same measured goal for at least 3 s. Every consumed sample must qualify. Any position/speed violation or feedback coverage gap ends the cycle. |
| `RETURNING` | Original measured starting elbow. Require a fresh 0.5 s qualification interval, within a separate 10 s deadline. |
| `SUCCEEDED` | Reach, full hold and return all completed. Stop external references and request planner handback. |

A sample qualifies when goal error is ≤0.02 rad, elbow speed ≤0.1 rad/s, maximum arm speed ≤0.2 rad/s and maximum body speed ≤0.3 rad/s. These measured-speed thresholds apply to the completion intervals and throughout `HOLDING`; they are **not enforced as motion-speed ceilings throughout `REACHING` or `RETURNING`**.

The reference correction retains gain 0.8/s, a 0.01 rad error deadband, a 0.05 rad/s reference-slew bound and ±0.25 rad bounds around the captured planner reference. An additional 0–2 rad envelope is specific to the experiment, not a certified hardware limit. Other body reference positions and the captured hand references remain fixed, while SONIC determines actual whole-body movement.

In reach/return, a feedback gap over 80 ms restarts qualification; feedback age of 150 ms ends the attempt. During hold, a gap over 80 ms ends the attempt immediately as `HOLD_GAP`, including when no new packet arrives. Only a fresh qualifying sample can complete the three-second hold; a wall-clock timer alone cannot advance it. Duplicate or invalid samples do not advance state. Timeout, cancellation, reference saturation and hold failure are terminal and retain the stage in which they occurred.

The runner (`/docs/artifacts/g1_sonic_hold_return_20261007/demo_hold_return_sim.py`) retains pre/post-encoding freshness checks, the native 200 ms receiver lease, sampled posture checks, bounded planner handback and sampled settling. Reference publishing requires support off, zero automatic resets, finite physics state, pelvis height ≥0.5 m and tilt ≤10°. These are checks in a best-effort process, not continuous-time or hard real-time guarantees.

## Complete-cycle results

Initial poses varied across separate simulator launches. No controller gain, tolerance, duration or reference bound was tuned between these runs.

| Trial | Full cycle duration | Measured hold | Worst elbow error during hold | Final return error | Largest other left-arm joint drift during hold |
|---|---:|---:|---:|---:|---:|
| cycle01 | 13.60 s | 3.000 s / 151 samples | 0.01021 rad | 0.00720 rad | 0.03123 rad |
| cycle02 | 8.64 s | 3.021 s / 152 samples | 0.01384 rad | 0.00706 rad | 0.01706 rad |

Joint drift is the maximum absolute displacement from the first hold sample, excluding the controlled elbow, over the other six left-arm joints. It demonstrates why a successful elbow hold is insufficient to claim a fixed hand pose. This step did not measure end-effector translation/orientation or test object contact, grasp forces or load.

The offline audit (`/docs/artifacts/g1_sonic_hold_return_20261007/demo_audit_cycle.py`) independently reconstructs reach/return qualification from raw telemetry and checks every sample of the full hold. It checks reference bounds in the actual encoder inputs, reference continuity across transitions, original receipt timestamps, terminal publishing cutoff and recorded physics. Handback and subsequent settling checks use the runner's recorded results. [Audit data](/docs/artifacts/g1_sonic_hold_return_20261007/audit.json) distinguish task outcome from consistency of the recorded evidence.

First cycle: elbow goal, measured response and arm speed (`/docs/artifacts/g1_sonic_hold_return_20261007/cycle01/response.png`)

The orange line is the encoder reference relative to the initial measured elbow. Its limits are centered on the captured planner reference. The green goal changes at the start of return; the reference itself remains continuous. The red speed line is the qualification/hold threshold, not an enforced ceiling during movement.

## Motion transients

| Trial | Peak measured elbow speed during the cycle | Peak measured speed across both arms |
|---|---:|---:|
| cycle01 | 0.480 rad/s | 1.884 rad/s |
| cycle02 | 0.231 rad/s | 0.231 rad/s |

These peaks exclude later planner handback and deliberate shutdown. In cycle01 the largest arm peak was at body motor index 19 (left wrist roll), near 12.08 s. The trace shows relatively slow reference changes followed by faster measured movement; a reference slew limit does not itself impose the same limit on measured joint speed.

**User replay assessment, 2026-10-07:** the hand was in contact with the leg and then slipped free, producing this brief peak. The user instructed us to ignore this spike for the current progression. We record it as a user-identified contact-release event and set it aside for free-space speed tuning. This causal assessment comes from the user's visual review; contact forces were not independently analyzed. The recorded peak and original trial outcomes remain in the evidence. This assessment applies to the cycle01 return spike and does not reclassify the separate `cancel_return01` hold failure.

The MuJoCo replay (`/docs/artifacts/g1_sonic_hold_return_20261007/mujoco_replay/index.html`) includes the full-speed cycle and a quarter-speed view of this interval. The next free-space trial should provide hand-to-thigh clearance so contact release does not confound the tracking measurements.

## Failure handling and software verification

The runner includes cancellation during hold, a stalled task feedback consumer during hold, and cancellation during return. Each injection is scheduled 0.5 s after entering the relevant stage. The result must record that the injection actually occurred; reaching a different terminal condition beforehand is not a passed fault test. The stale-consumer scenario leaves the actual telemetry receiver and native controller running, so it does not simulate physical sensor or DDS loss.

| Trial | Outcome | Interpretation | Observed planner handback after request |
|---|---|---|---:|
| cancel_hold01 | `CANCELLED` in hold | Cancellation injected 0.500 s into hold; no further references published. | 0.245 s |
| stale_hold01 | `HOLD_GAP` | Attempt ended 0.0805 s after the last consumed sample. | 0.244 s |
| cancel_return01 | `HOLD_VIOLATION` | **Hold failed; return cancellation was not exercised.** Body motor index 26 reached 0.2184 rad/s against the 0.2 rad/s arm hold threshold. Elbow error remained 0.01538 rad. | 0.162 s |
| cancel_return02 | `CANCELLED` in return | Unchanged repeat; cancellation injected 0.517 s into return after a full three-second hold. | 0.163 s |

The stale-hold report may describe physical telemetry still received by the observer after the task consumer was cut off. Those rows are not claimed as continuous task-consumed hold coverage. The failed return-cancellation attempt is retained in the aggregate results alongside the successful repeat. Passing evidence-consistency checks does not change that attempt's failed intended outcome.

Across all six trials, the recorded arm-hold/cycle/handback windows had support disabled, no automatic resets and maximum pelvis tilt below 4.78°. Every trial obtained a sampled post-handback settling window of 0.565–0.626 s, including the failed hold. All native controllers exited with code 0, cleanup reported no errors, and the final process check found no remaining probe/controller processes. Reference publishing averaged 49.67–49.86 Hz across individual runs.

The 26 unit tests (`/docs/artifacts/g1_sonic_hold_return_20261007/test_elbow_cycle.py`) passed. They cover complete phase ordering, reference continuity, original bounds, hold violation, command-time hold gaps, cancellation/staleness in each stage, distinct stage deadlines, duplicate/invalid feedback and saturation. The tests first exposed 17 failures when run against the copied reach-only helper, then passed after the phase extension. No entire-repository test result is claimed for this artifact-only change.

Each native run records the runner/helper hashes, controller command and namespace identity. [Verification evidence](/docs/artifacts/g1_sonic_hold_return_20261007/verification.json) records software checks. The preceding experiment's artifacts and the other session's implementation-log files remain unchanged.

## Reproduce

Use the private namespace and asset procedure in the [receiver-expiry pilot](/docs/platforms/humanoid/g1/sonic-reference-expiry.md#reproduce-on-this-workstation), with:

- Probe: `docs/artifacts/g1_sonic_hold_return_20261007/demo_hold_return_sim.py`.
- Scenario: `--scenario cycle`, `cancel_hold`, `stale_hold` or `cancel_return`.
- The unchanged guarded binary `/tmp/dimos-sonic-expiry-20261006/gear_sonic_deploy/target/release/g1_deploy_onnx_ref`, SHA-256 `b9a95ae89b4ce328526715b0cee30689d06dbf5c23f23479dc64ddbfce19750e`.
- Existing assets under `/tmp/dimos-sonic-sim-assets-20261006` and a new output directory for each trial.

The runner rejects the host network namespace and any interface inventory other than loopback. Reference and feedback ports are 15556/15557 within the isolated namespace. Native `stop` remains a damping/shutdown operation; subsequent simulated falls are excluded from standing/settling claims.

The artifact auditor accepts completed run directories and writes JSON plus a `response.png` in each directory. It performs no controller communication:

```bash skip
MPLCONFIGDIR=/tmp/dimos-cycle-matplotlib .venv/bin/python \
  docs/artifacts/g1_sonic_hold_return_20261007/demo_audit_cycle.py \
  docs/artifacts/g1_sonic_hold_return_20261007/cycle01 \
  docs/artifacts/g1_sonic_hold_return_20261007/cycle02 \
  docs/artifacts/g1_sonic_hold_return_20261007/cancel_hold01 \
  docs/artifacts/g1_sonic_hold_return_20261007/stale_hold01 \
  docs/artifacts/g1_sonic_hold_return_20261007/cancel_return01 \
  docs/artifacts/g1_sonic_hold_return_20261007/cancel_return02 \
  --output docs/artifacts/g1_sonic_hold_return_20261007/audit.json
```

## Next service milestone

Establish a starting pose and reach/return path with **hand-to-thigh clearance**, then validate the full arm's motion and hold behavior in free space. Record all seven arm joints and end-effector pose, assess tracking and hand-pose stability, and retain measured-speed monitoring and bounded cancellation/recovery. Per the user's replay assessment, tuning away the cycle01 contact-release spike is not a prerequisite for this next simulation step. Then extend measured control to the joint group and verify Cartesian accuracy and collision behavior.

Dex3 grasp/release, loaded holding, D435i-to-robot calibration, the production command-session adapter and supervised hardware commissioning still follow that work in the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md). These results establish a small unloaded simulation milestone toward the object-to-tray service.
