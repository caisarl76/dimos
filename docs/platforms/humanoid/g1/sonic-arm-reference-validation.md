# DimOS + external SONIC: typed status and first arm reference

The next integration step is complete: DimOS can read the guarded receiver's state, and an isolated MuJoCo trial sent a small elbow reference through the real SONIC encoder and C++ controller. **Handover worked; the incremental tracking threshold was missed.** This reference construction is not ready to serve as a precise manipulation executor.

The tests ran on the workstation on 2026-10-06–07. Neither PC2 nor either physical G1 was contacted or modified in this step. SONIC remained the only simulated body/hand motor-command owner. The local prototype is not merged upstream or deployed on PC2.

## Where this fits in the service

The intended path is:

```text
DimOS task / motion reference
  → SONIC encoder → 64-value token + hand references
  → guarded C++ SONIC receiver and policy
  → body / hand motor commands
  → measured state + receiver status → DimOS verification
```

The earlier DimOS manipulation-panel exercises used a different control path with independently controlled arms. Their millimeter-scale model-FK results do not establish equivalent tracking through SONIC. Here, an encoded whole-body reference influences a learned policy; a successfully delivered token does not establish that the requested joint or hand pose was reached.

For pick-and-place, three observations need separate checks:

1. **Reference accepted:** the receiver accepted an increasing, valid reference.
2. **Controller handed back:** a successful policy tick used the local idle planner after interruption.
3. **Physical objective achieved:** measured pose, motion, grasp and placement satisfy their own criteria.

Only the first two controller observations and a sampled unloaded settling check are established by this experiment. There was no object, tray, grasp or Cartesian reach.

## Typed feedback completed

`SonicReferenceGuard` (`/dimos/robot/unitree/g1/sonic_feedback.py`) is now an optional immutable field of `SonicFeedback`. `SonicFeedbackModule` (`/dimos/robot/unitree/g1/sonic_feedback_module.py`) carries it on the existing typed stream and read RPCs. The [feedback guide](/docs/platforms/humanoid/g1/sonic-feedback.md#receiver-guard-status) explains each field and gives a Python example.

Legacy telemetry without the object continues to work and returns `None` for guard status. A malformed guard object rejects the packet. Stale, faulted and stopped receivers do not expose cached confirmation as current. Expiry and planner confirmation can both be true: the external reference expired and the planner has subsequently taken over.

The actual module passed the complete stationary fault trial, including duplicate/older references, malformed payloads, lost publisher, no first packet, explicit re-arm and manual handback: **20 runtime assertions**, 1,041 valid telemetry packets at final handback, zero invalid packets/rejected feedback indices. This run's 201 pre-shutdown physics samples had zero automatic resets, support disabled, height 0.784–0.788 m and maximum tilt 2.57°.

Evidence: [typed trial result](/docs/artifacts/g1_sonic_status_20261006/run01/result.json), [summary](/docs/artifacts/g1_sonic_status_20261006/run01/summary.json), [1,042-frame replay check](/docs/artifacts/g1_sonic_status_20261006/replay-summary.json).

## Moving-reference experiment

The experimental encoder helper (`/docs/artifacts/g1_sonic_status_20261006/demo_arm_encoder.py`) builds the inspected SONIC v1.1 mode-0 input: 1,751 floats, with ten future body position/velocity frames spaced 100 ms apart, heading-relative root orientation, and zeros for unused mode-specific fields. Absolute joint positions are converted from motor order to encoder order; the left elbow is motor slot 18 and encoder slot 21. The real ONNX encoder produces the 64-value token. This is a standalone experiment, not a production `ManipulatorAdapter`.

The trial captured the idle planner's body reference, held all other reference joints and both hand targets, and requested:

- A +0.1 rad left-elbow change using a two-second smooth ramp, followed by a one-second hold.
- A two-second return ramp and one-second hold.
- Another reach interrupted after approximately 0.8 seconds by closing the publisher socket.

The helper's layout, joint ordering, heading removal and ramp checks passed. Source review agreed with the C++ observation layout. An offline comparison with a planner token was **not an exact encoder-parity check**: the held-joint, zero-velocity reference differed from the planner's true future motion window. Matching exact inputs across Python and C++ remains part of the tracking diagnosis.

### Measured result

| Measurement | Result |
|---|---:|
| Captured nominal elbow reference | 1.017663 rad |
| Measured elbow after initial hold | 1.037475 rad |
| Requested reach endpoint | 1.117663 rad |
| Measured reach endpoint | 1.068889 rad |
| Absolute reach error | −0.048774 rad / −2.79° |
| Measured change from hold | +0.031413 rad / +1.80° |
| Error against the requested +0.1 rad increment | −0.068587 rad / −3.93° |
| Return error against nominal reference | +0.018683 rad |
| Return change from measured starting pose | −0.001129 rad |

The exploratory tracking criterion required absolute reach, incremental reach, absolute return and relative return errors each within 0.05 rad. **The incremental reach failed.** The absolute endpoint happened to satisfy this loose joint threshold; that is insufficient evidence of accurate manipulation. The cause has not been isolated between reference construction, encoder/runtime differences, learned tracking response and timing. Increasing the requested angle to compensate is not yet justified.

Recorded reference, measured elbow, arm speed and pelvis tilt (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/tracking.png`)

At interruption, typed feedback reported an armed, unexpired stream with accepted/applied reference index 379. The last observed elbow speed was 0.122 rad/s. The final reference was generated approximately 17.5 ms before socket closure.

| Interruption observation | Result |
|---|---:|
| First received expired sample after closure | 200 ms |
| Receiver-reported age in that sample | 217 ms |
| First received planner confirmation after closure | 300 ms |
| End of verified sampled settling window after closure | 2.423 s |
| Common settling-window duration | 0.568 s |
| Max body/arm speed across checked window samples | 0.1275 rad/s |

The settling check examined every recorded sample in its candidate window: 33 feedback samples and 12 physics samples, with at least 0.5 seconds of common time coverage. It required fresh data with bounded gaps, planner confirmation, body speeds below 0.2 rad/s, arm speeds below 0.15 rad/s, finite upright physics, support disabled and zero automatic resets. This is sampled evidence, not a continuous-motion guarantee or proof of fresh underlying DDS acquisitions.

Across the complete arm-stream and recovery interval before deliberate shutdown, 212 physics samples showed height **0.7847–0.7861 m**, maximum tilt **3.25°**, support disabled and **zero automatic resets**. The typed module accepted 1,074 telemetry packets at handback without invalid packets or rejected indices. The controller then exited with code 0 and probe cleanup reported no errors. Deliberate shutdown applies damping and is excluded from the upright interval.

All timings are observations from one completed trial with a 200 ms receiver deadline, not validated worst-case bounds. The first arm attempt has retained raw traces but is incomplete: the probe timed out during controller shutdown and failed to serialize a NumPy boolean. The corrected run moves diagnostic writes after cleanup, normalizes JSON values, checks stream activity immediately before closure and verifies every recorded settling sample. Its exact executed source is saved as probe-used.py (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/probe-used.py`). A subsequent review added the return-to-measured-baseline criterion to the reusable probe; the [offline summary](/docs/artifacts/g1_sonic_status_20261006/arm-run02/summary.json) evaluates all four criteria and the outcome remains failed tracking.

Evidence: [result](/docs/artifacts/g1_sonic_status_20261006/arm-run02/result.json), [summary](/docs/artifacts/g1_sonic_status_20261006/arm-run02/summary.json), [namespace and command manifest](/docs/artifacts/g1_sonic_status_20261006/arm-run02/manifest.json), reference inputs (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/references.jsonl`), telemetry (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/telemetry.jsonl`), physics (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/physics.jsonl`).

## Reproduce and inspect

The [receiver-expiry guide](/docs/platforms/humanoid/g1/sonic-reference-expiry.md#reproduce-on-this-workstation) records the existing isolated controller worktree, build prerequisites and copied model assets. Use that private-namespace command with `docs/artifacts/g1_sonic_status_20261006/demo_arm_sim.py` as the probe and a new output directory. Keep the loopback-only namespace and explicit host namespace checks. The helper imports `demo_arm_encoder.py` from the same directory and uses the existing workstation ONNX Runtime package.

To inspect the saved result offline:

```bash skip
cd /home/jihun/work/dimos_dev
.venv/bin/python docs/artifacts/g1_sonic_status_20261006/demo_arm_summary.py \
  docs/artifacts/g1_sonic_status_20261006/arm-run02
```

The summary script only reads saved trial data and writes `summary.json` and `tracking.png`. To study the integration code, read the feedback dataclasses first, then `get_feedback()` / `get_status()` in the module, then the experimental encoder and probe. Reference encoding and telemetry observation are separate from motion completion.

Automated validation: 68 feedback tests + 55 reference tests + 9 G1 TF tests + 5 registry checks = **137 passing tests**. Ruff and scoped mypy passed for the production changes. The full repository suite was not run. The unchanged native receiver patch retains the limitations recorded in its [expiry guide](/docs/platforms/humanoid/g1/sonic-reference-expiry.md).

## Next steps

**2026-10-07 follow-up:** [Encoder parity and scheduling diagnosis](/docs/platforms/humanoid/g1/sonic-encoder-parity.md) now verifies matching native/Python inputs and tokens, including direct replay of the synthetic arm inputs. The publisher's scheduling was corrected to approximately 50 Hz, but two repeat trials still missed the tracking criterion. A measured-feedback reach experiment is now the next milestone; the original trial remains a historical failed accuracy baseline.

1. **Diagnose reference tracking.** Compare identical encoder inputs/outputs across the Python and C++ paths, check applied-reference timing, and characterize repeated small unloaded targets. Establish measured endpoint and settling criteria before mapping this path to DimOS's manipulation execution API.
2. **Build the bounded motion adapter.** Add explicit lifecycle, fresh measured feedback, cancellation and observed completion only after the reference path meets those criteria. Preserve SONIC's sole motor-command ownership.
3. **Validate Dex3 and contact.** Establish anatomical joint mapping, limits, tool frames, contact behavior and recovery while holding an object.
4. **Connect calibrated perception and the task.** Calibrate the D435i/tool transforms and timestamps, then build the known-pose grasp/place sequence, object localization, and outcome verification. Commission the exact PC2 build separately under local supervision.

The immediate next milestone is a reproducibly accurate unloaded reach through SONIC. Navigation remains later in the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md); the current receiver pilot deliberately disables planner movement commands while its guard is enabled.
