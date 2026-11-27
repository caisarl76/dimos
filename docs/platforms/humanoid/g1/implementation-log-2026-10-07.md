# G1 SONIC integration checkpoint on 2026-10-07

DimOS manages the service, while the existing external C++ SONIC controller remains the sole owner of body and hand motor commands. The integration now supports typed feedback and a simulation-tested reference-expiry prototype. A small unloaded elbow-reference trial failed its incremental tracking criterion, so precise manipulation execution remains unverified. Dex3 grasping and the object-to-tray service are still to implement and validate.

This checkpoint covers hardware observations on 2026-10-06 and local integration trials on 2026-10-06–07. The latest trials ran on the workstation without contacting or modifying PC2 or either physical G1. The guarded controller prototype is not merged upstream or deployed on PC2. The [arm-reference walkthrough](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) supplies the latest results; the [log index](/docs/platforms/humanoid/g1/implementation-log.md) gives the status overview.

## Hardware observations on 2026-10-06

### Hardware inventory and camera acquisition

**Hardware-observed:** the `pc2_222` inventory checked 29 body-state values, seven state slots per Dex3 hand, the robot-facing interface and D435i acquisition. Two native DimOS camera-only checks received RGB, aligned depth, calibration messages and camera TF at 640×480 and 15 fps, then stopped cleanly.

**Still unverified:** camera-to-robot extrinsics, calibrated tool frames and camera/body timestamp alignment. Delivered intrinsics and camera TF do not establish mounting calibration. A bracketed comparison found PC2 approximately 550 seconds behind the workstation; neither clock was adjusted. Firmware and motion readiness also remain open.

Evidence: [inventory](/docs/platforms/humanoid/g1/pc2-222-inventory-2026-10-06.md) and [native camera setup and results](/docs/platforms/humanoid/g1/pc2-222-camera-setup.md).

### Passive SONIC feedback

**Implemented and hardware-observed:** the local DimOS feedback module received real SONIC telemetry. A five-second read-only check accepted 249 valid packets and ended with fresh feedback. An independent eight-second capture received 400 SONIC packets and compared decoded positions against separately subscribed body and hand DDS state.

| Group | Largest observed position difference |
|---|---:|
| Body | 0.00384° |
| Left hand | 0.00169° |
| Right hand | 0.00369° |

These results establish wire decoding and close position agreement in the observed poses. They do not establish anatomical finger mapping, velocity agreement, simultaneous acquisition, underlying DDS sample freshness or manipulation accuracy. The workstation subscriber observed the existing controller; it did not install the latest integration on PC2. The inspected live session selected keyboard input, so a future hardware command integration must verify the actual input mode.

Evidence: [active hardware feedback session](/docs/platforms/humanoid/g1/sonic-feedback.md#active-planner-session-2026-10-06).

## Local implementation and simulation on 2026-10-06–07

### Typed receiver status

**Implemented:** optional immutable receiver-guard status is carried in DimOS feedback streams and read RPCs. Legacy telemetry without guard status remains supported. Malformed guard status rejects the packet, and stale, faulted or stopped feedback does not expose cached confirmation as current.

**Simulation-tested:** the actual DimOS module passed the stationary fault trial with 20 runtime assertions covering duplicate/older references, malformed payloads, lost publisher, no first packet, explicit re-arm and manual handback. The trial recorded zero automatic resets during its pre-shutdown physics interval. A separate replay decoded 1,042 recorded frames.

Evidence: [typed-status walkthrough](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md#typed-feedback-completed) and [trial result](/docs/artifacts/g1_sonic_status_20261006/run01/result.json).

### Reference expiry and planner handover

**Implemented as a local prototype and simulation-tested:** the guarded C++ receiver accepts valid increasing reference indices and expires an inactive stream. The tested receiver deadline was 200 ms. Expiry requests local idle-planner handover, with explicit re-arm sequencing and typed confirmation. SONIC retains body and hand motor-command ownership.

Acceptance, planner confirmation and physical completion are distinct observations. An installed reference index does not prove consumption by a particular policy tick; planner confirmation does not prove the requested pose was reached. The receiver prototype remains local, and loaded handover and recovery remain unverified.

Evidence: [receiver-expiry pilot and limitations](/docs/platforms/humanoid/g1/sonic-reference-expiry.md).

### Small unloaded arm reference

**Simulation-tested with failed tracking:** the experiment used the real SONIC ONNX encoder and C++ controller in isolated MuJoCo. Its helper constructed the mode-0 input of 1,751 floats and generated a 64-value token. All other body reference joints and both hand targets were held while the left elbow received a +0.100 rad request over a two-second ramp and one-second hold, followed by a return and an interrupted reach.

| Measurement | Result |
|---|---:|
| Captured nominal elbow reference | 1.017663 rad |
| Measured elbow after initial hold | 1.037475 rad |
| Requested reach endpoint | 1.117663 rad |
| Measured reach endpoint | 1.068889 rad |
| Absolute reach error | −0.048774 rad |
| Requested increment | +0.100000 rad |
| Measured increment from the initial hold | +0.031413 rad |
| Incremental reach error | −0.068587 rad |
| Absolute return error | +0.018683 rad |
| Return change from measured starting pose | −0.001129 rad |

The exploratory criterion required absolute reach, incremental reach, absolute return and relative return errors each within 0.05 rad. The incremental reach failed. The absolute endpoint fell within that loose threshold, but the measured movement was only about 31% of the requested change. This path cannot yet be treated as a precise manipulation executor.

**Still unverified:** the cause of the tracking shortfall. Reference construction, Python/C++ encoder or runtime differences, learned tracking response and timing remain diagnosis candidates. Source-layout agreement and a comparison against a different planner motion window do not establish exact encoder parity. Increasing the requested angle to compensate is not justified by this result.

Evidence: [measured-result walkthrough](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md#measured-result), [trial result](/docs/artifacts/g1_sonic_status_20261006/arm-run02/result.json), [offline summary](/docs/artifacts/g1_sonic_status_20261006/arm-run02/summary.json) and reference inputs (`/docs/artifacts/g1_sonic_status_20261006/arm-run02/references.jsonl`).

### Interruption and sampled settling

**Simulation-tested:** closing the publisher during the later reach produced an expired feedback sample after approximately 200 ms and planner confirmation after approximately 300 ms. The verified settling window ended 2.423 seconds after closure and covered 0.568 seconds of common observation time, with 33 feedback and 12 physics samples. Maximum checked body/arm speed was 0.1275 rad/s.

Across the arm-stream and recovery interval before deliberate shutdown, 212 physics samples showed pelvis height 0.7847–0.7861 m, maximum tilt 3.25°, support disabled and zero automatic resets. The module accepted 1,074 telemetry packets at handback without invalid packets or rejected feedback indices; the controller exited with code 0.

These are sampled observations from one completed unloaded trial, not worst-case stopping bounds or a continuous-motion guarantee. Deliberate shutdown applies damping and is excluded from the upright interval. There was no object, tray, grasp or Cartesian reach. The earlier incomplete arm attempt is not counted as a completed validation trial.

## Recorded software verification

The latest walkthrough records 137 passing checks: 68 feedback tests, 55 reference tests, nine G1 TF tests and five registry checks. Ruff and scoped mypy passed for the production changes. The full repository suite was not run.

The unchanged native receiver patch retains its separately recorded limits: 18 available native tests passed, while the full 19-test invocation failed in an existing FK test with a missing fixture. That is not a full native-suite pass. These are prior implementation results; no simulation or robot validation was rerun for this documentation checkpoint.

## Next milestone and remaining work

The next milestone is **identical Python/C++ encoder inputs followed by a repeatably accurate unloaded reach with measured completion and cancellation**.

1. Capture the same encoder observation vector in both paths, verify ordering, heading removal and future-frame timing, and compare encoder outputs. Relate sent references to receiver acceptance, installation and policy timing.
2. Repeat small unloaded reaches and returns through SONIC. Define endpoint, incremental movement, velocity and settling criteria before validating execution semantics. Preserve measured evidence for each attempt; the current 0.05 rad check is exploratory rather than a service accuracy requirement.
3. Establish bounded motion lifecycle behavior with fresh feedback, measured completion, cancellation and observed recovery. Publisher closure and planner confirmation alone do not constitute a completed task-cancellation API.

Dex3 anatomical mapping, limits, contact behavior, grasp verification and recovery while holding an object follow the unloaded-reach milestone. Calibrated perception and tool transforms, time alignment, the deterministic grasp/place sequence and measured placement verification follow under the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md). The exact PC2 integration build still requires separate hardware commissioning. General navigation remains later work.

Implementation is held at this checkpoint. Dex3 grasping and the object-to-tray service have not been implemented or validated, and the latest integration has not been deployed on hardware.
