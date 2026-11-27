# Native validation of the SONIC service coordinator

On 2026-10-13, the actual `SonicServiceModule` and `SonicExecutionModule`
passed live execution, cancellation and frozen-camera recovery checks with
Pi0.5 inference, native GEAR-SONIC and MuJoCo. This validates the service
interface introduced in the [coordinator guide](/docs/platforms/humanoid/g1/sonic-service-coordinator.md).
No real robot connection or command was made.

The authoritative run is
[`run01`](../../../artifacts/g1_sonic_service_native_20261013/run01/result.json).
Its raw result deliberately remains `native_service_cases_recorded_pending_audit`;
the separate [audit](/docs/artifacts/g1_sonic_service_native_20261013/run01/audit.json)
records `audit_passed: true`. This was the first native trial for this milestone.

## What ran

Two DimOS workers hosted the service and execution modules. The harness
published typed `SonicObservation` snapshots, then used the service's request,
status and cancel RPCs. The coordinator performed task inference and submitted
the resulting chunks to the execution module. Native SONIC controlled the
simulated body and both Dex3 hands through streaming v4.

Each observation paired a rendered head-camera RGB image with the 28 measured
arm/hand joints from the same physics snapshot. The original capture timestamp
survived inference and admission. An owned recording proxy forwarded the
existing LeRobot protocol bytes unchanged and recorded request/response evidence.

The run used the local `wbt_pi05_he_full/pretrained_model_ema` checkpoint,
noise scale zero, source rate 50 Hz and 350 ms inference-start cadence.
The instruction was `pick drink bottle from the table and handover`, matching
the selected policy's dataset. The scene contained no bottle, table or tray.
Camera geometry was nominal and uncalibrated.

All controller and DDS traffic stayed on loopback in a private user/network
namespace. An owned offline policy server used GPU 1; the existing GPU 0
server remained running. The
[manifest](/docs/artifacts/g1_sonic_service_native_20261013/run01/manifest.json)
records namespaces, source hashes, native binary/assets and configuration.
[Policy provenance](/docs/artifacts/g1_sonic_service_native_20261013/run01/policy-provenance.json)
records checkpoint, normalization, tokenizer and server hashes measured before
the run, with unchanged file sizes and modification times checked afterward.

## Results

| Case | Terminal outcome | Accepted / discarded predictions | Streaming duration | Handback request → native confirmation |
| --- | --- | ---: | ---: | ---: |
| Normal six-second request | `finished` | 17 / 1 | 6.001 s | 121 ms |
| Cancel during inference | `cancelled` | 5 / 1 | 1.517 s | 219 ms |
| Freeze camera | `failed`, observation stale | 6 / 0 | 2.194 s | 127 ms |

All three cases confirmed planner handback. Cancellation injection to native
confirmation took **231 ms**; the shorter 219 ms value starts when execution
requested handback. The late cancellation response arrived 56 ms after native
completion and was discarded.

Camera freeze to handback request took 678 ms, and freeze to confirmed planner
took 805 ms. The last accepted observation was already aging when the camera
froze: its age at handback was **853 ms**, consistent with the configured
850 ms expiry plus supervisor scheduling. Expiry uses source age rather than
elapsed time since the fault was injected.

The independent raw-evidence audit checked:

- All **338** camera snapshots against their physics joint states.
- All **30** task inference requests against original RGB/JPEG hashes and
  joint states, including the two discarded results.
- All **28** accepted prediction hashes and original observation times against
  both the service and execution journals.
- **647** contiguous pose indices across the three cases, native applied-index
  progress, and zero pose-reference packets after each handback request.
- Progress frozen during recovery, expected terminal outcomes, and
  `task_success=None` throughout.
- Upright physics without support or resets, and complete owned-process cleanup.

Input source age at inference request was 42–123 ms; admission age was
352–444 ms. Inference round trips were 299–325 ms. These are measurements from
this local run, not deployment latency guarantees.

## Replay and physical motion

Open the MuJoCo replay (`../../../artifacts/g1_sonic_service_native_20261013/run01/replay/live_policy_replay.mp4`)
and arm/hand speed plot (`../../../artifacts/g1_sonic_service_native_20261013/run01/replay/joint_speeds.png`).
The left pane replays recorded robot poses; the right pane holds the most
recent image actually submitted to the policy until the next request.

The video is 10.5 seconds, 1280 × 680, H.264 at 20 fps. Rendering used recorded
`qpos` and `mj_forward` only, with no control replay. Native physics used
MuJoCo 3.8.1; the offline renderer used 3.10.0. Speed plots use recorded joint
velocities rather than derivatives of the sampled video poses.

| Case | Peak arm-joint speed | Minimum pelvis height | Maximum pelvis tilt |
| --- | ---: | ---: | ---: |
| Normal | 3.52 rad/s | 0.780 m | 8.23° |
| Cancellation | 5.98 rad/s | 0.784 m | 6.96° |
| Camera freeze | 2.93 rad/s | 0.784 m | 8.10° |

The normal case's left-arm peak was 2.84 rad/s and right-arm peak 3.52 rad/s.
The video includes planner recovery; handback confirmation does not mean every
joint has physically settled.

Collision-geometry distance between a hand and the lower body reached
**−3.59 mm** in the normal case, **−2.62 mm** during the cancellation case
and **−1.67 mm** during camera loss. The normal-case minimum was between the
right index-finger base and right hip-roll link, during post-handback recovery.
These signed distances came from `mj_geomDistance` on the model's collision
geometries; negative values indicate overlap. The
[supplemental metrics](/docs/artifacts/g1_sonic_service_native_20261013/run01/supplemental-metrics.json)
record pairs and times. This run does not establish collision-free motion or
hardware readiness, and no cause is assigned to the speed peaks from these
measurements alone.

## Cleanup and verification

Service/execution stop RPCs returned, the coordinator workers and recording
proxy stopped, and the reference endpoint was successfully rebound for the
cleanup publisher. Planner mode was confirmed before native shutdown.
The native controller, simulator and camera processes all exited with code
zero; owned policy PID 683031 was reaped. The host process/GPU checks showed
the pre-existing policy and DimOS simulator still running, with the owned
policy's GPU allocation released.

An independent reviewer checked the harness and raw evidence. Review fixes
made policy cleanup bounded, kept preliminary results distinct from the audit,
derived handback time from the execution journal and rejected malformed JSONL
instead of silently skipping it. Four Python artifact scripts passed Ruff
lint/format checks; shell syntax and strict JSONL parsing were also checked.

Production coordinator/execution source and coordinator tests still match the
hashes in the prior [262-test verification record](/docs/artifacts/g1_sonic_service_20261012/verification.json).
Those tests and the full repository suite were not rerun for this milestone.
The new [verification record](/docs/artifacts/g1_sonic_service_native_20261013/verification.json)
identifies these checks and hashes the resulting artifacts.

## Reproduce with the existing local assets

The launcher uses workstation-specific paths recorded in its source. It needs
the existing native binary/assets, checkpoint, dataset, GPU 1, and permission
to create a private user/network namespace. Choose a fresh output directory;
preserve `run01` as evidence. These commands start simulation and inference.

```bash skip
bash docs/artifacts/g1_sonic_service_native_20261013/demo_run_trial.sh \
  docs/artifacts/g1_sonic_service_native_20261013/run02

.venv/bin/python docs/artifacts/g1_sonic_service_native_20261013/demo_audit.py \
  docs/artifacts/g1_sonic_service_native_20261013/run02

MUJOCO_GL=egl CUDA_VISIBLE_DEVICES=1 MUJOCO_EGL_DEVICE_ID=0 \
  .venv/bin/python docs/artifacts/g1_sonic_service_native_20261013/demo_render.py \
  --run docs/artifacts/g1_sonic_service_native_20261013/run02 \
  --scene /home/jihun/work/GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml
```

## Next integration step

Package a reusable observation adapter for pc2_222's D435i and measured
body/Dex3 state. Begin with read-only capture: verify joint ordering, RGB
layout, timestamps, image/state alignment and camera calibration against the
policy's training inputs. Cross-host clock mapping and the hardware hand
mapping must be explicit before commissioning a service profile.

DimOS continues to own service requests, observation routing, supervision and
outcome reporting. LeRobot supplies learned actions and SONIC retains motor
control. Hardware commissioning, collision-clearance validation and an
object-outcome verifier remain necessary before claiming bottle-to-tray success.
