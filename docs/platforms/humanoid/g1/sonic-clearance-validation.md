# SONIC arm clearance and hand-pose measurements

Two final workstation MuJoCo trials completed shoulder-roll preparation → elbow reach → three-second hold → elbow return → planner handback. Both hands remained **at least 20 mm from the modeled lower body throughout the recorded elbow cycle**. The experiments were recorded on 2026-10-07; final artifact verification was completed on 2026-10-08.

This follows the user's review of the earlier hand/leg contact-release spike. It establishes a clear starting configuration for the next manipulation experiment. The task controller still regulates one elbow; the full arm and hand pose are now measured, and a successful elbow return does not establish a successful hand-pose return.

## Results

The final trials used identical runner/helper hashes and the same guarded native SONIC binary. Starting poses varied between simulator launches.

| Measurement | cycle03 | cycle04 |
|---|---:|---:|
| Elbow cycle duration, excluding preparation/handback | 11.22 s | 11.66 s |
| Measured hold | 3.000 s | 3.000 s |
| Both hands' minimum recorded lower-body clearance | ≥20 mm | ≥20 mm |
| Maximum left hand-frame position drift during hold, relative to pelvis | 0.97 mm | 1.25 mm |
| Maximum left hand-frame orientation drift during hold, relative to pelvis | 0.30° | 0.39° |
| Left hand-frame return position error, relative to pelvis | 21.71 mm | 13.30 mm |
| Left hand-frame return orientation error, relative to pelvis | 8.55° | 4.52° |
| Final elbow position error | 0.01954 rad | 0.00619 rad |
| Peak measured speed across both arms during the elbow cycle | 0.231 rad/s | 0.287 rad/s |
| Observed planner handback after request | 0.122 s | 0.122 s |
| Verified sampled settling interval after handback | 0.620 s | 0.611 s |

“Hand frame” means the corresponding `wrist_yaw_link` body origin and orientation in the matching MuJoCo model, **not a calibrated grasp TCP**. Hold drift is measured from the first recorded hold pose. Return error compares the nearest recorded pose at cycle initialization with the final recorded cycle pose. World-frame measurements and all 14 arm-joint drift values are included in the [audit](/docs/artifacts/g1_sonic_clearance_20261007/audit.json).

The hand-pose metrics are observations, not completion criteria in this single-elbow controller. In particular, the 13–22 mm return errors show why the next step needs feedback over the whole arm.

## What changed

The experimental runner (`/docs/artifacts/g1_sonic_clearance_20261007/demo_hold_return_sim.py`) retains the previous encoder, elbow-cycle logic, native reference lease, posture checks, and handback behavior. It adds:

- A six-second cubic shoulder-roll reference change: left +0.2 rad, right −0.2 rad, with peak reference speed 0.05 rad/s. SONIC determines the actual motion.
- A readiness interval of at least 0.5 s with both hand clearances ≥20 mm, maximum arm speed ≤0.2 rad/s, and maximum body speed ≤0.3 rad/s. All recorded samples in the qualifying window are checked, including timing coverage and gaps.
- A recorded-clearance floor of 10 mm during the elbow cycle, checked before publishing and before feedback can complete the task. Violations end the attempt and request planner handback.
- Approximately 48 Hz geometry/pose recording, alongside approximately 50 Hz native telemetry. These are sampled observations, not continuous collision guarantees.

The geometry helper (`/docs/artifacts/g1_sonic_clearance_20261007/demo_clearance.py`) includes collision geoms from each wrist-roll subtree, including fingers, against the pelvis and both leg subtrees. It computes poses on separate MuJoCo data so measurement does not change dynamics. This checks hand-to-lower-body clearance; it is not a general self-collision or environment collision check.

Distances are queried with a **20 mm cap**. A capped result means “at least 20 mm,” and no closest pair is claimed when all candidates hit the cap. Below-cap gaps and penetration remain blocking measurements. This use follows MuJoCo's [distance-query implementation](https://github.com/google-deepmind/mujoco/blob/3.8.1/src/engine/engine_support.c), which returns the cutoff when a pair is farther away.

## Retained unsuccessful attempts and measurement fixes

| Trial | Recorded outcome | Interpretation |
|---|---|---|
| cycle01 | Preparation gate failed; elbow cycle not started | A 1 m mesh-distance query intermittently returned zero for separated pairs. The 20 mm proximity query removed those false blocking readings in all 290 saved readiness poses. |
| cycle02 | Elbow cycle succeeded; overall trial failed its settling deadline | Re-reading the enlarged 11 MB geometry log took approximately 157–177 ms per pass and delayed feedback receipt. The observer now reads only newly flushed records; acceptance limits were unchanged. |
| cycle03 | Complete trial passed | Clearance, elbow cycle, handback, sampled settling, and clean exit verified. |
| cycle04 | Unchanged repeat passed | Same checks passed with a different starting pose. |

The [actual-model proximity verification](/docs/artifacts/g1_sonic_clearance_20261007/proximity-verification-3.8.1.json) covers penetration, near-contact poses, positive gaps below the cap, and separated poses. No collision solver setting or robot model was changed to obtain the final trials. Both earlier failed trials remain in the aggregate audit.

## Evidence and replay

- MuJoCo video: preparation, reach, hold, return (`/docs/artifacts/g1_sonic_clearance_20261007/mujoco_replay/clearance_reach_hold_return.mp4`), normal speed, cycle04, approximately 18.7 s including preparation.
- Video page (`/docs/artifacts/g1_sonic_clearance_20261007/mujoco_replay/index.html`) and preview (`/docs/artifacts/g1_sonic_clearance_20261007/mujoco_replay/preview.png`).
- [All-trial audit](/docs/artifacts/g1_sonic_clearance_20261007/audit.json) and [verification record](/docs/artifacts/g1_sonic_clearance_20261007/verification.json).
- Focused checks (`/docs/artifacts/g1_sonic_clearance_20261007/pytest-green.log`): 16 passed. Ruff check/format and both final trial audits passed. No full-repository test result is claimed for these experimental artifacts.

The live simulation used MuJoCo 3.8.1; the video uses MuJoCo 3.10.0 only to render recorded positions, without rerunning dynamics. It displays nearest recorded poses and separately sampled velocity telemetry.

All native trials ran inside a private network namespace with only loopback. No PC2 connection or real robot command occurred. The C++ SONIC controller remained the sole body/hand command owner. Both final controllers exited with code 0 and no cleanup errors; the final process check found no remaining experiment processes. No production DimOS modules or robot deployments changed in this step.

## Next step toward the service

Extend the measured feedback goal from the elbow to all seven joints of the selected arm, beginning in this clear configuration. Preserve the clearance and handback checks, and measure both joint return error and hand-frame translation/orientation error. Then define and validate a Cartesian target using a calibrated hand/tool frame before adding object grasp/release. The [service plan](/docs/platforms/humanoid/g1/service-development-plan.md) retains Dex3, perception calibration, and hardware commissioning as later steps.
