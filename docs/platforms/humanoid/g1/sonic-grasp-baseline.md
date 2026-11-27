# Bottle grasp development: Dex3 fixture and native SONIC

Work on 2026-10-09–12 continues the [bottle-to-tray baseline](/docs/platforms/humanoid/g1/sonic-pick-place-baseline.md).
The objective is a measured grasp and lift before adding transfer to the tray.
All motion experiments here use local simulation. On October 12 a separate,
read-only SSH check of `pc2` (`192.168.0.223`) found no matching SONIC/DimOS
process or TCP listener on ports 5555–5557. No hardware mode or actuator command
was sent. This snapshot does not describe `pc2_222`.

## What this stage establishes

The standing G1 performed a measured grasp, lift and hold through DimOS and
native SONIC in `native08`. The bottle bottom reached **0.79094 m**, about
**91 mm above the table**. The independent score recorded a supported finger
lift, then a drop during lowering. This establishes one simulated lift;
repeatable grasping and successful placement remain unproven.

The requested **40 mm lower grip** was tested on October 12. It lifts and returns
upright in the isolated hand fixture, but full-robot attempts `native11`–`native14`
have not established a lower grasp. The latest trial tips the bottle while
moving into pregrasp and terminates before closure. The next problem is the
collision-sensitive approach path under whole-body tracking.

- Hand fixture replay (`../../../artifacts/g1_grasp_baseline_20261009/fixture05/replay.mp4`)
- Hand fixture contact sheet (`../../../artifacts/g1_grasp_baseline_20261009/fixture05/contact_sheet.png`)
- Higher hand fixture replay (`../../../artifacts/g1_grasp_baseline_20261009/fixture06/replay.mp4`)
- Full G1 grasp/lift and lowering failure replay (`../../../artifacts/g1_grasp_baseline_20261009/native08/replay.mp4`)
- Latest lower-grasp attempt and abort (`../../../artifacts/g1_grasp_baseline_20261009/native14/replay.mp4`)
- Recorded training example (`../../../artifacts/g1_grasp_baseline_20261009/training_episode42.png`)
- [Machine-readable measurements](/docs/artifacts/g1_grasp_baseline_20261009/verification.json)

## Training data and initial state

The inspected dataset is
`/mnt/data/jihun/datasets/G1_WBT_GR00T/handover_bottle_sonic78_50hz_261002`:
43 episodes, 50 Hz, 28 measured arm/hand coordinates and 78 action coordinates.
Its task is `pick drink bottle from the table and handover`. The new service
requires placement in a tray; that behavior has not been trained by changing
the inference prompt.

Across the 43 first frames, the mean right-hand state is approximately
`[0.018, -0.011, -0.049, 0.032, 0.045, 0.035, 0.045]` rad: nearly open.
The earlier simulation policy run began with fingers near their closed limits.
Arm postures also differ. Episode 42 shows a clear bottle with a green cap on a
black case and a human recipient. The simulated green cylinder, brown table,
blue tray and object placement create another visible difference. These are
observed mismatches, not a causal attribution of the earlier policy failure.

The [comparison JSON](/docs/artifacts/g1_grasp_baseline_20261009/training_comparison.json)
preserves exact states, source paths and group boundaries.

## Contact fixture

The fixture copies the existing right Dex3 model, including joint limits,
inertias and contact geometry. Finger control uses PD gains 1.5/0.1 and bounded
motor torque: 2.45 Nm for thumb joint 0, 1.4 Nm for the other six joints.
These are simulation model/helper values, not calibrated physical grasp forces.
The free wrist follows a prescribed mocap target through a weld constraint.
The bottle is a free body and is never welded to the hand.

The provisional object is the existing 50 mm diameter, 160 mm tall, 80 g
cylinder. The table top is 0.70 m. Finger closure targets, in canonical
right-hand order (thumb 0/1/2, index 0/1, middle 0/1), are
`[0.04, -0.20, -1.30, 1.30, 1.50, 1.30, 1.50]` rad.

Nine wrist-relative XY grasp offsets passed the fixture lift criterion.
The higher central grasp uses a wrist at `[0.30, -0.19, 0.82]` m with identity
world orientation. Its bottle bottom reached **0.7982 m**, approximately
98 mm above the table. Thumb plus index/middle contact supported a lift of at
least 40 mm for **3.18 seconds**, with no table/tray support. Maximum bottle
XY displacement was **10.0 mm**. The fixture subsequently lowered and opened
the hand; the fixture's pass flag specifically scores lift.

`fixture06` raises the wrist to 0.86 m. It also passes lift: maximum bottle
bottom 0.7971 m and 3.14 seconds of qualified contact. However, XY displacement
reaches 25.3 mm during hold and **61.8 mm during release**, ending with the
bottle on its side. This is a successful fixture lift and an unsuccessful
upright replacement. The [phase audit](/docs/artifacts/g1_grasp_baseline_20261009/fixture06/phase_audit.json)
separates those observations. Full-robot release/withdrawal and final success
now require measured bottle tilt no greater than 15°, including rejection of
missing or nonfinite tilt measurements.

Early fixture versions exposed two modeling errors: a static mocap wrist
changed adjacent-body collision behavior, and a directly prescribed static
wrist did not provide physical wrist velocity for contact friction. The final
floating-wrist constraint corrected both. Those preliminary fixture runs do
not measure policy or whole-robot grasp ability. Physics uses MuJoCo 3.8.1;
saved-pose video rendering uses 3.10.0.

## Full G1 control route

The experimental reference generator uses the known simulation bottle pose:

```text
World wrist target + measured floating-base pose
  → orientation-aware IK
  → bounded joint-reference feedback (0.12 rad/s reference slew)
  → SONIC encoder: 64 tokens + 14 finger targets
  → DimOS SonicTaskSession: guarded v4 streaming
  → native SONIC: body and hand DDS
  → MuJoCo G1
```

Only native SONIC publishes robot actuator commands. The probe uses simulation
ground truth, not camera perception or Pi0.5 inference. The left hand retains
its captured target while the right hand performs the task. This artifact is
an experiment, not a registered DimOS service or a collision-planned hardware
controller.

Stages are raised preparation, right-hand opening, pregrasp, approach, closure,
lift, hold, lower, release and withdrawal. Transitions require measured pose,
finger state or object contact. The floating-base pose is refreshed for IK
every 0.5 seconds. Reference computation runs in a worker while the session
continues its 50 Hz lifecycle loop. Freshness limits remain 200 ms for native
feedback/physics, 100 ms for session ticks and 400 ms for reference observations.
Source timestamps are preserved; late work cannot publish after handback.

The full sequence can succeed only after the bottle is supported by the table,
upright, released, settled and at least 50 mm from either hand, followed by confirmed
planner handback and a two-second observation of the final object condition.
This sequence returns the bottle to the table; it does not yet transfer it to
the tray. Independent `PickPlaceScore` results remain separate.

## Diagnosed trials

| Trial | Measured result | Correction |
|---|---|---|
| `native01` | Opening timed out; fingers contacted table/tray despite open wire targets | Raise and orient right wrist before opening |
| `native02` | Joint target reached, but world wrist error was 41 mm | Recompute IK using the moving base pose |
| `native03` | Raised preparation and opening passed; transition IK delayed the stream tick | Move IK/encoding to a worker |
| `native04` | Preparation/opening passed; reference observation expired during pregrasp computation | Kinematics-only IK, previous-solution warm start, analytic Jacobian |
| `native05` | Pregrasp passed; 0.82 m-high forward approach failed IK with 22.8 mm position residual | Test a 0.86 m grasp in the hand fixture before whole-body execution |
| `native06` | Preparation timed out at 89 mm and 62° error; initial stance differed and IK selected a reversed-elbow posture | Anchor preparation/withdrawal XY to the initial pelvis position |
| `native07` | Preparation/opening/pregrasp passed; approach IK exceeded 10 mm tolerance as the base moved | Test a bottle 60 mm closer in a separate scene |
| `native08` | Grasp/lift/hold passed; bottle reached the table during lowering, then continued hand descent tipped it off the edge | Hold measured arm joints at first table contact; require upright settled support before release |
| `native09` | Starting pelvis X was −75.5 mm; approach failed IK before testing the contact correction | Reject starts outside a ±30 mm XY window around the fixed-table robot origin |
| `native10` | Initial pelvis was `[37.4, −57.3]` mm in XY; starting-position check rejected the run | Zero arm-reference packets sent; planner cleanup confirmed |
| `native11` | Wrist lowered to 0.82 m; approach failed IK at 12.2 mm residual as the base moved | Move bottle and wrist targets another 30 mm closer, retaining the lower height and relative grip offset |
| `native12` | Lower pose reached, but open middle finger tipped the bottle during approach; closure timed out | Use measured 10 mm approach waypoints and terminate if the bottle tips |
| `native13` | Bottle tipped during the first waypoint; the new check stopped the trial before closure | Test wider side clearance in the hand fixture |
| `native14` | Wider clearance still allowed index/middle-finger contact during pregrasp; terminated at 15.18° bottle tilt | Collision-check the entire lowered approach and bound its measured tracking error |

`native01`–`native07` recovered to the native planner and reaped their owned
simulator/controller children with exit code 0. None of those seven lifted the bottle.
Earlier source revisions are preserved beside their manifests, so their hash
checks remain reproducible after fixes. The timeout trials remain failures.

`native08` preserved unsupported standing, zero automatic resets and one initial
bottle placement. Maximum pelvis tilt was 8.57°, maximum measured arm joint
speed 1.83 rad/s, and maximum reference packet gap 29.3 ms. Cleanup confirmed
the planner and both owned processes exited with code 0. These control results
do not override the independent task score: **dropped**.

The [lowering contact audit](/docs/artifacts/g1_grasp_baseline_20261009/native08/lowering_contact_audit.json)
shows upright table support approximately two seconds into descent. The old
transition also required the nominal wrist target; the hand kept moving and
pressed the bottle with its wrist geometry. The bottle tipped around seven
seconds into lowering and reached the floor around eight seconds. The new
controller captures measured arm joints at first table support and disables
further descent IK, then waits for upright, settled table support before
opening. Unsupported loss of opposing finger contact for 200 ms, or a bottle
bottom below 0.65 m, now ends the trial early. This is a simulation correction,
not calibrated physical force control.

The original guarded executable under `/tmp/dimos-sonic-expiry-20261006` was
absent when work resumed on October 12. `native06`–`native11` used the existing
diagnostic executable from the [encoder-parity milestone](/docs/platforms/humanoid/g1/sonic-encoder-parity.md),
SHA-256 `cc3639ac9d39fe42c6ed73c5068596dc7a485b78a3f832c04eec9271f2529d30`.
Its saved patch adds optional logging to the guarded controller; that option
is disabled. The runner checks its recorded hash before launching and records
the original SDK libraries used to resolve its now-missing temporary RPATH.
Earlier binary hashes remain historical evidence; current audits mark the
missing executable as unavailable (`null`), not as a successful hash check.

Preparation now uses initial pelvis XY plus `[0.20, -0.30]` m and world Z
0.90 m, then holds that world target while the measured base moves. This keeps
initial translational stance differences from changing the intended reach.
It does not compensate arbitrary yaw or validate a collision-free path.
The [workspace comparison](/docs/artifacts/g1_grasp_baseline_20261009/ik_workspace_comparison.json)
records four failing seeds at the original approach point and successful
solutions at closer points, using the failed run's measured floating base.

`native08` uses its archived scene (`../../../artifacts/g1_grasp_baseline_20261009/native08/scene.xml`):
bottle source X is 0.34 m instead of 0.40 m. Pregrasp/approach/lift/lower X
targets also move 60 mm closer, preserving the fixture's relative grasp offset.
Robot joint/actuator layout, table, tray, bottle dimensions and contact
properties remain identical. The bottle is placed once before task start;
no reset or object-position write is allowed during the task.

The contact correction passed **46 focused tests** (23 grasp-reference tests
and 23 task-metric tests). `native09` stopped at IK and therefore did not validate
that correction through a loaded full-robot descent. Its cleanup still confirmed
planner handback and reaped both owned processes successfully. A fixed-table
starting-position check now runs before arm streaming; this is an explicit
experiment precondition, not autonomous positioning or a guarantee of reachability.
`native10` exercised that rejection with zero pose packets, confirmed planner
cleanup, and both children exiting with code 0. Its measured initial XY offset
was `[0.0374, -0.0573]` m. Neither `native09` nor `native10` is a successful test
of loaded contact-triggered release. All available source/binary hashes for
`native06`–`native10` match their manifests.

## Lower grasp follow-up, October 12

The requested lower grip changes wrist height from **0.86 to 0.82 m** and the
lift target from 0.96 to 0.92 m. The isolated hand fixture repeat (`fixture07`)
lifted the free bottle for 3.18 seconds, with maximum bottom height 0.79824 m,
maximum XY drift 9.96 mm, and final bottle tilt 0.60° after lowering/opening.
This confirms the prescribed-wrist contact geometry; the standing robot still
needs to demonstrate the full sequence.

- Lower grasp fixture replay (`../../../artifacts/g1_grasp_baseline_20261009/fixture07/replay.mp4`)
- [Fixture measurements](/docs/artifacts/g1_grasp_baseline_20261009/fixture07/audit.json)
- [Recorded reach-margin check](/docs/artifacts/g1_grasp_baseline_20261009/lower_grasp_reach_margin.json)

The simulation worker now accepts an opt-in, one-time translation of initial
pelvis XY to the table origin before object placement and task-ready. This
changes no joint, orientation, height or velocity, and is never repeated during
streaming. It is a simulation setup operation, not robot navigation. The runner
still checks the measured ±30 mm starting window after settling. Alignment count
and translation are recorded and checked throughout the task.

`native11` performed this setup once and reached approach, but did not grasp.
At its recorded failure pose, the 0.24 m wrist X target failed IK while 0.21 m
solved with negligible residual. The next scene therefore uses bottle X 0.31 m,
wrist X 0.21 m, and pregrasp X 0.15 m. Bottle dimensions, table, contact model,
grip offset and error tolerances are unchanged. Planner cleanup was confirmed
after `native11`, with both owned children exiting 0.

The temporary diagnostic executable and model cache disappeared before the
next launch. The runtime was restored in ignored `build/sonic-grasp-runtime/`
from the recorded base commit plus the saved expiry and diagnostic patches.
All nine patched source hashes and all seven model/cache hashes match earlier
provenance. The rebuilt binary has its own hash; it is not claimed to be
byte-identical to the deleted binary. Eighteen available C++ tests passed,
excluding the previously documented absent FK fixture, and 46 focused Python
tests passed. [Rebuild provenance](/docs/artifacts/g1_grasp_baseline_20261009/runtime-rebuild.json)
records the runtime path and hashes. The runner verifies binary and asset hashes
before launching. The restored cache/build occupies about 2 GB.

`native12` reached the lower grasp target, but open-middle-finger contact started
tipping the bottle during approach, before closure. The measured wrist wandered
about 19 mm sideways. The final object remained on the table, on its side;
no lift occurred. Full G1 replay (`../../../artifacts/g1_grasp_baseline_20261009/native12/replay.mp4`)
shows this failure. The session returned to the planner and both children exited 0.

Approach now uses six 10 mm waypoints, 0.06 rad/s reference slew, and measured
4 mm/5° settling for 0.5 seconds before each step. This narrower gate is specific
to clearance near the bottle; ordinary movement still uses the previous broader
gate. Pregrasp uses the same narrow gate. Bottle tilt above 15° (or an unavailable
tilt measurement) during pregrasp, approach or closure terminates the attempt
before another reference is submitted. `native13` exercised this abort at 15.48°
on the first waypoint, confirming planner handback and cleanup without closing.
Thus waypoints alone did not fix the finger collision.

Review found and corrected two interactions in the new path: the original
0.01 rad joint deadband could stall outside the 4 mm Cartesian gate, and the
stage could advance on a settled sample after its deadline. Pregrasp/approach
now use a 0.001 rad deadband; deadline failure takes priority over progression.
Both regression tests failed before the corrections. All **52 focused tests**
pass afterward (29 grasp-reference, 23 task-metric).

The [side-clearance fixture sweep](/docs/artifacts/g1_grasp_baseline_20261009/fixture08/sweep.json)
tested relative Y offsets 65, 75 and 85 mm at the lower height. All lifted and
ended upright. `native14` selects 75 mm, giving 25 mm more side
clearance than the central grasp. Its fixture reached bottom height 0.79993 m,
ended at 0.95° tilt, and displaced the bottle up to 34.35 mm in XY. Accordingly,
the current pregrasp/approach/lift/lower target Y is −0.215 m. This is a revised
contact geometry, not a claim that the whole-body tracking problem is solved.

`native14` still contacted the bottle while moving from the raised preparation
pose into pregrasp. Index-finger contact was recorded around 26.42 seconds after
session start, followed by middle-finger contact and bottle tilt above 15° at
27.19 seconds. The reference generator terminated at measured tilt 15.18°;
closure and lift never started. Planner handback was confirmed and both owned
children exited 0. Maximum recorded arm-joint speed was 0.804 rad/s and pelvis
tilt 7.95°. There were no automatic resets or in-task XY alignments. These are
failure/cleanup measurements, not a successful grasp. See the
[latest audit](/docs/artifacts/g1_grasp_baseline_20261009/native14/audit.json).
The task's closure stage never started, but planner handback changes the arm
and hand posture, visible at the end of the replay. The abort is a return to
planner control, not a physical pose hold or a validated loaded-object recovery.

## Reproduction

Use new output directories. Required local assets and external environments
are the same as the preceding SONIC milestones.

```bash skip
.venv/bin/python -m pytest -q \
  docs/artifacts/g1_grasp_baseline_20261009/test_grasp_reference.py

/home/jihun/work/GR00T-WholeBodyControl/.venv_sim/bin/python \
  docs/artifacts/g1_grasp_baseline_20261009/demo_hand_fixture.py \
  --output /tmp/NEW_FIXTURE --central-only --wrist-height 0.82

bash docs/artifacts/g1_grasp_baseline_20261009/demo_run_trial.sh \
  /home/jihun/work/dimos_dev/docs/artifacts/g1_grasp_baseline_20261009/NEW_NATIVE_RUN
```

The native launcher requires a private user/network namespace with loopback
as its only interface, verifies that it differs from the host namespace, and
owns only its simulator/controller children. It does not access a policy server
or a real robot network. A normal process exit means the bounded experiment
and cleanup completed; inspect `task_success` and the independent task score
for the manipulation outcome.

## Remaining service work

First collision-check the entire preparation-to-grasp path, including open
finger geometry, and account for measured whole-body tracking error. The lower
grip's contact fixture passes; the approach on the standing robot still fails.
Then validate the contact-triggered lower/release/withdraw sequence. One lift
has passed; a complete grasp-return cycle has not. Then test object offsets,
tray transfer and recovery from failed grasps. A visual-policy route needs matching camera/state preparation and data
or a checkpoint that demonstrates the tray task. The fixture alone does not
resolve that requirement.

Before a physical object trial, measure the real bottle/table/tray, establish
camera-to-robot calibration and physical hand limits, and verify the service's
unloaded motions and loaded-object recovery under the actual controller.
Package the verified sequence as a DimOS service once those behavior boundaries
are measured. Navigation comes after the fixed-table service is repeatable.
