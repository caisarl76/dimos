# Bottle-to-tray simulation baseline

Validated on 2026-10-09. The user selected a small plastic bottle for the first
G1 service. A physical bottle/table/tray scene now runs through the live
RGB/state → Pi0.5 → DimOS → native SONIC → MuJoCo pipeline.

**The control session completed; the manipulation task failed.** In the valid
12-second trial the bottle stayed on the table. There was no qualifying finger
contact, lift, or placement. This is a reproducible evaluation environment and
an observed policy baseline, not a completed pick-and-place service.

- Replay: recorded robot motion beside actual policy RGB inputs (`../../../artifacts/g1_pick_place_20261009/trial04/replay/task_policy_replay.mp4`)
- Measured joint speeds (`../../../artifacts/g1_pick_place_20261009/trial04/replay/joint_speeds.png`)
- Scene preview (`../../../artifacts/g1_pick_place_20261009/geometry01/preview.png`)
- [Session and task result](/docs/artifacts/g1_pick_place_20261009/trial04/result.json)
- [Provenance/task audit](/docs/artifacts/g1_pick_place_20261009/trial04/audit.json)
- Visibility and geometric-overlap audit (`/docs/artifacts/g1_pick_place_20261009/trial04/geometry_audit.json`, not committed)

## Scene and geometry

The scene builder (`../../../artifacts/g1_pick_place_20261009/demo_scene.py`)
reuses the existing G1 model with two Dex3 hands. It preserves every original
robot joint address, actuator mapping, limit and nominal head camera. A free
bottle joint is appended after the robot: 57 position coordinates, 55 velocity
coordinates and the original 43 actuators. Neither external repository was
modified.

Provisional dimensions, pending measurements of the real task setup:

| Item | Simulation value |
|---|---|
| Bottle | Cylinder approximation, 50 mm diameter × 160 mm tall, 80 g |
| Table | 600 × 900 mm surface, top at 700 mm |
| Tray | 220 × 180 mm interior, floor at 708 mm |
| Source center | `[0.40, -0.14, 0.78]` m when settled |
| Tray center XY | `[0.43, 0.14]` m |

The static [geometry check](/docs/artifacts/g1_pick_place_20261009/geometry01/scene_validation.json)
uses a previously measured standing planner posture. Four right-arm
position-only IK probes, pregrasp, grasp center, lift and above tray, are within
joint limits, with errors below 0.05 mm. The smallest sampled robot-to-table/tray
clearance is 49.4 mm. These are individual geometric poses; wrist orientation,
Dex3 grasp suitability, self-collision along the connecting paths, and SONIC
tracking of those poses are not validated. The probe site is not a calibrated
grasp TCP.

The static camera sees 6,517 bottle pixels and 39,223 tray pixels. During the
actual policy trial, all 34 submitted frames still show both objects, with
minimum visible areas of 5,568 and 12,610 pixels respectively. The bottle
touches an image edge in 16 frames and the tray in all 34; this is partial
visibility, not proof of complete framing throughout motion. No detector was
introduced: segmentation is an offline diagnostic and is never a policy input.

## Task scoring

TaskGeometry and PickPlaceScore (`../../../artifacts/g1_pick_place_20261009/demo_task_metrics.py`)
produce simulation ground-truth results separately from SONIC session status.
The required sequence is:

1. Observe the bottle supported by the source table, outside the tray, without
   hand contact.
2. Observe thumb plus index or middle finger contact on the same hand, each
   with a sampled normal force of at least 0.05 N, and the bottle bottom at
   least 40 mm above the table without table/tray support for 200 ms.
3. Observe the entire bottle inside the tray with a 3 mm margin, supported by
   its floor, without hand contact, with hands at least 50 mm away. Linear and
   angular speed must remain below 0.02 m/s and 0.25 rad/s for one second.

The containment calculation includes the cylinder's orientation and projected
extent. Initial placement in the tray, a close-hand command, or a finished
trajectory cannot pass. Drops, simulator resets, timestamp discontinuities,
and large position jumps inconsistent with measured speed invalidate the
relevant evidence. The jump check is bounded; it is not a general detector of
arbitrary teleports. Contact-force thresholds are simulation scoring choices,
not physical Dex3 force calibration.

Twenty-three focused tests cover scoring failures, dwell timing and actual
MuJoCo table/tray/hover support and containment geometry. Observer fixtures
hold the robot pose explicitly and do not claim standing or grasping ability.

## Valid trial: trial04

The existing policy server on local port 5561 loaded the WBT Pi0.5 handover
checkpoint. The task prompt was `pick the small plastic bottle from the table
and place it in the blue tray`. Changing the prompt does not train a tray task.
The checkpoint's local training configuration names the bottle-handover
dataset. Other local model configs inspected name Humanoid Everyday datasets;
this inspection did not establish a validated bottle-to-tray policy.

| Measurement | Result |
|---|---|
| Policy updates | 34 accepted, 34 distinct paired RGB/state inputs |
| Task duration / replan interval | 12 s / 350 ms |
| Policy round trip | 289–330 ms |
| Observation age at accepted update | 333–462 ms; 850 ms guard retained |
| Reference packets | 648 including startup holds; median interval 20.16 ms |
| Confirmed planner handback | 141 ms after request |
| Standing | Minimum pelvis 0.778 m, maximum tilt 10.30°, support disabled |
| Resets during scored interval | Zero |
| Cleanup | All three owned children exited with code 0 |
| Task outcome | `waiting_lift`, `task_success=false` |
| Closest hand-to-bottle gap | 24.4 mm |
| Maximum bottle displacement | 0.0019 mm |
| Qualifying finger contact / tray support | Zero samples / zero samples |

Peak measured arm speed was **5.89 rad/s**, at right wrist roll, about 0.672 s
after policy requests began. The user's previous acceptance of a particular
hand/leg slip spike does not assess this separate peak. Measured finger joints
exceeded nominal MJCF limits by up to 0.0080 rad (0.46°); physical command-limit
commissioning remains open.

An offline replay of the geometry found contacts/overlaps between hands and
the table/tray, with maximum penetration 2.42 mm. These are geometric overlap
measurements, not reconstructed contact forces or a collision-free path result.
Dynamics ran with MuJoCo 3.8.1; rendering and offline geometry checks used
3.10.0. The replay uses saved poses only and sends no commands. Its right panel
holds the actual submitted RGB between policy requests.

## Setup and corrected failures

The bottle is parked away from the robot during controller initialization and
the initial robot reset. After SONIC is standing, the simulator places the
bottle at the source exactly once, logs that setup, and allows it to settle.
The runner requires measured table support, position, low speed and hand
clearance before warmup/inference. No object pose assignment occurs during
the scored task. This models placing the object on the table before an attempt.

Earlier runs remain available and are not counted as successful task attempts:

- `trial01`: simulator import-path conflict; exited before native controller
  startup. Artifact-local imports fixed it.
- `trial02`: at 400 ms replanning, the previous observation reached 850 ms just
  as the next prediction arrived. Guarded planner recovery and cleanup worked.
  Replanning was shortened to 350 ms without increasing the age limit.
- `trial03`: the control session completed, but startup finger/bottle contact
  knocked the bottle off the table before policy execution. Its `waiting_source`
  score exposed the invalid initial condition. Parking and explicit source
  preparation fixed the setup for `trial04`.

The independent audit (`../../../artifacts/g1_pick_place_20261009/demo_audit.py`)
checks source hashes, camera/physics pairing, named state order, accepted
predictions, lifecycle, the one-time setup boundary and replayed task score.
All 286 captured camera frames were matched to physics snapshots. A separate
read-only review found no remaining material issue in the reviewed fixes and
trial04 evidence. No physical robot was contacted or commanded in this stage.

## Reproduction and remaining work

From the repository root, with the existing local policy server, native SONIC
binary/assets and GR00T environments used by the preceding milestone:

```bash skip
.venv/bin/python -m pytest -q docs/artifacts/g1_pick_place_20261009/test_task_metrics.py
bash docs/artifacts/g1_pick_place_20261009/demo_run_trial.sh \
  /home/jihun/work/dimos_dev/docs/artifacts/g1_pick_place_20261009/NEW_RUN live
.venv/bin/python docs/artifacts/g1_pick_place_20261009/demo_audit.py \
  docs/artifacts/g1_pick_place_20261009/NEW_RUN
```

The launcher uses a fresh loopback-only network namespace for simulator and
native controller. Its host bridge accesses only the existing local inference
server. Use a new output directory each time. The runner's success exit means
the session contract passed; inspect `task_success` for the manipulation result.

The next milestone is a **verified grasp-and-lift baseline** in this scene:
compare camera/state and object geometry with the handover training examples,
establish an orientation-aware Dex3 grasp and approach through SONIC, then
validate transfer, release and withdrawal. A policy route needs matching
demonstrations or a checkpoint shown to perform the tray task; the current
trial does not isolate whether prompt, visual domain, initial state or grasp
geometry caused the failure.

Real object/table measurements, camera extrinsics, physical hand/controller
limits, supervised unloaded commissioning and loaded-object recovery remain
before a hardware task attempt. The reusable DimOS service API and eventual
navigation integration follow a measured manipulation baseline.
