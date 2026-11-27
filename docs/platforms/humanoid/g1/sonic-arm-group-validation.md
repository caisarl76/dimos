# Seven-joint SONIC arm experiment: 2026-10-08

The left-arm experiment now controls all seven measured joint errors through the
existing guarded SONIC reference path. One native MuJoCo trial completed reach,
three-second hold, return, planner handback, and settling. Two further trials
aborted during holding. This is a working simulation prototype with unresolved
repeatability, not a commissioned manipulation executor.

The user accepted approximately 10–20 mm of position error for this stage. The
experiment therefore uses **20 mm** as the hand-frame position tolerance. This
decision does not change the results or criteria of earlier experiments.

## Motion and ownership

After the existing shoulder-roll clearance preparation, capture the measured
left-arm joints and pose. Request shoulder pitch −0.15 rad and elbow +0.10 rad;
the other five joint goals remain at their captured values. Compute a frozen
hand-pose goal using those joints and the captured waist. In these runs the
requested displacement was approximately +43 mm forward, +9 mm left, +6 mm up,
relative to the pelvis. This is a joint-goal experiment with a pose acceptance
check; it does not plan a straight Cartesian path or preserve the original hand
orientation.

Python integrates the seven measured errors into bounded encoder references.
The native C++ SONIC controller remains the sole body/hand motor-command owner.
Other body references remain fixed during the arm cycle; SONIC supplies the
whole-body response. Neither Dex3 grasp actions nor an object/table were added.

The measured frame is the **`left_wrist_yaw_link` origin relative to `pelvis`**.
It is computed from waist and arm joints in one feedback packet. It is not a
calibrated fingertip, tool center point, or grasp point. Return uses the frozen
captured seven joints and pose, including the original waist contribution.

The reference envelope is the original reference ±0.25 rad, clipped to model
joint limits, with per-joint slew at most 0.05 rad/s. Reach and return require
all joint errors ≤0.05 rad, position error ≤20 mm, orientation error ≤10°, both
arms' maximum joint speed ≤0.20 rad/s, and body maximum joint speed ≤0.30 rad/s
for 0.5 seconds. Holding must maintain those criteria for three seconds; a
sample gap over 80 ms aborts holding. Reach and return each have a 15-second
deadline. The speed criteria qualify dwell/hold, rather than imposing an
instantaneous speed cap throughout travel.

## Recorded outcomes

All three runs used identical experiment sources and native binary. Each ran
inside a private network namespace containing only loopback. No robot host was
contacted or commanded.

| Run | Result | Pose at reach | Subsequent outcome |
| --- | --- | --- | --- |
| `cycle01` | Full cycle succeeded | 6.89 mm / 0.73° error | 3.020 s hold; return 6.05 mm / 1.91°; clean native exit |
| `cycle02` | Hold aborted | 6.54 mm / 2.14° error | Elbow speed 0.20493 rad/s exceeded 0.20; planner handback and settling confirmed; native exit −6 after shutdown |
| `cancel_hold01` | Hold aborted before injection | 8.85 mm / 1.56° error | Shoulder-yaw error 0.05074 rad exceeded 0.05; cancellation was **not injected or validated**; clean native exit |

In `cycle01`, actual displacement at reach qualification was **[39.08, 4.23,
10.33] mm**, compared with requested **[42.64, 8.82, 6.64] mm**. At the end of
holding, actual displacement was [42.44, 6.77, 8.81] mm. Maximum position error
during holding was 7.16 mm and maximum orientation error was 2.59°. Return's
largest joint error was 0.03536 rad. The complete cycle took 10.03 seconds,
excluding preparation and handback.

Both hands stayed at least 20 mm from the modeled lower body during every
recorded active-cycle sample in all three runs. The distance query is capped at
20 mm, so the logs establish a lower bound. This checks hand/lower-body geometry,
not collision clearance against every robot link or a task environment.

Peak measured arm speeds during the respective active cycles were 0.263, 0.252,
and 0.336 rad/s. These include travel. In `cycle02`, the decisive 0.20493 rad/s
sample occurred about 1.64 seconds into holding while position error was only
5.77 mm. In `cancel_hold01`, holding began with a maximum joint error of
0.04959 rad, and crossed 0.05 rad about 0.16 seconds later while position error
was 7.92 mm. The user's position tolerance was satisfied at both aborts.

Planner handback was confirmed in 122, 164, and 122 ms respectively, followed by
the sampled settling interval. `cycle02` then encountered a CycloneDDS C++
`EntityDelegate::prevent_callbacks()` assertion during process teardown, after
the normal-stop log. Its exit status was −6. This is recorded separately from
the earlier successful handback; teardown reliability remains unresolved.

## Evidence and reproduction

- MuJoCo replay of completed cycle (`../../../artifacts/g1_sonic_arm_group_20261008/mujoco_replay/seven_joint_reach_hold_return.mp4`)
- Replay page (`../../../artifacts/g1_sonic_arm_group_20261008/mujoco_replay/index.html`)
- [All-run audit](/docs/artifacts/g1_sonic_arm_group_20261008/audit.json)
- Experiment and raw captures (`../../../artifacts/g1_sonic_arm_group_20261008/`)
- Seven-joint controller (`../../../artifacts/g1_sonic_arm_group_20261008/demo_arm_cycle.py`)
- [FK cross-check](/docs/artifacts/g1_sonic_arm_group_20261008/fk-verification.json)

The replay displays recorded qpos with a full-body view, arm close-up, phase
labels, and measured speed overlays. It includes clearance preparation and the
completed arm cycle, ending before planner handback. It does not rerun dynamics.

The 16 focused controller tests pass, including all-joint/pose qualification,
20 mm tolerance boundaries, return to the captured pose, cancellation, stale
feedback, hold gaps, reference bounds, and deadlines. Ruff checks pass. The
offline audit reconstructs goals/errors from raw feedback, checks reference
packing and bounds, dwell/hold windows, clearance, posture, handback, and exit.
Its aggregate result is deliberately **false**: `cycle01` passes, while the two
hold aborts retain their failed task outcomes.

FK agrees with 31 saved model poses to approximately 1.3e−15 m at identical
joint values. During these live captures, nearest-time model pose versus
telemetry FK differed by at most 1.16 mm, with nearest-sample offsets up to
18.2 ms. This supports mapping consistency, not hardware calibration or clock
synchronization.

```bash skip
# Pure controller tests; no robot or controller is started.
.venv/bin/python -m pytest docs/artifacts/g1_sonic_arm_group_20261008/test_arm_cycle.py -q

# Offline audit: exit 1 is expected because two task outcomes failed.
.venv/bin/python docs/artifacts/g1_sonic_arm_group_20261008/demo_audit.py \
  docs/artifacts/g1_sonic_arm_group_20261008/cycle01 \
  docs/artifacts/g1_sonic_arm_group_20261008/cycle02 \
  docs/artifacts/g1_sonic_arm_group_20261008/cancel_hold01
```

For an additional authorized simulation trial, `demo_run_trial.sh` takes a new
absolute output directory and optional scenario. It requires permission to
create the private network namespace and uses the recorded local native
binary/assets. It is not a PC2 launch command. See each run's `manifest.json`
for exact executable arguments, source hashes, and binary SHA-256.

## Next implementation step

Keep the 20 mm position tolerance. First test a narrower entry condition for
holding (for example ≤0.04 rad joint error and ≤0.15 rad/s arm speed throughout
the existing 0.5-second dwell), while retaining the present hold limits. This
directly addresses entering holding at the acceptance boundary. It does not
explain the later speed excursion; measure the ongoing reference corrections
and response before deciding whether to change their gain or hold behavior.

Then repeat the complete cycle and reach the intended cancellation injection.
Investigate the native DDS teardown assertion separately. After those results,
expose a bounded arm command session to DimOS planning, add short Cartesian
waypoints, and then Dex3 grasp/release actions. Camera extrinsics and grasp-point
calibration remain necessary for the first object-to-tray task. Hardware motion
commissioning remains a later step.
