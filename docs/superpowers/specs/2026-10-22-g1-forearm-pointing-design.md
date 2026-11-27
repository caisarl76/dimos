# G1 left arm: forearm pointing for point_at_object

Status: approved 2026-10-22, with two corrections folded in (hold-sample accuracy, image-edge grid). Author: dimos-dev-84, 2026-10-22. Builds on Tier 1
(`aa798e221`). The user skipped questions, so every decision below is my call unless marked.

## Intent

`point_at_object` points a straight left arm from the shoulder. That reaches only about the left
third of the head-camera image, measured from the idle pose on a 7×7 pixel grid:
- toward the centre, the upper arm comes within 2 cm of the torso;
- on the right, the target is out of reach.

The camera looks at the floor straight ahead, so most things a person asks about ("point at the
box") are refused.

People point at nearby things with a bent elbow: the forearm aims, and the hand stays in front of
the body. Doing the same lets the left arm point at the whole camera view without crossing near
the body.

## Evidence (throwaway MuJoCo probes, 2026-10-22)

The probes used the real scene and checker, from the idle pose, over a 7×7 head-camera pixel grid.

**Probe 4** is the one that tests this design. It used:
- the floor-intersection aim point from §1;
- the hard posture checks from §2;
- the search order from §2: refine, re-score, then path-check;
- a 7×7 grid that includes the image edges and all four corners:
  u ∈ {0, 106, 213, 320, 426, 533, 639} and v ∈ {0, 80, 160, 240, 320, 400, 479}.

Results:
- **Coverage:** all 49 pixels solve and pass the path check. The worst IK error is 0.04° after
  refinement.
- **Speed:** a solve takes 0.19 s median and 0.25 s at worst on the workstation.
- **Distances:** ray lengths to the aim point run from 1.32 m (bottom centre) to 2.85 m (top
  centre), and up to 3.18 m at the top corners. Probe 3 used the same method but started its grid
  at v = 40, so it did not cover the top edge.

**Earlier probes:**
- With a fixed 2 m aim point, the poses looked like pointing: upper arm slightly forward (pitch
  −5…−35°, roll 10…30°), forearm turned in by shoulder yaw (−15…−75°), elbow joint 30–75°, and the
  hand 0.12–0.26 m ahead at about waist height. Strokes were 7–8 s.
- Without a posture preference, the solver picked "chicken wing" poses: roll at its 120° limit,
  elbow fully bent, forearm folded down. They were safe but did not read as pointing. That is why
  §2 makes posture a hard check, not just a preference.

## Goals

- `point_at_object` points the forearm (the line from elbow to wrist, extended) at the object,
  within 3°, anywhere in the head-camera image the left arm can reach without nearing the body.
  The probe says that is the whole image from idle.
- The pose reads as pointing to a person watching: hand in front, elbow bent moderately, no joint
  at a limit.
- Same safety as every arm skill: path check from the measured pose, envelopes, smooth retreat on
  any stop, nothing sent when no pose qualifies.

## Non-goals

- Changing `point_at(azimuth, elevation)`, which stays the straight-arm direction skill. It could
  adopt forearm mode later.
- Tracking a moving object, re-aiming, the right arm, hands.
- Depth sensing: the camera is colour only.
- The dual-G1 container, which exposes no pointing skills.

## Anti-goals

- Contorted poses that are technically within 3°: joints at their limits, or the forearm folded
  down behind the upper arm.
- A confident point at the wrong place: the forearm line must pass through the aim point, and the
  report says how far off it is.
- Any motion when no pose qualifies.

## Decided design (my calls)

### 1. Aim point

- **The ray:** the ray goes through the box centre, using the head-camera intrinsics and mount from
  Tier 1.
- **Distance convention:** the length along that ray from the camera origin.
- **The aim point:** where the ray meets the floor plane, with the distance clamped to 0.5–4.0 m.
  A ray that does not descend (which can't happen with this mount unless the torso pitches far
  back) uses 4.0 m.
- **The floor plane:** from forward kinematics of the measured pose. It is the lower of the two
  ankle-roll origins, minus the sole offset, which the builder reads from the foot geometry.
- **Typical distances:** the camera sits about 1.26 m up, looking 26–69° down. Rays run about
  1.34 m (bottom centre) to 2.9 m (top centre), and about 3.3 m at the top corners, all inside the
  clamp.
- **Why depth matters here:** the forearm line has to pass through the point, and the wrist sits
  about 0.3 m from the camera, so a wrong depth turns into an aim error.
- **Raised objects:** objects on a table are aimed slightly behind themselves. That is accepted, and
  the prompt says so.

### 2. Forearm IK (`ArmKinematics.solve_forearm(q, point)`)

**Unknowns.** Shoulder pitch, roll and yaw, and the elbow, within `JOINT_BOUNDS`. The wrists and
every other joint come from the measured pose.

**Error.** The angle between the forearm direction (elbow-link origin to wrist-yaw-link origin) and
the direction from the wrist to the aim point.

**Natural-pose acceptance (hard checks, all required).** These are in the torso frame, measured at
the solved pose:
- the wrist is at least 0.10 m in front of the torso origin (x ≥ 0.10);
- the forearm points forward (the x component of its unit vector is at least 0.2);
- the elbow is bent 5–70° from straight (joint value 1.43 rad minus the bend);
- shoulder roll is at most 60°;
- every one of the four joints stays at least 5° inside its bound.

The probe's chosen poses all pass these checks.

**Preference among accepted poses.** The distance to the natural pointing posture
`PREF = (−45°, 25°, 0°, 30°)` (pitch, roll, yaw, elbow), with weights `(1, 1, 0.7, 0.7)`.

**Search order.**
1. A coarse 4-D grid: 10° steps on pitch and roll, 15° on yaw and elbow, roll ≥ 10°.
2. Keep seeds within 8°, rank them by distance to `PREF`, and take the first 40.
3. Refine each seed locally to the smallest error (a compass search, as in the straight-arm IK).
4. Keep the refined poses that are within 3° and pass the acceptance checks. Re-score them by
   distance to `PREF` and sort.
5. Path-check them in that order from the measured pose (`start=base`). The first that passes wins.

The guarantee is the best posture among the searched candidates, not a global optimum.

**Refusal.** If nothing passes, the skill refuses and names the cause: "cannot point within 3 deg",
"no natural pose points there", or "every pose that points there comes near the <body>". Nothing
is sent.

**Time budget.** Under 1 s on the workstation (the probe measured 0.29 s at worst). Expect about
1 s on pc2, inside the existing encode wait.

### 3. Skill behaviour

- `point_at_object` switches to forearm mode, with the same interface (`description`,
  `hold_seconds`) and the same failure messages.
- It runs as a one-keyframe plan through `_run`: envelopes `pose_margins`, `require_check=True`,
  and the usual retreat.
- The report adds:
  - `mode: "forearm"`;
  - `aim_point_m` (torso frame) and `aim_distance_m` (ray length);
  - `pose_deg`;
  - `planned_aim_error_deg`: from the IK at the solved pose;
  - `measured_aim_error_deg`: the median forearm error over timestamped feedback samples taken
    inside the hold window. The window runs from 0.5 s after arrival to 0.5 s before departure,
    using the plan's timing from stream start. It counts only samples taken before any stop or
    retreat, and is null when there are no valid samples. `reached_q` is not used: it records
    the furthest progress anywhere in the motion, which can fall in the outbound overshoot or the
    return. This is the same metric the simulator test computes.
- It drops the azimuth/elevation cone check, which applied to the straight arm.
- The prompt says the robot points with its forearm, can point anywhere it sees, and that
  objects on a table may be aimed slightly behind themselves.

### 4. Kept as is

Envelopes, the speed cap, the retreat, the lifecycle and `stop_arm`, `query_boxes` and the vision
model setup.

## Validation

There are two different accuracies, each with its own bar:
- **Planned accuracy:** the IK's own error at the solved pose. Bar: ≤ 3°.
- **Measured accuracy:** how far the real or simulated arm points from the aim point while holding.
  It includes SONIC's tracking shortfall; Tier 1's point_at reached 88–94% of the move in MuJoCo.
  The median over the hold-window samples is used, as for the report.
  Bar: ≤ 8° in MuJoCo, a provisional value that the first end-to-end run checks and records.

**Unit tests:**
- the forearm direction from forward kinematics;
- all 49 pixels of the 7×7 grid, from idle, with the real floor-or-clamp aim point. The grid is
  u ∈ {0, 106, 213, 320, 426, 533, 639} and v ∈ {0, 80, 160, 240, 320, 400, 479}, so it includes
  the image edges and all four corners. Each one
  - solves with planned error ≤ 3°,
  - passes every natural-pose acceptance check (asserted per case),
  - and clears the path check;
- refinement re-scoring: when refinement reorders the candidates, the posture-best accepted
  candidate is the one returned;
- refusal when the aim point is behind the robot;
- the floor intersection against the known camera height and ray length.

**MuJoCo end-to-end** (`native_e2e`). `point_at_object` is driven with a fake vision model whose
boxes sit at the image centre, the right edge and the bottom right.
- During the hold: take the stand-in's logged joint poses over a hold window (from 0.5 s after
  arrival to 0.5 s before departure, using the plan's timing). Compute the forearm error by forward
  kinematics: the median must be ≤ 8°. The report's `measured_aim_error_deg` comes from the
  skill's own feedback samples over the same window, and must agree with it to within 2°.
- Over the whole motion, as for every case: reach ≥ 0.8, tilt ≤ 3°, contacts ≥ 7, peak speed
  ≤ 2 rad/s, and handback.
- The retreat is checked separately: the arm ends within 0.05 rad of its start pose.

**Robot** (operator). The same three cases with real objects, once the 10-frame grounding run has
picked a vision model. Record the measured errors.

## Left to the builder

- The refinement method and step sizes, and the sole offset read from the foot geometry.
- How many candidates to path-check, within the 1 s budget.
- Whether `PREF` needs tuning after the simulator runs. The acceptance checks stay as they are.
