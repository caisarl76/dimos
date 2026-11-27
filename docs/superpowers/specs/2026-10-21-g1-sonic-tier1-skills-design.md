# G1 SONIC arm skills, Tier 1: gestures, longer holds and camera-guided pointing

Status: design, builder-checked, user decisions in, awaiting approval. Author: dimos-dev-84, 2026-10-21. Builds on
`jihun/feat/g1-sonic-arm-skill` at `28bd4cf83`.

## Intent

The G1 (pc2 .223, no hands, head D435i colour camera) can already move its left arm through
SONIC from a DimOS skill: one joint or one full-range pose, out and back, with a path check,
balance-centred envelopes and a smooth retreat. An agent can call those skills, but the result
is stiff: a single out-and-back move with a one-second hold, and no use of what the camera sees.

Tier 1 makes the arm expressive and connected to perception, so an agent can respond naturally
to "wave at them", "raise your hand and keep it up", "point at the red box":

1. **Gestures**: multi-pose motions in one call (wave, point forward, stop sign, raise hand,
   beckon).
2. **Longer holds**: keep a pose for up to 20 s before returning.
3. **Pointing**: point the arm in a direction, or at an object found in the camera image.

## Goals

- `perform_gesture(name)` with five named gestures, each a short keyframe sequence that ends at
  rest and hands back to the planner.
- A hold duration of 1–20 s on the named poses and joint targets.
- `point_at(azimuth_deg, elevation_deg)` and `point_at_object(description)`.
- `stop_arm()`: the agent can stop a running hold or gesture ("put your arm down"); the arm
  retreats smoothly (user decision).
- Every new motion keeps today's safety properties: path check before sending, envelope and
  speed watch while streaming, a smooth retreat on any stop, one arm command at a time, never
  starting or stopping native control.
- Every new skill runs end to end against the MuJoCo stand-in in an automated test before it
  goes to the robot.

## Non-goals

- Right arm, two-arm gestures, waist motion, walking, hands (Tier 2 or other sessions).
- Raising the arm speed above 0.47 rad/s. Gestures run at the commissioned speed until the
  robot verifies them; speed is a later, separate decision.
- Tracking a moving object, or re-aiming while pointing (`point_at_object` aims once).
- Two-robot synchronization (the dual-G1 design, `2026-10-21-g1-dual-sonic-arm-design.md`,
  another session). Tier 1 must stay reusable by it.

## Anti-goals (failure even if it works)

- A stop that hands a raised or moving arm straight to the native planner (the snap measured in
  MuJoCo: 23 rad/s, feet off the floor). Every stop must retreat along a planned path. The one
  accepted exception is overspeed during a retreat (§1).
- A gesture that only passes because a limit was loosened for it. Envelopes stay as today; if a
  gesture trips them, the gesture changes.
- Pointing that looks confident at the wrong thing. `point_at_object` reports what it found and
  where; if nothing is found it says so and does not move.
- Skills that only work in simulation, or only on the workstation: the robot PC runs them.

## What exists (the base)

`dimos/robot/unitree/g1/sonic_arm_skill.py` and `sonic_arm_reference.py` at `28bd4cf83`:

- `stroke_plan(base, delta)`: min-jerk out, hold, back, with each stroke stretched so the
  largest joint peaks at `PEAK_SPEED` = 0.47 rad/s; `SonicEncoder` turns poses into tokens;
  `token_schedule` adds 2 s blends from and to the planner token.
- `_move`: measured base pose, plan, `ArmCollisionChecker.first_contact` (2 cm, start-pose
  contacts tolerated unless closed by 5 mm), encode, re-check the body did not move, profile
  limits, then `_stream`.
- `_stream`: one chunk, or two split at the hold (`chunk_rows`, session limit 1000 rows); on an
  envelope or speed stop, `retreat_row` resumes at the matching point of the plan's way back
  (progress mapping) and finishes there; an overspeed during the retreat cancels.
- Envelopes: `group_margins` for `move_upper_body`; `pose_margins` (legs and waist 0.15 rad,
  left arm 0.45, right arm 0.35) for poses and joint targets; `max_dq` (config default 3.0; the
  robot and MuJoCo runs passed `--max-dq 2.0` on the command line, and gesture tuning and the
  forced-stop tests use 2.0).
- `ObserveSkill` and `CameraModule` (D435i colour, 640×480, factory intrinsics, URDF mount on
  `torso_link`, pitched 0.83 rad down) in `unitree-g1-sonic-arm`.
- `sonic_sim_peer.py` (sync artifacts): real-time MuJoCo stand-in for the guarded controller,
  feet-down startup. It imports the lerobot harness from `~/work/lerobot` and the HF SONIC
  models.

## Decided design

### 1. Keyframe engine (shared by every skill)

A plan is a list of **keyframes**: absolute targets for the left-arm joints the skill sets
(shoulder pitch, roll, yaw, elbow; wrists stay at their measured values unless a gesture sets
them), each with a hold duration. The engine builds, from the measured base pose:

- the forward path: lead-in, then for each keyframe a min-jerk segment from the previous
  keyframe (each segment stretched so its largest joint peaks at `PEAK_SPEED`) and its hold,
  then a final segment back to the base pose, lead-out;
- the token plan: `token_schedule` around the encoded forward path, as today.

**Chunking.** The session takes at most 1000 rows (20 s) per chunk. The forward plan is split
only where the reference is stationary. Where a split is needed and the plan has no hold close
enough, the engine lengthens a keyframe's pause to 1.24 s (my call). That is the session's 0.3 s
blend plus the encoder's 45-row lookahead, so the encoded tokens, not just the poses, are still
where the split falls. The chunk blend then happens
while the arm is still, and no blend ripple lands on a moving arm. This generalizes
`chunk_rows`, and plans that fit in one chunk still stream as one.

**Retreat (the key change; my call after review).** `retreat_row`'s progress mapping only works
for out-and-back plans. Re-tracing a whole gesture in reverse was rejected for two reasons:
- a wave stopped on its last swing would wave backwards for about 25 s;
- the reverse path would be over 1000 rows with no stationary point to split at, so it would
  fail to submit and the arm would snap.

Instead the retreat is short and bounded:
- **During forward segment *i*, at fraction *f*:** the arm reverses along segment *i* back to
  keyframe *i*−1 (the same path it just came along, min-jerk, at the same speed cap). It then
  takes a direct min-jerk segment from keyframe *i*−1 to the base pose, with lead-out and the
  blend to the planner token.
- **During the hold at keyframe *i*:** the arm takes the direct segment from keyframe *i* home.
- **On the final return segment:** it keeps going.
- **For a one-keyframe plan:** this is exactly today's `retreat_row` behaviour, which must stay
  identical for `move_upper_body`, poses and joint targets.

**Precomputed.** At plan time the engine builds, path-checks and encodes the direct-home segment
for each distinct keyframe. Wave has three distinct keyframes, because its swings revisit the
same two yaw targets. Each retreat is then one segment back plus one segment home, well under
1000 rows (so one chunk) and under about 15 s.

**Encode cost.** On pc2 the encoder costs about 4.3 ms per row (`SonicEncoder` docstring: 1.5 s
per 350 rows). A wave of about 1650 forward rows plus three home segments takes about 9 s before
motion starts. That is accepted for now. The builder may batch encodes but must not stream before
every retreat is ready.

**Overspeed during a retreat** still cancels at once, as today. This is an accepted exception to
the "every stop retreats" rule: overspeed means the reference is no longer being followed, so
streaming more of it does not help. Short retreats keep the exposure small.

**Path check.** `ArmCollisionChecker.first_contact` runs over the forward path and over each
direct-home segment. A reverse along a forward segment revisits checked positions.

**Existing skills move onto the engine.** `move_upper_body`, `move_left_arm_pose` and
`move_left_arm_joints` become one-keyframe plans; their behaviour, envelopes and tests stay as
they are (all 44 current tests stay green).

### 2. `perform_gesture(name)`

`GestureName = Literal["wave", "point_forward", "stop_sign", "raise_hand", "beckon"]`, each a
fixed keyframe list in code. Starting values (my call, to tune in MuJoCo; elbow +1.43 rad is
straight, 0 is bent 90°):

| Gesture | Keyframes (pitch, roll, yaw, elbow, rad; hold s) |
|---|---|
| wave | raise (−0.3, 1.3, 0.0, 0.0; 0) → yaw −0.35 (0) → yaw +0.35 (0) → ×3 → back to raise (0.5) → rest |
| point_forward | (−1.5, 0.25, 0.0, 1.35; 2.0) → rest |
| stop_sign | (−1.3, 0.25, 0.0, 0.0; 2.0) → rest (forearm up, palm forward) |
| raise_hand | (−2.6, 0.45, 0.0, 1.2; 2.0) → rest |
| beckon | (−1.2, 0.3, 0.0, 0.6; 0) → elbow 0.0 ↔ 0.6 ×2 → rest |

At 0.47 rad/s these are slow. With `stroke_plan`'s 2 s minimum per segment, a wave of three
swings is about 27 s of motion and about 33 s with leads and blends. That is accepted for the
first robot verification; speed is a later decision.

Envelope: `pose_margins` (as for full-range poses). Gestures must pass in MuJoCo with the
feet-down startup at least three times in a row before going to the robot.

### 3. Longer holds

Not a separate skill (my call; smaller interface than the proposed `hold_pose`): an optional
`hold_seconds: float = 1.0` parameter, range 1–20, on `move_left_arm_pose` and
`move_left_arm_joints`, which sets the one keyframe's hold. Gestures keep their fixed holds.

### 3b. `stop_arm()` (user decision)

A skill that asks the running arm motion to stop. The streaming loop treats the request exactly
like an envelope stop: the same short retreat (§1), then handback. It returns at once with
"stopping" or "no arm motion running", and the motion's own result reports "stopped on request".

- It must not take `CAP_MOVEMENT`, which the running motion holds.
- Checked while planning: the agent (`McpClient`) handles one message at a time and an
  `instant` skill blocks it until it returns, so the motion skills become
  `lifecycle="background"`, the same pattern as `go_to` / `stop_navigation`. They return at once
  and report their result as a tool update, and `stop_arm` (no capability) runs alongside.
- Nothing else changes: the safety stops still act on their own.

### 4. `point_at(azimuth_deg, elevation_deg, hold_seconds=2.0)`

Directions are in the torso frame: azimuth 0 straight ahead, positive to the robot's left;
elevation 0 horizontal, positive up. Reachable cone for the left arm (refuse outside, my call):
azimuth −35…+110°, elevation −75…+75°. The downward reach matters, because the head camera's
rays run 26–69° below horizontal; a −40° floor would have refused about half of what the camera
sees.

The cone is only a coarse request bound (ruled during the build, 2026-10-22). MuJoCo showed the
clear region is irregular, not a rectangle:
- clear: straight ahead at shoulder height (az 4°, el 5°) and ahead-and-down at moderate roll
  (18°, −33°);
- not clear: ahead-and-down at small roll (8°, −36°) brings the upper arm within 2 cm of the torso;
- unreachable: below −60° the wrist meets the hip, and the far right of the bound is out of reach.

So each request inside the bound is decided by its own IK (≤ 3°) and path check, and is refused
with the reason before anything is sent.

Inverse kinematics: a straight arm (elbow 1.35 rad, yaw 0) pointing so the shoulder-to-wrist
vector matches the direction. Solve shoulder pitch and roll by a small numeric minimization over
MuJoCo forward kinematics on the same `scene_43dof.xml` (already on pc2 for the path check; a
`kinematics` helper next to `ArmCollisionChecker`), seeded from a coarse grid. Refuse if the best
solution is more than 3° off. Then run it as a one-keyframe plan (path check, envelopes,
retreat), reporting the achieved direction.

### 5. `point_at_object(description, hold_seconds=2.0)`

1. Take the latest frame from the camera stream (as `observe` does).
2. `VlModel.query_detections(frame, description)` (DimOS's existing interface); pick the
   largest box (VLM boxes all carry confidence 1.0). No box: return "not found", no motion.
3. Box centre pixel → ray in `camera_optical` with the factory intrinsics → torso frame via the
   camera mount (the blueprint's `torso_link → camera_link` transform and the optical rotation).
4. Parallax: the arm points from the shoulder, the ray starts at the camera. Assume the object
   is 2 m along the ray (my call; there is no depth) and point the shoulder at that point.
5. `point_at` with that direction. Outside the reachable cone: say so, no motion.
6. Report the label, the pixel box, the direction and whether it was clipped.

The VL model is a container config (`vl_model_name`), used through DimOS's OpenAI-compatible
client, so it runs through the Genon gateway like the agent. Which model gives usable pixel
boxes is measured during the build (below). Limitation to state in the prompt: the head camera
looks down at the floor about 0.3–2 m ahead, so objects above about waist height may be out of
view.

### 6. MuJoCo stand-in in the repo, and end-to-end tests

Port `sonic_sim_peer.py` into DimOS as `dimos/robot/unitree/g1/sonic_sim_peer.py` and give it
its own copies of what it imports today, with attribution, so it no longer imports from
`~/work/lerobot` or the artifact directory:
- the lerobot harness's `Robot`/`Decoder` (PD plant and SONIC decoder);
- `arm_reference.py`'s `IdlePlanner` (`planner_sonic.onnx`) and the anchored `Encoder`, or
  DimOS's `SonicEncoder` where it is equivalent.

The model directory (the HF SONIC snapshot) and the scene path become arguments; the scene path
today points into a separate GR00T worktree. Keep a thin wrapper at the artifact path (or tell the
dual-G1 session the new path; it runs the artifact today).

Add a pytest e2e module marked `native_e2e` (excluded by default; there is no MuJoCo marker, and
these run in real time for minutes), skipped when the models or the scene are absent, that starts the stand-in and the execution module, then runs each new
skill and asserts reach, envelopes, return to rest and handback.

### 7. Agent prompt and limits

- `G1_SONIC_ARM_SYSTEM_PROMPT` lists the gestures, `hold_seconds`, `point_at` (with the cone)
  and `point_at_object` (with the camera limitation), and keeps the error/result rules.
- Gesture plans are capped at 40 s. Worst case is a stop near the end, followed by a retreat of
  up to 15 s, so the profile's `max_duration` becomes 75 s.
- `max_observation_age` stays at 35 s, because the session checks it per chunk (≤ 20 s each).
  `_move`'s own precheck changes: today it compares that age against the whole plan
  (`len(plan)/RATE + 0.5`), which refuses any plan over about 34.5 s. It must compare against
  the longest chunk instead. `mcp_timeout` needs no change, because the motion
  skills return at once (§3b).

### 8. Coordination with the dual-G1 design

That design reuses `sonic_arm_skill.py`'s per-robot logic and plans its own `wave`. The engine
and the gesture table live in the single-robot module as reusable functions, not methods tied to
one execution reference, so the dual container can expose `perform_gesture` with its `robot`
parameter. That session has not started code (no dual files or branch on 2026-10-21), so the
Tier-1 engine lands first (my call) and the dual container builds on it. Tell that session once
this design is approved, so `wave` is implemented once.

## Validation and acceptance

- Unit tests:
  - keyframe plan timing and speed cap;
  - chunk splits only at stationary rows, every chunk ≤ 1000 rows;
  - retreat selection (segment *i*, fraction *f* → reverse to keyframe *i*−1, then home;
    every retreat ≤ 1000 rows);
  - IK accuracy against forward kinematics across the cone;
  - pixel → torso-frame direction against the known mount.
- Integration tests (scripted peer): each skill streams, holds, returns and hands back; a stop
  mid-segment reverses to the previous keyframe and goes home, with the stream ending at the
  planner token.
- MuJoCo e2e: every gesture and a set of pointing directions complete, envelopes hold, tilt
  ≤ 3°, floor contacts ≥ 7, and a forced stop mid-gesture retreats without a speed spike.
- `point_at_object`: on 10 saved head-camera frames with hand-labelled boxes, the chosen model's
  box centre is within 30 px for at least 8. If no model reaches that, the skill ships disabled
  with that finding.
- All existing tests stay green; ruff and mypy clean.
- Robot (operator), on the stand at 0.47 rad/s: each gesture, then `stop_arm` mid-gesture,
  then pointing.
- The 10 head-camera frames are captured during the first robot session, with objects placed
  0.5–2 m ahead on the floor and on a low table. I label them.

## Deployment to pc2 (user decision)

pc2 moves onto this branch. No more hand-adapted backports. This is the first robot step,
before any Tier-1 run:
1. Back up pc2's current DimOS copy.
2. Install this branch's files.
3. Run the arm-skill tests on pc2.
4. Confirm the VL client, `openai` and MuJoCo import there.

The retreat-only backport becomes unnecessary once this is done.

## Left to the builder

- Exact keyframe values and holds within the table's intent, tuned in MuJoCo.
- Internal structure of the engine and how existing skills are expressed on it.
- IK method details and tolerances (within the 3° refusal rule).
- Which VL model meets the grounding bar; how to convert its coordinates.
- Test names and fixtures.
