# G1 arm engine: faster cap, both arms and Dex3 hands, imported gestures

Status: the three decisions were answered by the user on 2026-10-23 (below). Phase 1 is committed locally as c70216be0 (not yet pushed). Phase 2 was revised after review on 2026-10-23 (findings 1–5, folded in below) and awaits approval. Author: dimos-dev-84, 2026-10-23. Requested by the
user through the dual-G1 session. Builds on `0d664204e`.

## Outcome (2026-10-23, after the MuJoCo validation)

Two motions were dropped under decision 3, with the evidence recorded in `gestures/SOURCE.md`:
- `open_window`: refused by the both-arm path check from the stand-in's standing pose (it comes
  near the other arm's wrist).
- `wave_two_hands`: right_hip_pitch reached −0.151 rad, beyond the 0.15 leg envelope, in 3 of 3
  runs.

The retained imported motions are `wave_one_hand`, `v_sign`, `good`, `support_person` and
`two_hand_hearts`. Where this document names `wave_two_hands` or `open_window` as test or
hardware examples, read `two_hand_hearts` and `support_person` instead.

## Intent

The user wants `perform_gesture(robot="both", name=...)` to play the physical_ai_agent team's
motions on both G1s:
- both arms, plus the Dex3 hands on pc2_222;
- through OUR token pipeline: encoder, guarded session, envelopes, retreat. Their planner path
  (`upper_body_position`, hand joints) is not wanted.
- Only their motion data comes over, as a one-time import copied into this repo, with no live
  loading from theirs.

## Phase 1: speed cap (measured 2026-10-23)

This was a sim-only override of `keyframe_plan`'s peak speed, with no product change. The full
`native_e2e` suite (13 cases) passed at every speed.

| | 0.47 (today) | 0.6 | 0.75 |
|---|---|---|---|
| Peak measured \|dq\| (watchdog 2.0) | 1.15 | 1.24 | 1.34 |
| Max tilt (limit 3°; point_at(0, 0) 3.5°) | 2.97° | 2.78° | 2.77° |
| Min floor contacts | 7 | 7 | 7 |
| Max leg deviation (envelope 0.15) | 0.145 | 0.143 | 0.142 |
| Wave duration | 46 s | 41 s | 39 s |

**Adopted** (decision 1): `PEAK_SPEED = 0.75`. Watch the first robot runs closely,
because it changes the real robot's commissioned speed. On 2026-10-22, pc2_222 (with Dex3 hands)
tripped the knee envelope during raise_hand even at 0.47.

## Phase 2 goals

- `perform_gesture` gains the seven imported motions under their own names:
  - `wave_one_hand`, `wave_two_hands`, `v_sign`, `good`;
  - `open_window`, `support_person`, `two_hand_hearts`.

  These names go into `GestureName`, which the dual container passes through. Our existing names
  are unchanged. Their aliases are dropped, because `wave` collides.
- Both arms, all 7 joints each, wrists included. Targets can be absolute or start-relative
  (`delta`), as in their files. Their keyframe `t` gaps become minimum segment durations under our
  cap.
- Dex3 hand targets go into the pose stream's 7+7 hand columns, interpolated with the arm. pc2_223
  runs with hands disabled and ignores them.
- The same safety as every arm skill:
  - envelopes on both arms;
  - a path check covering both arms (each arm against the body, and the arms against each other);
  - the smooth retreat, with the hands retreating too;
  - nothing sent on refusal.
  The Dex3 hand geometry stays unchecked (`TODO(dex3)`): the user says that is not critical while
  speed is managed.

## Non-goals

- Their planner path, live loading from their repo, and their aliases.
- Waist motion: none of the seven moves the waist; the loader rejects any that would.
- New motions beyond these seven.
- Checking the hands' collision geometry.

## Anti-goals

- A motion that "plays" only because a limit was loosened. Envelopes, the speed watchdog and the
  path check are unchanged, as in Tier 1: if a motion trips them, it is dropped or retuned, never
  exempted (decision 3).
- Finger commands in the wrong order. Their data names the fingers. Our stream's hand slots use the
  Dex3 command order (motor-slot order, `right_hand_order="native"`). The loader maps names to slots
  explicitly (see "Finger mapping"), and an asymmetric finger check on pc2_222 confirms it before
  any hand motion.

## Decided design (my calls)

### 1. The import

- **Files.** Copy the motion data of the seven `data/*.json` from `physical_ai_agent@dev 3430faa`, with the
  `_doc` fields removed, into `dimos/robot/unitree/g1/gestures/`. Add a `SOURCE.md` giving the repo, commit, path, the
  date, and the authors from `git log`.
- **Loader.** A small loader parses the subset of their schema we use: `t`, `delta`, `absolute`,
  `hands`, `return_s`. Unknown keys are errors.
- **Joint mapping, by name only.**
  - Their upper-body order (`UB_NAMES`) interleaves left and right: left_shoulder_pitch,
    right_shoulder_pitch, left_shoulder_roll, and so on.
  - Our `MOTOR_NAMES` lists the whole left arm before the whole right arm.
  - So the loader maps each joint name to our motor index explicitly; there is no positional
    copy.
  - A permutation test feeds a distinct value for every one of their 17 names and checks that
    each lands on the right motor index.
  - Waist entries must be absent or zero.
- **Their limits.** Their joint limits (URDF), and the Dex3 command limits for each hand, are
  copied in.
  - **At load,** absolute targets and hand targets are checked against these limits.
  - **At preparation:** a start-relative (`delta`) target can only be checked once the start is
    known. A valid start plus a valid delta can still leave the limit: support_person's elbow
    delta of −0.5 from a start at −0.8 gives −1.3, beyond the −1.047 limit. So every keyframe
    target is resolved against the robot's MEASURED start pose in `keyframes_of(base)`, then
    checked against the limits, before anything is sent.
  - **For `robot="both"`,** the dual container already plans and checks both robots before
    starting either, so a limit failure on one robot moves neither.
  - A failure refuses with the joint, the value and the limit.
- **Our tighter bounds.** Our `JOINT_BOUNDS` for the left arm (elbow −30…82° and so on) are tighter
  than theirs. Imported targets are checked against the URDF limits only. The envelopes, the path
  check and the watchdog are the safety, as for gestures today.

### 2. Engine changes

`keyframe_plan` already plans in 29 joints. Three things change.

- **Hands.**
  - `Keyframe` gets an optional `hands: NDArray[14] | None`.
  - `KeyframePlan` gets `hands: NDArray[N, 14]`, interpolated with the same min-jerk profile per
    segment.
  - Segment duration also counts finger displacement against the cap, as their player does.
  - The start and end hand pose is the MEASURED hand state from feedback, not an assumed fist. A
    finger the file leaves unspecified takes their documented default (the deploy fist), only in
    keyframes that specify hands.
  - `_move` sends `plan.hands` per row instead of the constant measured hands.
  - `retreat_poses` returns the matching hand rows, so the hands reverse and return home with the
    arm.
  - **The finger speed cap applies to what is transmitted, not only to what is planned.**
    - On a retreat or any later chunk, the session blends from the last action to the new
      chunk, but it step-limits only the 64 body tokens (`token_step_limit`). The 14 hand values
      are unlimited.
    - Reversing at peak finger velocity therefore roughly doubles the finger speed: in the
      review's reproduction for `wave_two_hands`, a planned 0.749 rad/s peak became 1.417 rad/s.
    - The fix: `SonicModelProfile` gains `hand_step_limit: float | None` (rad per 50 Hz step,
      default None, so existing profiles are unchanged). `SonicTaskSession` clamps each
      transmitted hand value's step to it, exactly as it clamps tokens.
    - The arm blueprint sets it to `PEAK_SPEED / 50` (0.015).
    - Test: stop `wave_two_hands` at the row of peak finger velocity, run the session's send
      path, and assert every transmitted hand step is ≤ 0.015.
  - **Finger mapping, explicit and by name.** Their JSON names fingers (thumb_0…2, index_0…1,
    middle_0…1). The Dex3 command order, which the deploy and our stream's motor slots both use, is
    thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1, on BOTH hands. Their `ORDER`
    table records this, with the right hand confirmed on hardware.
    - The loader maps names to slots with that table.
    - A unit test sends a distinct value per finger through the real packet encoder
      (`encode_token_reference`), decodes the packet, and checks each slot.
- **Segment timing for imported motions.** Duration is
  `max(their dt, 1.875·max|Δ|/PEAK_SPEED, 0.5 s)`, where `max|Δ|` includes the fingers.
  - The 2 s floor (`move_s`) stays for our existing skills only, so they behave exactly as today.
  - Their `return_s` is the final segment's minimum duration.
  - Repeated identical keyframes become holds, which also gives chunk splits stillness.
- **Envelope for imported motions.** Legs and waist 0.15 rad, both arms 0.45 rad, plus each joint's
  largest planned displacement (`pose_margins` generalized to two moving arms).

### 3. Path check for both arms

`ArmCollisionChecker` gains `arms: tuple[str, ...] = ("left",)`.
- The default keeps every existing skill unchanged.
- Imported motions use `("left", "right")`. That checks each arm's links against the torso, pelvis
  and legs, and the left arm's links against the right arm's.
- The hand geoms stay ignored (`TODO(dex3)`).
- The start-pose baseline (`start=base`) works as before.

### 4. Skill surface

- `GESTURES` becomes `dict[str, GestureSpec]`:
  - our five as before (left arm, 4 joints, holds);
  - the seven imported ones from the loader.
- `gesture_keyframes(name, base)` returns `Keyframe`s with both arms and the hands set.
- `perform_gesture` chooses the envelope and the checker arms by spec.
- `GESTURE_MAX_S` rises to 45 s, since `two_hand_hearts` is 20 s plus a 5 s return, plus the
  stretch from our cap.
- The prompt lists the new names in one line each.

## Validation

**Unit tests:**
- **The loader:** all seven load; a bad key, a waist entry, an out-of-limit absolute target or a
  hand value outside the deploy limits is refused.
- **Joint mapping:** the name-based permutation test, with a distinct value per joint.
- **Finger mapping:** the per-slot packet test.
- **Resolved-target limits:** support_person's −0.5 elbow delta from a −0.8 start is refused at
  preparation, and nothing is sent.
- **Hand interpolation:** the hands follow the arm's profile, and a large finger change stretches
  the segment.
- **Hand rows** are present in the retreat units.
- **The hand step limit** holds on every transmitted command, including a retreat at peak finger
  velocity.
- **The both-arm checker:** an arm-against-arm contact is detected, and the default `("left",)`
  behaves as before.
- **The hands are actually commanded** (MuJoCo has no hands): for each imported motion with hands,
  the hand columns `_stream` submits equal `plan.hands` at sampled rows, and stay within the step
  limit.
- All existing tests stay green.

**MuJoCo e2e** (`native_e2e`; the stand-in has no hands). Each of the seven must show that the
gesture was performed, not just that the robot stayed safe.
- **Tracking.** At each keyframe, take the stand-in's logged joint angles over a window: from the
  planned arrival to the planned departure (both offset by +0.3 s), or ±0.3 s around the arrival
  for a keyframe without a hold. In that window, every joint the motion commands (both arms, wrists
  included) must come within 0.25 rad of its target, and a scalar reach is not enough. The
  tolerance allows for SONIC's elbow sag, which measured up to 0.18 rad in the forearm runs. Print
  the worst joint and its error for each keyframe.
- **Stability:** tilt ≤ 3°, floor contacts ≥ 7, |dq| ≤ 2.0, handback, and both arms back within
  0.05 rad.
- **Stop:** a `stop_arm` mid-`wave_two_hands` retreats both arms.
- **Failures:** a motion that fails tracking, an envelope, or the path check is dropped from the
  import, with the finding (decision 3).

**Hardware** (operator, through the dual session):
1. pc2_223 (no hands): `wave_two_hands` and `open_window`, to check the arms.
2. pc2_222, the finger-order check first. This is an operator-only RPC, `check_dex3_finger_order`
   (`@rpc`, not a skill, so the agent cannot call it). It holds an ASYMMETRIC hand pose for 3 s,
   with the arms at rest:
   - left hand: index open, middle closed, thumb_2 half;
   - right hand: middle open, index closed, thumb_0 rotated.

   The operator confirms by eye that each named finger did what was commanded. If index and middle
   are swapped on either hand, stop: no hand motion runs until the mapping is fixed.
3. pc2_222: the remaining hand motions, at the adopted cap.

## Decisions (user, 2026-10-23)

1. **Speed.** `PEAK_SPEED = 0.75` rad/s for ALL arm skills. The dual-G1 two-stand-in run at 0.75
   also passed all six of its checks: peak |dq| 1.25, tilt ≤ 1.73°.
2. **Data.** Strip the free-text fields (`_doc`) and keep only the motion data (`t`, `delta`,
   `absolute`, `hands`, `return_s`), plus a `SOURCE.md` giving the repo, commit, path, date and
   authors. The files are therefore re-serialized, not copied byte for byte (§1 is amended
   accordingly).
3. **Failing motions.** A motion that fails our checks is dropped from the import, and the finding
   is reported. Nothing is loosened or exempted.

## Left to the builder

- Loader structure.
- How `GestureSpec` represents our five versus the imported seven.
- How the hand rows travel through `_move` and the retreat units.
- The test fixtures.
