# G1 waist motion for the invite and thanks gestures: design

Date: 2026-10-26. Repository: caisarl76/dimos (fork), base `main` 26010a9bb. Builds on the SONIC arm
skill (`dimos/robot/unitree/g1/sonic_arm_skill.py`) and the hand-authored gestures in
`dimos/robot/unitree/g1/gestures/` (loader `imported_gestures.py`).

## Intent

Two demo gestures should use the waist, so the robot reaches a wider audience without moving its
feet:

- **invite**: the robot raises both hands in the existing palms-forward greeting, then turns at
  the waist about 20° to one side and about 20° to the other, so it faces people across a wider
  arc. The user wants a **waist** rotation, not a turn on the legs, because turning on the legs
  could move the robot from where it stands.
- **thanks**: the robot bows forward at the waist by about 20°, holds briefly, and comes back
  upright.

Both robots (pc2_222 and pc2_223) can yaw and pitch the waist (user-confirmed).

## Goals

- invite and thanks gain waist yaw (invite) and waist pitch (thanks) targets aiming at about 20°
  of real motion. The command is 0.349 rad, or more up to 0.40 rad if SONIC follows the waist only
  partly (see "Known risk"). Both return to the start pose like every gesture.
- The motions run through the existing guarded SONIC path: keyframe plan, path check, envelope
  watch, speed cap, `stop_arm` retreat, and voice-line timing, all unchanged in behaviour.
- During these two gestures only, the leg joints may move further than today's 0.15 rad
  envelope. The new limit is sized from MuJoCo measurement, never above 0.25 rad (user's
  ceiling).

## Non-goals

- Waist motion for any other gesture, or as a general skill or agent capability ("just those two
  gestures for now").
- Waist roll.
- Hand collision checking (dropped: the gestures run cleanly on both robots; `TODO(dex3)` stays).
- The dual-robot container: the dimos-dev-1a session ports it after this lands, as with earlier
  phases.

## Anti-goals

- The robot steps or drifts on its feet during invite or thanks.
- Any other gesture, or any other skill, gets waist motion or a wider leg envelope.
- A leg envelope above 0.25 rad.
- Loosening a threshold to make a failing run pass. If the leg peak plus margin exceeds 0.25 rad,
  stop and report to the user (a reduced waist angle is tried only after their decision). Never
  raise the ceiling.

## Task 1 outcome (user decisions, 2026-10-26)

- **invite:** waist_yaw ±0.40 rad gives about 8.7° of real turn each way (SONIC follows 26–38%; accepted). Leg margin 0.23 on all leg joints, hip yaw included.
- **thanks:** the hands-together pose is raised to lower-chest height (left/right shoulder pitch −1.05/−0.75, roll ±0.50, yaw ∓0.70, elbow 0.20), because the bow with today's pose brought the hands to the thighs and the path check refused it.
  - waist_pitch +0.40 absolute gives 21.2° (+12.4° beyond the 8.8° lean the robot already has at rest).
  - Leg margin 0.23, with hip yaw kept at 0.15.
- **"Don't change the robot's position" is checked on the feet:** each foot moves < 1 cm and turns < 2°. The pelvis sways 2.6–3.6 cm and 4–7° over planted feet even in today's unmodified gestures, so pelvis drift is reported only.
- **Torso-vs-legs contacts exclude the hand links,** matching the arm path check.

## Task 5 outcome (user decisions, 2026-10-27)

- **Feet are checked on the robots.** In the MuJoCo stand-in every gesture, waist or not, slides the feet 6.9–16.7 mm and turns them 2.8–9.4°, so the < 1 cm / < 2° check can't judge the waist. The e2e prints the feet; the operator marks each foot before running invite and thanks on the robots.
- **Robot runs start on pc2_222.** dimos-dev-1a's dual run of 2026-10-27 at b69fe01f2, with no waist motion and the robots on stands, logged leg max deviation (`max_deviation_rad.legs`, 222 / 223):
  - invite 0.126 / 0.117;
  - thanks 0.139 / 0.100;
  - wave 0.087 / 0.062.

  222 runs 0.01–0.04 rad above 223 and sits into its left leg, so the bow and the turn may come close to the 0.23 margin there. Run invite and thanks on 222 first, and log `max_deviation_rad.legs` and the worst leg joint.
- **invite: arm tracking during the turn accepted (option A2).** The waist turn drags the shoulders off target (yaw up to 0.360, pitch up to 0.320 rad, against 0.25; option A, shoulder yaw only, left the pitch failing). On invite's two turn keyframes the arm tracking is printed, not asserted. invite's raise and return to centre, and every other gesture, keep the 0.25 rad check. The arm pose during the turn is judged on the robots.

## Known risk: SONIC may follow the waist only partly

When `move_upper_body` was built, SONIC followed only 18-48% of an isolated waist yaw command in
MuJoCo, and waist turns were left out (`sonic_arm_skill.py`, the MOTIONS comment; commit
0c5fbaf82). Waist pitch was never measured.

**User's decision:** command more, up to the 0.40 rad cap, and accept whatever real angle that
gives. So:

- **Step 0 of the plan, before any other work:** a MuJoCo measurement of real waist yaw and pitch
  for commands of 0.349 rad and 0.40 rad, in the gesture's own arm pose. Use the smallest command
  whose real angle reaches about 20°; otherwise use 0.40 rad. Record the commands and real angles
  in `gestures/SOURCE.md`.
- **Waist tracking is recorded, not asserted.** The waist's real angle is reported per case, and
  the per-joint 0.25 rad tracking check covers every other commanded joint. This follows from the
  user accepting a partial real angle; it is not a loosened threshold for any other joint.

## How it works (decided)

1. **Loader** (`imported_gestures.py`). Today any waist entry other than a zero delta is refused.
   - A small per-gesture table allows the waist: invite may set `waist_yaw`; thanks may set
     `waist_pitch`. Every other gesture, and `waist_roll` everywhere, is still refused with the
     existing message.
   - Allowed waist targets are absolute and are checked against `UB_LIMITS` like any set joint,
     plus a waist cap of |target| ≤ 0.40 rad (my call: about 23°, a guard against typos).
2. **Gesture data.** Hand-authored JSON keyframes, in the existing format.
   - invite: keep the raise and its palms-forward wave. Add waist_yaw keyframes, +0.349 then
     −0.349 then 0, while the hands stay raised. The robot comes back to the start on the return.
   - thanks: keep the hands-together pose. Add waist_pitch +0.349 (forward), with a short hold,
     then upright.
   - The sign convention (which `waist_pitch` sign is forward, which `waist_yaw` sign is left) is
     verified in MuJoCo and recorded in `gestures/SOURCE.md`'s "Hand-authored" section, with the
     tuning notes.
3. **Envelope.** The gesture envelope already adds each joint's planned displacement to its
   margin (`gesture_margins` plus the planned displacement), so waist joints get "planned +
   body margin" without new code. Verify this with a test.
   - Legs: a per-gesture leg margin (a new `GestureSpec` field, default `None` = today's
     `body_margin` of 0.15) for invite and thanks only.
   - It applies to the leg pitch, roll, knee and ankle joints. **Hip yaw keeps 0.15 rad** in
     both gestures, so the turn comes from the waist and not from twisting at the hips (my call,
     from the user's "don't change the robot's position").
   - Value: the measured MuJoCo peak leg deviation for that gesture plus 0.05 rad, rounded up to
     0.01, capped at 0.25. If the cap binds, stop and report (anti-goal).
4. **Path check.** `ArmCollisionChecker` already sets all 29 joints, waist included, on the
   MuJoCo model. So the arms are checked against the turned or bent torso as planned. Verify with
   a test that the planned path for both gestures is clear.
   - It skips body-to-body contacts, so it cannot check the bow's torso clearance. Torso against
     pelvis/legs is checked in MuJoCo instead: the stand-in logs those contact pairs, and any pair
     not present at the start fails (plan, "Torso clearance").
5. **Speed, timing, retreat and voice: unchanged.**
   - The 0.75 rad/s peak speed applies to the waist joints, so a 20° move takes at least about
     0.9 s; the segment stretches automatically.
   - The retreat path (`retreat_poses`) already covers all 29 joints, so `stop_arm` mid-turn or
     mid-bow returns the waist smoothly.
   - Voice lines keep starting as the arm starts moving.

## Validation

- **Unit:**
  - the loader allows waist_yaw for invite and waist_pitch for thanks;
  - it still refuses waist for every other gesture and waist_roll for these two;
  - the cap is enforced;
  - the envelope includes the planned waist displacement;
  - the per-gesture leg margin applies to invite and thanks only, and is ≤ 0.25;
  - GestureName and the gesture-set test still agree.
- **MuJoCo e2e** (`test_sonic_arm_sim_e2e.py`, sonic_sim_peer stand-in):
  - **Coordination:** ask dimos-dev-1a before using MuJoCo, pin the simulator to cores 8-15, and
    say when it's free. Batch the sim runs to keep the number of sessions small.
  1. Step 0 (above), then measure the leg peak for each gesture and set the margin from it.
  2. Then the standard per-case checks (outcome "returned", tilt ≤ 3°, contacts ≥ 7, |dq| ≤ 2.0,
     handback, arms and waist back within 0.05 rad).
  3. Per-joint tracking as for the imported gestures: every commanded joint except the waist within
     0.25 rad in its window. The waist's real angle is reported.
  4. **No drift (my call: the check for the main anti-goal).** The stand-in's log gains pelvis x/y
     and heading. Each case asserts the pelvis moved less than 3 cm and turned less than 3° from
     its start. The stand-in is shared, so tell dimos-dev-1a about the log change.
  5. A `stop_arm` mid-turn (invite) and mid-bow (thanks) retreats home.
- **Robots:** operator runs on pc2_223 and pc2_222 after review. These are the user's actions.

## Left to the builder

- Exact keyframe timings and hold lengths, within the speed cap and the existing gesture style.
- Where the per-gesture waist table and leg margin live, as long as the dual container can reuse
  them through the existing portable functions (`gesture_plan_inputs`).
- Test names and structure.
