# G1 Waist Gestures Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** invite turns the waist about 20° to each side with both hands raised, and thanks bows about 20° forward. Both run through the existing guarded SONIC gesture path, and the legs get a wider (measured, ≤ 0.25 rad) envelope during these two gestures only.

**Architecture:**
- The gesture loader allows waist yaw for invite and waist pitch for thanks only.
- `GestureSpec` gains a per-gesture leg margin, applied by `gesture_margins` to every leg joint except hip yaw.
- The gesture JSONs carry the waist keyframes. The MuJoCo stand-in logs pelvis x/y/heading, so the e2e can assert no drift.
- A step-0 MuJoCo measurement picks the waist command (0.349 or up to 0.40 rad) and the leg margins before any production change.

**Tech Stack:** Python 3.10–3.12, numpy, MuJoCo (`sonic_sim_peer`), pytest.

**Spec:** `docs/superpowers/specs/2026-10-26-g1-waist-gestures-design.md`

## Global Constraints

**Task 1 outcome (user decisions, 2026-10-26). These values override anything below that conflicts:**
- Waist commands, measured in MuJoCo:
  - invite `waist_yaw` ±0.40 rad (+ = left), real turn about 8.7° each way;
  - thanks `waist_pitch` +0.40 rad absolute (+ = forward), real 21.2° absolute (+12.4° beyond the 8.8° stance lean).
- Leg margins:
  - invite 0.23 rad on ALL 12 leg joints, hip yaw included (user: yes);
  - thanks 0.23 rad on the leg joints except hip yaw, which keeps 0.15.
- thanks uses the raised-hands pose (user: yes), with the wrists unchanged from today's start pose:

  | joint | left | right |
  |---|---|---|
  | shoulder_pitch | −1.05 | −0.75 |
  | shoulder_roll | 0.50 | −0.50 |
  | shoulder_yaw | −0.70 | 0.70 |
  | elbow | 0.20 | 0.20 |

  Keep today's hands-down first keyframe; the bow happens on arrival at the raised pose.
- **Drift = the FEET, not the pelvis** (user: yes):
  - the stand-in also logs both feet (ankle_roll_link) planar position and yaw;
  - the e2e asserts that each foot moves < 0.01 m and turns < 2° over the whole window;
  - **Task 5 outcome (user, 2026-10-27):** the stand-in's feet slide in every gesture (6.9–16.7 mm, 2.8–9.4°, waist or not), so the feet are printed only and checked on the robots. On invite's two turn keyframes the arm tracking is printed, not asserted (option A2; the waist drags the shoulder yaw and pitch up to 0.36 rad).
  - pelvis drift and heading are reported, not asserted. Today's unmodified gestures already sway the pelvis 2.6–3.6 cm and 4–7°.
- **Contacts:** the hand links are excluded from the torso side of `self_contacts` (user: yes), as in the arm path check. Baseline = every pair seen in the pre-arming stance.

- Waist motion only in `invite` (`waist_yaw`) and `thanks` (`waist_pitch`). `waist_roll` stays refused everywhere. Every other gesture still refuses any waist entry other than a zero delta, with the existing message.
- Waist targets are absolute, inside `UB_LIMITS`, and |target| ≤ 0.40 rad (`WAIST_CAP_RAD = 0.40`).
- The waist command is 0.349 rad, or the smallest of {0.349, 0.40} whose real angle reaches about 20° (0.349 rad) in MuJoCo, otherwise 0.40.
- Leg margin for invite and thanks = the measured MuJoCo peak leg deviation + 0.05 rad, rounded up to 0.01, never above 0.25 rad (`LEG_MARGIN_CEILING = 0.25`).
  - It applies to leg joints 0, 1, 3, 4, 5, 6, 7, 9, 10, 11.
  - Hip yaw (2, 8) keeps `body_margin` (0.15).
  - If peak + 0.05 > 0.25 (or hip yaw > 0.15, or drift or contacts fail), STOP AND REPORT to the user; never raise the ceiling. Any reduced-command experiment happens only after the user decides. This overrides the spec's "reduce the waist angle" anti-goal wording.
- Every other gesture and skill keeps today's envelope exactly.
- Speed cap (0.75 rad/s), retreat, `stop_arm`, voice timing and the path check are unchanged in behaviour.
- No drift: in the MuJoCo e2e of invite and thanks, measured from the pelvis pose at the window start, over the whole window (motion, retreat and settling up to handback + 0.5 s):
  - the MAXIMUM planar displacement is < 0.03 m;
  - the MAXIMUM wrapped heading difference (in (-180°, 180°]) is < 3°.
  - The final value alone is not enough.
- Return: on normal completion AND after a stop, the arms and all three waist joints are back within `ARM_RETURN_RAD` (0.05) of the measured start pose.
- Torso clearance: the arm checker skips body-to-body contacts (`sonic_arm_reference.py`, `_near`), so it cannot validate the bow. The stand-in logs MuJoCo contacts between torso-side bodies (torso_link and above the waist joints, arms included) and pelvis/leg bodies.
  - A contact pair present at the window start is baseline.
  - Any new pair during the motion fails.
- Waist tracking is reported, not asserted. Every other commanded joint keeps `TRACK_RAD` (0.25).
- MuJoCo use:
  - ask dimos-dev-1a first (SendMessage `uds:/run/user/1000/cc-socks/182277.sock`, name dimos-dev-1a);
  - pin to cores 8–15 (`taskset -c 8-15`);
  - say when it's free;
  - batch the runs.
- Run e2e with: `GEAR_SONIC_MODEL_DIR=/home/jihun/.cache/huggingface/hub/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2 GEAR_SONIC_SCENE=/home/jihun/work/GR00T-WholeBodyControl-hand-tracking-watchdog/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml GEAR_SONIC_ENCODER=/home/jihun/work/GR00T-WholeBodyControl/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx taskset -c 8-15 .venv/bin/python -m pytest -m native_e2e dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py -s -p no:cacheprovider`
- No commits or pushes by subagents. No `git stash` or `git checkout`. No robots. Python 3.10-compatible code (no 3.11-only APIs).
- Local checks per task:
  - `GEAR_SONIC_ENCODER=… .venv/bin/python -m pytest dimos/robot/unitree/g1 dimos/robot/test_all_blueprints_generation.py -q -p no:cacheprovider -n 8`;
  - ruff check, ruff format --check, and mypy on touched non-test files.

## Review Focus

1. A gesture JSON other than invite/thanks that sets a waist joint must still be refused at load, even a hand-authored one added later. Owned by Task 2.
2. `waist_roll` on invite or thanks must be refused. Owned by Task 2.
3. The wider leg margin must never reach hip yaw, or any gesture or skill other than invite/thanks. Owned by Task 3.
4. A `stop_arm` while the waist is turned or bowed must bring the waist back (retreat), not leave it bent. Owned by Task 5.
5. A waist command over 0.40 rad, e.g. a typo of 4.0, must be refused before anything is sent. Owned by Task 2.

---

### Task 1: Step 0: measure waist tracking, leg peaks and drift in MuJoCo

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_sim_peer.py`. The run log gains:
  - `pelvis_xy` (x, y, world) and `heading_deg` (pelvis yaw in world, from `xmat`);
  - `torso_rel_deg`: signed (yaw, pitch, roll) of `torso_link` relative to the pelvis, from R = R_pelvisᵀ·R_torso, intrinsic z-y-x. Positive pitch means the torso's forward axis tips down (a bow); positive yaw means a turn to the robot's left. This establishes the waist sign convention independently of the joint;
  - `self_contacts`: the torso-side × pelvis/leg contact pairs in `d.contact`. They are stored so that the existing blanket `np.asarray(...)` .npz conversion keeps them readable without pickle: a fixed-width numeric array (N × K of body-id pairs, -1 padded) with a separate name table, or one JSON string per sample. Whichever is chosen, `np.load(..., allow_pickle=False)` must read it back.

  All are recorded next to `pelvis_z`/`tilt`, and every existing key is unchanged.
- Create (scratch, not committed): `/tmp/claude-1000/-home-jihun-work-dimos-dev/1a444f13-d052-462f-be63-10716b21bc96/scratchpad/waist_measure/test_waist_measure.py`, a throwaway native_e2e test reusing the `stand_in`/`_run` harness from `test_sonic_arm_sim_e2e.py`.

**Interfaces:**
- Produces: log keys `pelvis_xy` (N×2 float), `heading_deg` (N float), `torso_rel_deg` (N×3 float) and `self_contacts` (N lists of pairs) in the stand-in log that `test_sonic_arm_sim_e2e.py` reads.
- Produces: a table in `task-1-report.md` with, per gesture and command (0.349, 0.40):
  - the real peak waist angle (rad, deg);
  - the peak deviation of each leg joint (hip yaw separately);
  - the max planar pelvis drift (m) and the max wrapped heading change (°), over the whole window;
  - the real torso yaw/pitch relative to the pelvis (signed, from `torso_rel_deg`);
  - any torso–pelvis/leg contact pair that is not in the baseline;
  - tilt and minimum contacts.

- [ ] **Step 1:** Add the two log keys to `sonic_sim_peer.py`. Keep every existing key unchanged.
- [ ] **Step 2:** Write the scratch test. It `mocker.patch.dict`s `sonic_arm_skill.GESTURES` with invite/thanks variants whose `keyframes_of` returns today's keyframes plus waist targets:
  - invite: `waist_yaw` +c, −c, 0 on the hands-raised keyframes;
  - thanks: `waist_pitch` +c held 1.5 s, then 0;
  - for c in (0.349, 0.40) and both signs of pitch and yaw. The forward and left signs come from `torso_rel_deg`, not from the unsigned `tilt`.
  - Patch the leg envelope wide (0.35) for the measurement only.
- [ ] **Step 3:** Ask dimos-dev-1a for MuJoCo. Run the scratch test pinned to cores 8–15. Release MuJoCo.
- [ ] **Step 4:** Report:
  - the forward-pitch sign and the left-yaw sign;
  - the chosen command per gesture (per Global Constraints);
  - leg margin = peak (non-hip-yaw legs) + 0.05, rounded up to 0.01, ≤ 0.25;
  - hip-yaw peaks;
  - the drift numbers.

  If any leg margin would exceed 0.25, or hip yaw exceeds 0.15, or drift exceeds 3 cm / 3°, report it as BLOCKED with the numbers. The controller decides.

### Task 2: Loader allows waist yaw for invite and waist pitch for thanks

**Files:**
- Modify: `dimos/robot/unitree/g1/imported_gestures.py` (`_WAIST` check in `_joints`, `parse_gesture`).
- Test: `dimos/robot/unitree/g1/test_imported_gestures.py`

**Interfaces:**
- Produces:
  - `WAIST_GESTURES: dict[str, tuple[str, ...]] = {"invite": ("waist_yaw",), "thanks": ("waist_pitch",)}`;
  - `WAIST_CAP_RAD = 0.40`;
  - `_joints(raw, absolute, where, waist: tuple[str, ...] = ())`. A waist entry that is a zero delta stays allowed for every gesture, as today; an existing test requires it. Any other waist entry is refused unless its name is in `waist`, it is absolute, and |value| ≤ `WAIST_CAP_RAD`;
  - `parse_gesture` passes `WAIST_GESTURES.get(name, ())`.

- [ ] **Step 1:** Write the failing tests:
  - `test_invite_may_set_waist_yaw`: a parsed invite with `absolute: {"waist_yaw": 0.349}` resolves with q[12] == 0.349.
  - `test_thanks_may_set_waist_pitch`: the same for q[14].
  - `test_waist_roll_is_refused_even_for_invite` (Review Focus 2).
  - `test_other_gestures_still_refuse_waist`, parametrized over the names in `RETAINED + AUTHORED` other than invite/thanks; `pytest.raises(ValueError, match="moves the waist")` (Review Focus 1).
  - `test_waist_over_the_cap_is_refused`: invite with waist_yaw 0.41, and with 4.0 (Review Focus 5).
  - `test_waist_delta_is_refused_for_invite`: a non-zero waist delta is refused; waist targets must be absolute.
  - `test_zero_waist_delta_is_still_allowed`: a `delta: {"waist_yaw": 0.0}` parses for any gesture, as today.
- [ ] **Step 2:** Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_imported_gestures.py -q -p no:cacheprovider`. Expected: the new tests FAIL.
- [ ] **Step 3:** Implement as in Interfaces. The refusal message for disallowed gestures stays "moves the waist; waist entries must be zero deltas".
- [ ] **Step 4:** Run the tests again (PASS), then the local checks.

### Task 3: Per-gesture leg margin (invite and thanks only)

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py` (`GestureSpec`, `_gesture_table`, `gesture_margins`, `gesture_plan_inputs`, the `perform_gesture` prepare at the `gesture_margins(...)` call).
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Interfaces:**
- Consumes: the Task 1 leg margins (two floats).
- Produces:
  - `GestureSpec.leg_margin: float | None = None`;
  - `GESTURE_LEG_MARGINS: dict[str, tuple[float, tuple[int, ...]]] = {"invite": (0.23, ALL_LEG_JOINTS), "thanks": (0.23, WIDE_LEG_JOINTS)}`, with `ALL_LEG_JOINTS = tuple(range(12))`. `GestureSpec` also gains `leg_joints: tuple[int, ...] = WIDE_LEG_JOINTS`, and `gesture_margins(..., leg_margin=None, leg_joints=WIDE_LEG_JOINTS)` widens exactly `leg_joints`;
  - `LEG_MARGIN_CEILING = 0.25`;
  - `WIDE_LEG_JOINTS = (0, 1, 3, 4, 5, 6, 7, 9, 10, 11)`;
  - `gesture_margins(envelope, body, pose_arm, other_arm, leg_margin: float | None = None)` sets `WIDE_LEG_JOINTS` to `leg_margin` when given;
  - `gesture_plan_inputs` is unchanged (still a 5-tuple: the dual container calls it). A new portable `gesture_leg_margin(name: str) -> float | None` returns `GESTURES[name].leg_margin` and raises ValueError for an unknown name.

- [ ] **Step 1:** Write the failing tests:
  - `test_only_invite_and_thanks_widen_the_legs`: for every `GESTURES` name, the leg_margin is None except invite/thanks, which are ≤ 0.25 (Review Focus 3).
  - `test_wide_leg_margin_skips_hip_yaw`: `gesture_margins("two_arm", 0.15, 0.45, 0.35, leg_margin=0.23)` gives [2] == [8] == 0.15 and the WIDE_LEG_JOINTS == 0.23.
  - `test_invite_widens_hip_yaw_too`: invite's envelope has all 12 leg joints at 0.23; thanks's has hip yaw at 0.15.
  - `test_waist_envelope_includes_the_planned_waist`: perform_gesture("thanks"), through the `skill` fixture with the session spied, uses an `allowed` whose waist_pitch entry ≥ the planned |Δ| + body_margin.
  - `test_leg_margin_ceiling_is_enforced`: a GestureSpec with leg_margin 0.26 is refused by `_gesture_table` validation.
- [ ] **Step 2:** Run them. Expected: FAIL.
- [ ] **Step 3:** Implement as in Interfaces. Pass `spec.leg_margin` into `gesture_margins` in `perform_gesture`. Add `test_gesture_leg_margin_is_portable`: `gesture_leg_margin("invite") == GESTURE_LEG_MARGINS["invite"]`, `gesture_leg_margin("wave") is None`, and an unknown name raises.
- [ ] **Step 4:** PASS, then the local checks.

### Task 4: invite and thanks gain their waist keyframes

**Files:**
- Modify: `dimos/robot/unitree/g1/gestures/invite.json`, `dimos/robot/unitree/g1/gestures/thanks.json`, `dimos/robot/unitree/g1/gestures/SOURCE.md` (the "Hand-authored" section).
- Test: `dimos/robot/unitree/g1/test_sonic_arm_keyframes.py`, `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Interfaces:**
- Consumes: Task 1's commands and signs; Task 2's loader; Task 3's margins.

- [ ] **Step 1:** Write the failing tests:
  - `test_invite_turns_the_waist_both_ways`: invite's resolved keyframes reach waist_yaw +c and −c (Task 1's c and sign) and end at the start.
  - `test_thanks_bows_forward`: thanks reaches waist_pitch at the forward sign × c and ends at the start.
  - `test_waist_gestures_pass_the_path_check` (needs_scene): both plans are clear with `ArmCollisionChecker(SCENE, arms=("left","right"))` from `ROBOT_IDLE`.
- [ ] **Step 2:** Run them. Expected: FAIL.
- [ ] **Step 3:** Edit the JSONs:
  - invite: waist_yaw on the hands-raised keyframes, +c, −c, then 0;
  - thanks: waist_pitch forward c with a hold of about 1.5 s, then 0.

  Keep the arm targets and timings. Retime only so each waist segment respects the 0.75 rad/s cap (the planner stretches automatically). Record in SOURCE.md: the sign convention, the commands, Task 1's real angles, and the leg margins.
- [ ] **Step 4:** PASS. Run `test_each_gesture_plans_within_the_limits` (≤ `GESTURE_MAX_S`, peak speed), then the local checks.

### Task 5: MuJoCo e2e: no drift, waist reported, stop mid-turn and mid-bow

**Files:**
- Modify: `dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py`

**Interfaces:**
- Consumes: Task 1's log keys; Tasks 2–4.
- Produces:
  - `MAX_DRIFT_M = 0.03`, `MAX_TURN_DEG = 3.0`, `WAIST = (12, 13, 14)`;
  - the tracking helper skips WAIST joints when asserting and prints their real angle.

- [ ] **Step 1:** In `test_imported_gesture_tracks_and_hands_back`:
  - for invite and thanks, assert drift < `MAX_DRIFT_M` and heading change < `MAX_TURN_DEG` from the window start;
  - print drift for every case;
  - exclude WAIST from the TRACK assertion but print the waist's peak real angle;
  - keep the existing arm-return assertion for EVERY gesture, unchanged. Add the WAIST return (every waist joint within `ARM_RETURN_RAD` of the measured start): asserted for invite and thanks, print-only for the others;
  - assert no new torso–pelvis/leg contact pair against the window-start baseline.
- [ ] **Step 2:** Add `test_stop_mid_waist_retreats_home`, parametrized over ("invite", mid-turn) and ("thanks", mid-bow).
  - It does NOT use the harness's 60%-of-plan stop timing, which can land after the waist has returned.
  - The trigger uses LIVE execution feedback (`get_feedback().body_q`). The simulator writes its .npz log only when it exits, so the log can't drive the stop. The saved log then verifies afterwards that the stop happened inside the waist segment with the waist displaced.
  - Instead it rebuilds the plan as `_move` does to find the waist segment's rows. It waits, with a timeout, until (a) the plan clock is inside that segment and (b) the logged waist joint is displaced ≥ 50% of its real peak from Task 1, in the commanded direction. Only then does it call `stop_arm`.
  - Assert "stopped early: stop requested", a retreat chunk, handback, and arms and WAIST back within `ARM_RETURN_RAD` of the measured start.
  - Apply the drift and torso-contact checks over the motion, the retreat and the settling to handback + 0.5 s (Review Focus 4).
- [ ] **Step 3:** Ask dimos-dev-1a for MuJoCo. Run the whole e2e module once with the Global Constraints command. Release MuJoCo.
- [ ] **Step 4:** Expected: all pass. If a waist gesture fails tracking, envelope, drift or stability, report BLOCKED with the numbers. Never loosen a threshold; the controller decides, per the spec's anti-goal.
- [ ] **Step 5:** Run the local checks.
