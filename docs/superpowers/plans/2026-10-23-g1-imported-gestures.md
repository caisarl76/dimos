# G1 Imported Gestures (Phase 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Play the physical_ai_agent team's seven motions through our token pipeline. That means
both arms with all 7 joints each, Dex3 hands with a speed cap on what is transmitted, a both-arm
path check, name-based mappings, and per-joint tracking validation.

**Architecture:**
- A session-level hand step limit, kept in its own change so other branches can merge it.
- Hand rows carried through the keyframe engine and its retreats.
- A both-arm collision checker.
- A loader for the copied motion data.
- `perform_gesture` driven by gesture specs.
- An operator-only finger-order RPC that runs through the same session.
- MuJoCo e2e with per-joint tracking windows.

**Tech Stack:** Python 3.12, numpy, MuJoCo, DimOS skills, pytest, `sonic_sim_peer`.

**Spec:** `docs/superpowers/specs/2026-10-23-g1-imported-gestures-design.md`, approved 2026-10-23.
It builds on local commit `c70216be0` (Phase 1: `PEAK_SPEED` 0.75), plus the PR #1 review fixes, which land first.

## Global Constraints

- **Source.** `/home/jihun/work/physical_ai_agent`, pinned commit `3430faa0059b656f6dba665fbdeeed7ed3899f45`. Read every file with
  `git show 3430faa0059b656f6dba665fbdeeed7ed3899f45:<path>`. Never read from the `dev` branch, which moves, and never touch their
  working tree.
  - Motions: `wave_one_hand`, `wave_two_hands`, `v_sign`, `good`, `open_window`,
    `support_person`, `two_hand_hearts`.
  - Copy the motion data only: `t`, `delta`, `absolute`, `hands`, `return_s`. `_doc` and `aliases`
    are stripped.
  - `SOURCE.md` gives the repo, the full commit `3430faa0059b656f6dba665fbdeeed7ed3899f45`, the path, the date (2026-10-23), the
    authors from `git log`, and
    any motions dropped, with the reason.
- **Joint mapping is by name.** Their `UB_NAMES` interleaves left and right; our `MOTOR_NAMES`
  groups the left arm, then the right. The waist must be absent or zero.
- **Finger mapping is by name**, into the Dex3 command order. Both hands use thumb_0, thumb_1,
  thumb_2, middle_0, middle_1, index_0, index_1. Our stream's slots are in the same order
  (`right_hand_order="native"`).
- **Limits are copied from the source.**
  - URDF joint limits (`upper_body.LIMITS`);
  - Dex3 command limits per hand (`hands.LIMITS`);
  - the deploy fist (`hands.DEFAULT`): unspecified fingers in a keyframe that has hands.
  - Absolute and hand targets are checked at load. Every target is checked again after it is
    resolved against the measured base, before anything is sent.
- **Hand step limit.** `SonicModelProfile.hand_step_limit: float | None = None` (rad per step).
  - The session clamps each transmitted hand value's change per step to it.
  - Single-robot `SONIC_ARM_PROFILE` sets it to `PEAK_SPEED / RATE` = 0.015.
  - The dual profiles must set the same (the dual session; see Task 7).
- **Timing for imported motions.** Segment duration = `max(their dt, 1.875·max|Δ|/PEAK_SPEED,
  0.5 s)`, where `max|Δ|` includes the fingers. The final segment's minimum is `return_s`. Our five
  gestures and the existing skills keep the 2 s floor.
- **Envelope for imported motions.** Legs and waist 0.15 rad, both arms 0.45 rad, plus each
  joint's largest planned displacement.
- **Path check.** Imported motions use `ArmCollisionChecker(arms=("left", "right"))`: each arm
  against the body, and left against right. The default is `("left",)`, unchanged. The hand geoms
  stay ignored (`TODO(dex3)`).
- **`GESTURE_MAX_S`** = 45.
- **Tracking acceptance** (e2e). Every commanded arm joint must come within 0.25 rad of its target:
  - in the window from planned arrival + 0.3 s to planned departure + 0.3 s;
  - or ±0.3 s around arrival, for a keyframe with no hold.
- **Policy.** A motion that fails any check is dropped from the import and reported. Nothing is
  loosened or exempted.
- **Process.**
  - No commits except where the user asks; the controller commits.
  - Never use `git stash`.
  - MuJoCo turns are coordinated with dimos-dev-1a.
  - mypy strict, ruff clean, imports at the top.

## Review Focus

1. **A stop at peak finger velocity.** The transmitted hand step must stay ≤ 0.015 through the
   session's chunk blend, not only in the plan. Owned by Task 1, exercised again in Task 5.
2. **A resolved delta target outside a limit** (support_person's elbow −0.5 from a −0.8 start) is
   refused before anything is sent. Task 5.
3. **A finger index/middle swap** is caught by the packet test, then by the asymmetric hardware
   check. Tasks 4 and 5.
4. **Arms that don't follow while everything stays stable:** per-joint tracking windows must
   catch it. Task 6.
5. **Hands on a robot without hands** (pc2_223, `--disable-dex3-hands`). Hand columns are still
   sent and ignored there; nothing may assume the hand feedback is meaningful beyond the start and
   end values. Tasks 2 and 5.

---

## File Structure

| File | Change |
|---|---|
| `dimos/robot/unitree/g1/sonic_session.py` | `hand_step_limit` field and clamp (Task 1; a separate commit) |
| `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sonic_arm.py` | `SONIC_ARM_PROFILE.hand_step_limit = PEAK_SPEED / RATE` (Task 1) |
| `dimos/robot/unitree/g1/sonic_arm_reference.py` | `Keyframe.hands`, `KeyframePlan.hands`, timing options, `retreat_hands`, `ArmCollisionChecker(arms=…)`, `two_arm_margins` (Tasks 2 and 3) |
| `dimos/robot/unitree/g1/gestures/` (new) | 7 JSON files and `SOURCE.md`; the loader and limit tables moved to `imported_gestures.py` beside it, because `dimos/` allows no `__init__.py` (Task 4) |
| `dimos/robot/unitree/g1/sonic_arm_skill.py` | `GestureSpec`, the per-row hands in `_move`, `perform_gesture`, `check_dex3_finger_order` (Task 5) |
| `dimos/robot/unitree/g1/system_prompt.py` | The new gesture names (Task 5) |
| Tests | `test_sonic_session.py`, `test_sonic_arm_keyframes.py`, `test_sonic_arm_skill.py`, new `test_imported_gestures.py`, `test_sonic_arm_sim_e2e.py` |

---

### Task 1: Session hand step limit (a separate, shared change)

**Interfaces:**
- `SonicModelProfile.hand_step_limit: float | None = None`, validated as > 0 when set.
- `SonicTaskSession`: wherever it clamps tokens (`action[:64] = last[:64] + clip(...,
  token_step_limit)`, around `sonic_session.py:536`), also clamp `action[64:78]` against
  `_last_action[64:78]` by `hand_step_limit` when it is set. Apply the same clamp on every path
  that emits an action, including the seed and arming paths, if they emit hand values.
- The blueprint factors `SONIC_ARM_PROFILE` (it already exists as a module constant) to set
  `hand_step_limit=PEAK_SPEED / RATE`.

- [ ] **Step 1: write the failing tests.**
  - `test_sonic_session.py::test_hand_step_limit_clamps_transmitted_hands`: a chunk whose hand
    columns jump by 1.0 rad is transmitted in steps of ≤ 0.015 when `hand_step_limit=0.015`.
  - `test_sonic_session.py::test_no_hand_step_limit_by_default`: the existing behaviour is
    unchanged.
  - `test_sonic_arm_skill.py::test_single_robot_profile_limits_hand_steps`: the blueprint's
    profile has `hand_step_limit == pytest.approx(PEAK_SPEED / RATE)`.
- [ ] **Step 2:** run them and see them fail.
- [ ] **Step 3:** implement.
- [ ] **Step 4:** `pytest dimos/robot/unitree/g1 -q` green; ruff and mypy clean.
- [ ] **Step 5:** report. The controller commits this task separately once the user OKs commits,
  and sends the hash to the dual session.

---

### Task 2: Hands in the keyframe engine

**Interfaces:**
- `Keyframe(target: NDArray[29], hold_s: float = 0.0, hands: NDArray[14] | None = None,
  min_s: float = 0.0)`. `min_s` is the requested minimum duration of the segment arriving at this
  keyframe.
- `keyframe_plan(base, keyframes, *, base_hands: NDArray[14] | None = None, lead_s=1.0,
  move_s=2.0, peak_speed=PEAK_SPEED, split_every=None, return_min_s: float = 0.0)` returns a
  `KeyframePlan` with the new field `hands: NDArray[N, 14]`.
  - When `base_hands` is None, the hand rows are zeros.
  - A keyframe with `hands=None` keeps the previous hand target.
  - Segment duration is `max(move_s, kf.min_s, 1.875·max(|Δjoint|, |Δhand|)/peak_speed)`. The
    final segment also gets `return_min_s`.
  - The hands interpolate with the same min-jerk sampling as the joints.
  - Imported motions call it with `move_s=0.5`.
- `retreat_hands(plan, segment) -> NDArray[M, 14]` is row-aligned with `retreat_poses(plan,
  segment)`. Keep `retreat_poses`' signature unchanged.
- `stroke_plan` is unchanged.

- [ ] **Step 1: write the failing tests** (`test_sonic_arm_keyframes.py`).
  - Hand rows follow the min-jerk profile.
  - A finger change of 1.75 rad stretches the segment to ≥ 1.875·1.75/0.75 s.
  - `min_s` stretches a short move, and `return_min_s` the final segment.
  - `retreat_hands` reverses and returns the hands to `base_hands`, row-aligned with
    `retreat_poses`, for every segment of a multi-keyframe plan.
  - With no hands given, `plan.hands` is all zeros and every existing test passes unchanged.
- [ ] **Steps 2–4:** RED, implement, GREEN (the g1 suite).

---

### Task 3: Both-arm collision checker and two-arm envelope

**Interfaces:**
- `ArmCollisionChecker(scene, margin=0.01, closer=0.005, arms: tuple[str, ...] = ("left",))`.
  - With `("left", "right")`, the arm geoms are both arms' `shoulder_yaw`, `elbow` and `wrist`
    links.
  - The ignored set depends on `arms`.
    - For `("left",)` it is exactly today's `_IGNORED`: the left shoulder pitch and roll, the
      hands, and the world. The right shoulder links stay checked against the left arm.
    - For `("left", "right")` it adds the right shoulder pitch and roll, which sit next to the
      torso.
  - Contacts are counted between each arm and the non-arm body, and between the left-arm and
    right-arm geoms.
  - The start-baseline rule is unchanged.
- `two_arm_margins(body: float, arms: float) -> NDArray[29]`: legs and waist at `body`, both arms
  at `arms`.

- [ ] **Step 1: write the failing tests** (`needs_scene`).
  - Two arms crossed in front of the torso are reported as an arm-against-arm contact.
  - A right arm swung into the torso is reported.
  - `arms=("left",)` gives byte-identical results to today on the existing checker tests.
  - A left wrist placed against the RIGHT shoulder links (pitch or roll) is still reported with
    `arms=("left",)`. That pins the original exclusions.
  - `two_arm_margins` values.
- [ ] **Steps 2–4:** RED, implement, GREEN.

---

### Task 4: Copied data and loader

**Files:** create `dimos/robot/unitree/g1/gestures/__init__.py`, the seven `<name>.json` files, and
`SOURCE.md`.

**Interfaces:**
```python skip
UB_LIMITS: dict[str, tuple[float, float]]        # copied, by joint name (their upper_body.LIMITS)
HAND_ORDER = ("thumb_0","thumb_1","thumb_2","middle_0","middle_1","index_0","index_1")  # both hands
HAND_LIMITS: dict[str, tuple[tuple[float, float], ...]]   # "left"/"right", command order
HAND_DEFAULT: dict[str, tuple[float, ...]]                # deploy fist, command order
@dataclass(frozen=True)
class ImportedKeyframe: t: float; delta: dict[int, float]; absolute: dict[int, float]; hands: NDArray[14] | None
@dataclass(frozen=True)
class ImportedGesture: name: str; keyframes: tuple[ImportedKeyframe, ...]; return_s: float
def load_gestures() -> dict[str, ImportedGesture]
def resolve(g: ImportedGesture, base: NDArray[29], base_hands: NDArray[14]) -> list[Keyframe]
    # ValueError naming joint/value/limit if any resolved target leaves UB_LIMITS
```
- **The JSON copies** are built by a throwaway script in the scratchpad from
  `git show 3430faa0059b656f6dba665fbdeeed7ed3899f45:motion_control/motion_control/gestures/data/<name>.json`. Keep exactly the data keys, `sort_keys`, indent 2. Do not
  commit the script.
- **Joint keys** map to motor indices through `MOTOR_NAMES`. **Hand keys** map to slots through
  `HAND_ORDER`; the left hand fills slots 0–6 and the right hand 7–13.
- **`resolve`.** For each keyframe, a joint's target is its `absolute` value if given, otherwise
  `base + delta`; joints the keyframe doesn't name stay at base.
  - The hands are `HAND_DEFAULT` with the named fingers overridden, or the previous hands when the
    keyframe has none. The first keyframe's hands are `base_hands`.
  - Each segment's `min_s` is the gap from the previous keyframe's `t`.
  - Identical consecutive targets come out as segments with zero displacement. They act as holds,
    and give chunk splits a still stretch.
  - The final segment's minimum is `return_s` (passed as `return_min_s` by Task 5).

- [ ] **Step 1: write the failing tests** (new `test_imported_gestures.py`).
  - Every name listed in `gestures.RETAINED` loads. `RETAINED` is the module's tuple of
    imported names: all seven at first, fewer if Task 6 drops any.
  - **Permutation:** a synthetic gesture sets a distinct value per name for all 14 arm joints;
    each lands at `MOTOR_NAMES.index(name)`.
  - **Finger packet:** distinct values per named finger on each hand, through `resolve`, then
    `encode_token_reference(left_hand=h[:7], right_hand=h[7:])` and a decode. Each value is in the
    expected slot.
  - **Refusals:** a waist delta; an unknown key; an absolute value out of `UB_LIMITS`; a hand value
    outside `HAND_LIMITS`.
  - **Resolved-target refusal:** support_person from a base with elbow −0.8 raises, naming
    `left_elbow` or `right_elbow`, the value and the limit.
  - **`_doc` stripped:** no file contains it.
  - **`SOURCE.md` complete:** it names commit `3430faa`.
- [ ] **Steps 2–4:** RED, implement, GREEN.

---

### Task 5: Skill surface, hands in the stream, finger-order RPC

**Interfaces:**
- **`GestureSpec`:** `name`; `keyframes_of(base, base_hands) -> list[Keyframe]`; `arms`
  (`("left",)` or `("left", "right")`); `envelope` (`"pose"` or `"two_arm"`); `move_s`;
  `return_min_s`.
- **`GESTURES: dict[str, GestureSpec]`:** our five, plus the seven from `load_gestures()`.
  `GestureName` becomes a `Literal` of all twelve names; a test asserts it equals `set(GESTURES)`.
- **`gesture_keyframes(name, base, base_hands=None)`** stays available for the dual container.
- **`_move`:**
  - It takes the base hands from the feedback (`left_hand_q + right_hand_q`) and passes them to
    `keyframe_plan`.
  - It sends `plan.hands` per row, instead of the constant measured hands.
  - Its retreat rows use `retreat_hands`.
  - The "already there" check also counts hand displacement.
  - `perform_gesture` picks the checker and the envelope from the spec. Keep a second
    `ArmCollisionChecker` instance with `arms=("left", "right")`, built in `start()` when the
    collision model is set; imported motions require it.
  - Resolved-target limit failures surface as `ValueError` before anything is sent.
- **`GESTURE_MAX_S = 45.0`.** The prompt lists the seven new names in one line each, with the
  hands noted.
- **`check_dex3_finger_order` is a managed background motion** started by an `@rpc`, NOT a
  `@skill`, so the agent cannot call it.
  - The worker serves RPCs one at a time, so a synchronous RPC that runs the motion would block
    `stop_arm` until it ended.
  - The RPC takes `_lifecycle` exactly as `_launch` does: it refuses if a motion is running, sets
    `_running = "check_dex3_finger_order"`, and clears `_stop`.
  - It starts the same daemon thread pattern, through `_run`, so the motion uses the same guarded
    session, `hand_step_limit` and retreat. Then it RETURNS PROMPTLY with "started".
  - There is no tool stream, because this is not a skill. The thread's cleanup, under
    `_lifecycle`, stores the final text in `self._last_operator_result`, logs it, and clears
    `_running` and `_watch`. Factor `_launch`'s worker body into a shared helper, so both paths use
    one cleanup.
  - A second `@rpc`, `operator_result() -> str | None`, returns the stored text.
  - `stop_arm` stops the check like any motion.
  - The plan: the arms stay at the measured base, and the hands go to the asymmetric pose for a
    3 s hold, then return.
    - LEFT (command order): thumb_0 0.0, thumb_1 0.0, thumb_2 0.875, middle_0 −1.57,
      middle_1 −1.75, index_0 0.0, index_1 0.0. That is index open, middle closed, thumb half.
    - RIGHT: thumb_0 0.6, thumb_1 0.0, thumb_2 −1.75, middle_0 0.0, middle_1 0.0, index_0 1.57,
      index_1 1.75. That is middle open, index closed, thumb rotated.
  - The result text names the pose, for the operator to confirm by eye.
- **Motions that move only the hands** (the diagnostic, or any future one).
  - The body displacement is zero, so `handle_feedback`'s progress (`q·delta/(delta·delta)`) would
    be NaN, and the report would say "reached 0%".
  - When `delta` is all zeros, skip the body progress, but keep the body envelope (the margins) and
    the speed watchdog.
  - Report the hand result instead. Unavailable feedback must be told apart from a hand that
    failed to move (user clarification, 2026-10-23).
    - **Availability is decided only from configuration**, never from the readings. Add
      `hands_mounted: bool = False` to `SonicArmSkillConfig`; the blueprint and pc2_222's run set
      it, and pc2_223 runs `--disable-dex3-hands`.
    - **`hands_mounted` is False:** report "hand feedback unavailable (no hands configured)", and
      claim nothing about the hand pose.
    - **`hands_mounted` is True:** zero or unchanged readings are NOT "unavailable"; they are a
      hand that did not move.
      - "Hand pose held" requires tracking evidence: during the hold window, every commanded finger
        is within 0.2 rad of its target. Report the hand reach and the worst finger.
      - Otherwise report "hand did not follow: <finger> off by X rad", which is a failure outcome,
        not success.
    - The final message must never claim "did not follow the reference" for a hand-only motion on
      body grounds, and must never report NaN.
- **Unit tests** (scripted peer):
  - For `wave_two_hands`, the hand columns submitted equal `plan.hands` at sampled rows.
  - After the session, transmitted hand steps stay ≤ 0.015 through a stop issued at the
    peak-finger-velocity row (exercising Task 1 in the real stream).
  - The support_person resolved-limit refusal sends nothing.
  - `check_dex3_finger_order` submits the asymmetric pose, refuses while a motion runs, and is not
    in the skill list.
  - **Through the real worker interface:** deploy the container in a worker as the existing worker
    tests do (`worker_execution` / `dimos_cluster` fixtures in `test_sonic_execution_module.py`).
    Call `check_dex3_finger_order` and check it returns "started" in under 1 s. Then call
    `stop_arm` mid-hold and check it returns promptly, the motion retreats, and `operator_result()`
    reports "stopped early: stop requested".
  - **The hand-only result:**
    - `hands_mounted=True` with hand feedback that follows the targets reports "hand pose held",
      with no NaN.
    - `hands_mounted=True` with all-zero or unchanged hand feedback reports "hand did not follow"
      as a failure, NOT "unavailable".
    - `hands_mounted=False` reports "hand feedback unavailable (no hands configured)".
  - `GestureName` equals the set of gesture names.
  - All existing tests stay green.
- [ ] **Steps 1–4:** RED, implement, GREEN.

---

### Task 6: MuJoCo e2e with per-joint tracking

- Add one case per imported motion, plus `stop_arm` mid-`wave_two_hands`. Also add the finger
  check, which on the stand-in checks the hand-only reporting path end to end, without hands.
- **Tracking.** For each keyframe, compute the window from the plan's timing (the plan start comes
  from the journal's first `chunk_accepted.observed_at`, as before). Every commanded joint must
  come within 0.25 rad of its target in that window; print the worst joint and its error per
  keyframe.
- **Stability, as for every case:** tilt ≤ 3°, contacts ≥ 7, |dq| ≤ 2.0, handback, and both arms
  back within 0.05 rad.
- **Failures.** A motion that fails tracking, an envelope or the path check is dropped. Never
  loosen a threshold. To drop it, all of these, in one change:
  - delete its JSON;
  - remove it from `gestures.RETAINED`, `GestureName` and the prompt's gesture list;
  - remove its e2e case;
  - add a "Dropped" entry with the measured reason to `SOURCE.md`;
  - update the Task 7 hardware sequence to the retained set.

  Then verify the final tree: the g1 suite is green, the `GestureName == set(GESTURES)` test
  passes, the prompt test passes, and `grep -r <dropped name> dimos/` finds only `SOURCE.md`.
- The controller arranges MuJoCo with dimos-dev-1a. The full suite runs once; one rerun is allowed,
  for harness bugs only.

---

### Task 7: Dual-robot integration (coordination; the dual session implements on its branch)

**Make it portable from our side (during Tasks 2–5).** Put the new behaviour in module-level
functions that take no container state, so the dual branch calls them instead of re-implementing:
- `gesture_plan_inputs(name, base, base_hands) -> (keyframes, move_s, return_min_s, arms,
  envelope)`;
- `hand_rows(plan)`;
- the checker's `arms` choice and `two_arm_margins`;
- `gestures.resolve`.

`_move` uses these too.

**Commits and push.** After the user OKs commits, the controller makes two commits: Task 1 alone
(the session hand limit), then Tasks 2–6. These are pushed in one batch with Phase 1
(`c70216be0`) to `v2`. Send dimos-dev-1a the hashes.

**Required on the dual branch.** That branch uses `ArmMove`, `gesture_move` and `SonicArm.prepare`;
its execution path still sends constant hands, and uses the old timing, envelope and checker.
Keeping `gesture_keyframes` available does NOT integrate the changes. The dual session must:
1. Port the per-row hands (`plan.hands` into the stream, `retreat_hands` into the retreats), the
   imported timing (`move_s`, `return_min_s`), the two-arm envelope, and the both-arm checker into
   `SonicArm.prepare`, `ArmMove` and `gesture_move`, using the shared functions above. Two pieces
   live inside `_move`, not among the shared functions, and must be ported too:
   - the padded hand rows;
   - the split mask `token_still & plan_still`, so a split never lands mid-finger-motion.

   `KeyframePlan.hands` is now a required field, so any direct `KeyframePlan(...)` construction
   breaks.
2. Set `hand_step_limit` on both dual execution profiles. They inherit it from `SONIC_ARM_PROFILE`;
   a test asserts both.
3. Expose an operator `@rpc check_dex3_finger_order(robot)` on the dual container, plus
   `operator_result(robot)`. They delegate to that robot's managed background motion through its
   own guarded, speed-limited session, return promptly, and are stoppable by `stop_arm`.
4. Add integration tests:
   - `robot="both"` `two_hand_hearts` transmits MOVING hand rows (not constant) on each robot, step
     limited;
   - an imported motion's both-arm path check refuses an arm-against-arm contact;
   - when preparation fails on one robot (e.g. a resolved-limit refusal from that robot's base),
     NEITHER robot starts;
   - the finger-check RPC returns promptly and stops through `stop_arm` via the worker interface.
   - **The dual container on two stand-ins** (PR #1 review addition; the dual session owns it):
     - an imported gesture on both robots records the start skew;
     - an asymmetric preparation refusal sends to neither robot;
     - a stop or fault on one robot retreats both;
     - differing measured hand starts, including the handless configuration, exercise the
       hand-dependent timing.
5. Rerun the two-stand-in run, then the hardware sequence, adjusted to the retained set from
   Task 6:
   1. pc2_223: `support_person` and `two_hand_hearts` (arms only; pc2_223 has no hands).
      `wave_two_hands` and `open_window` were dropped in Task 6; see `gestures/SOURCE.md`.
   2. pc2_222 at planner idle, before any hand motion: read `left_hand_q`/`right_hand_q` and
      compare slot for slot with `HAND_DEFAULT`. Zeros mean hand feedback is not wired; a
      permutation means feedback order differs from command order. Either one stops the run
      until fixed. Then `check_dex3_finger_order`, confirmed by eye; on failure, the logged
      "Hand result" base/target/held values tell the cases apart;
   3. pc2_222: the remaining hand motions.

The controller sends these requirements with the hashes, and reviews the dual session's report
against them.
