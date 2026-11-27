# G1 SONIC Tier-1 Arm Skills Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add gestures, longer holds, pointing (by direction and at a camera-detected object) and an
agent-callable stop to the G1 left-arm SONIC skills. Every stop must retreat smoothly.

**Architecture:**
- **Keyframe engine:** a pure-numpy engine in `sonic_arm_reference.py` turns keyframes into a
  min-jerk forward path. It also gives a bounded retreat for every stop point: reverse along the
  current segment, then go straight home.
- **Skill container:** `SonicArmSkillContainer` moves onto the engine. Its arm skills become
  `background` skills, so the agent stays free to call `stop_arm`.
- **Pointing:** MuJoCo forward kinematics plus the head-camera mount turn a direction, or a
  detected box, into a one-keyframe plan.
- **Simulation:** a MuJoCo stand-in for the guarded controller moves into the repo for
  end-to-end tests.

**Tech Stack:** Python 3.12, numpy, MuJoCo, onnxruntime (SONIC v1.1 encoder), ZMQ, DimOS
modules/skills/MCP, OpenAI-compatible VL client (Genon gateway), pytest.

**Spec:** `docs/superpowers/specs/2026-10-21-g1-sonic-tier1-skills-design.md` (approved
2026-10-21). Read it first; this plan argues from it.

## Global Constraints

- **Speed:** peak arm speed stays `PEAK_SPEED` = 0.47 rad/s, measured on the sampled plan
  (`max|diff| * RATE`); no skill raises it. Today's `stroke_plan` overshoots this by up to 0.7%
  (0.4721 rad/s for a 0.7 rad stroke): `round(T * RATE)` samples span only `n - 1` intervals.
  Task 1 corrects the sampling.
- **Envelopes are unchanged:**
  - `pose_margins(body 0.15, moving arm 0.45, other arm 0.35)` for poses, joint targets,
    gestures and pointing;
  - `group_margins(0.15, 0.20, 0.30)` for `move_upper_body`;
  - `max_dq` config default 3.0; robot and MuJoCo runs pass `--max-dq 2.0`.
- **Session chunk limit:** at most 1000 rows (`_MAX_CHUNK_ROWS`, 20 s at `RATE` = 50).
  - Splits happen only where the encoded tokens stay unchanged for `_SPLIT_STILL_ROWS` = 15 rows
    (0.3 s, the session's blend).
  - The encoder looks 45 rows ahead (`ENCODER_LOOKAHEAD_ROWS`) and its velocity term uses one row
    on each side. Poses therefore have to be still for `_SPLIT_POSE_ROWS = 15 + 45 + 2 = 62`
    rows (1.24 s) to give 15 still token rows.
  - `_CHUNK_OVERLAP` = 25 stays.
- **Retreats:** every retreat is one chunk of at most 1000 rows. Overspeed during a retreat
  cancels at once; this is the only accepted exception to "every stop retreats".
- **Limits:**
  - gesture forward plans ≤ 40 s (`GESTURE_MAX_S`);
  - `hold_seconds` 1–20;
  - `point_at` cone: azimuth −35…+110°, elevation −75…+75° (torso frame; azimuth positive to
    the robot's left, elevation positive up);
  - IK refuses when the solution is more than 3° off;
  - `point_at_object` assumes the object is 2.0 m along the camera ray.
- **Profile:** `max_duration` 75 s; `max_observation_age` stays 35 s and is checked against the
  longest chunk.
- **Boundaries:** never start or stop the native controller, and never send motor commands
  outside the guarded session. Robot runs are operator actions.
- **Code style:**
  - mypy strict and ruff clean; imports at the top of the file;
  - every `@skill` has a docstring and annotated parameters;
  - manual scripts start with `demo_`.
- **Commits:** the user commits only on request. Each task's commit step means "stage, and
  commit only if the user has said to commit during execution"; otherwise leave it staged and
  report.
- **Private details stay out of commits:** no LAN IPs, pc2 paths or keys in committed files. The
  Genon key is read only as `GENON_API_KEY` from `.env.genon`, which is never sourced whole.

## Review Focus

1. **Stop when nothing is moving, or during encoding:** `stop_arm` must say "no arm motion
   running", or make the pending motion send nothing ("stopped on request; nothing sent"). It
   must never start a stream. Test owned by Task 3.
2. **Stop while already retreating** (a second `stop_arm`, or an envelope breach followed by a
   stop): the retreat continues unchanged, with no second resubmission. Owned by Task 3.
3. **A second arm-skill call while one runs, including a retry of the same skill.** It is
   refused. The first motion's tool stream and its movement capability stay open until that
   motion ends, and it streams to completion.
   - Same skill: MCP's same-tool takeover rebinds the live stream to the retry's acquire token.
     A refusal that called `stop_tool` would close the running motion's stream and free its
     capability.
   - Owned by Task 3, tested through the MCP tool-stream harness.
4. **A gesture started with the arm not at rest** (e.g. still raised from an interrupted run):
   the plan starts from the measured pose, is path-checked from there, and its home path returns
   to that measured pose, not to a nominal rest. Owned by Task 4.
5. **Vision call failure looks like "not found":** `query_detections` swallows exceptions and
   returns no boxes, so Task 6 does not use it. Its own `query_boxes` lets a failed call surface
   as "vision model <name> failed: …", distinct from "found no '<description>'". No motion
   either way. Owned by Task 6.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `dimos/robot/unitree/g1/sonic_arm_reference.py` | Pure planning: keyframe engine, retreat units, `ArmKinematics`, head-camera geometry, encoder anchor | 1, 5, 6, 7 |
| `dimos/robot/unitree/g1/sonic_arm_skill.py` | Container: chunking, `_move` and `_stream` on the engine, background launch, `stop_arm`, gestures, pointing skills | 1–6 |
| `dimos/robot/unitree/g1/test_sonic_arm_keyframes.py` (new) | Engine, chunking, retreat, kinematics and camera-geometry unit tests | 1, 5, 6 |
| `dimos/robot/unitree/g1/test_sonic_arm_skill.py` | Container tests against the scripted peer (existing 44, adapted, plus new ones) | 2–6 |
| `dimos/robot/unitree/g1/system_prompt.py` | `G1_SONIC_ARM_SYSTEM_PROMPT` | 3–6 |
| `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sonic_arm.py` | Profile `max_duration` 75, camera constants imported, docstring CLI usage | 2, 3, 6 |
| `dimos/robot/unitree/g1/sonic_sim_peer.py` (new) | Real-time MuJoCo stand-in for the guarded controller (plant, decoder, idle planner, feet-down startup) | 7 |
| `dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py` (new) | `native_e2e` end-to-end skill runs against the stand-in | 7 |
| `dimos/robot/unitree/g1/demo_vl_grounding.py` (new) | Measures VL box accuracy on labelled head-camera frames | 8 |

---

### Task 1: Keyframe engine, retreat units and general chunking

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_reference.py` (add after `stroke_plan`; make
  `stroke_plan` a wrapper)
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py:103-130` (`chunk_rows`, `_SPLIT_STILL_ROWS`)
- Test: `dimos/robot/unitree/g1/test_sonic_arm_keyframes.py` (new)

**Interfaces:**
- Produces, in `sonic_arm_reference.py`:
  ```python skip
  @dataclass(frozen=True)
  class Keyframe:
      target: NDArray[np.float64]  # [29] absolute motor-order pose
      hold_s: float = 0.0

  @dataclass(frozen=True)
  class Segment:
      start: int  # first pose row of the move (the arm leaves `src`)
      end: int    # one past its last row (the arm is at `dst` at end - 1)
      src: int    # keyframe index left; -1 = the base pose
      dst: int    # keyframe index reached; -1 = the base pose (the final return)

  @dataclass(frozen=True)
  class KeyframePlan:
      base: NDArray[np.float64]       # [29]
      targets: NDArray[np.float64]    # [K,29] keyframe poses after any inserted pauses
      poses: NDArray[np.float64]      # [N,29] lead-in, segments and holds, final return, lead-out
      segments: list[Segment]         # in time order; the last has dst == -1
      still: NDArray[np.bool_]        # [N] poses unchanged from row-1 through row+_SPLIT_POSE_ROWS-2
                                      # (enough for 15 still tokens; the token check is Task 2's)

  def keyframe_plan(base, keyframes: Sequence[Keyframe], *, lead_s: float = 1.0,
                    move_s: float = 2.0, peak_speed: float = PEAK_SPEED,
                    split_every: int | None = None) -> KeyframePlan
  def retreat_poses(plan: KeyframePlan, segment: int) -> NDArray[np.float64]
  def retreat_route(plan: KeyframePlan, row: int) -> tuple[int, int] | None
  ```
- Produces, in `sonic_arm_reference.py`: `ENCODER_LOOKAHEAD_ROWS = 45`. Also
  `ArmCollisionChecker.first_contact(poses, every=5, start: NDArray[Any] | None = None)`: the
  tolerated-contact baseline comes from `start` (default `poses[0]`). Item 3: a retreat begins
  with a raised arm, so a baseline from its own first pose would flag the normal
  wrist-near-hip rest it returns to.
- Produces, in `sonic_arm_skill.py`:
  - `chunk_rows(plan_rows: int, still: NDArray[np.bool_]) -> list[tuple[int, int]]`, where
    `still` is over token-plan rows;
  - `token_still(tokens: NDArray[Any]) -> NDArray[np.bool_]`, true where the token rows `r`
    through `r+14` equal row `r` within 1e-9;
  - `_SPLIT_STILL_ROWS = 15` and `_SPLIT_POSE_ROWS = _SPLIT_STILL_ROWS + ENCODER_LOOKAHEAD_ROWS + 2`.

**Algorithms the signatures don't pin:**
- **Segments:**
  - Each segment is min-jerk from its source pose to its destination, with
    `T = max(move_s, 1.875 * max|Δ| / peak_speed)`.
  - It is sampled as `n = ceil(T * RATE) + 1` rows, so its `n - 1` intervals cover at least `T`
    and the sampled peak never exceeds `peak_speed`. (Today's `round(T * RATE)` rows overshoot.)
  - Holds repeat the target. The final return goes from the last keyframe to `base`.
  - `stroke_plan` becomes a wrapper over this in the same task. The commissioned 0.5 rad / 2 s
    stroke gains one row (101 instead of 100).
- **`split_every`:**
  - While building, track the pose rows since the last `still` row.
  - If adding the next segment would push that gap past `split_every`, raise the current
    keyframe's hold to at least `_SPLIT_POSE_ROWS / RATE` (1.24 s).
  - A 1.0 s hold is shorter than that and is lengthened the same way when a split must fall
    there (e.g. the overhead poses).
  - If one segment alone is longer than `split_every`, raise
    `ValueError("segment longer than a session chunk")`.
- **`retreat_poses(plan, s)`** returns only the part of the retreat a stop can reach (item 2):
  - for a non-final segment: `poses[seg.start:seg.end][::-1]`, then a min-jerk segment from
    `seg.src`'s pose to `base` when `src >= 0` (same duration and sampling rule), then `lead_s`
    of `base`;
  - for the final segment, only its home part: from the last keyframe to `base` (the final
    return itself), then `lead_s` of `base`.
  - So raise_hand's hold-to-home retreat is about 707 token rows, not the 1,264 a reversed final
    return would add.
- **`retreat_route(plan, row)`:**
  - inside segment `s` (not the final one): returns `(s, seg.end - 1 - row)`;
  - in the lead-in, before segment 0: returns `(0, len(segment 0))`;
  - in the hold after keyframe `k`: returns `(s_next, len(s_next))`, where `s_next` is the
    segment leaving `k`;
  - in the hold after the last keyframe: returns `(s_final, 0)`;
  - on the final return or the lead-out: returns `None`.
- **`chunk_rows`:** greedy. While `plan_rows - start > 1000`, take the largest `s > start`
  with `still[s]` and `s + 25 - start <= 1000`; emit `(start, s + 25)` and set `start = s`.
  Raise `ValueError("plan cannot be split into session chunks")` when there is no such `s`. One
  chunk when it fits.

- [ ] **Step 1: Write the failing tests** in `test_sonic_arm_keyframes.py`:

```python skip
BASE = np.zeros(29); BASE[[15, 16, 18]] = 0.19, 0.19, 0.59  # hanging rest, motor order

def kf(pitch, roll, yaw, elbow, hold=0.0):
    t = BASE.copy(); t[[15, 16, 17, 18]] = pitch, roll, yaw, elbow
    return Keyframe(t, hold)

@pytest.mark.parametrize("joint,delta", [(15, -0.35), (16, 0.7), (18, -1.05), (15, -2.99)])
def test_one_keyframe_is_a_symmetric_stroke_at_or_under_the_peak_speed(joint, delta):
    d = np.zeros(29); d[joint] = delta
    plan = keyframe_plan(BASE, [Keyframe(BASE + d, 1.0)])
    np.testing.assert_array_equal(plan.poses, stroke_plan(BASE, d))  # stroke_plan wraps the engine
    np.testing.assert_array_equal(plan.poses, plan.poses[::-1])        # out and back mirror
    assert np.abs(np.diff(plan.poses, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert np.count_nonzero(np.all(plan.poses == BASE + d, axis=1)) == 1 + 50 + 1  # arrival, hold, departure

def test_every_segment_respects_the_peak_speed():
    plan = keyframe_plan(BASE, [kf(-0.3, 1.3, 0, 0), kf(-0.3, 1.3, -0.35, 0), kf(-0.3, 1.3, 0.35, 0)])
    assert np.abs(np.diff(plan.poses, axis=0)).max() * RATE <= PEAK_SPEED + 1e-9
    assert [(s.src, s.dst) for s in plan.segments] == [(-1, 0), (0, 1), (1, 2), (2, -1)]

def test_every_retreat_of_one_keyframe_is_a_suffix_of_the_plan():
    d = np.zeros(29); d[15] = -0.35
    plan = keyframe_plan(BASE, [Keyframe(BASE + d, 1.0)])
    for s in range(len(plan.segments)):
        unit = retreat_poses(plan, s)
        np.testing.assert_array_equal(unit, plan.poses[-len(unit):])  # whole unit: no extra encode
    for row in range(len(plan.poses)):
        if retreat_route(plan, row) is None:
            assert row >= plan.segments[-1].start

def test_hold_after_the_last_keyframe_retreats_only_the_way_home():
    plan = keyframe_plan(BASE, [kf(-2.6, 0.45, 0.0, 1.2, hold=2.0)])  # raise_hand
    row = plan.segments[0].end + 20
    s, offset = retreat_route(plan, row)
    assert (s, offset) == (len(plan.segments) - 1, 0)
    assert len(retreat_poses(plan, s)) + 100 <= 1000  # ~707 token rows with the blend

def test_stop_mid_swing_reverses_to_the_previous_keyframe_then_goes_home():
    plan = keyframe_plan(BASE, [kf(-0.3, 1.3, 0, 0), kf(-0.3, 1.3, -0.35, 0), kf(-0.3, 1.3, 0.35, 0)])
    seg = plan.segments[2]  # yaw -0.35 -> +0.35
    row = (seg.start + seg.end) // 2
    s, offset = retreat_route(plan, row)
    unit = retreat_poses(plan, s)
    np.testing.assert_allclose(unit[offset], plan.poses[row])
    np.testing.assert_allclose(unit[offset + (row - seg.start)], plan.targets[1])  # back at yaw -0.35
    np.testing.assert_allclose(unit[-1], BASE)
    assert len(unit) - offset < len(plan.poses) - row + 300  # never re-traces the gesture

def test_stop_in_a_hold_goes_straight_home():
    plan = keyframe_plan(BASE, [kf(-0.3, 1.3, 0, 0, hold=1.0), kf(-0.3, 1.3, -0.35, 0)])
    row = plan.segments[0].end + 10
    s, offset = retreat_route(plan, row)
    np.testing.assert_allclose(retreat_poses(plan, s)[offset], plan.targets[0])

def test_long_sequences_get_pauses_only_where_a_split_needs_one():
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    plan = keyframe_plan(BASE, swings, split_every=300)
    still_rows = np.nonzero(plan.still)[0]
    assert np.diff(still_rows).max() <= 300
    assert 0 < sum(h > 0 for h in plan_keyframe_holds(plan)) < len(swings)  # helper below

@needs_scene
def test_retreats_keep_the_measured_start_contact_baseline():
    checker = ArmCollisionChecker(SCENE)
    plan = keyframe_plan(ROBOT_IDLE, [Keyframe(raised(ROBOT_IDLE), 1.0), Keyframe(raised2(ROBOT_IDLE), 0.0)])
    unit = retreat_poses(plan, 1)  # starts raised, ends at the wrist-near-hip rest
    assert checker.first_contact(unit) is not None                     # own-start baseline: false alarm
    assert checker.first_contact(unit, start=ROBOT_IDLE) is None       # measured-start baseline

@needs_encoder
def test_a_split_pause_gives_still_tokens_for_the_session_blend():
    swings = [kf(-1.2, 0.3, 0, e) for e in (0.6, 0.0) * 8]
    plan = keyframe_plan(BASE, swings, split_every=300)
    tokens = SonicEncoder(ENCODER).encode(plan.poses)
    still = token_still(tokens)
    for r in np.nonzero(plan.still)[0]:
        assert still[r]  # every pose-still row yields 15 unchanged tokens

def test_chunks_split_only_where_still_and_fit_the_session():
    still = np.zeros(2600, bool); still[[900, 1700]] = True
    chunks = chunk_rows(2600, still)
    assert chunks == [(0, 925), (900, 1725), (1700, 2600)]
    assert all(e - s <= 1000 for s, e in chunks)
    with pytest.raises(ValueError):
        chunk_rows(2600, np.zeros(2600, bool))
```

  Test helpers and constants:
  - `plan_keyframe_holds` is a 3-line helper. It reads back each keyframe's hold as the run of
    rows at `targets[k]` between `segments[k].end` and `segments[k+1].start`.
  - `raised` / `raised2` set two left-arm targets from the measured pose (e.g. shoulder pitch
    −1.5, then −1.5 with roll 0.6).
  - `ROBOT_IDLE`, `SCENE` and `needs_scene` come from the existing real-model tests.
  - `ENCODER` / `needs_encoder` find `model_encoder.onnx` via env `GEAR_SONIC_ENCODER`, and skip
    when it is absent. Also keep the
  two-chunk cases of the old `test_long_plans_split_into_two_chunks_at_the_hold`, rewritten to
  the new `chunk_rows` signature (a `still` mask with only the hold start set).

- [ ] **Step 2: Run the tests and verify they fail**

  Run: `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_keyframes.py -v`
  Expected: FAIL, ImportError for `Keyframe` / `keyframe_plan`.

- [ ] **Step 3: Implement** `Keyframe`, `Segment`, `KeyframePlan`, `keyframe_plan`,
  `retreat_poses` and `retreat_route` in `sonic_arm_reference.py`, and `chunk_rows` with
  `_SPLIT_STILL_ROWS` in `sonic_arm_skill.py`, using the algorithms above.

- [ ] **Step 4: Make `stroke_plan` a wrapper:** it returns `keyframe_plan(base,
  [Keyframe(base + delta, hold_s)], lead_s=..., move_s=..., peak_speed=...).poses`. Then update
  the legacy expectations that the corrected sampling shifts. Each stroke changes from
  `round(T * RATE)` to `ceil(T * RATE) + 1` rows, so it gains one or two rows: +1 for 0.35, 0.5
  and 0.7 rad, +2 for 1.05 and 2.99 rad at 0.47 rad/s. Compute each expectation from the
  formula, not by adding one row. The tests to update:
  - plan lengths and row indices in `test_pure_reference_parts`,
    `test_big_strokes_slow_down_to_the_commissioned_peak_speed` and the split and
    one-chunk-ripple tests;
  - any speed assert there tightens to `<= PEAK_SPEED + 1e-9`.

  Note each changed expectation in the commit message.

- [ ] **Step 5: Run the tests and verify they pass.**

  Run: `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_keyframes.py dimos/robot/unitree/g1/test_sonic_arm_skill.py -v`
  Expected: all PASS. The encoder test passes when `GEAR_SONIC_ENCODER` is set, and is skipped
  otherwise; run it once with the variable set.

- [ ] **Step 6: Commit** (see Global Constraints)

```bash skip
git add dimos/robot/unitree/g1/sonic_arm_reference.py dimos/robot/unitree/g1/sonic_arm_skill.py dimos/robot/unitree/g1/test_sonic_arm_keyframes.py dimos/robot/unitree/g1/test_sonic_arm_skill.py
git commit -m "feat(g1): keyframe engine with bounded retreats for the SONIC arm skills"
```

---

### Task 2: Run every arm skill on the engine; `hold_seconds`; plan-wide prechecks

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py` (`_run`, `_run_targets`, `_move`,
  `_stream`; delete `retreat_row` and the `progress` mapping)
- Modify: `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sonic_arm.py`
  (`max_duration=75.0`, profile comment)
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Interfaces:**
- Consumes: Task 1's `Keyframe`, `KeyframePlan`, `keyframe_plan`, `retreat_poses`,
  `retreat_route`, `chunk_rows`.
- Produces:
  - `_run(self, label: str, keyframes_of: Callable[[NDArray[np.float64]], list[Keyframe]],
    allowed: NDArray[np.float64], *, require_check: bool) -> SkillResult`;
  - `_stream(self, task_id: str, plan: NDArray[np.float64], chunks: list[tuple[int, int]],
    retreat: Callable[[int], NDArray[np.float64] | None], profile: dict[str, Any],
    watch: _Watch) -> str`, where `retreat(token_row)` returns the full token-plan rows to
    submit (unit tokens from the offset, end blend to the planner token, hand columns), or
    `None` to keep going;
  - `move_left_arm_pose(self, pose: LeftArmPose, hold_seconds: float = 1.0)`;
  - `move_left_arm_joints(..., hold_seconds: float = 1.0)`.

**Decisions:**
- **Building and checking the plan in `_move`:**
  - Build `KeyframePlan` with `split_every = _MAX_CHUNK_ROWS - _CHUNK_OVERLAP - pad`, where
    `pad` is the token-plan blend length.
  - Path-check `plan.poses`, plus every retreat unit that is not a suffix of `plan.poses`. Every
    check passes `start=base`, the measured pose.
  - Size-check every unit, which `retreat_poses` already trims to its reachable part: raise
    `ValueError("a retreat is longer than one session chunk")` if a unit plus its blend exceeds
    1000 rows.
  - Encode `plan.poses` once. Then encode each distinct non-suffix unit once, keyed by
    `poses.tobytes()`; a suffix unit reuses the forward tokens' tail.
  - Build every unit's token rows with `token_schedule` (its leading blend dropped), so the
    tests' `token_schedule` patch applies.
  - Chunk with `chunk_rows(len(plan), token_still(token_plan))`. That is the token check, not
    the pose mask.
- **The watch:**
  - `delta` = the keyframe displacement with the largest norm from `base`;
  - `allowed = allowed + max_k |targets[k] - base|`;
  - report keys stay, plus `"keyframes": K`.
- **Prechecks:**
  - refuse when `max_observation_age <= longest_chunk/RATE + 0.5`;
  - refuse when `max_duration <= len(plan)/RATE + longest_retreat/RATE + 2.0`.
  - Messages unchanged.
- **The stream:** `_stream` on the first abort calls `retreat(row)`. When that is not `None`,
  it submits the returned rows with `observed_at = now` and sets `plan = those rows` and
  `plan_start = now`. Overspeed after the retreat starts cancels, as today.
- **Holds:** `hold_seconds` is validated as finite and 1 ≤ value ≤ 20, else
  `ValueError("hold_seconds must be within 1..20")`. `move_upper_body` keeps a 1.0 s hold.
- **Test fixture:**
  - patch `sonic_arm_skill.keyframe_plan` with a wrapper that caps every hold at 0.1 s and
    passes `lead_s=0.1, move_s=0.3, peak_speed=100.0`;
  - drop the `stroke_plan` patch;
  - test timings stay as today.

- [ ] **Step 1: Write the failing tests** (add to `test_sonic_arm_skill.py`):

```python skip
def test_one_keyframe_moves_encode_the_plan_once(skill, mocker):
    module, _ = skill
    spy = mocker.spy(module._encoder, "encode")
    run(module, 20.0, follows)
    assert spy.call_count == 1  # the retreat reuses the plan's own way back

def test_long_hold_splits_inside_the_hold_and_passes_the_age_check(skill, mocker):
    module, _ = skill
    mocker.patch.object(sonic_arm_skill, "keyframe_plan", partial(keyframe_plan, lead_s=0.1, move_s=0.3, peak_speed=100.0))  # real holds
    mocker.patch.object(sonic_arm_skill, "_MAX_CHUNK_ROWS", 60)
    result = module.move_left_arm_pose("left_arm_side_horizontal", hold_seconds=2.0)  # feeder as in the pose tests
    assert result.metadata["chunks"] >= 2

@pytest.mark.parametrize("hold", [0.5, 21.0, float("nan")])
def test_hold_seconds_outside_1_to_20_sends_nothing(skill, hold):
    module, _ = skill
    with pytest.raises(ValueError, match="hold_seconds"):
        module.move_left_arm_pose("left_arm_side_horizontal", hold_seconds=hold)
```

  Add `"chunks": len(chunks)` to the report. Rewrite
  `test_retreat_resumes_at_the_matching_point_of_the_way_back` to call `_stream`'s retreat
  through a real stop (as `test_leg_outside_its_envelope_returns_smoothly_then_hands_back`
  does), asserting that the first token of the retreat chunk equals the plan's way-back token
  at the matching row. Delete the `retreat_row` import.

- [ ] **Step 2: Run them and verify they fail**

  Run: `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py -k "encode_the_plan_once or long_hold or hold_seconds" -v`
  Expected: FAIL (`hold_seconds` unexpected keyword; `chunks` missing).

- [ ] **Step 3: Implement** the `_run` / `_move` / `_stream` changes and `hold_seconds` per the
  decisions above; update the docstrings ("hold for `hold_seconds`, 1-20 s"). Set the
  blueprint's `max_duration=75.0` and update its profile comment.

- [ ] **Step 4: Run the whole file and verify it passes**

  Run: `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py dimos/robot/unitree/g1/test_sonic_arm_keyframes.py -v`
  Expected: all PASS. That is the 44 existing tests (adapted) plus the new ones; repeat the run
  3 times to catch the known event-loop flake.

- [ ] **Step 5: Commit**

```bash skip
git commit -am "refactor(g1): arm skills on the keyframe engine; hold_seconds; per-chunk age check"
```

---

### Task 3: Background lifecycle and `stop_arm`

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py`
- Modify: `dimos/robot/unitree/g1/system_prompt.py`
- Modify: `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sonic_arm.py` (docstring: how a
  CLI caller sees the result)
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Why:** the agent (`McpClient`) handles one message at a time, and an `instant` skill blocks it
until the skill returns. So `stop_arm` is only callable mid-motion if the motion skills are
`@skill(uses=[CAP_MOVEMENT], lifecycle="background")`. They return at once and report through
the tool stream, the same pattern as `PointNav.go_to` / `stop_navigation`
(`dimos/agents/skills/point_nav.py`).

**Interfaces:**
- Produces:
  - `_launch(self, tool: str, prepare: Callable[[], Callable[[], SkillResult]]) -> str`.
    `prepare()` validates the arguments, raising `ValueError` as today, and returns the work
    callable, e.g. `lambda: self._run(...)`.
  - `@skill def stop_arm(self) -> str` (no `uses`);
  - `self._stop: threading.Event`;
  - `self._lifecycle: threading.Lock` and `self._running: str | None`. These replace `_busy`,
    which is deleted.
- Every motion skill is `return self._launch(<skill name>, lambda: <validate; return work>)`.
  `_launch` checks for a running motion before it calls `prepare()`, so a same-skill retry with
  invalid arguments still keeps the running motion's capability.
  - Without that order the hold is lost: MCP's same-tool takeover already gave it to the retry,
    and the retry's exception cleanup releases it.

**Decisions:**
- **One lifecycle lock (`_lifecycle`) serializes every running-state transition.** The running
  check, `start_tool`, setting `_running` and clearing `_stop` all happen in one critical section.
  So does the worker's whole cleanup: the final `tool_update`, `stop_tool`, then clearing
  `_running` and `_watch`.
  - So a call arriving during completion either sees the motion running (and rebinds, with the
    final update still to come on that stream) or sees it idle (and opens a fresh stream). It
    never reuses a stream that is about to close.
- **`_launch`, refusals:** checked under the lock, before `prepare()`. They never close or
  release what a running motion holds.
  - **Same skill already running** (`self._running == tool`): call `self.start_tool(tool)`, then
    return `f"{tool} is already running; its result will follow as a tool update. Call stop_arm
    to stop it."` without `stop_tool`.
    - Why: MCP's same-tool takeover has moved the capability hold to this call's token, and
      `start_tool` rebinds the live stream to that token.
    - Returning normally hands the hold to that stream, so it is released when the first
      motion's stream stops.
  - **A different arm skill running** (reachable only by direct RPC, since MCP's capability
    check refuses it first): raise `RuntimeError("an arm move is already running")` before
    `start_tool`.
- **`_launch`, starting a motion:**
  - With nothing running, still under the lock, it calls `work = prepare()`. A `ValueError`
    propagates, and nothing has been opened.
  - It then calls `self.start_tool(tool)`, sets `self._running = tool` and clears `self._stop`.
  - It starts a daemon thread that runs `work()` outside the lock.
  - The thread then enters the lock and posts the final `tool_update`:
    - on success, `replace(result, duration_ms=...).agent_encode()[0]["text"]`;
    - on an exception, `f"{tool} failed: {exc}"`.
  - Still inside the lock, it calls `stop_tool(tool)` and clears `_running` and `_watch`, in
    that order.
  - `_launch` returns `f"{tool} started; a tool update reports the result. Call stop_arm to stop
    early."`.
- **The busy lock leaves `_run`.** `_running` under `_lifecycle` is the only busy state;
  `_run` keeps the rest.
- **`stop_arm`:**
  - nothing running (`_running` is None, read under the lock): returns `"No arm motion is
    running."`;
  - otherwise it sets `_stop` and returns `"Stopping: the arm returns smoothly to its start
    pose; a tool update reports the result."`.
- **Where the stop takes effect:**
  - `_move` checks `_stop` after encoding and before `start_task`:
    `RuntimeError("stopped on request; nothing sent")`;
  - `_stream` treats `_stop` like an envelope abort (`watch.abort = "stop requested"` under the
    lock, only if `abort` is None). The result message reads "… stopped early: stop requested".
- **Test fixture:** patch `module._launch` with `lambda tool, prepare: prepare()()`, so the
  existing tests keep receiving `SkillResult` / raises. New tests use the real `_launch` and capture
  `tool_update` with `mocker.spy`.

- [ ] **Step 1: Write the failing tests:**

```python skip
def test_motion_skill_returns_at_once_and_reports_through_the_tool_stream(real_launch_skill, mocker):
    module, _ = real_launch_skill
    updates = mocker.spy(module, "tool_update")
    text = module.move_upper_body("left_arm_forward", 20.0)
    assert "started" in text
    wait_until(lambda: updates.call_count == 1)
    assert json.loads(updates.call_args.args[1])["message"].startswith("left_arm_forward reached")

def test_stop_arm_with_nothing_running_says_so(skill):
    assert skill[0].stop_arm() == "No arm motion is running."

def test_stop_mid_motion_retreats_smoothly_and_hands_back(real_launch_skill, mocker):
    # stop_arm once the stream is armed; assert the result says "stop requested", a retreat
    # chunk was submitted (sequence 1) and handback was confirmed

def test_stop_during_encoding_sends_nothing(real_launch_skill, mocker):
    # make encode() call module.stop_arm() before returning; assert the update says
    # "stopped on request; nothing sent" and the peer saw no pose packets

def test_second_stop_during_the_retreat_changes_nothing(real_launch_skill, mocker):
    # stop, wait for the retreat chunk, stop again; exactly one retreat submission (spy submit_chunk)

def test_second_motion_while_one_runs_is_refused_and_the_first_completes(real_launch_skill):
    # start a motion, call a different motion skill directly: RuntimeError "already running";
    # the first's update is a success

def test_same_skill_retry_through_mcp_keeps_the_running_stream_and_capability(tool_helper_module, skill_context):
    # Through the MCP tools/call path with a real CapabilityRegistry, as in
    # dimos/agents/mcp/test_tool_stream.py::test_takeover_then_stop_releases_capability:
    # call move_upper_body, then call it again while it runs.
    # - the retry returns "already running" (not an error);
    # - the registry still holds CAP_MOVEMENT and the first stream is still open while the motion runs;
    # - after the motion's final tool update, the stream is stopped and CAP_MOVEMENT is free;
    # - a different arm skill called mid-motion is refused by the registry ("held by 'move_upper_body'")

def test_invalid_same_skill_retry_through_mcp_keeps_the_capability(tool_helper_module, skill_context):
    # same harness: start move_upper_body, retry it with angle_deg=0 while it runs;
    # the retry returns "already running" (invalid args are not even validated),
    # CAP_MOVEMENT stays held through the motion and is free after its final update

def test_call_arriving_during_completion_gets_a_fresh_stream(real_launch_skill, mocker):
    # make the worker's final tool_update block on an event; meanwhile call the same skill:
    # it waits on _lifecycle, then starts a new motion with a new stream (start_tool called twice,
    # stop_tool once for the old run); release; the new run completes and reports on its own stream

def test_concurrent_starts_start_exactly_one_motion(real_launch_skill):
    # 8 threads call move_upper_body together: exactly one "started"; the rest
    # "already running" (same skill); one execution task was started (spy start_task)
```

  `real_launch_skill` is the `skill` fixture without the `_launch` patch. `wait_until` polls up
  to 10 s.

- [ ] **Step 2: Run them and verify they fail**

  Run: `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py -k "tool_stream or stop or second_motion" -v`
  Expected: FAIL (`stop_arm` missing; skills return a `SkillResult`, not "started").

- [ ] **Step 3: Implement** `_launch`, `stop_arm`, the `_stop` checks and the `lifecycle` change
  on all three motion skills. Update the skill docstrings ("Returns at once; a tool update
  reports the result"). Update `G1_SONIC_ARM_SYSTEM_PROMPT`:
  - the motion skills return at once and their result arrives as a tool update;
  - call `stop_arm` when asked to stop or put the arm down;
  - never start another arm skill before the previous one's update arrives.

  Then update `test_agent_prompt_names_only_skills_the_blueprint_has` if it lists skills.

- [ ] **Step 4: Run and verify:** the whole of `test_sonic_arm_skill.py`, 3 times. Expected:
  all PASS.

- [ ] **Step 5: Check the real dispatch path once against the sim (no robot).** Start
  `sonic_sim_peer.py` from the artifact directory on spare ports, then:

  ```
  dimos run unitree-g1-sonic-arm --max-dq 2.0 ... --daemon
  dimos mcp call move_left_arm_pose --arg pose=left_hand_over_head --arg hold_seconds=10
  dimos mcp call stop_arm   # within the hold
  ```

  Ports, private Zenoh group and encoder path are as in the §24 retest command. Expected: the
  second call returns at once, and the sim log shows the arm returning with no dq spike above
  2.0. Record what the first `dimos mcp call` printed. If the final result is not shown there,
  document in the blueprint docstring where an operator sees it (e.g. `dimos log -f`).

- [ ] **Step 6: Commit**

```bash skip
git commit -am "feat(g1): background arm skills and stop_arm with a smooth retreat"
```

---

### Task 4: `perform_gesture`

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py`, `system_prompt.py`
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`, `test_sonic_arm_keyframes.py`

**Interfaces:**
- Produces:
  - `GestureName = Literal["wave", "point_forward", "stop_sign", "raise_hand", "beckon"]`;
  - `GESTURES: dict[str, list[tuple[tuple[float, float, float, float], float]]]`, holding
    absolute left-arm (pitch, roll, yaw, elbow) in rad plus a hold in s;
  - `GESTURE_MAX_S = 40.0`;
  - `@skill(uses=[CAP_MOVEMENT], lifecycle="background") def perform_gesture(self, name:
    GestureName) -> str`.
- `_run` gains its final signature here, so later tasks only add skills (the dual-G1 session
  splits `_move`/`_stream` out after this task): `_run(self, label: str, keyframes_of: ...,
  allowed: ..., *, require_check: bool, report_extra: Callable[[NDArray[np.float64]],
  dict[str, Any]] | None = None) -> SkillResult`. `report_extra(base)` is merged into the
  report's metadata. Tasks 5 and 6 use it for their direction and box fields, and must not
  change `_move`, `_stream` or `_Watch`.
- The table is a module-level constant and keyframes are built by a module-level
  `gesture_keyframes(name: str, base: NDArray[np.float64]) -> list[Keyframe]` (wrists and every
  non-left-arm joint from `base`). This keeps it reusable by the dual-G1 container.

**Starting values** (spec §2; tune in MuJoCo within this intent):

| name | keyframes |
|---|---|
| wave | (−0.3, 1.3, 0.0, 0.0; 0), then (yaw −0.35; 0), (yaw +0.35; 0) ×3, then back to (−0.3, 1.3, 0.0, 0.0; 0.5) |
| point_forward | (−1.5, 0.25, 0.0, 1.35; 2.0) |
| stop_sign | (−1.3, 0.25, 0.0, 0.0; 2.0) |
| raise_hand | (−2.6, 0.45, 0.0, 1.2; 2.0) |
| beckon | (−1.2, 0.3, 0.0, 0.6; 0), then elbow (0.0; 0), (0.6; 0) ×2 |

All gestures end with the engine's return to `base`. Envelope: `pose_margins`. Path check: the
real checker when configured, otherwise none, as for named poses.

- [ ] **Step 1: Write the failing tests:**

```python skip
@pytest.mark.parametrize("name", get_args(GestureName))
def test_each_gesture_plans_within_the_limits(name):
    plan = keyframe_plan(BASE, gesture_keyframes(name, BASE), split_every=1000 - 25 - 100)
    assert len(plan.poses) / RATE <= GESTURE_MAX_S
    assert all(len(retreat_poses(plan, s)) + 100 <= 1000 for s in range(len(plan.segments)))
    assert set(GESTURES) == set(get_args(GestureName))

@needs_scene
@pytest.mark.parametrize("name", get_args(GestureName))
def test_each_gesture_and_its_retreats_clear_the_body(name):
    checker = ArmCollisionChecker(SCENE)
    plan = keyframe_plan(ROBOT_IDLE, gesture_keyframes(name, ROBOT_IDLE))
    assert checker.first_contact(plan.poses) is None
    assert all(checker.first_contact(retreat_poses(plan, s), start=ROBOT_IDLE) is None for s in range(len(plan.segments)))

def test_gesture_from_a_raised_arm_returns_to_where_it_started(skill):
    # feeder starts with shoulder pitch at -1.0; assert the report's base pose and the last
    # streamed token match the raised start, not the nominal rest

def test_stop_mid_wave_reverses_one_swing_then_goes_home(skill):
    # leg leaves its envelope during the 4th swing; the retreat chunk is shorter than one
    # swing plus the way home, and handback is confirmed
```

  `ROBOT_IDLE` and `needs_scene` / `SCENE` reuse the existing real-model tests' constants
  (`test_robot_idle_pose_near_the_hip_does_not_block_moves_away`).

- [ ] **Step 2: Run them and verify they fail** (ImportError for `GestureName`).

- [ ] **Step 3: Implement** the table, `gesture_keyframes` and `perform_gesture`: validate
  `name`, refuse plans over `GESTURE_MAX_S` with `ValueError`, and `_launch` as in Task 3. Add
  the gestures and their durations to the prompt (wave ≈ 33 s at the current speed).

- [ ] **Step 4: Run and verify:** both test files PASS. If a gesture fails the path check,
  change its keyframes, not the checker.

- [ ] **Step 5: Commit**

```bash skip
git commit -am "feat(g1): perform_gesture with five keyframe gestures"
```

---

### Task 5: `ArmKinematics` and `point_at`

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_reference.py`, `sonic_arm_skill.py`,
  `system_prompt.py`
- Test: `test_sonic_arm_keyframes.py`, `test_sonic_arm_skill.py`

**Interfaces:**
- Produces, in `sonic_arm_reference.py`:
  ```python skip
  POINT_AZIMUTH_DEG = (-35.0, 110.0); POINT_ELEVATION_DEG = (-75.0, 75.0)
  POINT_ELBOW = 1.35  # rad, near straight
  class ArmKinematics:
      def __init__(self, scene: Path) -> None
      def shoulder(self, q: NDArray[Any]) -> NDArray[np.float64]  # [3] left_shoulder_pitch_link origin, torso_link frame
      def direction(self, q: NDArray[Any]) -> tuple[float, float]  # (az, el) deg of shoulder -> left_wrist_yaw_link
      def solve(self, q: NDArray[Any], azimuth_deg: float, elevation_deg: float) -> NDArray[np.float64]
          # [4] (pitch, roll, yaw=0, elbow=POINT_ELBOW); ValueError("... more than 3 deg off") when it can't
  ```
  These are built on the same `scene_43dof.xml` as `ArmCollisionChecker`. Frames come from MuJoCo
  body poses at `q`, using the same qpos layout as `ArmCollisionChecker._near`. `solve`
  searches shoulder pitch and roll within `JOINT_BOUNDS`. The method is the builder's (e.g. a
  coarse grid then a local refine); it must run under 0.5 s on the workstation.
- Produces, in `sonic_arm_skill.py`:
  `@skill(uses=[CAP_MOVEMENT], lifecycle="background") def point_at(self, azimuth_deg: float,
  elevation_deg: float, hold_seconds: float = 2.0) -> str`.
  - It needs `collision_model_path` (it builds `ArmKinematics` in `start()` beside the checker).
  - It raises `ValueError` outside the cone, and the `solve` error when unreachable.
  - It runs a one-keyframe plan through `_run`, with `pose_margins` and `require_check=True`.
  - It adds `"achieved_direction_deg"` (FK of the solved pose) to the report.

- [ ] **Step 1: Write the failing tests:**

```python skip
@needs_scene
@pytest.mark.parametrize("az", [-30.0, 0.0, 45.0, 100.0])
@pytest.mark.parametrize("el", [-70.0, -30.0, 0.0, 40.0, 70.0])
def test_solved_arm_points_where_asked(az, el):
    kin = ArmKinematics(SCENE)
    q = ROBOT_IDLE.copy(); q[[15, 16, 17, 18]] = kin.solve(ROBOT_IDLE, az, el)
    got = kin.direction(q)
    assert angle_between(got, (az, el)) <= 3.0  # test helper: great-circle angle of two (az, el) pairs, deg

@needs_scene
def test_cone_grid_is_reachable_and_clear_of_the_body():
    # 7 x 7 grid over the cone: every solve succeeds and its stroke_plan passes first_contact.
    # Where it fails, narrow POINT_* (and the spec's numbers in the prompt) rather than loosen
    # the checker, and record the new cone in the commit message.

@pytest.mark.parametrize("az,el", [(-40.0, 0.0), (0.0, 80.0), (120.0, 0.0)])
def test_point_at_outside_the_cone_sends_nothing(skill, az, el):
    with pytest.raises(ValueError, match="cone"):
        skill[0].point_at(az, el)
```

- [ ] **Step 2: Run them and verify they fail** (ImportError `ArmKinematics`).

- [ ] **Step 3: Implement** `ArmKinematics` and `point_at`, and add `point_at` with its cone to
  the prompt.

- [ ] **Step 4: Run and verify:** both files PASS, and `uv run mypy
  dimos/robot/unitree/g1/sonic_arm_reference.py dimos/robot/unitree/g1/sonic_arm_skill.py` is
  clean.

- [ ] **Step 5: Commit**

```bash skip
git commit -am "feat(g1): point_at with MuJoCo kinematics for the left arm"
```

---

### Task 6: Head-camera geometry and `point_at_object`

**Files:**
- Modify: `sonic_arm_reference.py`, `sonic_arm_skill.py`, `system_prompt.py`,
  `blueprints/basic/unitree_g1_sonic_arm.py`
- Test: `test_sonic_arm_keyframes.py`, `test_sonic_arm_skill.py`

**Interfaces:**
- Produces, in `sonic_arm_reference.py`; the blueprint imports these instead of its own
  copies:
  ```python skip
  HEAD_CAMERA_K = (606.62, 607.12, 325.54, 245.35)  # fx, fy, cx, cy at 640x480
  HEAD_CAMERA_XYZ = (0.0576235, 0.01753, 0.41987)    # torso_link -> camera_link
  HEAD_CAMERA_QUAT_XYZW = (0.0, 0.40355, 0.0, 0.91496)
  OBJECT_RANGE_M = 2.0
  def pixel_ray(u: float, v: float) -> NDArray[np.float64]  # unit vector, torso_link frame
  def aim_from_pixel(u: float, v: float, shoulder: NDArray[Any]) -> tuple[float, float]  # (az, el) deg
  ```
  - Optical to `camera_link`: `link = (z, -x, -y)` of the optical ray `((u-cx)/fx, (v-cy)/fy, 1)`.
  - The aim point is `HEAD_CAMERA_XYZ + OBJECT_RANGE_M * ray`, seen from `shoulder`.
- Produces, in `sonic_arm_skill.py`:
  - config `vl_model: str | None = None`. When `None`, `point_at_object` refuses with
    "point_at_object is disabled: no vl_model configured".
  - input `color_image: In[Image]`;
  - `@skill(uses=[CAP_MOVEMENT], lifecycle="background") def point_at_object(self,
    description: str, hold_seconds: float = 2.0) -> str`.
  - The VL client is `OpenAIVlModel(model_name=self.config.vl_model)` with no `auto_resize`,
    built in `start()`. It reaches the gateway through `OPENAI_BASE_URL` / `OPENAI_API_KEY` in
    the environment.
  - Module-level `query_boxes(vl: VlModel, image: Image, query: str, box_scale:
    Literal["pixels", "normalized_1000"] = "pixels") -> list[tuple[str, float, float, float,
    float]]`:
    - it calls `vl.query_json(image, BOX_PROMPT.format(query=query))`, a prompt asking for
      `[["label", x1, y1, x2, y2], ...]` or `[]`, and lets exceptions propagate;
    - it converts raw coordinates first: ×width/1000 and ×height/1000 for `normalized_1000`;
    - only then does it clip to the image, dropping malformed entries and boxes with zero area
      after clipping.
    - Item 7: `query_detections` validates bounds before any conversion, so it would drop a
      valid normalized box such as (700, 600, 900, 900) on 640×480.
    - `demo_vl_grounding.py` imports and uses this same function.
- Flow:
  1. Get a frame with `color_image.get_next(timeout=5.0)`; a timeout raises "no camera frame".
  2. Call `query_boxes` and pick the largest box by area.
     - An exception raises `RuntimeError(f"vision model {vl_model} failed: {exc}; nothing
       sent")`.
  3. With no box, raise `RuntimeError(f"found no '{description}' in the camera image; nothing
     sent")`.
  4. Aim from the box centre, then check the cone: outside it, raise `RuntimeError` with the
     direction.
  5. Otherwise run the `point_at` path, adding `label`, `box_px` and `direction_deg` to the
     report.
  - Steps 1–4 run inside `_launch`'s work function, because a VL call can take seconds.
- **Coordinates:** config `vl_box_scale: Literal["pixels", "normalized_1000"] = "pixels"`,
  passed to `query_boxes`. Task 8 sets it with the chosen model.

- [ ] **Step 1: Write the failing tests:**

```python skip
def test_image_centre_looks_down_the_camera_axis():
    ray = pixel_ray(325.54, 245.35)
    np.testing.assert_allclose(ray, [np.cos(0.8306), 0.0, -np.sin(0.8306)], atol=2e-3)  # 47.6 deg down

def test_pixel_left_and_up_turn_left_and_up():
    c = pixel_ray(325.54, 245.35)
    assert pixel_ray(100.0, 245.35)[1] > c[1]  # left of centre -> +y (robot's left)
    assert pixel_ray(325.54, 50.0)[2] > c[2]   # above centre -> higher

def test_aim_from_pixel_points_the_shoulder_at_the_object_2m_out():
    shoulder = np.array([0.0, 0.15, 0.24])
    az, el = aim_from_pixel(325.54, 245.35, shoulder)
    p = np.array(HEAD_CAMERA_XYZ) + 2.0 * pixel_ray(325.54, 245.35) - shoulder
    assert (az, el) == pytest.approx((np.degrees(np.arctan2(p[1], p[0])), np.degrees(np.arctan2(p[2], np.hypot(p[0], p[1])))))

def test_point_at_object_with_no_match_says_so_and_sends_nothing(skill, mocker):
    # fake VL query_json returns []; assert "found no 'red box'" and no pose packets

def test_point_at_object_vision_failure_is_not_reported_as_not_found(skill, mocker):
    # fake VL query_json raises; assert "vision model <name> failed" and no pose packets

def test_normalized_boxes_convert_before_bounds_checks():
    vl = FakeVl([["box", 700, 600, 900, 900]])
    (label, x1, y1, x2, y2), = query_boxes(vl, IMAGE_640x480, "box", "normalized_1000")
    assert (x1, y1, x2, y2) == pytest.approx((448, 288, 576, 432))
    assert query_boxes(vl, IMAGE_640x480, "box", "pixels") == []  # read as pixels it lies off-image: zero area after clipping

def test_point_at_object_points_at_the_largest_box(skill, mocker):
    # two fake boxes; assert the report's box_px is the larger one and direction_deg matches aim_from_pixel

def test_point_at_object_without_a_model_is_disabled(skill):
    with pytest.raises(RuntimeError, match="disabled"):
        skill[0].point_at_object("red box")
```

- [ ] **Step 2: Run them and verify they fail.**

- [ ] **Step 3: Implement** the camera constants and functions, switch the blueprint to the
  shared constants, and add the `color_image` input (autoconnect wires it to `CameraModule`, as
  for `ObserveSkill`). Implement `point_at_object` and add it to the prompt, with its limitation:
  "the head camera looks at the floor 0.3–2 m ahead; objects above about waist height may be out
  of view".

- [ ] **Step 4: Run and verify:** both files PASS. Also run
  `uv run pytest dimos/robot/test_all_blueprints_generation.py`; expected PASS with
  `all_blueprints.py` unchanged, since no blueprint names change.

- [ ] **Step 5: Commit**

```bash skip
git commit -am "feat(g1): point_at_object from a VL box on the head camera"
```

---

### Task 7: MuJoCo stand-in in the repo and end-to-end tests

**Files:**
- Create: `dimos/robot/unitree/g1/sonic_sim_peer.py`
- Modify: `sonic_arm_reference.py` (`SonicEncoder.encode(..., anchor: NDArray[Any] | None = None)`)
- Create: `dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py`
- Modify: `docs/artifacts/g1_pc2_223_sync_20261014/sonic_sim_peer.py` to a 5-line wrapper that
  calls the new module's `main()`. This keeps the dual-G1 session's commands working.

**Interfaces:**
- `python -m dimos.robot.unitree.g1.sonic_sim_peer --feedback ADDR --reference ADDR
  --model-dir DIR --scene XML [--name N] [--log NPZ]`: the same behaviour, protocol and npz keys
  as the artifact.
- `startup_on_floor(robot, dec, planner_token_of) -> tuple[NDArray, dict]`, copied verbatim.
- **Port with attribution** (Apache-2.0 header plus a "ported from" line for each source):
  - lerobot `examples/g1_dex3_training/sonic_roundtrip_audit.py`: `Robot` (lines ~62–184),
    `gravity`, `Decoder` (~198–235), and the constants they use. Drop `startup_hang` and
    everything after `Decoder`.
  - the artifact `arm_reference.py`: `IdlePlanner`, `anchor6`, and the `planner_final`
    context-building from `demo_sim_arm_reference.py`.
- **Encoder:** use DimOS `SonicEncoder` with the new `anchor` parameter; the default stays the
  identity anchor, so outputs are unchanged.

**E2E test:**
- Marked `pytest.mark.native_e2e`.
- Skipped unless the HF GEAR-SONIC snapshot and the scene exist (paths from env
  `GEAR_SONIC_MODEL_DIR` / `GEAR_SONIC_SCENE`).
- Starts the peer as a subprocess on free ports, then builds `SonicExecutionModule` and
  `SonicArmSkillContainer` in-process against it, with the blueprint's profile and `max_dq=2.0`.
  It uses the real `_launch`, so `_lifecycle`, `_running`, `_stop` and the thread behave as on the robot.
  - Each case calls the skill, then waits up to 120 s for its final `tool_update` (spy) and parses
    the `SkillResult` JSON.
  - Normal cases assert the message reports success (no "stopped early", no "failed"),
    `reached_fraction >= 0.8` and confirmed handback (the message ends "control returned to
    SONIC's planner").
  - The stop case calls `stop_arm()` 60% of the way through the wave's planned duration and
    asserts all of these:
    - the call returns "Stopping…";
    - the final message contains "stopped early: stop requested";
    - the execution journal shows a second submitted chunk (the retreat);
    - handback is confirmed;
    - the arm is back within 0.05 rad of its start pose before the wave's planned end time.
- Asserts from the peer's npz after each run:
  - tilt ≤ 3°;
  - floor contacts ≥ 7 throughout;
  - final left-arm pose within 0.05 rad of the start;
  - no dq above 2.0 rad/s.
- Cases:
  - every gesture;
  - `point_at` at (0, 0), (45, −30), (90, 20);
  - `move_left_arm_pose("left_hand_over_head", hold_seconds=10)`;
  - `stop_arm` 60% through a wave.

- [ ] **Step 1: Write the e2e test file** with the cases above. Run it:
  `uv run pytest -m native_e2e dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py -v`.
  Expected: FAIL / ImportError (no `dimos.robot.unitree.g1.sonic_sim_peer`).

- [ ] **Step 2: Port the stand-in** and add the encoder anchor. Check that the startup still
  ends standing: run the module alone for 10 s and confirm the printed `standing:` info shows
  `floor_contacts >= 7` and a pelvis z near 0.79.

- [ ] **Step 3: Run the e2e tests and verify they pass.** Tune gesture keyframes (Task 4 table)
  only within their intent when a case fails. Record per-case peak dq, tilt and duration in the
  test output (`-s`).

- [ ] **Step 4: Check that the default run excludes them:** `uv run pytest
  dimos/robot/unitree/g1 -q` shows them deselected. Expected: PASS.

- [ ] **Step 5: Commit**

```bash skip
git add dimos/robot/unitree/g1/sonic_sim_peer.py dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py dimos/robot/unitree/g1/sonic_arm_reference.py
git commit -m "test(g1): MuJoCo stand-in for the guarded SONIC controller and arm-skill e2e runs"
```

---

### Task 8: VL grounding measurement

**Files:**
- Create: `dimos/robot/unitree/g1/demo_vl_grounding.py`

**Interfaces:** `python dimos/robot/unitree/g1/demo_vl_grounding.py --frames DIR --model NAME
[--box-scale pixels|normalized_1000]`.
- `DIR` holds `*.png` head-camera frames and `labels.json`:
  `{"frame.png": {"query": "red box", "box": [x1, y1, x2, y2]}}`.
- It prints, per frame, the chosen box, its centre error in px and pass/fail at 30 px, then the
  pass count.
- The key is read from `.env.genon` (only the `GENON_API_KEY` line) into `OPENAI_API_KEY`, with
  `OPENAI_BASE_URL=https://api.genon.ai/v1`. It never prints the key.

- [ ] **Step 1: Write the script** using `OpenAIVlModel` and `sonic_arm_skill.query_boxes`
  (passing `--box-scale`), with the same largest-box rule as `point_at_object`.
- [ ] **Step 2: Smoke-run** it on the three unlabelled `camera-check/` frames, with hand-made
  labels, against `openai/gpt-5.6-luna`. Expected: it runs end to end and prints three rows.
- [ ] **Step 3: When the 10 robot frames exist (first robot session), run each candidate.**
  Candidates are `openai/gpt-5.6-luna` and any Qwen VL model the gateway lists. Pass bar:
  ≥ 8/10 within 30 px. Set `vl_model` in the blueprint to the winner. If none passes, leave
  `vl_model=None` (disabled) and record the numbers in the sync notes.
- [ ] **Step 4: Commit** the script (not the frames or the key).

---

### Task 9: pc2 onto this branch, then robot verification (operator-run)

Not code. It runs when pc2 is powered and the operator is present.

- [ ] **Step 1:** Back up pc2's current DimOS file set (the manifest in
  `docs/artifacts/g1_pc2_223_sync_20261014/dimos-manifest.json` lists it) to a dated directory
  on pc2.
- [ ] **Step 2:** Install this branch's files on pc2 (the sync guide's method) and run
  `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py
  dimos/robot/unitree/g1/test_sonic_arm_keyframes.py` there. Expected: all PASS. Then confirm
  `python -c "import mujoco, openai; from dimos.models.vl.openai import OpenAIVlModel"` works.
- [ ] **Step 3 (operator, robot on the stand, controller started by the operator):** at
  0.47 rad/s, run each gesture; then `stop_arm` mid-wave; then `point_at` at (0, 0) and
  (45, −30). Capture the 10 head-camera frames for Task 8, with objects 0.5–2 m ahead on the
  floor and on a low table.
- [ ] **Step 4:** Review each run's journal and the result metadata, with the same checks as the
  e2e asserts. Write up the results in the sync notes (not committed).

---

## Notes for the executor

- **Ordering:** Tasks 1→2→3 are sequential (each changes the same container paths). 4, 5 and 6
  depend on 3. 7 depends on 4–6 for its cases but can start its port after 3. Task 8's script
  can be written any time; its measurement waits on Task 9.
- **Dual-G1 session:** it builds its dual container on this engine and `GESTURES`. Its design's
  `mcp_timeout=60` no longer matters for these skills, because they return at once.
