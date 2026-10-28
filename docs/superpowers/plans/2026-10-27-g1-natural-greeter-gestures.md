# G1 223 Natural Greeter Gestures Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add four ARDY-generated gestures (`wave_natural`, `invite_natural`, `take_right_natural`,
`entrance_right_natural`) that SonicArm plays as one continuous 50 Hz clip with a per-clip arm speed
limit, plus a greeter `--natural` flag.

**Architecture:** Each clip is a 14-column arm CSV under `gestures/clips/`, made offline from
ARDY's qpos by a tool script. `keyframe_plan` gains a verbatim *clip segment* (`Keyframe.path`): a
slow min-jerk approach to the clip's first row, the clip rows as they are, then the usual slow way
home. A stop inside the clip continues the plan's own tail, so nothing extra is encoded. SonicArm
widens the envelope by the planned rows, raises the arm joints' overspeed limit only during the clip
window, and cancels to the planner only on an overspeed mid-clip (user, 2026-10-27: an envelope
trip mid-clip finishes the clip like a stop).

**Tech Stack:** Python 3.12, numpy, scipy (`CubicSpline`, already a dependency), pytest, MuJoCo
stand-in (`test_sonic_arm_sim_e2e.py`).

**Spec:** `docs/superpowers/specs/2026-10-27-g1-natural-greeter-gestures-design.md`

## Global Constraints

- Worktree `/home/jihun/work/dimos-natural`, branch `jihun/feat/g1-natural-gestures` from fork/main 2adb8ddc0. Upstream is unset on purpose; push only with `git push fork HEAD:refs/heads/jihun/feat/g1-natural-gestures`, and only when the user asks.
- **No commits** unless the user asks. A "Commit" step below means: `git add` the files and report. Never add Co-Authored-By trailers.
- Existing gestures, `max_dq` (operator value, `le=5.0` validation), `PEAK_SPEED` 0.75, leg envelopes and the path check are unchanged for every non-clip motion and for the approach, way home and retreats.
- Clip speeds (the user's): wave_01 0.7x, invite_03 0.8x, entrance_right_01 0.8x, take_constrained_01 0.8x.
- Clip speed limit: arm motors 15-28 only, `min(CLIP_DQ_CEILING, CLIP_DQ_FACTOR * planned arm peak)`, never below `config.max_dq`; legs and waist stay at `config.max_dq`. Starting values `CLIP_DQ_FACTOR = 1.2`, `CLIP_DQ_CEILING = 7.5` rad/s. A clip whose planned arm peak exceeds the ceiling is refused ("nothing sent").
- Stop mid-clip (user's option A): the clip plays to its end, then the slow way home; the voice line is cut at once. An envelope trip mid-clip finishes the clip the same way (user, 2026-10-27, after the stand-in); only an overspeed mid-clip cancels the task at once.
- mypy strict, imports at the top, `uv run` / `.venv` for tests; `sonic_arm_skill.py` / `sonic_arm_reference.py` are shared: tell dimos-dev-84 (`SendMessage`) before Task 2 starts editing them.
- The MuJoCo stand-in is shared: run it pinned with `taskset -c 8-15`, after checking no other session is running a sim.

## Review Focus

1. A stop that lands in the **approach** (before the clip) must reverse the approach at the normal limit, not inherit the clip's raised limit: covered by `test_stop_in_the_approach_reverses_without_the_clip_limit` (Task 4).
2. A stop landing in the **last rows** of the clip or just after it (tail window) must not double-submit or cancel: covered by `test_stop_after_the_clip_goes_home_as_today` (Task 4).
3. Leg or waist speed above `max_dq` **during** the clip must still trip: covered by `test_leg_speed_mid_clip_still_trips_at_max_dq` (Task 4).
4. A clip CSV with a value outside `UB_LIMITS`, a wrong header, NaN or under 2 rows must refuse at load, not at the robot: covered by `test_clip_refusals` (Task 1).
5. The measured start pose differing from the clip's first row (223's idle arms vs ARDY's hanging arms) must still produce a smooth approach and a path check of every row: covered by `test_clip_plan_approaches_plays_verbatim_and_returns` (Task 2) and `test_clip_gesture_path_checks_every_row` (Task 3).

---

### Task 1: Clip converter, clip data and loader

**Files:**
- Create: `dimos/robot/unitree/g1/tool_ardy_clip.py`
- Create: `dimos/robot/unitree/g1/gestures/clips/{wave_natural,invite_natural,take_right_natural,entrance_right_natural}.csv`
- Modify: `dimos/robot/unitree/g1/imported_gestures.py` (constants + `load_clips`)
- Modify: `dimos/robot/unitree/g1/gestures/SOURCE.md` (new section "ARDY clips")
- Test: `dimos/robot/unitree/g1/test_imported_gestures.py`

**Interfaces:**
- Produces (`tool_ardy_clip.py`): `convert(qpos: NDArray[np.float64], speed: float, src_fps: float = 25.0) -> NDArray[np.float64]` returning `[M, 14]` arm rows (motors 15-28) at `RATE` (50 Hz); constants `STILL_DQ = 0.05` (rad/s), `SETTLE_S = 0.5`. CLI: `python -m dimos.robot.unitree.g1.tool_ardy_clip IN.csv SPEED OUT.csv`.
- Produces (`imported_gestures.py`): `CLIP_GESTURES: tuple[str, ...] = ("wave_natural", "invite_natural", "take_right_natural", "entrance_right_natural")`, `CLIP_DIR = DATA_DIR / "clips"`, `CLIP_COLUMNS = MOTOR_NAMES[15:29]`, `parse_clip(text: str, source: str = "") -> NDArray[np.float64]` (`[M, 14]`), `load_clips() -> dict[str, NDArray[np.float64]]`.

- [ ] **Step 1: Write the failing tests** in `test_imported_gestures.py`

```python
def test_convert_resamples_slows_and_picks_the_arm_columns() -> None:
    t = np.arange(150) / 25
    qpos = np.zeros((150, 36))
    qpos[:, 7 + 15] = np.where((t > 1) & (t < 3), 0.3 * np.sin(2 * np.pi * (t - 1)), 0.0)
    qpos[:, 7 + 0] = 1.0  # a leg column: never copied
    arms = tool_ardy_clip.convert(qpos, 0.5)
    assert arms.shape[1] == 14 and np.all(arms[:, 1:] == 0)
    moving_s = np.count_nonzero(np.abs(np.diff(arms[:, 0])) * RATE > 0.05) / RATE
    assert moving_s == pytest.approx(4.0, abs=0.1)  # 2 s of motion at 0.5x
    assert len(arms) / RATE == pytest.approx(4.0 + 2 * tool_ardy_clip.SETTLE_S, abs=0.1)  # trimmed

def test_every_clip_gesture_loads_at_50_hz_within_limits() -> None:
    clips = gestures.load_clips()
    assert set(clips) == set(gestures.CLIP_GESTURES)
    for name, rows in clips.items():
        assert rows.shape[1] == 14 and len(rows) > RATE, name
        for j, joint in enumerate(gestures.CLIP_COLUMNS):
            low, high = gestures.UB_LIMITS[joint]
            assert low <= rows[:, j].min() and rows[:, j].max() <= high, (name, joint)

@pytest.mark.parametrize("bad", ["header", "limit", "nan", "short"])
def test_clip_refusals(bad: str) -> None:
    ...  # build a valid 3-row CSV text from CLIP_COLUMNS, then break it per `bad`
    with pytest.raises(ValueError, match=...):  # header: "columns"; limit: the joint name; nan: "finite"; short: "rows"
        gestures.parse_clip(text, "synthetic")
```

Also extend `test_source_md_complete` so every `CLIP_GESTURES` name appears in SOURCE.md.

- [ ] **Step 2: Run** `uv run pytest dimos/robot/unitree/g1/test_imported_gestures.py -v`. Expected: the new tests FAIL (module/attributes missing).

- [ ] **Step 3: Implement `convert` in `tool_ardy_clip.py`**: arms = `qpos[:, 7 + 15 : 7 + 29]`; source times `arange(n) / src_fps`; output times `arange(0, t[-1] / speed, 1 / RATE) * speed` through `scipy.interpolate.CubicSpline` per column; trim to the rows from `SETTLE_S` before the first row whose arm `|diff| * RATE > STILL_DQ` to `SETTLE_S` after the last one, clipped to the clip. The CLI reads ARDY's headerless CSV (`np.loadtxt(..., delimiter=",")`), writes the header line `",".join(CLIP_COLUMNS)` and rows with `%.6f`. The module docstring says it is the offline tool behind `gestures/clips/` and names SOURCE.md.

- [ ] **Step 4: Implement `parse_clip` / `load_clips`** in `imported_gestures.py`: the header must equal `CLIP_COLUMNS` in order ("columns" in the error); every value finite ("finite"); at least 2 rows ("rows"); each column inside `UB_LIMITS` (message names the joint, the value and the limit; reuse `_check_limit`). `load_clips` reads `CLIP_DIR / f"{name}.csv"` for each name in `CLIP_GESTURES`.

- [ ] **Step 5: Generate the four clip files** from the user's picks:

```bash
R=~/work/g1-demo-gestures/ardy/raw; C=dimos/robot/unitree/g1/gestures/clips; mkdir -p $C
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/wave_01.csv 0.7 $C/wave_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/invite_03.csv 0.8 $C/invite_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/take_constrained_01.csv 0.8 $C/take_right_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/entrance_right_01.csv 0.8 $C/entrance_right_natural.csv
```

Print each file's duration and planned arm peak `|diff| * RATE`; the wave's should be about 6 rad/s. Report the four numbers.

- [ ] **Step 6: Write SOURCE.md "ARDY clips"**: ARDY repo and commit `8163bbf`, model `ARDY-G1-RP-25FPS-Horizon52`, 6.0 s, `--num_samples 4 --seed 0`. Then a table of gesture, sample, prompt (copied verbatim from `~/work/g1-demo-gestures/ardy/README.md`) and speed. Add take_right's constraint (`raw/take_right_constraint.json`, right hand and hips pinned on frames 40/50/60/70 to `take_right.json`'s arm pose), that the raw CSVs and generation scripts live in `~/work/g1-demo-gestures/ardy/` (not committed), and the regenerate commands from Step 5.

- [ ] **Step 7: Run** `uv run pytest dimos/robot/unitree/g1/test_imported_gestures.py -v`. Expected: all pass.

- [ ] **Step 8: Stage** `tool_ardy_clip.py`, `gestures/clips/`, `imported_gestures.py`, `SOURCE.md` and the test file, then report.

---

### Task 2: Verbatim clip segments in `keyframe_plan`

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_reference.py` (`Keyframe`, `Segment`, `keyframe_plan`, `_retreat_rows`, `retreat_route`)
- Test: `dimos/robot/unitree/g1/test_sonic_arm_keyframes.py`

**Interfaces:**
- Produces: `Keyframe.path: NDArray[np.float64] | None = None`. These are `[M, 29]` rows played verbatim at `RATE` as the segment arriving at this keyframe. `path[0]` must equal the previous target (the base for the first keyframe) and `path[-1]` must equal `target`; otherwise `ValueError("clip path must start at the previous target and end at its own")`. `move_s`, `peak_speed` and `min_s` do not apply, and the hands hold the previous target.
- Produces: `Segment.clip: bool = False`. It is True for a path segment.
- Produces: `retreat_route` for a row inside a clip segment returns `(s, row - seg.start)`, and `_retreat_rows(plan, s)` for a clip segment returns the plan's tail `rows[seg.start:]`: the rest of the clip, the way home and the lead-out. So `prepare`'s `is_tail` is True and nothing extra is encoded.

- [ ] **Step 1: Write the failing tests**

```python
def _clip_keyframes(base, path):  # approach to path[0], then the clip verbatim
    return [Keyframe(path[0]), Keyframe(path[-1], path=path)]

def test_clip_plan_approaches_plays_verbatim_and_returns():
    base = np.zeros(29); base[15] = 0.3  # the start differs from the clip's first row
    path = np.zeros((100, 29)); path[:, 17] = 0.4 * np.sin(np.linspace(0, 4 * np.pi, 100))
    plan = keyframe_plan(base, _clip_keyframes(base, path), move_s=1.0, return_min_s=1.0)
    approach, clip, home = plan.segments
    assert not approach.clip and clip.clip and not home.clip
    assert np.array_equal(plan.poses[clip.start:clip.end], path)
    for seg in (approach, home):  # the slow strokes keep the peak speed
        assert np.abs(np.diff(plan.poses[seg.start:seg.end], axis=0)).max() * RATE <= PEAK_SPEED + 1e-6

def test_a_stop_inside_a_clip_continues_the_plan_tail():
    ...  # same plan; for row = clip.start + 30:
    assert retreat_route(plan, row) == (1, 30)
    assert np.array_equal(retreat_poses(plan, 1), plan.poses[clip.start:])

def test_a_stop_in_the_approach_still_reverses_it():
    ...  # row inside the approach: retreat_route returns (0, approach.end - 1 - row) as today

def test_a_clip_path_must_join_its_neighbours():
    ...  # path[0] != previous target -> ValueError match "clip path must start"
```

- [ ] **Step 2: Run** `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_keyframes.py -v`. Expected: the new tests FAIL, the old ones pass.

- [ ] **Step 3: Implement** in `sonic_arm_reference.py`: in `keyframe_plan`'s segment loop, a keyframe with `path` uses the path (with the held hand target in its hand columns) instead of `_segment_poses`, after the join check, and records `Segment(..., clip=True)`. The `split_every` "segment longer than a session chunk" check applies to it unchanged. Then `retreat_route` and `_retreat_rows` behave as in the Interfaces block. Add a line on `clip` to the `Keyframe`/`Segment` docstrings.

- [ ] **Step 4: Run** the whole keyframe file. Expected: all pass, the existing retreat tests unchanged.

- [ ] **Step 5: Stage** both files and report.

---

### Task 3: Clip gestures in the gesture table and `prepare`

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py`. Add the constants near `GESTURE_MAX_S`, the `GestureName` literal, `_gesture_table`, `SonicArm.prepare`, `_Watch` and the `perform_gesture` docstring.
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Interfaces:**
- Consumes: `load_clips`, `CLIP_GESTURES` (Task 1); `Keyframe.path`, `Segment.clip` (Task 2).
- Produces: constants `CLIP_MOVE_S = 1.0` (the approach's and the return's minimum, s), `CLIP_DQ_FACTOR = 1.2`, `CLIP_DQ_CEILING = 7.5` (rad/s), `CLIP_DQ_TAIL_S = 0.5` (how long the clip limit outlasts the clip's last row, for tracking lag); `ARM_MOTORS = (*LEFT_ARM, *RIGHT_ARM)`.
- Produces: clip `GestureSpec`s: `keyframes_of(base, base_hands)` returns `[Keyframe(rows[0]), Keyframe(rows[-1], path=rows)]` where `rows = np.tile(base, (M, 1))` with `rows[:, 15:29] = clip`; `arms=("left", "right")`, `envelope="two_arm"`, `move_s=CLIP_MOVE_S`, `return_min_s=CLIP_MOVE_S`.
- Produces: `_Watch.fast_rows: tuple[int, int] | None = None` (token-plan rows `[pad + clip.start, pad + clip.end)`), `_Watch.fast_max_dq: float = 0.0`, `_Watch.fast_window: tuple[float, float] | None = None` (monotonic, set in Task 4).

- [ ] **Step 1: Write the failing tests** (use `skill`, `FakeChecker`, `call_with_feed` and `mocker.spy(sonic_arm_skill, "_Watch")` as `test_waist_envelope_includes_the_planned_waist` does)

```python
@pytest.mark.parametrize("name", CLIP_GESTURES)
def test_clip_gesture_plans_its_clip_with_a_raised_arm_limit(skill, mocker, name):
    module, _ = skill
    module._arm._both_checker = FakeChecker()
    watch = mocker.spy(sonic_arm_skill, "_Watch")
    call_with_feed(module, lambda: module.perform_gesture(name), lambda t: np.zeros(29))
    kw = watch.call_args.kwargs
    clip = load_clips()[name]
    peak = np.abs(np.diff(clip, axis=0)).max() * RATE
    assert kw["fast_max_dq"] == pytest.approx(max(module.config.max_dq, min(CLIP_DQ_CEILING, CLIP_DQ_FACTOR * peak)))
    assert kw["fast_rows"][1] - kw["fast_rows"][0] == len(clip)
    # the envelope covers the clip's swing on every arm joint
    assert np.all(kw["allowed"][15:29] >= np.abs(clip - 0.0).max(axis=0) - 1e-9)

def test_clip_gesture_path_checks_every_row(skill, mocker): ...      # FakeChecker sees len(plan.poses) rows incl. the clip
def test_a_clip_faster_than_the_ceiling_is_refused(skill, mocker): ...  # patch a clip x3 speed -> "nothing sent", no start_task
def test_keyframe_gestures_keep_their_envelope_and_limit(skill, mocker): ...  # "wave", "invite": allowed unchanged vs before, fast_rows None
```

- [ ] **Step 2: Run** `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py -k "clip or keep_their" -v`. Expected: FAIL.

- [ ] **Step 3: Implement.**
  - Add the four names to `GestureName`. Add the clip specs in `_gesture_table`.
  - In `prepare`, widen the envelope with `allowed + np.abs(kplan.poses - base).max(axis=0)` instead of the keyframe deltas.
  - Take `delta` from the planned pose row farthest from `base` (`argmax` of the row norm). For min-jerk plans this is the deepest keyframe, so the existing reports don't change. Keep `deepest` (the keyframe index) for `hold` and `hand_target`, as today. For a clip, the two keyframes have no hold between them, so `hold_rows` comes out `None`.
  - For a plan with a clip segment:
    - The planned arm peak is `np.abs(np.diff(kplan.poses[clip.start:clip.end, 15:29], axis=0)).max() * RATE`.
    - Refuse with `RuntimeError(f"{label} clip peaks at {peak:.1f} rad/s, over the {CLIP_DQ_CEILING:g} rad/s ceiling; nothing sent")` when it exceeds `CLIP_DQ_CEILING`.
    - Otherwise set `fast_max_dq = max(self.config.max_dq, min(CLIP_DQ_CEILING, CLIP_DQ_FACTOR * peak))` and `fast_rows = (pad + clip.start, pad + clip.end)` on the `_Watch`.
  - Update `perform_gesture`'s docstring with the four names: 223 demo, ARDY human motion, both arms, played at human speed, about 7-10 s.

- [ ] **Step 4: Run** `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py dimos/robot/unitree/g1/test_sonic_arm_keyframes.py -v`. Expected: all pass, including every existing gesture test.

- [ ] **Step 5: Stage and report.**

---

### Task 4: Clip-phase speed limit, mid-clip stop and mid-clip abort

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_arm_skill.py` (`SonicArm.handle_feedback`, `SonicArm._stream`, `_execute`'s message)
- Test: `dimos/robot/unitree/g1/test_sonic_arm_skill.py`

**Interfaces:**
- Consumes: `_Watch.fast_rows`, `fast_max_dq`, `fast_window`, `ARM_MOTORS`, `CLIP_DQ_TAIL_S` (Task 3).
- Behaviour:
  - In `_stream`, at `plan_start` and under the lock: `watch.fast_window = (plan_start + fast_rows[0] / RATE, plan_start + fast_rows[1] / RATE + CLIP_DQ_TAIL_S)`.
  - In `handle_feedback`, the per-joint limit is `max_dq`. For a sample whose `received_monotonic` falls in `fast_window`, the `ARM_MOTORS` limit is `fast_max_dq` instead.
  - Both `overspeed_at` and the speed abort use that limit. The abort message names the joint, its speed and its limit.
  - In `_stream`, when an abort is first seen inside `fast_window` and it is not a stop (`not self.stop.is_set()`): cancel the task and return `"envelope"` at once.
  - When a stop or abort retreat starts before `fast_window[0]`, set `watch.fast_window = None` before sending it.
  - A stop inside the window sends the plan-tail retreat from Task 2, so the clip plays on. `_execute` then reports `"{label} stopped early: stop requested (finished the clip, then returned)"`.

- [ ] **Step 1: Write the failing tests** (use the patterns of `test_overspeed_during_the_way_back_hands_back_at_once` and `test_stop_mid_motion_retreats_smoothly_and_hands_back`; feed `body_dq` through `handle_feedback`; shorten a clip with `mocker.patch.dict` on `GESTURES` if timing needs it)

```python
def test_arm_speed_inside_the_clip_window_is_allowed_up_to_the_clip_limit(skill, mocker): ...
    # left_shoulder_yaw dq 5.0 in the window: result "...returned", status outcome "finished"
def test_arm_speed_over_the_clip_limit_mid_clip_cancels_at_once(skill, mocker): ...
    # dq 9.0 in the window: "stopped early" + "rad/s >", status outcome "cancelled", no retreat chunk submitted
def test_leg_speed_mid_clip_still_trips_at_max_dq(skill, mocker): ...
    # left_knee dq 2.5 (> max_dq 2.0 in the fixture config? set config.max_dq=2.0) in the window: cancelled
def test_arm_speed_over_max_dq_outside_the_clip_still_trips(skill, mocker): ...
    # dq 3.0 during the approach (before the window): "stopped early", smooth retreat as today
def test_stop_mid_clip_finishes_the_clip_then_returns(real_launch_skill, mocker): ...
    # stop_arm inside the window: the retreat chunk's rows equal the plan's remaining rows (tail),
    # message contains "finished the clip, then returned", hush() called at the stop, outcome finished
def test_stop_in_the_approach_reverses_without_the_clip_limit(real_launch_skill, mocker): ...
    # stop before the window: watch.fast_window is None afterwards; arm dq 3.0 on the way back cancels
def test_stop_after_the_clip_goes_home_as_today(real_launch_skill, mocker): ...
    # stop during the way home: retreat(...) is None, one finish, no cancel
```

- [ ] **Step 2: Run** `uv run pytest dimos/robot/unitree/g1/test_sonic_arm_skill.py -k "clip or approach" -v`. Expected: FAIL.

- [ ] **Step 3: Implement** the behaviour above. Keep `stream_dq` / `return_dq` as the raw peak |dq| (the report stays comparable). Take `_execute`'s clip-stop wording from whether the stop's retreat started inside `fast_window` (record it on `_Watch` as `stopped_in_clip: bool = False`).

- [ ] **Step 4: Run** the full g1 unit suite: `uv run pytest dimos/robot/unitree/g1 -v -m "not native_e2e"`. Expected: all pass. Then `uv run mypy dimos/robot/unitree/g1`. Expected: no errors.

- [ ] **Step 5: Stage and report.**

---

### Task 5: Dual container refuses the clip gestures

**Files:**
- Modify: `dimos/robot/unitree/g1/sonic_dual_arm_skill.py` (`perform_gesture`)
- Test: `dimos/robot/unitree/g1/test_sonic_dual_arm_skill.py`

- [ ] **Step 1: Write the failing test** `test_clip_gestures_are_refused_on_the_dual_container`: for each `CLIP_GESTURES` name and each `robot` in `("222", "223", "both")`, `perform_gesture` returns or raises a message containing `"single-robot"` and nothing is sent (no `start_task` on either fake execution).
- [ ] **Step 2: Run it.** Expected: FAIL.
- [ ] **Step 3: Implement** the check at the top of the dual `perform_gesture`: `f"{name} is a 223 single-robot gesture (unitree-g1-sonic-arm); not available on the dual container"`, in the same refusal form the dual skill uses for other invalid input. Add a note to its docstring.
- [ ] **Step 4: Run** `uv run pytest dimos/robot/unitree/g1/test_sonic_dual_arm_skill.py -v`. Expected: all pass.
- [ ] **Step 5: Stage and report.**

---

### Task 6: Greeter `--natural`

**Files:**
- Modify: `dimos/robot/unitree/g1/greeter.py` (`acts`, CLI, module docstring table)
- Test: `dimos/robot/unitree/g1/test_greeter.py`

**Interfaces:**
- Produces: `acts(take_side: Side, entrance_side: Side, natural: bool = False) -> list[Act]`. With `natural=True` it returns `[("invite_natural", "06_invite"), ("take_right_natural", "07_take"), ("entrance_right_natural", "10_entrance"), ("wave_natural", "11_wave")]`. With a left side it raises `ValueError("--natural has right-side take and entrance only")`. CLI flag: `--natural` (store_true).

- [ ] **Step 1: Write the failing tests**: `test_natural_acts_pair_the_clip_gestures_with_the_same_lines` (the exact list above; `acts("right", "right")` is unchanged) and `test_natural_refuses_left_sides` (both sides).
- [ ] **Step 2: Run** `uv run pytest dimos/robot/unitree/g1/test_greeter.py -v`. Expected: FAIL.
- [ ] **Step 3: Implement**, and add the natural set to the module docstring's table and usage line.
- [ ] **Step 4: Run** the greeter tests. Expected: all pass.
- [ ] **Step 5: Stage and report.**

---

### Task 7: MuJoCo stand-in runs, tuning and videos

**Files:**
- Modify: `dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py` (a `test_clip_gesture_tracks_and_hands_back`, parametrized over `CLIP_GESTURES`)

The clip has no keyframe holds, so `_tracking`'s per-keyframe check does not fit. The new test checks the result message: `"of the planned pose and returned; " + HANDBACK`, with no "stopped early" and no "nothing sent". It also checks `_physics(sim, window, BOTH_ARMS)` (contacts, tilt, feet) and `_body`. It prints, without asserting:
- the arm amplitude ratio: the measured peak-to-peak of the clip's busiest joint over the clip window, divided by the planned peak-to-peak;
- the measured arm lag against the plan, from the best cross-correlation shift;
- `stream_peak_dq` against `fast_max_dq`;
- `max_deviation_rad.legs`.

It also adds `test_stop_mid_clip_finishes_then_returns_in_sim`, with the wave only, using the existing `stop=True` path of `_run`.

- [ ] **Step 1: Write both tests**, reusing `_run`, `stand_in`, `_settle`, `_physics` and `_body` unchanged.
- [ ] **Step 2: Check the machine is free.** Run `pgrep -af "sonic_sim_peer|mujoco"` and `ListAgents`, and ask the user if another session is running a sim.
- [ ] **Step 3: Run** them pinned to cores 8-15, with the environment the existing sim runs use:

```bash
GEAR_SONIC_MODEL_DIR=$HOME/.cache/huggingface/hub/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2 \
GEAR_SONIC_SCENE=$HOME/work/GR00T-WholeBodyControl-hand-tracking-watchdog/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml \
taskset -c 8-15 uv run pytest dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py -k "clip" -s -v
```

  Expected: 5 pass. Record per gesture: amplitude ratio, lag, peak dq against the limit, legs, contacts and tilt.
- [ ] **Step 4: Tune only if needed.**
  - A clip that trips its own speed limit while tracking well: raise `CLIP_DQ_FACTOR` (it stays under the ceiling).
  - A clip whose legs pass the margin: report it. Don't widen the margin without the user.
  - An amplitude ratio below about 0.6 on the wave means SONIC isn't reproducing it. Report that to the user with the slower-playback fallback (the spec's Risks), and don't change speeds yourself.
- [ ] **Step 5: Render** each stand-in run from the sim log (`np.load(sim.log)` `t`, `q`) to an mp4. Reuse the spike's `~/work/g1-demo-gestures/ardy/scripts/analyze.py` render approach, with the scene_43dof model. Save to `~/work/g1-demo-gestures/ardy/sim/`. These are for the user to judge naturalness.
- [ ] **Step 6: Stage** the test file and report the table, the videos and any tuning to the user. The hardware run on 223 is the user's operator action: give them the `perform_gesture` MCP commands (`dimos mcp call perform_gesture --arg name=wave_natural --arg line=11_wave`, and so on) and `python -m dimos.robot.unitree.g1.greeter --natural`.
