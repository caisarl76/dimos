# G1 Forearm Pointing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `point_at_object` aim the left forearm through a floor-intersected aim point, with a
natural, collision-free pose, anywhere in the head-camera image. Report planned accuracy and
measured accuracy, the latter taken over the hold.

**Architecture:**
- **Pure geometry and IK in `sonic_arm_reference.py`:**
  - the aim point from a pixel plus the measured pose's floor;
  - the forearm error;
  - a natural-pose acceptance check;
  - `solve_forearm`, which runs grid → refine → filter → re-score → path-check order.
- **The skill container:**
  - collects timestamped hold-window feedback samples;
  - passes them to `report_extra`;
  - switches `point_at_object` to forearm mode.
- **MuJoCo e2e:** checks hold-window accuracy against the report.

**Tech Stack:** Python 3.12, numpy, MuJoCo, DimOS skills, pytest, and the in-repo MuJoCo stand-in
(`sonic_sim_peer`).

**Spec:** `docs/superpowers/specs/2026-10-22-g1-forearm-pointing-design.md` (approved 2026-10-22,
with both corrections folded in). Builds on `aa798e221`.

## Global Constraints

- **Aim point.** It lies along the pixel's ray from the camera origin.
  - The length is the floor intersection, clamped to 0.5–4.0 m. A ray that does not descend uses
    4.0 m.
  - The floor plane is the lower of the two `*_ankle_roll_link` origins in the measured pose, minus
    the sole offset read from the foot geometry.
  - Everything is in the torso_link frame.
- **Forearm error.** It is the angle between (wrist-yaw-link origin − elbow-link origin) and
  (aim point − wrist-yaw-link origin).
- **Natural-pose acceptance.** All of these are required:
  - wrist x ≥ 0.10 m (torso frame);
  - forearm unit vector x ≥ 0.2;
  - elbow bend from straight within 5–70° (straight = 1.43 rad);
  - shoulder roll ≤ 60°;
  - each of pitch, roll, yaw and elbow at least 5° inside `JOINT_BOUNDS`.
- **Preference.** `FOREARM_PREF = (−45°, 25°, 0°, 30°)` (pitch, roll, yaw, elbow) with weights
  `(1, 1, 0.7, 0.7)`.
- **Search.**
  1. A grid with 10° steps on pitch and roll, and 15° on yaw and elbow; roll ≥ 10°.
  2. Seeds within 8°, ranked by distance to `FOREARM_PREF`; the first 40.
  3. Each refined to the smallest error.
  4. Keep those within 3° that pass acceptance; re-score them by distance to `FOREARM_PREF`.
  5. Path-check in that order with `start=base`; the first clear one wins.
- **Accuracy bars.**
  - Planned (IK) error ≤ 3°.
  - Measured error: the median over hold-window samples, ≤ 8° in MuJoCo (provisional; record it).
  - The hold window runs from arrival + 0.5 s to departure − 0.5 s, from the plan's own timing.
  - Only samples taken before any abort count. The value is null when there are none.
- **Image grid.** u ∈ {0, 106, 213, 320, 426, 533, 639}; v ∈ {0, 80, 160, 240, 320, 400, 479}.
- **Time.** `solve_forearm` takes < 1 s on the workstation.
- **Unchanged.** Envelopes (`pose_margins`), `PEAK_SPEED` 0.47 rad/s, the retreat, the lifecycle,
  `stop_arm`, and the straight-arm `point_at`.
- **Process.**
  - No commits; leave changes uncommitted.
  - No robot runs.
  - MuJoCo runs need a turn agreed with the dual-G1 session (dimos-dev-1a); the controller
    arranges it.
- **Merge friendliness.** The dual-G1 branch (918e8005f) moved `_Watch`, `_stream` and
  `handle_feedback` into a per-robot `SonicArm`. Keep Task 2's edits to those functions small and
  self-contained (a few added lines each, no reshuffling), so they port cleanly.
- **Code style.** mypy strict, ruff clean, imports at the top (MuJoCo imported lazily, as
  `ArmKinematics` does), and match the surrounding style.

## Review Focus

1. **Feedback during a stop or retreat must never count as a hold sample.** If a sample arrives
   after `watch.abort` is set, or after the retreat replaced the plan, `measured_aim_error_deg` must
   ignore it. If no clean samples remain, it is null. Task 2 owns this.
2. **A stop before the hold starts gives null measured accuracy,** never an error computed from the
   rise. Task 2.
3. **The waist or legs are not at idle.** The floor plane and the IK use the measured pose, not
   `ROBOT_IDLE`. Test with a base whose waist pitch is 0.1 rad. Task 1.
4. **A ray close to horizontal** (torso pitched back) clamps to 4.0 m; it must not divide by zero
   or give a negative length. Task 1.
5. **No accepted pose** (the aim point is behind or above the robot) refuses with the reason, and
   nothing is sent. Tasks 1 and 3.

---

## File Structure

| File | Change |
|---|---|
| `dimos/robot/unitree/g1/sonic_arm_reference.py` | Add `ArmKinematics.aim_point`, `forearm_error`, `natural_pose`, `solve_forearm` and the `FOREARM_*` constants. Delete `aim_from_pixel` and `OBJECT_RANGE_M` (Task 3). |
| `dimos/robot/unitree/g1/sonic_arm_skill.py` | Hold-window sample collection on `_Watch`; `report_extra(base, hold_q)`; `point_at_object` forearm mode. |
| `dimos/robot/unitree/g1/system_prompt.py` | Describe forearm pointing for `point_at_object`. |
| `dimos/robot/unitree/g1/test_sonic_arm_keyframes.py` | Geometry and IK tests. |
| `dimos/robot/unitree/g1/test_sonic_arm_skill.py` | Hold-sample and skill tests. |
| `dimos/robot/unitree/g1/test_sonic_arm_sim_e2e.py` | Three `point_at_object` e2e cases. |

---

### Task 1: Aim point, forearm error, acceptance and `solve_forearm`

**Files:** Modify `sonic_arm_reference.py`; test `test_sonic_arm_keyframes.py`.

**Interfaces (produces):**
```python skip
FOREARM_PREF = np.radians([-45.0, 25.0, 0.0, 30.0]); FOREARM_WEIGHTS = np.array([1.0, 1.0, 0.7, 0.7])
AIM_RANGE_M = (0.5, 4.0)
class ArmKinematics:  # added methods
    def aim_point(self, q: NDArray[Any], u: float, v: float) -> tuple[NDArray[np.float64], float]
        # (point in torso frame, ray length m)
    def forearm_error(self, q: NDArray[Any], point: NDArray[Any]) -> float  # deg
    def natural_pose(self, q: NDArray[Any]) -> str | None  # None, or the first failed check's name
    def solve_forearm(self, q: NDArray[Any], point: NDArray[Any],
                      path_clear: Callable[[NDArray[np.float64]], str | None]) -> NDArray[np.float64]
        # [4] pitch, roll, yaw, elbow. path_clear(full_29_pose) returns the contacted body or None.
        # ValueError messages: "cannot point within 3 deg" / "no natural pose points there" /
        # "every pose that points there comes near the <body>"
```

- `aim_point` uses `pixel_ray` and `HEAD_CAMERA_XYZ`. The floor comes from forward kinematics of
  `q`: the ankle-roll origins, minus the sole offset. Read the offset from the foot geom: the
  lowest point of the foot collision geometry below the ankle-roll origin at the zero pose.
- `solve_forearm` follows the Global Constraints search exactly. Precompute the coarse grid's FK
  once per call; the probe took 0.4 s for 27k poses, 0.19 s per solve. Keep everything in one
  `MjData` (single motion thread).

- [ ] **Step 1: write the failing tests**
  - `test_forearm_covers_the_whole_image` (`needs_scene`). For each of the 49 grid pixels:
    - `P, t = kin.aim_point(ROBOT_IDLE, u, v)`, with `0.5 <= t <= 4.0`;
    - `q4 = kin.solve_forearm(ROBOT_IDLE, P, clear)`, where `clear` uses
      `ArmCollisionChecker.first_contact(stroke_plan(...), start=ROBOT_IDLE)`;
    - assert `kin.forearm_error(pose, P) <= 3.0` and `kin.natural_pose(pose) is None`.
    - Collect the per-pixel solve times; assert the maximum is < 1.0 s.
  - `test_aim_point_ray_length_matches_the_camera_height` (`needs_scene`). Centre and bottom pixels
    give lengths of about 1.68 m and 1.32 m (within 0.05). The top corners give about 3.18 m. The
    point's floor height equals the floor plane within 1 mm.
  - `test_aim_point_follows_the_measured_waist` (`needs_scene`). A base with waist pitch +0.1 rad
    changes `t` for the centre pixel, and the point still lies on the floor plane.
  - `test_aim_point_clamps_a_level_ray` (`needs_scene`). Monkeypatch `pixel_ray` to return a level
    or upward ray: `t == 4.0`, and the result is finite.
  - `test_refined_candidates_are_rescored_before_the_path_check`. Monkeypatch the refine helper so
    that the seed ranked first refines to a pose farther from `FOREARM_PREF` than the second seed's
    result. Assert that the returned pose is the second one, and that `path_clear` was called on it
    first.
  - `test_unreachable_point_is_refused`. A point 1 m behind the torso raises `ValueError`
    ("cannot point" or "no natural pose").
  - `test_all_blocked_is_refused_with_the_body`. With `path_clear` always returning "torso_link",
    it raises `ValueError` mentioning "torso_link".
- [ ] **Step 2:** run the tests and confirm they fail (ImportError or AttributeError).
- [ ] **Step 3:** implement.
- [ ] **Step 4:** `GEAR_SONIC_ENCODER=… .venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_arm_keyframes.py -q -p no:cacheprovider` all pass; mypy and ruff are clean.
- [ ] **Step 5:** leave it staged-free and uncommitted; report.

---

### Task 2: Hold-window samples and `report_extra(base, hold_q)`

**Files:** Modify `sonic_arm_skill.py`; test `test_sonic_arm_skill.py`.

**Interfaces:**
- `_Watch` gains:
  - `hold_rows: tuple[int, int] | None` (token-plan rows of the hold window);
  - `plan_start: float | None`;
  - `hold_q: list[NDArray[np.float64]]`.
- **`_move`** sets `hold_rows` for the deepest keyframe's hold:
  - start: `pad + seg.end - 1 + 25`;
  - end: `pad + next_seg.start - 25`;
  - set to None when the hold is shorter than 50 rows plus 1.
- **`_stream`** sets `watch.plan_start` (under `self._lock`) at the first submit. It never updates
  it for a retreat.
- **`handle_feedback`** appends `body_q` only when all hold:
  - `watch.abort is None`;
  - not returning;
  - `plan_start` is set;
  - `hold_rows[0] <= (now - plan_start) * RATE <= hold_rows[1]`.
- **The `report_extra` signature becomes**
  `Callable[[NDArray[np.float64], NDArray[np.float64] | None], dict[str, Any]]`. Its second
  argument is `np.asarray(hold_q)`, or None when empty.
  - Adapt `point_at`'s `report_extra` (it ignores the second argument).
  - Add a generic report key `"hold_samples": len(hold_q)`.
  - This changes the contract the dual-G1 session passes through; the controller tells it.

- [ ] **Step 1: write the failing tests** (scripted peer, fast fixture). The `skill` fixture caps
  holds at 0.1 s, which is too short for a window. Patch `keyframe_plan` with fast timings but real
  holds, as `test_long_hold_splits_inside_the_hold_and_passes_the_age_check` does, and use
  `hold_seconds=2`.
  - `test_hold_samples_come_only_from_the_hold_window`. A pose motion with a 2 s hold under a
    patched fast plan. The feeder plays telemetry; count the samples. Every sample's time lies
    inside the window, and the count is > 0.
  - `test_no_hold_samples_after_a_stop`. A leg leaves its envelope during the rise: `hold_samples`
    is 0, and `report_extra` receives None.
  - `test_samples_stop_at_a_mid_hold_stop`. Call stop_arm mid-hold: samples after the stop
    timestamp are not counted.
- [ ] **Step 2:** run and see them fail.
- [ ] **Step 3:** implement.
- [ ] **Step 4:** `test_sonic_arm_skill.py` is green 2×; mypy and ruff are clean.
- [ ] **Step 5:** uncommitted; report.

---

### Task 3: `point_at_object` in forearm mode

**Files:** Modify `sonic_arm_skill.py`, `system_prompt.py` and `sonic_arm_reference.py` (delete
`aim_from_pixel` and `OBJECT_RANGE_M`); tests in `test_sonic_arm_skill.py` and
`test_sonic_arm_keyframes.py` (remove the two `aim_from_pixel` tests).

**Behaviour:**
- The frame check, `query_boxes`, the largest box and the failure messages are unchanged.
- Remove the azimuth/elevation cone check from `point_at_object` only.
- In `keyframes_of(base)`:
  - `P, t = kin.aim_point(base, cx, cy)`;
  - `q4 = kin.solve_forearm(base, P, clear)`, where `clear(pose)` runs
    `self._checker.first_contact(stroke_plan(base, pose - base), start=base)`;
  - the goal is the base with the left-arm targets set to `q4`;
  - one keyframe, held for `hold_seconds`;
  - `require_check=True`, so `_move`'s own path check also runs.
- **Report** (via `report_extra(base, hold_q)`):
  - `mode: "forearm"`;
  - `label`, `box_px`;
  - `aim_point_m` (rounded to 3 decimals) and `aim_distance_m`;
  - `pose_deg`;
  - `planned_aim_error_deg`;
  - `measured_aim_error_deg`: the median of `kin.forearm_error(sample, P)` over `hold_q`, or None.
- **Prompt:** "points at the object with its forearm, anywhere the head camera sees it; objects
  on a table may be aimed slightly behind themselves; refuses with the reason if no natural pose
  points there".

- [ ] **Step 1: write the failing tests.**
  - `test_point_at_object_aims_the_forearm_at_the_floor_point` (`needs_scene`; real kinematics and
    checker in the fast fixture). With a centre box, the report has `mode == "forearm"`,
    `planned_aim_error_deg <= 3`, and `aim_distance_m` of about 1.68.
  - `test_point_at_object_right_edge_is_pointable` (`needs_scene`). A box at u≈620 succeeds,
    where it was refused before this change.
  - `test_point_at_object_reports_measured_error_from_hold_samples`. Use a fake kinematics whose
    `forearm_error` returns a known value per sample: the reported median is right.
  - Keep the existing not-found, vision-failure and disabled tests green.
- [ ] **Step 2:** run and see them fail.
- [ ] **Step 3:** implement.
- [ ] **Step 4:** the whole g1 suite is green.
- [ ] **Step 5:** uncommitted; report.

---

### Task 4: MuJoCo e2e for forearm pointing

**Files:** Modify `test_sonic_arm_sim_e2e.py`.

- Add three cases using a fake vision model with fixed boxes: centre (320, 240), right edge
  (620, 240) and bottom right (560, 450), each with `hold_seconds = 3`. They use the real
  kinematics, checker and encoder, as the existing cases do.
- **Hold-window metric from the stand-in npz.** Take the logged `q` samples whose time lies in the
  hold window. Convert the plan timing using the stream start, read from the journal's first
  `chunk_accepted`. Compute `ArmKinematics.forearm_error` against the report's `aim_point_m`.
  - The median must be ≤ 8°.
  - It must be within 2° of `measured_aim_error_deg`.
  - Print both.
- **Standard asserts:** success message, reach ≥ 0.8, tilt ≤ 3°, contacts ≥ 7, peak speed
  ≤ 2 rad/s, handback, and the arm back within 0.05 rad of its start.
- [ ] **Step 1:** write the cases.
- [ ] **Step 2:** the controller arranges a MuJoCo turn, then runs
  `pytest -m native_e2e test_sonic_arm_sim_e2e.py -v -s`. If the 8° bar fails, record the measured
  values and stop; do not loosen it without a ruling.
- [ ] **Step 3:** all e2e cases pass, including the 10 existing ones; uncommitted; report.
