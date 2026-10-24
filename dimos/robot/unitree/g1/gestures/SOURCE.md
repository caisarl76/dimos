# Source of the imported gestures

- **Repository:** `physical_ai_agent`
- **Commit:** `3430faa0059b656f6dba665fbdeeed7ed3899f45`
- **Path:** `motion_control/motion_control/gestures/data/<name>.json`
- **Imported:** 2026-10-23, a one-time copy; nothing is loaded from that repository at runtime.
- **Authors** (`git log --format='%an'` on those files at that commit): kwminji, 최창윤

## What was copied

The motion data of `wave_one_hand`, `v_sign`, `good`, `support_person` and `two_hand_hearts`
(and of the two dropped below): the top-level `name`, `return_s` and `keyframes`, and each
keyframe's `t`, `delta`, `absolute` and `hands`. Values are unchanged; the files are re-serialized
with sorted keys and an indent of 2.

## What was stripped

- `_doc` (free-text notes) from every file.
- `aliases` (`wave`, `wave_both`, `v`): `wave` collides with an existing DimOS gesture name.

The limit tables in `../imported_gestures.py` are copied from the same commit: the URDF joint limits
(`upper_body.LIMITS`), and the Dex3 command order, deploy command limits and deploy default hand
(`hands.ORDER`, `hands.LIMITS`, `hands.DEFAULT`).

## Dropped

Both were imported, then dropped on 2026-10-23 after the MuJoCo e2e (the `sonic_sim_peer`
stand-in, `test_sonic_arm_sim_e2e.py`); their JSON files were deleted.

- `open_window`: refused by the both-arm path check from the stand-in's standing pose, "the
  planned arm path comes within 2 cm of the right_wrist_yaw_link (other arm); nothing sent". It
  cleared the check only from the idle pose used in the unit tests.
- `wave_two_hands`: right_hip_pitch reached -0.151, -0.152 and -0.151 rad, beyond the 0.15 rad leg
  envelope, in 3 of 3 runs, so the motion stopped early. The arms tracked within 0.163 rad.
