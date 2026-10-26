# Source of the imported gestures

- **Repository:** `physical_ai_agent`
- **Commit:** `3430faa0059b656f6dba665fbdeeed7ed3899f45`
- **Path:** `motion_control/motion_control/gestures/data/<name>.json`
- **Imported:** 2026-10-23, a one-time copy; nothing is loaded from that repository at runtime.
- **Authors** (`git log --format='%an'` on those files at that commit): kwminji, 최창윤

## What was copied

The motion data of `wave_one_hand`, `v_sign`, `good`, `support_person` and `two_hand_hearts`
(and of the two dropped below): the top-level `name`, `return_s` and `keyframes`, and each
keyframe's `t`, `delta`, `absolute` and `hands`. Values are unchanged, except for the one change
listed below; the files are re-serialized with sorted keys and an indent of 2.

## Changed after import

- `wave_one_hand` (2026-10-26, at the user's request after a pc2_222 run): the source has no
  `hands` targets, so it waved with the deploy fist. Every keyframe from the first raise on
  (`t` >= 2.2; never `t` = 0) now sets the right hand open, all seven Dex3 joints 0.0 (the right hand
  of the dropped `wave_two_hands`). The left hand is not named, so it stays at the deploy fist, and
  both hands return to the measured start on the way back. The finger travel counts toward the
  peak-speed timing, so the raise and the return each take 4.4 s instead of about 2 s.

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

## Hand-authored

`invite`, `take_left`, `take_right`, `entrance_left`, `entrance_right` and `thanks` are not
imported: they were written on 2026-10-26 for the G1 223 demo (voice lines 06_invite, 07_take,
08_entrance, 09_thanks), in the same format, loaded and checked like the imported gestures
(`AUTHORED` in `../imported_gestures.py`). Arms only, no hands and no waist (the loader refuses
waist motion). The poses were set on scene_43dof.xml with MuJoCo kinematics: the pointing arms with
`ArmKinematics.solve` (take: azimuth 45, elevation -25 deg; entrance: 70, 0), the right arm as the
left's mirror (roll and yaw negated), each palm turned forward by wrist roll alone (take 0.6,
entrance 0.85 rad: a near-straight arm's shoulder yaw of 0.45-0.55 was not tracked); invite's palms face forward (shoulder yaw 0.6, wrist roll
0.5) and sway once by shoulder roll +-0.15 rad; thanks brings the forearms forward first, then
holds the hands in front of the waist, the left above the right. Every plan cleared the both-arm
`ArmCollisionChecker` from the stand-in's stance and from ROBOT_IDLE.

Tuned in the MuJoCo stand-in (`test_imported_gesture_tracks_and_hands_back`, 2026-10-26): in this
sim the right hip pitch creeps during every hold, so holds are 1.5 s (thanks 3 s), the pointing
arms take 4 s to arrive (3 s dipped the right foot's contacts on arrival), take's wrist roll is
0.6 (1.15 did the same and the elbow lagged), and invite sways
once; invite with shoulder yaw 1.2 was not tracked (yaw is a loose joint) and tripped the knee, and
with its hands 20 cm forward tripped the 0.15 hip envelope; take at 50-60 deg and entrance at 80 deg
on the right dropped to 6 floor contacts. Final run: all six passed, legs 0.07-0.13 rad.
