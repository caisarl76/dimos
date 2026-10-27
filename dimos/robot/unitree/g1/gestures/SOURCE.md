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

`invite`, `take_left`, `take_right`, `entrance_left`, `entrance_right`, `thanks` and `bow` are not
imported: they were written on 2026-10-26 for the G1 223 demo (voice lines 06_invite, 07_take,
08_entrance, 09_thanks), in the same format, loaded and checked like the imported gestures
(`AUTHORED` in `../imported_gestures.py`). Arms only, no hands; only invite, thanks and bow command the waist (below, the loader refuses it elsewhere). The poses were set on scene_43dof.xml with MuJoCo kinematics: the pointing arms with
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

Waist (invite, thanks, bow; 2026-10-26/27, measured in the MuJoCo stand-in). Sign, from `torso_rel_deg`:
+waist_yaw turns left, +waist_pitch bows forward. Commands are absolute and capped at 0.40 rad.

- invite: `waist_yaw` +0.40, then -0.40, then back to the measured start; the real turn is about
  8.7 deg each way. Leg margin 0.23 rad on all 12 leg joints, hip yaw included.
- thanks: `waist_pitch` +0.40 on arrival at the raised pose, held 1.5 s, then back to the measured
  start (not 0.0: the stance leans +0.154, and 0.0 would lean back past it); the real bow is
  21.2 deg absolute, +12.4 deg beyond the 8.8 deg lean of the stance itself (it leans 0.154 rad
  forward). Leg margin 0.23 rad on the leg joints except hip yaw (0.15).
- thanks' hands are raised (shoulder pitch -1.05/-0.75, roll 0.50/-0.50, yaw -0.70/0.70 left/right,
  elbow 0.20): today's pose plus the bow was refused by the path check, the hands at the thighs.
  The hands-down first keyframe stays.
- bow (2026-10-27, user: a new gesture; thanks keeps its own bow): the arms go to shoulder pitch
  -0.40, roll 0.25/-0.25, yaw 0, elbow 1.20 (forearms hanging forward), then `waist_pitch` +0.40,
  held 1.5 s; the way home brings the waist back. The forward shoulder pitch keeps the hands in
  front of the thighs: the hands sit below the waist joint, so a bow alone swings them back into
  the legs (with the robot's own idle arms the path check refuses it at the hips). Real bow 0.367
  rad absolute (+0.213 beyond the stance); legs peak 0.118 (0.138 on a mid-bow stop), so leg
  margin 0.19 rad on the leg joints except hip yaw (0.15).
- invite's waist turn drags the shoulders off their targets (yaw up to 0.360, pitch up to 0.320 rad
  in MuJoCo); accepted for invite (user, 2026-10-27). On the two turn keyframes the e2e prints the
  arm tracking rather than asserting it; the raise and the return to centre are still asserted.
- Foot placement is checked on the robots, not in MuJoCo: the stand-in's feet slide and pivot in
  every gesture, waist or not (6.9-16.7 mm, 2.8-9.4 deg), so the e2e prints the feet only.
