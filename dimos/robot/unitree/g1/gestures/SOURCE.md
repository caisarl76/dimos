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
holds the hands together in front of the belly (2026-10-27, below). Every plan cleared the both-arm
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
- thanks: `waist_pitch` +0.40 on arrival at the belly pose, held 1.5 s, then back to the measured
  start (not 0.0: the stance leans +0.154, and 0.0 would lean back past it); the real bow is
  +0.214 rad beyond the stance. Leg margin 0.23 rad on the leg joints except hip yaw (0.15).
- thanks' hands are at the belly (2026-10-27, user: the raised hands, shoulder pitch -1.05/-0.75 and
  elbow 0.20 with the wrists 30-37 cm forward, made 223 stumble forward on hardware): shoulder
  pitch 0.0, roll 0.20/-0.20, yaw -0.30/0.30 left/right, elbow -0.30, so the upper arms hang and
  the Dex3 hands meet with the wrists 19-21 cm forward of the pelvis. In the bow the upper-body
  CoM is 9.7 cm forward of the pelvis (was 14.7); MuJoCo legs peak 0.140 (was 0.179), 0.112 on a
  mid-bow stop. The hands-down first keyframe stays.
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

## ARDY clips

`clips/*.csv` are arm trajectories (header = the 14 arm joint names, `CLIP_COLUMNS`; 50 Hz rows in
rad) generated with ARDY (nv-tlabs/ardy @ `8163bbf`), model `ARDY-G1-RP-25FPS-Horizon52`, 6.0 s,
`--num_samples 4 --seed 0`. `tool_ardy_clip.py` takes ARDY's 25 fps MuJoCo qpos CSV, keeps the arm
columns, resamples with a cubic spline at the speed factor, trims to the motion plus 0.5 s each side, then
eases the playback speed 0 -> speed -> 0 over 0.4 s at both ends (the raw samples start and end mid-motion),
so each clip starts and ends at rest and is 0.4 s longer than the trimmed motion.

| gesture | sample | prompt | speed |
|---|---|---|---|
| `wave_natural` | wave_01 | A person stands in place and waves hello with the left hand. | 0.7x |
| `invite_natural` | invite_03 | A person stands in place and opens both arms in a warm welcoming gesture, inviting people to come closer. | 0.8x |
| `entrance_right_natural` | entrance_right_01 | A person stands in place and extends the right arm out to the right side with an open palm, showing the way to the entrance. | 0.8x |
| `take_right_natural` | take_constrained_01 | A person stands in place and reaches the right arm out to the front right with an open palm, presenting an object on a table as if saying please take one. | 0.8x |

`take_right_natural` is constrained with `raw/take_right_constraint.json`: right hand and hips pinned
on frames 40/50/60/70 to `take_right.json`'s arm pose. The raw CSVs and generation scripts live in
`~/work/g1-demo-gestures/ardy/` (not committed). Regenerate:

```bash
R=~/work/g1-demo-gestures/ardy/raw; C=dimos/robot/unitree/g1/gestures/clips
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/wave_01.csv 0.7 $C/wave_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/invite_03.csv 0.8 $C/invite_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/take_constrained_01.csv 0.8 $C/take_right_natural.csv
python -m dimos.robot.unitree.g1.tool_ardy_clip $R/entrance_right_01.csv 0.8 $C/entrance_right_natural.csv
```
