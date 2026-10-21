# Startup LiDAR blind spots and a waist sweep, 2026-10-21

Investigation only: no robot or control change. Reproduce with
[demo_startup_coverage.py](/docs/artifacts/g1_navigation_sonic_20261013/demo_startup_coverage.py).

## What is hidden at startup

At startup, before any walking, three robot sessions show the same unknown-cell pattern
by bearing from the robot's heading:

| Bearing | 0.5-1.0 m unknown | 1.0-2.0 m unknown |
|---|---|---|
| Forward, -30 to +30 deg | 29-40% | 0% |
| Front-left, +30 to +60 deg | 90-92% | 49-58% |
| Front-right, -60 to -30 deg | 82-88% | 37-44% |
| Behind, 120-180 deg | 68-100% | 15-27% |

There are two causes:

1. **Occlusion wedges** at about +-30-60 deg, unknown even 1-2 m out where the floor is
   well inside the field of view: the head frame and cables, fixed to the sensor mount.
   In the recorded scan a third of the beams (6,813 of 20,064) return nothing. The
   MID360S reports returns inside its minimum range, about 0.1 m, as zeros, which fits
   structure right next to the sensor.
2. **A field-of-view limit for the near ring.** Mounted upside down, the MID360S sees at
   most about 52 deg below horizontal (the steepest 1% of returns are at -49 deg). The
   sensor is 1.31 m above the floor, so floor nearer than about 1.0 m is not visible in
   any direction (nearest floor return 0.99 m). Taller objects are: a 40 cm box from
   about 0.66 m.

## What a waist yaw sweep would add

The waist yaw axis passes within 4 mm of the sensor, and the occluders turn with the
sensor. Rotating the recorded scan's floor returns about that axis estimates the floor
bearings seen:

| Distance | Static | +-30 deg | +-60 deg | +-90 deg | +-150 deg |
|---|---|---|---|---|---|
| 0.5-1.0 m | 1% | 11% | 21% | 31% | 51% |
| 1.0-1.5 m | 41% | 80% | 96% | 100% | 100% |
| 1.5-2.0 m | 51% | 84% | 99% | 100% | 100% |

One 0.1 s scan is sparse; the static figures improve over a few seconds, but the
wedges stay. So a sweep of about +-60 deg would fill the 1-2 m ring at startup, while
the near ring stays mostly unseen without a forward lean (waist pitch) or a short walk.

## Can SONIC turn the waist?

- The G1 waist limits (gear_sonic_deploy g1_29dof model) are yaw +-2.618 rad and roll and
  pitch +-0.52 rad. A 0.3 rad forward lean would move the downward limit from about
  -52 deg to -69 deg and show the floor from about 0.5 m.
- Navigation drives SONIC through its planner; the only other route is the pose route
  (`start_task` and `submit_chunk`), which takes SONIC tokens, not joint angles. Turning
  a joint trajectory into tokens needs the SONIC encoder, which the arm-skill branch
  (`jihun/feat/g1-sonic-arm-skill`, `sonic_arm_reference`) has and this branch does not.
- The arm session left waist turns out of its skills because SONIC tracked only 18-48%
  of an isolated waist yaw reference. A +-90 deg command may therefore turn the torso
  only about +-16-43 deg: still most of the 1-2 m gain above.

## Options

1. A startup turn with the existing planner: turning the whole body in place sweeps
   the occluders the same way. No new control path, but the robot turns in place at
   2-3 deg/s, so +-60 deg takes about a minute.
2. A waist yaw sweep through the pose route, using the arm branch's encoder: faster
   and the feet stay put. Needs that encoder path on this branch, a simulation check of
   balance and handback, and care with the head cables and the support cable.
3. A waist pitch lean for the near ring, through the same path; balance risk to check
   in simulation first.
4. As now: walk a curve before a goal, which also makes the near ring known.
