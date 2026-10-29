# Why voxels a person leaves in front of a standing G1 survive

Sessions 2026-10-22-3..6: the operator stood in front of the robot to place a box, and
after they left, some of their voxels stayed in the map and blocked the goal cases. The
map refresh (`refresh_map`) works around it. This study looks for the cause.

`demo_voxel_clearing_study.py` feeds the real `VoxelRayMapper` (the peer's 0.05 m voxels,
other settings default) with scans from a modelled MID360 at the G1 head mount. The model
is the sensor upside down at 1.26 m, a -7..52 deg field, PointLio's 0.5 m blind range,
8000 rays per scan and 2 cm range noise. The scene is a room with a 40 cm box and a 12 cm
step that must stay mapped. A person (a 0.18 m radius cylinder, 1.7 m tall) stands ahead
for 5 s, then is gone for 10 s. Results: `results.json` (seeded, reproducible).

## Result

The stale voxels are kept by `graze_cos` (default 0.7): a ray only clears a voxel when
it meets the voxel's surface at least that squarely. A sensor that never moves sees the
person's flanks edge-on, so every later ray through those voxels is oblique and none may
clear them. 80-90% of the stale voxels are on the flanks.

`grace_depth` (no clearing within 0.2 m of a hit) has almost no effect. The nearby floor
not being visible (nearest floor seen at 0.98 m) is not the cause either: the stale
voxels are at every height, mostly 0.3-2 m.

Costmap cells blocked (above the 0.10 m climb height) 10 s after the person left:

| Setting | Person at 0.6 m | 0.8 m | 1.0 m | 1.5 m | 40 cm box kept | 12 cm step kept | Floor 1-2 m kept |
|---|---|---|---|---|---|---|---|
| peer (graze 0.7, grace 0.2) | 14 | 17 | 20 | 16 | 0.89 | 0.99 | 0.996 |
| grace 0.05 | 14 | 17 | 16 | 16 | 0.88 | 0.98 | 0.989 |
| graze 0.5 | 11 | 10 | 14 | 11 | 0.89 | 0.98 | 0.995 |
| **graze 0.3** | **1** | **3** | **5** | **4** | **0.89** | **0.96** | **0.995** |
| graze 0 | 0 | 0 | 0 | 0 | 0.86 | 0.96 | 0.995 |
| grace 0.05 + graze 0 | 0 | 0 | 0 | 0 | 0.69 | 0.88 | 0.936 |

"Kept" is the share of the box footprint cells still blocked, or of floor cells mapped,
at the end. The box's 0.89 is the peer baseline, not a loss.

## What it suggests

- `graze_cos` 0.3 removes almost all the stale cells, and costs nothing measurable on
  the floor or the 40 cm box. The 12 cm step loses 3% of its cells.
- `graze_cos` 0 clears everything, but starts eroding the 40 cm box.
- Do not lower `grace_depth`: with graze 0 it erodes the floor and the boxes.

This is a model: a rigid cylinder for the person, uniform rays, no self-occlusion by the
head frame, and a person who appears and vanishes rather than walking. Before changing
the peer's `RayTracingVoxelMap(graze_cos=...)`, replay a `record_lidar` recording from
the robot (operator steps in front, then leaves) under 0.7 and 0.3 and compare the map
after with `map_after` in the recording.
