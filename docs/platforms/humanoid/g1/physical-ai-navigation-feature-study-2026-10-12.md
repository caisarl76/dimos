# Physical AI Navigation features worth adapting for DimOS G1

Date: 2026-10-12. This is a feature study and brainstorming result, not an approved implementation design.

## Recommendation

Keep DimOS's navigation composition and selectively adapt the source repository's G1 perception, carrying, and evaluation ideas. The largest benefits are around the planner: reliable frames and timestamps, body-aware sensor processing, independent obstacle evidence, changing robot geometry, and records that explain navigation failures.

A Nav2 adapter is a useful optional comparison backend. Replacing DimOS's entire navigation stack would add ROS lifecycle, action, and deployment work while duplicating capabilities already present.

Assumption: the initial application is indoor G1 navigation on mostly level floors, eventually combined with pick, carry, and place tasks. Stair climbing, footstep planning, and fall recovery are separate capabilities; this source does not establish them.

## What was examined

- Source checkout: `/home/jihun/work/physical_ai_navigation`, clean HEAD `81573bf3a938fbe1c97824b543b7ea7846e946ac`.
- DimOS: `/home/jihun/work/dimos_dev`, HEAD `7529659c88c76eb4800552ad1d4fa5dd584ecc80`, including its existing working-tree changes. Those changes were preserved.
- Source code, launch scripts, active configuration, simulation probes, analyzer, and recorded development notes were compared with DimOS blueprints, navigation interfaces, mapping, embodiment, and the existing manipulation point-cloud filter.
- The source's `camera`, `DualMap`, and `physical_ai_mujoco_env` submodules are uninitialized in this checkout. Their integration points were examined; their implementations were not inspected. FAST-LIO localization and the real Homie server live in other repositories.

Distinguish implemented source behavior, historical hardware observations reported in documents, and proposed DimOS work. This study did not launch ROS, simulation, a locomotion controller, or a hardware navigation task.

## Existing DimOS capability and actual gaps

| Area | Existing DimOS capability | Useful addition |
|---|---|---|
| G1 navigation | `unitree-g1-nav-simple`: FAST-LIO, ray-traced voxel mapping, height costmap, A*, arrival/cancel, stuck/obstacle/deviation replanning, movement mux, high-level DDS execution | Sensor and controller commissioning; depth obstacle integration; carry-aware geometry |
| Whole-body execution | `unitree-g1-groot-wbc`: simulation or real whole-body backend, with navigation; real path uses Point-LIO | The same navigation contract across vendor and policy controllers, with separately measured motion behavior |
| Body filtering | `PointCloudSelfFilter` already loads URDF collision geometry, filters captures, and emits current/previous-body voxel clearing masks | G1 wiring, complete live link poses, sensor timing, and separately managed held-object geometry |
| Geometry and local planning | `Embodiment`, local planner, and follower contracts already model swept boxes, clearance, gait behavior, and speed limits | A measured G1 embodiment and a coordinated runtime update for posture/cargo changes; current local planner defaults to Go2 and constructs its embodiment at initialization |
| Semantic goals | Tagged locations, visual object tracking, spatial-memory lookup, and `navigate_with_text` already exist | Reachable approach poses and reliable goal completion/status; an explicitly composed navigation provider |
| Evaluation | Planner evaluator, tests, recordings, Rerun, and run logs already exist | Goal-centered execution reports combining sensors, filtering, geometry, planning, control, and achieved motion |

Sources: simple G1 navigation (`/dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_simple.py`), whole-body blueprint (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py`), existing self-filter (`/dimos/manipulation/planning/utils/point_cloud_self_filter.py`), embodiment (`/dimos/navigation/embodiment/base.py`), local planner (`/dimos/navigation/experimental/dannav/local_planner/module.py`), and navigation skills (`/dimos/agents/skills/navigation.py`).

The [current navigation verification record](/docs/platforms/humanoid/g1/navigation-verification-2026-10-09.md) reports live sensor-only Point-LIO/map/costmap delivery on October 12, while physical goal navigation and robot-body calibration remain unverified. The original direct FAST-LIO path has separate sensor compatibility and destination issues. Reusing the source's FAST-LIO launch sequence would not by itself resolve those issues.

## Candidate features

Effort is relative integration scope, not a time estimate. “High” generally crosses perception, planning, and control interfaces.

| Priority | Feature | Adoption style | Effort |
|---|---|---|---|
| First | Frame, capture-time, and controller calibration | Adapt the commissioning lessons | Medium |
| First | Per-goal recording and failure analysis | Extend DimOS recording/evaluation | Medium |
| First | G1 body filtering | Extend the existing DimOS filter | Medium |
| Next | Depth + LiDAR evidence, with correct clearing | Adapt the data model and regression cases | High |
| Next | Ground filtering under walking sway | Adapt the method and experiments | Medium |
| Next | Independent command stop/slow gate | Adapt the architecture; verify actual wiring | Medium–high |
| Carry tasks | Runtime posture/cargo collision envelope | Adapt geometry, coordinated state, and tests | High |
| Carry tasks | Held-object acquisition and tracking | Develop from the source's lessons; avoid copying its current heuristic unchanged | High |
| Task navigation | Reachable standoff and final approach poses | Extend existing semantic goals | Medium |
| Regression | Sensor-realistic simulation and perturbations | Adapt scenarios and scoring | Medium |
| Refinement | Stable routes and explicit recovery/status | Extend existing behavior where a gap is demonstrated | Medium |
| Fixed sites | Prior maps and relocalization | Evaluate after local navigation works | High |
| Optional | Nav2 navigation provider | Isolated adapter/comparison backend | High |

### 1. Frame, timing, and locomotion commissioning

The source contains practical evidence about an upside-down MID-360, inconsistent base-height definitions, torso camera pitch, cloud/TF clock mismatch, and low-speed yaw commands that the walking controller fails to execute. Its bridge compensates with configurable yaw/lateral sign inversion, rotation deadband compensation, a watchdog, and command duration. These are controller-specific workarounds, not universal G1 constants. [Bridge][bridge], [self-filter][self-filter], [launch configuration][nav-config].

DimOS already represents the sensor mount flip and live waist motion in G1TfPublisher (`/dimos/robot/unitree/g1/g1_tf_publisher.py`). Preserve proper transforms and establish one convention for the ground, pelvis, sensor, `odom`, and `map` frames. Normalize capture timestamps before matching joint poses to clouds. The source uses latest TF to work around its clock mismatch; copying that choice would weaken DimOS's existing capture-time filter semantics.

Measure commanded versus achieved forward, lateral, and yaw motion for each execution backend. Fit G1 speed, stopping distance, latency, slew, gait deadband, and turning envelope rather than inheriting Go2 or Homie values.

Useful acceptance experiments: sensor rotation/translation consistency, clock-offset measurement, moving-waist/arm captures, stale-state detection, and measured low-speed/final-heading response.

### 2. Per-goal recording and navigation failure analysis

The source analyzer automatically records around a goal, keeps pre-roll and post-roll, and correlates path changes/stalls with new sensor obstacles, filter failures, footprint changes, recovery, and controller messages. This is one of its most reusable product ideas. [Analyzer implementation][analyzer], [analyzer operation][analyzer-doc].

Extend DimOS's existing run recording and evaluator with goal IDs, terminal outcomes, requested/achieved poses, commands, localization age, per-sensor evidence, footprint versions, and replanning reasons. Generate a replay and compact report: success, final position/yaw error, distance, duration, stalls, minimum observed clearance, and component latency/load.

Prefer structured events over parsing log text. Attribution from temporal correlation should remain a candidate explanation. Record lateral motion too: the source analyzer's stall heuristic primarily checks forward velocity and yaw, which is insufficient for holonomic G1 motion. Costmap overlap is a collision proxy; simulation contacts or independently observed physical contact must remain a separate metric.

Acceptance: a recorded obstacle insertion, a sensor outage, a cargo change, and a preempted goal can each be replayed with the right event ordering and outcome.

### 3. G1 body filtering using the existing DimOS implementation

The source filters points inside robot-link collision shapes and retains points for links whose TF is missing. Current `body_filter_always=True` enables body removal even when not carrying; held-object filtering remains carry-gated. Older source documentation describes empty-handed passthrough and is stale. [Body geometry][body], [current ROS wrapper][self-filter].

Adapt G1 joint/hand coverage, margin experiments, and diagnostics to the existing DimOS self-filter. Its capture-time lookup and current/previous-body clearing mask are valuable capabilities to retain. However, its missing-transform policy drops the entire capture, unlike the source's per-link skip. Choose an explicit G1 policy and expose missing-link/filter health so a dropped stream cannot silently look like free space.

The current G1 sensor-mount TF publisher covers waist/sensors; a body filter needs the collision links for arms, legs, torso, and the installed hands. Reuse DimOS robot-model/FK facilities where available and verify the actual blueprint supplies that complete tree.

Acceptance: moving arms do not become navigation obstacles, adjacent real obstacles remain, old robot-shaped ghosts clear, and incomplete/stale TF yields a visible degraded state.

### 4. Multi-sensor obstacle evidence and clearing

The active source configuration separates depth and LiDAR obstacle layers and merges them by maximum cost. Each sensor has distinct marking and clearing sources. Ground-filtered points mark obstacles; returns that include ground supply rays for clearing. This addresses two different failures: one sensor erasing another's obstacle, and removed objects leaving permanent marks because ground-removal also removed clearing rays. [Active costmap configuration][nav-config].

DimOS's ray mapper already clears free space. The useful addition is per-sensor evidence ownership and correct ray origins/occlusion, not another generic clearing implementation. Do not blindly concatenate obstacle-only depth points into the existing LiDAR stream. Preserve sensor identity, capture pose, ground/free-space evidence, and independent persistence before merging navigation occupancy.

Acceptance: a depth-only obstacle survives contradictory sparse LiDAR clearing, then disappears after valid observations establish that it was removed. Include low obstacles, thin objects, partial occlusion, and sensor dropout. Depth noise filtering must be evaluated alongside the delay it adds to detecting a newly entering obstacle.

### 5. Ground handling during walking

The source compares fixed-plane, per-frame RANSAC, and attitude-based ground removal, and injects sensor/TF mismatch to reproduce floor leakage during body sway. Its RANSAC fitter selects the largest plane without a gravity-normal constraint, so a dominant wall or tabletop can be misclassified. The temporal filter is in the uninitialized camera submodule and was not inspected. [Ground filter][ground], [simulation launcher][sim-launch].

Keep ground evidence for DimOS's height costmap and clearing rays while separately deriving obstacle points. Use calibrated attitude/height and a gravity-constrained fit for depth, with explicit unknown output when the fit is unreliable. Evaluate ramps, table edges, floor noise, and drop-offs; removing ground points does not establish support or safe terrain.

Acceptance: walking-like roll/pitch does not create obstacle rings, walls remain obstacles, and missing floor returns are not automatically declared traversable.

### 6. Independent stop/slow gating

The source configures a collision monitor after velocity smoothing and has bridge command expiry/watchdog behavior. Its real launcher defaults the collision monitor off, and the bridge normally reads ungated `/cmd_vel`. The reviewed simulation launcher does not launch the complete monitor/smoother path. These settings do not establish an active protective chain. [Real launcher][real-launch], [bridge launcher][bridge-launch].

Consider a DimOS command governor after movement arbitration, covering navigation and teleop: raw near-field obstacles, stale perception/localization, controller readiness, and command expiry produce bounded commands or a hold. Verify downstream behavior on process failure; publishing zero twist alone is not evidence of a timely physical stop. Nav2's documented collision monitor similarly operates separately from costmap/planner processing. [Official monitor documentation][nav2-monitor].

Acceptance: injected obstacles and stale feeds produce a measured response through the exact actuator path, for each supported controller.

### 7. Runtime posture and cargo collision envelope

The source projects wrist-frame object voxels into the base frame, unions them with the body footprint, adds an outward margin, and falls back to an enclosing box if polygon simplification is needed. It also updates inflation and goal tolerances and reasserts parameters after node restarts. Hard collision shape and preferred clearance are treated separately. [Geometry][carry], [runtime node][carry-node]. Nav2 officially supports changing footprints as manipulation changes robot shape. [Official footprint documentation][nav2-footprint].

For DimOS, start with measured empty-handed, arms-forward, and known-cargo envelopes. A manipulation-confirmed carry state can select geometry immediately, before the object becomes visible to a sensor. Later, observations can refine it. Use a versioned geometry state shared by global planning, local planning, speed governance, and the follower; invalidate plans when geometry grows. A static `robot_width` update alone will miss forward protrusion and rotation sweep.

Do not copy automatic shrink-on-missing-observation, spatial caps, or the source's inflation scale as guarantees. Its current node permits shrinking to the base footprint, and clipping extreme points can exclude real cargo. Missing observation is not release. Keep a conservative registered-object bound until confirmed release; treat out-of-bound cargo as a distinct failure.

Acceptance: an empty G1 can pass a doorway that a loaded G1 rejects; in-place yaw checks the cargo sweep; restart or temporary occlusion cannot silently reduce the acknowledged collision envelope.

### 8. Held-object identity and cross-sensor filtering

The source grows a voxel component from hand-adjacent returns and publishes wrist-frame object voxels for filtering sparse LiDAR returns. No seed or excessive growth removes no object points. This is useful algorithmic material, but its hardware notes report hand-contacted obstacles being mistaken for cargo and objects falling outside the camera view. Acquisition/tracking with object identity is described as follow-up work, not established by the inspected `HeldConnectedFilter`. [Algorithm][held], [reported hardware limitations][carry-notes].

Prefer an attached-object record from DimOS manipulation: object ID, grasp transform, geometry, confidence, timestamp, and confirmed release. Separate “remove this owned object from obstacle observations” from “include it in robot collision geometry.” Use hand-connected growth only as bounded evidence around that registered object, with merge/reacquisition checks. Reuse the sensor-independent wrist-frame representation if measurements justify it.

Acceptance: touching a table does not erase the table; carrying an object outside depth FOV retains its collision envelope; after release the object becomes an environmental obstacle.

### 9. Reachable semantic approach and final alignment

DualMap's inspected integration boundary is a target pose relayed to `NavigateToPose`; semantic mapping already overlaps DimOS's spatial-memory navigation. The more useful lesson is to generate a reachable approach pose rather than navigate to an object's surface or center. The source's hardware notes identify near-object goals and final yaw with cargo as recurring failures. [Goal relay][relay], [hardware notes][carry-notes].

Extend DimOS goal resolution to consider target geometry, stand-off distance, arm reach, navigation clearance, and the rotation envelope. Use task-specific arrival criteria: reach a room, approach a pickup surface, or align for handover. Do not silently relax an alignment requirement when the footprint grows.

Acceptance: “go to the table” selects a reachable free pose; a pickup approach meets its position/yaw requirements or reports which condition is infeasible.

### 10. Simulation scenarios that expose perception failures

Adapt the source's raycast self/held/environment labels, moving obstacles, dropouts, sensor-only vibration, and filter scoring. Keep ground truth for scoring separate from inputs used by the operational algorithm. The source's carry simulation defaults to a ground-truth held hull because its camera can miss the cargo; that validates footprint consumption, not sensor-derived cargo acquisition. [Launcher][sim-launch], [filter probe][probe].

DimOS already has MuJoCo and scene composition. Extend these rather than import a second simulator, port layout, and process launcher. G1 WBC simulation currently excludes robot geometry from its LiDAR setup; a body-filter evaluation needs a deliberate mode that includes self returns.

Use fast planar navigation tests for perception/planning regressions and whole-body tests for sway, tracking, stopping, and balance. Neither replaces moving-hardware validation. In the source probe, environment loss is removed environment points divided by all environment points, despite some documentation describing a different denominator; preserve metric definitions explicitly.

### 11. Stable routes, goal lifecycle, and recovery

The source behavior tree keeps a route until the goal changes or the path becomes invalid to reduce left/right oscillation. DimOS's A* is already event-driven and its newer Rust local planner has incumbent-route revalidation and commitment hysteresis. Reuse the symmetric-obstacle test and route-change metrics before adding another replanning layer. [Behavior tree][bt], DimOS A* (`/dimos/navigation/go2/replanning_a_star/global_planner.py`), Rust planner (`/dimos/navigation/local_planner/rust/src/planner.rs`).

Adapt explicit accepted/active/succeeded/aborted/canceled outcomes and bounded recovery attempts. The current A* module declares `navigation_state` with a TODO to publish it; the navigation language skill can return while movement continues. Keep movement ownership until terminal status, support preemption, and distinguish timeout, blocked geometry, stale localization, and controller failure.

Recovery should depend on measured G1 capabilities and available clearance. The source's generic costmap clearing, spin, wait, and backup sequence is a reference; clearing occupancy must not remove a persistent physical obstacle, and cargo can make rotation/backup infeasible.

### 12. Prior maps and optional Nav2 backend

Pre-mapping followed by localization is useful for repeated tasks in a known site. Evaluate a versioned map plus localization quality and an independent local obstacle view. The source's PCD-to-grid converter declares empty cells inside the point-cloud bounding box free; that shortcut lacks observed-free-space evidence and should not be copied into DimOS's map semantics. FAST-LIO saved-map localization itself is outside this checkout. [Map converter][map-converter], [source overview][overview].

If a comparison backend is worthwhile, wrap Nav2 goals, cancellation, status, and frame conversion behind DimOS's existing navigation interface, with command output still passing through the selected DimOS execution path. Use the same scenes, sensors, geometry, and evaluation metrics. Do not run two active velocity-producing navigation providers concurrently.

Three adoption choices are reasonable: selective native improvements (recommended), an optional Nav2 provider for comparison/ROS interoperability, or wholesale stack replacement. The last has the highest migration cost and no demonstrated need from this comparison. SmacPlanner2D/RPP are reference alternatives; the README's MPPI description does not match the active source configuration.

## Suggested order

1. Establish the frame/time/execution contract and goal-centered recording using the current sensor-only evidence as the starting point.
2. Validate empty-handed navigation: complete body poses, the existing self-filter, depth/LiDAR evidence, ground handling, and a verified stop/slow path.
3. Add known-object carry envelopes and reachable manipulation approach poses; then evaluate perception-derived attached-object tracking.
4. Run paired simulation/replay regressions and moving-hardware evaluations. Compare a Nav2 provider only if there is a concrete planner/controller limitation or ROS integration need.

The first reviewable deliverable could be a G1 navigation recording/report plus three repeatable cases: a depth-only obstacle, a moving arm near a real obstacle, and a known carried box at a doorway. These expose practical gaps before choosing a new planner.

## Verification performed for this study

Ran the source's `test_body_model.py`, `test_held_model.py`, and `test_carry_footprint_model.py` directly with DimOS's Python environment and bytecode writing disabled. All three printed `ALL PASS` and exited 0. The footprint check emitted a NumPy warning while deliberately exercising nonfinite input. These are standalone check scripts, not collected pytest tests; an initial pytest attempt stopped during collection because the body script exits at import.

No ROS integration, analyzer E2E, simulator rollout, missing submodule implementation, or physical walking was tested. Historical success/failure reports are attributed to source documents, not reproduced results.

## Source references

Source links below are pinned to the reviewed clean source commit. DimOS links refer to this workspace's current files.

[overview]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/README.md
[body]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/self_filter/body_model.py
[self-filter]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/self_filter/self_body_filter.py
[held]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/self_filter/held_model.py
[carry]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/self_filter/carry_footprint_model.py
[carry-node]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/self_filter/carry_footprint_node.py
[carry-notes]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/docs/0723_carry_실기현황_정리.md
[ground]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/nav/d435_ground_filter.py
[nav-config]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/config/nav2_params.yaml
[bt]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/config/bt_replan_on_invalid.xml
[bridge]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/src/nav/cmd_vel_to_homie.py
[bridge-launch]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/launch/run_cmd_vel_bridge.sh
[real-launch]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/launch/run_nav2_stack.sh
[sim-launch]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/sim/run_sim_planar.sh
[probe]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/sim/diag_self_filter_probe.py
[relay]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/sim/dualmap_goal_relay.py
[analyzer]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/pc/nav_analyzer/lib/analyzer.js
[analyzer-doc]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/docs/sim/nav_analyzer.md
[map-converter]: https://github.com/genonai/physical_ai_navigation/blob/81573bf3a938fbe1c97824b543b7ea7846e946ac/scripts/pcd_to_occupancy_grid.py
[nav2-footprint]: https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/
[nav2-monitor]: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/
