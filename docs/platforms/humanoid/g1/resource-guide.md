# Unitree G1 resources for a humanoid manipulation service

Resource review performed on 2026-10-02 against local DimOS commit `7529659c8`; workstation simulation was subsequently exercised on 2026-10-06 (Asia/Seoul). This guide prioritizes real G1 manipulation, then the navigation, perception, learning, and operations needed for a service. Findings come from repository source, primary upstream documentation, the read-only PC2 inventory/telemetry check, and the simulation test below. No real-robot motion commands or controller handoffs were performed. The user confirmed two 29-body-joint robots: `pc2` has no hands attached; `pc2_222` has two Dex3 hands.

The next milestone is a stationary pick-one-object-and-place-in-a-tray service on `pc2_222`, confirmed by the user on 2026-10-06. The [powered-on inventory](/docs/platforms/humanoid/g1/pc2-222-inventory-2026-10-06.md) verified 29 body plus 7+7 Dex3 joint-state values and D435i RGB/depth acquisition at 640×480/15 fps. The [native DimOS camera setup and repeatable check](/docs/platforms/humanoid/g1/pc2-222-camera-setup.md) now pass on `.222`. Camera-to-robot calibration, timestamp synchronization, command ownership, and supported arm-control mode remain to be established. Follow the [service development and commissioning plan](/docs/platforms/humanoid/g1/service-development-plan.md) and [official control-interface review](/docs/platforms/humanoid/g1/control-interface-review.md) for the proposed architecture, integration gaps, and hardware evidence gates.

**Recommendation:** begin the hardware walkthrough with PC2 inventory and subscription-only telemetry. Use the G1 teleoperation and manipulation composition as the first substantial code reference: it connects arm planning, operator input, recording, and a custom balance controller. Running it changes control ownership, so it is not a connectivity test. Budget separate work for the actual hand, object perception, and verified task completion. The inspected configuration has 29 body motors and rubber-hand endpoints, so it does not establish compatibility with a different G1 variant or dexterous hand. Sources: teleop composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`), robot model (`/dimos/robot/unitree/g1/manip_config.py`), DDS connection (`/dimos/robot/unitree/g1/wholebody_connection.py`).

**Tested MuJoCo exercise on the workstation.**

With the real robot powered off, the following command started all six modules and passed the startup health check in `/home/jihun/work/dimos_dev`:

```bash skip
MUJOCO_GL=egl .venv/bin/dimos --simulation mujoco --scene-package none \
  --viewer none run unitree-g1-teleop --daemon
```

The simulator exposes 29 body joints through shared memory. GR00T balance/walk ONNX models loaded with CUDA available, and the simulator automatically armed the balance task. Module discovery confirmed `MujocoSimModule` and no `G1WholeBodyConnection`. The manipulation page responded at `http://localhost:8095`; the WebXR page responded at `https://localhost:8443/teleop` (self-signed certificate). Headset interaction itself was not tested.

Runtime discovery returned the exact group IDs **`left_arm` and `right_arm`**, with module instance `G1Manipulation`. Each arm completed a planned +0.1 rad elbow movement and a return trajectory at speed scale 0.2. All four executions reported `COMPLETED`, measured joints changed, and the maximum observed endpoint error across these checks was approximately 0.053 rad (3 degrees). This is a functional motion check, not a precision or task-success evaluation.

To explore this simulation, activate `.venv`, open `dimos shell`, and use:

```python skip
from dimos.manipulation.sdk import Arm

left = Arm.from_app(app, instance_name="G1Manipulation", group="left_arm")
print(left.info)
print(left.joints())
print(left.pose())
initial = left.joints()
target = initial.copy()
target[3] += 0.1
left.move_joints(target, speed_scale=0.2, timeout=15.0)
left.move_joints(initial, speed_scale=0.2, timeout=15.0)
```

The run ID was `20261006-000320-unitree-g1-teleop`; its logs are under `logs/20261006-000320-unitree-g1-teleop/`. Use `dimos status` to check whether it is still running and `dimos stop` to end it. This composition has no simulated RGB camera or gripper, and the teleop composition excludes navigation. Continue with WebXR and recording before selecting a separate navigation composition.

**Python exercise: inspect → plan → preview → execute.**

The reusable G1 simulation example (`/scripts/demo_g1_sim_manipulation.py`) attaches to this running stack. It requires `MujocoSimModule`, rejects a runtime containing `G1WholeBodyConnection`, and selects the `G1Manipulation` instance explicitly. Keep browser motion controls, WebXR, and other motion scripts idle while it runs: both arms share one pending plan on this module.

Open `http://localhost:8095` on the workstation (or `http://192.168.0.62:8095` from the same LAN), then run from this checkout:

```bash skip
.venv/bin/python scripts/demo_g1_sim_manipulation.py
```

1. Read the printed joint names, positions in radians, tip poses in metres, and group capabilities. Both groups have seven joints and no gripper.
2. For each arm, the script plans a +0.1 rad elbow target with `plan_to_joints()`. Planning produces a pending plan; it does not execute it.
3. `preview_plan()` requests a two-second visualization. Inspect the preview, then press Enter to call `execute()` with that exact plan ID. The script prints execution status and measured joint error, then repeats this sequence for the return. There are four Enter prompts in total.
4. Finally, `plan_to_poses()` requests an unreachable world-frame tip position of `[10, 10, 10]` metres. The script checks for an IK failure, no generated plan, and no pending plan. It never executes this request.

For an unattended simulation check, use `.venv/bin/python -u scripts/demo_g1_sim_manipulation.py --auto`. This replaces each Enter prompt with a two-second delay. On 2026-10-06, that command exited successfully with these measured results:

| Trajectory | Execution status | Maximum joint endpoint error |
|---|---|---|
| Left elbow +0.1 rad | `COMPLETED` | 0.0116 rad |
| Left return | `COMPLETED` | 0.0387 rad |
| Right elbow +0.1 rad | `COMPLETED` | 0.0278 rad |
| Right return | `COMPLETED` | 0.0472 rad |

The unreachable target returned `FAILED`, with `IK failed: NO_SOLUTION: Pink IK did not converge within the iteration budget`. Preview RPCs succeeded; browser rendering was not visually inspected during this automated run. The 0.08 rad error threshold in the example is a functional check for this exercise, not an accuracy specification. Every run captures fresh starting joints; controller tracking error means repeated runs can drift.

The initial attempt also caught a pre-existing `PLANNING` state left by an earlier collision-planning exception. Logs showed the exception and a completed last execution. Calling `clear_planned_path()` cleared that stale planning state and returned the module to `IDLE`; controller code was unchanged. Clearing a plan does not cancel an executing trajectory. The example refuses a busy/faulted start, requests `cancel()` if its own motion sequence fails, and reports the cancellation result without retrying motion. Ctrl-C at the first interactive preview prompt was also exercised: the pending plan was cleared before any execution call. Cancellation returned the cached `COMPLETED` result of the previous run; that does not mean the previewed plan executed. On normal exit the example disconnects its client and leaves MuJoCo running.

**Cartesian exercise: reach a hand-tip XYZ target.**

After the joint exercise, run the Cartesian-reaching example (`/scripts/demo_g1_sim_cartesian.py`). It attaches to the same MuJoCo stack and selects `left_arm`. Open the manipulation panel before starting, and keep other motion controls idle:

```bash skip
.venv/bin/python scripts/demo_g1_sim_cartesian.py
```

There are two Enter prompts: one to execute the reach after previewing it, and one to execute the return. The script captures the starting joints and hand-tip pose, adds `0.01` metres to Z, and copies the starting orientation. Its core request is:

```python skip
initial_pose = arm.pose()
target = PoseStamped(
    frame_id="world",
    position=[initial_pose.x, initial_pose.y, initial_pose.z + 0.01],
    orientation=initial_pose.orientation,
)
planned = arm.rpc.plan_to_poses({arm.info.id: target}, speed_scale=0.2)
```

Inverse kinematics finds arm joint values for the requested tip pose; joint-path planning then finds a path to that configuration. `plan_to_poses()` targets the endpoint and does not require the hand to follow a straight line. The script previews and executes the exact returned plan ID, reads feedback after a 0.5-second settling interval, and reports XYZ error, Euclidean position error, and quaternion orientation error. It then plans and executes a return to the captured joint values.

These are coordinates reported by the manipulation model in its `world` frame; the configured G1 base link is `pelvis`, and the tip is `left_rubber_hand`. Pose feedback is forward kinematics from joint feedback, not an independent hand-position measurement. Camera/object coordinates will require a calibrated transform into the planning frame. Changing a pose's `frame_id` alone does not transform its coordinates.

On 2026-10-06, `.venv/bin/python -u scripts/demo_g1_sim_cartesian.py --auto` completed both trajectories and exited successfully:

| Measurement | Reach | Return |
|---|---|---|
| Execution status | `COMPLETED` | `COMPLETED` |
| XYZ error | [-3.07, +0.51, -3.03] mm | [-0.78, +1.24, +0.32] mm |
| Position error | 4.34 mm from requested target | 1.50 mm from starting tip position |
| Orientation error | 0.85 degrees | 0.56 degrees |

The requested upward change was 10 mm; the observed change was 6.97 mm. Maximum return joint error was 0.0062 rad. The script reports whether reach position error is within a 5 mm learning target, separately from trajectory completion. This run met that target; it is one measurement, not an accuracy guarantee. An accuracy miss is printed as `NOT MET`, while failed planning/execution stops the exercise without an automatic return or retry. As in the first exercise, it requests cancellation on errors/interrupts and leaves MuJoCo running on exit.

**Straight-line exercise: Cartesian path with collision checking.**

Use the same example with `--linear` to request a straight +10 mm translation of the left hand tip:

```bash skip
.venv/bin/python scripts/demo_g1_sim_cartesian.py --linear
```

The first Enter prompt starts planning and execution immediately. This reach has no blue preview because the SDK call performs both operations together:

```python skip
result = arm.move_linear(
    dz=0.01,
    check_collision=True,
    speed_scale=0.2,
    timeout=15.0,
)
```

Watch the grey robot's feedback during the reach. The second Enter prompt follows a blue preview of the return to the captured starting joints. That return uses joint-path planning and is not constrained to a straight hand-tip path. Without `--linear`, the previous endpoint-planning exercise remains available.

`move_linear()` requests a Cartesian path through intermediate poses, preserving the starting orientation. With `check_collision=True`, it checks against the configured collision world and rejects a colliding path; it does not route around an obstacle. In this G1 stack, the collision model includes the measured body joints, while only the selected arm moves through the planned path. The teleop composition does not automatically populate that world with perceived room objects. Collision checking is a planning check against the model, not a guarantee about every physical state during execution.

On 2026-10-06, `.venv/bin/python -u scripts/demo_g1_sim_cartesian.py --linear --auto` completed both trajectories. The reach returned `RoboPlan Cartesian path found`, with 149 path waypoints and a 1.48-second trajectory. Measured results were:

| Measurement | Reach | Return |
|---|---|---|
| Execution status | `COMPLETED` | `COMPLETED` |
| XYZ error | [-3.10, +0.81, -3.02] mm | [-0.89, +1.85, +0.08] mm |
| Position error | 4.40 mm | 2.06 mm |
| Orientation error | 0.90 degrees | 0.68 degrees |

Observed upward displacement was 6.98 mm for the requested 10 mm; maximum return joint error was 0.0058 rad. The script measures endpoint feedback after settling, not the maximum deviation along the executed path. This tested a successful path with collision checking enabled; an intentionally blocked path was not tested in this lesson. The `--auto` flag skips the prompts, so the linear reach starts as soon as state capture finishes.

**First hardware lesson: verified setup on 2026-10-02.**

| Item | Observed value or user confirmation |
|---|---|
| Current learning robot | SSH alias `pc2`, Wi-Fi `192.168.0.223`, 29 body joints, no hands attached |
| Robot for later grasping work | SSH alias `pc2_222`, `192.168.0.222`, 29 body joints and two Dex3 hands; configuration supplied by user, not inspected in this check |
| PC2 host and hardware | `unitree-g1-nx`; NVIDIA Jetson Orin NX; aarch64; approximately 16 GB RAM |
| Operating system | Ubuntu 22.04.5; NVIDIA L4T R36.4.3 |
| Internal robot network | `enP8p1s0`, `192.168.123.164/24`; route to PC1 `192.168.123.161` uses this interface |
| SSH/Wi-Fi network | `wlx140a02f31e42`, `192.168.0.223/24` |
| DimOS checkout on PC2 | `/home/unitree/dimos`, branch `main`, commit `7dd83be80`; differs from this workstation's reviewed checkout |
| DimOS environment | `/home/unitree/dimos/.venv/bin/python`, Python 3.12.13; DimOS package metadata version 0.0.14 |

The DimOS virtual environment has neither CycloneDDS nor either inspected Unitree SDK distribution installed. Its own `pyproject.toml` declares the `unitree-dds` extra with `unitree-sdk2py-dimos>=1.0.2` and `cyclonedds>=0.10.5`. The general `dds` extra is not the complete Unitree dependency set. A system Python package being present does not make it available inside the DimOS virtual environment.

Subsequent setup: PC2's existing native CycloneDDS installation at `~/cyclonedds/install` was verified as 0.10.5. After installing the `unitree-dds` extra, the user confirmed imports from DimOS's Python 3.12 environment with CycloneDDS 0.10.5 and `unitree-sdk2py-dimos` 1.0.3. The missing-dependency observation above records the initial inventory, not the current installation state.

System Python 3.10 has CycloneDDS 0.10.2 and Unitree SDK metadata version 1.0.1, but importing that installed SDK fails: its package initializer imports `b2`, while that package directory is absent. The existing `/home/unitree/unitree_sdk2_python` source checkout contains the missing modules and passed an import check. Running Python from that checkout provided the SDK for this session's telemetry probe; no package installation or persistent environment change was made. This does not repair the incomplete installed package or validate DimOS's dependency requirements.

The five-second probe created only a `ChannelSubscriber` for `rt/lowstate`, using DDS domain `0`, interface `enP8p1s0`, and the `unitree_hg` `LowState_` type. It received **5,211 messages** over a measured five-second sample span; the robot tick advanced from `4002204` to `4007206`. All first 29 motor-position values were finite, and IMU orientation, angular velocity, and acceleration arrived. The message contains 35 storage slots; the confirmed body has 29 joints. Reported `mode_machine` was `5`; the raw version field was `[0, 0]`, which does not establish a firmware release. Reception proves this DDS observation path works, not controller readiness or robot health.

For our next lesson, prepare and verify the Unitree DDS dependencies in DimOS's own environment, matching its installed checkout. Then choose the controller route before starting a hardware blueprint. `dimos hardware g1 status` queries a running DimOS coordinator; it is not a standalone DDS connectivity probe. Its implementation is in the hardware CLI (`/dimos/cli/hardware/g1.py`).

**Official G1 EDU hardware baseline for this walkthrough.**

Unitree documents PC1 as the motion-control computer at `192.168.123.161/24` and PC2 as the development computer at `192.168.123.164/24`. These are factory addresses; confirm the installed network configuration. DimOS on PC2 communicates with robot services over the internal network. Its DDS network-interface selection refers to PC2's robot-facing NIC. Successful SSH access alone does not establish DDS communication. Source: [Unitree networking guide](https://github.com/unitreerobotics/xr_teleoperate/wiki/CycloneDDS-and-UnitreeSDK-%28en%29).

G1 EDU is a configurable platform with secondary-development support. Unitree lists 23–43 total DoF depending on options. A 29-body-joint configuration has 12 leg, three waist, and 14 arm joints; adding two seven-joint Dex3 hands gives 43 by arithmetic. Body and hand counts must therefore be recorded separately. The current product page also lists multiple optional computing-module models, so PC2's exact GPU, RAM, and OS cannot be inferred from the name alone. Sources: [G1 specifications](https://www.unitree.com/g1/), [Dex3 specifications](https://www.unitree.com/Dex3-1/).

The official interfaces separate three control choices:

| Choice | Official interface | Meaning for a manipulation service |
|---|---|---|
| Request locomotion | [G1 LocoClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_client.hpp) | Sends velocity and posture requests to Unitree's locomotion service; application code does not generate leg joint targets. |
| Command the upper body while retaining vendor motion control | [Seven-joint arm example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp) using `rt/arm_sdk` | Publishes weighted upper-body commands. Unitree's [XR teleoperation guide](https://github.com/unitreerobotics/xr_teleoperate) explicitly documents simultaneous motion control through `--motion`, with supported-mode and end-effector restrictions. Verify firmware and configuration before choosing this route. |
| Supply a body controller | [Direct low-level example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/low_level/g1_low_level_example.py) using `rt/lowcmd` | Releases the active motion mode and sends body motor commands. The custom controller must provide the required body control; vendor balance must not be assumed to continue. |

Unitree also provides an explicit [user-controller handoff example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_userctrl_dds_example.cpp) using `rt/user_lowcmd`, `SwitchToUserCtrl`, and `SwitchToInternalCtrl`. This is a separate firmware-dependent route, not a reason to mix control interfaces. The [official user manual](https://marketing.unitree.com/article/en/G1/User_Manual.html), printed page 21, explains the underlying issue: the built-in controller starts at boot, and direct SDK commands can conflict with its output. This review read relevant manual image pages and official GitHub documentation/source; the support portal's developer articles did not return readable bodies.

**Implication for DimOS:** manipulation does not inherently require GR00T or SONIC. Unitree offers the vendor-motion-plus-arm route above. This checkout's G1 teleop blueprint (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`) instead selects DimOS's GR00T balance/walk backend. That is a repository implementation choice; adding an `arm_sdk` manipulation backend would be integration work. No built-in GEAR-SONIC connection was identified in this review.

Hands are another interface: Unitree's [Dex3 example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/dex3/g1_dex3_example.cpp) commands seven motors per hand through separate hand command/state topics. Body-joint control alone does not implement grasping. That executable moves fingers automatically. For the first observation exercise, the [subscription-only state example](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/read_low_state_hg.cpp) is the relevant pattern; its current print loop covers only twelve motors and needs adaptation to inspect the full G1 body.

**Read these repository resources first, in this order.**

| Priority | Resource | Why it matters for your service | Current scope |
|---|---|---|---|
| 1 | [G1 hardware guide](/docs/platforms/humanoid/g1/index.md) and hardware CLI (`/dimos/cli/hardware/g1.py`) | Network access, installation, activation, ready pose, status, and shutdown workflow. | Guide requires G1 EDU with SDK/SSH access. Read the documentation discrepancies below before treating it as an exact runbook. |
| 2 | G1 teleop blueprint (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`) | Best single file for understanding the manipulation stack: GR00T control core, WebXR, planner/Viser, RealSense, episode monitor, and recorder. | Upper-body workflow; navigation is excluded and headset sticks are not connected to walking. |
| 3 | G1 manipulation model (`/dimos/robot/unitree/g1/manip_config.py`), [planning groups](/docs/capabilities/manipulation/planning_groups.md), and [Python API](/docs/capabilities/manipulation/python_api.md) | Named left/right arm groups, joint and pose planning, straight moves, plan preview, execution, and cancellation. | Each arm has seven joints. All 29 measured body joints inform collision checking; only the arms are planned. Tips are `left_rubber_hand` and `right_rubber_hand`; no gripper is configured. |
| 4 | G1 WBC composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py`), GR00T task (`/dimos/control/tasks/g1_groot_wbc_task/g1_groot_wbc_task.py`), and control coordinator (`/dimos/control/README.md`) | Understand balance while reaching and which controller owns each joint. | Balance/walk ONNX policies own 15 leg/waist joints. Arm trajectories and WebXR share the other 14; WebXR has higher priority. This is not evidence of general contact-rich whole-body planning. |
| 5 | WebXR input (`/dimos/teleop/webxr/README.md`), G1 IK (`/dimos/robot/unitree/g1/teleop_ik.py`), and [Pink tuning](/docs/capabilities/manipulation/pink_ik_tuning.md) | Bimanual reference capture, inverse kinematics, joint limits, filtering, tracking error, and operator takeover. | Shared bimanual task requires both grips. The G1 model and objective parameters need revisiting when hand geometry changes. |
| 6 | Imitation learning (`/dimos/imitation/README.md`), DataPrep example (`/dimos/imitation/dataprep/example_config.json`), and G1 recorder composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`) | Turn operator demonstrations into LeRobot v3 or HDF5 datasets with synchronized observations and episode boundaries. | G1 adds absolute controller-pose streams to measured joints, RGB, and episode status. These controller poses are not the final IK joint commands. |
| 7 | LeRobot policy runtime (`/dimos/imitation/policy/lerobot/README.md`) | Local integration point for learned skills: checkpoint loading, preflight, fresh-observation checks, action chunks, rollout status, and stop. | Requires absolute targets in native joint coordinates. The G1 teleop composition does not include this policy module; G1 and hand action mapping still require integration. |
| 8 | [xArm grasping guide](/docs/capabilities/manipulation/xarm-grasp.md), pick-and-place module (`/dimos/manipulation/pick_and_place_module.py`), and grasp verification (`/dimos/manipulation/grasp_verification.py`) | Useful references for object scan → grasp proposal → approach → grasp → place, including perception and scene obstacles. | Shipped examples target xArm. Adapting them to G1 requires hand geometry/control, camera transforms, reachability, collision configuration, and a suitable success signal. |

The distinction between the two G1 control paths matters. The standard agent composition (`/dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`) exposes navigation, observation, speech, and the G1 movement/gesture container (`/dimos/robot/unitree/g1/skill_container.py`). Its arm actions are vendor gestures such as waving and handshakes. The GR00T manipulation composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py`) instead connects a low-level whole-body backend and coordinator tasks. Recommendation: decide control ownership explicitly when adding service skills; combining these compositions requires more than enabling all blueprints.

**Resources for the other parts of a humanoid service.**

| Topic | Useful repository resources | Application and limits |
|---|---|---|
| Navigation and mapping | G1 navigation blueprint (`/dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_simple.py`), onboard stack (`/dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_onboard.py`), GR00T navigation composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py`), [navigation overview](/docs/capabilities/navigation/index.md) | The older path combines FastLIO2 and vendor high-level control. The real GR00T path composes MID-360/Point-LIO, voxel mapping, height costs, A* replanning, and movement management. Study their sensor and controller dependencies separately. A height costmap is not a footstep or manipulation-contact planner. |
| Localization and returning to places | [Relocalization guide](/docs/capabilities/navigation/relocalization.md) and [spatial memory](/docs/capabilities/memory/index.md) | References for persistent maps and recalling locations. The relocalization examples are Go2-oriented; verify integration with the selected G1 odometry and control path. |
| Object perception and memory | G1 perception composition (`/dimos/robot/unitree/g1/blueprints/perceptive/_perception_and_memory.py`), [perception overview](/docs/capabilities/perception/index.md), and [xArm grasping](/docs/capabilities/manipulation/xarm-grasp.md) | G1 currently composes experimental object tracking and spatial memory. WorldBelief is documented as an experimental xArm6 stack. Reuse the ideas without assuming they provide G1 grasp poses or reliable object identity automatically. |
| Cameras and coordinate frames | RealSense module (`/dimos/hardware/sensors/camera/realsense/README.md`), [camera calibration](/docs/usage/camera_calibration.md), and [transforms](/docs/usage/transforms.md) | Intrinsics, RGB-D geometry, camera mounting, and timestamped transforms are central to reaching a detected object. The chessboard workflow calibrates the camera; the robot-to-camera extrinsic transform is an additional requirement. |
| Simulation and repeatable scenes | Scene packages (`/dimos/simulation/scenes/README.md`) and G1 teleop composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`) | Office/supermarket scene assets and MuJoCo provide useful development environments. This G1 teleop simulation records joints/controller poses but does not supply a color camera by default. Add one before relying on it for visual policy data. |
| Language and service orchestration | [Agent architecture](/docs/capabilities/agents/index.md), G1 skills (`/dimos/robot/unitree/g1/skill_container.py`), and G1 prompt (`/dimos/robot/unitree/g1/system_prompt.py`) | MCP and typed skills can expose application actions. Recommendation: represent a service action such as a handover with explicit preconditions, execution status, cancellation, and observed completion. The current G1 gesture API is not an object-handover implementation. |
| Operator interface and remote support | [Web overview](/docs/web/index.md), [Web SDK](/docs/web/web_sdk.md), [bridge](/docs/web/bridge.md), and [relay](/docs/web/relay.md) | Reusable camera, telemetry, teleoperation, and application UI infrastructure. The walkthrough uses Go2; a G1 service needs its own channels, status model, and control wiring. |
| Installation and onboard performance | [Installer](/docs/installation/index.md), [DDS setup](/docs/usage/transports/dds.md), dependency extras (`/pyproject.toml`), and [profiling](/docs/development/profiling_dimos.md) | Unitree DDS dependencies are distinct from the general manipulation extra. The G1 blueprint uses a 100 Hz real coordinator with 50 Hz policy inference; camera and recorder workers are isolated and images use shared memory. Treat these as configured rates, not measured guarantees for your machine. |
| Evaluation and incident diagnosis | [Testing guide](/docs/development/testing.md), G1 planning integration test (`/dimos/robot/unitree/g1/test_g1_manipulation_integration.py`), [agent evals](/docs/usage/evals.md), and [runtime CLI](/docs/usage/cli.md) | Existing tests and per-run logs are useful foundations. The G1 planning test is marked `self_hosted`, so the default fast test selection excludes it. Agent evaluation infrastructure does not by itself establish physical task success. |

**Primary external resources to compare with the local implementation.**

These sources document their own integrations. Their presence does not establish compatibility with this DimOS checkout. The order below prioritizes manipulation; SDK documentation remains foundational for every route.

| Priority | Resource | What to research |
|---|---|---|
| 1 | [Unitree XR teleoperation](https://github.com/unitreerobotics/xr_teleoperate) | G1 23/29-DoF teleoperation and recording, with Dex1, Dex3, Inspire, and BrainCo options. Match the hand and camera setup. The documented internally wired Dex1 route excludes simultaneous motion mode. |
| 2 | [Unitree LeRobot integration](https://github.com/unitreerobotics/unitree_lerobot) | Demonstration conversion, training, and real G1 arm/hand deployment. Examples include ACT, diffusion, π0/π0.5, and GR00T. The former `unitree_IL_lerobot` URL redirects here; current documentation specifies LeRobot dataset v3.0. |
| 3 | [Hugging Face LeRobot G1 guide](https://huggingface.co/docs/lerobot/unitree_g1) | Direct support for 23/29-DoF G1, real hardware and MuJoCo, GR00T/Holosoma locomotion controllers, exoskeleton teleoperation, training, and rollout. Compare this native integration before maintaining a separate policy-to-hardware adapter. |
| 4 | [Unitree Isaac Lab simulation](https://github.com/unitreerobotics/unitree_sim_isaaclab) | G1 manipulation task scenes and hand variants, including Dex1/Dex3/Inspire examples and whole-body tasks. Useful for checking observation/action interfaces against a matching simulated embodiment. |
| 5 | [NVIDIA GR00T WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) | G1 SONIC checkpoints, training, teleoperation, and C++ deployment, alongside Decoupled WBC. Research this when balance, reaching, and walking need coordinated motion. |
| 6 | [NVIDIA Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T) and [G1 VLA workflow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_workflow.html) | Custom-data fine-tuning and policy serving. The documented SONIC → N1.7 workflow covers collection through real G1 inference, with a specific latent-motion-plus-hands action contract. |
| 7 | [Unitree G1 specifications](https://www.unitree.com/g1/) | Distinguish standard and EDU development access and available joint/hand configurations. Confirm the delivered configuration rather than choosing a controller from the product name alone. |
| 8 | [Unitree SDK2 G1 examples](https://github.com/unitreerobotics/unitree_sdk2/tree/main/example/g1) | C++ examples for high/low-level control, Dex3, and audio. Use these as the reference for a hand adapter and the selected hardware control mode. |
| 9 | [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python) | Python DDS request/response and pub/sub integration, including G1 examples. Compare upstream dependencies with this repo's `unitree-sdk2py-dimos` package rather than assuming they are identical. |
| 10 | [Unitree ROS2](https://github.com/unitreerobotics/unitree_ros2) | Official ROS2/CycloneDDS integration and G1 examples. Useful when existing perception or navigation components use ROS2; transport integration still needs a service/control design. |
| 11 | [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco) | Low-level controller and SDK-interface simulation. Its documentation notes that simulated sport-state availability can differ from real hardware after the motion service is disabled. |
| 12 | [OpenPI](https://github.com/Physical-Intelligence/openpi) | Alternative policy training/serving, observation/action transforms, and normalization. The opened top-level examples do not establish a turnkey G1 integration; treat it as a model comparison requiring an adapter. |
| 13 | [Nav2 robot setup guide](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/) | General navigation requirements for transforms, odometry, sensing, and footprint configuration. Useful if adopting ROS2 navigation; a G1 locomotion interface and localization remain your integration work. |

**The policy contracts are not interchangeable.** The [SONIC VLA workflow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_workflow.html) specifies 64 motion-latent values plus seven joints for each hand. DimOS's LeRobot runtime (`/dimos/imitation/policy/lerobot/README.md`) expects absolute native joint targets, and its current G1 WBC task (`/dimos/control/tasks/g1_groot_wbc_task/g1_groot_wbc_task.py`) uses balance/walk ONNX policies. Inference from these interfaces: SONIC outputs require a dedicated execution integration, while a policy trained for the existing joint-target contract can follow a different route. The SONIC tutorial also describes LeRobot v2.1 data, whereas [Unitree's adapter](https://github.com/unitreerobotics/unitree_lerobot) describes v3.0. Check dataset format, features, joint order, and action semantics together.

**Upstream SONIC update, 2026-10-06:** [the issue/PR and source review](/docs/platforms/humanoid/g1/sonic-upstream-status.md) found an open implementation stack #4304 → #4305 → #3557. Native SONIC remains absent from the reviewed main and local checkout. The PR runs SONIC inference inside DimOS with all 29 body joints under its control; it is not a bridge to the user's existing `g1_deploy_onnx_ref`. The user chose to retain that existing controller while inspecting integration options. Hardware validation and Dex3/service composition remain separate work.

**Integration gaps and documentation discrepancies found in this checkout.**

1. **Hand support is the first manipulation decision.** The inspected G1 hardware path lists 29 body motors, and the manipulation configuration has rubber-hand tips without gripper groups. I found no Dex3/Dex1 driver in the inspected G1 composition. Plan the end-effector adapter, feedback, geometry, joint naming, limits, and demonstration/action schema together. Sources: connection (`/dimos/robot/unitree/g1/wholebody_connection.py`), model (`/dimos/robot/unitree/g1/manip_config.py`), [custom hardware integration guide](/docs/capabilities/manipulation/adding_a_custom_arm.md).
2. **Self-collision support does not mean the scene is fully perceived.** The G1 planner sees the full body model, but its teleop composition disables RealSense point-cloud production and does not wire a scene-mapping pipeline. Recommendation: add calibrated environment obstacles for autonomous reaches. The xArm grasping composition is a concrete reference for point-cloud self-filtering and voxel obstacles. Sources: G1 composition (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`), [xArm scene obstacles](/docs/capabilities/manipulation/xarm-grasp.md).
3. **The teleop button instructions disagree.** The [G1 guide](/docs/platforms/humanoid/g1/index.md) and blueprint docstring say X+A. Current arm input implementation (`/dimos/teleop/webxr/extensions.py`) and teleop IK task (`/dimos/control/tasks/teleop_ik_task/teleop_ik_task.py`) use left/right middle-finger grips. Treat the current implementation as the source for engagement behavior. This review records the discrepancy without changing the guide.
4. **Define learning actions before collecting many demonstrations.** Default DataPrep uses the next measured joint state as the action. G1's extra Cartesian streams hold absolute operator controller poses; the relative reference and IK conversion happen internally. Recommendation: record the actual commanded joint targets if command-level imitation is intended, and include measured/commanded hand states. Sources: imitation guide (`/dimos/imitation/README.md`), G1 recorder (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_teleop.py`), IK conversion (`/dimos/control/tasks/teleop_ik_task/teleop_ik_task.py`).
5. **Activation is a control transition.** The low-level backend releases vendor sport mode, and the GR00T arming path emits a pose ramp even while later policy output remains in dry-run. The hardware guide's staged activation is therefore part of the controller design. Sources: DDS startup (`/dimos/robot/unitree/g1/wholebody_connection.py`), arming implementation (`/dimos/control/tasks/g1_groot_wbc_task/g1_groot_wbc_task.py`), hardware CLI (`/dimos/cli/hardware/g1.py`).

**Recommended research and implementation order.**

These are recommendations based on the resources above, not capabilities already validated on your hardware:

1. Confirm the body DoF variant, firmware/SDK access, hand model, cameras, and PC2 environment. Establish subscription-only state reception over the robot-facing NIC before selecting a control stack.
2. Choose vendor locomotion plus upper-body control, or a custom whole-body controller. Establish stationary arm-and-hand operation, calibrated frames, and repeatable teleoperation on that route. Keep a clear distinction between operator input, commanded actions, and measured state.
3. Implement one constrained service task, such as table-to-bin pick-and-place, with observable grasp/place success and a bounded recovery path. Reuse the xArm workflow as a reference and measure completion rate and interventions.
4. Collect task demonstrations, freeze the joint/image/action schema, and compare a learned policy against the scripted baseline. Add preflight and stop behavior through the coordinator.
5. Add navigation between workstations and explicit transitions between walking and manipulation. Carrying objects and moving while reaching require separate evaluation from stationary manipulation.
6. Add operator UI, language-facing skills, task logs, and regression cases. Track task success, cycle time, intervention rate, dropped-object failures, and recovery success.
