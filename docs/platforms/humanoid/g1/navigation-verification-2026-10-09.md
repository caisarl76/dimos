# G1 navigation verification on pc2, 2026-10-09

Target: `pc2`, `unitree@192.168.0.223`, hostname `unitree-g1-nx`.
This session is separate from the concurrent GEAR-SONIC/manipulation integration.
Sensor diagnostic compositions have no locomotion module. After fresh operator
confirmation on Oct 12, supervised finite velocity commands were sent through
the production DimOS `move()` method using a task-initialized Unitree client.
No mode setter, stand-up request, controller release, or G1 locomotion-module
startup has been issued. No planner goal has been submitted.

## Result so far

**Resumed on 2026-10-12.** Initial work was limited to sensors. After another
power cycle, the user confirmed standing Run mode, a clear area, an operator
holding the remote stop, and exclusive control. GET at 07:18 UTC received
8,313 body samples and reported FSM 802 in `ai` mode. The first supervised
0.08 m/s, 0.8-second velocity command returned code 0, but the operator reported
that the robot stayed in place. Walking is not established by that RPC response.

The adapted DDS → DimOS PointLIO → ray map → costmap composition passed its
live sensor-data checks on 2026-10-12: 604 LiDAR, odometry, TF, global-map and
local-map messages, plus 586 costmaps in 60 seconds. The final map has 30,078
points and the costmap is 302 × 256 at 0.05 m resolution. Reported sensor-position
change after warmup was 3.3 mm; this is a stability observation without external
pose ground truth. Robot-body calibration and physical goal navigation remain
unverified.

The original FAST-LIO path remains blocked. On 2026-10-09 it started all five
modules but received zero FAST-LIO data in each of two 45-second captures. No
live localization or map was produced by those earlier captures.

GET-only discovery identifies the actual sensor as device type **35 (MID360S)**.
The pinned Livox SDK 1.2.5 and generated `MID360` configuration expect type 9.
GET-only sensor parameters also show that raw points and IMU are unicast to
Unitree's controller at `192.168.123.161`, rather than pc2. These are two separate
blockers for the current direct FAST-LIO path. No sensor destination was changed.
The Oct 9 operator switch to Run mode yielded GET FSM 802; the first Oct 12
reboot yielded FSM 0, and the later operator-confirmed Run stage yielded FSM
802. Four forward pulses have now returned code 0 without operator-observed
walking, including 0.3 m/s for two seconds. The WebRTC virtual-joystick route
is present in DimOS, but its authentication preflight requires a device key;
no WebRTC joystick command has been sent. All 1,232 recorded host environment fingerprints still matched after
the completed diagnostics.

## What DimOS supplies

| Path | Features and requirements |
|---|---|
| `unitree-g1-nav-simple` | FAST-LIO2 localization from Mid-360 LiDAR/IMU, voxel mapping with ray clearing, height-based costmap, A* path planning, goal arrival/cancellation, obstacle/stuck/deviation replanning, click-to-goal, teleop override, and Unitree high-level velocity execution. |
| `unitree-g1-groot-wbc` | A separate whole-body controller composition. Real hardware uses Point-LIO and the navigation middle; it releases vendor sport control. It has not been launched here. |
| `unitree-g1-agentic` | Adds navigation language tools, tagged-location/semantic-map queries, and exploration-related APIs. Its base composition does not include the self-contained A* planner in `nav-simple`; tool existence alone does not establish a runnable navigation provider. |

The planner and mapper are DimOS components. FAST-LIO is an external localization
algorithm wrapped by DimOS. Unitree supplies the robot sensor/state interfaces
and locomotion SDK. This is not an installation of Unitree's proprietary SLAM or
navigation service, and it does not replace robot firmware.

The simple blueprint configures 0.05 m voxels, 1.4 m overhead clearance, a 0.10 m
terrain threshold, 0.6 m robot width, 0.8 m rotation diameter, and a 1.2 m initial
safe radius. The terrain threshold is a costmap parameter, not a demonstrated
step-climbing capability.

## Fresh evidence

Evidence is under `docs/artifacts/g1_navigation_20261009/` in the original workspace (not imported into this branch).

- Local navigation suite: **30 passed**, including movement-manager override,
  A* and goal validation, stubbed agent tool routing, and navigation/G1 TF checks.
  The agent tests do not demonstrate a physical navigation task.
- Seven navigation implementation files (planner module/core, movement manager,
  cost mapper, ray mapper/FAST-LIO wrappers, DDS execution adapter) have matching
  SHA-256 hashes on pc2 and the local checkout. The Zenoh implementation also
  matches. This relates the selected local checks to pc2's source; it does not
  replace hardware verification.
- The separately attempted mapping test selection ran no checks: two native
  extension skips and four `self_hosted` deselections. A local binding build was
  blocked by the workstation's old Cargo, which cannot read lockfile version 4.
  Those checks are not reported as passing.
- Initial live DDS body capture: **8,314 samples in 8.01 seconds**; second capture
  after operator confirmation: **8,307 samples**. The body message exposes 35
  slots; the probe records the first 29 body positions/velocities. No joint-name
  or calibration claim follows from the slot count.
- GET-only status on both captures: motion mode `ai`, server API `1.0.0.0`,
  FSM ID 0, FSM mode 0, and balance query return code 7301. These responses do not
  establish standing balance or permission to issue locomotion commands.
- A later zero-torque capture received 8,328 body-state samples. The operator
  then explicitly reported switching to Run mode. The next capture, started at
  08:15:01 UTC, received 8,321 samples and GET FSM **802** (code 0), motion mode
  `ai`, and balance query return code 7301. This resolves the zero-torque
  discrepancy; velocity authority and measured walking are still unverified.
  DimOS's `FsmState` enum does not contain 802, so its `get_state()` would label
  this value `UNKNOWN_802`. No stand-up or FSM setter was issued.
- Raw Mid-360 DDS: **99 scans in 10.06 seconds**. The stored last scan has
  **20,064 finite XYZ points**, frame `livox_frame`, and 22-byte point records.
- Vendor `/dog_odom`: **10,064 samples** during that window, frame `odom`, child
  `robot_center`. This stream was not calibrated or compared against FAST-LIO.
- Advertised SLAM mapping/relocation odometry, SLAM points, and global map yielded
  **zero samples**. DDS advertisement does not establish an active localization
  service.
- Recorded real scan: DimOS PointCloud2 LCM round trip and height-cost processing
  passed the diagnostic assertions. Result grid: **252 × 328**. Cold/JIT-inclusive
  height-cost calculation: **0.111 seconds** on the workstation. This is one scan
  in its sensor frame, not a registered map or a real-time performance benchmark.
- Container smoke: Python prefix `/opt/navigation/.venv`, only loopback network
  interface, both native executables present, FAST-LIO linked dependencies
  resolved, and the shipped navigation blueprint imported without starting it.
  The initial check failed because importing visualization creates LCM; setting
  `LCM_DEFAULT_URL=memq://` for this smoke check fixed the import without enabling
  networking or changing host routes.
- The first live startup found a missing Open3D dependency, `libgfortran.so.5`.
  It was added only in a derived task image. The subsequent default and debug
  captures each started FAST-LIO, ray mapping, cost mapping, A*, and the movement
  manager. Each captured for 45 seconds with **zero stream samples**, then
  failed the required-stream assertion and shut down its workers/native
  processes. The exact task container was removed; integrity check exit code 0.
- GET-only UDP discovery returned 11 valid responses from `192.168.123.120:56000`:
  device type 35, command port 56100. Firmware product text reports
  `DevType:Mid-360 FmType:App FmVer:35010108 BuildTime:2025/06/09`.
  Type 35 is named `kLivoxLidarTypeMid360s` in the
  [official Livox SDK 1.3.1 header](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/include/livox_lidar_def.h).
  The older SDK's device-manager lookup ignores device types absent from its
  configured type map; the generated configuration contains only `MID360`.
- GET-only sensor destination queries: status `192.168.123.161:56201`, point
  cloud `192.168.123.161:56301`, and IMU `192.168.123.161:56401`. A newer SDK alone
  does not make those unicast streams available to pc2. Do not redirect them
  without accounting for the existing Unitree controller's sensor feed.

The initial-safe-radius operation changed 1,444 cells on that recorded scan,
including 110 occupied and 1,288 unknown cells. Hardware commissioning must check
the corresponding clear area and coordinate origin. The plot labels this as a
sensor-frame diagnostic; it is not a walkability or collision-avoidance result.

## pc2 prerequisites and environment isolation

pc2 runs Ubuntu 22.04 on aarch64 with Python 3.12.13. Robot-facing interface:
`enP8p1s0`, `192.168.123.164/24`. LiDAR `192.168.123.120` and internal controller
`192.168.123.161` responded to connectivity checks. The DDS module defaults to
`eth0`; this requires a task-specific override before hardware startup.

The clean installed DimOS source is commit
`7dd83be804e0e9dd2dd6210268250bcabf06617c`.
Its configured FAST-LIO executable and repository-root `voxel_ray_tracing`
executable remain missing in the original workspace. Neither Nix nor Cargo was
found in the checked host paths. The newly built binaries and build tools exist
only inside the task image.

The user authorized provisioning while requiring existing workspaces and Python
environments to remain unaffected. Prepared installation scope:

- Task directory: `/home/unitree/dimos-navigation-verify-20261009`.
- Base Docker image: `dimos-g1-navigation-verify:20261009`.
- Runtime image: `dimos-g1-navigation-verify:20261009-runtime`.
- Container Python environment: `/opt/navigation/.venv`.
- Source: a Git archive of pc2's installed commit; untracked `.env`, credentials,
  existing virtual environments, and Git metadata are not build inputs.
- Only the archive, Dockerfile, and smoke script enter the Docker build context.
- Nix, Rust, system dependencies, and Python packages install inside the image.
  No existing checkout or virtual environment is mounted into the container.
- Smoke execution uses `--network none` and asserts that only loopback exists.
- **1,232 file fingerprints** cover discovered existing Python environment
  configuration/distribution metadata, DimOS manifests, shell configuration,
  and the host package database. The provisioning script checks these before
  and after provisioning/runtime. Post-build and both runtime rechecks passed.

The first build failed because Docker bridge setup requires an unavailable
iptables `raw` table on this Jetson kernel. The prepared retry uses Docker build
host networking, without changing daemon, firewall, kernel, group, or host package
configuration. The retry reached Python dependency setup and failed because the
CycloneDDS 0.10.5 Python binding could not locate the Ubuntu multiarch library.
The prepared Dockerfile now builds matching CycloneDDS 0.10.5 under
`/opt/cyclonedds` inside the image and explicitly sets `CYCLONEDDS_HOME`.
The revised retry built CycloneDDS, installed the Python dependencies, built
the voxel mapper in 8 minutes 16 seconds, and built FAST-LIO with its native
transport dependencies. The base image and derived runtime image passed the
network-disabled smoke check. `Dockerfile.runtime` adds only `libgfortran5` to
the container, resolving the Open3D runtime import failure. The user
requested and enabled `unitree` Docker group access; this is an explicit host
account permission change, separate from container dependency installation.
Another fingerprint check after the failed build passed for all 1,232 recorded
files.
Build steps contain no hardware client startup. The subsequent
smoke execution remains network-isolated. Recorded post-install and post-runtime
fingerprint checks passed; this verifies the recorded files, not every host file.

The prepared sensor-only runtime contains FAST-LIO, ray mapping, cost mapping,
A* planning, and the movement manager, with no G1 locomotion endpoint or goals.
It uses a private loopback Zenoh discovery group with gossip enabled, process-only
memory limits, and a mount of only the task output directory. Container cleanup
targets the exact created ID; environment fingerprint checks also run on failure.
Capture requires live LiDAR, odometry, TF, map, and costmap streams. A second
review and script lint/syntax checks passed. Actual runtime started successfully
but both captures failed because all required streams were absent.

Collected remote evidence is saved locally in
`docs/artifacts/g1_navigation_20261009/pc2-runtime/` and in
`pc2-runtime-evidence.tar`. It includes smoke outputs, image metadata, both
sensor capture logs/summaries, discovery and sensor destination responses,
latest GET-only controller telemetry, and the runtime integrity result.

## Remaining hardware verification

1. Calibrate and check the G1 body/sensor frames, actual footprint, initial-safe
   radius, and physical floor/obstacle clearance. The present map's origin is
   the initialized LiDAR pose, not a validated standing G1 base frame.
2. Select a supported deployment input path. The task DDS adapter proves a
   sensor-data route without redirecting Unitree's feed, but is not integrated
   into a shipped blueprint. The original FAST-LIO route still needs MID360S
   support and shared input access. Official SDK 1.3.1 supports type 35 and its
   sample config uses `Mid360s`; neither dependency nor source change was made.
3. Reuse the built task images; do not rerun `provision.sh`, which intentionally
   refuses an existing image tag. Walking conditions were freshly confirmed
   on Oct 12; each pulse checks current GET state and body tilt.
4. Establish velocity authority in the current controller mode and sole
   command ownership before enabling autonomous velocity execution. FSM 802
   and successful velocity RPCs alone do not demonstrate physical authority.
5. With the operator and remote stop present, measure short forward/yaw commands,
   explicit stop and command-timeout behavior.
6. Verify a short clear-area goal, arrival, cancellation, teleop preemption, and
   an obstacle/replanning case while recording pose, costmap, path, commands,
   status, and stop latency. Do not infer these results from local tests.

Full physical navigation is **pending**, not passed.

## Sensor-only resumption, 2026-10-12

Both previous task images survived the reboot and all 1,232 recorded host
fingerprints matched before further work. The fresh readiness probe received
8,319 body-state samples in eight seconds and GET FSM 0. GET-only LiDAR
destination queries still report the original controller at `192.168.123.161`.
No raw-feed destination or controller mode was changed.

A receive-only DDS input capture collected **100 point clouds and 2,000 IMU
messages in 10.003 seconds**. Cloud frame is `livox_frame`; the final cloud has
20,064 points and 22-byte records. The per-point `time` field is float32, with
observed values from 5,000 to 100,320,000. The interpretation as nanosecond
offsets is used by the diagnostic adapter; scan-start header semantics have
not been independently calibrated. IMU acceleration norm ranged from 0.9963
to 1.0097, consistent with g units at rest.

To test the existing DimOS PointLIO path without redirecting the stock raw
feed, a derived task image builds the pinned `pointlio_native` binary:
`dimos-g1-navigation-verify:20261012-pointlio`. The build finished in 1 minute
53 seconds. It changes container layers only. A task-only adapter converts
DDS acceleration from g to m/s² and the float point-time field to UINT32
`offset_time`, retaining XYZ and timestamps. The frame is renamed to
`mid360_link` without a coordinate transform; this is not a calibrated G1
body-frame adapter. Intensity and ring are excluded. A local wire round trip
checked XYZ, per-point time, header timestamps, acceleration conversion, and
unchanged angular velocity.

The diagnostic composition contains PointLIO, ray mapping, cost mapping, A*,
and the movement manager. It contains no G1 locomotion endpoint, creates no
DDS command writer, and submits no navigation goal. This adapted composition
does not establish that the unchanged `unitree-g1-nav-simple` blueprint works.
The first startup stopped before capture because the probe imported message
modules instead of classes. Its cleanup and fingerprint check passed. After
correcting the task-only imports, a peer-mode capture produced 604 odometry/TF
and 603 LiDAR messages, but no maps. Its post-warmup reported position change
was 4.0 mm. Changing only the diagnostic transport topology to a dedicated
ephemeral loopback router with client sessions restored downstream delivery.
This is a task transport adjustment, not a production-source fix or a claim
that the shipped peer topology is generally broken.

The completed 60.013-second router capture exited successfully. It published
12,196 IMU inputs and 608 cloud inputs; counts include a small startup backlog.

| Output | Samples | Observed span rate | Final receipt age |
|---|---:|---:|---:|
| PointLIO cloud | 604 | 10.08 Hz | 0.085 s |
| Odometry | 604 | 10.08 Hz | 0.086 s |
| TF | 604 | 10.08 Hz | 0.086 s |
| Global voxel map | 604 | 10.08 Hz | 0.052 s |
| Local voxel map | 604 | 10.08 Hz | 0.051 s |
| Height costmap | 586 | 9.99 Hz | 0.027 s |

The final PointLIO cloud has 4,478 points, global map 30,078 points, and local
map 24,413 points. The global map spans x −6.175…4.575 m, y −7.725…5.375 m,
and z −1.475…0.575 m in `odom`. Saved poses and cloud coordinates are finite,
maps/costmap are nonempty, and required streams are fresh. The costmap has
62,697 unknown, 9,812 zero-cost, 3,732 occupied-cost, and 1,071 intermediate-cost
cells. These categories do not certify a safe walking surface. The configured
1.2 m initial-safe radius still overrides costs near the origin.

After the first ten seconds, 498 pose samples give per-axis standard deviations
of 1.63/1.64/1.30 mm and first-to-last displacement of 3.27 mm. There is no
external pose reference or moving-robot evaluation. The sensor's body-frame
extrinsics, header timing convention, odometry accuracy during walking, and
real obstacle clearing remain unverified.

The six selected PointLIO, mapper, and Zenoh Python/Rust implementation hashes
match the task image and this checkout. Image ID:
`sha256:01980cefa285106c334c71d9dafb8db6b8ff24ded4d6539f6d91baaab56178b0`.
All 1,232 recorded host fingerprints match after runtime. GET-only queries
confirm all three original LiDAR destinations are unchanged. The exact task
container was removed and the diagnostic router stopped. No host dependency,
Python environment, sensor configuration, controller mode, or other session's
source was changed during this resumption.

New scripts and evidence are under
`docs/artifacts/g1_navigation_20261012/`. The archive retains the successful
capture and earlier diagnostic failures. See `docs/artifacts/g1_navigation_20261012/sensor-mapping.png` (original workspace, not imported)
for the saved map, costmap, and reported sensor position.

## MID360S compatibility and fixes

MID360S is not interchangeable with MID360 at the SDK discovery/configuration
layer. pc2's sensor reports device type 35; the current DimOS Livox dependency
is SDK 1.2.5 and its shared configuration helper writes the `MID360` key, mapped
to type 9. SDK 1.3.1 adds `kLivoxLidarTypeMid360s = 35` and the case-sensitive
configuration key `Mid360s`. Sources: [old enum](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.2.5/include/livox_lidar_def.h),
[new enum](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/include/livox_lidar_def.h),
[configuration parser](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/sdk_core/parse_cfg_file.cpp).

A local compile-only check builds the actual shared config helper against the
1.3.1 headers and asserts the type-9/type-35 constants. It still produces only
`MID360`, demonstrating why a dependency upgrade alone is insufficient. A
separate JSON-key demonstration uses `Mid360s`; neither JSON is an approved
live startup configuration. This check never initializes the SDK or calls the
network. It does not prove that all FAST-LIO sources link or run with 1.3.1.
Evidence: `docs/artifacts/g1_navigation_20261012/mid360s-config-check/`.

| Blocker | Required fix | Current verification |
|---|---|---|
| SDK does not recognize type 35 | Update the pinned SDK and its Nix fetch hash/lock; rebuild the full native targets in an isolated image. | New-header config-helper compile passed; full upgraded native build untested. |
| Generated JSON selects type 9 | Carry an explicit sensor-model option through Python/native config into the shared helper, retaining the old type-9 default. | Current helper still writes `MID360`; no production fix applied. |
| Raw cloud/IMU are unicast to `.161`, not pc2 `.164` | Use Unitree's existing DDS feed, or arrange a vendor-supported shared raw feed with both receivers accounted for. | Task-only DDS → PointLIO → DimOS map/costmap works without redirecting the feed. |
| Startup can alter a shared sensor | Use a genuinely passive input path; guard both SDK autoconfiguration and wrapper-level setters. | Receive-only DDS used for the successful sensor captures. |

The last row matters after adding type-35 support: SDK 1.3.1 defaults
`master_sdk` to true. Its discovery path can configure sensor destinations.
DimOS's FAST-LIO connection callback also explicitly requests normal LiDAR
work mode and enables IMU output. Setting `master_sdk: false` gates SDK
autoconfiguration but does not disable those explicit wrapper commands or
make the existing unicast feed arrive on pc2. The current helper additionally
sets multicast `224.1.1.5` on Linux. Do not start an upgraded direct driver with
that default on the shared sensor without a coordinated data-input design.
See [Livox discovery/configuration handler](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/sdk_core/command_handler/general_command_handler.cpp)
and [MID360S configuration writes](https://github.com/Livox-SDK/Livox-SDK2/blob/v1.3.1/sdk_core/command_handler/mid360s_command_handler.cpp).

The working route separates ownership: Unitree receives the hardware feed and
publishes DDS; the task adapter converts its message layout/units; DimOS runs
PointLIO, ray mapping, height cost mapping and planning modules. The adapter
must become a supported blueprint input before claiming that a shipped G1
navigation blueprint handles this robot unchanged. FAST-LIO specifically
would need its own DDS input support or a shared raw-feed solution.

## Live frame and conservative-costmap check, Oct 12

The receive-only composition was extended with the existing `G1TfPublisher`.
It publishes `mid360_link → torso_link → base_link`, with the waist edge read
from DDS. Independently recorded body state permits comparison against the
publisher; edge presence alone would not establish that its waist input is live.
The costmap's initial safe radius was set to zero for this check.

The first frame diagnostic failed its body-sample assertion and caused 3,158
reported native IMU queue drops. Its main-process reader consumed every
1 kHz body sample. Replacing that diagnostic reader's history with `KeepLast(1)`
removed the queue-drop warnings in the repeat capture; no core module or host
environment was changed. The failed capture is retained as evidence.

The repeat 60.000-second capture exited successfully: 605 clouds, 605 odometry
messages, 605 global maps and 585 costmaps, with 12,353 latest-body samples.
The live waist publisher also logged its first LowState receipt. After warmup,
967 waist-transform comparisons give median/max rotation differences of
0.00093°/0.00334° and maximum translation difference 2.4 µm against the nominal
joint chain. Nearest body receipt differs by at most 17.8 ms in that comparison.
These establish live nominal kinematic consistency, not physical extrinsic
calibration. The assumed body-IMU frame and nominal sensor mount differ from
LiDAR IMU gravity by a median 1.83°; resolve that assumption/calibration before
claiming an accurate robot-base pose.

The final conservative grid marks 278 cells unknown and only 3 zero-cost cells
in the evaluated 0.8 m wide footprint swept over a short forward pulse. No
occupied cell is marked in that particular region, but unknown is not verified
clearance. Separately, applying the blueprint's 1.2 m initial safe radius to the
earlier 30,078-point registered map changes 1,387 cells, including 83 occupied
and 1,291 unknown cells. The operator-confirmed clear area is sufficient for
the supervised velocity diagnostic; it does not validate this blanket map
override or autonomous collision avoidance. Body/ground frame validation,
near-field observation and appropriate treatment of unknown space remain
prerequisites for goal navigation.

Evidence: `docs/artifacts/g1_navigation_20261012/pc2-frames/frames-20261012/`,
`costmap-preflight.json`, and the preserved initial frame archive.

## Supervised DDS walking checks, Oct 12

The operator confirmed a clear area, standing Run mode, a remote stop and
exclusive motion control. Only bounded forward velocity requests were sent;
no mode switch, controller release or joint command was used. Each request
passed through the production DimOS `G1HighLevelDdsSdk.move()` method with a
task-initialized Unitree client, without starting the hardware module lifecycle.

| Forward command | Velocity RPC code | Operator observation | Nominal base displacement |
|---|---|---|---|
| 0.08 m/s for 0.8 s | 0 | Stayed in place | Not retained in the initial failed diagnostic |
| 0.2 m/s for 0.8 s | 0 | Stayed in place | 6.59 mm |
| 0.3 m/s for 1 s | 0 | Stayed in place; physical remote walking works | 6.07 mm |
| 0.3 m/s for 2 s | 0 | Stayed in place | 3.64 mm |

Displacements use PointLIO composed with nominal mount and live waist TF, with
median positions before/after each pulse. They are not external pose ground
truth. Together with the operator's observations, they do not establish SDK
walking. Increasing the duration to two seconds did not resolve the failure.

The first diagnostic incorrectly asserted that `StopMove()` returns a numeric
RPC code. The installed SDK sends a zero-velocity request but returns `None`.
The resulting assertion failed and cleanup ran; a later explicit zero-velocity
request returned code 0. Subsequent diagnostics used `SetVelocity(0, 0, 0)`
to retain the acknowledgment for the same stop payload. Their explicit and
finally stops all returned code 0. The last explicit stop was sent about
2.227 s after the two-second forward request; this is a send-time observation,
not measured physical stop latency.

A receive-only API observer saw only each task's forward request and explicit
zero request in the observed windows for the last two pulses. It did not see
another velocity RPC overriding them. This does not exclude arbitration by
the physical remote or an internal controller. Read-only status reports FSM
802 and motion mode `ai`; supported SDK velocity authority in that firmware
state remains unresolved. No unknown FSM setter was guessed.

All task containers stopped and all 1,232 recorded host hashes matched after
the captures. Goal following, autonomous obstacle avoidance and physical
arrival/cancellation remain unverified.

Evidence: `pc2-walk/`, `pc2-vendor-walk/`, and `pc2-duration-walk/` under
`docs/artifacts/g1_navigation_20261012/`, including the original failed trace,
motion parameters, observed requests, map/pose captures and walking analyses.

## WebRTC virtual-joystick authentication, Oct 12

DimOS also has a different input route: `G1HighLevelWebRtc` delegates to the
shared `UnitreeWebRTCConnection`. With `velocity_api=False`, its serializer
publishes `{"lx": -y, "ly": x, "rx": -yaw, "ry": 0}` on
`rt/wirelesscontroller`. This is virtual joystick input over WebRTC; physical
remote radio packets are not transmitted. The existence of this code does
not establish that this G1 firmware accepts that topic for walking. Values
on the joystick path are stick commands; their physical speed is uncalibrated.

At 09:17 UTC, a task-only preflight on pc2 received HTTP 200 from
`192.168.123.161:9991/con_notify` and decoded `data2=3`. Port 8081 is closed.
The installed WebRTC driver is 2.1.2. This handshake requires a per-device
AES-128 key, and none is configured for the task. No SDP offer, cloud request,
joystick command or motion-mode setter was sent during the preflight.

The shared DimOS connection constructor selects a motion mode and disables
traffic saving on connect. The diagnostic therefore uses the underlying
driver directly for authentication and GET-only connection checks, keeping
those side effects out of this verification stage. Once a correct key is
available, the next stages are a validated data-channel connection, GET-only
controller state, and a supervised bounded joystick test with explicit zero
commands. Authentication success alone will not count as walking success.

The user requested key-retrieval instructions. Task helper
`webrtc-20261012/save_webrtc_key.sh` on pc2 runs in the existing task image,
prompts for the bound account's email/password and region, reads its device
binding list, and asks the user to select pc2's serial number. It writes only
the selected device key with mode 600 into the task's mode-700 private folder
`webrtc-auth-private/`. Passwords, tokens and keys are not printed or included
in reports. No existing host environment or account binding is modified.

Evidence: `docs/artifacts/g1_navigation_20261012/webrtc-auth-preflight.json`.
Driver authentication contract: [upstream LAN signaling source](https://github.com/legion1581/unitree_webrtc_connect/blob/v2.1.2/unitree_webrtc_connect/unitree_auth.py).

The user ran the cloud helper and reported `login/email` code 1000 with no key
saved, then clarified that a teammate owns the Unitree Explore account. This
does not identify the meaning or cause of the login failure. The account owner
must retrieve the key; this session has no account credentials and will not
retry login on their behalf.

An additional compatibility issue was established independently: installed
driver 2.1.2 sends cloud header `AppName: G1`, while [upstream 2.2.0](https://github.com/legion1581/unitree_webrtc_connect/releases/tag/v2.2.0)
corrects the Unitree Explorer account namespace to `B2`. The task key helper
now subclasses the existing cloud client and changes only that header in its
own process. An offline check inside the pc2 image passed for `G1 → B2` and
preserved `Go2 → Go2`. No package was upgraded, no cloud request was made by
this check, and all recorded host hashes still matched. Empty email/password
input is now rejected locally before a cloud request. The helper has not yet
been verified against a successful account login.

A separate serializer check executes the extracted, unchanged DimOS
`_publish_movement` function body with a fake pub/sub endpoint. Forward-stick
and zero-stick payloads passed. Its source hash
`807d7d38dd37acde6be3369840c6ee4e53348d237c821d685e05b90a35928a29`
matches pc2's task image. This checks serialization only, not the live G1
bridge, joystick arbitration, or stopping behavior. Evidence:
`webrtc-joystick-payload-check.json` and `demo_webrtc_joystick_payload.py`.
## SONIC planner follow-up, 2026-10-13

The user selected native SONIC planner mode for direction, speed and independent
facing, with a later pose-mode handoff to VLA manipulation. Read the existing
`/home/jihun/work/GR00T-WholeBodyControl` checkout at
`7c6d710b783a984ede504d24a65bc70e486b3b1d`, preserving its local changes.
The [integration design](/docs/superpowers/specs/2026-10-13-g1-sonic-navigation-design.md)
records the packet contract, frame conversion, sole-publisher ownership,
guarded handoff and software/simulation/hardware acceptance sequence.

The offline [protocol check](/docs/artifacts/g1_navigation_sonic_20261013/protocol-check.json)
passed 11 checks for planner layout, mode-only command parity, existing DimOS
sonic78 pose encoding and reference-frame conversion. This is serialization/math
evidence only; no SONIC motion or mode command was sent to pc2.

Read-only pc2 inspection found an existing SONIC binary, v1.1 assets and V2 planner
ONNX, but no running controller, task container or reference/feedback listener.
The source revision is `a9d20b2ac0949244d94461a1a3263f38c5027c4a`; the binary hash is
`bc0b17c45d06e458b847c217ec62c9e1bbb8d7db66f6688750beceb5c773a3b5`.
Its inspected source lacks the newer reference-frame telemetry and pose receiver
guard. An executable hash does not establish which source changes it contains.

The local guarded manipulation runtime deliberately returns from
`OnPlannerReceived()` whenever its reference guard is enabled. Consequently,
planner walking cannot be added merely by publishing packets to that build.
The guard must be extended in a private native copy and validated for locomotion;
removing it would invalidate the existing manipulation lease/handoff evidence.
The existing DimOS execution module owns the command/pose publisher and should
also arbitrate navigation, preventing simultaneous navigation and VLA publishers.

Navigation stop uses IDLE with zero movement, not just zero speed. Frame conversion
must use fresh measured heading and native reference correction. SONIC's measured
translation visualization is a placeholder, so PointLIO remains the navigation
position source. The earlier near-field costmap/clearance limits remain open.

The design was revised after the user's source review: a dedicated PointLio
SONIC composition excludes the stock SDK, checks command-consumer wiring and
requires command/planner/pose subscriptions. It specifies held/thresholded
direction and facing, IDLE turning and tiny-speed handling, aligned follower
speed, typed frame feedback, explicit navigation rearm, public planner
cancellation, suspended planner leases during pose and off-board heavy processing.
Short source-command pauses use recoverable IDLE; motion deadlines are separate
from the publisher-liveness watchdog. Runtime navigation-input readiness rejects
fresh Twist computed from stale odometry or maps. Network command freshness is
bounded by original source age plus correlated request RTT. An independent
read-only review confirmed correction of its two additional blocking ambiguities.
These remain specified behavior, not completed software or hardware validation.

The final design review amendments are incorporated: the follower's angular
minimum is configurable and set to zero for SONIC navigation; each host runs its
own validated blueprint, joined by the new SonicNavigationLink transport.
Existing advancing sensor/LIO/map/costmap stamps are reused for readiness, and
native planner packets retain sequence/deadline without a request-ID field.
Body-frame commands require accurate sensor-to-base extrinsics rather than a
map-to-IMU heading alignment. The [staged implementation plan](/docs/superpowers/plans/2026-10-13-g1-sonic-navigation.md)
proves the controller path with local Twist/MuJoCo before the off-board link,
then proceeds to isolated pc2 commissioning. This is prepared work awaiting
plan review; no proposed production changes or robot motion occurred in this
design/plan revision.

## Two-host link simulation, 2026-10-14

Software and simulation evidence only. No pc2 process, no robot motion and no
external network traffic. All runs happened inside a private loopback-only
network namespace on the workstation. The setup:

- **The two "hosts"**: both validated blueprints, each with its own zenoh router.
  - The pc2 graph has the read-only sensor sender, the relay and the one execution owner.
  - The peer graph has PointLio, ray map, CostMapper, A*, MovementManager and the peer, and runs in its own process with its monotonic clock offset by 10,000–40,000 s.
- **Controller and physics**: the private native SONIC controller and headless MuJoCo physics.
- **What crosses between hosts**: only the private link, the sensor relay and the native endpoints. The link and both native endpoints run through byte-level delay/hold proxies.
- **Sensor input**: recorded MID360S cloud/IMU payloads reach the production sender through a simulation-only DDS reader stand-in, scoped to that one module. The waist lowstate comes from physics.

The x86 `pointlio_native` and `voxel_ray_tracing` were built from the locked
sources at pinned PointLio core `3c1dcf8b`.
Harness, evidence and per-run hashes are in
[two-host-sim/results.json](/docs/artifacts/g1_navigation_sonic_20261013/two-host-sim/results.json).

**Result.** Three full trials of 79 cases each with the final harness, all analyzed with the Task 5 analyzer:

- Trials 1 and 3 passed every case.
- Trial 2 failed one case, `left_turn_settled`, with the physics settling gate unchanged.
  - After the turn the robot was physically still within about 2 s: speed stayed below 0.013 m/s.
  - The gate's 20 ms yaw-rate estimate kept jittering at about ±0.033 rad/s, just above its 0.03 limit, so the 0.5 s dwell kept restarting; settling took 6.76 s against the 3 s limit.
  - An earlier, now superseded set of three trials, made before a harness proxy leak was fixed, showed the same 2-of-3 pattern (one `forward_settled` miss at 3.15 s).
- The wire checks found valid planner sequences and 0.10–0.20 m/s moving packets (about 900 moving, 1,700–1,900 IDLE and 190 pose packets per trial).
- The Stage A on-host matrix still passes 26/26 after the harness refactor.

| Case group | Measured across the three final trials |
| --- | --- |
| Teleop through the link (MovementManager → peer → link → relay → execution) | forward 0.073–0.180 m, backward −0.133 to −0.260 m, turns +0.073–0.108 / −0.097 to −0.164 rad over 2 s; command source age at pc2 receipt p95 7–52 ms |
| Source pause / outage | IDLE without latch 0.29 s after the last emission; session ended at the 2 s outage |
| Link delay | 20 ms network RTT keeps moving; 150 ms RTT stops motion in 0.13–0.15 s |
| Link hold | IDLE without latch in 0.14–0.20 s; a 2.5 s hold ends the session |
| Frozen stage, while teleop keeps sending fresh Twist | admission closed after raw LiDAR 0.31–0.33 s, IMU 0.22–0.24 s, waist edge 0.49–0.57 s, PointLio 0.31–0.36 s, ray map 0.94–1.03 s, CostMapper worker 0.93–0.98 s; stays closed until an explicit begin |
| Goals on the recorded map | unknown and occupied goals rejected with the goal cell unchanged; the free goal is also rejected because the robot's start cell is unknown. The 1.2 m disk stays 76% unknown (no startup clearing) |
| Goal through the full chain (sim-only synthetic free costmap replacing a frozen CostMapper) | A* plan → 0.2 m/s commands → 0.18–0.21 m of physical walking; cancel reached the public planner RPC and was confirmed IDLE |
| Cancel stall (A* worker stopped) | admission closed at once; pose activation refused while the cancel was unresolved; resolved 0.84–1.79 s after resume |
| Native watchdog / feedback loss / pose expiry | latched in 0.26–0.27 s; `feedback_lost` at 0.35 s; pose expired in 0.20–0.24 s without a planner latch |
| VLA pose phase with the sensor relay active | CPU: physics 58–62%, native 14–15%, peer host 178–181%, pc2 host 29–30%; native LowState/IMU age ≤ 5.9 ms, none over 50 ms. Physics steps: p99 lateness 12–15 ms, max 85–99 ms, no resets |
| Restarts | peer loss: IDLE in 0.09–0.15 s, then the session ends; peer stamp rollback latches until a fresh peer (ready again in 0.64–0.72 s); a restarted peer or publisher never resumes the old session; recovery needs a fresh peer plus an explicit begin |

**Defects found and fixed during Task 9**, each committed separately with its own regression test:

1. CostMapper's base-typed occupancy config made any blueprint pinning `HeightCostConfig` unparseable by `dimos run`. This was a pre-existing upstream bug.
2. The composed `odom→base_link` edge refreshes only at the LIO scan rate, so its 200 ms budget expired between 10 Hz polls. With your approval it now uses the 350 ms LIO budget; the 200 ms waist edge is still enforced before composition.
3. The relay could never recover from a restarted peer. With your approval it now uses per-peer-instance sequence epochs, started after a confirmed cancel or when no session is live.
4. A goal round trip longer than the ~195 ms readiness margin ended the session. With your approval, the relay now refreshes readiness while holding the motion proof during a pending goal.
5. After a pc2 restart, the relay's foreign-request latch never cleared, even for a fresh peer. It now clears only when neither side owns a request.

**Findings, not fixed:**

- The link opens a new connection for every request. Its application round trip is therefore several network RTTs, and network RTT above roughly 30 ms stops motion safely.
- For an occupied goal, A* substituted a "safe goal" 2.3 m from the requested point. No path existed in these runs, but this relocation needs review before physical goals.
- The physics settling gate missed once in each set of three trials. The cause is 20 ms yaw-rate jitter (about ±0.033 rad/s) against its 0.03 limit, not actual motion; any change to how the gate is measured needs approval.
- One earlier attempt (attempt 30) saw the peer still reporting the previous request at a new goal. Its planner received no duplicate goal, and six audited trials did not reproduce it, so it remains unexplained.
- On this shared workstation, host-process startup stalled physics steps; the harness now starts both hosts while the robot is still supported.

**Limits of this evidence:**

- The recorded sensors do not move with the simulated robot, so the peer's odometry, settling and map are static. Physics settling is measured separately.
- No goal was reached on the real recorded map, because the near-field unknowns remain.
- The torso→MID360 mount is the nominal URDF value, not a commissioned extrinsic.
- None of this is pc2 load or hardware evidence. The exact FK fixture and the commissioning gates remain open.

## pc2 sessions and turn tuning, 2026-10-15 to 2026-10-17

The robot was suspended from a ceiling support in every session, so walking was
seen but distance and turn angle are not ground truth.

- Verified on pc2: the full workstation peer, Wi-Fi link, relay, execution owner
  and guarded native path. A walk command entered native walk mode within about
  0.1 s, the legs stepped, and every case returned to IDLE unlatched. `idle`, the
  native operator stop, settling and handback all passed.
- Fixed on the way: a late relay reply over Wi-Fi is now a recoverable IDLE
  (`3f986f271`). Peer settling uses a 0.5 s pose window, because LIO jitter on a
  standing robot read as motion (`b014d85d1`). The launcher gained an operator
  start step (`5a864e4c8`) and refuses to start the graph beside a foreign SONIC
  controller (`ec2fdb3a5`).
- Turning: the native turns in IDLE at a rate that scales with its facing error.
  The original 0.10 rad lead stalled turns near 5 deg. The operator approved a
  0.20 rad lead (`facing_lookahead` 1.00 s, `max_facing_offset` 0.20 rad,
  `808ece4c8`). The workstation peer also ramps the relayed turn rate down over
  0.6 s at a turn stop (`2bdec7165`). In simulation, 4 s turns reach 38-44 deg,
  post-turn settling is 2.2-2.9 s, and all three goal-following cases reach
  their goals (the old settings missed the 60 deg right goal).
- Known native characteristic: walking curves to the right cover 20-40 cm
  against 50-58 cm to the left for the same heading change, in both test orders.
  The tracker's commands are mirror images, so the slowdown comes from the SONIC
  planner or policy, or the simulated G1 model. The follower closes the loop
  and still reaches goals. Pure turns in place show no right bias.
- Not yet installed on pc2: the 0.20 rad lead needs re-provisioning (attempt 9),
  and pc2 has been unreachable since 2026-10-16.

Ground tests still needed, with the robot standing on the floor:

- `forward_020` repeated, with tape measurements against LIO base odometry and
  the logged sensor pose.
- `turn_left_020` and `turn_right_020` with the 0.20 rad lead, measured against a
  floor mark.
- Curves or short goals both ways, to check whether the right-curve slowdown
  seen in simulation also occurs on the robot.
- The sideways drift during forward walking (about -4 cm per run while
  suspended), the mount-yaw check.
- `cancel` and `source_pause`, not yet run on the robot.

### Released-support session, 2026-10-19

With the support released and the ceiling cable slack, `forward_020` covered 18 and
27 cm in a straight line (under 1 cm sideways), and LIO base and sensor poses agreed
within 1-2 cm after each stop. `cancel` and `source_pause` passed again. Repeated turns
in the same direction shrank (right -11.3, -5.5, -4.5 deg; -11.7 again after a forward
walk). Simulation of the same sequence kept repeated turns constant at about -11 deg,
so this is a real-robot effect; the operator judges foot friction to be the main cause.
A turn made right after a walk is larger in both robot and simulation.

### Self-filter and the first physical goal, 2026-10-19

Session 10 refused `goal_1m`: the costmap held 49-50 occupied cells within
0.5 m of the base, a blob centred on the robot with points mostly below 0.55 m.
PointLio's 0.5 m blind sphere sits around the head, so the legs and feet were
mapped along the walked trail. The workstation receiver now drops points within
0.45 m of the base_link z axis and below 0.5 m above the pelvis before PointLio
(`a2ff9a2da`); the two-host simulation matrix passed unchanged.

Session 11, same pc2 install, live costmap watched in the Rerun web viewer
(`demo_peer_rerun_view.py`):

- `costmap_check` showed 0 occupied cells within 0.5 m in all four checks.
- The first `goal_1m`, before any walking, was refused because the robot cell was
  unknown: with self points gone, the floor under the feet is only observed from
  elsewhere. After `forward_020` x2, `curve_left_020`, `turn_right_020` and
  `curve_right_020` the robot cell was free.
- `goal_1m` passed: goal 1.0 m straight ahead, reached within 0.3 m after 17.0 s,
  final error 0.24 m, no latch, no feedback gaps. About 11 s of that was a slow
  in-place turn of 22 deg to the right (about 2 deg/s) before walking. The walked
  track bowed up to 0.35 m right of the straight line, away from occupied cells
  0.63-0.77 m to its left: A* keeping clearance, not drift.
- The final `operator_stop` ended in `feedback_lost` as designed (the native was
  stopped), 21.6 s after Enter.

### Support crane and the second physical goal, 2026-10-20

Session 2026-10-20-4 (commit `6a179cf38`, self-filter and the 45 deg
initial-rotation tolerance) refused `goal_1m` six times on 71 to 3 occupied
cells within 0.5 m: a band 15-45 cm above the floor, about 1 m wide, along and
behind the walked path. The robot's own 0.45 m disk was clear in the last
snapshot, so this was not the self-filter. The support crane the robot hangs
from stands 50-80 cm away. The harness now has `--ignore-rear-occupancy`, off by
default, which leaves the rear half of the 0.5 m disk out of the `goal_1m`
pre-check (`2e672912e`). Replayed on session 4's snapshots it would have admitted
the last three attempts (31, 31, 3 cells, all behind) and still refused the first
three (8-9 cells ahead).

Session 2026-10-20-5, with the option on:

- `goal_1m` was refused twice for an unknown robot cell before walking and twice
  for one occupied cell ahead, then passed after `curve_left_020`: reached within
  0.3 m 8.6 s after the goal was sent, 0.29 m short of the point, cross-track
  0.14 m, no latch or feedback gaps. The planner went straight to path following
  (first heading error about 14 deg, under the old 20 deg threshold too), so the
  45 deg tolerance was not exercised.
- `forward_020` covered 13, 16, 24 and 23 cm; `curve_left_020` 44 cm and 62 deg.
- `source_pause` was refused at begin by the pc2 execution ("fresh navigation
  readiness required") with the peer ready and the link round trip at most 26 ms;
  the next case's begin succeeded.
