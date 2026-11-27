# Live delivery observations on pc2_222: 2026-10-14

The first bounded hardware check of the delivery adapter passed. An eight-second
subscriber-only run produced **219 paired observations** from the existing head
camera and independent body/left-hand/right-hand DDS readers. Acquisition age
and acquisition skew remain unknown.

This exercises the [delivery-freshness prototype](/docs/platforms/humanoid/g1/sonic-delivery-observations.md)
through actual DimOS typed input/output transports. It does not commission the
service, inference or robot motion.

## Current processes

After reboot the initial inspection found no matching camera or SONIC process.
The user chose to start SONIC manually and confirmed supported hardware, a clear
area, an emergency-stop operator and planner mode. A subsequent inspection found:

| Process | Observed configuration |
| --- | --- |
| Camera PID 9269 | `/home/unitree/g1_sonic_eval/.venv/bin/python`, `g1_head_camera_server.py`, `/dev/video4`, `egocentric`, 640×480, publish target 30 Hz, ZMQ port 5555 |
| SONIC PID 9644 | Existing `g1_deploy_onnx_ref`, SONIC v1.1 paths, `enP8p1s0`, **`--input-type keyboard`**, `--output-type all`, `--zmq-host localhost`, telemetry port 5557 |

The native-camera diagnostic refused to open the already-owned camera. The
successful follow-up subscribed to the existing server instead. Its installed
source hash matched the inspected LeRobot source exactly:
`6fc07ad0e1b75725b05462279a6f732fbc7f47278ca1ca5f2fff108b8dbc16e3`.
This server encodes an RGB array directly through OpenCV JPEG; decoding without
another channel swap preserves its intended RGB convention.

The camera source identity included the local boot ID, process ID and process
start tick. The probe checked that process throughout collection and refused a
restart. The wire protocol itself contains no publisher-session identity.
DDS publication timestamps and camera source stamps were opaque advancing
markers; local monotonic receipt times defined freshness.

## Measured results

| Check | Result |
| --- | --- |
| Camera deliveries | 222; approximately 27.6/s |
| Body deliveries | 998 |
| Left / right hand deliveries | 987 / 987 |
| SONIC feedback deliveries | 401; approximately 50.0/s |
| Paired observations | 219 |
| Adapter rejections / faults | 0 / none |
| Age since oldest receipt at adapter output | Median 13.75 ms; p95 14.42 ms; maximum 16.02 ms |
| Span between paired component receipts | Median 6.46 ms; maximum 12.00 ms |
| Final adapter status | `fresh` |
| Acquisition age / skew | `null` / `null` |

DDS counts reflect this probe's latest-sample readers and polling loop, not
the publishers' original sampling rates. Age/span describe delivery only and
exclude earlier source/transport queues. The provisional budgets were 200 ms
delivery age and 100 ms receipt span. A short trace does not establish bounds
for future delays or sustained reliability.

The captured image (`../../../artifacts/g1_delivery_live_20261014/pc2-camera.png`)
shows a small bottle on a black equipment case. A target tray is not visible.
This establishes useful bottle framing; camera extrinsics, reachability and
grasp/placement readiness remain unverified. The software hand ordering was
left thumb/middle/index and right thumb/index/middle; physical finger identity
was not tested.

## Control boundary and next work

The user reported planner mode. The subscribed telemetry contained a 64D token
and measured joints, but no `reference_guard` object, so the probe did not
independently establish guarded planner handback readiness.

The current keyboard input handler cannot receive the DimOS streaming route.
Before a service run, the operator must launch the compatible `zmq_manager`
route, and the installed native runtime must expose and pass the required
reference-guard feedback checks. Changing the input flag alone does not prove
that requirement. No command to arm, switch mode, move, stop or restart SONIC
was sent by this check.

Next, package the existing-camera/DDS receivers into the persistent deployment,
confirm finger mapping, verify the guarded controller route, and measure
dropout/recovery behavior with the chosen policy. A supervised object task
follows those checks, with its outcome measured separately.

## Evidence and cleanup

The probe source (`../../../artifacts/g1_delivery_live_20261014/demo_existing_camera_delivery.py`)
ran through a temporary PC2 import overlay. Its decoder self-check verified the
RGB convention, owned image data, preserved receipt and malformed-input
rejection before hardware subscription. Ruff and formatting checks passed.
Production modules were unchanged; their prior 366-test regression was not
rerun for this diagnostic.

The result (`/docs/artifacts/g1_delivery_live_20261014/pc2-result.json`, not committed),
[metrics](/docs/artifacts/g1_delivery_live_20261014/metrics.json),
[source manifest](/docs/artifacts/g1_delivery_live_20261014/existing-camera-manifest.json)
and [post-check inventory](/docs/artifacts/g1_delivery_live_20261014/pc2-after.json)
are retained. The probe exited 0, removed its temporary overlay and closed its
subscribers/adapter. Camera PID 9269 and SONIC PID 9644 remained running on the
same ports. It made no persistent installation on PC2.

## Subsequent operator restart with `zmq_manager`

The operator relaunched SONIC as PID 15850 with `--input-type zmq_manager`,
`--zmq-host 192.168.0.62` and `--output-type all`, then reported
`Init Done / waiting for a start command`. This supersedes the earlier keyboard
process. The reference destination is workstation port 5556; no listener was
present there during inspection. Camera PID 9269 remained running.

A three-second workstation subscription received no `g1_debug` samples. A
separate local PC2 subscription received `robot_config`, confirming that the
output connection worked. The inspected source publishes configuration while
waiting; measured control telemetry requires the control loop. No start command
was sent merely to obtain telemetry.

The installed checkout is `087f9ac01d46f6d8e4d0b73c01ae64799f292a38` with local
changes, including the native controller, state logger and output handler.
Inspection of 45 native source/header files found no reference-guard/timeout
implementation. A **dry-run only** of the simulation-tested patch did not apply
to `zmq_manager.hpp`, `zmq_output_handler.hpp` or `g1_deploy_onnx_ref.cpp`.
Nothing was patched or rebuilt. Next, port the guard into an isolated copy while
preserving those local changes, then build/validate it before the operator's
next launch. The current input-mode change alone does not satisfy the execution
module's guard prerequisite.

See the [restart readiness record](/docs/artifacts/g1_delivery_live_20261014/zmq-manager-readiness.json)
and patch check (`../../../artifacts/g1_delivery_live_20261014/pc2-guard-apply-check.log`).
