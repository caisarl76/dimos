# Read-only SONIC feedback in DimOS

This adapter receives telemetry from the external C++ `g1_deploy_onnx_ref` runtime. It is the first implementation step in the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md). It creates one ZeroMQ SUB socket and exposes DimOS streams and read RPCs. Starting or stopping it does not launch, stop, or send commands to the robot controller.

The implementation is in this workstation checkout. It has not been copied to PC2. It targets the inspected 29-body-joint, two-Dex3 deployment on `pc2_222`; a controller without both hand arrays will be rejected rather than given invented zero hand states. This is local integration work, not a claim that upstream DimOS main includes SONIC support.

## Run the bounded check

From `/home/jihun/work/dimos_dev`, the current venv already has the required packages. For a fresh environment, add the optional extra while preserving existing installed extras:

```bash skip
uv sync --extra sonic-feedback --inexact
```

Connect explicitly to the existing controller's **feedback** endpoint:

```bash skip
.venv/bin/python scripts/demo_g1_sonic_feedback.py \
  --endpoint tcp://192.168.0.222:5557 --seconds 5
```

The script exits after five seconds and prints a JSON status plus the last sample it observed. Exit `0` means the stream was fresh at the final status check; exit `2` means it was not. An older `last_observed_sample` may be printed for diagnosis when final status is stale; it is not a current measurement. Ctrl-C exits `130` after subscriber cleanup.

This check does not start SONIC. `waiting` with zero accepted packets means no valid telemetry arrived during the check. It does not distinguish an absent publisher, a wrong endpoint, a network problem, or a controller that has not begun publishing. ZeroMQ connecting successfully does not prove the remote application is running.

## Python interface

```python skip
from dimos.robot.unitree.g1.sonic_feedback_module import SonicFeedbackModule

module = SonicFeedbackModule(
    endpoint="tcp://192.168.0.222:5557",
    stale_after=0.2,
)
try:
    module.start()
    print(module.get_status())
    sample = module.get_feedback()  # None until a fresh valid sample arrives
    if sample is not None:
        body, left_hand, right_hand = sample.joint_states()
        print(dict(zip(body.name, body.position, strict=True)))
finally:
    module.stop()
```

The same module can be composed with `SonicFeedbackModule.blueprint(endpoint=...)` in a service blueprint. Its generated module registry key is `sonic-feedback-module`; this change adds no controller blueprint. Keep the feedback module separate from a hardware connection that owns motor commands.

| Interface | Result |
| --- | --- |
| `feedback: Out[SonicFeedback]` | Immutable body/hand position and velocity tuples, controller index, source timestamp, local receipt timestamps, optional `SonicReferenceGuard` |
| `body_joint_state: Out[JointState]` | All 29 joints, using the existing `g1/...` motor-order names |
| `left_hand_joint_state`, `right_hand_joint_state` | Seven slots per hand named `g1/left_hand_motor_0` through `6`, and likewise for the right hand |
| `get_feedback()` | Latest sample only while fresh and the receiver is running without a fault; otherwise `None` |
| `get_status()` | `waiting`, `fresh`, `stale`, `fault`, or `stopped`, plus local age, packet counters, and a `reference_guard` dictionary when fresh and present |

Joint positions are radians and velocities radians/second. Body values are already converted to motor order with offsets restored by the inspected C++ publisher; the adapter applies no second conversion. Hand values retain the publisher's exact slot order, without anatomical labels or right-hand permutation. Mapping those slots to a Dex3 planning model remains a separate validation step. The adapter does not expose measured torque as calibrated effort or infer a floating-base pose from visualization fields.

### Receiver guard status

The [local C++ expiry prototype](/docs/platforms/humanoid/g1/sonic-reference-expiry.md) publishes an additional `reference_guard` object. DimOS now validates it and exposes it as an immutable `SonicReferenceGuard` on the same feedback sample. This extension has been exercised with the guarded controller in isolated MuJoCo; it has not been installed on PC2.

For an already-started module, a read-only inspection is:

```python skip
sample = module.get_feedback()
if sample is None:
    print("No current feedback")
elif sample.reference_guard is None:
    print("Controller did not report receiver guard status")
else:
    guard = sample.reference_guard
    print("Streaming armed:", guard.armed)
    print("Reference expired:", guard.expired)
    print("Planner handover confirmed:", guard.planner_confirmed)
    print("Accepted / installed indices:", guard.accepted_index, guard.applied_index)
```

| Field | Meaning and limitation |
|---|---|
| `armed`, `expired` | Receiver stream lease state. Expiry can coexist with confirmed planner handover. |
| `planner_selected`, `planner_confirmed` | Selected input path versus confirmation of a successful local planner/encoder policy tick. Neither proves physical settling or task completion. |
| `rearm_requires_planner` | Receiver requires a consumed planner acknowledgment before another streaming request. |
| `accepted_index`, `applied_index` | Validated reference high-water and endpoint-installed index. Installation does not prove consumption by a particular policy tick. `-1` means none. |
| `rejected`, `epoch` | Receiver reference rejection count and arm generation. These differ from the subscriber's telemetry packet counters. |
| `age_ms` | Receiver's monotonic age since accepted receipt, or since arm during first-frame grace; `-1` means unavailable. Previous-epoch indices can remain during that grace period. |

A legacy packet without this object still decodes, with `reference_guard=None`. DimOS does not infer planner confirmation from legacy visualization fields. A present but incomplete, mistyped, nonfinite or contradictory guard object rejects the entire packet and does not refresh the cache. `get_status()` hides old guard state when feedback becomes stale, faulted or stopped. Stream consumers must likewise expire their cached sample.

Validation added 32 feedback cases, bringing the feedback suite to **68 passing tests**. Together with the reference encoder's 55 tests and 14 existing G1 TF/registry checks, **137 checks passed**. The real module also passed the 20-assertion isolated fault trial; 1,042 previously recorded frames decoded with matching guard fields. See the [typed feedback and moving-reference report](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) for runtime evidence and limits. No new motion RPC or motor writer was added.

## Freshness and timestamps

- A sample becomes stale when local monotonic time since receipt reaches `stale_after` (default 0.2 seconds). This threshold is a configurable observation policy, not a validated control deadline.
- `JointState.ts` is local wall time at receipt. `SonicFeedback.source_timestamp` preserves nonzero `ros_timestamp`; zero becomes `None`. No clock correction or synchronization is inferred.
- `received_monotonic` belongs to the receiving process's host. Use `get_status()`/`get_feedback()` for age; do not subtract it from another computer's monotonic clock.
- Only increasing controller indices are accepted. Duplicate or older indices do not refresh the cache or emit streams. Start a new subscriber session after a controller index reset.
- Streams emit new samples only; they do not replay a cached pose during a disconnect. Consumers must enforce freshness instead of treating their last stream value as indefinitely current.
- Stream freshness does not prove that the C++ controller's underlying DDS samples are fresh. The inspected wire schema does not supply per-sensor capture-age guarantees. This module is for observation and integration work; it is not sufficient feedback validation for autonomous motion.

Malformed MessagePack, wrong vector lengths, missing body/hand arrays, nonnumeric/nonfinite values and invalid metadata are rejected. A startup or receive/publish failure sets `fault`, making `get_feedback()` return `None`; startup errors also propagate to the caller. Explicit `stop()` cleans up and clears the fault. The transport uses the inspected single-part `g1_debug` prefix plus MessagePack contract; it does not parse the command/reference protocol.

## Validation record

On 2026-10-06, **50 focused tests passed**: 36 feedback/parser/lifecycle tests, 9 existing G1 TF tests and 5 registry-generation checks. The socket tests exercise the real module against a local XPUB fixture, including subscription readiness, typed streams, invalid-frame recovery, stale expiry, startup/output faults and shutdown. These are synthetic protocol fixtures, not recorded hardware packets. Ruff lint/format checks and scoped mypy checks passed. The full repository suite was not run; pytest emitted a subprocess `ResourceWarning` during startup.

```bash skip
CI=1 .venv/bin/python -m pytest \
  dimos/robot/unitree/g1/test_sonic_feedback.py \
  dimos/robot/unitree/g1/test_g1_tf_publisher.py \
  dimos/robot/test_all_blueprints_generation.py -q --no-cov
```

The registry was regenerated using the repository's generator test. `CI=1` above verifies generated content without requiring the working-tree changes to have been committed. No commit or push was made.

The initial five-second subscriber check against `tcp://192.168.0.222:5557` returned `waiting`, zero accepted/invalid packets and no sample (exit 2). It did not establish live compatibility. After the user started SONIC, the [active-session check below](#active-planner-session-2026-10-06) established live decoding and observed position agreement. No controller or streamer was launched by these checks.

Sources for the wire contract, read on PC2:

- `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/output_interface/zmq_output_handler.hpp`
- `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp`
- `/home/unitree/g1_sonic_eval/code/sonic_policy_streamer.py`

The [deployment trace](/docs/platforms/humanoid/g1/sonic-upstream-status.md#installed-external-controller-follow-up-trace) records the repository revision and dirty source state. The current session's executable was subsequently identified through `/proc`. A separate [offline reference encoder and cancellation source audit](/docs/platforms/humanoid/g1/sonic-reference-contract.md) now covers the command format. Reference publishing, cancellation and motion execution are not implemented here.

## Compare SONIC with independent DDS feedback

The capture script (`/scripts/demo_g1_sonic_dds_capture.py`) runs on PC2 using its existing Python packages. It constructs DDS readers for `rt/lowstate` and `rt/dex3/{left,right}/state`, plus a ZeroMQ subscriber to the SONIC feedback endpoint. It constructs no command writers or motion clients. Both streams are observed in one process using PC2's monotonic clock, avoiding the workstation-to-PC2 clock offset for this diagnostic.

Run these commands from the workstation checkout. SSH sends the script through stdin; it does not install code on PC2. The capture and report stay on the workstation:

```bash skip
ssh -o BatchMode=yes -o ConnectTimeout=8 pc2_222 \
  /home/unitree/dimos/.venv/bin/python -B - \
  --interface enP8p1s0 --endpoint tcp://127.0.0.1:5557 --seconds 8 \
  < scripts/demo_g1_sonic_dds_capture.py > /tmp/g1-sonic-dds-capture.json

.venv/bin/python scripts/demo_g1_sonic_dds_compare.py \
  /tmp/g1-sonic-dds-capture.json --max-gap 0.05
```

The offline analyzer (`/scripts/demo_g1_sonic_dds_compare.py`) reuses the actual DimOS decoder on the saved SONIC wire bytes. It reports maximum absolute position/velocity differences for each joint, comparing the unchanged motor slots. It rejects repeated SONIC indices and declines pairs whose local read times differ by more than 50 ms, whose DDS values are invalid, or whose topic had multiple observed DDS publication handles. The 50 ms setting is a diagnostic pairing limit, not a sensor-age guarantee or validated control deadline. The capture records at most 20 SONIC rows per second, while counting all packets it receives.

`comparison_available` (exit 0) means each group has at least one usable pair; it does **not** mean the errors are acceptable. Inspect the per-joint errors and read-time gaps. `insufficient_data` (exit 2) means one or more groups lack a usable pair. In particular, no SONIC publisher produces no comparison even when DDS is healthy. Agreement in stationary or symmetric poses cannot prove every joint permutation or anatomical finger label. These are local read times, not simultaneous sensor capture times, and a new SONIC index does not prove fresh underlying DDS measurements.

### Follow-up result, 2026-10-06

PC2 was reachable, with no matching SONIC controller/streamer process and no TCP listener on 5557 during the process check. A subsequent eight-second localhost capture started at `2026-10-06T09:34:55.599297+00:00` according to PC2's clock:

| Stream | Samples received/taken | Position/velocity slots | Observed publication handles |
| --- | ---: | --- | ---: |
| SONIC `g1_debug` | 0 | No sample | Not applicable |
| DDS body | 1,973 | 29 / 29 | 1 |
| DDS left hand | 1,972 | 7 / 7 | 1 |
| DDS right hand | 1,970 | 7 / 7 | 1 |

DDS counts are observations of the polling reader with keep-last history, not publisher-frequency or packet-loss measurements. Publication handles identify the sources seen in this capture, not authenticated robot identity. This initial capture returned `insufficient_data` and supplied no SONIC comparison; the subsequent active-session result follows.

Evidence: [raw capture](/docs/artifacts/g1_pc2_222_20261006/sonic-dds-followup-capture.json), [comparison report](/docs/artifacts/g1_pc2_222_20261006/sonic-dds-followup-comparison.json). Six controlled offline cases checked known joint differences, stale pairs, empty captures, multiple DDS sources, repeated SONIC indices and malformed packets. Both scripts passed Ruff and scoped mypy. No robot process, mode, clock setting or installed file was changed.

### Active planner session, 2026-10-06

The user reported SONIC active in planner mode. A five-second run of the actual DimOS feedback module against `tcp://192.168.0.222:5557` accepted **249 packets**, with zero invalid packets, zero rejected indices and no receiver error. Final status was `fresh`, with local arrival age **18.49 ms**. Its last source timestamp was `None` because the wire `ros_timestamp` was zero; this does not resolve camera/robot time synchronization.

A separate eight-second PC2 capture, starting at `2026-10-06T09:40:50.607839+00:00` on PC2's clock, received **400 SONIC packets**, approximately 50 Hz, and retained 134 rows. All 134 decoded successfully with increasing indices. One body row preceded the first DDS body sample and was left unpaired. Each DDS topic had one observed publication handle.

| Group | Paired rows | Largest position difference | Largest velocity difference | Largest local read-time gap |
| --- | ---: | ---: | ---: | ---: |
| Body, 29 joints | 133 | 0.00006694 rad / 0.00384° | 0.07542 rad/s | 2.457 ms |
| Left hand, 7 slots | 134 | 0.00002944 rad / 0.00169° | 0.22901 rad/s | 2.282 ms |
| Right hand, 7 slots | 134 | 0.00006437 rad / 0.00369° | 0.57252 rad/s | 2.119 ms |

This verifies wire compatibility and close **position agreement in the observed poses** through two independently subscribed feedback paths. The samples are not simultaneous sensor captures. Velocity differences are recorded separately; no velocity-agreement tolerance has been validated. This does not establish every anatomical finger mapping, end-effector accuracy, underlying sensor age or motion readiness.

The live process was PID `25395`. `/proc/25395/exe` resolved to `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref`; hashing that executable returned `5526b184875f6f5e392cf6a5c95a8f55ab2d0a698177078a214db2231a0ec425`. Its arguments selected the SONIC v1.1 decoder, encoder and observation YAML, `planner/target_vel/V2/planner_sonic.onnx`, **`--input-type keyboard --output-type all`**, and interface `enP8p1s0`. The same PID owned TCP listener 5557 and remained running after the probes. This identifies the current session, not the earlier exited PID `39396`.

The `keyboard` input selection differs from the previously traced `zmq_manager` VLA launcher. Reading this session's telemetry works, but command integration must account for its input mode. Planner mode was user-reported; the telemetry adapter does not infer it. No mode switch, reference publisher, motion command, process restart, robot file change or clock adjustment was performed.

Evidence: [validation summary](/docs/artifacts/g1_pc2_222_20261006/sonic-planner-validation-summary.json), live module output (`/docs/artifacts/g1_pc2_222_20261006/sonic-planner-module.log`), raw paired capture (`/docs/artifacts/g1_pc2_222_20261006/sonic-dds-planner-capture.json`), [per-joint comparison](/docs/artifacts/g1_pc2_222_20261006/sonic-dds-planner-comparison.json). The subsequent [offline reference-contract step](/docs/platforms/humanoid/g1/sonic-reference-contract.md) validates encoding and audits lifecycle source. An [isolated stationary SONIC simulation trial](/docs/platforms/humanoid/g1/sonic-simulation-validation.md) records runtime handover, disconnect and shutdown behavior. The follow-up [typed feedback and moving-reference report](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) covers the guarded receiver. Loaded handovers remain unvalidated before hardware command integration.
