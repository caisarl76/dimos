# PC2 body and Dex3 timing audit

The 2026-10-13 read-only audit confirmed that body and both Dex3 state streams
are available on pc2_222. It did **not** establish sensor acquisition times.
The current `SonicMeasuredState` contract requires timing information that the
installed Unitree messages do not explicitly expose for the hands. Adding a
timestamp to SONIC's outgoing telemetry alone cannot satisfy that contract.

This advances the [observation adapter](/docs/platforms/humanoid/g1/sonic-observation-adapter.md) work by
identifying the actual source boundary and measuring reader behavior. The
adapter's service-output gate remains unchanged. No controller commands,
mode changes, camera starts, policy inference or robot software installations
were performed during this audit.

## Latest-sample results

The eight-second probe used DDS domain 0 and interface `enP8p1s0`, with
best-effort, volatile, `KeepLast(1)` readers. It took one sample per topic per
iteration and paused 10 ms between iterations.

| Stream | Valid samples | Observed take rate | Median DDS source → PC2 take-end difference |
| --- | ---: | ---: | ---: |
| Body | 681 | 85.01 Hz | −5.62 ms |
| Left Dex3 | 678 | 84.63 Hz | −6.05 ms |
| Right Dex3 | 678 | 84.63 Hz | −6.09 ms |

These are the probe's sampling rates, not the publisher frequencies. Its
latest-only history deliberately replaces intermediate samples. Negative
source-to-take differences show that the publisher and PC2 wall-clock values
cannot be used interchangeably. They are **not negative network latency**.
The difference combines clock offset, publication semantics, buffering and
reader execution; it does not measure motor acquisition age.

All retained positions were finite. The body message contained 35 slots;
the probe inspected the first 29 for this robot. Each hand contained seven
motors. Each stream had one active publication handle during the trace, with
no DDS source-timestamp repeats or regressions. Body ticks advanced in the
latest-sample capture. The hand and body reserve arrays were zero throughout.

The probe reported no invalid samples, record-limit discards or DDS
loss/rejection callbacks. Zero DDS counters do not establish lossless capture:
best-effort transport and history replacement can discard data without those
callbacks. Publication handles are local to a capture; do not compare their
numeric values across processes.

Evidence: [summary](/docs/artifacts/g1_sonic_timing_20261013/pc2-summary.json),
compressed raw samples (`../../../artifacts/g1_sonic_timing_20261013/pc2-capture.json.gz`).

## Why the first capture was repeated

The first trace used `KeepLast(256)` and took batches of up to 64 samples.
It retained 6,736 body, 6,074 left-hand and 6,164 right-hand samples. Body
deserialization/take calls had a 34.23 ms median duration, and the median
DDS-source-to-take-end difference reached 243.12 ms. The corresponding hand
differences were 64.81 and 68.42 ms.

This exposed reader backlog. It cannot be reported as robot sensor latency.
The latest-only repeat reduced median take-call durations to 0.65 ms for
body, 0.20 ms for left hand and 0.18 ms for right hand. Both traces are retained
with their exact source versions and QoS. Raw data was losslessly compressed
from approximately 15.5 MB to 1.07 MB total.

The history trace also contained 270 repeated body ticks despite advancing
DDS timestamps. Without confirmed tick semantics, that does not establish
frozen hardware. It does show why a new DDS publication must not automatically
be interpreted as a new motor measurement.

Evidence: [history summary](/docs/artifacts/g1_sonic_timing_20261013/history-pc2-summary.json),
history raw samples (`../../../artifacts/g1_sonic_timing_20261013/history-pc2-capture.json.gz`).

## Timestamp provenance

| Source | What is established | Remaining gap |
| --- | --- | --- |
| `LowState_.tick` | Installed body message has a uint32 counter | Acquisition relationship, units, reset/wrap semantics and clock conversion |
| `HandState_`, `MotorState_` | No named timestamp/counter field in the inspected installed definitions | Individual motor/hand acquisition freshness |
| DDS `sample_info.source_timestamp` | Publisher metadata; present and advancing in this capture | Publisher timestamp implementation and sensor-to-publication delay |
| Native `DataBuffer::GetDataWithTime()` | Timestamp assigned by `SetData()` using local `steady_clock::now()` | This is buffer update timing, not established acquisition timing |
| Native `g1_debug.ros_timestamp` | ROS output clock sample, or zero without that output | Does not identify individual measurements |
| Local take-call bracket | Before/after times around the reader operation | Does not bound packet arrival or prior queue residence |

The official [Unitree HandState definition](https://raw.githubusercontent.com/unitreerobotics/unitree_ros2/master/cyclonedds_ws/src/unitree/unitree_hg/msg/HandState.msg)
and [LowState definition](https://raw.githubusercontent.com/unitreerobotics/unitree_ros2/master/cyclonedds_ws/src/unitree/unitree_hg/msg/LowState.msg)
match the relevant installed field layout. The live installed field inventory
is included in the capture rather than inferred from upstream alone.

CycloneDDS 0.10.5's [writer implementation](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/0.10.5/src/core/ddsc/src/dds_write.c)
sets ordinary writes to the writer's current DDS clock; `dds_write_ts` instead
accepts an application-supplied timestamp. Which path a vendor publisher uses,
and what it timestamps, must be established at that publisher. The generic DDS
field name does not prove sensor acquisition semantics.

Native source inspected in the local runtime checkout:

- `include/utils.hpp`: `DataBuffer::SetData()` and `GetDataWithTime()`.
- `include/dex3_hands.hpp`: hand callback buffers and `getStateWithTime()`.
- `src/g1_deploy_onnx_ref.cpp`: `LowStateHandler()`, `GatherRobotStateToLogger()`
  and `GetRosTimestamp()`.

These live under
`build/sonic-grasp-runtime/source/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`.
No native runtime source or binary was changed by this audit.

## Current host and process observations

State-source discovery matched `basic_service` PID 2569 for the body and
`g1_dex_protocol_v2.0` PIDs 2763/2772 for the hands, all advertised on the
internal computer at `192.168.123.161`. These are advertised identities,
not authenticated ownership or firmware versions.

PC2's time-sync status selected that internal computer, with a reported
offset of +18.565 ms and jitter of 11.481 ms. This status is from a separate
read and may reflect an earlier NTP exchange; it is not a correction to
subtract from each recorded DDS sample. No clock configuration was changed.

Controller inventory changed during the checks. The first trace discovered
SONIC PID 42699. That PID was absent after the latest-only trace. A subsequent
process read showed a new deployment, PID 48536, using `--input-type zmq_manager`
and `--output-type all`. This session did not start, stop or signal either
deployment. Process arguments alone do not establish planner readiness or
streaming ownership.

An existing `d435_publisher_zmqrgb` process was also running. Its arguments
included 640×480 at 30 FPS, `use_hw_timestamp:=false`, a Livox time-sync topic,
and a final `zmq_port:=5559` argument. Those options require source inspection
before reusing its timestamps. Do not launch another camera owner while this
publisher is active. The previous native-camera validation used a different
camera path and timestamp contract.

Evidence: clock/process read (`../../../artifacts/g1_sonic_timing_20261013/pc2-clock-and-processes.log`)
and [temporary-overlay cleanup](/docs/artifacts/g1_sonic_timing_20261013/pc2-cleanup.json).

## Reusable diagnostic

Source: capture script (`/scripts/demo_g1_sonic_dds_timing.py`) and
offline analyzer (`/dimos/robot/unitree/g1/sonic_dds_timing.py`).
The script requires both files to be available in PC2's Python environment;
this audit used a temporary import overlay instead of installing them.

```bash skip
.venv/bin/python scripts/demo_g1_sonic_dds_timing.py \
  --interface enP8p1s0 --seconds 8 --read-mode latest \
  > /tmp/g1-dds-timing.json
```

The script creates state and discovery readers only, supports 2–15 seconds,
and caps records at 20,000 by default. `--read-mode history` reproduces the
batching diagnostic. Exit zero means all three state topics produced valid
data, not that observation synchronization or control readiness passed.

**43 focused tests passed**, including 12 new timing-analysis tests and all
31 existing observation-adapter/pairing tests. Ruff, format and focused mypy
checks passed. Source review found no remaining material issue. The complete
repository suite was not run; the existing pytest startup subprocess warning
remains. See the [verification record](/docs/artifacts/g1_sonic_timing_20261013/verification.json).

## Remaining implementation decision

The immediate hardware task is to establish a defensible measurement-time
contract at the Unitree state source. Two routes can support the observation
adapter:

1. Instrument the actual sensor/state producer to preserve per-component
   acquisition times, source identity and restart identity through DDS and
   SONIC. The available hand publisher may require vendor support or access
   to its implementation. Timestamping SONIC callbacks provides receipt
   freshness only.
2. Establish bounded acquisition intervals from documented sampling/buffering
   behavior and measured clock/transport uncertainty. The
   [observation adapter](/docs/platforms/humanoid/g1/sonic-observation-adapter.md#input-and-output-contract)
   now retains and checks the full intervals for each body/hand component.
   Hardware estimates and bounds remain to be established. Observed means,
   maxima from this short trace, or an arbitrary configured margin are
   insufficient bounds.

Neither exact acquisition times nor bounded acquisition estimates have been
established from the stock messages inspected here. A receipt-based service
experiment would be a different timing contract and must be described and
evaluated as such; this audit does not silently enable one.

After selecting and validating the timing contract, connect the measured-state
producer, exercise camera/state dropout handling, confirm physical finger
mapping and bottle/tray camera framing, and commission a supervised service
run with a matching policy. DimOS continues to own task orchestration and
observation handling; LeRobot supplies the policy and SONIC owns motor control.

## Follow-up: available publisher sources and external camera

A further read-only PC2 inspection on 2026-10-13 checked the available source
packages. The user reported that vendor timing documentation/source access is
not available or not known. The bounded inventory found 179 files in
`/home/unitree/xr-dex3-packages` (excluding build/install/log directories) and
793 files in `/home/unitree/unitree_sdk2-main`. The relevant matches were SONIC
hand integration code, SDK examples and message headers. The actual
`basic_service` and `g1_dex_protocol_v2.0` publisher implementations were not
found in those checked packages. This is not an exhaustive search of PC2 or
the internal computer, and it does not establish that Unitree cannot provide
the required information.

The current official [HandState message](https://raw.githubusercontent.com/unitreerobotics/unitree_ros2/master/cyclonedds_ws/src/unitree/unitree_hg/msg/HandState.msg)
still exposes no named acquisition timestamp/counter. The official
[LowState message](https://raw.githubusercontent.com/unitreerobotics/unitree_ros2/master/cyclonedds_ws/src/unitree/unitree_hg/msg/LowState.msg)
has `tick`, but the definition does not explain its timing semantics. The
Unitree documentation portal did not expose enough article content through
this browser to establish an acquisition-age bound. No bound was inferred
from that limitation.

The installed camera source was readable at
`/home/unitree/minji/physical_ai_agent/camera/src/d435_cpp/src/d435_publisher_zmqrgb.cpp`.
Its contents and SHA-256 are retained in the
[source capture](/docs/artifacts/g1_source_timing_20261013/pc2-source-inventory.json).
These are source findings, not proof of the exact binary used in a prior run:

| Source location | Behavior | Integration consequence |
| --- | --- | --- |
| `on_time_sync_`, lines 291–304 | Computes incoming LiDAR/IMU stamp minus local ROS clock at callback, then filters that offset | Includes upstream timestamp and delivery behavior; not a measured acquisition-time conversion |
| `make_stamp_`, lines 308–320 | With `use_hw_timestamp=false`, uses current local ROS time; adds the filtered offset when initialized | Result is not the original camera acquisition time or necessarily PC2 UNIX time |
| Capture loop, lines 353–377 | Waits for frames, aligns depth/color, then creates the stamp | Delivery/processing precedes the fallback stamp |
| Hardware-time option, lines 249–260 | Attempts global-time enable; a failure is logged and execution continues | Enabling the option alone does not verify the frame's clock domain or bound its error |
| Capture diagnostics, lines 422–428 | Warns if synchronization is missing/stale; retained offset remains in use | A timestamp can continue advancing while the mapping is stale |
| `publish_zmq_`, lines 526–550 | Sends MessagePack with `timestamps` and binary JPEG `images`; converts RGB to BGR before OpenCV encoding | This is a different wire/color contract from LeRobot's JSON/base64 camera path; decoding and clock semantics both need explicit adaptation |

Do not feed this external camera's stamp directly into
`SonicObservationModule.handle_color_image()`, whose input contract is an
original same-host UNIX camera timestamp. A source adapter must preserve
provenance instead of relabeling the filtered stamp. The existing native
DimOS RealSense route remains a separate option when camera ownership is
available; its timestamp-error bound still needs commissioning.

The first process check saw SONIC PID 7227 using `--input-type zmq_manager`
and `--zmq-host 192.168.0.62`, with no matching camera publisher. The later
saved source inventory contained no matching SONIC, camera publisher or
policy streamer process. These are time-separated snapshots from an
unprivileged account. This session did not start, stop, signal or modify any
of them, and the initial command line is not a current readiness assertion.

Evidence: [package inventory](/docs/artifacts/g1_source_timing_20261013/pc2-publisher-files.json)
and [verification record](/docs/artifacts/g1_source_timing_20261013/verification.json).
No product code changed and no hardware acquisition, inference or motion ran
in this follow-up.

### Practical prototype option

The remaining choice is a timing requirement for the first supervised
prototype. The strict adapter already supports exact or bounded acquisition
times; that requirement remains unmet on hardware. A delivery-freshness
prototype would instead make this limitation explicit:

- Observe each body/hand stream independently and retain local monotonic
  receipt times, source identity and sequence evidence where exposed.
- Require new camera frames and body/hand deliveries within configured
  deadlines, across startup, inference, streaming and recovery. Reusing a
  cached value must not refresh its recorded receipt time.
- Preserve camera source stamp/domain separately; report acquisition age
  and acquisition skew as **unknown**, not zero or "verified".
- Test frozen camera, individual-hand dropout, publisher restart and stopped
  telemetry before admitting a supervised hardware run.
- Keep a distinct observation contract/profile so it cannot accidentally be
  passed off as the strict acquisition-time path.

This would detect missing delivery; it would not detect a publisher repeatedly
sending old sensor values with fresh metadata. It is closer to the existing
LeRobot workflow, while making each input's delivery freshness visible. It
still needs implementation, budget selection and hardware validation. The
user was asked which timing requirement to use; no delivery-based control
path was enabled by this audit.

If verified acquisition bounds are required, request from Unitree the exact
firmware-specific sampling and buffering behavior, the body's `tick` units
and reset/wrap semantics, each hand's sample identity/freshness semantics,
and the relationship between those samples and DDS publication timestamps.
The answer must distinguish new publication from new sensor measurement.

### Selected follow-up

The user selected the delivery-freshness prototype. Its
[implementation and operating guide](/docs/platforms/humanoid/g1/sonic-delivery-observations.md) now covers
independent component receipts, explicit service/execution profile agreement,
dropout recovery and a bounded native-camera/DDS diagnostic. Acquisition age
and skew remain unknown; no bounds were inferred from the earlier traces.

Local validation passed 366 SONIC tests. The latest PC2 preflight found an
active `g1_head_camera_server.py` owning `/dev/video4`, so the new native-camera
probe was deferred without altering that process. The delivery path has not
yet been validated with all four live hardware inputs or commissioned for motion.

On 2026-10-14, the [first live delivery pairing check](/docs/platforms/humanoid/g1/sonic-delivery-live-validation.md)
passed using the already-running LeRobot camera server and independent DDS
readers: 219 paired observations, no rejected deliveries, unknown acquisition
age/skew. Guarded service commissioning remains pending.
