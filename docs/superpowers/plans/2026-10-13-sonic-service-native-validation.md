# Native validation of the SONIC service coordinator

The user authorized the next MuJoCo/Pi0.5 validation after the coordinator's
262-test software milestone. Reuse the established private user/network
namespace and local assets. All controller and DDS traffic stays on its
loopback interface. Leave the existing GPU 0 policy server running; launch one
owned offline Pi0.5 server on GPU 1 and reap it afterward.

Compose the real `SonicServiceModule` and `SonicExecutionModule` in two DimOS
workers. Supply the existing paired MuJoCo RGB/joint snapshots through the
observation stream. The harness uses service request/status/cancel RPCs; only
the service module performs task inference and execution orchestration.

Required cases:

1. Six seconds of accepted live policy execution, at least 15 predictions,
   automatic finish and confirmed planner handback.
2. Cancel after at least 1.5 seconds while a policy request is outstanding;
   observe handback and discard its late result without new references.
3. Freeze the camera after 1.5 seconds; preserve its source timestamp, expire
   the observation at the unchanged 850 ms execution budget and confirm handback.

Record original source snapshots, service/native journals, service RPC status,
unchanged policy requests/replies, reference packets, native feedback and
physics. A recording proxy forwards the existing policy protocol byte-for-byte;
it supplies evidence only. Audit accepted predictions against original RGB,
joint state, source time, policy output hashes and native action indices.
Check unsupported upright physics, no resets, no references after handback,
unknown object success, bounded cleanup and released endpoints.

Render the live case and arm/hand speeds from recorded physics, without
rerunning control. Obtain independent evidence review and retain failed trials
if any. This is an empty-scene integration/recovery check; hardware alignment
and bottle-to-tray success remain separate validation work.

Completed on 2026-10-13: `run01` passed all three cases and the independent
raw-evidence audit. The replay and arm/hand speed plot were rendered and
inspected; all owned processes were reaped. The
[validation guide](/docs/platforms/humanoid/g1/sonic-service-native-validation.md)
records results, provenance, cleanup and limitations, including measured
hand/lower-body collision-geometry overlap. Production modules were unchanged.
The next step is the reusable hardware observation adapter and read-only
alignment validation.
