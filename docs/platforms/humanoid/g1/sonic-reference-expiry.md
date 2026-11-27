# SONIC receiver reference expiry: stationary simulation pilot

On 2026-10-06, an opt-in receiver guard was added to an isolated C++ SONIC checkout and tested with DimOS references and MuJoCo. Repeated duplicate packets and publisher disconnection expired the stream; the controller returned to its idle planner. Late packets could not restart streaming. The physical G1 and its running controller were unchanged.

This is a local prototype patch, not an upstream DimOS feature or a deployed PC2 change. The [previous trial](/docs/platforms/humanoid/g1/sonic-simulation-validation.md) measured the unmodified receiver retaining the last reference indefinitely.

## What changed

The new `--reference-timeout-ms` option enables a narrow stationary v4 contract. The default is unchanged. The trial used **200 ms**, measured on the receiving controller's monotonic clock.

- An entirely valid packet with a strictly increasing index refreshes the deadline. Token and hand arrays must all validate before any part is accepted. Duplicate indices, older indices, malformed values, and missing packets do not refresh it.
- Entering streaming starts the same bounded deadline even if no first reference arrives. The index high-water survives re-arm for the receiver process's lifetime.
- Expiry requests an idle planner handover. A cached external token may remain active during planner initialization; expiry is not instantaneous physical stopping.
- Recovery requires the receiver to consume a valid planner command and then a separate stream command. Endpoint keyboard toggles cannot bypass this sequence. Observe feedback between those requests because the existing command buffer can collapse rapidly sent mode changes.
- This pilot disables the planner movement/VR topic while the guard is enabled. Navigation is outside its contract.
- A new `reference_guard` telemetry object reports receiver state and planner confirmation. Confirmation requires a successful policy tick using the local encoder and enabled, initialized planner motion in effective encoder mode 0. It is not a task cancellation or measured-settling result.

No extra DDS motor writer was introduced. SONIC remains the body and hand command owner.

## Measured result

The successful [run02 result](/docs/artifacts/g1_sonic_expiry_20261006/run02/result.json) contains **20 passing runtime assertions**. The [offline summary](/docs/artifacts/g1_sonic_expiry_20261006/run02/summary.json) additionally verifies that the no-first-frame case entered a new armed epoch without accepting a reference before expiring.

| Case | Observed outcome |
|---|---|
| Healthy increasing references | Stream accepted and applied; returned token matched the supplied token |
| Older reference with changed token | Rejected; token unchanged |
| NaN in final hand slot, with newer index | Entire packet rejected; high-water and token unchanged |
| Repeated duplicate references | Expired; first expiry sample had reference age 210 ms; planner confirmed afterward |
| Publisher socket closed | First expiry sample had reference age 208 ms; planner confirmed 370 ms after socket closure |
| Late increasing references and repeated stream commands | No automatic re-arm |
| Malformed planner acknowledgment | Did not unlock re-arm |
| Explicit planner acknowledgment followed by stream request | New streaming epoch accepted |
| No first frame after arming | Expired at observed age 209 ms; planner confirmed afterward |
| Manual handback | Planner confirmation observed |
| Keyboard Enter burst during streaming | Did not toggle the guarded stream |

These are sampled observations from one successful trial, not guaranteed latency bounds. A final healthy-packet drain occurred before socket closure; therefore expiry appeared roughly 150 ms after closure while reference age was already about 208 ms.

Across the streaming/fault/recovery interval before deliberate controller shutdown, 201 physics samples showed pelvis height **0.769–0.784 m**, maximum tilt **4.39°**, support disabled, and **zero automatic resets**. The DimOS feedback module received **1,042 valid packets**, with no malformed packets or rejected feedback indices at the final handback check. The guard's rejected reference count is a separate metric.

The private network namespace was `net:[4026533320]`, with loopback as its only interface, distinct from host namespace `net:[4026531840]`. Trial processes exited; a targeted process check found no survivors.

## Source and verification

- Isolated worktree: `/tmp/dimos-sonic-expiry-20261006`
- Branch: `jihun/fix/sonic-reference-expiry`
- Base commit: `7c6d710b783a984ede504d24a65bc70e486b3b1d`
- Complete source patch, including tests (`/docs/artifacts/g1_sonic_expiry_20261006/sonic-reference-expiry.patch`)
- [Design and limits](/docs/artifacts/g1_sonic_expiry_20261006/design.md)
- [Source, binary and script hashes](/docs/artifacts/g1_sonic_expiry_20261006/provenance.json)
- Simulation probe (`/docs/artifacts/g1_sonic_expiry_20261006/demo_expiry.py`) and raw telemetry (`/docs/artifacts/g1_sonic_expiry_20261006/run02/telemetry.jsonl`)

The reviewed controller built successfully. Ten new deterministic C++ tests cover lease timing and packet validation, including NaN rejection under the deployment build's `-ffast-math`. All **18 available native tests passed**. The full 19-test invocation crashes in the existing `FK.TestFKAndGlobalVelocities` test because `reference/bones_072925_test/` is absent; that test dereferences an empty motion list. The same fixture is absent in the original checkout. This is not a full-suite pass. See full invocation log (`/docs/artifacts/g1_sonic_expiry_20261006/native-tests.log`) and available-test results (`/docs/artifacts/g1_sonic_expiry_20261006/native-tests-reviewed.log`).

The first probe attempt stopped on a test observation race: its high-water snapshot preceded the final healthy packet's arrival. Telemetry showed the malformed reference was rejected. The successful probe drains that healthy send before taking the snapshot. A source review also found and corrected a keyboard bypass, shared lifecycle flag races, and pre-inference encoder-mode confirmation before run02.

The patch was checked with `git apply --reverse --check` against the tested worktree. It is saved in this repository so that the source changes survive removal of the temporary worktree. Neither repository was committed or pushed.

## Reproduce on this workstation

The existing temporary worktree contains the built controller; the previous trial's copied ONNX/TensorRT assets remain under `/tmp/dimos-sonic-sim-assets-20261006`. The probe uses the original GR00T checkout only for its simulation environment and reference assets. For rebuilding the worktree, use its CMake project with the workstation's TensorRT and ONNX Runtime paths; the patch applies to the base commit above. Do not substitute the installed PC2 binary.

Use a new output directory for each run. The command below deliberately creates a private namespace with no robot network interface:

```bash skip
cd /home/jihun/work/dimos_dev
sonic_host_netns="$(readlink /proc/self/ns/net)"
unshare --user --map-root-user --net /bin/bash -c '
  set -e
  ip link set lo up
  exec timeout -k 10 210 /home/jihun/work/dimos_dev/.venv/bin/python -u \
    docs/artifacts/g1_sonic_expiry_20261006/demo_expiry.py \
    --groot /home/jihun/work/GR00T-WholeBodyControl \
    --binary /tmp/dimos-sonic-expiry-20261006/gear_sonic_deploy/target/release/g1_deploy_onnx_ref \
    --assets /tmp/dimos-sonic-sim-assets-20261006 \
    --output /tmp/dimos-sonic-expiry-repeat \
    --reference-port 15556 --feedback-port 15557 --host-netns "$1"
' _ "$sonic_host_netns"
```

## What remains before the pick-and-place service

1. Receiver status is now exposed and validated through DimOS's typed feedback API. The [follow-up trial](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) repeats all 20 runtime assertions through that API. A production command-session adapter remains to be built; handover confirmation, measured settling and task completion remain separate observations.
2. A small encoded elbow reference and interruption trial is now complete. Handover and sampled settling were observed, but the +0.1 rad reference produced only +0.031 rad of measured change and missed the incremental tracking criterion. Diagnose this reference path before offering precise arm execution. The 64-value SONIC token is a learned latent reference, not an arm joint or Cartesian target.
3. Establish Dex3 anatomical mapping and a credible hand/contact model, then define recovery while holding an object. This unloaded trial does not establish grasp behavior.
4. Validate camera/tool transforms and timing, then commission the exact PC2 controller build and guarded hardware procedures before connecting the object-to-tray service.

The protocol still lacks session nonces, capture timestamps and acknowledged command transactions. Delayed increasing packets can satisfy the receiver lease, and a restarted sender must continue above its previous high-water. `applied_index` means installed in the endpoint, not necessarily consumed by the same published policy tick. During first-frame grace, `age_ms` counts from arm while accepted/applied indices may describe the preceding epoch. Planner initialization failure retains the upstream stop/damping behavior; a safe recovery on planner failure has not been established.
