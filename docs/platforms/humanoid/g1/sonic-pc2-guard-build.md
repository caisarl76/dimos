# Isolated SONIC reference guard on pc2_222

On 2026-10-14, the existing simulation-tested reference guard was ported to the
exact SONIC sources installed on pc2_222 and compiled on its ARM computer.
**The build is prepared for operator commissioning; it has not been run against
the physical robot.** SONIC remains the body and Dex3 motor-command owner.

The guard addresses a specific integration problem: if DimOS stops supplying
fresh references, the receiver requests an idle planner handback instead of
retaining its last external reference indefinitely. The provisional timeout is
200 ms. This bounds the receiver's reference lease, not physical stopping time.
See the [guard contract and prior simulation results](/docs/platforms/humanoid/g1/sonic-reference-expiry.md).

## Package and preservation

PC2 package: `/home/unitree/dimos-sonic-guard-20261014`.

- Original checkout: `/home/unitree/GR00T-WholeBodyControl`, HEAD
  `087f9ac01d46f6d8e4d0b73c01ae64799f292a38` plus captured local modifications.
- All 73 captured original source/build files still match their hashes. The
  original executable remains unchanged. No commit, install, reboot, controller
  start/stop, mode change or motion command was issued by this preparation.
- The port preserves PC2's motor-gain options and body/hand torque telemetry.
  Ten files differ from the captured source: nine original guard-port files and
  one additional telemetry test file.
- Build outputs, logs and model/cache copies are isolated. The copied model and
  motion assets occupy approximately 1.94 GB logically; the copy requested
  filesystem reflinks where supported. No models were downloaded.
- Third-party libraries and GoogleTest source are reused from the existing
  installation. Keep the original checkout in place while using this package.
- The build uses GCC 11.4, CUDA 12.6, TensorRT 10.3 and ONNX Runtime 1.16.3.
  `HAS_ROS2=0`: `--output-type all` exposes the available ZMQ output in this build.
  ROS2 is not required for this DimOS/SONIC route.

Original binary SHA256:
`5526b184875f6f5e392cf6a5c95a8f55ab2d0a698177078a214db2231a0ec425`.

Prepared binary SHA256:
`c59a92204944d1fb242911dd5111ac8cb1a1144810809da05453f8ea9ac7d42e`.

## Verification

- ARM controller and native test targets compiled successfully with the normal
  `-O3 -ffast-math` flags, using two low-priority build jobs.
- **14 native tests passed:** five lease cases, five packet-validation cases,
  and four actual ZMQ/MessagePack round trips covering guard on/off and heading
  present/absent. Round trips preserved all three torque arrays and all ten
  guard fields, and consumed exactly one complete MessagePack object.
- One existing test, `FK.TestFKAndGlobalVelocities`, was excluded because its
  required `reference/bones_072925_test/` fixture is absent in the original
  installation. This is not a full-suite pass.
- Usage, eight invalid-timeout cases and a missing-required-route case were
  checked before controller construction. Runtime libraries resolve without
  missing dependencies. The valid motor-control command was not executed.
- An independent source review found no blocking port issue. Its suggested
  telemetry coverage was added. The first test compilation exposed the
  upstream output headers' include-order dependency; the test was corrected to
  match the native executable's include order, without changing runtime code.
- The earlier MuJoCo guard trial remains evidence for the original guard logic.
  This exact ARM port has not had an end-to-end simulation or hardware recovery
  trial. Neither these tests nor planner confirmation establish grasp success.

## Latest observed processes

PC2 rebooted after the initial source snapshot. At the final inventory, camera
PID 2952 and the original SONIC PID 3355 were running. SONIC used
`zmq_manager`, `enP8p1s0`, SONIC v1.1 and `--zmq-host localhost`.
Camera and telemetry ports 5555/5557 were listening; no local reference publisher
was listening on 5556. These are point-in-time observations, not persistent PIDs.

The prepared launcher therefore defaults to **127.0.0.1:5556 on PC2**. DimOS's
local observation/session/execution adapters will own that publisher in the next
commissioning step. The previous workstation route, 192.168.0.62, can be selected
explicitly, but the delivery timestamps used by those adapters require a shared
monotonic-clock domain; do not split that pipeline across hosts without an
explicit timing design.

## Operator launch

The user owns controller deployment. On PC2, preview the command without
starting anything:

```bash skip
bash /home/unitree/dimos-sonic-guard-20261014/launch-guarded.sh
```

For the unloaded commissioning step, have the robot supported, the work area
clear and the emergency-stop operator ready. Shut down the current SONIC session
using the normal supported operator procedure. **Launching the new controller
performs native posture and hand initialization before `Init Done`; it can move
before any subsequent start command.**

Then run:

```bash skip
bash /home/unitree/dimos-sonic-guard-20261014/launch-guarded.sh --run
```

This checks the prepared binary hash and refuses to launch while another SONIC
process matches its process check. It sets the inspected library paths and calls
the isolated executable directly with:

- the copied SONIC v1.1 decoder, encoder, observation configuration and planner;
- the physical DDS interface `enP8p1s0`, with CRC checking enabled;
- `--input-type zmq_manager --output-type all`;
- `--zmq-host 127.0.0.1 --zmq-port 5556 --reference-timeout-ms 200`.

The original `deploy.sh` does not forward the new timeout option; use the new
launcher. It does not invoke the original environment setup script or rebuild
anything. To intentionally use the previous workstation publisher instead:

```bash skip
bash /home/unitree/dimos-sonic-guard-20261014/launch-guarded.sh --run --zmq-host 192.168.0.62
```

For this first launch, leave it at `Init Done / waiting for a start command` and
report its terminal output. Configuration messages are expected while waiting;
`g1_debug` and its `reference_guard` object require the native control loop.
No automatic start is included in the launcher.

## Remaining commissioning

1. Verify the operator-launched process uses the prepared binary, models,
   timeout and endpoint; check the waiting-state configuration without motion.
2. Package the existing camera/DDS delivery adapters and session/execution
   modules as the persistent PC2 service. Their prior live validation is in
   [delivery observations](/docs/platforms/humanoid/g1/sonic-delivery-live-validation.md).
3. With the operator controlling native startup, observe guarded planner
   feedback and run a bounded unloaded stream/expiry/recovery trial. Confirm
   measured settling separately from planner confirmation.
4. Connect the chosen LeRobot policy, verify hand mapping and the lowered grasp
   workspace, then attempt the supervised bottle-to-tray task. Measure the
   physical outcome separately from software execution status.

This stationary guard disables the planner movement/VR topic. Navigation and
loaded-object recovery remain outside the tested contract. Sensor acquisition
age remains unknown; the current adapter checks delivery timing only.

## Evidence

The artifact directory (`../../../artifacts/g1_pc2_guard_20261014/`) contains the
exact source snapshot, port patch (`../../../artifacts/g1_pc2_guard_20261014/pc2-reference-guard.patch`),
source hashes, [independent review](/docs/artifacts/g1_pc2_guard_20261014/review.md),
reproducible build scripts, launch script and model hashes.
See native test results (`../../../artifacts/g1_pc2_guard_20261014/pc2-validation/native-tests.log`),
[validation result](/docs/artifacts/g1_pc2_guard_20261014/pc2-validation/validation-result.json)
and [final inventory](/docs/artifacts/g1_pc2_guard_20261014/pc2-validation/post-build-inventory.json).
