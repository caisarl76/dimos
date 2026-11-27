# DimOS → external SONIC: first MuJoCo contract trial

On 2026-10-06, a bounded headless trial exercised the workstation C++ SONIC controller and MuJoCo G1 with the actual DimOS reference encoder and feedback module. The planner → streamed-reference → planner sequence completed. Publisher loss retained the reference; shutdown applied damping and the unsupported simulated robot fell.

This is one stationary control-contract trial. It does not qualify pick-and-place, walking, grasp retention, collision avoidance or hardware cancellation.

## Observed results

| Check | Measurement | Service implication |
| --- | --- | --- |
| DimOS v4 reference → C++ | Reported token matched the sent 64-value planner token exactly. | The encoder works with this executable, beyond the offline serializer comparison. |
| Duplicate index `100`, changed token | Marker accepted, error below `5e-11`; 15 distinct feedback samples. | A repeated input index does not reject a changed reference. |
| Older index `99`, different marker | Marker accepted, error below `5e-11`; 16 distinct feedback samples. | Input indices provide no effective ordering protection here. |
| Publisher closed for two seconds | 101 distinct feedback samples; retained token error `0`; feedback `fresh`. | Fresh robot feedback does not prove fresh commands. Disconnect is not cancellation. |
| Planner handback | Controller logged planner mode, reinitialized planner and resumed encoder. Through streaming/handback, height stayed 0.7860–0.7877 m, maximum tilt 2.69°, zero automatic resets. | Handback worked for this stationary, unloaded reference. |
| Controller stop | Simulator received body `kp=0`, `kd=8`; first recorded damping sample about 120 ms after request. Controller exited `0`. | Shutdown is not a posture hold. This sampled timing is not a guaranteed deadline. |
| Physics after stop | Robot fell; two automatic resets recorded. | Do not equate shutdown with balanced task cancellation. |
| Feedback after exit | DimOS reported `stale`, arrival age 5.54 seconds when checked. | Expiry worked; this check did not measure its exact transition latency. |

The final fresh module observation had 1,069 accepted packets, zero invalid packets and zero rejected **feedback** indices. These differ from the input `frame_index` tested above.

Recorded height and tilt across publisher loss, handback and damping (`/docs/artifacts/g1_sonic_sim_20261006/run02/control-lifecycle.png`)

The upward jumps after shutdown are simulator resets following falls. The worker counts simulation-time discontinuities on every physics step. The explicit reset during virtual-support release is excluded from the automatic-reset counter, so an upright reset pose cannot silently count as uninterrupted stability.

## Environment and limits

- GR00T checkout `/home/jihun/work/GR00T-WholeBodyControl`, observed revision `7c6d710b783a984ede504d24a65bc70e486b3b1d`; existing local edits preserved.
- Controller `gear_sonic_deploy/target/release/g1_deploy_onnx_ref`, SHA256 `eaaeb1c61e8285e7c08f071c4bec40955effd3e9d561318b9ea7c126c8c138d4`.
- SONIC v1.1 decoder/encoder and V2 target-velocity planner, existing TensorRT caches on GPU 0 (RTX 3060). [Asset hashes](/docs/artifacts/g1_sonic_sim_20261006/assets.json) identify copied files.
- MuJoCo 3.8.1 in `.venv_sim`, `scene_43dof.xml`, 29 body plus 7+7 hand joints, configured 5 ms physics timestep. This was not a real-time performance benchmark.
- `zmq_manager` input, `zmq` output, DDS domain `0` on `lo`; reference port `15556`, feedback port `15557`.
- Actual DimOS `encode_command`, `encode_token_reference` and `SonicFeedbackModule`. References used a captured planner token and existing hand commands. Index tests applied distinct ±0.001 markers to token element zero. No Cartesian reach or grasp was attempted.

All probe processes ran in a newly created `unshare --user --map-root-user --net` namespace with **only loopback**, with no physical interface or veth. Trial namespace `net:[4026533302]` differed from separately verified host `net:[4026531840]`. The raw launch's comparison ID came from Codex's restricted environment; [the isolation record](/docs/artifacts/g1_sonic_sim_20261006/isolation-verification.json) preserves that distinction. Domain 0 was isolated by the namespace, not by a different numeric domain ID.

The real G1 and existing DimOS simulation were unchanged. Probe processes exited; a subsequent targeted process check found no remaining controller or simulator worker from the trial.

This executable differs from the PC2 executable recorded during hardware feedback validation. Findings apply directly to this workstation build; equivalent PC2 behavior remains to be commissioned. The XML scene itself warns that its hand model is not stable for simulation. This trial establishes neither anatomical Dex3 mapping nor grasp/contact fidelity.

## Evidence

- [Summary](/docs/artifacts/g1_sonic_sim_20261006/run02/summary.json), [phase results](/docs/artifacts/g1_sonic_sim_20261006/run02/result.json), [per-phase physics](/docs/artifacts/g1_sonic_sim_20261006/run02/physics-summary.json).
- [Exact command and binary hash](/docs/artifacts/g1_sonic_sim_20261006/run02/manifest.json), controller log (`/docs/artifacts/g1_sonic_sim_20261006/run02/deploy.log`), simulator log (`/docs/artifacts/g1_sonic_sim_20261006/run02/sim.log`).
- Telemetry (`/docs/artifacts/g1_sonic_sim_20261006/run02/telemetry.jsonl`), physics and received body gains (`/docs/artifacts/g1_sonic_sim_20261006/run02/physics.jsonl`).
- Bounded probe (`/docs/artifacts/g1_sonic_sim_20261006/demo_contract.py`), simulator worker (`/docs/artifacts/g1_sonic_sim_20261006/demo_sim_host.py`), offline analyzer (`/docs/artifacts/g1_sonic_sim_20261006/demo_analyze.py`).

The first launch failed before control because the temporary planner copy lacked `V2` in its directory path. The native loader selects its version from the path. Restoring `planner/target_vel/V2/` fixed initialization; [run01](/docs/artifacts/g1_sonic_sim_20261006/run01/result.json) retains that failure. No model download or rebuild was required.

## Repeat on this workstation

Prepared assets are under `/tmp/dimos-sonic-sim-assets-20261006`: copied decoder/encoder files and caches, observation YAML, and the planner under `planner/target_vel/V2/`. Restore those files from the source locations above if temporary storage has been cleared. Use a new output directory; the probe refuses to overwrite a trial.

Run from the normal workstation terminal:

```bash skip
cd /home/jihun/work/dimos_dev
sonic_host_netns="$(readlink /proc/self/ns/net)"
unshare --user --map-root-user --net /bin/bash -c '
  set -e
  ip link set lo up
  exec timeout -k 10 210 /home/jihun/work/dimos_dev/.venv/bin/python -u \
    /home/jihun/work/dimos_dev/docs/artifacts/g1_sonic_sim_20261006/demo_contract.py \
    --groot /home/jihun/work/GR00T-WholeBodyControl \
    --assets /tmp/dimos-sonic-sim-assets-20261006 \
    --output /tmp/dimos-sonic-contract-repeat \
    --reference-port 15556 --feedback-port 15557 --host-netns "$1"
' sonic-sim "$sonic_host_netns"
```

The scripts require a separate namespace with only `lo`. They terminate their child processes and save results on handled failures; the outer command adds a time limit. `probe_completed` means the experiment reached its end, not that every observed behavior is acceptable for a service.

Regenerate the summary/plot without a controller:

```bash skip
MPLCONFIGDIR=/tmp/dimos-matplotlib .venv/bin/python \
  docs/artifacts/g1_sonic_sim_20261006/demo_analyze.py \
  docs/artifacts/g1_sonic_sim_20261006/run02
```

The three scripts passed scoped Ruff lint/format checks. This step did not change the production encoder/feedback module, rebuild SONIC or run the full repository test suite.

## Next service-development step

**Follow-up completed:** an opt-in [receiver-expiry and planner-confirmation pilot](/docs/platforms/humanoid/g1/sonic-reference-expiry.md) now passes an isolated stationary MuJoCo fault trial. Its controller patch is local and has not been deployed on PC2. The remaining service/session work below still applies.

Define and test the DimOS session/cancellation interface in simulation: separate measured-state freshness from reference freshness, reject stale/out-of-order references at the service boundary, stop task progression on failure, and verify the commissioned planner handover before reporting cancellation. A mode request alone is insufficient evidence; shutdown/damping remains a separate operator action.

Publisher-side checks alone cannot handle a crashed publisher. Reference expiry must also be enforced at the receiving controller or an independently supervised control boundary, with a tested fallback. The controller integration also needs observable mode/handover confirmation; the current DimOS feedback interface provides no task acknowledgement. These requirements come before enabling autonomous hardware execution.

Manipulation still needs validated SONIC reference generation from task goals, Dex3 mapping/model with credible contact behavior, and calibrated camera/tool frames. A handover while holding an object must establish what happens to hand targets; this stationary trial supplied no evidence for that case.
