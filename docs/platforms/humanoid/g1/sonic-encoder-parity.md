# SONIC arm tracking: encoder parity and timing diagnosis

On 2026-10-07, the [first moving-reference trial](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) was followed by three instrumented, isolated MuJoCo trials. **The tested encoder conversions agree, and sent token content reaches the decoder. Correcting reference scheduling to approximately 50 Hz did not make the reach accurate.**

This step used the workstation only. No PC2 session, robot mode, hardware package or physical motor command was changed. The changes are diagnostic artifacts, not a production manipulation adapter.

## What was compared

The temporary C++ diagnostic captures raw inputs inside the actual SONIC observation gatherers: ten future position/velocity frames in motor order, their adjusted reference orientations and the robot quaternion. It records the final 1,751-float encoder input and 64-float TensorRT output after encoding. Python reconstructs those same observations through the experimental helper and compares the output of the same ONNX model.

Across **1,946 native encoder frames** from three runs:

| Comparison | Result |
|---|---|
| Joint positions, joint velocities and unused input blocks | Exact agreement |
| Largest orientation/input difference | 5.96 × 10⁻⁸, below the declared 10⁻⁶ tolerance |
| Python ONNX Runtime versus native TensorRT tokens, identical inputs | Exact agreement in all captured frames |
| Python reconstructed inputs versus native tokens | Exact token agreement in all captured frames |

The captured inputs contain enough variation to distinguish swapped elbows, incorrect orientation flattening, omitted velocities and shifted future frames; each deliberately wrong interpretation exceeds the comparison tolerance. These are checks on the recorded mode-0 cases without upper-body overrides, not proof for every encoder mode or robot pose.

The native encoder is bypassed during external-token streaming, so those native captures concern planner inputs before/after streaming. To test the synthetic arm inputs directly, a separate offline C++ replay (`/docs/artifacts/g1_sonic_parity_20261007/demo_encoder_replay.cpp`) uses the actual `EncoderEngine`, FP32 configuration, CUDA graph capture and existing TensorRT library. **All 279 saved synthetic arm inputs produced tokens exactly equal to Python's outputs.** It instantiates no DDS or robot interface.

A separate trace records the decoder observation buffer after successful inference/output. After excluding the first 100 ms of the stream transition, **1,234 decoder records** matched the latest preceding sent token content. Decoder timestamps are recorded after inference; reference timestamps precede socket sending. Tokens can repeat across packets. This establishes content agreement, not packet identity, precise delivery latency or individual trajectory acknowledgments.

Evidence: [three-run summary](/docs/artifacts/g1_sonic_parity_20261007/summary.json), [run01 parity](/docs/artifacts/g1_sonic_parity_20261007/run01/parity-summary.json), [run02 parity](/docs/artifacts/g1_sonic_parity_20261007/run02/parity-summary.json), [run03 parity](/docs/artifacts/g1_sonic_parity_20261007/run03/parity-summary.json), [synthetic-input replay](/docs/artifacts/g1_sonic_parity_20261007/run01/external-encoder-parity.json), [sensitivity checks](/docs/artifacts/g1_sonic_parity_20261007/sensitivity-checks.json).

## Scheduling correction and tracking result

The old experimental publisher scheduled the next send 20 ms after completing each send/encode cycle and polled feedback in 10 ms blocks. Its observed reference rate was approximately 33 Hz. The new diagnostic probe (`/docs/artifacts/g1_sonic_parity_20261007/demo_parity_sim.py`) maintains a 20 ms deadline, shortens polling near that deadline and skips missed periods without sending a catch-up burst.

All runs requested the same +0.1 rad nominal elbow-reference change, using a two-second ramp followed by a one-second hold. The captured starting stance and measured offsets vary between runs; this is not a deterministic paired benchmark proving that one rate improves or worsens tracking.

| Trial | Mean reference rate | Measured elbow change | Absolute endpoint error | Tracking criterion |
|---|---:|---:|---:|---|
| Original scheduling, instrumented | 33.46 Hz | +0.02074 rad | −0.10172 rad | Missed |
| Corrected scheduling, first run | 49.82 Hz | +0.00848 rad | −0.06938 rad | Missed |
| Corrected scheduling, repeat | 49.86 Hz | +0.01081 rad | −0.09099 rad | Missed |

The criterion requires absolute and incremental reach errors, and absolute and relative return errors, each within 0.05 rad. Sending nearer 50 Hz alone did not meet it. Occasional longer send gaps remained; approximately 50 Hz is the observed mean, not a real-time deadline guarantee.

All three trials completed the interruption/handover checks and verified an overlapping sampled settling window of at least 0.5 seconds. Before deliberate shutdown, support was disabled, automatic resets were zero, and maximum recorded pelvis tilt was below 3.34°. Each controller exited with code 0; cleanup reported no errors. These are unloaded simulation observations. They do not qualify object contact, loaded recovery or physical hardware.

## What the token sweep tells us

An offline sweep (`/docs/artifacts/g1_sonic_parity_20261007/demo_token_sweep.py`) held the captured body pose, heading and velocities fixed while changing all ten future elbow targets from +0 to +0.1 rad in 0.001 rad steps. Its **101 inputs produced 14 distinct tokens**; the first observed token change was at +0.028 rad on that grid, for this particular held input. This is not an exact continuous threshold. The largest change in a token coordinate was 0.0625.

This demonstrates intervals where nearby joint references encode identically. A simple controller that assumes every small reference correction changes the latent input may stall. It does **not** establish quantization as the sole cause of the simulated tracking errors, nor does it establish the achievable accuracy of SONIC in general. The learned policy also consumes body state/history, and its action is not the requested reference joint angle.

Evidence: [sweep result](/docs/artifacts/g1_sonic_parity_20261007/run01/token-sweep.json), saved offsets/tokens (`/docs/artifacts/g1_sonic_parity_20261007/run01/token-sweep.npz`).

## Reproduce the diagnosis

The new probe uses the same private loopback namespace procedure as the [expiry pilot](/docs/platforms/humanoid/g1/sonic-reference-expiry.md#reproduce-on-this-workstation). Substitute the new diagnostic probe, `/tmp/dimos-sonic-parity-20261007/g1_deploy_onnx_ref`, and a new output directory. Its policy-input logging option creates the encoder and decoder traces. Do not substitute a physical robot network interface.

Analyze a completed capture offline:

```bash skip
cd /home/jihun/work/dimos_dev
.venv/bin/python docs/artifacts/g1_sonic_parity_20261007/demo_analyze_parity.py \
  docs/artifacts/g1_sonic_parity_20261007/run01 \
  --encoder /tmp/dimos-sonic-sim-assets-20261006/model_encoder.onnx
```

The diagnostic patch (`/docs/artifacts/g1_sonic_parity_20261007/encoder-parity-diagnostic.patch`) adds logging to the preceding guarded native controller. The previous worktree's source and tested binary were restored after building the separate diagnostic binary. [Native provenance](/docs/artifacts/g1_sonic_parity_20261007/native-provenance.json) records the hashes. Diagnostic logging adds workload; its timing measurements are observations under that workload. No production DimOS module changed in this step.

## Next implementation milestone

Input conversion and reference-content delivery now have direct evidence. The next milestone is a **bounded reach driven by measured feedback in simulation**: define the measured goal, reference limits, completion tolerances, settling interval, timeout and planner handback behavior. It must account for the observed token plateaus and whole-body response, then pass repeated unloaded trials before becoming a DimOS manipulation executor.

The existing open-loop reference test remains a failed accuracy baseline. Dex3 grasping, calibrated end-effector accuracy and the object-to-tray task still follow the successful reach milestone. The [service plan](/docs/platforms/humanoid/g1/service-development-plan.md) remains the broader roadmap.

**Follow-up:** The [bounded feedback-reach experiment](/docs/platforms/humanoid/g1/sonic-feedback-reach.md) now meets its sampled elbow-goal criterion in two final-runner simulation trials. Holding the reached pose through a grasp and returning under command remain separate validation steps.
