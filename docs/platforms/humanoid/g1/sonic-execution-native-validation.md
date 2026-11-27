# Native validation of the DimOS SONIC execution module

This milestone connects the [execution module](/docs/platforms/humanoid/g1/sonic-execution-module.md) to
live MuJoCo observations, the existing LeRobot Pi0.5 client and guarded native
SONIC. DimOS owns task admission, streaming, status, cancellation and recovery;
the policy supplies actions and SONIC retains motor control.

The harness builds `SonicExecutionModule.blueprint()` with one DimOS worker
and calls its RPC proxy. It replaces the earlier experiment's direct
`SonicTaskSession` loop. Camera rendering and blocking inference run outside
the module's event loop. The native controller, simulator and temporary policy
server run in a private network namespace containing only loopback.

## Validated result: 2026-10-12

[`run05/result.json`](/docs/artifacts/g1_sonic_module_native_20261012/run05/result.json)
and the
[independent audit](/docs/artifacts/g1_sonic_module_native_20261012/run05/audit.json)
pass all three cases through the actual DimOS worker:

| Case | Outcome | Accepted predictions | Confirmed handback after request |
| --- | --- | ---: | ---: |
| Live RGB/state for 6.014 seconds | Finished | 18 | 140 ms |
| Cancel after about 1.51 seconds | Cancelled; outstanding result rejected | 5 | 128 ms |
| Freeze camera after about 1.51 seconds | Observation expired; planner recovered | 6 | 116 ms |

Camera loss is an expected task failure and a passing recovery check. The
session expired at an observation age of 850.013 ms, about 639 ms after the
freeze. A fresh frame already submitted for inference could finish after the
freeze; its timestamp stayed unchanged. No pose references followed the first
handback command in any case.

The audit matched all 331 camera snapshots to their physics state and checked
all 29 accepted prediction hashes against the journals. During normal
streaming, 18 distinct images and joint states produced inference round trips
of 298–314 ms. Predictions arrived with observations aged 344–457 ms. Native
accepted and applied reference indices advanced throughout execution.

The robot remained unsupported and upright, without resets. In the live
case, minimum pelvis height was 0.781 m and maximum tilt was 8.43°. Peak arm
speed including recovery was 3.348 rad/s in the live case and 5.310 rad/s
across all cases. These are measurements, not hardware motion-limit approval.
The largest peak was right wrist roll about 1.17 seconds before cancellation;
it is not attributed to the cancellation command.

Terminal task status was captured through RPC before shutdown. The module
stopped, the coordinator released its worker, the reference endpoint could be
rebound, and simulator/controller/camera children exited with code 0. The
launcher also reaped its temporary policy process. The existing host policy
server was left running. No real robot was connected.

- MuJoCo replay with the actual RGB inputs (`../../../artifacts/g1_sonic_module_native_20261012/run05/replay/live_policy_replay.mp4`)
- Measured arm and hand speeds (`../../../artifacts/g1_sonic_module_native_20261012/run05/replay/joint_speeds.png`)
- [Runtime/source manifest](/docs/artifacts/g1_sonic_module_native_20261012/run05/manifest.json)
- [Policy configuration and recorded hashes](/docs/artifacts/g1_sonic_module_native_20261012/run05/policy-provenance.json)

The replay renders recorded physics without running control. Its right pane
holds the latest image submitted to the policy until the next request.

Verification: **238 focused SONIC tests passed**, including 16 module tests;
focused Ruff formatting/lint and production mypy passed. An independent review
checked raw journals, wire packets, native telemetry and cleanup evidence and
found no blocking issues. The full repository suite was not run. The existing
pytest startup `ResourceWarning` remains visible in the test log.

Test log (`../../../artifacts/g1_sonic_module_native_20261012/pytest.log`) and
[verification record](/docs/artifacts/g1_sonic_module_native_20261012/verification.json).

## Required checks

| Case | Required result |
| --- | --- |
| Live camera and measured state | Six seconds in `running`, at least 15 accepted predictions, then confirmed planner handback. |
| Cancel after 1.5 seconds in `running` | Stop references, confirm planner handback and reject a late prediction. |
| Freeze camera after 1.5 seconds in `running` | Preserve the last capture timestamp, expire the observation and confirm planner handback. |

Camera input must be younger than 200 ms when requested. The task expires
when its last accepted observation reaches 850 ms; inference completion does
not refresh that timestamp. Chunks use their original 50 Hz rate, policy
requests are scheduled every 350 ms, and native reference expiry is 200 ms.

The independent audit matches camera RGB/state to physics snapshots, hashes
policy predictions against the module's journal, checks contiguous wire
indices and native accepted/applied progress, and rejects references after
the first handback command. It also checks unsupported upright physics,
terminal RPC status and child-process cleanup. Wire receipt spacing can batch
at the observer and is not a measurement of publisher scheduling jitter.

## Findings during integration

The first trial exposed a receive-loop race in `SonicExecutionModule`: a
packet arriving immediately after an empty-queue check could trigger the
backlog guard even though the receive budget had not been consumed. Both
socket loops now check for excess backlog only after draining all 64 allowed
packets. Six regression cases cover the arrival race, exactly 64 packets and
genuine overflow on both sockets. The limits remain unchanged.

A second trial rejected a policy result whose observation was already
1.13 seconds old. It recorded failure and confirmed planner recovery, with
clean shutdown. Its roughly one-second inference latency exceeded the
configured freshness budget. The shared policy server remained running;
the follow-up uses a temporary copy on GPU 1.

The temporary server's first 120-second startup window expired before native
simulation launched. The diagnostic launcher allows up to 300 seconds for
the same local checkpoint to initialize, enables offline loading, and limits
OMP/MKL threads to four. This startup window does not change task timing.
Timed stack traces identified CPU model initialization during the load.

With the temporary server, `run04` passed normal streaming and cancellation.
However, the camera-loss case expired before its intended injection: capture
age, 400 ms request spacing, inference and RPC delivery consumed the 850 ms
budget. The next result reached admission just after expiry. The harness now
requests every 350 ms to provide more margin; model actions, observation-age
limits and native reference expiry remain unchanged. `run04` remains a failed
overall trial because the intended camera-freeze test was not exercised.

## Evidence and reproduction

Artifacts live in
`g1_sonic_module_native_20261012` (`../../../artifacts/g1_sonic_module_native_20261012/`).
The failed trials are retained as `run01` and `run02`; `run03` contains only
launcher/policy logs because native simulation never started. `run01` also
exposed a harness reporting bug: the worker's RPC endpoint closes during
`stop()`. The harness now captures terminal status before shutdown and records
stop completion, coordinator cleanup and endpoint release separately.

The private launcher uses this workstation's existing checkpoint and guarded
runtime assets; it is not a portable installation command:

```bash skip
bash docs/artifacts/g1_sonic_module_native_20261012/demo_run_private_policy_trial.sh \
  /absolute/path/to/a/new/output-directory
.venv/bin/python docs/artifacts/g1_sonic_module_native_20261012/demo_audit.py \
  /absolute/path/to/the/output-directory
```

The scene is the empty G1 checker-floor scene. The checkpoint was trained for
bottle handover, and its output here tests integration and recovery. Object
success remains unknown. D435i calibration, hardware hand ordering, physical
motion limits and recovery while holding an object need separate validation.

The next service-layer step is to connect a coordinator to this execution
interface: request a task, provide observations to the selected policy, report
progress, handle cancel/failure and verify the object-level result. That keeps
grasp strategy and policy training with the action provider.
