# Handover: pc2 (.223) waist grinding after SONIC planner startup

> **Update (later session, 2026-10-14):** Offline executable comparison is done. See
> [offline-binary-comparison.md](/docs/artifacts/g1_pc2_223_incident_20261014/offline-binary-comparison.md).
> Summary: the compiled constants and motor path are equivalent, and the original
> binary matches its captured source. The guarded launcher skipped `setup_env.sh`,
> so `jetson_clocks` was **not** applied. The first `jetson_clocks` of this boot came at
> 16:31:30, after the incident. As a result, policy ran at 5.06 ms (original p50 1.77 ms)
> and sense-to-command latency was about 2×. The operator confirmed that the 16:31 run of the original SONIC (clocks pinned,
> same boot) did not grind.
> **Resolved 17:27:** the guarded binary was retested with the clock-preflight
> launcher (`launch-guarded.sh` `456bb1fd…`, clocks pinned) and was smooth. Policy
> took 1.71 ms, and the waist CSV is as smooth as the original's. Root cause:
> the launcher skipped `jetson_clocks`. See §8 of the note.
> **Startup-ordering hazard (step 4 below):** measured from the retest and original
> CSVs instead of a fake-engine harness. Both builds behave alike, and the
> example→planner switch step (0.2–0.29 rad) is smaller than the policy's own
> settling swings. No native change is warranted. See §10 of the note.

## Start here

**Unresolved hardware incident. The robot is stopped and supported, confirmed
by the operator. Do not restart SONIC or repeat the planner-start command to
reproduce the noise.** The latest user request is to hand this investigation
to a new session. No corrective controller change has been made.

Investigate why the newly staged guarded SONIC controller made a grinding
noise when the operator ran `start-idle-planner.py --start-idle-planner`.
The operator thinks the noise came from the **waist**, described it as two
commands fighting, and confirmed that **the original SONIC deployment ran
smoothly on this same robot and support setup**. Two simultaneous controllers
are a hypothesis, not an established finding.

The broader project is DimOS orchestration of GEAR-SONIC on Unitree G1. The
immediate task is diagnosing this incident, not resuming manipulation or
pick-and-place commissioning.

## Boundaries and workspace

- Local repository: `/home/jihun/work/dimos_dev` (dirty; many session files are
  untracked). Preserve all existing work. No commit/push requested.
- Robot: SSH `pc2`, `unitree@192.168.0.223`, onboard hostname `unitree-g1-nx`.
  **29 body joints; no hands attached.** Inspire hands are planned. Do not
  confuse this with pc2_222 (.222), which has Dex3 hands.
- The user owns native controller startup. All assistant-side investigation
  so far has been read-only on hardware: process/filesystem inspection and an
  eight-second DDS reader probe. No start/stop/reference/motor commands were
  sent by the assistant during diagnosis.
- Do not kill or disable Unitree services, especially `emergency_stop`, merely
  because discovery lists a command writer. A discovered endpoint need not
  be transmitting commands.
- Do not edit `docs/platforms/humanoid/g1/implementation-log.md`; another
  session owns it. This handover and its incident artifacts are separate.
- Before any later physical retest, prepare a concrete reviewed change and
  offline evidence, and obtain current operator readiness/authorization.
  Earlier support confirmation is not permission to restart during diagnosis.
- Feet-on-floor versus freely hanging was asked, but the reply only identified
  the waist. That detail remains unknown.

## What happened

1. We synced a separate DimOS copy and built a guarded native SONIC copy on
   .223, preserving its original installed repositories and native binary.
2. The operator launched the new copy. Before planner startup, a read-only
   process check verified exactly one `g1_deploy_onnx_ref` process, PID 9570,
   matching the staged binary hash and options below. Port 5557 was listening;
   port 5556 had no listener. This was not a DDS ownership check.
3. SONIC printed `Init Done`. The operator then ran the planner-start helper
   from a second terminal.
4. The native log transitioned to CONTROL, briefly reported the example
   `neutral_kick_R_001__A543`, initialized an IDLE planner (~36 ms inference),
   then reported `planner_motion`. The operator heard grinding.
5. The helper printed planner confirmation at feedback index 5 and exited.
6. The operator pressed `O/o`. The native log reports emergency stop, cleanup,
   and normal process exit. Operator confirmed: **“Stopped; robot supported.”**
7. Post-stop inspection found no SONIC, DimOS service, camera publisher, or
   policy streamer on pc2, and no listeners on ports 5555/5556/5557.

Exact user-pasted output is preserved in:

- Native terminal (`../../../artifacts/g1_pc2_223_incident_20261014/operator-sonic-terminal.log`)
- Planner helper terminal (`../../../artifacts/g1_pc2_223_incident_20261014/operator-helper-terminal.log`)

The native log contains two timing reports with LowState ages around 8–9 ms
and policy inference around 5 ms. Those reports do not establish worst-case
startup timing or motor behavior.

## Installations and exact build identity

| Item | Path on pc2 |
|---|---|
| Original native repository | `/home/unitree/GR00T-WholeBodyControl` |
| Original executable | `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref` |
| New staged native root | `/home/unitree/dimos-sonic-guard-pc2-223-20261014` |
| New executable | `<staged root>/gear_sonic_deploy/target/release/g1_deploy_onnx_ref` |
| Original DimOS | `/home/unitree/dimos` |
| New DimOS copy | `/home/unitree/dimos-g1-service-20261014` |
| Python used by helper | `/home/unitree/dimos/.venv/bin/python` |

New executable SHA-256:
`9c6ee5fbde7fc62121a35d62450919dc87125e727ec243afd179415d4b73cdfa`.

Original executable SHA-256, verified preserved during sync:
`bc0b17c45d06e458b847c217ec62c9e1bbb8d7db66f6688750beceb5c773a3b5`.

Original native source HEAD:
`a9d20b2ac0949244d94461a1a3263f38c5027c4a`, plus pre-existing local changes in
setup_env, state logging, telemetry, and native main. The sync preserved those
changes. The source tree is not proof that the previously working executable
was built from those exact bytes: **binary-to-source equivalence remains an
open check**.

DimOS base HEAD: `7dd83be804e0e9dd2dd6210268250bcabf06617c`, overlaid with 63
local SONIC implementation/test/guide files and a regenerated registry. No
DimOS execution or service blueprint was running during the incident.

The active native launch used:

```text
network interface: enP8p1s0
decoder: <staged root>/gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx
reference directory: <staged root>/gear_sonic_deploy/reference/example/
--obs-config <staged root>/gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml
--encoder-file <staged root>/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx
--planner-file <staged root>/gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx
--input-type zmq_manager --output-type all
--zmq-host 127.0.0.1 --zmq-port 5556
--reference-timeout-ms 200
--disable-dex3-hands
```

111 model/cache/reference files were copied from **pc2 itself**, with hashes
verified; .222 TensorRT engines were not used. The new build reused pc2's
existing SDK libraries via a `thirdparty` symlink. Host inventory: Ubuntu
22.04.5, JetPack **6.2.1**, CUDA 12.6, TensorRT 10.3, Python 3.12.13.

The launcher bypasses the original `deploy.sh`/`just run` wrapper and explicitly
sets DDS/CUDA/TensorRT/ONNX library paths and DDS preloads. Differences in
wrapper environment, compiler options and runtime initialization still need
comparison with the previously smooth deployment.

## Evidence collected and what it does / does not prove

### Planner helper and guard

The helper sends one existing-protocol command:
`{"start": true, "stop": false, "planner": true}`. It opens a ZMQ publisher,
not a DDS motor writer. It sends no pose/token reference stream.

At index 5, telemetry showed:

```text
planner_selected=true, planner_confirmed=true
armed=false, expired=false, rearm_requires_planner=false
accepted_index=-1, applied_index=-1, epoch=0, rejected=0, age_ms=-1
```

This supports that no guarded streamed reference was accepted/applied and no
stream-expiry recovery was active. `planner_confirmed` is computed after policy
command creation. It is **not** a certificate of safe startup targets, torque,
mechanical behavior or exclusive DDS control ownership.

### Post-stop DDS observation

Raw evidence:
[dds-after-stop.json](/docs/artifacts/g1_pc2_223_incident_20261014/dds-after-stop.json).
Probe source reused unchanged:
`docs/artifacts/g1_pc2_222_20261006/demo_dds_ownership_probe.py`.
Despite the old artifact directory name, this invocation ran on **pc2 (.223)**
using `enP8p1s0`. It creates discovery/data readers and no command writers.

Eight-second observation after the operator stopped SONIC:

| Topic | Discovered writer | Samples received |
|---|---|---|
| `rt/lowcmd` | `emergency_stop`, PID 2909, controller host `192.168.123.161` | 0 |
| `rt/arm_sdk` | `python3`, PID 2927, controller host `192.168.123.161` | 0 |
| `rt/lowstate` | One writer | 8320 |
| Dex3 left/right state | Vendor `g1_dex_protocol_v2.0` endpoints | 0 |
| Dex3 left/right command | None discovered | 0 |

Last LowState `mode_machine` was 5. Do not infer a model incompatibility from
that number alone. Discovered Dex3 state endpoints do not mean hands exist;
the operator confirmed no hands and no hand samples arrived.

**This post-stop capture neither proves nor rules out simultaneous publishers
during the incident.** Do not disable the emergency-stop process on the basis
of this endpoint list. No contemporaneous incident DDS recording exists.

### Startup source review: concrete finding, unproven acoustic cause

An independent read-only review by `pc2_startup_incident_review` (gpt-6-astra,
high reasoning), plus main-agent inspection, found an inherited startup
ordering hazard. Paths below are relative to the staged native source folder
`gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`:

1. `src/g1_deploy_onnx_ref.cpp:2271`: load example index 0, frame 0, paused.
2. `include/input_interface/zmq_manager.hpp:578`: set `operator_state.start`
   before waiting for `planner_motion`. The waiting input thread does not
   block the independent control thread.
3. `src/g1_deploy_onnx_ref.cpp:3839`: WAIT_FOR_CONTROL accepts start without
   a planner-readiness gate.
4. `src/g1_deploy_onnx_ref.cpp:758`: paused playback still contributes the
   example frame's joint targets to the policy observations.
5. `src/g1_deploy_onnx_ref.cpp:3961`: `CreatePolicyCommand()` runs **before**
   `CurrentFrameAdvancement()` at line 4061, which selects the planner motion
   around lines 3305–3307. Thus at least the first successful policy command
   can use the example reference before planner takeover.
6. `src/g1_deploy_onnx_ref.cpp:3135`: body target mapping is direct
   `default_angles + action * g1_action_scale`, with normal gains and no
   startup target-step limiter in that path. The command writer runs at 500 Hz.
7. `src/g1_deploy_onnx_ref.cpp:3179`: first planner trajectory is copied directly;
   the eight-frame blending branch applies to subsequent replans.

Potential discontinuities are INIT posture → example-conditioned policy →
planner-conditioned policy. **A motion named “kick” is not proof that a kick
was played**: initial playback is paused at frame 0. Actual posture and policy
outputs are needed to quantify any abrupt target.

Crucially, these paths also exist in the captured original source. The guard
and no-hands patch does not change body gains, joint mapping, startup gating
or the first planner trajectory selection. Since the operator reports smooth
original behavior, do not prematurely label this inherited hazard as the root
cause of the difference.

IDLE `target_height=-1` and `target_vel=-1` are native planner initialization
defaults (`localmotion_kplanner.hpp:347`), not negative motor commands.

### Missing incident measurements

The launcher did not enable `--enable-csv-logs`. Native CSV logging defaults to
false. No new state/action CSVs were found under the staged or original native
deploy trees. The ring buffer ended with the process. We have user-pasted
terminal text and the helper confirmation, not a recorded motor-target/torque
trace or an audio recording. Do not claim a replay or physical reproduction
has passed or failed.

The earlier 18 native tests, 371 DimOS tests and eight helper checks validated
software seams and serialization. They did **not** test the physical startup
behavior now under investigation.

## Local source and artifacts for resuming

- Exact pre-port native snapshot:
  `/tmp/dimos-pc2-223-native-20261014/`.
- Staged native source used for the build:
  `/tmp/dimos-pc2-223-port-20261014/`.
- Persistent source archive if `/tmp` is gone:
  `docs/artifacts/g1_pc2_223_sync_20261014/pc2-native-source.tar.gz`.
- Exact port delta **against the captured dirty source baseline**:
  `docs/artifacts/g1_pc2_223_sync_20261014/pc2-223-native.patch`.
  Use this for attribution; the temporary checkout's Git diff also includes
  pre-existing pc2 local changes and is not solely our patch.
- Baseline hashes/status: `.../native-baseline.json`.
- Final native source hashes: `.../native-source-final.json`.
- Native build/test/assets/verification reports: `.../native-validation/`.
- DimOS test/import results: `.../dimos-validation/`.
- Launcher/helper sources and their hashes are preserved in the sync artifacts.
- Prior setup guide: [pc2-223-sonic-sync.md](/docs/platforms/humanoid/g1/pc2-223-sonic-sync.md).
  Its startup commands are **on hold for this incident**.

No debug code or controller fix was installed during this investigation.
No active tool session or robot test is waiting for the next agent.

## Recommended investigation order

1. Read the operator logs, exact port delta, and DDS evidence above. Establish
   the known-good launch route: model/config, input mode, startup sequence,
   build revision and support/foot-contact condition. Do not ask the operator
   to repeat a question already answered (original smooth; suspected waist).
2. **Compare the actual old and new executables**, not only their source trees.
   Check compiled `kps`, `kds`, `default_angles`, `g1_action_scale`, waist
   mapping/mode and linked libraries. Original executable is 5,588,736 bytes;
   its recorded mtime was `1789535717.5519443`. Its `nm -C` output contains
   `kps`, `kds`, `default_angles`, and `g1_action_scale` symbols, so static
   array extraction is possible without executing it. No numeric ELF-array
   comparison has been completed yet.
3. Compare original `deploy.sh` + `just run` + `scripts/setup_env.sh` with our
   direct launcher: build type/options, environment/library selection, model
   cache paths, clock/performance setup and control options. Shell history
   contains prior SONIC v1.1 `zmq_manager` deployments, including local and
   workstation ZMQ hosts; it does not identify which one was the smooth run.
4. Build an **offline** discriminator for startup ordering using fake robot
   state, fake or offline policy/planner engines, and a memory-only command
   sink. Sweep planner delays; compare baseline and staged command sequences,
   especially waist target steps. Avoid launching the native executable on a
   real DDS interface, including for a purported “test.”
5. Treat competing writers as unresolved unless actual traffic supports it.
   If later authorized hardware commissioning becomes necessary, arrange a
   bounded reader-only capture of lowcmd publication handles and per-joint
   targets alongside lowstate before operator startup. Endpoint discovery
   alone is insufficient. Never ask the operator to continue grinding for data.
6. Only after evidence isolates the cause, prepare a minimal reviewed fix with
   an offline regression test. Keep the preserved original installation intact.
   Any physical validation remains an explicit operator-controlled step.

A previous `nm` probe accidentally matched `weak_ptr` through a loose `k_p`
substring and produced excessive output. Use exact symbol names/regex and save
large disassembly to artifacts rather than flooding the conversation.
