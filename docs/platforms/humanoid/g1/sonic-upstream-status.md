# Upstream G1 SONIC integration status

Checked **2026-10-06, 07:06 UTC** against `dimensionalOS/dimos` GitHub REST API.

The upstream project has a concrete GEAR-SONIC implementation under review. Its three implementation PRs are open and unmerged; the existence of runnable code in those branches does not establish availability in `main`. The current stack is **#4304 → #4305 → #3557**. [Controller PR](https://github.com/dimensionalOS/dimos/pull/3557)

## Issues and implementation PRs

| Item | Exact title | Status at check | Meaning |
| --- | --- | --- | --- |
| [#2453](https://github.com/dimensionalOS/dimos/issues/2453) | Sonic walking implementation/demo for G1 | Open | Tracking issue synced from DIM-1004; no GitHub comments or implementation details. |
| [#2895](https://github.com/dimensionalOS/dimos/issues/2895) | Sonic Policy for G1 | Open | Tracking issue synced from DIM-1189; no GitHub comments or implementation details. |
| [#3591](https://github.com/dimensionalOS/dimos/issues/3591) | Test SONIC on G1 | Open | Requests real-G1 testing and explicitly links #3557. Its older body still calls that PR a draft. |
| [#4304](https://github.com/dimensionalOS/dimos/pull/4304) | Prepare G1 control before activation and preserve command timing | Open, non-draft, unmerged | Layer 1: control preparation, hardware command delivery/fault handling, simulation timing. Targets `main`. |
| [#4305](https://github.com/dimensionalOS/dimos/pull/4305) | Add SONIC v1.1 inference and verified model setup | Open, non-draft, unmerged | Layer 2: planner/encoder/decoder inference, motion references, pinned model downloads and hash verification. Targets #4304's branch. |
| [#3557](https://github.com/dimensionalOS/dimos/pull/3557) | SONIC whole-body controller for the G1 | Open, non-draft, unmerged | Layer 3: `ControlCoordinator` task, `unitree-g1-sonic-wbc` blueprint, Jetson installer and usage docs. Targets #4305's branch. |
| [#3721](https://github.com/dimensionalOS/dimos/pull/3721) | feat(g1): add PICO SONIC WebXR teleoperation | Open, draft, unmerged | Separate headset integration, depending on #3557; its reported end-to-end test is in MuJoCo. |

For all four unmerged PRs above, the pull-request API returned `merged: false` and `merged_at: null`. Their descriptions document proposed functionality and author-reported validation, not released support.

## Review direction and validation limits

Repository member TomCC7 requested changes on August 30, questioning why the integration needed ZMQ and suggesting a direct model port unless compatibility with an external NVIDIA tool was needed. The author replied on September 28 that ZMQ ingestion and decoding had been removed in favor of `ControlCoordinator` and existing DimOS interfaces. This records the design direction; it is not evidence of final approval. [Review](https://github.com/dimensionalOS/dimos/pull/3557#pullrequestreview-5059418887), [author update](https://github.com/dimensionalOS/dimos/pull/3557#issuecomment-5876039666)

TomCC7 also requested downloading the models from Hugging Face instead of storing them in Git LFS. The author reports switching to a pinned `nvidia/GEAR-SONIC` revision with SHA-256 verification. [Review comment](https://github.com/dimensionalOS/dimos/pull/3557#discussion_r3887927212), [author response](https://github.com/dimensionalOS/dimos/pull/3557#discussion_r4125423243)

The August testing issue initially described simulation-only validation and subsequently identified MuJoCo stepping during model loading as a cold-start failure. The current #3557 description reports a successful office-scene navigation simulation but explicitly leaves crawl stability, timed floor transitions and earlier hardware instability unresolved; the PR split added no new physical-robot validation. No public merge date or hardware-readiness commitment was found in the inspected human discussion. [Initial testing request](https://github.com/dimensionalOS/dimos/issues/3591#issuecomment-5365164276), [cold-start report](https://github.com/dimensionalOS/dimos/issues/3591#issuecomment-5403979392), [current PR description](https://github.com/dimensionalOS/dimos/pull/3557)

Some related work has merged: #3721's other prerequisites, [#3436: G1 Quest teleop](https://github.com/dimensionalOS/dimos/pull/3436) and [#3695: PICO body tracking](https://github.com/dimensionalOS/dimos/pull/3695), merged on September 3 and September 16 respectively. [#4306: RPC shell startup compatibility](https://github.com/dimensionalOS/dimos/pull/4306), extracted from #3557, merged on October 5 at 23:48 UTC. None of those merges is the SONIC controller merge.

## Source verification

The independently checked upstream `main` tip was [`4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0`](https://github.com/dimensionalOS/dimos/commit/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0), committed October 6 at 05:34 UTC. Its complete recursive tree (`truncated=false`) has no SONIC-named paths. GitHub code search for `SONIC repo:dimensionalOS/dimos` returned one result, the provenance comment in `g1_29dof.xml` (`/dimos/robot/unitree/g1/assets/g1_29dof.xml`). The existing `unitree_g1_groot_wbc.py` (`/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py#L98`) composes the distinct GR00T balance/walking task and arm tasks. No built-in SONIC blueprint was found on this main snapshot. The user's checkout remains `7529659c88c76eb4800552ad1d4fa5dd584ecc80`; it was not updated.

The reviewed PR head is `1ae85240dc5dee41c32464bf7a6edee19c35dd2f`. Its [controller README](https://github.com/dimensionalOS/dimos/blob/1ae85240dc5dee41c32464bf7a6edee19c35dd2f/dimos/control/tasks/g1_sonic_wbc_task/README.md) identifies SONIC v1.1 as the supported bundle and explicitly says the updated controller has not received another physical activation. The [blueprint](https://github.com/dimensionalOS/dimos/blob/1ae85240dc5dee41c32464bf7a6edee19c35dd2f/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sonic_wbc.py) instantiates DimOS's own SONIC task and `G1WholeBodyConnection` for hardware. This is native inference and body-command ownership, not a client adapter to an already-running `g1_deploy_onnx_ref`. The blueprint explicitly prohibits concurrent ownership with that C++ process.

The [task implementation](https://github.com/dimensionalOS/dimos/blob/1ae85240dc5dee41c32464bf7a6edee19c35dd2f/dimos/control/tasks/g1_sonic_wbc_task/g1_sonic_wbc_task.py) claims all 29 body joints. It exposes velocity, gait, motion-clip, upper-body and wrist/head-reference inputs. `set_upper_body` takes 14 arm joints in DDS order as encoder hints; `set_vr_3point` takes root-relative wrist/head hints and explicitly does not promise servo-accurate end-effector tracking. Inference for the service: the existing direct-arm MuJoCo exercises are not proof of the same tracking or collision behavior through SONIC. The inspected SONIC blueprint has no Dex3 driver, RGB-D module, or manipulation planner composition, so a complete grasp/place service is additional integration work.

PR heads observed for reproducibility:

| PR | Head commit | Base branch / API-reported base commit |
| --- | --- | --- |
| #4304 | `b50b226e3eceb6c4742a66c497f0daf698503f98` | `main` / `576d54783434358be14ed55a25999ce20de0dfef` |
| #4305 | `6d4cfc9fa41142a0c94e5565b125d93aa73105af` | `pim/fix/g1-control-foundation` / `b50b226e3eceb6c4742a66c497f0daf698503f98` |
| #3557 | `1ae85240dc5dee41c32464bf7a6edee19c35dd2f` | `pim/feat/g1-sonic-inference` / `6d4cfc9fa41142a0c94e5565b125d93aa73105af` |

The API-reported PR base SHA is recorded as metadata and is not a claim about the latest `main` tip.

## Direction for the user's robots

The user selected **keep the existing controller and inspect how DimOS can integrate**. The [robot inventory](/docs/platforms/humanoid/g1/pc2-222-inventory-2026-10-06.md#follow-up-publisher-ownership-and-time-sources) captured `g1_deploy_onnx_ref` publishing body and both Dex3 command topics before that process exited. A subsequent read-only inspection identified an installed SONIC v1.1 deployment, its launcher, and its ZeroMQ input/feedback protocol, described below. The earlier PID's exact executable and launch arguments could not be recovered. Do not assume the upstream Python runtime is a drop-in bridge to this deployment.

The passive DimOS feedback adapter has now been implemented and [checked during the user's live SONIC session](/docs/platforms/humanoid/g1/sonic-feedback.md#active-planner-session-2026-10-06). Recommended next work is offline validation of reference encoding and the installed controller's input/lifecycle contract. Preserve the existing controller as the hardware baseline while evaluating the upstream PR stack separately in simulation if pursued. A bridge retaining the C++ controller and a migration to the native DimOS runtime are different implementation choices. Camera acquisition, perception, recording, service orchestration, and read-only feedback can be reused around either route; calibrated geometry, clock alignment, hand ownership, and observed task success still require validation. No PR branch, model bundle, or controller was installed or launched for this review.

## Installed external controller: follow-up trace

Inspected on 2026-10-06. Evidence and saved run settings: [deployment trace JSON](/docs/artifacts/g1_pc2_222_20261006/existing-sonic-deployment.json). These findings identify the installed workflow; they do not establish that the exited PID used a particular binary or model.

The workstation launcher `/home/jihun/work/lerobot/examples/g1_dex3_training/g1_groot_real_run.sh` has a `deploy` branch that connects to `pc2_222`, enters `~/GR00T-WholeBodyControl/gear_sonic_deploy`, sources `scripts/setup_env.sh`, and selects:

```text
--cp policy/sonic_v1_1/model
--obs-config policy/sonic_v1_1/observation_config.yaml
--input-type zmq_manager --zmq-host localhost real
```

This is a record of the launcher's arguments, not an instruction to run it now. The script was only read. On PC2, the encoder, decoder and observation YAML exist under `policy/sonic_v1_1/`; `deploy.sh` expands the model prefix to `model_encoder.onnx` and `model_decoder.onnx`. Its unqualified default is a different prefix, `policy/release/model`, so the explicit v1.1 arguments matter. The PC2 repository is at `087f9ac01d46f6d8e4d0b73c01ae64799f292a38` with local modifications, including controller and telemetry source. A second deployment binary exists in `GR00T-WholeBodyControl-sonic-telemetry-20261001`; the artifact records both hashes. Neither tree was changed.

The task policy and SONIC motion controller are separate layers:

```text
Workstation task-policy server (configured with a Psi0 checkpoint)
  -> PC2 sonic_policy_streamer.py: motion tokens + Dex3 targets
  -> PC2 g1_deploy_onnx_ref: SONIC motion inference and DDS commands
  -> G1 body and both hands
```

The workstation process listing showed `sonic_policy_server.py --port 5560 --noise-scale 0` with checkpoint path `/mnt/data/jihun/g1_models/he_psi0_sonic78sonicstate_ho5_matched640k/pretrained_model`. That is observed launch configuration, not an independent check of loaded weights. Despite its name, the launcher allows a model override; its default GR00T checkpoint is not evidence that the observed server uses GR00T.

Saved PC2 runs `run22_he_psi0_laptop_t0.npz` and `run23_he_psi0_laptop_t0.npz` record task `close a laptop g1`, server `tcp://192.168.0.62:5560`, `action_space=sonic78`, robot state input, and planner-to-pose-to-planner operation. Each contains 1,865 recorded rows with 64 motion-token values, 14 hand targets, and 28 policy-state values per row. The run23 text log reports streamed-token mode and return to planner mode. These records do not prove task success or identify the historical server's loaded checkpoint. They are evidence for the existing laptop workflow, not a trained object-to-tray service.

### Interface available to a DimOS bridge

Source inspected on PC2: `/home/unitree/g1_sonic_eval/code/sonic_policy_streamer.py` and `/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp`.

| Channel | Observed source contract | Integration implication |
| --- | --- | --- |
| Feedback, port 5557 | Streamer subscribes to `g1_debug` with a msgpack payload. Fields include 29 body positions and 7 positions per hand. | A passive DimOS subscriber can consume full body/hand state. The streamer's 28-D policy observation uses only body indices 15–28 plus both hands; retain all 29 body joints for robot modeling. |
| References, port 5556 | Streamer binds a PUB socket; C++ connects as subscriber. `pose` protocol v4 carries `token_state[1,64]`, `frame_index[1]`, and left/right hand targets `[1,7]`. | A DimOS integration must cooperate with or replace this reference publisher, preserving a single selected command source. This is not a direct joint-trajectory execution API. |
| Mode requests, same port | `command` protocol v1 includes start/stop/planner; C++ also consumes the `planner` topic for movement and optional upper-body/hand references. | Model mode transitions, cancellation and handoff explicitly; do not equate a process stop with a verified balanced hold. |

The reference protocol uses a topic prefix, a 1,280-byte padded JSON header and typed little-endian payload, not a plain JSON command socket. The installed streamer schedules publication at 20 ms intervals and resamples policy chunks; the saved runs specify a 30 Hz policy time base. Joint order, hand conversion (`dex3_right_order=dataset` in these runs), freshness, timeout and recovery behavior still need validation before command integration. Source inspection alone does not verify those behaviors on hardware.

The architecture matches NVIDIA's documented [VLA inference flow](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/vla_inference.html), including 64 SONIC token values plus 7+7 hand targets. The installed source and recorded arguments are the baseline for this robot; current upstream examples may differ.

At the start of this follow-up inspection, PC2 had no matching controller or streamer process; the workstation policy-server process was present when checked separately. Subsequently, after user confirmation, a [read-only telemetry module](/docs/platforms/humanoid/g1/sonic-feedback.md) was implemented locally and tested with synthetic ZeroMQ messages. Its first live check received no telemetry. After the user started SONIC in reported planner mode, the adapter received real packets and an independent DDS comparison found close position agreement in the observed poses.

The [active-session evidence](/docs/artifacts/g1_pc2_222_20261006/sonic-planner-validation-summary.json) identifies PID `25395`, its `/proc` executable path and SHA-256, and SONIC v1.1 launch arguments. This session uses **`--input-type keyboard --output-type all`**, unlike the earlier `zmq_manager` VLA launcher. Telemetry integration works with this session; a reference bridge must account for that input-mode difference. No robot process or mode was changed by the probes. Offline/simulation command validation, anatomical mapping and lifecycle commissioning remain open. The exact executable/arguments of the earlier exited PID `39396` remain unresolved.

## Search scope

GitHub issue/PR searches covered all states for `repo:dimensionalOS/dimos sonic` (9 results), `repo:dimensionalOS/dimos "GEAR-SONIC"` (2), and `repo:dimensionalOS/dimos sonic in:comments` (5), each with `per_page=100` and `incomplete_results=false`. One result concerned unrelated Cartesia Sonic speech. Relevant issue bodies, #3591 comments, #3557 comments/reviews/inline discussion, other implementation PR comments, and explicit PR merge metadata were inspected. Public web search supplemented discovery. Private Linear records, unreferenced branches, and unindexed terminology were not exhaustively searched; search absence alone does not prove implementation absence. No robot or policy execution was performed.
