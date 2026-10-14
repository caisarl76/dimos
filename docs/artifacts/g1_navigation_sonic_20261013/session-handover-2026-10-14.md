# G1 SONIC navigation session handover: 2026-10-14

## Resume here

**Tasks 1–7 are complete. Task 8 is next.** The corrected local/headless motion path passes its original movement and stopping gates. The exact legacy FK recording is still unavailable, so the full native suite and hardware commissioning remain blocked. Continue the remaining software work with that gate visible.

This handover captures implementation/evidence checkpoint **`c634967a2`** on **`codex/feat/g1-sonic-navigation`**. A subsequent documentation commit adds this handover. Plan checkboxes and older ledger entries retain historical states; use the latest verification records below for current status.

The original session was `01a11f87-0f25-7182-a8a2-b9f2fd212d59`, interrupted by remote compaction/stream failure (request ID `5820dace-794a-4079-b6e2-09fb489d5ae8`). This session recovered its local logs/worktree, completed Task 6, implemented Task 7, investigated the exact FK resource in official sources, and corrected/retested the earlier headless motion failures. Recovery is complete; restarting Tasks 1–7 is unnecessary.

1. Work in the existing navigation worktree shown below. Read the approved [plan](/docs/superpowers/plans/2026-10-13-g1-sonic-navigation.md) and [normative design](/docs/superpowers/specs/2026-10-13-g1-sonic-navigation-design.md), concentrating on Task 8 and topology/admission constraints.
2. Inspect Git status and verify original-source hashes against `sonic-baseline-import.json` and `baseline-native-source.json`. A changed upstream baseline requires a separately labelled import/rebase at the task boundary, preserving navigation changes and rerunning affected checks.
3. Read Task 7's [verification](/docs/artifacts/g1_navigation_sonic_20261013/navigation-link-verification.md) and the [motion gate report](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/README.md). Implement Task 8 with failing tests first; recompute its starting commit from current HEAD.
4. Continue Tasks 9–10 after their preceding software gates pass. Task 11 requires fresh applicable operator/robot readiness and release of the robot by the other session.

## Workspace and preservation boundaries

| Item | Location/status |
| --- | --- |
| Active worktree | `/tmp/dimos-g1-sonic-navigation-20261013` |
| Branch | `codex/feat/g1-sonic-navigation` |
| Original DimOS workspace | `/home/jihun/work/dimos_dev`; preserve the other manipulation session's work |
| Existing official/local GR00T checkout | `/home/jihun/work/GR00T-WholeBodyControl`; use read-only |
| Private native deployment root | Worktree `build/sonic-navigation-runtime/source/gear_sonic_deploy` |
| Private CMake build | Worktree `build/sonic-navigation-runtime/cmake` |
| Original native application source | `/home/jihun/work/dimos_dev/build/sonic-grasp-runtime/source/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref`; preserve |
| Shared Python environment | Worktree `.venv` symlinks to `/home/jihun/work/dimos_dev/.venv`; use existing packages without install/sync |
| Intended remote target | SSH alias `pc2`, `192.168.0.223` |

At handover the navigation worktree has **no uncommitted tracked changes**. It also contains intentionally preserved untracked raw trial traces, private build products, and the `.venv` symlink. Keep this worktree and its artifacts; a Git-only copy does not contain every raw trace or built binary. Detailed `horizon/coverage-all.json` and several earlier archives use Git LFS.

The original workspace still has the same four tracked modifications: `dimos/robot/all_blueprints.py`, `docs/platforms/humanoid/g1/index.md`, `pyproject.toml`, and `uv.lock`, plus the other session's untracked work. This session did not edit them. Last integrity checks matched **20/20 original imported Python files and 51/51 original native source files**. The imported code is explicitly labelled as the manipulation session's initially unreviewed baseline, with three separately recorded updates; inspect `baseline-update3-evidence.json` when resolving overlapping service/observation changes.

Follow repository AGENTS.md model routing and native Codex workflows. Main owns diagnosis/integration/review; this session used fresh `gpt-6-astra`/high children for difficult diagnosis and reviews. The requested `gpt-5.6-luna` worker model was unavailable in this tool's model list. No OMX workflow was used. CodeGraph was not initialized for this worktree; the original checkout's index does not represent this branch.

## Completed work

| Task | Implemented behavior | Current verdict |
| --- | --- | --- |
| 1 | Instance-scoped follower angular minimum; SONIC follower speed/turn envelope with `simulation=""` | Complete |
| 2 | Planner codec, immutable held intent, frame conversion and coherent native guard feedback | Complete |
| 3 | Private opt-in native planner lease/parser/receiver, sequence barriers, watchdog and telemetry | Implementation complete; exact legacy FK full-suite gate remains open |
| 4 | Serialized planner/pose ownership in `SonicExecutionModule`, explicit begin/end, bounded peer cancellation and measured settling | Complete |
| 5 | Local native MuJoCo path, fixed/live translating noise attribution, expiry/watchdog and pose handoff proofs | Complete host-only; later motion corrections verified below |
| 6 | Seven-input progress/readiness monitor and pinned native PointLIO source-stamp replay | Complete; calibration/full spatial replay still separate |
| 7 | Bounded private navigation link, off-board peer and execution-host relay | Complete, reviewed and committed |

Task 6 uses monotonic receipt ages, advancing original source stamps, rollback latches and same-domain ancestor lag. Budgets are LiDAR/LIO/odom **350 ms**, IMU/dynamic base TF **200 ms**, map/costmap **1 s**. Duplicates cannot refresh progress. Native replay produced 96 cloud/odom/TF/map outputs and 83 costmaps; idle, duplicate and IMU-only quiet windows produced no extra outputs. This replay repeated **one saved cloud at 100 recorded stamps with 2,000 recorded IMUs**; it is a source-clock/progress proof, not a full scan-series spatial replay or base-extrinsic calibration. See [sensor-readiness audit](/docs/artifacts/g1_navigation_sonic_20261013/sensor-readiness-audit.md). All 1,232 stored pc2 environment fingerprints matched after that replay.

Task 7 files are `sonic_navigation_link.py`, `sonic_navigation_peer_module.py`, `sonic_navigation_relay_module.py` and their tests under `dimos/robot/unitree/g1/`. The relay exposes the existing `SonicNavigationControlSpec`; the peer injects public `ReplanningAStarPlannerSpec` methods.

- Version 1 JSON over ROUTER/DEALER: exact schema, one part, maximum 64 KiB, explicit endpoints, independent one-pending poll/control slots. Polling is 10 Hz; native heartbeat remains independently 50 Hz.
- Original velocity emission sequence/age is retained. Deadline is execution-host `t0 + 0.350 - source_age`; readiness uses `min(t0 + budget - input_age)`. Readiness/settling evidence requires RTT at most 100 ms; control transport has a separate 500 ms bound. Absolute clocks are never compared across hosts.
- Healthy sensor-only startup creates sequence-zero, zero-valued intent with expired velocity deadline. It permits the explicit IDLE handshake. Source-barrier equality refreshes readiness only; motion requires a newer real emission.
- Goal/cancel generation fences discard delayed transition polls. Missing goal replies close admission. Cancellation publishes closed admission before public `cancel_goal()` and fresh `get_state()==IDLE` acknowledgment.
- A new session ending before its first goal can bind cancellation through optional `previous_request_id`, only after the previous owner was confirmed cancelled/IDLE. Preserve that binding through lost requests; retired IDs cannot reacquire ownership.
- Settling is measured from advancing odometry pose differences, never a default zero Twist. Source rollback, frozen inputs and unexpected peer ownership changes close admission.

## Motion failures: correction and evidence

Historical `on-host-sim/run02` commanded +0.2 m/s for two seconds but moved -0.03797 m; `run08` missed the original stopping gate. Their failure verdicts and traces remain intact. Run02 did not record enough controller/contact state for exact historical closed-loop reproduction.

Two native reference defects/confounds were addressed without changing motion limits or acceptance windows:

1. **Terminal reference consistency:** guarded resampling now gives the held final joint pose zero joint velocity; genuine moving samples and legacy behavior remain intact. Three native regressions pass. Matched original/terminal-only A/B still reproduced a stopping failure, so this alone was insufficient.
2. **Physics pacing:** absolute 5 ms step deadlines remove accumulated sleep drift; step size and wall-clock acceptance windows remain unchanged. Separate absolute/relative comparisons still had direction failures. Diagnostic runner exit-zero had hidden false case flags; it now returns failure if any original diagnostic case fails, with eight preserved-case regressions.
3. **Future-window maintenance:** the encoder needs 46 frames, but generated trajectories can leave only 62 while legacy walking refresh runs at one second. Guarded active SLOW_WALK now refreshes at `remaining <= enabled_window + ceil(planner_dt*50) + lookahead + 1` (currently 54 frames). IDLE/pose/paused/legacy paths are excluded. Horizon refresh has its own CSV reason; actual source changes retain their independent attribution. Six native regressions pass.

The corrected configuration passed **nine two-second forward/stop cycles in three fresh processes, plus three fresh full 26-case matrices: 99 checks**. Forward displacement was 0.08794–0.30117 m; longest measured settling was 2.66823 s. Original thresholds remain **0.03 m/s, 0.03 rad/s, continuous 0.5 s dwell, 3 s timeout**. Matrices include backward/turn directions, translating noise, source expiry, native/publisher watchdogs, feedback loss and pose handoff.

The independent audit matches all 36 reference values against actual blank-delimited native CSV buffers. It reproduces the earlier failed matrix's 12 windows with at least three terminal samples (maximum four). All six corrected runs have **18 translating windows / 2,132 eligible frames / zero terminal or clamped samples**, without unmatched or ambiguous post-transition samples. It rejects incomplete traces and excessive exclusions; recorded transition exclusions are 0.0549–0.2392 s, bounded by 350 ms, and target gaps are bounded by 120 ms (observed maximum 20.1 ms). Seven audit regressions pass. See [coverage report](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon/coverage.md).

All **21 comparison trials** are retained. Nine earlier processes are unaccepted because of physical failures or incomplete proof; one physically passing earlier matrix has an unmatched native ACK in the noise audit. Raw hashes, verdicts and source/binary manifests are in `motion-resolution/trial-summary.json` and its four trial directories. Later passes did not overwrite earlier failures. Headless regression gates are closed for this configuration; this is not real-robot commissioning or a statistical reliability claim.

Private native binary SHA256:

```text
edbb873f166009e567d4d546fee32e4a9b54bed2251abc22377038cbf01a406b
```

The native suite is **41 passed / 1 failed**, with only unchanged `FK.TestFKAndGlobalVelocities` failing. The preserved original comparison binary is `ce396b2b...`; terminal-only is `3325d6dc...`. Full hashes/source attestations are in `motion-resolution/horizon/provenance.json` and `ab-binaries.json`.

The private source contains four patch stages: `native-navigation.patch`, `native-navigation-diagnostics.patch`, `motion-resolution/native-terminal-hold.patch`, then `motion-resolution/native-horizon-refresh.patch`. Each patch's base directory differs as documented; follow its replay instructions. Both supplemental patches replay byte-identically. `prepare_g1_sonic_navigation_runtime.py` snapshots a **new** destination and accepts only one optional patch; it does not automatically assemble this full stack. Prefer verifying/reusing the existing private build. Preserve archived patch context whitespace for byte-identical replay; repository whitespace hooks explicitly exclude `.patch` files.

## FK resource blocker

The unchanged test requires the authentic **`bones_072925_test.pkl`** or exact exported **`reference/bones_072925_test/`**, plus matching **`g1/g1_29dof.xml`**. Both relative paths are absent in the private test runtime. The XML exists in the official local checkout, but staging it alone leaves an unchecked empty-motion access; restore the recording and matching XML together.

Checked local names/history, eight official branch heads, official documentation/issues and published model/dataset manifests. No exact fixture payload, checksum or intended clip was located. The general BONES archive was not downloaded/exhaustively inspected, so this is not proof that no public copy exists. An earlier attempted example substitute produced 396 failures in both original/private binaries; it is not equivalent coverage. Preserve FK assertions, tolerances and failure status.

Official current main `45c03facbf848e0c743091b7d9c17952096ff908` contains the [motion exporter](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/45c03facbf848e0c743091b7d9c17952096ff908/gear_sonic/data_process/export_motion_to_deploy.py), recommended by a [collaborator](https://github.com/NVlabs/GR00T-WholeBodyControl/issues/241#issuecomment-5957798919). It enables new training-motion exports, but the legacy recording identity/XML/export contract remains unresolved. It was fetched for inspection only, not installed into either shared source tree.

Publisher clarification should request the exact pickle/CSV tree, intended first clip/order, dataset/export revision and matching XML hash for `FK.TestFKAndGlobalVelocities`. NVIDIA lists **gear-wbc@nvidia.com** on its [support page](https://nvlabs.github.io/GR00T-WholeBodyControl/resources/support.html). No external message has been sent; contacting others needs explicit user instruction. Read [FK investigation](/docs/artifacts/g1_navigation_sonic_20261013/fk-resolution.md) before further resource work. Once authentic inputs arrive, stage them privately, verify provenance and rerun the unchanged FK test and full suite.

## Remaining jobs

| Task | Required work and completion boundary |
| --- | --- |
| **8: next** | Implement read-only DDS sensor relay, strict role/endpoint graph validation and two independent blueprints. Verify forbidden owners/modules rejected before worker/socket/control startup; conversion/source-progress/reset tests pass. Regenerate blueprint registry through its test. |
| 9 | Extend local harness to full two-host/link simulation with recorded sensors and real graph wiring. Verify delay/drop/stalled RPC, per-stage freeze, restart/recovery, unknown/occupied footprint, VLA load and phase handback. Commit complete measured proof. |
| 10 | Assemble isolated ARM64 pc2 package, matched source/model/config/TensorRT manifests and passive `--check-only` readiness. Recheck 1,232 fingerprints, ownership and unchanged environments. Produce reviewable launch/stop package. |
| 11 | Fresh native-SONIC operator/robot readiness, tested stop and exclusive controller ownership; supported commissioning and bounded real cases, then calibrated short goal and final report. Exact FK and calibration gates must close first. |

Task 8 creates `sonic_navigation_sensor_module.py`, `sonic_navigation_graph.py`, their tests and peer/pc2 blueprints under `dimos/robot/unitree/g1/blueprints/navigation/`. Relay clouds separately from IMU/state/TF using unicast PUB/SUB, two-part allowlisted channel + existing LCM payload, maximum 16 MiB, HWM 2 and bounded per-channel drain. Promote preserved timestamps, Livox offsets, g-to-m/s² conversion and read-only state/waist TF. Require calibrated extrinsics. Use the normative plan for exact graph allowlists and remapping validation.

**Task 8 source-location gotcha:** its proven adapter is absent from this worktree but exists at `/home/jihun/work/dimos_dev/docs/artifacts/g1_navigation_20261012/demo_dds_lio_adapter.py`, SHA256 `e09185b79e7472179d9b5bcde159f5456f90f3f49b573c57dbe5caa1a007a5ae` at handover. Read that original reference without modifying it; capture provenance if copying reference bytes into the worktree. The original historical navigation report is likewise in the original workspace until Task 9 imports/updates it appropriately.

One motor controller and one execution publisher remain mandatory. Peer consumes off-board `cmd_vel`; pc2 produces/consumes only atomic `NavigationInput`. Planner and pose admission are mutually exclusive. Source expiry 350 ms is recoverable IDLE; continuous outage 2 s ends/cancels navigation; native publisher watchdog 250 ms and feedback timeout 200 ms latch. Pose lease remains 200 ms. Preserve occupied/unknown map cells, set initial safe radius to zero and disable merged-map input. Earlier stock SDK/remote/operator readiness is not authorization for a new native walking launch.

## Verification and operating notes

Recorded results: **504 compatibility tests** and **126 integrated Task 7 tests** (overlapping), **34 final artifact tests**, two timing checks, and native **41/42** with the FK dependency failure. Mypy passed four affected production files; required hooks passed. Later-added artifact tests mean a fresh broad collection can exceed the earlier 504 count.

From the navigation worktree, the focused commands are:

```bash
.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation_link.py dimos/robot/unitree/g1/test_sonic_navigation_peer_module.py dimos/robot/unitree/g1/test_sonic_navigation_session.py dimos/robot/unitree/g1/test_sonic_execution_module.py dimos/robot/unitree/g1/test_sonic_navigation_readiness.py -q
.venv/bin/python -m pytest docs/artifacts/g1_navigation_sonic_20261013/test_navigation_noise.py docs/artifacts/g1_navigation_sonic_20261013/test_navigation_trial_verdict.py docs/artifacts/g1_navigation_sonic_20261013/test_navigation_horizon_proof.py docs/artifacts/g1_navigation_sonic_20261013/test_navigation_stamp_replay.py -q
python3 docs/artifacts/g1_navigation_sonic_20261013/demo_navigation_timing_checks.py
.venv/bin/pre-commit run
```

Sandboxed pytest may fail because `pytest_rerunfailures` opens a local socket; use the normal approved escalation when that happens. Linked-worktree Git index writes also require escalation. Activate the existing venv before committing and run hooks explicitly; commit hooks were not automatically installed. Stage task-owned files selectively. No push occurred. Existing simulations used loopback-only isolated namespaces, GPU 1 and bounded process timeouts; all this session's simulation jobs finished. Keep unrelated jobs intact and inspect current ownership/load before any new launch.

The execution ledger `.superpowers/sdd/2026-10-13-g1-sonic-navigation/progress.md` persists locally but is ignored/untracked. The committed reports are the portable status source. Temporary `/tmp/run_g1_motion_*_20261014.sh` and `/tmp/verify_g1_navigation_task7.py` are convenience scripts, not the sole source of reproducibility; maintained harnesses, patch replays, manifests and trial hashes are versioned.

Key continuation commits:

| Commit | Change |
| --- | --- |
| `bed1be536`, `cd66c16fe` | Task 6 monitor, replay and original-stamp proof |
| `6d73ddde2` | Task 7 bounded link/peer/relay and reviewed regressions |
| `8b6f0a6ce` | Official FK resource investigation |
| `ee9ee8222` | Native terminal/horizon corrections, pacing/verdict fixes and all trial evidence |
| `c634967a2` | Final headless gate status and verification hashes |

### Suggested prompt for the new session

> Continue G1 SONIC navigation from `/tmp/dimos-g1-sonic-navigation-20261013` on `codex/feat/g1-sonic-navigation`. Read `docs/artifacts/g1_navigation_sonic_20261013/session-handover-2026-10-14.md` and the approved plan/spec first. Tasks 1–7 and corrected headless motion gates are complete; start Task 8. Preserve original workspaces/environments and the exact FK failure status. Carry the remaining software tasks forward; real native robot launch requires fresh applicable readiness/authorization.
