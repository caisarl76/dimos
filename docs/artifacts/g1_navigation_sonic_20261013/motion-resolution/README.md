# Guarded planner terminal reference correction

The terminal-reference inconsistency is corrected and CPU-tested. It was insufficient on its own: matched A/B trials still reproduced stopping failures, and a subsequent full matrix reproduced reverse displacement. A separately tested guarded horizon correction now passes the original measured windows across nine forward/stop cycles and three full 26-case matrices. The independent realized-window audit is recorded separately under `horizon/`. The exact legacy FK fixture remains unavailable, so hardware commissioning is still blocked.

The opt-in `--guarded-planner` path now sets `PlannerConfig.zero_terminal_velocity`. Native resampling sets the last generated planner frame's joint velocities to zero. Positions and all genuine pre-terminal velocities are untouched. The full-body, lower-body and encoder-parity paths share this terminal velocity buffer when padding their future reference windows, so no observation-gather copy or special pose logic was introduced. Legacy configuration defaults to the old behavior. Pose reference producers/merging, planner leases, source deadlines, frozen facing, FK assertions, original source, assets, and simulator pacing are unchanged.

The terminal patch changes three private source files (all within the expanded worker allowlist):

- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/localmotion_kplanner.hpp`
- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`
- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/unit_tests/test_planner_terminal_hold.cpp`

No private CMake or additional header modification was necessary for that patch. The existing test-source glob requires a configure step to discover the new test. The initial bounded implementation had no allowlist deviations or controller launches. The existing CPU receiver test used a loopback-only ephemeral socket after sandbox rejection; its unsandboxed rerun was approved. Main subsequently ran the recorded headless comparisons in namespaces containing only loopback; no real robot was controlled.

## Evidence

- `baseline.json`: SHA256 of both existing source files before the correction.
- `terminal-red.log`: true numerical failure with the old resampler (an inert config field was declared to compile the requested opt-in API). 1 failed / 2 passed. The terminal-buffer invariant returned nonzero velocities as expected.
- `terminal-green.log`: all 3 new CPU regressions pass after the correction. They exercise the real `LocalMotionPlannerBase` resampler using a deterministic backend-output fixture, verify the shared terminal buffer for both encoder index sets, preserve genuine moving samples/terminal positions, and preserve legacy terminal velocity. They do not instantiate a native controller or prove policy/physics behavior.
- `native-suite.log`: initial sandboxed full suite attempt; the existing ZMQ receiver test aborts because socket permission is denied (exit 134), and FK reports missing assets before that.
- `native-suite-unsandboxed.log`: completed full CPU suite, **35 passed / 1 failed**, exit 1. The only failure is the unchanged `FK.TestFKAndGlobalVelocities`, which requires missing `reference/bones_072925_test/` and `g1/g1_29dof.xml`. All planner lease, reference lease/validation, receiver and VR safety tests pass. This is not a green full suite or an FK waiver.
- `build.log`: successful private `run_tests` and `g1_deploy_onnx_ref` build.
- `native-terminal-hold.patch`: supplemental patch containing the exact three-file correction and regression.
- `patch-replay.log`: applied the supplemental patch to copies of the saved pre-fix source; all three replayed files byte-match the compiled files.
- `provenance.json`: source/patch/binary SHA256, command results and limits.

The rebuilt native SHA256 is `3325d6dcfb3f77a6f2fd45c2840826b7a53ee09018b42e1aba152ce7d03d2a76`; rebuilt CPU test SHA256 is `d0091b8629ecfe377dfa2f4a5a45f9e7b9a8cf1c38c69fbe553bc9a8ee628213`. The prior FK attestation remains tied to its older binary hash and has not been overwritten.

## Replay

Apply on a **private** source snapshot already carrying the existing navigation and diagnostics patches (`../native-navigation.patch`, `../native-navigation-diagnostics.patch`) and the documented grasp/reference baseline. Verify the two input source hashes against `baseline.json`; this supplemental patch is not a replacement for those earlier patches.

From the private source root (the directory containing `gear_sonic_deploy/`):

```bash
patch --dry-run -p1 < /absolute/path/to/motion-resolution/native-terminal-hold.patch
patch -p1 < /absolute/path/to/motion-resolution/native-terminal-hold.patch
```

Then reuse the existing private CMake configuration and installed dependencies; from the isolated DimOS worktree:

```bash
cmake -S build/sonic-navigation-runtime/source/gear_sonic_deploy -B build/sonic-navigation-runtime/cmake
cmake --build build/sonic-navigation-runtime/cmake --target run_tests g1_deploy_onnx_ref -j2
build/sonic-navigation-runtime/source/gear_sonic_deploy/target/release/run_tests --gtest_filter='PlannerTerminalHold.*'
build/sonic-navigation-runtime/source/gear_sonic_deploy/target/release/run_tests
```

No original fixture, skip, assertion change, or substitute motion was used to hide the FK failure. No acceptance window or threshold changed. Native A/B trials must preserve all original cases and separately control the simulation pacing variable before claiming either old physical failure resolved.

## Controlled follow-up

All 21 trials are retained with raw hashes in `trial-summary.json` and the four trial directories. Several early diagnostic processes exited zero despite false movement-case verdicts. They are explicitly scored as failures in the aggregate; the runner now returns failure when any original diagnostic case fails. Eight regressions against preserved trial cases prevent this reporting error from recurring.

1. `ab-trials/`: exact original binary `ce396b2b...` versus terminal-only `3325d6dc...`, unchanged relative physics pacing, three fresh processes each. Both variants reproduced stopping failure. The terminal fix is a demonstrated reference-consistency repair, not a sufficient physical cure.
2. `pacing-trials/`: the same terminal-only binary, assets, harness, worker and production source hashes for three absolute and three relative pacing runs. Absolute pacing removes accumulated scheduler sleep drift while retaining the 5 ms physics step. All stop cases pass, but five processes contain a failed direction case; pacing alone does not clear the motion gate.
3. `fullmatrix-trials/`: terminal-only plus absolute pacing, three fresh processes. One initial forward case produces -0.003492 m under the original two-second criterion; the other two complete all 26 physical cases. One of those two has an unacknowledged native replan in the noise audit and remains unaccepted as a complete proof. The complete failed trace confirms correct positive intent and identifies premature terminal samples in the encoder's future window.
4. `horizon-trials/`: terminal consistency plus absolute pacing and guarded horizon maintenance, three fresh repeated-transition processes and three full matrices. Every original case passes: **99 checks**, with positive forward displacements 0.08794–0.30117 m and maximum measured settling time 2.66823 s. The criteria remain 0.03 m/s, 0.03 rad/s, 0.5 s continuous dwell, and a 3 s timeout. Full matrices also verify backward/turn directions, translating noise/source attribution, source expiry, native watchdogs, publisher loss and pose handoff.

`native-horizon-refresh.patch` and [horizon/README.md](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon/README.md) describe the additional four-file correction. The enabled decoder needs 46 future-window frames, while generated motions can leave only 62 frames and the legacy walking refresh interval is one second. Guarded active translating playback now refreshes before remaining frames reach the enabled window plus eight scheduling-reserve frames. IDLE, pose, paused and legacy paths are excluded. Horizon maintenance has its own recorded reason; real intent changes still require their original wire/source attribution. Six native decision regressions and eight attribution regressions cover these distinctions.

The final native binary is `edbb873f166009e567d4d546fee32e4a9b54bed2251abc22377038cbf01a406b`. Its full CPU suite is **41 passed / 1 unchanged FK fixture failure**. Both supplemental patches replay byte-identically. All 51 original native source hashes and 20 upstream Python hashes remain unchanged. Historical failing traces retain their failure verdicts; exact reconstruction of run02's unrecorded controller/contact state is unavailable. The new evidence establishes the behavior of the corrected configuration under these headless trials, not a statistical reliability claim or real-robot commissioning.
