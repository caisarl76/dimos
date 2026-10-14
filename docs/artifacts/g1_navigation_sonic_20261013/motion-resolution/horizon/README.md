# Guarded planner horizon refresh

The enabled encoder requests ten samples at stride five: 46 control frames including the current frame. Native SLOW_WALK previously replanned periodically at 1 s, whereas a merged trajectory can contain only 62 frames. Its future observation window can therefore reach the final held pose before replanning. In corrected/absolute fullmatrix-1, twelve sampled windows contained at least three terminal repeats; passing fullmatrix-2 and -3 had zero and five such windows. This confirms missing coverage, not physical causation of the failed displacement. Other passing trials also had padding.

The private correction adds an independently reported horizon maintenance reason. It applies only with guarded planner enabled, active planner motion, playback, SLOW_WALK, positive speed, and a nonzero enabled observation window. Negative direction vectors remain eligible; speed is a magnitude. Remaining frames include the current frame. Refresh occurs at `remaining <= window + ceil(planner_dt * 50) + configured_lookahead + 1`. Current settings give window 46, margin 8, threshold 54. The same tested helper runs in production.

The existing motion mutex protects a coherent motion identity/play/frame/window snapshot and is released before inference or planner mutex acquisition. Horizon-triggered inference uses that captured frame. Other planning calls retain their prior frame behavior. This change does not renew leases, alter facing, change terminal references, or change IDLE/pose planning decisions. CSV appends `horizon_refresh,horizon_remaining,horizon_window,horizon_margin`; existing source and periodic reasons remain independently truthful, including mixed reasons.

The margin reserves one polling period, generation lookahead, and one apply tick. Observed model times of approximately 4.6–13.6 ms plus a 20 ms apply tick fit the reserve, but scheduling is not bounded by this change. Native simulation must independently verify realized future-window coverage. Cases with new motion transition latency should be distinguished from steady translating maintenance, and IDLE terminal holds remain expected.

## Verification

- `red.log`: two intended numerical/decision failures before implementation; four exclusion tests passed.
- `green-cpu.log`: 32/32 horizon, terminal, planner lease, and pose validation tests passed (the loopback receiver test runs in the full suite).
- `native-suite.log`: 41/42 passed, including the existing loopback receiver and VR safety tests. The unchanged `FK.TestFKAndGlobalVelocities` fails because `reference/bones_072925_test/` and `g1/g1_29dof.xml` are absent.
- `build-final.log`: `run_tests` and `g1_deploy_onnx_ref` built with `-j2`, exit 0.
- `provenance.json`: four changed/new file hashes, before bytes/hashes, binary hashes, preserved terminal-only comparison binary hash, and all 51 original native source hashes unchanged. No file allowlist deviations.
- `patch-replay.log`: supplemental patch applied to saved pre-change files in `/tmp/nav-horizon-patch-replay`; all four output files byte-match the compiled sources.

## Replay

Apply `../native-horizon-refresh.patch` after the earlier native navigation/diagnostics and terminal-hold patches, from the directory containing `gear_sonic_deploy`:

```sh
patch -p1 --batch -i /absolute/path/native-horizon-refresh.patch
cmake -S gear_sonic_deploy -B /path/to/private-cmake
cmake --build /path/to/private-cmake --target run_tests g1_deploy_onnx_ref -j2
```

CMake reconfiguration includes the new test through the existing source glob; no CMake file changed. `before/` preserves the exact two pre-change private source files. The saved terminal-only binary `build/sonic-navigation-runtime/motion-ab/corrected-g1_deploy_onnx_ref` remains SHA256 `3325d6dcfb3f77a6f2fd45c2840826b7a53ee09018b42e1aba152ce7d03d2a76` for one-variable A/B trials.

The bounded CPU implementation ended with physical gates unresolved. Main subsequently ran six fresh isolated headless processes: nine forward/stop cycles and three full 26-case matrices pass under the unchanged 2 s command/sign and 3 s measured-settling requirements. [Coverage](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon/coverage.md) independently audits realized reference windows; [the parent report](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/README.md) retains all failed and passing comparisons. The original FK fixture and later hardware/calibration checks still block commissioning. No real robot was controlled.
