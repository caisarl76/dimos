# Independent horizon coverage proof

The old failed fullmatrix-1 reproduces **12 windows with at least three terminal samples**, maximum four. All six new runs have **zero terminal or strictly clamped samples across 2,132 eligible recorded target frames in 18 translating windows**. Each new window has zero unmatched, ambiguous, contradictory, or unhealthy post-transition samples. Physical case verdicts remain separate fields.

| Run | Translating windows | Eligible frames | Terminal windows | Max terminal / clamped slots |
|---|---:|---:|---:|---:|
| fullmatrix-trials/fullmatrix-1 | 1 | 90 | 32 | 4 / 4 |
| horizon-trials/transitions-1 | 3 | 279 | 0 | 0 / 0 |
| horizon-trials/transitions-2 | 3 | 273 | 0 | 0 / 0 |
| horizon-trials/transitions-3 | 3 | 276 | 0 | 0 / 0 |
| horizon-trials/fullmatrix-1 | 3 | 431 | 0 | 0 / 0 |
| horizon-trials/fullmatrix-2 | 3 | 437 | 0 | 0 / 0 |
| horizon-trials/fullmatrix-3 | 3 | 436 | 0 | 0 / 0 |

The new windows comprise nine repeated forwards, three matrix forwards, three backwards, and three noisy translating traces. Initial transition exclusions range from 0.0549 to 0.2392 s and are individually recorded; old failed forward excludes 0.199397 s. The original physical windows and sign results are unchanged. New replan model times are at most 5.160 ms; first uniquely observed merged-buffer latency is at most 40.070 ms from replan start. Those observed times are reported, never used to infer buffer lengths.

Method: split planner CSV at explicit blank buffer boundaries; compare all 36 root/quaternion/joint values against the recorded target frame index with absolute tolerance 1e-5 for six-significant-digit planner CSV serialization. Count requested offsets 0,5,...,45 against the matched buffer length. Terminal slots include the exact final frame (`index >= length-1`); strict clamping means `index > length-1`. Unique matches are required; ambiguous evidence cannot pass. Healthy active mode-1 feedback is required after initial transition; other samples remain visible and prevent a proof pass.

Full matrices have three initialization buffers because later pose handbacks reinitialize the planner. Each used translating buffer’s ordinal replan association is independently checked against its first unique content match and consecutive completion records. Later lifecycle buffers outside selected windows are not paired. Coverage requires recorded target frames at both measured-window boundaries and throughout the window: start, end, and internal gaps must each be at most 0.12 s. The initial transition exclusion must be at most 0.350 s. These explicit proof bounds use existing measurement/source freshness budgets and do not alter the physical 2 s commands, sign threshold, or 3 s settling criterion. Feedback age must satisfy `0 <= age < 0.2 s`. Missing/excessive gaps and excessive exclusions fail closed with reasons. This is not a hard real-time guarantee.

Regressions: fixture replay detects padding despite a passing physical case; changing only joint 28 rejects a root/timestamp-only match; equal buffers remain ambiguous. Initial RED: two assertion failures. Review added four RED counterexamples: missing start, internal gap, truncated end, and excessive initial exclusion. Final project pytest: 7 passed. Ruff: passed. No native/harness files changed.

Reproduce from the worktree root:

```sh
python3 docs/artifacts/g1_navigation_sonic_20261013/demo_navigation_horizon_proof.py \
  docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/fullmatrix-trials/fullmatrix-1 \
  docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon-trials/transitions-{1,2,3} \
  docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon-trials/fullmatrix-{1,2,3} \
  --output docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/horizon/coverage-all.json
```

This combined comparison exits 1 because the preserved baseline has padding; each of the six corrected reports has `coverage_passed: true`. `coverage-all.json` contains per-frame evidence, replan reason metadata, latencies, and hashes of all five input artifacts. `coverage-summary.json` contains analyzer/test hashes and compact totals.

After the temporal-proof correction, all six corrected runs still pass. Across their 18 windows, maximum start gap is 0.019266 s, end gap 0.018726 s, and internal gap 0.020100 s. The preserved baseline still fails for terminal padding. `coverage-gap-red.log` records the four review counterexamples before correction.
