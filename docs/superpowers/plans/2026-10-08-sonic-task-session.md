# SONIC task session implementation plan

> **For agentic workers:** Use superpowers:executing-plans for inline implementation and a final independent review.

**Goal:** Implement and test a transport-free DimOS session around an already-running, guarded SONIC controller.

**Architecture:** One single-threaded, one-shot session owns a supplied packet sink. The caller supplies monotonic time and decoded feedback, so tests need neither sockets nor sleeps. A profile pins sonic78 source FPS, hand order, and explicit native-planner handback; an incremental journal records lifecycle outcomes.

**Tech stack:** Python dataclasses/enums, NumPy, existing DimOS SONIC codecs, pytest.

**Spec:** [LeRobot integration reference](/docs/platforms/humanoid/g1/lerobot-sonic-integration-reference.md), especially “Work to carry into the DimOS adapter”. The user's latest “proceed” authorizes this next offline stage.

## Constraints and decisions

- Work in the current checkout: prerequisite SONIC modules are untracked here. Preserve unrelated changes and the other session's implementation logs. No commit, transport, robot connection, process launch, or blueprint registration.
- Add optional coherent 64D `token_state` to decoded feedback. Old passive telemetry remains valid without it; starting a session requires it plus fresh planner confirmation.
- Explicit 30/50 Hz sonic78 profile; all incoming hands are in the selected dataset order. Convert measured native right-hand slots into that order at startup, and convert back only at serialization.
- Startup seeds the planner token with measured hand targets, never implicit zero hands. Require a new guarded epoch and an applied reference from this session before accepting policy actions.
- 50 Hz maximum output, no catch-up bursts. Deadlines use caller monotonic time, not tick counts. Chunks expire by observation age and finite horizon. Interpolate within valid chunks; blend from the last output at replacement.
- This stage requests native planner handback directly and checks feedback. It does not implement a loaded-grasp recovery or promise a smooth physical return. Explicit `handback_policy="native_planner"` acknowledges this boundary. Cancellation never sends `stop=true` or starts/stops the native controller.
- No cross-process ownership claim: the injected sink must have one external publisher owner. Detect visible unexpected epoch/high-water changes; wire v4 has no session nonce.
- A terminal result reports streaming finished/cancelled/failed and handback confirmation separately. Object placement and settling remain unverified.

## Review focus

1. Delayed inference from an older task or observation must not refresh a deadline.
2. Guard indices retained from a previous epoch must not activate this session.
3. Stale/duplicate feedback must not confirm handback.
4. Packet or journal failures must stop policy streaming without claiming confirmed recovery.
5. Cancelled or completed instances must not emit another policy reference.

## Tasks

- [x] Add failing optional-token decode tests; implement validation in `sonic_feedback.py`; run focused tests.
- [x] Add `sonic_session.py` and adjacent tests for profile/chunk validation, startup, output ordering/rate, expiry, finish/cancel, handback timeout, and I/O failure. Regression failures observed before fixes.
- [x] Add an incremental JSONL journal and a transport-free demonstration with repeatable results under `docs/artifacts/g1_sonic_session_20261008/`.
- [x] Run focused SONIC tests, Ruff and production mypy; get an independent design/spec review and resolve material findings.
- [x] Publish a session guide and update the existing integration roadmap with precise offline evidence and remaining native MuJoCo work.

Verification: `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_feedback.py dimos/robot/unitree/g1/test_sonic_reference.py dimos/robot/unitree/g1/test_sonic_session.py -q`; Ruff on touched Python files; focused mypy. Baseline before changes: 123 SONIC tests passed (existing subprocess ResourceWarning).

## Completion evidence

Final focused suite: 182 passed (54 session, 73 feedback, 55 reference tests).
Ruff check/format passed for five Python files; focused production mypy passed
for two files with `--follow-imports=silent`. Six scripted cases passed in
`docs/artifacts/g1_sonic_session_20261008/run02/result.json`; source hashes match.

Independent gpt-6-astra/high review reproduced late-chunk deadline bypass,
late acknowledgement, regressed handback epoch and exhausted wire index.
All were fixed with regressions. Follow-up review found that two observations
before a tick could erase ownership-loss evidence; the session now latches
and journals that fault. Both retained-epoch and retained-high-water tests
pass; the reviewer confirmed no remaining material issue in the narrow review.

No transport, native simulation, camera, GPU, SSH, robot commands, commit or
push was performed. Native MuJoCo integration remains the next separately
observable milestone; direct planner handback has no loaded-grasp guarantee.
