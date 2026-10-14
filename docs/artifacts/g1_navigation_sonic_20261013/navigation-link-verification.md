# Task 7: bounded navigation peer and relay

The private version 1 link, off-board peer, and execution-host relay are implemented. Verification is host-only: **61 new link/peer tests, 126 integrated tests, and 504 SONIC/navigation compatibility tests pass**. Mypy passes for all four affected production files; Ruff passes. Two independent review passes reproduced lifecycle defects before correction, and the final review found no remaining issues in that scope.

The peer consumes actual MovementManager `cmd_vel` emissions and the seven monitored sensor/map streams. It injects the public `ReplanningAStarPlannerSpec` and calls `set_goal()`, `cancel_goal()`, and `get_state()`. The relay publishes only atomic `NavigationInput` messages. Polling runs independently of the execution module's existing native heartbeat.

JSON envelopes have an exact allowlisted schema, version, correlation ID and request ID. Frames are single-part and at most 64 KiB; malformed/duplicate fields, unknown operations, nonfinite numbers, stale correlations and excess frames are rejected. Poll and control use independent bounded slots and caller-owned sockets. Original source sequence and age survive caching. Local deadlines use the execution host's request-start time, source/input ages and a maximum 100 ms round trip, including settling evidence; cancellation transport has its separate 500 ms bound. No remote absolute clock or network pickle is used.

Readiness can be healthy before the first velocity emission. That produces a zero-valued input with sequence zero and an expired velocity deadline, allowing the explicit IDLE handshake without admitting motion. Readiness at the handshake's sequence barrier can advance, while motion still requires a strictly newer real emission. Repeated velocity sequences preserve the earlier velocity deadline. Frozen inputs, source rollback and unexpected peer ownership changes close admission.

Cancellation publishes an unready input before the remote call. Success requires both the public cancellation result and a fresh public IDLE state, correlated to the current request. Settling uses advancing odometry pose differences, never the default zero Twist: at most 0.03 m/s and 0.03 rad/s continuously for at least 0.5 s. Repeated odometry does not increase the measured interval.

Goal/cancel transitions fence polls by generation. Expected replies from a previous ownership binding cannot latch a false peer reset during an unresolved transition. A new explicit session can end before its first goal: the optional cancellation payload `previous_request_id` compares against a previously confirmed cancelled/IDLE owner. This binding is retained across lost goal/cancel requests and acknowledged only after a fresh cancellation/state check. Retired request IDs cannot reacquire ownership. No retry is reported as success merely because a request was sent.

## Verification evidence

The initial two-file test run failed collection because all three production modules were missing. Additional regressions exposed lost-goal gate closure, goal/poll races, stale acceptance, first-goal startup, cancellation before a new goal, slow settling proof, and lost binding retries. Each substantive defect was reproduced before its correction. The final independent review examined the corrected production paths without modifying them.

- `/tmp/g1-navigation-task7-final.xml`: 126 passed, no failures/errors/skips.
- `/tmp/g1-navigation-task7-broad.xml`: 504 passed, no failures/errors/skips.
- `navigation-link-verification.json`: JUnit hashes, affected source hashes, review result and scope.

These tests exercise real loopback JSON sockets, public planner RPC ordering, both control/poll slots, frozen streams and the actual native-admission session. They do not constitute the two-host graph, real extrinsic calibration, or physical commissioning; those remain later plan tasks. The exact legacy FK fixture is still missing, and physical failure investigation is separately recorded in [motion-resolution/README.md](/docs/artifacts/g1_navigation_sonic_20261013/motion-resolution/README.md). The original workspace, shared environment and original native source were not modified.
