# G1 SONIC Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Verify DimOS navigation through one guarded SONIC controller, first in MuJoCo and then on pc2, preserving the existing manipulation route and environments.

**Architecture:** Extend the existing SonicExecutionModule publisher with mutually exclusive planner/pose admission. Prove that controller path with a local Twist fixture before introducing SonicNavigationLink, a new private unicast request/reply component between two independently built and validated blueprints. Heavy PointLio/mapping/A* processing runs on the workstation; the publisher and native controller share pc2's monotonic clock.

**Tech Stack:** DimOS typed streams/RPC Specs, Python/pytest, ZeroMQ, private C++ SONIC receiver, CMake/GTest, MuJoCo, existing isolated Docker runtime, ARM64 TensorRT.

**Spec:** [G1 navigation through SONIC planner mode](/docs/superpowers/specs/2026-10-13-g1-sonic-navigation-design.md). Read it with this plan; its admission and handoff rules are normative.

## Global Constraints

- Target only pc2, `192.168.0.223`; preserve installed controllers, source changes, Python environments and unrelated jobs on both hosts.
- Use the local GR00T checkout and guarded runtime; no clone/download of a replacement source tree. Create an isolated DimOS worktree at execution time and import the required dirty SONIC files in a separate, clearly labelled `chore: import SONIC baseline (hash manifest)` commit before Task 2. This is unreviewed code from the manipulation session, not navigation implementation. A clean HEAD lacks these interfaces. Record every imported hash and original HEAD; if the other session updates any imported file, stop at the next task boundary, import/rebase onto its new reviewed baseline and resolve/retest affected navigation changes before continuing. Never cherry-pick navigation edits over its workspace or silently recopy changed files.
- Native changes belong to `build/sonic-navigation-runtime/source`, copied from `build/sonic-grasp-runtime/source`. Persist a reviewed patch and manifests in this task's artifacts; never edit the other session's source or binary.
- One motor controller and one execution publisher. No stock SDK/WebRTC/FastLio composition, automatic motor startup, simultaneous planner/pose references or `stop=true` navigation cancel.
- Legacy planner header is 1,280 bytes; minimal packet is 1,323 bytes. Guarded packets add only sequence/deadline; native request ID and issue timestamp are omitted.
- Planner modes IDLE/SLOW_WALK only; height -1; no arms/hands/VR overrides. Initial walk range 0.10–0.20 m/s, yaw rate limit 0.20 rad/s, facing lookahead 1.00 s, offset limit 0.20 rad (amended 2026-10-16 from 0.50 s / 0.10 rad: IDLE turns stalled near 5 deg; see the design amendment), angular threshold 0.02 rad, speed threshold 0.01 m/s.
- Follower `nerf_speed=0.2/0.55`, `min_angular_velocity=0.0`, `simulation=""`, including the native MuJoCo stage. Other compositions retain the 0.2 rad/s angular minimum.
- Source expiry 350 ms produces recoverable IDLE; continuous source outage 2 s ends/cancels navigation. Native publisher watchdog 250 ms and feedback timeout 200 ms latch admission; pose reference lease remains 200 ms.
- Input progress budgets: LiDAR/LIO/odom 350 ms, IMU/dynamic base TF 200 ms, map/costmap 1 s. Static calibrated TF does not expire. Source stamps must advance; receipt of a duplicate is insufficient.
- Peer polls at 10 Hz, native heartbeats at 50 Hz; command RTT limit 100 ms. For reply age `a`, request start `t0`, receipt `t1`, local source deadline is `t0 + 0.350 - a`; never compare absolute clocks across hosts.
- Workstation navigation pipeline has five modules plus the peer, initially `n_workers=5`; no LLM/heavy viewer. Two blueprints each get graph validation; there is no DimOS host-partition feature.
- Use `initial_safe_radius_meters=0`, disable merged-map input, preserve unknown/occupied cells, require accurate sensor-to-base extrinsics and base odometry. No map-to-IMU heading alignment is needed for body-frame commands.
- Activate the existing venv for commits; stage only task-owned files. No broad `git add`, environment synchronization or push. Each implementation task gets a selective commit after its checks pass.

## Review Focus

1. Tiny alternating follower yaw error must not become ±0.2 rad/s/facing chatter; regression in Tasks 1–2 and native replan trace in Task 5.
2. Expiry between healthy heartbeat ticks must stop motion without latching publisher loss; deterministic clocks in Tasks 3–4.
3. Delayed pre-rearm packets, publisher restart and pose handback must never revive old movement; sequence/barrier tests in Tasks 3–5.
4. Cached maps/odom, stamp rollback and stale atomic NavigationInput must not manufacture freshness; Tasks 6–7.
5. Lost cancel reply or a blocked velocity poll must close admission immediately and prevent pose activation; Tasks 4, 7 and 9.

---

## Stage A: Local controller path

Stage A has no dependency on the off-board protocol or robot networking. Its readiness/cancel fixture is explicitly test-only; production defaults stay closed without real input supervision.

### Task 1: Configurable follower angular minimum

**Files:** Modify `dimos/navigation/go2/replanning_a_star/{controllers,module,global_planner,local_planner}.py`; create `dimos/navigation/go2/replanning_a_star/test_controllers.py`; extend `test_global_planner.py` in that directory.

**Interfaces:**
- Produces `ReplanningAStarPlannerConfig.min_angular_velocity: float = 0.2`, finite and nonnegative.
- Extend `GlobalPlanner(global_config: GlobalConfig, *, min_angular_velocity: float = 0.2)`, `LocalPlanner(global_config: GlobalConfig, navigation_map: NavigationMap, goal_tolerance: float, *, min_angular_velocity: float = 0.2)` and `PController(global_config: GlobalConfig, speed: float, control_frequency: float, *, min_angular_velocity: float = 0.2)`; constructors return instances as usual. Forward the keyword through the module and, if needed, PdController's parent call; never mutate a class attribute globally.

- [ ] Write `test_min_angular_velocity_is_instance_scoped`: default controller at speed 0.2 maps errors ±0.01 to ±0.2, configured-zero controller to ±0.005, and constructing the latter does not alter the former. Reject negative/NaN/infinite config.
- [ ] Write `test_sonic_follower_envelope`: with `nerf_speed=0.2/0.55`, zero angular minimum and simulation disabled (`""`), forward speed is at most 0.2 and turns are at most 0.2; a small pure turn is 0.005, never 0.8. Verify module → GlobalPlanner → LocalPlanner propagation.
- [ ] Run `.venv/bin/python -m pytest dimos/navigation/go2/replanning_a_star/test_controllers.py dimos/navigation/go2/replanning_a_star/test_global_planner.py -q`; new cases fail on the missing option/current floor.
- [ ] Implement the keyword/config propagation and instance minimum; preserve all existing defaults.
- [ ] Rerun that command, require all cases passing, then selectively commit these files.

### Task 2: Planner codec, held intent and coherent feedback

**Files:** Create `dimos/robot/unitree/g1/sonic_navigation.py` and `test_sonic_navigation.py`; modify `sonic_feedback.py` and `test_sonic_feedback.py` in the same directory. Do not change the upstream sender.

**Interfaces:**
- Produces frozen `PlannerIntent(mode: int, movement: tuple[float,float,float], facing: tuple[float,float,float], speed: float, height: float = -1.0)` with float32 motion values; `NavigationLimits` holds the spec's six motion limits/thresholds.
- `encode_planner(intent: PlannerIntent, *, sequence: int | None = None, deadline: float | None = None) -> bytes`; guarded fields must appear together as `planner_sequence: i64[1]` and `planner_deadline: f64[1]`. Legacy mode has neither. Guarded minimal size is 1,339 bytes.
- `encode_navigation_control(action: Literal["rearm", "end"], *, sequence: int, deadline: float) -> bytes`: existing command start/stop false, planner true, plus `navigation_control: i32[1]` (1 rearm, 2 end) and the same guarded sequence/deadline. These fields are local protocol extensions.
- `PlannerIntentTracker(limits: NavigationLimits).update(twist: Twist, feedback: SonicFeedback, *, stopping: bool = False) -> PlannerIntent`; `heartbeat() -> PlannerIntent` returns the identical held motion values. Invalid/unsupported input returns stopped IDLE plus a rejection status, or raises a typed validation error caught by admission.
- Extend SonicFeedback with optional coherent quaternion/active fields and `planner_guard: PlannerGuardStatus | None`. Define frozen `PlannerGuardStatus(mode: int, high_water: int, applied_sequence: int, rearm_barrier: int, lease_armed: bool, latched: bool, idle_ack_sequence: int, fault: str | None)`; publish wire status as one `planner_guard` map in the existing g1_debug MessagePack payload. Missing guard/frame fields remain passive-compatible, never navigation-ready.

- [ ] Write `test_planner_wire_contract`: assert legacy length 1323/header 1280; guarded length 1339; decode little-endian seq/deadline; reject one missing guard field, malformed shape/nonfinite values and unsupported overrides. Compare legacy payloads against the existing local sender/golden check.
- [ ] Write `test_intent_holds_heartbeat_and_stop_heading`: 100 noisy feedback samples without new source commands leave motion bytes identical; stop snapshots yaw once; pure-turn input `(vx=0,wz=0.2)` yields IDLE, zero speed/movement and yaw lead 0.1; `vx=0.09,wz=0` yields stopped IDLE. Test forward/backward and reference reset.
- [ ] Write `test_small_sign_changing_yaw_does_not_chatter`: follower-derived ±0.005 rad/s at fixed/noisy measured heading yields ±0.0025 lead; threshold holds facing, never a ±0.10 swing. Verify conversion at reference yaws 0, π/2 and -1.2, inactive/missing frames and unsupported strafe admission.
- [ ] Extend feedback tests: legacy absence is allowed; exact wxyz shapes/nonzero finite norm and active [0]/[1] required when present; malformed packet does not refresh receipt; guard counters are nonnegative and acknowledged sequence cannot exceed high-water.
- [ ] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation.py dimos/robot/unitree/g1/test_sonic_feedback.py -q`; confirm new failures, implement the pure components/decoder, rerun for PASS and selectively commit.

### Task 3: Private native planner guard and telemetry

**Files:** Create `scripts/prepare_g1_sonic_navigation_runtime.py` and `docs/artifacts/g1_navigation_sonic_20261013/native-navigation.patch`. Patch only the private copy at `build/sonic-navigation-runtime/source/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/`: create `include/input_interface/planner_lease.hpp`, `planner_packet.hpp` and planner lease/parser/receiver/shared-oracle unit tests; modify `include/input_interface/zmq_manager.hpp`, `zmq_packed_message_subscriber.hpp`, `include/output_interface/zmq_output_handler.hpp`, `src/g1_deploy_onnx_ref.cpp` and that directory's `CMakeLists.txt` only if needed for test wiring.

**Interfaces:**
- Consumes Task 2's exact planner/control field names, dtypes and feedback MessagePack object. Adds an opt-in `--guarded-planner` flag; existing stationary-pilot defaults still discard planner motion.
- Produces native `PlannerLease`: `Rearm(int64_t sequence, double deadline, double now) -> bool`, `Accept(const GuardedPlannerPacket&, double now) -> bool`, `End(int64_t sequence, double deadline, double now) -> bool`, `EnterPose() -> void`, `Handback(double now) -> void`, `Tick(double now) -> void`, `status() const -> PlannerGuardStatus`. `GuardedPlannerPacket` contains Task 2's intent plus int64 sequence/double deadline. An accepted buffered packet is rechecked before installation.
- Startup is disarmed/closed. Rearm sets a high-water barrier, drains pending intent, then requires a fresh exact-zero IDLE above it. End disarms but does not clear an existing latch. Retain high-water through pose/handback. Reset/restart never admits movement automatically.

- [ ] Prepare an immutable baseline manifest of the shared source, binary/build config and model files; refuse mismatches against the inspected 51-file baseline until reviewed. Copy source and build prerequisites into the private directory, excluding generated build products; no rebuild in the shared checkout. Record the snapshot hashes, not an invented native git commit.
- [ ] Create `dimos/robot/unitree/g1/test_data/sonic_planner_lease_cases.json` as the shared deadline/sequence/phase transition oracle, and `unit_tests/test_planner_lease_cases.cpp` in the private native directory to replay it through real PlannerLease. Task 4 adds `dimos/robot/unitree/g1/test_utils/sonic_planner_lease.py` and `test_planner_lease_cases.py` to replay the same JSON through the Python stand-in. Cases include literal times/actions/expected high-water, applied sequence, IDLE/latch/armed/ack; both runtimes must produce identical expected transitions. Add GTests `PlannerLease.ExpiredPending`, `HealthyHeartbeatMotionExpiry`, `DuplicateDoesNotRenew`, `RearmBarrier`, `EndCannotClearLatch`, `PoseSuspendsPlannerLease`, `HandbackNeedsNewBegin` and `LegacyPlannerRejected`. At motion deadline 10.010 with last healthy receipt 10.000, `Tick(10.015)` must be IDLE and nonlatched; `Tick(10.251)` without further accepted receipts must latch. A seq 10 rearm then seq 11 exact-zero IDLE acknowledges 11; seq ≤10 cannot rearm or move. Apply IDLE/SLOW_WALK only and reject invalid horizontal vectors/height/speed.
- [ ] Configure/build privately: `cmake -S build/sonic-navigation-runtime/source/gear_sonic_deploy -B build/sonic-navigation-runtime/cmake -DBUILD_ROS2=OFF -DBUILD_DEPLOY_TESTS=ON`; then `cmake --build build/sonic-navigation-runtime/cmake --target run_tests --parallel 2`. Use the existing approved native build environment/dependencies; if unavailable, report the concrete missing capability before provisioning anything.
- [ ] Run `build/sonic-navigation-runtime/source/gear_sonic_deploy/target/release/run_tests --gtest_filter=PlannerLease.*`, demonstrate failure, then implement the opt-in parser/admission/install checks and coherent guard/frame telemetry. Add a private trace at the native motion-state-change replan branch so Task 5 can count those events separately from scheduled inference; validate feedback packet bounds before publication. Watchdog is based on accepted receipt, independent of capped motion deadline `min(deadline, now+0.250)`; duplicates/invalid packets do not refresh it.
- [ ] Rerun new tests and the existing `run_tests` suite; require PASS. Export the exact private diff to `native-navigation.patch`, record updated source hashes and selectively commit the preparer, patch and evidence manifest. Shared baseline hashes must still match.

**Task 3 regression gate (2026-10-13):** private controller builds; all 32 planner/pose/safety tests pass. Full suite has one unchanged legacy FK failure because its exact `reference/bones_072925_test` fixture is unavailable. The local official example is not equivalent and fails in both baseline/private binaries (396 assertions each). Keep this gate open; continue host-only Task 4 work and disclose it at Stage A review. Do not substitute generated data or weaken FK assertions.

### Task 4: Navigation admission in the existing execution owner

**Files:** Create `dimos/robot/unitree/g1/sonic_navigation_session.py` and `test_sonic_navigation_session.py`; modify `sonic_execution_module.py`, `test_sonic_execution_module.py`, `sonic_service_module.py` and `test_sonic_service_module.py` in that directory. Preserve the existing SonicTaskSession/action APIs.

**Interfaces:**
- Define frozen `NavigationInput(twist: Twist, source_sequence: int, velocity_deadline: float, readiness_deadline: float, ready: bool, fault: str | None)` in `sonic_navigation.py`. Deadlines are on the execution host's monotonic clock; Task 7 constructs them. Test fixtures construct them locally.
- Execution has only `navigation_input: In[NavigationInput]`, carrying the Twist, original source sequence and both deadlines atomically over pLCM. It has no `cmd_vel: In[Twist]`. Admit only a newer valid source sequence after explicit begin; bare Twist arrival is not an admission route. An expired atomic input forces IDLE. All stream callbacks enqueue immutable samples for the serialized owner loop; they do not mutate phase or send packets from callback threads.
- Add execution RPCs `begin_navigation() -> dict[str, Any]`, `end_navigation() -> dict[str, Any]`, `get_navigation_status() -> dict[str, Any]`; async implementations follow existing RPC conventions. Returned status includes phase, local request ID, source sequence, guard ack, cancellation_pending and fault.
- Define optional typed `SonicNavigationControlSpec`: `cancel_navigation(request_id: str) -> dict[str, Any]`, `set_navigation_goal(request_id: str, goal: PoseStamped) -> bool`, `get_navigation_peer_status() -> dict[str, Any]`. Add `navigation_enabled: bool = False` to execution config; production navigation-enabled execution requires its local relay injection; pose-only existing deployments may omit it. Cancellation ack includes matching ID, `cancelled=True` and fresh `NavigationState.IDLE` evidence.
- Execution also exposes `set_navigation_goal(request_id: str, goal: PoseStamped) -> bool` through the same bounded peer worker. Service adds `request_navigation(goal: PoseStamped) -> dict[str, Any]` using begin then set goal. Existing `request_service(...)` must await resolved cancellation/confirmed IDLE/settling before policy preflight and `start_task()`. Extend its execution Spec with all four navigation RPCs; retain current manipulation APIs.
- Native applied IDLE and planner cancellation do not measure settling. After the quick matching cancel ack, poll peer status separately for matching request ID, current readiness, measured planar speed and yaw rate both <=0.03 in magnitude, and a continuous settled interval >=0.5 s. PointLIO pose differences on advancing source stamps provide that evidence; default zero odometry Twist and commanded zero velocity do not. Relay translates its proof age to a local readiness deadline, capped at 350 ms. Keep the proof fresh until pose admission; a blocked control RPC cannot block planner heartbeats or shutdown. Production injected RPC transport waits are capped at 500 ms, with one outstanding call and no retry queue behind it.

- [x] Write `test_atomic_input_requires_explicit_begin`: pre-begin and stale/duplicate atomic inputs cannot move or refresh a target; heartbeat resends held motion bytes with only seq/deadline changing. Startup binds one publisher without motor start/stop. Concurrent begins and stale context from before begin are rejected.
- [x] Write deterministic clock tests: at source age 0.350 send IDLE without cancel/latch; new source at 0.400 resumes; at 2 s continuous outage end/cancel. At feedback age 0.200 or loss of **any** command/planner/pose subscription, close gate/latch/cancel. No active motion survives `_fault`, module shutdown or a blocked cancel operation.
- [x] Write `test_pose_waits_for_cancel_and_idle`: close gate before asynchronous cancel; missing/wrong-ID cancel acknowledgment blocks `start_task`. After correct cancellation and fresh native IDLE, use existing coherent token/hand seeding; pose lasts >0.250 without planner heartbeat. Existing pose timeout 0.200 still applies; handback clears targets and waits for explicit begin. Publisher restart requires a fresh handshake above native high-water, never resetting to zero.
- [x] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation_session.py dimos/robot/unitree/g1/test_sonic_execution_module.py dimos/robot/unitree/g1/test_sonic_service_module.py -q`; confirm new failures, implement a single serialized owner loop and nonblocking cancel worker, then require PASS. Keep recoverable navigation fault latches separate from the module's irrecoverable `_error`; fresh readiness plus explicit begin must be able to recover the specified faults.
- [x] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_*.py -q`; require existing manipulation tests passing, then selectively commit. No remote peer implementation is needed for these tests. Add a test-only Python PlannerLease stand-in that replays the same normative JSON cases as the native GTest, never a production alternative controller.

### Task 5: On-host MuJoCo proof

**Files:** Create `docs/artifacts/g1_navigation_sonic_20261013/demo_navigation_sim.py`, `demo_navigation_sim_worker.py` and `demo_analyze_navigation.py`; add `on-host-sim/manifest.json`, `on-host-sim/results.json` and per-run telemetry after execution. Add optional timestamped diagnostic sidecars in the private native `src/g1_deploy_onnx_ref.cpp`, exported as `native-navigation-diagnostics.patch` with hash provenance. Read existing simulation assets/probes without changing them. Fix the observed tiny-turn drift in `sonic_navigation.py` with regressions in `test_sonic_navigation.py`; keep the native control/admission rules unchanged.

**Interfaces:**
- Consumes Tasks 1–4 and the existing seeded sonic78 task session.
- Produces `demo_navigation_sim.py --stage on-host --output PATH` and `demo_analyze_navigation.py PATH`; analysis exits zero only when all declared checks pass. Bind explicitly configured loopback endpoints and DDS domain 0 isolated inside a fresh loopback-only network namespace (native SDK fixes domain 0); no robot NIC in this stage.

- [x] Add harness assertions before control changes: one native controller/publisher, stable initial IDLE, native guard enabled, required feedback/subscriptions ready, simulation fixture explicitly selected. Snapshot baseline source/asset/environment hashes and reuse the existing local simulator configuration.
- [x] Run a 5 s IDLE baseline, 2 s +0.2 forward, 2 s -0.2 backward and 2 s pure turns each direction, separated by stopped IDLE. Record simulator ground-truth planar speed/yaw/displacement, measured joints/IMU and reference fields separately. Assert intended displacement/yaw signs and final planar speed <0.03 m/s/yaw rate <0.03 rad/s held for 0.5 s within a 3 s settling timeout; failure blocks progression rather than adjusting pass criteria after the run.
- [x] Inject 250 ms source pauses (no cancellation), a >2 s source outage, lost feedback/subscriptions and a stopped publisher. Check Task 4's statuses and Task 3's native behavior, including a motion expiry between healthy heartbeats.
- [x] Run the approved separate `--noise-proof fixed` and `--noise-proof live` paths. Keep the ≤1 native motion-state-replan bound, including walking entry, for a deterministic constant-heading zero-crossing follower/±0.005 rad noise fixture. In live translation retain production measured-yaw conversion; log exact source inputs, conversion operands, wire packets and native applied acknowledgments, and attribute every native state replan to an admitted changed state and every emitted direction change to a fresh threshold crossing or mode/sign/reference transition. No heartbeat-only or unexplained replans. Measure five seconds from applied walking, count replans from the preceding request, and bound initial admission latency. Record all genuine body-yaw changes separately from sensor noise. Verify real turns still change facing when accumulated measured motion crosses 0.02 rad.
- [x] Run planner → seeded pose for at least 1 s → confirmed planner IDLE; cancel pose and expire its source separately. Check unchanged 200 ms pose lease, suspended planner watchdog and no old motion after handback. Analyze results with `demo_analyze_navigation.py`; all assertions must pass before Stage B. Selectively commit harness and compact evidence, preserving failures as evidence.

Observed during execution: run02's unchanged two-second forward check failed
at -0.03797 m. Runs03–04 demonstrate forward motion with the same assets, but
do not explain or erase that failure. Run05 passed all four movement signs and
stops, then exposed a separate noisy-facing drift bug. The approved facing
threshold alone was insufficient when tiny nonzero turn requests followed
measured post-turn drift. The revised spec adds a 0.01 rad/s turn deadband;
regressions cover both translation and IDLE. Preserve all failures and keep
commissioning blocked until the transition discrepancy is resolved.

Run08 also failed the unchanged three-second measured-settling gate. Run09
translated with active admission, held facing and literal heartbeats, but
recorded 19 native replans as actual body yaw changed the converted movement
direction. Task 5 remains open. The user approved separating deterministic
constant-heading noise proof from live body-yaw attribution in
`docs/artifacts/g1_navigation_sonic_20261013/stage-a-test-amendment.md`.
Run09's recorded failure remains unchanged; stopping and commissioning gates
are retained. Stage B still depends on the completed Stage A checks/review.

### Stage A review gate

- [x] After Task 5, obtain a fresh read-only architecture/code review of Tasks 1–5, including the imported baseline boundary, atomic input wiring, shared lease fixtures and MuJoCo evidence. The reviewer uses Astra/high and a limited context; it performs no implementation. Fix blocking findings with regression tests before starting Stage B. No additional user permission check is required for this already approved gate.

## Stage B: New two-host transport and navigation graphs

### Task 6: Readiness from existing advancing source stamps

**Files:** Create `dimos/robot/unitree/g1/sonic_navigation_readiness.py` and `test_sonic_navigation_readiness.py`; create `dimos/mapping/test_costmapper_stamps.py` for the nonempty/empty source-stamp assertions. No planned changes to shared PointLio/ray-map/CostMapper implementation.

**Interfaces:**
- Produces frozen `InputProgress(name: str, source_stamp: float, age: float, upstream_lag: float, valid: bool)`, `ReadinessSnapshot(ready: bool, inputs: tuple[InputProgress,...], fault: str | None)` and `NavigationReadiness.observe(name: str, stamp: float, *, valid: bool, received_monotonic: float) -> None`, `.snapshot(now: float) -> ReadinessSnapshot`.
- Required names: `lidar_raw`, `imu_raw`, `lio_cloud`, `odom`, `base_tf`, `global_map`, `global_costmap`. Track progress only on increasing original stamp; track dependency lag in the same sensor clock, and local age on a monotonic clock. Static extrinsic validity is a separate prerequisite. Source rollback invalidates the running peer's chain; recovery requires a fresh peer instance/commissioning handshake, healthy progression and a new explicit navigation request. Do not silently reset the native sequence high-water.

- [ ] Write `test_cached_inputs_cannot_refresh_readiness`: repeat identical stamps while feeding new Twists; at 0.350 LiDAR/odom fail, at 0.200 IMU/dynamic TF fail and at 1.000 map fails. Empty map/costmap, invalid base odometry and unknown extrinsic are unready. Map stamps lagging their current LIO/raw ancestors beyond the respective budget are unready even if arrivals are new.
- [ ] Write `test_stamp_reset_requires_new_session` and `test_static_transform_does_not_expire`; compare sensor-relative lag only within its known clock domain. Wallclock jumps cannot alter receipt-age checks.
- [ ] Replay the pinned PointLio/DDS data: assert preserved adapter stamps, processed LIO output stamp progression, ray map stamp equals consumed cloud stamp, nonempty costmap stamp equals map stamp. Delayed/duplicate input and an idle process must not produce apparently progressing output. Audit the pinned PointLio core dependency locally; if a stage regenerates cache stamps, first add a failing test identifying that exact stage, then amend the plan with the smallest provenance fix.
- [ ] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation_readiness.py -q`, verify failures, implement the peer-local monitor, then require PASS plus the replay assertions. Selectively commit. Keep empty-map readiness rejection rather than modifying the shared empty-output behavior.

### Task 7: SonicNavigationLink, peer and pc2 relay

**Files:** Create `dimos/robot/unitree/g1/sonic_navigation_link.py`, `sonic_navigation_peer_module.py`, `sonic_navigation_relay_module.py`, `test_sonic_navigation_link.py` and `test_sonic_navigation_peer_module.py` in the same directory.

**Interfaces:**
- Consumes Task 6's ReadinessSnapshot, Task 4's NavigationInput/ControlSpec and existing `ReplanningAStarPlannerSpec` (`set_goal`, `cancel_goal`, `get_state`). The peer injects that existing Spec locally; no private MovementManager method calls.
- Define frozen `NavigationPollReply(correlation_id: str, request_id: str | None, source_sequence: int, twist: Twist | None, source_age: float, readiness: ReadinessSnapshot)` and `bound_navigation_input(reply: NavigationPollReply, *, t0: float, t1: float) -> NavigationInput`; reject RTT >0.100 or negative/nonfinite ages. Set velocity deadline `t0+0.350-source_age`; readiness deadline is the minimum `t0+budget-input.age` across inputs. Expired/missing readiness is never marked ready.
- Version 1 private link uses bounded JSON over ZeroMQ ROUTER/DEALER, explicit endpoint config, 64 KiB maximum frame, one part, at most one pending poll and one independent control request. Strict schema includes version, operation, correlation/local request ID and finite six-component Twist; no pickle on the network. Do not embed remote absolute monotonic deadlines.
- Peer input `cmd_vel: In[Twist]` is the only off-board consumer; increment original source sequence once per actual MovementManager emission. Serve age at reply time, not at caching time. Raw/map inputs feed Task 6's monitor. Relay outputs only atomic `navigation_input: Out[NavigationInput]`, with the Twist and bounded deadlines. It may repeat the held source sequence, velocity and original velocity deadline while advancing independently validated sensor readiness; a repeated sequence never refreshes motion. There is no pc2 cmd_vel publisher/consumer.
- Relay implements Task 4's SonicNavigationControlSpec. Close local admission before remote cancel; poll/cancel use separate bounded request slots. Reply to cancel only after public `cancel_goal()` succeeds and a fresh `get_state()==NavigationState.IDLE` is obtained, preserving request correlation. Missing reply is unresolved, never success.

- [ ] Write `test_unsynchronized_clock_deadline`: peer clock can be 10,000 while pc2 t0=10.0/t1=10.08, reported source age 0.12; assert local velocity deadline 10.23 and motion heartbeat never exceeds it. Cached sequence, stale correlation, age NaN/-1, RTT 0.101, excess packet size and invalid version cannot refresh a target.
- [ ] Write `test_cancel_preempts_blocked_poll`: never-completing poll does not prevent cancel dispatch; gate closes before dispatch. An old request's ack cannot authorize a new request or pose. Assert the injected public cancel/state RPCs were called in order; socket receipt alone is insufficient.
- [ ] Write `test_freeze_inputs_while_twists_continue`: freeze each Task 6 input; reply becomes unready and execution ends/cancels despite 10 Hz new Twists. Simulate stale/duplicate atomic inputs and source rollback: motion cannot outlive its last validated deadline. A changed peer process starts unready and requires an explicit new navigation session.
- [ ] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation_link.py dimos/robot/unitree/g1/test_sonic_navigation_peer_module.py -q`, confirm failures, implement bounded transport and both modules, rerun PASS and selectively commit.

### Task 8: Read-only sensor relay and independent graph validation

**Files:** Create `dimos/robot/unitree/g1/sonic_navigation_sensor_module.py`, `sonic_navigation_graph.py`, `test_sonic_navigation_sensor_module.py`, `test_sonic_navigation_graph.py`, `blueprints/navigation/unitree_g1_sonic_navigation_peer.py` and `blueprints/navigation/unitree_g1_sonic_navigation_pc2.py`. Regenerate `dimos/robot/all_blueprints.py` through its test; never hand edit it.

**Interfaces:**
- Sensor module promotes only the proven read-only DDS conversion from `docs/artifacts/g1_navigation_20261012/demo_dds_lio_adapter.py`, preserving sensor stamps, Livox offsets and g→m/s² conversion; relay G1 state/dynamic waist TF without any motor publisher. Use separate explicitly configured ZeroMQ PUB/SUB unicast TCP endpoints for clouds and IMU/state/TF so a cloud burst cannot starve IMU. Each sensor message has two parts (allowlisted channel name and existing LCM-encoded payload), with 16 MiB maximum payload, HWM=2 and bounded drain-to-latest per channel. Preserve source stamps and expose raw progress to the peer; this is new relay code, not an existing host-partition facility. Socket/process/source reset invalidates readiness instead of replaying queued data as live. Accurate commissioned static extrinsics are required config.
- `validate_sonic_navigation_graph(blueprint: Blueprint, *, role: Literal["peer", "pc2"]) -> None` validates the resolved graph before worker start using an explicit role allowlist, remapped connections and endpoint ownership. Hook the validator into these two blueprint build paths, not a fictitious distributed graph feature.
- Peer graph: sensor receiver → PointLio → ray map → CostMapper → A* → MovementManager → SonicNavigationPeerModule; relevant raw/LIO/map inputs also fan out to its readiness monitor. The receiver can live inside the peer module to keep module count bounded. Peer alone consumes off-board cmd_vel.
- pc2 graph: read-only sensor sender, SonicNavigationRelayModule, existing SonicExecutionModule (and existing service module only when manipulation integration is enabled). Relay is the only NavigationInput producer, nothing on pc2 produces or consumes cmd_vel, and execution alone binds the native command endpoint. Default launch stays passive; graph validation cannot prove ownership of unrelated native processes.

- [ ] Write graph tests that add stock DDS SDK, FastLio2, an unknown module, WholeBodyConnection, second execution owner, alternate source or second cmd_vel consumer and assert build rejection **before** any worker/socket/control-mode startup. Test valid graphs independently; remapping cannot bypass checks.
- [ ] Write conversion/relay replay tests: preserve source stamp, XYZ+offset schema and IMU units; no motor write API calls. Unknown extrinsics, empty/cached clouds and transport reset produce unready input progression.
- [ ] Run `.venv/bin/python -m pytest dimos/robot/unitree/g1/test_sonic_navigation_sensor_module.py dimos/robot/unitree/g1/test_sonic_navigation_graph.py -q`; confirm failures, implement both graph builders/validation and sensor adapter, then require PASS.
- [ ] Regenerate with `.venv/bin/python -m pytest dimos/robot/test_all_blueprints_generation.py -q`; inspect only task entries relative to the imported dirty baseline, run the focused SONIC/planner suites and Ruff on changed Python files, and selectively commit.

### Task 9: Complete link simulation and failure recovery

**Files:** Extend Task 5's `demo_navigation_sim.py` with `--stage two-host`; add `two-host-sim/results.json` and hashes/telemetry in the same artifact folder. Update `docs/platforms/humanoid/g1/navigation-verification-2026-10-09.md` with measured results.

**Interfaces:** Consumes both independently validated Task 8 graphs, Task 7's link and Task 4's service/navigation RPCs; produces full-stack simulation evidence. Use separate processes/clock offsets/network-delay injection; this remains isolated from the robot.

- [ ] Run the existing Stage A suite through the link using recorded sensor/map inputs and real graph wiring. Exercise a clear synthetic goal and a conservative blocked/unknown footprint; goal rejection must retain occupied/unknown cells and never invoke startup-disk clearing.
- [ ] Drop/delay replies, stall cancel/state RPC, stop peer, freeze each sensor/LIO/map stage and restart peer/publisher. Verify bounded deadlines, immediate local gating, explicit recovery, unresolved cancel blocking pose and no old target after handback.
- [ ] Measure worker/relay load, command age, native state/control lateness and replan triggers during the existing VLA phase. Analyze with Task 5's analyzer; require all software assertions passing before generating a robot launch package. Commit compact evidence and report; failures remain visible.

## Stage C: Isolated pc2 commissioning

### Task 10: Reviewable ARM64 package and passive readiness checks

**Files:** Create `docs/artifacts/g1_navigation_sonic_20261013/pc2-package/Dockerfile`, `provision.sh`, `launch.sh`, `stop.sh` and `manifest.json`; create `demo_pc2_navigation_readiness.py` in the parent artifact folder. Deployment target is a new subdirectory `/home/unitree/dimos-navigation-verify-20261009/sonic-navigation-20261013/` only.

**Interfaces:** Package Tasks 1–9, matched model/config hashes and a distinct ARM64 native binary. `launch.sh --check-only` does not start a motor controller; actual launch uses an explicit commissioning argument. Pin all task images/venvs, mount unrelated source/assets read-only and keep TensorRT caches/logs under the task directory. Use build-only host networking for the known Jetson Docker bridge limitation; do not change host firewall/daemon settings.

- [ ] Assemble/build privately with the existing isolated capabilities; record C++ source/patch/compiler/build-option, DimOS snapshot, model and binary hashes. ONNX version alone is insufficient: model/config/TensorRT must match the commissioned profile. Do not reuse a host-specific TensorRT engine across incompatible architectures/devices.
- [ ] Recheck the stored 1,232 environment fingerprints plus installed controller/shared source hashes and unrelated processes before/after provisioning; require no unintended differences. Validate ARM64 codec/feedback/guard checks with robot motion disabled. Provision no host/Python dependencies implicitly.
- [ ] Run passive readiness probe: source stamps/progress, input ages/extrinsics, measured joint/IMU feedback, available CPU/GPU, endpoints and existing motor writers. Save failures/status without sending a walking/mode command. Native performance under control is verified only after Task 11's fresh operator readiness.
- [ ] Review the complete launch/stop package and simulation evidence, then selectively commit. Software preparation is authorized; actual robot launch depends on fresh Task 11 conditions.

### Task 11: Supervised real-robot verification and report

**Files:** Create `docs/artifacts/g1_navigation_sonic_20261013/demo_pc2_navigation_walk.py`; record `hardware/results.json` and telemetry in that folder; update the navigation verification report.

**Interfaces:** Harness uses existing execution/service RPCs, permits only bounded named test cases and records reference vs measured state. It never silently increases speed or changes controller ownership to obtain motion.

- [ ] Obtain fresh confirmation: pc2 standing/Run-ready in a clear area, operator present with a tested stop under **native SONIC**, other session released this robot. Inspect runtime ownership again; use only the private task package. Earlier stock-remote walking/confirmation does not cover a new native launch.
- [ ] Demonstrate IDLE and operator stop before commanded movement; commission the matched native model/controller on a supported/controlled setup using the existing deployment workflow. Persistent >50 ms low-level-state lateness or missed 50 Hz control intervals blocks movement until diagnosed; record load/freshness with the sensor relay active.
- [ ] Send +0.10 m/s for 1 s, IDLE and settle; only after measured success test +0.20 m/s for 2 s and ±0.10 rad/s IDLE turns for 1 s. Record independent observation/video and calibrated LIO/base displacement, mode/guard ack, source age and stopped behavior. If it stays in place, report accepted vs applied vs observed separately and diagnose rather than escalating commands.
- [ ] Verify explicit cancel and a controlled source pause/publisher watchdog using the agreed supported setup; verify phase handback with the existing seeded pose route only after the corresponding simulation passed. Operator stop stays available throughout.
- [ ] Admit one short physical goal only after accurate sensor-to-base extrinsics, obstacle/unknown footprint coverage and independent map clearance are established. Existing near-field unknowns remain a blocker, not a reason to clear cells. Finish by cancelling, confirming stationary state and stopping only task-owned processes; record fingerprint/shared-source equality and remaining limitations.
- [ ] Update the report with distinct software/simulation/hardware verdicts and measured evidence; run documentation/link checks and commit only this task's evidence/report. Navigation verification is complete only when required real-robot cases pass; blocked goals remain explicitly unverified.

## Plan review and execution boundary

Self-review maps every design section to Tasks 1–11: topology/ownership (8), codec/frames/feedback (2), admission/phase/cancel (3–4), freshness/placement/readiness (6–9), provenance/isolation (3,10) and simulation/hardware acceptance (5,9,11). The five Review Focus failures have explicit owning tests. Interface names and deadline clock domains are shared across the tasks; Stage A fixtures cannot enable a production readiness bypass.

This is a prepared plan, not completed implementation. Review it and choose native or subagent-driven execution before changing the linked interfaces. Native execution with fresh Stage A and final reviews is selected because the codec, native guard and execution owner share tightly coupled state/sequence semantics. A fresh bounded review is still required before robot commissioning.
