# Stationary SONIC reference-expiry pilot

Authorized scope: local DimOS → rebuilt SONIC → MuJoCo, private network namespace with only loopback. No PC2 changes or real-robot commands.

Base SONIC source: `7c6d710b783a984ede504d24a65bc70e486b3b1d`, isolated worktree `/tmp/dimos-sonic-expiry-20261006`, branch `jihun/fix/sonic-reference-expiry`.

## Contract

- Explicit `--reference-timeout-ms 50..2000`; omission preserves existing operation. Requires `zmq_manager`, planner and encoder. Tests use 200 ms; this value is experimental, not a commissioned hardware setting.
- Guarded mode accepts the DimOS encoder's single-frame v4 message: token64 f32, index i64, two hand7 f32 vectors, finite values and exact layouts. Command input requires three u8 boolean scalars. Planner movement/VR topic and endpoint keyboard switching are disabled for this stationary pilot.
- The receiver's monotonic clock owns expiry. Stream entry starts a bounded first-reference deadline. Only an entirely validated, strictly increasing reference index refreshes it. Checking the deadline and accepting a frame share one mutex. The index high-water survives re-arm for the lifetime of the receiver.
- At expiry, incoming references cannot revive the stream. The manager requests planner IDLE and clears pending movement state. An already queued stop remains effective. Cached external reference data can remain in use during planner initialization; expiry does not imply instantaneous physical stopping.
- Recovery requires the receiver to consume a valid planner=true command and then a separate stream command. A publisher must observe feedback between these commands; sending both quickly can collapse them in the existing latest-command buffer.
- Telemetry reports receiver state plus whether the completed policy tick used the local encoder with the initialized, enabled planner motion in effective encoder mode0. The latter is confirmation of the control path, not a task cancellation or settling certificate.

## Limits

The protocol has no session nonce, capture timestamp, authenticated publisher identity, or acknowledged command transaction. Increasing delayed packets can still be accepted. A restarted publisher must continue above the receiver's high-water. `applied_index` means installed in the endpoint, not proof that the same index was consumed by the published policy tick. `age_ms` is time since arm until a valid reference arrives, and then time since accepted receipt; indices can describe the previous epoch during startup grace.

Hands follow the existing unloaded planner transition. Held-object recovery, anatomical mapping, grasp force, navigation, moving reference generation, collision checking, and real hardware commissioning remain outside this trial. Planner initialization failure retains the upstream stop/damping behavior; no safe fallback on failure is claimed.

## Validation

Deterministic C++ tests cover deadline equality, no first frame, duplicate/out-of-order frames, no late revival, re-arm ordering, complete packet validation including NaN under `-ffast-math`, endian handling, and malformed commands. A bounded MuJoCo probe checks actual telemetry/token behavior, keyboard isolation, duplicate traffic expiry, publisher disconnection, explicit recovery, manual handback, and upright physics without hidden automatic resets.
