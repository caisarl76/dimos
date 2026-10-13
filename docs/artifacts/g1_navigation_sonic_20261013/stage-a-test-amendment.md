# Approved Stage A noise-test amendment

Status: approved by the user's "confirmed. proceed" on 2026-10-13. No control-semantic change is implemented or
proposed. Task 5 and the Stage A simulation gate remain open; Stage B has not
started. The approved native execution method and sole motor owner are retained.

The current design converts each fresh body-frame command through measured IMU
yaw into the planner frame. The five-second live test in run09 walked forward
0.610 m while physics yaw ranged from -0.043 to +0.103 rad. It emitted 19 native
motion-state replans: entry into walking, then 18 direction changes of
0.02162–0.04576 rad. All exceed the existing 0.02 rad threshold. Facing remained
bit-identical and same-source heartbeats retained identical motion payloads.
These results fail the approved aggregate limit of one native replan. They do
not demonstrate a constant-heading noise test: the harness moves its lookahead
target with actual body yaw.

Keep measured-yaw body-frame conversion. Holding the movement vector in world
coordinates would alter what straight body-frame commands mean and needs a
different control design. Instead, separate two assertions without increasing
the fixed-input replan bound:

1. **Deterministic constant-heading fixture.** Use fixed nominal IMU and reference
   headings, alternating ±0.005 rad sensor noise, and the actual PController
   zero-crossing output at 0.2 m/s. Advance source sequence at 10 Hz and insert
   50 Hz heartbeats. Require identical movement/facing payloads after entry into
   walking. Replay those intents with one labelled fixture publisher into the
   private guarded native simulator, retaining its real rearm, sequence,
   deadline and applied acknowledgments. Require at most one native
   motion-state replan, including walking entry; report scheduled replans
   separately. This is a codec/native fixed-input proof, distinct from live
   feedback conversion.
2. **Live translating proof.** Retain real physics, production feedback
   conversion, PController.advance and ±0.005 rad sensor noise. Require active
   admission, advancing sources, forward displacement, literal same-source
   heartbeats and held facing for deadband commands. Log each atomic source
   input and correlate every changed motion state with an admitted fresh
   source threshold crossing or mode/sign/reference transition. Correlate each
   native state replan with that changed state; reject unexplained or
   heartbeat-induced replans. Report all actual body-yaw-driven replans, rather
   than excluding them from the recorded total.

Record command request, first moving packet and first native applied-walking
acknowledgment separately. Start the five-second measurement at applied walking,
but count replans from the preceding request so entry is included. Verify bounded
admission latency and permit only the corresponding initial IDLE prefix.

Implementation: add these two labelled harness paths and their raw
analyzer checks, rerun the constant-heading/native and live proofs, then request
the pending Stage A evidence review. Do not replace or relabel run09's failure.

No stopping criterion changes: measured planar speed and absolute yaw rate must
be below 0.03 for 0.5 seconds within three seconds. Run08 failed that criterion.
The earlier run02 reverse displacement and absent exact
`reference/bones_072925_test` fixture also remain commissioning blockers.
Hardware verification requires resolving those gates and fresh operator/sole
motion-control readiness. No real robot controller was launched in this stage.

Evidence: [run09 results](on-host-sim/run09/results.json),
[raw analysis](on-host-sim/run09/analysis.json),
[run08 stopping failure](on-host-sim/run08/results.json),
[source/test verification](task5-progress-evidence.json).
