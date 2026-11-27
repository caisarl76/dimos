# Native SONIC task-session validation

Verified 2026-10-09, continuing the `20261008` session artifacts. The new
DimOS `SonicTaskSession` ran against the guarded native C++ SONIC controller
and dynamic MuJoCo G1 simulation. One recorded episode and four fault cases
passed their declared control/session checks. A separate native smoke test
passed before the full run. No physical G1 commands were sent.

This extends the [offline session foundation](/docs/platforms/humanoid/g1/sonic-task-session.md).
Production session code needed no changes during this native validation.
The new runner is a bounded experiment, not yet a production transport module
or registered DimOS blueprint.

## Inspect the motion

- MuJoCo replay video, normal episode (`../../../artifacts/g1_sonic_native_session_20261008/run01/replay/native_session_replay.mp4`).
- Measured arm/hand speed plot (`../../../artifacts/g1_sonic_native_session_20261008/run01/replay/joint_speeds.png`).
- Replay preview (`../../../artifacts/g1_sonic_native_session_20261008/run01/replay/preview.png`).
- [Independent evidence audit](/docs/artifacts/g1_sonic_native_session_20261008/run01/audit.json).

The video renders recorded MuJoCo `qpos` with `mj_forward`, at 1× wall-time
playback, with whole-robot and close-up views. It does not rerun dynamics.
Velocity overlays and the plot come from measured native `body_dq`/hand
telemetry, not from the requested latent-token step limit.

## What ran

The private network namespace had only loopback, separate from the host
namespace. Both DDS and ZMQ were confined to it. The sole reference publisher
bound loopback port 15556; native feedback used 15557. The existing native
receiver guard had its 200 ms reference lease enabled. The simulator used
the 43-DoF G1/Dex3 scene; elastic support was released before each task session.

The controller binary SHA-256 matched the earlier expiry validation:
`b9a95ae89b4ce328526715b0cee30689d06dbf5c23f23479dc64ddbfce19750e`.
The [manifest](/docs/artifacts/g1_sonic_native_session_20261008/run01/manifest.json)
pins the binary, DimOS/session/serializer sources, simulator helper, model
assets and dataset files. GPU 1 was used; existing GPU workloads were left
running.

The input was **episode 42, 561 sonic78 frames, 11.22 seconds at 50 Hz** from
`handover_bottle_sonic78_50hz_261002`. Dataset metadata labels all 43 episodes
as training data. This is a recorded-action execution test, not held-out
policy evaluation or a new VLA inference result. The simulation contained no
object or table interaction.

The runner supplied 40-frame chunks every 0.4 s with their original timeline,
native 50 Hz source rate, explicit MuJoCo right-hand swap, 0.3 s chunk blends
and 0.05 token-step limit. Sampling and blending mean the emitted stream is
not a byte-for-byte playback of every original frame. Its median send interval
was **20.16 ms**, approximately **49.6 Hz**; the normal replay's maximum gap
was **21.54 ms**. All accepted references kept increasing wire indices across
the sessions.

## Session outcomes

“Passed” below means the injected scenario produced its declared outcome and
the sampled posture envelope remained valid. It does not mean a grasp or
placement succeeded. Time to confirmation is measured by the session after
its handback request, rather than a guaranteed controller latency.

| Scenario | Observed session outcome | Session planner confirmation |
| --- | --- | --- |
| Complete recorded episode | `finished` | 141 ms |
| Cancel after about 0.8 s of streaming | `cancelled` | 121 ms |
| Withhold replacement inference chunk | `failed: observation stale` | 141 ms after handback request |
| Withhold feedback from task consumer | `failed: feedback stale; handback timeout` | Correctly unconfirmed after 2.02 s |
| Close the reference publisher | `failed: packet send failed` | 323 ms after failed-send handback request |

The inference-stall case stopped at approximately 0.667 s of observation age.
Its explicit 0.65 s observation-age limit fired before the 0.8 s chunk horizon;
this trial does not relabel that failure as horizon exhaustion.

For feedback loss, the independent recorder stayed active while the session
stopped receiving feedback. That recorder observed planner recovery about
125 ms after the request. The session still reported **unconfirmed**, which
is the intended behavior when its own feedback is missing.

For publisher loss, the actual XPUB socket was closed. No later policy packet
was sent. Native telemetry reported reference expiry at **210.5 ms reference
age**, followed by planner confirmation. The publisher was recreated only
after this evidence was finalized, to stop the owned simulation controller.

## Measured motion and limits

Across the five recorded cases, pelvis height stayed at least **0.7805 m**,
maximum tilt was **8.44°**, support was disabled, and there were **zero automatic
resets**. These are sampled observations of this run.

The normal episode's peak arm-joint speed was **3.64 rad/s**, at the right wrist
roll during entry into the recorded motion. After its handback request, the
peak arm speed was **3.39 rad/s**. The final approximately half-second window
spanned 0.499 s and had maximum body-joint speed **0.022 rad/s**.

The largest arm peak across all cases was **6.44 rad/s**, also at right wrist
roll, during the publisher-loss case. It occurred **0.358 s after streaming
began and 0.449 s before publisher closure**. The nearest physics sample showed
right-hand/hip clearance at zero and slight left-hand/hip penetration. This
peak is retained and associated with the startup/contact interval; it is not
evidence that publisher expiry caused that peak. Its precise physical cause
has not been isolated. The older user-accepted contact spike remains a
separate observation.

All cases ended with low measured speeds in their final 0.479–0.499 s sampled
windows. Those windows are reported with actual duration, sample count and
maximum gap in the audit, rather than assuming that 25 samples equal 0.5 s.
Direct native planner handback can still move the arms and hands substantially.
This test does not establish loaded-grasp retention, hand/body clearance,
joint-speed bounds or physical hardware readiness.

## Artifacts and process cleanup

- Runner (`../../../artifacts/g1_sonic_native_session_20261008/demo_native_session.py`),
  namespace launcher (`../../../artifacts/g1_sonic_native_session_20261008/demo_run_trial.sh`),
  audit (`../../../artifacts/g1_sonic_native_session_20261008/demo_audit.py`), and
  renderer (`../../../artifacts/g1_sonic_native_session_20261008/demo_render.py`).
- [Raw result](/docs/artifacts/g1_sonic_native_session_20261008/run01/result.json),
  telemetry (`../../../artifacts/g1_sonic_native_session_20261008/run01/telemetry.jsonl`),
  references (`../../../artifacts/g1_sonic_native_session_20261008/run01/references.jsonl`),
  physics (`../../../artifacts/g1_sonic_native_session_20261008/run01/physics.jsonl`),
  and per-case `*-session.jsonl` journals beside them.

Both owned child processes eventually exited with code **0** and were reaped.
The controller exceeded the initial eight-second shutdown wait; that timeout
warning remains in the result. The subsequent bounded cleanup reaped both
processes with exit code 0.
The earlier smoke runner terminated its simulator with SIGTERM after its
successful case; the final runner waits for the simulator stop-file shutdown.

The first launch was rejected by temporary filesystem exhaustion before any
child process started. A later check showed 7.3 GB free and directory creation
working; no files were deleted by this session. An OSMesa render attempt failed
because that GL backend was unavailable; the successful render uses EGL.

## Next integration step

Connect the existing LeRobot policy server to this validated session in
simulation, preserving checkpoint source FPS, sonic78 action layout and camera/
state preprocessing. Start with recorded observations, then add live simulated
camera input. Continue measuring startup and handback motion, and improve
contact clearance/recovery before hardware commissioning or loaded grasping.

The native guard remains a local prototype proven in isolated simulation.
Its installation and exact controller version on PC2 have not been validated
by this test. The original object-to-tray service still needs its object,
grasp/placement evidence, camera/robot calibration and task-specific policy.
