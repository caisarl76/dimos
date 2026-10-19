# pc2 SONIC navigation package (Task 10)

Target: pc2 `unitree@192.168.0.223` (29 body joints, no hands). Everything goes under
`/home/unitree/dimos-navigation-verify-20261009/sonic-navigation-20261013/`. Existing
controllers, source trees, Python environments and images stay unchanged.

**Status, 2026-10-19:** re-provisioned on pc2 (attempt 10, 16:47-16:52 CST); evidence
in [pc2-provision/summary.json](/docs/artifacts/g1_navigation_sonic_20261013/pc2-package/pc2-provision/summary.json).

- The ARM64 native tests pass 41/42; only the FK fixture test fails. The controller is
  `dc59a4ab…`, identical on all ten builds. Two `--check-only` runs were ready with
  nothing blocking.
- Attempt 10 adds the walk agent's `goal` operation for `goal_1m` (`823f852cf`), on top
  of the 0.20 rad facing lead (`808ece4c8`, attempt 9).
- Launch order: `native`, `start`, `graph`.
- Attempt 6 (`.attempt6-no-operator-start`): in the operator session at 14:30 CST the
  graph started right after the native. The native never left init and sent no
  feedback, so the harness ran no case
  ([hardware/session-20261015-1](/docs/artifacts/g1_navigation_sonic_20261013/hardware/session-20261015-1/results.json)).
- Attempt 5 (`.attempt5-probe-waist-every-sample`): the operator's first
  `--commission … native` at 14:02 CST was refused, correctly by the gate, but on a
  false reading. The probe converted the waist transform on every ~1040 Hz lowstate
  sample, 20 times the sensor sender's rate, which stalled its own reader for
  0.1–0.33 s at a time. It now uses the sender's 50 Hz decimation; the budgets are
  unchanged.
- Fingerprints, original native source and shared controllers were unchanged before
  and after. No controller or graph was started.
- `launch.sh --check-only` is **ready, with nothing blocking**, in two consecutive
  runs: the mount is accepted and lowstate's repeated ticks are tolerated.
  - The first run, just after the build at load 1.54, blocked on a 205 ms IMU receipt
    gap against the 200 ms budget. LiDAR and lowstate had simultaneous large gaps, so
    it was a shared reader stall. The budget is unchanged.
- Earlier attempts are kept as `.attemptN-*`:
  - 1–2: smoke check (`zmq`, numba cache).
  - 3: IMU DDS type.
  - 4: strict lowstate probe, superseded to install the fixed probe, graph ownership
    guard, walk agent and accepted mount.
- The manipulation session ran its own controller and hold-reference tests while
  attempts 2 and 3 ran.

## Contents

| File | Purpose |
| --- | --- |
| `assemble.py` | Rebuilds the payload from committed inputs and writes `manifest.json` |
| `native-navigation-pc2.patch` | Navigation guard port onto pc2's guarded native source |
| `inputs/` | Unchanged pc2 baseline archive, manipulation port patch, hash lists and launcher, imported with provenance |
| `Dockerfile`, `package/` | pc2 graph image (sensor sender, relay, execution owner), smoke check, private zenoh router |
| `provision.sh` | Fingerprint checks, native stage/build/tests, model copy, image build, smoke check |
| `launch.sh` | `--check-only` (passive) or `--commission operator-ready-YYYYMMDD native\|graph` |
| `stop.sh` | Stops only the task graph container; the native controller uses the operator shutdown |
| `x86-port-sim/` | x86 evidence for the ported source |

## Native port

The base is pc2's own source: `a9d20b2` plus local changes. The manipulation session's
guarded port is applied on top; that is the build that ran smoothly on this robot on
10-14 with clocks pinned. The navigation delta is then ported onto it. It differs from
the workstation delta only where pc2 differs: there is no VR3PT filter, so operator
stop omits that term, and the planner-guard telemetry is added to pc2's `pack_state`.

On x86 the port builds. Native tests pass 41/42; only the unchanged FK fixture test
fails. Three MuJoCo Stage A matrices with the port binary pass all 26 cases each. The
strict noise-attribution audit passed in 2 of 3 runs: one unmatched acknowledgment,
the same intermittent miss Task 5 recorded once.

The native command is the pc2-proven launcher's command plus `--guarded-planner`. It
keeps `--disable-dex3-hands`, the 200 ms reference timeout, CSV logs and the
`jetson_clocks` preflight that resolved the 10-14 waist incident.

## Sequence on pc2

1. Copy `build/sonic-navigation-pc2/payload/` to `<task>/package/`, then run
   `bash provision.sh`. It refuses an existing task directory or image. It checks
   before and after: the 1,232 stored fingerprints, pc2's 71 original native source
   files and both installed controller binaries. It also snapshots processes.
2. `bash launch.sh --check-only` is passive. It verifies hashes and runs the
   reader-only probe, which checks LiDAR, IMU and lowstate progress, the 29 joints,
   live sensor conversions, clock pinning, existing SONIC processes, task ports,
   `rt/lowcmd` traffic and writers, and host load.
3. Commissioning is Task 11. It needs fresh standing, clear-area, stop and
   sole-controller readiness, plus the other session's release of the robot:
   `sudo jetson_clocks`, then `launch.sh --commission operator-ready-<today> native`
   in one terminal, `... start` once in another (the operator start: planner IDLE),
   then `... graph`. The graph owns the reference endpoint, so `start` refuses while
   it runs; the 14:30 CST session started the graph first and the native waited.
   - The native component refuses unless the probe passes. The only tolerated issue is
     the uncommissioned extrinsic, because the peer refuses goals without it.
   - `--commission` starts real motor control.

## Task 11 preparation (2026-10-15)

- `launch.sh --commission … graph` now refuses while any SONIC controller other than
  this task's binary runs. The manipulation launcher subscribes to `127.0.0.1:5556`
  by default, which the execution owner binds. That controller was running on pc2 at
  about 13:20 CST.
- Static mount check (`demo_pc2_extrinsic_check.py`, reader-only, three repeats in
  [pc2-provision/…attempt4…/checks/extrinsic](/docs/artifacts/g1_navigation_sonic_20261013/pc2-package/pc2-provision/sonic-navigation-20261013.attempt4-strict-lowstate-probe/checks/extrinsic)).
  It compares the nominal URDF mount (`torso_to_mid360`) with pelvis IMU gravity
  through the live waist chain:
  - Mid-360 accelerometer: pitch −1.08 to −1.10°, roll 0.02°.
  - LiDAR floor plane: pitch −0.69 to −0.70°, roll −0.22°.
  - Floor against the Mid-360 IMU (mount-independent): 0.47°.
  - The upside-down roll and the URDF pitch are confirmed. The remaining ~1° pitch
    offset can come from the mount, pelvis IMU bias (it reads 9.65 m/s²) or the waist
    encoders; the static check cannot separate them. It moves base x/y by about
    7 mm (0.42 m lever) and does not change heading.
  - Yaw is checked dynamically by the harness's forward case.
- Walk harness: `demo_pc2_navigation_walk.py` on the workstation, plus
  `demo_pc2_walk_agent.py`, which ships in `package/` and attaches to the graph
  container. It offers bounded named cases in plan order.

## Pending

Commissioning blockers:

- The exact FK fixture.
- Fresh operator readiness and the manipulation session's release of the robot. The
  operator runs the deployment commands: `sudo jetson_clocks`, then
  `launch.sh --commission operator-ready-<today> native` and `... graph`, and the
  workstation harness `demo_pc2_navigation_walk.py --operator-ready operator-ready-<today>
  --output <artifact>/hardware/<session>`.
