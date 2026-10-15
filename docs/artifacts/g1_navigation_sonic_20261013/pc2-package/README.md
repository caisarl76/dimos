# pc2 SONIC navigation package (Task 10)

Target: pc2 `unitree@192.168.0.223` (29 body joints, no hands). Everything goes under
`/home/unitree/dimos-navigation-verify-20261009/sonic-navigation-20261013/`. Existing
controllers, source trees, Python environments and images stay unchanged.

**Status, 2026-10-15:** provisioned on pc2 at the third attempt; evidence in
[pc2-provision/summary.json](/docs/artifacts/g1_navigation_sonic_20261013/pc2-package/pc2-provision/summary.json).

- The ARM64 build passes native tests 41/42; only the FK fixture test fails. The
  controller SHA256 is `dc59a4ab…`, identical on all three builds.
- Image `sha256:6a27e211…` (arm64) was built, and its network-disabled smoke check
  passed: graph validation and the planner codec.
- Before and after the final run, the 1,232 fingerprints, the 71 original native
  source files and both shared controller binaries were unchanged.
- Attempts 1 and 2 stopped at the smoke check: `zmq` was missing, then numba had no
  cache. Both are fixed in the package; their directories are kept as `.attemptN-*`.
- The manipulation session was running its own controller and hold-reference tests
  while attempts 2 and 3 ran, so provisioning load overlapped live robot control.
- `launch.sh --check-only` has **not run**. It is paused until the robot's use is
  coordinated with that session.

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
   in one terminal and `... graph` in another.
   - The native component refuses unless the probe passes. The only tolerated issue is
     the uncommissioned extrinsic, because the peer refuses goals without it.
   - `--commission` starts real motor control.

## Pending

The passive readiness report (`launch.sh --check-only`) still needs pc2. The
sensor-to-base extrinsic, the exact FK fixture and fresh operator readiness remain
commissioning blockers.
