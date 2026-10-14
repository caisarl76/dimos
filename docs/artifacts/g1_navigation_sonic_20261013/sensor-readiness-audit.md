# Sensor readiness and resumed session

Recovered from session `01a11f87-0f25-7182-a8a2-b9f2fd212d59` after remote
compaction failed. The active branch is `codex/feat/g1-sonic-navigation` in
`/tmp/dimos-g1-sonic-navigation-20261013`. Tasks 1-5 and their commits were
preserved; the first unfinished task was the source-stamp readiness monitor.

The monitor expires each input only from the local monotonic time when its
original sensor stamp advances. Duplicate arrivals cannot extend that time.
It checks derived stamps against raw ancestors in an explicitly known common
clock domain. Stamp rollback latches the chain unready until a new peer instance;
empty/invalid outputs cannot establish a source-stamp high-water mark. Static
extrinsic calibration is a separate prerequisite and remains unverified.

## Pinned source audit

PointLIO core is pinned to `3c1dcf8b9dfe6402aaf141e116f4caef6c5c8088`.
The existing pc2 task image retains that exact Cargo checkout; its source was
copied for a read-only audit. `PointLio::process()` calls `run_once()`, which
requires a synchronized LiDAR scan before fresh processing. With the default
`publish_odometry_without_downsample=false`, estimator time comes from the scan
end, not the host wall clock. The wrapper drains scans and publishes only after
fresh processing; odometry, TF and cloud share `o.ts`, and rate limiting uses
that estimator time. Reprocessing a duplicate cannot advance those output stamps.

RayTracingVoxelMap passes the consumed cloud header stamp into its global/local
maps. Nonempty CostMapper output retains that map stamp. The existing empty-map
branch may assign a default timestamp, so content must reject it for readiness;
shared mapper implementation is unchanged.

## Offline native proof on pc2

The existing navigation image ran only PointLIO, ray mapping and CostMapper in
a network-disabled container with a private loopback Zenoh router. There were
no DDS readers/writers, goals, velocity publishers or motor controllers.

The saved fixture contains 2,000 IMU samples, 100 raw-cloud timestamps and one
20,064-point raw cloud. The replay explicitly repeats that one cloud at the
recorded timestamps and extends the final IMU times to drain scans. This checks
timestamp propagation and idle behavior; it does not reproduce the original
sequence of spatial scans or commission the sensor-to-base extrinsic.

All recorded assertions passed: 96 advancing cloud, odometry, TF and global-map
outputs, and 83 costmaps. TF stamps match odometry; map stamps match consumed
LIO clouds; costmap stamps match consumed maps. Idle time, exact duplicate raw
inputs and fresh IMU-only input produced no additional output. Adapter checks
preserved header stamps, XYZ, Livox offsets and acceleration conversion.

The container exited successfully and removed itself. All 1,232 stored pc2
environment fingerprints matched afterward. The original manipulation workspace
and its 20 imported source hashes were unchanged. Results, raw output stamps,
image/native/source hashes and replay limits are in
[the replay manifest](stamp-replay/manifest.json) and
[the measured results](stamp-replay/results/results.json).

Host preparation can proceed to the two-host link after the readiness task's
tests and selective commit. Hardware commissioning remains blocked by the
missing exact FK regression fixture, run02's unexplained reverse displacement
and run08's stopping failure. No walking result is claimed by this replay.
