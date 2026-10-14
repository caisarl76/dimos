# Task 8: read-only sensor relay and graph validation

Software/headless verification only. No DDS hardware, pc2 launch or robot motion.

## Implemented

- `sonic_navigation_sensor_module.py`
  - **`SonicNavigationSensorSender` (pc2):** reads cloud, IMU and `rt/lowstate` through DDS readers only. It publishes two-part `[channel, LCM payload]` frames on separate unicast PUB sockets:
    - cloud socket: `lidar_raw`
    - state socket: `imu_raw`, `waist_tf`
    - SNDHWM 2, 16 MiB payload limit. Waist frames are relayed at 50 Hz and stamped with the lowstate millisecond tick.
  - **`SonicNavigationSensorReceiver` (workstation):** accepts only allowlisted channels per socket, enforces exactly two parts and the size limit, and rejects non-positive source stamps.
    - Cloud drains publish only the newest cloud; IMU keeps its order. Draining is bounded at 64 frames.
    - A socket disconnect drops queued data for 0.5 s, longer than every readiness budget, and clears the waist edge immediately.
    - Base odometry and the `odom→base_link` TF are composed from PointLio `odom→mid360_link`, the commissioned `torso_to_sensor` extrinsic and the relayed waist edge, carrying the LIO stamp. Without an extrinsic, or with a waist edge older than 200 ms (duplicate ticks don't refresh it), nothing is produced.
- **Conversion code:** promoted from `g1_navigation_20261012/demo_dds_lio_adapter.py` (SHA256 `e09185b7…a5ae`; the original file is unmodified). The only change: asserts became explicit `ValueError`s.
- **`sonic_navigation_graph.py`:** checks per-role module allowlists and duplicate owners. The resolved producer→consumer stream edges must match the role's expected set exactly, so a remapping cannot bypass it. Module-reference remapping is rejected. Endpoints must be explicit `tcp://host:port` with distinct ports. Safety settings are checked: zero safe radius, zero follower turn floor, `navigation_enabled`, and `extrinsic_valid` only together with an extrinsic.
- **Core hook:** a new `Blueprint.validators(...)`, merged by `autoconnect`. Both `ModuleCoordinator.build` and `load_blueprint` run it on the final graph and the resolved module kwargs, before system configuration, transports or workers. The registry generator now recognizes `.validators(...)`.
- **Blueprints:** `unitree-g1-sonic-navigation-peer` and `unitree-g1-sonic-navigation-pc2`.
  - In the peer blueprint, PointLio odometry is remapped to `lio_odometry` and CostMapper `merged_map` to a disabled name.
  - Peer globals: `n_workers=5`, `nerf_speed=0.2/0.55`, `simulation=""`.

## Results

- Task 8 tests: 9 sensor, 18 graph and 2 coordinator-hook tests pass. Nine targeted mutations (hold-off, cloud drain, waist refresh, stamp guard, waist budget, reset clearing, edge check, duplicates, endpoint ports) each fail at least one test.
- Focused SONIC/planner/coordinator suites: 578 passed. 1 failed: `test_module_reloading`, which fails identically with the HEAD core files, so it is pre-existing and environmental.
- Ruff, mypy (6 production files) and the pre-commit hooks pass. The registry is regenerated through its test.

## Open items carried forward

- The DDS reader path (`_open_readers`) imports cyclonedds/unitree_sdk2py lazily. These aren't in the workstation environment, so the sender is tested with injected samples and real ZMQ sockets only. Task 10 must exercise it on pc2.
- Readiness needs commissioning configuration that isn't in the registered blueprints: `torso_to_sensor`, peer `extrinsic_valid`, and `clock_domains`. All seven peer inputs carry Livox sensor stamps, but that domain assignment still needs verifying against real data. Until then the registered peer stays unready by design.
- The validator checks module kwargs, not GlobalConfig overrides (for example `nerf_speed`). It can't see LCM publishers outside the graph or native processes; runtime ownership inspection is still required.
