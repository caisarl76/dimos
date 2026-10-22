# G1 SONIC navigation deployment

DimOS navigation through one guarded SONIC controller on a Unitree G1, split over two
hosts:

- **pc2** (`unitree@192.168.0.223`, the robot's Jetson): the native SONIC controller and a
  container running the pc2 graph (sensor sender, relay, execution owner).
- **The workstation**: the peer graph (sensor receiver, PointLIO, ray map, costmap,
  replanning A*, MovementManager, peer module), the walk harness and the viewer.

Verification history, robot sessions and simulation evidence are in
[docs/artifacts/g1_navigation_sonic_20261013](/docs/artifacts/g1_navigation_sonic_20261013/)
and the [verification report](/docs/platforms/humanoid/g1/navigation-verification-2026-10-09.md).
Provisioning attempts 1-12 are recorded in
[pc2-provision/summary.json](/docs/artifacts/g1_navigation_sonic_20261013/pc2-package/pc2-provision/summary.json);
attempt 12 (2026-10-22, commit `7320a98e1`) is the installed one.

## Layout

| Path | Purpose |
| --- | --- |
| `pc2/assemble.py` | Rebuilds the pc2 payload from committed inputs into `build/sonic-navigation-pc2/payload`, writes `pc2/manifest.json` |
| `pc2/inputs/`, `pc2/native-navigation-pc2.patch` | pc2's baseline native source, the manipulation port and the navigation guard port, with provenance |
| `pc2/Dockerfile`, `pc2/package/` | The pc2 graph image: graph entry, configs, smoke check, private zenoh router |
| `pc2/provision.sh` | On pc2: fingerprint checks, native build and tests, model copy, image build, smoke check |
| `pc2/launch.sh` | On pc2: `--check-only`, or `--commission operator-ready-<today> native\|start\|graph\|graph-agentic` |
| `pc2/stop.sh` | Stops only the task's graph container |
| `pc2_tools/` | Shipped in the pc2 image: `readiness_probe.py` (`--check-only`), `operator_start.py` (`start`), `walk_agent.py` (attached by the walk harness), and `extrinsic_check.py`, the static mount check |
| `workstation/peer_host.py` | The peer graph with its private zenoh router, started by the walk harness |
| `workstation/walk_harness.py` | The supervised robot session: bounded named cases, evidence in `results.json` |
| `workstation/rerun_view.py` | Live Rerun web view of the peer: costmaps, voxel map, robot marker, path |
| `workstation/extrinsic-acceptance.json` | The operator-accepted torso-to-MID360 mount |

## Provision pc2

Run when the code changes; the robot must be idle (no SONIC process, no task container).

1. On the workstation, from a clean checkout:
   `python dimos/robot/unitree/g1/deploy/sonic_navigation/pc2/assemble.py`.
2. On pc2, set the previous install aside as
   `sonic-navigation-20261013.attemptN-<why>`, remove the image
   `dimos-g1-sonic-navigation:20261013`, and copy the payload to
   `/home/unitree/dimos-navigation-verify-20261009/sonic-navigation-20261013/package/`.
3. On pc2, in `package/`: `bash provision.sh`. It refuses an existing task directory or
   image, and checks the 1,232 stored environment fingerprints, pc2's 71 original
   native source files and the shared controllers before and after.
4. `bash launch.sh --check-only` must report ready with nothing blocking.

## Run a session

The operator runs every pc2 command. Each needs fresh standing, clear-area, stop and
sole-controller readiness, and today's token. `--commission` starts real motor
control.

On pc2, in `package/`, three terminals, in this order:

```bash
sudo jetson_clocks
bash launch.sh --commission operator-ready-<today> native   # the SONIC controller
bash launch.sh --commission operator-ready-<today> start    # once: the operator start, planner IDLE
bash launch.sh --commission operator-ready-<today> graph    # or graph-agentic, see below
```

On the workstation, from the repository root:

```bash
PYTHONPATH=$PWD .venv/bin/python dimos/robot/unitree/g1/deploy/sonic_navigation/workstation/walk_harness.py \
  --operator-ready operator-ready-<today> \
  --output docs/artifacts/g1_navigation_sonic_20261013/hardware/session-<date>-<n> \
  [--prior-session <earlier session today>] [--ignore-rear-occupancy]
```

- The walk harness starts the peer graph and attaches the walk agent to the pc2
  container. A case is offered once its prerequisites passed today.
- `--ignore-rear-occupancy` leaves the support crane behind the robot out of the goal
  pre-check. It is for supported experiments only.
- The MID360 cannot see the floor within about 1 m of the robot, so whatever passes
  through there (the operator setting up a box) stays in the map. The goal cases ask
  the operator to confirm the area clear and to stand at least 1 m to the side of the
  path, then refresh the map: everything 0.12 m above the mapped floor within 2.5 m
  is cleared, and live scans rebuild what is still visible within 2 s. The
  `refresh_map` case does the same on its own. A real object nearer than about 0.7 m
  is not seen again after a refresh.
- A goal case ends after 3 s when the planner found no path to the goal.

Viewer, in another terminal, once the harness shows its menu:

```bash
PYTHONPATH=$PWD .venv/bin/python dimos/robot/unitree/g1/deploy/sonic_navigation/workstation/rerun_view.py
```

Open the printed URL, `http://localhost:9878/?url=rerun%2Bhttp://localhost:9877/proxy`.
Restart the viewer after each new harness session.

The workstation peer needs the PointLIO and ray-map native binaries at
`dimos/hardware/sensors/lidar/pointlio/rust/result/bin/pointlio_native` and
`dimos/mapping/ray_tracing/rust/result/bin/voxel_ray_tracing`. They are built by nix. On
a host without nix, build both with a cargo of 1.78 or newer (`cargo build --release`
in each `rust/` directory) and link the binaries from `target/release` there.

## Agent skills: `graph-agentic`

`graph-agentic` starts the same graph plus `SonicNavigationSkills` and an MCP server on
pc2's LAN address (`192.168.0.223:9990`, `pc2/package/config/pc2-graph-agentic.json`).
Use it in place of `graph`.

- The skills are `walk_to(forward_m, left_m)`, `stop_walking()` and
  `navigation_status()`.
- They act only inside a navigation request the operator began, and never begin one
  themselves.
- The validator refuses an MCP server bound to all interfaces. `--check-only` checks
  that port 9990 is free.

Once the walk harness has begun a request, start the agent on the workstation:

```bash
MCPCLIENT__MCP_SERVER_URL=http://192.168.0.223:9990/mcp dimos run unitree-g1-sonic-navigation-agent
```

## Safety notes

- `launch.sh --commission … graph` refuses while any SONIC controller other than this
  task's binary runs: other controllers subscribe to `127.0.0.1:5556`, which the
  execution owner binds.
- `start` refuses while the graph container exists.
- Navigation commands are bounded by the commissioned envelope: 0.2 m/s walking,
  0.2 rad/s turning and a 0.2 rad facing lead, enforced on pc2 and in the peer
  validator.
- The operator's native stop stays live in every session.
