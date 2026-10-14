# eval_loop: Ivan's go2 PGO vs gsc_pgo

Runs both on the exact same scans and poses, at their default configs, and renders raw / Ivan / gsc
top-downs. `live_timing.py` replays a recording through Ivan's live module path to measure per-frame cost.

## Setup

```bash
# 1. gsc_pgo's offline runner, built from this branch
(cd dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust && nix develop path:. --command cargo build --release --features native --bin pgo-eval)

# 2. Ivan's PGO lives on main, so every python step runs with a main checkout first on the path
git worktree add --detach ../dimos_main origin/main
export PYTHONPATH=$PWD/../dimos_main
source .venv/bin/activate          # needs gtsam + open3d
cd dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_loop_results   # prepared dbs, maps, pngs land here
```

`PGO_EVAL=/path/to/pgo-eval` overrides the binary location.

## Run

`prep.py` turns a recording into what both PGOs take: a `lidar` stream of world-frame scans, each
stamped with the lidar's world pose (sensor-frame lidar is registered through odom + the recorded tf).

```bash
R=/path/to/mid360_recordings   # wherever the drive is mounted
python prep.py small_loop_go2     $R/2026-05-28_10-54am-PST__outdoor_small_loop/mem2.db  lidar          odom
python prep.py gir_stairs2_go2    $R/2026-06-02_10-03pm-PST__gir_stairs2/mem2.db         lidar          odom
python prep.py gir_park1_pointlio $R/2026-06-02_11-12pm-PST__gir_park1/mem2.db           pointlio_lidar pointlio_odometry
python prep.py huge_loop_pointlio $R/2026-06-04_12-57pm-PST__huge_loop_realsense/mem2.db pointlio_lidar pointlio_odometry
python prep.py huge_loop_fastlio  $R/2026-06-04_12-57pm-PST__huge_loop_realsense/mem2.db fastlio_lidar  fastlio_odometry

python bench.py  small_loop_go2 gir_stairs2_go2 gir_park1_pointlio huge_loop_pointlio huge_loop_fastlio
python render.py small_loop_go2 gir_stairs2_go2 gir_park1_pointlio huge_loop_pointlio huge_loop_fastlio
```

Go2 recordings that already have a pose-stamped `lidar` stream (Ivan's own set: `hk_village1`..`6`,
`go2_hongkong_office`) skip `prep.py`: `bench.py` falls back to `data/<name>.db` (pull with `get_data`).

Outputs per dataset: `<name>.summary.json` (loops, runtime, occupied 0.2 m voxels, max shift) and
`<name>.topdown.png`. Voxel % is a rough sharpness number; trust the pictures.

## Live cost on a long recording

```bash
python live_timing.py huge_loop_pointlio & ./watchdog.sh $! 32   # kill at 32 GB RSS or <12 GB free
python plot_timing.py huge_loop_pointlio                          # -> huge_loop_pointlio.live_timing.html
```

`live_timing.py` stops itself if one frame takes over 30 s and saves partial timings every 500 frames.
