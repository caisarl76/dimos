# G1 post-merge workstation recovery (2026-10-21)

This records recovery after merging upstream `36f54ee2c` into the arm-skill
and SONIC navigation branches. Both checkouts use
`/home/jihun/work/dimos_dev/.venv`; updating it affects both. No robot deployment
was performed.

## Shared Python environment

The installed RoboPlan was `0.6.0`, while `uv.lock` requires `0.7.0`.
The following locked sync completed in the arm-skill checkout:

```bash
uv sync --locked --inexact --group lint \
  --extra unitree --extra manipulation --extra sonic-feedback
```

Use `--inexact` to retain other sessions' additional installed packages.
`unitree` includes the agent/perception and mapping dependencies;
`manipulation` includes planning, control and simulation; `sonic-feedback`
provides the external SONIC wire dependencies. The default test group remains
enabled, and `lint` supplies mypy and the planning stubs/dependencies.
Do not use `--all-extras` for these workstation workflows: `unitree-dds`
and `dds` pull in CycloneDDS, which needs a separately provisioned native
library. Neither CycloneDDS nor the Unitree DDS SDK was installed here.
Real DDS hardware workflows still require that separate setup.

The sync replaced RoboPlan with `0.7.0` and `dimos-lcm` with `0.1.6`, rebuilt
the editable dimOS package, and installed their missing locked dependencies.
A repeated dry run reported **no changes**. Neither `pyproject.toml` nor
`uv.lock` needed editing. The three previously missing APIs are present in
`roboplan.core`: `SceneContext`, `loadUrdfSceneDescriptionFromXml`, and
`loadJointLimitsConfig`.

`uv pip check` still reports six metadata incompatibilities. Five correspond
to repository overrides (GTSAM/pytest, pytest-asyncio/pytest, the two consumers
of plain OpenCV, and moondream/Pillow); the sixth is missing `ipywidgets` for
the existing Open3D installation. This is not a claim that every installed
optional package is consistent. The selected locked workflow and its G1
checks are the validation boundary.

## Native modules without Nix

Use the existing Cargo **1.92.0** toolchain. System Cargo 1.75 cannot read the
v4 lockfiles. The following builds succeeded from the arm checkout, using the
existing Cargo download cache:

```bash
export PATH="$HOME/.rustup/toolchains/1.92.0-x86_64-unknown-linux-gnu/bin:$PATH"
cargo build --release --locked --offline \
  --manifest-path dimos/hardware/sensors/lidar/pointlio/rust/Cargo.toml \
  --bin pointlio_native
cargo build --release --locked --offline \
  --manifest-path dimos/mapping/ray_tracing/rust/Cargo.toml \
  --bin voxel_ray_tracing
```

On a machine without the cached dependencies, omit `--offline` to download
the locked sources. Run from the checkout root: `.cargo/config.toml` puts
the binaries in that checkout's `target/release`.

The arm checkout now has gitignored links from each source directory's
`result/bin/<executable>` to its own `target/release/<executable>`. They do
not depend on the navigation worktree. For a new checkout, create them after
building (these commands deliberately refuse to replace existing files):

```bash
mkdir -p dimos/hardware/sensors/lidar/pointlio/rust/result/bin
mkdir -p dimos/mapping/ray_tracing/rust/result/bin
ln -s "$PWD/target/release/pointlio_native" \
  dimos/hardware/sensors/lidar/pointlio/rust/result/bin/pointlio_native
ln -s "$PWD/target/release/voxel_ray_tracing" \
  dimos/mapping/ray_tracing/rust/result/bin/voxel_ray_tracing
```

Both Python wrappers passed `_prepare_native()` without invoking Nix.
`NativeModule._maybe_build()` accepts existing binaries when neither
`auto_build` nor the global `build_native` flag is enabled. Explicit rebuilds
still use the configured Nix command; rerun the Cargo commands manually after
source/lockfile changes. This workstation repair does not change upstream's
build defaults or install Nix.

## Split LFS storage

Retain the split storage. Do not change `.lfsconfig` to the fork until all
upstream objects have actually been mirrored. The fork returned HTTP 404 for
all five handoff objects: `data/.lfs/go2_l1_knock.db.tar.gz` and
`docs/platforms/safety/assets/r1pro-{overview,remote-modes,remote,switches}.png`.
An upstream fetch failed because Git has no credentials configured for
`https://lfs.dimensionalos.com`. GitHub authentication does not automatically
configure that separate host.

For a fresh fork checkout, skip automatic smudging, then fetch each required
asset from the server that owns it. For example, from the checkout root:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/caisarl76/dimos.git
cd dimos
git remote add origin-upstream https://github.com/dimensionalOS/dimos.git
git -c lfs.url=https://github.com/caisarl76/dimos.git/info/lfs \
  lfs fetch --include='path/to/required/branch-asset' --exclude='' origin HEAD
git lfs checkout path/to/required/branch-asset
git -c lfs.url=https://lfs.dimensionalos.com/dimensionalOS/dimos \
  lfs fetch --include='path/to/required/upstream-asset' --exclude='' origin-upstream HEAD
git lfs checkout path/to/required/upstream-asset
```

Select the desired branch before fetching its assets. Configure upstream-host
credentials through the normal credential manager before the upstream fetch.
`--exclude=''` overrides the workstation's global `lfs.fetchexclude=*` and the
repository's data exclusion. Use a command-scoped `-c lfs.url=...` for fork
uploads too; do not rely on `remote.fork.lfspushurl` overriding `.lfsconfig`.
Do not publish a supposedly complete mirror with `lfs.allowincompletepush`.

The fork did contain `cafe-smol.jpg.tar.gz`. It was fetched successfully,
extracted to `data/cafe-smol.jpg`, and copied into the offloaded archive
directory. SHA-256 of the archive:
`dd0c1e5aa5e8ec856cb471c5ed256c2d3a5633ed9a1e052291680eb86bf89a5e`.
The decoded image is 193 × 256, and the recorded MCP image test passes.

## Offloaded data and remaining integration work

`data/.lfs` remains a symlink to
`/mnt/data/jihun/dimos_dev_offload/data/.lfs`. The tracked-file deletions in
`git status` are an existing consequence of replacing a tracked directory
with a symlink; they were not staged or hidden with index flags. Do not
commit those deletions. Before a checkout/merge that touches this directory,
move the symlink aside, use `GIT_LFS_SKIP_SMUDGE=1`, preserve any resulting
pointer directory separately, then restore the symlink. Do not run the fresh
clone's `git lfs checkout` recipe through this directory symlink; materialize
needed archives in the offloaded directory instead.

The two-host physics settle gate remains an open validation issue. The
handoff's 84/85 result with `left_turn_settled` at 4.16 s exceeds the 3 s gate.
Earlier evidence also records yaw-rate jitter around the 0.03 rad/s limit.
No control thresholds, watchdogs, or pass criteria were relaxed here, and a
new two-host physics run was not performed. Passing Python regressions does
not establish that this physics timing issue is fixed.

The pc2 service migration remains owned by the separate deployment session:
its older generic `SkillResult` contract must be reconciled before deploying
the merged arm skill. The backup `stash@{0}` was retained.

## Verification

| Check | Result |
| --- | --- |
| Arm checkout: `mypy dimos/robot/unitree/g1` | Pass, 55 source files |
| Navigation checkout: same mypy command | Pass, 58 source files |
| Arm checkout: `pytest dimos/robot/unitree/g1 -q --tb=short` | 431 passed, 3 deselected |
| Navigation checkout: same pytest command | 568 passed, 3 deselected |
| `pytest dimos/agents/mcp/test_mcp_client.py::test_image -q --tb=short` | 1 passed |
| Locked release Cargo builds for both native binaries | Pass |
| Python wrappers' native preparation with existing binaries | Pass without Nix |
| Selected arm-checkout uv sync, repeated as a dry run | No changes |
| `git diff --check` | Pass |

Tests used the existing `.venv/bin` tools. Navigation checks ran from that
worktree with `PYTHONPATH=/tmp/dimos-g1-sonic-navigation-20261013`, so they
tested navigation source despite the shared editable install pointing at the
arm checkout. A navigation sync dry run proposed only rebinding that editable
install; no dependency changes. Its sync was not applied.

The first arm suite run, concurrent with release compilation and navigation
tests, had 430 passes and one `test_wrist_inside_its_envelope_passes` control
tick deadline failure. The individual test and then the entire arm suite
passed after those jobs finished. This demonstrates timing sensitivity under
load; it does not establish that deadline misses cannot recur.
