# FK fixture investigation: 2026-10-14

The exact legacy FK regression remains **blocked**. No authentic `bones_072925_test` fixture was located, so no fixture was manufactured, aliased, or substituted and no FK source, assertions, or tolerances were changed.

The unchanged upstream test loads `reference/bones_072925_test/`, constructs `RobotFK("g1/g1_29dof.xml")`, and compares recomputed poses and velocities with the recording in the first loaded clip. It does not check the loader result and does not select a named clip; the reader uses filesystem directory order. The converter's usage text identifies the expected input as `bones_072925_test.pkl`. Neither the test nor converter publishes an input checksum or identifies that first clip.

**Root cause and provenance.** The test has been byte-identical since NVIDIA's initial SONIC commit `2a5e6a1b55d9d0205a96e757b86007ec020cf589` (2026-02-19), including the stale fixture path. The complete local Git object-name history contains neither the fixture directory nor pickle. The official untruncated GitHub tree at `45c03facbf848e0c743091b7d9c17952096ff908` also lacks it; its fixture-path commit query and release list are empty. This demonstrates an upstream missing-test-data packaging dependency, but does **not** prove that renaming `reference/example` is a valid repair. Sources: [original FK test](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/2a5e6a1b55d9d0205a96e757b86007ec020cf589/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/unit_tests/test_fk.cpp), [converter](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/2a5e6a1b55d9d0205a96e757b86007ec020cf589/gear_sonic_deploy/reference/convert_motions.py), [official tree](https://api.github.com/repos/NVlabs/GR00T-WholeBodyControl/git/trees/45c03facbf848e0c743091b7d9c17952096ff908?recursive=1).

The [NVIDIA model manifest](https://huggingface.co/api/models/nvidia/GEAR-SONIC) at `6733128a3d8a523b1418b06bca3cdf61c8b0987f` has no matching artifact. The [BONES dataset manifest](https://huggingface.co/api/datasets/bones-studio/seed) at `2f59b2077b9da34dd4e43618e705c7cb962c9a66` exposes general motion archives, without a separately identified legacy regression fixture; archive contents were not downloaded or audited. A local name search covered 80,992 directories under `~/work`, the Hugging Face cache, and `/tmp`, with dependency directories pruned and symlinks unfollowed; it found only unrelated Rerun snapshot filenames. Permission errors and the interrupted broader `/mnt` search are recorded, so this is not a claim that no copy exists anywhere.

**Fresh reproduction.** From `build/sonic-navigation-runtime/source/gear_sonic_deploy`:

```bash
target/release/run_tests --gtest_filter=FK.*
```

Exit **1**, 0/1 tests passed. The loader reports the missing fixture directory, followed by `Couldn't parse xml file g1/g1_29dof.xml`. Both relative paths are absent in this private runtime. The XML itself is available in the original official checkout, so that is a separate resolvable staging omission; copying it alone cannot restore the missing motion recording. The original test would then access `motions[0]` after an unchecked failed load, so this investigation did not stage the XML alone.

The existing example-substitution diagnostics show **396 failures in both baseline and private builds** (148 quaternion assertions at line 72 and 248 angular-velocity assertions at line 85). The first loaded clip was `neutral_kick_R_001__A543_M`. Those logs establish parity on that attempted substitute, not recovery of the original regression. The FK test, implementation, header, and motion reader in the private runtime match the baseline; all **51/51 original native source hashes** were freshly verified unchanged.

**Required remedy.** Obtain the original `bones_072925_test.pkl` or its exported CSV tree from its publisher with a content manifest and intended clip/order, then stage it with the matching XML in the private runtime and rerun the unchanged test and full suite. There is no evidence-backed path-only fix available from the published files checked here. A guarded loader failure (`ASSERT_TRUE(ReadFromCSV(...))` and a nonempty-motion assertion) could improve the upstream diagnostic, but would not close this gate; it was not applied. A new independently defined FK fixture would need explicit acceptance as replacement coverage and must not be reported as the recovered legacy regression.

Evidence is in [fk-resolution/](/docs/artifacts/g1_navigation_sonic_20261013/fk-resolution): fresh test output, source integrity, local search coverage, and compact official GitHub/Hugging Face metadata. No robot, simulator, viewer, or GPU job was launched.

**Packaging proposal for review, not an implemented fix.** Preserve the legacy test and its failure status. Package its external inputs through an explicit fixture-root setting, require both assets before invocation, verify publisher-provided hashes, and fail with the missing artifact name before indexing an empty motion list. Once the publisher identifies the intended clip, make that selection deterministic instead of relying on directory iteration. CI should report the legacy gate as unavailable/blocked when inputs are absent, never count an exclusion or skip as full FK success. A manifest must pin the dataset/export revision, XML hash, selected clip, body/joint ordering, 50 Hz sampling, expected quaternion representation, velocity preprocessing, and the existing tolerances. These details are necessary because this test compares quaternion components directly and compares filtered velocities with separately recorded data.

An independent hermetic check against analytic poses or MuJoCo and independently defined velocity references could test useful mathematical properties, but cannot currently be certified to satisfy **the same recorded-data contract**: the legacy dataset and intended preprocessing/clip are unknown. Therefore no such replacement is proposed as resolution of this blocker. In particular, the existing example failures include opposite-sign quaternion components representing potentially equivalent orientations, as well as angular-velocity discrepancies; changing quaternion equivalence rules or the velocity oracle would alter the contract and requires a separate reviewed decision. No test additions or packaging changes were made in this investigation.

## Follow-up: official resource instructions

The official-resource review found a **supported export pipeline that the installed checkout lacks**, but still no download URL or source-motion identity for the exact legacy fixture. This is an unresolved publisher-provenance question; the evidence does not establish that the resource is definitely private.

The [motion-reference guide](https://nvlabs.github.io/GR00T-WholeBodyControl/references/motion_reference.html#provided-conversion-script) documents a deploy-format dictionary containing joint poses/velocities, body world poses/velocities, body indexes, and frame count. Its converter accepts an already prepared dictionary; it does not retrieve a dataset. The [published converter](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/45c03facbf848e0c743091b7d9c17952096ff908/gear_sonic_deploy/reference/convert_motions.py) names `bones_072925_test.pkl` only in command examples.

NVIDIA's [download guide](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/download_models.html#sample-motion-data-quick-start) identifies the downloadable walking sample as `sample_data/robot_filtered/210531/walk_forward_amateur_001__A001.pkl` and its mirrored counterpart. Its [canonical model manifest](https://huggingface.co/nvidia/GEAR-SONIC/blob/6733128a3d8a523b1418b06bca3cdf61c8b0987f/config.json) lists the sample prefix and seven SMPL archive parts, with no legacy FK entry. The manifest was downloaded and hashed; motion payloads were not downloaded.

The [training-data page](https://nvlabs.github.io/GR00T-WholeBodyControl/user_guide/training_data.html) points to [BONES-SEED](https://huggingface.co/datasets/bones-studio/seed). Its `g1.tar.gz` is a 23,499,973,647-byte archive of general G1 trajectories, not an identified copy of `bones_072925_test.pkl`. The [dataset card](https://huggingface.co/datasets/bones-studio/seed/blob/2f59b2077b9da34dd4e43618e705c7cb962c9a66/README.md) describes `g1/csv/{date}/{motion_name}.csv`; no mapping from that corpus to the exact legacy test subset was found. The archive was not downloaded or exhaustively inspected.

A collaborator's [answer to issue 117](https://github.com/NVlabs/GR00T-WholeBodyControl/issues/117#issuecomment-4316694406) points to `convert_soma_csv_to_motion_lib.py`. That produces the training representation, not the deploy dictionary. A later [collaborator answer to issue 241](https://github.com/NVlabs/GR00T-WholeBodyControl/issues/241#issuecomment-5957798919) identifies the missing bridge: `gear_sonic/data_process/export_motion_to_deploy.py`. The exporter is present in official `main` at `45c03fac...` ([source](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/45c03facbf848e0c743091b7d9c17952096ff908/gear_sonic/data_process/export_motion_to_deploy.py), blob `61b216dd4250a17af0faec4ee1b3520a5d59b9e7`). It loads `MotionLibRobot`, uses its computed arrays at 50 Hz, selects the 14 deploy bodies, and calls the existing CSV writer. Its default XML is `g1_29dof_rev_1_0.xml`, whereas the legacy test names `g1/g1_29dof.xml`. The official exporter source was fetched only to `/tmp` for inspection; no shared checkout or runtime code was changed.

For a **new reference motion** in an official checkout containing this exporter, the documented resource path can be used as follows. These commands were not executed and would not recover or replace the legacy fixture:

```bash
hf download nvidia/GEAR-SONIC \
  sample_data/robot_filtered/210531/walk_forward_amateur_001__A001.pkl \
  --revision 6733128a3d8a523b1418b06bca3cdf61c8b0987f \
  --local-dir /tmp/official-sonic-motion
python gear_sonic/data_process/export_motion_to_deploy.py \
  /tmp/official-sonic-motion/sample_data/robot_filtered/210531/walk_forward_amateur_001__A001.pkl \
  --output-dir /tmp/official-sonic-motion/deploy-walking \
  --target-fps 50 --device cpu
```

All eight current official branches were checked: seven through matching locally available commit objects, and current `main` through its official tree and fetched relevant sources. None supplied the named fixture path; resource-name references remained converter/help/test examples. The official tags and releases lists were empty. Relevant official issue responses describe generic conversion/export, without identifying the original regression clip or input checksum.

The remaining precise request to the publisher is: provide `bones_072925_test.pkl` or its exact exported CSV tree, the intended first clip, dataset/export revision, and matching XML hash for `FK.TestFKAndGlobalVelocities`. NVIDIA lists **gear-wbc@nvidia.com** on its [support page](https://nvlabs.github.io/GR00T-WholeBodyControl/resources/support.html). No message was sent. The supported exporter enables separate coverage work, but it does not establish the historical fixture's identity and the legacy gate remains blocked.

Structured URL/finding evidence is in `fk-resolution/official-resource-investigation.json`, alongside branch metadata, collaborator responses, exporter history, release manifest, and archive metadata. No FK test was rerun during the concurrent native rebuild.
