# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Assemble the pc2 payload reproducibly from committed inputs; write manifest.json.

Native source = pc2 baseline archive + manipulation port patch + navigation port
patch, verified at each step. DimOS source = `git archive` of a clean commit.
Run on the workstation; copy the payload directory to <task>/package on pc2.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from typing import Any

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
INPUTS = HERE / "inputs"
PORT_DIR = "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref"
SHARED_CONTROLLERS = {
    "/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref": "bc0b17c45d06e458b847c217ec62c9e1bbb8d7db66f6688750beceb5c773a3b5",
    "/home/unitree/dimos-sonic-guard-pc2-223-20261014/gear_sonic_deploy/target/release/g1_deploy_onnx_ref": "9c6ee5fbde7fc62121a35d62450919dc87125e727ec243afd179415d4b73cdfa",
}
DIMOS_PATHS = ["pyproject.toml", "uv.lock", "README.md", "LICENSE", "dimos"]
PACKAGE_FILES = ["Dockerfile", "provision.sh", "launch.sh", "stop.sh"]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_hashes(root: Path) -> dict[str, str]:
    return {
        str(p.relative_to(root)): sha(p)
        for p in sorted(root.rglob("*"))
        if p.is_file()
        and not p.is_symlink()
        and not {"target", "thirdparty"} & set(p.relative_to(root).parts)
    }


def patch(directory: Path, diff: Path) -> None:
    subprocess.run(
        ["patch", "-p1", "--forward", "--no-backup-if-mismatch", "--quiet", "-i", str(diff)],
        cwd=directory,
        check=True,
    )


def sums(rows: dict[str, str]) -> str:
    return "".join(f"{digest}  {name}\n" for name, digest in sorted(rows.items()))


def deterministic_tar(root: Path, names: list[str], destination: Path) -> None:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w", format=tarfile.PAX_FORMAT) as archive:
        for name in sorted(names):
            info = archive.gettarinfo(str(root / name), arcname=name)
            info.mtime, info.uid, info.gid, info.uname, info.gname = 0, 0, 0, "", ""
            with (root / name).open("rb") as stream:
                archive.addfile(info, stream)
    destination.write_bytes(buffer.getvalue())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/sonic-navigation-pc2/payload")
    parser.add_argument(
        "--compare-tree", type=Path, default=ROOT / "build/sonic-navigation-pc2/navigation"
    )
    args = parser.parse_args()
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    dirty = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", *DIMOS_PATHS], cwd=ROOT
    ).returncode
    if dirty:
        raise SystemExit("DimOS source differs from HEAD; commit before assembling")
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite {args.output}")
    out = args.output
    (out / "package/config").mkdir(parents=True)

    baseline = json.loads((INPUTS / "native-baseline.json").read_text())["sha256"]
    final = json.loads((INPUTS / "native-source-final.json").read_text())
    with tempfile.TemporaryDirectory() as scratch:
        tree = Path(scratch) / "native"
        tree.mkdir()
        with tarfile.open(INPUTS / "pc2-native-source.tar.gz") as archive:
            archive.extractall(tree, filter="data")
        assert {k: v for k, v in tree_hashes(tree).items() if k in baseline} == baseline, (
            "pc2 baseline"
        )
        patch(tree, INPUTS / "pc2-223-native.patch")
        assert {k: v for k, v in tree_hashes(tree).items() if k in final} == final, (
            "manipulation port"
        )
        patch(tree / PORT_DIR, HERE / "native-navigation-pc2.patch")
        native = tree_hashes(tree)
        if args.compare_tree.exists():
            assert native == tree_hashes(args.compare_tree), (
                "differs from the x86-verified port tree"
            )
        deterministic_tar(tree, list(native), out / "native-source.tar")

    (out / "native-source.sha256").write_text(sums(native))
    (out / "native-baseline.sha256").write_text(sums(baseline))
    assets = json.loads((INPUTS / "assets-manifest.json").read_text())
    (out / "models.sha256").write_text(sums({k: v["sha256"] for k, v in assets.items()}))
    (out / "shared-controllers.sha256").write_text(sums(SHARED_CONTROLLERS))
    shutil.copy(ROOT / "dimos/robot/unitree/g1/test_data/sonic_planner_lease_cases.json", out)
    with (out / "dimos-source.tar").open("wb") as stream:
        subprocess.run(
            ["git", "archive", commit, *DIMOS_PATHS], cwd=ROOT, stdout=stream, check=True
        )
    for name in PACKAGE_FILES:
        shutil.copy2(HERE / name, out / name)
    for path in sorted((HERE / "package").rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts:
            target = out / "package" / path.relative_to(HERE / "package")
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    for name in (
        "demo_pc2_navigation_readiness.py",
        "demo_pc2_walk_agent.py",
        "demo_pc2_operator_start.py",
    ):
        shutil.copy2(HERE.parent / name, out / "package")

    files = {str(p.relative_to(out)): sha(p) for p in sorted(out.rglob("*")) if p.is_file()}
    (out / "manifest.sha256").write_text(sums(files))
    manifest: dict[str, Any] = {
        "target": "pc2 unitree@192.168.0.223, /home/unitree/dimos-navigation-verify-20261009/sonic-navigation-20261013",
        "dimos_source_commit": commit,
        "native": {
            "baseline": "pc2 /home/unitree/GR00T-WholeBodyControl a9d20b2 + local changes (71 files)",
            "manipulation_port_patch_sha256": sha(INPUTS / "pc2-223-native.patch"),
            "navigation_port_patch_sha256": sha(HERE / "native-navigation-pc2.patch"),
            "source_files": len(native),
            "build": "host JetPack 6.2.1 toolchain, Release, -j2 nice 19, thirdparty -> original SDK",
            "expected_tests": "all pass except FK.TestFKAndGlobalVelocities (fixture absent)",
        },
        "models": {"files": len(assets), "source": "pc2's own original install, hash-verified"},
        "shared_controllers_unchanged": SHARED_CONTROLLERS,
        "base_image": {
            "tag": "dimos-g1-navigation-verify:20261012-pointlio",
            "id": "sha256:01980cefa285106c334c71d9dafb8db6b8ff24ded4d6539f6d91baaab56178b0",
        },
        "image": "dimos-g1-sonic-navigation:20261013",
        "payload_sha256": files,
        "inputs_sha256": {p.name: sha(p) for p in sorted(INPUTS.iterdir()) if p.is_file()},
    }
    (HERE / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        json.dumps(
            {
                "payload": str(out),
                "files": len(files),
                "commit": commit,
                "native_files": len(native),
            }
        )
    )


if __name__ == "__main__":
    main()
