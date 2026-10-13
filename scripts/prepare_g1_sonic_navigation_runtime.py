#!/usr/bin/env python3
"""Snapshot the inspected SONIC source without modifying its shared checkout."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(
    source: Path, destination: Path, baseline: Path, evidence: Path, patch: Path | None = None
) -> None:
    source = source.resolve(strict=True)
    destination = destination.resolve()
    evidence = evidence.resolve()
    if destination.exists() or source == destination or source in destination.parents:
        raise ValueError("Destination must be a new directory outside the source tree")
    if evidence == source or source in evidence.parents:
        raise ValueError("Evidence must be written outside the shared source tree")
    manifest = json.loads(baseline.read_text())
    project = source / "src/g1/g1_deploy_onnx_ref"
    changed = [
        name
        for name, expected in manifest["files_sha256"].items()
        if not (project / name).is_file() or sha256(project / name) != expected
    ]
    actual = {
        str(path.relative_to(project))
        for path in project.rglob("*")
        if path.is_file()
        and (path.suffix in {".hpp", ".h", ".cpp", ".cmake"} or path.name == "CMakeLists.txt")
    }
    changed.extend(sorted(actual - manifest["files_sha256"].keys()))
    if changed:
        raise ValueError(f"Inspected native baseline changed: {changed}")
    ignored = shutil.ignore_patterns(".git", "target", "build", "__pycache__", "*.pyc")
    shutil.copytree(source, destination, ignore=ignored, symlinks=False)
    if patch is not None:
        subprocess.run(
            [
                "patch",
                "--batch",
                "--forward",
                "-p1",
                "--directory",
                str(destination / "src/g1/g1_deploy_onnx_ref"),
                "-i",
                str(patch.resolve(strict=True)),
            ],
            check=True,
        )
    snapshot = {
        str(path.relative_to(destination)): sha256(path)
        for path in sorted(destination.rglob("*"))
        if path.is_file()
    }
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_text(
        json.dumps(
            {
                "recorded_at": datetime.now(timezone.utc).isoformat(),
                "source": str(source),
                "private_destination": str(destination),
                "baseline_manifest_sha256": sha256(baseline),
                "patch_sha256": sha256(patch) if patch is not None else None,
                "scope": "Source snapshot only; build and model attestations follow separately",
                "files_sha256": snapshot,
            },
            indent=2,
        )
        + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--patch", type=Path)
    args = parser.parse_args()
    prepare(args.source, args.destination, args.baseline, args.evidence, args.patch)


if __name__ == "__main__":
    main()
