import hashlib
import json
from pathlib import Path

import pytest

from scripts.prepare_g1_sonic_navigation_runtime import prepare


@pytest.fixture
def source(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "shared"
    project = root / "src/g1/g1_deploy_onnx_ref"
    project.mkdir(parents=True)
    (project / "CMakeLists.txt").write_text("baseline\n")
    (root / "target").mkdir()
    (root / "target/ignored").write_text("generated binary")
    baseline = tmp_path / "baseline.json"
    baseline.write_text(
        json.dumps({"files_sha256": {"CMakeLists.txt": hashlib.sha256(b"baseline\n").hexdigest()}})
    )
    return root, baseline


def test_snapshot_preserves_source_and_excludes_generated_output(
    source: tuple[Path, Path], tmp_path: Path
) -> None:
    root, manifest = source
    destination = tmp_path / "private"
    evidence = tmp_path / "snapshot.json"
    prepare(root, destination, manifest, evidence)
    assert (root / "src/g1/g1_deploy_onnx_ref/CMakeLists.txt").read_text() == "baseline\n"
    assert not (destination / "target").exists()
    assert json.loads(evidence.read_text())["files_sha256"]


def test_changed_baseline_refuses_before_creating_destination(
    source: tuple[Path, Path], tmp_path: Path
) -> None:
    root, manifest = source
    (root / "src/g1/g1_deploy_onnx_ref/CMakeLists.txt").write_text("changed\n")
    with pytest.raises(ValueError, match="baseline changed"):
        prepare(root, tmp_path / "private", manifest, tmp_path / "evidence.json")
    assert not (tmp_path / "private").exists()


def test_uninspected_native_file_refuses_snapshot(
    source: tuple[Path, Path], tmp_path: Path
) -> None:
    root, manifest = source
    (root / "src/g1/g1_deploy_onnx_ref/new.hpp").write_text("new native logic")
    with pytest.raises(ValueError, match="baseline changed"):
        prepare(root, tmp_path / "private", manifest, tmp_path / "evidence.json")


def test_evidence_cannot_modify_shared_source(source: tuple[Path, Path], tmp_path: Path) -> None:
    root, manifest = source
    with pytest.raises(ValueError, match="Evidence"):
        prepare(root, tmp_path / "private", manifest, root / "evidence.json")


def test_existing_destination_is_never_overwritten(
    source: tuple[Path, Path], tmp_path: Path
) -> None:
    root, manifest = source
    destination = tmp_path / "private"
    destination.mkdir()
    with pytest.raises(ValueError, match="Destination"):
        prepare(root, destination, manifest, tmp_path / "evidence.json")
