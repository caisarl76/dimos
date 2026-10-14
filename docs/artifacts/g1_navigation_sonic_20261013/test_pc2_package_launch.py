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

"""Offline launcher gating with stubbed docker/pgrep/sha256sum and a fake sysfs."""

from datetime import date
import json
import os
from pathlib import Path
import subprocess

import pytest

LAUNCH = Path(__file__).with_name("pc2-package") / "launch.sh"
DOCKER = """#!/usr/bin/env bash
echo "docker $*" >> "$STUB_LOG"
if [[ "$*" == *demo_pc2_navigation_readiness.py* ]]; then
    out=$(printf '%s\\n' "$@" | sed -n 's/^type=bind,source=\\(.*\\),target=\\/verification$/\\1/p')
    printf '{"blocking": %s}\\n' "$STUB_BLOCKING" > "$out/readiness.json"
fi
"""


@pytest.fixture
def host(tmp_path, monkeypatch):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, body in {
        "docker": DOCKER,
        "pgrep": '#!/usr/bin/env bash\n[[ "${STUB_SONIC:-0}" == 1 ]]\n',
        "sha256sum": '#!/usr/bin/env bash\necho "sha256sum $*" >> "$STUB_LOG"\n',
    }.items():
        (bin_dir / name).write_text(body)
        (bin_dir / name).chmod(0o755)
    sysfs = tmp_path / "sys"
    for cpu in (0, 1):
        freq = sysfs / f"devices/system/cpu/cpu{cpu}/cpufreq"
        freq.mkdir(parents=True)
        (freq / "scaling_min_freq").write_text("1728000\n")
        (freq / "cpuinfo_max_freq").write_text("1728000\n")
    gpu = sysfs / "class/devfreq/17000000.gpu"
    gpu.mkdir(parents=True)
    (gpu / "min_freq").write_text("918000000\n")
    (gpu / "max_freq").write_text("918000000\n")
    task = tmp_path / "task"
    (task / "native/gear_sonic_deploy/target/release").mkdir(parents=True)
    binary = task / "native/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    binary.write_text(f"#!/usr/bin/env bash\necho native-started > {tmp_path}/native-started\n")
    binary.chmod(0o755)
    (task / "package").mkdir()
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "SONIC_NAV_TASK_ROOT": str(task),
        "SONIC_SYSFS_ROOT": str(sysfs),
        "STUB_LOG": str(tmp_path / "calls.log"),
        "STUB_BLOCKING": "[]",
    }
    return tmp_path, env


def run(env, *args, **extra):
    return subprocess.run(
        ["bash", str(LAUNCH), *args], env={**env, **extra}, capture_output=True, text=True
    )


def calls(root):
    path = root / "calls.log"
    return path.read_text() if path.exists() else ""


TOKEN = f"operator-ready-{date.today():%Y%m%d}"


@pytest.mark.parametrize(
    "args",
    [
        (),
        ("--commission",),
        ("--commission", "operator-ready-20000101", "native"),
        ("--commission", TOKEN, "both"),
        ("--check-only", "extra"),
        ("--run",),
    ],
)
def test_refuses_without_exact_mode_token_and_component(host, args) -> None:
    root, env = host
    assert run(env, *args).returncode == 2
    assert "docker" not in calls(root) and not (root / "native-started").exists()


def test_check_only_runs_reader_probe_and_reports_blocking(host) -> None:
    root, env = host
    assert run(env, "--check-only").returncode == 0
    probe = next(line for line in calls(root).splitlines() if "readiness" in line)
    assert "--network host" in probe and "--pid host" in probe and "--cap-drop ALL" in probe
    result = run(env, "--check-only", STUB_BLOCKING=json.dumps(["active_motor_commands"]))
    assert result.returncode == 1 and "active_motor_commands" in result.stdout
    assert not (root / "native-started").exists()


def test_native_commission_tolerates_only_the_uncommissioned_extrinsic(host) -> None:
    root, env = host
    blocked = run(env, "--commission", TOKEN, "native", STUB_BLOCKING='["sonic_process_running"]')
    assert blocked.returncode == 1 and not (root / "native-started").exists()
    started = run(
        env, "--commission", TOKEN, "native", STUB_BLOCKING='["extrinsic_uncommissioned"]'
    )
    assert started.returncode == 0 and (root / "native-started").exists()


def test_native_commission_refuses_unpinned_clocks_or_a_running_controller(host) -> None:
    root, env = host
    (Path(env["SONIC_SYSFS_ROOT"]) / "devices/system/cpu/cpu1/cpufreq/scaling_min_freq").write_text(
        "729600\n"
    )
    assert run(env, "--commission", TOKEN, "native").returncode == 1
    (Path(env["SONIC_SYSFS_ROOT"]) / "devices/system/cpu/cpu1/cpufreq/scaling_min_freq").write_text(
        "1728000\n"
    )
    assert run(env, "--commission", TOKEN, "native", STUB_SONIC="1").returncode == 1
    assert not (root / "native-started").exists()
