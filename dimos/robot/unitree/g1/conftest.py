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

"""Two scripted SONIC executions, one per robot, for the multi-robot arm tests."""

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import pytest

from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile
from dimos.robot.unitree.g1.test_sonic_execution_module import ScriptedPeer
from dimos.utils.testing.waiting import wait_until

# test_sonic_arm_skill's execution profile: an observation age covering the plan, and a tick gap
# that tolerates the ~170 ms event-loop stalls of everything sharing one process.
ARM_PROFILE = SonicModelProfile(
    name="scripted-peer",
    source_fps=50,
    right_hand_order="native",
    handback_policy="native_planner",
    seed_seconds=0.06,
    blend_seconds=0.04,
    first_chunk_timeout=1.0,
    handback_timeout=0.4,
    max_observation_age=6.0,
    max_tick_gap=0.19,
)


@contextmanager
def scripted_execution(journal_dir: Path) -> Iterator[tuple[SonicExecutionModule, ScriptedPeer]]:
    """A started SonicExecutionModule against its own ScriptedPeer, ready for a task."""
    peer = ScriptedPeer()
    peer.thread.start()
    try:
        assert peer.ready.wait(5) and peer.error is None
        module = SonicExecutionModule(
            reference_bind="tcp://127.0.0.1:*",
            feedback_endpoint=peer.feedback_endpoint,
            journal_dir=journal_dir,
            profile=ARM_PROFILE,
        )
        try:
            module.start()
            peer.reference_endpoint = module.get_status()["reference_endpoint"]
            peer.connected.set()
            wait_until(lambda: module.get_status()["ready"], timeout=3)
            yield module, peer
        finally:
            module.stop()
    finally:
        peer.stopping.set()
        peer.connected.set()
        peer.thread.join(timeout=3)
        assert not peer.thread.is_alive() and peer.error is None


@pytest.fixture
def dual_executions(tmp_path):
    """Robot 222's and robot 223's (execution, peer)."""
    with (
        scripted_execution(tmp_path / "222") as first,
        scripted_execution(tmp_path / "223") as second,
    ):
        yield first, second
