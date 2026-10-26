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

from collections.abc import Iterator
from pathlib import Path
import shutil
import tempfile

import pytest

from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile
from dimos.robot.unitree.g1.sonic_test_support import ScriptedPeer
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def ipc_dir() -> Iterator[Path]:
    """A short directory for ipc:// sockets: macOS caps AF_UNIX paths at 103 bytes, and pytest's
    tmp_path there (/private/var/folders/...) leaves too little room for a socket name."""
    path = Path(tempfile.mkdtemp(prefix="sonic", dir="/tmp"))
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


@pytest.fixture
def peer():
    peer = ScriptedPeer()
    peer.thread.start()
    try:
        assert peer.ready.wait(5)
        assert peer.error is None
        yield peer
    finally:
        peer.stopping.set()
        peer.connected.set()
        peer.thread.join(timeout=3)
        assert not peer.thread.is_alive()
        assert peer.error is None


@pytest.fixture
def execution_config(peer, tmp_path):
    return dict(
        reference_bind="tcp://127.0.0.1:*",
        feedback_endpoint=peer.feedback_endpoint,
        journal_dir=tmp_path,
        profile=SonicModelProfile(
            name="scripted-peer",
            source_fps=50,
            right_hand_order="native",
            handback_policy="native_planner",
            seed_seconds=0.06,
            blend_seconds=0.04,
            first_chunk_timeout=1.0,
            handback_timeout=0.4,
        ),
    )


@pytest.fixture
def execution(peer, execution_config):
    module = SonicExecutionModule(**execution_config)
    try:
        module.start()
        peer.reference_endpoint = module.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: module.get_status()["ready"], timeout=3)
        yield module, peer
    finally:
        module.stop()


@pytest.fixture
def tool_transport(mocker):
    """The transport every ToolStream publishes on, mocked as in test_tool_stream."""
    transport = mocker.MagicMock()
    mocker.patch("dimos.agents.mcp.tool_stream.make_transport", return_value=transport)
    return transport
