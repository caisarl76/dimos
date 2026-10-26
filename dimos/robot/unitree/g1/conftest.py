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

import asyncio
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import shutil
import tempfile

import pytest

from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp import mcp_server
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile
from dimos.robot.unitree.g1.sonic_test_support import ARM_PROFILE, ScriptedPeer, scripted_peer
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


@pytest.fixture
def mcp(real_launch_skill, tool_transport, monkeypatch):
    """McpServer's tools/call and tool-stream handlers over the requesting test module's arm
    module (its real_launch_skill fixture), with a fresh registry."""
    module, _ = real_launch_skill
    skills = {s.func_name: s for s in module.get_skills()}
    registry = CapabilityRegistry()
    monkeypatch.setattr(mcp_server.app.state, "skills_by_name", skills)
    monkeypatch.setattr(mcp_server.app.state, "cap_registry", registry)
    monkeypatch.setattr(mcp_server.app.state, "event_loop", None)
    # Every stop frame reaches the server's handler, which releases the hold it carries.
    tool_transport.publish.side_effect = mcp_server._fan_out_to_sse_queues
    rpc_calls = {name: getattr(module, name) for name in skills}

    def call(name, /, **arguments):
        params = {"name": name, "arguments": arguments}
        request = {"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": params}
        response = asyncio.run(mcp_server.handle_request(request, list(skills.values()), rpc_calls))
        result = response["result"]
        return result["content"][0]["text"], result.get("isError", False)

    return call, registry


@contextmanager
def scripted_execution(journal_dir: Path) -> Iterator[tuple[SonicExecutionModule, ScriptedPeer]]:
    """A started SonicExecutionModule against its own ScriptedPeer, ready for a task."""
    with scripted_peer() as peer:
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


@pytest.fixture
def dual_executions(tmp_path):
    """Robot 222's and robot 223's (execution, peer), for the multi-robot arm tests."""
    with (
        scripted_execution(tmp_path / "222") as first,
        scripted_execution(tmp_path / "223") as second,
    ):
        yield first, second
