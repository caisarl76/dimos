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

"""Real local ZMQ contract tests; the peer simulates protocol state, not physics."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import numpy as np
import pytest
import zmq

from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_test_support import LEASE_STALL_RERUN, WAIT_S
from dimos.utils.testing.waiting import wait_until

pytestmark = LEASE_STALL_RERUN


@pytest.fixture
def worker_execution(peer, execution_config, dimos_cluster):
    module = dimos_cluster.deploy(SonicExecutionModule, **execution_config)
    try:
        module.start()
        peer.reference_endpoint = module.get_status()["reference_endpoint"]
        peer.connected.set()
        wait_until(lambda: module.get_status()["ready"], timeout=WAIT_S)
        yield module
    finally:
        module.stop()


def begin(module):
    status = module.start_task()
    wait_until(lambda: module.get_status()["task"]["phase"] == "waiting_for_chunk", timeout=WAIT_S)
    return status["task_id"]


def submit(module, task_id, sequence=0, observed_at=None):
    observed_at = module.get_feedback().received_monotonic if observed_at is None else observed_at
    module.submit_chunk(task_id, sequence, observed_at, np.full((50, 78), 0.2))


def test_start_is_passive_and_reference_binding_is_exclusive(execution, tmp_path):
    module, peer = execution
    assert peer.snapshot() == []
    other = SonicExecutionModule(
        reference_bind=peer.reference_endpoint,
        feedback_endpoint=peer.feedback_endpoint,
        journal_dir=tmp_path / "other",
        profile=module.config.profile,
    )
    try:
        with pytest.raises(zmq.ZMQError, match="Address already in use"):
            other.start()
        assert peer.snapshot() == []
    finally:
        other.stop()


def test_finish_streams_actions_and_confirms_handback_without_task_success(execution):
    module, peer = execution
    task_id = begin(module)
    submit(module, task_id)
    wait_until(lambda: module.get_status()["task"]["last_frame_index"] > 8, timeout=WAIT_S)
    module.finish_task(task_id)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()
    assert status["task"]["outcome"] == "finished"
    assert status["task"]["handback_confirmed"] is True
    assert status["task_success"] is None
    packets = peer.snapshot()
    poses = [p for p in packets if p.startswith(b"pose")]
    indices = [int(np.frombuffer(p[1540:1548], dtype="<i8")[0]) for p in poses]
    assert indices == sorted(set(indices))
    np.testing.assert_allclose(np.frombuffer(poses[0][1548:], dtype="<f4")[:7], 0.1)
    assert all(p[-3:-1] == b"\0\0" for p in packets if p.startswith(b"command"))
    last_command = max(i for i, packet in enumerate(packets) if packet.startswith(b"command"))
    assert not any(packet.startswith(b"pose") for packet in packets[last_command + 1 :])
    events = [json.loads(line) for line in Path(status["journal_path"]).read_text().splitlines()]
    assert events[-1]["event"] == "terminal"


def test_overlap_old_task_ids_and_late_chunks_are_rejected(execution):
    module, _ = execution
    first = begin(module)
    with pytest.raises(RuntimeError, match="active"):
        module.start_task()
    module.cancel_task(first)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    assert module.get_status()["task"]["outcome"] == "cancelled"
    second = begin(module)
    assert second != first
    with pytest.raises(ValueError, match="task"):
        submit(module, first)
    with pytest.raises(ValueError, match="task"):
        module.cancel_task(first)
    submit(module, second)
    module.finish_task(second)


def test_observation_time_is_preserved_and_stale_chunks_cannot_refresh_session(execution):
    module, _ = execution
    task_id = begin(module)
    submit(module, task_id)
    with pytest.raises(ValueError, match="observation"):
        submit(module, task_id, sequence=1, observed_at=time.monotonic() - 5)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert status["reason"] == "observation stale"
    assert status["handback_confirmed"] is True


def test_missing_feedback_fails_without_claiming_handback(execution):
    module, peer = execution
    begin(module)
    peer.pause_feedback.set()
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert "feedback stale" in status["reason"]
    assert status["handback_confirmed"] is False
    assert module.get_feedback() is None


def test_action_source_failure_is_distinct_from_finish(execution):
    module, _ = execution
    task_id = begin(module)
    module.fail_task(task_id, "policy worker failed")
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()["task"]
    assert status["outcome"] == "failed"
    assert status["reason"] == "policy worker failed"
    assert status["handback_confirmed"] is True


def test_module_stop_waits_for_planner_and_keeps_terminal_status(execution):
    module, _ = execution
    begin(module)
    module.stop()
    status = module.get_status()
    assert status["state"] == "stopped"
    assert status["task"]["outcome"] == "cancelled"
    assert status["task"]["handback_confirmed"] is True
    assert module.get_feedback() is None


def test_subscription_loss_fails_and_stops_policy_output(execution):
    module, peer = execution
    task_id = begin(module)
    submit(module, task_id)
    peer.drop_pose_subscription.set()
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()
    assert status["state"] == "fault"
    assert status["task"]["outcome"] == "failed"
    assert "subscriptions lost" in status["task"]["reason"]
    assert status["task"]["handback_confirmed"] is True
    with pytest.raises(RuntimeError, match="subscriptions lost"):
        module.start_task()


def test_concurrent_stops_do_not_close_loop_before_bounded_handback(execution, mocker):
    module, peer = execution
    begin(module)
    peer.pause_feedback.set()
    # Intercept final loop closure so the regression reports its premature call
    # instead of stranding the first stop on an event loop closed by the second.
    close_module = module._close_module
    phases_at_close = []
    mocker.patch.object(
        module,
        "_close_module",
        side_effect=lambda: phases_at_close.append(module.get_status()["task"]["phase"]),
    )
    started = time.monotonic()
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(module.stop)
            wait_until(lambda: module.get_status()["task"]["phase"] == "handback", timeout=WAIT_S)
            second = pool.submit(module.stop)
            first.result(timeout=WAIT_S)
            second.result(timeout=WAIT_S)
        assert phases_at_close == ["done", "done"]
        # Bounded by the profile's 0.4 s handback timeout (a hung stop never returns); the
        # margin is for a loaded runner.
        assert time.monotonic() - started < 5
        status = module.get_status()["task"]
        assert status["outcome"] == "failed"
        assert status["handback_confirmed"] is False
    finally:
        close_module()


def test_worker_rpc_preserves_chunks_and_terminal_results(worker_execution):
    module = worker_execution
    task_id = begin(module)
    submit(module, task_id)
    assert module.get_status()["task"]["last_chunk_sequence"] == 0
    module.finish_task(task_id)
    wait_until(lambda: module.get_status()["task"]["phase"] == "done", timeout=WAIT_S)
    status = module.get_status()["task"]
    assert status["outcome"] == "finished"
    assert status["handback_confirmed"] is True


@pytest.mark.parametrize("socket_name", ["_publisher", "_subscriber"])
def test_arrival_after_empty_poll_is_not_reported_as_receive_overload(
    execution_config, mocker, socket_name
):
    module = SonicExecutionModule(**execution_config)
    try:
        publisher = mocker.patch.object(module, "_publisher")
        subscriber = mocker.patch.object(module, "_subscriber")
        publisher.poll.return_value = subscriber.poll.return_value = 0
        target = getattr(module, socket_name)
        # Queue is empty at the receive boundary; a new packet arrives immediately
        # afterward. A second poll must not mistake it for an exhausted budget.
        target.poll.side_effect = [0, zmq.POLLIN]
        module._receive()
        assert target.recv.call_count == target.recv_multipart.call_count == 0
    finally:
        module.stop()


@pytest.mark.parametrize("socket_name", ["_publisher", "_subscriber"])
@pytest.mark.parametrize("extra_packet", [False, True])
def test_receive_budget_boundary(execution_config, mocker, socket_name, extra_packet):
    module = SonicExecutionModule(**execution_config)
    try:
        publisher = mocker.patch.object(module, "_publisher")
        subscriber = mocker.patch.object(module, "_subscriber")
        publisher.poll.return_value = subscriber.poll.return_value = 0
        target = getattr(module, socket_name)
        target.poll.side_effect = [zmq.POLLIN] * 64 + [zmq.POLLIN if extra_packet else 0]
        target.recv.return_value = b"\x01pose"
        target.recv_multipart.return_value = [b"g1_debuginvalid"]
        if extra_packet:
            with pytest.raises(RuntimeError, match="backlog exceeded receive budget"):
                module._receive()
        else:
            module._receive()
        assert target.recv.call_count + target.recv_multipart.call_count == 64
    finally:
        module.stop()
