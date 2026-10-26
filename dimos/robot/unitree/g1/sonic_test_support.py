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

"""Shared helpers for the G1 SONIC tests; fixtures built on them live in conftest.py."""

from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

import msgpack  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
import pytest
import zmq

from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.sonic_arm_reference import PEAK_SPEED, RATE, Keyframe, keyframe_plan
from dimos.robot.unitree.g1.sonic_arm_skill import _FINISH_HOLD_ROWS
from dimos.robot.unitree.g1.sonic_delivery_observation import (
    SonicCameraDelivery,
    SonicJointDelivery,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback, SonicReferenceGuard
from dimos.robot.unitree.g1.sonic_observation import SonicDeliveryStamp
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile

if TYPE_CHECKING:
    from dimos.robot.unitree.g1.sonic_arm_skill import SonicArm

# The longest a test waits for a condition. A loaded CI runner under coverage needs seconds for
# what takes milliseconds locally (a worker deploy took over 5 s); a passing test never waits it.
WAIT_S = 30.0


class ScriptedPeer:
    def __init__(self) -> None:
        self.ready = threading.Event()
        self.connected = threading.Event()
        self.stopping = threading.Event()
        self.pause_feedback = threading.Event()
        self.drop_pose_subscription = threading.Event()
        self.lock = threading.Lock()
        self.packets: list[bytes] = []
        # The Dex3 readings it reports; a test may set them before the skill measures them.
        self.left_hand_q = [0.1] * 7
        self.right_hand_q = [0.2] * 7
        self.error: Exception | None = None
        self.feedback_endpoint: str | None = None
        self.reference_endpoint: str | None = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self) -> None:
        try:
            with zmq.Context() as ctx, ctx.socket(zmq.PUB) as pub, ctx.socket(zmq.SUB) as sub:
                pub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.LINGER, 0)
                sub.setsockopt(zmq.SUBSCRIBE, b"command")
                sub.setsockopt(zmq.SUBSCRIBE, b"pose")
                port = pub.bind_to_random_port("tcp://127.0.0.1")
                self.feedback_endpoint = f"tcp://127.0.0.1:{port}"
                self.ready.set()
                if not self.connected.wait(WAIT_S) or self.stopping.is_set():
                    return
                assert self.reference_endpoint is not None
                sub.connect(self.reference_endpoint)
                planner, epoch, accepted, index, next_feedback = True, 1, -1, 0, 0.0
                while not self.stopping.is_set():
                    if self.drop_pose_subscription.is_set():
                        sub.setsockopt(zmq.UNSUBSCRIBE, b"pose")
                        self.drop_pose_subscription.clear()
                    if sub.poll(3):
                        packet = sub.recv()
                        with self.lock:
                            self.packets.append(packet)
                        if packet.startswith(b"command"):
                            selected = bool(packet[-1])
                            if planner and not selected:
                                epoch += 1
                            planner = selected
                        elif not planner:
                            accepted = int(np.frombuffer(packet[1540:1548], dtype="<i8")[0])
                    now = time.monotonic()
                    if now >= next_feedback and not self.pause_feedback.is_set():
                        index += 1
                        pub.send(
                            b"g1_debug"
                            + msgpack.packb(
                                {
                                    "index": index,
                                    "ros_timestamp": 0.0,
                                    "body_q": [0.0] * 29,
                                    "body_dq": [0.0] * 29,
                                    "left_hand_q": self.left_hand_q,
                                    "left_hand_dq": [0.0] * 7,
                                    "right_hand_q": self.right_hand_q,
                                    "right_hand_dq": [0.0] * 7,
                                    "token_state": [0.125] * 64,
                                    "reference_guard": {
                                        "armed": not planner,
                                        "expired": False,
                                        "planner_selected": planner,
                                        "planner_confirmed": planner,
                                        "rearm_requires_planner": False,
                                        "accepted_index": accepted,
                                        "applied_index": accepted,
                                        "rejected": 0,
                                        "epoch": epoch,
                                        "age_ms": 0.0,
                                    },
                                }
                            )
                        )
                        next_feedback = now + 0.01
        except Exception as exc:
            self.error = exc
            self.ready.set()

    def snapshot(self) -> list[bytes]:
        with self.lock:
            return list(self.packets)


@contextmanager
def scripted_peer() -> Iterator[ScriptedPeer]:
    """A running ScriptedPeer; connect it by setting reference_endpoint, then connected."""
    peer = ScriptedPeer()
    peer.thread.start()
    try:
        assert peer.ready.wait(WAIT_S) and peer.error is None
        yield peer
    finally:
        peer.stopping.set()
        peer.connected.set()
        peer.thread.join(timeout=WAIT_S)
        assert not peer.thread.is_alive() and peer.error is None


def feedback(
    now: float,
    index: int = 100,
    *,
    planner: bool = True,
    epoch: int = 7,
    accepted: int = 41,
    applied: int | None = None,
) -> SonicFeedback:
    return SonicFeedback(
        index=index,
        source_timestamp=None,
        received_at=1000.0 + now,
        received_monotonic=now,
        body_q=(0.0,) * 29,
        body_dq=(0.0,) * 29,
        left_hand_q=(0.1,) * 7,
        left_hand_dq=(0.0,) * 7,
        right_hand_q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        right_hand_dq=(0.0,) * 7,
        token_state=(0.125,) * 64,
        reference_guard=SonicReferenceGuard(
            armed=not planner,
            expired=False,
            planner_selected=planner,
            planner_confirmed=planner,
            rearm_requires_planner=False,
            accepted_index=accepted,
            applied_index=accepted if applied is None else applied,
            rejected=0,
            epoch=epoch,
            age_ms=0.0,
        ),
    )


def pose_values(packet: bytes) -> np.ndarray:
    payload = packet[4 + 1280 :]
    return np.concatenate(
        [np.frombuffer(payload[:256], dtype="<f4"), np.frombuffer(payload[264:], dtype="<f4")]
    )


def joints(
    group: Literal["body", "left_hand", "right_hand"],
    marker: int = 1,
    received_at: float = 10.0,
    source_id: str | None = None,
) -> SonicJointDelivery:
    values = tuple(float(i) for i in range(29 if group == "body" else 7))
    return SonicJointDelivery(
        group, values, SonicDeliveryStamp(source_id or group, marker, received_at)
    )


def camera(
    marker: int = 1, received_at: float = 10.01, source_id: str = "camera"
) -> SonicCameraDelivery:
    return SonicCameraDelivery(
        np.full((480, 640, 3), [10, 20, 30], np.uint8),
        SonicDeliveryStamp(source_id, marker, received_at),
    )


_MODEL = "gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
SCENE = next(
    (
        root / _MODEL
        for root in (
            Path.home() / "work/GR00T-WholeBodyControl-hand-tracking-watchdog",
            Path.home() / "GR00T-WholeBodyControl",  # the G1 PC2
        )
        if (root / _MODEL).exists()
    ),
    Path("/nonexistent"),
)
needs_scene = pytest.mark.skipif(
    not SCENE.exists(), reason="gear_sonic G1 MuJoCo model not on this host"
)
ROBOT_IDLE = np.array(
    [
        -0.312,
        0.044,
        0,
        0.669,
        -0.363,
        0,
        -0.312,
        0,
        0,
        0.669,
        -0.363,
        0,
        0,
        0,
        0,
        -0.02,
        0.284,
        -0.651,
        0.984,
        -0.152,
        -0.06,
        0.011,
        0.2,
        -0.2,
        0,
        0.6,
        0,
        0,
        0,
    ]
)


IMAGE_640x480 = Image.from_numpy(np.zeros((480, 640, 3), np.uint8))


class FakeVl:
    """query_json answers with a fixed payload, or raises it if it is an exception."""

    def __init__(self, payload: Any) -> None:
        self.payload = payload
        self.prompts: list[str] = []

    def query_json(self, image: Image, prompt: str) -> Any:
        self.prompts.append(prompt)
        if isinstance(self.payload, Exception):
            raise self.payload
        return self.payload


# The arm-skill tests' scripted execution: an observation age covering the plan, and a tick gap
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
    hand_step_limit=PEAK_SPEED / RATE,  # as SONIC_ARM_PROFILE
)
PEER_TOKEN = 0.125  # ScriptedPeer token_state
FAKE_GAIN = 0.25  # Fake token value 0 per rad of summed joint offset; keeps big poses in the bound.


class FakeEncoder:
    def __init__(self, path: Any) -> None:
        pass

    def encode(self, poses: NDArray[np.float64]) -> NDArray[np.float64]:
        tokens = np.full((len(poses), 64), PEER_TOKEN)
        tokens[:, 0] += FAKE_GAIN * (poses - poses[0]).sum(axis=1)  # The planned offsets.
        return tokens


def real_hold_plan(base: NDArray[np.float64], keyframes: Sequence[Keyframe], **kwargs: Any) -> Any:
    """The real engine with short leads and moves but the requested holds."""
    return keyframe_plan(
        base, keyframes, **(kwargs | dict(lead_s=0.1, move_s=0.3, peak_speed=100.0))
    )


def fast_keyframe_plan(
    base: NDArray[np.float64], keyframes: Sequence[Keyframe], **kwargs: Any
) -> Any:
    """The real engine with short leads, moves and holds, so a run takes about a second."""
    capped = [replace(k, hold_s=min(k.hold_s, 0.1), min_s=min(k.min_s, 0.1)) for k in keyframes]
    fast: dict[str, Any] = dict(lead_s=0.1, move_s=0.3, peak_speed=100.0)
    fast["return_min_s"] = min(kwargs.get("return_min_s", 0.0), 0.1)
    return keyframe_plan(base, capped, **(kwargs | fast))


def wait_until(condition: Callable[[], Any], timeout: float = WAIT_S) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.01)


def tokens(packets: list[bytes]) -> NDArray[np.float32]:
    return np.array([np.frombuffer(p[1284:1540], "<f4") for p in packets if p.startswith(b"pose")])


def unheld(chunk: NDArray[np.float64]) -> NDArray[np.float64]:
    """A last chunk less its finish hold, which must repeat the chunk's own last row."""
    rows, hold = chunk[:-_FINISH_HOLD_ROWS], chunk[-_FINISH_HOLD_ROWS:]
    assert np.all(hold == rows[-1])
    return rows


def frames(transport: Any, method: str) -> list[Any]:
    return [c.args[0] for c in transport.publish.call_args_list if c.args[0]["method"] == method]


def gate_start(module: Any, mocker: Any) -> tuple[threading.Event, Any]:
    """Holds the motion before its execution task starts until the returned event is set."""
    gate = threading.Event()
    real = module._execution.start_task
    start = mocker.patch.object(
        module._execution, "start_task", side_effect=lambda: gate.wait(10) and real()
    )
    return gate, start


def stop_in_background(module: Any, mocker: Any) -> tuple[threading.Thread, threading.Event]:
    """Starts module.stop() on a thread and holds it just after its retreat step, while the
    coordinator would still be tearing the module down; returns (stopper, release)."""
    stopping, release = threading.Event(), threading.Event()
    retreat = module._retreat_before_stop

    def retreat_then_hold() -> None:
        retreat()
        stopping.set()
        release.wait(10)

    mocker.patch.object(module, "_retreat_before_stop", side_effect=retreat_then_hold)
    stopper = threading.Thread(target=module.stop)
    stopper.start()
    assert stopping.wait(10)
    return stopper, release


def step(s: float) -> float:
    """At the plan's depth mid-run."""
    return 1.0 if 0.5 < s < 0.9 else 0.0


@contextmanager
def feeding_arms(
    arms: Sequence["SonicArm"],
    shape: Callable[[float], float] = step,
    noise: tuple[float, float] = (0.0, 0.0),
    extra: tuple[Any, Any] = (None, None),
) -> Iterator[None]:
    """One feeder per arm: base + shape(s) * planned displacement, plus extra(s) joint offsets.

    s is the sample's receipt time after the arm's chunk-0 submission. Before that submission
    the feeder adds uniform +-noise rad to every joint.
    """
    stopped = threading.Event()

    def feed(arm: "SonicArm", noise: float, extra: Any) -> None:
        rng = np.random.default_rng(0)
        while not stopped.is_set():
            try:
                state = arm._execution.get_feedback()
            except TimeoutError:  # A dead execution worker's proxy.
                state = None
            watch = arm._watch
            if state is not None and watch is not None:
                q = watch.base.copy()
                if watch.send_at is None:
                    q += rng.uniform(-noise, noise, 29)
                else:
                    s = state.received_monotonic - watch.send_at
                    q += shape(s) * watch.delta
                    for joint, value in (extra(s) if extra else {}).items():
                        q[joint] += value
                arm.handle_feedback(replace(state, body_q=tuple(q)))
            time.sleep(0.01)

    feeders = [
        threading.Thread(target=feed, args=args, daemon=True)
        for args in zip(arms, noise, extra, strict=True)
    ]
    for feeder in feeders:
        feeder.start()
    try:
        yield
    finally:
        stopped.set()
        for feeder in feeders:
            feeder.join(2)


def no_pose(peer: ScriptedPeer) -> bool:
    return not any(p.startswith(b"pose") for p in peer.snapshot())


def task(arm: "SonicArm") -> Any:
    return arm._execution.get_status()["task"]
