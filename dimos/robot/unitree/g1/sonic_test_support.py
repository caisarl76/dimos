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

from pathlib import Path
import threading
import time
from typing import Any, Literal

import msgpack  # type: ignore[import-untyped]
import numpy as np
import pytest
import zmq

from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.sonic_delivery_observation import (
    SonicCameraDelivery,
    SonicJointDelivery,
)
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback, SonicReferenceGuard
from dimos.robot.unitree.g1.sonic_observation import SonicDeliveryStamp

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
                                    "left_hand_q": [0.1] * 7,
                                    "left_hand_dq": [0.0] * 7,
                                    "right_hand_q": [0.2] * 7,
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
