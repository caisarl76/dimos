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

"""Read-only pc2 sensor relay to the off-board navigation peer.

The sender reads MID360S cloud/IMU and G1 lowstate over DDS (readers only) and
publishes two-part ``[channel, LCM payload]`` frames on separate unicast PUB
sockets, so a cloud burst cannot starve IMU/state. The receiver republishes the
original source stamps and composes base-frame odometry from PointLio, the
commissioned torso->sensor extrinsic and the relayed waist edge.

Cloud/IMU conversion is promoted from the proven task adapter
``docs/artifacts/g1_navigation_20261012/demo_dds_lio_adapter.py`` (SHA256
``e09185b79e7472179d9b5bcde159f5456f90f3f49b573c57dbe5caa1a007a5ae``), with its
asserts turned into explicit errors. It keeps DDS XYZ/timing, excludes
intensity/ring and normalizes the observed IMU g units to m/s^2.
"""

from __future__ import annotations

import math
import struct
import threading
from time import monotonic
from typing import Any, Literal

from dimos_lcm.sensor_msgs import Imu as LCMImu, PointCloud2 as LCMPointCloud2, PointField
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
import zmq
from zmq.utils.monitor import recv_monitor_message

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1.g1_tf_publisher import (
    _WAIST_PITCH_IDX,
    _WAIST_ROLL_IDX,
    _WAIST_YAW_IDX,
    base_to_torso,
)

Stream = Literal["cloud", "state"]

G = 9.80665
SENSOR_FRAME = "mid360_link"
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024
SOCKET_HWM = 2
MAX_DRAIN = 64
# Longer than every raw/base readiness budget, so a reset is always observed as stale.
RESET_HOLDOFF_SECONDS = 0.5
WAIST_BUDGET_SECONDS = 0.200
WAIST_RELAY_PERIOD_TICKS = 20  # lowstate ms ticks; relays the 500 Hz edge at 50 Hz
CHANNELS: dict[Stream, frozenset[bytes]] = {
    "cloud": frozenset({b"lidar_raw"}),
    "state": frozenset({b"imu_raw", b"waist_tf"}),
}
CHANNEL_STREAM: dict[str, Stream] = {"lidar_raw": "cloud", "imu_raw": "state", "waist_tf": "state"}


def set_header(target: Any, source: Any) -> None:
    target.seq = 0
    target.stamp.sec = source.stamp.sec
    target.stamp.nsec = source.stamp.nanosec
    target.frame_id = SENSOR_FRAME


def encode_imu(sample: Any) -> bytes:
    msg = LCMImu()
    set_header(msg.header, sample.header)
    for name in ("x", "y", "z"):
        setattr(msg.angular_velocity, name, getattr(sample.angular_velocity, name))
        setattr(msg.linear_acceleration, name, getattr(sample.linear_acceleration, name) * G)
    # PointLIO ignores orientation. Retain the DDS values and covariance metadata.
    for name in ("x", "y", "z", "w"):
        setattr(msg.orientation, name, getattr(sample.orientation, name))
    for name in ("orientation_covariance", "angular_velocity_covariance"):
        setattr(msg, name, list(getattr(sample, name)))
    msg.linear_acceleration_covariance = [v * G**2 for v in sample.linear_acceleration_covariance]
    return msg.lcm_encode()  # type: ignore[no-any-return]


def encode_cloud(sample: Any) -> bytes:
    if sample.is_bigendian:
        raise ValueError("Unexpected cloud endianness")
    fields = {f.name: f for f in sample.fields}
    names = ("x", "y", "z", "time")
    if not all(
        name in fields and fields[name].datatype == 7 and fields[name].count == 1 for name in names
    ):
        raise ValueError("Unexpected cloud fields")
    dtype = np.dtype(
        {
            "names": names,
            "formats": ["<f4"] * 4,
            "offsets": [fields[n].offset for n in names],
            "itemsize": sample.point_step,
        }
    )
    points = np.ndarray(
        (sample.height, sample.width),
        dtype=dtype,
        buffer=bytes(sample.data),
        strides=(sample.row_step, sample.point_step),
    ).ravel()
    times = points["time"]
    if not (np.isfinite(times).all() and (times >= 0).all() and (times <= 150_000_000).all()):
        raise ValueError("Invalid per-point nanoseconds")
    packed = np.empty(
        len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_time", "<u4")]
    )
    for name in ("x", "y", "z"):
        if not np.isfinite(points[name]).all():
            raise ValueError("Invalid XYZ")
        packed[name] = points[name]
    packed["offset_time"] = np.rint(times).astype("<u4")
    msg = LCMPointCloud2()
    set_header(msg.header, sample.header)
    msg.height, msg.width = 1, len(points)
    msg.fields = []
    for index, name in enumerate(packed.dtype.names or ()):
        field = PointField()
        field.name, field.offset, field.datatype, field.count = (
            name,
            index * 4,
            6 if name == "offset_time" else 7,
            1,
        )
        msg.fields.append(field)
    msg.fields_length = len(msg.fields)
    msg.is_bigendian, msg.is_dense = False, sample.is_dense
    msg.point_step, msg.row_step = 16, len(points) * 16
    msg.data = packed.tobytes()
    msg.data_length = len(msg.data)
    return msg.lcm_encode()  # type: ignore[no-any-return]


def encode_waist(sample: Any) -> bytes:
    """torso_link -> base_link from lowstate, stamped by its own millisecond tick."""
    edge = -base_to_torso(
        float(sample.motor_state[_WAIST_YAW_IDX].q),
        float(sample.motor_state[_WAIST_ROLL_IDX].q),
        float(sample.motor_state[_WAIST_PITCH_IDX].q),
    )
    edge.ts = int(sample.tick) * 1e-3
    return TFMessage(edge).lcm_encode()


ENCODERS = {"lidar_raw": encode_cloud, "imu_raw": encode_imu, "waist_tf": encode_waist}


def _stamped(ts: float | None) -> bool:
    return ts is not None and math.isfinite(ts) and ts > 0


def _waist_edge(msg: TFMessage) -> Transform | None:
    if len(msg.transforms) != 1:
        return None
    (edge,) = msg.transforms
    q, p = edge.rotation, edge.translation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if (
        (edge.frame_id, edge.child_frame_id) != ("torso_link", "base_link")
        or not _stamped(edge.ts)
        or not all(math.isfinite(v) for v in values)
        or abs(math.hypot(*values[3:]) - 1) >= 1e-3
    ):
        return None
    return edge


def livox_imu_type() -> type:
    """MID360S IMU DDS type: standard sensor_msgs/Imu layout, used by readers only.

    Field annotations are real objects. Under `from __future__ import annotations`
    they would be strings naming these lazily imported types, which CycloneDDS
    cannot resolve (the pc2 readiness probe failed that way on 2026-10-15).
    """
    from dataclasses import dataclass

    from cyclonedds.idl import IdlStruct  # type: ignore[import-not-found]
    from cyclonedds.idl.annotations import autoid, final  # type: ignore[import-not-found]
    from cyclonedds.idl.types import array, float64  # type: ignore[import-not-found]
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import (  # type: ignore[import-not-found]
        Quaternion_,
        Vector3_,
    )
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_  # type: ignore[import-not-found]

    covariance = array[float64, 9]
    fields = {
        "header": Header_,
        "orientation": Quaternion_,
        "orientation_covariance": covariance,
        "angular_velocity": Vector3_,
        "angular_velocity_covariance": covariance,
        "linear_acceleration": Vector3_,
        "linear_acceleration_covariance": covariance,
    }
    namespace = {"__annotations__": fields, "__module__": __name__, "__qualname__": "Imu_"}
    imu = type(IdlStruct)("Imu_", (IdlStruct,), namespace, typename="sensor_msgs.msg.dds_.Imu_")
    return dataclass(final(autoid("sequential")(imu)))  # type: ignore[no-any-return]


class SonicNavigationSensorSenderConfig(ModuleConfig):
    cloud_bind: str = Field(pattern=r"^tcp://", max_length=2048)
    state_bind: str = Field(pattern=r"^tcp://", max_length=2048)
    network_interface: str = ""
    dds_domain: int = Field(default=0, ge=0)


class SonicNavigationSensorSender(Module):
    """pc2 side: DDS readers only, no motor or mode interface."""

    config: SonicNavigationSensorSenderConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._context: zmq.Context[Any] | None = None
        self._sockets: dict[Stream, zmq.Socket[Any]] = {}
        self._readers: dict[str, Any] = {}
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_waist_tick: int | None = None
        self.encode_errors = 0

    @property
    def bound_endpoints(self) -> dict[Stream, str]:
        return {name: s.getsockopt_string(zmq.LAST_ENDPOINT) for name, s in self._sockets.items()}

    @rpc
    def start(self) -> None:
        if self._thread is not None:
            return
        super().start()
        self._context = zmq.Context()
        for name, bind in (("cloud", self.config.cloud_bind), ("state", self.config.state_bind)):
            socket = self._context.socket(zmq.PUB)
            socket.setsockopt(zmq.SNDHWM, SOCKET_HWM)
            socket.setsockopt(zmq.LINGER, 0)
            socket.bind(bind)
            self._sockets[name] = socket  # type: ignore[index]
        self._readers = self._open_readers()
        self._thread = threading.Thread(target=self._serve, name="sonic-sensor-send", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        for socket in self._sockets.values():
            socket.close()
        self._sockets.clear()
        if self._context is not None:
            self._context.term()
            self._context = None
        super().stop()

    def relay(self, channel: str, sample: Any) -> None:
        if channel == "waist_tf":
            tick = int(sample.tick)
            last = self._last_waist_tick
            if last is not None and 0 <= tick - last < WAIST_RELAY_PERIOD_TICKS:
                return
            self._last_waist_tick = tick
        try:
            payload = ENCODERS[channel](sample)
        except ValueError:
            self.encode_errors += 1
            return
        if len(payload) > MAX_PAYLOAD_BYTES:
            self.encode_errors += 1
            return
        try:
            self._sockets[CHANNEL_STREAM[channel]].send_multipart(
                [channel.encode(), payload], flags=zmq.NOBLOCK
            )
        except zmq.Again:
            pass  # HWM reached: drop rather than queue stale sensor data

    def _serve(self) -> None:
        while not self._stopping.is_set():
            for channel, reader in self._readers.items():
                for sample in reader.take(MAX_DRAIN):
                    if sample.sample_info.valid_data:
                        self.relay(channel, sample)
            self._stopping.wait(0.002)

    def _open_readers(self) -> dict[str, Any]:
        # Lazy DDS imports: present in the pc2 task image, absent from the workstation env.
        from cyclonedds.domain import DomainParticipant  # type: ignore[import-not-found]
        from cyclonedds.qos import Policy, Qos  # type: ignore[import-not-found]
        from cyclonedds.sub import DataReader  # type: ignore[import-not-found]
        from cyclonedds.topic import Topic  # type: ignore[import-not-found]
        from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
            ChannelFactoryInitialize,
        )
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import (  # type: ignore[import-not-found]
            PointCloud2_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # type: ignore[import-not-found]
            LowState_,
        )

        Imu_ = livox_imu_type()
        if self.config.network_interface:
            ChannelFactoryInitialize(self.config.dds_domain, self.config.network_interface)
        else:
            ChannelFactoryInitialize(self.config.dds_domain)
        participant = DomainParticipant(self.config.dds_domain)

        def reader(topic: str, kind: Any, depth: int) -> Any:
            qos = Qos(
                Policy.Reliability.BestEffort,
                Policy.Durability.Volatile,
                Policy.History.KeepLast(depth),
            )
            return DataReader(participant, Topic(participant, topic, kind), qos=qos)

        return {
            "lidar_raw": reader("rt/utlidar/cloud_livox_mid360", PointCloud2_, 2),
            "imu_raw": reader("rt/utlidar/imu_livox_mid360", Imu_, MAX_DRAIN),
            "waist_tf": reader("rt/lowstate", LowState_, 1),
        }


class SonicNavigationSensorReceiverConfig(ModuleConfig):
    cloud_endpoint: str = Field(pattern=r"^tcp://", max_length=2048)
    state_endpoint: str = Field(pattern=r"^tcp://", max_length=2048)
    # Commissioned torso_link -> mid360_link: x y z qx qy qz qw. None keeps base outputs off.
    torso_to_sensor: tuple[float, float, float, float, float, float, float] | None = None
    odom_frame: str = Field(default="odom", min_length=1)
    base_frame: str = Field(default="base_link", min_length=1)


class SonicNavigationSensorReceiver(Module):
    """Workstation side: relays raw sensors and publishes odom->base from LIO and waist."""

    config: SonicNavigationSensorReceiverConfig
    lidar_raw: Out[PointCloud2]
    imu_raw: Out[Imu]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    lio_odometry: In[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._holdoff: dict[Stream, float] = {}
        self._waist: tuple[Transform, float] | None = None
        self.rejected_frames = 0
        self._sensor_to_torso: Transform | None = None
        extrinsic = self.config.torso_to_sensor
        if extrinsic is not None:
            if (
                not all(math.isfinite(v) for v in extrinsic)
                or abs(math.hypot(*extrinsic[3:]) - 1) >= 1e-3
            ):
                raise ValueError("torso_to_sensor needs finite xyz and a unit quaternion")
            self._sensor_to_torso = -Transform(
                translation=Vector3(*extrinsic[:3]),
                rotation=Quaternion(*extrinsic[3:]),
                frame_id="torso_link",
                child_frame_id=SENSOR_FRAME,
            )

    @rpc
    def start(self) -> None:
        if self._thread is not None:
            return
        super().start()
        if self.lio_odometry.transport is not None:
            self.register_disposable(
                Disposable(self.lio_odometry.subscribe(self.observe_lio_odometry))
            )
        self._thread = threading.Thread(target=self._serve, name="sonic-sensor-recv", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        super().stop()

    def _decode(self, stream: Stream, parts: list[bytes]) -> tuple[bytes, Any] | None:
        if len(parts) != 2 or parts[0] not in CHANNELS[stream] or len(parts[1]) > MAX_PAYLOAD_BYTES:
            return None
        channel, payload = parts
        try:
            if channel == b"lidar_raw":
                cloud = PointCloud2.lcm_decode(payload)
                return (channel, cloud) if _stamped(cloud.ts) else None
            if channel == b"imu_raw":
                imu = Imu.lcm_decode(payload)
                return (channel, imu) if _stamped(imu.ts) else None
            edge = _waist_edge(TFMessage.lcm_decode(payload))
            return None if edge is None else (channel, edge)
        except (ValueError, struct.error):
            return None

    def handle_frames(self, stream: Stream, frames: list[list[bytes]]) -> None:
        """Process one bounded drain; clouds keep only the newest, IMU keeps order."""
        now = monotonic()
        accepted = []
        with self._lock:
            if now < self._holdoff.get(stream, 0.0):
                return  # queued across a reset is never live
            for parts in frames:
                decoded = self._decode(stream, parts)
                if decoded is None:
                    self.rejected_frames += 1
                    continue
                channel, msg = decoded
                if channel == b"waist_tf":
                    if self._waist is None or msg.ts > self._waist[0].ts:
                        self._waist = (msg, now)
                else:
                    accepted.append((channel, msg))
        if stream == "cloud":
            accepted = accepted[-1:]
        for channel, msg in accepted:
            (self.lidar_raw if channel == b"lidar_raw" else self.imu_raw).publish(msg)

    def handle_reset(self, stream: Stream) -> None:
        with self._lock:
            self._holdoff[stream] = monotonic() + RESET_HOLDOFF_SECONDS
            if stream == "state":
                self._waist = None

    def observe_lio_odometry(self, msg: Odometry) -> None:
        if (
            self._sensor_to_torso is None
            or (msg.frame_id, msg.child_frame_id) != (self.config.odom_frame, SENSOR_FRAME)
            or not _stamped(msg.ts)
        ):
            return
        with self._lock:
            waist = self._waist
            if waist is None or monotonic() - waist[1] >= WAIST_BUDGET_SECONDS:
                return
        sensor = Transform(
            translation=msg.position,
            rotation=msg.orientation,
            frame_id=self.config.odom_frame,
            child_frame_id=SENSOR_FRAME,
        )
        base = sensor + self._sensor_to_torso + waist[0]
        base.child_frame_id, base.ts = self.config.base_frame, msg.ts
        self.odometry.publish(
            Odometry(
                ts=msg.ts,
                frame_id=self.config.odom_frame,
                child_frame_id=self.config.base_frame,
                pose=Pose(position=base.translation, orientation=base.rotation),
            )
        )
        self.tf.publish(TFMessage(base))

    def _serve(self) -> None:
        context = zmq.Context()
        poller = zmq.Poller()
        sockets: dict[Any, tuple[Stream, bool]] = {}
        try:
            for stream, endpoint in (
                ("cloud", self.config.cloud_endpoint),
                ("state", self.config.state_endpoint),
            ):
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.RCVHWM, SOCKET_HWM)
                socket.setsockopt(zmq.MAXMSGSIZE, MAX_PAYLOAD_BYTES)
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.SUBSCRIBE, b"")
                monitor = socket.get_monitor_socket(zmq.EVENT_DISCONNECTED)
                socket.connect(endpoint)
                for target, is_monitor in ((socket, False), (monitor, True)):
                    poller.register(target, zmq.POLLIN)
                    sockets[target] = (stream, is_monitor)  # type: ignore[assignment]
            while not self._stopping.is_set():
                for target, _ in poller.poll(50):
                    stream, is_monitor = sockets[target]
                    if is_monitor:
                        recv_monitor_message(target)
                        self.handle_reset(stream)
                        continue
                    frames = []
                    for _ in range(MAX_DRAIN):
                        try:
                            frames.append(target.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                    self.handle_frames(stream, frames)
        finally:
            for target in sockets:
                target.close(linger=0)
            context.term()
