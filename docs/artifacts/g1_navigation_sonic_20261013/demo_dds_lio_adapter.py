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

"""Task-only converters: keep DDS XYZ/timing; normalize the observed IMU g units.

No DDS writers or hardware calls. MID360S intensity/ring fields are excluded.
This preserves the original sensor coordinate basis, named mid360_link to match
PointLio; it does not apply a robot-body extrinsic calibration.
"""

from typing import Any

from dimos_lcm.sensor_msgs import Imu as LCMImu, PointCloud2 as LCMPointCloud2, PointField
import numpy as np


def set_header(target: Any, source: Any) -> None:
    target.seq = 0
    target.stamp.sec = source.stamp.sec
    target.stamp.nsec = source.stamp.nanosec
    target.frame_id = "mid360_link"


def encode_imu(sample: Any) -> bytes:
    msg = LCMImu()
    set_header(msg.header, sample.header)
    for name in ("x", "y", "z"):
        setattr(msg.angular_velocity, name, getattr(sample.angular_velocity, name))
        setattr(msg.linear_acceleration, name, getattr(sample.linear_acceleration, name) * 9.80665)
    # PointLIO ignores orientation. Retain the DDS values and covariance metadata.
    for name in ("x", "y", "z", "w"):
        setattr(msg.orientation, name, getattr(sample.orientation, name))
    for name in ("orientation_covariance", "angular_velocity_covariance"):
        setattr(msg, name, list(getattr(sample, name)))
    msg.linear_acceleration_covariance = [
        v * 9.80665**2 for v in sample.linear_acceleration_covariance
    ]
    return msg.lcm_encode()


def encode_cloud(sample: Any) -> bytes:
    assert not sample.is_bigendian, "Unexpected cloud endianness"
    fields = {f.name: f for f in sample.fields}
    names = ("x", "y", "z", "time")
    assert all(
        name in fields and fields[name].datatype == 7 and fields[name].count == 1 for name in names
    )
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
    assert np.isfinite(times).all() and (times >= 0).all() and (times <= 150_000_000).all(), (
        "Invalid per-point nanoseconds"
    )
    packed = np.empty(
        len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_time", "<u4")]
    )
    for name in ("x", "y", "z"):
        assert np.isfinite(points[name]).all(), "Invalid XYZ"
        packed[name] = points[name]
    packed["offset_time"] = np.rint(times).astype("<u4")
    msg = LCMPointCloud2()
    set_header(msg.header, sample.header)
    msg.height, msg.width = 1, len(points)
    msg.fields = []
    for index, name in enumerate(packed.dtype.names):
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
    return msg.lcm_encode()
