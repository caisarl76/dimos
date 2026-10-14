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

"""Simulation-only type names for the two-host DDS reader stand-in; no SDK clients."""

from __future__ import annotations

import sys
from types import ModuleType
from typing import Any


def _register(name: str, **attrs: Any) -> None:
    module = ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module


def _kind(name: str) -> type:
    return type(name, (), {})


_register("unitree_sdk2py.core", __path__=[])
_register("unitree_sdk2py.core.channel", ChannelFactoryInitialize=lambda *args: None)
_register("unitree_sdk2py.idl", __path__=[])
_register(
    "unitree_sdk2py.idl.geometry_msgs.msg.dds_",
    Quaternion_=_kind("Quaternion_"),
    Vector3_=_kind("Vector3_"),
)
_register("unitree_sdk2py.idl.sensor_msgs.msg.dds_", PointCloud2_=_kind("PointCloud2_"))
_register("unitree_sdk2py.idl.std_msgs.msg.dds_", Header_=_kind("Header_"))
_register("unitree_sdk2py.idl.unitree_hg.msg.dds_", LowState_=_kind("LowState_"))
