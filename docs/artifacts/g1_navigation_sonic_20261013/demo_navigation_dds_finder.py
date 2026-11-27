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

"""Serve the two-host DDS replay shim only to the production sensor sender's imports.

DimOS's own optional DDS transport probe still sees no CycloneDDS. The shim
directory is never placed on sys.path.
"""

from __future__ import annotations

import importlib.machinery
from pathlib import Path
import sys
from types import ModuleType
from typing import Any

SHIM = Path(__file__).with_name("two-host-dds-shim")
SENDER = "dimos.robot.unitree.g1.sonic_navigation_sensor_module"


class SensorDdsFinder:
    @staticmethod
    def find_spec(name: str, path: Any = None, target: ModuleType | None = None) -> Any:
        if name not in ("cyclonedds", "unitree_sdk2py"):
            return None
        # Only the frame executing the import statement counts (skipping import
        # machinery), so imports made while the sender module is itself being
        # imported, such as DimOS's own DDS probe, still see no CycloneDDS.
        frame = sys._getframe(1)
        while frame is not None and frame.f_globals.get("__name__", "").startswith("importlib"):
            frame = frame.f_back
        if frame is not None and frame.f_globals.get("__name__") == SENDER:
            return importlib.machinery.PathFinder.find_spec(name, [str(SHIM)])
        return None


def install() -> None:
    if not any(isinstance(f, SensorDdsFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, SensorDdsFinder())
