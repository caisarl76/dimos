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

import pytest
import reactivex as rx
from reactivex.observable import Observable

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.spec import CameraHardware
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_nav import (
    unitree_g1_primitive_no_nav,
)


class _OtherCamera(CameraHardware):
    def __init__(self) -> None:
        pass

    def image_stream(self) -> Observable[Image]:
        return rx.empty()

    @property
    def camera_info(self) -> CameraInfo:
        return CameraInfo()


def test_device_override_rejects_a_non_webcam() -> None:
    module = CameraModule(hardware=_OtherCamera(), camera_device="/dev/video4")
    try:
        with pytest.raises(
            ValueError, match="needs a Webcam; this blueprint's camera is _OtherCamera"
        ):
            module.start()
    finally:
        module.stop()


def test_camera_device_leaves_other_modules_device_shorthand_alone() -> None:
    # A CameraModule field named `device` would make this blueprint's --device ambiguous.
    parsed = BlueprintConfigParser(unitree_g1_primitive_no_nav).parse(["--device=CPU"], environ={})
    assert parsed.module_kwargs("voxelgridmapper")["device"] == "CPU"
