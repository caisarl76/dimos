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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam, WebcamConfig
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm import (
    HEAD_CAMERA_DEVICE,
    _head_camera,
    unitree_g1_sonic_arm,
)


def _started_head_camera(cli: list[str], environ: dict[str, str]) -> WebcamConfig:
    """Parse the blueprint as `dimos run` does and start its CameraModule without a device."""
    parsed = BlueprintConfigParser(unitree_g1_sonic_arm).parse(
        ["--sonicarmskillcontainer.encoder-path=unused.onnx", *cli], environ=environ
    )
    module = CameraModule(**parsed.module_kwargs("cameramodule"))
    try:
        with pytest.MonkeyPatch.context() as patch:
            patch.setattr(Webcam, "image_stream", lambda self: rx.empty())
            module.start()
        assert isinstance(module.hardware, Webcam)
        config = module.hardware.config
    finally:
        module.stop()
    # Everything but the device is the blueprint factory's.
    commissioned = _head_camera().config
    assert (config.width, config.height, config.fps) == (
        commissioned.width,
        commissioned.height,
        commissioned.fps,
    )
    for field in ("width", "height", "distortion_model", "D", "K", "P", "frame_id"):
        assert getattr(config.camera_info, field) == getattr(commissioned.camera_info, field)
    return config


def test_head_camera_defaults_to_the_commissioned_device() -> None:
    assert _started_head_camera([], {}).camera_index == HEAD_CAMERA_DEVICE


@pytest.mark.parametrize(
    ("cli", "environ", "expected"),
    [
        (["--cameramodule.camera-device=/dev/video4"], {}, "/dev/video4"),
        ([], {"CAMERAMODULE__CAMERA_DEVICE": "2"}, 2),
    ],
)
def test_head_camera_device_is_overridable(
    cli: list[str], environ: dict[str, str], expected: int | str
) -> None:
    assert _started_head_camera(cli, environ).camera_index == expected
