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

"""G1 upper-body motion skill and head camera over an independently launched, guarded SONIC controller.

The operator launches and stops native control and selects planner idle; this blueprint never
does either. Run it on the robot PC, beside the controller, with the SONIC v1.1 encoder:

    dimos run unitree-g1-sonic-arm \\
        --sonicarmskillcontainer.encoder-path=<deploy>/policy/sonic_v1_1/model_encoder.onnx
    dimos mcp call move_upper_body --arg motion=left_arm_forward --arg angle_deg=20
    dimos mcp call move_left_arm_pose --arg pose=left_hand_over_head
    dimos mcp call stop_arm

The motion skills return at once ("started"); their result arrives as a tool update, which
`dimos mcp call` does not print. It is also logged: follow it with `dimos log -f`.

Operator rule: call stop_arm and wait for its tool update before `dimos stop` or stopping the
controller. Shutdown also retreats a running motion first, but only best-effort: the execution
module may stop at the same time and hand the raised arm to the native planner.

move_left_arm_joints also needs the gear_sonic MuJoCo model for its path check:
--sonicarmskillcontainer.collision-model-path=<gear_sonic>/data/robot_model/model_data/g1/scene_43dof.xml
    dimos mcp call observe

The head camera is the G1's D435i colour stream, read through V4L2 (no depth).
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.unitree.g1.sonic_arm_reference import (
    HEAD_CAMERA_K,
    HEAD_CAMERA_QUAT_XYZW,
    HEAD_CAMERA_SIZE,
    HEAD_CAMERA_XYZ,
)
from dimos.robot.unitree.g1.sonic_arm_skill import SonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile

# The D435i colour interface (USB interface 3) on the G1 PC2's internal head-camera port; by-path
# stays fixed across reboots, unlike /dev/videoN, and by-id names collide with the depth interface.
HEAD_CAMERA_DEVICE = "/dev/v4l/by-path/platform-3610000.usb-usb-0:3:1.3-video-index0"
_FX, _FY, _CX, _CY = HEAD_CAMERA_K
_WIDTH, _HEIGHT = HEAD_CAMERA_SIZE


def _head_camera() -> Webcam:
    return Webcam(
        camera_index=HEAD_CAMERA_DEVICE,
        width=_WIDTH,
        height=_HEIGHT,
        fps=15,
        camera_info=CameraInfo(
            height=_HEIGHT,
            width=_WIDTH,
            distortion_model="plumb_bob",
            D=[0.0] * 5,
            K=[_FX, 0.0, _CX, 0.0, _FY, _CY, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[_FX, 0.0, _CX, 0.0, 0.0, _FY, _CY, 0.0, 0.0, 0.0, 1.0, 0.0],
            frame_id="camera_optical",
        ),
    )


# The skills stream a plan in chunks of at most 20 s, split only inside a hold: up to 15.4 s for
# move_upper_body, 31 s for the slowest full-range pose (overhead at 0.47 rad/s) and up to 20 s more
# for a held pose. max_duration also covers a retreat after a late stop.
SONIC_ARM_PROFILE = SonicModelProfile(
    name="g1-sonic-arm",
    source_fps=50,
    right_hand_order="native",
    handback_policy="native_planner",
    max_observation_age=35.0,
    max_duration=75.0,
)


unitree_g1_sonic_arm = autoconnect(
    CameraModule.blueprint(
        hardware=_head_camera,
        transform=Transform(
            translation=Vector3(*HEAD_CAMERA_XYZ),
            rotation=Quaternion(*HEAD_CAMERA_QUAT_XYZW),
            frame_id="torso_link",
            child_frame_id="camera_link",
        ),
    ),
    ObserveSkill.blueprint(),
    SonicExecutionModule.blueprint(
        # The guarded native launcher's fixed local endpoints (reference in, telemetry out).
        reference_bind="tcp://127.0.0.1:5556",
        feedback_endpoint="tcp://127.0.0.1:5557",
        journal_dir=STATE_DIR / "sonic-arm",
        profile=SONIC_ARM_PROFILE,
    ),
    SonicArmSkillContainer.blueprint(),
    McpServer.blueprint(),
)
