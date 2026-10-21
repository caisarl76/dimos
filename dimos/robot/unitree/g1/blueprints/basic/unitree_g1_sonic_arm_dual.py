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

"""Two G1s' left-arm skills from one workstation, over independently launched SONIC controllers.

Runbook. Each G1 (222 on pc2_222, 223 on pc2_223) runs its guarded SONIC launcher, which binds
telemetry on 127.0.0.1:5557 and reads references from 127.0.0.1:5556. Before `dimos run`:

1. Start the controller on each PC2 and confirm planner idle.
2. Open the tunnels, one per robot (-R carries the PC2's reference port to this process, -L its
   telemetry here):

       ssh -N -R 5556:127.0.0.1:15556 -L 15557:127.0.0.1:5557 pc2_222
       ssh -N -R 5556:127.0.0.1:16556 -L 16557:127.0.0.1:5557 pc2_223

3. Run, with the SONIC v1.1 encoder and the gear_sonic MuJoCo model for the path check:

       dimos run unitree-g1-sonic-arm-dual \\
           --dualsonicarmskillcontainer.encoder-path=<deploy>/policy/sonic_v1_1/model_encoder.onnx \\
           --dualsonicarmskillcontainer.collision-model-path=<gear_sonic>/data/robot_model/model_data/g1/scene_43dof.xml
       dimos mcp call move_upper_body --arg robot=both --arg motion=left_arm_forward --arg angle_deg=20
       dimos mcp call perform_gesture --arg robot=222 --arg name=wave

WARNING: a dropped tunnel still snaps a raised arm: the controller loses its reference and the arm
returns to the planner pose at once. Keep both robots on stands.

The motion skills return at once; the result arrives as a tool update (also logged: `dimos log -f`).
Only one motion runs at a time across both robots; stop_arm stops it.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.sonic_dual_arm_skill import DualSonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile

# Same profile as unitree-g1-sonic-arm.
_PROFILE = SonicModelProfile(
    name="g1-sonic-arm",
    source_fps=50,
    right_hand_order="native",
    handback_policy="native_planner",
    max_observation_age=35.0,
    max_duration=75.0,
)


unitree_g1_sonic_arm_dual = autoconnect(
    SonicExecutionModule.blueprint(
        instance_name="g1_222",
        reference_bind="tcp://127.0.0.1:15556",
        feedback_endpoint="tcp://127.0.0.1:15557",
        journal_dir=STATE_DIR / "sonic-arm" / "222",
        profile=_PROFILE,
    ),
    SonicExecutionModule.blueprint(
        instance_name="g1_223",
        reference_bind="tcp://127.0.0.1:16556",
        feedback_endpoint="tcp://127.0.0.1:16557",
        journal_dir=STATE_DIR / "sonic-arm" / "223",
        profile=_PROFILE,
    ),
    DualSonicArmSkillContainer.blueprint(),
    McpServer.blueprint(),
).remappings(
    [
        ("g1_222", "feedback", "feedback_222"),
        ("g1_223", "feedback", "feedback_223"),
        ("g1_222", "execution_status", "execution_status_222"),
        ("g1_223", "execution_status", "execution_status_223"),
        (DualSonicArmSkillContainer, "_execution_222", "g1_222"),
        (DualSonicArmSkillContainer, "_execution_223", "g1_223"),
    ]
)
