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

"""pc2 half of SONIC navigation: read-only sensor sender, relay and the one execution owner.

Nothing here produces or consumes cmd_vel. Native endpoints, model profile and
journal directory are explicit configuration; graph validation cannot prove
ownership of processes outside this blueprint.
"""

from functools import partial

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_navigation_graph import validate_sonic_navigation_graph
from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule
from dimos.robot.unitree.g1.sonic_navigation_sensor_module import SonicNavigationSensorSender

unitree_g1_sonic_navigation_pc2 = (
    autoconnect(
        SonicNavigationSensorSender.blueprint(),
        SonicNavigationRelayModule.blueprint(),
        SonicExecutionModule.blueprint(navigation_enabled=True),
    )
    .global_config(n_workers=3, robot_model="unitree_g1", viewer="none")
    .validators(partial(validate_sonic_navigation_graph, role="pc2"))
)
