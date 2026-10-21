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


"""pc2 navigation graph plus agent skills, served to a workstation agent over MCP.

The skills act only inside a request the operator began (see sonic_navigation_skills).
McpServer must bind an explicit interface address; the validator refuses all
interfaces.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_pc2 import (
    unitree_g1_sonic_navigation_pc2,
)
from dimos.robot.unitree.g1.sonic_navigation_skills import SonicNavigationSkills

unitree_g1_sonic_navigation_pc2_agentic = autoconnect(
    unitree_g1_sonic_navigation_pc2,
    SonicNavigationSkills.blueprint(),
    McpServer.blueprint(),
)
