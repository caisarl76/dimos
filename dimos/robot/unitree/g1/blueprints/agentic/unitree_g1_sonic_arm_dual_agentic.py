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

"""unitree-g1-sonic-arm-dual plus an LLM agent that can call the two robots' left-arm skills.

Needs the model's API key (e.g. OPENAI_API_KEY) in the environment or .env. Talk to it with
`dimos agent-send "make both robots wave"`.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm_dual import (
    unitree_g1_sonic_arm_dual,
)
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_DUAL_ARM_SYSTEM_PROMPT

unitree_g1_sonic_arm_dual_agentic = autoconnect(
    unitree_g1_sonic_arm_dual,
    McpClient.blueprint(system_prompt=G1_SONIC_DUAL_ARM_SYSTEM_PROMPT),
).global_config(mcp_timeout=60)  # A full-range overhead pose takes about 40 s.
