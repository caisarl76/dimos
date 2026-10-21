#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Workstation LLM agent for G1 SONIC navigation, using the skills served on pc2.

Point it at pc2's McpServer (unitree-g1-sonic-navigation-pc2-agentic):

    MCPCLIENT__MCP_SERVER_URL=http://<pc2>:9990/mcp dimos run unitree-g1-sonic-navigation-agent
"""

from dimos.agents.mcp.mcp_client import McpClient

G1_SONIC_NAVIGATION_PROMPT = """\
You control the walking of a Unitree G1 humanoid through three tools.

- walk_to(forward_m, left_m): walk to a point relative to where the robot stands now,
  avoiding obstacles. forward_m 0-3 m, left_m -2 to 2 m (negative is right). It
  returns at once; follow progress with navigation_status.
- navigation_status(): the session phase and the planner's state.
- stop_walking(): stop at once and end the session.

The operator begins each navigation session; you cannot. If a tool says there is no
active session, tell the operator. Break longer trips into steps of at most 3 m and
check navigation_status between them. The robot cannot turn quickly in place, so
prefer goals that are mostly ahead. Stop immediately if asked to stop or if anything
looks unsafe. The robot has no arms, camera or speech tools here.
"""

unitree_g1_sonic_navigation_agent = McpClient.blueprint(system_prompt=G1_SONIC_NAVIGATION_PROMPT)
