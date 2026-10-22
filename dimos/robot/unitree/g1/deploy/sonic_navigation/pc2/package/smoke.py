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

"""Network-disabled image check: imports, pc2 graph validation and ARM64 planner codec."""

import json
from pathlib import Path
import socket

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_sonic_navigation_pc2 import (
    unitree_g1_sonic_navigation_pc2 as blueprint,
)
from dimos.robot.unitree.g1.sonic_navigation import PlannerIntent, encode_planner
from dimos.robot.unitree.g1.sonic_navigation_graph import validate_sonic_navigation_graph

assert [name for _, name in socket.if_nameindex()] == ["lo"], "run with --network none"
import cyclonedds.domain  # noqa: F401  # the real reader stack must import in the image
import unitree_sdk2py.idl.unitree_hg.msg.dds_  # noqa: F401

config = Path(__file__).with_name("config") / "pc2-graph.json"
parsed = BlueprintConfigParser(blueprint).parse(config_path=config, environ={})
kwargs = {a.name: {**a.kwargs, **parsed.module_kwargs(a.name)} for a in blueprint.active_blueprints}
validate_sonic_navigation_graph(blueprint, kwargs, role="pc2")
idle = encode_planner(PlannerIntent(0, (0, 0, 0), (1, 0, 0), 0.0), sequence=1, deadline=1.0)
assert idle.startswith(b"planner") and len(idle) == 1339, len(idle)
try:
    encode_planner(PlannerIntent(1, (1, 0, 0), (1, 0, 0), 0.25))
    raise AssertionError("over-limit walking speed must be rejected")
except ValueError:
    pass
print(json.dumps({"smoke": "passed", "modules": sorted(kwargs), "planner_packet_bytes": len(idle)}))
