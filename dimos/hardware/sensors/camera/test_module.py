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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_nav import (
    unitree_g1_primitive_no_nav,
)


def test_webcam_module_leaves_other_modules_device_shorthand_alone() -> None:
    # A WebcamModule field named `device` would make this blueprint's --device ambiguous.
    parsed = BlueprintConfigParser(unitree_g1_primitive_no_nav.solidify()).parse(
        ["--device=CPU"], environ={}
    )
    assert parsed.module_kwargs("voxelgridmapper")["device"] == "CPU"
