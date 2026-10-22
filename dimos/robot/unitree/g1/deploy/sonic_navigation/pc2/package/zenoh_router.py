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

"""Private loopback zenoh router for the pc2 graph's internal streams (no LAN scouting)."""

import json
import signal
import sys

import zenoh

config = zenoh.Config()
for key, value in {
    "mode": "router",
    "listen/endpoints": [f"tcp/127.0.0.1:{int(sys.argv[1])}"],
    "connect/endpoints": [],
    "scouting/multicast/enabled": False,
    "scouting/gossip/enabled": True,
}.items():
    config.insert_json5(key, json.dumps(value))
router = zenoh.open(config)
print(f"zenoh router on 127.0.0.1:{sys.argv[1]}", flush=True)
signal.sigwait({signal.SIGTERM, signal.SIGINT})
router.close()
