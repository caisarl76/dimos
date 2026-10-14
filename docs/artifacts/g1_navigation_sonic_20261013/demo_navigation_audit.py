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

"""Simulation-only audit trail of link control calls; behavior is unchanged.

Installed by the two-host entry modules (and so in their re-imported workers)
when SONIC_TWO_HOST_OUTPUT is set. Each call is logged before and after, on the
local monotonic clock of the process that handled it.
"""

from __future__ import annotations

import functools
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

_LOCK = threading.Lock()


def _log(name: str, **row: Any) -> None:
    path = Path(os.environ["SONIC_TWO_HOST_OUTPUT"]) / f"{name}.jsonl"
    with _LOCK, path.open("a") as stream:
        stream.write(
            json.dumps({"pid": os.getpid(), "monotonic": time.monotonic(), **row}, default=str)
            + "\n"
        )


def _wrap(cls: type, method: str, log: str, describe: Any) -> None:
    original = getattr(cls, method)
    if getattr(original, "__audited__", False):
        return

    @functools.wraps(original)
    def audited(self: Any, *args: Any, **kwargs: Any) -> Any:
        _log(log, call=method, phase="begin", **describe(args, kwargs))
        try:
            result = original(self, *args, **kwargs)
        except Exception as exc:
            _log(log, call=method, phase="raised", error=repr(exc))
            raise
        summary = (
            {k: result.get(k) for k in ("request_id", "accepted", "cancelled", "state", "fault")}
            if isinstance(result, dict)
            else result
        )
        _log(log, call=method, phase="end", result=summary)
        return result

    audited.__audited__ = True  # type: ignore[attr-defined]
    setattr(cls, method, audited)


def install() -> None:
    from dimos.robot.unitree.g1.sonic_navigation_peer_module import SonicNavigationPeerModule
    from dimos.robot.unitree.g1.sonic_navigation_relay_module import SonicNavigationRelayModule

    _wrap(
        SonicNavigationRelayModule,
        "set_navigation_goal",
        "audit-relay",
        lambda a, k: {"request_id": a[0] if a else k.get("request_id")},
    )
    _wrap(
        SonicNavigationRelayModule,
        "cancel_navigation",
        "audit-relay",
        lambda a, k: {"request_id": a[0] if a else k.get("request_id")},
    )

    def control(args: Any, kwargs: Any) -> dict[str, Any]:
        request = args[0] if args else kwargs["request"]
        return {"operation": request["operation"], "request_id": request["request_id"]}

    _wrap(SonicNavigationPeerModule, "_control", "audit-peer", control)
