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

"""Cue 223's demo gestures from 222's handover loop, hands-free.

Follows the VoLoAgent loop's evidence events on 222 over ssh and calls perform_gesture on 223's MCP
server (unitree-g1-sonic-arm, on this workstation):

    trial_start                            -> invite + 06_invite
    trial_end "handover complete"          -> take_<side> + 07_take (222 holds the bottle out)
    taken_confirmed                        -> entrance_<side> + 08_entrance, then thanks + 09_thanks
    trial_end otherwise, a freeze, the end -> nothing more this round

One gesture runs at a time and is never cut short; one cue waits, and a newer cue replaces it. A refused
or failed gesture is logged and dropped. Events written while ssh is down are skipped. It never calls
stop_arm and never touches the controller. Run it after 223's blueprint is up:

    python -m dimos.robot.unitree.g1.handover_cues \\
        --session-dir '~/GR00T-WholeBodyControl-loop-20261026/sessions/<the loop's --session-dir>'
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
import queue
import subprocess
import threading
import time
from typing import Any, Literal

import requests

from dimos.agents.mcp.constants import DEFAULT_MCP_URL
from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.agents.mcp.tool_stream import TOOL_STREAM_STOPPED_METHOD
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TOOL = "perform_gesture"
HANDOVER = "handover complete"  # 222's trial_end reason when the bottle is held out
DROP = frozenset({"stop_key", "loop_freeze", "native_freeze", "session_end"})
RECONNECT_S = 3.0
GESTURE_TIMEOUT_S = (
    60.0  # no stop notice by then: assume it ended (perform_gesture plans are <=45 s)
)

Side = Literal["left", "right"]
Cue = list[tuple[str, str]]

# Runs on 222 (its system python3): prints the events appended to trial_*_evidence.jsonl from now on,
# one JSON object per line, in the order they were written.
_FOLLOW = r"""
import glob, json, os, sys, time
pattern = os.path.join(os.path.expanduser(sys.argv[1]), "trial_*_evidence.jsonl")
done = {p: os.path.getsize(p) for p in glob.glob(pattern)}
while True:
    events = []
    for p in glob.glob(pattern):
        with open(p, "rb") as f:
            f.seek(done.get(p, 0))
            chunk = f.read()
        whole = chunk[: chunk.rfind(b"\n") + 1]
        done[p] = done.get(p, 0) + len(whole)
        for line in whole.splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                pass
    for e in sorted(events, key=lambda e: e.get("at", 0)):
        print(json.dumps(e), flush=True)
    time.sleep(0.2)
"""


def cue_of(event: dict[str, Any], take_side: Side, entrance_side: Side) -> Cue | None:
    """The gestures an event cues ([] clears the waiting cue), or None when it is not a cue."""
    kind = event.get("kind")
    if kind == "trial_start":
        return [("invite", "06_invite")]
    if kind == "trial_end":
        return [(f"take_{take_side}", "07_take")] if event.get("reason") == HANDOVER else []
    if kind == "taken_confirmed":
        return [(f"entrance_{entrance_side}", "08_entrance"), ("thanks", "09_thanks")]
    if kind in DROP:
        return []
    return None


class Cuer:
    """One gesture at a time; the cue that arrives while it runs waits, and a newer one replaces it."""

    def __init__(
        self,
        start: Callable[[str, str], bool],
        take_side: Side,
        entrance_side: Side,
        timeout_s: float = GESTURE_TIMEOUT_S,
    ) -> None:
        self._start = start
        self._sides = (take_side, entrance_side)
        self._timeout_s = timeout_s
        self.waiting: Cue = []
        self.busy_until: float | None = None

    def on_event(self, event: dict[str, Any]) -> None:
        cue = cue_of(event, *self._sides)
        if cue is None:
            return
        if self.waiting:
            logger.info("Cue replaced", dropped=self.waiting, by=event.get("kind"))
        self.waiting = cue

    def on_stopped(self) -> None:
        self.busy_until = None

    def tick(self, now: float) -> None:
        if self.busy_until is not None and now >= self.busy_until:
            logger.warning("No stop notice from 223; assuming the gesture ended")
            self.busy_until = None
        if self.busy_until is None and self.waiting:
            name, line = self.waiting.pop(0)
            if self._start(name, line):
                self.busy_until = now + self._timeout_s


def follow_222(host: str, session_dir: str, out: queue.Queue[tuple[str, Any]]) -> None:
    while True:
        proc = subprocess.Popen(
            [
                "ssh",
                "-o",
                "ServerAliveInterval=5",
                "-o",
                "ServerAliveCountMax=2",
                host,
                "python3",
                "-u",
                "-",
                session_dir,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        assert proc.stdin is not None and proc.stdout is not None
        proc.stdin.write(_FOLLOW)
        proc.stdin.close()
        logger.info("Following 222", host=host, session_dir=session_dir)
        for line in proc.stdout:
            event = json.loads(line)
            logger.info("222 event", kind=event.get("kind"), reason=event.get("reason"))
            out.put(("event", event))
        logger.warning("Lost 222's events; reconnecting", exit_code=proc.wait())
        time.sleep(RECONNECT_S)


def follow_223(url: str, out: queue.Queue[tuple[str, Any]]) -> None:
    """The MCP server's tool-stream: perform_gesture's result and its stop notice."""
    while True:
        try:
            with requests.get(url, stream=True, timeout=(5, 60)) as resp:
                for raw in resp.iter_lines(decode_unicode=True):
                    if not raw or not raw.startswith("data: "):
                        continue
                    msg = json.loads(raw[len("data: ") :])
                    params = msg.get("params") or {}
                    if msg.get("method") == TOOL_STREAM_STOPPED_METHOD:
                        if params.get("tool_name") == TOOL:
                            out.put(("stopped", None))
                    elif params.get("logger") == TOOL:
                        logger.info("223 result", text=params.get("data"))
        except requests.RequestException as exc:
            logger.warning("223's MCP stream down; reconnecting", error=str(exc))
        time.sleep(RECONNECT_S)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--session-dir", required=True, help="the loop's --session-dir, as a path on 222"
    )
    parser.add_argument("--host", default="pc2_222", help="ssh host running 222's loop")
    parser.add_argument("--mcp-url", default=DEFAULT_MCP_URL, help="223's MCP endpoint")
    parser.add_argument(
        "--take-side", choices=["left", "right"], default="right", help="222, from 223"
    )
    parser.add_argument("--entrance-side", choices=["left", "right"], default="right")
    args = parser.parse_args()

    mcp = McpAdapter(args.mcp_url)

    def start(name: str, line: str) -> bool:
        try:
            text = mcp.call_tool_text(TOOL, {"name": name, "line": line})
        except (requests.RequestException, McpError) as exc:
            logger.error("Gesture not sent: 223's MCP unreachable", gesture=name, error=str(exc))
            return False
        started = text.startswith(f"{TOOL} started")
        (logger.info if started else logger.error)(
            "Gesture call", gesture=name, line=line, reply=text
        )
        return started

    inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
    threading.Thread(
        target=follow_222, args=(args.host, args.session_dir, inbox), daemon=True
    ).start()
    threading.Thread(target=follow_223, args=(args.mcp_url, inbox), daemon=True).start()
    cuer = Cuer(start, args.take_side, args.entrance_side)
    while True:
        try:
            kind, item = inbox.get(timeout=0.5)
            if kind == "event":
                cuer.on_event(item)
            else:
                cuer.on_stopped()
        except queue.Empty:
            pass
        cuer.tick(time.monotonic())


if __name__ == "__main__":
    main()
