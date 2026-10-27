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

    trial_end "handover complete"          -> take_<side> + 07_take (222 holds the bottle out)
    taken_confirmed                        -> entrance_<side> + 10_entrance ("... have a great time")
    trial_end otherwise, a freeze, the end -> nothing more this round

Rounds take about 27 s, so only these two (about 15 s each) fit. One gesture runs at a time and is never
cut short; one cue waits, and a newer cue replaces it, so a late take gives way to the entrance. A refused
or failed gesture is logged and dropped. Events written while ssh is down are skipped, and losing 223's
MCP stream drops the waiting cue and every cue until it is back. Ctrl-C or SIGTERM ends the ssh and its follower on 222. It never calls
stop_arm and never touches the controller. Run it after 223's blueprint is up:

    python -m dimos.robot.unitree.g1.handover_cues \\
        --session-dir '~/GR00T-WholeBodyControl-loop-20261026/sessions/<the loop's --session-dir>'
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import http.client
import json
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Literal
from urllib.parse import urlsplit

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
# No stop notice by then: assume it ended (perform_gesture plans are <=45 s).
GESTURE_TIMEOUT_S = 60.0
CONNECT_S = 5.0  # 223's stream: connect, then headers; the only wait close() cannot cut
STREAM_TIMEOUT_S = (
    60.0  # the server sends a keepalive every 20 s; silence this long means it is gone
)
JOIN_S = 2 * CONNECT_S

Side = Literal["left", "right"]
Cue = list[tuple[str, str]]

# Runs on 222 (its system python3): prints the events appended to trial_*_evidence.jsonl from now on,
# one JSON object per line, in the order they were written, then a blank line each pass. The first blank
# line says the starting point is taken; a failed one ends the script once ssh is gone.
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
            except ValueError as exc:
                print("skipped a malformed record in %s: %s" % (p, exc), file=sys.stderr, flush=True)
    for e in sorted(events, key=lambda e: e.get("at", 0)):
        print(json.dumps(e))
    print(flush=True)
    time.sleep(0.2)
"""


def cue_of(event: dict[str, Any], take_side: Side, entrance_side: Side) -> Cue | None:
    """The gestures an event cues ([] clears the waiting cue), or None when it is not a cue."""
    kind = event.get("kind")
    if kind == "trial_end":
        return [(f"take_{take_side}", "07_take")] if event.get("reason") == HANDOVER else []
    if kind == "taken_confirmed":
        return [(f"entrance_{entrance_side}", "10_entrance")]
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
        self.connected = False  # 223's MCP stream; without it a gesture's end is never heard

    def on_event(self, event: dict[str, Any]) -> None:
        cue = cue_of(event, *self._sides)
        if cue is None:
            return
        if cue and not self.connected:
            logger.info("Cue dropped: 223's MCP stream is down", dropped=cue)
            cue = []
        if self.waiting:
            logger.info("Cue replaced", dropped=self.waiting, by=event.get("kind"))
        self.waiting = cue

    def on_stopped(self) -> None:
        self.busy_until = None

    def on_mcp(self, connected: bool) -> None:
        """While 223's stream is down its state is unknown: cues are dropped, nothing is waited on."""
        if connected != self.connected:
            logger.info("223's MCP stream " + ("connected" if connected else "lost"))
        self.connected = connected
        if not connected:
            if self.waiting:
                logger.info("Cue dropped: lost 223's MCP stream", dropped=self.waiting)
            self.waiting = []
            self.busy_until = None

    def tick(self, now: float) -> None:
        if self.busy_until is not None and now >= self.busy_until:
            logger.warning("No stop notice from 223; assuming the gesture ended")
            self.busy_until = None
        if self.busy_until is None and self.waiting:
            name, line = self.waiting.pop(0)
            if self._start(name, line):
                self.busy_until = now + self._timeout_s


def step(cuer: Cuer, inbox: queue.Queue[tuple[str, Any]], timeout: float) -> None:
    """Apply every message already in, then start what waits: a freeze that came with a stop notice wins."""
    messages: list[tuple[str, Any]] = []
    try:
        messages.append(inbox.get(timeout=timeout))
        while True:
            messages.append(inbox.get_nowait())
    except queue.Empty:
        pass
    for kind, item in messages:
        if kind == "event":
            cuer.on_event(item)
        elif kind == "stopped":
            cuer.on_stopped()
        else:
            cuer.on_mcp(item)
    cuer.tick(time.monotonic())


class Followers:
    """222's events over ssh and 223's MCP tool stream, each on its own thread, into one inbox."""

    def __init__(self, host: str, session_dir: str, mcp_url: str) -> None:
        self.inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._command = ["ssh", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2"]
        self._command += [host, "python3", "-u", "-", session_dir]
        self._mcp_url = mcp_url
        self._stop = threading.Event()
        self._lock = threading.Lock()  # close() against a follower registering its ssh or stream
        self._ssh: subprocess.Popen[str] | None = None
        self._sse: socket.socket | None = None
        self._threads = [
            threading.Thread(target=self._follow_222, name="follow-222", daemon=True),
            threading.Thread(target=self._follow_223, name="follow-223", daemon=True),
        ]

    def start(self) -> None:
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        """Stop both followers: ends and reaps the ssh, ends the stream, joins the threads.

        The stream's socket is shut down, not closed: that wakes a read blocked in another thread.
        """
        with self._lock:
            self._stop.set()
            ssh, sse = self._ssh, self._sse
        if ssh is not None:
            ssh.terminate()
        if sse is not None:
            try:
                sse.shutdown(socket.SHUT_RDWR)
            except OSError:  # the stream already ended
                pass
        for thread in self._threads:
            thread.join(timeout=JOIN_S)

    def _follow_222(self) -> None:
        while True:
            with self._lock:
                if self._stop.is_set():
                    return
                self._ssh = proc = subprocess.Popen(
                    self._command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
                )
            assert proc.stdin is not None and proc.stdout is not None
            try:
                try:
                    proc.stdin.write(_FOLLOW)
                    proc.stdin.close()
                except OSError:  # ssh already gone: its stdout ends at once
                    pass
                logger.info("Following 222", command=" ".join(self._command))
                for line in proc.stdout:
                    if not line.strip():
                        continue
                    try:
                        event = json.loads(line)
                    except ValueError:
                        logger.warning("Not an event from 222", line=line.strip())
                        continue
                    logger.info("222 event", kind=event.get("kind"), reason=event.get("reason"))
                    self.inbox.put(("event", event))
            finally:
                proc.terminate()
                code = proc.wait()
            if not self._stop.is_set():
                logger.warning("Lost 222's events; reconnecting", exit_code=code)
            self._stop.wait(RECONNECT_S)

    def _follow_223(self) -> None:
        """perform_gesture's result and its stop notice; tells the Cuer when the stream is up or lost."""
        url = urlsplit(self._mcp_url)
        while True:
            with self._lock:
                if self._stop.is_set():
                    return
            try:
                self._stream_223(url.hostname or "localhost", url.port or 80, url.path or "/")
            except Exception as exc:  # any end of the stream, close()'s shutdown included
                if not self._stop.is_set():
                    logger.warning("223's MCP stream down; reconnecting", error=repr(exc))
            if self._stop.is_set():
                return
            self.inbox.put(("mcp", False))
            self._stop.wait(RECONNECT_S)

    def _stream_223(self, host: str, port: int, path: str) -> None:
        # http.client on a socket opened here, not requests: the socket is registered before the request
        # goes out, so close() can cut the header wait as well as the stream.
        sock = socket.create_connection((host, port), timeout=CONNECT_S)
        try:
            with self._lock:
                if self._stop.is_set():
                    return
                self._sse = sock
            conn = http.client.HTTPConnection(host, port)
            conn.sock = sock
            conn.request("GET", path, headers={"Accept": "text/event-stream"})
            resp = conn.getresponse()  # headers within CONNECT_S
            if resp.status != 200:
                raise ConnectionError(f"HTTP {resp.status} {resp.reason}")
            sock.settimeout(STREAM_TIMEOUT_S)
            self.inbox.put(("mcp", True))
            for raw in resp:
                line = raw.decode().rstrip("\r\n")
                if not line.startswith("data: "):
                    continue
                msg = json.loads(line[len("data: ") :])
                params = msg.get("params") or {}
                if msg.get("method") == TOOL_STREAM_STOPPED_METHOD:
                    if params.get("tool_name") == TOOL:
                        self.inbox.put(("stopped", None))
                elif params.get("logger") == TOOL:
                    logger.info("223 result", text=params.get("data"))
        finally:
            with self._lock:
                self._sse = None
            sock.close()


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

    cuer = Cuer(start, args.take_side, args.entrance_side)
    followers = Followers(args.host, args.session_dir, args.mcp_url)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # so a SIGTERM also reaps the ssh
    followers.start()
    try:
        while True:
            step(cuer, followers.inbox, timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        followers.close()


if __name__ == "__main__":
    main()
