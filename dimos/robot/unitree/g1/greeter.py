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

"""Play 223's demo gestures and lines to the crowd in a random order, hands-free.

Calls perform_gesture on 223's MCP server (unitree-g1-sonic-arm, on this workstation), on its own:

    invite + 06_invite              greet, and point people to 222's water
    take_<side> + 07_take           point at 222's bottle: take it
    entrance_<side> + 10_entrance   show the way to the entrance
    wave + 11_wave                  wave: thanks for coming

Each cycle plays all four once in a fresh random order, and the same act never plays twice in a row.
Each act waits for 223's stop notice from the MCP tool stream, then a pause. A refused or failed act is
logged and skipped; while the stream is down nothing is sent. Ctrl-C or SIGTERM ends the loop and leaves a
running gesture to finish: `dimos mcp call stop_arm` stops it. It never touches the controller. Run it
after 223's blueprint is up:

    python -m dimos.robot.unitree.g1.greeter
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterable, Iterator
import http.client
import json
import random
import signal
import socket
import sys
import threading
import time
from typing import Literal
from urllib.parse import urlsplit

import requests

from dimos.agents.mcp.constants import DEFAULT_MCP_URL
from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.agents.mcp.tool_stream import TOOL_STREAM_STOPPED_METHOD
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TOOL = "perform_gesture"
RECONNECT_S = 3.0
RETRY_S = 5.0  # after a refused or failed act
# No stop notice by then: assume it ended (perform_gesture plans are <=45 s).
GESTURE_TIMEOUT_S = 60.0
CONNECT_S = 5.0  # 223's stream: connect, then headers; the only wait close() cannot cut
STREAM_TIMEOUT_S = (
    60.0  # the server sends a keepalive every 20 s; silence this long means it is gone
)
JOIN_S = 2 * CONNECT_S

Side = Literal["left", "right"]
Act = tuple[str, str]  # gesture, line


def acts(take_side: Side, entrance_side: Side) -> list[Act]:
    return [
        ("invite", "06_invite"),
        (f"take_{take_side}", "07_take"),
        (f"entrance_{entrance_side}", "10_entrance"),
        ("wave", "11_wave"),
    ]


def shuffled(items: list[Act], rng: random.Random) -> Iterator[Act]:
    """Endless cycles of every item once, each in a fresh order, never the same item twice in a row."""
    last = None
    while True:
        cycle = items[:]
        rng.shuffle(cycle)
        if cycle[0] == last:
            cycle[0], cycle[-1] = cycle[-1], cycle[0]
        yield from cycle
        last = cycle[-1]


class GestureStream:
    """223's MCP tool stream on a thread: `up` while it is connected, `ended` on a stop notice or a loss."""

    def __init__(self, mcp_url: str) -> None:
        self.up = threading.Event()
        self.ended = threading.Event()
        self._mcp_url = mcp_url
        self._stop = threading.Event()
        self._lock = threading.Lock()  # close() against the thread registering its socket
        self._sse: socket.socket | None = None
        self._thread = threading.Thread(target=self._follow, name="follow-223", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        """End the stream and join the thread.

        The socket is shut down, not closed: that wakes a read blocked in the other thread.
        """
        with self._lock:
            self._stop.set()
            sse = self._sse
        if sse is not None:
            try:
                sse.shutdown(socket.SHUT_RDWR)
            except OSError:  # the stream already ended
                pass
        self._thread.join(timeout=JOIN_S)

    def _follow(self) -> None:
        url = urlsplit(self._mcp_url)
        while True:
            with self._lock:
                if self._stop.is_set():
                    return
            try:
                self._stream(url.hostname or "localhost", url.port or 80, url.path or "/")
            except Exception as exc:  # any end of the stream, close()'s shutdown included
                if not self._stop.is_set():
                    logger.warning("223's MCP stream down; reconnecting", error=repr(exc))
            self.up.clear()
            self.ended.set()  # a gesture's end can no longer be heard
            if self._stop.wait(RECONNECT_S):
                return

    def _stream(self, host: str, port: int, path: str) -> None:
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
            logger.info("223's MCP stream connected")
            self.up.set()
            for raw in resp:
                line = raw.decode().rstrip("\r\n")
                if not line.startswith("data: "):
                    continue
                msg = json.loads(line[len("data: ") :])
                params = msg.get("params") or {}
                if msg.get("method") == TOOL_STREAM_STOPPED_METHOD:
                    if params.get("tool_name") == TOOL:
                        self.ended.set()
                elif params.get("logger") == TOOL:
                    logger.info("223 result", text=params.get("data"))
        finally:
            with self._lock:
                self._sse = None
            sock.close()


def run(
    plays: Iterable[Act],
    start: Callable[[str, str], bool],
    stream: GestureStream,
    pause_s: float,
    timeout_s: float = GESTURE_TIMEOUT_S,
    retry_s: float = RETRY_S,
) -> None:
    """Play each act once 223's stream is up and the last one has ended, pause_s apart."""
    for name, line in plays:
        if not stream.up.is_set():
            logger.info("Waiting for 223's MCP stream")
            stream.up.wait()
        stream.ended.clear()
        if not start(name, line):
            time.sleep(retry_s)
            continue
        if not stream.ended.wait(timeout_s):
            logger.warning("No stop notice from 223; assuming the gesture ended", gesture=name)
        time.sleep(pause_s)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--mcp-url", default=DEFAULT_MCP_URL, help="223's MCP endpoint")
    parser.add_argument(
        "--take-side", choices=["left", "right"], default="right", help="222, from 223"
    )
    parser.add_argument("--entrance-side", choices=["left", "right"], default="right")
    parser.add_argument("--pause-s", type=float, default=3.0, help="rest between acts")
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

    stream = GestureStream(args.mcp_url)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # so a SIGTERM also ends the stream
    stream.start()
    plays = shuffled(acts(args.take_side, args.entrance_side), random.Random())
    try:
        run(plays, start, stream, args.pause_s)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()


if __name__ == "__main__":
    main()
