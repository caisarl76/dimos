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

from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import random
import threading
import time
from typing import Any

from dimos.robot.unitree.g1.greeter import GestureStream, acts, run, shuffled


def test_shuffled_plays_each_act_once_per_cycle_and_never_twice_in_a_row() -> None:
    items = acts("right", "left")
    assert [name for name, _ in items] == ["invite", "take_right", "entrance_left", "wave"]
    plays = shuffled(items, random.Random(0))
    seen = [next(plays) for _ in range(4 * 200)]
    for i in range(0, len(seen), 4):
        assert sorted(seen[i : i + 4]) == sorted(items)
    assert all(a != b for a, b in itertools.pairwise(seen))
    assert len({tuple(seen[i : i + 4]) for i in range(0, len(seen), 4)}) > 10  # orders do vary


def test_run_waits_for_each_act_to_end() -> None:
    stream = GestureStream("http://unused/mcp")
    sent: list[str] = []
    replies = iter([True, False, True, True])  # the second is refused: skipped

    def start(name: str, line: str) -> bool:
        sent.append(name)
        if name == "c":  # its stop notice comes later
            threading.Timer(0.1, stream.ended.set).start()
        return next(replies)

    threading.Timer(0.3, stream.up.set).start()  # nothing is sent before the stream is up
    began = time.monotonic()
    run([("a", "1"), ("b", "2"), ("c", "3"), ("d", "4")], start, stream, 0, 1.0, 0)
    took = time.monotonic() - began
    assert sent == ["a", "b", "c", "d"]
    # up (0.3) + a's missing stop notice (timeout 1.0) + c's stop notice (0.1) + d's timeout (1.0)
    assert 2.3 < took < 3.2


def _server(do_get: Callable[[BaseHTTPRequestHandler], None]) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            do_get(self)

        def log_message(self, *args: Any) -> None:
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _stream(server: ThreadingHTTPServer) -> GestureStream:
    return GestureStream(f"http://127.0.0.1:{server.server_port}/mcp")


def _timed_close(stream: GestureStream) -> float:
    began = time.monotonic()
    stream.close()
    return time.monotonic() - began


def test_stream_reports_stop_notices_and_close_ends_a_blocked_stream() -> None:
    release = threading.Event()

    def sse(handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header("Content-Type", "text/event-stream")
        handler.end_headers()
        handler.wfile.write(b": connected\n\n")
        handler.wfile.write(
            b'data: {"method": "dimos/tool_stopped", "params": {"tool_name": "x"}}\n\n'
        )
        handler.wfile.flush()
        time.sleep(0.2)
        handler.wfile.write(
            b'data: {"method": "dimos/tool_stopped", "params": {"tool_name": "perform_gesture"}}\n\n'
        )
        handler.wfile.flush()
        release.wait(60)  # then silent: the client's read blocks

    server = _server(sse)
    stream = _stream(server)
    try:
        stream.start()
        assert stream.up.wait(30)
        assert not stream.ended.is_set()  # another tool's stop is not ours
        assert stream.ended.wait(30)
    finally:
        took = _timed_close(stream)
        release.set()
        server.shutdown()
        server.server_close()
    assert not stream._thread.is_alive()
    assert took < 2, f"close() took {took:.1f} s: it waited the stream out instead of ending it"


def test_close_cuts_a_header_wait() -> None:
    asked, release = threading.Event(), threading.Event()

    def stall(handler: BaseHTTPRequestHandler) -> None:
        asked.set()
        release.wait(60)  # no headers yet

    server = _server(stall)
    stream = _stream(server)
    try:
        stream.start()
        assert asked.wait(30)
    finally:
        took = _timed_close(stream)
        release.set()
        server.shutdown()
        server.server_close()
    assert not stream._thread.is_alive()
    assert took < 2, f"close() took {took:.1f} s: it waited the header wait out"


def test_a_lost_stream_ends_the_wait_and_close_in_the_reconnect_wait_asks_no_more() -> None:
    asks: list[float] = []

    def refuse(handler: BaseHTTPRequestHandler) -> None:
        asks.append(time.monotonic())
        handler.send_error(500)

    server = _server(refuse)
    stream = _stream(server)
    try:
        stream.start()
        assert stream.ended.wait(30)  # now in the 3 s reconnect wait
        assert not stream.up.is_set()
    finally:
        took = _timed_close(stream)
        server.shutdown()
        server.server_close()
    assert len(asks) == 1
    assert not stream._thread.is_alive()
    assert took < 2
