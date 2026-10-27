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
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import IO, Any

from dimos.robot.unitree.g1.handover_cues import _FOLLOW, HANDOVER, Cuer, Followers, cue_of, step


def test_cue_of() -> None:
    assert cue_of({"kind": "trial_start"}, "left", "right") == [
        ("take_left", "07_take"),
        ("entrance_right", "10_entrance"),
    ]
    assert cue_of({"kind": "trial_end", "reason": HANDOVER}, "right", "right") is None
    assert cue_of({"kind": "trial_end", "reason": "watchdog"}, "right", "right") == []
    assert cue_of({"kind": "loop_freeze"}, "right", "right") == []
    assert cue_of({"kind": "taken_confirmed"}, "right", "right") is None
    assert cue_of({"kind": "rearm_decision"}, "right", "right") is None


def test_one_runs_and_the_newest_cue_waits() -> None:
    sent: list[str] = []
    accept = True

    def start(name: str, line: str) -> bool:
        sent.append(name)
        return accept

    cuer = Cuer(start, "right", "right", timeout_s=60)
    cuer.on_mcp(True)
    trial = {"kind": "trial_start"}

    cuer.on_event(trial)
    cuer.tick(0)
    assert sent == ["take_right"]
    cuer.on_event({"kind": "trial_end", "reason": HANDOVER})  # the entrance keeps waiting
    cuer.tick(1)
    assert sent == ["take_right"]  # never cut short
    cuer.on_stopped()
    cuer.tick(2)
    assert sent == ["take_right", "entrance_right"]

    cuer.on_event(trial)  # the next round, while the entrance runs
    cuer.on_stopped()
    cuer.tick(3)
    cuer.on_event({"kind": "trial_end", "reason": "watchdog"})  # no handover: no entrance
    cuer.on_stopped()
    cuer.tick(4)
    assert sent == ["take_right", "entrance_right", "take_right"] and not cuer.waiting

    cuer.on_event(trial)
    cuer.tick(5)
    cuer.on_event(trial)  # 223 is behind: the newer round replaces the waiting entrance
    cuer.on_stopped()
    cuer.tick(6)
    assert sent[-2:] == ["take_right", "take_right"]
    cuer.on_event({"kind": "loop_freeze"})  # a freeze drops the waiting entrance
    cuer.on_stopped()
    cuer.tick(7)
    assert sent.count("entrance_right") == 1 and not cuer.waiting

    accept = False  # refused: dropped, not retried; the next in line is tried
    cuer.on_event(trial)
    cuer.tick(8)
    cuer.tick(9)
    assert sent[-2:] == ["take_right", "entrance_right"] and cuer.busy_until is None

    accept = True  # a lost stop notice frees 223 after the timeout
    cuer.on_event(trial)
    cuer.tick(10)
    cuer.tick(69)
    assert sent[-1] == "take_right"
    cuer.tick(70)
    assert sent[-1] == "entrance_right"


def test_step_applies_everything_in_before_starting() -> None:
    sent: list[str] = []

    def start(name: str, line: str) -> bool:
        sent.append(name)
        return True

    cuer = Cuer(start, "right", "right")
    inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
    inbox.put(("mcp", True))
    inbox.put(("event", {"kind": "trial_start"}))
    step(cuer, inbox, timeout=0)
    assert sent == ["take_right"] and cuer.waiting

    inbox.put(("stopped", None))  # the take ends and 222 freezes in the same breath
    inbox.put(("event", {"kind": "loop_freeze"}))
    step(cuer, inbox, timeout=0)
    assert sent == ["take_right"] and not cuer.waiting and cuer.busy_until is None


def test_cues_from_an_outage_are_dropped() -> None:
    sent: list[str] = []

    def start(name: str, line: str) -> bool:
        sent.append(name)
        return True

    cuer = Cuer(start, "right", "right")
    inbox: queue.Queue[tuple[str, Any]] = queue.Queue()
    inbox.put(("event", {"kind": "trial_start"}))  # before the stream is up
    step(cuer, inbox, timeout=0)
    assert sent == []

    inbox.put(("mcp", True))
    inbox.put(("event", {"kind": "trial_start"}))
    step(cuer, inbox, timeout=0)
    inbox.put(("mcp", False))  # lost while the entrance waits, and a round starts in the outage
    inbox.put(("event", {"kind": "trial_start"}))
    step(cuer, inbox, timeout=0)
    assert sent == ["take_right"] and not cuer.waiting and cuer.busy_until is None

    inbox.put(("event", {"kind": "trial_start"}))  # still down, though a POST may work
    inbox.put(("mcp", True))
    step(cuer, inbox, timeout=0)
    assert sent == ["take_right"]
    inbox.put(("event", {"kind": "trial_start"}))  # back: new cues play
    step(cuer, inbox, timeout=0)
    assert sent == ["take_right", "take_right"]


def _reader(stream: IO[str]) -> tuple[queue.Queue[str], threading.Thread]:
    """Lines from stream on a queue, so every read in a test has a deadline."""
    lines: queue.Queue[str] = queue.Queue()

    def read() -> None:
        for line in stream:
            lines.put(line)

    thread = threading.Thread(target=read, daemon=True)
    thread.start()
    return lines, thread


def _next(lines: queue.Queue[str], deadline: float) -> str:
    try:
        return lines.get(timeout=max(0.0, deadline - time.monotonic()))
    except queue.Empty:
        raise AssertionError("no line from the follower before the deadline") from None


def _kinds_until(lines: queue.Queue[str], kinds: set[str], deadline: float) -> list[str]:
    """The event kinds read until all of kinds are seen (the follower prints a blank line each pass)."""
    seen: list[str] = []
    while not kinds <= set(seen):
        line = _next(lines, deadline)
        if line.strip():
            seen.append(json.loads(line)["kind"])
    return seen


def test_follow_prints_new_whole_lines_in_order(tmp_path: Path) -> None:
    old = tmp_path / "trial_001_evidence.jsonl"
    old.write_text(json.dumps({"kind": "before", "at": 0}) + "\n")
    proc = subprocess.Popen(
        [sys.executable, "-u", "-c", _FOLLOW, str(tmp_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert proc.stdout is not None and proc.stderr is not None
    lines, reader = _reader(proc.stdout)
    deadline = time.monotonic() + 30
    try:
        assert _next(lines, deadline) == "\n"  # the starting point is taken
        with old.open("a") as f:
            f.write(json.dumps({"kind": "taken_confirmed", "at": 2}) + "\n")
            f.write("not json\n")
            f.write('{"kind": "to_ready", "at"')  # still being written
        (tmp_path / "trial_002_evidence.jsonl").write_text(
            json.dumps({"kind": "rearm_begin", "at": 3}) + "\n"
        )
        first = _kinds_until(lines, {"taken_confirmed", "rearm_begin"}, deadline)
        with old.open("a") as f:
            f.write(": 2.5}\n")
        rest = _kinds_until(lines, {"to_ready"}, deadline)
    finally:
        proc.kill()
        proc.wait()
        reader.join(timeout=10)
        stderr = proc.stderr.read()
        proc.stdout.close()
        proc.stderr.close()
    assert first + rest == ["taken_confirmed", "rearm_begin", "to_ready"]
    assert "malformed record in " + str(old) in stderr


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


def _followers(tmp_path: Path, server: ThreadingHTTPServer) -> Followers:
    followers = Followers("unused", str(tmp_path), f"http://127.0.0.1:{server.server_port}/mcp")
    followers._command = [sys.executable, "-u", "-", str(tmp_path)]  # the remote side, run here
    return followers


def _timed_close(followers: Followers) -> float:
    began = time.monotonic()
    followers.close()
    return time.monotonic() - began


def test_close_ends_the_ssh_and_a_blocked_stream(tmp_path: Path) -> None:
    release = threading.Event()

    def stream(handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header("Content-Type", "text/event-stream")
        handler.end_headers()
        handler.wfile.write(b": connected\n\n")
        handler.wfile.flush()
        release.wait(60)  # then silent: the client's read blocks

    server = _server(stream)
    followers = _followers(tmp_path, server)
    follower = None
    try:
        followers.start()
        assert followers.inbox.get(timeout=30) == ("mcp", True)
        deadline = time.monotonic() + 30
        while (follower := followers._ssh) is None:
            assert time.monotonic() < deadline, "the follower never started"
            time.sleep(0.01)
    finally:
        took = _timed_close(followers)
        release.set()
        server.shutdown()
        server.server_close()
    assert follower.poll() is not None
    assert not any(thread.is_alive() for thread in followers._threads)
    assert took < 2, f"close() took {took:.1f} s: it waited the stream out instead of ending it"


def test_close_cuts_a_header_wait(tmp_path: Path) -> None:
    asked, release = threading.Event(), threading.Event()

    def stall(handler: BaseHTTPRequestHandler) -> None:
        asked.set()
        release.wait(60)  # no headers yet

    server = _server(stall)
    followers = _followers(tmp_path, server)
    try:
        followers.start()
        assert asked.wait(30)
    finally:
        took = _timed_close(followers)
        release.set()
        server.shutdown()
        server.server_close()
    assert not any(thread.is_alive() for thread in followers._threads)
    assert took < 2, f"close() took {took:.1f} s: it waited the header wait out"


def test_close_in_the_reconnect_wait_asks_no_more(tmp_path: Path) -> None:
    asks: list[float] = []

    def refuse(handler: BaseHTTPRequestHandler) -> None:
        asks.append(time.monotonic())
        handler.send_error(500)

    server = _server(refuse)
    followers = _followers(tmp_path, server)
    try:
        followers.start()
        assert followers.inbox.get(timeout=30) == ("mcp", False)  # now in the 3 s reconnect wait
    finally:
        took = _timed_close(followers)
        server.shutdown()
        server.server_close()
    assert len(asks) == 1
    assert not any(thread.is_alive() for thread in followers._threads)
    assert took < 2
