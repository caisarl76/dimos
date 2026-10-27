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

import json
from pathlib import Path
import subprocess
import sys
import time

from dimos.robot.unitree.g1.handover_cues import _FOLLOW, HANDOVER, Cuer, cue_of


def test_cue_of() -> None:
    assert cue_of({"kind": "trial_end", "reason": HANDOVER}, "left", "right") == [
        ("take_left", "07_take")
    ]
    assert cue_of({"kind": "trial_end", "reason": "watchdog"}, "right", "right") == []
    assert cue_of({"kind": "taken_confirmed"}, "right", "left") == [
        ("entrance_left", "10_entrance")
    ]
    assert cue_of({"kind": "loop_freeze"}, "right", "right") == []
    assert cue_of({"kind": "trial_start"}, "right", "right") is None
    assert cue_of({"kind": "rearm_decision"}, "right", "right") is None


def test_one_runs_and_the_newest_cue_waits() -> None:
    sent: list[str] = []
    accept = True

    def start(name: str, line: str) -> bool:
        sent.append(name)
        return accept

    cuer = Cuer(start, "right", "right", timeout_s=60)
    handover = {"kind": "trial_end", "reason": HANDOVER}

    cuer.on_event(handover)
    cuer.tick(0)
    assert sent == ["take_right"]
    cuer.on_event({"kind": "taken_confirmed"})
    cuer.on_event({"kind": "trial_start"})  # the next round does not drop the waiting entrance
    cuer.tick(1)
    assert sent == ["take_right"]  # never cut short
    cuer.on_stopped()
    cuer.tick(2)
    assert sent == ["take_right", "entrance_right"]

    cuer.on_event(handover)  # 223 is behind: the late take gives way to the entrance
    cuer.on_event({"kind": "taken_confirmed"})
    cuer.on_stopped()
    cuer.tick(3)
    assert sent == ["take_right", "entrance_right", "entrance_right"]

    cuer.on_event(handover)
    cuer.on_event({"kind": "loop_freeze"})  # a freeze drops the waiting take
    cuer.on_stopped()
    cuer.tick(4)
    assert sent[-1] == "entrance_right" and not cuer.waiting

    accept = False  # refused: dropped, not retried
    cuer.on_event(handover)
    cuer.tick(5)
    cuer.tick(6)
    assert sent.count("take_right") == 2 and cuer.busy_until is None

    accept = True  # a lost stop notice frees 223 after the timeout
    cuer.on_event(handover)
    cuer.tick(10)
    cuer.on_event({"kind": "taken_confirmed"})
    cuer.tick(69)
    assert sent[-1] == "take_right"
    cuer.tick(70)
    assert sent[-1] == "entrance_right"


def test_follow_prints_new_whole_lines_in_order(tmp_path: Path) -> None:
    old = tmp_path / "trial_001_evidence.jsonl"
    old.write_text(json.dumps({"kind": "before", "at": 0}) + "\n")
    proc = subprocess.Popen(
        [sys.executable, "-u", "-c", _FOLLOW, str(tmp_path)], stdout=subprocess.PIPE, text=True
    )
    try:
        time.sleep(0.5)
        with old.open("a") as f:
            f.write(json.dumps({"kind": "taken_confirmed", "at": 2}) + "\n")
            f.write('{"kind": "to_ready", "at"')  # still being written
        (tmp_path / "trial_002_evidence.jsonl").write_text(
            json.dumps({"kind": "rearm_begin", "at": 3}) + "\n"
        )
        time.sleep(0.5)
        with old.open("a") as f:
            f.write(": 2.5}\n")
        time.sleep(0.5)
    finally:
        proc.kill()
    kinds = [json.loads(line)["kind"] for line in proc.communicate()[0].splitlines()]
    assert kinds == ["taken_confirmed", "rearm_begin", "to_ready"]
