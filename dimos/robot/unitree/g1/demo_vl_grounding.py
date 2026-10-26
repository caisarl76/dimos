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

"""Measure how well a vision model grounds objects in labelled G1 head-camera frames.

    python dimos/robot/unitree/g1/demo_vl_grounding.py --frames DIR [--model NAME]
        [--box-scale pixels|normalized_1000] [--base-url URL] [--env-file FILE --key-var NAME]

DIR holds 640x480 `*.png` frames and `labels.json`:
`{"frame.png": {"query": "red box", "box": [x1, y1, x2, y2]}}`. Each frame is queried through
`query_boxes`, the largest box is chosen (as `point_at_object` does), and the frame passes when
its centre is within 30 px of the labelled box's centre. The OpenAI-compatible endpoint comes from
--base-url (default: OPENAI_BASE_URL). The API key comes from OPENAI_API_KEY, or with --env-file
from that file's single `<key-var>=` line (nothing else is read); it is never printed.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import get_args

from openai import AuthenticationError

from dimos.models.vl.openai import OpenAIVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.g1.sonic_arm_reference import HEAD_CAMERA_SIZE
from dimos.robot.unitree.g1.sonic_arm_skill import BoxScale, query_boxes

PASS_PX = 30.0


def load_key(env_file: Path, var: str) -> str:
    """Put the env file's `var` into OPENAI_API_KEY (this process only); nothing else is read."""
    prefix = f"{var}="
    with env_file.open() as f:
        for line in f:
            if line.startswith(prefix):
                key = line[len(prefix) :].strip().strip("\"'")
                if key:
                    os.environ["OPENAI_API_KEY"] = key
                    return key
    sys.exit(f"error: no non-empty {var} line in {env_file}")


def centre(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return (x1 + x2) / 2, (y1 + y2) / 2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--model", default="openai/gpt-5.6-luna")
    parser.add_argument("--box-scale", choices=get_args(BoxScale), default="pixels")
    parser.add_argument("--base-url", help="OpenAI-compatible endpoint (default: OPENAI_BASE_URL)")
    parser.add_argument("--env-file", type=Path, help="read the API key from this file")
    parser.add_argument("--key-var", default="OPENAI_API_KEY", help="the key's name in --env-file")
    args = parser.parse_args()

    if args.base_url:
        os.environ["OPENAI_BASE_URL"] = args.base_url
    if args.env_file is not None:
        if not args.env_file.is_file():
            sys.exit(f"error: env file {args.env_file} not found")
        key = load_key(args.env_file, args.key_var)
    elif not (key := os.environ.get("OPENAI_API_KEY", "")):
        sys.exit("error: set OPENAI_API_KEY or pass --env-file")
    labels = json.loads((args.frames / "labels.json").read_text())
    vl = OpenAIVlModel(model_name=args.model)

    print(f"model={args.model} box_scale={args.box_scale} pass<={PASS_PX:g}px")
    passed = 0
    for name, label in sorted(labels.items()):
        query = label["query"]
        prefix = f"{name} '{query}':"
        path = args.frames / name
        if not path.is_file():
            print(f"{prefix} FAIL (missing frame)")
            continue
        image = Image.from_file(path)
        if (image.width, image.height) != HEAD_CAMERA_SIZE:
            print(f"{prefix} FAIL (frame is {image.width}x{image.height}, not 640x480)")
            continue
        try:
            boxes = query_boxes(vl, image, query, args.box_scale)
        except AuthenticationError:
            print(f"{prefix} FAIL (vision call raised AuthenticationError)")  # may echo key parts
            continue
        except Exception as exc:
            reason = str(exc).replace(key, "***")
            print(f"{prefix} FAIL (vision call raised {type(exc).__name__}: {reason})")
            continue
        if not boxes:
            print(f"{prefix} FAIL (no box)")
            continue
        chosen = max(boxes, key=lambda b: (b[3] - b[1]) * (b[4] - b[2]))
        cx, cy = centre(*chosen[1:])
        tx, ty = centre(*label["box"])
        err = math.hypot(cx - tx, cy - ty)
        ok = err <= PASS_PX
        passed += ok
        all_boxes = ", ".join(f"{b[0]}[{b[1]:.0f},{b[2]:.0f},{b[3]:.0f},{b[4]:.0f}]" for b in boxes)
        print(
            f"{prefix} chosen {chosen[0]!r} [{chosen[1]:.0f},{chosen[2]:.0f},{chosen[3]:.0f},"
            f"{chosen[4]:.0f}] err={err:.1f}px {'PASS' if ok else 'FAIL'} (all: {all_boxes})"
        )
    print(f"passed {passed}/{len(labels)}")


if __name__ == "__main__":
    main()
