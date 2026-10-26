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

"""Two G1s' arm skills and gestures (left arm; both arms and Dex3 hands for imported gestures), over independently launched SONIC controllers.

Runbook. Each G1's PC2 runs its guarded SONIC launcher, which binds telemetry on 127.0.0.1:5557
and reads references from 127.0.0.1:5556. Use each PC2's ssh alias (today: 222 `pc2_222`, 223
`pc2`). The camera publisher on 222 (/home/unitree/gumin/d435_color_publisher.py) binds
0.0.0.0:5556: it must not run during a dual session (check: `ssh pc2_222 pgrep -af d435_color_publisher`).

1. Start each controller and confirm planner idle, BEFORE the tunnels (the idle helper binds
   127.0.0.1:5556; with a tunnel already holding it, it fails "Address already in use"):

       223: cd ~/dimos-sonic-guard-pc2-223-20261014 && bash launch-guarded.sh --run
            (checks the Jetson clocks itself)
       222: sudo jetson_clocks
            cd ~/dimos-sonic-guard-20261014 && bash launch-guarded.sh --run
            (no clock check, hence jetson_clocks; not executable, hence bash; Dex3 hands initialise)
       both, then: /home/unitree/dimos/.venv/bin/python start-idle-planner.py --start-idle-planner
            and expect "Planner confirmed".

2. Open the tunnels, one per robot (-R carries the PC2's reference port to this process, -L its
   telemetry here). ExitOnForwardFailure makes ssh exit if the PC2's 5556 is still held instead
   of silently running without the reference forward. ServerAlive* drops a dead link within
   ~10 s, so the robot's reference lease expires and DimOS sees stale feedback:

       ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=2 -R 5556:127.0.0.1:15556 -L 15557:127.0.0.1:5557 pc2_222
       ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=2 -R 5556:127.0.0.1:16556 -L 16557:127.0.0.1:5557 pc2

3. Run, with the SONIC v1.1 encoder, the gear_sonic MuJoCo model for the path check (the
   imported gestures and move_left_arm_joints need it) and 222's Dex3 hands declared:

       dimos run unitree-g1-sonic-arm-dual \\
           --dualsonicarmskillcontainer.max-dq=2.0 \\
           --dualsonicarmskillcontainer.hands-mounted-222=true \\
           --dualsonicarmskillcontainer.encoder-path=<deploy>/policy/sonic_v1_1/model_encoder.onnx \\
           --dualsonicarmskillcontainer.collision-model-path=<gear_sonic>/data/robot_model/model_data/g1/scene_43dof.xml
       dimos mcp call move_upper_body --json-args '{"robot": "both", "motion": "left_arm_forward", "angle_deg": 20}'
       dimos mcp call perform_gesture --json-args '{"robot": "222", "name": "wave"}'

   max-dq 2.0 rad/s is the validated value (sim acceptance); the config default is 3.0.
   (`--arg robot=223` also works: the container takes the integers 222/223 as the strings.)

4. Readiness check (`dimos shell` needs a TTY; this does not):

       PYTHONPATH=<this checkout> <its venv>/bin/python -c "from dimos.porcelain.dimos import Dimos; app = Dimos.connect(); print(app.g1_222.get_status()); print(app.g1_223.get_status()); app.stop()"

   Look at `ready`, `subscriptions_ready`, `feedback_fresh`, `feedback_age_s`. A skill saying
   "SONIC is not in confirmed planner idle" while the planner is idle usually means a tunnel
   forward is missing. (`app.stop()` only closes this client.)

Dex3 hands are per robot: hands_mounted_222 and hands_mounted_223, both false by default (the
single-robot hands_mounted is refused here). 222 has Dex3 hands; 223 runs with them disabled (its
hand columns are still sent and ignored). Only a robot with hands_mounted_<robot> accepts the
operator finger check and reports a hand-only motion's hand pose; every robot plans and streams
its imported gestures' hand rows from its own measured hands. Both execution profiles limit the
hand step per transmitted row (SONIC_ARM_PROFILE.hand_step_limit).

Operator finger check (an RPC, not an agent skill), with a running blueprint and
`app = Dimos.connect()` as in the readiness check (step 4):

       app.DualSonicArmSkillContainer.check_dex3_finger_order("222")   # "started" at once
       app.DualSonicArmSkillContainer.operator_result("222")           # None while it runs

   It holds the body and moves 222's fingers to FINGER_CHECK_POSE ("LEFT index open, middle
   closed, thumb half; RIGHT middle open, index closed, thumb rotated") for 3 s, then back;
   stop_arm stops it. On failure the "Hand result" log line and the result's hand_base,
   hand_target and hand_held tell the cases apart.

Voice lines (perform_gesture's `line`) play <voice-dir>/<line>.wav (16 kHz mono) on a robot's
speaker over ssh, through its g1_audio.py. A robot speaks only with its own voice host set (the
single-robot voice-host is refused here); voice-command, voice-dir and voice-volume are shared, so
the command's python and script paths must exist on every robot with a voice host. Both robots
have the wavs (01-04, 06-09) and g1_audio.py; the demo gives only 223 a voice, so add to step
3's `dimos run` (and voice-host-222=pc2_222 for 222 to speak too):

       --dualsonicarmskillcontainer.voice-host-223=pc2 \\
       --dualsonicarmskillcontainer.voice-command="~/miniconda3/envs/unitree/bin/python ~/g1_audio.py"
       (optional, defaults shown: --dualsonicarmskillcontainer.voice-dir=~/lines,
        --dualsonicarmskillcontainer.voice-volume=60, 0-100,
        --dualsonicarmskillcontainer.voice-lead-s=1.4, 0-3: launch-to-heard delay, measured on 223)
       dimos mcp call perform_gesture --json-args '{"robot": "both", "name": "wave", "line": "01_intro"}'

   With "both", only the robots with a voice host speak (here 223); a line is refused, with
   nothing sent, when no selected robot has one. Each robot launches its line voice-lead-s before
   its own arm starts moving (timed from its own plan), never once a stop or abort is seen; it is
   cut when its motion ends, when it retreats (stop_arm, a safety stop, the other robot's stop)
   and on shutdown: stop_arm cuts the line. A failed line is reported in that
   robot's result metadata ("voice") and never fails the motion.

Hardware order for the imported gestures (after the two-stand-in run):
   1. 223: perform_gesture "support_person", then "two_hand_hearts" (arms only: 223 has no hands).
   2. 222 at planner idle, before any hand motion: read left_hand_q/right_hand_q
      (app.g1_222.get_feedback()) and compare slot for slot with imported_gestures.HAND_DEFAULT.
      Zeros mean hand feedback is not wired; a permutation means the feedback order differs from
      the command order. Either one stops the run until fixed.
   3. 222: check_dex3_finger_order("222"), confirmed by eye against FINGER_CHECK_POSE.
   4. 222: the remaining hand motions (wave_one_hand, v_sign, good, two_hand_hearts), then "both".

To restart one robot's controller: `dimos stop`, stop that robot's tunnel, then repeat the bring-up
order (planner idle on the PC2, tunnel, `dimos run`).

Shutdown order: stop_arm (if a motion runs; wait for its tool update) -> `dimos stop` -> tunnels
-> controllers.

A `both` command plans (encodes) each robot in turn before either moves: expect about 2 s for a
pose and about 10-16 s for a wave before motion starts, and a wave of about 40-45 s from "started"
to the result (the skill durations are motion only).

WARNING: a dropped tunnel still snaps a raised arm: the controller loses its reference and the arm
returns to the planner pose at once. Keep both robots on stands.

The motion skills return at once; the result arrives as a tool update (also logged: `dimos log -f`).
Only one motion runs at a time across both robots; stop_arm stops it.

Why stop_arm first: shutdown also retreats a running motion, but only best-effort: the execution
modules may stop at the same time and hand the raised arms to the native planner.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm import SONIC_ARM_PROFILE
from dimos.robot.unitree.g1.sonic_dual_arm_skill import DualSonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule

_PROFILE = SONIC_ARM_PROFILE  # The single-robot blueprint's, so the two never drift apart.


unitree_g1_sonic_arm_dual = autoconnect(
    SonicExecutionModule.blueprint(
        instance_name="g1_222",
        reference_bind="tcp://127.0.0.1:15556",
        feedback_endpoint="tcp://127.0.0.1:15557",
        journal_dir=STATE_DIR / "sonic-arm" / "222",
        profile=_PROFILE,
    ),
    SonicExecutionModule.blueprint(
        instance_name="g1_223",
        reference_bind="tcp://127.0.0.1:16556",
        feedback_endpoint="tcp://127.0.0.1:16557",
        journal_dir=STATE_DIR / "sonic-arm" / "223",
        profile=_PROFILE,
    ),
    DualSonicArmSkillContainer.blueprint(),
    McpServer.blueprint(),
).remappings(
    [
        ("g1_222", "feedback", "feedback_222"),
        ("g1_223", "feedback", "feedback_223"),
        ("g1_222", "execution_status", "execution_status_222"),
        ("g1_223", "execution_status", "execution_status_223"),
        (DualSonicArmSkillContainer, "_execution_222", "g1_222"),
        (DualSonicArmSkillContainer, "_execution_223", "g1_223"),
    ]
)
