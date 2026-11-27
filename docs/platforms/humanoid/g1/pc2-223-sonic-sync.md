# pc2 (.223): DimOS and guarded SONIC sync

**Waist-grinding incident resolved (2026-10-14).** The first launch ground because
the launcher skipped `scripts/setup_env.sh`, so `jetson_clocks` never ran. Policy
inference took about 5 ms instead of 1.8 ms, and control latency doubled. The
launcher now refuses to start unless the Jetson clocks are pinned. The same guarded
binary then ran smoothly (operator retest at 17:27). See the
[handover](/docs/platforms/humanoid/g1/handover-pc2-223-grinding-2026-10-14.md) and the
[analysis](/docs/artifacts/g1_pc2_223_incident_20261014/offline-binary-comparison.md).

Verified on 2026-10-14. Hardware confirmed by the operator: **29 body joints,
no hands attached; Inspire hands planned**. This is a separate configuration
from pc2_222, which has two Dex3 hands.

## Installed copies

| Component | Location on pc2 |
|---|---|
| Guarded native controller and operator helpers | `/home/unitree/dimos-sonic-guard-pc2-223-20261014` |
| DimOS with this session's SONIC modules | `/home/unitree/dimos-g1-service-20261014` |
| Reused Python environment | `/home/unitree/dimos/.venv/bin/python` |
| Original DimOS, preserved | `/home/unitree/dimos` |
| Original SONIC source/binary, preserved | `/home/unitree/GR00T-WholeBodyControl` |

The DimOS copy starts from pc2's installed commit
`7dd83be804e0e9dd2dd6210268250bcabf06617c`. It overlays 63 implementation,
test, fixture and guide files from this workstation and regenerates its own
blueprint registry. Run `./run-python ...` from the new copy to select its
source while using the installed Python dependencies. This does not replace
the installed `dimos` command or automatically start a blueprint.

The native port starts from pc2's commit
`a9d20b2ac0949244d94461a1a3263f38c5027c4a` plus its existing local changes.
It preserves the torque and hand receipt-age telemetry. It adds the existing
reference-expiry guard and an explicit `--disable-dex3-hands` option, which
skips hand DDS initialization and writes. Missing hand feedback remains invalid.

The new launcher selects `zmq_manager`, a 200 ms reference timeout and no-hands
mode. With `--run`, it refuses to start unless every CPU's minimum clock equals
its hardware maximum and the GPU's minimum clock equals its maximum, which is the
state `sudo jetson_clocks` produces. It also writes native CSV logs to
`<staged root>/logs/<timestamp>/` for each run. References are received from **127.0.0.1:5556 on pc2**; telemetry is on
5557. This differs from the earlier workstation-hosted `192.168.0.62` route.
Use the local operator helper below with this launch configuration.

Native binary SHA-256:
`9c6ee5fbde7fc62121a35d62450919dc87125e727ec243afd179415d4b73cdfa`.
Launcher SHA-256 (clock preflight version):
`456bb1fd5b840310ea7b1a95a6a9b6247a07dca15a3be57ebb72395226c259b8`. The incident
version is kept beside it as `launch-guarded.incident-v1.sh`.

111 model/cache/reference files (1,935,954,770 bytes) were copied and hash
verified from pc2 itself. Its TensorRT caches were retained; no caches were
copied from pc2_222. The staged native build reuses the original SDK libraries
through a `thirdparty` symlink. The DimOS clone also shares Git objects with
its original repository, so retain both original directories.

## Validation completed

- Native ARM build and dependency resolution passed on pc2 (JetPack 6.2.1,
  TensorRT 10.3, CUDA 12.6, Ubuntu 22.04).
- **18 native tests passed**: reference lease/validation and eight actual
  MessagePack wire cases covering guard, heading and hand availability.
- The unrelated existing FK test was excluded because
  `reference/bones_072925_test/` is absent.
- **371 DimOS tests passed**: 366 SONIC tests plus five registry tests, with
  synthetic peers and local transports. Imports resolved to the staged copy.
- **Eight offline operator-helper checks passed**, including rejection of a
  missing or repeated no-hands option. Launcher/helper previews and camera
  `--help` also passed.
- Original native source/binary hashes and the clean original DimOS checkout
  were preserved.

On 2026-10-14 at 17:27, with clocks pinned, the operator ran a physical
IDLE-planner startup on the supported robot, and it was smooth. Policy took
1.71 ms and LowState age was 3.1–4.0 ms. The waist trace was as smooth as in an
original-binary run.

The software checks above establish build and software compatibility. They do
not establish balance under motion, reference-expiry behavior on this physical robot, or
camera frame quality. No real controller, camera server, service, or command
publisher was started by the sync.

The guard handles interrupted motion-reference streams by requesting stationary
planner control. It is not an emergency stop and does not establish a bound on
the age of measurements produced by Unitree firmware.

## Manual operator sequence

Pin the Jetson clocks once after every pc2 boot. The setting does not survive a
reboot. Then check them and preview, without starting robot control:

```bash skip
ssh pc2
cd /home/unitree/dimos-sonic-guard-pc2-223-20261014
sudo jetson_clocks                  # after each boot; same as scripts/setup_env.sh
bash launch-guarded.sh --check-clocks   # must print "Clocks pinned."
bash launch-guarded.sh              # dry run: prints the command, starts nothing
```

When the robot is supported, the area is clear, and the operator is at the
emergency stop, the operator can start the prepared controller:

```bash skip
bash launch-guarded.sh --run
```

This starts real body motor control and its initialization movement. The
launcher refuses to start if the clocks are not pinned. It checks its binary
hash, refuses to launch over an existing SONIC process, and prints the CSV log
folder for the run. The operator remains responsible for the normal shutdown procedure.

If the native terminal reaches `Init Done` and waits for a start command, use
a second SSH terminal. Preview first:

```bash skip
cd /home/unitree/dimos-sonic-guard-pc2-223-20261014
/home/unitree/dimos/.venv/bin/python start-idle-planner.py
```

The following operator action starts native planner control and can move the
robot:

```bash skip
/home/unitree/dimos/.venv/bin/python start-idle-planner.py --start-idle-planner
```

The helper verifies the exact running binary and required no-hands options,
owns the local reference port exclusively, waits for the command subscription,
sends one start/planner command, then waits for three advancing
planner-confirmed telemetry samples. It sends no motion-reference trajectory.
Confirmation is not a settling or command-transaction guarantee. On failure it
does not retry the start or automatically stop the robot; inspect the native
terminal and use the operator shutdown procedure if necessary.

### Zero-motion guard test (milestone 3, step 1)

`hold-reference-test.py` (current SHA-256 `74be0bfb…385bfa1`; the first
version `c4074163…6b90a` is kept on pc2 as `hold-reference-test.v1.py`) is
installed in the guarded helper folder, and its preview runs on pc2. It imports the session from the synced DimOS copy, so set
`PYTHONPATH` as shown below. Run it only from confirmed planner IDLE.

1. **Seed.** It drives the existing DimOS `SonicTaskSession`: a 1 s seed
   captures the planner's current token.
2. **Arm.** It arms the reference stream.
3. **Hold.** It streams that same token for `--hold-seconds` (default 2 s). It
   submits no policy chunk.
4. **End.** It finishes in one of two ways:
   - `--mode expire` (the default) stops sending. The native 200 ms lease must
     expire and select planner IDLE.
   - `--mode handback` sends an explicit planner request.
5. **Watch the return.** It keeps observing for 3 s after the return to the
   planner. PASS also requires that no body joint exceed `--max-return-dq`
   (default 0.5 rad/s) from the end of the stream until then. The output
   splits `stream_peak_dq` from `return_peak_dq` and reports
   `guard_passed` / `return_motion_ok` separately.

Safety:
- If any body joint moves more than `--max-deviation` (default 0.10 rad) from
  the start posture, it requests the planner at once.
- Ctrl-C and errors also send a planner request.
- It never sends start or stop.
- Holding the planner token is intended to keep the robot still, but this has
  not been proven. That is what the test measures. Supporting evidence: the
  retest's `token_state.csv` is exactly constant after 2 s of IDLE (note §11).
  Start the test only after IDLE has settled.

Preview first, then run the operator action, which can move the robot:

```bash skip
cd /home/unitree/dimos-sonic-guard-pc2-223-20261014
export PYTHONPATH=/home/unitree/dimos-g1-service-20261014
/home/unitree/dimos/.venv/bin/python hold-reference-test.py
/home/unitree/dimos/.venv/bin/python hold-reference-test.py --run
```

It prints:
- PASS or FAIL;
- the phase timeline;
- the expiry latency;
- the maximum body deviation and velocity;
- the final guard state;
- the path of the session journal, which is written under `logs/`.

After an expiry, `rearm_requires_planner` stays true until the next session
sends a planner request; each session's seed does this. Offline check:
`demo_check_hold_reference.py` (fake guarded controller, in-process ZMQ)
passes 9/9. Mutations that remove the silent stop or the return-motion verdict
fail it.

**First physical runs (2026-10-15 08:54–08:56, operator):** there were three
runs, two in expire mode and one in handback. All three had correct guard
timing: expiry after 0.198 s, and the handback confirmed. The robot did not
move while streaming; the token equalled the captured value exactly. The
operator saw a brief fidget, and it came from the planner **return**: the
planner restarts and replays about 1.6 s of motion. Arms reached up to
1.0 rad/s, the waist 0.19 rad/s and the legs at most 0.21 rad/s. There was no
stepping, and the posture after was within 0.022 rad of the posture before.
No velocity is commanded on this path. v1 stopped measuring at planner
confirmation and reported PASS. The current version would have reported
FAIL (`return_motion_ok: false`). See note §12.

## Left-arm skill (`unitree-g1-sonic-arm`)

The commissioned left-arm move is now a DimOS skill: `move_left_arm_forward(angle_deg)`, 1–28°.
It is installed in the DimOS copy on pc2 (2026-10-15). It ran on the robot through the agent variant at 16:59 (note §16).

**End-to-end check on the workstation.** The real blueprint ran under `dimos run`, isolated from
another local session by a private `ZENOH_SCOUT_ADDR`, against a fake controller on spare ports. It
used the real v1.1 encoder. `dimos mcp call move_left_arm_forward --arg angle_deg=17` returned
success in 16 s. The controller-side journal shows the full session, and the fake controller saw
only planner-select commands, never start or stop. The MCP client timeout (`mcp_timeout`, 30 s)
covers the skill's roughly 18 s on the robot.

Files (`dimos/robot/unitree/g1/`, SHA prefixes):
- `sonic_arm_reference.py` (`b2817523`): the encoder and plan, ported from `arm_reference.py`. On pc2 it
  produces tokens bit-identical to the script that ran on the robot.
- `sonic_arm_skill.py` (`166cb813`): `SonicArmSkillContainer`.
- `test_sonic_arm_skill.py` (`48f94520`): 14 tests against the real `SonicExecutionModule` and its
  scripted controller. They pass locally and on pc2.
- `blueprints/basic/unitree_g1_sonic_arm.py` (`72e81aa4`): `SonicExecutionModule`, the skill and
  `McpServer`. The registry was regenerated locally and on pc2; pc2's previous registry is kept as
  `all_blueprints.before-arm-skill.py`.

What the skill does, the same as `arm-forward-test.py` at 13:20:
1. Requires the execution module to be ready, meaning confirmed planner idle with fresh feedback.
   It never starts or stops native control.
2. Encodes from the measured pose, then re-checks that the body hasn't moved more than 0.05 rad.
3. Streams the 11 s plan as **one open-loop chunk**, with 2 s token blends at both ends.
4. Watches every telemetry sample against the per-group envelopes (legs and waist 0.15, arms 0.20,
   wrists 0.30 rad, plus |delta| on the moving shoulder) and 3 rad/s. Any breach cancels to the
   planner.
5. Finishes with a confirmed planner handback, then watches the return for 3 s.

It reports success only if the session's own outcome is `finished`, the handback is confirmed, and the
shoulder reached at least half the planned swing. The result metadata carries the reach, the worst
deviation per group, peak speeds and the journal path. It holds the `movement` capability, so MCP
won't run it alongside another motion skill.

Design notes:
- **Resubmitting like a policy source was rejected.** Every chunk restarts the session blend. With
  the default 0.3 s blend, that would ripple the reference speed at 2.5 Hz (peak 0.93 against 0.47)
  and leave it up to 0.04 behind the commissioned stream. The blueprint's execution profile therefore
  sets `max_observation_age` to 12 s, as the robot script did.
- **Bug caught before installing.** A mutation test showed the skill would report success after the
  session had failed on its own. It now reads the session's outcome, and two tests cover this.

Operator sequence. Run the controller and `start-idle-planner.py` as for the arm test, and don't run
`arm-forward-test.py` at the same time: both bind 5556, and the second refuses. Then:

```bash skip
cd /home/unitree/dimos-g1-service-20261014
./run-python -m dimos.cli.dimos --viewer none run unitree-g1-sonic-arm \
    --encoder-path=/home/unitree/dimos-sonic-guard-pc2-223-20261014/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx
# second terminal:
cd /home/unitree/dimos-g1-service-20261014
./run-python -m dimos.cli.dimos mcp call move_left_arm_forward --arg angle_deg=17
```

17° is the 0.3 rad move that passed at 11:56. Session journals go to `~/.local/state/dimos/sonic-arm/`.
Stop the blueprint with Ctrl-C (or `dimos stop`); that requests the planner, and native control keeps
running.

**Agent variant: `unitree-g1-sonic-arm-agentic`** (installed on pc2 2026-10-15). **Run on the robot
2026-10-15 16:59, PASS:** "swing your left arm forward 17 degrees" made one tool call, the shoulder
reached 15.9°, the handback was confirmed, and the worst use of any group limit was 42% (wrists) (note §16). It is the same blueprint plus `McpClient`.
- **Prompt.** It uses `G1_SONIC_ARM_SYSTEM_PROMPT` (`system_prompt.py`), not the general G1 prompt,
  which advertises walking, gestures and navigation this blueprint doesn't have. The prompt names only
  `move_left_arm_forward`, keeps requests to 1–28°, and allows one call at a time. It says not to retry
  after NOT_READY, and to stop calling skills after HANDBACK_UNCONFIRMED. A test checks the prompt names
  only skills the container has.
- **Offline end-to-end check.** On the workstation, with a scripted `MockModel`, the fake controller
  and a private Zenoh group:
  `dimos agent-send "swing your left arm forward 17 degrees"` → tool call → skill OK in 15.9 s →
  result back to the agent → final reply. The session finished and the handback was confirmed; there
  were planner-select commands only.
- **Genon gateway (checked 2026-10-15).** `https://api.genon.ai/v1` is OpenAI-compatible but offers
  **Chat Completions only**; `/v1/responses` returns 404.
  - DimOS sends bare `gpt-5*` names to the Responses API, so name the model with LangChain's provider
    prefix: `--model openai:openai/gpt-5.6-luna`.
  - The key in `.env.genon` (`GENON_API_KEY`) is allowed `gpt-5.6-luna`, `-sol` and `-terra` and
    `gpt-5.5`, not `gpt-6-luna`.
  - **Live end-to-end on the workstation** with `gpt-5.6-luna` and the fake controller:
    - "What can you do?" described only the arm skill.
    - "40 degrees" was declined, with 28° offered, and no call was made.
    - "17 degrees" made one call, the skill returned OK in 15.8 s, and the agent reported 17.2°.
  - Read only `GENON_API_KEY` from `.env.genon`; the file's header says never to source all of it.
- **API key on pc2.** pc2 can reach the OpenAI API, but no key is configured there. Plain-OpenAI
  form:

```bash skip
cd /home/unitree/dimos-g1-service-20261014
OPENAI_API_KEY=... ./run-python -m dimos.cli.dimos --viewer none run unitree-g1-sonic-arm-agentic \
    --encoder-path=/home/unitree/dimos-sonic-guard-pc2-223-20261014/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx
# second terminal:
cd /home/unitree/dimos-g1-service-20261014
./run-python -m dimos.cli.dimos agent-send "swing your left arm forward 17 degrees"
```

**With Genon on pc2 (set up 2026-10-15):**
- **Key.** Only the `GENON_API_KEY` line was copied to `/home/unitree/dimos-g1-service-20261014/.env.genon`
  (mode 600, owner `unitree`). The value is single-quoted in the file, so strip the quotes before
  using it.
- **Check.** From pc2, `init_model("openai:openai/gpt-5.6-luna")` returned a correct
  `move_left_arm_forward` tool call through the gateway.
- **Launcher.** `run-arm-agent.sh` (`ab0f45ad…`; source in `docs/artifacts/g1_pc2_223_sync_20261014/`)
  reads only that key, sets the gateway URL and the `openai:` model, and starts the agentic blueprint
  with the staged encoder. It never starts or stops SONIC.

```bash skip
cd /home/unitree/dimos-g1-service-20261014
./run-arm-agent.sh
# second terminal:
./run-python -m dimos.cli.dimos agent-send "swing your left arm forward 17 degrees"
```

pc2's previous `system_prompt.py` is kept as `system_prompt.before-arm-agent.py`.

## Head camera in the SONIC blueprints (2026-10-15)

`unitree-g1-sonic-arm`, and therefore `-agentic`, now include `CameraModule` with the `Webcam` (V4L2)
backend and `ObserveSkill`. The blueprint file's SHA prefix is `8f6141cf`.

- **Device.** `/dev/v4l/by-path/platform-3610000.usb-usb-0:3:1.3-video-index0`, which is USB interface 3
  (color) and currently `/dev/video4`. The `by-id` names are ambiguous between the depth and color
  interfaces.
- **Image.** 640×480 at 15 fps, RGB. `CameraInfo` carries the factory intrinsics, and the mount is the
  URDF `d435_joint` on `torso_link` (pitched 0.831 rad down). There is no depth; the native RealSense
  module isn't built on pc2, and there is no nix there.
- **Checked on pc2.** `CameraModule` → `ObserveSkill` gave 640×480 RGB frames with correct colors; three
  `observe()` calls took 0.16 s. The frame is `camera-check/dimos-observe.png`.
- **Agent on pc2** (SONIC not running, private Zenoh group). "What do you see in front of you right
  now?" → `observe()` (41 ms) → `gpt-5.6-luna` through Genon described the speckled floor and the
  wheeled cart in view. The Genon gateway accepts image input.
- **Prompt.** `G1_SONIC_ARM_SYSTEM_PROMPT` lists `observe()`. pc2's previous files are kept as
  `*.before-camera.py`.

## Upper-body motions (2026-10-15)

The skill is now `move_upper_body(motion, angle_deg)`, 1–28°, replacing `move_left_arm_forward`:

| Motion | Joint | Status |
|---|---|---|
| `left_arm_forward` | shoulder pitch, negative | robot-validated (−0.476 rad at −0.5 rad) |
| `left_arm_sideways` | shoulder roll, positive (outward only) | offline only: MuJoCo reached 95% at 28°, side motion 0.195 (wrist) |
| `left_forearm_raise` | elbow, negative | offline only: MuJoCo reached 88% at 28°, side motion 0.115 |

Waist turns were dropped because SONIC tracked only 18–48% in MuJoCo. Evidence is in note §17. pc2's
previous skill is kept as `sonic_arm_skill.before-motions.py`.

**Robot runs 2026-10-19 09:24** (note §18). All six were clean, with the handback confirmed:

| Motion | 10° | 20° | 28° |
|---|---|---|---|
| Sideways | 7.7° | 18.8° | 28.2° |
| Forearm raise | 4.8° (`NOT_REACHED`) | 12.9° | 26.0° |

Small moves fall short because of joint friction against the soft arm gains.

**Hardware warning from that session.** The left ankle carried 39% of its torque limit at idle, and
its drivers reached 80 °C, still rising about 10 °C/min. Check the support and foot contact before
the next session.

**Ranges since 2026-10-19** (note §19): forward 10–28°, sideways 10–40°, forearm raise 20–60°. Strokes
slow down to keep peak speed at or below 0.47 rad/s. The pc2 copies from before this change are kept as
`*.before-ranges.py`.

**Step-up 2026-10-19 09:57** (note §20). Sideways 34° → 34.7° and 40° → 37.5°; forearm raise 40° →
34.8° and 60° → 58.6°. All were clean with the handback confirmed, committed as `15402655b`. SONIC's
own shoulder-yaw coordination used 88–99% of the 0.20 rad arm envelope at sideways 40° and forearm
40/60°. **Left ankle roll driver reached 91 °C in 133 s, still rising; fix the support and stance
before more runs.**

Operator validation, stepping up:

```bash skip
# with the controller in planner idle and ./run-arm-agent.sh running:
./run-python -m dimos.cli.dimos mcp call move_upper_body --arg motion=left_arm_sideways --arg angle_deg=34
./run-python -m dimos.cli.dimos mcp call move_upper_body --arg motion=left_arm_sideways --arg angle_deg=40
./run-python -m dimos.cli.dimos mcp call move_upper_body --arg motion=left_forearm_raise --arg angle_deg=40
./run-python -m dimos.cli.dimos mcp call move_upper_body --arg motion=left_forearm_raise --arg angle_deg=50
./run-python -m dimos.cli.dimos mcp call move_upper_body --arg motion=left_forearm_raise --arg angle_deg=60
# stop and review if any max_deviation_rad passes about 2/3 of its group limit
```

## Full-range left-arm poses (2026-10-19, installed on pc2, not yet run on the robot)

New skills (note §23):
- `move_left_arm_pose(pose)`: `left_arm_forward_horizontal`, `left_arm_side_horizontal`,
  `left_hand_over_head`, `left_hand_over_head_sideways`.
- `move_left_arm_joints(shoulder_pitch_deg, shoulder_roll_deg, shoulder_yaw_deg, elbow_deg)`: absolute
  angles. The planned path is checked against the body with the MuJoCo model, and unsafe paths are
  refused before anything is sent.

Both move at 0.47 rad/s and take 25–41 s per call, so pass `--timeout 60` to `dimos mcp call`. Their
envelope watches balance: legs and waist 0.15 rad. The moving arm may follow the plan within 0.45 rad,
because SONIC coordinates the whole arm, and the other arm gets 0.35 rad. Raise the speed only after
the robot runs verify these.

Run the blueprint with the collision model so `move_left_arm_joints` works (`run-arm-agent.sh` already
passes it):

```bash skip
./run-python -m dimos.cli.dimos --viewer none run unitree-g1-sonic-arm \
    --encoder-path=/home/unitree/dimos-sonic-guard-pc2-223-20261014/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx \
    --collision-model-path=/home/unitree/GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml
```

## Overhead retest in MuJoCo and next steps (2026-10-21)

**Overhead retest** (note §25), run against a real-time MuJoCo stand-in for the guarded controller
(`sonic_sim_peer.py`).
- Overhead via the side completed.
- Forward overhead stopped on the leg envelope; the sim's legs read about 0.07 rad high.
- **Fixed:** a safety stop used to hand the raised arm straight to the native planner, which snapped
  it home at up to 23 rad/s and lifted the feet in MuJoCo. The skill now retreats smoothly along the
  plan's way back before handing back.
- **Not fixed (native):** the same snap would follow a lost stream, because the guard expires to the
  planner.
- **Before any robot overhead run:** install `pc2-backport-retreat/` on pc2. pc2 runs pre-#4404 code.

**Work split.**
- Synchronizing two G1s with one skill call moves to another session.
- This session's next step is implementing more skills that operate the robot.

## Camera and Inspire follow-up

The standalone `g1_head_camera_server.py` was copied unchanged from
`~/work/lerobot/examples/g1_dex3_training/`. It publishes RGB JPEG images using
the existing policy-streamer convention and opens no robot DDS control channel.
The head camera is a RealSense D435I (serial 254322071593, firmware 5.15.1.55).
**Verified 2026-10-15:** `/dev/video4` (USB interface 3) is the color stream; `video0`–`video3` are the
depth/IR interface.
- **Snapshots.** `g1_head_camera_server.py --snapshot` gave real-color frames at 640×480 and 1280×720:
  the floor in front of the robot, with the support frame at the left edge in the wide view. The images
  are in `docs/artifacts/g1_pc2_223_sync_20261014/camera-check/`.
- **Field of view.** 640×480 is a narrower 4:3 view, not a scaled-down 1280×720.
- **Factory intrinsics at 640×480** (read with `pyrealsense2` in system Python, no stream started):
  fx 606.62, fy 607.12, cx 325.54, cy 245.35, zero distortion. The lerobot
  `cameras/d435i_head.json` uses the nominal 615/615/320/240 and should take these values.
- **Not run:** the server's streaming mode. The helper provides no depth.
- **Preprocessing.** The lerobot policies take 3×480×640 RGB, so capture natively at 640×480 with no
  `--out-size`. The server JPEG-encodes RGB, and the streamer decodes it without color conversion.

`device-profile.json` records commissioning facts; generic DimOS modules do not
load it automatically. No grasp blueprint was activated. The existing grasp
observation/action contract expects 14 arm values and 14 Dex3 hand values and
requires real hand deliveries. Do not substitute zeros for absent hands.

Next milestones:

1. Done 2026-10-14: the guarded controller with Dex3 disabled started into
   IDLE planner smoothly once the clocks were pinned. The CSV is in
   `docs/artifacts/g1_pc2_223_incident_20261014/retest-20261014-172735/`.
   Body telemetry reviewed (analysis note §9): no motor errors, no missed ticks,
   no steady-state vibration, and commands executed exactly. Watch item:
   hip-roll and left ankle-roll driver temperatures rose to 46–48 °C in 46 s
   during supported idle. This was later closed: the original run reached the
   same hip-roll peaks and ankle drivers up to 90 °C (note §11). The inherited startup-ordering hazard (example
   reference for the first 2–3 ticks) was measured and matches the original
   build; no change is needed (note §10).
2. Done 2026-10-15 (camera part): RGB frames confirmed on `/dev/video4`, with factory intrinsics
   recorded in `device-profile.json`. The camera is wired into the SONIC blueprints, and the agent's
   `observe()` works on pc2 (see "Head camera in the SONIC blueprints"). Preprocessing is native 640×480 RGB (see above). Choosing
   the policy waits on hands (item 5): every existing grasp policy needs hand state and actions.
3. Done 2026-10-15: a small body/arm-only reference and return, including the guard's
   loss-of-stream behavior under operator supervision. The zero-motion hold runs expired and
   handed back (note §12), and the left arm moved to −0.5 rad and back (3a, note §15).
3a. Done 2026-10-15 11:56, operator-run, PASS (note §14): `arm-forward-test.py` (left
   shoulder pitch −0.3 rad forward and back) with `arm_reference.py`.
   - Reached −0.296 rad and returned. Other joints stayed within 0.112 rad of the
     0.15 margin. Peak speed was 0.70 rad/s and tilt 1.0°. There were no motor errors,
     and the handback was confirmed.
   - A pre-existing 1–2 tick token blip at arming, also present in the hold runs, is
     the controller re-zeroing a 0.5–2° heading mismatch on the switch tick (note §14).
     It is accepted and not fixed.
   - Starting the reference from the planner's own frame shrinks the handoff, but in
     simulation it doubles the side motion (0.20 rad). It was tried and rejected; the
     v1 script stays installed (note §14).
   - At −0.5 rad (13:06), it cancelled on the right wrist roll at +0.155 rad. That motion was
     decoder-commanded at low torque (note §15). The envelopes are now per group: legs and
     waist 0.15, arms 0.20, wrists 0.30 rad (`c4b50201…`), installed on pc2 2026-10-15.
     Retest at −0.5 rad (13:20): PASS. The shoulder reached −0.476 rad; worst use of the envelopes
     was legs 36%, arms 41%, wrists 34% (note §15).
   - On pc2: hashes match, the venv has `onnxruntime` 1.24.1, the staged
     encoder is v1.1 (`fb97de22…`), and encoding takes 1.44 s on one thread.
   - `demo_check_arm_forward.py` (6/6) and `demo_check_hold_reference.py` pass
     there. The previous hold check is kept as `.v2`.
   - MuJoCo, on the floor: reached −0.269 rad, other joints within 0.10 rad,
     tilt 2.2°, all feet in contact (note §13).
   - The operator runs it from settled planner IDLE, with the same
     `PYTHONPATH` as the hold test:
     `hold-reference-test.py` → `arm-forward-test.py` → `arm-forward-test.py --run`.
4. TODO, deferred and not a priority: smooth the planner return. Every guard
   return cold-starts the planner, which replays its 1.47 s idle-settle clip:
   up to 0.27 rad of travel and 0.77 rad/s, mostly left wrist roll. That speed
   is affordable, about 3–4% of the joint speed limits. The wrist motion may
   be the hand contacting the pelvis. Planned fix: play that clip 2.5× slower
   on guard returns only (note §12a).
5. When Inspire hands are installed, identify the exact model and transport;
   implement its adapter, feedback timing, limits and joint mapping; then adapt
   the policy observation/action contract and validate in simulation before a
   physical grasp trial.

Local evidence and the exact native delta are in
`docs/artifacts/g1_pc2_223_sync_20261014` (`../../../artifacts/g1_pc2_223_sync_20261014/`).
