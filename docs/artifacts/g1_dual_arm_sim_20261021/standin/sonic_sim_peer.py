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

"""Real-time MuJoCo stand-in for the guarded native SONIC controller (offline; no robot).

Speaks the guarded controller's local protocol to a DimOS SonicExecutionModule: binds a g1_debug
feedback PUB, subscribes to the module's command/pose references, and keeps the reference-guard
semantics the session relies on (planner idle -> planner=false arms a new epoch -> accepted/applied
pose indices -> planner=true or a 200 ms stream gap returns to the planner). Tokens drive the SONIC
v1.1 decoder and a floor-standing G1 from the lerobot sonic_roundtrip_audit harness: 50 Hz policy,
200 Hz physics. The planner's idle token is stood in for by encoding the planner model's final idle
frame (matches the robot's switch-tick token, note §14). No hands: hand slots report zeros.

Startup keeps the feet on the floor, as on the robot after it is lowered from the gantry: the model's
start pose stands on the floor, an elastic band holds the pelvis at that height through the 2 s INIT
ramp and the start of SONIC in planner idle, then fades out over 2 s. The harness's startup_hang ran
SONIC while hanging 1 m up (26 rad/s, 100% torque saturation), dropped it, reset it onto the floor
with the hanging history and switched tokens instantly; this one ends at 0.5 deg tilt and 0 rad/s.

Usage: python sonic_sim_peer.py --feedback tcp://127.0.0.1:15557 --reference tcp://127.0.0.1:15556 \
           [--name robot0] [--log sim_robot0.npz]
"""

import argparse
import signal
import time

from arm_reference import Encoder, IdlePlanner
import demo_sim_arm_reference as sim  # sets up the lerobot harness with the v1.1 model and scene
import msgpack
import mujoco
import numpy as np
import zmq

TICK = 0.02
LEASE = 0.2  # the guarded controller's reference timeout


def startup_on_floor(robot, dec, planner_token_of):
    """INIT ramp and SONIC start with the feet down, held by a band that then fades out."""
    h = sim.harness
    mujoco.mj_resetData(robot.m, robot.d)
    mujoco.mj_forward(robot.m, robot.d)
    target, scale = robot.d.xpos[robot.pelvis].copy(), [1.0]

    def band():  # the harness's elastic band, at standing height and scaled by `scale`
        pos, rot = robot.d.xpos[robot.pelvis], robot.d.xmat[robot.pelvis].reshape(3, 3)
        lin, ang = robot.d.qvel[0:3], rot @ robot.d.qvel[3:6]
        w, v = robot.d.qpos[3], robot.d.qpos[4:7]
        rotvec = v / max(np.linalg.norm(v), 1e-12) * 2 * np.arctan2(np.linalg.norm(v), w)
        return scale[0] * np.r_[10000 * (target - pos) - 1000 * lin, -1000 * rotvec - 10 * ang]

    robot.band_wrench = band
    q0 = robot.state()[0]
    for j in range(100):  # INIT: 2 s PD ramp to the default pose
        r = min(1.0, (j + 1) / 50)
        robot.step(q0 * (1 - r) + h.DEFAULT * r, band=True)
    dec.reset()
    token = planner_token_of(robot.state()[0])
    for _ in range(50):  # SONIC starts in planner idle, still held
        robot.step(dec(token, *robot.state()), band=True)
    for i in range(100):  # the band fades out over 2 s
        scale[0] = 1 - (i + 1) / 100
        robot.step(dec(token, *robot.state()), band=True)
    for _ in range(150):  # free standing
        robot.step(dec(token, *robot.state()))
    return token, {
        "floor_contacts": robot.floor_contacts(),
        "pelvis_z": float(robot.d.xpos[robot.pelvis, 2]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--feedback", required=True, help="bind address for g1_debug feedback")
    parser.add_argument(
        "--reference", required=True, help="the execution module's reference endpoint"
    )
    parser.add_argument("--name", default="sim")
    parser.add_argument("--log", default=None, help="npz of the simulated state, written on exit")
    args = parser.parse_args()

    h = sim.harness
    encoder = Encoder(sim.HF / "sonic_v1_1/model_encoder.onnx")
    robot, dec = h.Robot(), h.Decoder()
    planner = IdlePlanner(sim.HF / "planner_sonic.onnx")

    def planner_token_of(q):
        joints, anchor = planner.final(q.astype(np.float32))
        return encoder.encode(joints[None], anchor)[0]

    planner_token, info = startup_on_floor(robot, dec, planner_token_of)
    print(f"[{args.name}] standing: {info}", flush=True)

    ctx = zmq.Context()
    pub, sub = ctx.socket(zmq.PUB), ctx.socket(zmq.SUB)
    for s in (pub, sub):
        s.setsockopt(zmq.LINGER, 0)
    pub.bind(args.feedback)
    sub.setsockopt(zmq.SUBSCRIBE, b"command")
    sub.setsockopt(zmq.SUBSCRIBE, b"pose")
    sub.connect(args.reference)

    planner, expired, rearm_requires_planner = True, False, False
    epoch, accepted, index, token = 0, -1, 0, planner_token
    last_pose = 0.0
    log: dict[str, list] = {
        k: [] for k in ("t", "armed", "q", "dq", "pelvis_z", "tilt", "contacts", "tick_ms")
    }
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    next_tick = time.monotonic()
    try:
        while not stop:
            now = time.monotonic()
            while sub.poll(0):
                packet = sub.recv()
                if packet.startswith(b"command"):
                    selected = bool(packet[-1])
                    if selected:
                        rearm_requires_planner, expired = False, False
                    if planner and not selected and not rearm_requires_planner:
                        epoch, accepted, expired, last_pose = epoch + 1, -1, False, now
                        print(f"[{args.name}] armed epoch {epoch}", flush=True)
                    elif selected and not planner:
                        print(f"[{args.name}] planner (handback)", flush=True)
                    planner = selected if not rearm_requires_planner else True
                elif packet.startswith(b"pose") and not planner:
                    accepted = int(np.frombuffer(packet[1540:1548], dtype="<i8")[0])
                    token = np.frombuffer(packet[1284:1540], dtype="<f4").astype(np.float64)
                    last_pose = now
            if (
                not planner and now - last_pose > LEASE
            ):  # stream gap: the guard returns to the planner
                planner, expired, rearm_requires_planner = True, True, True
                print(f"[{args.name}] reference expired -> planner", flush=True)
            applied = planner_token if planner or accepted < 0 else token
            q, dq, quat, gyro = robot.state()
            started = time.monotonic()
            robot.step(dec(applied, q, dq, quat, gyro))
            index += 1
            pub.send(
                b"g1_debug"
                + msgpack.packb(
                    {
                        "index": index,
                        "ros_timestamp": 0.0,
                        "body_q": robot.state()[0].tolist(),
                        "body_dq": robot.state()[1].tolist(),
                        "left_hand_q": [0.0] * 7,
                        "left_hand_dq": [0.0] * 7,
                        "right_hand_q": [0.0] * 7,
                        "right_hand_dq": [0.0] * 7,
                        "token_state": [float(v) for v in applied],
                        "reference_guard": {
                            "armed": not planner,
                            "expired": expired,
                            "planner_selected": planner,
                            "planner_confirmed": planner,
                            "rearm_requires_planner": rearm_requires_planner,
                            "accepted_index": accepted,
                            "applied_index": accepted,
                            "rejected": 0,
                            "epoch": epoch,
                            "age_ms": (now - last_pose) * 1000 if not planner else -1.0,
                        },
                    }
                )
            )
            log["t"].append(now)
            log["armed"].append(not planner)
            log["q"].append(q)
            log["dq"].append(dq)
            log["pelvis_z"].append(float(robot.d.xpos[robot.pelvis, 2]))
            log["tilt"].append(float(np.degrees(np.arccos(np.clip(-h.gravity(quat)[2], -1, 1)))))
            log["contacts"].append(robot.floor_contacts())
            log["tick_ms"].append((time.monotonic() - started) * 1000)
            next_tick += TICK
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()  # fell behind real time; the log's tick_ms shows it
    except KeyboardInterrupt:
        pass
    finally:
        if args.log:
            np.savez(args.log, **{k: np.asarray(v) for k, v in log.items()})
            print(f"[{args.name}] wrote {args.log}", flush=True)
        pub.close()
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
