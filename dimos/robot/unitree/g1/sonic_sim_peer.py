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
#
# ---------------------------------------------------------------------------------------------
# Ported sources
#
# - Robot, gravity, Decoder and the gain, scale and default-pose constants they use are ported
#   from lerobot examples/g1_dex3_training/sonic_roundtrip_audit.py (and NOMINAL_BODY from
#   sonic_targets.py in the same directory), lerobot commit 8d4503c60e:
#     Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#     Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
#     except in compliance with the License. You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#   Changes: the scene and model directory are arguments; only the parts the stand-in uses are
#   kept (no hanging startup, palm FK or audit code); the elastic band is a replaceable callable.
# - IdlePlanner and anchor6 from the G1 sync artifacts' arm_reference.py (not in the repo), the
#   planner context from demo_sim_arm_reference.py's planner_final, and startup_on_floor, the
#   guard loop and the CLI from sonic_sim_peer.py in the same directory (Dimensional Inc.).
# ---------------------------------------------------------------------------------------------

"""Real-time MuJoCo stand-in for the guarded native SONIC controller (offline; no robot).

Speaks the guarded controller's local protocol to a DimOS SonicExecutionModule: binds a g1_debug
feedback PUB, subscribes to the module's command/pose references, and keeps the reference-guard
semantics the session relies on (planner idle -> planner=false arms a new epoch -> accepted/applied
pose indices -> planner=true or a 200 ms stream gap returns to the planner). Tokens drive the SONIC
v1.1 decoder and a floor-standing G1: 50 Hz policy, 200 Hz physics. The planner's idle token is
stood in for by encoding the planner model's final idle frame (matches the robot's switch-tick
token). No hands: hand slots report zeros.

Startup keeps the feet on the floor, as on the robot after it is lowered from the gantry: the
model's start pose stands on the floor, an elastic band holds the pelvis at that height through
the 2 s INIT ramp and the start of SONIC in planner idle, then fades out over 2 s.

Usage: python -m dimos.robot.unitree.g1.sonic_sim_peer --feedback tcp://127.0.0.1:15557 \\
           --reference tcp://127.0.0.1:15556 --model-dir <HF GEAR-SONIC snapshot> \\
           --scene <gear_sonic>/data/robot_model/model_data/g1/scene_43dof.xml \\
           [--name robot0] [--log sim_robot0.npz]
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import hashlib
from pathlib import Path
import signal
import time
from typing import Any

import msgpack  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
import zmq

from dimos.robot.unitree.g1.sonic_arm_reference import ISAAC_FROM_MOTOR, MOTOR_NAMES, SonicEncoder

TICK = 0.02
LEASE = 0.2  # the guarded controller's reference timeout
PLANNER_SHA256 = "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea"  # planner V2

# policy_parameters.hpp (via the lerobot harness)
ARM = {"5020": 0.003609725, "7520_14": 0.010177520, "7520_22": 0.025101925, "4010": 0.00425}
EFFORT = {"5020": 25.0, "7520_14": 88.0, "7520_22": 139.0, "4010": 5.0}
W = 10 * 2.0 * 3.1415926535
_LEG = ["7520_22", "7520_22", "7520_14", "7520_22", "5020", "5020"]
_ARM = ["5020"] * 5 + ["4010"] * 2
MOTOR_TYPE = _LEG + _LEG + ["7520_14", "5020", "5020"] + _ARM + _ARM
DOUBLE = {4, 5, 10, 11, 13, 14}  # ankles, waist roll/pitch: 2x kp and kd
KP = np.array([ARM[t] * W * W * (2 if i in DOUBLE else 1) for i, t in enumerate(MOTOR_TYPE)])
KD = np.array([2 * 2.0 * ARM[t] * W * (2 if i in DOUBLE else 1) for i, t in enumerate(MOTOR_TYPE)])
SCALE = np.array([0.25 * EFFORT[t] / (ARM[t] * W * W) for t in MOTOR_TYPE])
# hpp default_angles (lerobot sonic_targets.NOMINAL_BODY), motor order
DEFAULT = np.array(
    [-0.312, 0, 0, 0.669, -0.363, 0] * 2 + [0, 0, 0]
    + [0.2, 0.2, 0, 0.6, 0, 0, 0]
    + [0.2, -0.2, 0, 0.6, 0, 0, 0],
    dtype=np.float64,
)  # fmt: skip
DEFAULT_ISAAC = DEFAULT[ISAAC_FROM_MOTOR]
MOTOR_FROM_ISAAC = np.argsort(ISAAC_FROM_MOTOR)
BODY_NAMES = [f"{name}_joint" for name in MOTOR_NAMES]
HIST = 10


class Robot:
    """The G1 on the floor of scene_43dof.xml: 200 Hz physics, PD torques computed here."""

    def __init__(self, scene: Path) -> None:
        import mujoco

        self._mujoco = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(scene))
        self.m.opt.timestep = 0.005
        self.d = mujoco.MjData(self.m)
        jid = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in BODY_NAMES]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([self.m.jnt_dofadr[j] for j in jid])
        act_joint = {int(self.m.actuator_trnid[a, 0]): a for a in range(self.m.nu)}
        self.act = np.array([act_joint[j] for j in jid])
        self.ctrl_lo, self.ctrl_hi = self.m.actuator_ctrlrange[self.act].T
        self.pelvis = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.band_wrench: Callable[[], NDArray[np.float64]] = self.hanging_band

    def floor_contacts(self) -> int:
        return sum(
            1
            for i in range(self.d.ncon)
            if self.floor in (self.d.contact[i].geom1, self.d.contact[i].geom2)
        )

    def state(
        self,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        q = self.d.qpos[self.qadr].copy()
        dq = self.d.qvel[self.vadr].copy()
        quat = self.d.qpos[3:7].copy()
        gyro = self.d.qvel[3:6].copy()  # free-joint angular velocity is body-local
        return q, dq, quat, gyro

    def hanging_band(self) -> NDArray[np.float64]:
        """gear_sonic ElasticBand (unitree_mujoco): spring-damper holding the pelvis at (0,0,1), upright."""
        pos, rot = self.d.xpos[self.pelvis], self.d.xmat[self.pelvis].reshape(3, 3)
        lin, ang = self.d.qvel[0:3], rot @ self.d.qvel[3:6]
        f = 10000 * (np.array([0, 0, 1.0]) - pos) - 1000 * lin
        w, v = self.d.qpos[3], self.d.qpos[4:7]
        angle = 2 * np.arctan2(np.linalg.norm(v), w)
        rotvec = v / max(float(np.linalg.norm(v)), 1e-12) * angle
        return np.asarray(np.r_[f, -1000 * rotvec - 10 * ang], np.float64)

    def step(
        self, target_motor: NDArray[Any], n: int = 4, band: bool = False
    ) -> NDArray[np.float64]:
        """Returns the per-joint fraction of substeps whose PD torque hit the actuator limit."""
        sat = np.zeros(29)
        for _ in range(n):
            self.d.xfrc_applied[self.pelvis] = self.band_wrench() if band else 0
            q, dq = self.d.qpos[self.qadr], self.d.qvel[self.vadr]
            tau = KP * (target_motor - q) - KD * dq
            sat += (tau <= self.ctrl_lo) | (tau >= self.ctrl_hi)
            self.d.ctrl[self.act] = np.clip(tau, self.ctrl_lo, self.ctrl_hi)
            self._mujoco.mj_step(self.m, self.d)
        return sat / n


def gravity(quat: NDArray[Any]) -> NDArray[np.float64]:
    w, x, y, z = quat / np.linalg.norm(quat)
    # rotate (0,0,-1) by conj(quat)
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return np.asarray(r.T @ np.array([0.0, 0.0, -1.0]), np.float64)


class Decoder:
    """SONIC v1.1 decoder at 50 Hz: token plus 10-frame proprioception history -> motor targets."""

    def __init__(self, path: Path) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()  # type: ignore[attr-defined]
        so.intra_op_num_threads = so.inter_op_num_threads = 1
        self.s = ort.InferenceSession(str(path), so, providers=["CPUExecutionProvider"])
        self.name = self.s.get_inputs()[0].name
        assert self.s.get_inputs()[0].shape[-1] == 994 and self.s.get_outputs()[0].shape[-1] == 29
        self.reset()

    def reset(self) -> None:
        z = np.zeros(29)
        self.h: dict[str, list[NDArray[np.float64]]] = {
            "ang": [np.zeros(3)] * HIST,
            "q": [z] * HIST,
            "dq": [z] * HIST,
            "act": [z] * HIST,
            "g": [np.array([0, 0, -1.0])] * HIST,
        }
        self.last = z.copy()

    def __call__(
        self,
        token: NDArray[Any],
        q_motor: NDArray[Any],
        dq_motor: NDArray[Any],
        quat: NDArray[Any],
        gyro: NDArray[Any],
    ) -> NDArray[np.float64]:
        def push(k: str, v: NDArray[np.float64]) -> None:
            self.h[k] = [*self.h[k][1:], v]  # oldest -> newest

        push("ang", gyro)
        push("q", q_motor[ISAAC_FROM_MOTOR] - DEFAULT_ISAAC)
        push("dq", dq_motor[ISAAC_FROM_MOTOR])
        push("act", self.last.copy())
        push("g", gravity(quat))
        obs = np.concatenate(
            [token] + self.h["ang"] + self.h["q"] + self.h["dq"] + self.h["act"] + self.h["g"]
        ).astype(np.float32)
        out = np.asarray(self.s.run(None, {self.name: obs[None]})[0][0], np.float64)
        self.last = out
        return np.asarray(DEFAULT + out[MOTOR_FROM_ISAAC] * SCALE, np.float64)  # absolute targets


def anchor6(quat: NDArray[Any]) -> NDArray[np.float64]:
    """Root quaternion (w, x, y, z) with its yaw removed -> first two rotation-matrix columns, row-major."""
    w, x, y, z = quat
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = np.cos(-yaw / 2), np.sin(-yaw / 2)
    w, x, y, z = c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w  # (c,0,0,s) * quat
    m = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z)],
            [2 * (x * z - w * y), 2 * (y * z + w * x)],
        ]
    )
    return np.asarray(m.reshape(-1), np.float64)


class IdlePlanner:
    """Final frame of the planner's IDLE clip from a still context, as the controller's planner
    produces it (seed 1234 = LocalMotionPlanner initial_random_seed; matches the logged switch-tick
    token)."""

    def __init__(self, path: Path) -> None:
        import onnxruntime as ort

        if hashlib.sha256(path.read_bytes()).hexdigest() != PLANNER_SHA256:
            raise ValueError(f"{path} is not planner_sonic.onnx V2")
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])

    def final(self, body_motor: NDArray[Any]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Measured [29] motor-order pose -> (planner idle joints [29] motor order, anchor [6])."""
        ctx = np.zeros((1, 4, 36), np.float32)
        ctx[0, :, 2], ctx[0, :, 3], ctx[0, :, 7:] = 0.788740, 1, body_motor
        allowed = np.zeros((1, 11), np.int64)
        allowed[0, :6] = 1
        qpos, n = self.session.run(
            None,
            {
                "context_mujoco_qpos": ctx,
                "target_vel": np.array([-1], np.float32),
                "mode": np.array([0], np.int64),
                "movement_direction": np.zeros((1, 3), np.float32),
                "facing_direction": np.array([[1, 0, 0]], np.float32),
                "random_seed": np.array([1234], np.int64),
                "height": np.array([-1], np.float32),
                "has_specific_target": np.zeros((1, 1), np.int64),
                "specific_target_positions": np.zeros((1, 4, 3), np.float32),
                "specific_target_headings": np.zeros((1, 4), np.float32),
                "allowed_pred_num_tokens": allowed,
            },
        )
        frame = np.asarray(qpos[0, int(np.ravel(n)[0]) - 1], np.float64)
        return frame[7:], anchor6(frame[3:7])


def startup_on_floor(
    robot: Robot, dec: Decoder, planner_token_of: Callable[[NDArray[Any]], NDArray[np.float64]]
) -> tuple[NDArray[np.float64], dict[str, Any]]:
    """INIT ramp and SONIC start with the feet down, held by a band that then fades out."""
    import mujoco

    mujoco.mj_resetData(robot.m, robot.d)  # type: ignore[attr-defined]
    mujoco.mj_forward(robot.m, robot.d)
    target, scale = robot.d.xpos[robot.pelvis].copy(), [1.0]

    def band() -> NDArray[np.float64]:  # the harness's elastic band, at standing height, scaled
        pos, rot = robot.d.xpos[robot.pelvis], robot.d.xmat[robot.pelvis].reshape(3, 3)
        lin, ang = robot.d.qvel[0:3], rot @ robot.d.qvel[3:6]
        w, v = robot.d.qpos[3], robot.d.qpos[4:7]
        rotvec = v / max(float(np.linalg.norm(v)), 1e-12) * 2 * np.arctan2(np.linalg.norm(v), w)
        return np.asarray(
            scale[0] * np.r_[10000 * (target - pos) - 1000 * lin, -1000 * rotvec - 10 * ang]
        )

    robot.band_wrench = band
    q0 = robot.state()[0]
    for j in range(100):  # INIT: 2 s PD ramp to the default pose
        r = min(1.0, (j + 1) / 50)
        robot.step(q0 * (1 - r) + DEFAULT * r, band=True)
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


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--feedback", required=True, help="bind address for g1_debug feedback")
    parser.add_argument(
        "--reference", required=True, help="the execution module's reference endpoint"
    )
    parser.add_argument(
        "--model-dir",
        type=Path,
        required=True,
        help="HF GEAR-SONIC snapshot (sonic_v1_1/, planner_sonic.onnx)",
    )
    parser.add_argument("--scene", type=Path, required=True, help="gear_sonic scene_43dof.xml")
    parser.add_argument("--name", default="sim")
    parser.add_argument("--log", default=None, help="npz of the simulated state, written on exit")
    args = parser.parse_args(argv)

    encoder = SonicEncoder(args.model_dir / "sonic_v1_1/model_encoder.onnx")
    robot, dec = Robot(args.scene), Decoder(args.model_dir / "sonic_v1_1/model_decoder.onnx")
    idle_planner = IdlePlanner(args.model_dir / "planner_sonic.onnx")

    def planner_token_of(q: NDArray[Any]) -> NDArray[np.float64]:
        joints, anchor = idle_planner.final(q.astype(np.float32))
        token: NDArray[np.float64] = encoder.encode(joints[None], anchor)[0]
        return token

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
    log: dict[str, list[Any]] = {
        k: [] for k in ("t", "armed", "q", "dq", "pelvis_z", "tilt", "contacts", "tick_ms")
    }
    stop: list[bool] = []
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
            if not planner and now - last_pose > LEASE:  # stream gap: back to the planner
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
            log["tilt"].append(float(np.degrees(np.arccos(np.clip(-gravity(quat)[2], -1, 1)))))
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
            arrays: dict[str, Any] = {k: np.asarray(v) for k, v in log.items()}
            np.savez(args.log, **arrays)
            print(f"[{args.name}] wrote {args.log}", flush=True)
        pub.close()
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
