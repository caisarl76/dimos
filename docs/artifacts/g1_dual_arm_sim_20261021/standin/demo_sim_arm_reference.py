"""Closed-loop MuJoCo check of the left-arm-forward token stream (offline; no robot).

Plant and v1.1 decoder: lerobot examples/g1_dex3_training/sonic_roundtrip_audit.py (gains, layout and action
scale from g1_deploy_onnx_ref + policy_parameters.hpp; 200 Hz physics, 50 Hz decoder). The planner's idle
token is stood in for by encoding the planner model's final idle frame from the measured pose, which matches
the robot's logged idle token to within one quantization step in 9/64 values (note §13).
Usage: python demo_sim_arm_reference.py [--delta -0.3]
"""

import argparse
import importlib
from pathlib import Path
import sys

import mujoco
import numpy as np
import onnxruntime as ort

from arm_reference import LEFT_SHOULDER_PITCH, Encoder, arm_poses, token_schedule

HF = Path.home() / ".cache/huggingface/hub/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2"
LEROBOT = Path.home() / "work/lerobot/examples/g1_dex3_training"
SCENE = Path.home() / "work/GR00T-WholeBodyControl-hand-tracking-watchdog/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
NAMES = ["LHipPitch", "LHipRoll", "LHipYaw", "LKnee", "LAnklePitch", "LAnkleRoll", "RHipPitch", "RHipRoll",
         "RHipYaw", "RKnee", "RAnklePitch", "RAnkleRoll", "WaistYaw", "WaistRoll", "WaistPitch", "LShPitch",
         "LShRoll", "LShYaw", "LElbow", "LWrRoll", "LWrPitch", "LWrYaw", "RShPitch", "RShRoll", "RShYaw",
         "RElbow", "RWrRoll", "RWrPitch", "RWrYaw"]

sys.path.insert(0, str(LEROBOT))
harness = importlib.import_module("sonic_roundtrip_audit")
harness.MODEL, harness.SCENE = HF / "sonic_v1_1", str(SCENE)


def planner_final(planner, joints_motor):
    ctx = np.zeros((1, 4, 36), np.float32)
    ctx[0, :, 2], ctx[0, :, 3], ctx[0, :, 7:] = 0.788740, 1, joints_motor
    allowed = np.zeros((1, 11), np.int64)
    allowed[0, :6] = 1
    qpos, n = planner.run(None, {
        "context_mujoco_qpos": ctx, "target_vel": np.array([-1], np.float32), "mode": np.array([0], np.int64),
        "movement_direction": np.zeros((1, 3), np.float32), "facing_direction": np.array([[1, 0, 0]], np.float32),
        "random_seed": np.array([1234], np.int64), "height": np.array([-1], np.float32),
        "has_specific_target": np.zeros((1, 1), np.int64), "specific_target_positions": np.zeros((1, 4, 3), np.float32),
        "specific_target_headings": np.zeros((1, 4), np.float32), "allowed_pred_num_tokens": allowed})
    return qpos[0, int(np.ravel(n)[0]) - 1, 7:].astype(np.float64)


def run(scenario, delta, encoder, planner):
    robot, dec = harness.Robot(), harness.Decoder()
    band = scenario == "support"
    stand = encoder.encode(harness.DEFAULT[None])[0]
    if band:  # hang on the band: INIT ramp, then SONIC
        mujoco.mj_resetData(robot.m, robot.d)
        mujoco.mj_forward(robot.m, robot.d)
        q0 = robot.state()[0]
        for j in range(100):
            robot.step(q0 + (harness.DEFAULT - q0) * min(1.0, (j + 1) / 50), band=True)
        dec.reset()
        for _ in range(100):
            robot.step(dec(stand, *robot.state()), band=True)
    else:
        harness.startup_hang(robot, dec, stand)
    planner_token = encoder.encode(planner_final(planner, robot.state()[0].astype(np.float32))[None])[0]
    for _ in range(150):  # planner IDLE settles
        robot.step(dec(planner_token, *robot.state()), band=band)
    base = robot.state()[0]
    tokens = token_schedule(planner_token, encoder.encode(arm_poses(base, LEFT_SHOULDER_PITCH, delta)))
    tokens = np.vstack([tokens, np.repeat(planner_token[None], 100, 0)])  # planner after handback
    for i in range(1, len(tokens)):  # SonicTaskSession token_step_limit (0.05 per tick)
        tokens[i] = tokens[i - 1] + np.clip(tokens[i] - tokens[i - 1], -0.05, 0.05)
    qs, dqs, tilt, contacts, sat = [], [], 0.0, [], np.zeros(29)
    for token in tokens:
        q, dq, quat, gyro = robot.state()
        sat = np.maximum(sat, robot.step(dec(token, q, dq, quat, gyro), band=band))
        qs.append(q)
        dqs.append(dq)
        tilt = max(tilt, float(np.degrees(np.arccos(np.clip(-harness.gravity(quat)[2], -1, 1)))))
        contacts.append(robot.floor_contacts())
    qs, dqs = np.array(qs), np.array(dqs)
    dev = np.abs(qs - base).max(0)
    others = np.delete(np.arange(29), LEFT_SHOULDER_PITCH)
    j = others[int(dev[others].argmax())]
    k = int(np.abs(dqs).max(0).argmax())
    lsp = qs[:, LEFT_SHOULDER_PITCH] - base[LEFT_SHOULDER_PITCH]
    return {
        "scenario": scenario, "delta": delta,
        "left_shoulder_pitch_reached": round(float(lsp.min() if delta < 0 else lsp.max()), 3),
        "left_shoulder_pitch_end": round(float(lsp[-1]), 3),
        "other_joint_max_dev": (NAMES[j], round(float(dev[j]), 3)),
        "peak_dq": (NAMES[k], round(float(np.abs(dqs).max()), 2)),
        "end_posture_max_dev": round(float(np.abs(qs[-1] - base).max()), 3),
        "max_tilt_deg": round(tilt, 2),
        "min_floor_contacts": None if band else int(min(contacts)),
        "max_torque_saturation": round(float(sat.max()), 2),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delta", type=float, default=-0.3)
    args = parser.parse_args()
    encoder = Encoder(HF / "sonic_v1_1/model_encoder.onnx")
    planner = ort.InferenceSession(str(HF / "planner_sonic.onnx"), providers=["CPUExecutionProvider"])
    for scenario in ("support", "floor"):
        for delta in (0.0, args.delta):
            print(run(scenario, delta, encoder, planner), flush=True)


if __name__ == "__main__":
    main()
