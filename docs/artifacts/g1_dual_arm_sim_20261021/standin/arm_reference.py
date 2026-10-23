"""Joint-space arm reference -> SONIC v1.1 tokens, shared by the simulation check and the robot script.

Encoder mode 0 input layout follows lerobot examples/g1_dex3_training/sonic_targets.build_encoder_inputs:
ten future frames 5 ticks apart at 50 Hz, absolute joints and velocities in IsaacLab order, identity
terminal pose held with zero velocity. Unlike that converter, the baseline is the planner's own idle
frame (IdlePlanner, seeded from the measured pose) and the anchor carries that frame's root roll/pitch with
zero yaw error, i.e. what the controller itself encodes on the planner->stream switch tick (note §14). The
robot's ~0.5-2 deg heading mismatch against the planner is accepted.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

ISAAC_FROM_MOTOR = np.array(
    [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
)
LEFT_SHOULDER_PITCH = 15  # motor order; negative = hand forward (URDF axis +y, arm hanging)
ENCODER_SHA256 = "fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae"  # SONIC v1.1
PLANNER_SHA256 = "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea"  # planner_sonic.onnx V2
RATE = 50
IDENTITY_ANCHOR = np.array([1.0, 0, 0, 1, 0, 0])


def _session(path: Path, sha256: str):
    import onnxruntime as ort

    if hashlib.sha256(path.read_bytes()).hexdigest() != sha256:
        raise ValueError(f"{path} has an unexpected hash")
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    return ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])


def anchor6(quat: np.ndarray) -> np.ndarray:
    """Root quaternion (w, x, y, z) with its yaw removed -> first two rotation-matrix columns, row-major."""
    w, x, y, z = quat
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = np.cos(-yaw / 2), np.sin(-yaw / 2)
    w, x, y, z = c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w  # (c,0,0,s) * quat
    m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x)]])
    return m.reshape(-1)


class IdlePlanner:
    """Final frame of the planner's IDLE clip from a still context, as the controller's planner produces it
    (seed 1234 = LocalMotionPlanner initial_random_seed; matches the logged switch-tick token, note §14)."""

    def __init__(self, path: Path) -> None:
        self.session = _session(path, PLANNER_SHA256)

    def final(self, body_motor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Measured [29] motor-order pose -> (planner idle joints [29] motor order, anchor [6])."""
        ctx = np.zeros((1, 4, 36), np.float32)
        ctx[0, :, 2], ctx[0, :, 3], ctx[0, :, 7:] = 0.788740, 1, body_motor
        allowed = np.zeros((1, 11), np.int64)
        allowed[0, :6] = 1
        qpos, n = self.session.run(None, {
            "context_mujoco_qpos": ctx, "target_vel": np.array([-1], np.float32), "mode": np.array([0], np.int64),
            "movement_direction": np.zeros((1, 3), np.float32), "facing_direction": np.array([[1, 0, 0]], np.float32),
            "random_seed": np.array([1234], np.int64), "height": np.array([-1], np.float32),
            "has_specific_target": np.zeros((1, 1), np.int64), "specific_target_positions": np.zeros((1, 4, 3), np.float32),
            "specific_target_headings": np.zeros((1, 4), np.float32), "allowed_pred_num_tokens": allowed})
        frame = qpos[0, int(np.ravel(n)[0]) - 1].astype(np.float64)
        return frame[7:], anchor6(frame[3:7])


class Encoder:
    def __init__(self, path: Path) -> None:
        self.session = _session(path, ENCODER_SHA256)

    def encode(self, body_motor: np.ndarray, anchor: np.ndarray = IDENTITY_ANCHOR) -> np.ndarray:
        """[N,29] motor-order poses at 50 Hz, constant root anchor [6] -> [N,64] tokens."""
        body = np.asarray(body_motor, np.float64)[:, ISAAC_FROM_MOTOR]
        vel = np.gradient(body, 1 / RATE, axis=0) if len(body) > 1 else np.zeros_like(body)
        vel[-1] = 0
        body = np.vstack([body, np.repeat(body[-1:], 46, 0)])
        vel = np.vstack([vel, np.zeros((46, 29))])
        rows = np.arange(len(body_motor))[:, None] + np.arange(10)[None] * 5
        x = np.zeros((len(body_motor), 1751), np.float32)
        x[:, 4:294] = body[rows].reshape(-1, 290)
        x[:, 294:584] = vel[rows].reshape(-1, 290)
        x[:, 584:644] = np.tile(anchor, 10)
        return np.concatenate([self.session.run(None, {"obs_dict": row[None]})[0] for row in x]).astype(np.float64)


def min_jerk(n: int) -> np.ndarray:
    s = np.linspace(0, 1, n)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def arm_poses(base: np.ndarray, joint: int, delta: float, *, lead_s=1.0, move_s=2.0, hold_s=1.0) -> np.ndarray:
    """Base pose, lead-in, move by delta, hold, move back, lead-out. [N,29] at 50 Hz."""
    lead, move, hold = (int(round(s * RATE)) for s in (lead_s, move_s, hold_s))
    offset = np.r_[np.zeros(lead), delta * min_jerk(move), np.full(hold, delta), delta * min_jerk(move)[::-1],
                   np.zeros(lead)]
    poses = np.tile(np.asarray(base, np.float64), (len(offset), 1))
    poses[:, joint] += offset
    return poses


def blend(a: np.ndarray, b: np.ndarray, seconds: float) -> np.ndarray:
    w = np.linspace(0, 1, int(round(seconds * RATE)) + 1)[1:, None]
    return (1 - w) * a + w * b


def token_schedule(captured: np.ndarray, motion_tokens: np.ndarray, blend_s: float = 2.0) -> np.ndarray:
    """Planner token -> blend -> motion tokens -> blend -> planner token, at 50 Hz."""
    captured = np.asarray(captured, np.float64)
    return np.vstack([blend(captured, motion_tokens[0], blend_s), motion_tokens,
                      blend(motion_tokens[-1], captured, blend_s)])


if __name__ == "__main__":  # self-check of the pure parts (no encoder needed)
    p = arm_poses(np.zeros(29), LEFT_SHOULDER_PITCH, -0.3)
    assert p.shape == (50 + 100 + 50 + 100 + 50, 29) and np.allclose(np.delete(p, LEFT_SHOULDER_PITCH, 1), 0)
    assert p[0, 15] == 0 and p[-1, 15] == 0 and np.isclose(p[:, 15].min(), -0.3)
    assert np.abs(np.diff(p[:, 15])).max() * RATE < 0.3 * 1.875 / (99 / RATE) + 1e-3  # min-jerk peak 1.875*d/T
    tokens = token_schedule(np.zeros(64), np.ones((10, 64)))
    assert len(tokens) == 100 + 10 + 100 and np.allclose(tokens[[0, -1]], [[0.01], [0.0]]) and np.allclose(tokens[100:110], 1)
    assert np.allclose(anchor6(np.array([np.cos(0.3), 0, 0, np.sin(0.3)])), IDENTITY_ANCHOR)  # yaw only -> identity
    pitch = anchor6(np.array([np.cos(0.05), 0, np.sin(0.05), 0]))  # 0.1 rad pitch, yaw-free
    assert np.allclose(pitch, [np.cos(0.1), 0, 0, 1, -np.sin(0.1), 0])
    print("arm_reference self-check ok")
