"""Bounded headless SONIC simulation worker; requires a private loopback namespace."""

import argparse
import json
import math
import os
from pathlib import Path
import socket
import time

from gear_sonic.utils.mujoco_sim.base_sim import BaseSimulator
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--host-netns", required=True)
    args = parser.parse_args()
    if [name for _, name in socket.if_nameindex()] != ["lo"]:
        raise RuntimeError("Simulation requires a network namespace with only lo")
    if os.readlink("/proc/self/ns/net") == args.host_netns:
        raise RuntimeError("Refusing the host network namespace")
    config = SimLoopConfig(interface="sim", enable_onscreen=False).load_wbc_yaml()
    assert config["INTERFACE"] == "lo" and config["DOMAIN_ID"] == 0
    (args.output / "sim-config.json").write_text(json.dumps(config, indent=2, default=str))
    sim = BaseSimulator(config=config, onscreen=False, offscreen=False)
    env = sim.sim_env
    original_publish = sim.unitree_bridge.PublishLowState
    noise_tick = 0
    published_noise = 0.0

    def publish_with_yaw_noise(obs):
        # Affect this worker's sensor publication only, never physics or shared
        # simulator source. Left quaternion multiplication adds world yaw.
        nonlocal noise_tick, published_noise
        published_noise = 0.0
        if (args.output / "imu-noise").exists():
            noise_tick += 1
            angle = 0.005 if noise_tick % 2 else -0.005
            published_noise = angle
            pose = np.array(obs["floating_base_pose"], copy=True)
            w, x, y, z = pose[3:7]
            c, s = math.cos(angle / 2), math.sin(angle / 2)
            pose[3:7] = [c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w]
            obs = {**obs, "floating_base_pose": pose}
        original_publish(obs)

    sim.unitree_bridge.PublishLowState = publish_with_yaw_noise
    start = time.monotonic()
    next_record = 0.0
    released = False
    resets = 0
    (args.output / "sim-ready").touch()
    try:
        with (args.output / "physics.jsonl").open("w") as log:
            while time.monotonic() - start < 240 and not (args.output / "sim-stop").exists():
                step_started = time.monotonic()
                if not released and (args.output / "release").exists():
                    sim.handle_keyboard_button("9")
                    sim.handle_keyboard_button("backspace")
                    released = True
                before_step = env.mj_data.time
                env.sim_step()
                if env.mj_data.time <= before_step:
                    resets += 1
                elapsed = time.monotonic() - start
                if elapsed >= next_record:
                    next_record = elapsed + 0.02
                    q = env.mj_data.qpos
                    tilt = math.degrees(
                        math.acos(float(np.clip(1 - 2 * (q[4] ** 2 + q[5] ** 2), -1, 1)))
                    )
                    bridge = sim.unitree_bridge
                    with bridge.low_cmd_lock:
                        kp = [float(m.kp) for m in bridge.low_cmd.motor_cmd[:29]]
                        kd = [float(m.kd) for m in bridge.low_cmd.motor_cmd[:29]]
                    row = {
                        "monotonic": time.monotonic(),
                        "sim_time": float(env.mj_data.time),
                        "pelvis": q[:3].tolist(),
                        "tilt_deg": tilt,
                        "band": bool(env.elastic_band.enable),
                        "automatic_resets": resets,
                        "kp": kp,
                        "kd": kd,
                        "qpos": q.tolist(),
                        "qvel": env.mj_data.qvel.tolist(),
                        "published_base_quat": list(bridge.low_state.imu_state.quaternion),
                        "imu_noise_rad": published_noise,
                        "linear_speed_mps": float(np.linalg.norm(env.mj_data.qvel[:2])),
                        "max_abs_qvel": float(np.max(np.abs(env.mj_data.qvel))),
                    }
                    text = json.dumps(row)
                    log.write(text + "\n")
                    log.flush()
                    temp = args.output / "physics.tmp"
                    temp.write_text(text)
                    temp.replace(args.output / "physics-latest.json")
                time.sleep(max(0, config["SIMULATE_DT"] - (time.monotonic() - step_started)))
    finally:
        sim.close()


if __name__ == "__main__":
    main()
