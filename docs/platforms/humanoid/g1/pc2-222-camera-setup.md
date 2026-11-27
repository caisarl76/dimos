# pc2_222 native RealSense camera setup

This follows the [powered-on inventory](/docs/platforms/humanoid/g1/pc2-222-inventory-2026-10-06.md) on 2026-10-06. The robot is supported or suspended, as confirmed by the user. This session configures and tests camera acquisition; it does not commission robot motion.

## How this checkout accesses the D435i

The existing RealSenseCamera wrapper (`/dimos/hardware/sensors/camera/realsense/camera.py`) launches a Rust capture process (`/dimos/hardware/sensors/camera/realsense/rust/src/main.rs`). The process uses native `librealsense` and publishes typed DimOS messages over Zenoh. Python configures the module and receives those messages. `pyrealsense2` is useful for separate SDK diagnostics but is not a runtime dependency of this module.

The initial inventory incorrectly treated the missing Python binding as the DimOS integration blocker. The actual missing pieces were `realsense_native` and native `librealsense`. The default source build uses Nix, which was absent on this PC2; this session builds the same source with user-local dependencies instead.

| Component | Location/version |
|---|---|
| DimOS source | `/home/unitree/dimos`, commit `7529659c88c76eb4800552ad1d4fa5dd584ecc80` |
| Camera executable | `/home/unitree/dimos/dimos/hardware/sensors/camera/realsense/rust/target/release/realsense_native` |
| Native library prefix | `/home/unitree/.local/share/dimos/native-realsense/librealsense-2.58.1` |
| Library source | Official `realsenseai/librealsense` tag `v2.58.1`, commit `bf2778061d5dd29776e9aca8765f75852671760b` |
| Rust toolchain | `rustc 1.99.0 (b940084d7 2026-09-28)`, isolated under `/home/unitree/.local/share/dimos/native-realsense/` |
| Build recipe | Recorded Python build script (`/docs/artifacts/g1_pc2_222_20261006/demo_build_native_camera.py`); native Cargo dependencies use `--locked` |
| Build log on PC2 | `/home/unitree/.local/share/dimos/native-realsense/build-20261006.log` |
| Build evidence | [Metadata, library commit, and executable SHA-256](/docs/artifacts/g1_pc2_222_20261006/native-build.json) |

The executable's build script embeds the native library directory in its runtime search path. Keep the installed library prefix with the binary; copying only the executable to the other robot does not install its native dependencies.

## Backend and permissions

The first build selected RSUSB, a user-space USB backend supported by the [RealSense Jetson guide](https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.1/doc/installation_jetson.md). Its first acquisition failed to resolve streams. A direct native-library diagnostic isolated `RS2_USB_STATUS_ACCESS`: `/dev/bus/usb/001/003` was `root:root`, mode `0664`, and not writable by `unitree`. A repeat through the existing system-Python binding still acquired RGB/depth successfully.

A narrowly scoped D435i access rule was prepared, but `sudo -n` required a password and exited before the script executed. No udev rule or permission change was applied. The selected build therefore uses **V4L2** (`FORCE_RSUSB_BACKEND=OFF`) and the existing video-device permissions. No kernel patches, firmware updates, system-package installation, or Python package changes were made. The camera module itself applies its configured streaming/emitter/exposure options during acquisition.

## Camera-only check

The bounded probe (`/docs/artifacts/g1_pc2_222_20261006/demo_dimos_camera_probe.py`) creates only `RealSenseCamera`, with unique topic names and loopback-only Zenoh transport. It receives color, depth, color/depth calibration, and camera TF messages; checks image sizes and aligned intrinsics/frame IDs; and stops the camera subprocess in `finally`. It saves no images and imports no Unitree control module. It does not start the full robot blueprint or a persistent service.

Configuration for this USB 2.1 connection:

```python skip
RealSenseCamera.blueprint(
    width=640,
    height=480,
    fps=15,
    frame_id="d435_link",
    enable_color=True,
    enable_depth=True,
    align_depth_to_color=True,
    enable_infrared=False,
    enable_imu=False,
    enable_pointcloud=False,
)
```

The standard camera config defaults to width 848, and the standalone visualization blueprint enables infrared. The bounded check explicitly selects the already-observed 640×480 RGB/depth profiles; the default visualization launch is not the tested configuration.

Native depth output uses `16UC1` / DimOS `DEPTH16`, with raw sensor units. The earlier hardware probe measured approximately `0.001 m` per unit. Account for that scale before interpreting depth as metric positions. Alignment matches the RGB optical frame; it does not establish a camera-to-pelvis or camera-to-world transform.

**Validation passed twice**, first from the temporary probe and then from the installed launcher. Run the same bounded camera check from the workstation:

```bash skip
ssh pc2_222 sh /home/unitree/.local/share/dimos/native-realsense/check-camera
```

The launcher selects the robot's DimOS venv, existing native executable, and loopback transport. It waits up to 40 seconds for the required messages, prints `CAMERA_PROBE_RESULT`, and closes the camera. The successful installed run took approximately eight seconds including startup and shutdown.

| Installed check | Observed result |
|---|---|
| RGB | 91 frames, 91 distinct timestamps, 640×480 `uint8` RGB, 15.21 callbacks/s |
| Aligned depth | 90 frames, 90 distinct timestamps, 640×480 `uint16` DEPTH16, 15.04 callbacks/s |
| Calibration | Six messages each for color and aligned depth; matching intrinsic matrices and `d435_color_optical_frame` IDs |
| Camera TF | 91 messages; these do not validate the camera-to-robot mounting transform |
| Last depth frame | 99.60% positive pixels; median positive value 1248 raw units |
| Shutdown | Python launcher and native camera process both exited with code 0 |

Both runs logged two dropped color frames near startup and one suppressed duplicate depth frame. These short acquisitions establish transport and acquisition functionality; they do not measure sustained reliability, geometric accuracy, or RGB/body timestamp synchronization. Images were not saved or visually inspected. Infrared, IMU, and point-cloud streams were disabled.

Evidence: [installed check result](/docs/artifacts/g1_pc2_222_20261006/native-camera-installed.json), complete installed check log (`/docs/artifacts/g1_pc2_222_20261006/native-camera-installed.log`), [first successful result](/docs/artifacts/g1_pc2_222_20261006/native-camera.json), and launcher source (`/docs/artifacts/g1_pc2_222_20261006/check-camera`).

## Clock alignment remains open

A [bracketed SSH clock comparison](/docs/artifacts/g1_pc2_222_20261006/clock-comparison.json) found the robot's clock approximately **550 seconds behind the workstation**. The request took 0.214 seconds, and the remote sample has one-second resolution. This establishes a relative discrepancy, not which machine has correct UTC. No clock settings were changed. Resolve host time alignment and validate sensor/body timestamp domains before cross-host recording, freshness checks, or perception fusion. Camera callback-rate estimates above use monotonic elapsed time and are not an estimate of synchronization accuracy.

Follow-up [time-source and controller inspection](/docs/platforms/humanoid/g1/pc2-222-inventory-2026-10-06.md#follow-up-publisher-ownership-and-time-sources) found PC2 explicitly synchronized to the internal robot computer at `192.168.123.161`, while the workstation uses `ntp.ubuntu.com`. Preserve the intended robot timing relationship when planning a correction; changing PC2 alone is not yet a validated solution. The same inspection observed a separate deployment publishing body and both-hand commands and then exiting, so the earlier controller status must be rechecked before any hardware commissioning.

## Robot-control information remains separate

Ubuntu, ROS2, JetPack, TensorRT, Unitree's client SDK, and the robot's vendor motion-control software have different versions. Installed packages report `nvidia-jetpack 6.2.1+b38` and `nvidia-l4t-core 36.4.3-20250107174145`; these do not identify the robot's motion-control release. Record the Unitree app's connected-robot software version label/value when available. Do not update software just to obtain that information.

The status response `7301` means `UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE` in the [official Unitree error definitions](https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2/main/include/unitree/robot/g1/loco/g1_loco_error.hpp). The successful FSM query returned `0`. A [read-only recheck after camera setup](/docs/artifacts/g1_pc2_222_20261006/controller-status-after-camera.json) returned the same controller values and a clean robot Git checkout. These observations and the supported/suspended posture do not establish vendor standing-balance readiness.

[Passive discovery](/docs/artifacts/g1_pc2_222_20261006/dds-endpoints.json) found a reader for `rt/arm_sdk` and a reader for each Dex3 command topic. It also saw an existing arm-command writer, two low-level command writers, and two body-state writers. Advertised endpoints do not prove active command delivery or ownership. Attribute those sources and confirm supported mode/firmware before adding a command publisher. Camera calibration, hardware arm/Dex3 adapters, and supervised manipulation commissioning remain under the [service plan](/docs/platforms/humanoid/g1/service-development-plan.md).
