# Robot Safety

Read this before you touch any robot. The general rules apply to every robot; each robot then has its own section.

## 1. The rules

1. Never work alone on hardware. One person runs code, one person stands at the E-stop.
2. Nothing of yours goes inside the robot's reach while it is powered on.
3. New code runs in sim first. First run on hardware: slow speed, hand on the E-stop.
4. Software stop is not a safety stop. If anything looks wrong, hit the E-stop first, debug later.
5. Never leave a powered robot unattended. Walking away = E-stop pressed or power off.
6. Don't change firmware, joint limits, speed limits or safety settings. Ask your Dimensional engineer.
7. Anything breaks, sparks, smells or hurts someone: power off, then tell us (see [Contacts](#8-contacts)).

## 2. Your workspace

- Keep a clear zone around the robot: its full reach + 30 cm. Nobody and nothing inside while powered.
- Fixed arms stay clamped to the table. Don't move them.
- Keep cables off the walkway and away from the joints.
- No liquids, loose sleeves, lanyards or loose long hair near the robot.
- Objects you pick up: under the robot's payload limit; nothing sharp, glass or heavy unless approved.

## 3. Three ways to stop (fastest first)

| What | How | What it does |
|---|---|---|
| Hardware E-stop | Hit the red button | Stops all motion. Works even if the software hangs. |
| Teleop release | Let go of the grip (Quest) / E-STOP button (browser) | Robot stops following you. |
| Software stop | Ctrl-C or `dimos stop` | Shuts dimos down. Can take seconds, can fail. |

## 4. Power on

1. Area clear, nobody in the zone.
2. E-stop released (twist it up). Some robots boot into a fault if it is pressed.
3. Power the robot on and wait until it is ready (see its section below).
4. Start dimos with the blueprint listed in the robot's section.
5. First move: small, slow, watched.

## 5. Power off

1. Move the robot to its rest pose.
2. Stop dimos: `dimos stop`.
3. Hold the arms, or make sure nothing is under them: some arms drop when power is cut.
4. Power off.

## 6. After an E-stop or fault

- Find out why it happened before you release the E-stop.
- Follow the recovery steps in the robot's section. Some robots need a full power cycle.
- Tell your engineer if your code caused it.

## 7. AI agents, policies and autonomous code

- Motion driven by an LLM agent (skills, MCP): someone at the E-stop the whole time.
- Trained policies: first runs at low speed, inside a small workspace.

## 8. Contacts

Your Dimensional engineer is your first contact for anything about the hardware. In an emergency, call **911** first.

---

## Galaxea R1 Pro

![R1 Pro standing upright](assets/r1pro-overview.png)

Two 7-joint arms on a 4-joint torso, on a wheeled base. The base can drive and the torso can fold and stand up, so keep a clear floor around the whole robot, not just in front of the arms.

### Switches and E-stop

![R1 Pro soft switch, emergency stop button, power switch and charging port](assets/r1pro-switches.png)

| Control | Where | What it does |
|---|---|---|
| Emergency stop button | Top of the back of the chassis | Stops all motion. It does **not** cut power. |
| Power switch (rocker) | Bottom of the back of the chassis | Main power. |
| Soft switch | Under the torso, at the back | Turns the computer and drives on and off. |
| Charging port (48 V) | Bottom of the back of the chassis | Charger only. |

### Remote controller

![R1 Pro remote controller](assets/r1pro-remote.png)

![Remote controller switch positions for each mode](assets/r1pro-remote-modes.png)

- Before you turn the remote on, push all four switches (SWA, SWB, SWC, SWD) **up**.
- Whenever you are not driving, keep all four switches up. The robot stays stopped.
- **SWD down = emergency stop**, in any mode.

### Power on

1. Floor clear around the robot, nothing under or next to the arms.
2. Emergency stop button released. Remote on, all switches up.
3. Turn the power switch on.
4. Press the soft switch and wait for the blue light.
5. Start dimos on the robot, for example `dimos run r1pro-coordinator`. See the [R1 Pro README](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/galaxea/r1pro/README.md) for the full setup and the other blueprints.

### Power off

1. `dimos stop`.
2. Hold the arms steady, or make sure nothing is under them. They can drop when power is cut.
3. Hold the soft switch until the blue light goes off.
4. Turn the power switch off.

### After an E-stop

The R1 Pro needs a full restart after its E-stop (button or SWD) has been used. Releasing it is not enough; carrying on without a restart makes the robot misbehave.

1. `dimos stop`.
2. Release the E-stop and put SWD back up.
3. Turn the power switch off, then follow **Power on** from the top.

Images: [Galaxea Dynamics R1 Pro documentation](https://docs.galaxea-dynamics.com/R1Pro/en/docs/2026/getting_started/quick_start/r1pro_quick_start/).
