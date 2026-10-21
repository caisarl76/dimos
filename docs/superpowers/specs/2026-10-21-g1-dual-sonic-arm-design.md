# Two G1s from one workstation process: synchronized SONIC arm skills

Status: user decisions incorporated, awaiting approval, 2026-10-21.

## Intent

Operate two Unitree G1s, pc2_222 (192.168.0.222) and pc2_223 (192.168.0.223),
from **one DimOS process on the workstation**. The first target is
synchronization: one command makes both robots perform the same arm skill at
the same time.

The demo: `dimos agent-send "both robots wave"`. Both robots raise the left
forearm and swing it side to side a few times. Their motions start within
0.5 s of each other.

Today every robot runs its own DimOS process on its own PC2
(`unitree-g1-sonic-arm`). That process talks to the local guarded SONIC
controller over `127.0.0.1:5556` (reference, DimOS binds) and
`127.0.0.1:5557` (feedback, native binds). Nothing in the stack knows about a
second robot.

## Goals

1. One `dimos run` on the workstation drives both robots' arm skills.
2. Every arm skill takes `robot`: `"222"`, `"223"` or `"both"`. The agent uses it
   to send per-robot commands ("223 raise your arm") or joint ones ("both wave").
3. `both` starts the two motions within **0.5 s** of each other. The skew is
   measured from each robot's feedback (first sample that moves away from its
   start), and it is logged and returned in the skill result.
4. `both` is all-or-nothing at the start. If either robot is not ready, busy,
   fails the path check or fails planning, **neither moves** and the result
   names the robot and reason.
5. Wave and the other Tier-1 gestures come from `perform_gesture(name)` in
   the Tier-1 design (`2026-10-21-g1-sonic-tier1-skills-design.md`), exposed
   here with `robot`. There is no separate wave.
6. The setup is proven on two MuJoCo sims on the workstation first. Hardware
   comes the next morning, with both robots on support stands.

## Non-goals

- Lockstep motion beyond the start. The open-loop chunks run independently once
  started.
- Walking, navigation and the head cameras. The `observe` skill is dropped from
  this blueprint because the cameras are read over USB on each PC2.
- Polishing the wave motion.
- More than these two robots. The robot set is fixed at 222 and 223.
- Changes to the guarded native launcher or the SONIC controller on the PC2s.

## Anti-goals (failure even if it "works")

- One robot moves while the other refuses a `both` command at the start.
- Breaking the existing single-robot `unitree-g1-sonic-arm` blueprint or its
  skill signatures. It still runs on a PC2 as today.
- Any path where a fault on one robot leaves the other streaming unattended.

## Decided design

**Process topology.** One workstation process holds two `SonicExecutionModule`
instances, `instance_name` `g1_222` and `g1_223`, with their streams remapped
apart. Both connect only to workstation-localhost ports:

| Robot | reference_bind (DimOS binds) | feedback_endpoint |
| --- | --- | --- |
| 222 | `tcp://127.0.0.1:15556` | `tcp://127.0.0.1:15557` |
| 223 | `tcp://127.0.0.1:16556` | `tcp://127.0.0.1:16557` |

Each instance gets its own `journal_dir`
(`STATE_DIR / "sonic-arm" / "<robot>"`). The `SonicModelProfile` is the same as
in the single-robot blueprint.

**Reaching the robots: SSH tunnels.** The native launchers keep their fixed
localhost endpoints. One ssh per robot (hosts `pc2_222` / `pc2_223` in
`~/.ssh/config`, user `unitree`):

```
ssh -N -R 5556:127.0.0.1:15556 -L 15557:127.0.0.1:5557 pc2_222
ssh -N -R 5556:127.0.0.1:16556 -L 16557:127.0.0.1:5557 pc2_223
```

`-R` makes the PC2's `127.0.0.1:5556` reach the workstation bind, and `-L`
brings the native feedback publisher to the workstation. The DimOS config is
therefore identical in sim and on hardware: only what serves those four ports
changes. The tunnels are started by the operator, with the commands documented
in the blueprint docstring. DimOS does not manage them.

**Sim (my call).** Run two copies of the real-time MuJoCo stand-in for the
guarded native controller, `dimos.robot.unitree.g1.sonic_sim_peer` once Tier 1
moves it into the repo (`--model-dir`, `--scene`), otherwise
`docs/artifacts/g1_pc2_223_sync_20261014/sonic_sim_peer.py`, in the host
network namespace on the ports in the table above:

```
python sonic_sim_peer.py --name g1_222 --feedback tcp://127.0.0.1:15557 --reference tcp://127.0.0.1:15556
python sonic_sim_peer.py --name g1_223 --feedback tcp://127.0.0.1:16557 --reference tcp://127.0.0.1:16556
```

The stand-in speaks the guarded controller's ZMQ protocol, sends g1_debug
feedback, keeps the guard semantics (200 ms expiry, planner handback) and
starts on the floor in planner idle (`startup_on_floor`). It uses no DDS, so the
two copies cannot cross-talk and need no namespaces. Sync is a property of the
DimOS side, so the stand-in is enough to prove it. Native-controller behaviour
is proven on hardware the next morning. Running two native-binary MuJoCo sims
(guarded build from
`/tmp/dimos-g1-sonic-navigation-20261013/build/sonic-navigation-runtime/`,
one private `unshare --net` per sim, bridged ports) is a fallback only if the
stand-in proves inadequate. Record each stand-in's real-time lateness. A sim
fall under load is a load problem, not a sync failure.

**Known native gap:** a lost stream on hardware still lets the native planner
snap a raised arm home. `c1e58e3ea` retreats along the plan after a safety
stop, but it cannot cover a dropped tunnel. Keep the robots on stands.

**Bring-up order (my call).** On hardware: planner idle on each robot (`start-idle-planner.py`
on the PC2, which needs 5556 itself), then the tunnels, then `dimos run`.
Restarting one robot means stopping DimOS and that robot's tunnel and repeating
the order. A robot that drops out while DimOS runs just reads as not ready.

**Builds on Tier 1 (agreed with that session, 2026-10-21).** The Tier-1
keyframe engine, background lifecycle with `stop_arm`, and `perform_gesture`
(its tasks 1-4) land first on `jihun/feat/g1-sonic-arm-skill`. The dual work
builds on them and does not refactor `_move`/`_stream` in parallel.

**Skills.** A dual-robot skill container exposes `move_upper_body`,
`move_left_arm_pose`, `move_left_arm_joints` (with Tier-1 `hold_seconds`) and
`perform_gesture`, each with an added `robot: Literal["222", "223", "both"]`
parameter. They run in the background like the Tier-1 skills: they return at
once and report through a tool update. `stop_arm()` stops every running arm with
the smooth retreat. The per-robot engine (plan, path check, envelope watch,
chunking, retreat, handback) is pulled out of `SonicArmSkillContainer` into a
per-robot object that both containers use. Each robot keeps its own watch,
feedback input and execution reference. The encoder and collision checker are
shared, loaded once on the workstation. Camera-based Tier-1 skills
(`point_at_object`) are not exposed.

**`both` sequence.** (1) Plan and path-check both. (2) Check both executions are
ready and idle. (3) Start both tasks and submit both chunks concurrently, one
thread per robot, released together. (4) Wait for both and return one result
per robot plus the measured start skew.

**One arm command at a time (user decision).** The existing process-wide
`CAP_MOVEMENT` hold is kept. A command to one robot waits for any running arm
command to finish, even one on the other robot. `both` is the way to move them
together. The per-robot busy lock stays only as a guard.

**Fault on one robot during `both` (user decision).** If either robot leaves its
envelope or goes over speed, both robots ease back to their start poses the same
way the existing envelope retreat works
(retreat along the plan, `c1e58e3ea`). Neither is hard-cancelled mid-motion.
A transport or execution failure on one robot (no feedback, task error) still
takes the existing failure path on that robot, and the other eases back.

**Blueprints.** Add `unitree-g1-sonic-arm-dual` (execution ×2, dual skill
container, McpServer) and `unitree-g1-sonic-arm-dual-agentic` (+ McpClient with
the G1 SONIC arm prompt extended to explain `robot`).
Regenerate `all_blueprints.py`.

**Files the workstation needs:** the SONIC v1.1 encoder
(`model_encoder.onnx`) and the gear_sonic `scene_43dof.xml`, passed as the
container's `encoder_path` / `collision_model_path`, as today.

## Acceptance

Sim (tonight):
- `dimos agent-send "both robots wave"`: both sim arms wave, skew < 0.5 s
  (reported), both hand back to the planner.
- `dimos agent-send "robot 223 raise your left arm forward"`: only 223 moves.
- With one sim stopped after DimOS is up, a `both` command moves neither and
  names the missing robot.
- Existing `test_sonic_arm_skill.py` stays green, plus one small test for the
  `both` all-or-nothing start and skew reporting (fake executions).

Hardware (next morning, user launches both guarded controllers, robots on
stands): the same three checks over the SSH tunnels. The user confirms pc2_222
is usable in the same no-hands setup as pc2_223 from next morning.

## Left to the builder

- The internal split between the per-robot runner and the dual container, and
  how the two execution references are bound (spec injection with remappings,
  or explicit refs).
- How the start barrier is implemented, as long as goal 3 is measured and met.
- Sim launcher scripts. Keep them under `docs/artifacts/` like earlier trials,
  not in the product code.
