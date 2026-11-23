# G1 demo runbook (docker compose)

Everything runs from this directory on the demo workstation. Steps 1 to 3 are checks; 4 onward is
the demo. The robots are driven from the tmux panes that `demo.sh up` opens; every pane has its
command typed, not run: press Enter in the order given. Nothing here needs a dimos checkout:
the bundle is `compose.yaml`, `compose.demo.yaml`, `demo.sh`, `.env`, `volo/deploy/g1-loop/compose.yaml`
and the two loaded images (`dimos-g1-demo`, `volo-g1-workstation:g1-loop-final-20261028`).

## 0. New workstation, once

`.env` next to the compose files (compose and `demo.sh` both read it):

```
VOLO_DIR=./volo
PC2_SSH_HOST=unitree@192.168.0.223
PC2_222_SSH_HOST=unitree@192.168.0.222
SONIC_ENCODER=<path>/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2/sonic_v1_1/model_encoder.onnx
GEAR_SONIC_G1_DIR=<path>/g1
WORKSTATION_IP=<this machine's address on the robot network>
SSH_DIR=<path>/ssh
```

`WORKSTATION_IP` goes into 222's loop profile (`policy_host`) and the loop's VLM URL: the harness on
222 finds the policy and VLM servers through it; `demo.sh` refuses to start without it (no default). `SSH_DIR` is needed when `id -u` is not 1000: the
containers run as uid 1000 and cannot read another user's `~/.ssh`, which shows up as the tunnel
restarting with "Host key verification failed". Make the copy once:

```
sudo install -d -o 1000 -g 1000 -m 700 <path>/ssh
sudo install -o 1000 -g 1000 -m 600 ~/.ssh/id_rsa <path>/ssh/id_rsa      # the key the robots accept
ssh-keyscan 192.168.0.223 192.168.0.222 | sudo tee <path>/ssh/known_hosts >/dev/null
sudo chown 1000:1000 <path>/ssh/known_hosts
```

## 1. Clean slate

```
./demo.sh down
```

## 2. Verify files, ssh copy, images

```
grep -c SSH_DIR compose.yaml                                   # 2
grep -E 'WORKSTATION_IP|SSH_DIR|SSH_HOST' .env
sudo ls -la "$(sed -n 's/^SSH_DIR=//p' .env)"                  # drwx------ 1000, id_rsa 600, known_hosts non-empty
docker images | grep -E 'dimos-g1-demo|volo-g1-workstation'
docker compose -f compose.demo.yaml --profile workstation config --services   # tunnel dimos policy vlm
```

## 3. The tunnel's ssh, exactly as the container runs it

```
docker compose -f compose.demo.yaml run --rm --entrypoint ssh tunnel -o BatchMode=yes unitree@192.168.0.223 true && echo tunnel-ssh-ok
```

"Host key verification failed": pc2 is missing from the mounted `known_hosts` (or the directory is
unreadable, see step 0). "Permission denied (publickey)": wrong key file in the copy.

## 4. GPU servers and the panes

```
./demo.sh up
```

Starts policy and vlm, makes sure tunnel and dimos are stopped, opens tmux session `g1-demo`.
Window `ws` follows the server logs: wait for `ready on port 5571` (about 14 min from a cold disk).
`up` refuses while the session exists: attach to it or `demo.sh down` first.

## 5. Window 223, pane order

- `ctrl`: pc2's sudo password, clock check, guarded controller. Wait for the stance.
- `planner`: idle planner preview, then start; when it confirms and exits, the same line brings
  tunnel and dimos up. Check: `docker compose -f compose.demo.yaml ps tunnel` says Up and stays Up.
- `log`: wait until the blueprint lists its MCP tools (about a minute).
- `test`: one natural invite gesture with its line.
- `greeter`: the show. Ctrl-C ends it; the running gesture finishes.

"SONIC is not in confirmed planner idle": the controller is not attached through the tunnel, the
planner pane did not complete, or the test came within the first minute. Which one:

```
docker compose -f compose.demo.yaml logs --tail 20 tunnel
docker compose -f compose.demo.yaml exec dimos python -c "from dimos.porcelain.dimos import Dimos; a=Dimos.connect(); s=a.SonicExecutionModule.get_status(); print({k: s[k] for k in ('state','subscriptions_ready','feedback_fresh','feedback_age_s','ready','error')}); f=a.SonicExecutionModule.get_feedback(); print(None if f is None else f.reference_guard); a.stop()"
```

`subscriptions_ready` false or `feedback_fresh` false: tunnel or controller. Both true with
`planner_confirmed` false or `armed` true: redo the planner pane (step 7's note).

"stream did not arm (tick deadline missed); control returned to the planner": the blueprint's own
streaming loop (in the dimos container) stalled for more than 100 ms while arming, and the session
aborted itself; pc2 stays in planner idle. Every journal event is fsynced from that loop, and the
journal lives in the `dimos-state` volume: a Docker data root on an HDD that the policy server is
loading from at the same time, or CPU starvation while the GPU servers start, both do it. Wait for
`ready on port 5571` and a quiet box, then press Enter on the test pane again. If it recurs on an
idle box: `docker info --format '{{.DockerRootDir}}'`, `cat /proc/loadavg`, `docker stats --no-stream`.

## 6. Window 222, pane order

- `T0`: RealSense camera server on :5555. Leave it running.
- `T1`: SONIC deploy with planner hold. Wait for `Init Done`.
- Before `T2`, warm the policy server from 222 (the harness gives its first request 10 s, once;
  a cold server misses that). Talks only to the server, nothing reaches the robot:

  ```
  ssh unitree@192.168.0.222 'cd ~/GR00T-WholeBodyControl-loop-20261026 && PYTHONPATH=$PWD ~/GR00T-WholeBodyControl-harness-20261022/venv/bin/python -c "import numpy as np, time; from gear_sonic.utils.inference.lerobot_sonic_policy import LeRobotSonicPolicyClient as C; c=C(\"<WORKSTATION_IP>\", 5571, 30000); t=time.monotonic(); a,m=c.get_action({\"lerobot\":{\"state\":np.zeros(28,np.float32),\"image\":np.zeros((480,640,3),np.uint8),\"task\":\"probe\"}}); print(\"warm\", round(time.monotonic()-t,2), \"s\", m)"'
  ```

  Prints `warm <seconds> {...}`; run it until the time is well under 10 s.
- `T2`: prints `policy_host: <WORKSTATION_IP>`, then the harness.
- `k`: waits for T2 to subscribe, sends `k` once, prints status: `controller_running: true`,
  `policy_ready: true`. Never twice; `k` is a toggle.
- `rec`: telemetry recorder.
- `T3`: the loop. "Policy prewarm unavailable": the harness has no policy (wrong `policy_host`,
  policy server not ready) or no camera frames (T0). "upright_check: APIConnectionError": the loop
  cannot reach the VLM at its `--vlm-base-url`: a pane opened before `WORKSTATION_IP` was set still
  carries the old address; the vlm container is not serving yet (`curl -s http://127.0.0.1:8301/v1/models`
  on the workstation lists `qwen3-vl-4b` when it is); or the workstation's firewall blocks 8301 and
  5571 from the robot network (`curl -s --max-time 5 http://<WORKSTATION_IP>:8301/v1/models` from 222;
  `sudo ufw allow from 192.168.0.0/24 to any port 8301 proto tcp`, same for 5571).
- `T4`: verdict console; silent until a trial ends.

The harness prewarms the policy exactly once: at its first camera frame it sends one request with a
10 s deadline, and `policy_ready` is whatever that request returned, for the life of that harness.
So anything wrong at that moment (no T0, wrong `policy_host`, server still loading, cold server,
firewall) leaves `policy_ready: false` even after the cause is fixed. Fix the cause, warm the server
(above), restart T2, then `k` once. A policy server restart invalidates the harness the same way.

## 7. Shutdown

```
docker compose -f compose.demo.yaml exec dimos dimos mcp call stop_arm   # wait for its tool update in the log pane
# 222 panes: Ctrl-C T3, rec, T2, T1, T0; 223 panes: Ctrl-C greeter, then the controller in ctrl
./demo.sh down
```

Only the policy server: `docker compose -f compose.demo.yaml --profile workstation stop policy`
(`up -d --no-build policy` brings it back).

If anything on pc2 is restarted mid-demo, the planner script needs pc2's port 5556 free, which the
tunnel holds: `docker compose -f compose.demo.yaml stop tunnel dimos`, then the planner pane again.

## 8. Known issues (demo of 2026-10-29)

- 223: gestures and voice were interrupted mid-execution. Both cross the network between the
  workstation and pc2 on every gesture: the ticks go through the ssh tunnel (`-R 5556`), the
  feedback comes back through it (`-L 5557`), pc2's guard releases to the planner when the
  reference goes stale, and each voice line is an ssh command to pc2. A link that stalls for more
  than about 100 ms cuts the motion; a stalled ssh cuts the line. Not fixed. Next time, measure
  first (`ping -i 0.2 192.168.0.223` during a gesture; `ss -ti dst 192.168.0.223` for retransmits;
  is the workstation on Wi-Fi?), then either put the workstation on the wired robot network or
  move the tick source onto pc2 so the link carries commands only.
- 222: unstable from the visual gap between the demo room and the training data. Out of scope here.
