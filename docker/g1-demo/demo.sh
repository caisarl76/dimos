#!/usr/bin/env bash
# The demo in one command: `docker/g1-demo/demo.sh up` starts the GPU servers of the workstation
# stack (compose.demo.yaml: VoLo's VLM on GPU 0 and policy on GPU 1) and opens tmux session
# "g1-demo": a window per robot, a titled pane per terminal of the runbooks. Each pane has its
# command typed but NOT run: press Enter in the order of the pane titles. 223's tunnel + blueprint
# come up from the planner pane: start-idle-planner.py needs pc2's port 5556 to itself, which the
# tunnel takes once it runs (so `up` stops both when they are still up from an earlier run).
#   window 223   ctrl     pc2: sudo jetson_clocks (asks the password), clock check, guarded controller
#                planner  pc2: idle planner (preview, then start); then tunnel + blueprint up
#                log      dimos log -f (tool updates, voice)
#                test     one gesture + line over MCP (the natural ARDY clip version)
#                greeter  the 223 show with the natural clips (Ctrl-C ends it; drop --natural for
#                         the keyframe gestures)
#   window 222   T0       pc2_222: RealSense ego camera server on :5555 (the harness needs its frames
#                         before it can prewarm the policy; without it `loop` faults with
#                         "Policy prewarm unavailable" and status shows observation_age_s null)
#                T1       pc2_222: deploy (SONIC v1.1 under policy/sonic_v1_1, where its TRT caches
#                         are; deploy.sh's policy/release default has no models there) with
#                         --harness-planner-hold, wait for the stance
#                T2       pc2_222: native harness (restart it after any policy restart)
#                k        pc2_222: send `k` ONCE (toggle!) as soon as T2 subscribes to the key port,
#                         then status: controller_running + policy_ready
#                rec      pc2_222: telemetry recorder
#                T3       pc2_222: loop runner (gates)
#                T4       pc2_222: verdict console (s / f <n> / r)
#   window ws    compose logs: policy must say "ready on port 5571" (~4 min; ~15 min from a cold HDD)
# `demo.sh down` closes the session and the stack; stop_arm first (see compose.yaml's header).
# `up` refuses while the session exists (killing it would kill the controllers in its panes).
# Hosts: PC2_SSH_HOST (223, default pc2), PC2_222_SSH_HOST (default pc2_222), e.g. user@ip when there
# is no ~/.ssh/config; from the environment or the .env next to this script, resolved by compose
# itself (so the panes and the containers agree). The 222 paths are the runbook's folders on PC2.
set -euo pipefail
cd "$(dirname "$0")"
S=g1-demo
DC="docker compose -f compose.demo.yaml"

COMPOSE_ENV=$($DC config --environment)  # compose's interpolation environment: .env + the environment
var() {  # var <KEY> <default>: like ${KEY:-default} in a compose file
  local v
  v=$(printf '%s\n' "$COMPOSE_ENV" | sed -n "s/^$1=//p")
  printf '%s' "${v:-$2}"
}
P223=$(var PC2_SSH_HOST pc2)
P222=$(var PC2_222_SSH_HOST pc2_222)
LOOP='~/GR00T-WholeBodyControl-loop-20261026'
PY='~/GR00T-WholeBodyControl-harness-20261022/venv/bin/python'
CLI="PYTHONPATH=\$PWD $PY -m vlm_orchestrator.harness.g1.cli --profile session-profile.yaml --endpoint ipc:///tmp/volo-g1-harness.sock"
NAME=demo-$(date +%m%d-%H%M)

pane() {  # pane <window> <title> <command>: a titled pane in the window, the command typed, not run
  if tmux list-windows -t "$S" -F '#W' | grep -qx "$1"; then
    tmux split-window -t "$S:$1" -c "$PWD"
  else
    tmux new-window -t "$S" -n "$1" -c "$PWD"
    tmux set-option -w -t "$S:$1" pane-border-status top
    tmux set-option -w -t "$S:$1" pane-border-format " #{pane_title} "
  fi
  tmux select-pane -t "$S:$1" -T "$2"
  tmux send-keys -t "$S:$1" "$3"
  tmux select-layout -t "$S:$1" tiled
}

case ${1:-up} in
  up)
    if tmux has-session -t "$S" 2>/dev/null; then
      echo "tmux session $S exists: attach to it (tmux attach -t $S) or run $0 down first" >&2
      exit 1
    fi
    $DC --profile workstation up -d --no-build policy vlm
    $DC stop tunnel dimos
    tmux new-session -d -s "$S" -n ws -c "$PWD" -x 220 -y 60
    tmux send-keys -t "$S:ws" "$DC logs -f policy vlm" Enter
    pane 223 ctrl "ssh -t $P223 'sudo jetson_clocks && cd /home/unitree/dimos-sonic-guard-pc2-223-20261014 && bash launch-guarded.sh --check-clocks && bash launch-guarded.sh --run'"
    pane 223 planner "ssh -t $P223 'cd /home/unitree/dimos-sonic-guard-pc2-223-20261014 && /home/unitree/dimos/.venv/bin/python start-idle-planner.py && /home/unitree/dimos/.venv/bin/python start-idle-planner.py --start-idle-planner' && $DC up -d --no-build tunnel dimos"
    pane 223 log "$DC exec dimos dimos log -f"
    pane 223 test "$DC exec dimos dimos mcp call perform_gesture --arg name=invite_natural --arg line=06_invite"
    pane 223 greeter "$DC run --rm greeter --natural --pause-s 3"
    pane 222 T0 "ssh -t $P222 'cd ~/GR00T-WholeBodyControl && .venv_camera/bin/python -u -m gear_sonic.camera.composed_camera --ego-view-camera realsense --realsense-enable-depth --port 5555'"
    pane 222 T1 "ssh -t $P222 'cd ~/GR00T-WholeBodyControl-deploy-20261027/gear_sonic_deploy && just run g1_deploy_onnx_ref enP8p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost --harness-planner-hold'"
    pane 222 T2 "ssh -t $P222 'cd $LOOP && PYTHONPATH=\$PWD $PY -u -m gear_sonic.scripts.run_vla_inference --harness-endpoint ipc:///tmp/volo-g1-harness.sock --harness-profile session-profile.yaml 2>&1 | tee harness-\$(date +%H%M%S).log'"
    # XPUB: publish k only once T2's key subscriber (created late in its init) is attached; a plain PUB drops it.
    pane 222 k "ssh -t $P222 'cd $LOOP && $PY -c \"import zmq,sys,time; s=zmq.Context().socket(zmq.XPUB); s.bind(\\\"tcp://127.0.0.1:5580\\\"); print(\\\"waiting for T2 to subscribe to the key port...\\\", flush=True); sys.exit(\\\"no key subscriber in 120 s: is T2 running?\\\") if not s.poll(120000) else s.recv(); s.send_string(\\\"k\\\"); time.sleep(0.5); print(\\\"k sent\\\")\" && sleep 3 && $CLI status'"
    pane 222 rec "ssh -t $P222 'cd $LOOP && PYTHONPATH=\$PWD $PY -u g1_record_telemetry.py telemetry-$NAME.npz --seconds 1800 2>&1 | tee recorder-$NAME.log'"
    pane 222 T3 "ssh -t $P222 'cd $LOOP && $CLI loop --session-dir sessions/$NAME --trials 10 --vlm-base-url http://192.168.0.62:8301/v1 --vlm-model qwen3-vl-4b'"
    pane 222 T4 "ssh -t $P222 'cd $LOOP && $CLI verdict --session-dir sessions/$NAME'"
    tmux select-window -t "$S:223"
    [[ -n ${TMUX:-} ]] && tmux switch-client -t "$S" || tmux attach -t "$S"
    ;;
  down)
    tmux kill-session -t "$S" 2>/dev/null || true
    $DC --profile workstation down
    ;;
  *) echo "usage: $0 [up|down]" >&2; exit 2 ;;
esac
