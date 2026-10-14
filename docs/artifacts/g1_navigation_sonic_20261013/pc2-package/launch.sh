#!/usr/bin/env bash
# pc2 SONIC navigation launcher.
#   launch.sh --check-only
#       Passive: hashes, fingerprints, clocks and the DDS-reader readiness probe.
#       Starts no controller and sends no command.
#   launch.sh --commission operator-ready-YYYYMMDD native|graph
#       Operator-only, today's date, after fresh standing/clear-area/stop readiness:
#       native  - starts real SONIC motor control in this terminal (same command as the
#                 pc2-proven guarded launcher, plus --guarded-planner).
#       graph   - starts the pc2 navigation graph (sensor sender, relay, execution owner).
set -euo pipefail

verify_root=/home/unitree/dimos-navigation-verify-20261009
task="${SONIC_NAV_TASK_ROOT:-$verify_root/sonic-navigation-20261013}"
sysfs_root="${SONIC_SYSFS_ROOT:-/sys}"  # overrides exist only for the offline launcher test
package="$task/package"
source_dir="$task/native/gear_sonic_deploy"
image=dimos-g1-sonic-navigation:20261013
container=dimos-g1-sonic-navigation-graph-20261013

clocks_pinned() {  # unchanged rule from the pc2-proven launcher: the `sudo jetson_clocks` state
    local ok=true cpus=0 dir min max gpu="$sysfs_root/class/devfreq/17000000.gpu"
    for dir in "$sysfs_root"/devices/system/cpu/cpu[0-9]*/cpufreq; do
        [[ -r $dir/scaling_min_freq ]] || continue
        cpus=$((cpus + 1))
        min=$(<"$dir/scaling_min_freq"); max=$(<"$dir/cpuinfo_max_freq")
        [[ $min == "$max" ]] || { echo "CPU clock not pinned: ${dir%/cpufreq} min=$min hw_max=$max" >&2; ok=false; }
    done
    ((cpus > 0)) || { echo 'No CPU frequency controls found' >&2; ok=false; }
    if [[ -r $gpu/min_freq && -r $gpu/max_freq ]]; then
        min=$(<"$gpu/min_freq"); max=$(<"$gpu/max_freq")
        [[ $min == "$max" ]] || { echo "GPU clock not pinned: min=$min max=$max" >&2; ok=false; }
    else
        echo "GPU frequency controls not found: $gpu" >&2; ok=false
    fi
    [[ $ok == true ]]
}

verify_package() {
    (cd "$package" && sha256sum --check --quiet manifest.sha256)
    sha256sum --check --quiet "$verify_root/environment-before.sha256"
    sha256sum --check --quiet "$package/shared-controllers.sha256"
    sha256sum --check --quiet "$task/provision/controller.sha256"
    (cd "$source_dir" && sha256sum --check --quiet "$package/models.sha256")
}

check_only() {  # $1: blocking issues tolerated (space separated)
    verify_package
    local out="$task/checks/$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$out"
    # Readers only. Host PID/network namespaces let it see existing controllers and ports.
    docker run --rm --init --network host --pid host --user 1000:1000 --cap-drop ALL \
        --security-opt no-new-privileges --cpus 2 --memory 2g \
        --mount "type=bind,source=$out,target=/verification" \
        --env XDG_STATE_HOME=/verification/state --env XDG_CACHE_HOME=/tmp/cache \
        "$image" python /opt/sonic-navigation/package/demo_pc2_navigation_readiness.py \
        --config /opt/sonic-navigation/package/config/probe.json \
        --output /verification/readiness.json || true
    if clocks_pinned; then echo 'Clocks pinned.'; fi
    python3 - "$out/readiness.json" "$1" <<'EOF'
import json, sys
report = json.load(open(sys.argv[1]))
blocking = [b for b in report["blocking"] if b not in sys.argv[2].split()]
print(json.dumps({"report": sys.argv[1], "blocking": report["blocking"], "tolerated": sys.argv[2].split()}))
sys.exit(1 if blocking else 0)
EOF
}

case "${1:-}" in
    --check-only)
        (($# == 1)) || { echo 'Usage: launch.sh --check-only' >&2; exit 2; }
        check_only ""
        echo 'Passive check complete; nothing was started.'
        ;;
    --commission)
        (($# == 3)) || { echo 'Usage: launch.sh --commission operator-ready-YYYYMMDD native|graph' >&2; exit 2; }
        [[ "$2" == "operator-ready-$(date +%Y%m%d)" ]] \
            || { echo "Commissioning needs today's token: operator-ready-$(date +%Y%m%d)" >&2; exit 2; }
        ;;
    *) echo 'Usage: launch.sh --check-only | --commission operator-ready-YYYYMMDD native|graph' >&2; exit 2 ;;
esac
[[ "$1" == --commission ]] || exit 0

case "$3" in
    native)
        # The probe must find no SONIC process and free task ports; the navigation peer
        # enforces the extrinsic before any goal, so only that issue is tolerated here.
        check_only "extrinsic_uncommissioned"
        clocks_pinned || { echo 'Refusing to start: run `sudo jetson_clocks` first.' >&2; exit 1; }
        if pgrep -f '(^|/)g1_deploy_onnx_ref([[:space:]]|$)' > /dev/null; then
            echo 'Another SONIC controller is running; use the operator shutdown first.' >&2; exit 1
        fi
        logs_dir="$task/logs/native-$(date +%Y%m%d-%H%M%S)"
        sdk_lib="$source_dir/thirdparty/unitree_sdk2/thirdparty/lib/aarch64"
        export LD_LIBRARY_PATH="$sdk_lib:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/opt/onnxruntime/lib"
        export LD_PRELOAD="$sdk_lib/libddsc.so:$sdk_lib/libddscxx.so"
        command=(
            "$source_dir/target/release/g1_deploy_onnx_ref" enP8p1s0
            "$source_dir/policy/sonic_v1_1/model_decoder.onnx" "$source_dir/reference/example/"
            --obs-config "$source_dir/policy/sonic_v1_1/observation_config.yaml"
            --encoder-file "$source_dir/policy/sonic_v1_1/model_encoder.onnx"
            --planner-file "$source_dir/planner/target_vel/V2/planner_sonic.onnx"
            --input-type zmq_manager --output-type all
            --zmq-host 127.0.0.1 --zmq-port 5556
            --reference-timeout-ms 200 --guarded-planner
            --disable-dex3-hands
            --enable-csv-logs --logs-dir "$logs_dir"
        )
        mkdir -p "$logs_dir"
        printf '%q ' "${command[@]}" > "$logs_dir/command.txt"
        echo "Starting real SONIC motor control. Native logs: $logs_dir"
        cd "$source_dir"
        exec "${command[@]}"
        ;;
    graph)
        verify_package
        if docker container inspect "$container" > /dev/null 2>&1; then
            echo "Graph container already exists: $container" >&2; exit 1
        fi
        out="$task/runs/graph-$(date +%Y%m%d-%H%M%S)"
        mkdir -p "$out/journal"
        echo "Starting the pc2 navigation graph; journal and logs: $out"
        exec docker run --rm --init --name "$container" --network host --user 1000:1000 \
            --cap-drop ALL --security-opt no-new-privileges --cpus 2 --memory 4g \
            --ulimit memlock=67108864:67108864 \
            --mount "type=bind,source=$out,target=/verification" \
            --env XDG_STATE_HOME=/verification/state --env XDG_CACHE_HOME=/tmp/cache \
            --env XDG_CONFIG_HOME=/tmp/config --env HOME=/tmp \
            "$image" /opt/sonic-navigation/package/run-pc2-graph.sh
        ;;
    *) echo 'Component must be native or graph' >&2; exit 2 ;;
esac
