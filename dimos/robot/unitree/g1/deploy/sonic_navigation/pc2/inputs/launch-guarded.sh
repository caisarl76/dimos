#!/usr/bin/env bash
# Default: display the command. --run is an explicit operator motor-control launch.
set -euo pipefail
stage_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$stage_dir/gear_sonic_deploy"
run_controller=false
check_clocks_only=false
reference_host=127.0.0.1
sysfs_root="${SONIC_SYSFS_ROOT:-/sys}"  # override only for the offline launcher test

# Every known-good SONIC launch sourced scripts/setup_env.sh, which runs `sudo jetson_clocks`.
# The 2026-10-14 waist-grinding run did not: policy took ~5 ms instead of ~1.8 ms.
clocks_pinned() {
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

while (($#)); do
    case "$1" in
        --run) run_controller=true; shift ;;
        --dry-run) run_controller=false; shift ;;
        --check-clocks) check_clocks_only=true; shift ;;
        --zmq-host)
            if (($# < 2)) || [[ -z "$2" ]]; then
                echo '--zmq-host requires a host' >&2; exit 2
            fi
            reference_host="$2"; shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
if [[ "$check_clocks_only" == true ]]; then
    if clocks_pinned; then echo 'Clocks pinned.'; exit 0; fi
    echo 'Clocks not pinned. Run: sudo jetson_clocks' >&2; exit 1
fi
logs_dir="$stage_dir/logs/$(date +%Y%m%d-%H%M%S)"
sdk_lib="$source_dir/thirdparty/unitree_sdk2/thirdparty/lib/aarch64"
export LD_LIBRARY_PATH="$sdk_lib:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/opt/onnxruntime/lib"
export LD_PRELOAD="$sdk_lib/libddsc.so:$sdk_lib/libddscxx.so"
command=(
    "$source_dir/target/release/g1_deploy_onnx_ref"
    enP8p1s0
    "$source_dir/policy/sonic_v1_1/model_decoder.onnx"
    "$source_dir/reference/example/"
    --obs-config "$source_dir/policy/sonic_v1_1/observation_config.yaml"
    --encoder-file "$source_dir/policy/sonic_v1_1/model_encoder.onnx"
    --planner-file "$source_dir/planner/target_vel/V2/planner_sonic.onnx"
    --input-type zmq_manager --output-type all
    --zmq-host "$reference_host" --zmq-port 5556
    --reference-timeout-ms 200
    --disable-dex3-hands
    --enable-csv-logs --logs-dir "$logs_dir"
)
printf 'Native controller command:\n'
printf '%q ' "${command[@]}"
printf '\n'
if [[ "$run_controller" != true ]]; then
    echo 'Dry run only. --run starts real motor control, including its initialization movement.'
    exit 0
fi
if pgrep -f '(^|/)g1_deploy_onnx_ref([[:space:]]|$)' >/dev/null; then
    echo 'Another SONIC controller is running. Use the normal operator shutdown before relaunching.' >&2
    exit 1
fi
if ! clocks_pinned; then
    echo 'Refusing to start: Jetson clocks are not pinned. Run `sudo jetson_clocks` (as scripts/setup_env.sh does), then relaunch.' >&2
    exit 1
fi
cd "$stage_dir"
sha256sum --check controller.sha256
mkdir -p "$logs_dir"
echo "CSV logs: $logs_dir"
cd "$source_dir"
exec "${command[@]}"
