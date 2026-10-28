#!/usr/bin/env bash
# Two-host MuJoCo regression, isolated in a private user + network namespace.
# Usage: run_two_host.sh <run-name>   -> two-host-sim/<run-name>/ and <run-name>.runner.log
# Needs build/sonic-navigation-runtime (native controller + assets), the PointLIO and
# ray-map result/bin links, and a quiet host (one sim at a time).
set -uo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
groot=${GROOT:-/home/jihun/work/GR00T-WholeBodyControl}
out="$here/two-host-sim/$1"
mkdir -p "$(dirname "$out")"
HOST_NETNS=$(readlink /proc/self/ns/net)
export HOST_NETNS ROOT="$root" GROOT="$groot"
timeout --signal=TERM --kill-after=30s 900 unshare --user --map-root-user --net /bin/bash -c '
    set -euo pipefail
    ip link set lo up
    ulimit -l 65536
    cd "$ROOT"
    export PYTHONPATH="$ROOT:$GROOT"
    export PYTHONDONTWRITEBYTECODE=1
    exec .venv/bin/python -u docs/artifacts/g1_navigation_sonic_20261013/demo_navigation_sim.py --stage two-host --output "$1" --groot "$GROOT" --host-netns "$HOST_NETNS" --gpu 1 --reference-port 18556 --feedback-port 18557 --pacing absolute
' two-host "$out" > "${out}.runner.log" 2>&1
rc=$?
echo "exit $rc" >> "${out}.runner.log"
exit $rc
