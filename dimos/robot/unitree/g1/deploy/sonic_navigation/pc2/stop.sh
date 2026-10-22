#!/usr/bin/env bash
# Stop only this task's pc2 navigation graph container (exact name). Losing its
# execution publisher latches the native planner guard to IDLE within 250 ms.
# The native controller is never signalled here: stop it with the operator procedure
# (`O`/`o` in its terminal) while the robot is supported.
set -euo pipefail
container=dimos-g1-sonic-navigation-graph-20261013
if docker container inspect "$container" > /dev/null 2>&1; then
    docker stop --time 10 "$container"
    echo "Stopped $container."
else
    echo "No task graph container is running ($container)."
fi
if pgrep -f '(^|/)g1_deploy_onnx_ref([[:space:]]|$)' > /dev/null; then
    echo 'A SONIC controller is still running: use the operator shutdown in its terminal.'
fi
