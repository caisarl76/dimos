#!/usr/bin/env bash
# Container entry for the pc2 navigation graph. The graph is validated by its
# blueprint before any worker or socket starts; it never starts the native controller.
set -euo pipefail
package=/opt/sonic-navigation/package
python "$package/zenoh_router.py" 17111 &
router=$!
trap 'kill "$router" 2>/dev/null || true' EXIT
for _ in $(seq 50); do
    python -c 'import socket; socket.create_connection(("127.0.0.1", 17111), 0.1)' 2>/dev/null && break
    sleep 0.1
done
dimos run unitree-g1-sonic-navigation-pc2 --config "$package/config/pc2-graph.json"
