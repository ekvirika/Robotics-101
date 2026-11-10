#!/usr/bin/env bash
# Two daemons on this one computer, standing in for two machines:
#   robot       runs sensor and display   (deploy: machine: robot)
#   gpu-server  runs compute              (deploy: labels: gpu: "true")
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mkdir -p out
cleanup() {
  dora stop --name distributed >/dev/null 2>&1 || true
  dora down >/dev/null 2>&1 || true
  pkill -f "dora daemon --machine-id" 2>/dev/null || true
  pkill -f "dora coordinator" 2>/dev/null || true
}
trap cleanup EXIT

rm -f out/dataflow.dora-session.yaml
dora coordinator > out/coordinator.log 2>&1 &
sleep 2
dora daemon --machine-id robot > out/daemon-robot.log 2>&1 &
dora daemon --machine-id gpu-server --labels gpu=true \
  --local-listen-port 53292 > out/daemon-gpu.log 2>&1 &
sleep 4
dora start dataflow.yml --name distributed --detach
sleep "${RUN_SECONDS:-15}"
echo
echo "== display output, compute ran in another daemon:"
dora logs distributed --node display | grep "display (" | tail -5
