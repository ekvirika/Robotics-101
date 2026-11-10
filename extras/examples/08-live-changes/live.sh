#!/usr/bin/env bash
# Starts the graph in managed mode, then changes it while it runs.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
step() { printf '\n\033[1m== %s\033[0m\n' "$*"; sleep "${PAUSE:-6}"; }
cleanup() {
  dora stop --name live >/dev/null 2>&1 || true
  dora down >/dev/null 2>&1 || true
}
trap cleanup EXIT

dora up
dora start dataflow.yml --name live --detach
step "filter v1 is running: only values >= 10 reach the display"
dora logs live --node display | grep "display:" | tail -4

dora node add --from-yaml logger.yml --dataflow live
step "added a logger that reads the raw sensor values, nothing else restarted"
dora logs live --node logger | grep "logger:" | tail -4

dora node replace filter --from-yaml filter-v2.yml --dataflow live
step "replaced the filter with v2 (>= 20) while the sensor and display kept running"
dora logs live --node display | grep "display:" | tail -6

dora node list --dataflow live
