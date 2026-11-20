#!/usr/bin/env bash
# Tell the robot on Isaac Lab: run dataflow.yml across this computer (mac) and the Brev GPU
# machine (brev), over one SSH connection. Nothing is opened to the internet.
#
#   ./run_on_brev.sh <brev machine>            e.g. ./run_on_brev.sh isaac-g1
#   ROBOT_SAY="Put the blue bottle on the green mark" ./run_on_brev.sh isaac-g1
#   then, for typing or speaking, in a second terminal:  ./run_on_brev.sh chat
#
# dora resolves relative paths per daemon, not from this file, so the mac paths in
# dataflow.yml are made absolute in out/dataflow.run.yml before it starts.
#
# Before: `brev login` and `brev refresh` (adds the machine to ~/.ssh/config), the repository
# cloned to /workspace/ttr on the machine (BREV_REPO; run with PYTHONPATH, so the installed
# /workspace/g1 stays as it is), and here: .venv with ".[agent,voice,dashboard]" and a .env
# with the key (agent.py says which variables): next to pyproject.toml, or ENV_FILE=<path>.
#
# The SSH connection carries three ports, all on 127.0.0.1 at both ends:
#   COORD  this computer -> brev   the dora coordinator, which runs on brev (6213)
#   PEER_B this computer -> brev   brev's daemon (zenoh), which this computer's daemon dials
#   PEER_M brev -> this computer   this computer's daemon (zenoh), which brev's daemon dials
# Since zenoh 1.9 daemons don't relay, so each one names the other (--zenoh-connect), and
# each listens on its own port so a daemon never dials itself through the tunnel.

set -euo pipefail

COORD=6213  # not dora's default 6013: a graph of Tornike's can run beside this one
PEER_B=7547
PEER_M=7548
BREV_REPO=${BREV_REPO:-/workspace/ttr}                  # this branch, cloned next to the installed /workspace/g1
BREV_CONTAINER=${BREV_CONTAINER-vscode}                 # SSH lands on the host, Isaac Lab is in this container ("" = host)
BREV_ENV="export PATH=/root/.local/bin:\$PATH PYTHONPATH=$BREV_REPO/source:$BREV_REPO"  # dora; this branch before the installed one

here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
dora=${DORA:-dora}
env_file=${ENV_FILE:-$repo/.env}                       # the key; another project's .env works too

if [[ ${1:-} == chat ]]; then
    export DORA_ZENOH_CONNECT=tcp/127.0.0.1:$PEER_M     # this computer's daemon (no multicast)
    exec "$repo/.venv/bin/python" "$here/nodes/chat.py"
fi

host=${1:?usage: run_on_brev.sh <brev machine> | chat}
[[ -f $env_file ]] || { echo "no $env_file: the agent needs its key there" >&2; exit 1; }
mkdir -p "$here/out"

in_container() { [[ -n $BREV_CONTAINER ]] && echo "docker exec $1 $BREV_CONTAINER"; }
on_brev() { ssh "$host" "$(in_container -i) bash -lc '$BREV_ENV; $*'"; }
on_brev_detached() { ssh "$host" "$(in_container -d) bash -lc '$BREV_ENV; $*'"; }

stop() {
    echo "stopping: dataflow, daemons, coordinator, tunnel"
    "$dora" destroy --coordinator-port "$COORD" >/dev/null 2>&1 || true
    on_brev "pkill -f \"dora daemon --machine-id ttr\"; pkill -f \"dora coordinator --port $COORD\"" >/dev/null 2>&1 || true  # only ours
    [[ -n ${daemon:-} ]] && kill "$daemon" 2>/dev/null || true
    [[ -n ${tunnel:-} ]] && kill "$tunnel" 2>/dev/null || true
}
trap stop EXIT

ssh -N -o ExitOnForwardFailure=yes \
    -L "$COORD:127.0.0.1:$COORD" -L "$PEER_B:127.0.0.1:$PEER_B" -R "$PEER_M:127.0.0.1:$PEER_M" \
    "$host" &
tunnel=$!
sleep 3
kill -0 "$tunnel" 2>/dev/null || { echo "SSH tunnel to $host failed" >&2; exit 1; }

on_brev "git -C $BREV_REPO pull -q --ff-only && git -C $BREV_REPO log --oneline -1"
on_brev_detached "dora coordinator --port $COORD >/tmp/ttr-coordinator.log 2>&1"
sleep 2
on_brev_detached "cd $BREV_REPO && dora daemon --machine-id ttr --coordinator-addr 127.0.0.1 --coordinator-port $COORD \
    --zenoh-no-multicast --zenoh-listen 127.0.0.1:$PEER_B --zenoh-connect tcp/127.0.0.1:$PEER_M \
    >/tmp/ttr-daemon.log 2>&1"
sleep 3

set -a
# shellcheck disable=SC1091
source "$env_file"
set +a
"$dora" daemon --machine-id mac --coordinator-addr 127.0.0.1 --coordinator-port "$COORD" \
    --zenoh-no-multicast --zenoh-listen "127.0.0.1:$PEER_M" --zenoh-connect "tcp/127.0.0.1:$PEER_B" \
    >"$here/out/daemon-mac.log" 2>&1 &
daemon=$!
sleep 3

sed -e "s|\.\./\.\./\.venv/bin/python|$repo/.venv/bin/python|" -e "s|args: -u nodes/|args: -u $here/nodes/|" \
    "$here/dataflow.yml" >"$here/out/dataflow.run.yml"

cd "$here"
echo "starting; for the chat: ./run_on_brev.sh chat   picture: <Brev VS Code link>/viewer/"
"$dora" start out/dataflow.run.yml --name tell-the-robot --coordinator-port "$COORD" --attach
