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
# installed on the machine (README, Install), and here: .venv with ".[agent,voice,dashboard]" and a .env
# with the key (agent.py says which variables): next to pyproject.toml, or ENV_FILE=<path>.
#
# The SSH connection carries three ports, all on 127.0.0.1 at both ends:
#   COORD  this computer -> brev   the dora coordinator, which runs on brev
#   PEER_B this computer -> brev   brev's daemon (zenoh), which this computer's daemon dials
#   PEER_M brev -> this computer   this computer's daemon (zenoh), which brev's daemon dials
# Since zenoh 1.9 daemons don't relay, so each one names the other (--zenoh-connect), and
# each listens on its own port so a daemon never dials itself through the tunnel.

set -euo pipefail

COORD=6013
PEER_B=7447
PEER_M=7448
BREV_REPO=${BREV_REPO:-/workspace/g1}                   # where the README's Install clones it
BREV_EXEC=${BREV_EXEC:-}                                # e.g. "docker exec -i vscode" if not already inside
BREV_PATH='export PATH=$HOME/.local/bin:$PATH'          # where pip puts dora on the Isaac Launchable

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

on_brev() { ssh "$host" "$BREV_EXEC bash -lc '$BREV_PATH; $*'"; }

stop() {
    echo "stopping: dataflow, daemons, coordinator, tunnel"
    "$dora" destroy --coordinator-port "$COORD" >/dev/null 2>&1 || true
    on_brev "pkill -f \"dora daemon --machine-id brev\"; pkill -f \"dora coordinator\"" >/dev/null 2>&1 || true
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

on_brev "cd $BREV_REPO && (nohup dora coordinator --port $COORD >/tmp/dora-coordinator.log 2>&1 &) && sleep 2 && \
    (nohup dora daemon --machine-id brev --coordinator-addr 127.0.0.1 --coordinator-port $COORD \
        --zenoh-no-multicast --zenoh-listen 127.0.0.1:$PEER_B --zenoh-connect tcp/127.0.0.1:$PEER_M \
        >/tmp/dora-daemon.log 2>&1 &)"
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
