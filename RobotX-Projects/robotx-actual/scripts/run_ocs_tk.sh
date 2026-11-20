#!/usr/bin/env bash
# The OCS operator console in tkinter (v0.1.N Phase 1, docs/TK_CONSOLE.md) - started BESIDE scripts/run_ocs.sh, which keeps serving
# the web page at :8096 (the fallback). The console is a client of that page's HTTP contract; it starts no broker and no tmux window.
#
#   scripts/run_ocs_tk.sh [--ts | --lan] [--theme light|dark] [--fast-hz 5]
#     --ts | --lan   the network profile (as run_ocs.sh); the console only needs OCS_WEB_PORT (network.env, default 8096)
#   Needs Tk 8.6 in the venv's python: brew install python-tk@3.10 (Apple's system Tk 8.5 is not usable).
#   stop: close the window (the pollers stop with it)
set -euo pipefail
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KIT"
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac; done
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT" 2>/dev/null || true
ARGS=()
for _a in "$@"; do case "$_a" in --ts|--lan) ;; *) ARGS+=("$_a") ;; esac; done
PY="${PY:-$KIT/.venv/bin/python}"
[ -x "$PY" ] || { echo "no venv at $PY - run 'make venv' first" >&2; exit 1; }
OCS_WEB_PORT="${OCS_WEB_PORT:-8096}"
"$PY" -c "import tkinter, sys; sys.exit(0 if tkinter.TkVersion >= 8.6 else 1)" 2>/dev/null \
  || { echo "no Tk 8.6 in $PY - brew install python-tk@3.10 (the venv's python3.10)" >&2; exit 1; }
# one console per Mac (the launcher's OCS button may call this again): a live pid in logs/ocs/ocs_tk.pid = already open, exit 0
PIDFILE="$KIT/logs/ocs/ocs_tk.pid"
mkdir -p "$KIT/logs/ocs"
if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE" 2>/dev/null)" 2>/dev/null && ps -p "$(cat "$PIDFILE")" -o command= | grep -q ocs_tk; then
  echo "the tk console is already open (pid $(cat "$PIDFILE"))"; exit 0
fi
echo $$ > "$PIDFILE"                                  # exec keeps this pid for the python process
curl -s -m 1 -o /dev/null "http://127.0.0.1:$OCS_WEB_PORT/meta.json" \
  || echo "note: nothing answers on :$OCS_WEB_PORT yet (scripts/run_ocs.sh) - the console waits and shows DOWN until it does" >&2
exec "$PY" -m optiprimex.tools.ocs_tk --ocs "http://127.0.0.1:$OCS_WEB_PORT" ${ARGS[@]+"${ARGS[@]}"}
