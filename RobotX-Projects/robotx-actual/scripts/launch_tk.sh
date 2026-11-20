#!/usr/bin/env bash
# scripts/launch_tk.sh - the launcher as a tkinter window (v0.1.83.3). A SEPARATE script from scripts/launch.sh (Dmitrii: "make sure
# that we use different launcher.sh, I will play with old one now"): it runs its OWN launcher backend (optiprimex.tools.launcher, the
# same code, the same plan / launch / stop) on :8093 (LAUNCH_TK_PORT), never the web page's :8094, then opens the tk window on it.
# Both backends see what runs (the launch record + the probe), so neither launches on top of the other.
#   scripts/launch_tk.sh               the backend (if :8093 does not answer yet) + the window
#   scripts/launch_tk.sh --dry-run     a dry backend: LAUNCH / STOP ALL only show what they would run
#   scripts/launch_tk.sh --dark        the dark palette
# Closing the window leaves the backend and the stacks running (like closing the web page); the backend: logs/launch/launcher_tk_backend.log,
# stop it with: pkill -f 'optiprimex.tools.launcher --port 8093'.
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$KIT"
PY="$KIT/.venv/bin/python"; [ -x "$PY" ] || PY=python3
PORT="${LAUNCH_TK_PORT:-8093}"
[ "$PORT" = 8094 ] && { echo "launch_tk.sh: 8094 is the web launcher's port - pick another LAUNCH_TK_PORT" >&2; exit 2; }
DRY=(); THEME=(--theme light)
for a in "$@"; do case "$a" in --dry-run) DRY=(--dry-run) ;; --dark) THEME=(--theme dark) ;; --light) THEME=(--theme light) ;;
  *) echo "unknown: $a (--dry-run, --dark)" >&2; exit 2 ;; esac; done
"$PY" -c "import tkinter" 2>/dev/null || { echo "launch_tk.sh: this Python has no tkinter ($PY)" >&2; exit 3; }
up() { curl -fsS -m 1 "http://127.0.0.1:$PORT/presets.json" >/dev/null 2>&1; }
if ! up; then
  mkdir -p "$KIT/logs/launch"
  # v0.1.84.1: its own session (start_new_session) - a Ctrl-C in this terminal or closing it no longer stops the backend (the
  # launcher re-arms SIGINT on purpose, and nohup ... & stayed in this terminal's process group: "page closed" at 14:42)
  BPID=$("$PY" - "$KIT/logs/launch/launcher_tk_backend.log" "$PY" -m optiprimex.tools.launcher --port "$PORT" ${DRY[@]+"${DRY[@]}"} <<'PYEOF'
import subprocess, sys
log = open(sys.argv[1], "ab")
p = subprocess.Popen(sys.argv[2:], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
print(p.pid)
PYEOF
)
  for i in $(seq 1 30); do up && break; sleep 0.2; done
  up || { echo "launch_tk.sh: the backend did not come up on :$PORT - see logs/launch/launcher_tk_backend.log" >&2; exit 4; }
  echo "[launch_tk] backend started on :$PORT${DRY[@]+ (dry run)} (pid $BPID, its own session: Ctrl-C here leaves it running)"
else
  echo "[launch_tk] backend already on :$PORT"
fi
exec "$PY" -m optiprimex.tools.launcher_tk --url "http://127.0.0.1:$PORT" "${THEME[@]}"
