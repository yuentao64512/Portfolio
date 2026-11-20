#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec 9>"${PKG}/.logger.lock"
if ! flock -n 9; then
  echo "RoboCommand recorder is already running." >&2
  exit 2
fi
export PYTHONPATH="${PKG}:${PYTHONPATH:-}"
exec python3 -m robocommand.record_run "$@"
