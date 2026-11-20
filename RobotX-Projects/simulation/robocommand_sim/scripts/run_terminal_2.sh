#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec 9>"${PKG}/.terminal_2.lock"
if ! flock -n 9; then
  echo "Mock RoboCommand / OCS is already running." >&2
  exit 2
fi
export PYTHONPATH="${PKG}:${PYTHONPATH:-}"
exec python3 -m robocommand.mock_robocommand --config "${PKG}/config/task1_world.yaml" "$@"

