#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON=/home/optiprime-agxorin/actual_buoy/.venv/bin/python

if [[ ! -x "$PYTHON" ]]; then
  echo "Python runtime not found: $PYTHON" >&2
  exit 1
fi

exec "$PYTHON" "$ROOT/predict_buoys_zed.py" "$@"
