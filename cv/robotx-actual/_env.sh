#!/usr/bin/env bash
# Sourced by cv/*/run.sh: reads ../network.env (command line wins) and picks the interpreter for the CV programs:
#   1. $CV_VENV/bin/python            network.env CV_VENV, our own venv once built (cv/.venv by the recipe in cv/README.md)
#   2. cv/.venv/bin/python            same, default location
#   3. $AGX_BUOY_VENV/bin/python      the students' venv (pyzed, torch, ultralytics, tensorrt via system site-packages) - read-only use
#   4. python3                        last resort (will lack pyzed)
CV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIT_ROOT="$(cd "$CV_DIR/.." && pwd)"
. "$KIT_ROOT/scripts/_netenv.sh"; load_network_env "$KIT_ROOT"      # network.env + network.<NET_PROFILE>.env
cv_python() {
  if [ -n "${CV_VENV:-}" ] && [ -x "$CV_VENV/bin/python" ]; then echo "$CV_VENV/bin/python"; return; fi
  if [ -x "$CV_DIR/.venv/bin/python" ]; then echo "$CV_DIR/.venv/bin/python"; return; fi
  if [ -n "${AGX_BUOY_VENV:-}" ] && [ -x "$AGX_BUOY_VENV/bin/python" ]; then echo "$AGX_BUOY_VENV/bin/python"; return; fi
  command -v python3
}
