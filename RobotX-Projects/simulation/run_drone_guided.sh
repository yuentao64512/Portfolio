#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL="${SCRIPT_DIR}/scripts/drone_guided_control.py"

usage() {
  echo "Usage:"
  echo "  $0 takeoff ALTITUDE_M"
  echo "  $0 move FORWARD_MPS RIGHT_MPS DOWN_MPS DURATION_S"
  echo "  $0 hold DURATION_S"
  echo "  $0 land"
  echo "  $0 disarm"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

case "$1" in
  takeoff)
    altitude="${2:-5}"
    exec python3 "${CONTROL}" --connection tcp:127.0.0.1:5760 takeoff "${altitude}" --arm
    ;;
  move)
    if [[ $# -ne 5 ]]; then
      usage
      exit 2
    fi
    exec python3 "${CONTROL}" --connection tcp:127.0.0.1:5760 move "$2" "$3" "$4" "$5"
    ;;
  hold)
    duration="${2:-5}"
    exec python3 "${CONTROL}" --connection tcp:127.0.0.1:5760 hold "${duration}"
    ;;
  land)
    exec python3 "${CONTROL}" --connection tcp:127.0.0.1:5760 land
    ;;
  disarm)
    exec python3 "${CONTROL}" --connection tcp:127.0.0.1:5760 disarm
    ;;
  *)
    usage
    exit 2
    ;;
esac
