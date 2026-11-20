#!/usr/bin/env bash
# cv/cannon: the water-cannon aiming link - follows the mission's aim (local/usv/cannon/aim on the boat broker
# BOAT_HOST:BOAT_PORT of network.env; v0.1.57) and sends dock_cannon.v1 HMAC packets to the Pi (cannon_config.json: offsets,
# pi_ip/pi_port, key_file path). The contract: cv/cannon/README.md.
#   usage: cannon/run.sh --self-test | --synthetic-send | --dry-run | [--source aim|file] [--destination host:port] [--hz N]
#   --source aim (default) needs paho-mqtt; --source file (legacy: the docking status file) and --self-test are stdlib only
#   (the key file must exist on this host; it is read, never copied).
#          cannon/run.sh -h | --help              this text (cannon_sender.py --help for the full list)
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../_env.sh"
PY="$(cv_python)"
exec "$PY" "$HERE/cannon_sender.py" --config "$HERE/cannon_config.json" --mqtt "${BOAT_HOST:-127.0.0.1}:${BOAT_PORT:-1884}" "$@"
