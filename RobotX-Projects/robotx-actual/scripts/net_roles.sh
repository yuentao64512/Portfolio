#!/bin/bash
# v0.1.94 (Dmitrii 2026-11-14): the Mac's wired adapters take their roles from what they are plugged into (scripts/net_roles.py).
#   scripts/net_roles.sh status            the roles now (no addresses printed)
#   sudo scripts/net_roles.sh dhcp         once: every wired LAN service on DHCP
#   sudo scripts/net_roles.sh install      once: the root LaunchDaemon (re-run after editing network*.env)
#   sudo scripts/net_roles.sh apply        one pass by hand · uninstall
exec /usr/bin/python3 "$(cd "$(dirname "$0")" && pwd)/net_roles.py" "$@"
