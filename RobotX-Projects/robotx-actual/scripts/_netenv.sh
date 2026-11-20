#!/usr/bin/env bash
# Sourced by the launchers (run_*.sh, stop_*.sh) and by cv/_env.sh: loads the network addresses and ports.
#
#   network.env            the base: ports, FCU link, paths, and NET_PROFILE=ts|lan (which address set is in use)
#   network.ts.env         the addresses on the Tailscale tailnet (lab, internet)         OCS_HOST BOAT_HOST AGX_HOST ORIN_SSH AGX_SSH
#   network.lan.env        the addresses on the pool switch (192.168.1.x, no internet)    the same five keys
#
# Precedence: a variable already set in the environment (the command line) > the overlay of the selected profile > network.env.
# The profile: NET_PROFILE in the environment (the launchers set it from --ts / --lan) > the NET_PROFILE line of network.env > ts.
# Usage in a script:   KIT_ROOT=/path/to/MASTER_SCRIPTS; . "$KIT_ROOT/scripts/_netenv.sh"; load_network_env "$KIT_ROOT"
load_network_env() {
  local root="$1" f k v line
  NETENV="${NETENV:-$root/network.env}"
  if [ -z "${NET_PROFILE:-}" ] && [ -f "$NETENV" ]; then
    NET_PROFILE="$(sed -n 's/^NET_PROFILE=[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$NETENV" | head -1)"
  fi
  NET_PROFILE="${NET_PROFILE:-ts}"
  export NET_PROFILE
  NETENV_OVERLAY="$(dirname "$NETENV")/network.$NET_PROFILE.env"
  for f in "$NETENV_OVERLAY" "$NETENV"; do                       # first file wins: overlay before base
    [ -f "$f" ] || continue
    while IFS= read -r line || [ -n "$line" ]; do
      line="${line%%#*}"; line="${line#"${line%%[![:space:]]*}"}"; line="${line%"${line##*[![:space:]]}"}"
      [ -z "$line" ] && continue
      k="${line%%=*}"; v="${line#*=}"; v="${v#"${v%%[![:space:]]*}"}"; v="${v%"${v##*[![:space:]]}"}"
      case "$k" in *[!A-Za-z0-9_]*|"") continue ;; esac
      if [ -z "${!k+x}" ]; then export "$k=$v"; fi
    done < "$f"
  done
  if [ ! -f "$NETENV_OVERLAY" ]; then
    echo "network: no $NETENV_OVERLAY (profile '$NET_PROFILE') - addresses come from $NETENV only" >&2
  fi
}

# v0.1.78.2: the venue rc-test's address for the simulator (run_sim.sh / run_bench.sh --judge venue), which loads no other address:
# only VENUE_RC_HOST / VENUE_RC_PORT / VENUE_RC_WEB, from the environment, else network.<profile>.env, else network.env.
load_venue_rc() {
  local root="$1" f k line prof base
  base="${NETENV:-$root/network.env}"
  prof="${NET_PROFILE:-$(sed -n 's/^NET_PROFILE=[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$base" 2>/dev/null | head -1)}"
  for f in "$(dirname "$base")/network.${prof:-ts}.env" "$base"; do
    [ -f "$f" ] || continue
    for k in VENUE_RC_HOST VENUE_RC_PORT VENUE_RC_WEB; do
      [ -n "${!k:-}" ] && continue
      line="$(sed -n "s/^$k=[[:space:]]*\([^[:space:]#]*\).*/\1/p" "$f" | head -1)"
      [ -n "$line" ] && export "$k=$line"
    done
  done
  [ -n "${VENUE_RC_HOST:-}" ] && [ -n "${VENUE_RC_PORT:-}" ]
}
