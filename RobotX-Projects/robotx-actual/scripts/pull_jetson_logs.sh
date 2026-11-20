#!/usr/bin/env bash
# Copy the Jetsons' logs and recordings to this Mac (WP44, session A, 2026-11-12: "lets copy them to mac now" - after every run).
# Read-only on the Jetsons: rsync pulls, never deletes, never writes there. Resumable: a second run sends only what is new.
#
#   scripts/pull_jetson_logs.sh [--ts | --lan] [--nx | --agx] [--dry-run] [--date YYYY-MM-DD]
#     NX  (ORIN_SSH): ~/Desktop/dm3/MASTER_SCRIPTS/logs/ + usv_ui/logs/   -> logs/jetsons/<date>/nx/{logs,usv_ui_logs}/
#     AGX (AGX_SSH) : ~/Desktop/dm3/recordings/ + MASTER_SCRIPTS/logs/     -> logs/jetsons/<date>/agx/{recordings,tree_logs}/
#   Then a check per source: files and bytes on the Jetson = on the Mac (a host still writing shows "+n new" - run it again).
#   The addresses come from network.env / network.<profile>.env via scripts/_netenv.sh (never typed here). logs/ is never archived.
set -euo pipefail
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KIT"
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac; done
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT" 2>/dev/null || true
DO_NX=1; DO_AGX=1; DRY=0; DAY="$(date +%Y-%m-%d)"
while [ $# -gt 0 ]; do
  case "$1" in
    --ts|--lan) shift ;;
    --nx) DO_AGX=0; shift ;;
    --agx) DO_NX=0; shift ;;
    --dry-run) DRY=1; shift ;;
    --date) DAY="$2"; shift 2 ;;
    *) echo "unknown option $1 (--ts|--lan --nx|--agx --dry-run --date D)" >&2; exit 2 ;;
  esac
done
REMOTE_KIT="${REMOTE_KIT:-Desktop/dm3/MASTER_SCRIPTS}"        # relative to the Jetson user's home (CLAUDE.md: the same tree everywhere)
DEST="$KIT/logs/jetsons/$DAY"
SSHO=(-o BatchMode=yes -o ConnectTimeout=5)
RC=0

count_remote() {   # host dir -> "files bytes"
  ssh "${SSHO[@]}" "$1" "d=\$HOME/$2; [ -d \"\$d\" ] || { echo '0 0'; exit 0; }; cd \"\$d\" && printf '%s %s\n' \"\$(find . -type f | wc -l)\" \"\$(find . -type f -printf '%s\n' | awk '{s+=\$1} END {print s+0}')\""
}
count_local() {
  [ -d "$1" ] || { echo "0 0"; return; }
  ( cd "$1" && printf '%s %s\n' "$(find . -type f | wc -l | tr -d ' ')" "$(find . -type f -exec stat -f %z {} + 2>/dev/null | awk '{s+=$1} END {print s+0}')" )
}
pull() {   # label host remote_dir local_dir
  local label="$1" host="$2" rdir="$3" ldir="$4"
  if [ "$DRY" -eq 1 ]; then echo "would copy $label: $host:~/$rdir/ -> ${ldir#$KIT/}/"; return; fi
  mkdir -p "$ldir"
  local t0; t0=$(date +%s)
  if rsync -a --partial -e "ssh ${SSHO[*]}" "$host:$rdir/" "$ldir/"; then
    read -r rf rb < <(count_remote "$host" "$rdir")
    read -r lf lb < <(count_local "$ldir")
    local mb=$(( lb / 1000000 )) dt=$(( $(date +%s) - t0 ))
    if [ "$rf" = "$lf" ] && [ "$rb" = "$lb" ]; then
      echo "✓ $label: $lf files, $mb MB, identical to the Jetson (${dt}s) -> ${ldir#$KIT/}/"
    else
      echo "⚠ $label: Jetson $rf files / $rb B, Mac $lf / $lb - still being written? run it again" ; RC=1
    fi
  else
    echo "✗ $label: rsync failed ($host unreachable?)"; RC=1
  fi
}
if [ "$DO_NX" -eq 1 ]; then
  [ -n "${ORIN_SSH:-}" ] || { echo "✗ NX: ORIN_SSH not set (network.env)"; RC=1; }
  if [ -n "${ORIN_SSH:-}" ]; then
    pull "NX logs" "$ORIN_SSH" "$REMOTE_KIT/logs" "$DEST/nx/logs"
    pull "NX usv_ui logs" "$ORIN_SSH" "$REMOTE_KIT/usv_ui/logs" "$DEST/nx/usv_ui_logs"
  fi
fi
if [ "$DO_AGX" -eq 1 ]; then
  [ -n "${AGX_SSH:-}" ] || { echo "✗ AGX: AGX_SSH not set (network.env)"; RC=1; }
  if [ -n "${AGX_SSH:-}" ]; then
    pull "AGX recordings" "$AGX_SSH" "Desktop/dm3/recordings" "$DEST/agx/recordings"
    pull "AGX tree logs" "$AGX_SSH" "$REMOTE_KIT/logs" "$DEST/agx/tree_logs"
  fi
fi
[ "$DRY" -eq 1 ] || echo "Mac free: $(df -h "$KIT" | tail -1 | awk '{print $4}')"
exit $RC
