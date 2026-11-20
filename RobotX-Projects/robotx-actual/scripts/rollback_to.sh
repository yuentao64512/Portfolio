#!/usr/bin/env bash
# Roll the Mac's tree back to an archived version (v0.1.64, D0 of the 2026-11-08 overnight brief: "a fork, so we could go back").
#   scripts/rollback_to.sh VERSION            e.g. v0.1.63 | 0.1.63 | v0.1.63_pre_uav  (= _archive/MASTER_SCRIPTS_<VERSION>.tgz)
#   scripts/rollback_to.sh --dry-run VERSION  list what would move / stay, change nothing
#   scripts/rollback_to.sh -h | --help        this text
# It refuses while anything of ours runs (status.sh must be clean). Everything the archives carry is MOVED (never deleted) into
# _archive/rolled_<stamp>_from_v<current>/, then the version's tgz is extracted fresh, so no file added after VERSION survives.
# Kept in place: .venv logs _archive drone network*.env (and usv_ui/co.json, usv_ui/points.json, handbook/*.pdf, pathing_sim/*.pdf -
# what the archives exclude - copied back from the rolled tree).
# Then: .venv/bin/python -m pip install -e . (re-registers the console scripts), __version__, make quick, a DEVLOG block.
# The Jetsons are not touched: their way back is GUIDE §1's loop with the previous tgz.
set -uo pipefail
case "${1:-}" in -h|--help|"") sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
DRY=0; [ "$1" = "--dry-run" ] && { DRY=1; shift; }
V="${1:?VERSION}"; V="v${V#v}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT" || exit 2
TGZ="_archive/MASTER_SCRIPTS_${V}.tgz"
[ -f "$TGZ" ] || { echo "REFUSED: no $TGZ"; ls _archive/MASTER_SCRIPTS_v*.tgz 2>/dev/null | tail -5; exit 2; }
[ -d optiprimex ] && [ -f pyproject.toml ] || { echo "REFUSED: $ROOT is not the tree"; exit 2; }
CUR="$(sed -n 's/^__version__ = "\(.*\)"/\1/p' optiprimex/__init__.py 2>/dev/null)"; CUR="${CUR:-unknown}"
STAMP="$(date +%Y%m%d_%H%M%S)"; DEST="_archive/rolled_${STAMP}_from_v${CUR}"
KEEP=" .venv logs _archive drone network.env network.lan.env network.ts.env "
MOVE=(); STAY=()
for e in $(ls -A); do
  case "$KEEP" in *" $e "*) STAY+=("$e"); continue ;; esac
  case "$e" in network*.env) STAY+=("$e"); continue ;; esac
  MOVE+=("$e")
done
echo "rollback: v$CUR -> $V from $TGZ (sha256 $(sha256sum "$TGZ" | cut -c1-16)...)"
echo "  move to $DEST/: ${MOVE[*]}"
echo "  stay:  ${STAY[*]} (+ usv_ui/co.json usv_ui/points.json handbook/*.pdf pathing_sim/*.pdf copied back)"
[ $DRY -eq 1 ] && { echo "dry run: nothing changed"; exit 0; }
STATUS_CLOCK=0 scripts/status.sh >/dev/null 2>&1 || { echo "REFUSED: something of ours is running (scripts/status.sh) - stop it first"; exit 3; }
TMP="$(mktemp -d "$ROOT/_archive/.extract_XXXXXX")" || exit 4
COPYFILE_DISABLE=1 tar -xzf "$TGZ" -C "$TMP" || { echo "REFUSED: $TGZ does not extract - nothing moved"; rm -rf "$TMP"; exit 4; }
[ -f "$TMP/MASTER_SCRIPTS/optiprimex/__init__.py" ] || { echo "REFUSED: $TGZ has no MASTER_SCRIPTS/ tree - nothing moved"; rm -rf "$TMP"; exit 4; }
mkdir -p "$DEST" && mv "${MOVE[@]}" "$DEST/" || { echo "FAILED moving the tree aside - see $DEST"; exit 5; }
( shopt -s dotglob; mv "$TMP/MASTER_SCRIPTS/"* . ) && rmdir "$TMP/MASTER_SCRIPTS" "$TMP" || { echo "FAILED placing $V - the rolled tree is in $DEST"; exit 5; }
for f in usv_ui/co.json usv_ui/points.json "$DEST"/handbook/*.pdf "$DEST"/pathing_sim/*.pdf; do f="${f#$DEST/}"; [ -f "$DEST/$f" ] && [ -d "$(dirname "$f")" ] && cp -p "$DEST/$f" "$f"; done   # what the archives exclude
echo "-- pip install -e ."; .venv/bin/python -m pip install -e . -q 2>&1 | tail -3
NEW="$(.venv/bin/python -c 'import optiprimex; print(optiprimex.__version__)' 2>&1)"; echo "__version__ = $NEW"
echo "-- make quick"; QL="$(make quick 2>&1 | tail -1)"; echo "$QL"
{ printf '\n---\n\n## rolled back to %s on %s (scripts/rollback_to.sh)\n\nThe tree was v%s; it is in `%s/` (moved, not deleted). ' "$V" "$STAMP" "$CUR" "$DEST"
  printf 'Extracted fresh from `%s`; kept .venv, logs, network*.env, _archive, drone, usv_ui/co.json + points.json. __version__ = %s.\n' "$TGZ" "$NEW"
  printf '`make quick`: %s\n' "$QL"
  printf 'The rolled versions'"'"' DEVLOG blocks live in the moved copy (`%s/DEVLOG.md`).\n' "$DEST"; } >> DEVLOG.md
echo "done: $V in place; v$CUR in $DEST/"
