#!/usr/bin/env bash
# Runs ON THE MAC: bring the ZED recordings home from the AGX (v0.1.40, ROADMAP WP9b / §7c: the event's data pipeline).
# rsync over AGX_SSH (network*.env through scripts/_netenv.sh; --ts / --lan pick the profile) from the AGX's recordings folder into
# a staging folder OUTSIDE CV_TRAIN - CV_TRAIN imports from there itself (its own agent, its own PROVENANCE):
#   ~/Documents/BH1/RobotX/recordings/<YYYY-MM-DD>/<stamp>_<label>/{cam.svo2, frames.jsonl, session.json, SHA256SUMS}
# The AGX keeps its copy: free space there only after the copy is checked (the line to type is printed at the end).
#
# usage: scripts/agx_pull_clips.sh [--ts|--lan] [--dest DIR] [--session NAME] [--with-replays] [--force] [--dry-run]
#        scripts/agx_pull_clips.sh --list [--ts|--lan]     what is on the AGX (sessions, sizes, the recorder's state), nothing copied
#   --dest DIR      default REC_HOME/<today> (REC_HOME default ~/Documents/BH1/RobotX/recordings); never inside CV_TRAIN
#   --session NAME  only that session (repeatable); default every session
#   --with-replays  also the replay/ folders a --svo replay left beside a clip (default: skipped, they can be rebuilt)
#   --force         copy even while the recorder is RECORDING (the running clip arrives cut short)
#   --dry-run       print the commands, contact nothing, create nothing
#   remote folder: AGX_REC_DIR_REMOTE (default Desktop/dm3/recordings, relative to the AGX user's home)
set -uo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; esac; done
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT" 2>/dev/null
AGX="${AGX_SSH:-agxorin}"
REMOTE="${AGX_REC_DIR_REMOTE:-Desktop/dm3/recordings}"
REMOTE_KIT="${JETSON_KIT:-Desktop/dm3/MASTER_SCRIPTS}"; REMOTE_KIT="${REMOTE_KIT#\~/}"
DEST=""; SESS=(); DRY=0; LIST=0; FORCE=0; REPLAYS=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ts|--lan) shift ;;
    --dest) DEST="${2:-}"; shift 2 || { echo "--dest DIR" >&2; exit 2; } ;;
    --session) [[ "${2:-}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { echo "--session NAME (a folder name: letters, digits, _ -)" >&2; exit 2; }
               SESS+=("$2"); shift 2 ;;
    --with-replays) REPLAYS=1; shift ;;
    --force) FORCE=1; shift ;;
    --dry-run) DRY=1; shift ;;
    --list) LIST=1; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ -z "$DEST" ] && DEST="${REC_HOME:-$HOME/Documents/BH1/RobotX/recordings}/$(date +%Y-%m-%d)"
case "$DEST" in *CV_TRAIN*) echo "refused: --dest $DEST is inside CV_TRAIN - stage outside it (CV_TRAIN imports from the staging folder itself)" >&2; exit 2 ;; esac
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "$AGX")
STATUS_CMD="python3 $REMOTE_KIT/cv/buoys_multi/rec_core.py status $REMOTE"
RS=(rsync -a --partial --human-readable --stats -e "ssh -o BatchMode=yes -o ConnectTimeout=8")
[ $REPLAYS -eq 0 ] && RS+=(--exclude 'replay/')
SRC=()
if [ ${#SESS[@]} -eq 0 ]; then SRC=("$AGX:$REMOTE/"); else for s in "${SESS[@]}"; do SRC+=("$AGX:$REMOTE/$s"); done; fi
show() { local x; for x in "$@"; do case "$x" in *[[:space:]\;\&\*\|\'\"]*) printf "'%s' " "$x" ;; *) printf '%s ' "$x" ;; esac; done; echo; }
if [ $DRY -eq 1 ]; then
  echo "DRY RUN (network $NET_PROFILE, AGX $AGX): nothing contacted, nothing created"
  [ $LIST -eq 1 ] && { echo "would run: $(show "${SSH[@]}" "cd $REMOTE && du -sh */ ; $STATUS_CMD")"; exit 0; }
  echo "would check: $(show "${SSH[@]}" "$STATUS_CMD")"
  echo "would run:   mkdir -p $DEST && $(show "${RS[@]}" "${SRC[@]}" "$DEST/")"
  echo "then:        shasum -a 256 per session -> $DEST/<session>/SHA256SUMS; a line in $DEST/PULLED.txt"
  exit 0
fi
if [ $LIST -eq 1 ]; then "${SSH[@]}" "cd $REMOTE 2>/dev/null && du -sh */ 2>/dev/null; $STATUS_CMD"; exit $?; fi
ST="$("${SSH[@]}" "$STATUS_CMD" 2>&1)" || { echo "AGX $AGX not reachable or no recordings folder: $ST" >&2; exit 1; }
echo "$ST"
if echo "$ST" | grep -q ": RECORDING" && [ $FORCE -eq 0 ]; then
  echo "refused: the recorder is RECORDING - scripts/agx_record.sh stop on the AGX (or the page's RECORDER card) first, or --force" >&2; exit 1
fi
mkdir -p "$DEST"
"${RS[@]}" "${SRC[@]}" "$DEST/" || { echo "rsync failed (rerun: --partial resumes)" >&2; exit 1; }
GOT=()
for d in "$DEST"/*/; do
  [ -f "$d/session.json" ] || continue
  n="$(basename "$d")"; GOT+=("$n")
  ( cd "$d" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 shasum -a 256 > SHA256SUMS )
done
echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) from $AGX:$REMOTE (network $NET_PROFILE): ${GOT[*]:-nothing}" >> "$DEST/PULLED.txt"
du -sh "$DEST" 2>/dev/null
echo "staged in $DEST: ${#GOT[@]} session(s) with SHA256SUMS. CV_TRAIN imports from here (never copy into CV_TRAIN by hand)."
echo "free the AGX only after checking (open a session.json, the clip's size): ${SSH[*]} 'rm -rf $REMOTE/<session>'"
