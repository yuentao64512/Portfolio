#!/usr/bin/env bash
# ON THE AGX (v0.1.34.2): copy a published CV_TRAIN model into the kit - cv/models/<name>/ - and check it against the model's own
# sha256.txt. CV_TRAIN (~/Desktop/dm3/CV_TRAIN, the third agent's folder) is read only; the kit never writes there. The model folders
# are not in the archive (the .engine is built for this AGX).
#   ./sync_cvtrain.sh NAME [--force]        e.g. ./sync_cvtrain.sh cvt-v1-l      CVTRAIN_DIR=~/Desktop/dm3/CV_TRAIN (default)
#   then:  buoys_multi/run.sh --headless --model ../cv/models/NAME      (its input size, conf, iou; only its `buoy` class)
# usage:  -h | --help prints this
set -euo pipefail
case "${1:-}" in -h|--help|"") sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; [ -n "${1:-}" ] && exit 0 || exit 2 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="$1"; FORCE="${2:-}"
SRC="${CVTRAIN_DIR:-$HOME/Desktop/dm3/CV_TRAIN}/models/$NAME"
DST="$HERE/models/$NAME"
[ -d "$SRC" ] || { echo "no model folder $SRC (published by CV_TRAIN?)" >&2; exit 1; }
[ -f "$SRC/sha256.txt" ] || { echo "$SRC has no sha256.txt - not a published model" >&2; exit 1; }
if [ -d "$DST" ] && [ "$FORCE" != "--force" ]; then echo "already there: $DST (--force replaces it)"; else
  mkdir -p "$DST"; cp -f "$SRC"/* "$DST"/; chmod -R a-w "$DST"/* 2>/dev/null || true; fi
cd "$DST" && sha256sum -c sha256.txt && echo "OK: $DST matches its sha256.txt" || { echo "MISMATCH: do not use $DST" >&2; exit 1; }
