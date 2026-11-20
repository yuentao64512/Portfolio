#!/usr/bin/env bash
# ON THE AGX: copy the model files from the students' folders into cv/*/models (they are not in the archive: 63 MB + 23 MB,
# and the TensorRT .engine is built for this exact Jetson). Sources from ../network.env (AGX_BUOY_SRC, AGX_DOCK_SRC).
# Copies only; prints sha256 so PROVENANCE.md can be checked. Re-runnable (cp -n keeps an existing copy; --force replaces).
# usage: ./sync_models.sh [--force]           -h | --help prints this
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_env.sh"
BUOY_SRC="${AGX_BUOY_SRC:-/home/optiprime-agxorin/actual_buoy}"
DOCK_SRC="${AGX_DOCK_SRC:-/home/optiprime-agxorin/docking_cv}"
CP="cp -n"; [ "${1:-}" = "--force" ] && CP="cp -f"
mkdir -p "$HERE/buoys/models" "$HERE/docking/models"
[ -f "$BUOY_SRC/best.pt" ] && $CP "$BUOY_SRC/best.pt" "$HERE/buoys/models/best.pt" || echo "NOTE: $BUOY_SRC/best.pt not found"
for f in dock_partial_best.pt dock_partial_best.onnx dock_partial_best.engine; do
  [ -f "$DOCK_SRC/models/$f" ] && $CP "$DOCK_SRC/models/$f" "$HERE/docking/models/$f" || echo "NOTE: $DOCK_SRC/models/$f not found"
done
echo "models in place:"; for f in "$HERE"/buoys/models/* "$HERE"/docking/models/*; do [ -f "$f" ] && printf "  %s  %s\n" "$(sha256sum "$f" | cut -c1-16)" "$f"; done
echo "expected (Mac copies 2026-10-29): 351a554489e9afac best.pt | 7a230babd962d33d .pt | 205b0816d123c148 .onnx | 8e85a12a13062126 .engine"
