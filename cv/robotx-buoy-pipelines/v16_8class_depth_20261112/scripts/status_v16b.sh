#!/usr/bin/env bash
set -euo pipefail

WORK=/home/robotx/buoy_actual/v16_8class_depth_20261112
RUN=v16b_balanced_from_v16best_1280_100ep_20261112
PID_FILE="$WORK/train_v16b_balanced.pid"
LOG="$WORK/logs/train_v16b_balanced.log"
BEST="$WORK/runs/$RUN/weights/best.pt"

echo "=== v16b training ==="
if [[ -s "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Status: RUNNING (PID $(cat "$PID_FILE"))"
else
    echo "Status: STOPPED/FINISHED"
fi

python3 - "$LOG" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(errors="ignore").replace("\r", "\n") if path.exists() else ""
ansi = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
rows = [ansi.sub("", line) for line in text.splitlines()
        if "/100" in line and "GPU_mem" not in line]
print("Latest:", rows[-1].strip() if rows else "waiting for first epoch")
for key in ("EarlyStopping:", "epochs completed", "Results saved to"):
    hits = [ansi.sub("", line).strip() for line in text.splitlines() if key in line]
    if hits:
        print(hits[-1])
PY

if [[ -f "$BEST" ]]; then
    stat -c 'Best model: %n (%s bytes, modified %y)' "$BEST"
else
    echo "Best model: not created yet"
fi

if [[ -f "$WORK/BALANCED_VALIDATION_COMPLETE" ]]; then
    echo "Validation: COMPLETE"
    tail -n 35 "$WORK/logs/validate_v16b_balanced.log" 2>/dev/null || true
else
    echo "Validation: PENDING"
fi

echo "=== GPU ==="
nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu \
    --format=csv,noheader 2>/dev/null || true

