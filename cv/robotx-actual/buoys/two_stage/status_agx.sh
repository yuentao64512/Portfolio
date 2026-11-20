#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Host: $(hostname)"
echo "Runtime: /home/optiprime-agxorin/actual_buoy/.venv/bin/python"
if [[ -f "$ROOT/tensorrt_export.pid" ]]; then
  pid="$(cat "$ROOT/tensorrt_export.pid")"
  if kill -0 "$pid" 2>/dev/null; then
    echo "TensorRT export: running (PID $pid)"
  else
    echo "TensorRT export: stopped (PID $pid)"
  fi
else
  echo "TensorRT export: not started"
fi

for file in model_a_best.pt model_b_best.pt model_a_best.engine model_b_best.engine; do
  if [[ -f "$ROOT/models/$file" ]]; then
    stat -c '%n %s bytes' "$ROOT/models/$file"
  else
    echo "missing: $ROOT/models/$file"
  fi
done

if [[ -f "$ROOT/TENSORRT_EXPORT_COMPLETE" ]]; then
  echo "TensorRT engines: COMPLETE"
else
  echo "TensorRT engines: not complete"
fi

if [[ -f "$ROOT/DEPLOYMENT_VERIFIED" ]]; then
  echo "TensorRT smoke test: VERIFIED"
elif [[ -f "$ROOT/tensorrt_verify.pid" ]] && kill -0 "$(cat "$ROOT/tensorrt_verify.pid")" 2>/dev/null; then
  echo "TensorRT smoke test: waiting in background"
else
  echo "TensorRT smoke test: not verified"
fi

echo "--- export log ---"
tail -n 15 "$ROOT/tensorrt_export.log" 2>/dev/null || true

echo "--- smoke-test log ---"
tail -n 10 "$ROOT/tensorrt_smoke_test.log" 2>/dev/null || true
tail -n 5 "$ROOT/tensorrt_verify_background.log" 2>/dev/null || true
