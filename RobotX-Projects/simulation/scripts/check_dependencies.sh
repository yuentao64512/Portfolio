#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
missing=0

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    echo "OK command: $1"
  else
    echo "MISSING command: $1"
    missing=1
  fi
}

check_file() {
  if [[ -e "$1" ]]; then
    echo "OK file: ${1#"${SCRIPT_DIR}/"}"
  else
    echo "MISSING file: ${1#"${SCRIPT_DIR}/"}"
    missing=1
  fi
}

check_command gz
check_command python3
check_command cmake
check_command colcon

python3 - <<'PY'
try:
    from gz.msgs10.double_pb2 import Double  # noqa: F401
    from gz.msgs10.odometry_pb2 import Odometry  # noqa: F401
    from gz.transport13 import Node  # noqa: F401
except ImportError as exc:
    print(f"MISSING Python Gazebo transport binding: {exc}")
    raise SystemExit(1)
print("OK Python Gazebo transport binding")
PY

for file in \
  "${SCRIPT_DIR}/models/usv_four_thruster/model.sdf" \
  "${SCRIPT_DIR}/models/usv_two_thruster/model.sdf" \
  "${SCRIPT_DIR}/worlds/usv_four_thruster.world" \
  "${SCRIPT_DIR}/worlds/usv_two_thruster.world" \
  "${SCRIPT_DIR}/models/blueboat/meshes/blueboat_hull_collision.stl" \
  "${SCRIPT_DIR}/lib/libNavigationBeacons.so" \
  "${SCRIPT_DIR}/asv_wave_sim_ws/install/lib/libgz-waves1-hydrodynamics-system.so"; do
  check_file "${file}"
done

if [[ "${missing}" -ne 0 ]]; then
  echo "Dependency check failed. See the MISSING lines above." >&2
  exit 1
fi

echo "All local runtime dependencies are present."
