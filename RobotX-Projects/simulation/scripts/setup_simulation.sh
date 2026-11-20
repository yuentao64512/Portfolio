#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WAVE_REPO="${ROOT_DIR}/asv_wave_sim_ws/src/asv_wave_sim"
WAVE_PATCH="${ROOT_DIR}/patches/asv_wave_sim-no-gnuplot.patch"

git -C "${ROOT_DIR}" submodule update --init --recursive

if git -C "${WAVE_REPO}" apply --unidiff-zero --check "${WAVE_PATCH}" 2>/dev/null; then
  git -C "${WAVE_REPO}" apply --unidiff-zero "${WAVE_PATCH}"
elif git -C "${WAVE_REPO}" apply --unidiff-zero --reverse --check "${WAVE_PATCH}" 2>/dev/null; then
  echo "Wave build patch is already applied."
else
  echo "Cannot apply ${WAVE_PATCH}; inspect the wave submodule state." >&2
  exit 1
fi

if [[ ! -e "${ROOT_DIR}/asv_wave_sim_ws/install/lib/libgz-waves1-hydrodynamics-system.so" ]]; then
  (
    cd "${ROOT_DIR}/asv_wave_sim_ws"
    colcon build --symlink-install --merge-install --cmake-args \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DBUILD_TESTING=OFF \
      -DCMAKE_CXX_STANDARD=17
  )
else
  echo "Wave plugins are already built."
fi

"${ROOT_DIR}/scripts/build_navigation_beacons.sh"
ctest --test-dir "${ROOT_DIR}/plugins/navigation_beacons/build" --output-on-failure
"${ROOT_DIR}/scripts/check_dependencies.sh"
