#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/plugins/navigation_beacons/build"

cmake -S "${ROOT_DIR}/plugins/navigation_beacons" -B "${BUILD_DIR}" -DBUILD_TESTING=ON
cmake --build "${BUILD_DIR}" -j "${BUILD_JOBS:-2}"
