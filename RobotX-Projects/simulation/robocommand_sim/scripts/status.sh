#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Gazebo services:"
gz service -l 2>/dev/null | grep -E '/world/usv_four_thruster/(create|set_pose)' || true
echo
echo "RoboCommand processes:"
pgrep -af 'robocommand\.(mock_robocommand|team_console|uav_perception|boat_agent|sim_boat_driver|light_ctl)' || true
echo
echo "Recent logs:"
for file in "${PKG}"/logs/*.log; do
  [[ -e "${file}" ]] || continue
  echo "--- ${file}"
  tail -n 5 "${file}"
done

