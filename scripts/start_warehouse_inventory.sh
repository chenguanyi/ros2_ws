#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<EOF
Usage:
  scripts/start_warehouse_inventory.sh [launch args...]

Examples:
  scripts/start_warehouse_inventory.sh
  scripts/start_warehouse_inventory.sh mission_mode:=target
  scripts/start_warehouse_inventory.sh use_viewer:=false use_rviz:=true

Default launch args:
  mission_mode:=inventory
EOF
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

cd "${WORKSPACE_ROOT}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
fi

exec ros2 launch my_launch warehouse_inventory.launch.py mission_mode:=inventory "$@"
