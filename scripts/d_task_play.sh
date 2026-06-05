#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<EOF
Usage:
  scripts/d_task_play.sh <bag_path> [ros2 bag play extra args...]

Examples:
  scripts/d_task_play.sh bags/d_task_20260605_143000
  scripts/d_task_play.sh bags/d_task_20260605_143000 --rate 0.5
  scripts/d_task_play.sh bags/d_task_20260605_143000 --start-offset 10
EOF
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

case "$1" in
  -h|--help)
    usage
    exit 0
    ;;
esac

BAG_PATH="$1"
shift

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "error: bag path does not exist: ${BAG_PATH}" >&2
  exit 2
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

exec ros2 bag play "${BAG_PATH}" "$@"
