#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<EOF
Usage:
  scripts/d_task_play.sh [bag_path] [ros2 bag play extra args...]

Examples:
  scripts/d_task_play.sh
  scripts/d_task_play.sh --rate 0.5
  scripts/d_task_play.sh bags/d_task_20260605_143000
  scripts/d_task_play.sh bags/d_task_20260605_143000 --rate 0.5
  scripts/d_task_play.sh bags/d_task_20260605_143000 --start-offset 10
EOF
}

latest_bag_path() {
  local bag_root="${WORKSPACE_ROOT}/bags"
  local -a bags=()

  if [[ ! -d "${bag_root}" ]]; then
    return 1
  fi

  mapfile -t bags < <(
    find "${bag_root}" -maxdepth 1 -type d -name 'd_task_*' -printf '%T@\t%p\n' \
      | sort -nr \
      | sed -n $'1s/^[^\t]*\t//p'
  )

  [[ "${#bags[@]}" -gt 0 ]] || return 1
  printf '%s\n' "${bags[0]}"
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

if [[ $# -eq 0 || "${1}" == -* ]]; then
  if ! BAG_PATH="$(latest_bag_path)"; then
    echo "error: no d_task bags found under ${WORKSPACE_ROOT}/bags" >&2
    exit 2
  fi
else
  BAG_PATH="$1"
  shift
fi

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "error: bag path does not exist: ${BAG_PATH}" >&2
  exit 2
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
fi

echo "Playing D task rosbag: ${BAG_PATH}"
exec ros2 bag play "${BAG_PATH}" "$@"
