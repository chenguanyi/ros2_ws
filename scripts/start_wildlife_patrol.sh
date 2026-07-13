#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  scripts/start_wildlife_patrol.sh [launch args...]

Examples:
  scripts/start_wildlife_patrol.sh
  scripts/start_wildlife_patrol.sh display:=false
  scripts/start_wildlife_patrol.sh animal_confirm_frames:=3

The script opens a terminal automatically when a graphical desktop is available.
Set WILDLIFE_PATROL_NO_TERMINAL=1 to force foreground/no-window mode.

This script is suitable for a desktop autostart entry.
It starts the wildlife patrol launch and forwards all arguments to ROS 2.
EOF
}

open_terminal() {
  local command_line=""
  local argument
  printf -v command_line '%q ' "${SCRIPT_DIR}/start_wildlife_patrol.sh"
  for argument in "$@"; do
    printf -v command_line '%s%q ' "${command_line}" "${argument}"
  done
  command_line="WILDLIFE_PATROL_TERMINAL_CHILD=1 ${command_line}; rc=\$?; echo; echo \"wildlife patrol exited with code \${rc}\"; read -r -p \"Press Enter to close this terminal...\""

  if command -v terminator >/dev/null 2>&1; then
    local terminator_command
    printf -v terminator_command '%q' "${command_line}"
    exec terminator --title="Wildlife Patrol" --command="bash -lc ${terminator_command}"
  elif command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal -- bash -lc "${command_line}"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    exec xfce4-terminal --command="bash -lc '${command_line}'"
  elif command -v lxterminal >/dev/null 2>&1; then
    exec lxterminal --command="bash -lc '${command_line}'"
  elif command -v xterm >/dev/null 2>&1; then
    exec xterm -hold -e bash -lc "${command_line}"
  fi

  echo "warning: no supported terminal emulator found; running in the current shell" >&2
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
  --terminal)
    shift
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" && "${WILDLIFE_PATROL_TERMINAL_CHILD:-0}" != "1" ]]; then
      open_terminal "$@"
    fi
    ;;
esac

if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" &&
  "${WILDLIFE_PATROL_TERMINAL_CHILD:-0}" != "1" &&
  "${WILDLIFE_PATROL_NO_TERMINAL:-0}" != "1" ]]; then
  open_terminal "$@"
fi

cd "${WORKSPACE_ROOT}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_ROOT}/install/setup.bash"
else
  echo "error: workspace is not built: ${WORKSPACE_ROOT}/install/setup.bash" >&2
  exit 1
fi
set -u

export ROS_LOG_DIR="${WORKSPACE_ROOT}/mylog/wildlife_patrol_boot"
mkdir -p "${ROS_LOG_DIR}"

echo "Starting wildlife patrol from ${WORKSPACE_ROOT}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "ROS_LOG_DIR=${ROS_LOG_DIR}"

exec ros2 launch my_launch wildlife_patrol.launch.py "$@"
