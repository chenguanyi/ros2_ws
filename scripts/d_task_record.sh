#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEFAULT_TOPICS=(
  /tf
  /tf_static
  /scan
  /map
  /map_updates
  /target_position
  /target_velocity
  /velocity_map
  /active_controller
  /height
  /height_raw
  /laser_array/ground_height
  /laser_array/min_range
  /laser_array/obstacle_height
  /laser_array/raw_percentile
  /laser_array/obstacle_below
  /visual_takeover_active
  /fine_data
  /warehouse_inventory/barcode_value
  /warehouse_inventory/barcode_candidate
  /warehouse_inventory/route
  /warehouse_inventory/scan_result
  /warehouse_inventory/all_results
  /warehouse_inventory/query_result
  /warehouse_inventory/target_id
  /d_task/mode
  /d_task/route
  /d_task/qr_id
  /d_task/status
  /mission_complete
  /is_st_ready
  /mission_step
  /arm_command
  /magnet_command
  /signal_command
  /rosout
)

FULL_TOPICS=(
  /camera/image_raw
  /warehouse_inventory/side_camera/image_raw
  /warehouse_inventory/barcode_overlay
)

usage() {
  cat <<EOF
Usage:
  scripts/d_task_record.sh [OPTIONS]

Options:
  -o, --output DIR   Output rosbag directory.
                    Default: bags/d_task_YYYYmmdd_HHMMSS
      --full         Also record image topics.
  -h, --help         Show this help.

Default topics:
$(printf '  %s\n' "${DEFAULT_TOPICS[@]}")

Extra topics with --full:
$(printf '  %s\n' "${FULL_TOPICS[@]}")
EOF
}

FULL=0
OUT="${WORKSPACE_ROOT}/bags/d_task_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output)
      if [[ $# -lt 2 ]]; then
        echo "error: $1 requires a directory argument" >&2
        exit 2
      fi
      OUT="$2"
      shift 2
      ;;
    --full)
      FULL=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      echo >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

TOPICS=("${DEFAULT_TOPICS[@]}")
if [[ "${FULL}" -eq 1 ]]; then
  TOPICS+=("${FULL_TOPICS[@]}")
fi

mkdir -p "$(dirname "${OUT}")"

echo "Recording D task rosbag:"
echo "  output: ${OUT}"
echo "  full:   ${FULL}"
echo "  topics: ${#TOPICS[@]}"
printf '    %s\n' "${TOPICS[@]}"

exec ros2 bag record -o "${OUT}" "${TOPICS[@]}"
