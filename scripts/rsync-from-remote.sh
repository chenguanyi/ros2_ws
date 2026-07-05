#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=remote-common.sh
source "$SCRIPT_DIR/remote-common.sh"

DRY_RUN=${DRY_RUN:-0}
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=1
  shift
fi

if [[ $# -ne 0 ]]; then
  fail "usage: $0 [--dry-run]"
fi

load_remote_env
validate_remote_path REMOTE_SRC_DIR "$REMOTE_SRC_DIR"

ssh_args=()
mapfile -t ssh_args < <(ssh_base_args)

rsync_args=(
  -az
  --human-readable
  --info=stats2,progress2
  -e "ssh ${ssh_args[*]}"
  --exclude '.git/'
  --exclude '.claude/'
  --exclude 'scripts/remote.env'
  --exclude 'build/'
  --exclude 'install/'
  --exclude 'log/'
  --exclude 'dist/'
  --exclude 'out/'
  --exclude 'target/'
  --exclude '.cache/'
  --exclude '.pytest_cache/'
  --exclude '__pycache__/'
  --exclude 'node_modules/'
)

if [[ "$DRY_RUN" == "1" ]]; then
  rsync_args+=(--dry-run --itemize-changes)
fi

if [[ -n "${RSYNC_EXTRA_EXCLUDES:-}" ]]; then
  # Intentionally split user-provided exclude patterns on whitespace.
  # shellcheck disable=SC2206
  extra_excludes=( $RSYNC_EXTRA_EXCLUDES )
  for pattern in "${extra_excludes[@]}"; do
    rsync_args+=(--exclude "$pattern")
  done
fi

printf 'Bootstrapping %s:%s/ -> %s/\n' "$(ssh_target)" "$REMOTE_SRC_DIR" "$PROJECT_ROOT"
printf 'No --delete is used in this direction; existing local-only files are kept.\n'
if [[ "$DRY_RUN" == "1" ]]; then
  printf 'DRY RUN: no files will be changed.\n'
fi

rsync "${rsync_args[@]}" "$(ssh_target):$REMOTE_SRC_DIR/" "$PROJECT_ROOT/"
