#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=remote-common.sh
source "$SCRIPT_DIR/remote-common.sh"

SYNC_FIRST=0
if [[ "${1:-}" == "--sync" ]]; then
  SYNC_FIRST=1
  shift
fi

if [[ $# -ne 0 ]]; then
  fail "usage: $0 [--sync]"
fi

if [[ "$SYNC_FIRST" == "1" ]]; then
  "$SCRIPT_DIR/rsync-to-remote.sh"
fi

load_remote_env
: "${REMOTE_RUN_CMD:?REMOTE_RUN_CMD is required}"
validate_remote_path REMOTE_WORK_DIR "$REMOTE_WORK_DIR"

printf 'Running remote command on %s in %s\n' "$(ssh_target)" "$REMOTE_WORK_DIR"
remote_cd_and_run "$REMOTE_RUN_CMD"
