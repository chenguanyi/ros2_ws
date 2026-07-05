#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=remote-common.sh
source "$SCRIPT_DIR/remote-common.sh"

if [[ $# -ne 0 ]]; then
  fail "usage: $0"
fi

"$SCRIPT_DIR/rsync-to-remote.sh"

load_remote_env
: "${REMOTE_BUILD_CMD:?REMOTE_BUILD_CMD is required}"
validate_remote_path REMOTE_WORK_DIR "$REMOTE_WORK_DIR"

printf 'Running remote build on %s in %s\n' "$(ssh_target)" "$REMOTE_WORK_DIR"
remote_cd_and_run "$REMOTE_BUILD_CMD"
