#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=remote-common.sh
source "$SCRIPT_DIR/remote-common.sh"

if [[ $# -ne 0 ]]; then
  fail "usage: $0"
fi

load_remote_env
validate_remote_path REMOTE_WORK_DIR "$REMOTE_WORK_DIR"

work_dir=$(quote_remote "$REMOTE_WORK_DIR")
args=()
mapfile -t args < <(ssh_base_args)

printf 'Opening remote shell on %s in %s\n' "$(ssh_target)" "$REMOTE_WORK_DIR"
ssh -t "${args[@]}" "$(ssh_target)" "cd $work_dir && exec \${SHELL:-/bin/sh} -l"
