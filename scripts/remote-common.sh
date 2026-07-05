#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
REMOTE_ENV_FILE=${REMOTE_ENV_FILE:-"$SCRIPT_DIR/remote.env"}

fail() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

quote_remote() {
  printf "%q" "$1"
}

load_remote_env() {
  if [[ ! -f "$REMOTE_ENV_FILE" ]]; then
    fail "missing $REMOTE_ENV_FILE; copy scripts/remote.env.example to scripts/remote.env and edit it"
  fi

  # shellcheck source=/dev/null
  source "$REMOTE_ENV_FILE"

  : "${REMOTE_USER:?REMOTE_USER is required}"
  : "${REMOTE_HOST:?REMOTE_HOST is required}"
  : "${REMOTE_PORT:?REMOTE_PORT is required}"
  : "${REMOTE_SRC_DIR:?REMOTE_SRC_DIR is required}"
  : "${REMOTE_WORK_DIR:?REMOTE_WORK_DIR is required}"
}

validate_remote_path() {
  local name=$1
  local value=$2

  [[ -n "$value" ]] || fail "$name must not be empty"
  [[ "$value" = /* ]] || fail "$name must be an absolute path: $value"

  case "$value" in
    /|/home|/home/|/root|/root/|~|~/*)
      fail "$name is too broad for rsync --delete safety: $value"
      ;;
  esac

  if [[ "$value" =~ ^/home/[^/]+/?$ ]]; then
    fail "$name is too broad for rsync --delete safety: $value"
  fi
}

ssh_target() {
  printf '%s@%s' "$REMOTE_USER" "$REMOTE_HOST"
}

ssh_base_args() {
  printf '%s\n' -p "$REMOTE_PORT"
  if [[ -n "${SSH_EXTRA_OPTS:-}" ]]; then
    # Intentionally split user-provided extra options.
    # shellcheck disable=SC2206
    local extra=( $SSH_EXTRA_OPTS )
    printf '%s\n' "${extra[@]}"
  fi
}

run_remote() {
  local command=$1
  local args=()
  mapfile -t args < <(ssh_base_args)
  ssh "${args[@]}" "$(ssh_target)" "$command"
}

remote_cd_and_run() {
  local command=$1
  local work_dir
  work_dir=$(quote_remote "$REMOTE_WORK_DIR")
  run_remote "cd $work_dir && $command"
}
