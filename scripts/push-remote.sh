#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

echo '==> Dry-run: checking local-to-remote sync changes'
"$SCRIPT_DIR/rsync-to-remote.sh" --dry-run

echo
echo '==> Push: syncing local source to remote mirror'
"$SCRIPT_DIR/rsync-to-remote.sh"

echo
echo 'Remote push completed.'
