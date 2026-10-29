#!/usr/bin/env bash
# User-operated finite segments using our guarded console, not continuous key teleop.
set -euo pipefail
[[ $# == 1 && $1 == --supervised ]] || {
  echo 'Usage: ./scripts/teleop.sh --supervised (user-operated hardware action)' >&2; exit 2;
}
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
capture_console="${MAPPING_CONSOLE_BIN:-$HOME/jetson-mapping-console-v2/build/run_mapping_console}"
[[ -x "$capture_console" ]] || { echo "Build console v2 first: $capture_console" >&2; exit 1; }
exec "$capture_console" --supervised
