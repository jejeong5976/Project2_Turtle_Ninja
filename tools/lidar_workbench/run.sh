#!/usr/bin/env bash
# The user runs this launcher. Default hardware is mock.
set -eo pipefail
[[ $# -ge 1 ]] || { echo "Usage: $0 WORKSPACE [workbench options]" >&2; exit 2; }
workbench_workspace="$(realpath -- "$1")"
shift
source /opt/ros/lyrical/setup.bash
source "$workbench_workspace/install/local_setup.bash"
exec "$workbench_workspace/install/lidar_workbench/lib/lidar_workbench/lidar_workbench" "$@"
