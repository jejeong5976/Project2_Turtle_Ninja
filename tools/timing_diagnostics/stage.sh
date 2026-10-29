#!/usr/bin/env bash
# Source preparation only. Does not build, launch nodes, open serial or run Git.
set -euo pipefail
[[ $# == 3 ]] || { echo 'Usage: stage.sh WHEEL_SOURCE MAPPING_PACKAGE NEW_WORKSPACE' >&2; exit 2; }
trace_script_dir=$(cd -- "$(dirname -- "$0")" && pwd)
trace_wheel=$(realpath -- "$1")
trace_mapping=$(realpath -- "$2")
trace_workspace=$(realpath -m -- "$3")
[[ ! -e "$trace_workspace" ]] || { echo 'Workspace must be new; refusing overwrite.' >&2; exit 2; }
for trace_package in hardware swerve_controller interfaces; do
  [[ -f "$trace_wheel/ros2_ws/src/$trace_package/package.xml" ]] || exit 2
done
[[ -f "$trace_mapping/package.xml" && -f "$trace_wheel/firmware/nt_opencr/nt_packet.h" ]] || exit 2
mkdir -p "$trace_workspace/src"
for trace_package in hardware swerve_controller interfaces; do
  cp -a "$trace_wheel/ros2_ws/src/$trace_package" "$trace_workspace/src/"
done
cp -a "$trace_mapping" "$trace_workspace/src/coin_d4_mapping_driver"
cp -a "$trace_wheel/firmware" "$trace_workspace/"
(cd "$trace_workspace/src" && sha256sum --check "$trace_script_dir/BASE.sha256")
patch --batch --fuzz=0 -d "$trace_workspace/src" -p1 < "$trace_script_dir/instrument.patch"
for trace_package in hardware swerve_controller coin_d4_mapping_driver; do
  cp "$trace_script_dir/nt_timing_trace.hpp" "$trace_workspace/src/$trace_package/include/"
done
cp -a "$trace_script_dir" "$trace_workspace/timing_tools"
printf 'Prepared diagnostic sources: %s\nNo hardware was started.\n' "$trace_workspace"
