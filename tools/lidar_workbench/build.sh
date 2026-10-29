#!/usr/bin/env bash
# Offline build only. Never starts ROS nodes, opens devices, or executes Git.
set -euo pipefail
[[ $# == 2 ]] || { echo "Usage: $0 VERIFIED_UPSTREAM NEW_WORKSPACE" >&2; exit 2; }
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
repository="$(realpath -- "$script_dir/../..")"
diagnostics="$repository/tools/lidar_diagnostics"
upstream="$(realpath -- "$1")"
workspace="$(realpath -m -- "$2")"
[[ ! -e "$workspace" ]] || { echo "Refusing existing workspace; choose a new path" >&2; exit 2; }
(cd "$upstream" && sha256sum --check "$diagnostics/source.sha256")
package="$workspace/src/lidar_workbench"
mkdir -p "$package/vendor"
cp "$script_dir/"*.cpp "$script_dir/"*.hpp "$script_dir/CMakeLists.txt" "$script_dir/package.xml" \
  "$script_dir/record_qos.yaml" "$script_dir/workbench.rviz" "$package/"
cp -a "$upstream/include" "$upstream/src" "$package/vendor/"
cp "$upstream/LICENSE" "$upstream/CMakeLists.txt" "$upstream/package.xml" "$package/vendor/"
patch --batch --fuzz=0 -d "$package/vendor" -p1 < "$diagnostics/timing.patch"
patch --batch --fuzz=0 -d "$package/vendor" -p1 < "$diagnostics/live_mapping.patch"
for header in angle_grid experimental_scan timing_recorder host_time_anchor motion_model mapping_scan mapping_frame; do
  cp "$diagnostics/$header.hpp" "$package/vendor/include/coin_d4_driver/"
done
# Freeze the wheel/bringup sources and settings used by this build. No upstream edits.
mkdir -p "$workspace/baseline/ros2_ws/src"
cp -a "$repository/config" "$workspace/baseline/"
mkdir -p "$workspace/baseline/firmware/nt_opencr"
cp "$repository/firmware/nt_opencr/nt_packet.h" "$workspace/baseline/firmware/nt_opencr/"
for source in interfaces hardware swerve_controller description manipulation bringup; do
  cp -a "$repository/ros2_ws/src/$source" "$workspace/baseline/ros2_ws/src/"
done
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/lyrical/setup.bash
    cd "$1"
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
      --base-paths src baseline/ros2_ws/src --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    colcon test --packages-select lidar_workbench --event-handlers console_direct+
    colcon test-result --verbose
  ' _ "$workspace"
printf '\nBuilt isolated desktop workbench: %s\n' "$workspace"
