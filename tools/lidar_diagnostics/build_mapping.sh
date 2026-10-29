#!/usr/bin/env bash
# Build only; never opens a device or starts a ROS node.
set -euo pipefail
[[ $# == 2 ]] || { echo "Usage: $0 VERIFIED_UPSTREAM NEW_WORKSPACE" >&2; exit 2; }
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
upstream="$(realpath -- "$1")"
workspace="$(realpath -m -- "$2")"
[[ ! -e "$workspace" ]] || { echo "Refusing existing workspace" >&2; exit 2; }
(cd "$upstream" && sha256sum --check "$script_dir/source.sha256")
copy="$workspace/src/coin_d4_mapping_driver"
mkdir -p "$copy"
cp -a "$upstream/include" "$upstream/src" "$copy/"
cp "$upstream/CMakeLists.txt" "$upstream/package.xml" "$upstream/LICENSE" "$copy/"
patch --batch --fuzz=0 -d "$copy" -p1 < "$script_dir/timing.patch"
patch --batch --fuzz=0 -d "$copy" -p1 < "$script_dir/live_mapping.patch"
for header in angle_grid experimental_scan timing_recorder host_time_anchor motion_model mapping_scan mapping_frame; do
  cp "$script_dir/$header.hpp" "$copy/include/coin_d4_driver/"
done
cp "$script_dir/mapping_node.cpp" "$script_dir/test_mapping_node.cpp" "$script_dir/test_timing_parser.cpp" "$copy/src/"
cp "$script_dir/mapping_package.CMakeLists.txt" "$copy/CMakeLists.txt"
cp "$script_dir/mapping_package.xml" "$copy/package.xml"
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/lyrical/setup.bash
    cd "$1"
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --packages-select coin_d4_mapping_driver
  ' _ "$workspace"
printf '\nBuilt isolated mapping prototype: %s\n' "$workspace"
