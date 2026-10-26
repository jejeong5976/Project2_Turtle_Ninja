#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 ]]; then
  echo "Usage: $0 UPSTREAM_SOURCE NEW_TIMING_WORKSPACE" >&2
  exit 2
fi
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
upstream="$(realpath -- "$1")"
workspace="$(realpath -m -- "$2")"
if [[ -e "$workspace" ]]; then
  echo "Refusing to overwrite existing workspace: $workspace" >&2
  exit 2
fi
(cd "$upstream" && sha256sum --check "$script_dir/source.sha256")
mkdir -p "$workspace/src/coin_d4_timing_driver"
copy="$workspace/src/coin_d4_timing_driver"
cp -a "$upstream/include" "$upstream/src" "$copy/"
cp "$upstream/CMakeLists.txt" "$upstream/package.xml" "$upstream/LICENSE" "$copy/"
patch --batch --fuzz=0 -d "$copy" -p1 < "$script_dir/timing.patch"
cp "$script_dir/angle_grid.hpp" "$script_dir/experimental_scan.hpp" \
  "$script_dir/timing_recorder.hpp" "$copy/include/coin_d4_driver/"
cp "$script_dir/test_timing_parser.cpp" "$copy/src/"
cp "$script_dir/source.sha256" "$workspace/upstream-source.sha256"
mkdir "$workspace/timing-inputs"
cp "$script_dir/timing.patch" "$script_dir/angle_grid.hpp" "$script_dir/experimental_scan.hpp" \
  "$script_dir/timing_recorder.hpp" "$script_dir/test_timing_parser.cpp" \
  "$script_dir/build_timing.sh" "$workspace/timing-inputs/"
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/lyrical/setup.bash
    cd "$1"
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
      --packages-select coin_d4_timing_driver
  ' _ "$workspace"
printf '\nBuilt timing diagnostic: %s\n' "$workspace"
