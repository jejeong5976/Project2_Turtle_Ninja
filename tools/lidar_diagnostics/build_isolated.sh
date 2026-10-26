#!/usr/bin/env bash
# Copy and instrument the inspected upstream source; never modify its working installation.
set -euo pipefail
if [[ $# != 2 ]]; then
  echo "Usage: $0 UPSTREAM_SOURCE NEW_DIAGNOSTIC_WORKSPACE" >&2
  exit 2
fi
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
upstream="$(realpath -- "$1")"
workspace="$(realpath -m -- "$2")"
if [[ -e "$workspace" ]]; then
  echo "Refusing to overwrite existing diagnostic workspace: $workspace" >&2
  exit 2
fi
(cd "$upstream" && sha256sum --check "$script_dir/source.sha256")
mkdir -p "$workspace/src/coin_d4_diagnostic_driver"
copy="$workspace/src/coin_d4_diagnostic_driver"
cp -a "$upstream/include" "$upstream/src" "$upstream/launch" "$upstream/params" "$copy/"
cp "$upstream/CMakeLists.txt" "$upstream/package.xml" "$upstream/LICENSE" "$copy/"
patch --batch --fuzz=0 -d "$copy" -p1 < "$script_dir/instrumentation.patch"
cp "$script_dir/diagnostic_recorder.hpp" "$copy/include/coin_d4_driver/"
cp "$script_dir/source.sha256" "$workspace/upstream-source.sha256"
# Build with only the ROS underlay, avoiding accidental dependency on normal driver overlays.
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/lyrical/setup.bash
    cd "$1"
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
      --packages-select coin_d4_diagnostic_driver
  ' _ "$workspace"
printf '\nBuilt diagnostic workspace: %s\nWorking driver source and installation were not changed.\n' "$workspace"
