#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
set -u
cmake -S "$root/tools/maze_mapping_lab" -B "$root/build/maze_mapping_lab" -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/build/maze_mapping_lab" -j 4
