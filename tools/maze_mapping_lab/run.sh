#!/usr/bin/env bash
# Simulation-only: no serial API, device option, firmware or controller launch.
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
[[ $# -gt 0 ]] || { echo 'Usage: run.sh scene build|generate|render|validate|summarize ... | replay ... | constrained PROFILE ... | controlled-suite [NEW_OUTPUT_DIRECTORY]' >&2; exit 2; }
operation=$1
shift
set +u
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$root/output/maze_mapping_lab/ros_logs"
mkdir -p "$ROS_LOG_DIR"
case "$operation" in
  scene) exec "$root/build/maze_mapping_lab/maze_scene" "$@" ;;
  replay) exec "$root/build/maze_mapping_lab/maze_replay" "$@" ;;
  suite)
    [[ $# == 0 ]] || { echo 'suite accepts no additional arguments' >&2; exit 2; }
    mkdir -p "$root/output/maze_mapping_lab/suite_logs"
    for case_name in coverage040 open040 north040 coverage025 coverage080 holonomic040 coverage040_seed2; do
      echo "START $case_name"
      "$root/tools/maze_mapping_lab/run.sh" constrained six_core_6g \
        --dataset "$root/output/maze_mapping_lab/datasets_v2/$case_name" \
        --output "$root/output/maze_mapping_lab/trials/$case_name" \
        --baseline "$root/config/slam_front_experiment.yaml" --rate 10 \
        > "$root/output/maze_mapping_lab/suite_logs/$case_name.log" 2>&1
      echo "COMPLETE $case_name"
    done
    ;;
  controlled-suite)
    [[ $# -le 1 ]] || exit 2
    suite_output=$(realpath -m -- "${1:-$root/output/maze_mapping_lab/controlled}")
    [[ ! -e $suite_output ]] || { echo 'Controlled-suite output must be a new directory' >&2; exit 2; }
    mkdir -p "$suite_output/logs"
    for case_name in perimeter040 coverage040 open040 north040 coverage025 coverage080 holonomic040 holonomic025 holonomic040_seed2 coverage040_seed2 ideal040 coverage040_repeat coverage040_loop075 coverage040_noloop; do
      dataset_name=$case_name
      extra=()
      case "$case_name" in
        coverage040_repeat) dataset_name=coverage040 ;;
        coverage040_loop075) dataset_name=coverage040; extra=(--loop-search-distance 0.75) ;;
        coverage040_noloop) dataset_name=coverage040; extra=(--loop-closing 0) ;;
      esac
      dataset_root=datasets_v2
      [[ $dataset_name != perimeter040 ]] || dataset_root=datasets
      echo "START controlled $case_name"
      "$root/tools/maze_mapping_lab/run.sh" constrained six_core_6g \
        --dataset "$root/output/maze_mapping_lab/$dataset_root/$dataset_name" \
        --output "$suite_output/$case_name" \
        --baseline "$root/config/slam_front_experiment.yaml" --rate 30 --lockstep 1 --domain 222 \
        "${extra[@]}" > "$suite_output/logs/$case_name.log" 2>&1
      echo "COMPLETE controlled $case_name"
    done
    ;;
  constrained)
    [[ $# -gt 0 ]] || exit 2
    profile=$1
    shift
    case "$profile" in
      desktop) quota=2400%; memory=28G ;;
      six_core_6g) quota=600%; memory=6G ;;
      two_core_4g) quota=200%; memory=4G ;;
      one_core_2g) quota=100%; memory=2G ;;
      quarter_core_1g) quota=25%; memory=1G ;;
      custom) quota=; memory= ;;
      *) echo 'Unknown simulation resource profile' >&2; exit 2 ;;
    esac
    cpu_list=0-5
    forwarded=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --cpu-quota-percent)
          [[ $# -ge 2 && $2 =~ ^[1-9][0-9]*$ ]] || { echo 'CPU quota must be a positive integer percentage' >&2; exit 2; }
          quota="$2%"; shift 2 ;;
        --memory-max)
          [[ $# -ge 2 && $2 =~ ^[1-9][0-9]*[MG]$ ]] || { echo 'Memory cap must use positive M or G units, e.g. 2G' >&2; exit 2; }
          memory=$2; shift 2 ;;
        --cpu-list)
          [[ $# -ge 2 && $2 =~ ^[0-9]+([,-][0-9]+)*$ ]] || { echo 'Invalid CPU affinity list' >&2; exit 2; }
          cpu_list=$2; shift 2 ;;
        *) forwarded+=("$1"); shift ;;
      esac
    done
    [[ -n $quota && -n $memory ]] || { echo 'Custom profile requires --cpu-quota-percent and --memory-max' >&2; exit 2; }
    # Fresh, automatically collected user service; only this offline replay and
    # its owned SLAM child are constrained. No global clock/power-mode changes.
    exec systemd-run --user --wait --pipe --collect --working-directory="$root" \
      -p "CPUQuota=$quota" -p "MemoryMax=$memory" -p MemorySwapMax=0 \
      -p CPUAccounting=yes -p MemoryAccounting=yes \
      /usr/bin/taskset -c "$cpu_list" "$root/tools/maze_mapping_lab/run.sh" replay --profile "$profile" "${forwarded[@]}"
    ;;
  *) echo 'Unknown operation' >&2; exit 2 ;;
esac
