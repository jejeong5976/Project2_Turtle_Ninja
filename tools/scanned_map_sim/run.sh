#!/usr/bin/env bash
set -eo pipefail
sim_tools=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$sim_tools/paths.sh"
sim_maps=
sim_generation_options=false
sim_mode=custom sim_speed=0.025 sim_yaw=0.10 sim_shift=0 sim_seed=11 sim_walls=all sim_map=matched sim_route=l2 sim_rviz=false sim_prepare=false sim_deadline=600
sim_config="$sim_tools/maze.yaml"
sim_unknown=blocked sim_penalty=2.0 sim_ttl=4.0 sim_scenario=scanned
while (( $# )); do
  case "$1" in
    --unknown-policy) sim_unknown=${2:?}; shift 2;;
    --unknown-penalty) sim_penalty=${2:?}; shift 2;;
    --observation-ttl) sim_ttl=${2:?}; shift 2;;
    --scenario) sim_scenario=${2:?}; sim_generation_options=true; shift 2;;
    --maps) sim_maps=${2:?}; shift 2;;
    --mode) sim_mode=${2:?}; shift 2;;
    --speed) sim_speed=${2:?}; shift 2;;
    --yaw-rate) sim_yaw=${2:?}; shift 2;;
    --shift) sim_shift=${2:?}; sim_generation_options=true; shift 2;;
    --seed) sim_seed=${2:?}; sim_generation_options=true; shift 2;;
    --walls) sim_walls=${2:?}; sim_generation_options=true; shift 2;;
    --map-mode) sim_map=${2:?}; sim_generation_options=true; shift 2;;
    --route) sim_route=${2:?}; shift 2;;
    --config) sim_config=${2:?}; sim_generation_options=true; shift 2;;
    --goal-timeout) sim_deadline=${2:?}; shift 2;;
    --rviz) sim_rviz=true; shift;;
    --prepare-only) sim_prepare=true; shift;;
    --help) printf '%s\n' 'run.sh [--mode custom|original] [--speed MPS] [--yaw-rate RADPS]' '       [--shift 0..0.05] [--seed INTEGER] [--walls inner|all]' '       [--map-mode matched|stale|partial] [--route l2|maze|GOALS_FILE]' '       [--unknown-policy blocked|observe-before-enter] [--unknown-penalty POSITIVE] [--observation-ttl SECONDS]' '       [--scenario scanned|corridor|concealed|partial|turn]' '       [--maps PREPARED_SCENARIO_DIRECTORY]' '       [--rviz] [--prepare-only] [--goal-timeout SECONDS] [--config YAML]'; exit 0;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2;;
  esac
done
if [[ ! "$sim_mode" =~ ^(custom|original)$ || ! "$sim_seed" =~ ^[0-9]+$ || ! "$sim_deadline" =~ ^[0-9]+$ ]]; then printf 'Invalid mode, seed or deadline\n' >&2; exit 2; fi
if (( sim_deadline<30 || sim_deadline>3600 )); then printf 'Goal deadline must be 30..3600 seconds\n' >&2; exit 2; fi
if [[ "$sim_rviz" == true && -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then printf 'RViz requires a graphical terminal. Omit --rviz for headless operation.\n' >&2; exit 2; fi
if [[ -n "$sim_maps" && "$sim_generation_options" == true ]]; then printf '%s\n' '--maps cannot be combined with map generation options.' >&2; exit 2; fi
if [[ ! -x "$sim_helpers/feed" || ! -x "$sim_helpers/import_scenario" || ! -x "$sim_helpers/startup" ]]; then printf 'First run: bash %s/build.sh\n' "$sim_tools" >&2; exit 2; fi
exec 9>/tmp/scanned-map-controller-simulation.lock
flock -n 9 || { printf 'Another scanned-map simulation is running.\n' >&2; exit 2; }
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source "$sim_ros_setup"
source "$sim_install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=$((70 + RANDOM % 100))
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY ROUTE_FIXTURE_SCENARIO
export FASTDDS_DEFAULT_PROFILES_FILE="$sim_tools/loopback.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE" PYTHONDONTWRITEBYTECODE=1
mkdir -p "$sim_results"
export SCANNED_SIM_RUN
SCANNED_SIM_RUN=$(mktemp -d "$sim_results/$(date +%Y%m%d-%H%M%S)-$sim_mode-XXXXXX")
export SCANNED_SIM_RVIZ="$sim_rviz" SCANNED_SIM_DEADLINE="$sim_deadline"
export ROS_LOG_DIR="$SCANNED_SIM_RUN/ros-log"
printf 'Run directory: %s\n' "$SCANNED_SIM_RUN"
export SCANNED_SIM_HELPERS="$sim_helpers"
if [[ -n "$sim_maps" ]]; then
  "$sim_helpers/import_scenario" "$sim_maps" "$SCANNED_SIM_RUN/maps" "$sim_route" | tee "$SCANNED_SIM_RUN/prepare.log"
elif [[ "$sim_scenario" == scanned ]]; then
  cp -- "$sim_config" "$SCANNED_SIM_RUN/input-config.yaml"
"$sim_helpers/prepare" "$sim_config" "$SCANNED_SIM_RUN/maps" "$sim_shift" "$sim_seed" "$sim_walls" "$sim_map" "$sim_route" | tee "$SCANNED_SIM_RUN/prepare.log"
else
  "$sim_helpers/prepare_unknown" "$SCANNED_SIM_RUN/maps" "$sim_scenario" | tee "$SCANNED_SIM_RUN/prepare.log"
fi
ros2 run body_route_controller configure_trial --mode "$sim_mode" --speed "$sim_speed" --yaw-rate "$sim_yaw" --unknown-policy "$sim_unknown" --unknown-penalty "$sim_penalty" --observation-ttl "$sim_ttl" --output "$SCANNED_SIM_RUN/profile"
printf 'domain: %s\nmode: %s\nspeed_mps: %s\nyaw_rate_radps: %s\n' "$ROS_DOMAIN_ID" "$sim_mode" "$sim_speed" "$sim_yaw" > "$SCANNED_SIM_RUN/runtime-settings.yaml"
if [[ "$sim_prepare" == true ]]; then printf 'Maps prepared; no ROS graph launched. See maps/layouts.png and maps/scenario.yaml\n'; exit 0; fi
printf 'MOCK ONLY: scanned-map truth, synthetic lidar, fixed start, loopback DDS. Ctrl+C stops.\n'
set +e
ros2 launch "$sim_tools/simulation.launch.py" 2>&1 | tee "$SCANNED_SIM_RUN/launch.log"
sim_launch_rc=${PIPESTATUS[0]}
if [[ -f "$SCANNED_SIM_RUN/startup.yaml" ]]; then
  printf '\nStartup report:\n'
  cat -- "$SCANNED_SIM_RUN/startup.yaml"
fi
"$sim_helpers/evaluate" "$SCANNED_SIM_RUN" | tee "$SCANNED_SIM_RUN/evaluation.txt"
sim_evaluate_rc=${PIPESTATUS[0]}
printf '\nTrajectory: %s/trajectory.png\nResults: %s/evaluation.yaml\n' "$SCANNED_SIM_RUN" "$SCANNED_SIM_RUN"
if (( sim_launch_rc != 0 || sim_evaluate_rc != 0 )); then printf 'Simulation failed or incomplete; retained for diagnosis.\n' >&2; exit 1; fi
printf 'Simulation passed the trial, truth-arrival and swept-footprint checks.\n'
