#!/usr/bin/env bash
# 실제 planner_server 경로 은행 만들기 한 번에: paths.launch.py (map_server · planner_server) → make_nav2_paths.py → 정리
# 결과: sim/robot_sim/rl/data/local_planner_nav2/{path_bank,dice_legs}_{navfn,smac2d,imitation}.npz · stats.json · paths_launch.log
# 예) bash rl/tasks/local_planner_nav2/run_make_paths.sh                 (전체, 약 10–20 분 예상)
#     bash rl/tasks/local_planner_nav2/run_make_paths.sh --maps dice --per_map 40     (빠른 확인)
# 인자는 make_nav2_paths.py 로 그대로 넘김. NAV2_PARAMS=<yaml> 이면 config/nav2.yaml 대신 그 파일 (팽창 비교 등)
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(cd "$HERE/../../.." && pwd)"                       # sim/robot_sim
OUT="$SIM/rl/data/local_planner_nav2"
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=17 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 같은 망 로봇과 안 섞이게

setsid ros2 launch "$HERE/paths.launch.py" ${NAV2_PARAMS:+params:="$NAV2_PARAMS"} > "$OUT/paths_launch.log" 2>&1 &
LAUNCH=$!
cleanup() {                                                 # 비대화형 셸의 & 프로세스는 SIGINT 를 무시 → 프로세스 묶음에 TERM
  kill -TERM -- -$LAUNCH 2>/dev/null || true
  for _ in $(seq 20); do kill -0 $LAUNCH 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- -$LAUNCH 2>/dev/null || true
}
trap cleanup EXIT
echo "[run] planner_server 기다림 (로그 $OUT/paths_launch.log)"
for _ in $(seq 60); do
  ros2 action list 2>/dev/null | grep -q '^/compute_path_to_pose$' && break
  sleep 1
done
ros2 action list 2>/dev/null | grep -q '^/compute_path_to_pose$' || { echo "[run] planner_server 안 뜸"; tail -30 "$OUT/paths_launch.log"; exit 1; }
sleep 2                                                     # lifecycle 활성화
cd "$SIM"
python3 "$HERE/make_nav2_paths.py" "$@"
