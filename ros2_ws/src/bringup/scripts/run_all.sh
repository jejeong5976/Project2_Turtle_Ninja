#!/usr/bin/env bash
# 창 하나에서 켜기: 떠 있는 실행 정리 → 자동 위치/시작 복구 → STANDBY 확인 → 녹화 → robot_cli.
# 주행 정책·도킹 값은 아래 launch 인자로, 팔/인지 실행값은 [3/6] 에서 넘김. 임계값은 각 노드 한 곳에만 둠.
# 사용: bash run_all.sh  (환경 변수 INIT=x,y,yaw_deg · MAP=지도경로 · P=미션설정 선택 가능)
# 종료는 $ROBOT_TOOLS/stop_robot.sh 가 담당함. 이 스크립트는 /mission/start를 자동 호출하지 않음.
set -euo pipefail
ROBOT_PROJECT=${ROBOT_PROJECT:-$HOME/p2_integ}
ROBOT_TOOLS=${ROBOT_TOOLS:-$HOME/real_1111}
P=${P:-$HOME/planning_run.yaml}
MAP=${MAP:-$HOME/maps/arena7}
INIT=${INIT:--0.357,-0.005,-1.1}
RUN_ALL_LOG_DIR=${RUN_ALL_LOG_DIR:-$HOME/run_all_logs}
RUN_ALL_BAG_DIR=${RUN_ALL_BAG_DIR:-$HOME/bags}
run_stamp=$(date +%m%d_%H%M%S)
mkdir -p "$RUN_ALL_LOG_DIR" "$RUN_ALL_BAG_DIR"
set +u
source /opt/ros/lyrical/setup.bash
source "$ROBOT_PROJECT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=17 FASTDDS_BUILTIN_TRANSPORTS=DEFAULT ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "[1/6] 기존 로봇 · 녹화 정리"
bash "$ROBOT_TOOLS/stop_robot.sh" > "$RUN_ALL_LOG_DIR/stop_$run_stamp.log" 2>&1 || true
# stop_robot.sh에는 rosbag 종료가 없어 녹화는 여기서 끝냄.
pkill -INT -f 'ros2 bag record' 2>/dev/null || true
sleep 2
# 공유 메모리는 여기서 지우지 않음 (DDS 정리는 위 stop_robot.sh 가 함).
ros2 daemon stop >/dev/null 2>&1 || true

echo "[2/6] 로봇 켜기 — 지도 $MAP · 설정 $P · 초기 탐색 중심 $INIT"
MAP="$MAP" INIT="$INIT" SKIP_CONFIRM=1 setsid nohup bash "$ROBOT_TOOLS/start_robot_amcl.sh" "$P" \
  mission_loc:=amcl nav_policy:=nav_nf5 dock_policy:=dock3 dock_tol:=0.015 dock_search:=0.30 \
  startup_recovery:=true auto_initial_pose:=true wrist_respawn:=true \
  > "$RUN_ALL_LOG_DIR/robot_$run_stamp.log" 2>&1 < /dev/null &
robot_pid=$!

echo "[3/6] 기존 팔 · 인식 실행 설정 적용"
# 실행 뒤 켜는 두 값 (held_image_compensate · wrist.held.enable). 서비스가 늦게 떠도 사람 입력 없이 다시 시도함.
control_setting_ready=false
perception_setting_ready=false
settings_deadline=$((SECONDS + 240))
while (( SECONDS < settings_deadline )); do
  if [[ "$control_setting_ready" != true ]] && timeout 5 ros2 param set /control_master held_image_compensate true >/dev/null 2>&1; then
    control_setting_ready=true
  fi
  if [[ "$perception_setting_ready" != true ]] && timeout 5 ros2 param set /perception wrist.held.enable true >/dev/null 2>&1; then
    perception_setting_ready=true
  fi
  if [[ "$control_setting_ready" == true && "$perception_setting_ready" == true ]]; then
    break
  fi
  if ! kill -0 "$robot_pid" 2>/dev/null; then
    echo "로봇 실행이 끝남 — $RUN_ALL_LOG_DIR/robot_$run_stamp.log"
    tail -n 30 "$RUN_ALL_LOG_DIR/robot_$run_stamp.log"
    exit 1
  fi
  sleep 1
done
if [[ "$control_setting_ready" != true || "$perception_setting_ready" != true ]]; then
  echo "기존 실행 설정 적용 미완료: control=$control_setting_ready perception=$perception_setting_ready"
fi

echo "[4/6] 자동 위치 맞춤 · lifecycle 복구 · 손목 영상 · STANDBY 현재 상태 확인"
# lifecycle RESET/STARTUP의 유일한 주인은 watchdog임. 여기서 같은 서비스를 또 호출하지 않음.
ros2 run navigation startup_report.py --timeout 240 || true

echo "[5/6] 녹화 → $RUN_ALL_BAG_DIR/mission_$run_stamp"
setsid nohup ros2 bag record -s mcap -o "$RUN_ALL_BAG_DIR/mission_$run_stamp" --all-topics \
  --exclude-regex '.*(image|Image|depth|points|compressed|camera_info|aligned|lidar_front|lidar_rear|/map$|costmap).*' \
  > "$RUN_ALL_LOG_DIR/bag_$run_stamp.log" 2>&1 < /dev/null &

echo "[6/6] robot_cli (자동 맞춤과 복구는 백그라운드에서 계속 동작함)"
echo "  시작 로그: $RUN_ALL_LOG_DIR/robot_$run_stamp.log"
exec ros2 run robot_cli robot_cli
