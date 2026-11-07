#!/usr/bin/env bash
# Isaac 검증 (MPPI): run_isaac.sh 와 같은 브리지 · 팀 로봇 · run_goals.py (dice 10 곳) 로 Nav2 MPPI 를 잼 → RL 정책과 같은 조건 비교.
#   MPPI 샘플 분산 vx_std · vy_std 를 바꿔 볼 수 있음 (IROS 2024 4WIDS MPPI 논문: 3D(a) 분산 1.0 → 3D(b) 0.55 로 조향 속도 6.0 → 3.6 rad/s, 성공 33 → 58 %)
#   설정 = 레포 config/nav2.yaml 을 복사해 std 만 바꾼 파일 (OUT/nav2_<이름>.yaml, 레포 파일은 안 바꿈)
# 사용: run_isaac_mppi.sh <이름> [vx·vy std (기본 = 레포 값 0.15)] [wz std (기본 = 레포 값 0.4)] [녹화 초 (기본 260)]
#   예) run_isaac_mppi.sh mppi_base            (지금 설정)
#       run_isaac_mppi.sh mppi_std010 0.10      (vx · vy std 0.15 → 0.10)
# 환경변수: MOVEIT_SIM · NAV_WS · OUT · ROS_DOMAIN_ID (run_isaac.sh 와 같음)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SIM="$(cd "$HERE/../../../.." && pwd)"                      # sim/robot_sim
REPO="$(cd "$SIM/../.." && pwd)"
MOVEIT_SIM="${MOVEIT_SIM:-$REPO/../Project2_moveit/sim/robot_sim}"
NAV_WS="${NAV_WS:?navigation 패키지를 빌드한 install 폴더를 NAV_WS 로 줄 것}"
OUT="${OUT:-$SIM/rl/data/local_planner/isaac}"
TAG=$1; STD=${2:-}; WSTD=${3:-}; SECS=${4:-260}
mkdir -p "$OUT"
set +u; source /opt/ros/lyrical/setup.bash; source "$NAV_WS/setup.bash"; set -u   # ROS setup 은 없는 변수를 씀
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TN_ROBOT=team
# 설정: 레포 nav2.yaml 복사 + std 바꿈
PARAMS="$OUT/nav2_$TAG.yaml"
python3 - "$REPO/config/nav2.yaml" "$PARAMS" "$STD" "$WSTD" <<'EOF'
import re, sys
src, dst, std, wstd = sys.argv[1:5]
s = open(src).read()
if std:
    s = re.sub(r'(\n\s+vx_std:\s*)[0-9.]+', r'\g<1>' + std, s); s = re.sub(r'(\n\s+vy_std:\s*)[0-9.]+', r'\g<1>' + std, s)
if wstd:
    s = re.sub(r'(\n\s+wz_std:\s*)[0-9.]+', r'\g<1>' + wstd, s)
open(dst, 'w').write(s)
print('MPPI std:', re.findall(r'\n\s+(v[xy]_std|wz_std):\s*([0-9.]+)', s))
EOF
# 이 실행이 띄운 프로세스에 표식 (환경변수, 자식이 물려받음) → 정리 · 재시도 때 표식 있는 것만 끔 (다른 시험 · 다른 사람 프로세스 안 건드림)
RUN="mppi_${TAG}_$$"
marked() {   # $1 = 표식 값 → 그 표식을 가진 내 프로세스 pid
  for p in $(pgrep -u "$USER" .); do
    [ "$p" = "$$" ] && continue
    { tr '\0' '\n' < /proc/$p/environ; } 2>/dev/null | grep -qx "NT_RUN_MARK=$1" && echo $p
  done; true
}
kill_marked() {   # $1 = 표식 값, INT → 4 s → KILL
  local ps; ps=$(marked "$1"); [ -n "$ps" ] && kill -INT $ps 2>/dev/null; sleep 4
  ps=$(marked "$1"); [ -n "$ps" ] && kill -9 $ps 2>/dev/null; true
}
cleanup() { kill_marked "$RUN.ros"; kill_marked "$RUN.bridge"; }
trap cleanup EXIT
# 1. 브리지 (run_isaac.sh 와 같음)
(export NT_RUN_MARK="$RUN.bridge"; cd "$MOVEIT_SIM" && setsid scripts/sim_nav_bridge.sh --signal_delay -1 --real_speed --record "$OUT/$TAG.mp4" --record_secs $SECS \
   --record_view=0.0,-6.2,6.8,0.0,0.1,0.0 > "$OUT/bridge_$TAG.log" 2>&1) &
timeout 400 bash -c "until grep -qE '^\[bridge\] 경기장|Traceback' '$OUT/bridge_$TAG.log'; do sleep 2; done" || { echo BRIDGE_FAIL; exit 1; }
export NT_RUN_MARK="$RUN.ros"          # 여기부터 띄우는 ROS 노드 표식
# 2. 팀 로봇 모델 (TF) → 위치 추정 (map_server · AMCL) 먼저 → map → odom TF 확인 → Nav2 주행 부분 (planner · MPPI · bt_navigator · collision_monitor)
#    한 launch 로 같이 띄우면 planner 가 AMCL 보다 먼저 map 프레임을 기다리다 시간 초과로 실패함 (2026-11-05)
URDF="$MOVEIT_SIM/robot_description/team/robot.urdf"
RSP_YAML="$OUT/rsp_$TAG.yaml"         # URDF 를 명령줄로 넘기면 'Argument list too long' → 파라미터 파일로
python3 - "$URDF" "$RSP_YAML" <<'PYEOF'
import json, sys
urdf = open(sys.argv[1]).read().replace('filename="/', 'filename="file:///')
open(sys.argv[2], 'w').write('robot_state_publisher:\n  ros__parameters:\n    use_sim_time: true\n    robot_description: ' + json.dumps(urdf) + '\n')
PYEOF
start_rsp() {   # RSP + scan_merger (/scan = collision_monitor 입력, 실물과 같은 몸 상자)
  ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$RSP_YAML" > "$OUT/rsp_$TAG.log" 2>&1 &
  ros2 run navigation scan_merger --ros-args --params-file "$(ros2 pkg prefix navigation)/share/navigation/config/navigation.yaml" \
     -p use_sim_time:=true >> "$OUT/rsp_$TAG.log" 2>&1 &
}
start_rsp
ok=
for try in 1 2 3; do
  ros2 run nav2_map_server map_server --ros-args -r __node:=map_server --params-file "$PARAMS" -p use_sim_time:=true \
     -p yaml_filename:="$MOVEIT_SIM/maps/maze_dice_map.yaml" > "$OUT/loc_$TAG.log" 2>&1 &
  ros2 run nav2_amcl amcl --ros-args -r __node:=amcl --params-file "$PARAMS" -p use_sim_time:=true -p set_initial_pose:=true \
     -p initial_pose.x:=-1.468 -p initial_pose.y:=-2.202 -p initial_pose.yaw:=3.14159265 >> "$OUT/loc_$TAG.log" 2>&1 &
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_localization -p use_sim_time:=true \
     -p autostart:=true -p "node_names:=['map_server','amcl']" >> "$OUT/loc_$TAG.log" 2>&1 &
  if timeout 60 bash -c "until timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q Translation; do sleep 1; done"; then
    ros2 launch navigation nav2.launch.py localization:=slam params:="$PARAMS" use_sim_time:=true mission_hold:=false > "$OUT/nav_$TAG.log" 2>&1 &
    if timeout 90 bash -c "until grep -q 'Managed nodes are active' '$OUT/nav_$TAG.log'; do sleep 1; done"; then ok=1; break; fi
  fi
  echo "NAV2 retry $try (loc_$TAG.log · nav_$TAG.log)"; kill_marked "$RUN.ros"
  start_rsp
done
[ -z "$ok" ] && { echo NAV2_FAIL; tail -20 "$OUT/nav_$TAG.log"; exit 1; }
sleep 3
# 3. 10 곳 주행 (정답 자세로 잼)
timeout 1200 python3 "$HERE/run_goals.py" "$OUT/res_$TAG.json" 2>&1 | tee "$OUT/goals_$TAG.log"
# 4. 녹화 끝
timeout 600 bash -c "until grep -q '녹화 끝' '$OUT/bridge_$TAG.log'; do sleep 3; done"
grep '녹화 끝' "$OUT/bridge_$TAG.log"
echo DONE $TAG
