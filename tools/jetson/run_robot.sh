#!/usr/bin/env bash
# 젯슨 로봇 실행 — 저장소 맨 위에서
#   bash tools/jetson/run_robot.sh drive mock      바퀴만 · 로봇 없이 (가짜 모터)
#   bash tools/jetson/run_robot.sh drive opencr    바퀴만 · 실물 (주행 OpenCR)
#   bash tools/jetson/run_robot.sh full  mock      팔까지 · 로봇 없이
#   bash tools/jetson/run_robot.sh full  opencr    팔까지 · 실물 (시연 모드: 켤 때 바퀴 잠금 → 팔 HOME 성공 뒤 주행)
#   bash tools/jetson/run_robot.sh slam  opencr    바퀴 + 앞 · 뒤 라이다 + slam_toolbox 지도 만들기 + 위치 감시 — 지도 저장 tools/jetson/save_map.sh <이름>
#   bash tools/jetson/run_robot.sh locate opencr ~/maps/<이름>   저장한 지도로 위치 찾기 (map→odom) + 위치 감시
#   라이다: ROBOTIS coin_d4_driver (sudo apt install ros-lyrical-coin-d4-driver). 포트 NT_LIDAR_FRONT=/dev/serial/by-path/… NT_LIDAR_REAR=…
#     (ls -l /dev/serial/by-path/ 로 확인. 뒤를 NT_LIDAR_REAR= 로 비우면 앞만)
#   뒤에 launch 인자를 더 붙일 수 있음 (예: steering_policy:=nearest). 단 이 스크립트가 검사하는 인자
#   (hardware · hardware_map · arm · opencr_port(s) · lidar_*_port · slam_mode · map · use_sim_time · robot) 는 덧붙일 수 없음.
#   실물 시작 확인을 건너뛰려면 NT_YES=1
# 포트: 기본 /dev/opencr_drive · /dev/opencr_arm (tools/jetson/make_udev_rules.py). 규칙 없이 쓰려면
#   NT_DRIVE_PORT=/dev/serial/by-path/… NT_ARM_PORT=/dev/serial/by-path/… (젯슨에서 주행 보드 = …usb-0:2.4.3:1.0)
# 기록: runs/robot_<모드>_<hw>_<시각>/ (launch.log · 컨트롤러 파라미터 · 조향 범위). 끝내기 Ctrl+C (명령이 0.3 s 끊기면 바퀴는 스스로 멈춤)
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
MODE=${1:-}; HW=${2:-}; shift 2 2>/dev/null || true
case "$MODE/$HW" in drive/mock|drive/opencr|full/mock|full/opencr|slam/opencr|locate/opencr) ;; *) sed -n 2,15p "$0"; exit 2;; esac
MAPARG=""
if [ "$MODE" = locate ]; then MAPARG=${1:-}; shift || true; [ -f "${MAPARG}.posegraph" ] || { echo "✗ 지도 ${MAPARG}.posegraph 없음 (확장자 빼고)"; exit 2; }; fi
DISTRO=${ROS_DISTRO_WANT:-lyrical}
[ -f "$ROOT/ros2_ws/install/setup.bash" ] || { echo "✗ 빌드 먼저: bash tools/jetson/build_jetson.sh"; exit 2; }
set +u; source /opt/ros/$DISTRO/setup.bash   # ROS setup 은 없는 변수를 씀
source "$ROOT/ros2_ws/install/setup.bash"; set -u
RUN="$ROOT/runs/robot_${MODE}_${HW}_$(date +%Y%m%dT%H%M%S)"; mkdir -p "$RUN"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$RUN/wrapper.log"; }
# 같은 컴퓨터에 다른 ros2_control 이 있으면 거부 (같은 포트 · /cmd_vel · TF 주인이 둘이 됨)
if pgrep -f "lib/controller_manager/ros2_control_[n]ode" >/dev/null; then
  say "✗ 다른 ros2_control_node 가 떠 있음:"; pgrep -af "lib/controller_manager/ros2_control_[n]ode" | tee -a "$RUN/wrapper.log"; exit 3
fi
SHARE=$(ros2 pkg prefix bringup)/share/bringup/config
ARGS=(hardware:=$HW)
[ "$MODE" = drive ] && ARGS+=(arm:=false "hardware_map:=$SHARE/hardware_map_drive_only.yaml")
port_ok() {   # 실제 장치를 읽고 쓸 수 있나 (udev 규칙의 그룹 · 권한과 상관없이)
  local p; p=$(readlink -f "$1"); [ -e "$p" ] && [ -r "$p" ] && [ -w "$p" ]
}
PKG=bringup; LAUNCH=robot.launch.py
if [ "$MODE" = slam ] || [ "$MODE" = locate ]; then   # navigation/slam.launch.py 가 robot.launch.py (바퀴만) 를 포함
  PKG=navigation; LAUNCH=slam.launch.py
  LF=${NT_LIDAR_FRONT-/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0}; LR=${NT_LIDAR_REAR-}
  [ -n "$LF$LR" ] || { say "✗ 라이다 포트 없음 — NT_LIDAR_FRONT · NT_LIDAR_REAR (ls -l /dev/serial/by-path/)"; exit 4; }
  for L in $LF $LR; do port_ok "$L" || { say "✗ 라이다 포트 $L 를 열 수 없음 (없거나 권한) — ls -l /dev/serial/by-path/"; exit 4; }; done
  [ -n "$LR" ] || say "⚠ 뒤 라이다 없이 앞만 (NT_LIDAR_REAR)"
  ros2 pkg prefix coin_d4_driver >/dev/null 2>&1 || { say "✗ coin_d4_driver 없음 — sudo apt install ros-lyrical-coin-d4-driver"; exit 4; }
  ARGS+=(arm:=false "hardware_map:=$SHARE/hardware_map_drive_only.yaml" "lidar_front_port:=$LF" "lidar_rear_port:=$LR")
  if [ "$MODE" = locate ]; then ARGS+=(slam_mode:=localization "map:=$MAPARG"); else ARGS+=(slam_mode:=mapping); fi
fi
if [ "$HW" = opencr ]; then
  DP=${NT_DRIVE_PORT:-/dev/opencr_drive}; AP=${NT_ARM_PORT:-/dev/opencr_arm}
  port_ok "$DP" || { say "✗ 주행 보드 포트 $DP 를 열 수 없음 (없거나 권한 — make_udev_rules.py 또는 NT_DRIVE_PORT)"; exit 4; }
  PORTS="bus_drive=$(readlink -f "$DP")"
  if [ "$MODE" = full ]; then   # slam · locate 는 바퀴 보드만
    port_ok "$AP" || { say "✗ 팔 보드 포트 $AP 를 열 수 없음 (없거나 권한 — make_udev_rules.py 또는 NT_ARM_PORT)"; exit 4; }
    [ "$(readlink -f "$DP")" != "$(readlink -f "$AP")" ] || { say "✗ 주행 · 팔 포트가 같은 장치를 가리킴"; exit 4; }
    PORTS="$PORTS,bus_arm=$(readlink -f "$AP")"
  fi
  ARGS+=("opencr_ports:=$PORTS")
  say "실물: $PORTS"
  if [ "${NT_YES:-0}" != 1 ]; then
    read -r -p "처음이면 바퀴를 띄웠나? 주변이 비었나? 시작하려면 y: " ok
    [ "$ok" = y ] || { say "취소"; exit 0; }
  fi
fi
for x in "$@"; do   # 위에서 검사한 값을 뒤 인자로 덮어쓰지 못하게 (같은 인자는 마지막 값이 이김)
  case "${x%%:=*}" in
    hardware|hardware_map|arm|opencr_port|opencr_ports|lidar_front_port|lidar_rear_port|slam_mode|map|use_sim_time|robot)
      say "✗ $x 는 덧붙일 수 없음 — 모드 · 하드웨어 · 포트는 이 스크립트의 인자와 NT_* 환경변수로"; exit 2;;
  esac
done
ARGS+=("$@")
say "ros2 launch $PKG $LAUNCH ${ARGS[*]}  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}) → $RUN"
# 별도 프로세스 그룹으로 띄우고 그룹 전체에 SIGINT (백그라운드 명령은 SIGINT 를 무시한 채 시작하므로 $! 에만 보내면 안 꺼짐)
set -m
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch "$PKG" "$LAUNCH" "${ARGS[@]}" > "$RUN/launch.log" 2>&1 &
LP=$!
set +m
stop_launch() {
  kill -INT -- -"$LP" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$LP" 2>/dev/null || break; sleep 0.5; done
  kill -0 "$LP" 2>/dev/null && { say "SIGINT 로 안 꺼져 SIGTERM"; kill -TERM -- -"$LP" 2>/dev/null; }
  say "끝"
}
trap stop_launch EXIT
trap 'exit 130' INT TERM
for i in $(seq 1 90); do
  grep -qa "activated swerve_controller" "$RUN/launch.log" && break
  kill -0 "$LP" 2>/dev/null || { say "✗ launch 가 먼저 끝남:"; tail -15 "$RUN/launch.log"; exit 5; }
  sleep 0.5
done
grep -qa "activated swerve_controller" "$RUN/launch.log" || { say "✗ 45 s 안에 swerve_controller 활성화 안 됨:"; tail -15 "$RUN/launch.log"; exit 5; }
grep -a -m1 "모듈별 조향 범위\|조향 영점 없음" "$RUN/launch.log" | sed 's/.*launch.user\]: //' | tee -a "$RUN/wrapper.log" || true
RANGES=$(grep -ao "/tmp/nt_steer_ranges_[A-Za-z0-9_]*\.yaml" "$RUN/launch.log" | head -1)
[ -n "$RANGES" ] && cp "$RANGES" "$RUN/steer_ranges.yaml" 2>/dev/null
ros2 param dump /swerve_controller > "$RUN/swerve_parameters.yaml" 2>/dev/null || true
say "준비됨 — /cmd_vel (TwistStamped) 로 움직임. $( [ "$MODE" = full ] && echo '시연 모드: 팔 HOME(/arm_task task 0) 성공 전에는 바퀴 잠금')"
say "Ctrl+C 로 끝"
wait "$LP"
