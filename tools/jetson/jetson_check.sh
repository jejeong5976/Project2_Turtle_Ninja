#!/usr/bin/env bash
# 젯슨 상태 확인 — 읽기만 함. 아무것도 설치 · 변경하지 않음.
#   bash tools/jetson/jetson_check.sh            (결과를 그대로 복사해서 노트북에서 확인)
# 보는 것: OS · CPU 종류 · ROS 배포판 · 빌드 도구 · 필요한 ROS 패키지 · 시리얼 포트 · dialout 권한 · 포트 이름 고정 · 전원 모드 · 메모리 · 디스크
ROS_DISTRO_WANT=${ROS_DISTRO_WANT:-lyrical}
ok() { printf "  ✓ %s\n" "$*"; }
no() { printf "  ✗ %s\n" "$*"; }
echo "== 기본"
. /etc/os-release; echo "  OS: $PRETTY_NAME · $(uname -m) · 커널 $(uname -r)"
[ -f /etc/nv_tegra_release ] && echo "  젯슨: $(head -1 /etc/nv_tegra_release)"
command -v nvpmodel >/dev/null && echo "  전원 모드: $(nvpmodel -q 2>/dev/null | head -1)"
echo "  메모리: $(LC_ALL=C free -h | awk '/Mem/{print $2" 중 "$7" 남음"}') · 스왑: $(LC_ALL=C free -h | awk '/Swap/{print $2}') · 디스크(/): $(df -h / | awk 'NR==2{print $4" 남음"}')"
echo "== ROS"
out=$(ls /opt/ros 2>/dev/null); [ -n "$out" ] && echo "$out" | sed 's/^/  설치된 배포판: /' || no "/opt/ros 없음"
if [ -f /opt/ros/$ROS_DISTRO_WANT/setup.bash ]; then ok "$ROS_DISTRO_WANT 있음"; . /opt/ros/$ROS_DISTRO_WANT/setup.bash; else no "$ROS_DISTRO_WANT 없음 (노트북은 lyrical) — 다른 배포판이면 알려 주세요"; fi
for c in colcon rosdep xacro; do command -v $c >/dev/null && ok "$c" || no "$c 없음"; done
echo "== 필요한 ROS 패키지 (우리 package.xml 의존성)"
miss=()
for p in rosidl_default_generators std_msgs controller_interface controller_manager controller_manager_msgs hardware_interface pluginlib rclcpp rclcpp_action rclcpp_lifecycle \
         joint_state_broadcaster joint_trajectory_controller imu_sensor_broadcaster robot_state_publisher tf2_ros xacro \
         control_msgs nav_msgs sensor_msgs geometry_msgs trajectory_msgs action_msgs lifecycle_msgs rosidl_default_runtime ament_cmake_gtest; do
  ros2 pkg prefix $p >/dev/null 2>&1 || miss+=($p)
done
[ ${#miss[@]} -eq 0 ] && ok "모두 있음" || no "없음: ${miss[*]}"
python3 -c "import yaml" 2>/dev/null && ok "python3-yaml" || no "python3-yaml 없음"
echo "== 시리얼 포트 · 권한"
id -nG | grep -qw dialout && ok "$(whoami) 가 dialout 그룹 (포트 열기 가능)" || no "$(whoami) 가 dialout 그룹이 아님 → sudo usermod -aG dialout $(whoami) 후 다시 로그인 (udev 규칙 GROUP=dialout)"
out=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null); [ -n "$out" ] && echo "$out" | sed 's/^/  /' || echo "  꽂힌 시리얼 장치 없음"
for n in opencr_drive opencr_arm; do [ -e /dev/$n ] && ok "/dev/$n → $(readlink -f /dev/$n)" || no "/dev/$n 없음 (포트 고정 규칙 필요: tools/jetson/make_udev_rules.py)"; done
echo "== SLAM · 지연"
for p in slam_toolbox coin_d4_driver; do ros2 pkg prefix $p >/dev/null 2>&1 && ok "$p" || no "$p 없음 → sudo apt install ros-$ROS_DISTRO_WANT-${p//_/-}"; done
ls -l /dev/serial/by-path/ 2>/dev/null | sed 's/^/  /'   # 앞 · 뒤 라이다 · 주행 보드 USB 자리 (run_robot.sh 의 NT_LIDAR_FRONT · NT_LIDAR_REAR · NT_DRIVE_PORT)
# ros2_control 100 Hz 루프의 실시간 우선순위: 0 이면 "Could not enable FIFO RT scheduling" 경고 + 주기 흔들림 가능 (control.ros.org 실시간 안내)
r=$(ulimit -r); [ "$r" != 0 ] && ok "실시간 우선순위 한도 rtprio=$r" || no "rtprio 0 — 제어 루프가 보통 우선순위로 돎 (필요하면 /etc/security/limits.d 에 rtprio, 사람이 직접)"
echo "== 다른 ROS 실행 중인지"
out=$(pgrep -af "lib/controller_manager/ros2_control_[n]ode|ros2 [l]aunch"); [ -n "$out" ] && echo "$out" | sed 's/^/  /' || ok "없음"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(없음=0)}"
