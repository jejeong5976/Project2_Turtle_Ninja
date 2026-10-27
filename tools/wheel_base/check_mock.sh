#!/usr/bin/env bash
# Local ROS simulation only. Every launch hard-codes hardware:=mock.
# Uses a private ROS home/domain and stops only its own timeout-wrapped launches.
set -eo pipefail
test_root=${1:?Usage: bash check_mock.sh VALIDATION_ROOT}
repo_root=$(cd "$(dirname "$0")/../.." && pwd)
source /opt/ros/lyrical/setup.bash
source "$test_root/install/setup.bash"
export ROS_DOMAIN_ID=218
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_HOME="$test_root/ros_home"
export ROS_LOG_DIR="$test_root/runtime_logs"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
launch_pid=
stop_launch() {
  if [ -n "$launch_pid" ]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    launch_pid=
  fi
}
trap stop_launch EXIT
start_launch() {
  local mode=$1
  timeout --signal=INT --kill-after=5s 45s ros2 launch bringup drive_only.launch.py \
    hardware:=mock mode:="$mode" > "$test_root/mock_$mode.log" 2>&1 &
  launch_pid=$!
  local controller=joint_state_broadcaster
  [ "$mode" = drive ] && controller=swerve_controller
  for attempt in $(seq 1 15); do
    if timeout 2s ros2 control list_controllers > "$test_root/controllers_$mode.txt" 2>/dev/null && \
      grep -Eq "$controller.*active" "$test_root/controllers_$mode.txt"; then
      return 0
    fi
    kill -0 "$launch_pid" 2>/dev/null || break
  done
  cat "$test_root/mock_$mode.log"
  return 1
}
start_launch feedback
timeout 5s ros2 control list_hardware_components > "$test_root/hardware_feedback.txt"
grep -q 'mock_components/GenericSystem' "$test_root/hardware_feedback.txt"
grep -q 'inactive' "$test_root/hardware_feedback.txt"
if grep -Eq 'swerve_controller|arm_controller|gripper_controller' "$test_root/controllers_feedback.txt"; then
  echo 'Unexpected controller in feedback mode'; exit 1
fi
timeout 7s ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 5 > "$test_root/feedback_joints.yaml"
stop_launch
start_launch drive
timeout 5s ros2 control list_hardware_components > "$test_root/hardware_drive.txt"
grep -q 'mock_components/GenericSystem' "$test_root/hardware_drive.txt"
grep -Eq 'label=active' "$test_root/hardware_drive.txt"
timeout 5s ros2 param dump /swerve_controller > "$test_root/runtime_controller.yaml"
timeout 8s ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.025}}}' --rate 10 --times 12 \
  > "$test_root/mock_command.log"
sleep 1
timeout 7s ros2 topic echo /odom nav_msgs/msg/Odometry --once --timeout 5 > "$test_root/mock_odom.yaml"
"$test_root/validate_profile" "$repo_root/config" "$test_root/validation/drive_only_opencr.urdf" \
  "$test_root/runtime_controller.yaml" "$test_root/mock_odom.yaml"
stop_launch
echo 'PASS: mock feedback keeps bus inactive; mock drive has calibrated limits, +X odometry, and command-timeout stop.'
