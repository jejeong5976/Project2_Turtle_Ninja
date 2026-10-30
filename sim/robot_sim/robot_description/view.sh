#!/usr/bin/env bash
# 보기(관절 슬라이더 + RViz). 빌드 · ROS 패키지 없이 실행 (robot.urdf 직접 읽음)
cd "$(dirname "$0")"; source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=219 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${TMPDIR:-/tmp}/nt_release_ros_logs"
exec ros2 launch "$PWD/launch/view.launch.py" "$@"
