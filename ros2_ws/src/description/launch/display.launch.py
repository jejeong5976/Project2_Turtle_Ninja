"""
display.launch.py — 로봇 모양만 RViz 로 보기
=====================================================
모터 · 컨트롤러 없이 URDF 만 확인할 때. 슬라이더 창으로 관절을 손으로 움직여 볼 수 있음.

  ros2 launch description display.launch.py [arm_mount_x:=0.085] [with_accessories:=false]
  → RViz 는 rviz/navigation.rviz 를 base_footprint 기준으로 염 (지도 · 라이다 표시는 비어 있음)
  ⚠ 슬라이더 시작값은 0 (팔이 펴진 자세). 주행 자세(HOME)는 hardware_map.yaml 의 initial 값
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("description"), "urdf", "robot.urdf.xacro"]),
            " hardware:=mock",
            " arm_mount_x:=", LaunchConfiguration("arm_mount_x"),
            " arm_mount_y:=", LaunchConfiguration("arm_mount_y"),
            " with_accessories:=", LaunchConfiguration("with_accessories"),
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("arm_mount_x", default_value="0.0", description="팔 받침 x [m] — 미확정"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0", description="팔 받침 y [m] — 미확정"),
        DeclareLaunchArgument("with_accessories", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=[
            "-d", PathJoinSubstitution([FindPackageShare("description"), "rviz", "navigation.rviz"]),
            "-f", "base_footprint"]),
    ])
