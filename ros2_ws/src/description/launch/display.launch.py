"""
display.launch.py — 로봇 모양만 RViz 로 보기
=====================================================
모터 · 컨트롤러 없이 URDF 만 확인할 때. 슬라이더 창으로 관절을 손으로 움직여 볼 수 있음.

  ros2 launch description display.launch.py [arm_mount_x:=0.085] [with_accessories:=false]
  → RViz 는 rviz/navigation.rviz 를 base_footprint 기준으로 염 (지도 · 라이다 표시는 비어 있음)
  ⚠ 슬라이더 시작값은 0 (팔이 펴진 자세). 주행 자세(HOME)는 hardware_map.yaml 의 initial 값
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _slider_gui(context, description_command):
    # JSP는 robot_description 파라미터를 읽지 않는다. 슬라이더용 URDF 파일을 직접 넘긴다.
    description = description_command.perform(context)
    fd, path = tempfile.mkstemp(prefix="gripper_sliders_", suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(description)

    def cleanup(_context):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             arguments=[path],
             # 전체 모델 토픽이 수동 관절 없는 슬라이더 모델을 덮어쓰지 않게 한다.
             remappings=[("robot_description", "gripper_slider_robot_description")]),
    ]


def generate_launch_description():
    def description_command(with_linkage):
        return Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("description"), "urdf", "robot.urdf.xacro"]),
            " hardware:=mock",
            " hardware_map:=", LaunchConfiguration("hardware_map"),
            " arm_mount_x:=", LaunchConfiguration("arm_mount_x"),
            " arm_mount_y:=", LaunchConfiguration("arm_mount_y"),
            " with_accessories:=", LaunchConfiguration("with_accessories"),
            " with_gripper_linkage:=", with_linkage,
        ])

    robot_description = ParameterValue(description_command("true"), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("hardware_map", default_value=PathJoinSubstitution([
            FindPackageShare("bringup"), "config", "hardware_map.yaml"])),
        DeclareLaunchArgument("arm_mount_x", default_value="0.0", description="팔 받침 x [m] — 미확정"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0", description="팔 받침 y [m] — 미확정"),
        DeclareLaunchArgument("with_accessories", default_value="true"),
        # 슬라이더 관절 상태 + 그리퍼 연결 막대 계산값 → joint_states_corrected (실물과 같은 C++ 노드, 처짐 보정은 끔)
        Node(package="manipulation", executable="arm_state_publisher",
             parameters=[PathJoinSubstitution([FindPackageShare("bringup"), "config", "gripper_linkage.yaml"]),
                         {"sag_correction": False}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}],
             remappings=[("joint_states", "joint_states_corrected")]),
        OpaqueFunction(function=_slider_gui,
                       kwargs={"description_command": description_command("false")}),
        Node(package="rviz2", executable="rviz2", arguments=[
            "-d", PathJoinSubstitution([FindPackageShare("description"), "rviz", "navigation.rviz"]),
            "-f", "base_footprint"]),
    ])
