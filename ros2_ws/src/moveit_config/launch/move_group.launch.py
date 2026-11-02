"""
팔 MoveIt 계획 서버 (move_group) + rviz.

  # 로봇(또는 mock)을 먼저 띄운 뒤 — joint_states · 컨트롤러는 bringup 이 냄
  ros2 launch bringup robot.launch.py hardware:=mock
  ros2 launch moveit_config move_group.launch.py            # 계획만 (기본)
  ros2 launch moveit_config move_group.launch.py use_rviz:=true

execute:=false (기본): 계획만 하고 실행은 막음. 바퀴 · 팔 차례는 control_master 가 관리하므로
  MoveIt 이 arm_controller 에 직접 보내면 바퀴가 움직이는 중에 팔이 움직일 수 있음 → 실행은 control_master 를 거치게 연결할 것.
execute:=true 는 mock · 바퀴를 세운 시험에서만.
관절 상태 = /joint_states_moveit (manipulation/arm_state_publisher: 엔코더 값 + 그리퍼 연결 막대 3 관절). bringup 없이 띄우면 joint_states:=joint_states
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _nodes(context):
    execute = LaunchConfiguration("execute").perform(context) == "true"
    hw_map = LaunchConfiguration("hardware_map").perform(context) or os.path.join(
        get_package_share_directory("bringup"), "config", "hardware_map.yaml")
    xacro = os.path.join(get_package_share_directory("description"), "urdf", "robot.urdf.xacro")
    cfg = (MoveItConfigsBuilder("ninja_turtle", package_name="moveit_config")
           .robot_description(file_path=xacro, mappings={"hardware_map": hw_map})
           .robot_description_semantic(file_path="config/ninja_turtle.srdf")
           .robot_description_kinematics(file_path="config/kinematics.yaml")
           .joint_limits(file_path="config/joint_limits.yaml")
           .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
           .planning_pipelines(pipelines=["ompl"])
           .to_moveit_configs())
    js = LaunchConfiguration("joint_states").perform(context)
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      remappings=[("joint_states", js)],
                      parameters=[cfg.to_dict(), {"allow_trajectory_execution": execute,
                                                  "publish_robot_description_semantic": True}])
    out = [move_group]
    if LaunchConfiguration("use_rviz").perform(context) == "true":
        out.append(Node(package="rviz2", executable="rviz2", output="log", remappings=[("joint_states", js)],
                        arguments=["-d", os.path.join(get_package_share_directory("moveit_config"), "config", "moveit.rviz")],
                        parameters=[cfg.robot_description, cfg.robot_description_semantic,
                                    cfg.robot_description_kinematics, cfg.planning_pipelines, cfg.joint_limits]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false", choices=["true", "false"],
                              description="true = arm_controller 로 직접 실행 (mock · 바퀴 세운 시험 전용)"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hardware_map", default_value="", description="비우면 bringup 의 hardware_map.yaml"),
        DeclareLaunchArgument("joint_states", default_value="joint_states_moveit",
                              description="MoveIt 이 읽는 관절 상태 (엔코더 + 그리퍼 연결 막대). arm_state_publisher 없이 띄우면 joint_states"),
        OpaqueFunction(function=_nodes),
    ])
