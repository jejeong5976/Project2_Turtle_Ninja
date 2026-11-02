"""sim_nav2.launch.py — 시뮬 Nav2 시험: robot_state_publisher(배포판 URDF) + navigation/nav2.launch.py (use_sim_time) + RViz

  먼저 시뮬:  sim/robot_sim/scripts/sim_nav_bridge.sh [--viz kit --camera --real]
  그다음:    ros2 launch sim/robot_sim/scripts/sim_nav2.launch.py [map:=<지도.yaml>] [rviz:=false]
  기본 지도 = sim/robot_sim/maps/maze_dice_map.yaml (tools/layout_to_occupancy.py --maze dice)
  navigation 패키지가 빌드 · source 돼 있어야 함. 설정 = 레포 config/nav2.yaml (params:= 로 바꿈)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # sim/robot_sim
DESC = os.path.join(SIM, "robot_description")
REPO_NAV2 = os.path.join(SIM, "..", "..", "config", "nav2.yaml")   # 레포 config/nav2.yaml


def dice_start():
    import json
    m = json.load(open(os.path.join(SIM, "mazes", "dice_layout.json")))["mission"]["start_pose"]["sim_m_deg"]
    return f"{m[0]},{m[1]},{m[2]}"


def rsp(context):
    # robot.urdf 의 상대 메시 경로 -> file:// 절대 경로 (RViz 표시용, robot_description/launch/view.launch.py 와 같음)
    urdf = open(os.path.join(DESC, "robot.urdf")).read().replace('filename="', 'filename="file://' + DESC + "/")
    return [Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
                 parameters=[{"robot_description": urdf, "use_sim_time": True}])]


def generate_launch_description():
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("navigation"), "launch", "nav2.launch.py")),
        launch_arguments={"map": LaunchConfiguration("map"), "use_sim_time": "true",
                          "rviz": LaunchConfiguration("rviz"), "params": LaunchConfiguration("params"),
                          "initial_pose": LaunchConfiguration("initial_pose")}.items())
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(SIM, "maps", "maze_dice_map.yaml")),
        DeclareLaunchArgument("params", default_value=os.path.normpath(REPO_NAV2)),
        DeclareLaunchArgument("rviz", default_value="true"),
        # 기본 = 주사위 경기장 출발 자세 (dice_layout.json mission.start_pose: 발제 (734, 950) mm, −X 방향)
        DeclareLaunchArgument("initial_pose", default_value=dice_start()),
        OpaqueFunction(function=rsp),
        nav,
    ])
