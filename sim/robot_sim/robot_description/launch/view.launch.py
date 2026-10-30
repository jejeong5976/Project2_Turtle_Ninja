# 보기 전용: robot_state_publisher + (선택) 관절 슬라이더 + RViz. 실물 제어 노드 없음.
import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 이 폴더(robot_description/). ROS 패키지 없이 robot.urdf 의 상대 메시 경로를 file:// 절대 경로로 바꿔 RViz 에 넘김
share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def make(ctx):
    desc = open(os.path.join(share, "robot.urdf")).read().replace('filename="', 'filename="file://' + share + "/")
    gui = LaunchConfiguration("gui").perform(ctx) == "true"
    rviz = LaunchConfiguration("rviz").perform(ctx)
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": desc}])]
    j5 = float(LaunchConfiguration("j5").perform(ctx))   # 캡처용: 손목 롤 시작값 (D435 가 따라 도는지 보기)
    cfg = json.load(open(os.path.join(share, "config", "initial_pose.json")))
    pose = LaunchConfiguration("pose").perform(ctx)
    if pose not in ("parked", "p0"):
        raise ValueError("pose must be parked or p0")
    initial = dict(cfg["positions"] if pose == "parked" else cfg["poses"]["C1_P0"]["q_rad"])
    if pose == "p0":
        initial["gripper_finger_a_joint"] = cfg["positions"]["gripper_finger_a_joint"]
    initial["arm_joint5"] = initial.get("arm_joint5", 0.0) + j5
    nodes.append(Node(package="joint_state_publisher_gui" if gui else "joint_state_publisher",
                      executable="joint_state_publisher_gui" if gui else "joint_state_publisher",
                      parameters=[{"zeros": initial}]))
    if rviz != "false":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "view.rviz")]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("pose", default_value="parked"), DeclareLaunchArgument("gui", default_value="true"),
                              DeclareLaunchArgument("rviz", default_value="true"), DeclareLaunchArgument("j5", default_value="0.0"), OpaqueFunction(function=make)])
