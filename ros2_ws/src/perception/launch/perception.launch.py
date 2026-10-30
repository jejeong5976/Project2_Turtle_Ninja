"""
인지 노드 실행: 노드 하나(perception)가 광각 + 손목 탐지와 바코드 판독을 모두 맡음.
카메라 드라이버는 따로 띄운다 (광각 /wide_cam/*, RealSense /wrist_cam/*, align_depth 켜기).

  ros2 launch perception perception.launch.py \
      wide_model:=$HOME/models/wide_fp16.onnx provider:=cuda

provider: cuda (기본) | tensorrt (Jetson, 처음 실행 때 엔진을 만들어 모델 옆에 저장) | cpu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    params = PathJoinSubstitution([FindPackageShare("perception"), "config", "perception.yaml"])

    args = [
        DeclareLaunchArgument("params", default_value=params),
        DeclareLaunchArgument("wide_model", default_value="", description=".onnx"),
        DeclareLaunchArgument("wrist_model", default_value="", description="비우면 wide_model 사용"),
        DeclareLaunchArgument("provider", default_value="cuda", description="cuda | tensorrt | cpu"),
        DeclareLaunchArgument("use_barcode", default_value="true"),
    ]

    perception = Node(
        package="perception", executable="perception_node", name="perception", output="screen",
        parameters=[arg("params"), {
            "wide.model": arg("wide_model"),
            "wrist.model": arg("wrist_model"),
            "provider": arg("provider"),
            "barcode.enabled": arg("use_barcode"),
        }],
    )
    return LaunchDescription(args + [perception])
