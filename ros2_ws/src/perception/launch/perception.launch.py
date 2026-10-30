"""
인지 노드 실행: 탐지(광각 + 손목, 한 프로세스) + 바코드 판독.
카메라 드라이버는 따로 띄운다 (광각 /wide_cam/*, RealSense /wrist_cam/*, align_depth 켜기).

  ros2 launch perception perception.launch.py \
      wide_model:=$HOME/models/wide_736x1280.engine wrist_model:=$HOME/models/wrist_480x640.engine

개발 PC 에서는 .pt 를 그대로 넣어도 됨 (device:=cpu 가능).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    params = PathJoinSubstitution([FindPackageShare("perception"), "config", "perception.yaml"])

    args = [
        DeclareLaunchArgument("params", default_value=params),
        DeclareLaunchArgument("wide_model", default_value="", description=".engine 또는 .pt"),
        DeclareLaunchArgument("wrist_model", default_value="", description="비우면 wide_model 사용"),
        DeclareLaunchArgument("device", default_value="0", description="0 (GPU) | cpu"),
        DeclareLaunchArgument("use_barcode", default_value="true"),
    ]

    detectors = Node(
        package="perception", executable="detectors", output="screen",
        # dict 파라미터는 두 노드 모두에 들어감 → 노드가 자기 camera 에 맞는 *_model 을 고름
        parameters=[arg("params"), {
            "wide_model": arg("wide_model"),
            "wrist_model": arg("wrist_model"),
            "device": arg("device"),
        }],
    )
    barcode = Node(
        package="perception", executable="barcode", name="barcode", output="screen",
        parameters=[arg("params")],
        condition=IfCondition(arg("use_barcode")),
    )
    return LaunchDescription(args + [detectors, barcode])
