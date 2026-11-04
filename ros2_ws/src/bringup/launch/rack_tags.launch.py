"""
적재함 칸 바닥 AprilTag 재기 (manipulation rack_tag_node) — 팔은 안 움직임, 손목 카메라 영상 · TF 만 읽음.
robot.launch.py (TF) 와 손목 카메라 드라이버가 떠 있어야 함. 칸 위치 · 태그 ID 는 config/rack_cells.yaml 에서 읽음.

  ros2 launch bringup rack_tags.launch.py
  팔을 멈춘 채 칸 태그가 보이게 둔 뒤 (칸마다 몇 초):
  ros2 service call /rack_tag_node/report std_srvs/srv/Trigger    # 칸별 차 · 적재함 전체 이동 · 칸별 옮길 양
  ros2 service call /rack_tag_node/reset std_srvs/srv/Trigger     # 모은 값 지움
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    path = os.path.join(get_package_share_directory("bringup"), "config", "rack_cells.yaml")
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    rack, tags = data["rack_cells"], data["rack_tags"]
    names = list(rack["cells"])
    missing = [n for n in names if n not in tags["ids"]]
    if missing:
        raise RuntimeError(f"rack_cells.yaml rack_tags.ids 에 칸이 빠짐: {missing}")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("min_view_cos", default_value="0.94",
                              description="태그 광선이 수직에서 이 cos 안이어야 씀 (0.94 = 20°, 비스듬할수록 각도 오차가 커짐)"),
        DeclareLaunchArgument("camera_frame", default_value="d435_color_optical_frame",
                              description="TF 조회 이름 (URDF). 드라이버 영상 header 는 camera_color_optical_frame 이라 덮어씀"),
        Node(package="manipulation", executable="rack_tag_node", output="both", parameters=[{
            "image_topic": arg("image_topic"),
            "info_topic": arg("info_topic"),
            "camera_frame": arg("camera_frame"),
            "min_view_cos": ParameterValue(arg("min_view_cos"), value_type=float),
            "floor_z": float(rack["floor_z"]),
            "tag_size_m": float(tags["size_m"]),
            "cell_names": names,
            "cell_x": [float(rack["cells"][n]["xy"][0]) for n in names],
            "cell_y": [float(rack["cells"][n]["xy"][1]) for n in names],
            "cell_ids": [int(tags["ids"][n]) for n in names],
        }]),
    ])
