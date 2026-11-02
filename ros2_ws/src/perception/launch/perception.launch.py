"""perception.launch.py — 인지 노드 perception 실행

역할
  - perception_node 하나를 띄움: 광각 · 손목 · 신호등 · 바코드
  - 설정 yaml 을 골라 넘기고, 인자로 준 값만 덮어씀
입력: launch 인자
  - params: 설정 yaml. 비우면 레포 config/perception.yaml, 그것도 없으면 코드 기본값
  - wide_model · wrist_model: TensorRT .engine 경로
  - publish_debug: true 면 /perception/wrist_debug_image 발행
  - use_sim_time: 기본 false
출력
  - 노드 /perception. 토픽 · 서비스는 perception_node.cpp 참고
실행
  ros2 launch perception perception.launch.py wide_model:=<광각>.engine wrist_model:=<손목>.engine
  ros2 launch perception perception.launch.py wrist_model:=<손목>.engine publish_debug:=true use_sim_time:=true
note
  - wide_model · wrist_model · publish_debug 는 인자를 준 경우에만 yaml 값을 덮어씀
  - params 에 없는 파일을 주면 오류
  - 레포 config/perception.yaml 은 bringup 이 share/bringup/config 로 설치함, yaml 고르는 순서는 mission.launch.py 와 같음
  - 엔진 옆에 같은 이름 .names.txt 필요. 만드는 법은 trt_engine.hpp
  - 카메라 드라이버는 따로 띄움: 광각 /wide_cam/*, RealSense align_depth.enable:=true → /camera/camera/*
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bringup_params():
    """bringup share 에 설치된 config/perception.yaml (원본: 레포 config/) 경로, 없으면 빈 문자열"""
    try:
        path = os.path.join(get_package_share_directory("bringup"), "config", "perception.yaml")
    except PackageNotFoundError:
        return ""
    return path if os.path.isfile(path) else ""


def make_node(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    params = arg("params")
    if params:
        params = os.path.abspath(os.path.expanduser(params))
        if not os.path.isfile(params):
            raise RuntimeError(f"params 파일 없음: {params}")
    else:
        params = bringup_params()

    overrides = {"use_sim_time": arg("use_sim_time").lower() == "true"}
    for launch_arg, key in (("wide_model", "wide.model"), ("wrist_model", "wrist.model")):
        if arg(launch_arg):
            overrides[key] = os.path.abspath(os.path.expanduser(arg(launch_arg)))
    if arg("publish_debug"):
        overrides["wrist.publish_debug"] = arg("publish_debug").lower() == "true"

    parameters = ([params] if params else []) + [overrides]
    return [
        LogInfo(msg=f"perception params: {params or '(없음, 코드 기본값)'}"),
        Node(
            package="perception",
            executable="perception_node",
            name="perception",
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value="", description="비우면 레포 config/perception.yaml → 없으면 코드 기본값"),
        DeclareLaunchArgument("wide_model", default_value="", description="광각 상자 검출 .engine"),
        DeclareLaunchArgument("wrist_model", default_value="", description="손목 퍽 분할 .engine"),
        DeclareLaunchArgument("publish_debug", default_value="", description="true 면 /perception/wrist_debug_image"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=make_node),
    ])
