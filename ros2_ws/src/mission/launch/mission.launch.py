"""mission.launch.py — 미션 관리 노드 mission_manager 실행

역할
  - mission_manager 하나를 띄움, 실행기는 planning 패키지 라이브러리로 같은 노드 안에서 돎
  - 설정 yaml 을 골라 넘기고, 인자로 준 값만 덮어씀
입력: launch 인자
  - params: 설정 yaml. 비우면 레포 config/planning.yaml, 그것도 없으면 코드 기본값
  - dice_map: 경기장 좌표 파일. 비우면 빌드 때 정한 레포 maps/dice_map.yaml
  - require_motion_mode: 비우면 yaml · 코드 값 true
  - localization: slam | amcl, nav2.launch.py 와 같은 이름. slam → localization_source health, amcl → amcl. 비우면 yaml · 코드 값 health
  - use_sim_time: 기본 false
출력
  - 노드 /mission_manager. 토픽 · 서비스는 mission_manager.cpp 참고
실행
  ros2 launch mission mission.launch.py
  ros2 launch mission mission.launch.py use_sim_time:=true require_motion_mode:=false
  ros2 launch mission mission.launch.py use_sim_time:=true localization:=amcl     # 시뮬 (nav2 localization:=amcl 과 같이)
  ros2 launch mission mission.launch.py dice_map:=<다른 파일> params:=<다른 yaml>
note
  - dice_map · require_motion_mode · localization 은 인자를 준 경우에만 yaml 값을 덮어씀
  - 레포 config/planning.yaml 은 bringup 이 share/bringup/config 로 설치함
  - dice_map 은 경기장마다 바뀌는 파일이라 install 밖 레포 maps/ 에 둠. 지도 폴더는 CMake -DMISSION_MAPS_DIR
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bringup_params():
    """bringup share 에 설치된 config/planning.yaml (원본: 레포 config/) 경로, 없으면 빈 문자열"""
    try:
        path = os.path.join(get_package_share_directory("bringup"), "config", "planning.yaml")
    except PackageNotFoundError:
        return ""
    return path if os.path.isfile(path) else ""


def make_node(context):
    params = LaunchConfiguration("params").perform(context) or bringup_params()
    dice_map = LaunchConfiguration("dice_map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    require_mode = LaunchConfiguration("require_motion_mode").perform(context)
    loc = LaunchConfiguration("localization").perform(context).strip().lower()
    if loc not in ("", "slam", "amcl"):
        raise RuntimeError(f"localization 은 slam | amcl (지금 {loc})")

    overrides = {"use_sim_time": use_sim_time}
    if dice_map:
        overrides["dice_map"] = dice_map
    if require_mode:
        overrides["require_motion_mode"] = require_mode.lower() == "true"
    if loc:
        overrides["localization_source"] = "health" if loc == "slam" else "amcl"

    parameters = ([params] if params else []) + [overrides]
    return [
        LogInfo(msg=f"planning params: {params or '(없음, 코드 기본값)'}"),
        Node(
            package="mission",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value="", description="비우면 레포 config/planning.yaml → 없으면 코드 기본값"),
        DeclareLaunchArgument("dice_map", default_value="", description="비우면 레포 maps/dice_map.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("require_motion_mode", default_value="", description="비우면 yaml · 코드 값 (true)"),
        DeclareLaunchArgument("localization", default_value="", description="slam | amcl, 비우면 yaml · 코드 값 (health = slam)"),
        OpaqueFunction(function=make_node),
    ])
