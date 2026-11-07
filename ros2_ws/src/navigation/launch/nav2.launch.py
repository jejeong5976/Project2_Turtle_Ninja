"""nav2.launch.py — Nav2 실행 (planner · controller(MPPI) · behavior · bt_navigator · collision_monitor
                                 + localization:=amcl 이면 map_server · AMCL)

  # 실물 (기본 localization:=slam): slam.launch.py 를 먼저 띄움 -> map -> odom · /map 은 slam_toolbox
  ros2 launch navigation slam.launch.py hardware:=opencr slam_mode:=mapping map:=$HOME/maps/arena   # 시연: 저장 지도 이어 그리기
  ros2 launch navigation nav2.launch.py
  ros2 launch navigation nav2.launch.py mission_hold:=false                                  # planning 없이 Nav2 만 시험
  # AMCL (시뮬 · slam_toolbox 없이): map_server + AMCL 이 map -> odom · /map
  ros2 launch navigation nav2.launch.py localization:=amcl map:=<지도.yaml> use_sim_time:=true rviz:=true
  ros2 launch navigation nav2.launch.py localization:=amcl map:=<지도.yaml> initial_pose:=-1.468,-2.202,180  # 출발 자세로 AMCL 시작

  - 설정 yaml: params 인자 -> 레포 config/nav2.yaml (bringup 이 share/bringup/config 로 설치)
  - localization (기본 slam): map -> odom 주인은 하나만
      slam = 이 launch 는 위치 찾기를 안 띄움. slam_toolbox 가 냄 (2026-11-05 시연 = mapping 으로 저장 posegraph 이어 그리기,
             지도 고정이 필요하면 slam_mode:=localization. 둘 다 시작 자세 = 매핑 시작 자세 = 출발 자세)
             map · initial_pose 인자는 안 씀
      amcl = map_server (map:=<지도.yaml>) + AMCL (initial_pose). slam_toolbox 를 같이 띄우면 안 됨
  - 지도 (amcl): slam_toolbox 로 만든 map.yaml (save_map.sh) 또는 시뮬 시험용 sim/robot_sim/maps/maze_dice_map.yaml
  - nav2_bringup 기본 navigation_launch 는 route · docking · following · velocity_smoother 까지 띄움
    -> 쓰는 노드만 띄움. controller · behavior -> cmd_vel_nav -> collision_monitor (비상 정지) -> cmd_vel_safe
       -> cmd_vel_hold (/mission/hold 가 true 면 0) -> /cmd_vel (TwistStamped)
  - mission_hold (기본 true, 미션 실행): hold 를 받기 전에도 막음 (start_hold), hold 가 2.5 s 넘게 안 오면 막음
    (mission_manager 가 죽으면 멈춤, hold 는 1 Hz). false = planning 없이 Nav2 만 시험할 때 (그냥 넘김)
  - 운영 규칙: Nav2 를 띄운 동안 teleop 금지 (/cmd_vel 직접 발행은 hold · collision_monitor 를 우회하고 0 과 섞임).
    매핑 · 수동 시험은 Nav2 없이
       (가속 한계는 swerve_controller 가 지킴)
  - 받는 것: /odom, TF odom -> base_link (swerve_controller · 시뮬 브리지), /lidar_front/scan, /lidar_rear/scan (costmap · AMCL), /scan (collision_monitor, scan_merger 는 이 launch 가 안 띄움 → slam.launch.py · 시뮬 launch)
  - 받는 것 (slam): TF map -> odom · /map (slam_toolbox)
  - 보내는 것: /cmd_vel, TF map -> odom (amcl 일 때만), navigate_to_pose 액션 (mission_manager 가 부름)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

LOCALIZATION = ["map_server", "amcl"]
NAVIGATION = ["planner_server", "controller_server", "behavior_server", "bt_navigator", "collision_monitor"]
# controller · behavior 의 속도 명령은 collision_monitor (비상 정지) 를 거쳐 /cmd_vel 로 나감
TO_MONITOR = [("cmd_vel", "cmd_vel_nav")]
# /mission/hold 가 이만큼 안 오면 막음 (mission_manager 는 1 Hz + 바뀔 때 발행)
HOLD_TIMEOUT_S = 2.5


def bringup_params():
    """bringup share 에 설치된 config/nav2.yaml (원본: 레포 config/) 경로, 없으면 빈 문자열"""
    try:
        path = os.path.join(get_package_share_directory("bringup"), "config", "nav2.yaml")
    except PackageNotFoundError:
        return ""
    return path if os.path.isfile(path) else ""


def make_nodes(context):
    params = LaunchConfiguration("params").perform(context) or bringup_params()
    if not params:
        raise RuntimeError("nav2 설정 yaml 없음: params:=<파일> 을 주거나 bringup 을 빌드할 것 (레포 config/nav2.yaml)")
    loc = LaunchConfiguration("localization").perform(context).strip().lower()
    if loc not in ("slam", "amcl"):
        raise RuntimeError(f"localization 은 slam | amcl (지금 {loc})")
    map_yaml = LaunchConfiguration("map").perform(context)
    if loc == "amcl" and not map_yaml:
        raise RuntimeError("localization:=amcl 이면 map:=<지도.yaml> 필요")
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    mission_hold = LaunchConfiguration("mission_hold").perform(context).lower() == "true"
    # allow_substs: yaml 안 $(find-pkg-share nav2_bt_navigator) 같은 치환을 풀어 줌 (bt_navigator 행동 트리 폴더)
    common = [ParameterFile(params, allow_substs=True), {"use_sim_time": sim}]
    # 출발 자세 (map 기준 "x,y,yaw_deg"): 주면 AMCL 이 그 자세로 시작 (출발 구역 고정, 발제). 비우면 /initialpose 를 기다림
    init = LaunchConfiguration("initial_pose").perform(context).strip()
    amcl_init = []
    if loc == "slam" and (init or map_yaml):
        # slam_toolbox 가 지도 · 시작 자세를 정함 -> 받아도 버림 (조용히 무시하면 지도가 다른 줄 모름)
        print("[nav2.launch] localization:=slam -> map · initial_pose 인자 안 씀 (slam.launch.py 의 map 을 쓸 것)")
    if loc == "amcl" and init:
        x, y, yaw = (float(v) for v in init.split(","))
        amcl_init = [{"set_initial_pose": True,
                      "initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": yaw * 3.141592653589793 / 180.0}}]
    out = "screen"
    nodes = []
    if loc == "amcl":
        nodes += [
            Node(package="nav2_map_server", executable="map_server", name="map_server", output=out,
                 parameters=common + [{"yaml_filename": map_yaml}]),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output=out, parameters=common + amcl_init),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 output=out, parameters=[{"use_sim_time": sim, "autostart": True, "node_names": LOCALIZATION}]),
        ]
    nodes += [
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output=out, parameters=common),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output=out,
             parameters=common, remappings=TO_MONITOR),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output=out,
             parameters=common, remappings=TO_MONITOR),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output=out,
             parameters=common),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output=out,
             parameters=common),
        # planning /mission/hold → 막는 중엔 Nav2 명령 버리고 0
        Node(package="navigation", executable="cmd_vel_hold", name="cmd_vel_hold", output=out,
             parameters=[{"use_sim_time": sim, "cmd_vel_in": "cmd_vel_safe", "cmd_vel_out": "/cmd_vel",
                          "start_hold": mission_hold, "hold_timeout_s": HOLD_TIMEOUT_S if mission_hold else 0.0}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output=out, parameters=[{"use_sim_time": sim, "autostart": True, "node_names": NAVIGATION}]),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        rviz_cfg = os.path.join(get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz")
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
                          parameters=[{"use_sim_time": sim}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("localization", default_value="slam",
                              description="slam = map -> odom 은 slam_toolbox (slam.launch.py) / amcl = map_server + AMCL"),
        DeclareLaunchArgument("map", default_value="", description="지도 yaml (localization:=amcl 일 때 map_server)"),
        DeclareLaunchArgument("params", default_value="", description="비우면 레포 config/nav2.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("initial_pose", default_value="", description="출발 자세 map 기준 'x,y,yaw_deg' (localization:=amcl 일 때만)"),
        DeclareLaunchArgument("mission_hold", default_value="true",
                              description="true = /mission/hold 받기 전 · 끊기면 막음 (미션). false = 그냥 넘김 (Nav2 단독 시험)"),
        OpaqueFunction(function=make_nodes),
    ])
