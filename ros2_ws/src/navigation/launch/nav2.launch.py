"""nav2.launch.py — Nav2 실행 (planner · collision_monitor + 지역 계획기 + localization:=amcl 이면 map_server · AMCL)
  지역 계획기 local_planner:
    rl (기본) = RL 노드 둘 (navigation/scripts/local_planner_node.py, 정책 레포 config/rl/):
                주행 rl_nav = navigate_to_pose (nf5) · 정밀 접근 rl_dock = dock_to_pose (dock3, planning dock_action)
                controller(MPPI) · behavior · bt_navigator 는 안 띄움 (navigate_to_pose 가 겹침)
    mppi = controller(MPPI) · behavior · bt_navigator + 정밀 접근 rl_dock. 박스에 붙어 선 자리 출발 처리 (빠져나오기 · 영역 끄기) 없음
  정밀 접근 dock: simple (기본) = simple_dock_node.py (규칙: 돌고 → 곧게 붙음, 라이다 원래 점으로 옆 벽 · 사람 멈춤) / rl = rl_dock (dock3)

  # 실물 (기본 localization:=slam): slam.launch.py 를 먼저 띄움 -> map -> odom · /map 은 slam_toolbox
  ros2 launch navigation slam.launch.py hardware:=opencr slam_mode:=mapping map:=$HOME/maps/arena   # 시연: 저장 지도 이어 그리기
  ros2 launch navigation nav2.launch.py
  ros2 launch navigation nav2.launch.py mission_hold:=false                                  # planning 없이 Nav2 만 시험
  ros2 launch navigation nav2.launch.py local_planner:=mppi                                  # 주행을 MPPI 로
  ros2 launch navigation nav2.launch.py dock:=rl                                             # 정밀 접근을 팀 RL dock3 으로 (기본 simple — 실물: dock3 이 옆 벽에 부딪힘)
  # AMCL (시뮬 · slam_toolbox 없이): map_server + AMCL 이 map -> odom · /map
  ros2 launch navigation nav2.launch.py localization:=amcl map:=<지도.yaml> use_sim_time:=true rviz:=true
  ros2 launch navigation nav2.launch.py localization:=amcl map:=<지도.yaml> initial_pose:=-1.468,-2.202,180  # 출발 자세로 AMCL 시작

  - 설정 yaml: params 인자 -> 레포 config/nav2.yaml (bringup 이 share/bringup/config 로 설치)
  - localization (기본 slam): map -> odom 주인은 하나만
      slam = 이 launch 는 위치 찾기를 안 띄움. slam_toolbox 가 냄 (시연 = mapping 으로 저장 posegraph 이어 그리기,
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
  - 보내는 것: /cmd_vel, TF map -> odom (amcl 일 때만), navigate_to_pose · dock_to_pose 액션 (mission_manager 가 부름)
  - RL 노드 cmd_vel_nav -> collision_monitor (MPPI 와 같은 길). rl 에서는 local costmap 이 없어 FootprintApproach 는 안 돎 (PolygonStop 만)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

LOCALIZATION = ["map_server", "amcl"]
NAVIGATION_MPPI = ["planner_server", "controller_server", "behavior_server", "bt_navigator", "collision_monitor"]
NAVIGATION_RL = ["planner_server", "collision_monitor"]
# RL 노드 공통: 영역 끄기 대상 (collision_monitor 영역 이름 = config/nav2.yaml)
POLYGONS = "PolygonStop,FootprintApproach"
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


def rl_files():
    """bringup share 의 config/rl (원본 레포 config/rl). 없으면 오류"""
    d = os.path.join(get_package_share_directory("bringup"), "config", "rl")
    for f in ("nav_nf5.npz", "nav_nf5.json", "dock3.npz", "dock3.json", "team_swerve.yaml"):
        if not os.path.isfile(os.path.join(d, f)):
            raise RuntimeError(f"RL 정책 파일 없음: {d}/{f} (레포 config/rl, bringup 다시 빌드)")
    return d


def rl_node(name, policy, cfg, d, sim, extra):
    """local_planner_node.py 하나. 정책 = numpy npz (torch 없음)"""
    args = ["--node_name", name, "--policy", os.path.join(d, policy), "--cfg", os.path.join(d, cfg),
            "--swerve_yaml", os.path.join(d, "team_swerve.yaml")] + (["--sim_time"] if sim else []) + extra
    return Node(package="navigation", executable="local_planner_node.py", name=name, output="screen", arguments=args)


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
    lp = LaunchConfiguration("local_planner").perform(context).strip().lower()
    if lp not in ("rl", "mppi"):
        raise RuntimeError(f"local_planner 는 rl | mppi (지금 {lp})")
    dock_tol = LaunchConfiguration("dock_tol").perform(context)
    dock = LaunchConfiguration("dock").perform(context).strip().lower()
    dock_vmax = LaunchConfiguration("dock_vmax").perform(context)
    if dock not in ("rl", "simple"):
        raise RuntimeError(f"dock 은 rl | simple (지금 {dock})")
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
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output=out,
             parameters=common),
    ]
    rld = rl_files()
    if lp == "mppi":
        nodes += [
            Node(package="nav2_controller", executable="controller_server", name="controller_server", output=out,
                 parameters=common, remappings=TO_MONITOR),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output=out,
                 parameters=common, remappings=TO_MONITOR),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output=out,
                 parameters=common),
        ]
    else:
        # 주행: navigate_to_pose. 붙어 선 자리 출발 = 경로 실패면 뒤로 빠져나오기 · 25 cm 벗어날 때까지 영역 끔 (Isaac · 07)
        nodes.append(rl_node("rl_nav", "nav_nf5.npz", "nav_nf5.json", rld, sim,
                             ["--action_name", "navigate_to_pose", "--tol_pos", "0.05", "--tol_yaw_deg", "5",
                              "--near_disable_polygons", POLYGONS, "--near_ok_s", "0"]))
    # 정밀 접근 (박스 · 장소 정차 0.2575 = 면 1 cm, planning dock_action): 그동안 영역 끔 · 목표 odom 고정 · 라이다로 박스 맞춤
    if dock == "simple":
        # 규칙 정밀 접근: 같은 dock_to_pose · 영역 끔 · 목표 odom 고정 · 박스 맞춤 (box_tracker.py). dock_tol = 더 안 줄 때 받아 주는 거리
        nodes.append(Node(package="navigation", executable="simple_dock_node.py", name="simple_dock", output="screen",
                          arguments=["--tol_ok", dock_tol, "--v_max", dock_vmax, "--disable_polygons", POLYGONS] + (["--sim_time"] if sim else [])))
    else:
        nodes.append(rl_node("rl_dock", "dock3.npz", "dock3.json", rld, sim,
                             ["--action_name", "dock_to_pose", "--tol_pos", dock_tol, "--tol_yaw_deg", "2",
                              "--disable_polygons", POLYGONS, "--goal_frame", "odom", "--box_track"]))
    nodes += [
        # planning /mission/hold → 막는 중엔 Nav2 명령 버리고 0
        Node(package="navigation", executable="cmd_vel_hold", name="cmd_vel_hold", output=out,
             parameters=[{"use_sim_time": sim, "cmd_vel_in": "cmd_vel_safe", "cmd_vel_out": "/cmd_vel",
                          "start_hold": mission_hold, "hold_timeout_s": HOLD_TIMEOUT_S if mission_hold else 0.0}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output=out, parameters=[{"use_sim_time": sim, "autostart": True,
                                      "node_names": NAVIGATION_MPPI if lp == "mppi" else NAVIGATION_RL}]),
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
        DeclareLaunchArgument("local_planner", default_value="rl",
                              description="rl = RL 주행 (nf5) + 정밀 접근 (dock3) / mppi = MPPI · bt_navigator + 정밀 접근"),
        DeclareLaunchArgument("dock", default_value="simple",
                              description="정밀 접근 dock_to_pose: simple = simple_dock_node.py (규칙, 기본: 실물 B1 0.5 cm · 0.15 m/s) / rl = rl_dock (dock3)"),
        DeclareLaunchArgument("dock_vmax", default_value="0.15",
                              description="dock:=simple 최고 속도 [m/s] (박스 가까이서는 남은 거리에 비례해 느려짐). 실물: 0.05 는 너무 느림"),
        DeclareLaunchArgument("dock_tol", default_value="0.015",
                              description="정밀 접근 성공 거리 [m] (Isaac: 0.01 이면 판정 안에 못 들어와 시간 초과 한 번)"),
        DeclareLaunchArgument("mission_hold", default_value="true",
                              description="true = /mission/hold 받기 전 · 끊기면 막음 (미션). false = 그냥 넘김 (Nav2 단독 시험)"),
        OpaqueFunction(function=make_nodes),
    ])
