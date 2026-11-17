"""nav2.launch.py — Nav2 실행 (planner · collision_monitor + 지역 계획기 + localization:=amcl 이면 map_server · AMCL)
  지역 계획기 local_planner:
    rl (기본) = RL 노드 둘 (navigation local_planner_node (C++, src/local_planner_node.cpp), 정책 레포 config/rl/):
                주행 rl_nav = navigate_to_pose (nav_policy, 기본 nf5) · 정밀 접근 rl_dock = dock_to_pose (dock_policy, 기본 dock3, planning dock_action)
                controller(MPPI) · behavior · bt_navigator 는 안 띄움 (navigate_to_pose 가 겹침)
    mppi = controller(MPPI) · behavior · bt_navigator + 정밀 접근 rl_dock. 박스에 붙어 선 자리 출발 처리 (빠져나오기 · 영역 끄기) 없음
  정밀 접근 dock: rl (기본) = rl_dock (dock3) / simple = simple_dock_node (규칙: 돌고 → 곧게 붙음, 라이다 원래 점으로 옆 벽 · 사람 멈춤)

  # 실물 (기본 localization:=slam): slam.launch.py 를 먼저 띄움 -> map -> odom · /map 은 slam_toolbox
  ros2 launch navigation slam.launch.py hardware:=opencr slam_mode:=mapping map:=$HOME/maps/arena   # 시연: 저장 지도 이어 그리기
  ros2 launch navigation nav2.launch.py
  ros2 launch navigation nav2.launch.py mission_hold:=false                                  # planning 없이 Nav2 만 시험
  ros2 launch navigation nav2.launch.py local_planner:=mppi                                  # 주행을 MPPI 로
  ros2 launch navigation nav2.launch.py nav_policy:=nav_nf7                                   # 주행 정책을 nf7 로 (기본 nav_nf5)
  ros2 launch navigation nav2.launch.py dock:=simple                                         # 정밀 접근을 규칙 노드로 (기본 rl = dock3)
  ros2 launch navigation nav2.launch.py nav_block_wait_s:=3 nav_max_reversals:=3              # 주행 rl_nav: 막히면 서서 기다림 · 제자리 회전 반전 제한 (기본 0 = 끔)
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
  - 시작 복구 startup_recovery (기본 true = 복구만):
      true  = lifecycle_manager autostart 가 켬 (autostart 와 같은 시작 시간) + startup_lifecycle_watchdog 이 실패한 manager 만 고침:
              Aborting bringup (is_active false) → 그 manager RESET→노드 정합→STARTUP (최대 3) / 응답 유실로 멈춘 manager (is_active 무응답)
              → 그 lifecycle_manager 프로세스를 끝내고 launch respawn 이 다시 띄움 (autostart 다시, 최대 2) / AMCL ACTIVE 인데 map→odom 없음 → RESET
              lifecycle_manager 는 respawn (멈춘 manager 를 다시 띄우는 유일한 길). Nav2 노드 (AMCL 등) 는 respawn 안 함 (AMCL 이 출발 자세로 다시 켜지면 위치가 틀림)
      own   = autostart false, 감시기가 STARTUP 도 맡음 (실패가 잦아 비교용으로만)
      false = autostart 만 (실패하면 복구 없음)
  - 받는 것: /odom, TF odom -> base_link (swerve_controller · 시뮬 브리지), /lidar_front/scan, /lidar_rear/scan (costmap · AMCL), /scan (collision_monitor, scan_merger 는 이 launch 가 안 띄움 → slam.launch.py · 시뮬 launch)
  - 받는 것 (slam): TF map -> odom · /map (slam_toolbox)
  - 보내는 것: /cmd_vel, TF map -> odom (amcl 일 때만), navigate_to_pose · dock_to_pose 액션 (mission_manager 가 부름)
  - RL 노드 cmd_vel_nav -> collision_monitor (MPPI 와 같은 길). FootprintApproach 는 nav2.yaml 의 고정 점 (±0.16) 이라 rl 에서도 돎
    (/scan 점이 그 안에 들면 그 방향 명령을 0 으로. 출발 자리 영역 끄기는 PolygonStop 만 → 박스 면은 scan_merger 가 지워야 출발함)
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
# 주행 노드 출발 자리 영역 끄기 = PolygonStop 만: FootprintApproach (진행 방향 1 s 안 충돌만 감속, 멀어지는 쪽은 안 막음) 는 켜 둠
#   → 붙어 선 자리 출발도 사람 앞 감속 남음 (PolygonStop 은 멀어지는 명령까지 0 으로 막아 좁은 칸 출발에서 갇힘)
NEAR_POLYGONS = "PolygonStop"
# controller · behavior 의 속도 명령은 collision_monitor (비상 정지) 를 거쳐 /cmd_vel 로 나감
TO_MONITOR = [("cmd_vel", "cmd_vel_nav")]
# /mission/hold 가 이만큼 안 오면 막음 (mission_manager 는 1 Hz + 바뀔 때 발행)
HOLD_TIMEOUT_S = 2.5


def startup_config(mode, sim, manage_localization, nav_nodes):
    """startup_recovery → (lifecycle_manager 공통 파라미터, manager respawn 인자, 감시기 파라미터 또는 None)
      true (기본) = autostart + manager respawn + 감시기 복구만 / own = 감시기가 시작 (autostart false) / false = autostart 만"""
    if mode not in ("true", "own", "false"):
        raise RuntimeError(f"startup_recovery 는 true | own | false (지금 {mode})")
    own = mode == "own"            # 최초 자동 시작을 감시기가 맡음 (autostart false)
    recover = mode == "true"       # 기본: autostart 가 켜고 감시기는 실패한 manager 만 복구
    manager = {"use_sim_time": sim, "autostart": not own, "bond_timeout": 15.0}
    if own:
        manager["service_timeout"] = 15.0
    # 복구 모드: 감시기가 멈춘 manager 프로세스를 끝내면 launch 가 다시 띄움 (autostart 다시). 그 밖에 manager 가 죽어도 다시 뜸
    respawn = {"respawn": True, "respawn_delay": 2.0} if recover else {}
    if not (own or recover):
        return manager, respawn, None
    # 복구 모드 시간 (노드 기본값): autostart_grace_s 30 · abort_confirm_s 6 · hang_s 20 · localization_output_timeout_s 30
    # manager_timeout_s 15: 감시기가 보낸 RESET/STARTUP 응답 기한 (정상 2–5 s, bond 기다림 최대 15 s). 넘으면 같은 callback group 의
    #   is_active 로 확인 (명령이 아직 도는 중이면 끝난 뒤에 답함 → 정상 시작을 다시 RESET 하지 않음), hang_s 무응답이면 manager 재시작.
    #   RESET 요청 자체가 유실될 수 있음 → 30 s 는 너무 김
    wd = {"use_sim_time": sim, "manage_localization": manage_localization, "recover_only": recover,
          "localization_nodes": LOCALIZATION, "navigation_nodes": nav_nodes}
    if recover:
        wd["manager_timeout_s"] = 15.0
    return manager, respawn, wd


def bringup_params():
    """bringup share 에 설치된 config/nav2.yaml (원본: 레포 config/) 경로, 없으면 빈 문자열"""
    try:
        path = os.path.join(get_package_share_directory("bringup"), "config", "nav2.yaml")
    except PackageNotFoundError:
        return ""
    return path if os.path.isfile(path) else ""


def rl_files(policies):
    """bringup share 의 config/rl (원본 레포 config/rl). 이번 실행에 쓰는 정책 npz · json 만 검사, 없으면 오류"""
    d = os.path.join(get_package_share_directory("bringup"), "config", "rl")
    for f in [p + ext for p in policies for ext in (".npz", ".json")] + ["team_swerve.yaml"]:
        if not os.path.isfile(os.path.join(d, f)):
            raise RuntimeError(f"RL 정책 파일 없음: {d}/{f} (레포 config/rl, bringup 다시 빌드)")
    return d


def rl_node(name, policy, cfg, d, sim, extra):
    """local_planner_node (C++) 하나. 정책 = npz (export.py). 인자는 파이썬 판 scripts/local_planner_node.py 와 같음"""
    args = ["--node_name", name, "--policy", os.path.join(d, policy), "--cfg", os.path.join(d, cfg),
            "--swerve_yaml", os.path.join(d, "team_swerve.yaml")] + (["--sim_time"] if sim else []) + extra
    return Node(package="navigation", executable="local_planner_node", name=name, output="screen", arguments=args)


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
    manager_common, manager_respawn, watchdog = startup_config(
        LaunchConfiguration("startup_recovery").perform(context).strip().lower(), sim, loc == "amcl",
        NAVIGATION_MPPI if lp == "mppi" else NAVIGATION_RL)
    dock = LaunchConfiguration("dock").perform(context).strip().lower()
    dock_policy = LaunchConfiguration("dock_policy").perform(context).strip()
    # 비우면 dock5 = 0.01 (학습 성공 거리), 그 밖 (dock3 · simple) = 0.015
    #   (dock3 는 0.01 이면 판정 밖에서 시간 초과가 남, simple 은 규칙 노드 기본 tol_ok)
    dock_tol = LaunchConfiguration("dock_tol").perform(context).strip() or (
        "0.01" if dock == "rl" and dock_policy == "dock5" else "0.015")
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
            # bond_timeout 15 s: 아래 navigation 과 같게 (기본 4 s 는 stage2 에 카메라 · 인지 · Nav2 · mission 이 한꺼번에 뜰 때
            #   map_server bond 가 늦어 'unable to be reached after 4.00s by bond' → AMCL 안 켜짐)
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 output=out, parameters=[dict(manager_common, node_names=LOCALIZATION)], **manager_respawn),
        ]
        # AMCL 은 움직여야 고침 → 서 있는 동안 (켤 때 · /initialpose 뒤) request_nomotion_update 로 맞춤 (출발 자세가 조금 틀리면 STANDBY 가 안 풀림)
        #   AMCL initial_pose 는 공분산 0 (입자 한 점) → amcl_settle 이 같은 자세를 퍼짐 (±5 cm · ±3°) 있게 /initialpose 한 번 다시 줌
        if LaunchConfiguration("amcl_settle").perform(context).strip().lower() == "true":
            ix, iy, iyaw = (float(v) for v in init.split(",")) if init else (0.0, 0.0, 0.0)
            nodes.append(Node(package="navigation", executable="amcl_settle", name="amcl_settle", output=out,
                              parameters=[{"use_sim_time": sim, "init_x": ix, "init_y": iy, "init_yaw_deg": iyaw,
                                           "init_std_xy": 0.05 if init else 0.0, "init_std_yaw_deg": 3.0}]))
    nodes += [
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output=out, parameters=common),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output=out,
             parameters=common),
    ]
    nav_policy = LaunchConfiguration("nav_policy").perform(context).strip()
    rld = rl_files(([nav_policy] if lp == "rl" else []) + ([dock_policy] if dock == "rl" else []))
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
        # 주행: navigate_to_pose. 붙어 선 자리 출발 = 경로 실패면 뒤로 빠져나오기 · 25 cm 벗어날 때까지 PolygonStop 끔
        #   nav_block_wait_s · nav_max_reversals: 0 (기본) 이면 인자를 안 붙임
        guard = []
        for la, opt in (("nav_block_wait_s", "--block_wait_s"), ("nav_max_reversals", "--max_reversals")):
            v = LaunchConfiguration(la, default="0").perform(context).strip()
            if v and float(v) > 0:
                guard += [opt, str(int(float(v))) if opt == "--max_reversals" else v]
        nodes.append(rl_node("rl_nav", nav_policy + ".npz", nav_policy + ".json", rld, sim,
                             ["--action_name", "navigate_to_pose", "--tol_pos", "0.05", "--tol_yaw_deg", "5",
                              "--near_disable_polygons", NEAR_POLYGONS, "--near_ok_s", "0"] + guard))
    # 정밀 접근 (박스 · 장소 정차 0.2575 = 면 1 cm, planning dock_action): 그동안 영역 끔 · 목표 odom 고정 · 라이다로 박스 맞춤
    if dock == "simple":
        # 규칙 정밀 접근: 같은 dock_to_pose · 영역 끔 · 목표 odom 고정 · 박스 맞춤 (FaceTrack, simple_dock_core). dock_tol = 더 안 줄 때 받아 주는 거리
        nodes.append(Node(package="navigation", executable="simple_dock_node", name="simple_dock", output="screen",
                          arguments=["--tol_ok", dock_tol, "--v_max", dock_vmax, "--disable_polygons", POLYGONS] + (["--sim_time"] if sim else [])))
    else:
        # 박스 맞춤 = FaceTrack: 지도 박스 자리 ±box_search 안에서 찾음 (박스 없이 딴 지도 · 도면 좌표, 실제 박스는 8–19 cm 어긋남)
        nodes.append(rl_node("rl_dock", dock_policy + ".npz", dock_policy + ".json", rld, sim,
                             ["--action_name", "dock_to_pose", "--tol_pos", dock_tol, "--tol_yaw_deg", "2",
                              "--disable_polygons", POLYGONS, "--goal_frame", "odom", "--box_track",
                              "--box_search", LaunchConfiguration("dock_search").perform(context),
                              "--box_topic", "/dock/box_measured", "--box_near_shift", "0.02", "--map_topic", "/map"]))
    nodes += [
        # planning /mission/hold → 막는 중엔 Nav2 명령 버리고 0
        Node(package="navigation", executable="cmd_vel_hold", name="cmd_vel_hold", output=out,
             parameters=[{"use_sim_time": sim, "cmd_vel_in": "cmd_vel_safe", "cmd_vel_out": "/cmd_vel",
                          "start_hold": mission_hold, "hold_timeout_s": HOLD_TIMEOUT_S if mission_hold else 0.0}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output=out, parameters=[dict(manager_common, node_names=NAVIGATION_MPPI if lp == "mppi" else NAVIGATION_RL)],
             **manager_respawn),
    ]
    if watchdog:
        # C++ 판 (CPU). 파이썬 판 startup_lifecycle_watchdog.py 도 설치돼 있음 (own 만, 복구 모드는 C++ 판에만 있음)
        nodes.append(Node(package="navigation", executable="startup_lifecycle_watchdog",
                          name="startup_lifecycle_watchdog", output=out, respawn=True, respawn_delay=2.0,
                          parameters=[watchdog]))
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        rviz_cfg = os.path.join(get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz")
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
                          parameters=[{"use_sim_time": sim}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("startup_recovery", default_value="true", choices=["true", "own", "false"],
                              description="true (기본) = autostart + 실패한 lifecycle manager 만 복구 (RESET 최대 3 · 멈춘 manager 재시작 최대 2) / "
                                          "own = 감시기가 시작도 맡음 (비교용) / false = autostart 만"),
        DeclareLaunchArgument("localization", default_value="slam",
                              description="slam = map -> odom 은 slam_toolbox (slam.launch.py) / amcl = map_server + AMCL"),
        DeclareLaunchArgument("map", default_value="", description="지도 yaml (localization:=amcl 일 때 map_server)"),
        DeclareLaunchArgument("params", default_value="", description="비우면 레포 config/nav2.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("initial_pose", default_value="", description="출발 자세 map 기준 'x,y,yaw_deg' (localization:=amcl 일 때만)"),
        DeclareLaunchArgument("amcl_settle", default_value="true", choices=["true", "false"],
                              description="localization:=amcl: 켤 때 시작 자세를 퍼짐 ±5 cm · ±3° 로 다시 주고 서 있는 동안 AMCL request_nomotion_update 15 번 "
                                          "(/initialpose 뒤에도). false = initial_pose 공분산 0 (움직여야 고침)"),
        DeclareLaunchArgument("local_planner", default_value="rl",
                              description="rl = RL 주행 (nav_policy) + 정밀 접근 (dock_policy) / mppi = MPPI · bt_navigator + 정밀 접근"),
        DeclareLaunchArgument("nav_policy", default_value="nav_nf5",
                              description="local_planner:=rl 주행 정책 = config/rl/<이름>.npz · .json (nav_nf5 기본 / nav_nf7)"),
        DeclareLaunchArgument("dock", default_value="rl",
                              description="정밀 접근 dock_to_pose: rl = rl_dock (dock_policy, 기본) / simple = simple_dock_node (규칙 노드)"),
        DeclareLaunchArgument("dock_policy", default_value="dock3",
                              description="dock:=rl 정책 이름 (레포 config/rl/<이름>.npz · .json). dock3 기본 / dock5 = 박스 ±18 cm · 앞쪽 라이다 관측"),
        DeclareLaunchArgument("dock_search", default_value="0.20",
                              description="dock:=rl 박스 찾기 반지름 [m] (지도 박스 자리에서)"),
        DeclareLaunchArgument("dock_vmax", default_value="0.15",
                              description="dock:=simple 최고 속도 [m/s] (박스 가까이서는 남은 거리에 비례해 느려짐). 실물: 0.05 는 너무 느림"),
        DeclareLaunchArgument("dock_tol", default_value="",
                              description="정밀 접근 성공 거리 [m]. 비우면 dock5 0.01 · dock3 · dock:=simple 0.015"),
        DeclareLaunchArgument("nav_block_wait_s", default_value="0",
                              description="주행 rl_nav: 막힘 (경로 없음 · PolygonStop · 경로 앞 50 cm 점) 이면 서서 최대 이만큼 기다림 [s]. 0 = 끔 (켤 때 3)"),
        DeclareLaunchArgument("nav_max_reversals", default_value="0",
                              description="주행 rl_nav: 제자리 회전 반전이 10 s 안 이만큼 넘으면 1 s 멈춤 뒤 5 s 한 방향만. 0 = 끔 (켤 때 3)"),
        DeclareLaunchArgument("mission_hold", default_value="true",
                              description="true = /mission/hold 받기 전 · 끊기면 막음 (미션). false = 그냥 넘김 (Nav2 단독 시험)"),
        OpaqueFunction(function=make_nodes),
    ])
