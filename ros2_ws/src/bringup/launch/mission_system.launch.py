"""mission_system.launch.py — 시연 전체 실행: 로봇 (바퀴 · 팔) + 라이다 · SLAM + Nav2 + 카메라 · 인지 + mission 을 한 번에

역할
  - 아래 launch 를 묶고 AMCL 자동 초기 위치 노드를 연결함. 노드마다 주인은 하나:
      navigation/slam.launch.py  → bringup/robot.launch.py (arm:=true, ros2_control · swerve · control_master · camera_drift · RSP)
                                   + 라이다 드라이버 · scan_merger · slam_toolbox (map → odom) · pose_guard
                                   (localization:=amcl 이면 slam_toolbox 만 빼고 그대로 — slam:=false)
      bringup/bringup.launch.py  → 손목 D435 · 광각 usb_cam · perception_node (+ 선택 디버그 압축). nav · mission 은 여기서 안 켬 (중복)
      navigation/nav2.launch.py  → planner_server · collision_monitor · cmd_vel_hold · rl_nav (navigate_to_pose) · dock_to_pose
                                   (localization:=amcl 이면 + map_server (<map>.yaml) · AMCL (map → odom, initial_pose 에서 시작))
      mission/mission.launch.py  → mission_manager (STANDBY 에서 /mission/start 기다림, 자동 출발 안 함)
  - 실물 조합 검사: hardware:=opencr 면 라이다 · 지도 · 인지 엔진이 다 있어야 함 (노드를 띄우기 전에 거부)
  - 포트 · 사람 확인은 tools/jetson/run_robot.sh mission 이 함 (실물은 그 스크립트로 띄움)
받는 것: launch 인자 (아래 DeclareLaunchArgument)
보내는 것: 띄운 노드들이 내는 것 (각 launch 머리 설명). 운영자 명령 = ros2 service call /mission/start std_srvs/srv/Trigger '{}'
실행
  bash tools/jetson/run_robot.sh mission opencr ~/maps/arena5                      # 실물 (권장)
  bash tools/jetson/run_robot.sh mission opencr ~/maps/arena6 localization:=amcl   # 실물, 위치 = AMCL (arena6.yaml, 출발 자세 0,0,0)
  ros2 launch bringup mission_system.launch.py hardware:=opencr opencr_ports:=bus_drive=…,bus_arm=… \\
      lidar_front_port:=… lidar_rear_port:=… map:=$HOME/maps/arena5
  ros2 launch bringup mission_system.launch.py                                      # 로봇 없이 조립 확인 (mock, 라이다 · 카메라 · 인지 끔)
note
  - launch 인자는 포함한 launch 와 이름 공간을 같이 씀 → 자식과 이름이 같은 인자 (map · hardware_map …) 는 자식에 값을 직접 넘김
  - mock 기본값: 라이다 · 카메라 · 인지 끔 → scan_merger · slam 은 입력 대기, mission 은 'STANDBY 준비 안 됨' 이 정상 (이름 · 연결 확인용)
  - 개발 PC 는 TensorRT 가 없어 perception_node 가 빌드 안 됨 → perception:=false
  - 신호등 12 점멸은 mission_manager 가 STANDBY 부터 기록 → 'STANDBY 준비 완료' 뒤에 신호를 시작해야 놓치지 않음
  - Fast DDS 전송 (dds_transport) 은 맨 앞에서 정해 모든 자식 노드에 적용. 밖에서 보는 PC 도 같은 값
  - 켜는 순서: 0 s 로봇 · 컨트롤러 · 라이다 → stage2_delay_s (20) Nav2 · AMCL · mission → + camera_delay_s (10) 카메라 둘 · 인지
      (Nav2 lifecycle 시작이 카메라 · TensorRT 시작과 같은 순간에 겹치면 서비스 응답 · 구독이 유실될 수 있음.
       Nav2 autostart 는 stage2 뒤 3–7 s 에 끝남 → 카메라를 10 s 늦추면 겹치지 않음. STANDBY 까지 약 35 s)
  - 시작 복구 startup_recovery (기본 true): Nav2 autostart + 실패한 lifecycle manager 만 복구 (nav2.launch.py 설명)
  - localization (기본 slam): map → odom 주인 하나
      slam = slam_toolbox (slam_mode mapping = 저장 지도 이어 그리기 / localization = 지도 고정), map = <지도> (.posegraph · .data)
      amcl = map_server + AMCL (nav2.launch.py localization:=amcl). 지도 = <map>.yaml (save_map.sh 가 같이 저장, map 이 .yaml 로 끝나면 그대로)
             시작 자세 initial_pose = 'x,y,yaw_deg' (map 좌표, 기본 0,0,0 = 지도 그리기 시작 자세 = 출발 자세). slam_mode 는 안 씀
             AMCL 은 stage2_delay_s 뒤 (Nav2 와 같이) 켜짐 → 그 전까지 pose_guard 는 'map→odom 없음' (mission 도 그 뒤에 뜸)
      mission 위치 판단 mission_loc: auto (기본) = localization:=amcl 이면 amcl, 아니면 health
                                     health = pose_guard /localization_health (slam · amcl 공통, map → odom · 지도 맞음 · 튐)
                                     amcl = /amcl_pose 공분산 + localization lifecycle (localization:=amcl 일 때만)
                                     (벽만 그린 지도에서는 박스 앞에 설 때마다 pose_guard 지도 맞음이 42–56 % 로 떨어져 level 2
                                      → health 면 박스마다 LOCALIZATION_LOST 로 PICK · 이동이 막힘. 그래서 amcl 이면 공분산으로 판단)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, name, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(share(pkg, "launch", name)),
                                   launch_arguments=list(args.items()))


def has_executable(pkg, exe):
    try:
        return os.path.isfile(os.path.join(get_package_prefix(pkg), "lib", pkg, exe))
    except PackageNotFoundError:
        return False


def check(arg, on):
    """실물 조합 검사. 문제 목록 (빈 목록 = 통과)"""
    bad = []
    real = arg("hardware") == "opencr"
    # 실물: 라이다 · 지도 · 감시 필수 (SLAM 없이 미션을 돌리면 위치 추정 상실로 바로 멈춤)
    if real:
        if not on("lidar") or not (arg("lidar_front_port") and arg("lidar_rear_port")):
            bad.append("실물은 앞 · 뒤 라이다 둘 다 필요 (lidar:=true, lidar_front_port · lidar_rear_port)")
        if not on("guard"):
            bad.append("실물은 위치 감시 pose_guard 필요 (guard:=true, mission 이 /localization_health 로 위치 판단)")
        if not on("perception"):
            bad.append("실물 미션은 인지 필요 (perception:=true)")
    if "drive_only" in arg("hardware_map"):
        bad.append("hardware_map 이 바퀴만 (drive_only) — 미션은 팔 포함 지도")
    m = os.path.expanduser(arg("map"))
    if arg("localization") == "amcl":
        if not m:
            bad.append("localization:=amcl 은 map:= 필수 (save_map.sh 로 저장한 <지도> 또는 <지도>.yaml)")
        elif not os.path.isfile(amcl_map(m)):
            bad.append(f"AMCL 지도 {amcl_map(m)} 없음 (save_map.sh 가 .pgm 과 같이 저장)")
        try:
            pose = [float(v) for v in arg("initial_pose").split(",")]
        except ValueError:
            pose = []
        if len(pose) != 3:
            bad.append(f"initial_pose '{arg('initial_pose')}' 는 'x,y,yaw_deg' (map 좌표)")
        if not on("nav"):
            bad.append("localization:=amcl 은 nav:=true 필요 (AMCL 은 nav2.launch.py 가 띄움)")
    elif m:
        for ext in (".posegraph", ".data"):
            if not os.path.isfile(m + ext):
                bad.append(f"지도 {m}{ext} 없음 (확장자 빼고 줌, tools/jetson/save_map.sh 로 저장한 것)")
    elif real and arg("slam_mode") == "localization":
        bad.append("slam_mode:=localization 은 map:= 필수")
    if arg("mission_loc") == "amcl" and arg("localization") != "amcl":
        bad.append("mission_loc:=amcl 은 localization:=amcl 일 때만 (/amcl_pose 가 없음)")
    # 인지: 노드 · 엔진 · 이름표
    if on("perception"):
        if not has_executable("perception", "perception_node"):
            bad.append("perception_node 가 설치 안 됨 (TensorRT 없는 빌드) — perception:=false 로 끄거나 Jetson 에서 빌드")
        for key in ("wide_model", "wrist_model"):
            p = os.path.expanduser(arg(key))
            if not os.path.isfile(p):
                bad.append(f"{key} {p} 없음")
            elif not os.path.isfile(os.path.splitext(p)[0] + ".names.txt"):
                bad.append(f"{key} 옆 {os.path.splitext(p)[0]}.names.txt 없음")
    dm = os.path.expanduser(arg("dice_map"))
    if dm and not os.path.isfile(dm):
        bad.append(f"dice_map {dm} 없음")
    return bad


def mission_loc(arg):
    """mission 위치 판단: auto = localization:=amcl 이면 amcl (공분산), 아니면 health (pose_guard)"""
    v = arg("mission_loc")
    if v == "auto":
        return "amcl" if arg("localization") == "amcl" else "health"
    return v


def amcl_map(m):
    """AMCL 지도 yaml: map 이 .yaml 로 끝나면 그대로, 아니면 <map>.yaml (save_map.sh 가 posegraph 와 같이 저장)"""
    return m if m.endswith(".yaml") else m + ".yaml"


def make(context):
    arg = lambda name: context.perform_substitution(LaunchConfiguration(name)).strip()  # noqa: E731
    on = lambda name: arg(name).lower() == "true"  # noqa: E731
    bad = check(arg, on)
    if bad:
        raise RuntimeError("mission_system 거부:\n  - " + "\n  - ".join(bad))

    out = []
    if arg("dds_transport"):   # 이 뒤에 띄우는 모든 노드에 적용
        out.append(SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", arg("dds_transport")))
    amcl = arg("localization") == "amcl"
    mloc = mission_loc(arg)
    m = os.path.expanduser(arg("map"))
    loc_txt = f"amcl map={amcl_map(m)} initial_pose={arg('initial_pose')}" if amcl else \
        f"slam slam_mode={arg('slam_mode')} map={arg('map') or '(새로 매핑)'}"
    out.append(LogInfo(msg=f"mission_system: hardware={arg('hardware')} localization={loc_txt} "
                           f"mission_loc={mloc} perception={arg('perception')} dock={arg('dock')}"))

    # 1. 로봇 + 라이다 + SLAM (robot.launch 는 slam.launch 안에서만 포함 → robot_state_publisher · ros2_control 하나)
    #    localization:=amcl → slam:=false (slam_toolbox 만 뺌, map 도 안 넘김 — posegraph 검사 안 함)
    slam = {"hardware": arg("hardware"), "arm": "true", "lidar": arg("lidar"), "guard": arg("guard"),
            "slam": "false" if amcl else "true", "slam_mode": arg("slam_mode"), "map": "" if amcl else m,
            "lidar_front_port": arg("lidar_front_port"), "lidar_rear_port": arg("lidar_rear_port"),
            "opencr_ports": arg("opencr_ports"), "use_rviz": arg("use_rviz"), "slam_rviz": arg("slam_rviz"),
            # launch 인자는 전역이라 꼭 값을 넘김 (빈 값이 slam.launch 기본을 덮음) — 비우면 전체 hardware_map.yaml (팔 포함)
            "hardware_map": arg("hardware_map") or share("bringup", "config", "hardware_map.yaml")}
    out.append(include("navigation", "slam.launch.py", slam))

    # 2 ~ 5 는 stage2_delay_s 뒤에 띄움: 다 같이 켜면 스포너 · control_master 가 controller_manager 서비스를 못 찾는 일이 잦음
    #   → 로봇 · 컨트롤러가 먼저, 그 뒤 카메라 · 인지 · Nav2 · mission. 0 = 한꺼번에
    late = []

    # 2. 카메라 · 인지 (nav · mission 은 아래에서 한 번만). Nav2 보다 camera_delay_s 늦게 (아래 TimerAction)
    cameras = []
    if on("wrist_cam") or on("wide_cam") or on("perception"):
        cameras.append(include("bringup", "bringup.launch.py", {
            "dds_transport": arg("dds_transport"), "wrist_cam": arg("wrist_cam"), "wide_cam": arg("wide_cam"),
            "wrist_respawn": arg("wrist_respawn"), "wrist_respawn_delay": arg("wrist_respawn_delay"),
            "wide_device": arg("wide_device"), "perception": arg("perception"),
            "wide_model": os.path.expanduser(arg("wide_model")), "wrist_model": os.path.expanduser(arg("wrist_model")),
            "publish_debug": arg("publish_debug"), "jpeg_quality": arg("jpeg_quality"),
            "nav": "false", "mission": "false"}))

    # 3. Nav2. localization slam = map → odom 은 slam_toolbox (map · initial_pose 비움 — 위 posegraph 경로가 새지 않게)
    #            amcl = Nav2 가 map_server (<map>.yaml) + AMCL (initial_pose) 를 같이 띄움
    if on("nav"):
        late.append(include("navigation", "nav2.launch.py", {
            "localization": "amcl" if amcl else "slam", "map": amcl_map(m) if amcl else "",
            "initial_pose": arg("initial_pose") if amcl else "", "local_planner": arg("local_planner"),
            "startup_recovery": arg("startup_recovery"),
            "dock": arg("dock"), "dock_policy": arg("dock_policy"), "dock_search": arg("dock_search"),
            "dock_vmax": arg("dock_vmax"), "dock_tol": arg("dock_tol"), "mission_hold": arg("mission_hold"),
            "rviz": arg("rviz")}))

    # AMCL 시작 위치를 스캔으로 맞춤. 임계값/탐색 폭은 노드 기본값 한 곳에 둠.
    # /map · /scan · 정지 odom 이 준비되면 스스로 시작하며 미션의 hold/FAULT 는 바꾸지 않음.
    if amcl and on("auto_initial_pose"):
        ix, iy, iyaw = (float(v) for v in arg("initial_pose").split(","))
        # C++ 판. 파이썬 판 auto_initial_pose.py 도 설치돼 있음 (바꾸기 = executable 이름만)
        late.append(Node(package="navigation", executable="auto_initial_pose", name="auto_initial_pose",
                         output="screen", parameters=[{"init_x": ix, "init_y": iy, "init_yaw_deg": iyaw}]))

    # 4. mission (STANDBY 에서 /mission/start 기다림)
    if on("mission"):
        # mission.launch localization: slam → localization_source health (pose_guard) / amcl → /amcl_pose 공분산
        mission = {"localization": "amcl" if mloc == "amcl" else "slam"}
        if arg("planning_params"):
            mission["params"] = os.path.expanduser(arg("planning_params"))
        if arg("dice_map"):
            mission["dice_map"] = os.path.expanduser(arg("dice_map"))
        late.append(include("mission", "mission.launch.py", mission))

    # 늦게 띄울 것 (stage2_delay_s > 0): Nav2 · mission, 그 뒤 camera_delay_s 더 늦게 카메라 · 인지
    delay = float(arg("stage2_delay_s") or "0")
    cam_delay = delay + max(0.0, float(arg("camera_delay_s") or "0"))
    if delay > 0.0 and late:
        out.append(LogInfo(msg=f"mission_system: Nav2 · mission 은 {delay:.0f} s 뒤 (로봇 · 컨트롤러 먼저), "
                               f"카메라 · 인지는 {cam_delay:.0f} s 뒤 (Nav2 lifecycle 시작과 안 겹치게)"))
        out.append(TimerAction(period=delay, actions=late))
    else:
        out += late
    if cameras:
        if cam_delay > 0.0:
            out.append(TimerAction(period=cam_delay, actions=cameras))
        else:
            out += cameras
    return out


def generate_launch_description():
    home = os.path.expanduser("~")
    A = DeclareLaunchArgument
    return LaunchDescription([
        # 로봇 · SLAM
        A("stage2_delay_s", default_value="20", description="Nav2 · mission 을 이만큼 뒤에 띄움 [s] (컨트롤러 먼저). 0 = 한꺼번에"),
        A("camera_delay_s", default_value="10",
          description="카메라 · 인지를 stage2 보다 이만큼 더 늦게 [s] (Nav2 lifecycle 시작 3–7 s 와 TensorRT · 카메라 부하가 안 겹치게). 0 = Nav2 와 같이"),
        A("hardware", default_value="mock", choices=["mock", "opencr"], description="mock = 가짜 모터 / opencr = 실물"),
        A("opencr_ports", default_value="auto", description="실물 보드 'bus_drive=<주행>,bus_arm=<팔>' (run_robot.sh 가 채움)"),
        A("hardware_map", default_value="", description="비우면 전체 hardware_map.yaml (팔 포함). drive_only 는 안 됨"),
        A("lidar", default_value="false", choices=["true", "false"], description="라이다 드라이버 (실물 true 필수)"),
        A("lidar_front_port", default_value="", description="앞 라이다 /dev/serial/by-path/…"),
        A("lidar_rear_port", default_value="", description="뒤 라이다 /dev/serial/by-path/…"),
        A("slam_mode", default_value="mapping", choices=["mapping", "localization"],
          description="mapping = 저장 지도 이어 그리기 (시연 기본, map 비우면 새로) / localization = 지도 고정"),
        A("map", default_value="", description="저장 지도 (확장자 빼고, .posegraph · .data). localization:=amcl 이면 <map>.yaml"),
        A("localization", default_value="slam", choices=["slam", "amcl"],
          description="map → odom: slam = slam_toolbox (slam_mode) / amcl = map_server + AMCL (<map>.yaml, initial_pose)"),
        A("initial_pose", default_value="0,0,0",
          description="localization:=amcl 시작 자세 'x,y,yaw_deg' (map 좌표). 0,0,0 = 지도 그리기 시작 자세 (출발 자세에서 그렸으면 출발)"),
        A("auto_initial_pose", default_value="true", choices=["true", "false"],
          description="AMCL: 정지 중 지도/스캔으로 시작 위치 자동 맞춤, 낮은 지도 일치도에서 자동 재시도"),
        A("startup_recovery", default_value="true", choices=["true", "own", "false"],
          description="true (기본) = Nav2 autostart + 실패한 lifecycle manager 만 복구 (RESET 최대 3 · 멈춘 manager 재시작 최대 2) / "
                      "own = 감시기가 시작도 맡음 (11-13 예전, 비교용) / false = autostart 만"),
        A("mission_loc", default_value="auto", choices=["auto", "health", "amcl"],
          description="mission 위치 판단: auto (기본) = localization:=amcl 이면 amcl 아니면 health / health = pose_guard /localization_health / "
                      "amcl = /amcl_pose 공분산 (localization:=amcl 일 때만)"),
        A("guard", default_value="true", choices=["true", "false"], description="pose_guard (/localization_health)"),
        A("use_rviz", default_value="false", choices=["true", "false"], description="로봇 모델 rviz"),
        A("slam_rviz", default_value="false", choices=["true", "false"], description="지도 · 스캔 rviz"),
        # 카메라 · 인지
        A("dds_transport", default_value="LARGE_DATA", description="FASTDDS_BUILTIN_TRANSPORTS ('' 면 안 바꿈)"),
        A("wrist_cam", default_value="false", choices=["true", "false"], description="손목 D435 (실물 run_robot.sh 는 true)"),
        A("wrist_respawn", default_value="true", choices=["true", "false"], description="D435 프로세스 종료 시 자동 재시작"),
        A("wrist_respawn_delay", default_value="3.0", description="D435 재시작 대기 [s]"),
        A("wide_cam", default_value="false", choices=["true", "false"], description="광각 Arducam"),
        A("wide_device", default_value="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"),
        A("perception", default_value="false", choices=["true", "false"], description="perception_node (TensorRT 빌드 필요)"),
        A("wide_model", default_value=os.path.join(home, "models", "best_wide.engine")),
        A("wrist_model", default_value=os.path.join(home, "models", "best_wrist.engine")),
        A("publish_debug", default_value="false", choices=["true", "false"]),
        A("jpeg_quality", default_value="30"),
        # Nav2
        A("nav", default_value="true", choices=["true", "false"]),
        A("local_planner", default_value="rl", choices=["rl", "mppi"]),
        A("dock", default_value="rl", choices=["rl", "simple"]),
        A("dock_policy", default_value="dock3"),
        A("dock_search", default_value="0.20"),
        A("dock_vmax", default_value="0.15"),
        A("dock_tol", default_value="", description="비우면 nav2.launch 기본 (dock5 0.01 · dock3 · simple 0.015)"),
        A("mission_hold", default_value="true", choices=["true", "false"], description="true = mission 이 /mission/hold 로 주행 허가"),
        A("rviz", default_value="false", choices=["true", "false"], description="Nav2 rviz"),
        # mission
        A("mission", default_value="true", choices=["true", "false"]),
        A("planning_params", default_value="", description="비우면 레포 config/planning.yaml"),
        A("dice_map", default_value="", description="비우면 레포 maps/dice_map.yaml (arena5 전용 표 등)"),
        OpaqueFunction(function=make),
    ])
