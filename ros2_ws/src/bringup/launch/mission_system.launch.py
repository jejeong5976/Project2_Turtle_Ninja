"""mission_system.launch.py — 시연 전체 실행: 로봇 (바퀴 · 팔) + 라이다 · SLAM + Nav2 + 카메라 · 인지 + mission 을 한 번에

역할
  - 기존 launch 를 묶기만 함 (노드를 새로 정의하지 않음). 노드마다 주인은 하나:
      navigation/slam.launch.py  → bringup/robot.launch.py (arm:=true, ros2_control · swerve · control_master · camera_drift · RSP)
                                   + 라이다 드라이버 · scan_merger · slam_toolbox (map → odom) · pose_guard
      bringup/bringup.launch.py  → 손목 D435 · 광각 usb_cam · perception_node (+ 선택 디버그 압축). nav · mission 은 여기서 안 켬 (중복)
      navigation/nav2.launch.py  → planner_server · collision_monitor · cmd_vel_hold · rl_nav (navigate_to_pose) · dock_to_pose
      mission/mission.launch.py  → mission_manager (STANDBY 에서 /mission/start 기다림, 자동 출발 안 함)
      bringup/rack_tags.launch.py → rack_tag_node (선택, 빈손 적재함 스캔 보정 때만)
  - 실물 조합 검사: hardware:=opencr 면 라이다 · 지도 · 인지 엔진이 다 있어야 함 (노드를 띄우기 전에 거부)
  - 포트 · 사람 확인은 tools/jetson/run_robot.sh mission 이 함 (실물은 그 스크립트로 띄움)
받는 것: launch 인자 (아래 DeclareLaunchArgument)
보내는 것: 띄운 노드들이 내는 것 (각 launch 머리 설명). 운영자 명령 = ros2 service call /mission/start std_srvs/srv/Trigger '{}'
실행
  bash tools/jetson/run_robot.sh mission opencr ~/maps/arena5                      # 실물 (권장)
  ros2 launch bringup mission_system.launch.py hardware:=opencr opencr_ports:=bus_drive=…,bus_arm=… \\
      lidar_front_port:=… lidar_rear_port:=… map:=$HOME/maps/arena5
  ros2 launch bringup mission_system.launch.py                                      # 로봇 없이 조립 확인 (mock, 라이다 · 카메라 · 인지 끔)
note
  - launch 인자는 포함한 launch 와 이름 공간을 같이 씀 → 자식과 이름이 같은 인자 (map · hardware_map …) 는 자식에 값을 직접 넘김
  - mock 기본값: 라이다 · 카메라 · 인지 끔 → scan_merger · slam 은 입력 대기, mission 은 'STANDBY 준비 안 됨' 이 정상 (이름 · 연결 확인용)
  - 개발 PC 는 TensorRT 가 없어 perception_node 가 빌드 안 됨 → perception:=false
  - 신호등 12 점멸은 mission_manager 가 STANDBY 부터 기록 → 'STANDBY 준비 완료' 뒤에 신호를 시작해야 놓치지 않음
  - Fast DDS 전송 (dds_transport) 은 맨 앞에서 정해 모든 자식 노드에 적용. 밖에서 보는 PC 도 같은 값
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    if m:
        for ext in (".posegraph", ".data"):
            if not os.path.isfile(m + ext):
                bad.append(f"지도 {m}{ext} 없음 (확장자 빼고 줌, tools/jetson/save_map.sh 로 저장한 것)")
    elif real and arg("slam_mode") == "localization":
        bad.append("slam_mode:=localization 은 map:= 필수")
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


def make(context):
    arg = lambda name: context.perform_substitution(LaunchConfiguration(name)).strip()  # noqa: E731
    on = lambda name: arg(name).lower() == "true"  # noqa: E731
    bad = check(arg, on)
    if bad:
        raise RuntimeError("mission_system 거부:\n  - " + "\n  - ".join(bad))

    out = []
    if arg("dds_transport"):   # 이 뒤에 띄우는 모든 노드에 적용
        out.append(SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", arg("dds_transport")))
    out.append(LogInfo(msg=f"mission_system: hardware={arg('hardware')} slam_mode={arg('slam_mode')} "
                           f"map={arg('map') or '(새로 매핑)'} perception={arg('perception')} dock={arg('dock')}"))

    # 1. 로봇 + 라이다 + SLAM (robot.launch 는 slam.launch 안에서만 포함 → robot_state_publisher · ros2_control 하나)
    slam = {"hardware": arg("hardware"), "arm": "true", "lidar": arg("lidar"), "guard": arg("guard"),
            "slam_mode": arg("slam_mode"), "map": os.path.expanduser(arg("map")),
            "lidar_front_port": arg("lidar_front_port"), "lidar_rear_port": arg("lidar_rear_port"),
            "opencr_ports": arg("opencr_ports"), "use_rviz": arg("use_rviz"), "slam_rviz": arg("slam_rviz"),
            # launch 인자는 전역이라 꼭 값을 넘김 (빈 값이 slam.launch 기본을 덮음) — 비우면 전체 hardware_map.yaml (팔 포함)
            "hardware_map": arg("hardware_map") or share("bringup", "config", "hardware_map.yaml")}
    out.append(include("navigation", "slam.launch.py", slam))

    # 2 ~ 5 는 stage2_delay_s 뒤에 띄움 (11-10 밤: 다 같이 켜면 스포너 · control_master 가 controller_manager 서비스를 못 찾는 일이 잦음
    #   → 로봇 · 컨트롤러가 먼저 뜬 뒤 무거운 카메라 · 인지 · Nav2 · mission). 0 = 예전처럼 한꺼번에
    late = []

    # 2. 카메라 · 인지 (nav · mission 은 아래에서 한 번만)
    if on("wrist_cam") or on("wide_cam") or on("perception"):
        late.append(include("bringup", "bringup.launch.py", {
            "dds_transport": arg("dds_transport"), "wrist_cam": arg("wrist_cam"), "wide_cam": arg("wide_cam"),
            "wide_device": arg("wide_device"), "perception": arg("perception"),
            "wide_model": os.path.expanduser(arg("wide_model")), "wrist_model": os.path.expanduser(arg("wrist_model")),
            "publish_debug": arg("publish_debug"), "jpeg_quality": arg("jpeg_quality"),
            "nav": "false", "mission": "false"}))

    # 3. Nav2 (localization = slam 고정: map → odom 은 slam_toolbox 하나)
    if on("nav"):
        late.append(include("navigation", "nav2.launch.py", {
            "localization": "slam", "map": "", "local_planner": arg("local_planner"),   # map = AMCL 용, 위 posegraph 경로가 새지 않게
            "dock": arg("dock"),
            "dock_vmax": arg("dock_vmax"), "dock_tol": arg("dock_tol"), "mission_hold": arg("mission_hold"),
            "rviz": arg("rviz")}))

    # 4. mission (STANDBY 에서 /mission/start 기다림)
    if on("mission"):
        mission = {"localization": "slam"}
        if arg("planning_params"):
            mission["params"] = os.path.expanduser(arg("planning_params"))
        if arg("dice_map"):
            mission["dice_map"] = os.path.expanduser(arg("dice_map"))
        late.append(include("mission", "mission.launch.py", mission))

    # 5. 적재함 칸 태그 (빈손 LOOK slot:=rack 보정 때만)
    if on("rack_tags"):
        late.append(include("bringup", "rack_tags.launch.py", {"held_fresh": arg("held_fresh")}))
    delay = float(arg("stage2_delay_s") or "0")
    if delay > 0.0 and late:
        out.append(LogInfo(msg=f"mission_system: 카메라 · 인지 · Nav2 · mission 은 {delay:.0f} s 뒤 (로봇 · 컨트롤러 먼저)"))
        out.append(TimerAction(period=delay, actions=late))
    else:
        out += late
    return out


def generate_launch_description():
    home = os.path.expanduser("~")
    A = DeclareLaunchArgument
    return LaunchDescription([
        # 로봇 · SLAM
        A("stage2_delay_s", default_value="20", description="카메라 · 인지 · Nav2 · mission 을 이만큼 뒤에 띄움 [s] (컨트롤러 먼저). 0 = 한꺼번에"),
        A("hardware", default_value="mock", choices=["mock", "opencr"], description="mock = 가짜 모터 / opencr = 실물"),
        A("opencr_ports", default_value="auto", description="실물 보드 'bus_drive=<주행>,bus_arm=<팔>' (run_robot.sh 가 채움)"),
        A("hardware_map", default_value="", description="비우면 전체 hardware_map.yaml (팔 포함). drive_only 는 안 됨"),
        A("lidar", default_value="false", choices=["true", "false"], description="라이다 드라이버 (실물 true 필수)"),
        A("lidar_front_port", default_value="", description="앞 라이다 /dev/serial/by-path/…"),
        A("lidar_rear_port", default_value="", description="뒤 라이다 /dev/serial/by-path/…"),
        A("slam_mode", default_value="mapping", choices=["mapping", "localization"],
          description="mapping = 저장 지도 이어 그리기 (시연 기본, map 비우면 새로) / localization = 지도 고정"),
        A("map", default_value="", description="저장 지도 (확장자 빼고, .posegraph · .data)"),
        A("guard", default_value="true", choices=["true", "false"], description="pose_guard (/localization_health)"),
        A("use_rviz", default_value="false", choices=["true", "false"], description="로봇 모델 rviz"),
        A("slam_rviz", default_value="false", choices=["true", "false"], description="지도 · 스캔 rviz"),
        # 카메라 · 인지
        A("dds_transport", default_value="LARGE_DATA", description="FASTDDS_BUILTIN_TRANSPORTS ('' 면 안 바꿈)"),
        A("wrist_cam", default_value="false", choices=["true", "false"], description="손목 D435 (실물 run_robot.sh 는 true)"),
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
        A("dock_vmax", default_value="0.15"),
        A("dock_tol", default_value="0.015"),
        A("mission_hold", default_value="true", choices=["true", "false"], description="true = mission 이 /mission/hold 로 주행 허가"),
        A("rviz", default_value="false", choices=["true", "false"], description="Nav2 rviz"),
        # mission
        A("mission", default_value="true", choices=["true", "false"]),
        A("planning_params", default_value="", description="비우면 레포 config/planning.yaml"),
        A("dice_map", default_value="", description="비우면 레포 maps/dice_map.yaml (arena5 전용 표 등)"),
        A("held_fresh", default_value="false", choices=["true", "false"], description="선택: 쥔 퍽 신선한 측정 (rack_tags 필요)"),
        A("rack_tags", default_value="false", choices=["true", "false"], description="rack_tag_node (적재함 보정 때만)"),
        OpaqueFunction(function=make),
    ])
