"""
주행 + 앞 · 뒤 라이다 + 라이다 합치기 + slam_toolbox + 위치 감시.

  # 지도 만들기 (실물): tools/jetson/run_robot.sh slam opencr  (이 launch 를 부름)
  ros2 launch navigation slam.launch.py hardware:=opencr opencr_ports:=bus_drive=<주행 보드> lidar_front_port:=<앞> lidar_rear_port:=<뒤>
  # 저장한 지도 이어 그리기 (시연 기본): slam_mode:=mapping map:=$HOME/maps/arena  (확장자 빼고 — arena.posegraph · arena.data)
  # 위치 찾기만 (지도 고정): slam_mode:=localization map:=$HOME/maps/arena
  # 라이다 없이 (bag 재생 · 시뮬): robot:=false lidar:=false use_sim_time:=true — 스캔은 밖에서 /lidar_front/scan · /lidar_rear/scan
  # 뒤 라이다 없이: lidar_rear_port:='' (앞만으로 합침)

띄우는 것
  bringup/robot.launch.py: 모델 · ros2_control · swerve_controller (/odom · odom→base_footprint TF). 기본은 바퀴만 (arm:=false).
  coin_d4_driver/multi_coin_d4_node: 앞 → /lidar_front/scan (lidar_front_link), 뒤 → /lidar_rear/scan (lidar_rear_link)
  scan_merger: 둘을 몸 중심 360° /scan (base_footprint) 하나로. 한쪽이 끊기면 /scan 을 내지 않음
  slam_toolbox: mapping (online_async, map:= 주면 저장 지도 불러와 이어 그림) 또는 localization (map:= 필수) — config/slam_toolbox.yaml
  pose_guard: /localization_health (정상 · 경고 · 정지) · /imu_motion_bias (guard:=false 면 안 띄움)
TF 주인은 하나씩: odom→base_footprint = swerve_controller, base_footprint→lidar_*_link = robot_state_publisher, map→odom = slam_toolbox.
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIDAR_POSE = ["lidar_front_x", "lidar_front_y", "lidar_front_z", "lidar_front_yaw",
              "lidar_rear_x", "lidar_rear_y", "lidar_rear_z", "lidar_rear_yaw"]
ROBOT_ARGS = ["hardware", "hardware_map", "opencr_port", "opencr_ports", "arm", "use_rviz", "enforce_command_limits"] + LIDAR_POSE
LIDAR_DEFAULTS = {"lidar_front_x": "0.1145", "lidar_front_y": "0.0", "lidar_front_z": "0.060", "lidar_front_yaw": "3.141592653589793",
                  "lidar_rear_x": "-0.1145", "lidar_rear_y": "0.0", "lidar_rear_z": "0.060", "lidar_rear_yaw": "0.0"}


def _sim_time(context):
    return LaunchConfiguration("use_sim_time").perform(context) == "true"


def _check(context):
    if _sim_time(context) and LaunchConfiguration("robot").perform(context) == "true":
        raise RuntimeError("use_sim_time:=true 는 robot:=false 와 같이 (로봇 bringup 은 실제 시계만 씀 — 시계가 갈라짐)")
    return []


def _lidars(context):
    """앞 · 뒤 포트 → multi_coin_d4_node + 합치기 (뒤 포트가 비면 앞만)"""
    share = FindPackageShare("navigation").perform(context)
    if LaunchConfiguration("lidar").perform(context) != "true":
        rear = LaunchConfiguration("rear_scan").perform(context) == "true"
        scans, out = ["/lidar_front/scan"] + (["/lidar_rear/scan"] if rear else []), []
    else:
        sides = [(s, LaunchConfiguration(f"lidar_{s}_port").perform(context)) for s in ("front", "rear")]
        sides = [(s, p) for s, p in sides if p]
        if not sides:
            raise RuntimeError("lidar:=true 인데 lidar_front_port · lidar_rear_port 가 둘 다 비었음")
        for s, p in sides:
            if not os.path.exists(p):
                raise RuntimeError(f"{s} 라이다 포트 {p} 없음 — ls -l /dev/serial/by-path/ 로 확인")
        params = {"lidar_list": [f"lidar_{s}" for s, _ in sides]}
        for s, p in sides:
            params[f"lidar_{s}"] = {"port": p, "frame_id": f"lidar_{s}_link", "baudrate": 230400, "version": 4,
                                    "topic_name": f"/lidar_{s}/scan", "reverse": False, "warmup_time": 5}
        scans = [f"/lidar_{s}/scan" for s, _ in sides]
        out = [LogInfo(msg="라이다: " + " · ".join(f"{s} {p}" for s, p in sides)),
               Node(package="coin_d4_driver", executable="multi_coin_d4_node", name="lidars", output="screen",
                    parameters=[{"use_sim_time": False}, params], sigterm_timeout="15", sigkill_timeout="15")]
    out.append(Node(package="navigation", executable="scan_merger", name="scan_merger", output="screen",
                    parameters=[os.path.join(share, "config", "navigation.yaml"),
                                {"scans": scans, "use_sim_time": _sim_time(context)}]))
    return out


def _slam(context):
    mode = LaunchConfiguration("slam_mode").perform(context)
    if mode not in ("mapping", "localization"):
        raise RuntimeError(f"slam_mode 는 mapping | localization (지금 {mode})")
    with open(os.path.join(FindPackageShare("navigation").perform(context), "config", "slam_toolbox.yaml")) as fh:
        d = yaml.safe_load(fh)
    p = d["slam_toolbox"]["ros__parameters"]
    p["mode"] = mode
    out = []
    m = os.path.expanduser(LaunchConfiguration("map").perform(context))
    if mode == "localization" and not m:
        raise RuntimeError("localization 은 map:= 필수 (tools/jetson/save_map.sh 로 저장한 경로, 확장자 빼고)")
    if m:
        # 저장 지도 불러오기 = mapping · localization 공통. 시작 자세 = 지도 첫 노드 (지도 딸 때 출발 자세와 같게 놓아야 함)
        if not (os.path.exists(m + ".posegraph") and os.path.exists(m + ".data")):
            raise RuntimeError(f"지도 {m}.posegraph · .data 가 없음 — tools/jetson/save_map.sh 로 저장한 경로(확장자 빼고)를 map:= 로")
        p["map_file_name"], p["map_start_at_dock"] = m, True
        out.append(LogInfo(msg=f"{'이어 그리기' if mode == 'mapping' else '위치 찾기'}: 지도 {m}"))
    f = tempfile.NamedTemporaryFile("w", prefix="slam_toolbox_", suffix=".yaml", delete=False)
    yaml.safe_dump(d, f)
    f.close()
    launch_file = "localization_launch.py" if mode == "localization" else "online_async_launch.py"
    out.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", launch_file])),
        launch_arguments={"slam_params_file": f.name, "use_sim_time": LaunchConfiguration("use_sim_time"),
                          "autostart": "true", "use_lifecycle_manager": "false"}.items()))
    return out


def _guard(context):
    if LaunchConfiguration("guard").perform(context) != "true":
        return []
    share = FindPackageShare("navigation").perform(context)
    return [Node(package="navigation", executable="pose_guard", name="pose_guard", output="screen",
                 parameters=[os.path.join(share, "config", "navigation.yaml"), {"use_sim_time": _sim_time(context)}])]


def generate_launch_description():
    arg = LaunchConfiguration
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("bringup"), "launch", "robot.launch.py"])),
        launch_arguments={k: arg(k) for k in ROBOT_ARGS}.items(),
        condition=IfCondition(arg("robot")))
    rviz = Node(package="rviz2", executable="rviz2", name="slam_rviz", output="log",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("navigation"), "rviz", "slam_view.rviz"])],
                condition=IfCondition(arg("slam_rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | opencr"),
        DeclareLaunchArgument("arm", default_value="false", choices=["true", "false"], description="false = 팔 OpenCR 을 안 붙이고 바퀴만"),
        # 팔 OpenCR 을 붙이면(arm:=true) 전체 지도, 아니면 주행 보드만
        DeclareLaunchArgument("hardware_map", default_value=PathJoinSubstitution([FindPackageShare("bringup"), "config", PythonExpression(
            ["'hardware_map.yaml' if '", LaunchConfiguration("arm"), "' == 'true' else 'hardware_map_drive_only.yaml'"])])),
        DeclareLaunchArgument("opencr_port", default_value="auto"),
        DeclareLaunchArgument("opencr_ports", default_value="auto"),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"], description="로봇 모델 rviz (bringup)"),
        DeclareLaunchArgument("enforce_command_limits", default_value="true", choices=["true", "false"],
                              description="false = 관절 명령 한계 끔 (bringup robot.launch.py, arm:=false 일 때만)"),
        DeclareLaunchArgument("slam_rviz", default_value="false", choices=["true", "false"], description="지도 · 스캔 rviz (rviz/slam_view.rviz)"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in LIDAR_DEFAULTS.items()],
        DeclareLaunchArgument("robot", default_value="true", choices=["true", "false"], description="false = 로봇 bringup 안 띄움 (bag 재생 · 시뮬 · 이미 떠 있을 때)"),
        DeclareLaunchArgument("lidar", default_value="true", choices=["true", "false"], description="false = 라이다 드라이버 안 띄움 (스캔은 밖에서, 합치기는 띄움)"),
        DeclareLaunchArgument("rear_scan", default_value="true", choices=["true", "false"], description="lidar:=false 일 때 /lidar_rear/scan 도 받나 (앞 라이다 하나뿐인 bag 이면 false)"),
        DeclareLaunchArgument("lidar_front_port", default_value="", description="앞 라이다 시리얼 (/dev/serial/by-path/…), 비우면 안 씀"),
        DeclareLaunchArgument("lidar_rear_port", default_value="", description="뒤 라이다 시리얼, 비우면 안 씀"),
        DeclareLaunchArgument("slam_mode", default_value="mapping", description="mapping | localization"),
        DeclareLaunchArgument("map", default_value="", description="저장한 지도 (확장자 빼고). mapping 이면 이어 그리기 (비우면 새로), localization 이면 필수"),
        DeclareLaunchArgument("guard", default_value="true", choices=["true", "false"], description="위치 감시 pose_guard 띄움"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"], description="bag 재생 · 시뮬 때만 true (robot:=false 와 같이)"),
        OpaqueFunction(function=_check),
        robot,
        OpaqueFunction(function=_lidars),
        OpaqueFunction(function=_slam),
        OpaqueFunction(function=_guard),
        rviz,
    ])
