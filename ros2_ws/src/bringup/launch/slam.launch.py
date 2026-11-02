"""
주행 + 앞 · 뒤 라이다 + slam_toolbox (2026-10-31, 젯슨). 바퀴만 (arm:=false) 이 기본.

  # 지도 만들기 (실물): tools/jetson/run_robot.sh slam opencr  (이 launch 를 부름)
  ros2 launch bringup slam.launch.py hardware:=opencr opencr_ports:=bus_drive=<주행 보드> lidar_front_port:=<앞> lidar_rear_port:=<뒤>
  # 위치 추정 (저장한 지도): slam_mode:=localization map:=$HOME/maps/arena  (확장자 빼고 — arena.posegraph · arena.data)
  # 라이다 없이 (모의 · bag 재생 · 시뮬): lidar:=false — 스캔은 밖에서 /lidar_front/scan · /lidar_rear/scan 으로 들어옴
  # 뒤 라이다 없이: lidar_rear_port:='' (앞만으로 합침)

띄우는 것
  robot.launch.py (모델 · ros2_control · swerve_controller → /odom · odom→base_footprint TF, URDF 에 lidar_front_link · lidar_rear_link)
  coin_d4_driver/multi_coin_d4_node (ROBOTIS 공식, apt ros-lyrical-coin-d4-driver): 앞 → /lidar_front/scan (lidar_front_link),
    뒤 → /lidar_rear/scan (lidar_rear_link). 이름은 팀 Nav2 · 시뮬 브리지(sim/robot_sim/scripts/sim_nav_bridge.py) 와 같음
  bringup/scan_merger.py: 둘을 몸 중심 360° 스캔 /scan (base_footprint) 하나로 — 자기 몸에 맞은 빔 버림 (slam_toolbox · AMCL 은 스캔 하나만 받음)
  slam_toolbox online_async (mapping) 또는 localization — config/slam_mapping.yaml · slam_localization.yaml (scan_topic /scan)
TF 주인은 하나씩: odom→base_footprint = swerve_controller, base_footprint→lidar_*_link = robot_state_publisher, map→odom = slam_toolbox.
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIDAR_POSE = ["lidar_front_x", "lidar_front_y", "lidar_front_z", "lidar_front_yaw",
              "lidar_rear_x", "lidar_rear_y", "lidar_rear_z", "lidar_rear_yaw"]
ROBOT_ARGS = ["hardware", "hardware_map", "opencr_port", "opencr_ports", "arm", "use_rviz"] + LIDAR_POSE
LIDAR_DEFAULTS = {"lidar_front_x": "0.1145", "lidar_front_y": "0.0", "lidar_front_z": "0.060", "lidar_front_yaw": "3.141592653589793",
                  "lidar_rear_x": "-0.1145", "lidar_rear_y": "0.0", "lidar_rear_z": "0.060", "lidar_rear_yaw": "0.0"}


def _lidars(context):
    """앞 · 뒤 포트 → multi_coin_d4_node 설정 + 합치기 노드 (뒤 포트가 비면 앞만)"""
    if LaunchConfiguration("lidar").perform(context) != "true":
        scans = ["/lidar_front/scan", "/lidar_rear/scan"]
        out = []
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
    out.append(Node(package="bringup", executable="scan_merger.py", name="scan_merger", output="screen",
                    parameters=[{"scans": scans, "use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true"}]))
    return out


def _slam(context):
    mode = LaunchConfiguration("slam_mode").perform(context)
    share = FindPackageShare("bringup").perform(context)
    if mode not in ("mapping", "localization"):
        raise RuntimeError(f"slam_mode 는 mapping | localization (지금 {mode})")
    params = os.path.join(share, "config", f"slam_{mode}.yaml")
    out = []
    if mode == "localization":
        m = os.path.expanduser(LaunchConfiguration("map").perform(context))
        if not (os.path.exists(m + ".posegraph") and os.path.exists(m + ".data")):
            raise RuntimeError(f"지도 {m}.posegraph · .data 가 없음 — tools/jetson/save_map.sh 로 저장한 경로(확장자 빼고)를 map:= 로")
        with open(params) as fh:
            d = yaml.safe_load(fh)
        d["slam_toolbox"]["ros__parameters"]["map_file_name"] = m
        f = tempfile.NamedTemporaryFile("w", prefix="nt_slam_loc_", suffix=".yaml", delete=False)
        yaml.safe_dump(d, f); f.close()
        params = f.name
        out.append(LogInfo(msg=f"위치 추정: 지도 {m} (설정 {params})"))
    launch_file = "localization_launch.py" if mode == "localization" else "online_async_launch.py"
    out.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", launch_file])),
        launch_arguments={"slam_params_file": params, "use_sim_time": LaunchConfiguration("use_sim_time"),
                          "autostart": "true", "use_lifecycle_manager": "false"}.items()))
    return out


def generate_launch_description():
    arg = LaunchConfiguration
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("bringup"), "launch", "robot.launch.py"])),
        launch_arguments={k: arg(k) for k in ROBOT_ARGS}.items(),
        condition=IfCondition(arg("robot")))
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | opencr"),
        DeclareLaunchArgument("hardware_map", default_value=PathJoinSubstitution([FindPackageShare("bringup"), "config", "hardware_map_drive_only.yaml"])),
        DeclareLaunchArgument("opencr_port", default_value="auto"),
        DeclareLaunchArgument("opencr_ports", default_value="auto"),
        DeclareLaunchArgument("arm", default_value="false", description="SLAM 은 바퀴만이 기본"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in LIDAR_DEFAULTS.items()],
        DeclareLaunchArgument("robot", default_value="true", description="false = 주행 bringup 안 띄움 (bag 재생 · 시뮬 · 이미 떠 있을 때)"),
        DeclareLaunchArgument("lidar", default_value="true", description="false = 라이다 드라이버 안 띄움 (스캔은 밖에서, 합치기는 띄움)"),
        DeclareLaunchArgument("lidar_front_port", default_value="", description="앞 라이다 시리얼 (/dev/serial/by-path/…), 비우면 안 씀"),
        DeclareLaunchArgument("lidar_rear_port", default_value="", description="뒤 라이다 시리얼, 비우면 안 씀"),
        DeclareLaunchArgument("slam_mode", default_value="mapping", description="mapping | localization"),
        DeclareLaunchArgument("map", default_value="", description="localization 때 저장한 지도 (확장자 빼고)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="bag 재생 · 시뮬 때만 true"),
        robot,
        OpaqueFunction(function=_lidars),
        OpaqueFunction(function=_slam),
    ])
