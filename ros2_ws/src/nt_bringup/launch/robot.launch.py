"""
robot.launch.py — 로봇(Jetson) 쪽 전부: 모델 + ros2_control + 컨트롤러 + control_master (+ 인지 · 판단 마스터)
=========================================================================================
노드 배치 (D17): 이 파일 = Jetson. Nav2 · SLAM · RViz 는 노트북 (설정은 TODO)
(노트북 한 대로 시험할 땐 둘 다 같은 노트북에서 띄워도 됨 — 토픽은 똑같음)

  # 몸만, 가짜 하드웨어 (기본) — cmd_vel 로 바퀴, /arm_task 로 팔 시험
  ros2 launch nt_bringup robot.launch.py
  # 미션 흐름까지 (마스터 3개, planning dry_run)
  ros2 launch nt_bringup robot.launch.py mission:=true use_rviz:=true
  # 실제 로봇 (Jetson) — OpenCR 2장(주행 · 팔, 2026-10-16) + 펌웨어(nt_opencr) + 플러그인(nt_hardware)
  ros2 launch nt_bringup robot.launch.py hardware:=opencr mission:=true      # 포트 = /dev/opencr_drive · /dev/opencr_arm (udev)
  # 로봇 없이 펌웨어까지: 가짜 보드(tools/fw_sim) 를 보드마다 먼저 띄우고
  ./tools/fw_sim/fw_sim --link /tmp/opencr_drive &  ./tools/fw_sim/fw_sim --link /tmp/opencr_arm &
  ros2 launch nt_bringup robot.launch.py hardware:=opencr opencr_ports:=bus_drive=/tmp/opencr_drive,bus_arm=/tmp/opencr_arm

띄우는 것
  robot_state_publisher : URDF → TF, /robot_description
  ros2_control_node     : 하드웨어 플러그인 + 컨트롤러를 100 Hz 로 돌리는 지휘자 (기성품)
  spawner × 4 (+1)      : controllers.yaml 의 컨트롤러 켜기 (+ opencr 이면 imu_sensor_broadcaster)
                          ⚠ spawner 마다 --param-file 로 controllers.yaml 을 줌 (Jazzy 부터 필수)
                          swerve_controller : /cmd_vel → 바퀴 · 조향, /odom + TF (nt_control 의 컨트롤러, 100 Hz 루프 안)
  control_master        : 팔(/arm_task) + 차례(/control/motion_mode) (nt_control)
  mission:=true 일 때    : perception_master (인지) · planning_master (판단, 기본 dry_run) — 마스터 3개 (D18)

카메라 · 라이다 드라이버는 아직 여기 없음 (TODO): 장비가 정해지면 Node / IncludeLaunchDescription 으로 추가
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = [
    "joint_state_broadcaster",
    "swerve_controller",       # 바퀴 (nt_control/SwerveController)
    "arm_controller",
    "gripper_controller",
]


def cfg(pkg, name):
    return PathJoinSubstitution([FindPackageShare(pkg), "config", name])


def generate_launch_description():
    desc = FindPackageShare("nt_description")
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | opencr"),
        DeclareLaunchArgument("hardware_map", default_value=cfg("nt_description", "hardware_map.yaml")),
        DeclareLaunchArgument("opencr_port", default_value="auto",
                              description="hardware:=opencr 일 때 포트 — 보드 1장일 때만 (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("opencr_ports", default_value="auto",
                              description="보드마다 포트: bus_drive=/tmp/opencr_drive,bus_arm=/tmp/opencr_arm (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("mission", default_value="false",
                              description="true = perception_master · planning_master 도 (미션 흐름)"),
        DeclareLaunchArgument("with_perception", default_value="true", description="mission 때 인지 마스터 켜기"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        # 팔 받침 위치 — 미확정 (robot.urdf.xacro 주석). 실측값이 나오면 여기서 바로 바꿔 볼 수 있게
        DeclareLaunchArgument("arm_mount_x", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0"),
    ]
    is_opencr = PythonExpression(["'", arg("hardware"), "' == 'opencr'"])
    with_perception = PythonExpression(["'", arg("mission"), "' == 'true' and '", arg("with_perception"), "' == 'true'"])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([desc, "urdf", "robot.urdf.xacro"]),
            " hardware:=", arg("hardware"),
            " hardware_map:=", arg("hardware_map"),
            " opencr_port:=", arg("opencr_port"),
            " opencr_ports:=", arg("opencr_ports"),
            " arm_mount_x:=", arg("arm_mount_x"),
            " arm_mount_y:=", arg("arm_mount_y"),
        ]),
        value_type=str,
    )
    ctrl_yaml = cfg("nt_bringup", "controllers.yaml")

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="both"),

        # Lyrical 의 ros2_control_node 는 URDF 를 /robot_description 토픽으로 받음 (robot_state_publisher 가 방송)
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[ctrl_yaml], remappings=[("~/robot_description", "/robot_description")], output="both"),

        Node(package="nt_control", executable="control_master",
             parameters=[cfg("nt_control", "control.yaml")], output="both"),

        Node(package="nt_perception", executable="perception_master",
             parameters=[cfg("nt_perception", "perception.yaml")], condition=IfCondition(with_perception),
             output="both"),
        Node(package="nt_planning", executable="planning_master",
             parameters=[cfg("nt_planning", "planning.yaml")], condition=IfCondition(arg("mission")),
             output="both"),

        Node(package="rviz2", executable="rviz2", condition=IfCondition(arg("use_rviz")),
             arguments=["-d", PathJoinSubstitution([desc, "rviz", "navigation.rviz"]), "-f", "odom"]),
    ]

    # 컨트롤러마다 spawner 하나. --param-file: Jazzy 부터 컨트롤러가 controller_manager 설정을 안 물려받음
    def spawner(name, **kw):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--param-file", ctrl_yaml], output="both", **kw)

    nodes += [spawner(c) for c in CONTROLLERS]
    nodes.append(spawner("imu_sensor_broadcaster", condition=IfCondition(is_opencr)))   # IMU 는 nt_hardware 플러그인만 채움

    return LaunchDescription(args + nodes)
