"""
로봇 모델·ros2_control·바퀴/팔 컨트롤러·control_master 실행.
기본 hardware:=mock. 인지·미션 노드와 카메라·라이다 드라이버는 별도 담당 패키지에서 구성한다.
설정은 bringup이 설치한 share/bringup/config에서 읽는다.
실물 OpenCR 실행은 보드·펌웨어·포트·기계 안전 조건의 별도 확인이 필요하다.
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
    "swerve_controller",       # 바퀴 (swerve_controller/SwerveController)
    "arm_controller",
    "gripper_controller",
]


def cfg(name):
    return PathJoinSubstitution([FindPackageShare("bringup"), "config", name])


def generate_launch_description():
    desc = FindPackageShare("description")
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | opencr"),
        DeclareLaunchArgument("hardware_map", default_value=cfg("hardware_map.yaml")),
        DeclareLaunchArgument("opencr_port", default_value="auto",
                              description="hardware:=opencr 일 때 포트 — 보드 1장일 때만 (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("opencr_ports", default_value="auto",
                              description="보드마다 포트: bus_drive=/tmp/opencr_drive,bus_arm=/tmp/opencr_arm (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        # 팔 받침 위치 — 미확정 (robot.urdf.xacro 주석). 실측값이 나오면 여기서 바로 바꿔 볼 수 있게
        DeclareLaunchArgument("arm_mount_x", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0"),
    ]
    is_opencr = PythonExpression(["'", arg("hardware"), "' == 'opencr'"])

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
    ctrl_yaml = cfg("controllers.yaml")

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="both"),

        # Lyrical 의 ros2_control_node 는 URDF 를 /robot_description 토픽으로 받음 (robot_state_publisher 가 방송)
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[ctrl_yaml], remappings=[("~/robot_description", "/robot_description")], output="both"),

        Node(package="manipulation", executable="control_master",
             parameters=[cfg("control.yaml")], output="both"),

        Node(package="rviz2", executable="rviz2", condition=IfCondition(arg("use_rviz")),
             arguments=["-d", PathJoinSubstitution([desc, "rviz", "navigation.rviz"]), "-f", "odom"]),
    ]

    # 컨트롤러마다 spawner 하나. --param-file: Jazzy 부터 컨트롤러가 controller_manager 설정을 안 물려받음
    def spawner(name, **kw):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--param-file", ctrl_yaml], output="both", **kw)

    nodes += [spawner(c) for c in CONTROLLERS]
    nodes.append(spawner("imu_sensor_broadcaster", condition=IfCondition(is_opencr)))   # IMU 는 hardware 플러그인만 채움

    return LaunchDescription(args + nodes)
