"""Standalone LDS-03 bringup; never starts wheel/arm control nodes."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    variant = arg("driver_variant")
    stationary = arg("stationary_test").lower() == "true"
    if variant == "experimental_stationary":
        if not stationary:
            raise RuntimeError("experimental_stationary requires stationary_test:=true")
        package, executable = "coin_d4_experimental_driver", "experimental_coin_d4_node"
    elif variant == "original":
        package, executable = "coin_d4_driver", "single_coin_d4_node"
    else:
        raise RuntimeError("Unknown driver_variant: " + variant)
    if not Path(arg("driver_config")).is_file():
        raise RuntimeError("driver_config file does not exist: " + arg("driver_config"))
    # Fail before starting any child process if its selected package is unavailable.
    get_package_prefix(package)
    desc = Path(get_package_share_directory("description"))
    nodes = []
    # Expand before starting any process: unknown geometry must fail before serial I/O.
    if arg("publish_model").lower() == "true":
        model = xacro.process_file(str(desc / "urdf/robot.urdf.xacro"), mappings={
            "with_lidars": "true", "lidar_mounts": arg("lidar_mounts"),
            "hardware_map": str(Path(get_package_share_directory("bringup")) /
                                "config/hardware_map.yaml"),
        }).toxml()
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          parameters=[{"robot_description": model}], output="screen"))
    for side in ("front", "rear"):
        overrides = {}
        if arg(side + "_port"):
            overrides["port"] = arg(side + "_port")
        if variant == "experimental_stationary":
            overrides["stationary_test"] = stationary
        parameters = [arg("driver_config")]
        if overrides:
            parameters.append(overrides)
        nodes.append(Node(
            package=package, executable=executable,
            namespace="lidar_" + side, name="driver",
            parameters=parameters,
            condition=IfCondition(LaunchConfiguration("enable_" + side)), output="screen"))
    nodes.append(Node(package="rviz2", executable="rviz2",
                      arguments=["-d", str(desc / "rviz/navigation.rviz"), "-f", arg("rviz_fixed_frame")],
                      condition=IfCondition(LaunchConfiguration("use_rviz"))))
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory("bringup")) / "config"
    return LaunchDescription([
        DeclareLaunchArgument("lidar_mounts", default_value=str(config / "lidar_mounts.yaml")),
        DeclareLaunchArgument("driver_config", default_value=str(config / "lidar_drivers.yaml")),
        DeclareLaunchArgument("driver_variant", default_value="experimental_stationary",
                              choices=["original", "experimental_stationary"],
                              description="Both variants use /lidar_front/scan and /lidar_rear/scan"),
        DeclareLaunchArgument("stationary_test", default_value="false", choices=["true", "false"],
                              description="Required acknowledgment for the stationary experimental driver"),
        DeclareLaunchArgument("front_port", default_value="", description="Optional override of YAML front port"),
        DeclareLaunchArgument("rear_port", default_value="", description="Optional override of YAML rear port"),
        DeclareLaunchArgument("rviz_fixed_frame", default_value="base_link"),
        DeclareLaunchArgument("publish_model", default_value="true", choices=["true", "false"],
                              description="false for raw-only tests or an existing robot TF owner"),
        DeclareLaunchArgument("enable_front", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_rear", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
