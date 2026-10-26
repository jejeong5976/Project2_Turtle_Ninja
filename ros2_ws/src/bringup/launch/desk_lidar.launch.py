"""Temporary desk alignment: front scan frame is the reference; no robot model."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    pose = {}
    for name in ("rear_x", "rear_y", "rear_z", "rear_roll", "rear_pitch", "rear_yaw"):
        try:
            pose[name] = float(arg(name))
        except ValueError as exc:
            raise RuntimeError(name + " must be a number (metres or radians)") from exc
        if not math.isfinite(pose[name]):
            raise RuntimeError(name + " must be finite")
    if arg("stationary_test").lower() != "true":
        raise RuntimeError("Desk alignment requires stationary_test:=true")
    if not Path(arg("driver_config")).is_file():
        raise RuntimeError("driver_config file does not exist: " + arg("driver_config"))
    get_package_prefix("coin_d4_experimental_driver")
    get_package_prefix("tf2_ros")
    bringup = Path(get_package_share_directory("bringup"))
    # Keep this checkpoint independent of the provisional robot model launch.
    drivers = []
    for side in ("front", "rear"):
        overrides = {"stationary_test": True, "use_sim_time": False,
                     "frame_id": "lidar_" + side + "_link", "topic_name": "scan"}
        if arg(side + "_port"):
            overrides["port"] = arg(side + "_port")
        drivers.append(Node(
            package="coin_d4_experimental_driver", executable="experimental_coin_d4_node",
            namespace="lidar_" + side, name="driver", output="screen",
            parameters=[arg("driver_config"), overrides]))
    transform = Node(package="tf2_ros", executable="static_transform_publisher",
        name="desk_rear_lidar_tf", output="screen", arguments=[
            "--x", str(pose["rear_x"]), "--y", str(pose["rear_y"]), "--z", str(pose["rear_z"]),
            "--roll", str(pose["rear_roll"]), "--pitch", str(pose["rear_pitch"]),
            "--yaw", str(pose["rear_yaw"]),
            "--frame-id", "lidar_front_link", "--child-frame-id", "lidar_rear_link"])
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", str(bringup / "rviz/desk_lidar.rviz")],
                condition=IfCondition(LaunchConfiguration("use_rviz")))
    return drivers + [transform, rviz]


def generate_launch_description():
    config = Path(get_package_share_directory("bringup")) / "config/lidar_drivers.yaml"
    args = [DeclareLaunchArgument(name, description="Rear scan origin pose relative to front scan frame; metres/radians")
            for name in ("rear_x", "rear_y", "rear_z", "rear_roll", "rear_pitch", "rear_yaw")]
    return LaunchDescription(args + [
        DeclareLaunchArgument("stationary_test", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("driver_config", default_value=str(config)),
        DeclareLaunchArgument("front_port", default_value=""),
        DeclareLaunchArgument("rear_port", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
