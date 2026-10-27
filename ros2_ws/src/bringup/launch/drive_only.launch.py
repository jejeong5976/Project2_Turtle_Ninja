"""Our wheel stack, with recovered calibration and no arm, IMU or LiDAR nodes.

Defaults: mock hardware, feedback mode. Real feedback mode sends CONFIG (motor
settings/torque-off), keeps hardware INACTIVE, and starts only joint feedback.
mode:=drive activates hardware/torque and the swerve controller. The user executes
hardware commands; this launch is not a passive serial listener.
"""
import json
import math
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_profile(hw, controllers):
    """Validate the coupled wheel configuration and derive controller tick limits."""
    names = ["fl", "fr", "rl", "rr"]
    joints = [f"{kind}_{name}_joint" for kind in ("drive", "steer") for name in names]
    if len(hw.get("buses", [])) != 1:
        raise ValueError("Drive-only requires exactly one bus")
    bus = hw["buses"][0]
    if (bus.get("name") != "bus_drive" or bus.get("joints") != joints or
            set(hw.get("motors", {})) != set(joints) or bus.get("imu", False)):
        raise ValueError("Drive-only requires bus_drive with exactly eight wheel/steering joints and no IMU")
    if hw.get("require_all_motors") is not True:
        raise ValueError("Drive-only requires all eight motors")
    margin = hw.get("steer_margin_ticks")
    if type(margin) is not int or not 0 <= margin <= 2047:
        raise ValueError("steer_margin_ticks must be an integer in 0..2047")
    ids, lower, upper = set(), [], []
    for joint in joints:
        m = hw["motors"][joint]
        motor_id, scale = m.get("id"), m.get("scale")
        if type(motor_id) is not int or not 0 <= motor_id <= 252 or motor_id in ids:
            raise ValueError(f"{joint}: invalid or duplicate motor ID")
        ids.add(motor_id)
        if type(scale) not in (int, float) or scale not in (-1.0, 1.0):
            raise ValueError(f"{joint}: wheel/steering scale must be +1 or -1")
        steering = joint.startswith("steer_")
        if m.get("mode") != ("position" if steering else "velocity"):
            raise ValueError(f"{joint}: wrong operating mode")
        if steering:
            zero = m.get("zero_tick")
            if type(zero) is not int or not 0 <= zero <= 4095:
                raise ValueError(f"{joint}: a measured zero_tick in 0..4095 is required")
            bounds = sorted(scale * (tick-zero) * math.tau/4096 for tick in (margin, 4095-margin))
            lower.append(bounds[0])
            upper.append(bounds[1])
        elif m.get("zero_tick", 2048) != 2048:
            raise ValueError(f"{joint}: velocity-mode zero_tick must remain 2048")
    params = controllers["swerve_controller"]["ros__parameters"]
    if params.get("module_names") != names:
        raise ValueError("Module order must remain fl/fr/rl/rr")
    for key in ("module_x", "module_y", "module_offset"):
        values = params.get(key, [])
        if len(values) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError(f"{key}: four finite geometry values are required")
    radius = params.get("wheel_radius")
    if type(radius) not in (int, float) or not math.isfinite(radius) or radius <= 0:
        raise ValueError("wheel_radius must be finite and positive")
    if params.get("require_motion_mode") is not False or params.get("use_imu") is not False:
        raise ValueError("This wheel-only profile has no arm permission or calibrated IMU")
    return dict(params, steer_min=lower, steer_max=upper)


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    hardware, mode, port = value("hardware"), value("mode"), value("port")
    if hardware not in ("mock", "opencr") or mode not in ("feedback", "drive"):
        raise ValueError("hardware must be mock/opencr; mode must be feedback/drive")
    if hardware == "opencr" and (not port or not Path(port).is_absolute()):
        raise ValueError("Real hardware requires an explicit absolute port:= path for the drive OpenCR")
    hw_path, controller_path = value("hardware_map"), value("controllers")
    with open(hw_path, encoding="utf-8") as f:
        hw = yaml.safe_load(f)
    with open(controller_path, encoding="utf-8") as f:
        controllers = yaml.safe_load(f)
    params = prepare_profile(hw, controllers)
    model = Path(get_package_share_directory("description")) / "urdf/drive_only.urdf.xacro"
    robot = xacro.process_file(str(model), mappings={
        "hardware": hardware, "hardware_map": hw_path, "controllers": controller_path,
        "opencr_port": port if hardware == "opencr" else "auto",
    }).toxml()
    # The spawner loads the complete profile and the two derived bounds. Compact
    # arrays are necessary: controller_manager's spawner splits ROS args on spaces.
    manager = dict(controllers["controller_manager"]["ros__parameters"])
    if mode == "feedback":
        manager["hardware_components_initial_state"] = {"inactive": ["bus_drive"]}
    controller_arguments = ["--param-file", controller_path, "--controller-ros-args",
        "--ros-args -p steer_min:=" + json.dumps(params["steer_min"], separators=(",", ":")) +
        " -p steer_max:=" + json.dumps(params["steer_max"], separators=(",", ":"))]
    nodes = [
        LogInfo(msg=f"Wheel-only {hardware}/{mode}: calibrated +X=old CAD -Y; "
                    "no LiDAR/arm/IMU TF. Real feedback sends CONFIG; drive enables torque."),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(robot, value_type=str)}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[manager], remappings=[("~/robot_description", "/robot_description")], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--param-file", controller_path], output="screen"),
    ]
    if mode == "drive":
        nodes.append(Node(package="controller_manager", executable="spawner",
                          arguments=["swerve_controller"] + controller_arguments, output="screen"))
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory("bringup")) / "config"
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "opencr"]),
        DeclareLaunchArgument("mode", default_value="feedback", choices=["feedback", "drive"]),
        DeclareLaunchArgument("port", default_value="", description="Explicit drive OpenCR path; required for opencr"),
        DeclareLaunchArgument("hardware_map", default_value=str(config / "hardware_map_drive_only.yaml")),
        DeclareLaunchArgument("controllers", default_value=str(config / "controllers_drive_only.yaml")),
        OpaqueFunction(function=setup),
    ])
