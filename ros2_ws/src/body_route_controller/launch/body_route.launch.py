"""Load the isolated controller profile through the existing navigation launch.

This launch never starts hardware drivers, releases hold or sends a goal.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    import yaml
    share = Path(get_package_share_directory("body_route_controller"))
    map_file = Path(LaunchConfiguration("map").perform(context)).expanduser()
    if not map_file.is_file():
        raise RuntimeError("Supply the existing scanned occupancy map YAML with map:=...")
    # The main launch supports YAML substitutions through ParameterFile.
    nav = Path(get_package_share_directory("navigation")) / "launch" / "nav2.launch.py"
    params = LaunchConfiguration("params", default="").perform(context)
    params = Path(params).expanduser() if params else share / "config" / "nav2.yaml"
    config = yaml.safe_load(params.read_text())
    if config["controller_server"]["ros__parameters"]["FollowPath"]["plugin"] not in (
            "body_route_controller::BodyRouteController", "nav2_mppi_controller::MPPIController",
            "body_route_controller::ObservationController"):
        raise RuntimeError("Unexpected controller profile")
    policy = config["planner_server"]["ros__parameters"]["GridBased"].get("unknown_policy", "blocked")
    controller = config["controller_server"]["ros__parameters"]
    wrapped = controller["FollowPath"]["plugin"] == "body_route_controller::ObservationController"
    if policy not in ("blocked", "observe-before-enter"):
        raise RuntimeError("Invalid unknown policy")
    if (policy == "observe-before-enter") != wrapped:
        raise RuntimeError("Unknown planning and execution policies must match; use configure_trial")
    for name in ("local_costmap", "global_costmap"):
        cost = config[name][name]["ros__parameters"]
        if wrapped:
            if cost["plugins"] != ["static_layer", "obstacle_layer", "observation_layer", "inflation_layer"]:
                raise RuntimeError("Observation layer must follow obstacles and precede inflation")
            if cost["observation_layer"]["unknown_policy"] != policy:
                raise RuntimeError("Costmap policy mismatch")
        elif "observation_layer" in cost["plugins"]:
            raise RuntimeError("Blocked profile must retain the standard costmap chain")
    if wrapped and controller["general_goal_checker"]["plugin"] != "body_route_controller::ObservedGoalChecker":
        raise RuntimeError("Provisional goals require observed clearance")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav)), launch_arguments={
        "params": str(params),
        "autostart": LaunchConfiguration("autostart", default="true").perform(context),
        "map": str(map_file), "localization": "amcl", "mission_hold": "true",
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
        "initial_pose": LaunchConfiguration("initial_pose").perform(context),
        "rviz": LaunchConfiguration("rviz").perform(context),
    }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("autostart", default_value="true", description="Lifecycle autostart; simulation uses an explicit readiness barrier"),
        DeclareLaunchArgument("params", default_value="", description="Generated comparison nav2.yaml; empty uses slow custom profile"),
        DeclareLaunchArgument("map", description="Scanned occupancy map YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("initial_pose", default_value="", description="Map x,y,yaw_degrees; no truth default"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=start),
    ])
