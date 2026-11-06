"""Exercise the actual Python launch API without executing any ROS nodes."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.parameter_descriptions import ParameterFile


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_overlay_inclusion_parameter_substitution_and_main_launch(tmp_path):
    share = Path(get_package_share_directory("body_route_controller"))
    context = LaunchContext()
    # Only file/launch handling is under test: no map server or other node runs.
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: synthetic.pgm\nresolution: 0.01\n")
    context.launch_configurations.update(
        map=str(map_file), use_sim_time="false", initial_pose="", rviz="false")
    entry = load_module(share / "launch/body_route.launch.py", "body_launch")
    assert len(entry.start(context)) == 1
    params = ParameterFile(str(share / "config/nav2.yaml"), allow_substs=True)
    try:
        config = yaml.safe_load(Path(params.evaluate(context)).read_text())
        bt_path = Path(config["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"])
        tree = ET.parse(bt_path)
        assert len(tree.findall(".//FollowPath")) == 1
        assert not tree.findall(".//Spin") and not tree.findall(".//BackUp")
        context.launch_configurations.update(
            params=str(share / "config/nav2.yaml"), localization="amcl", mission_hold="true")
        navigation = load_module(
            Path(get_package_share_directory("navigation")) / "launch/nav2.launch.py",
            "main_nav_launch")
        actions = navigation.make_nodes(context)
        assert len(actions) == 10  # AMCL/map, navigation, hold and lifecycle managers.
        assert config["controller_server"]["ros__parameters"]["FollowPath"]["require_hold_signal"]
        for name in ("local_costmap", "global_costmap"):
            costmap = config[name][name]["ros__parameters"]
            assert "static_layer" in costmap["plugins"]
            assert costmap["obstacle_layer"]["combination_method"] == 2
            assert costmap["inflation_layer"]["inflate_around_unknown"] is True
    finally:
        params.cleanup()


def test_generated_profiles_and_cli_rejections(tmp_path):
    import subprocess
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix("body_route_controller")) / "lib/body_route_controller/configure_trial"
    share = Path(get_package_share_directory("body_route_controller"))
    entry = load_module(share / "launch/body_route.launch.py", "comparison_launch")
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: test.pgm\nresolution: 0.01\n")
    for mode in ("custom", "original"):
        folder = tmp_path / mode
        result = subprocess.run([str(binary), "--mode", mode, "--speed", "0.08", "--yaw-rate", "0.3",
                                 "--output", str(folder)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        drive = yaml.safe_load((folder / "drive_limits.yaml").read_text())
        trial = yaml.safe_load((folder / "trial.yaml").read_text())["body_route_trial"]["ros__parameters"]
        assert type(drive["heading_hold_kp"]) is float
        assert drive["max_linear"] == trial["speed"] == 0.08
        assert drive["max_angular"] == trial["yaw_rate"] == 0.3
        assert trial["execute"] is False
        context = LaunchContext()
        context.launch_configurations.update(map=str(map_file), params=str(folder / "nav2.yaml"),
                                             use_sim_time="false", initial_pose="", rviz="false")
        actions = entry.start(context)
        assert dict(actions[0].launch_arguments)["params"] == str(folder / "nav2.yaml")
        params = ParameterFile(str(folder / "nav2.yaml"), allow_substs=True)
        try:
            resolved = yaml.safe_load(Path(params.evaluate(context)).read_text())
            assert Path(resolved["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]).is_file()
            plugin = resolved["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
            assert plugin == ("body_route_controller::BodyRouteController" if mode == "custom" else "nav2_mppi_controller::MPPIController")
        finally:
            params.cleanup()
        assert subprocess.run([str(binary), "--output", str(folder)], capture_output=True).returncode != 0
    for option, value in (("--mode", "typo"), ("--speed", "nan"), ("--speed", "0"),
                          ("--speed", "0.5"), ("--speed", "0.05oops"), ("--yaw-rate", "0")):
        folder = tmp_path / "invalid"
        assert subprocess.run([str(binary), option, value, "--output", str(folder)], capture_output=True).returncode != 0
        assert not folder.exists()

    folder = tmp_path / "integral_doubles"
    assert subprocess.run([str(binary), "--mode", "original", "--speed", "0.1", "--yaw-rate", "1.0",
                           "--linear-accel", "1.0", "--angular-accel", "6.0", "--output", str(folder)], capture_output=True).returncode == 0
    drive = yaml.safe_load((folder / "drive_limits.yaml").read_text())
    trial = yaml.safe_load((folder / "trial.yaml").read_text())["body_route_trial"]["ros__parameters"]
    nav = yaml.safe_load((folder / "nav2.yaml").read_text())["controller_server"]["ros__parameters"]["FollowPath"]
    for name in ("heading_hold_kp", "max_angular", "max_linear_accel", "max_angular_accel"):
        assert type(drive[name]) is float
    for name in ("yaw_rate", "linear_accel", "angular_accel"):
        assert type(trial[name]) is float
    for name in ("wz_max", "ax_max", "ax_min", "ay_max", "ay_min", "az_max"):
        assert type(nav[name]) is float


def test_observation_profiles_require_matching_execution_guard(tmp_path):
    import subprocess
    import pytest
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix("body_route_controller")) / "lib/body_route_controller/configure_trial"
    share = Path(get_package_share_directory("body_route_controller"))
    entry = load_module(share / "launch/body_route.launch.py", "observation_launch")
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: test.pgm\nresolution: 0.01\n")
    for mode in ("custom", "original"):
        folder = tmp_path / mode
        result = subprocess.run([str(binary), "--mode", mode, "--unknown-policy", "observe-before-enter",
                                 "--unknown-penalty", "3.0", "--output", str(folder)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        context = LaunchContext()
        context.launch_configurations.update(map=str(map_file), params=str(folder / "nav2.yaml"),
                                             use_sim_time="false", initial_pose="", rviz="false")
        assert len(entry.start(context)) == 1
        config = yaml.safe_load((folder / "nav2.yaml").read_text())
        config["controller_server"]["ros__parameters"]["FollowPath"]["plugin"] = "nav2_mppi_controller::MPPIController"
        (folder / "nav2.yaml").write_text(yaml.safe_dump(config))
        with pytest.raises(RuntimeError, match="must match"):
            entry.start(context)
    for option, value in (("--unknown-policy", "free"), ("--unknown-penalty", "0"),
                          ("--unknown-penalty", "nan"), ("--observation-ttl", "0")):
        assert subprocess.run([str(binary), option, value, "--output", str(tmp_path / "invalid")], capture_output=True).returncode != 0


def test_physical_front_policy_rejects_mask_rear_and_automatic_pose(tmp_path):
    import pytest
    share = Path(get_package_share_directory("body_route_controller"))
    entry = load_module(share / "launch/body_route.launch.py", "front_policy_launch")
    config = yaml.safe_load((share / "config/nav2.yaml").read_text())
    amcl = config["amcl"]["ros__parameters"]
    assert amcl["scan_topic"] == "/lidar_front/scan"
    assert amcl["set_initial_pose"] is False
    assert amcl["always_reset_initial_pose"] is True
    for name in ("local_costmap", "global_costmap"):
        obstacle = config[name][name]["ros__parameters"]["obstacle_layer"]
        topics = {obstacle[source]["topic"] for source in obstacle["observation_sources"].split()}
        assert topics == {"/lidar_front/scan", "/lidar_rear/scan"}
    monitor = config["collision_monitor"]["ros__parameters"]
    assert {monitor[source]["topic"] for source in monitor["observation_sources"]} == {"/lidar_front/scan", "/lidar_rear/scan"}
    path = tmp_path / "nav2.yaml"
    map_file = tmp_path / "map.yaml"
    map_file.write_text("image: synthetic.pgm\nresolution: 0.01\n")
    context = LaunchContext()
    context.launch_configurations.update(map=str(map_file), params=str(path), use_sim_time="false", initial_pose="", rviz="false")
    for topic in ("/localization/start_scan", "/lidar_rear/scan"):
        amcl["scan_topic"] = topic
        path.write_text(yaml.safe_dump(config))
        with pytest.raises(RuntimeError, match="front/scan"):
            entry.start(context)
    amcl["scan_topic"] = "/lidar_front/scan"
    path.write_text(yaml.safe_dump(config))
    included = entry.start(context)[0]
    assert dict(included.launch_arguments)["mission_hold"] == "true"
    context.launch_configurations["initial_pose"] = "1,2,90"
    with pytest.raises(RuntimeError, match="fresh reviewed"):
        entry.start(context)
    context.launch_configurations["use_sim_time"] = "true"
    assert len(entry.start(context)) == 1
