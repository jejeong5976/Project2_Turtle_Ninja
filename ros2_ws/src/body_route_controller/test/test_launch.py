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
    finally:
        params.cleanup()
