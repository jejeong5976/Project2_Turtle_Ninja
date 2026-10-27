"""Exercise Python launch/Xacro APIs without executing any launch actions.

Portable geometry/motor checks live in validate_profile.cpp. This file is only
the Python launch-framework adapter and negative configuration tests.
"""
import copy
import importlib.util
import json
import os
import sys
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
os.environ["ROS_LOG_DIR"] = str(out / "ros_log")
bringup = Path(get_package_share_directory("bringup"))
description = Path(get_package_share_directory("description"))
spec = importlib.util.spec_from_file_location("drive_launch", bringup / "launch/drive_only.launch.py")
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
hw_path = bringup / "config/hardware_map_drive_only.yaml"
cfg_path = bringup / "config/controllers_drive_only.yaml"
hw = yaml.safe_load(hw_path.read_text())
cfg = yaml.safe_load(cfg_path.read_text())
params = launch.prepare_profile(hw, cfg)
(out / "resolved_controller.yaml").write_text(yaml.safe_dump(params))

for hardware in ("mock", "opencr"):
    xml = xacro.process_file(str(description / "urdf/drive_only.urdf.xacro"), mappings={
        "hardware": hardware, "hardware_map": str(hw_path), "controllers": str(cfg_path),
        "opencr_port": "/nonexistent/offline-review",
    }).toxml()
    (out / f"drive_only_{hardware}.urdf").write_text(xml)
    root = ET.fromstring(xml)
    assert len(root.findall("ros2_control")) == 1
    assert len(root.findall("ros2_control/joint")) == 8
    assert not root.findall("ros2_control/sensor")
    assert not any(e.attrib.get("name", "").startswith(("lidar_", "imu", "joint1", "base_scan")) for e in root)
    assert root.find("ros2_control/hardware/plugin").text == (
        "mock_components/GenericSystem" if hardware == "mock" else "hardware/OpenCRHardware")


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(hardware="mock", mode="feedback", port="",
        hardware_map=str(hw_path), controllers=str(cfg_path)) | overrides)
    return ctx


for mode in ("feedback", "drive"):
    with patch.object(launch, "Node", side_effect=lambda **kw: kw):
        nodes = [n for n in launch.setup(context(mode=mode)) if isinstance(n, dict)]
    assert [n["package"] for n in nodes] == ["robot_state_publisher"] + ["controller_manager"] * (2 if mode == "feedback" else 3)
    manager = nodes[1]["parameters"][0]
    assert manager["enforce_command_limits"] is True
    if mode == "feedback":
        assert manager["hardware_components_initial_state"]["inactive"] == ["bus_drive"]
        assert nodes[-1]["arguments"][0] == "joint_state_broadcaster"
    else:
        assert "hardware_components_initial_state" not in manager
        assert nodes[-1]["arguments"][0] == "swerve_controller"
        args = nodes[-1]["arguments"][-1].split()
        assert args[0] == "--ros-args"
        assert json.loads(args[2].split(":=", 1)[1]) == params["steer_min"]
        assert json.loads(args[4].split(":=", 1)[1]) == params["steer_max"]

bad_maps = []
for change in ("duplicate_id", "missing_zero", "null_zero", "bad_scale", "wrong_mode", "arm", "imu", "two_buses", "margin", "partial"):
    bad = copy.deepcopy(hw)
    if change == "duplicate_id": bad["motors"]["drive_fr_joint"]["id"] = 1
    if change == "missing_zero": del bad["motors"]["steer_fl_joint"]["zero_tick"]
    if change == "null_zero": bad["motors"]["steer_fl_joint"]["zero_tick"] = None
    if change == "bad_scale": bad["motors"]["drive_fl_joint"]["scale"] = True
    if change == "wrong_mode": bad["motors"]["drive_fl_joint"]["mode"] = "position"
    if change == "arm": bad["motors"]["joint1"] = {"id": 21}
    if change == "imu": bad["buses"][0]["imu"] = True
    if change == "two_buses": bad["buses"].append(copy.deepcopy(bad["buses"][0]))
    if change == "margin": bad["steer_margin_ticks"] = 2048
    if change == "partial": bad["require_all_motors"] = False
    path = out / f"invalid_{change}.yaml"
    path.write_text(yaml.safe_dump(bad))
    bad_maps.append(path)
cases = [context(hardware_map=str(p)) for p in bad_maps]
cases += [context(hardware="opencr"), context(hardware="opencr", port="ttyACM0"), context(hardware="invalid")]
for ctx in cases:
    with patch.object(launch, "Node") as node:
        try:
            launch.setup(ctx)
        except (ValueError, KeyError, TypeError):
            pass
        else:
            raise AssertionError("Invalid profile reached node creation")
        node.assert_not_called()

# The shared macro change must preserve full-robot holding-torque defaults.
full = ET.fromstring(xacro.process_file(str(description / "urdf/robot.urdf.xacro"), mappings={
    "hardware": "opencr", "with_lidars": "false",
    "hardware_map": str(bringup / "config/hardware_map.yaml"),
}).toxml())
assert len(full.findall("ros2_control")) == 2
assert all(e.text == "false" for e in full.findall("ros2_control/hardware/param[@name='torque_off_on_deactivate']"))
assert not any(e.attrib.get("name", "").startswith("lidar_") for e in full)
mounts = yaml.safe_load((bringup / "config/lidar_mounts.yaml").read_text())
assert all(mounts[k] is None for k in ("plate_x", "plate_y", "plate_yaw"))
print(f"PASS: mock/opencr XML, feedback/drive launch declarations, {len(cases)} fail-before-node cases, unchanged full-robot torque defaults and mounting unknowns. No actions executed.")
