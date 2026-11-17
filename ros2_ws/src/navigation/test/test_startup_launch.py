"""시작 복구 launch 설정 확인 (실물 · ROS 노드 없이 launch 파일만 읽음).

nav2.launch.py startup_recovery:
  true (기본) = lifecycle_manager autostart + respawn, 감시기 recover_only
  own         = autostart false, 감시기가 시작
  false       = autostart 만 (감시기 · respawn 없음)
"""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

SOURCE = Path(__file__).resolve().parents[1] / "launch" / "nav2.launch.py"
spec = importlib.util.spec_from_file_location("nav2_launch_under_test", SOURCE)
nav2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav2)


def test_default_is_recover_only():
    ctx = LaunchContext()
    for a in nav2.generate_launch_description().entities:
        if isinstance(a, DeclareLaunchArgument):
            a.execute(ctx)
    assert ctx.launch_configurations["startup_recovery"] == "true"


def test_recover_mode_keeps_autostart_and_respawns_managers():
    manager, respawn, wd = nav2.startup_config("true", False, True, nav2.NAVIGATION_RL)
    assert manager["autostart"] is True
    assert manager["bond_timeout"] == 15.0
    assert "service_timeout" not in manager          # autostart 때와 같은 Nav2 기본 (false 모드와 같음)
    assert respawn == {"respawn": True, "respawn_delay": 2.0}
    assert wd["recover_only"] is True
    assert wd["manage_localization"] is True
    assert wd["manager_timeout_s"] == 15.0
    assert wd["localization_nodes"] == ["map_server", "amcl"]
    assert wd["navigation_nodes"] == ["planner_server", "collision_monitor"]


def test_own_mode_is_previous_watchdog_owned_startup():
    manager, respawn, wd = nav2.startup_config("own", False, True, nav2.NAVIGATION_RL)
    assert manager["autostart"] is False
    assert manager["service_timeout"] == 15.0
    assert respawn == {}
    assert wd["recover_only"] is False
    assert "manager_timeout_s" not in wd


def test_false_mode_is_plain_autostart():
    manager, respawn, wd = nav2.startup_config("false", False, True, nav2.NAVIGATION_RL)
    assert manager == {"use_sim_time": False, "autostart": True, "bond_timeout": 15.0}
    assert respawn == {}
    assert wd is None


def test_slam_localization_does_not_manage_amcl():
    _, _, wd = nav2.startup_config("true", True, False, nav2.NAVIGATION_MPPI)
    assert wd["manage_localization"] is False
    assert wd["use_sim_time"] is True
    assert wd["navigation_nodes"] == nav2.NAVIGATION_MPPI


def test_unknown_mode_rejected():
    with pytest.raises(RuntimeError):
        nav2.startup_config("maybe", False, True, nav2.NAVIGATION_RL)
