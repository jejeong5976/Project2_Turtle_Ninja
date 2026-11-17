"""켜는 순서 확인: mission_system.launch.py 가 Nav2 · mission 은 stage2_delay_s (20 s),
카메라 · 인지 (bringup.launch.py) 는 그보다 camera_delay_s (10 s) 늦게 띄우고, 시작 복구 기본이 true 인지.

launch 파일만 읽음 (노드 실행 안 함). navigation · bringup · mission 이 설치된 환경 (colcon test) 에서 돎.
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.utilities import type_utils

SOURCE = Path(__file__).resolve().parents[1] / "launch" / "mission_system.launch.py"
spec = importlib.util.spec_from_file_location("mission_system_under_test", SOURCE)
ms = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ms)


def context(extra=None):
    ctx = LaunchContext()
    for a in ms.generate_launch_description().entities:
        if isinstance(a, DeclareLaunchArgument):
            a.execute(ctx)
    ctx.launch_configurations.update(extra or {})
    return ctx


def timers(ctx):
    out = {}
    for a in ms.make(ctx):
        if isinstance(a, TimerAction):
            period = type_utils.perform_typed_substitution(ctx, a.period, float)
            names = []
            for x in a.actions:
                if isinstance(x, IncludeLaunchDescription):
                    x.launch_description_source.get_launch_description(ctx)   # 경로 풀기 (파일 읽기만)
                    names.append(Path(x.launch_description_source.location).name)
            out[period] = names
    return out


def test_defaults():
    c = context()
    assert c.launch_configurations["startup_recovery"] == "true"
    assert c.launch_configurations["stage2_delay_s"] == "20"
    assert c.launch_configurations["camera_delay_s"] == "10"


def test_cameras_start_after_nav2():
    t = timers(context({"wrist_cam": "true", "wide_cam": "true"}))
    assert sorted(t) == [20.0, 30.0]
    assert "nav2.launch.py" in t[20.0] and "mission.launch.py" in t[20.0]
    assert "bringup.launch.py" not in t[20.0]
    assert t[30.0] == ["bringup.launch.py"]


def test_camera_delay_zero_starts_cameras_with_nav2_stage():
    ctx = context({"wrist_cam": "true", "camera_delay_s": "0"})
    acts = [a for a in ms.make(ctx) if isinstance(a, TimerAction)]
    assert [type_utils.perform_typed_substitution(ctx, a.period, float) for a in acts] == [20.0, 20.0]


def test_no_cameras_means_no_camera_timer():
    t = timers(context())
    assert sorted(t) == [20.0]
