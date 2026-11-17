"""손목 D435 실행과 프로세스 종료 뒤 자동 재시작을 담당함.

입력: 설치된 realsense2_camera의 rs_launch 인자 + wrist_respawn/delay.
출력: /camera/camera 영상과 시작·종료·재시작 로그.
rs_launch.py 4.57.7의 파라미터 목록·기본값·YAML 병합·노드 종류를 사용함.
카메라 설정 둘(깊이 정렬, 드라이버 TF 끄기)만 기본값을 바꿈.
외부 launch에는 respawn 인자가 없으므로 실제 Node 생성에 직접 넣음.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def load_realsense_launch():
    """설치된 드라이버의 파라미터 선언 함수를 읽음. 노드는 실행하지 않음."""
    path = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    spec = importlib.util.spec_from_file_location("wrist_realsense_parameters", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def camera_actions(context, upstream, parameters):
    """rs_launch 4.57.7과 같은 설정으로 노드를 만들고 재시작을 추가함."""
    config_file = LaunchConfiguration("config_file").perform(context)
    from_file = {} if config_file == "''" else upstream.yaml_to_dict(config_file)
    global_settings = os.path.join(os.path.dirname(upstream.__file__), "..", "config", "global_settings.yaml")
    lifecycle = upstream.yaml_to_dict(global_settings).get("use_lifecycle_node", False)
    respawn = IfCondition(LaunchConfiguration("wrist_respawn")).evaluate(context)
    delay = float(LaunchConfiguration("wrist_respawn_delay").perform(context))
    output = LaunchConfiguration("output")
    if os.getenv("ROS_DISTRO") == "foxy":
        output = output.perform(context)
    node_type = LifecycleNode if lifecycle else Node
    camera = node_type(
        package="realsense2_camera", executable="realsense2_camera_node",
        namespace=LaunchConfiguration("camera_namespace"), name=LaunchConfiguration("camera_name"),
        parameters=[upstream.set_configurable_parameters(parameters), from_file],
        output=output,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True, respawn=respawn, respawn_delay=delay)
    starts = 0

    def started(event, _context):
        nonlocal starts
        starts += 1
        action = "시작" if starts == 1 else f"재시작 {starts - 1}회"
        return [LogInfo(msg=f"손목 D435 {action}: PID={event.pid}")]

    def exited(event, event_context):
        if event_context.is_shutdown:
            followup = "launch 종료 중"
        elif respawn:
            followup = f"{delay:g}초 뒤 자동 재시작"
        else:
            followup = "자동 재시작 꺼짐"
        return [LogInfo(msg=f"손목 D435 종료: PID={event.pid}, 종료 코드={event.returncode}, {followup}")]

    return [
        LogInfo(msg=f"손목 D435: {'LifecycleNode' if lifecycle else 'Node'}, "
                    f"자동 재시작={respawn}, 대기={delay:g}초"),
        RegisterEventHandler(OnProcessStart(target_action=camera, on_start=started)),
        RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=exited)),
        camera,
    ]


def generate_launch_description():
    upstream = load_realsense_launch()
    defaults = {"align_depth.enable": "true", "publish_tf": "false"}
    parameters = [dict(param, default=defaults.get(param["name"], param["default"]))
                  for param in upstream.configurable_parameters]
    return LaunchDescription(upstream.declare_configurable_parameters(parameters) + [
        DeclareLaunchArgument("wrist_respawn", default_value="true", description="손목 D435 종료 뒤 자동 재시작"),
        DeclareLaunchArgument("wrist_respawn_delay", default_value="3.0", description="손목 D435 재시작 대기 초"),
        OpaqueFunction(function=camera_actions, kwargs={"upstream": upstream, "parameters": parameters}),
    ])
