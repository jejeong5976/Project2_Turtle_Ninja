"""실물 없이 D435 launch 기본값과 실제 프로세스 재시작을 확인함.

source /opt/ros/<distro>/setup.bash 뒤 이 파일을 실행함.
카메라 노드는 가짜 Python 자식으로 교체하므로 장치·ROS 토픽을 사용하지 않음.
"""

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler,
                            SetLaunchConfiguration, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node


SOURCE = Path(__file__).resolve().parents[1] / "launch" / "wrist_camera.launch.py"
spec = importlib.util.spec_from_file_location("wrist_camera_under_test", SOURCE)
wrist = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wrist)


class WristCameraTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.logs = tempfile.TemporaryDirectory(prefix="wrist-launch-test-")
        cls.environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "230",
                                    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "ROS_LOG_DIR": cls.logs.name})
        cls.environment.start()

    @classmethod
    def tearDownClass(cls):
        cls.environment.stop()
        cls.logs.cleanup()

    def context_and_description(self, extra=None):
        context = LaunchContext()
        description = wrist.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update(extra or {})
        return context, description

    def test_upstream_defaults_and_node_configuration(self):
        upstream = wrist.load_realsense_launch()
        context, description = self.context_and_description()
        changed = {"align_depth.enable": "true", "publish_tf": "false"}
        for parameter in upstream.configurable_parameters:
            self.assertEqual(context.launch_configurations[parameter["name"]],
                             changed.get(parameter["name"], parameter["default"]))
        self.assertEqual(context.launch_configurations["camera_namespace"], "camera")
        self.assertEqual(context.launch_configurations["camera_name"], "camera")
        opaque = description.entities[-1]
        with patch.object(wrist, "Node", wraps=Node) as factory:
            opaque.execute(context)
        kwargs = factory.call_args.kwargs
        self.assertEqual(kwargs["package"], "realsense2_camera")
        self.assertEqual(kwargs["executable"], "realsense2_camera_node")
        self.assertIs(kwargs["respawn"], True)
        self.assertEqual(kwargs["respawn_delay"], 3.0)
        self.assertEqual(kwargs["parameters"][1], {})
        self.assertEqual(set(kwargs["parameters"][0]),
                         {p["name"] for p in upstream.configurable_parameters})

    def test_config_file_and_lifecycle_setting_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "camera.yaml"
            config.write_text("enable_depth: false\nserial_no: '12345'\n")
            upstream = wrist.load_realsense_launch()
            original_yaml = upstream.yaml_to_dict

            def load_yaml(path):
                return ({"use_lifecycle_node": True} if str(path).endswith("global_settings.yaml")
                        else original_yaml(path))

            with patch.object(wrist, "load_realsense_launch", return_value=upstream), \
                    patch.object(upstream, "yaml_to_dict", side_effect=load_yaml), \
                    patch.object(wrist, "LifecycleNode", wraps=wrist.LifecycleNode) as factory:
                context, description = self.context_and_description({"config_file": str(config)})
                description.entities[-1].execute(context)
            self.assertEqual(factory.call_args.kwargs["parameters"][1],
                             {"enable_depth": False, "serial_no": "12345"})
            self.assertIs(factory.call_args.kwargs["respawn"], True)

    def run_fake_camera(self, respawn):
        """실제 launch_ros Node가 -11 종료 두 번 뒤 세 번째 프로세스를 띄우는지 확인함."""
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "processes.jsonl"
            child = (
                "import json, os, pathlib, resource, signal, time\n"
                "resource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n"
                f"p = pathlib.Path({str(marker)!r})\n"
                "n = len(p.read_text().splitlines()) if p.exists() else 0\n"
                "with p.open('a') as f: f.write(json.dumps({'pid':os.getpid(), 'n':n})+'\\n')\n"
                "if n < 2: os.kill(os.getpid(), signal.SIGSEGV)\n"
                "time.sleep(30)\n"
            )
            starts, exits = [], []

            def fake_node(**kwargs):
                return Node(executable=sys.executable, arguments=["-c", child],
                            respawn=kwargs["respawn"], respawn_delay=kwargs["respawn_delay"],
                            output="screen", sigterm_timeout="1", sigkill_timeout="1")

            def started(event, _context):
                starts.append((event.pid, time.monotonic()))
                if len(starts) == 3:
                    return [TimerAction(period=0.2, actions=[
                        EmitEvent(event=Shutdown(reason="가짜 카메라 재시작 확인 완료"))])]

            def exited(event, _context):
                exits.append((event.returncode, time.monotonic()))
                if not respawn:
                    return [TimerAction(period=3.2, actions=[
                        EmitEvent(event=Shutdown(reason="자동 재시작 끄기 확인 완료"))])]

            # subprocess는 ROS 초기화를 하지 않지만, 실행 환경도 실물과 분리함.
            with patch.object(wrist, "Node", side_effect=fake_node):
                launch = LaunchService()
                launch.include_launch_description(LaunchDescription([
                    SetLaunchConfiguration("wrist_respawn", str(respawn).lower()),
                    RegisterEventHandler(OnProcessStart(on_start=started)),
                    RegisterEventHandler(OnProcessExit(on_exit=exited)),
                    TimerAction(period=12.0, actions=[EmitEvent(event=Shutdown(reason="시험 시간 초과"))]),
                    wrist.generate_launch_description(),
                ]))
                launch.run()
            records = [json.loads(line) for line in marker.read_text().splitlines()]
            if respawn:
                self.assertEqual(len(starts), 3)
                self.assertEqual([code for code, _ in exits[:2]], [-11, -11])
                self.assertEqual(len(set(pid for pid, _ in starts)), 3)
                for index in range(2):
                    self.assertGreaterEqual(starts[index + 1][1] - exits[index][1], 2.95)
            else:
                self.assertEqual(len(starts), 1)
                self.assertEqual(exits[0][0], -11)
            self.assertEqual(len(records), len(starts))
            print("가짜 D435 결과:", json.dumps({"respawn": respawn, "starts": starts, "exits": exits}))

    def test_process_death_restarts_twice(self):
        self.run_fake_camera(True)

    def test_respawn_can_be_disabled(self):
        self.run_fake_camera(False)


if __name__ == "__main__":
    unittest.main()
