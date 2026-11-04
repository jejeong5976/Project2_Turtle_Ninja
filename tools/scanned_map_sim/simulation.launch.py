"""Scanned maze controller simulation; hardware selection is fixed to mock."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ['SCANNED_SIM_RUN'])
    here = Path(__file__).resolve().parent
    scenario = yaml.safe_load((root / 'maps/scenario.yaml').read_text())
    initial = scenario['start']
    initial_pose = ','.join(str(v) for v in [initial[0], initial[1], initial[2] * 180 / 3.141592653589793])
    deadline = float(os.environ.get('SCANNED_SIM_DEADLINE', '600'))
    share = get_package_share_directory
    feed = ExecuteProcess(cmd=[str(Path(os.environ['SCANNED_SIM_HELPERS']) / 'feed')], output='screen')
    trial = ExecuteProcess(cmd=[get_package_prefix('body_route_controller') + '/lib/body_route_controller/route_trial',
        '--ros-args', '--params-file', str(root / 'profile/trial.yaml'),
        '-p', 'goals_file:=' + str(root / 'maps/goals.path'), '-p', 'execute:=true',
        '-p', 'max_goal_duration:=' + str(deadline), '-p', 'results_file:=' + str(root / 'result.yaml')], output='screen')
    startup = ExecuteProcess(cmd=[str(Path(os.environ['SCANNED_SIM_HELPERS']) / 'startup')], output='screen')

    def startup_finished(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='Startup verified; starting the supervised route trial.'), trial]
        return [LogInfo(msg='Startup failed; trial was not started. See startup.yaml and launch.log.'),
                EmitEvent(event=Shutdown(reason='simulation startup failed'))]

    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('bringup') + '/launch/robot.launch.py'), launch_arguments={
            'hardware': 'mock', 'mock_dynamics': 'true', 'arm': 'false', 'use_rviz': 'false',
            'hardware_map': str(here / 'hardware_map_mock.yaml'),
            'drive_test_params': str(root / 'profile/drive_limits.yaml'),
            'park_on_idle': 'false', 'blend_steer_drive': 'false',
        }.items()),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(share('body_route_controller') + '/launch/body_route.launch.py'), launch_arguments={
            'params': str(root / 'profile/nav2.yaml'), 'map': str(root / 'maps/navigation.yaml'),
            'use_sim_time': 'false', 'initial_pose': initial_pose, 'rviz': 'false', 'autostart': 'false',
        }.items())]),
        RegisterEventHandler(OnProcessExit(target_action=trial, on_exit=[EmitEvent(event=Shutdown(reason='scanned-map trial finished'))])),
        RegisterEventHandler(OnProcessExit(target_action=feed, on_exit=[EmitEvent(event=Shutdown(reason='simulation feed stopped'))])),
        RegisterEventHandler(OnProcessExit(target_action=startup, on_exit=startup_finished)),
        feed, TimerAction(period=6.0, actions=[startup]),
        TimerAction(period=90.0 + deadline * len(scenario['goals']), actions=[EmitEvent(event=Shutdown(reason='simulation deadline'))]),
    ]
    if os.environ.get('SCANNED_SIM_RVIZ') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(here / 'simulation.rviz')], output='log'))
    return LaunchDescription(actions)
