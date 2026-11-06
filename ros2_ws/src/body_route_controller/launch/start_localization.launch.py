"""Stationary start localization only; no drivers, controller, goals or hold publisher."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare(context):
    if LaunchConfiguration('reviewed_start').perform(context) != 'true':
        raise RuntimeError('Review current start placement and wall selection before enabling startup localization')
    paths = {key: Path(LaunchConfiguration(key).perform(context)).resolve()
             for key in ('config', 'map', 'nav2_params')}
    for path in paths.values():
        if not path.is_file():
            raise RuntimeError('Missing input: ' + str(path))
    amcl = yaml.safe_load(paths['nav2_params'].read_text())['amcl']['ros__parameters']
    if amcl['laser_model_type'] != 'likelihood_field':
        raise RuntimeError('This mask is validated for the likelihood_field model only')
    amcl.update(scan_topic='/localization/start_scan', set_initial_pose=False,
                always_reset_initial_pose=True, use_sim_time=False)
    # Existing AMCL settings remain intact except its input and initialization.
    # Operator applies the independently reviewed seed with 0.10 m / 5 deg uncertainty.
    return [
        Node(package='body_route_controller', executable='start_scan_filter',
             name='start_scan_filter', output='screen',
             parameters=[{'config': str(paths['config']), 'reviewed_start': True}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': str(paths['map']), 'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[amcl]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_start_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'], 'use_sim_time': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reviewed_start', default_value='false'),
        DeclareLaunchArgument('config'), DeclareLaunchArgument('map'),
        DeclareLaunchArgument('nav2_params'), OpaqueFunction(function=prepare),
    ])
