"""L2 A/B/C held session with full-map registration and front-lidar AMCL. Starts no trial client and sends no goal."""
from pathlib import Path
import re
import shutil
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIVE_VIEW = Path('/home/turtle/nt_three_mode_20261106_observe_timing_v2/diagnostic-build/live_view')


def start(context):
    ROOT = Path(LaunchConfiguration('packet_root').perform(context)).expanduser().resolve()
    body_prefix = Path(LaunchConfiguration("body_route_prefix").perform(context)).expanduser().resolve()
    overlays = {"body_route_controller": str(body_prefix), "coin_d4_driver": "/home/turtle/nt_lidar_metadata_20261106_v1/install"}
    for package, expected in overlays.items():
        if Path(get_package_prefix(package)) != Path(expected):
            raise RuntimeError("Select the verified corrected timing overlay for " + package)
    for executable in ("register_stationary", "full_map_startup", "front_localization_params"):
        if not (body_prefix / "lib/body_route_controller" / executable).is_file():
            raise RuntimeError("Require the reviewed full-map restoration build: " + executable)
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('axis-stop', 'continuous-reactive', 'continuous-preview'):
        raise RuntimeError('Select one prepared A/B/C profile')
    session = LaunchConfiguration('session').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', session):
        raise RuntimeError('session must be a new simple directory name')
    profile = ROOT / 'profiles' / mode
    trial = yaml.safe_load((profile / 'trial.yaml').read_text())['body_route_trial']['ros__parameters']
    drive = yaml.safe_load((profile / 'drive_limits.yaml').read_text())
    navigation = yaml.safe_load((profile / 'nav2.yaml').read_text())
    controller = navigation['controller_server']['ros__parameters']
    nav = controller['FollowPath']
    amcl = navigation['amcl']['ros__parameters']
    if (amcl['scan_topic'] != '/lidar_front/scan' or amcl['laser_model_type'] != 'likelihood_field'
            or amcl['set_initial_pose'] or not amcl['always_reset_initial_pose']):
        raise RuntimeError('Full-map reviewed initialization and front-lidar AMCL are required')
    continuous = mode != 'axis-stop'
    expected_policy = 'preview' if mode == 'continuous-preview' else 'reactive'
    if trial['execute'] or not trial['require_live_view']:
        raise RuntimeError('Physical trial must default to preview and require the visible live viewer')
    if trial['controller_mode'] != 'custom' or trial['unknown_policy'] != 'observe-before-enter':
        raise RuntimeError('Unexpected physical comparison profile')
    if (nav['plugin'] != 'body_route_controller::ObservationController' or
            nav['delegate_plugin'] != 'body_route_controller::BodyRouteController' or
            controller['general_goal_checker']['plugin'] != 'body_route_controller::ObservedGoalChecker' or
            navigation['planner_server']['ros__parameters']['GridBased']['unknown_policy'] != 'observe-before-enter'):
        raise RuntimeError('Observe-before-enter must cover planning, execution and goal completion')
    for name in ('local_costmap', 'global_costmap'):
        cost = navigation[name][name]['ros__parameters']
        observation = cost.get('observation_layer', {})
        if (cost['plugins'] != ['static_layer', 'obstacle_layer', 'observation_layer', 'inflation_layer'] or
                observation.get('plugin') != 'body_route_controller::ObservationLayer' or
                observation.get('unknown_policy') != 'observe-before-enter'):
            raise RuntimeError('Both costmaps require the matching observation layer')
        for key, expected in [('ttl', 4.0), ('scan_timeout', .3), ('latency', .5), ('linear_decel', .05), ('angular_decel', .2)]:
            if abs(observation[key] - expected) > 1e-9:
                raise RuntimeError('Unexpected observation stopping or freshness settings')
    if trial['drive_style'] != ('continuous' if continuous else 'axis-stop') or trial['heading_policy'] != expected_policy:
        raise RuntimeError('Profile directory and selected mode disagree')
    if drive['blend_steer_drive'] != continuous or drive['heading_hold_kp'] != 0.0:
        raise RuntimeError('Wheel blend/heading settings do not match the mode')
    for key, expected in [('max_linear', .025), ('max_angular', .10), ('max_linear_accel', .05), ('max_angular_accel', .2)]:
        if abs(drive[key] - expected) > 1e-9:
            raise RuntimeError('This first qualification requires the prepared slow limits')
    if nav['drive_style'] != trial['drive_style'] or nav['heading_policy'] != expected_policy:
        raise RuntimeError('Navigation and trial profiles disagree')
    if continuous and (trial['max_travel_angle_deg'] != 45 or abs(nav['max_travel_angle'] - 0.7853981633974483) > 1e-9):
        raise RuntimeError('Continuous qualification requires the 45 degree limit')
    ports = {}
    expected_ids = {'wheel': ('0483', '5740'), 'front': ('10c4', 'ea60'), 'rear': ('10c4', 'ea60')}
    for name in expected_ids:
        port = Path(LaunchConfiguration(name + '_port').perform(context))
        if port.parent != Path('/dev/serial/by-path') or not port.exists():
            raise RuntimeError('A currently verified by-path device is required: ' + name)
        metadata = subprocess.check_output(['udevadm', 'info', '--query=property', '--name=' + str(port)], text=True)
        properties = dict(line.split('=', 1) for line in metadata.splitlines() if '=' in line)
        if (properties.get('ID_VENDOR_ID'), properties.get('ID_MODEL_ID')) != expected_ids[name]:
            raise RuntimeError('Unexpected device type: ' + name)
        ports[name] = str(port)
    if len({str(Path(port).resolve()) for port in ports.values()}) != 3:
        raise RuntimeError('Wheel and two lidar ports must resolve to different devices')
    cfg = ROOT / 'runtime'
    hw = yaml.safe_load((cfg / 'hardware_map_drive_slow.yaml').read_text())
    if len(hw['buses']) != 1 or hw['buses'][0]['name'] != 'bus_drive' or len(hw['motors']) != 8:
        raise RuntimeError('Only the reviewed wheel-only hardware map is permitted')
    for name in ('steer_fl_joint', 'steer_fr_joint', 'steer_rl_joint', 'steer_rr_joint'):
        if hw['motors'][name]['profile_vel'] != 20 or hw['motors'][name]['profile_acc'] != 5:
            raise RuntimeError('Preserve the recorded slow steering settings')
    map_file = ROOT / 'map' / 'map.yaml'
    map_meta = yaml.safe_load(map_file.read_text())
    if map_meta['free_thresh'] != .196 or map_meta['occupied_thresh'] != .65:
        raise RuntimeError('Gray cells must remain unknown for observe-before-enter')
    for path in (LIVE_VIEW, cfg / 'laser_body_filter.yaml', map_file.parent / map_meta['image']):
        if not path.is_file():
            raise RuntimeError('Required runtime file missing: ' + str(path))
    run = ROOT / 'runs' / session
    run.mkdir(parents=True, exist_ok=False)
    assets = run / 'viewer'
    shutil.copytree(ROOT / 'viewer-assets', assets)

    def include(package, filename, arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())

    actions = [include('bringup', 'robot.launch.py', {
        'hardware': 'opencr', 'arm': 'false', 'use_rviz': 'false',
        'hardware_map': str(cfg / 'hardware_map_drive_slow.yaml'),
        'opencr_ports': 'bus_drive=' + ports['wheel'],
        'drive_test_params': str(profile / 'drive_limits.yaml'),
        'park_on_idle': 'false', 'steering_policy': 'coherent', 'enforce_command_limits': 'true',
    })]
    for name in ('front', 'rear'):
        actions.extend([
            Node(package='coin_d4_driver', executable='single_coin_d4_node', name='lidar_' + name, output='screen', parameters=[{
                'port': ports[name], 'frame_id': 'lidar_' + name + '_link', 'topic_name': '/lidar_' + name + '/scan_raw',
                'baudrate': 230400, 'version': 4, 'reverse': False, 'warmup_time': 5, 'use_sim_time': False,
            }]),
            Node(package='laser_filters', executable='scan_to_scan_filter_chain', name='body_filter_' + name, output='screen',
                 parameters=[str(cfg / 'laser_body_filter.yaml'), {'use_sim_time': False}],
                 remappings=[('scan', '/lidar_' + name + '/scan_raw'), ('scan_filtered', '/lidar_' + name + '/scan')]),
        ])
    actions.append(include('body_route_controller', 'body_route.launch.py', {
        'params': str(profile / 'nav2.yaml'), 'map': str(map_file), 'use_sim_time': 'false',
        'initial_pose': '', 'rviz': 'false',
    }))
    topics = ['/parameter_events', '/map', '/odom', '/amcl_pose', '/particle_cloud', '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_safe',
              '/joint_states', '/dynamic_joint_states', '/imu_sensor_broadcaster/imu', '/mission/hold',
              '/lidar_front/scan_raw', '/lidar_rear/scan_raw', '/lidar_front/scan', '/lidar_rear/scan',
              '/collision_monitor_state', '/controller_server/body_route_status', '/controller_server/observation_status', '/plan',
              '/local_costmap/local_costmap/observation_space', '/local_costmap/local_costmap/observed_free',
              '/global_costmap/global_costmap/observation_space', '/global_costmap/global_costmap/observed_free',
              '/local_costmap/costmap', '/local_costmap/costmap_raw', '/local_costmap/costmap_updates',
              '/global_costmap/costmap', '/global_costmap/costmap_raw', '/global_costmap/costmap_updates',
              '/local_costmap/published_footprint', '/global_costmap/published_footprint',
              '/trial/status', '/live_view/fresh', '/live_view/stop', '/live_view/diagnostics', '/swerve_controller/commands']
    actions.extend([
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(run / 'bag'), '--topics'] + topics, output='screen'),
        ExecuteProcess(cmd=[str(LIVE_VIEW), str(assets)], output='screen'),
        TimerAction(period=900., actions=[EmitEvent(event=Shutdown(reason='15-minute held session limit'))]),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('packet_root', description='New reviewed trial packet; historical packets are never edited'),
        DeclareLaunchArgument('body_route_prefix', description='Reviewed build containing restored localization and existing timing guards'),
        DeclareLaunchArgument('mode'), DeclareLaunchArgument('session'),
        DeclareLaunchArgument('wheel_port'), DeclareLaunchArgument('front_port'), DeclareLaunchArgument('rear_port'),
        OpaqueFunction(function=start),
    ])
