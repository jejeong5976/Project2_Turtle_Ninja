"""지역 계획기 정책 시험용 Nav2 일부: 지도 · AMCL · planner_server · collision_monitor · cmd_vel_hold · scan_merger (+ 로봇 URDF RSP).
scan_merger = 레포 navigation 패키지 (설정 share/navigation/config/navigation.yaml, 실물과 같은 몸 상자) → /scan → 정책 노드.
bt_navigator · controller_server 는 안 띄움 → navigate_to_pose 는 local_planner_node.py (같은 폴더) 가 받음.
예) ros2 launch rl/tasks/local_planner/deploy/rl_nav.launch.py params:=<레포>/config/nav2.yaml map:=<지도.yaml> urdf:=<팀 robot.urdf> initial_pose:=-1.468,-2.202,180
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def make(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    files = [arg('params')] + [f for f in arg('extra_params').split(',') if f]          # extra_params = 덮어쓰기 yaml (쉼표)
    common = [ParameterFile(f, allow_substs=True) for f in files] + [{'use_sim_time': True}]
    x, y, yaw = (float(v) for v in arg('initial_pose').split(','))
    urdf = open(arg('urdf')).read().replace('filename="/', 'filename="file:///')
    lc = {'use_sim_time': True, 'autostart': True}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=common + [{'yaml_filename': arg('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=common + [{'set_initial_pose': True, 'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw * 3.141592653589793 / 180}}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=common),
        Node(package='navigation', executable='scan_merger', name='scan_merger', output='screen',
             parameters=[ParameterFile(get_package_share_directory('navigation') + '/config/navigation.yaml', allow_substs=True),
                         {'use_sim_time': True}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen', parameters=common),
        Node(package='navigation', executable='cmd_vel_hold', name='cmd_vel_hold', output='screen',
             parameters=[{'use_sim_time': True, 'cmd_vel_in': 'cmd_vel_safe', 'cmd_vel_out': '/cmd_vel', 'start_hold': False, 'hold_timeout_s': 0.0}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
             parameters=[{**lc, 'node_names': ['map_server', 'amcl']}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[{**lc, 'node_names': ['planner_server', 'collision_monitor']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params'), DeclareLaunchArgument('map'), DeclareLaunchArgument('urdf'),
        DeclareLaunchArgument('initial_pose', default_value='-1.468,-2.202,180'),
        DeclareLaunchArgument('extra_params', default_value=''),
        OpaqueFunction(function=make),
    ])
