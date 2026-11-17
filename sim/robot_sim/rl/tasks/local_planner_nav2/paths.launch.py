"""경로 만들기용 Nav2 일부: map_server · planner_server (NavFn + Smac2D) · 정적 TF map → base_link.
make_nav2_paths.py 가 map_server/load_map 으로 지도를 바꿔 가며 compute_path_to_pose 를 부름.
설정 = 레포 config/nav2.yaml (배포와 같은 NavFn · 비용지도) → smac2d_planners.yaml (Smac2D 추가) → paths_only.yaml (실제 시계 · 라이다 층 뺌).
예) ros2 launch rl/tasks/local_planner_nav2/paths.launch.py      (보통은 run_make_paths.sh 가 띄움)
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]                                   # 레포 최상위
SIM = HERE.parents[2]                                    # sim/robot_sim


def make(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    params = [ParameterFile(f, allow_substs=True) for f in (arg('params'), str(HERE / 'smac2d_planners.yaml'), str(HERE / 'paths_only.yaml'))]
    return [
        Node(package='tf2_ros', executable='static_transform_publisher', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=params + [{'yaml_filename': arg('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=params),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_paths', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(REPO / 'config' / 'nav2.yaml')),
        DeclareLaunchArgument('map', default_value=str(SIM / 'rl' / 'data' / 'maps' / 'maze_dice_map.yaml')),
        OpaqueFunction(function=make),
    ])
