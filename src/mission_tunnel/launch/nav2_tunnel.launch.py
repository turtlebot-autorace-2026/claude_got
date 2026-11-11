"""터널용 Nav2 스택(맵서버·플래너·컨트롤러·행동서버·BT 내비게이터). AMCL 없이 lane_slam의 map TF 사용.

cmd_vel은 /cmd_vel/nav2로 내보내고 메인의 cmd_mux가 터널 구간에서만 실제 /cmd_vel로 전달한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('mission_tunnel'), 'config', 'nav2_tunnel.yaml')
    sim = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': sim}]
    remap = [('cmd_vel', '/cmd_vel/nav2')]
    names = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', description='터널 LiDAR 맵 yaml (map 프레임 정렬본)'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=common + [{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=common, remappings=remap),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=common, remappings=remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_tunnel',
             output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': names, 'bond_timeout': 0.0}]),
    ])
