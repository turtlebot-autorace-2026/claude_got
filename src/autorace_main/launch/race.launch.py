"""본 주행 런치: 차선맵 위치추정 + 경로추종 + 미션 노드 6종 + 터널 Nav2 + 미션 매니저.

    ros2 launch autorace_main race.launch.py            # 시뮬레이션
    ros2 launch autorace_main race.launch.py use_sim_time:=false   # 실차
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from autorace_main.paths import data_dir


def generate_launch_description():
    d = data_dir()
    sim = LaunchConfiguration('use_sim_time')
    routes = LaunchConfiguration('routes_file')
    share = get_package_share_directory
    lane_cfg = os.path.join(share('lane_slam'), 'config', 'lane_slam.yaml')
    follow_cfg = os.path.join(share('route_follower'), 'config', 'route_follower.yaml')
    tunnel_map = os.path.join(d, 'maps', 'tunnel_map.yaml')

    def node(pkg, exe, name, params=None, **kw):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=(params or []) + [{'use_sim_time': sim}], **kw)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lane_detector', default_value='yolo', description='yolo | hsv'),
        DeclareLaunchArgument('lane_python', default_value=os.path.expanduser('~/autorace/yolo_venv/bin/python'),
                              description='YOLO(onnxruntime-gpu/torch)가 설치된 파이썬. hsv면 무시'),
        DeclareLaunchArgument('map_prefix', default_value=os.path.join(d, 'maps', 'lane_map')),
        DeclareLaunchArgument('routes_file', default_value=os.path.join(d, 'routes', 'routes.yaml')),
        DeclareLaunchArgument('lidar_map', default_value=tunnel_map),
        DeclareLaunchArgument('mission_config', default_value=os.path.join(share('autorace_main'), 'config', 'mission.yaml')),
        DeclareLaunchArgument('start_mission', default_value='traffic_light'),
        DeclareLaunchArgument('debug_images', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0]',
                              description='map 기준 시작 자세(출발점이면 0). 중간 미션 시험용'),
        Node(package='lane_slam', executable='lane_slam_node', name='lane_slam', output='screen',
             prefix=PythonExpression(["'", LaunchConfiguration('lane_python'), "' if '",
                                      LaunchConfiguration('lane_detector'), "' == 'yolo' else ''"]),
             parameters=[lane_cfg, {'use_sim_time': sim, 'mode': 'localization',
                                    'lane_detector': LaunchConfiguration('lane_detector'),
                                    'map_prefix': LaunchConfiguration('map_prefix'),
                                    'lidar_map_yaml': LaunchConfiguration('lidar_map'),
                                    'initial_pose': LaunchConfiguration('initial_pose')}]),
        node('route_follower', 'route_follower_node', 'route_follower', [follow_cfg, {'routes_file': routes}]),
        node('autorace_main', 'cmd_mux', 'cmd_mux'),
        node('mission_traffic_light', 'traffic_light_node', 'traffic_light',
             [{'publish_debug': LaunchConfiguration('debug_images')}]),
        node('mission_intersection', 'intersection_node', 'intersection',
             [{'publish_debug': LaunchConfiguration('debug_images'), 'start_enabled': False}]),
        node('mission_construction', 'avoidance_node', 'construction_avoidance'),
        node('mission_parking', 'parking_node', 'parking', [{'routes_file': routes}]),
        node('mission_level_crossing', 'level_crossing_node', 'level_crossing',
             [{'publish_debug': LaunchConfiguration('debug_images')}]),
        node('mission_tunnel', 'tunnel_nav_node', 'tunnel_nav', [{'routes_file': routes}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('mission_tunnel'), 'launch', 'nav2_tunnel.launch.py')),
            launch_arguments={'use_sim_time': sim, 'map': LaunchConfiguration('lidar_map')}.items(),
            condition=IfCondition(LaunchConfiguration('nav2'))),
        node('autorace_main', 'mission_manager', 'mission_manager',
             [{'routes_file': routes, 'mission_config': LaunchConfiguration('mission_config'),
               'start_mission': LaunchConfiguration('start_mission')}]),
    ])
