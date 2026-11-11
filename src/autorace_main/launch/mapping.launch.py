"""맵핑 단계 런치: 차선 SLAM(맵핑) + 경로 기록기 + (선택) LiDAR SLAM + (sim) 가상 텔레옵.

예) 시뮬레이션 세션 A(전체 코스, LiDAR 맵 포함):
    ros2 launch autorace_main mapping.launch.py session:=full_left lidar:=true
    세션 B(오른쪽 갈림길만, 기존 차선맵 확장):
    ros2 launch autorace_main mapping.launch.py session:=right_branch extend_map:=true
실차: sim_driver:=false use_sim_time:=false 로 실행하고 텔레옵으로 주행하며
    ros2 topic pub --once /route_recorder/segment std_msgs/String "data: '{\"name\": \"start_to_fork\", \"speed\": 0.2}'"
처럼 구간을 표시한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from autorace_main.paths import data_dir


def generate_launch_description():
    d = data_dir()
    sim = LaunchConfiguration('use_sim_time')
    tunnel_share = get_package_share_directory('mission_tunnel')
    lane_cfg = os.path.join(get_package_share_directory('lane_slam'), 'config', 'lane_slam.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lane_detector', default_value='yolo', description='yolo | hsv'),
        DeclareLaunchArgument('lane_python', default_value=os.path.expanduser('~/autorace/yolo_venv/bin/python'),
                              description='YOLO(onnxruntime-gpu/torch)가 설치된 파이썬. hsv면 무시'),
        DeclareLaunchArgument('session', default_value='full_left'),
        DeclareLaunchArgument('extend_map', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='false', description='slam_toolbox로 LiDAR 맵도 생성'),
        DeclareLaunchArgument('sim_driver', default_value='true', description='sim 가상 텔레옵 사용'),
        DeclareLaunchArgument('map_prefix', default_value=os.path.join(d, 'maps', 'lane_map')),
        DeclareLaunchArgument('routes_file', default_value=os.path.join(d, 'routes', 'routes.yaml')),
        DeclareLaunchArgument('tunnel_map', default_value=os.path.join(d, 'maps', 'tunnel_map')),
        Node(package='lane_slam', executable='lane_slam_node', name='lane_slam', output='screen',
             prefix=PythonExpression(["'", LaunchConfiguration('lane_python'), "' if '",
                                      LaunchConfiguration('lane_detector'), "' == 'yolo' else ''"]),
             parameters=[lane_cfg, {'use_sim_time': sim, 'mode': 'mapping', 'publish_tf': False,
                                    'lane_detector': LaunchConfiguration('lane_detector'),
                                    'map_prefix': LaunchConfiguration('map_prefix'),
                                    'extend_map': LaunchConfiguration('extend_map')}]),
        Node(package='route_follower', executable='route_recorder', name='route_recorder', output='screen',
             parameters=[{'use_sim_time': sim, 'routes_file': LaunchConfiguration('routes_file')}]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(tunnel_share, 'config', 'slam_toolbox_mapping.yaml'), {'use_sim_time': sim}],
             condition=IfCondition(LaunchConfiguration('lidar'))),
        Node(package='mission_tunnel', executable='tunnel_map_align', name='tunnel_map_align', output='screen',
             parameters=[{'use_sim_time': sim, 'out_prefix': LaunchConfiguration('tunnel_map')}],
             condition=IfCondition(LaunchConfiguration('lidar'))),
        Node(package='autorace_sim_tools', executable='mapping_driver', name='mapping_driver', output='screen',
             parameters=[{'use_sim_time': sim, 'session': LaunchConfiguration('session')}],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('sim_driver'), "' == 'true'"]))),
    ])
