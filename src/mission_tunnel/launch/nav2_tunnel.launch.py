"""터널용 Nav2 스택(맵서버·플래너·컨트롤러·복구서버·BT 내비게이터). AMCL 없이 lane_slam의 map TF 사용.

ROS 배포판(ros_distro 인자, 기본 $ROS_DISTRO)에 따라 파라미터 파일과 복구 서버를 고른다.
  galactic : config/nav2_tunnel_galactic.yaml, nav2_recoveries/recoveries_server   (Jetson 실차)
  humble   : config/nav2_tunnel_humble.yaml,   nav2_behaviors/behavior_server      (개발 PC 시뮬레이션)
cmd_vel은 /cmd_vel/nav2로 내보내고 메인의 cmd_mux가 터널 구간에서만 실제 /cmd_vel로 전달한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 배포판별: (파라미터 파일, 복구 서버 패키지, 실행 파일, 노드 이름)
DISTRO = {
    'galactic': ('nav2_tunnel_galactic.yaml', 'nav2_recoveries', 'recoveries_server', 'recoveries_server'),
    'humble': ('nav2_tunnel_humble.yaml', 'nav2_behaviors', 'behavior_server', 'behavior_server'),
}


def _nodes(context):
    distro = LaunchConfiguration('ros_distro').perform(context).strip().lower()
    if distro not in DISTRO:
        distro = 'galactic' if distro < 'humble' else 'humble'   # foxy 등 이전 → galactic 형식
    yaml_name, rec_pkg, rec_exe, rec_name = DISTRO[distro]
    params = os.path.join(get_package_share_directory('mission_tunnel'), 'config', yaml_name)
    sim = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': sim}]
    remap = [('cmd_vel', '/cmd_vel/nav2')]
    names = ['map_server', 'planner_server', 'controller_server', rec_name, 'bt_navigator']
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=common + [{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=common, remappings=remap),
        Node(package=rec_pkg, executable=rec_exe, name=rec_name, output='screen',
             parameters=common, remappings=remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_tunnel',
             output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': names}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='터널 LiDAR 맵 yaml (map 프레임 정렬본)'),
        DeclareLaunchArgument('ros_distro', default_value=os.environ.get('ROS_DISTRO', 'galactic'),
                              description='galactic | humble'),
        OpaqueFunction(function=_nodes),
    ])
