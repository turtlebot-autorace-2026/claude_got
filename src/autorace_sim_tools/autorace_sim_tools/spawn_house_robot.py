"""(시뮬레이션 전용) 주차칸에 '하우스 로봇'(TurtleBot3 Burger 크기의 정적 물체)을 배치/제거.

  ros2 run autorace_sim_tools spawn_house_robot left     # 왼쪽 칸(진행방향 기준)에 배치
  ros2 run autorace_sim_tools spawn_house_robot right
  ros2 run autorace_sim_tools spawn_house_robot remove
월드 좌표는 맵 텍스처 기준 주차칸 중앙.
"""
import sys

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose

SLOTS = {'left': (0.84, 0.86), 'right': (0.28, 0.86)}   # 주차 진입 방향(-y) 기준 왼쪽=+x 칸
SDF = """<?xml version='1.0'?>
<sdf version='1.6'><model name='house_robot'><static>true</static>
<link name='body'>
  <collision name='c'><pose>0 0 0.096 0 0 0</pose><geometry><box><size>0.14 0.178 0.192</size></box></geometry></collision>
  <visual name='v'><pose>0 0 0.096 0 0 0</pose><geometry><box><size>0.14 0.178 0.192</size></box></geometry>
    <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
</link></model></sdf>"""


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else 'left'
    rclpy.init()
    node = rclpy.create_node('spawn_house_robot')
    dc = node.create_client(DeleteEntity, '/delete_entity')
    dc.wait_for_service(timeout_sec=5.0)
    f = dc.call_async(DeleteEntity.Request(name='house_robot'))
    rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
    if arg != 'remove':
        sc = node.create_client(SpawnEntity, '/spawn_entity')
        sc.wait_for_service(timeout_sec=5.0)
        req = SpawnEntity.Request(name='house_robot', xml=SDF)
        req.initial_pose = Pose()
        req.initial_pose.position.x, req.initial_pose.position.y = SLOTS[arg]
        f = sc.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        print('spawned house_robot at', arg, SLOTS[arg], f.result().status_message if f.result() else 'timeout')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
