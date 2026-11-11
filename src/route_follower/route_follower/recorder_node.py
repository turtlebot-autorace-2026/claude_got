"""경로 기록기: 맵핑 주행(텔레옵) 중 SLAM 자세(map→base_footprint)를 구간별로 기록.

/route_recorder/segment (String JSON) {"name": "p1", "reverse": false, "speed": 0.2, "max_offset": 0.0}
    → 새 구간 기록 시작. {"name": ""} 이면 기록 일시정지
/route_recorder/save (String, 아무 값) → routes_file에 저장(같은 이름 구간은 덮어씀, 나머지 유지)
종료(Ctrl+C) 시에도 자동 저장.
"""
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from autorace_common.geometry import yaw_from_quat
from autorace_common.paths import resample, save_routes, smooth
from autorace_common.ros_utils import from_json_msg, lookup_pose

import yaml


class RouteRecorder(Node):

    def __init__(self):
        super().__init__('route_recorder')
        P = self.declare_parameter
        self.routes_file = os.path.expanduser(P('routes_file', 'routes.yaml').value)
        self.spacing = P('spacing', 0.02).value
        self.map_frame = P('map_frame', 'map').value
        self.base_frame = P('base_frame', 'base_footprint').value
        self.pose_topic = P('pose_topic', '/lane_slam/pose').value   # 비우면 TF 사용
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_pose = None
        if self.pose_topic:
            self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)
        self.segments = {}
        self.current = None
        self.create_subscription(String, '/route_recorder/segment', self.on_segment, 10)
        self.create_subscription(String, '/route_recorder/save', lambda m: self.save(), 10)
        self.create_timer(0.05, self.tick)

    def on_segment(self, msg):
        c = from_json_msg(msg) or {'name': msg.data}
        name = c.get('name') or ''
        if not name:
            self.current = None
            self.get_logger().info('기록 일시정지')
            return
        meta = {k: v for k, v in c.items() if k != 'name'}
        self.segments[name] = {'points': [], **meta}
        self.current = name
        self.get_logger().info(f'구간 기록 시작: {name} {meta}')

    def on_pose(self, msg):
        self.last_pose = (msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation))

    def tick(self):
        if self.current is None:
            return
        p = self.last_pose if self.pose_topic else lookup_pose(self.tf_buffer, self.map_frame, self.base_frame)
        if p is None:
            return
        pts = self.segments[self.current]['points']
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= self.spacing:
            pts.append((p[0], p[1]))

    def save(self):
        existing = {}
        if os.path.exists(self.routes_file):
            with open(self.routes_file) as f:
                existing = (yaml.safe_load(f) or {}).get('routes', {}) or {}
        for name, seg in self.segments.items():
            if len(seg['points']) < 3:
                continue
            pts = smooth(resample(np.array(seg['points']), self.spacing), window=9, iterations=2)
            existing[name] = {**{k: v for k, v in seg.items() if k != 'points'}, 'points': pts}
        save_routes(self.routes_file, existing)
        self.get_logger().info(f'경로 저장: {self.routes_file} ({len(existing)}개)')


def main():
    rclpy.init()
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
