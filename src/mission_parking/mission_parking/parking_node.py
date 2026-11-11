"""주차 빈칸 판별 노드.

주차칸 영역은 기록 경로 park_<slot>_in 의 끝점·끝 방향으로 정의한다(칸 중앙에 완전히 들어간 자세).
/parking/enable(Bool true)을 받으면 LiDAR 스캔을 일정 횟수 누적해 각 칸 안의 점 개수를 세고
가장 비어 있는 칸을 /parking/slot(String JSON {"slot": "left", "counts": {...}})으로 발행한다.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from autorace_common.geometry import points_in_rect, transform_points
from autorace_common.paths import load_routes
from autorace_common.ros_utils import LATCHED, SENSOR, lookup_pose, to_json_msg


class ParkingNode(Node):

    def __init__(self):
        super().__init__('parking')
        P = self.declare_parameter
        self.routes_file = os.path.expanduser(P('routes_file', '').value)
        self.slots = list(P('slots', ['left', 'right']).value)
        self.prefer = P('prefer', 'left').value
        self.slot_length = P('slot_length', 0.42).value
        self.slot_width = P('slot_width', 0.22).value
        self.slot_shift = P('slot_shift', 0.0).value     # 끝점 기준 칸 중심을 진행방향으로 이동
        self.scans_needed = P('scans', 6).value
        self.min_points = P('occupied_points', 3).value
        self.map_frame = P('map_frame', 'map').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rects = self._slot_rects()
        self.active = False
        self.counts = {}
        self.n = 0
        self.pub = self.create_publisher(String, '/parking/slot', LATCHED)
        self.create_subscription(Bool, '/parking/enable', self.on_enable, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.get_logger().info(f'주차칸: {self.rects}')

    def _slot_rects(self):
        routes = load_routes(self.routes_file) if self.routes_file else {}
        rects = {}
        for s in self.slots:
            r = routes.get(f'park_{s}_in')
            if r is None:
                continue
            x, y, _ = r.end_pose()
            # 끝점 직전 0.25m 구간 방향 = 칸의 길이 방향(스무딩된 끝점 heading보다 안정적)
            px, py, _ = r.at(max(r.length - 0.25, 0.0))
            h = float(np.arctan2(y - py, x - px))
            rects[s] = {'cx': x + np.cos(h) * self.slot_shift, 'cy': y + np.sin(h) * self.slot_shift, 'yaw': h,
                        'length': self.slot_length, 'width': self.slot_width}
        return rects

    def on_enable(self, msg):
        self.active = bool(msg.data)
        self.counts = {s: 0 for s in self.rects}
        self.n = 0

    def on_scan(self, msg):
        if not self.active:
            return
        pose = lookup_pose(self.tf_buffer, self.map_frame, msg.header.frame_id)
        if pose is None:
            return
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < 2.0)
        pts = transform_points(pose, np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))))
        for s, rect in self.rects.items():
            self.counts[s] += int(points_in_rect(pts, rect).sum())
        self.n += 1
        if self.n >= self.scans_needed:
            self.active = False
            free = [s for s in self.rects if self.counts[s] < self.min_points * self.n / 2]
            if self.prefer in free:
                slot = self.prefer
            elif free:
                slot = free[0]
            else:
                slot = min(self.counts, key=self.counts.get)
            self.get_logger().info(f'주차칸 판단: {slot} (점 개수 {self.counts}, 스캔 {self.n})')
            self.pub.publish(to_json_msg({'slot': slot, 'counts': self.counts}))


def main():
    rclpy.init()
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
