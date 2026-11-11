"""(시뮬레이션 전용) 맵핑 주행용 가상 텔레옵.

실차에서는 사람이 조종기로 맵핑 주행을 하지만, 시뮬레이션에서는 이 노드가 대신 운전한다.
Gazebo의 /odom(odometry_source=world라 사실상 정답 자세)을 사용해 mapping_plan.yaml의 웨이포인트를 따라가고,
구간 시작/끝마다 /route_recorder/segment 를 발행해 경로 기록기가 'SLAM 추정 자세'로 경로를 기록하게 한다.
→ 주행 스택이 쓰는 맵·경로는 모두 카메라/odom 기반 SLAM 결과이며 정답 좌표는 운전에만 쓰인다.
"""
import json
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from autorace_common.geometry import wrap, yaw_from_quat
from autorace_common.paths import Path

from .course_texture import CourseTexture

TEXTURE = ('/home/yoon/gazebo_simenv/autorace_sim_share_20261109/src/autorace_gazebo/models/'
           'autorace_panel_surface_flat/materials/textures/course_surface.png')


class MappingDriver(Node):

    def __init__(self):
        super().__init__('mapping_driver')
        P = self.declare_parameter
        plan_file = P('plan_file', os.path.join(get_package_share_directory('autorace_sim_tools'),
                                                'config', 'mapping_plan.yaml')).value
        self.session = P('session', 'full_left').value
        self.texture = P('texture', TEXTURE).value
        self.lookahead = P('lookahead', 0.15).value
        self.start_delay = P('start_delay', 3.0).value
        with open(plan_file) as f:
            self.steps = yaml.safe_load(f)['sessions'][self.session]
        self.tex = CourseTexture(self.texture)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.seg_pub = self.create_publisher(String, '/route_recorder/segment', 10)
        self.save_pub = self.create_publisher(String, '/route_recorder/save', 10)
        self.done_pub = self.create_publisher(String, '/mapping_driver/done', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pose = None
        self.k = -1
        self.t0 = None
        self.finished = False
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'맵핑 주행 세션 {self.session}: {len(self.steps)} 단계')

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def next_step(self):
        if self.k >= 0:
            st = self.steps[self.k]
            if 'drive' in st and st['drive'].get('record'):
                self.seg_pub.publish(String(data=json.dumps({'name': ''})))
        self.k += 1
        self.publish(0.0, 0.0)
        if self.k >= len(self.steps):
            self.finished = True
            self.save_pub.publish(String(data='save'))
            self.done_pub.publish(String(data='done'))
            self.get_logger().info('맵핑 주행 완료')
            return
        st = self.steps[self.k]
        self.t_step = self.now()
        if 'drive' in st:
            d = st['drive']
            pts = np.array(d['points'], float)
            if d.get('center'):
                pts, _ = self.tex.center(pts)
            self.path = Path(pts, step=0.01)
            self.s = 0.0
            self.reverse = bool(d.get('reverse', False))
            self.speed = float(d.get('record', {}).get('speed', 0.15))
            self.speed = min(self.speed, 0.15)
            if d.get('record'):
                self.seg_pub.publish(String(data=json.dumps(d['record'])))
        self.get_logger().info(f'단계 {self.k}: {list(st.keys())[0]} '
                               f"{st.get('drive', {}).get('record', {}).get('name', '')}")

    def tick(self):
        if self.finished or self.pose is None:
            return
        if self.t0 is None:
            self.t0 = self.now()
        if self.now() - self.t0 < self.start_delay:
            return
        if self.k < 0:
            self.next_step()
            return
        st = self.steps[self.k]
        if 'wait' in st:
            if self.now() - self.t_step >= float(st['wait']):
                self.next_step()
            return
        if 'rotate' in st:
            err = wrap(float(st['rotate']) - self.pose[2])
            if abs(err) < 0.03:
                self.next_step()
            else:
                w = float(np.clip(2.0 * err, -1.2, 1.2))
                self.publish(0.0, math.copysign(max(abs(w), 0.3), w))
            return
        self.drive()

    def drive(self):
        x, y, th = self.pose
        i, s, _ = self.path.project((x, y), hint=self.s, window=0.3)
        self.s = max(self.s, s)
        rem = self.path.length - self.s
        ex, ey, _ = self.path.end_pose()
        if rem < 0.02 or (rem < 0.1 and math.hypot(ex - x, ey - y) < 0.02):
            self.next_step()
            return
        tx, ty, _ = self.path.at(self.s + min(self.lookahead, max(rem, 0.05)))
        vyaw = th + (math.pi if self.reverse else 0.0)
        dx, dy = tx - x, ty - y
        lx = math.cos(vyaw) * dx + math.sin(vyaw) * dy
        ly = -math.sin(vyaw) * dx + math.cos(vyaw) * dy
        alpha = math.atan2(ly, lx)
        if abs(alpha) > 0.8:
            self.publish(0.0, float(np.clip(2.0 * alpha, -1.2, 1.2)))
            return
        k = 2 * ly / max(lx * lx + ly * ly, 1e-4)
        v = self.speed / (1 + 0.1 * abs(k))
        v = min(v, 0.5 * rem + 0.04)
        self.publish(-v if self.reverse else v, float(np.clip(v * k, -2.0, 2.0)))

    def publish(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = MappingDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, getattr(rclpy.executors, 'ExternalShutdownException', KeyboardInterrupt)):
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
