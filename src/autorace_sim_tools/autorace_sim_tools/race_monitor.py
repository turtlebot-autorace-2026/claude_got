"""(시뮬레이션 전용) 주행 평가 모니터.

- Gazebo /odom(정답 자세)과 /lane_slam/pose를 비교해 위치추정 오차 기록
- 심판(race_manager) 이벤트(/autorace/race/events)와 미션 상태(/mission/state)를 함께 출력
- 로봇이 30초 이상 멈춰 있으면 경고(경기 규정의 동작불능 조건)
map 프레임 = 출발 자세 기준이므로 정답도 출발 자세(start_pose) 기준으로 변환해 비교한다.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from autorace_common.geometry import relative, wrap, yaw_from_quat


class RaceMonitor(Node):

    def __init__(self):
        super().__init__('race_monitor')
        self.start = tuple(self.declare_parameter('start_pose', [0.687, -1.999, 0.0]).value)
        # 공사구간 장애물(월드 좌표 cx, cy, sx, sy) — 로봇 중심과 상자 사이 거리가 robot_radius 미만이면 접촉
        flat = self.declare_parameter('obstacles', [1.707, 0.619, 0.25, 0.1, 1.993, 1.145, 0.25, 0.1,
                                                    1.707, 1.677, 0.25, 0.1]).value
        self.obstacles = [flat[i:i + 4] for i in range(0, len(flat), 4)]
        self.robot_radius = self.declare_parameter('robot_radius', 0.09).value
        self.contacts = 0
        self.in_contact = False
        self.gt = None
        self.errs = []
        self.mission = {}
        self.last_move = None
        self.last_pos = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/lane_slam/pose', self.on_pose, 10)
        self.create_subscription(String, '/autorace/race/events', self.on_event, 50)
        self.create_subscription(String, '/mission/state', self.on_mission, 10)
        self.create_timer(5.0, self.report)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p = m.pose.pose
        self.gt = relative(self.start, (p.position.x, p.position.y, yaw_from_quat(p.orientation)))
        w = (p.position.x, p.position.y)
        dmin = min(math.hypot(max(abs(w[0] - cx) - sx / 2, 0), max(abs(w[1] - cy) - sy / 2, 0))
                   for cx, cy, sx, sy in self.obstacles)
        if dmin < self.robot_radius and not self.in_contact:
            self.contacts += 1
            self.get_logger().warn(f'공사 장애물 접촉 #{self.contacts} (거리 {dmin:.3f})')
        self.in_contact = dmin < self.robot_radius
        if self.last_pos is None or math.hypot(w[0] - self.last_pos[0], w[1] - self.last_pos[1]) > 0.02:
            self.last_pos, self.last_move = w, self.now()

    def on_pose(self, m):
        if self.gt is None:
            return
        e = math.hypot(m.pose.position.x - self.gt[0], m.pose.position.y - self.gt[1])
        ey = wrap(yaw_from_quat(m.pose.orientation) - self.gt[2])
        self.errs.append((e, abs(ey)))

    def on_event(self, m):
        d = json.loads(m.data)
        if d.get('event') in ('light',):
            return
        self.get_logger().info(f"[심판] {d.get('event')} {({k: v for k, v in d.items() if k not in ('event', 'run', 'sim_t', 'config')})}")

    def on_mission(self, m):
        d = json.loads(m.data)
        if d.get('mission') != self.mission.get('mission'):
            self.get_logger().info(f"[미션] {d.get('mission')}")
        self.mission = d

    def report(self):
        if not self.errs:
            return
        recent = self.errs[-50:]
        e = max(r[0] for r in recent)
        mean = sum(r[0] for r in self.errs) / len(self.errs)
        mx = max(r[0] for r in self.errs)
        idle = self.now() - self.last_move if self.last_move else 0.0
        self.get_logger().info(f"위치오차 최근최대 {e:.3f} m / 평균 {mean:.3f} / 전체최대 {mx:.3f} | "
                               f"미션 {self.mission.get('mission')} {self.mission.get('detail')} | "
                               f"정지 {idle:.0f}s" + (' ⚠30초 정지' if idle > 30 else ''))


def main():
    rclpy.init()
    node = RaceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
