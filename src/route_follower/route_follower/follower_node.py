"""기록 경로 추종 노드(Pure Pursuit).

명령(/route_follower/command, std_msgs/String JSON)
  {"cmd": "follow", "route": "p1", "speed": 0.2, "reverse": false, "id": 3,
   "start_s": null, "end_s": null}                    경로 추종(끝에서 정지)
  {"cmd": "rotate", "yaw": 1.57, "id": 4}            map 기준 yaw로 제자리 회전
  {"cmd": "stop"}                                     즉시 정지
  {"cmd": "speed", "speed": 0.1}                      현재 명령의 속도 변경

상태(/route_follower/status, JSON 10Hz): state(idle/following/rotating/done), route, s, remaining, lat, id
보조 입력: /route_follower/offset (std_msgs/Float32, 왼쪽+ [m]) — 장애물 회피 노드가 주는 횡오프셋
출력: cmd_vel(기본 /cmd_vel/route), 활성 경로 /route_follower/path(nav_msgs/Path)
"""
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from autorace_common.geometry import quat_from_yaw, wrap
from autorace_common.paths import load_routes
from autorace_common.ros_utils import LATCHED, from_json_msg, lookup_pose, to_json_msg


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower')
        P = self.declare_parameter
        self.routes_file = os.path.expanduser(P('routes_file', '').value)
        self.map_frame = P('map_frame', 'map').value
        self.base_frame = P('base_frame', 'base_footprint').value
        self.lookahead = P('lookahead', 0.20).value
        self.lookahead_rev = P('lookahead_reverse', 0.15).value
        self.default_speed = P('speed', 0.20).value
        self.w_max = P('w_max', 2.2).value
        self.accel = P('accel', 0.6).value
        self.decel = P('decel', 0.5).value
        self.curv_gain = P('curvature_slowdown', 0.12).value
        self.goal_tol = P('goal_tolerance', 0.03).value
        self.rotate_tol = P('rotate_tolerance', 0.04).value
        self.align_angle = P('align_angle', 1.0).value      # 이보다 크면 먼저 제자리 정렬
        self.offset_rate = P('offset_rate', 0.6).value      # 횡오프셋 변화 제한 [m/s 주행거리당 m]
        self.min_speed = P('min_speed', 0.04).value
        self.lookahead_avoid = P('lookahead_avoid', 0.14).value
        self.stuck_time = P('stuck_time', 2.5).value        # 이 시간 동안
        self.stuck_dist = P('stuck_dist', 0.02).value       # 이 거리도 못 움직이면 끼임
        self.recover_time = P('recover_time', 1.2).value    # 끼임 시 반대 방향 이동 시간
        self.recover_until = None
        self.prog = None
        self.routes = load_routes(self.routes_file) if self.routes_file else {}
        self.get_logger().info(f'경로 {len(self.routes)}개 로드: {sorted(self.routes)}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, P('cmd_topic', '/cmd_vel/route').value, 10)
        self.status_pub = self.create_publisher(String, '/route_follower/status', 10)
        self.path_pub = self.create_publisher(PathMsg, '/route_follower/path', LATCHED)
        self.create_subscription(String, '/route_follower/command', self.on_command, 10)
        self.create_subscription(Float32, '/route_follower/offset', self.on_offset, 10)
        self.create_subscription(String, '/route_follower/reload', lambda m: self.reload(), 10)

        self.state = 'idle'
        self.cmd = {}
        self.path = None
        self.s = 0.0
        self.lat = 0.0
        self.v = 0.0
        self.offset_target = 0.0
        self.offset_time = None
        self.offset = 0.0
        self.aligning = False
        self.create_timer(1.0 / 20.0, self.control)
        self.create_timer(0.1, self.publish_status)

    def reload(self):
        self.routes = load_routes(self.routes_file)
        self.get_logger().info('경로 다시 로드')

    # ------------------------------------------------------------------ 입력
    def on_offset(self, msg):
        self.offset_target = float(msg.data)
        self.offset_time = self.get_clock().now()

    def on_command(self, msg):
        c = from_json_msg(msg)
        if not c:
            return
        kind = c.get('cmd')
        if kind == 'stop':
            self.state = 'idle'
            self.cmd = c
            self.stop()
        elif kind == 'speed':
            self.cmd['speed'] = float(c.get('speed', self.default_speed))
        elif kind == 'rotate':
            self.cmd = c
            self.state = 'rotating'
        elif kind == 'follow':
            name = c.get('route')
            if name not in self.routes:
                self.get_logger().error(f'없는 경로: {name}')
                return
            self.cmd = c
            self.path = self.routes[name]
            self.s = float(c['start_s']) if c.get('start_s') is not None else None
            self.end_s = float(c['end_s']) if c.get('end_s') is not None else self.path.length
            self.offset = 0.0
            self.aligning = True
            self.recover_until = None
            self.prog = None
            self.state = 'following'
            self.publish_path()
            self.get_logger().info(f"경로 추종 시작: {name} (len {self.path.length:.2f} m, "
                                   f"reverse={c.get('reverse', False)})")

    # ------------------------------------------------------------------ 제어
    def stop(self):
        self.v = 0.0
        self.cmd_pub.publish(Twist())

    def control(self):
        if self.state not in ('following', 'rotating'):
            return
        pose = lookup_pose(self.tf_buffer, self.map_frame, self.base_frame)
        if pose is None:
            return
        if self.state == 'rotating':
            err = wrap(float(self.cmd['yaw']) - pose[2])
            if abs(err) < self.rotate_tol:
                self.state = 'done'
                self.stop()
                return
            w = float(np.clip(2.5 * err, -self.cmd.get('w', 1.5), self.cmd.get('w', 1.5)))
            if abs(w) < 0.35:
                w = math.copysign(0.35, w)
            self.publish(0.0, w)
            return
        self.follow(pose)

    def check_stuck(self, pose, reverse):
        """주행 명령 중인데 stuck_time 동안 stuck_dist도 못 움직이면 True(벽·장애물에 끼임)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prog is None or math.hypot(pose[0] - self.prog[0], pose[1] - self.prog[1]) > self.stuck_dist \
                or abs(wrap(pose[2] - self.prog[3])) > 0.15:      # 제자리 회전도 진행으로 인정
            self.prog = (pose[0], pose[1], now, pose[2])
            return False
        return now - self.prog[2] > self.stuck_time

    def follow(self, pose):
        path = self.path
        reverse = bool(self.cmd.get('reverse', False))
        now = self.get_clock().now().nanoseconds * 1e-9
        # 끼임 복구: 진행 반대 방향으로 잠깐 물러난 뒤 다시 정렬
        if self.recover_until is not None:
            if now < self.recover_until:
                self.publish(0.08 if reverse else -0.08, 0.0)
                return
            self.recover_until = None
            self.prog = None
            self.aligning = True
        if self.check_stuck(pose, reverse):
            self.get_logger().warn(f'끼임 감지({path.name}) → {self.recover_time:.1f}s 후진 후 재정렬')
            self.recover_until = now + self.recover_time
            return
        speed = float(self.cmd.get('speed', self.default_speed))
        i, s, lat = path.project(pose[:2], hint=self.s, window=0.5)
        if self.s is None:
            self.s = s
        self.s = max(s, self.s - 0.05)           # 뒤로 크게 튀지 않게
        self.lat = lat
        remaining = self.end_s - self.s
        # 끝점 도달
        ex, ey, _ = path.at(self.end_s)
        dist_end = math.hypot(ex - pose[0], ey - pose[1])
        if remaining < self.goal_tol or (remaining < 0.15 and dist_end < self.goal_tol):
            self.state = 'done'
            self.stop()
            self.get_logger().info(f'경로 완료: {path.name}')
            return

        # 횡오프셋(회피) 갱신: 최근 0.6s 내 명령만 유효, 변화율 제한
        tgt = self.offset_target
        if self.offset_time is None or (self.get_clock().now() - self.offset_time).nanoseconds > 6e8:
            tgt = 0.0
        max_off = float(path.meta.get('max_offset', 0.0))
        tgt = float(np.clip(tgt, -max_off, max_off))
        step = self.offset_rate * max(abs(self.v), 0.05) / 20.0
        self.offset += float(np.clip(tgt - self.offset, -step, step))

        ld = self.lookahead_rev if reverse else self.lookahead
        if abs(self.offset) > 0.02 or abs(tgt) > 0.02:
            ld = min(ld, self.lookahead_avoid)          # 회피 중에는 짧은 전방주시로 빠르게 횡이동
        ld = min(ld, max(remaining, 0.06))
        tx, ty, th = path.at(self.s + ld)
        if abs(self.offset) > 1e-4:
            tx += -math.sin(th) * self.offset
            ty += math.cos(th) * self.offset
        # 가상 로봇(후진 시 뒤를 앞으로 간주) 좌표계에서 목표점
        vyaw = pose[2] + (math.pi if reverse else 0.0)
        dx, dy = tx - pose[0], ty - pose[1]
        lx = math.cos(vyaw) * dx + math.sin(vyaw) * dy
        ly = -math.sin(vyaw) * dx + math.cos(vyaw) * dy
        alpha = math.atan2(ly, lx)
        # 시작 시 방향 차이가 크면 제자리 정렬
        if self.aligning:
            if abs(alpha) > self.align_angle * 0.5:
                w = float(np.clip(2.0 * alpha, -1.5, 1.5))
                self.publish(0.0, w)
                return
            self.aligning = False
        if abs(alpha) > self.align_angle * 1.4:
            self.aligning = True
        L2 = lx * lx + ly * ly
        kappa = 2.0 * ly / max(L2, 1e-4)
        v = speed / (1.0 + self.curv_gain * abs(kappa))
        v = min(v, math.sqrt(2 * self.decel * max(remaining, 0.0)) + self.min_speed)
        v = min(v, abs(self.v) + self.accel / 20.0)
        v = max(v, self.min_speed)
        w = float(np.clip(v * kappa, -self.w_max, self.w_max))
        if abs(v * kappa) > self.w_max:
            v = self.w_max / abs(kappa)
        self.v = v
        self.publish(-v if reverse else v, w)

    def publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    # ------------------------------------------------------------------ 출력
    def publish_path(self):
        m = PathMsg()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        for (x, y), h in zip(self.path.pts[::3], self.path.heading[::3]):
            p = PoseStamped()
            p.header = m.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            q = quat_from_yaw(float(h))
            p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
            m.poses.append(p)
        self.path_pub.publish(m)

    def publish_status(self):
        st = {'state': self.state, 'id': self.cmd.get('id'), 'route': self.path.name if self.path else None}
        if self.state in ('following', 'done') and self.path is not None and self.s is not None:
            st.update(s=round(self.s, 3), remaining=round(self.end_s - self.s, 3), lat=round(self.lat, 3),
                      offset=round(self.offset, 3))
        self.status_pub.publish(to_json_msg(st))


def main():
    rclpy.init()
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, getattr(rclpy.executors, 'ExternalShutdownException', KeyboardInterrupt)):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
