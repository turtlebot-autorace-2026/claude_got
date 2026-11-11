"""공사구간 LiDAR 실시간 장애물 회피 노드.

현재 추종 중인 경로(/route_follower/path) 좌표계로 LiDAR 점을 바꿔 장애물별로 묶고,
아직 지나가지 않은 가장 가까운 장애물 옆의 더 넓은 빈 공간 중앙을 목표 횡오프셋으로 고른다.
(규정상 장애물 접촉은 감점이 없으므로 '통과'를 최우선으로 하는 결정적 규칙)
결과는 /route_follower/offset(Float32, 왼쪽+)으로 발행 → 경로 추종기가 목표점을 그만큼 옆으로 이동.

활성화: /construction/enable (Bool). 비활성 시 아무것도 발행하지 않음(추종기는 0.6s 후 오프셋 0으로 복귀).
상태: /construction/status (JSON: offset, clearance, n_obstacle_pts)
"""

import numpy as np
import rclpy
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from autorace_common.geometry import transform_points
from autorace_common.paths import Path
from autorace_common.ros_utils import LATCHED, SENSOR, from_json_msg, lookup_pose, to_json_msg


class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('construction_avoidance')
        P = self.declare_parameter
        self.max_offset = P('max_offset', 0.17).value
        self.horizon = P('horizon', 0.65).value          # 앞쪽 검사 거리[m]
        self.back = P('back', 0.10).value                # 뒤쪽 검사 거리[m]
        self.road_half_width = P('road_half_width', 0.29).value  # 경로 중심~도로 경계[m] (이 밖의 점은 무시)
        self.pass_gap = P('pass_gap', 0.15).value        # s 방향 이 간격 이상이면 다른 장애물
        # 로봇 중심이 장애물 끝 + pass_margin을 지나면 다음 장애물 기준으로 전환.
        # 음수 = 로봇 앞부분이 장애물을 벗어나는 즉시 다음 회피를 시작(장애물 간격이 좁은 슬라럼 대응, 접촉은 감점 없음)
        self.pass_margin = P('pass_margin', -0.06).value
        self.map_frame = P('map_frame', 'map').value
        self.enabled = P('start_enabled', False).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = None
        self.s = None
        self.offset = 0.0
        self.pub = self.create_publisher(Float32, '/route_follower/offset', 10)
        self.status_pub = self.create_publisher(String, '/construction/status', 10)
        self.create_subscription(PathMsg, '/route_follower/path', self.on_path, LATCHED)
        self.create_subscription(String, '/route_follower/status', self.on_status, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.create_subscription(Bool, '/construction/enable', self.on_enable, 10)

    def on_enable(self, msg):
        self.enabled = msg.data
        self.offset = 0.0
        self.get_logger().info(f'장애물 회피 {"활성" if msg.data else "비활성"}')

    def on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) >= 2:
            self.path = Path(pts, step=0.02)

    def on_status(self, msg):
        st = from_json_msg(msg) or {}
        self.s = st.get('s')
        if st.get('offset') is not None:
            self.offset = float(st['offset'])

    def on_scan(self, msg):
        if not self.enabled or self.path is None or self.s is None:
            return
        pose = lookup_pose(self.tf_buffer, self.map_frame, msg.header.frame_id)
        if pose is None:
            return
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < 1.2)
        obs = transform_points(pose, np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))))
        best = self.plan(obs)
        self.pub.publish(Float32(data=float(best[0])))
        self.status_pub.publish(to_json_msg({'offset': round(best[0], 3), 'margin': round(best[1], 3),
                                             'n_obs': int(best[2])}))

    def plan(self, obs):
        """간격(gap) 기반 회피 → (목표 오프셋, 최근접 장애물 횡거리 여유, 장애물 점 수).

        1) 장애물 점을 경로 좌표 (s 진행거리, l 횡거리, 왼쪽+)로 변환, 도로 폭 안·전방 horizon 안만 사용
        2) s 방향으로 pass_gap 이상 떨어지면 다른 장애물로 군집화
        3) 로봇 중심이 아직 지나가지 않은 가장 가까운 군집에서 왼쪽/오른쪽 빈 폭을 비교해
           넓은 쪽 빈 공간의 중앙을 목표 오프셋으로 선택(도로 경계 road_half_width 기준)
        """
        path = self.path
        s0 = float(self.s)
        if len(obs) == 0:
            return (0.0, 9.9, 0)
        # 경로 좌표 변환(가까운 경로점 기준)
        lo, hi = max(s0 - self.back, 0.0), min(s0 + self.horizon, path.length)
        idx = np.nonzero((path.s >= lo - 0.3) & (path.s <= hi + 0.3))[0]
        if len(idx) < 2:
            return (0.0, 9.9, 0)
        P = path.pts[idx]
        d2 = ((obs[:, None, :] - P[None, :, :]) ** 2).sum(-1)
        k = idx[np.argmin(d2, axis=1)]
        h = path.heading[k]
        v = obs - path.pts[k]
        ss = path.s[k] + np.cos(h) * v[:, 0] + np.sin(h) * v[:, 1]
        ll = -np.sin(h) * v[:, 0] + np.cos(h) * v[:, 1]
        W = self.road_half_width
        m = (ss >= lo) & (ss <= hi) & (np.abs(ll) <= W)
        ss, ll = ss[m], ll[m]
        if len(ss) == 0:
            return (0.0, 9.9, 0)
        order = np.argsort(ss)
        ss, ll = ss[order], ll[order]
        breaks = np.nonzero(np.diff(ss) > self.pass_gap)[0] + 1
        target, margin = 0.0, 9.9
        for cs, cl in zip(np.split(ss, breaks), np.split(ll, breaks)):
            if cs.max() + self.pass_margin < s0:       # 이미 지나간 장애물
                continue
            lmin, lmax = float(cl.min()), float(cl.max())
            free_left = W - lmax
            free_right = lmin + W
            if free_left >= free_right:
                target = (lmax + W) / 2.0
            else:
                target = (lmin - W) / 2.0
            target = float(np.clip(target, -self.max_offset, self.max_offset))
            margin = float(min(abs(target - lmin), abs(target - lmax)))
            break
        return (target, margin, len(ss))


def main():
    rclpy.init()
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
