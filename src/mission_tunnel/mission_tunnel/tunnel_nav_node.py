"""터널 Nav2 브리지 노드.

/tunnel/command (String JSON)
  {"cmd": "go"}                         기록 경로 goal_route 위 goal_s 지점(출구를 바라보는 자세)으로 NavigateToPose
  {"cmd": "go", "x":.., "y":.., "yaw":..} 지정 자세로 이동
  {"cmd": "cancel"}
/tunnel/status (String JSON): {"state": idle|active|succeeded|failed, "detail": ...}
"""
import os

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from autorace_common.geometry import quat_from_yaw
from autorace_common.paths import load_routes
from autorace_common.ros_utils import from_json_msg, to_json_msg


class TunnelNav(Node):

    def __init__(self):
        super().__init__('tunnel_nav')
        P = self.declare_parameter
        self.routes_file = os.path.expanduser(P('routes_file', '').value)
        self.goal_route = P('goal_route', 'tunnel_exit').value
        # 출구 경로 시작점에서 goal_s[m] 진행한 지점을 목표로: 좁은 출구를 '정면으로 바라보는' 자세에서
        # Nav2를 끝내고 이후는 경로 추종기가 출구를 빠져나간다.
        self.goal_s = P('goal_s', 0.20).value
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/tunnel/status', 10)
        self.create_subscription(String, '/tunnel/command', self.on_command, 10)
        self.state, self.detail = 'idle', ''
        self.goal_handle = None
        self.create_timer(0.2, lambda: self.status_pub.publish(to_json_msg({'state': self.state,
                                                                             'detail': self.detail})))

    def goal_pose(self, c):
        if 'x' in c:
            return float(c['x']), float(c['y']), float(c.get('yaw', 0.0))
        r = load_routes(self.routes_file)[self.goal_route]
        return r.at(self.goal_s)

    def on_command(self, msg):
        c = from_json_msg(msg) or {}
        if c.get('cmd') == 'cancel' and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.state = 'idle'
            return
        if c.get('cmd') != 'go':
            return
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.state, self.detail = 'failed', 'navigate_to_pose 서버 없음'
            return
        x, y, yaw = self.goal_pose(c)
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        q = quat_from_yaw(yaw)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = q[2], q[3]
        self.state, self.detail = 'active', f'goal ({x:.2f}, {y:.2f}, {yaw:.2f})'
        self.get_logger().info(f'터널 Nav2 목표 전송: {self.detail}')
        self.client.send_goal_async(g).add_done_callback(self.on_accept)

    def on_accept(self, fut):
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            self.state, self.detail = 'failed', 'goal rejected'
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        st = fut.result().status
        self.state = 'succeeded' if st == GoalStatus.STATUS_SUCCEEDED else 'failed'
        self.detail = f'status {st}'
        self.get_logger().info(f'터널 Nav2 결과: {self.state} ({st})')


def main():
    rclpy.init()
    node = TunnelNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
