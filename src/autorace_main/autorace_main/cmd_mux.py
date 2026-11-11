"""cmd_vel 선택기: 미션 매니저가 고른 제어원만 실제 /cmd_vel로 전달.

입력: /cmd_vel/route(경로추종), /cmd_vel/nav2(터널 Nav2), /cmd_vel/mission(매니저 직접 제어)
선택: /cmd_mux/select (String: route|nav2|mission|stop)
신선도(timeout) 지난 입력은 0 속도로 대체 → 노드가 죽어도 로봇이 계속 움직이지 않음.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

SOURCES = ('route', 'nav2', 'mission')


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')
        self.timeout = self.declare_parameter('timeout', 0.4).value
        self.max_v = self.declare_parameter('max_linear', 0.3).value
        self.max_w = self.declare_parameter('max_angular', 2.5).value
        self.selected = 'stop'
        self.last = {s: (None, Twist()) for s in SOURCES}
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        for s in SOURCES:
            self.create_subscription(Twist, f'/cmd_vel/{s}', lambda m, s=s: self.on_cmd(s, m), 10)
        self.create_subscription(String, '/cmd_mux/select', self.on_select, 10)
        self.create_timer(0.05, self.tick)

    def on_cmd(self, src, msg):
        self.last[src] = (self.get_clock().now(), msg)

    def on_select(self, msg):
        if msg.data != self.selected:
            self.get_logger().info(f'제어원: {self.selected} → {msg.data}')
        self.selected = msg.data if msg.data in SOURCES else 'stop'

    def tick(self):
        out = Twist()
        if self.selected in SOURCES:
            t, m = self.last[self.selected]
            if t is not None and (self.get_clock().now() - t).nanoseconds * 1e-9 < self.timeout:
                out.linear.x = max(-self.max_v, min(self.max_v, m.linear.x))
                out.angular.z = max(-self.max_w, min(self.max_w, m.angular.z))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
