"""차단바 상태 인식 노드.

차단바는 빨강/흰 줄무늬 막대다. 내려오면 /camera2 화면 중앙 띠에 '가로로 긴' 빨간 영역이,
올라가 있으면 세로 막대로 보인다. 가로 폭(열 범위)이 영상 폭의 일정 비율 이상이면 closed.
LiDAR 보조(use_lidar, 기본 끔): 정면 좁은 부채꼴에 가까운 점이 있으면 closed로 본다(막대가 스캔 높이에 걸릴 때).

출력: /level_crossing/state (String: closed|moving|open, latched, 디바운스 적용)
  moving = 차단바가 내려오거나 올라가는 중(대각선)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String

from autorace_common.ros_utils import LATCHED, SENSOR


class LevelCrossingNode(Node):

    def __init__(self):
        super().__init__('level_crossing')
        P = self.declare_parameter
        self.band = list(P('band', [150, 360]).value)            # 검사할 영상 행 범위 y0, y1
        self.min_col_pixels = P('min_col_pixels', 8).value       # 열마다 빨간 픽셀 최소 개수
        self.min_coverage = P('min_coverage', 0.20).value        # 빨간 열 비율
        self.min_extent = P('min_extent', 0.40).value            # 빨간 열 가로 범위 비율
        self.max_height = P('max_height', 170).value             # 빨간 영역 세로 범위 한계(세로 막대 제외)
        self.last_metric = None
        self.use_lidar = P('use_lidar', False).value
        self.enabled = P('start_enabled', False).value
        self.lidar_range = P('lidar_range', 0.35).value
        self.lidar_half_angle = P('lidar_half_angle', 0.25).value
        self.confirm = P('confirm_frames', 3).value
        self.publish_debug = P('publish_debug', False).value
        self.cam_closed = False
        self.lidar_closed = False
        self.state = 'open'
        self.cand, self.count = None, 0
        self.pub = self.create_publisher(String, '/level_crossing/state', LATCHED)
        self.dbg = self.create_publisher(CompressedImage, '/level_crossing/debug', 2)
        self.create_subscription(CompressedImage, P('image_topic', '/camera2').value, self.on_image, 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.create_subscription(Bool, '/level_crossing/enable', self.on_enable, 10)
        self.pub.publish(String(data=self.state))

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.cand, self.count = None, 0

    def detect(self, bgr):
        """내려온 차단바: 띠 영역에서 빨간 블록들이 가로로 넓게 분포(빨강-흰 줄무늬라 블록이 끊겨 있음).

        - coverage : 빨간 픽셀이 일정 개수 이상 있는 열의 비율
        - extent   : 가장 왼쪽~오른쪽 빨간 열의 폭 비율
        - 세워진 차단바는 좁은 세로 막대라 coverage가 작다.
        """
        y0, y1 = self.band
        hsv = cv2.cvtColor(bgr[y0:y1], cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, np.array([0, 120, 60]), np.array([10, 255, 255])) | \
            cv2.inRange(hsv, np.array([165, 120, 60]), np.array([180, 255, 255]))
        col = (red > 0).sum(axis=0)
        has = col >= self.min_col_pixels
        W = bgr.shape[1]
        if has.sum() == 0:
            return 'open', None
        cols = np.nonzero(has)[0]
        coverage = has.mean()
        extent = (cols.max() - cols.min()) / float(W)
        rows = np.nonzero((red > 0).sum(axis=1) >= 5)[0]
        height = (rows.max() - rows.min()) if len(rows) else 0
        self.last_metric = (round(float(coverage), 3), round(float(extent), 3), int(height))
        box = (int(cols.min()), int(y0 + (rows.min() if len(rows) else 0)), int(cols.max() - cols.min()), int(height))
        if coverage >= self.min_coverage and extent >= self.min_extent:
            # 가로로 넓고 납작하면 내려온 상태, 넓지만 세로로도 크면 움직이는 중(대각선)
            return ('closed' if height <= self.max_height else 'moving'), box
        if coverage >= self.min_coverage * 0.5 and extent >= self.min_extent * 0.6:
            return 'moving', box             # 대각선(내려오거나 올라가는 중). 좁은 세로 기둥(올라간 상태)은 제외
        return 'open', None

    def on_scan(self, msg):
        if not self.use_lidar:
            return
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        a = (a + np.pi) % (2 * np.pi) - np.pi
        sel = (np.abs(a) < self.lidar_half_angle) & np.isfinite(r) & (r > msg.range_min)
        self.lidar_closed = bool(np.sum(r[sel] < self.lidar_range) >= 3)

    def on_image(self, msg):
        if not self.enabled:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        r = self.detect(img)
        cam, box = r if r is not None else ('open', None)
        self.cam_closed = cam == 'closed'
        det = 'closed' if (self.cam_closed or self.lidar_closed) else cam
        if det == self.cand:
            self.count += 1
        else:
            self.cand, self.count = det, 1
        if self.count >= self.confirm and det != self.state:
            self.state = det
            self.get_logger().info(f'차단바: {det} (camera={self.cam_closed}, lidar={self.lidar_closed})')
            self.pub.publish(String(data=det))
        if self.publish_debug:
            cv2.rectangle(img, (0, self.band[0]), (img.shape[1] - 1, self.band[1]), (255, 0, 0), 1)
            if box:
                x, y, w, h = box
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            out = CompressedImage(header=msg.header, format='jpeg')
            out.data = cv2.imencode('.jpg', img)[1].tobytes()
            self.dbg.publish(out)


def main():
    rclpy.init()
    node = LevelCrossingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
