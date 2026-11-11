"""신호등 인식 노드.

/camera2(전방 카메라) 영상의 ROI에서 밝게 켜진 빨강/노랑/초록 램프 블롭을 찾는다.
연속 N프레임 같은 결과일 때만 상태를 바꿔 /traffic_light/state(String: red|yellow|green|none)로 발행.
디버그 영상: /traffic_light/debug (CompressedImage, publish_debug=true일 때)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from autorace_common.ros_utils import LATCHED

# HSV 범위(OpenCV): 켜진 램프는 채도·명도가 높다
COLORS = {
    'red': [((0, 120, 150), (10, 255, 255)), ((165, 120, 150), (180, 255, 255))],
    'yellow': [((18, 120, 150), (35, 255, 255))],
    'green': [((45, 100, 130), (90, 255, 255))],
}


class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light')
        P = self.declare_parameter
        self.roi = list(P('roi', [320, 0, 640, 380]).value)       # x0, y0, x1, y1
        self.min_area = P('min_area', 120).value
        self.max_area = P('max_area', 6000).value
        self.min_fill = P('min_fill', 0.45).value                  # 블롭 면적/외접사각형 (원형 램프 판별)
        self.confirm = P('confirm_frames', 3).value
        self.publish_debug = P('publish_debug', False).value
        self.enabled = True   # 출발 후 매니저가 /traffic_light/enable false로 끔(연산 절약)
        self.state = 'none'
        self.cand = None
        self.count = 0
        self.pub = self.create_publisher(String, '/traffic_light/state', LATCHED)
        self.dbg = self.create_publisher(CompressedImage, '/traffic_light/debug', 2)
        self.create_subscription(CompressedImage, P('image_topic', '/camera2').value, self.on_image, 5)
        self.create_subscription(Bool, '/traffic_light/enable', lambda m: setattr(self, 'enabled', m.data), 10)
        self.pub.publish(String(data=self.state))

    def detect(self, bgr):
        x0, y0, x1, y1 = self.roi
        roi = cv2.GaussianBlur(bgr[y0:y1, x0:x1], (5, 5), 0)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        best, best_area, boxes = 'none', 0, []
        for name, ranges in COLORS.items():
            m = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            n, _, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
            for i in range(1, n):
                x, y, w, h, a = stats[i]
                if not (self.min_area <= a <= self.max_area):
                    continue
                if a / float(w * h) < self.min_fill or not (0.3 < w / float(h) < 3.0):
                    continue
                boxes.append((name, x + x0, y + y0, w, h))
                if a > best_area:
                    best, best_area = name, a
        return best, boxes

    def on_image(self, msg):
        if not self.enabled:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        det, boxes = self.detect(img)
        if det == self.cand:
            self.count += 1
        else:
            self.cand, self.count = det, 1
        if self.count >= self.confirm and det != self.state:
            self.state = det
            self.get_logger().info(f'신호등: {det}')
            self.pub.publish(String(data=det))
        if self.publish_debug:
            x0, y0, x1, y1 = self.roi
            cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), (255, 0, 0), 2)
            for name, x, y, w, h in boxes:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(img, name, (x, y - 4), 0, 0.5, (0, 255, 255), 1)
            out = CompressedImage(header=msg.header, format='jpeg')
            out.data = cv2.imencode('.jpg', img)[1].tobytes()
            self.dbg.publish(out)


def main():
    rclpy.init()
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, getattr(rclpy.executors, 'ExternalShutdownException', KeyboardInterrupt)):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
