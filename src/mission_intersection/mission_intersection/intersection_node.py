"""갈림길 이정표(파란 원 + 흰 화살표) 방향 인식 노드.

1) /camera2에서 파란색 원형 블롭(세워진 표지판) 검출. 눕혀진 표지판은 납작하므로 종횡비로 제외
2) 원 내부 흰색 화살표에서 '세로 폭이 가장 큰 열'(화살촉)의 위치가 중심보다 왼쪽이면 left, 오른쪽이면 right
3) 프레임별 투표를 누적해 /intersection/direction (String: left|right|none, latched) 발행

/intersection/enable (Bool)로 투표 초기화·활성화. 비활성 중에도 디버그용 판단은 계속한다.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from autorace_common.ros_utils import LATCHED


def arrow_direction(bgr, box):
    """파란 원 bbox 안의 흰 화살표 방향 → ('left'|'right'|None, 신뢰도).

    회전 지시 표지판은 곡선 화살표(↱ / ↰)다. 화살표의 '머리+가로획'은 위쪽 절반, '기둥'은 아래쪽 절반에
    있으므로 (위쪽 흰 픽셀 무게중심 x − 아래쪽 무게중심 x) 부호로 방향을 정한다.
    직선 화살표(→/←)처럼 위·아래 차이가 작으면 가로 방향 무게중심-중앙 비교로 대체한다.
    """
    x, y, w, h = box
    roi = bgr[y:y + h, x:x + w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 80, 255]))
    mask = np.zeros_like(white)
    cv2.ellipse(mask, (w // 2, h // 2), (int(w * 0.42), int(h * 0.42)), 0, 0, 360, 255, -1)
    white &= mask
    ys, xs = np.nonzero(white)
    if len(xs) < 0.03 * w * h:
        return None, 0.0
    upper = ys < h / 2.0
    if upper.sum() < 5 or (~upper).sum() < 5:
        return None, 0.0
    d = (xs[upper].mean() - xs[~upper].mean()) / float(w)
    if abs(d) < 0.08:
        # 직선 화살표: 머리(세로 폭이 큰 쪽)가 무게중심을 끌어당김
        d = (xs.mean() - w / 2.0) / float(w)
        if abs(d) < 0.03:
            return None, 0.0
    return ('right' if d > 0 else 'left'), float(min(abs(d) * 4, 1.0))


class IntersectionNode(Node):

    def __init__(self):
        super().__init__('intersection')
        P = self.declare_parameter
        self.min_area = P('min_blue_area', 1500).value
        self.votes_needed = P('votes_needed', 4).value
        self.publish_debug = P('publish_debug', False).value
        self.enabled = P('start_enabled', True).value
        self.votes = {'left': 0.0, 'right': 0.0}
        self.decided = 'none'
        self.pub = self.create_publisher(String, '/intersection/direction', LATCHED)
        self.dbg = self.create_publisher(CompressedImage, '/intersection/debug', 2)
        self.create_subscription(CompressedImage, P('image_topic', '/camera2').value, self.on_image, 5)
        self.create_subscription(Bool, '/intersection/enable', self.on_enable, 10)
        self.pub.publish(String(data='none'))

    def on_enable(self, msg):
        self.enabled = msg.data
        self.votes = {'left': 0.0, 'right': 0.0}
        self.decided = 'none'
        self.pub.publish(String(data='none'))

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, np.array([100, 120, 50]), np.array([130, 255, 255]))
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        res = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if not (0.6 < w / float(h) < 1.6):        # 눕혀진 표지판 제외
                continue
            H, W = bgr.shape[:2]
            if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:   # 화면에 잘린 표지판 제외
                continue
            if a / (np.pi * w * h / 4.0) < 0.6:          # 원형도
                continue
            d, conf = arrow_direction(bgr, (x, y, w, h))
            res.append(((x, y, w, h), d, conf, a))
        return res

    def on_image(self, msg):
        if not self.enabled and not self.publish_debug:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        dets = self.detect(img)
        if self.enabled and dets:
            box, d, conf, area = max(dets, key=lambda r: r[3])
            if d:
                self.votes[d] += conf
                other = 'left' if d == 'right' else 'right'
                if self.votes[d] >= self.votes_needed and self.votes[d] > 2 * self.votes[other] \
                        and self.decided != d:
                    self.decided = d
                    self.get_logger().info(f'갈림길 방향: {d} (votes {self.votes})')
                    self.pub.publish(String(data=d))
        if self.publish_debug:
            for (x, y, w, h), d, conf, _ in dets:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, f'{d}:{conf:.2f}', (x, y - 4), 0, 0.5, (0, 255, 0), 1)
            out = CompressedImage(header=msg.header, format='jpeg')
            out.data = cv2.imencode('.jpg', img)[1].tobytes()
            self.dbg.publish(out)


def main():
    rclpy.init()
    node = IntersectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
