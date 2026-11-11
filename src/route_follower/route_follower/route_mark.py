"""맵핑 주행 중 경로 구간 표시 CLI (route_recorder에 전달).

  ros2 run route_follower route_mark <구간이름> [speed] [--reverse] [--max-offset 0.17]
  ros2 run route_follower route_mark pause      # 기록 일시정지(구간 사이 이동·정렬 시)
  ros2 run route_follower route_mark save       # routes.yaml 저장
"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return
    rclpy.init()
    node = rclpy.create_node('route_mark')
    seg = node.create_publisher(String, '/route_recorder/segment', 10)
    save = node.create_publisher(String, '/route_recorder/save', 10)
    t0 = time.time()
    while time.time() - t0 < 1.0 and seg.get_subscription_count() == 0:   # recorder 연결 대기
        rclpy.spin_once(node, timeout_sec=0.1)
    if a[0] == 'save':
        save.publish(String(data='save'))
        msg = 'save'
    else:
        d = {'name': '' if a[0] == 'pause' else a[0]}
        rest = a[1:]
        if '--reverse' in rest:
            d['reverse'] = True
            rest.remove('--reverse')
        if '--max-offset' in rest:
            i = rest.index('--max-offset')
            d['max_offset'] = float(rest[i + 1])
            del rest[i:i + 2]
        if rest:
            d['speed'] = float(rest[0])
        seg.publish(String(data=json.dumps(d)))
        msg = json.dumps(d, ensure_ascii=False)
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    print('sent', msg, '(recorder 연결:', seg.get_subscription_count(), ')')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
