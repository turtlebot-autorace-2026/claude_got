"""ROS 2 공용 헬퍼: JSON String 메시지, TF 기반 로봇 pose 조회, QoS."""
import json

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from .geometry import yaw_from_quat

# 최근 값 1개를 늦게 접속한 구독자에게도 전달(상태 토픽용)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def to_json_msg(obj):
    return String(data=json.dumps(obj, ensure_ascii=False))


def from_json_msg(msg):
    try:
        return json.loads(msg.data)
    except (ValueError, TypeError):
        return None


def lookup_pose(tf_buffer, target='map', source='base_footprint', time=None, timeout=0.0):
    """TF에서 target 기준 source의 (x, y, yaw). 실패 시 None."""
    try:
        t = tf_buffer.lookup_transform(target, source, time or Time(),
                                       timeout=Duration(seconds=timeout))
    except Exception:  # noqa: BLE001  (tf2 예외 종류가 다양함)
        return None
    tr = t.transform.translation
    return (tr.x, tr.y, yaw_from_quat(t.transform.rotation))
