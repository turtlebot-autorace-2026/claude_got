"""LiDAR SLAM(slam_toolbox) 맵을 차선맵 좌표계(map)로 정렬해 Nav2 맵(yaml+pgm)으로 저장.

맵핑 주행 중 slam_toolbox는 자체 프레임(lidar_map)에, 차선 SLAM은 map 프레임에 자세를 낸다.
같은 시각의 두 자세로 T(map←lidar_map)를 추정(평균)하고, /map 점유격자를 그 변환으로 재샘플링한다.

저장: /tunnel_map_align/save (std_msgs/String) 또는 종료 시 out_prefix.{yaml,pgm}
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from autorace_common.geometry import compose, inverse, yaw_from_quat
from autorace_common.ros_utils import LATCHED, lookup_pose


class MapAlign(Node):

    def __init__(self):
        super().__init__('tunnel_map_align')
        P = self.declare_parameter
        self.out_prefix = os.path.expanduser(P('out_prefix', 'tunnel_map').value)
        self.lidar_frame = P('lidar_map_frame', 'lidar_map').value
        self.base_frame = P('base_frame', 'base_footprint').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = []          # 전체 구간 샘플
        self.dark_samples = []     # 차선이 보이지 않는 구간(터널 내부) 샘플 → 터널 주변 정합 우선
        self.npts = 999
        self.create_subscription(String, '/lane_slam/status', self.on_status, 10)
        self.grid = None
        self.create_subscription(PoseStamped, '/lane_slam/pose', self.on_pose, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(String, '/tunnel_map_align/save', lambda m: self.save(), 10)

    def on_status(self, msg):
        try:
            import json
            self.npts = int(json.loads(msg.data).get('npts', 999))
        except (ValueError, TypeError):
            pass

    def on_pose(self, msg):
        lp = lookup_pose(self.tf_buffer, self.lidar_frame, self.base_frame, Time.from_msg(msg.header.stamp), 0.05)
        if lp is None:
            return
        mp = (msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation))
        T = compose(mp, inverse(lp))
        self.samples.append(T)
        if self.npts < 5:
            self.dark_samples.append(T)

    def on_map(self, msg):
        self.grid = msg

    def transform(self):
        if len(self.dark_samples) >= 30:
            a = np.array(self.dark_samples)
            self.get_logger().info(f'터널 구간 샘플 {len(a)}개로 정렬')
        else:
            a = np.array(self.samples[len(self.samples) // 10:])     # 초기 수렴 구간 제외
        th = math.atan2(np.mean(np.sin(a[:, 2])), np.mean(np.cos(a[:, 2])))
        return (float(np.median(a[:, 0])), float(np.median(a[:, 1])), th)

    def save(self):
        if self.grid is None or len(self.samples) < 20:
            self.get_logger().warn('저장할 맵/정렬 샘플 부족')
            return
        T = self.transform()
        g = self.grid
        res = g.info.resolution
        w, h = g.info.width, g.info.height
        data = np.array(g.data, np.int16).reshape(h, w)
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        # lidar_map 격자 모서리를 map 좌표로 변환해 출력 범위 결정
        corners = np.array([[ox, oy], [ox + w * res, oy], [ox, oy + h * res], [ox + w * res, oy + h * res]])
        c, s = math.cos(T[2]), math.sin(T[2])
        mc = np.column_stack((T[0] + c * corners[:, 0] - s * corners[:, 1], T[1] + s * corners[:, 0] + c * corners[:, 1]))
        mn, mx = mc.min(0), mc.max(0)
        W, H = int(math.ceil((mx[0] - mn[0]) / res)), int(math.ceil((mx[1] - mn[1]) / res))
        # 출력 격자 중심 → lidar_map 좌표 → 원본 격자 인덱스(최근접)
        jj, ii = np.meshgrid(np.arange(W), np.arange(H))
        X = mn[0] + (jj + 0.5) * res - T[0]
        Y = mn[1] + (ii + 0.5) * res - T[1]
        lx, ly = c * X + s * Y, -s * X + c * Y
        cj = np.floor((lx - ox) / res).astype(int)
        ci = np.floor((ly - oy) / res).astype(int)
        ok = (cj >= 0) & (cj < w) & (ci >= 0) & (ci < h)
        out = np.full((H, W), -1, np.int16)
        out[ok] = data[ci[ok], cj[ok]]
        img = np.full((H, W), 205, np.uint8)
        img[out == 0] = 254
        img[out >= 65] = 0
        img[(out > 0) & (out < 65)] = 205
        os.makedirs(os.path.dirname(os.path.abspath(self.out_prefix)), exist_ok=True)
        cv2.imwrite(self.out_prefix + '.pgm', np.flipud(img))
        meta = {'image': os.path.basename(self.out_prefix) + '.pgm', 'mode': 'trinary', 'resolution': float(res),
                'origin': [float(mn[0]), float(mn[1]), 0.0], 'negate': 0, 'occupied_thresh': 0.65,
                'free_thresh': 0.25}
        with open(self.out_prefix + '.yaml', 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False)
        self.get_logger().info(f'LiDAR 맵 저장 {self.out_prefix}.yaml  T(map←lidar_map)={np.round(T, 4).tolist()}')


def main():
    rclpy.init()
    node = MapAlign()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, getattr(rclpy.executors, 'ExternalShutdownException', KeyboardInterrupt)):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
