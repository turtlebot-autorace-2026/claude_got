"""차선 SLAM 노드.

mode=mapping       : odom 예측 + 상관(correlative) 매칭으로 자세 보정하며 차선 격자맵 생성
                     (extend_map=true면 기존 맵을 불러와 이어서 확장 → 여러 번 나눠 주행해 맵핑 가능)
mode=localization  : 저장된 차선맵에서 파티클 필터로 위치추정. lidar_map_yaml을 주면
                     lidar_regions(터널 등) 안에서는 LiDAR 점유맵 우도도 함께 사용

출력: TF map→odom, /lane_slam/pose(PoseStamped), /lane_slam/status(JSON), /lane_slam/map(OccupancyGrid)
"""
import math
import os
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from autorace_common.geometry import compose, inverse, point_in_polygon, quat_from_yaw, relative, transform_points, wrap
from autorace_common.ros_utils import LATCHED, SENSOR, lookup_pose, to_json_msg

from .estimator import CorrelativeMatcher, OccupancyField, ParticleFilter, score_lanes
from .grid_map import CLASSES, LaneMap
from .lane_extractor import GroundProjector, LaneExtractor, YoloLaneExtractor, optical_extrinsic_from_params


class LaneSlamNode(Node):

    def __init__(self):
        super().__init__('lane_slam')
        P = self.declare_parameter
        self.mode = P('mode', 'localization').value
        self.map_prefix = os.path.expanduser(P('map_prefix', '').value)
        self.extend_map = P('extend_map', False).value
        self.initial_pose = list(P('initial_pose', [0.0, 0.0, 0.0]).value)
        self.map_frame = P('map_frame', 'map').value
        self.odom_frame = P('odom_frame', 'odom').value
        self.base_frame = P('base_frame', 'base_footprint').value
        self.camera_frame = P('camera_frame', 'camera_optical_frame').value
        self.use_tf_extrinsics = P('use_tf_extrinsics', True).value
        self.cam_params = [P('cam_x', 0.076).value, P('cam_y', 0.0).value, P('cam_z', 0.135).value,
                           P('cam_pitch', 0.5515).value, P('cam_yaw', 0.0394).value]
        self.ground_z = P('ground_z', -0.010).value
        self.x_range = list(P('ground_x_range', [0.12, 0.85]).value)
        self.y_max = P('ground_y_max', 0.6).value
        self.publish_tf = P('publish_tf', True).value
        self.min_points = P('min_points', 12).value
        self.lf_sigma = P('lf_sigma', 0.03).value
        self.odom_noise = list(P('odom_noise', [0.0, 0.0]).value)   # 시험용 odom 잡음 비율 [이동, 회전]
        self.map_bounds = list(P('map_bounds', [-3.2, -0.8, 5.6, 5.6]).value)  # x0, y0, w, h
        hsv = {k: P(k, v).value for k, v in dict(white_v_min=170, white_s_max=60, yellow_h_min=18,
                                                  yellow_h_max=38, yellow_s_min=90, yellow_v_min=90,
                                                  pixel_stride=2, voxel=0.015).items()}
        self.hsv = hsv
        # 차선 검출기: 'yolo'(turtle_ws YOLO 분할 모델, 기본) 또는 'hsv'(색 임계값)
        self.detector = P('lane_detector', 'yolo').value
        default_model = ''
        try:
            from ament_index_python.packages import get_package_share_directory
            default_model = os.path.join(get_package_share_directory('lane_slam'), 'models', 'lane_yolo26n_seg.onnx')
        except Exception:  # noqa: BLE001
            pass
        self.yolo_model = os.path.expanduser(P('yolo_model', default_model).value)
        self.yolo_device = P('yolo_device', '').value
        self.yolo_conf = P('yolo_conf', 0.4).value
        pf_n = P('particles', 300).value
        pf_gain = P('pf_gain', 25.0).value
        self.lidar_map_yaml = os.path.expanduser(P('lidar_map_yaml', '').value)
        self.lidar_regions = P('lidar_regions', [0.0]).value   # [n, x1,y1,...,xn,yn, n, ...]
        self.lidar_gain = P('lidar_gain', 15.0).value
        self.lidar_only_in_region = P('lidar_only_in_region', True).value
        # 차선 점이 min_points 미만인 프레임이 이 횟수 이상 이어지면(터널 등) LiDAR 우도 사용
        self.lidar_dark_frames = P('lidar_dark_frames', 3).value
        self.dark = 0

        # 맵/추정기
        if self.mode == 'mapping':
            if self.extend_map and self.map_prefix and os.path.exists(self.map_prefix + '.npz'):
                self.map = LaneMap.load(self.map_prefix)
                self.get_logger().info(f'기존 차선맵 확장: {self.map_prefix}')
            else:
                b = self.map_bounds
                self.map = LaneMap(0.01, (b[0], b[1]), (b[2], b[3]))
            self.matcher = CorrelativeMatcher(sigma_xy=P('match_sigma_xy', 0.03).value,
                                              sigma_th=P('match_sigma_th', 0.01).value,
                                              gain=P('match_gain', 0.3).value)
        else:
            if not self.map_prefix or not os.path.exists(self.map_prefix + '.npz'):
                raise SystemExit(f'차선맵 없음: {self.map_prefix}')
            self.map = LaneMap.load(self.map_prefix)
            self.pf = ParticleFilter(pf_n, gain=pf_gain)
            self.pf.init(self.initial_pose)
        self.fields = self.map.likelihood_fields(self.lf_sigma) if self.map.has_data() else None
        self.known = self.map.known_mask() if self.map.has_data() else None
        self.lidar_field = None
        if self.lidar_map_yaml and os.path.exists(self.lidar_map_yaml):
            self.lidar_field = OccupancyField(self.lidar_map_yaml)
            self.get_logger().info(f'LiDAR 맵 로드: {self.lidar_map_yaml}')
        self.regions = self._parse_regions(self.lidar_regions)

        self.pose = tuple(self.initial_pose)       # map 기준 base 추정 자세
        self.last_odom = None                      # 직전 처리 시점 odom 자세
        self.map_to_odom = None
        self.last_insert_pose = None
        self.extractor = None
        self.rng = np.random.default_rng(1)
        self.n_frames = 0
        self.last_status = {}
        self.scan_offset = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/lane_slam/pose', 10)
        self.status_pub = self.create_publisher(String, '/lane_slam/status', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/lane_slam/map', LATCHED)
        self.create_subscription(CameraInfo, P('camera_info_topic', '/camera/camera_info').value,
                                 self.on_info, 10)
        self.create_subscription(CompressedImage, P('camera_topic', '/camera').value, self.on_image, 5)
        if self.lidar_field is not None:
            self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR)
        self.create_service(Trigger, '/lane_slam/save_map', self.srv_save)
        self.create_timer(1.0 / 30.0, self.publish_tf_timer)
        self.create_timer(3.0 if self.mode == 'mapping' else 30.0, self.publish_map)
        self.K = None
        self.get_logger().info(f'lane_slam 시작: mode={self.mode}, map={self.map_prefix}')

    # ------------------------------------------------------------------ 초기화
    @staticmethod
    def _parse_regions(flat):
        regions, i = [], 0
        flat = list(flat)
        while i < len(flat):
            n = int(flat[i])
            if n < 3:
                break
            pts = [(flat[i + 1 + 2 * k], flat[i + 2 + 2 * k]) for k in range(n)]
            regions.append(pts)
            i += 1 + 2 * n
        return regions

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.img_size = (msg.width, msg.height)

    def _ensure_extractor(self, w, h):
        if self.extractor is not None:
            return True
        if self.K is None:
            return False
        T = None
        if self.use_tf_extrinsics:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
                q = t.transform.rotation
                x, y, z, w_ = q.x, q.y, q.z, q.w
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w_), 2 * (x * z + y * w_)],
                              [2 * (x * y + z * w_), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w_)],
                              [2 * (x * z - y * w_), 2 * (y * z + x * w_), 1 - 2 * (x * x + y * y)]])
                T = np.eye(4)
                T[:3, :3] = R
                tr = t.transform.translation
                T[:3, 3] = (tr.x, tr.y, tr.z)
            except Exception:  # noqa: BLE001
                return False
        if T is None:
            T = optical_extrinsic_from_params(*self.cam_params)
        proj = GroundProjector(self.K, T, w, h, self.ground_z, tuple(self.x_range), self.y_max)
        if self.detector == 'yolo':
            self.extractor = YoloLaneExtractor(proj, self.hsv, self.yolo_model, self.yolo_device, self.yolo_conf)
            prov = self.extractor.model.session.get_providers() if self.extractor.onnx else 'torch'
            self.get_logger().info(f'YOLO 차선 검출기: {self.yolo_model} ({prov})')
        else:
            self.extractor = LaneExtractor(proj, self.hsv)
        self.get_logger().info(f'IPM 준비 완료 (ROI 시작 행 {proj.v0}, 검출기 {self.detector})')
        return True

    # ------------------------------------------------------------------ 공통 처리
    def _odom_at(self, stamp):
        return lookup_pose(self.tf_buffer, self.odom_frame, self.base_frame, Time.from_msg(stamp), timeout=0.05)

    def _advance(self, odom):
        """직전 처리 시점에서 odom 증분만큼 예측."""
        if self.last_odom is None:
            self.last_odom = odom
            return (0.0, 0.0, 0.0)
        d = relative(self.last_odom, odom)
        self.last_odom = odom
        if self.odom_noise[0] > 0 or self.odom_noise[1] > 0:
            tr = math.hypot(d[0], d[1])
            d = (d[0] * (1 + self.odom_noise[0]), d[1], d[2] + self.odom_noise[1] * tr + d[2] * self.odom_noise[1])
        if self.mode == 'mapping':
            self.pose = compose(self.pose, d)
        else:
            self.pf.predict(d)
        return d

    def _update_map_to_odom(self, odom):
        self.map_to_odom = compose(self.pose, inverse(odom))

    def in_lidar_region(self, pose):
        if self.dark >= self.lidar_dark_frames:
            return True
        if not self.regions:
            return not self.lidar_only_in_region
        return any(point_in_polygon(pose[:2], r) for r in self.regions)

    # ------------------------------------------------------------------ 카메라
    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None or not self._ensure_extractor(img.shape[1], img.shape[0]):
            return
        odom = self._odom_at(msg.header.stamp)
        if odom is None:
            return
        self._advance(odom)
        pts, _ = self.extractor.extract(img)
        npts = sum(len(p) for p in pts.values())
        self.dark = self.dark + 1 if npts < self.min_points else 0
        info = {'npts': npts}
        if self.mode == 'mapping':
            kp = self.map.filter_known(self.pose, pts, self.known) if self.fields is not None else {}
            nk = sum(len(p) for p in kp.values())
            if self.fields is not None and nk >= self.min_points:
                new, best, base = self.matcher.match(self.fields, self.map.origin, self.map.res,
                                                     self.pose, kp)
                if best > 0.35:
                    self.pose = new
                info.update(score=round(best, 3), base=round(base, 3))
            self._insert(pts)
        else:
            if npts >= self.min_points:
                sc = score_lanes(self.fields, self.map.origin, self.map.res, self.pf.p, pts)
                self.pf.update(sc)
                info['score'] = round(self.pf.last_score, 3)
            self.pose = self.pf.estimate()
            info['spread'] = round(self.pf.spread(), 4)
        self._update_map_to_odom(odom)
        self.n_frames += 1
        self._publish_pose(msg.header.stamp)
        info['pose'] = [round(v, 4) for v in self.pose]
        self.last_status = info
        self.status_pub.publish(to_json_msg(info))

    def _insert(self, pts):
        lp = self.last_insert_pose
        if lp is not None and math.hypot(self.pose[0] - lp[0], self.pose[1] - lp[1]) < 0.02 \
                and abs(wrap(self.pose[2] - lp[2])) < 0.03:
            return
        world = {c: transform_points(self.pose, pts[c]) for c in CLASSES}
        fov = transform_points(self.pose, self.extractor.proj.fov_polygon)
        self.map.integrate(world, fov)
        self.last_insert_pose = self.pose
        # 우도장은 비용이 있으므로 일정 프레임마다 갱신
        if self.fields is None or self.n_frames % 5 == 0:
            self.fields = self.map.likelihood_fields(self.lf_sigma)
            self.known = self.map.known_mask()

    # ------------------------------------------------------------------ LiDAR(터널 보조)
    def on_scan(self, msg):
        if self.mode != 'localization':
            return
        if not self.in_lidar_region(self.pf.estimate()):
            return
        odom = self._odom_at(msg.header.stamp)
        if odom is None:
            return
        if self.scan_offset is None:
            p = lookup_pose(self.tf_buffer, self.base_frame, msg.header.frame_id)
            if p is None:
                return
            self.scan_offset = p
        self._advance(odom)
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, 2.5))
        if ok.sum() < 20:
            return
        pts = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))
        pts = transform_points(self.scan_offset, pts)[::2]
        sc = self.lidar_field.score(self.pf.p, pts)
        g = self.pf.gain
        self.pf.gain = self.lidar_gain
        self.pf.update(sc)
        self.pf.gain = g
        self.pose = self.pf.estimate()
        self._update_map_to_odom(odom)
        self._publish_pose(msg.header.stamp)

    # ------------------------------------------------------------------ 출력
    def _publish_pose(self, stamp):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.map_frame
        ps.pose.position.x, ps.pose.position.y = float(self.pose[0]), float(self.pose[1])
        q = quat_from_yaw(self.pose[2])
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        self.pose_pub.publish(ps)

    def publish_tf_timer(self):
        if not self.publish_tf or self.map_to_odom is None:
            return
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + Duration(seconds=0.1)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x, t.transform.translation.y = float(self.map_to_odom[0]), float(self.map_to_odom[1])
        q = quat_from_yaw(self.map_to_odom[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_pub.sendTransform(t)

    def publish_map(self):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.info.resolution = float(self.map.res)
        m.info.width, m.info.height = self.map.w, self.map.h
        m.info.origin.position.x, m.info.origin.position.y = float(self.map.origin[0]), float(self.map.origin[1])
        m.info.origin.orientation.w = 1.0
        m.data = self.map.to_occupancy().ravel().tolist()
        self.map_pub.publish(m)

    def srv_save(self, _req, res):
        if not self.map_prefix:
            res.success, res.message = False, 'map_prefix 미설정'
            return res
        self.map.save(self.map_prefix)
        res.success, res.message = True, f'saved {self.map_prefix}'
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = LaneSlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.mode == 'mapping' and node.map_prefix:
            node.map.save(node.map_prefix)
            print(f'[lane_slam] 맵 저장: {node.map_prefix}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
