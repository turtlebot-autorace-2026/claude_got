"""카메라 영상 → 지면(base_footprint) 좌표의 차선 점(흰색/노란색).

1) 영상 픽셀 → 지면 좌표 LUT(IPM: 핀홀 역투영 + 지면 평면 교차)를 한 번 계산
2) HSV 임계값으로 흰색/노란색 차선 마스크
3) 마스크 픽셀을 LUT로 지면 좌표로 바꾸고 격자 다운샘플링
"""
import math

import cv2
import numpy as np


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def optical_extrinsic_from_params(x, y, z, pitch, yaw):
    """카메라 장착값(base 기준 위치, 아래로 숙인 pitch[+], yaw) → base←optical 4x4."""
    r_base_cam = rot_rpy(0.0, pitch, yaw)
    # camera_link(x전방,y왼쪽,z위) → optical(z전방,x오른쪽,y아래)
    r_cam_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    t = np.eye(4)
    t[:3, :3] = r_base_cam @ r_cam_opt
    t[:3, 3] = (x, y, z)
    return t


class GroundProjector:
    """픽셀(u,v) → 지면 좌표(x 전방, y 왼쪽) 룩업 테이블."""

    def __init__(self, K, T_base_opt, width, height, ground_z=0.0,
                 x_range=(0.12, 0.95), y_max=0.7):
        self.w, self.h = int(width), int(height)
        u, v = np.meshgrid(np.arange(self.w, dtype=float), np.arange(self.h, dtype=float))
        Kinv = np.linalg.inv(np.asarray(K, dtype=float).reshape(3, 3))
        rays = np.stack([u, v, np.ones_like(u)], -1) @ Kinv.T          # optical frame
        R, t = T_base_opt[:3, :3], T_base_opt[:3, 3]
        d = rays @ R.T                                                   # base frame 방향
        with np.errstate(divide='ignore', invalid='ignore'):
            lam = (ground_z - t[2]) / d[..., 2]
        gx = t[0] + lam * d[..., 0]
        gy = t[1] + lam * d[..., 1]
        valid = (d[..., 2] < -1e-6) & (lam > 0) & (gx >= x_range[0]) & (gx <= x_range[1]) & (np.abs(gy) <= y_max)
        self.gx = np.where(valid, gx, np.nan).astype(np.float32)
        self.gy = np.where(valid, gy, np.nan).astype(np.float32)
        self.valid = valid
        rows = np.nonzero(valid.any(axis=1))[0]
        self.v0 = int(rows.min()) if len(rows) else 0       # 지면 ROI가 시작되는 행
        self.fov_polygon = self._fov_polygon()

    def _fov_polygon(self):
        """지면에 투영된 유효 시야 경계(차선맵 '관측됨' 갱신용)."""
        m = self.valid.astype(np.uint8)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return np.zeros((0, 2))
        c = max(cnts, key=cv2.contourArea).reshape(-1, 2)
        c = c[:: max(1, len(c) // 60)]
        return np.column_stack((self.gx[c[:, 1], c[:, 0]], self.gy[c[:, 1], c[:, 0]]))

    def project(self, us, vs):
        return np.column_stack((self.gx[vs, us], self.gy[vs, us]))


class LaneExtractor:
    """HSV 기반 흰색/노란색 차선 점 추출기. 파라미터는 dict로 받아 실차 튜닝 가능."""

    def __init__(self, projector, params):
        self.proj = projector
        self.p = params
        self.stride = int(params.get('pixel_stride', 2))
        self.voxel = float(params.get('voxel', 0.015))

    def masks(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        p = self.p
        white = cv2.inRange(hsv, np.array([0, 0, p.get('white_v_min', 170)]),
                            np.array([180, p.get('white_s_max', 60), 255]))
        yellow = cv2.inRange(hsv, np.array([p.get('yellow_h_min', 18), p.get('yellow_s_min', 90),
                                            p.get('yellow_v_min', 90)]),
                             np.array([p.get('yellow_h_max', 38), 255, 255]))
        k = np.ones((3, 3), np.uint8)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, k)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
        return white, yellow

    def _points(self, mask):
        v0 = self.proj.v0
        sub = mask[v0::self.stride, ::self.stride]
        vs, us = np.nonzero(sub)
        vs = vs * self.stride + v0
        us = us * self.stride
        ok = self.proj.valid[vs, us]
        pts = self.proj.project(us[ok], vs[ok])
        return voxel_down(pts, self.voxel)

    def extract(self, bgr):
        if bgr.shape[1] != self.proj.w or bgr.shape[0] != self.proj.h:
            bgr = cv2.resize(bgr, (self.proj.w, self.proj.h))
        white, yellow = self.masks(bgr)
        return {'white': self._points(white), 'yellow': self._points(yellow)}, (white, yellow)


def voxel_down(pts, voxel):
    if len(pts) == 0:
        return np.zeros((0, 2))
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys[:, 0] * 100003 + keys[:, 1], return_index=True)
    return pts[idx]


# ---------------------------------------------------------------------------------------------
# YOLO 차선 분할(turtle_ws driving 패키지의 lane_yolo26n_seg 모델, 클래스 0=white, 1=yellow)
# 모델은 turtle_ws와 같은 BEV(원근변환) 영상으로 학습되어 있으므로 BEV에서 추론하고,
# 마스크 픽셀을 역원근변환으로 원본 픽셀로 되돌린 뒤 위의 IPM 룩업으로 지면 좌표를 얻는다.
# ---------------------------------------------------------------------------------------------
BEV_W, BEV_H = 640, 480
BEV_SRC = np.float32([[18, 451], [633, 477], [445, 157], [216, 153]])   # turtle_ws lane_estimator.py
BEV_DST = np.float32([[166, 474], [474, 474], [474, 122], [166, 122]])
BEV_M = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
BEV_M_INV = np.linalg.inv(BEV_M)
YOLO_CLASSES = {0: 'white', 1: 'yellow'}


class YoloLaneExtractor(LaneExtractor):
    """YOLO 분할 마스크 기반 차선 점 추출기. extract()의 입출력은 HSV 추출기와 같다."""

    def __init__(self, projector, params, model_path, device='', conf=0.4):
        super().__init__(projector, params)
        self.conf = float(conf)
        if model_path.endswith('.onnx'):
            from .yolo_onnx import YoloOnnx
            self.model = YoloOnnx(model_path, device)
            self.onnx = True
        else:
            from ultralytics import YOLO
            self.model = YOLO(model_path, task='segment')
            self.onnx = False
        self.device = device or None
        self.bev_stride = int(params.get('bev_stride', 3))
        # BEV 격자 픽셀 → 원본 픽셀 좌표를 미리 계산
        bu, bv = np.meshgrid(np.arange(0, BEV_W, self.bev_stride), np.arange(0, BEV_H, self.bev_stride))
        pts = np.stack([bu.ravel(), bv.ravel(), np.ones(bu.size)], 0).astype(np.float64)
        src = BEV_M_INV @ pts
        su = np.round(src[0] / src[2]).astype(np.int64)
        sv = np.round(src[1] / src[2]).astype(np.int64)
        ok = (su >= 0) & (su < projector.w) & (sv >= 0) & (sv < projector.h)
        ok[ok] = projector.valid[sv[ok], su[ok]]
        self.grid_bev = (bv.ravel()[ok], bu.ravel()[ok])     # (row, col) in BEV
        self.grid_src = (sv[ok], su[ok])                     # (row, col) in original image
        self.last_ms = 0.0

    def infer(self, bev):
        if self.onnx:
            r = self.model.predict(bev, conf=self.conf, retina_masks=True)
            return [(int(c), m) for c, m in zip(r['classes'], r['masks'])]
        res = self.model.predict(bev, imgsz=640, conf=self.conf, device=self.device,
                                 retina_masks=True, verbose=False)[0]
        if res.masks is None:
            return []
        masks = res.masks.data.cpu().numpy() > 0.5
        return list(zip(res.boxes.cls.cpu().numpy().astype(int), masks))

    def extract(self, bgr):
        import time
        t0 = time.time()
        if bgr.shape[1] != self.proj.w or bgr.shape[0] != self.proj.h:
            bgr = cv2.resize(bgr, (self.proj.w, self.proj.h))
        bev = cv2.warpPerspective(bgr, BEV_M, (BEV_W, BEV_H))
        merged = {'white': np.zeros((BEV_H, BEV_W), bool), 'yellow': np.zeros((BEV_H, BEV_W), bool)}
        for c, m in self.infer(bev):
            name = YOLO_CLASSES.get(int(c))
            if name is not None:
                if m.shape != (BEV_H, BEV_W):
                    m = cv2.resize(m.astype(np.uint8), (BEV_W, BEV_H), interpolation=cv2.INTER_NEAREST) > 0
                merged[name] |= m
        out = {}
        rows, cols = self.grid_bev
        sr, sc = self.grid_src
        for name, m in merged.items():
            sel = m[rows, cols]
            pts = self.proj.project(sc[sel], sr[sel])
            out[name] = voxel_down(pts, self.voxel)
        self.last_ms = (time.time() - t0) * 1000.0
        return out, merged
