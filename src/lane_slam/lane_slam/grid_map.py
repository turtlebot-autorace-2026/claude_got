"""차선 격자맵: 클래스(흰/노랑)별 검출 횟수와 관측 횟수를 누적, 우도장(likelihood field) 제공.

저장 형식(prefix 기준)
  prefix.yaml        : 해상도, 원점, 크기, 클래스 목록
  prefix.npz         : hits(클래스별), seen 원시 누적값 → 맵 이어서 확장 가능
  prefix_preview.png : 사람이 확인하는 컬러 미리보기
"""
import math
import os

import cv2
import numpy as np
import yaml

CLASSES = ('white', 'yellow')


class LaneMap:

    def __init__(self, resolution=0.01, origin=(-3.2, -0.8), size=(5.6, 5.6)):
        self.res = float(resolution)
        self.origin = np.array(origin, dtype=float)
        self.w = int(round(size[0] / self.res))
        self.h = int(round(size[1] / self.res))
        self.hits = {c: np.zeros((self.h, self.w), np.float32) for c in CLASSES}
        self.seen = np.zeros((self.h, self.w), np.float32)
        self._lf = None

    # ------------------------------------------------------------------ 좌표
    def to_cell(self, pts):
        ij = np.floor((np.asarray(pts) - self.origin) / self.res).astype(np.int64)
        return ij[:, 0], ij[:, 1]       # col(x), row(y)

    def inside(self, cx, cy):
        return (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)

    # ------------------------------------------------------------------ 갱신
    def integrate(self, pts_by_class, fov_world):
        """월드(map) 좌표 차선점과 지면 시야 다각형으로 누적."""
        if len(fov_world) >= 3:
            poly = np.round((np.asarray(fov_world) - self.origin) / self.res).astype(np.int32)
            m = np.zeros((self.h, self.w), np.uint8)
            cv2.fillPoly(m, [poly], 1)
            self.seen += m
        for c in CLASSES:
            p = pts_by_class.get(c)
            if p is None or len(p) == 0:
                continue
            cx, cy = self.to_cell(p)
            ok = self.inside(cx, cy)
            np.add.at(self.hits[c], (cy[ok], cx[ok]), 1.0)
        self._lf = None

    def probability(self, c):
        return self.hits[c] / np.maximum(self.seen, 1.0)

    def occupied(self, c, p_min=0.25, hits_min=2.0):
        return (self.probability(c) >= p_min) & (self.hits[c] >= hits_min)

    def known_mask(self, min_seen=4.0, erode=4):
        """충분히 관측된 영역(경계에서 erode 셀 안쪽). 맵핑 매칭의 frontier 편향 방지용."""
        k = (self.seen >= min_seen).astype(np.uint8)
        if erode > 0:
            k = cv2.erode(k, np.ones((2 * erode + 1, 2 * erode + 1), np.uint8))
        return k.astype(bool)

    def filter_known(self, pose, pts_by_class, known):
        """예측 자세에서 관측된 영역에 떨어지는 점만 남김."""
        from autorace_common.geometry import transform_points
        out = {}
        for c, p in pts_by_class.items():
            if p is None or len(p) == 0:
                out[c] = p
                continue
            w = transform_points(pose, p)
            cx, cy = self.to_cell(w)
            ok = self.inside(cx, cy)
            ok[ok] = known[cy[ok], cx[ok]]
            out[c] = p[ok]
        return out

    def has_data(self):
        return any(float(self.hits[c].sum()) > 0 for c in CLASSES)

    # ------------------------------------------------------------------ 우도장
    def likelihood_fields(self, sigma=0.03):
        """클래스별 exp(-d²/2σ²) 우도장(값 0~1, float32)."""
        if self._lf is not None and self._lf[0] == sigma:
            return self._lf[1]
        out = {}
        for c in CLASSES:
            occ = self.occupied(c)
            if occ.any():
                d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * self.res
                out[c] = np.exp(-(d * d) / (2 * sigma * sigma)).astype(np.float32)
            else:
                out[c] = np.zeros((self.h, self.w), np.float32)
        self._lf = (sigma, out)
        return out

    # ------------------------------------------------------------------ 입출력
    def save(self, prefix):
        os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
        np.savez_compressed(prefix + '.npz', seen=self.seen, **{'hits_' + c: self.hits[c] for c in CLASSES})
        meta = {'resolution': self.res, 'origin': [float(self.origin[0]), float(self.origin[1])],
                'width': self.w, 'height': self.h, 'classes': list(CLASSES), 'frame_id': 'map'}
        with open(prefix + '.yaml', 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False)
        cv2.imwrite(prefix + '_preview.png', self.preview())

    @classmethod
    def load(cls, prefix):
        with open(prefix + '.yaml') as f:
            meta = yaml.safe_load(f)
        m = cls(meta['resolution'], meta['origin'],
                (meta['width'] * meta['resolution'], meta['height'] * meta['resolution']))
        data = np.load(prefix + '.npz')
        m.seen = data['seen'].astype(np.float32)
        for c in CLASSES:
            m.hits[c] = data['hits_' + c].astype(np.float32)
        return m

    def preview(self):
        """위가 +y인 컬러 영상(흰=흰차선, 노랑=노란차선, 회색=관측 영역)."""
        img = np.zeros((self.h, self.w, 3), np.uint8)
        img[self.seen > 0] = (40, 40, 40)
        img[self.occupied('white')] = (255, 255, 255)
        img[self.occupied('yellow')] = (0, 220, 255)
        return cv2.flip(img, 0)

    def to_occupancy(self):
        """RViz용 int8 점유격자 데이터(행=y)."""
        occ = np.full((self.h, self.w), -1, np.int8)
        occ[self.seen > 0] = 0
        occ[self.occupied('white') | self.occupied('yellow')] = 100
        return occ


def lookup(field, origin, res, xy):
    """우도장 조회(범위 밖은 0). xy: (...,2)."""
    cx = np.floor((xy[..., 0] - origin[0]) / res).astype(np.int64)
    cy = np.floor((xy[..., 1] - origin[1]) / res).astype(np.int64)
    h, w = field.shape
    ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    out = np.zeros(cx.shape, np.float32)
    out[ok] = field[cy[ok], cx[ok]]
    return out


def angle_mean(a, w):
    return math.atan2(float(np.sum(w * np.sin(a))), float(np.sum(w * np.cos(a))))
