"""차선맵 기반 자세 추정기.

- CorrelativeMatcher : 맵핑 모드. odom 예측 자세 주변을 격자 탐색해 차선 우도 합이 최대인 자세 선택
- ParticleFilter     : 위치추정 모드. odom 예측 + 차선(및 선택적으로 LiDAR) 우도로 가중치 갱신
"""
import math

import numpy as np
import yaml

from .grid_map import angle_mean, lookup

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def _score(fields, origin, res, poses, pts_by_class, class_weight):
    """poses (N,3) 각각에 대해 클래스별 차선점 평균 우도의 가중합 → (N,)."""
    n = len(poses)
    total = np.zeros(n)
    wsum = 0.0
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    for cls, pts in pts_by_class.items():
        if pts is None or len(pts) == 0 or cls not in fields:
            continue
        x = poses[:, 0:1] + c * pts[None, :, 0] - s * pts[None, :, 1]
        y = poses[:, 1:2] + s * pts[None, :, 0] + c * pts[None, :, 1]
        v = lookup(fields[cls], origin, res, np.stack((x, y), -1))
        w = class_weight.get(cls, 1.0) * len(pts)
        total += w * v.mean(axis=1)
        wsum += w
    return total / wsum if wsum > 0 else total


class CorrelativeMatcher:
    """odom 예측 자세 주변 격자 탐색. 점수 지형이 충분히 뾰족할 때만 소프트 가중평균으로 부분 보정."""

    def __init__(self, xy_range=0.04, xy_step=0.01, th_range=0.04, th_step=0.01,
                 sigma_xy=0.03, sigma_th=0.01, beta=200.0, gain=0.3, min_peak=0.01):
        self.dxy = np.arange(-xy_range, xy_range + 1e-9, xy_step)
        self.dth = np.arange(-th_range, th_range + 1e-9, th_step)
        self.sigma_xy, self.sigma_th = sigma_xy, sigma_th
        self.beta, self.gain, self.min_peak = beta, gain, min_peak

    def match(self, fields, origin, res, pose, pts_by_class, class_weight=None):
        """→ (보정 자세, 최고점수, 예측자세 점수). 보정이 불확실하면 예측 자세 그대로."""
        gx, gy, gt = np.meshgrid(self.dxy, self.dxy, self.dth, indexing='ij')
        gx, gy, gt = gx.ravel(), gy.ravel(), gt.ravel()
        cand = np.column_stack((pose[0] + gx, pose[1] + gy, pose[2] + gt))
        sc = _score(fields, origin, res, cand, pts_by_class, class_weight or {})
        # 예측(odom)에서 벗어날수록 페널티 → 직선 구간 종방향/한쪽 차선 회전 모호성 억제
        pen = 0.5 * ((gx ** 2 + gy ** 2) / self.sigma_xy ** 2 + gt ** 2 / self.sigma_th ** 2) * 0.01
        obj = sc - pen
        base = float(sc[len(sc) // 2])
        best = float(np.max(sc))
        if best - float(np.median(sc)) < self.min_peak:
            return tuple(pose), best, base
        w = np.exp(self.beta * (obj - obj.max()))
        w /= w.sum()
        d = (float(np.sum(w * gx)), float(np.sum(w * gy)), float(np.sum(w * gt)))
        g = self.gain
        return (pose[0] + g * d[0], pose[1] + g * d[1], pose[2] + g * d[2]), best, base


class ParticleFilter:

    def __init__(self, n=300, alpha=(0.08, 0.02, 0.08, 0.02), gain=25.0, seed=0):
        self.n = int(n)
        self.alpha = alpha       # (회전→회전, 이동→회전, 이동→이동, 회전→이동) 잡음 계수
        self.gain = gain         # 우도 → 가중치 지수 이득
        self.rng = np.random.default_rng(seed)
        self.p = np.zeros((self.n, 3))
        self.w = np.full(self.n, 1.0 / self.n)
        self.last_score = 0.0

    def init(self, pose, sigma=(0.03, 0.03, 0.03)):
        self.p = np.column_stack([self.rng.normal(pose[i], sigma[i], self.n) for i in range(3)])
        self.w[:] = 1.0 / self.n

    def predict(self, delta):
        """delta = 로봇 좌표계 odom 증분 (dx, dy, dth)."""
        dx, dy, dth = delta
        trans = math.hypot(dx, dy)
        a1, a2, a3, a4 = self.alpha
        sr = a1 * abs(dth) + a2 * trans + 1e-4
        st = a3 * trans + a4 * abs(dth) + 1e-4
        n = self.n
        ndx = dx + self.rng.normal(0, st, n)
        ndy = dy + self.rng.normal(0, st * 0.5, n)
        ndt = dth + self.rng.normal(0, sr, n)
        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        self.p[:, 0] += c * ndx - s * ndy
        self.p[:, 1] += s * ndx + c * ndy
        self.p[:, 2] = (self.p[:, 2] + ndt + math.pi) % (2 * math.pi) - math.pi

    def update(self, score):
        """score: 입자별 평균 우도(0~1)."""
        self.last_score = float(np.max(score)) if len(score) else 0.0
        lw = self.gain * (score - np.max(score))
        self.w *= np.exp(lw)
        s = self.w.sum()
        if s <= 0 or not np.isfinite(s):
            self.w[:] = 1.0 / self.n
        else:
            self.w /= s
        if 1.0 / np.sum(self.w ** 2) < self.n * 0.5:
            self.resample()

    def resample(self):
        pos = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.searchsorted(np.cumsum(self.w), pos)
        idx = np.minimum(idx, self.n - 1)
        self.p = self.p[idx].copy()
        self.w[:] = 1.0 / self.n

    def estimate(self):
        x = float(np.sum(self.w * self.p[:, 0]))
        y = float(np.sum(self.w * self.p[:, 1]))
        t = angle_mean(self.p[:, 2], self.w)
        return (x, y, t)

    def spread(self):
        mx, my, _ = self.estimate()
        return float(math.sqrt(np.sum(self.w * ((self.p[:, 0] - mx) ** 2 + (self.p[:, 1] - my) ** 2))))


class OccupancyField:
    """Nav2 형식(yaml+pgm) 점유맵 → LiDAR 우도장. 터널 구간 위치추정 보조용."""

    def __init__(self, yaml_path, sigma=0.05):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        import os
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        self.res = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], dtype=float)
        occ_th = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))
        p = (img.astype(np.float32) / 255.0) if negate else (255 - img.astype(np.float32)) / 255.0
        occ = np.flipud(p > occ_th)          # 행 0 = 최소 y
        d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * self.res
        self.field = np.exp(-(d * d) / (2 * sigma * sigma)).astype(np.float32)

    def score(self, poses, pts):
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        x = poses[:, 0:1] + c * pts[None, :, 0] - s * pts[None, :, 1]
        y = poses[:, 1:2] + s * pts[None, :, 0] + c * pts[None, :, 1]
        return lookup(self.field, self.origin, self.res, np.stack((x, y), -1)).mean(axis=1)


def score_lanes(fields, origin, res, poses, pts_by_class, class_weight=None):
    return _score(fields, origin, res, poses, pts_by_class, class_weight or {})
