"""경로(폴리라인) 자료구조와 경로 파일(routes.yaml) 입출력.

routes.yaml 형식::

    frame_id: map
    routes:
      start_to_fork:
        speed: 0.22          # 기본 주행속도 [m/s]
        reverse: false       # true면 후진 추종
        max_offset: 0.0      # 장애물 회피 시 허용 횡오프셋 [m]
        points: [[x, y], ...]
"""
import math

import numpy as np
import yaml


class Path:
    """등간격 재샘플링된 2D 경로. s(누적거리), 진행방향 heading을 함께 보관."""

    def __init__(self, points, step=0.02, name='', meta=None):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        self.name = name
        self.meta = dict(meta or {})
        self.pts = resample(pts, step) if len(pts) >= 2 else pts
        d = np.linalg.norm(np.diff(self.pts, axis=0), axis=1) if len(self.pts) > 1 else np.zeros(0)
        self.s = np.concatenate(([0.0], np.cumsum(d)))
        self.length = float(self.s[-1]) if len(self.s) else 0.0
        if len(self.pts) > 1:
            diff = np.gradient(self.pts, axis=0)
            self.heading = np.arctan2(diff[:, 1], diff[:, 0])
        else:
            self.heading = np.zeros(len(self.pts))

    def __len__(self):
        return len(self.pts)

    def project(self, xy, hint=None, window=0.6):
        """xy를 경로에 투영 → (index, s, 횡거리 부호(+왼쪽)).

        hint(이전 s)가 있으면 그 주변 window[m] 안에서만 찾아 교차/중첩 구간 오인식을 막는다.
        """
        p = np.asarray(xy, dtype=float)
        if hint is None:
            idx = np.arange(len(self.pts))
        else:
            idx = np.nonzero((self.s >= hint - window) & (self.s <= hint + window))[0]
            if len(idx) == 0:
                idx = np.arange(len(self.pts))
        d2 = np.sum((self.pts[idx] - p) ** 2, axis=1)
        i = int(idx[np.argmin(d2)])
        h = self.heading[i]
        v = p - self.pts[i]
        lat = -math.sin(h) * v[0] + math.cos(h) * v[1]
        return i, float(self.s[i]), float(lat)

    def at(self, s):
        """누적거리 s 지점의 (x, y, heading)."""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.s, s))
        i = min(max(i, 1), len(self.pts) - 1)
        s0, s1 = self.s[i - 1], self.s[i]
        r = 0.0 if s1 - s0 < 1e-9 else (s - s0) / (s1 - s0)
        xy = self.pts[i - 1] * (1 - r) + self.pts[i] * r
        return float(xy[0]), float(xy[1]), float(self.heading[i])

    def offset_points(self, offsets):
        """각 점마다 왼쪽(+) 법선 방향으로 offsets[i]만큼 이동한 점열."""
        n = np.column_stack((-np.sin(self.heading), np.cos(self.heading)))
        return self.pts + n * np.asarray(offsets).reshape(-1, 1)

    def end_pose(self):
        return (float(self.pts[-1, 0]), float(self.pts[-1, 1]), float(self.heading[-1]))

    def start_pose(self):
        return (float(self.pts[0, 0]), float(self.pts[0, 1]), float(self.heading[0]))


def resample(pts, step):
    """폴리라인을 step 간격으로 재샘플링."""
    pts = np.asarray(pts, dtype=float)
    keep = [0]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-6:
            keep.append(i)
    pts = pts[keep]
    if len(pts) < 2:
        return pts
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(d)))
    n = max(int(math.ceil(s[-1] / step)), 1)
    ss = np.linspace(0.0, s[-1], n + 1)
    return np.column_stack((np.interp(ss, s, pts[:, 0]), np.interp(ss, s, pts[:, 1])))


def smooth(pts, window=7, iterations=2):
    """끝점을 고정한 이동평균 스무딩."""
    p = np.asarray(pts, dtype=float).copy()
    if len(p) < window + 2:
        return p
    k = np.ones(window) / window
    for _ in range(iterations):
        pad = window // 2
        ext = np.vstack([np.repeat(p[:1], pad, 0), p, np.repeat(p[-1:], pad, 0)])
        q = np.column_stack([np.convolve(ext[:, j], k, mode='valid') for j in range(2)])
        q[0], q[-1] = p[0], p[-1]
        p = q
    return p


def load_routes(path):
    """routes.yaml → {name: Path}."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for name, r in (data.get('routes') or {}).items():
        meta = {k: v for k, v in r.items() if k != 'points'}
        out[name] = Path(r['points'], name=name, meta=meta)
    return out


def save_routes(path, routes, frame_id='map'):
    """{name: {'points': Nx2, ...meta}} → routes.yaml."""
    data = {'frame_id': frame_id, 'routes': {}}
    for name, r in routes.items():
        item = {k: v for k, v in r.items() if k != 'points'}
        item['points'] = [[round(float(x), 4), round(float(y), 4)] for x, y in np.asarray(r['points'])]
        data['routes'][name] = item
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False, width=200)
