"""(시뮬레이션 전용) Gazebo 바닥 텍스처로 웨이포인트를 차선 중앙에 맞추는 도구.

텍스처(4096²)는 바닥 모델 yaw=-π 때문에 월드에서 180° 회전되어 있다.
world x = H - u*S/N,  world y = v*S/N - H   (H = S/2, S = 4×1.14537445 m)
"""
import math

import cv2
import numpy as np

from autorace_common.paths import resample, smooth

SIZE = 4 * 1.14537445


class CourseTexture:

    def __init__(self, path, res=0.005):
        tex = cv2.imread(path)
        if tex is None:
            raise FileNotFoundError(path)
        w = cv2.rotate(tex, cv2.ROTATE_180)
        self.res = res
        self.n = int(SIZE / res)
        self.img = cv2.resize(w, (self.n, self.n), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.lines = hsv[..., 2] > 100          # 흰/노랑/빨강 차선
        self.half = SIZE / 2

    def is_line(self, x, y):
        c = int((x + self.half) / self.res)
        r = int((self.half - y) / self.res)
        if 0 <= r < self.n and 0 <= c < self.n:
            return bool(self.lines[r, c])
        return False

    def center(self, points, step=0.02, search=0.40, width=(0.18, 0.70)):
        """폴리라인 각 점에서 법선 방향 좌/우 차선을 찾아 중앙으로 이동한 경로."""
        p = resample(np.asarray(points, float), step)
        d = np.gradient(p, axis=0)
        h = np.arctan2(d[:, 1], d[:, 0])
        out = p.copy()
        ok = np.zeros(len(p), bool)
        for i, (pt, hh) in enumerate(zip(p, h)):
            nx, ny = -math.sin(hh), math.cos(hh)
            hit = {}
            for side in (1, -1):
                for k in range(2, int(search / 0.005)):
                    t = side * k * 0.005
                    if self.is_line(pt[0] + nx * t, pt[1] + ny * t):
                        hit[side] = t
                        break
            if 1 in hit and -1 in hit and width[0] <= hit[1] - hit[-1] <= width[1]:
                m = (hit[1] + hit[-1]) / 2
                out[i] = pt + np.array([nx, ny]) * m
                ok[i] = True
        # 실패 지점은 원래 점 유지, 이후 스무딩
        out = smooth(out, window=11, iterations=3)
        out[0], out[-1] = p[0] if not ok[0] else out[0], p[-1] if not ok[-1] else out[-1]
        return out, ok

    def draw(self, paths, out_file, scale=1):
        v = self.img.copy()
        cols = [(255, 0, 255), (0, 255, 0), (255, 128, 0), (0, 128, 255), (255, 255, 0)]
        for k, pts in enumerate(paths):
            px = np.column_stack(((np.asarray(pts)[:, 0] + self.half) / self.res,
                                  (self.half - np.asarray(pts)[:, 1]) / self.res)).astype(np.int32)
            cv2.polylines(v, [px], False, cols[k % len(cols)], 1)
        cv2.imwrite(out_file, v)
