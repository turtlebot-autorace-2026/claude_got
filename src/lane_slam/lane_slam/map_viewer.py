"""차선맵 미리보기 + 경로(routes.yaml) 오버레이 PNG 생성 도구.

사용: ros2 run lane_slam lane_map_viewer <map_prefix> <out.png> [routes.yaml]
"""
import sys

import cv2
import numpy as np

from .grid_map import LaneMap


def render(map_prefix, routes_yaml=None, scale=2):
    m = LaneMap.load(map_prefix)
    img = cv2.flip(m.preview(), 0)          # 행 = y 증가 방향으로 되돌려 그린 뒤 마지막에 뒤집음
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    colors = [(255, 0, 255), (0, 255, 0), (255, 128, 0), (0, 128, 255), (255, 255, 0), (128, 0, 255)]
    if routes_yaml:
        from autorace_common.paths import load_routes
        for k, (name, path) in enumerate(load_routes(routes_yaml).items()):
            c = colors[k % len(colors)]
            px = ((path.pts - m.origin) / m.res * scale).astype(np.int32)
            cv2.polylines(img, [px], False, c, 1)
            cv2.circle(img, tuple(px[-1]), 3, c, -1)
            img_flip_label = (int(px[len(px) // 2, 0]), int(px[len(px) // 2, 1]))
            cv2.circle(img, img_flip_label, 2, c, -1)
    img = cv2.flip(img, 0)
    # 1 m 격자
    for gx in np.arange(np.ceil(m.origin[0]), m.origin[0] + m.w * m.res, 1.0):
        c = int((gx - m.origin[0]) / m.res * scale)
        cv2.line(img, (c, 0), (c, img.shape[0]), (60, 60, 60), 1)
    for gy in np.arange(np.ceil(m.origin[1]), m.origin[1] + m.h * m.res, 1.0):
        r = img.shape[0] - int((gy - m.origin[1]) / m.res * scale)
        cv2.line(img, (0, r), (img.shape[1], r), (60, 60, 60), 1)
    return img


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    img = render(sys.argv[1], sys.argv[3] if len(sys.argv) > 3 else None)
    cv2.imwrite(sys.argv[2], img)
    print('saved', sys.argv[2])


if __name__ == '__main__':
    main()
