"""경로 파일 편집 도구.

  ros2 run route_follower route_tool list   <routes.yaml>
  ros2 run route_follower route_tool shift  <routes.yaml> <앞구간> <뒷구간> <m>
        앞구간 끝의 <m>미터를 뒷구간 앞쪽으로 옮김(음수면 반대). 미션 시작 지점(예: 회피 활성화 시점)을 조정할 때 사용
  ros2 run route_follower route_tool set    <routes.yaml> <구간> <키> <값>      예) speed 0.15, max_offset 0.17
"""
import sys

import numpy as np
import yaml

from autorace_common.paths import Path, save_routes


def _load(fn):
    with open(fn) as f:
        return (yaml.safe_load(f) or {}).get('routes', {})


def main():
    a = sys.argv[1:]
    if len(a) < 2:
        print(__doc__)
        return
    cmd, fn = a[0], a[1]
    routes = _load(fn)
    if cmd == 'list':
        for k, r in routes.items():
            p = Path(r['points'])
            print(f"{k:24s} len {p.length:5.2f} m  " + ' '.join(f'{m}={v}' for m, v in r.items() if m != 'points'))
        return
    if cmd == 'shift':
        ra, rb, m = routes[a[2]], routes[a[3]], float(a[4])
        pa, pb = np.array(ra['points']), np.array(rb['points'])
        if m > 0:
            d = np.concatenate(([0], np.cumsum(np.linalg.norm(np.diff(pa, axis=0), axis=1))))
            cut = int(np.searchsorted(d, d[-1] - m))
            ra['points'], rb['points'] = pa[:cut + 1], np.vstack([pa[cut:-1], pb])
        else:
            d = np.concatenate(([0], np.cumsum(np.linalg.norm(np.diff(pb, axis=0), axis=1))))
            cut = int(np.searchsorted(d, -m))
            ra['points'], rb['points'] = np.vstack([pa, pb[1:cut + 1]]), pb[cut:]
    elif cmd == 'set':
        v = a[4]
        try:
            v = yaml.safe_load(v)
        except yaml.YAMLError:
            pass
        routes[a[2]][a[3]] = v
    else:
        print(__doc__)
        return
    save_routes(fn, routes)
    print('saved', fn)


if __name__ == '__main__':
    main()
