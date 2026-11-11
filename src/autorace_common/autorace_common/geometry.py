"""2D(SE2) 기하 유틸리티. 모든 각도는 rad, 좌표는 m."""
import math

import numpy as np


def wrap(a):
    """각도를 [-pi, pi)로 정규화."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    """geometry_msgs Quaternion(또는 x,y,z,w 속성 객체) → yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    """yaw → (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def compose(a, b):
    """SE2 합성 a∘b. a, b = (x, y, th)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(at + bt))


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def relative(a, b):
    """a 좌표계에서 본 b (= a^-1 ∘ b)."""
    return compose(inverse(a), b)


def transform_points(pose, pts):
    """pose(x,y,th)로 Nx2 점들을 변환."""
    if len(pts) == 0:
        return np.zeros((0, 2))
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    p = np.asarray(pts, dtype=float)
    return np.column_stack((x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]))


def point_in_polygon(pt, poly):
    """레이캐스팅 방식 다각형 내부 판정. poly = [[x,y],...]."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xc = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            if x < xc:
                inside = not inside
        j = i
    return inside


def points_in_rect(pts, rect):
    """회전 사각형 내부 점 마스크. rect = dict(cx, cy, yaw, length, width)."""
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    c, s = math.cos(rect['yaw']), math.sin(rect['yaw'])
    d = np.asarray(pts) - np.array([rect['cx'], rect['cy']])
    lx = c * d[:, 0] + s * d[:, 1]
    ly = -s * d[:, 0] + c * d[:, 1]
    return (np.abs(lx) <= rect['length'] / 2.0) & (np.abs(ly) <= rect['width'] / 2.0)
