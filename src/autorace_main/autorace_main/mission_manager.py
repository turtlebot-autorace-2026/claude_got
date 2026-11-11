"""미션 매니저(메인): 전체 주행 순서를 관리하는 상태머신.

각 미션은 제너레이터(코루틴)로 작성되어 있고 10Hz tick마다 한 단계씩 진행한다.
  1 traffic_light  : 초록불 '전환'을 보면 출발
  2 intersection   : 이정표 방향 투표 → fork_left / fork_right 경로 선택
  3 construction   : LiDAR 회피 노드 활성화 상태로 공사구간 경로 추종
  4 parking        : LiDAR로 빈칸 판별 → 진입·정지·후진 탈출·회전·복귀
  5 level_crossing : 정지선에서 차단바가 내려왔다 올라가면(또는 내려오지 않으면) 통과
  6 tunnel         : 입구까지 경로 추종 → Nav2로 터널 통과 → 출구 경로 추종
주행 자체는 route_follower(차선맵 위치추정 기반 기록 경로 추종)와 Nav2가 하고, 매니저는 명령만 보낸다.
상태: /mission/state (String JSON)
"""
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from autorace_common.paths import load_routes
from autorace_common.ros_utils import LATCHED, from_json_msg, lookup_pose, to_json_msg

from .paths import data_dir


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        P = self.declare_parameter
        self.routes_file = os.path.expanduser(P('routes_file', os.path.join(data_dir(), 'routes', 'routes.yaml')).value)
        cfg_file = P('mission_config', '').value
        self.cfg = {}
        if cfg_file and os.path.exists(cfg_file):
            with open(cfg_file) as f:
                self.cfg = (yaml.safe_load(f) or {}).get('mission', {})
        self.start_mission = P('start_mission', self.cfg.get('start_mission', 'traffic_light')).value
        self.speed_scale = P('speed_scale', float(self.cfg.get('speed_scale', 1.0))).value
        self.routes = load_routes(self.routes_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        pub = self.create_publisher
        self.route_cmd = pub(String, '/route_follower/command', 10)
        self.mux = pub(String, '/cmd_mux/select', 10)
        self.state_pub = pub(String, '/mission/state', LATCHED)
        self.inter_enable = pub(Bool, '/intersection/enable', 10)
        self.avoid_enable = pub(Bool, '/construction/enable', 10)
        self.park_enable = pub(Bool, '/parking/enable', 10)
        self.tunnel_cmd = pub(String, '/tunnel/command', 10)
        self.light_enable = pub(Bool, '/traffic_light/enable', 10)
        self.crossing_enable = pub(Bool, '/level_crossing/enable', 10)

        self.light = 'none'
        self.direction = 'none'
        self.slot = None
        self.barrier = 'open'
        self.route_status = {}
        self.tunnel_status = {}
        self.lane_status = {}
        sub = self.create_subscription
        sub(String, '/traffic_light/state', lambda m: setattr(self, 'light', m.data), LATCHED)
        sub(String, '/intersection/direction', lambda m: setattr(self, 'direction', m.data), LATCHED)
        sub(String, '/parking/slot', self.on_slot, LATCHED)
        sub(String, '/level_crossing/state', lambda m: setattr(self, 'barrier', m.data), LATCHED)
        sub(String, '/route_follower/status', lambda m: setattr(self, 'route_status', from_json_msg(m) or {}), 10)
        sub(String, '/tunnel/status', lambda m: setattr(self, 'tunnel_status', from_json_msg(m) or {}), 10)
        sub(String, '/lane_slam/status', lambda m: setattr(self, 'lane_status', from_json_msg(m) or {}), 10)

        self.cmd_id = 0
        self.mission = 'init'
        self.detail = ''
        self.log = []
        self.t_start = None
        self.gen = self.run()
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------------ 도우미
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_slot(self, msg):
        d = from_json_msg(msg) or {}
        self.slot = d.get('slot')

    def set_state(self, mission, detail=''):
        if mission != self.mission:
            self.get_logger().info(f'=== 미션: {mission} ===')
            self.log.append((round(self.now() - (self.t_start or self.now()), 1), mission))
        self.mission, self.detail = mission, detail

    def select(self, src):
        self.mux.publish(String(data=src))

    def send_route(self, cmd):
        self.cmd_id += 1
        cmd['id'] = self.cmd_id
        self.route_cmd.publish(to_json_msg(cmd))
        return self.cmd_id

    def follow(self, name, speed=None, reverse=None, end_s=None, stop_if=None):
        """경로 추종 명령 후 완료까지 대기하는 제너레이터. stop_if()가 참이 되면 즉시 정지하고 끝낸다."""
        r = self.routes[name]
        sp = (speed or float(r.meta.get('speed', 0.2))) * self.speed_scale
        rev = bool(r.meta.get('reverse', False)) if reverse is None else reverse
        self.select('route')
        cmd = {'cmd': 'follow', 'route': name, 'speed': sp, 'reverse': rev}
        if end_s is not None:
            cmd['end_s'] = float(end_s)
        cid = self.send_route(dict(cmd))
        self.detail = name
        t0 = self.now()
        while True:
            st = self.route_status
            if st.get('id') == cid and st.get('state') == 'done':
                return
            if stop_if is not None and stop_if():
                self.stop()
                return
            if st.get('id') != cid and self.now() - t0 > 1.0:     # 명령 유실 시 재전송
                cid = self.send_route(dict(cmd))
                t0 = self.now()
            yield

    def rotate_to(self, yaw):
        self.select('route')
        cid = self.send_route({'cmd': 'rotate', 'yaw': yaw})
        while not (self.route_status.get('id') == cid and self.route_status.get('state') == 'done'):
            yield

    def stop(self):
        self.send_route({'cmd': 'stop'})
        self.select('stop')

    def wait(self, sec):
        t0 = self.now()
        while self.now() - t0 < sec:
            yield

    def pose(self):
        return lookup_pose(self.tf_buffer, 'map', 'base_footprint')

    # ------------------------------------------------------------------ 미션 시퀀스
    def run(self):
        self.set_state('init')
        self.stop()
        while self.pose() is None or not self.lane_status:
            self.detail = 'lane_slam 대기'
            yield
        yield from self.wait(1.0)
        order = ['traffic_light', 'intersection', 'construction', 'parking', 'level_crossing', 'tunnel', 'finish']
        for m in order[order.index(self.start_mission):]:
            yield from getattr(self, 'm_' + m)()

    def m_traffic_light(self):
        self.set_state('traffic_light', '초록불 전환 대기')
        self.inter_enable.publish(Bool(data=True))
        seen_not_green = False
        while True:
            if self.light in ('red', 'yellow'):
                seen_not_green = True
            if self.light == 'green' and seen_not_green:
                break
            self.detail = f'신호 {self.light}'
            yield
        self.t_start = self.now()
        self.light_enable.publish(Bool(data=False))
        self.get_logger().info('초록불 → 출발!')
        self.inter_enable.publish(Bool(data=True))     # 출발 후 표시되는 이정표 투표 초기화

    def m_intersection(self):
        self.set_state('intersection')
        yield from self.follow('start_to_fork')
        if self.direction not in ('left', 'right'):
            # 표지판 카메라 화각이 좁으므로 두 갈래 경로 앞부분의 평균 방향(= 이정표가 있는 갈림길 중앙)으로 회전
            p = self.pose()
            if p is not None:
                look = float(self.cfg.get('intersection_look_s', 0.45))
                a = self.routes['fork_left'].at(look)
                b = self.routes['fork_right'].at(look)
                yaw = math.atan2((a[1] + b[1]) / 2 - p[1], (a[0] + b[0]) / 2 - p[0])
                self.detail = '이정표 쪽으로 회전'
                yield from self.rotate_to(yaw)
        self.stop()
        t0 = self.now()
        while self.direction not in ('left', 'right'):
            self.detail = '이정표 판단 대기'
            if self.now() - t0 > float(self.cfg.get('intersection_timeout', 8.0)):
                self.get_logger().warn('이정표 판단 실패 → 기본값 right')
                self.direction = 'right'
            yield
        self.get_logger().info(f'갈림길: {self.direction}')
        yield from self.follow(f'fork_{self.direction}')
        self.inter_enable.publish(Bool(data=False))

    def m_construction(self):
        self.set_state('construction')
        yield from self.follow('merge_to_construction')
        self.avoid_enable.publish(Bool(data=True))
        yield from self.follow('construction')
        self.avoid_enable.publish(Bool(data=False))

    def m_parking(self):
        self.set_state('parking')
        yield from self.follow('to_parking')
        self.stop()
        self.slot = None
        self.park_enable.publish(Bool(data=True))
        t0 = self.now()
        while self.slot is None:
            if self.now() - t0 > 4.0:
                self.slot = 'left'
            yield
        self.get_logger().info(f'주차칸: {self.slot}')
        yield from self.follow(f'park_{self.slot}_in')
        self.stop()
        yield from self.wait(float(self.cfg.get('parking_hold', 1.0)))
        yield from self.follow(f'park_{self.slot}_out')
        yield from self.rotate_to(self.routes['parking_exit'].start_pose()[2])
        yield from self.follow('parking_exit')

    def m_level_crossing(self):
        self.set_state('level_crossing')
        self.crossing_enable.publish(Bool(data=True))
        yield from self.follow('to_crossing')
        self.stop()
        yield from self.wait(0.8)
        if self.barrier != 'closed':
            # 정지 위치가 1번 센서 앞일 수 있음(센서 위치 임의·위치추정 오차) → 천천히 다가가다
            # 차단바가 움직이기 시작하면 즉시 정지
            self.detail = '차단바 앞 서행 접근'
            init = self.barrier            # 시작부터 moving(오검출 가능)이면 '새로 움직임'만 정지 조건으로
            yield from self.follow('crossing_to_tunnel', speed=float(self.cfg.get('crossing_creep_speed', 0.05)),
                                   end_s=float(self.cfg.get('crossing_creep_dist', 0.12)),
                                   stop_if=lambda: self.barrier == 'closed' or (
                                       self.barrier == 'moving' and init != 'moving'))
            self.stop()
        t0 = self.now()
        closed_seen = False
        open_since = self.now()
        wait_open = float(self.cfg.get('crossing_wait_if_open', 2.0))
        open_after_close = float(self.cfg.get('crossing_open_confirm', 1.0))
        while True:
            if self.barrier != 'open':                  # closed 또는 moving(내려오는/올라가는 중)
                closed_seen = closed_seen or self.barrier == 'closed'
                open_since = self.now()
                self.detail = f'차단바 {self.barrier} → 대기'
            else:
                self.detail = '차단바 열림 확인 중'
                if closed_seen and self.now() - open_since > open_after_close:
                    break
                if not closed_seen and self.now() - open_since > wait_open:
                    break
            if self.now() - t0 > float(self.cfg.get('crossing_timeout', 20.0)):
                break
            yield
        self.get_logger().info(f'차단바 통과 (closed_seen={closed_seen})')
        self.crossing_enable.publish(Bool(data=False))
        yield from self.follow('crossing_to_tunnel')

    def m_tunnel(self):
        self.set_state('tunnel')
        yield from self.follow('tunnel_entry')
        use_nav2 = bool(self.cfg.get('tunnel_use_nav2', True))
        ok = False
        if use_nav2:
            self.send_route({'cmd': 'stop'})
            self.select('nav2')
            self.tunnel_cmd.publish(to_json_msg({'cmd': 'go'}))
            t0 = self.now()
            while True:
                st = self.tunnel_status.get('state')
                if st == 'succeeded':
                    ok = True
                    break
                if (st == 'failed' and self.now() - t0 > 1.0) or self.now() - t0 > float(self.cfg.get('tunnel_timeout', 60.0)):
                    self.get_logger().warn(f'Nav2 터널 실패({st}) → 기록 경로+회피로 대체')
                    self.tunnel_cmd.publish(to_json_msg({'cmd': 'cancel'}))
                    break
                self.detail = f'Nav2 {st}'
                yield
        if not ok:
            self.avoid_enable.publish(Bool(data=True))
            yield from self.follow('tunnel_inside')
            self.avoid_enable.publish(Bool(data=False))
        yield from self.follow('tunnel_exit')

    def m_finish(self):
        self.stop()
        el = self.now() - (self.t_start or self.now())
        self.set_state('finish', f'완주 {el:.1f}s')
        self.get_logger().info(f'완주! 출발 후 {el:.1f}s, 기록 {self.log}')
        while True:
            yield

    # ------------------------------------------------------------------ 루프
    def tick(self):
        try:
            next(self.gen)
        except StopIteration:
            pass
        el = None if self.t_start is None else round(self.now() - self.t_start, 1)
        self.state_pub.publish(to_json_msg({'mission': self.mission, 'detail': self.detail, 'elapsed': el,
                                            'light': self.light, 'direction': self.direction,
                                            'slot': self.slot, 'barrier': self.barrier}))


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, getattr(rclpy.executors, 'ExternalShutdownException', KeyboardInterrupt)):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
