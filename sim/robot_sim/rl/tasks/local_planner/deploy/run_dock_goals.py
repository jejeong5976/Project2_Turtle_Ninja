#!/usr/bin/env python3
"""Isaac 검증: planning Executor 의 박스 · 장소 이동 두 단계를 그대로 흉내 냄 (주행 정책 + 정밀 접근 정책).
정차점마다 (레포 maps/dice_map.yaml, planning 과 같은 파일):
  1 단계 navigate_to_pose 목표 = 정차점 뒤 NAV_BACK (0.025 m = 박스 면 2 cm, Executor dock_nav_back_m)
  위치 (TF map → base_footprint, Executor 와 같음) 가 정차점 정면 통로 (뒤 HANDOFF 0.40 m · 옆 HANDOFF_LAT 0.15 m · 방향 HANDOFF_YAW 60°,
  Executor dock_handoff_m · _lat_m · _yaw_deg) 안 → 1 단계 취소 → 취소 결과 뒤
  2 단계 dock_to_pose 목표 = 정차점. 1 단계가 넘김 전에 끝나면 (성공) 바로 2 단계.  START 등 박스 · 장소 아닌 곳은 1 단계만 (목표 = 정차점)
잼 (정답 자세 /sim/ground_truth): 총 시간 · 넘김 시각 · 넘김 때 속도 · 2 단계 시간 · 최종 앞뒤 (+ = 박스 쪽) · 옆 · yaw 오차 ·
  박스 면 틈 (차체 ±0.145 ↔ 지도 박스, 시뮬 박스 = dice_map 자리) · 2 단계 중 최소 틈
순서 기본 B1 → L1 → B2 → B6 → L3 → B3 → B5 → B4 → L2 → START (환경변수 ORDER)
사람 시험 (환경변수 PERSON_LEG=<정차점 이름>): 그 구간에서 출발 뒤 PERSON_AFTER (1.0 m) 움직이면 지금 진행 방향 PERSON_AHEAD (0.6 m) 앞에
  '사람' 상자 (0.25 × 0.35 m, 브리지 /sim/person) 를 PERSON_HOLD (5 s) 세웠다 치움 → 사람과 최소 틈 · 멈춘 시간 · 그 뒤 계속 갔는지
사용: python3 run_dock_goals.py <결과.json>   (시스템 python3 + ROS, use_sim_time)
"""
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node

OX, OY = 2.202, 3.152                   # 경기장 → 시뮬 map
DICE_MAP = Path(__file__).resolve().parents[6] / 'maps' / 'dice_map.yaml'
_M = yaml.safe_load(DICE_MAP.read_text())
STOPS = {'START': (tuple(_M['start']), None)}
for _sec in ('boxes', 'places'):
    STOPS.update({k: (tuple(v['stop']), tuple(v['center'])) for k, v in _M[_sec].items()})
ORDER = os.environ.get('ORDER', 'B1,L1,B2,B6,L3,B3,B5,B4,L2,START').split(',')
NAV_BACK = float(os.environ.get('NAV_BACK', 0.025))
HANDOFF = float(os.environ.get('HANDOFF', 0.40))
HANDOFF_LAT = float(os.environ.get('HANDOFF_LAT', 0.15))
HANDOFF_YAW = math.radians(float(os.environ.get('HANDOFF_YAW', 60.0)))
BOX_HALF, BODY_HALF = 0.1025, 0.145
PERSON_LEG = os.environ.get('PERSON_LEG', '')
PERSON_AFTER = float(os.environ.get('PERSON_AFTER', 1.0))
PERSON_AHEAD = float(os.environ.get('PERSON_AHEAD', 0.6))
PERSON_HOLD = float(os.environ.get('PERSON_HOLD', 5.0))
PERSON_HALF = (0.125, 0.175)                                  # 브리지 사람 상자 0.25 × 0.35 (축 정렬)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def gap(pose, c, half=(BOX_HALF, BOX_HALF)):
    """차체 사각형 (±0.145) 둘레 점 ↔ 축 정렬 상자 (반 폭 half) 최소 부호 거리"""
    x, y, th = pose
    best = 1e9
    for i in range(15):
        t = -BODY_HALF + 2 * BODY_HALF * i / 14
        for px, py in ((BODY_HALF, t), (-BODY_HALF, t), (t, BODY_HALF), (t, -BODY_HALF)):
            wx, wy = x + math.cos(th) * px - math.sin(th) * py, y + math.sin(th) * px + math.cos(th) * py
            qx, qy = abs(wx - c[0]) - half[0], abs(wy - c[1]) - half[1]
            d = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
            best = min(best, d)
    return best



def in_handoff(p, sx, sy, syaw):
    """Executor::in_handoff_zone 과 같음: 정차점 뒤 0 ~ HANDOFF, 옆 HANDOFF_LAT 안, 방향 HANDOFF_YAW 안"""
    dx, dy = sx - p[0], sy - p[1]
    ahead = dx * math.cos(syaw) + dy * math.sin(syaw)
    lat = -dx * math.sin(syaw) + dy * math.cos(syaw)
    dyaw = (syaw - p[2] + math.pi) % (2 * math.pi) - math.pi
    return -0.02 < ahead < HANDOFF and abs(lat) < HANDOFF_LAT and abs(dyaw) < HANDOFF_YAW

class R(Node):
    def __init__(self):
        super().__init__('dock_goal_runner', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gt, self.v, self.steer = None, 0.0, None
        self.steer_names = ['steer_fl_joint', 'steer_fr_joint', 'steer_rl_joint', 'steer_rr_joint']
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_gt, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'v', math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock = ActionClient(self, NavigateToPose, 'dock_to_pose')
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.log = []
        self.person_pub = self.create_publisher(PointStamped, '/sim/person', 10)

    def person(self, x, y, show=True):
        m = PointStamped()
        m.header.frame_id = 'map'
        m.point.x, m.point.y, m.point.z = float(x), float(y), 0.0 if show else -1.0
        self.person_pub.publish(m)

    def on_js(self, m):
        idx = {nm: i for i, nm in enumerate(m.name)}
        if all(nm in idx for nm in self.steer_names):
            self.steer = [m.position[idx[nm]] for nm in self.steer_names]

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.log.append((self.now(), *self.gt, self.v, *(self.steer or [0.0] * 4)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_map(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def spin_for(self, s):
        t_end = time.time() + s
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, ac, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        for _ in range(30):
            fut = ac.send_goal_async(g)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if gh.accepted:
                return gh
            time.sleep(2.0)
        return None


def main():
    out = sys.argv[1]
    rclpy.init()
    n = R()
    while n.gt is None or not n.nav.wait_for_server(timeout_sec=0.5) or not n.dock.wait_for_server(timeout_sec=0.5) or n.pose_map() is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.spin_for(2.0)
    res = []
    for name in ORDER:
        (ax, ay, ayaw), center = STOPS[name]
        sx, sy = ax - OX, ay - OY
        docked = center is not None
        c = (center[0] - OX, center[1] - OY) if docked else None
        back = NAV_BACK if docked else 0.0
        nx, ny = sx - back * math.cos(ayaw), sy - back * math.sin(ayaw)
        i0 = len(n.log)
        t0 = n.now()
        gh = n.send(n.nav, nx, ny, ayaw)
        rf = gh.get_result_async()
        t_ho, v_ho, how, st1 = None, None, 'nav_done', None
        pev = dict(state='wait' if name == PERSON_LEG else 'off')
        while not rf.done() and n.now() - t0 < 150:
            rclpy.spin_once(n, timeout_sec=0.02)
            if pev['state'] == 'wait' and len(n.log) > i0 + 5:
                g0, g1 = n.log[i0], n.log[-1]
                if math.hypot(g1[1] - g0[1], g1[2] - g0[2]) > PERSON_AFTER and n.v > 0.05:
                    back = n.log[max(i0, len(n.log) - 10)]                 # 0.2 s 전 → 진행 방향
                    d = math.hypot(g1[1] - back[1], g1[2] - back[2])
                    if d > 1e-3:
                        ux, uy = (g1[1] - back[1]) / d, (g1[2] - back[2]) / d
                        pev.update(state='on', x=g1[1] + PERSON_AHEAD * ux, y=g1[2] + PERSON_AHEAD * uy, t=n.now(), v=n.v, k=len(n.log))
                        n.person(pev['x'], pev['y'])
            if pev['state'] == 'on' and n.now() - pev['t'] > PERSON_HOLD:
                n.person(0, 0, show=False)
                pev.update(state='done', k1=len(n.log))
            p = n.pose_map()
            if docked and p is not None and in_handoff(p, sx, sy, ayaw):
                t_ho, v_ho, how = n.now() - t0, n.v, 'handoff'
                gh.cancel_goal_async()
                tc = time.time()
                while not rf.done() and time.time() - tc < 5.0:     # 취소 결과 기다림 (Executor cancel_blocked 와 같음)
                    rclpy.spin_once(n, timeout_sec=0.02)
                break
        if rf.done():
            st1 = rf.result().status
        elif how != 'handoff':
            gh.cancel_goal_async()
            how = 'nav_timeout'
        st2, t_dock0 = None, None
        if docked and (how == 'handoff' or st1 == GoalStatus.STATUS_SUCCEEDED):
            if t_ho is None:
                t_ho, v_ho = n.now() - t0, n.v
            t_dock0 = n.now()
            gh2 = n.send(n.dock, sx, sy, ayaw)
            rf2 = gh2.get_result_async()
            rclpy.spin_until_future_complete(n, rf2, timeout_sec=60.0)
            st2 = rf2.result().status if rf2.done() else -1
            if not rf2.done():
                gh2.cancel_goal_async()
        if pev['state'] == 'on':
            n.person(0, 0, show=False)
            pev.update(state='done', k1=len(n.log))
        n.spin_for(0.5)
        t1 = n.now()
        L = [r for r in n.log[i0:] if r[0] >= t0]
        x, y, th = L[-1][1:4]
        row_steer_total = round(sum(abs(L[i + 1][j] - L[i][j]) for i in range(len(L) - 1) for j in range(5, 9)), 2)
        dx, dy = x - sx, y - sy
        row = dict(goal=name, nav_status=st1, how=how, dock_status=st2, total_s=round(t1 - t0, 2),
                   handoff_s=round(t_ho, 2) if t_ho is not None else None, v_handoff=round(v_ho, 3) if v_ho is not None else None,
                   dock_s=round(t1 - t_dock0, 2) if t_dock0 else None,
                   along_cm=round(100 * (math.cos(ayaw) * dx + math.sin(ayaw) * dy), 2),
                   lateral_cm=round(100 * (-math.sin(ayaw) * dx + math.cos(ayaw) * dy), 2),
                   yaw_deg=round(math.degrees(wrap(th - ayaw)), 2), steer_total_rad=row_steer_total)
        k3 = next((i for i, r_ in enumerate(L) if math.hypot(r_[1] - sx, r_[2] - sy) < 0.03), None)
        row['settle_s'] = round(t1 - L[k3][0], 2) if k3 is not None else None          # 3 cm 안 들어온 뒤 끝날 때까지
        if k3 is not None:                                                             # 그동안 yaw 흔들림 · 움직인 양
            Ls = L[k3:]
            row['settle_yaw_pp_deg'] = round(math.degrees(max(r_[3] for r_ in Ls) - min(r_[3] for r_ in Ls)), 2)
            row['settle_steer_rad'] = round(sum(abs(Ls[i + 1][j] - Ls[i][j]) for i in range(len(Ls) - 1) for j in range(5, 9)), 2)   # 네 바퀴 조향 이동 합
        if docked:
            row['gap_cm'] = round(100 * gap((x, y, th), c), 2)
            Ld = [r for r in L if t_dock0 and r[0] >= t_dock0]
            row['gap_min_dock_cm'] = round(100 * min(gap(r[1:4], c) for r in Ld), 2) if Ld else None
        if pev['state'] == 'done':                                   # 사람 있던 동안: 최소 틈 · 멈춘 시간 (몸 속도 < 0.02)
            Lp = n.log[pev['k']:pev['k1']]
            row['person'] = dict(v_at_show=round(pev['v'], 3), gap_min_cm=round(100 * min(gap(r[1:4], (pev['x'], pev['y']), PERSON_HALF) for r in Lp), 1),
                                 stopped_s=round(sum(Lp[i + 1][0] - Lp[i][0] for i in range(len(Lp) - 1) if Lp[i][4] < 0.02), 2),
                                 first_stop_s=next((round(r[0] - Lp[0][0], 2) for r in Lp if r[4] < 0.02), None))
        row['trace'] = L
        print(json.dumps({k: v for k, v in row.items() if k != 'trace'}, ensure_ascii=False), flush=True)
        res.append(row)
    json.dump(res, open(out, 'w'), indent=1)
    rclpy.shutdown()


main()
