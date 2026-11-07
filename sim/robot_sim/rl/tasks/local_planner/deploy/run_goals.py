#!/usr/bin/env python3
"""Isaac 검증: dice 정차 자세를 차례로 navigate_to_pose 로 보내고 정답 자세 (/sim/ground_truth) 로 잼.
목표마다: 총 시간, xy 10 cm 안 첫 시각 · 그때 yaw 오차, 0.5 m 남았을 때 yaw 오차, 끝 제자리 회전 시간 (|v| < 0.03 · |w| > 0.1), 최종 위치 · yaw 오차.
정차 자세 = 레포 maps/dice_map.yaml (planning 과 같은 파일). 순서 기본 B1 → L1 → B2 → B6 → L3 → B3 → B5 → B4 → L2 → START, 환경변수 ORDER=B1,START,... 로 바꿈.
MPPI (bt_navigator) 와 정책 노드 둘 다 navigate_to_pose 라 같은 스크립트로 비교함.
사용: python3 run_goals.py <결과.json>   (시스템 python3 + ROS, use_sim_time)
"""
import json, math, os, sys, time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

OX, OY = 2.202, 3.152                   # 경기장 → 시뮬 map (경기장 중심 원점)
DICE_MAP = Path(__file__).resolve().parents[6] / 'maps' / 'dice_map.yaml'    # 레포 maps/dice_map.yaml
_M = yaml.safe_load(DICE_MAP.read_text())
STOPS = {'START': tuple(_M['start'])}
for _sec in ('boxes', 'places'):
    STOPS.update({k: tuple(v['stop']) for k, v in _M[_sec].items()})
ORDER = ['B1', 'L1', 'B2', 'B6', 'L3', 'B3', 'B5', 'B4', 'L2', 'START']
if os.environ.get('ORDER'):
    ORDER = os.environ['ORDER'].split(',')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class R(Node):
    def __init__(self):
        super().__init__('yaw_goal_runner', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gt = None
        self.tw = (0.0, 0.0)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_gt, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'tw', (math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), m.twist.twist.angular.z)), 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.log = []

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.log.append((self.get_clock().now().nanoseconds * 1e-9, *self.gt, *self.tw))


def main():
    out = sys.argv[1]
    rclpy.init()
    n = R()
    while n.gt is None or not n.ac.wait_for_server(timeout_sec=0.5):
        rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(2.0)
    res = []
    for name in ORDER:
        ax, ay, ayaw = STOPS[name]
        gx, gy = ax - OX, ay - OY
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(ayaw / 2), math.cos(ayaw / 2)
        i0 = len(n.log)
        for _ in range(30):                  # Nav2 활성화 전이면 거절됨 → 다시 보냄
            fut = n.ac.send_goal_async(g)
            rclpy.spin_until_future_complete(n, fut)
            gh = fut.result()
            if gh.accepted:
                break
            time.sleep(2.0)
        t0 = n.get_clock().now().nanoseconds * 1e-9
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=150.0)
        t1 = n.get_clock().now().nanoseconds * 1e-9
        status = rf.result().status if rf.done() else -1
        if not rf.done():
            gh.cancel_goal_async()
        L = [r for r in n.log[i0:] if r[0] >= t0]
        d = [math.hypot(r[1] - gx, r[2] - gy) for r in L]
        e = [abs(wrap(r[3] - ayaw)) for r in L]
        k5 = next((i for i, v in enumerate(d) if v < 0.10), None)
        k50 = next((i for i, v in enumerate(d) if v < 0.5), None)
        spin = sum(0.02 for r in L if r[4] < 0.03 and abs(r[5]) > 0.1)
        row = dict(goal=name, status=status, total_s=round(t1 - t0, 2),
                   t_xy10cm_s=round(L[k5][0] - t0, 2) if k5 is not None else None,
                   yaw_err_at_xy_deg=round(math.degrees(e[k5]), 1) if k5 is not None else None,
                   yaw_err_at_0p5m_deg=round(math.degrees(e[k50]), 1) if k50 is not None else None,
                   tail_s=round(t1 - L[k5][0], 2) if k5 is not None else None, spin_s=round(spin, 2),
                   final_xy_cm=round(100 * d[-1], 1) if d else None, final_yaw_deg=round(math.degrees(e[-1]), 1) if e else None)
        row['trace'] = [r for r in L]
        print(json.dumps({k: v for k, v in row.items() if k != 'trace'}), flush=True)
        res.append(row)
    json.dump(res, open(out, 'w'), indent=1)
    rclpy.shutdown()


main()
