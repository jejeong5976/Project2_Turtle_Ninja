"""정밀 접근 중 라이다로 박스 위치 재기 → 정차 목표를 박스 기준으로 옮김 (ROS 없음, numpy 만).

쓰는 곳: local_planner_node.py --box_track (dock_to_pose 노드). SLAM 지도 때와 시연 때 박스 자리가 다를 수 있음 → 접근하면서 고침.
받는 것: 목표 자세 (지도 정차점, 박스를 봄), 스캔 점 (목표와 같은 프레임 xy, 촬영 시각 TF 로 바꾼 것), 로봇 xy
주는 것: 고친 목표 자세 = 잰 박스 중심에서 box_dist 떨어져 박스 정면을 봄
방법 (스캔 하나마다):
  - 지금 박스 추정 기준 앞면 (로봇 쪽 면) 근처 창 안 점만 (면 수직 ±win, 면 방향 ±(반 변 + 6 cm))
  - 직선 맞춤 (PCA, 1.5 cm 넘는 점 빼고 한 번 더) → 면 방향 · 위치. 기울기 > max_yaw 면 버림
  - 옆 위치: 면 점 양 끝 폭이 한 변 − 3 cm 이상 (모서리 둘 다 보임) 일 때만 가운데로, 아니면 이전 추정 유지
  - 중심 = 면 가운데 − 반 변 × 면 법선, 지도 중심에서 max_shift 넘으면 버림, 최근 med_n 개 중앙값
  - 로봇이 목표 freeze_dist 안이면 안 고침 (면이 scan_merger 몸 상자 · range_min 안으로 들어가 안 보임)
"""
import math

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class BoxTracker:
    def __init__(self, goal_xy, goal_yaw, box_dist=0.2575, half=0.1025, max_shift=0.08, max_yaw_deg=15.0,
                 win=0.08, min_pts=8, freeze_dist=0.10, med_n=5):
        self.yaw0 = float(goal_yaw)
        self.dist, self.h = box_dist, half
        self.c0 = np.asarray(goal_xy, float) + box_dist * np.array([math.cos(goal_yaw), math.sin(goal_yaw)])   # 지도 박스 중심
        self.c, self.th = self.c0.copy(), 0.0           # 지금 추정 (중심, 지도 대비 회전)
        self.max_shift, self.max_yaw = max_shift, math.radians(max_yaw_deg)
        self.win, self.min_pts, self.freeze = win, min_pts, freeze_dist
        self.hist = []
        self.med_n = med_n
        self.n_ok = 0
        self.last_why = ''
        self.face = None

    def goal(self):
        y = self.yaw0 + self.th
        return self.c - self.dist * np.array([math.cos(y), math.sin(y)]), _wrap(y)

    def update(self, pts, robot_xy):
        """pts (N,2) 스캔 점, robot_xy (2,). 고쳤으면 True"""
        g, _ = self.goal()
        if np.linalg.norm(np.asarray(robot_xy) - g) < self.freeze:
            self.last_why = 'freeze'
            return False
        m = self.measure(np.asarray(pts, float), np.asarray(robot_xy, float))
        if m is None:
            return False
        self.hist = (self.hist + [m])[-self.med_n:]
        H = np.array(self.hist)
        self.c, self.th = np.median(H[:, :2], axis=0), float(np.median(H[:, 2]))
        self.n_ok += 1
        return True

    def measure(self, pts, robot_xy):
        y = self.yaw0 + self.th
        n = -np.array([math.cos(y), math.sin(y)])        # 박스 → 로봇 쪽 면 법선
        t = np.array([-n[1], n[0]])
        f = self.c + self.h * n                           # 지금 추정 앞면 가운데
        rel = pts - f
        a, b = rel @ n, rel @ t
        sel = (np.abs(a) < self.win) & (np.abs(b) < self.h + 0.06)
        P = pts[sel]
        if len(P) < self.min_pts:
            self.last_why = f'점 {len(P)}'
            return None
        for _ in range(2):                                # 직선 맞춤 → 1.5 cm 넘는 점 빼고 다시
            mu = P.mean(0)
            _, _, vt = np.linalg.svd(P - mu)
            d = vt[0]
            res = np.abs((P - mu) @ vt[1])
            keep = res < 0.015
            if keep.sum() < self.min_pts:
                self.last_why = '직선 안 맞음'
                return None
            P = P[keep]
        mu = P.mean(0)
        nn = np.array([-d[1], d[0]])
        if nn @ (robot_xy - mu) < 0:
            nn = -nn
        th = _wrap(math.atan2(-nn[1], -nn[0]) - self.yaw0)   # 면 법선 반대 = 로봇이 볼 방향
        if abs(th) > self.max_yaw:
            self.last_why = f'기울기 {math.degrees(th):.0f}°'
            return None
        s = (P - mu) @ d
        if s.max() - s.min() >= 2 * self.h - 0.03:        # 모서리 둘 다 보임 → 옆 위치도 잼
            s_mid = 0.5 * (s.max() + s.min())
        else:                                             # 한쪽만 → 옆은 이전 추정을 이 면에 투영
            s_mid = float((self.c - mu) @ d)
        c = mu + s_mid * d - self.h * nn
        self.face = (mu + s.min() * d, mu + s.max() * d)   # 맞춘 면 양 끝 (그림 · 디버그용)
        if np.linalg.norm(c - self.c0) > self.max_shift:
            self.last_why = f'지도에서 {np.linalg.norm(c - self.c0) * 100:.0f} cm'
            return None
        self.last_why = 'ok'
        return np.array([c[0], c[1], th])
