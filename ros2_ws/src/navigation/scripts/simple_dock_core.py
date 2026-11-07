"""규칙으로 하는 정밀 접근 (박스 · 장소 정차) — ROS 없음, numpy 만. simple_dock_node.py 가 씀, test/test_simple_dock.py 가 시험.

쓰는 곳: nav2.launch.py dock:=simple (팀 RL dock3 대신, 11-07 실물: dock3 이 박스 앞에서 옆 벽에 부딪힘)
받는 것: 목표 자세 (odom, 박스를 봄 · 박스 중심에서 box_dist), 로봇 자세 (odom), 합친 스캔 점 (odom), 라이다 원래 점 (몸 좌표)
주는 것: 몸 좌표 속도 (vx, vy, wz), 끝 · 실패 판정
순서
  1. 돌기: 목표 방향과 align_deg 넘게 다르면 제자리 회전 (처음 한 번)
  2. 다가가기: 목표까지 곧게 (몸 좌표 vx · vy, 최고 v_max, 남은 거리에 비례해 느려짐) · 방향은 조금씩만 (w_fine)
  3. 끝: 위치 tol_pos · 방향 tol_yaw 안. 또는 tol_ok · tol_yaw_ok 안에서 stall_s 동안 1 mm 도 안 줄면 (바퀴가 아주 작은 명령을 못 따라갈 때)
박스 맞춤: box_tracker.BoxTracker (지도 박스 자리 ±8 cm · ±15° 안의 앞면만, 목표 10 cm 안에서는 고정)
안전 (매 주기, 라이다 원래 점 = scan_merger 몸 상자 (±0.17 × ±0.19) 에 안 지워진 점):
  - 가까운 점 = 몸 사각형 (±half_x · ±half_y) 바깥 띠 안, 박스 (+box_excl) 아닌 점. 몸 안쪽 (±self_in, 바퀴 · 기둥) 은 안 셈
  - 명령대로 움직이면 가까운 점 쪽으로 다가가는 몸 부분 (가장 가까운 몸 점의 속도 · 회전 포함) 이 있고, 그 점까지 틈이
    margin + 다가가는 속도 × t_react 보다 작으면 그 명령은 안 씀 (빠를수록 일찍 멈춤: 0.15 m/s 면 2 + 3.75 cm)
    돌기가 막힘 → 그 점에서 v_esc 로 떨어진 뒤 다시 돌기 · 다가가기가 막힘 → 회전 빼고, 그래도 막히면 그 점 쪽 성분 빼고 미끄러지듯
    다 막히면 멈추고 실패 (이유 = 막은 점 자리)
  - 실물 11-07 B1 앞 (팔 접힘): 몸 사각형 바깥 점은 앞 가운데 x 0.165–0.17 (박스 면 추정) 뿐, 0.17 넘는 자기 점 없음
"""
import math

import numpy as np

from box_tracker import BoxTracker


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DockCore:
    def __init__(self, goal_xy, goal_yaw, box_track=True, box_dist=0.2575, box_half=0.1025, v_max=0.15, v_min=0.012,
                 k_v=1.0, w_max=0.3, w_min=0.06, w_fine=0.1, k_w=1.0, align_deg=6.0, tol_pos=0.005, tol_yaw_deg=2.0,
                 tol_ok=0.015, tol_yaw_ok_deg=3.0, stall_s=1.5,
                 half_x=0.16, half_y=0.16, margin=0.02, self_in=0.165, box_excl=0.03, min_hits=3, v_esc=0.02,
                 toward_eps=0.002, t_react=0.25):
        self.g_xy, self.g_yaw = np.asarray(goal_xy, float), float(goal_yaw)
        self.box_dist = box_dist
        self.tr = BoxTracker(goal_xy, goal_yaw, box_dist=box_dist, half=box_half) if box_track else None
        self.box_half, self.box_excl = box_half, box_excl
        self.v_max, self.v_min, self.k_v = v_max, v_min, k_v
        self.w_max, self.w_min, self.w_fine, self.k_w = w_max, w_min, w_fine, k_w
        self.align = math.radians(align_deg)
        self.tol_pos, self.tol_yaw = tol_pos, math.radians(tol_yaw_deg)
        self.tol_ok, self.tol_yaw_ok, self.stall_s = tol_ok, math.radians(tol_yaw_ok_deg), stall_s
        self.best, self.t_best = float('inf'), 0.0
        self.hx, self.hy, self.m, self.self_in = half_x, half_y, margin, self_in
        self.min_hits, self.v_esc, self.eps, self.t_react = min_hits, v_esc, toward_eps, t_react
        # 띠 너비: 가장 빠른 몸 점 (병진 + 회전 × 모서리 반지름) 이 t_react 동안 가는 거리까지
        self.band = margin + t_react * (v_max + w_max * math.hypot(half_x, half_y))
        self.aligned = False
        self.mode = ''      # 지난 주기 명령 종류 (로그)
        self.why = ''

    # ---------------------------------------------------------------- 목표 · 박스
    def target(self):
        if self.tr is None:
            return self.g_xy.copy(), self.g_yaw
        xy, yaw = self.tr.goal()
        return np.asarray(xy, float), float(yaw)

    def box_pose(self):
        """지금 박스 추정 (중심 xy, 로봇이 볼 방향) — 안전 검사에서 박스 점을 뺄 때"""
        if self.tr is None:
            return self.g_xy + self.box_dist * np.array([math.cos(self.g_yaw), math.sin(self.g_yaw)]), self.g_yaw
        return self.tr.c.copy(), wrap(self.tr.yaw0 + self.tr.th)

    def update_box(self, pts_odom, robot_xy):
        if self.tr is not None and len(pts_odom):
            self.tr.update(np.asarray(pts_odom, float), np.asarray(robot_xy, float))

    # ---------------------------------------------------------------- 안전
    def near_points(self, raw_body, pose):
        """몸 둘레 band 띠 안 · 박스 아닌 점 (몸 좌표)"""
        P = np.asarray(raw_body, float).reshape(-1, 2)
        if len(P) == 0:
            return P
        P = P[~((np.abs(P[:, 0]) < self.self_in) & (np.abs(P[:, 1]) < self.self_in))]
        P = P[(np.abs(P[:, 0]) < self.hx + self.band) & (np.abs(P[:, 1]) < self.hy + self.band)]
        if len(P) == 0:
            return P
        c, byaw = self.box_pose()
        x, y, yaw = pose
        cr, sr = math.cos(yaw), math.sin(yaw)
        cb = np.array([cr * (c[0] - x) + sr * (c[1] - y), -sr * (c[0] - x) + cr * (c[1] - y)])
        a = byaw - yaw
        q = (P - cb) @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])   # 박스 좌표
        lim = self.box_half + self.box_excl
        return P[~((np.abs(q[:, 0]) < lim) & (np.abs(q[:, 1]) < lim))]

    def blockers(self, near, cmd):
        """cmd 로 움직이면 다가가는 가까운 점들: 몸에서 가장 가까운 점의 속도 · 점 쪽 방향 내적 (다가가는 속도) > eps 이고
        틈 < margin + 다가가는 속도 × t_react"""
        if len(near) == 0:
            return near
        vx, vy, w = cmd
        b = np.c_[np.clip(near[:, 0], -self.hx, self.hx), np.clip(near[:, 1], -self.hy, self.hy)]
        d = near - b
        n = np.hypot(d[:, 0], d[:, 1])
        d = d / np.maximum(n, 1e-6)[:, None]
        vb = np.c_[vx - w * b[:, 1], vy + w * b[:, 0]]
        closing = (vb * d).sum(1)
        return near[(closing > self.eps) & (n < self.m + closing * self.t_react)]

    def describe(self, P):
        p = P[np.argmin(np.maximum(np.abs(P[:, 0]) - self.hx, np.abs(P[:, 1]) - self.hy))]
        side = '앞' if p[0] > self.hx else '뒤' if p[0] < -self.hx else '왼쪽' if p[1] > 0 else '오른쪽'
        gap = max(abs(p[0]) - self.hx, abs(p[1]) - self.hy)
        return f'{side} 몸에서 {gap * 100:.1f} cm (몸 중심 기준 ({p[0]:+.3f}, {p[1]:+.3f}) m, 점 {len(P)})'

    def safe(self, near, cmd):
        return len(self.blockers(near, cmd)) < self.min_hits

    # ---------------------------------------------------------------- 제어
    def step(self, pose, raw_body, t=0.0):
        """pose (x, y, yaw) odom, t 시작부터 [s]. 돌려주는 것: (cmd (vx, vy, wz), 상태) 상태 = 'run' · 'done' · 'fail'"""
        x, y, yaw = pose
        gxy, gyaw = self.target()
        e = gxy - np.array([x, y])
        c, s = math.cos(yaw), math.sin(yaw)
        eb = np.array([c * e[0] + s * e[1], -s * e[0] + c * e[1]])     # 몸 좌표 남은 위치
        dist = float(np.hypot(*eb))
        dpsi = wrap(gyaw - yaw)
        if dist < self.tol_pos and abs(dpsi) < self.tol_yaw:
            self.why = f'도착 (남은 {dist * 100:.1f} cm · {math.degrees(dpsi):+.1f}°)'
            return (0.0, 0.0, 0.0), 'done'
        if dist < self.best - 0.001:
            self.best, self.t_best = dist, t
        if dist < self.tol_ok and abs(dpsi) < self.tol_yaw_ok and t - self.t_best > self.stall_s:
            self.why = f'도착 (더 안 줄어듦 {self.stall_s:.1f} s, 남은 {dist * 100:.1f} cm · {math.degrees(dpsi):+.1f}°)'
            return (0.0, 0.0, 0.0), 'done'
        near = self.near_points(raw_body, pose)
        if not self.aligned and abs(dpsi) > self.align:   # 1. 제자리 회전
            w = math.copysign(max(self.w_min, min(self.w_max, self.k_w * abs(dpsi))), dpsi)
            cmd = (0.0, 0.0, w)
            if self.safe(near, cmd):
                self.mode = '돌기'
                return cmd, 'run'
            blk = self.blockers(near, cmd)
            p = blk[np.argmin(np.hypot(blk[:, 0], blk[:, 1]))]
            b = np.clip(p, [-self.hx, -self.hy], [self.hx, self.hy])
            away = (b - p) / max(np.hypot(*(b - p)), 1e-6)
            esc = (float(away[0] * self.v_esc), float(away[1] * self.v_esc), 0.0)
            if self.safe(near, esc):
                self.mode = '돌기 막힘 → 떨어지기'
                return esc, 'run'
            self.why = '돌 자리 없음: ' + self.describe(blk)
            return (0.0, 0.0, 0.0), 'fail'
        self.aligned = True
        # 2. 다가가기
        v = min(self.v_max, self.k_v * dist)
        if dist > self.tol_pos:
            v = max(v, self.v_min)
            vb = eb / dist * v
        else:
            vb = np.zeros(2)
        w = max(-self.w_fine, min(self.w_fine, self.k_w * dpsi))
        if abs(dpsi) < self.tol_yaw:
            w = 0.0
        elif abs(w) < self.w_min / 2:
            w = math.copysign(self.w_min / 2, dpsi)
        cmd = (float(vb[0]), float(vb[1]), float(w))
        if self.safe(near, cmd):
            self.mode = '다가가기'
            return cmd, 'run'
        cmd0 = (cmd[0], cmd[1], 0.0)
        if np.hypot(cmd[0], cmd[1]) > 0 and self.safe(near, cmd0):
            self.mode = '다가가기 (회전 뺌)'
            return cmd0, 'run'
        blk = self.blockers(near, cmd0 if np.hypot(cmd[0], cmd[1]) > 0 else cmd)
        if len(blk) and np.hypot(cmd[0], cmd[1]) > 0:
            b = np.c_[np.clip(blk[:, 0], -self.hx, self.hx), np.clip(blk[:, 1], -self.hy, self.hy)]
            d = (blk - b).mean(0)
            d /= max(np.hypot(*d), 1e-6)
            v2 = np.array(cmd0[:2])
            v2 = v2 - max(0.0, float(v2 @ d)) * d
            slide = (float(v2[0]), float(v2[1]), 0.0)
            if np.hypot(*v2) > self.v_min / 2 and self.safe(near, slide):
                self.mode = '다가가기 (막힌 쪽 빼고 미끄러지기)'
                return slide, 'run'
        self.why = '막힘: ' + self.describe(blk if len(blk) else near)
        return (0.0, 0.0, 0.0), 'fail'
