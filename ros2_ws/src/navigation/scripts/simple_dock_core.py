"""규칙으로 하는 정밀 접근 (박스 · 장소 정차) — ROS 없음, numpy 만. simple_dock_node.py 가 씀, test/test_simple_dock.py 가 시험.

쓰는 곳: nav2.launch.py dock:=simple (팀 RL dock3 대신, 11-07 실물: dock3 이 박스 앞에서 옆 벽에 부딪힘)
받는 것: 목표 자세 (odom, 박스를 봄 · 박스 중심에서 box_dist), 로봇 자세 (odom), 합친 스캔 점 (odom), 라이다 원래 점 (몸 좌표)
주는 것: 몸 좌표 속도 (vx, vy, wz), 끝 · 실패 판정
순서
  1. 돌기: 목표 방향과 align_deg 넘게 다르면 제자리 회전 (처음 한 번)
  2. 다가가기: 목표까지 곧게 (몸 좌표 vx · vy, 최고 v_max, 남은 거리에 비례해 느려짐) · 방향은 조금씩만 (w_fine)
  3. 끝: 위치 tol_pos · 방향 tol_yaw 안. 또는 tol_ok · tol_yaw_ok 안에서 stall_s 동안 1 mm 도 안 줄면 (바퀴가 아주 작은 명령을 못 따라갈 때)
박스 맞춤 (FaceTrack, 스캔마다): 박스 앞면 = 직선 조각 (길이 0.12 m – 한 변 + 3 cm, 연속, 끝 밖으로 같은 직선이 안 이어짐 = 벽 아님)
  - 처음: 지도 박스 자리 ±search (0.25 m) · 방향 ±15° 안에서 박스 중심이 지도 자리에 가장 가까운 조각 (11-07 실물 arena5: 사람이 놓은
    박스 · 지도 각도 오차로 8–25 cm 어긋남, box_tracker 의 ±8 cm 로는 못 찾음)
  - 그 뒤: 지금 추정 ±track (6 cm) · 방향 ±8° 안 조각만 → 최근 5 개 중앙값. 박스 옆면 (90° 다름) 은 방향으로 걸러짐
    (box_tracker 의 한 직선 맞춤은 옆면 점이 섞이면 기울어 읽음, 시뮬: 22 cm 옆 박스에서 12° 틀어져 박스 모서리에 닿음)
  - 양 모서리가 다 보이면 (조각 ≥ 한 변 − 3 cm) 옆 위치도 고치고, 아니면 옆은 이전 추정 유지
  - 목표 freeze_dist (0.10 m) 안에서는 고정 (면이 몸 상자 안으로 들어가 안 보임)
안전 (매 주기, 라이다 원래 점 = scan_merger 몸 상자 (±0.17 × ±0.19) 에 안 지워진 점):
  - 가까운 점 = 몸 사각형 (±half_x · ±half_y) 바깥 띠 안, 박스 앞면 띠 (면 앞 box_excl ~ 면 뒤 face_depth, 옆 ±(한 변/2 + box_excl)) 아닌 점.
    몸 안쪽 (±self_in, 바퀴 · 기둥) 은 안 셈. 박스 옆면 · 뒤쪽은 막는 점으로 두고, 로봇 중심이 박스 앞면 정면 (옆 ±몸 반폭 0.16) 밖이면
    앞면 띠도 막는 점 (11-08 리뷰 · 시뮬: 박스 옆에서 넘어오면 예전 규칙은 몸이 박스 모서리를 7–10 cm 쓸고 감)
  - 틈은 원래 스캔 나이 (step 의 raw_age) 만큼 더 일찍: margin + 다가가는 속도 × (t_react + raw_age)
  - 명령대로 움직이면 가까운 점 쪽으로 다가가는 몸 부분 (가장 가까운 몸 점의 속도 · 회전 포함) 이 있고, 그 점까지 틈이
    margin + 다가가는 속도 × t_react 보다 작으면 그 명령은 안 씀 (빠를수록 일찍 멈춤: 0.15 m/s 면 2 + 3.75 cm)
    돌기가 막힘 → 그 점에서 v_esc 로 떨어진 뒤 다시 돌기 · 다가가기가 막힘 → 회전 빼고, 그래도 막히면 그 점 쪽 성분 빼고 미끄러지듯
    다 막히면 멈추고 실패 (이유 = 막은 점 자리)
  - 실물 11-07 B1 앞 (팔 접힘): 몸 사각형 바깥 점은 앞 가운데 x 0.165–0.17 (박스 면 추정) 뿐, 0.17 넘는 자기 점 없음
"""
import math

import numpy as np



def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DockCore:
    def __init__(self, goal_xy, goal_yaw, box_track=True, box_dist=0.2575, box_half=0.1025, v_max=0.15, v_min=0.012,
                 k_v=1.0, w_max=0.3, w_min=0.06, w_fine=0.1, k_w=1.0, align_deg=6.0, tol_pos=0.005, tol_yaw_deg=2.0,
                 tol_ok=0.015, tol_yaw_ok_deg=3.0, stall_s=1.5,
                 half_x=0.16, half_y=0.16, margin=0.02, self_in=0.165, box_excl=0.03, min_hits=3, v_esc=0.02,
                 toward_eps=0.002, t_react=0.25, search=0.25, face_depth=0.05):
        self.g_xy, self.g_yaw = np.asarray(goal_xy, float), float(goal_yaw)
        self.box_dist = box_dist
        self.tr = FaceTrack(goal_xy, goal_yaw, box_dist, box_half, search) if box_track else None
        self.box_half, self.box_excl, self.face_depth = box_half, box_excl, face_depth
        self.age = 0.0      # 이번 주기 원래 스캔 나이 [s] (step 이 받음)
        self.v_max, self.v_min, self.k_v = v_max, v_min, k_v
        self.w_max, self.w_min, self.w_fine, self.k_w = w_max, w_min, w_fine, k_w
        self.align = math.radians(align_deg)
        self.tol_pos, self.tol_yaw = tol_pos, math.radians(tol_yaw_deg)
        self.tol_ok, self.tol_yaw_ok, self.stall_s = tol_ok, math.radians(tol_yaw_ok_deg), stall_s
        self.best, self.t_best = float('inf'), 0.0
        self.hx, self.hy, self.m, self.self_in = half_x, half_y, margin, self_in
        self.min_hits, self.v_esc, self.eps, self.t_react = min_hits, v_esc, toward_eps, t_react
        # 띠 너비: 가장 빠른 몸 점 (병진 + 회전 × 모서리 반지름) 이 t_react + 스캔 나이 (node 가 0.3 s 까지 받음) 동안 가는 거리까지
        self.band = margin + (t_react + 0.3) * (v_max + w_max * math.hypot(half_x, half_y))
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
    def near_points(self, raw_body, pose, keep_box=False):
        """몸 둘레 band 띠 안 · 박스 아닌 점 (몸 좌표). keep_box = 박스 점도 다 막는 점 (제자리 회전: 몸 모서리가 박스 면을 쓸고 감)"""
        P = np.asarray(raw_body, float).reshape(-1, 2)
        if len(P) == 0:
            return P
        P = P[~((np.abs(P[:, 0]) < self.self_in) & (np.abs(P[:, 1]) < self.self_in))]
        P = P[(np.abs(P[:, 0]) < self.hx + self.band) & (np.abs(P[:, 1]) < self.hy + self.band)]
        if len(P) == 0 or keep_box:
            return P
        c, byaw = self.box_pose()
        x, y, yaw = pose
        cr, sr = math.cos(yaw), math.sin(yaw)
        cb = np.array([cr * (c[0] - x) + sr * (c[1] - y), -sr * (c[0] - x) + cr * (c[1] - y)])
        a = byaw - yaw
        q = (P - cb) @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])   # 박스 좌표
        lim = self.box_half + self.box_excl
        r = -cb @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])   # 로봇 중심 (박스 좌표)
        if not (r[0] < -self.box_half and abs(r[1]) < self.hy):
            return P   # 로봇이 박스 앞면 정면이 아님 (옆 · 대각, 몸 반폭 밖) → 박스 점도 막는 점 (모서리를 쓸고 가지 않게)
        face = (q[:, 0] > -lim) & (q[:, 0] < -self.box_half + self.face_depth) & (np.abs(q[:, 1]) < lim)   # 로봇 쪽 면 앞뒤 띠만
        return P[~face]

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
        return near[(closing > self.eps) & (n < self.m + closing * (self.t_react + self.age))]

    def describe(self, P):
        p = P[np.argmin(np.maximum(np.abs(P[:, 0]) - self.hx, np.abs(P[:, 1]) - self.hy))]
        side = '앞' if p[0] > self.hx else '뒤' if p[0] < -self.hx else '왼쪽' if p[1] > 0 else '오른쪽'
        gap = max(abs(p[0]) - self.hx, abs(p[1]) - self.hy)
        return f'{side} 몸에서 {gap * 100:.1f} cm (몸 중심 기준 ({p[0]:+.3f}, {p[1]:+.3f}) m, 점 {len(P)})'

    def safe(self, near, cmd):
        return len(self.blockers(near, cmd)) < self.min_hits

    # ---------------------------------------------------------------- 제어
    def step(self, pose, raw_body, t=0.0, raw_age=0.0):
        """pose (x, y, yaw) odom, t 시작부터 [s], raw_age 원래 스캔 나이 [s]. 돌려주는 것: (cmd (vx, vy, wz), 상태) 상태 = 'run' · 'done' · 'fail'"""
        self.age = max(0.0, float(raw_age))
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
        if not self.aligned and abs(dpsi) > self.align:   # 1. 제자리 회전 (박스 면도 막는 점 — 박스 앞 2 cm 에서 30° 돌면 모서리가 2 cm 파고듦)
            near = self.near_points(raw_body, pose, keep_box=True)
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


class FaceTrack:
    """박스 앞면 추적 (위 설명). c = 박스 중심, th = 지도 대비 회전, goal() = 정차 목표"""

    def __init__(self, goal_xy, goal_yaw, box_dist, half, search=0.25, track=0.06, track_deg=8.0, max_deg=15.0,
                 freeze_dist=0.10, med_n=5, tol=0.012, iters=80):
        self.yaw0, self.dist, self.h = float(goal_yaw), box_dist, half
        self.c0 = np.asarray(goal_xy, float) + box_dist * np.array([math.cos(goal_yaw), math.sin(goal_yaw)])
        self.c, self.th = self.c0.copy(), 0.0
        self.search, self.track, self.track_th, self.max_th = search, track, math.radians(track_deg), math.radians(max_deg)
        self.freeze, self.med_n, self.tol, self.iters = freeze_dist, med_n, tol, iters
        self.hist, self.n_ok, self.last_why = [], 0, ''
        self.rng = np.random.default_rng(0)

    def goal(self):
        y = self.yaw0 + self.th
        return self.c - self.dist * np.array([math.cos(y), math.sin(y)]), wrap(y)

    def update(self, P, robot_xy):
        g, _ = self.goal()
        if np.hypot(*(robot_xy - g)) < self.freeze:
            self.last_why = 'freeze'
            return False
        first = self.n_ok == 0
        m = self.find(P, robot_xy, self.c0 if first else self.c, self.search if first else self.track,
                      0.0 if first else self.th, self.max_th if first else self.track_th, first)
        if m is None:
            return False
        if first and self.search > 0.08:
            self.last_why = f'찾음 (지도에서 {np.hypot(*(m[:2] - self.c0)) * 100:.0f} cm · {math.degrees(m[2]):+.0f}°)'
        self.hist = (self.hist + [m])[-self.med_n:]
        H = np.array(self.hist)
        self.c, self.th = np.median(H[:, :2], axis=0), float(np.median(H[:, 2]))
        self.n_ok += 1
        if not first:
            self.last_why = 'ok'
        return True

    def find(self, P, robot_xy, center, radius, th_ref, th_tol, first):
        """center ±radius · 방향 th_ref ±th_tol 안 앞면 조각 → (중심 x, y, th) 또는 None (last_why = 이유)"""
        h = self.h
        Q = P[np.hypot(P[:, 0] - center[0], P[:, 1] - center[1]) < radius + 2 * h]
        if len(Q) < 8:
            self.last_why = f'점 {len(Q)}'
            return None
        best, why = None, '조각 없음'
        for _ in range(self.iters):
            i, j = self.rng.choice(len(Q), 2, replace=False)
            d = Q[j] - Q[i]
            L = float(np.hypot(*d))
            if not 0.03 < L < 2 * h + 0.03:
                continue
            d /= L
            nn = np.array([-d[1], d[0]])
            on = Q[np.abs((Q - Q[i]) @ nn) < self.tol]
            s = np.sort((on - Q[i]) @ d)
            if len(s) < 8:
                continue
            gaps = np.nonzero(np.diff(s) > 0.03)[0]          # 표본 두 점을 품은 연속 조각 (3 cm 넘게 비면 끊김)
            k = np.searchsorted(gaps, np.searchsorted(s, 0.0) - 1, side='right')   # 표본이 조각 첫 점이어도 그 조각 (11-08)
            lo = s[gaps[k - 1] + 1] if k > 0 else s[0]
            hi = s[gaps[k]] if k < len(gaps) else s[-1]
            if not lo <= L <= hi:
                continue
            seg = on[(((on - Q[i]) @ d) >= lo) & (((on - Q[i]) @ d) <= hi)]
            if len(seg) < 8:
                continue
            mu = seg.mean(0)                                  # 조각 점으로 직선 다시 맞춤 (표본 두 점 방향은 잡음이 큼)
            d = np.linalg.svd(seg - mu)[2][0]
            nn = np.array([-d[1], d[0]])
            ss = (seg - mu) @ d
            lo, hi = float(ss.min()), float(ss.max())
            Q_i = mu
            span = hi - lo
            if not 0.12 <= span <= 2 * h + 0.03:
                why = f'조각 길이 {span * 100:.0f} cm'
                continue
            sa = (P - Q_i) @ d                                # 끝 밖 2–14 cm 에 같은 직선 점 = 벽
            onP = np.abs((P - Q_i) @ nn) < self.tol
            if (onP & (((sa > hi + 0.02) & (sa < hi + 0.14)) | ((sa < lo - 0.02) & (sa > lo - 0.14)))).sum() >= 4:
                why = '면 밖으로 이어짐 (벽)'
                continue
            mid = Q_i + 0.5 * (lo + hi) * d
            if nn @ (robot_xy - mid) < 0:
                nn = -nn
            th = wrap(math.atan2(-nn[1], -nn[0]) - self.yaw0)
            if abs(wrap(th - th_ref)) > th_tol:
                why = f'기울기 {math.degrees(th):.0f}°'
                continue
            full = span >= 2 * h - 0.03
            if full or first:                                 # 양 모서리 보임 (처음엔 조각 가운데로)
                c = mid - h * nn
            else:                                             # 한쪽만 → 옆은 이전 추정을 이 면에 투영
                c = mid + ((self.c - mid) @ d) * d - h * nn
            dist = float(np.hypot(*(c - center)))
            if dist > radius:
                why = f'지도 자리에서 {dist * 100:.0f} cm' if first else f'추정에서 {dist * 100:.0f} cm'
                continue
            score = dist - (0.05 if full else 0.0)            # 같으면 양 모서리 보인 조각
            if best is None or score < best[0]:
                best = (score, np.array([c[0], c[1], th]))
        if best is None:
            self.last_why = why
            return None
        return best[1]
