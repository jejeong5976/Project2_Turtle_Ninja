#!/usr/bin/env python3
"""놓기 자세 후보 (2026-10-29, 시뮬 팔 모델 · 포트 없음): 실물 집기 순간과 같은 잡힘으로 퍽을 세운 채 격자 점에 놓는 팔 관절값.

  python3 place_ik.py                    # L1 · L3 (205 박스) 9 점 + 최대 도달 표 → place_poses.json · 화면 표
  python3 place_ik.py --box_y 0.03       # 박스 중심을 차체 기준 왼쪽 3 cm 에 (로봇 서는 자리 바꿔 보기)

방법
  1. 실물 집기 순간 (ARM_20261026T022813Z, 그리퍼가 닫히기 시작한 순간) 팔 관절값 → URDF 순기구학으로 손목(arm_link5_wrist_roll) 자세.
     그때 퍽 = 세운 채, 중심 xy = 집기 자리(차체 기준 앞 0.342 · 왼 0.077, 10-27 시뮬 집기 8/8 확인), 바닥 = 70 mm 박스 위.
     → 손목 기준 퍽 자세(잡힘)를 고정.
  2. 놓기: 퍽 중심을 격자 점 위(바닥 = 박스 위 + clearance), 퍽 축 = 수직(세운 채) 이 되는 팔 5 관절을 최소제곱으로 풂
     (퍽 수직축 회전은 자유 — 원기둥. 직육면체 퍽 방향은 판정 대상 아님으로 가정). 시작값 = 집기 자세 (같은 팔꿈치 위 자세).
  3. 검사: 관절 한계 여유, 손가락 · 팔 점이 박스 윗면 · 바닥 · 차체 위인지 (arm_poses.ArmFK 점), 위치 오차.
  틱 = config/joint_map.json (배포본 v10 지도, J2~J5 줄자 확인 · J1 은 '가정' 표시 있음).
로봇 서는 자리: 차체 앞(바퀴 포함 0.145 m)이 박스에서 gap 떨어짐 → 박스 중심 x = 0.145 + gap + 0.1025.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

SIM = Path(os.environ.get('NT_ROBOT_SIM', os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'), 'sim', 'robot_sim')))
REL = Path(os.environ.get('NT_URDF_RELEASE', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nt_urdf_release_20261027_layout')))
sys.path.insert(0, str(SIM))
from sim_core import arm_poses as ap   # noqa: E402

URDF = SIM / 'robot_description' / 'robot.urdf'
REPLAY = REL / 'config' / 'arm_replays' / 'ARM_20261026T022813Z.json'
JOINT_MAP = REL / 'config' / 'joint_map.json'
ARM = ap.ARM_JOINTS
SPOT = (0.342, 0.077)          # 집기 순간 퍽 중심 xy (차체 기준)
BASE_Z = 0.072                 # base_link 바닥 위 높이
BOX = 0.205                    # 205 박스
BOX_TOP = 0.070 - BASE_Z       # 박스 윗면 z (base_link 기준)
PUCK_H = 0.060
BODY_FRONT = 0.145             # 차체 앞 (바퀴 포함)


def _rot(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def _rpy(r, p, y):
    return _rot((0, 0, 1), y) @ _rot((0, 1, 0), p) @ _rot((1, 0, 0), r)


class Chain:
    """base_link → link 의 4×4 (움직이는 관절은 q dict)"""

    def __init__(self, urdf, tip):
        by_child = {j.find('child').get('link'): j for j in ET.parse(str(urdf)).getroot().findall('joint')}
        self.steps, link = [], tip
        while link in by_child:
            j = by_child[link]
            o = j.find('origin')
            xyz = np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()])
            rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
            ax = j.find('axis')
            self.steps.append((j.get('name'), j.get('type'), xyz, _rpy(*rpy),
                               [float(v) for v in ax.get('xyz').split()] if ax is not None else None))
            link = j.find('parent').get('link')
        assert link == 'base_link', link
        self.steps.reverse()

    def T(self, q):
        T = np.eye(4)
        for name, typ, xyz, R0, axis in self.steps:
            A = np.eye(4)
            A[:3, :3], A[:3, 3] = R0, xyz
            T = T @ A
            if typ == 'revolute':
                B = np.eye(4)
                B[:3, :3] = _rot(axis, q.get(name, 0.0))
                T = T @ B
        return T


def grasp_q():
    d = json.loads(REPLAY.read_text())
    t, q, g = np.array(d['t_s']), np.array(d['arm_q_rad']), np.array(d['finger_q_m'])
    io = int(np.argmax(g))
    tg = float(t[io + int(np.argmax(g[io:] < g[io] - 0.001))])
    return {n: float(np.interp(tg, t, q[:, i])) for i, n in enumerate(d['joint_names'])}, float(np.interp(tg, t, g))


def to_ticks(q):
    jm = {a['joint']: a for a in json.loads(JOINT_MAP.read_text())['arm']}
    return {n: round(jm[n]['ref_tick'] + (math.degrees(q[n]) - jm[n]['q_deg_at_ref']) / (jm[n]['sign'] * 360 / 4096)) for n in ARM}


def puck_T(x, y, z_center):
    T = np.eye(4)
    T[:3, 3] = (x, y, z_center)
    return T


class PlaceIK:
    def __init__(self):
        self.wrist = Chain(URDF, 'arm_link5_wrist_roll')
        self.fk = ap.ArmFK(URDF)
        self.qg, self.finger = grasp_q()
        z_c = BOX_TOP + PUCK_H / 2
        self.P_w = np.linalg.inv(self.wrist.T(self.qg)) @ puck_T(SPOT[0], SPOT[1], z_c)   # 손목 기준 퍽 (잡힘)
        self.lim = np.array([self.fk.limits[n] for n in ARM])
        tuck = ap.ArmFK(URDF).points({**dict(zip(ARM, [0.0] * 5))})
        self.body_min_z = None

    def puck(self, qv):
        q = dict(zip(ARM, qv))
        return self.wrist.T(q) @ self.P_w

    def solve(self, x, y, z_bottom, q0=None):
        z_c = z_bottom + PUCK_H / 2

        def res(qv):
            P = self.puck(qv)
            return np.r_[(P[:3, 3] - (x, y, z_c)) * 1000.0, P[:2, 2] * 200.0]   # mm, 수직 기울기 (0.005 rad ≈ 1 mm 무게)

        x0 = np.array([self.qg[n] for n in ARM] if q0 is None else q0)
        r = least_squares(res, x0, bounds=(self.lim[:, 0], self.lim[:, 1]), xtol=1e-12, ftol=1e-12)
        P = self.puck(r.x)
        pos_err = float(np.linalg.norm(P[:3, 3] - (x, y, z_c)) * 1000)
        tilt = math.degrees(math.acos(min(1.0, P[2, 2])))
        return r.x, pos_err, tilt

    def check(self, qv, box_xy):
        """간섭: 팔 · 손가락 점이 바닥 위, 박스 윗면 위(박스 발자국 안), 차체 위. 관절 한계 여유 [°]"""
        q = dict(zip(ARM, qv))
        q.update({f: self.finger for f in ap.FINGERS})
        pts = self.fk.points(q)
        inside_box = (np.abs(pts[:, 0] - box_xy[0]) < BOX / 2 + 0.005) & (np.abs(pts[:, 1] - box_xy[1]) < BOX / 2 + 0.005)
        lowest_over_box = float(pts[inside_box, 2].min() - BOX_TOP) * 1000 if inside_box.any() else None
        margin = float(np.degrees(np.minimum(qv - self.lim[:, 0], self.lim[:, 1] - qv)).min())
        return dict(lowest_over_box_mm=lowest_over_box, floor_ok=bool((pts[:, 2] > ap.FLOOR_Z + ap.FLOOR_MARGIN).all()),
                    joint_margin_deg=margin)


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument('--gap', type=float, default=0.02, help='[m] 차체 앞 ~ 박스 가장자리')
    ap_.add_argument('--box_y', type=float, default=0.0, help='[m] 박스 중심 차체 기준 왼쪽')
    ap_.add_argument('--clearance', type=float, default=0.003, help='[m] 놓는 순간 퍽 바닥 ~ 박스 윗면 (작게 떨어뜨림)')
    ap_.add_argument('--above', type=float, default=0.04, help='[m] 위에서 내려오는 시작 높이 (놓기 위)')
    ap_.add_argument('--out', default=str(Path(__file__).with_name('place_poses.json')))
    a = ap_.parse_args()
    ik = PlaceIK()
    bx = BODY_FRONT + a.gap + BOX / 2
    print(f'집기 자세 (실물) {[round(math.degrees(ik.qg[n]), 1) for n in ARM]}° · 틱 {list(to_ticks(ik.qg).values())}')
    P0 = ik.puck([ik.qg[n] for n in ARM])
    print(f'잡힘 확인: 집기 자세에서 퍽 중심 {np.round(P0[:3, 3] * 1000, 1)} mm (집기 자리 {SPOT}), 기울기 {math.degrees(math.acos(P0[2, 2])):.2f}°')
    print(f'박스 중심 (차체 기준) x {bx:.4f} y {a.box_y:.3f} m · 격자 ±60 mm · 퍽 바닥 = 박스 위 + {a.clearance * 1000:.0f} mm')
    rows = []
    for dy in (0.06, 0.0, -0.06):            # 왼 → 오른 (차체 기준)
        for dx in (-0.06, 0.0, 0.06):        # 가까운 → 먼
            x, y = bx + dx, a.box_y + dy
            q_pl, e_pl, t_pl = ik.solve(x, y, BOX_TOP + a.clearance)
            q_up, e_up, t_up = ik.solve(x, y, BOX_TOP + a.clearance + a.above, q_pl)
            c = ik.check(q_pl, (bx, a.box_y))
            ok = e_pl < 1.0 and t_pl < 1.0 and e_up < 1.0 and c['joint_margin_deg'] > 3 and c['floor_ok'] and \
                (c['lowest_over_box_mm'] is None or c['lowest_over_box_mm'] > 2)
            qd = dict(zip(ARM, q_pl))
            rows.append(dict(cell=f'{"앞" if dx > 0 else "가운데" if dx == 0 else "가까운"}{"·왼" if dy > 0 else "·오른" if dy < 0 else ""}',
                             xy_body_m=[round(x, 4), round(y, 4)], ok=bool(ok), pos_err_mm=round(e_pl, 2), tilt_deg=round(t_pl, 2),
                             place_deg=[round(math.degrees(v), 2) for v in q_pl], place_tick=list(to_ticks(qd).values()),
                             above_deg=[round(math.degrees(v), 2) for v in q_up], above_tick=list(to_ticks(dict(zip(ARM, q_up))).values()),
                             above_err_mm=round(e_up, 2), **{k: (round(v, 1) if isinstance(v, float) else v) for k, v in c.items()}))
    print(f"\n{'칸':10} {'차체 기준 x,y (m)':18} {'가능':4} {'오차':>5} {'기울':>5} {'관절 여유':>8} {'박스 위 최저':>10}  놓기 틱 [J1..J5]")
    for r in rows:
        print(f"{r['cell']:10} {str(r['xy_body_m']):18} {'예' if r['ok'] else '아니오':4} {r['pos_err_mm']:5.2f} {r['tilt_deg']:5.2f} "
              f"{r['joint_margin_deg']:8.1f} {str(r['lowest_over_box_mm']):>10}  {r['place_tick']}")
    # 최대 도달: 박스 위 70 mm 에서 퍽을 세워 놓을 수 있는 가장 먼 앞 거리 (옆 0 · ±6 cm)
    reach = {}
    for y in (0.0, 0.06, -0.06, 0.077):
        lo, hi = 0.15, 0.6
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            _, e, t = ik.solve(mid, y, BOX_TOP + a.clearance)
            lo, hi = (mid, hi) if (e < 1.0 and t < 1.0) else (lo, mid)
        reach[f'y={y:+.3f}'] = round(lo, 4)
    print('\n최대 도달 (퍽 중심 앞 거리, 박스 높이, 관절 한계 안 · 간섭 검사 없음):', reach)
    Path(a.out).write_text(json.dumps(dict(gap_m=a.gap, box_center_body_m=[round(bx, 4), a.box_y], clearance_m=a.clearance, above_m=a.above,
                                           grasp_deg=[round(math.degrees(ik.qg[n]), 2) for n in ARM], grasp_tick=list(to_ticks(ik.qg).values()),
                                           joints=ARM, cells=rows, max_reach_m=reach), ensure_ascii=False, indent=1))
    print(f'\n저장 {a.out}')


if __name__ == '__main__':
    main()
