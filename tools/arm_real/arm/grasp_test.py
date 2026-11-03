#!/usr/bin/env python3
"""퍽 위 멈추기 시험 자세 만들기 (2026-10-30, 계산만). 그리퍼는 안 씀 — 손끝 가운데가 퍽 중심 위에 오는지 눈으로 확인.

  $NT_PY grasp_test.py [--x 0.235 --y 0.0] [--heights 60 25] [--tilt 0] [--out grasp_poses.json]
  → grasp_poses.json (+ pose_safety.py 로 검사 → grasp_poses.checked.json → arm_pose_runner.py)

놓는 법 (기본값): 박스(205 × 205 × 70) 를 차체 앞면 3 cm 앞에 똑바로, 퍽(원기둥 Ø30 × 60) 을 박스 가운데 줄 위, 박스 앞 모서리에서 60 mm.
  = base_link 기준 퍽 중심 x 0.235 · y 0 (차체 앞면 x 0.145).
자세: 손끝 가운데(두 손가락 끝 사이) 가 퍽 윗면 가운데 위 heights mm (기본 60 · 25), 접근축 수직에서 tilt° (차체 쪽으로 기울임, 기본 0 = 수직).
  IK = bin12_reach 모델 (배포 URDF), 결과 q 를 '실제 관절' 로 보고 처짐 모델(sag_model.py) 로 엔코더 목표(enc) 를 따로 저장.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import arm_pose_fk as fk  # noqa: E402
import sag_model  # noqa: E402

BASE_Z = 0.072
BOX_TOP = 0.070 - BASE_Z          # 박스 윗면 (base_link z)
PUCK_H = 0.060


def q_to_ticks(q):
    out = []
    for a, x in zip(fk.JM['arm'], q):
        out.append(int(round(a['ref_tick'] + (math.degrees(x) - a['q_deg_at_ref']) / (a['sign'] * 360 / 4096))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--x', type=float, default=0.235)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--heights', type=float, nargs='+', default=[60, 25], help='손끝이 퍽 윗면 위 몇 mm')
    ap.add_argument('--tilt', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=None, help='[°] 네모 퍽 돌림 (base_link) — 손가락이 퍽 면에 수직으로 닫히게 (90° 마다 같음, 기본 닫힘축에 가장 가까운 면)')
    ap.add_argument('--out', default=str(HERE / 'grasp_poses.json'))
    ap.add_argument('--puck_r', type=float, default=0.015, help='[m] 검사용 퍽 반지름 (네모 30 mm 는 대각선 반 0.0212)')
    ap.add_argument('--note', default='')
    a = ap.parse_args()
    import pose_safety as ps
    C = ps.Checker()
    M = C.M

    def in_range(q):
        t = q_to_ticks(q)
        return all(max(C.tick_rng[k][0], C.ros_rng[k][0]) <= t[k] <= min(C.tick_rng[k][1], C.ros_rng[k][1]) for k in range(5))
    top = BOX_TOP + PUCK_H
    t = math.radians(a.tilt)
    az = math.atan2(a.y - M.j1_xy[1], a.x - M.j1_xy[0])                 # 차체 쪽(J1 쪽)으로 기울임
    app = np.array([math.sin(t) * math.cos(az), math.sin(t) * math.sin(az), -math.cos(t)])
    close = np.array([-math.sin(az), math.cos(az), 0.0])                   # 손가락 닫힘 = 좌우
    if a.yaw is not None:                                                  # 네모 퍽: 면 법선 4 개 중 기본 닫힘축에 가장 가까운 것 → 접근축에 수직으로 투영
        ns = [np.array([math.cos(math.radians(a.yaw) + k * math.pi / 2), math.sin(math.radians(a.yaw) + k * math.pi / 2), 0.0]) for k in range(4)]
        n = max(ns, key=lambda v: v @ close)
        close = n - (n @ app) * app
        close /= np.linalg.norm(close)
    poses = []
    seed = None
    for h in a.heights:
        p = np.array([a.x, a.y, top + h / 1000])
        sols = M.ik(p, app, close, seed=seed) if seed is not None else M.ik(p, app, close)
        good = [s for s in sols if s[1]['pos_mm'] < 1 and s[1]['app_deg'] < 1 and s[1]['roll_deg'] < 2 and in_range(s[0])]
        if not good:
            best = min(sols, key=lambda s: s[1]['pos_mm'] + s[1]['app_deg'])
            sys.exit(f'높이 {h} mm: IK 안 됨 (가장 가까운 오차 {best[1]}) — --tilt 를 늘리거나 퍽을 더 가까이')
        q0 = M.template['q']
        q5_rest = fk.ticks_to_q({'11': 4084, '12': 1102, '13': 2668, '14': 1971, '16': 2063, '15': 3504})[0][4]
        # 손가락 닫힘은 90° 마다 같음(퍽 원기둥 · 정육면체) → 손목 롤이 받침 자세(2063)에 가까운 해 먼저, 그다음 꺾임이 템플릿에 가까운 것
        q = min(good, key=lambda s: (round(abs(s[0][4] - q5_rest) / 0.3), np.linalg.norm(s[0][1:4] - q0[1:4])))[0]
        seed = q
        fq = 0.0
        ticks = q_to_ticks(q)
        qe = sag_model.encoder_goal_q(M, q, fq)
        enc = q_to_ticks(qe)
        tau = sag_model.torques(M, q, fq)
        tip, ax, cl, _ = M.grip(q)
        print(f'h{h:.0f}: 손끝 {np.round(tip * 1000, 1).tolist()} mm (바닥 위 {(tip[2] + BASE_Z) * 1000:.0f}) · 접근축 수직에서 {math.degrees(math.acos(min(1.0, max(-1.0, -ax[2])))):.1f}°'
              f' · 실제 목표 틱 {ticks} · 엔코더 목표 {enc} (차 {[e - k for e, k in zip(enc, ticks)]}) · τ {[round(tau[j], 2) for j in (2, 3, 4)]} N·m')
        poses.append(dict(name=f'h{h:.0f}', ticks=ticks, enc=enc, tip_mm=np.round(tip * 1000, 1).tolist()))
    box_cx = 0.145 + 0.03 + 0.1025
    obstacles = [dict(name='L_box', kind='box', xyz=[box_cx, 0.0, BOX_TOP - 0.035], dims=[0.1025, 0.1025, 0.035]),
                 dict(name='puck', kind='cyl', xyz=[a.x, a.y, BOX_TOP + PUCK_H / 2], dims=[a.puck_r, PUCK_H])]
    Path(a.out).write_text(json.dumps(dict(note=f'퍽 위 멈추기 시험 · 퍽 중심 x {a.x} y {a.y} (base_link) · 박스 205 몸통 앞 3 cm · grasp_test.py {a.note}',
                                           obstacles=obstacles, poses=poses), ensure_ascii=False, indent=1))
    print('저장', a.out)


if __name__ == '__main__':
    main()
