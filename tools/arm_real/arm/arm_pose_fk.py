#!/usr/bin/env python3
"""손으로 잡은 팔 자세(run_pose_read.sh 의 POSE_<시각>/04_pose_summary.json) → 손끝 위치 · 방향 (2026-10-30, 포트 안 씀).

  $NT_PY arm_pose_fk.py <POSE 폴더> [<POSE 폴더> …]

틱 → 관절: 배포본 config/joint_map.json 식 (q = q_deg_at_ref + sign × (틱 − ref_tick) × 360/4096). 그리퍼 간격 = 표 보간.
손끝 · 접근축 · 닫힘축 = bin12_reach 메시 모델 (배포 URDF, base_link 바닥 위 0.072 m).
보여 주는 것:
  손끝 가운데 (차체 기준 앞 · 왼 · 바닥 위 높이) — 집기 자세면 = 퍽을 쥔 자리 → 접근 학습 목표 (지금 가정 앞 0.342 · 왼 0.077)
  접근축 기울기 (수직에서) · 방위, 닫힘축 방위, 그리퍼 간격
  J1 방위 = J1 축에서 손끝 쪽 수평 방향 (0° = 차체 정면). '정면을 보게 잡은 자세' 면 이 값이 J1 영점 오차 → 정면 틱 = 지금 틱 + 보정
⚠ J1 · 그리퍼는 확장 위치 모드: 받침 자세에서 전원을 켰을 때만 틱 기준이 맞음.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
from reach_lib import REL  # noqa: E402
from reach_model import Model  # noqa: E402

JM = json.loads((REL / 'config' / 'joint_map.json').read_text())
BASE_Z = 0.072


def ticks_to_q(ticks):
    q = []
    for a in JM['arm']:                                   # 순서 = arm_joint1..5 (ID 11 12 13 14 16)
        t = ticks[str(a['id'])]
        q.append(math.radians(a['q_deg_at_ref'] + a['sign'] * (t - a['ref_tick']) * 360 / 4096))
    g = JM['gripper']
    tab = sorted(g['gap_table_tick_mm'])
    gap = float(np.interp(ticks[str(g['id'])], [t for t, _ in tab], [m for _, m in tab]))
    return np.array(q), gap, (gap - g['gap0_mm']) / 2000.0


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    M = Model(spacing=0.01)
    j1 = M.j1_xy
    for d in sys.argv[1:]:
        f = Path(d) / '04_pose_summary.json'
        if not f.exists():
            print(f'\n== {Path(d).name}: 요약 없음 (읽기가 끝까지 안 됨) — 건너뜀')
            continue
        s = json.loads(f.read_text())
        ticks = {k: v['pose_tick'] for k, v in s['joints'].items()}
        q, gap, fq = ticks_to_q(ticks)
        tip, a, c, T5 = M.grip(q)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -a[2]))))
        az_a = math.degrees(math.atan2(a[1], a[0]))
        az_c = math.degrees(math.atan2(c[1], c[0]))
        az_j1 = math.degrees(math.atan2(tip[1] - j1[1], tip[0] - j1[0]))
        j1_tick = ticks['11']
        fix = -az_j1 / (JM['arm'][0]['sign'] * 360 / 4096)          # 정면이 되려면 J1 을 이만큼 (틱)
        print(f"\n== {Path(d).name}")
        print(f"  틱 J1..J5 {[ticks[k] for k in ('11', '12', '13', '14', '16')]} · 그리퍼 {ticks['15']} (간격 약 {gap:.1f} mm)")
        print(f"  관절 [°] {np.degrees(q).round(1).tolist()}")
        print(f"  손끝 가운데: 앞 {tip[0]:.3f} · 왼 {tip[1]:+.3f} m · 바닥 위 {tip[2] + BASE_Z:.3f} m   (지금 집기 가정 앞 0.342 · 왼 +0.077)")
        print(f"  접근축: 수직에서 {tilt:.1f}° · 방위 {az_a:+.1f}° · 닫힘축 방위 {az_c:+.1f}°")
        print(f"  J1 방위 (J1 축 → 손끝, 0 = 정면) {az_j1:+.2f}° → 정면을 보게 잡았다면 정면 틱 ≈ {j1_tick + fix:.0f} (지도 가정 4088)")


if __name__ == '__main__':
    main()
