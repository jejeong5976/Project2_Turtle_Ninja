#!/usr/bin/env python3
"""칸 경로 옮기기 — 재생 경로(export_yaml.py 결과)의 칸 근처만 base_link 기준 dx · dy [mm] 옮김.
실물에서 퍽이 칸 옆 컵에 걸릴 때, 성공 궤적은 그대로 두고 칸 안쪽 구간만 맞추려고 씀.

  python3 shift_cell_paths.py <arm_paths.yaml 입력> <출력> <dx_mm,dy_mm> <칸 ...>
  예) python3 shift_cell_paths.py ../../../config/arm_paths.yaml ../../../config/arm_paths.yaml 0,5 S12

- 옮기는 경로: store_<칸> · retrieve_<칸> 둘 다 (넣을 때 · 꺼낼 때 같은 자리)
- 손끝이 칸 중심(config/rack_cells.yaml)에서 수평 35 mm 안이면 다 옮기고, 35~90 mm 는 비례해서 줄이고, 그 밖은 그대로
  STORE 는 P0_복귀 부터, RETRIEVE 는 P0 까지 안 건드림 (12 점에 걸쳐 0 → 1) → PICK 끝 · P0 · 받침 자세 그대로
- 관절 5 개로 손끝 위치 · 접근축 · 닫힘축을 최소제곱으로 맞춤 (위치에 큰 무게, 관절 변화가 작은 해)
- 경로 값은 처짐 앞먹임이 들어간 명령값 — 몇 mm 옮기는 정도는 처짐 토크 변화를 무시함
- 출력은 입력 글자 그대로에 q 줄과 '#   칸 보정:' 주석 줄만 바꿈. 바뀐 칸 · 옮긴 양 · 오차를 출력
"""
import re
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]
if len(sys.argv) < 5:
    sys.exit(__doc__)
src, dst = sys.argv[1], sys.argv[2]
dx_mm, dy_mm = (float(v) for v in sys.argv[3].split(','))
cells = sys.argv[4:]

km = yaml.safe_load(open(REPO / 'config/arm_kinematics.yaml'))['/**']['ros__parameters']['arm_model']
T0 = np.array(km['T_base_armbase']).reshape(4, 4)
JT = np.array(km['joint_T']).reshape(5, 4, 4)
AX = np.array(km['axis']).reshape(5, 3)
TIP5, APP5, CLOSE5 = np.array(km['tip5']), np.array(km['app5']), np.array(km['close5'])
REF, QREF, SIGN = np.array(km['ref_tick']), np.array(km['q_deg_at_ref']), np.array(km['sign'])
ZERO = np.array([4088, 2048, 2048, 2048, 2048], float)   # 팀 하드웨어 영점 (arm_kinematics.hpp zero_tick 기본)
rack = yaml.safe_load(open(REPO / 'config/rack_cells.yaml'))['rack_cells']['cells']
NEAR_MM, FAR_MM, RAMP = 35.0, 90.0, 12


def rot(a, t):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(4)
    R[:3, :3] = np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K
    return R


def cmd_to_q(c):   # 팀 명령 rad → 모델 관절 rad (arm_kinematics.cpp ticks_to_q)
    tick = ZERO + c * 4096 / (2 * np.pi)
    return np.radians(QREF + SIGN * (tick - REF) * 360 / 4096)


def grip(c):   # 손끝 · 접근축 · 닫힘축 (모델 base = base_link 와 x · y 같음)
    q = cmd_to_q(c)
    T = T0.copy()
    for j in range(5):
        T = T @ JT[j] @ rot(AX[j], q[j])
    R = T[:3, :3]
    return R @ TIP5 + T[:3, 3], R @ APP5, R @ CLOSE5


def feat(c):   # 위치 [mm] × 10 · 방향 × 100 (1° ≈ 1.7) — 관절 5 개로 6 개를 다 못 맞춤 → 위치 우선
    p, a, cl = grip(c)
    return np.concatenate([p * 1000 * 10, a * 100, cl * 100])


def solve(c0, dp_mm):
    goal = feat(c0) + np.concatenate([np.asarray(dp_mm) * 10, np.zeros(6)])
    c = c0.copy()
    lam = 1e-1   # 관절 변화 억제 (수직 손에서 J1 · J5 가 서로 대신 도는 것 막음)
    for _ in range(20):
        f = feat(c)
        J = np.stack([(feat(c + e) - f) / 1e-6 for e in np.eye(5) * 1e-6], axis=1)
        dc = np.linalg.solve(J.T @ J + lam * np.eye(5), J.T @ (goal - f) - lam * (c - c0))
        c = c + dc
        if np.abs(dc).max() < 1e-10:
            break
    return c


text = open(src).read()
paths = yaml.safe_load(text)['control_master']['ros__parameters']['arm_paths']
for cell in cells:
    cxy = np.array(rack[cell]['xy'])
    for kind in ('store', 'retrieve'):
        name = f'{kind}_{cell}'
        p = paths[name]
        nodes = p['nodes']
        lo, hi = (1, nodes.index('P0_복귀')) if kind == 'store' else (nodes.index('P0'), len(nodes) - 1)
        Q = np.array(p['q']).reshape(-1, 5)
        out = Q.copy()
        worst_pos = worst_app = 0.0
        for i, c in enumerate(Q):
            tip, app, _ = grip(c)
            d = np.linalg.norm(tip[:2] - cxy) * 1000
            w = float(np.clip((FAR_MM - d) / (FAR_MM - NEAR_MM), 0, 1)) * float(np.clip(min(i - lo, hi - i) / RAMP, 0, 1))
            if w == 0:
                continue
            dp = np.array([dx_mm * w, dy_mm * w, 0.0])
            out[i] = solve(c, dp)
            t2, a2, _ = grip(out[i])
            worst_pos = max(worst_pos, np.linalg.norm((t2 - tip) * 1000 - dp))
            worst_app = max(worst_app, np.degrees(np.arccos(np.clip(a2 @ app, -1, 1))))
        events = ' · '.join(f'{nodes[i]} {g} ({(grip(out[i])[0][0] - grip(Q[i])[0][0]) * 1000:+.2f}, '
                            f'{(grip(out[i])[0][1] - grip(Q[i])[0][1]) * 1000:+.2f}) mm'
                            for i, g in enumerate(p['grip']) if g != '-')
        print(f'{name}: 그리퍼 사건 {events} · 위치 오차 최대 {worst_pos:.3f} mm · 접근축 변화 최대 {worst_app:.2f}°'
              f' · 관절 변화 최대 {np.degrees(np.abs(out - Q).max()):.2f}° · 첫 점 · 끝 점 그대로 {np.allclose(out[[0, -1]], Q[[0, -1]])}')
        # 글자 그대로에서 이 경로의 q 줄 · 칸 보정 주석만 바꿈
        blk = re.search(rf'((?:^      # .*\n)*)(^      {name}:\n(?:^        .*\n)+)', text, re.M)
        head, body = blk.group(1), blk.group(2)
        qline = re.search(r'^        q: \[.*\]$', body, re.M).group(0)
        newq = '        q: [' + ', '.join(f'{v:.5f}' for v in out.reshape(-1)) + ']'
        note = f'      #   칸 보정: 칸 근처를 base_link ({dx_mm:+.1f}, {dy_mm:+.1f}) mm 옮김 (shift_cell_paths.py, 실물 확인)\n'
        head2 = re.sub(r'^      #   칸 보정:.*\n', '', head, flags=re.M) + note
        text = text.replace(head + body, head2 + body.replace(qline, newq))
open(dst, 'w').write(text)
