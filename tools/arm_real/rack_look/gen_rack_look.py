#!/usr/bin/env python3
"""LOOK_RACK 경로 만들기: config/arm_paths.yaml (실물 기록 STORE · 빈손 적재함 스캔) → config/rack_look_paths.yaml

칸 S 마다 두 경로 (control_master 가 arm_paths.look_rack_names 로 읽음)
  look_rack_S  : 박스 위 기준점 (PICK 끝 자리로 바꿔 이음) → 기록 STORE 앞부분 (실물 들기 lift_real … ENTER) → 관찰 자세 (그 칸을
                 빈손 스캔이 내려다본 자세, 칸 바닥 태그가 수직에서 약 7°) 에서 멈춤 — 퍽 쥔 채, perception 이 칸 바닥 태그를 잼
  store_look_S : 관찰 자세 → 기록 STORE 의 컵 입구 바로 위 점 (HOVER) → 그 뒤 기록 그대로 (넣기 · 놓기 · 받침)
골라 놓은 점 (ENTER · HOVER · 관찰 자세) = MoveIt 충돌 검사 (seg_check.cpp):
  팀 URDF (새 적재함 CAD 충돌) · 쥔 퍽 (34.8 × 34.8 × 60 + 3 mm, 손끝 = 윗면 − 50 mm) · 다른 11 칸에 선 퍽 · 앞 박스 (정차 0.2575) 와
  박스 안 퍽 2 개 · 팔 링크 여유 5 mm 로 관절 직선 0.5° 간격 — ENTER → 관찰, 관찰 → HOVER, 관찰 자세 모두 닿지 않음
  HOVER = 관찰 자세와 이어도 되는 가장 늦은 점 − 2 (그다음 점부터 쥔 퍽이 그 칸 컵에 닿음 = 넣기)
  관찰 자세에서 쥔 퍽은 화면 오른쪽 아래 (u ≥ 414, v ≥ 251) — 칸 태그와 안 겹치게 고름 (S10 은 스캔 자세 J1 −4°)
  /home/pa25/IsaacLab-course 없이: python3 gen_rack_look.py
"""
import math
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
SCAN_STOP = {'S1': 'scan_LB2', 'S2': 'scan_LB2', 'S3': 'scan_LB1', 'S4': 'scan_R1', 'S5': 'scan_LF1', 'S6': 'scan_LF2',
             'S7': 'scan_R2', 'S8': 'scan_LB1', 'S9': 'scan_R1', 'S10': 'scan_LF1', 'S11': 'scan_LF2', 'S12': 'scan_R2'}
SCAN_DQ = {'S10': [math.radians(-4.0), 0, 0, 0, 0]}   # 관찰 자세 고침 (관절 rad)
ENTER = {'S3': 8}                                      # 없으면 1 (lift_real 에서 바로 관찰 자세)
HOVER = {'S1': 140, 'S2': 144, 'S3': 88, 'S4': 40, 'S5': 36, 'S6': 74, 'S7': 65, 'S8': 89, 'S9': 55, 'S10': 36, 'S11': 70, 'S12': 63}


def path(P, name):
    p = P[name]
    n = len(p['nodes'])
    k = len(p['q']) // n
    return [dict(name=p['nodes'][i], q=p['q'][i * k:(i + 1) * k], stand=p['stand'][i], speed=p['speed'][i], grip=p['grip'][i],
                 stop=p['stop'][i], hold=(p.get('hold') or [0.0] * n)[i]) for i in range(n)]


def emit(name, nodes, ends_home, note):
    f = lambda xs: '[' + ', '.join(xs) + ']'
    lines = [f'      # {note}', f'      {name}:', f'        ends_home: {"true" if ends_home else "false"}',
             '        nodes: ' + f([f'"{n["name"]}"' for n in nodes]),
             '        q: ' + f([f'{v:.5f}' for n in nodes for v in n['q']]),
             '        stand: ' + f(['true' if n['stand'] else 'false' for n in nodes]),
             '        speed: ' + f([f'{float(n["speed"]):.1f}' for n in nodes]),
             '        grip: ' + f([f'"{n["grip"]}"' for n in nodes]),
             '        stop: ' + f(['true' if n['stop'] else 'false' for n in nodes]),
             '        hold: ' + f([f'{float(n["hold"]):.1f}' for n in nodes])]
    return '\n'.join(lines)


def main():
    d = yaml.safe_load(open(REPO / 'config' / 'arm_paths.yaml'))
    P = d['control_master']['ros__parameters']['arm_paths']
    scan = {n['name']: n for n in path(P, 'rack_scan')}
    out, names = [], []
    for c in range(1, 13):
        cs = f'S{c}'
        st = path(P, f'store_{cs}')
        rel = next(i for i, n in enumerate(st) if n['grip'] == 'release')
        h, j = HOVER[cs], ENTER.get(cs, 1)
        assert 1 <= j < h < rel, cs
        look = dict(scan[SCAN_STOP[cs]])
        look['q'] = [a + b for a, b in zip(look['q'], SCAN_DQ.get(cs, [0] * 5))]
        look.update(name='rack_look', stand=False, speed=0.0, grip='-', stop=True, hold=0.0)
        pre = [dict(n, stop=False, hold=0.0) for n in st[:j + 1]]
        out.append(emit(f'look_rack_{cs}', pre + [look], False,
                        f'look_rack_{cs}: 기록 store_{cs} 0–{j} → 관찰 ({SCAN_STOP[cs]}{" 고침" if cs in SCAN_DQ else ""}) 멈춤, 퍽 쥠'))
        start = dict(look, stop=False)
        out.append(emit(f'store_look_{cs}', [start] + st[h:], True,
                        f'store_look_{cs}: 관찰 → 기록 store_{cs} {h} (컵 입구 위, 놓기 {rel}) → 끝 (받침)'))
        names += [f'look_rack_{cs}', f'store_look_{cs}']
    head = ['# LOOK_RACK 경로 — tools/arm_real/rack_look/gen_rack_look.py 가 config/arm_paths.yaml 에서 만듦. 손으로 고치지 말 것',
            '#   look_rack_S: PICK 끝 → 관찰 자세 (퍽 쥔 채 칸 바닥 태그를 손목 카메라로) · store_look_S: 관찰 → 컵 입구 위 → 기록 넣기 → 받침',
            '#   점 고르기 = MoveIt 충돌 검사 (seg_check.cpp, 머리말 gen_rack_look.py)',
            'control_master:', '  ros__parameters:', '    arm_paths:', '      look_rack_names: [' + ', '.join(names) + ']']
    (REPO / 'config' / 'rack_look_paths.yaml').write_text('\n'.join(head + out) + '\n')
    print('config/rack_look_paths.yaml', len(names), 'paths')


main()
