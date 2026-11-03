#!/usr/bin/env python3
"""pose_safety 가 검사한 순서 파일(out/*.checked.json) → config/arm_paths.yaml (control_master 파라미터)

명령값 = enc(처짐 앞먹임, 없으면 ticks) + pgap → rad = (틱 − 영점) · 2π/4096 (영점 = hardware_map zero_tick)
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import collections
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from gen_paths import S_TO_OLD  # noqa: E402
OLD_TO_S = {v: k for k, v in S_TO_OLD.items()}
OUT = HERE / 'out'
ZERO = [4088, 2048, 2048, 2048, 2048]
T2R = 2 * math.pi / 4096
NAMES = [f'store_S{i}' for i in range(1, 13)] + [f'retrieve_S{i}' for i in range(1, 13)] + \
        ['look', 'look_barcode', 'home_from_look', 'home_from_look_barcode', 'unfold', 'fold']
ROT = {'store_S10_rot': 'store_S10', 'retrieve_S10_rot': 'retrieve_S10', 'retrieve_S7_rot': 'retrieve_S7'}   # 손목 돌린 후보, 실물 확인 전
ROT_NOTE = {'S10': '손목(J5) −90° (S5 가 차 있어도 손가락이 S5 퍽 쪽으로 안 벌어짐)',
            'S7': '손목(J5) +68° (탐색에서 가장 나은 각도: S12 퍽과 기본 자세 1 mm · 관절 ±2° 오차에서 −4 mm, 원래 −8 · −13 mm)'}
NAMES += [n for n in ROT if (OUT / f'{n}.checked.json').exists()]


def fmt(xs):
    return '[' + ', '.join(xs) + ']'


NEAR_DZ = 0.04   # 쥔 퍽(손가락) 바닥이 칸 테두리 + 40 mm 아래 · 적재함 영역(칸 중심 ± 40 mm) 안이면 칸 근처
NEAR_SPEED = 15.0


def near_flags(seq):
    """실물 재생 경로에서 칸 근처 점: 실물처럼 15 °/s, 나머지(박스 · 적재함 위 빈 공간)는 기본 속도 (실물 기록 최고 40 °/s)"""
    import numpy as np
    import pose_safety as ps
    import rack_load
    C = ps.Checker()
    rc = json.loads(Path(os.path.join(os.environ.get('NT_ARM_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'arm')), 'rack_cells.json')).read_text())
    xs = [c['xy'][0] for c in rc['cells'].values()]
    ys = [c['xy'][1] for c in rc['cells'].values()]
    out = []
    for w in seq:
        q, _ = C._qf(w['ticks'], grip=ps.GRIP_JM['held'])
        P = rack_load.puck_corners(C.M, q)
        c = P.mean(0)
        out.append(bool(min(xs) - 0.04 < c[0] < max(xs) + 0.04 and min(ys) - 0.04 < c[1] < max(ys) + 0.04
                        and P[:, 2].min() < rc['rim_z'] + NEAR_DZ))
    return out


def warn_summary(n):
    """안전 검사 경고 요약 (실물 성공 궤적이라 막지 않고 남김): 종류별 최악값"""
    f = OUT / f'{n}.safety.log'
    lines = [l for l in f.read_text().splitlines() if '✗' in l and '전체' not in l] if f.exists() else []
    d = json.loads((OUT / f'{n}.checked.json').read_text())
    lines += [w for w in d.get('safety_warnings', []) if w.startswith('[기준점')]
    near, rng = {}, {}
    for l in lines:
        for m in re.finditer(r'([a-z_0-9]+) ↔ ([A-Za-z_0-9 ]+?) (-?\d+) mm', l):
            k = f'{m.group(1)}↔{m.group(2).strip()}'
            k = re.sub(r'cup (\w+)( puck)?', lambda c: f'{OLD_TO_S.get(c.group(1), c.group(1))} {"퍽" if c.group(2) else "컵"}', k)
            near[k] = min(near.get(k, 99), int(m.group(3)))
        for m in re.finditer(r'(J\d) 틱 (\d+) 범위 밖 \((\d+)~(\d+)\)', l):
            t, lo, hi = int(m.group(2)), int(m.group(3)), int(m.group(4))
            over = lo - t if t < lo else t - hi
            rng[m.group(1)] = max(rng.get(m.group(1), 0), over)
    out = [f'{j} 범위 {v} 틱 넘음' for j, v in sorted(rng.items())] + [f'{k} {v} mm' for k, v in sorted(near.items(), key=lambda x: x[1])[:3]]
    return '; '.join(out) or '없음'


def stand_paths():
    """표준 펼치기(받침 → P0) · 접기(P0 → 받침) — pose_safety 의 실물 검증 순서. 기준점(박스 위 들기) 자세"""
    sys.path.insert(0, str(HERE))
    sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
    import pose_safety as ps
    mk = lambda items: [dict(name=n, ticks=list(t), stand=True, stop=False) for n, t in items]
    (OUT / 'unfold.checked.json').write_text(json.dumps(dict(sequence=mk([('받침', ps.REST)] + ps.STANDUP)), ensure_ascii=False))
    (OUT / 'fold.checked.json').write_text(json.dumps(dict(sequence=mk([('P0_복귀', ps.STANDUP[-1][1])] + [(n + '_복귀', t) for n, t in ps.RETURN[:-1]] +
                                                                [('받침_복귀', ps.RETURN[-1][1])])), ensure_ascii=False))


def main(dst):
    stand_paths()
    summ = {}
    for k in ('store', 'retrieve'):
        f = OUT / f'replay_summary_{k}.json'
        if f.exists():
            summ.update(json.loads(f.read_text()))
    lines = ['# 팔 실물 경로 — 실물 적재함 시험에서 성공한 실행 기록(arm_tests/runs 의 joint_states · 그리퍼 사건)을 그대로 재생해 만듦. 손으로 고치지 말 것',
             '#   재생: 16 틱 간격으로 줄이고 처짐 앞먹임(엔코더 명령값)으로 바꿈 → pose_safety 로 다시 검사 (경고는 남기고 막지 않음: 실물에서 성공한 궤적)',
             '# 점마다 관절 5 개 명령값 [rad] (처짐 앞먹임 포함), 받침 구간, 최고 속도 [°/s] (0 = 기본), 도착 뒤 그리퍼 (- = 없음), 멈춤',
             '#   속도: 칸 근처(쥔 퍽 바닥이 칸 테두리 + 40 mm 아래, 적재함 영역 안)만 15 °/s, 빈 공간은 기본 (실물 기록도 칸 근처 15 · 빈 공간 최고 40 °/s)',
             '# STORE S_k: 박스 위 기준점(퍽 쥠, PICK 끝) → 실물 들기 → 칸에 놓기 → 받침 (주행 허용)',
             '# RETRIEVE S_k: 받침 → 칸에서 꺼내기 → 실물 들기 → 박스 위 기준점 (퍽 쥠, PLACE 시작)',
             '# 칸마다 "그때 찬 칸" = 그 실물 실행 때 차 있던 칸. 그보다 더 찬 상태는 실물로 해 본 적 없음 (옆 칸 퍽과 가까운 칸은 경고에 나옴)',
             'control_master:', '  ros__parameters:', '    arm_paths:']
    have = []
    for n in NAMES:
        f = OUT / f'{n}.checked.json'
        if not f.exists():
            print('없음', n)
            continue
        seq = json.loads(f.read_text())['sequence']
        near = near_flags(seq) if n.startswith(('store_', 'retrieve_')) else None
        q, stand, speed, grip, stop, ids = [], [], [], [], [], []
        for i, s in enumerate(seq):
            if near is not None and not s['stand']:
                s['speed'] = NEAR_SPEED if near[i] else 0.0
            cmd = [b + g for b, g in zip(s.get('enc') or s['ticks'], s.get('pgap', [0] * 5))]
            q += [f'{(c - z) * T2R:.5f}' for c, z in zip(cmd, ZERO)]
            stand.append('true' if s['stand'] else 'false')
            speed.append(f"{float(s.get('speed') or 0.0):.1f}")
            grip.append(s.get('grip') or '-')
            stop.append('true' if s['stop'] else 'false')
            ids.append(json.dumps(s['name'], ensure_ascii=False))
        have.append(n)
        if n in ROT:
            cell = n.split('_')[1]
            lines += [f"      # {n}: {ROT[n]} 실물 재생 경로에서 칸 안 구간만 {ROT_NOTE[cell]}. 실물 확인 전 후보 — slot '{cell}_rot'",
                      f'      #   안전 검사 (나머지 11 칸 다 참): {warn_summary(n)}']
        elif n in summ:
            lines += [f"      # {n}: {summ[n]['desc'].replace('⚠ 안전 검사 경고 있음 · ', '')} · 그때 찬 칸 {', '.join(summ[n]['full_cells']) or '없음'}",
                      f'      #   안전 검사 경고: {warn_summary(n)}']
        lines += [f'      {n}:',
                  f"        ends_home: {'true' if seq[-1]['name'].startswith('받침') else 'false'}",
                  f'        nodes: {fmt(ids)}', f'        q: {fmt(q)}', f'        stand: {fmt(stand)}',
                  f'        speed: {fmt(speed)}', f'        grip: {fmt(grip)}', f'        stop: {fmt(stop)}']
    lines.insert(lines.index('    arm_paths:') + 1, f'      names: {fmt(have)}')
    st = json.loads((OUT / 'staging_ik.json').read_text())['poses']
    h40 = next(p for p in st if p['name'] == 'h40')
    lines += ['    # 박스 위 기준점 (퍽 쥔 들기, 실물 빼기 시험의 내려놓는 자리 210.2, 38.0 mm, -4° 위 40 mm): PICK 이 끝나는 자세 = STORE 가 시작하는 자세',
              f"    staging_pose: {fmt([f'{(c - z) * T2R:.5f}' for c, z in zip(h40['enc'], ZERO)])}"]
    Path(dst).write_text('\n'.join(lines) + '\n')
    print(f'{len(have)} 경로 → {dst}')


if __name__ == '__main__':
    main(sys.argv[1])
