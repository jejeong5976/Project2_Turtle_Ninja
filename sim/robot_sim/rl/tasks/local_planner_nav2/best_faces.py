#!/usr/bin/env python3
"""정차 면 고르기: 구간 시간표 (eval_legs.py) 로 미션 방문 순서마다 박스 · 장소 면 (N · E · S · W) 조합 중 이동 시간 합 최소 → 줄일 수 있는 시간.

방문 = '|' 로 나눈 묶음 (기본 = 출발 | 수집 collect_order [1, 2, 6, 4, 5, 3] | 배달 L1 L2 L3 | 재검증 L1 L2 L3 | 출발, A안 = 12 개 다 싣고 배달).
  배달 · 재검증 순서는 실제로는 transfer_solver 가 정함 → --permute 로 묶음 안 순서를 다 바꿔 봄 (수집도 넣으면 720 배).
방문마다 면 하나 (같은 박스라도 방문마다 다른 면 가능), 같은 자리 연속이면 0 s. 성공률 < --min_success 구간은 못 씀.
비교: '지금 면' = dice_map.yaml 의 정차 면 (별칭, 정차 거리는 gap 2 cm 기준 0.2825 m 로 바뀐 자리), 이동 시간만 (정차 · 팔 시간 제외).
예) python3 rl/tasks/local_planner_nav2/best_faces.py rl/data/local_planner_nav2/leg_times_follower_navfn.json --permute 2,3
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parents[1] / 'data' / 'local_planner_nav2'

p = argparse.ArgumentParser()
p.add_argument('leg_times')
p.add_argument('--route', default='START | B1 B2 B6 B4 B5 B3 | L1 L2 L3 | L1 L2 L3 | START')
p.add_argument('--permute', default='', help='순서를 바꿔 볼 묶음 번호 (0 부터, 쉼표). 예 2,3 = 배달 · 재검증')
p.add_argument('--min_success', type=float, default=0.75)
p.add_argument('--top', type=int, default=5)
a = p.parse_args()

T = json.loads(Path(a.leg_times).read_text())
z = np.load(DATA / f'dice_legs_{T["paths"]}.npz')
alias = dict(x.split('=') for x in map(str, z['aliases']))
stops = [str(x) for x in z['stop_names']]
faces = {}
for s in stops:
    faces.setdefault(s.split('.')[0], []).append(s)
legs = {}
for k, v in T['legs'].items():
    s, g = k.split('->')
    legs[(s, g)] = v['time_s'] if (v['success'] >= a.min_success and v['time_s'] is not None) else math.inf


def t(s, g):
    return 0.0 if s == g else legs.get((s, g), math.inf)


def best(visits):
    """visits = 물건 이름 목록 → (최소 시간, 면 목록) DP"""
    F = [faces[v] for v in visits]
    cost = {f: 0.0 for f in F[0]}
    back = [{f: None for f in F[0]}]
    for i in range(1, len(F)):
        nc, nb = {}, {}
        for f in F[i]:
            c, pf = min(((cost[q] + t(q, f), q) for q in F[i - 1]), key=lambda x: x[0])
            nc[f], nb[f] = c, pf
        cost, back = nc, back + [nb]
    f = min(cost, key=cost.get)
    tot, seq = cost[f], [f]
    for i in range(len(F) - 1, 0, -1):
        f = back[i][f]
        seq.append(f)
    return tot, seq[::-1]


def fixed(visits):
    seq = [alias.get(v, v) for v in visits]
    return sum(t(x, y) for x, y in zip(seq, seq[1:])), seq


groups = [g.split() for g in a.route.split('|')]
perm_ids = [int(x) for x in a.permute.split(',') if x]
base = sum(groups, [])
tb, sb = fixed(base)
tf, sf = best(base)
print(f'구간 {len(legs)} 개 (성공률 ≥ {a.min_success}), 면 후보: ' + ' '.join(f'{k}:{len(v)}' for k, v in faces.items()))
print(f'\n[주어진 순서] {" ".join(base)}')
print(f'  지금 면   {tb:7.1f} s  {" ".join(sb)}')
print(f'  최적 면   {tf:7.1f} s  {" ".join(sf)}   (−{tb - tf:.1f} s)')

if perm_ids:
    cands = [list(itertools.permutations(g)) if i in perm_ids else [tuple(g)] for i, g in enumerate(groups)]
    rows = []
    for combo in itertools.product(*cands):
        v = [x for g in combo for x in g]
        tt, ss = best(v)
        rows.append((tt, fixed(v)[0], v, ss))
    rows.sort(key=lambda r: r[0])
    print(f'\n[묶음 {perm_ids} 순서까지 바꿈] {len(rows)} 가지, 상위 {a.top}')
    for tt, tfx, v, ss in rows[:a.top]:
        print(f'  {tt:7.1f} s (지금 면이면 {tfx:7.1f})  {" ".join(ss)}')
    print(f'  → 주어진 순서 · 지금 면 대비 −{tb - rows[0][0]:.1f} s')
used = [(x, y) for x, y in zip(sf, sf[1:]) if x != y]
print('\n최적 면 구간 시간: ' + ', '.join(f'{x}->{y} {t(x, y):.1f}' for x, y in used))
