#!/usr/bin/env python3
"""경로 모양 비교: make_nav2_paths.py 가 같은 쌍으로 만든 imitation · navfn · smac2d · smac2d_far (있는 것 전부) 은행 · dice 구간.

경로마다 (0.05 m 점, 지도 거리장 = 장애물 칸까지 중심 거리):
  길이 [m] / 최소 · 평균 여유 [m] (로봇 중심 ↔ 벽) / 벽 근처 비율 (여유 < 0.25 m = 팽창 반경 안 길이 비율)
  총 꺾임 [rad] (0.1 m 현 방향 변화 합) / 최대 꺾임 [°] (0.2 m 안 방향 변화) / 급한 모서리 수 (0.2 m 안 30° 넘는 곳, 0.3 m 간격으로 셈)
  흉내 차이 [cm] (imitation ↔ navfn 점 사이 평균 최근접 거리, 같은 쌍) = 학습 경로 ↔ 실제 NavFn 차이
요약 = 묶음 (학습 · 평가 · dice) 별 중앙값 · p90. navfn 대비 차이 (<이름>−navfn) 는 쌍마다 뺀 값의 중앙값.
예) python3 rl/tasks/local_planner_nav2/compare_paths.py [--dir rl/data/local_planner_nav2] [--plot dice_paths.png]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import make_path_bank as mpb  # noqa: E402

WALL = 0.25                                              # 팽창 반경 (config/nav2.yaml)

p = argparse.ArgumentParser()
p.add_argument('--dir', default=str(ROOT / 'rl' / 'data' / 'local_planner_nav2'))
p.add_argument('--plot', default='', help='dice 구간 경로 겹친 그림 (png)')
p.add_argument('--plot_kinds', default='', help='그림에 넣을 것 (쉼표, 기본 전부)')
a = p.parse_args()
D = Path(a.dir)
KINDS = ['imitation', 'navfn'] + sorted(f.stem[len('path_bank_'):] for f in D.glob('path_bank_*.npz')
                                      if f.stem[len('path_bank_'):] not in ('imitation', 'navfn'))
OTHERS = KINDS[2:]                                       # navfn 과 비교할 것
_maps = {}


def dist_field(name):
    if name not in _maps:
        occ, res, origin, _ = mpb.load_map(name)
        _maps[name] = (ndimage.distance_transform_edt(~occ) * res, res, origin)
    return _maps[name]


def heading(pts, chord=2):
    d = pts[chord:] - pts[:-chord]
    return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))


def metrics(pts, name):
    df, res, origin = dist_field(name)
    H, W = df.shape
    ci = np.clip(((pts - origin) / res - 0.5).round().astype(int), 0, [W - 1, H - 1])
    clr = df[ci[:, 1], ci[:, 0]]
    th = heading(pts)
    w = 4                                                # 0.2 m
    turn = np.abs(th[w:] - th[:-w]) if len(th) > w else np.zeros(1)
    sharp, last = 0, -1e9
    for i in np.nonzero(turn > np.radians(30))[0]:
        if (i - last) * mpb.STEP >= 0.3:
            sharp += 1
            last = i
    return dict(length=(len(pts) - 1) * mpb.STEP, clr_min=float(clr.min()), clr_mean=float(clr.mean()),
                near_wall=float((clr < WALL).mean()), turn_total=float(np.abs(np.diff(th)).sum()) if len(th) > 1 else 0.0,
                turn_max_deg=float(np.degrees(turn.max())), sharp=sharp)


def mean_gap(p, q):
    d = np.linalg.norm(p[:, None] - q[None], axis=-1)
    return 0.5 * (d.min(1).mean() + d.min(0).mean())


def load(path):
    z = np.load(path)
    names = [str(x) for x in z['names']]
    out = []
    for i in range(len(z['n'])):
        k = int(z['n'][i])
        out.append((z['pts'][i, :k] + z['start'][i], names[int(z['src'][i])], int(z['split'][i])))
    return out, (list(map(str, z['legs'])) if 'legs' in z else None)


def summarize(group, rows):
    res = {}
    for kind in KINDS:
        r = [x[kind] for x in rows]
        res[kind] = {k: {'median': float(np.median([x[k] for x in r])), 'p90': float(np.percentile([x[k] for x in r], 90))} for k in r[0]}
    diff = {o: {k: float(np.median([x[o][k] - x['navfn'][k] for x in rows])) for k in rows[0]['navfn']} for o in OTHERS}
    gap = [x['gap_cm'] for x in rows]
    res['minus_navfn_median'] = diff
    res['imitation_vs_navfn_gap_cm'] = {'median': float(np.median(gap)), 'p90': float(np.percentile(gap, 90)), 'max': float(np.max(gap))}
    print(f'\n[{group}] 경로 {len(rows)} 개  (중앙값 / p90)')
    print(f'  {"":10s}' + ''.join(f'{k:>18s}' for k in KINDS) + ''.join(f'{o + "−navfn":>18s}' for o in OTHERS))
    lab = {'length': '길이 m', 'clr_min': '최소 여유 m', 'clr_mean': '평균 여유 m', 'near_wall': '벽 근처 비율',
           'turn_total': '총 꺾임 rad', 'turn_max_deg': '최대 꺾임 °', 'sharp': '급한 모서리'}
    for k, l in lab.items():
        print(f'  {l:10s}' + ''.join(f'{res[kd][k]["median"]:9.3f} /{res[kd][k]["p90"]:7.3f}' for kd in KINDS) + ''.join(f'{diff[o][k]:+18.3f}' for o in OTHERS))
    print(f'  흉내 ↔ 실제 NavFn 평균 거리: 중앙 {res["imitation_vs_navfn_gap_cm"]["median"]:.1f} cm · p90 {res["imitation_vs_navfn_gap_cm"]["p90"]:.1f} · '
          f'최대 {res["imitation_vs_navfn_gap_cm"]["max"]:.1f}')
    return res


def main():
    report = {}
    for what in ('path_bank', 'dice_legs'):
        files = {k: D / f'{what}_{k}.npz' for k in KINDS}
        if not all(f.exists() for f in files.values()):
            print(f'{what}: 파일 없음, 건너뜀')
            continue
        data = {k: load(f) for k, f in files.items()}
        N = len(data['navfn'][0])
        assert all(len(data[k][0]) == N for k in KINDS), '은행끼리 경로 수가 다름 (같은 실행 결과인지)'
        rows = {}
        for i in range(N):
            name, split = data['navfn'][0][i][1], data['navfn'][0][i][2]
            row = {k: metrics(data[k][0][i][0], name) for k in KINDS}
            row['gap_cm'] = 100 * mean_gap(data['imitation'][0][i][0], data['navfn'][0][i][0])
            if data['navfn'][1] is not None:
                row['leg'] = data['navfn'][1][i]
            group = 'dice 구간' if what == 'dice_legs' else ('학습 지도' if split == 0 else '평가 지도')
            rows.setdefault(group, []).append(row)
        for g, r in rows.items():
            report[g] = summarize(g, r)
            if what == 'dice_legs':
                report[g]['legs'] = r
    st = D / 'stats.json'
    if st.exists():
        report['stats'] = json.loads(st.read_text())
        s = report['stats']
        print('\n[계획] ' + ' · '.join(f'{k} 중앙 {s[k]["plan_ms"]["median"]:.1f} ms (p95 {s[k]["plan_ms"]["p95"]:.1f}), 실패 {sum(s[k]["fail"].values())}, '
                                       f'dice 실패 {len(s[k]["dice_fail"])}' for k in KINDS[1:] if k in s))
        print(f'[쌍 뺌] {s["pair_reject"]}')
    (D / 'compare.json').write_text(json.dumps(report, indent=1, ensure_ascii=False))
    print(f'\n→ {D / "compare.json"}')

    if a.plot and (D / 'dice_legs_navfn.npz').exists():
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        df, res, origin = dist_field('dice')
        fig, ax = plt.subplots(figsize=(7, 9))
        ax.imshow(df <= 0, origin='lower', cmap='Greys', extent=[origin[0], origin[0] + df.shape[1] * res, origin[1], origin[1] + df.shape[0] * res])
        ax.contour(np.linspace(origin[0], origin[0] + df.shape[1] * res, df.shape[1]), np.linspace(origin[1], origin[1] + df.shape[0] * res, df.shape[0]),
                   df, levels=[0.17, WALL], colors=['tab:orange', 'khaki'], linewidths=0.5)
        pk = [k for k in KINDS if not a.plot_kinds or k in a.plot_kinds.split(',')]
        for kind, col in zip(pk, ('tab:gray', 'tab:blue', 'tab:red', 'tab:green', 'tab:purple')):
            for j, (pts, _, _) in enumerate(load(D / f'dice_legs_{kind}.npz')[0]):
                ax.plot(pts[:, 0], pts[:, 1], lw=0.7, color=col, alpha=0.6, label=kind if j == 0 else None)
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        ax.set_title('dice legs: ' + ' / '.join(pk) + ' (orange = inscribed 0.17 m, khaki = inflation 0.25 m)', fontsize=9)
        fig.savefig(a.plot, dpi=140, bbox_inches='tight')
        print('그림 →', a.plot)


if __name__ == '__main__':
    main()
