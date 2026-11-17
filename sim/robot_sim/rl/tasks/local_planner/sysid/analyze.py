#!/usr/bin/env python3
"""record.py 결과 분석: 계단마다 지연 · 시간 상수 · 정상 비율 · 멈춤 거리 · 미끄럼, 최소 속도, 방향 전환 손실 → 학습 DR 범위 제안.
사용: python3 analyze.py ~/sysid/run1 [--plot]
정의 (계단의 주 축 = 명령이 가장 큰 축, odom 몸 속도 기준):
  지연 · 시간 상수  정상 속도의 10 % · 63 % 도달 시각으로 '순수 지연 + 1 차 지연' 을 맞춤 (τ = (t63 − t10) / 0.895)
  정상 비율  유지 구간 뒤 40 % 평균 / 목표
  멈춤      0 명령 → |속도| < 목표의 5 %, 그동안 간 거리
  미끄럼    map → base_footprint 이동 / odom 이동 (TF map 있을 때만, 회전은 yaw)
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def moved(P, ta, tb, rot):
    """자세 열 P (t, x, y, yaw) 의 ta → tb 이동 거리 (rot 면 |Δyaw|)"""
    if rot:
        y = np.unwrap(P[:, 3])
        return abs(float(np.interp(tb, P[:, 0], y) - np.interp(ta, P[:, 0], y)))
    return math.hypot(*(float(np.interp(tb, P[:, 0], P[:, i]) - np.interp(ta, P[:, 0], P[:, i])) for i in (1, 2)))


def first_cross(t, y, thr, t0):
    m = (t >= t0) & (y >= thr)
    return float(t[m][0]) if m.any() else math.nan


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run')
    p.add_argument('--plot', action='store_true')
    a = p.parse_args()
    run = Path(a.run).expanduser()
    d = np.load(run / 'data.npz')
    S = json.loads((run / 'segments.json').read_text())['segments']
    od = d['odom']                                  # t x y yaw vx vy wz
    tm = d['tf_map'] if d['tf_map'].size else np.zeros((0, 4))
    to = d['tf_odom'] if d['tf_odom'].size else np.zeros((0, 4))
    rows = []
    for s in S:
        cmd = np.array(s['cmd'])
        ax = int(np.argmax(np.abs(cmd)))
        tgt = cmd[ax]
        if abs(tgt) < 1e-9:
            continue
        sgn = math.copysign(1, tgt)
        m = (od[:, 0] >= s['t_start'] - 0.2) & (od[:, 0] <= s['t_end'])
        t, v = od[m, 0], od[m, 4 + ax] * sgn                     # 목표 방향 성분 (양수로)
        hold = s['t_zero'] - s['t_start']
        ss = (t >= s['t_zero'] - 0.4 * hold) & (t <= s['t_zero'])
        vss = float(v[ss].mean()) if ss.any() else math.nan
        ratio = vss / abs(tgt)
        # 정상 속도 기준 10 % · 63 % → 1 차 지연이면 t63 − t10 = τ (1 − ln(1/0.9)) = 0.895 τ, 순수 지연 = t10 − 0.105 τ
        t10 = first_cross(t, v, 0.10 * vss, s['t_start']) if vss > 0 else math.nan
        t63 = first_cross(t, v, 0.63 * vss, s['t_start']) if vss > 0 else math.nan
        tau = (t63 - t10) / 0.895
        delay = t10 - s['t_start'] - 0.105 * tau
        after = (t >= s['t_zero'])
        stopped = t[after & (np.abs(v) < 0.05 * abs(tgt))]
        t_stop = float(stopped[0] - s['t_zero']) if len(stopped) else math.nan
        # 멈춤 동안 간 거리 (odom 몸 속도 적분)
        k = after & (t <= (s['t_zero'] + (t_stop if math.isfinite(t_stop) else 2.0)))
        coast = float(np.sum(0.5 * (v[k][1:] + v[k][:-1]) * np.diff(t[k]))) if k.sum() > 1 else math.nan   # 사다리꼴 (시스템 numpy 1.x)
        # 미끄럼: 계단 전체에서 map 대 odom 이동 비 (TF 둘 다 있을 때)
        slip = math.nan
        if len(tm) > 2 and len(to) > 2:
            slip = moved(tm, s['t_start'], s['t_end'], ax == 2) / max(moved(to, s['t_start'], s['t_end'], ax == 2), 1e-9)
            if moved(to, s['t_start'], s['t_end'], ax == 2) < 1e-3:
                slip = math.nan
        rows.append(dict(name=s['name'], axis='xyw'[ax], target=float(tgt), delay_s=delay, tau_s=tau,
                         ratio=ratio, stop_s=t_stop, coast=coast, slip=slip))
    print(f"{'계단':16s} {'축':2s} {'목표':>6s} {'지연s':>6s} {'시정수s':>7s} {'정상비':>6s} {'멈춤s':>6s} {'멈춤거리':>8s} {'map/odom':>8s}")
    for r in rows:
        print(f"{r['name']:16s} {r['axis']:2s} {r['target']:+6.2f} {r['delay_s']:6.2f} {r['tau_s']:7.2f} {r['ratio']:6.2f} {r['stop_s']:6.2f} {r['coast']:8.3f} {r['slip']:8.3f}")
    f = lambda key, sel=lambda r: True: np.array([r[key] for r in rows if sel(r) and math.isfinite(r[key])])  # noqa: E731
    lin = lambda r: r['axis'] != 'w' and not r['name'].startswith('slow')  # noqa: E731
    slow = [r for r in rows if r['name'].startswith('slow') and r['target'] > 0]
    went = [r['target'] for r in slow if r['ratio'] > 0.5]
    print('\n## 학습 DR 제안 (cfg.py DRCfg)')
    for key, name in (('delay_s', '명령 → 움직임 지연 (cmd_delay + 조향 · 바퀴 지연)'), ('tau_s', '몸 속도 시간 상수 (tau_body · tau_wheel)')):
        x = f(key, lin)
        if len(x):
            print(f'  {name}: 중앙 {np.median(x):.3f} s, 범위 {x.min():.3f}–{x.max():.3f} s')
    x = f('ratio', lin)
    if len(x):
        print(f'  정상 비율 (odom / 명령): 중앙 {np.median(x):.3f}, 범위 {x.min():.3f}–{x.max():.3f}  (1 보다 작으면 컨트롤러 · 모터가 덜 냄)')
    x = f('slip', lin)
    if len(x):
        print(f'  미끄럼 (map / odom 이동): 중앙 {np.median(x):.3f}, 범위 {x.min():.3f}–{x.max():.3f}  → slip 범위')
    x = f('slip', lambda r: r['axis'] == 'w')
    if len(x):
        print(f'  회전 미끄럼 (map / odom yaw): 중앙 {np.median(x):.3f}')
    print(f"  최소 속도: {'0.01–0.03 m/s 전부 움직임' if len(went) == len(slow) else (f'{min(went):.2f} m/s 부터 움직임' if went else '0.03 m/s 까지 안 움직임')}"
          ' (못 움직이는 속도는 학습 플랜트에 데드밴드로 넣을 것)')
    t90 = [r for r in rows if r['name'] in ('turn90_b', 'turn90_back_b')]
    for r in t90:
        print(f"  방향 전환 90° ({r['name']}): 새 축 63 % 까지 {r['delay_s'] + r['tau_s']:.2f} s (지연 + 시간 상수)")
    (run / 'analysis.json').write_text(json.dumps(rows, indent=1, ensure_ascii=False))
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        cmd = d['cmd']
        fig, axs = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
        t0 = od[0, 0]
        for i, lab in enumerate(('vx', 'vy', 'wz')):
            axs[i].plot(cmd[:, 0] - t0, cmd[:, 1 + i], label='cmd', lw=1)
            axs[i].plot(od[:, 0] - t0, od[:, 4 + i], label='odom', lw=1)
            axs[i].set_ylabel(lab)
            axs[i].legend(loc='upper right')
        axs[-1].set_xlabel('t [s]')
        fig.savefig(run / 'response.png', dpi=110, bbox_inches='tight')
        print('그림 →', run / 'response.png')


if __name__ == '__main__':
    main()
