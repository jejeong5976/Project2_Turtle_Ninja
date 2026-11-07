#!/usr/bin/env python3
"""dice 경기장 정차점끼리 구간 (지역 계획기 RL 학습 · 평가용) → rl/data/local_planner/dice_legs.npz

정차점 = 출발점 · 박스 B1–B6 · 장소 L1–L3 (레포 maps/dice_map.yaml 의 start · stop, 경기장 m → 지도 좌표 = 경기장 − (2.202, 3.152)).
지도 = rl/data/maps/maze_dice_map (tools/layout_to_occupancy.py --maze dice --out_dir rl/data/maps, mazes/dice_layout.json 을 바꾸면 다시 만들 것).
경로 = make_path_bank 와 같은 NavFn 흉내 (비용지도 → Dijkstra 퍼텐셜 → 경사 하강 → 0.05 m 재표본).
저장 형식 = path_bank.npz 와 같음 (pts 는 시작 = 원점 기준) + start_yaw (출발 정차점 yaw) + legs (구간 이름).
예) python3 rl/tasks/local_planner/make_dice_legs.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.sparse.csgraph import dijkstra

ROOT = Path(__file__).resolve().parents[3]          # sim/robot_sim
sys.path.insert(0, str(ROOT / 'tools'))
import make_path_bank as mpb  # noqa: E402

OX, OY = 2.202, 3.152                    # 경기장 → 지도 (경기장 중심 원점)
DICE_MAP = ROOT.parents[1] / 'maps' / 'dice_map.yaml'   # 레포 maps/dice_map.yaml (planning 이 읽는 정차 자세)


def load_stops():
    """출발 · 박스 · 장소 정차 자세 (경기장 m, rad)"""
    m = yaml.safe_load(DICE_MAP.read_text())
    stops = {'START': tuple(m['start'])}
    for sec in ('boxes', 'places'):
        stops.update({k: tuple(v['stop']) for k, v in m[sec].items()})
    return stops


def main():
    STOPS = load_stops()
    occ, res, origin, _ = mpb.load_map('dice')
    cost = mpb.costmap(occ, res)
    H, W = cost.shape
    G = mpb.build_graph(cost)
    cell = {k: (int(round((y - OY - origin[1]) / res - 0.5)), int(round((x - OX - origin[0]) / res - 0.5))) for k, (x, y, _) in STOPS.items()}
    for k, (r, c) in cell.items():
        if cost[r, c] >= 253:
            print(f'경고: {k} 정차점 칸 비용 {cost[r, c]:.0f} (내접 영역)')
    items = []
    for gk, (gx, gy, gyaw) in STOPS.items():
        pot = dijkstra(G, indices=cell[gk][0] * W + cell[gk][1]).reshape(H, W)
        for sk, (sx, sy, syaw) in STOPS.items():
            if sk == gk:
                continue
            cells = mpb.descend(pot, cell[sk], cell[gk])
            if cells is None:
                print(f'  {sk}->{gk}: 경로 없음')
                continue
            xy = np.stack([origin[0] + (cells[:, 1] + 0.5) * res, origin[1] + (cells[:, 0] + 0.5) * res], 1)
            xy[0], xy[-1] = (sx - OX, sy - OY), (gx - OX, gy - OY)          # 칸 중심 대신 정확한 정차점
            pts = mpb.resample(xy)
            if len(pts) > mpb.MAX_PTS:
                print(f'  {sk}->{gk}: 점 {len(pts)} 개 > {mpb.MAX_PTS}, 뺌')
                continue
            items.append((f'{sk}->{gk}', pts, gyaw, syaw))
    N = len(items)
    P = np.zeros((N, mpb.MAX_PTS, 2), np.float32)
    n = np.zeros(N, np.int32)
    out = dict(goal_yaw=np.zeros(N, np.float32), start_yaw=np.zeros(N, np.float32), start=np.zeros((N, 2), np.float32))
    for i, (_, pts, gyaw, syaw) in enumerate(items):
        k = len(pts)
        P[i, :k] = pts - pts[0]
        P[i, k:] = pts[-1] - pts[0]
        n[i] = k
        out['goal_yaw'][i], out['start_yaw'][i], out['start'][i] = gyaw, syaw, pts[0]
    dst = ROOT / 'rl' / 'data' / 'local_planner' / 'dice_legs.npz'
    dst.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(dst, pts=P, n=n, level=np.ones(N, np.int8), split=np.zeros(N, np.int8),
                        src=np.full(N, (mpb.TRAIN_MAPS + mpb.EVAL_MAPS).index('dice'), np.int16),
                        names=np.array(mpb.TRAIN_MAPS + mpb.EVAL_MAPS), legs=np.array([it[0] for it in items]), **out)
    L = (n - 1) * mpb.STEP
    print(f'구간 {N} 개, 길이 중앙 {np.median(L):.2f} m · 최소 {L.min():.2f} · 최대 {L.max():.2f} → {dst}')
    for name, l in zip([it[0] for it in items], L):
        if name.startswith('START->') or name.endswith('->START'):
            print(f'  {name:12s} {l:.2f} m')


if __name__ == '__main__':
    main()
