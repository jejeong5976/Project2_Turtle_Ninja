#!/usr/bin/env python3
"""정밀 접근 구간 → rl/data/local_dock/dock_legs_v2.npz (local_planner env 의 dice 구간 형식 + box_id · box_centers · mode)

목표 = dice 정차 면 22 곳 (local_planner_nav2 dice_legs_navfn.npz 의 stop_names, 갈 수 없는 면 뺀 것).
  정차 거리 (박스 중심 → 로봇 중심) = 구간마다 0.2525–0.2675 m. 박스 · 장소 둘 다 0.2575 (2026-11-06 결정: 먼 줄 PLACE 손 30° 다시 쥐기가
  0.2575 에서만 닿음, 차체 앞 끝 0.145 m (바퀴 포함) ↔ 박스 면 1 cm). 범위 = 차체 틈 0.5–2 cm
출발 (정차점 기준, 박스 반대쪽 거리 d · 옆 lat · yaw 오차) 세 모드:
  pre   35 %: d 0.30–0.45 m, lat ±3 cm, yaw ±10°  — Nav2 에서 넘겨받는 자리 (정차점 0.40 m 안에서 넘김, 2026-11-06)
  fix   35 %: d −1–4 cm, lat ±3 cm, yaw ±5°       — 정차 뒤 측정해 몇 cm 고치기 (env 박스 밀림 ±6 cm · ±10° 와 겹쳐 짧은 이동)
  broad 30 %: d 0.03–0.15 (반) · 0.15–0.8 m (반), lat N(0, 5 cm) ±15 cm, yaw ±10° (반) · ±60° (반)  — MPPI 도착 자리 등
경로 = 출발 → 정차점 직선 (0.05 m 점). env box_dynamic 이면 리셋 때 박스를 밀고 목표 · 경로를 다시 만듦 (박스 번호 box_id)
버림: 출발 차체 (±0.145) 여유 < 0.5 cm, 직선 경로 중심이 벽 · 박스와 0.17 m 안 (정차점 0.15 m 안은 0.145). 목표 박스는 해석식 (지도 래스터 아님).
예) python3 rl/tasks/local_dock/make_dock_legs.py [--per_stop 300]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]                                   # sim/robot_sim
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import make_path_bank as mpb  # noqa: E402

OX, OY = 2.202, 3.152                                    # 경기장 → 지도
BOX_HALF = 0.1025
BODY_HALF = 0.145                                       # 실제 차체 앞 끝 (바퀴 포함) = dock 충돌 판정
SIDES = {'N': (0.0, 1.0), 'E': (1.0, 0.0), 'S': (0.0, -1.0), 'W': (-1.0, 0.0)}
MODES = ('pre', 'fix', 'broad')

p = argparse.ArgumentParser()
p.add_argument('--stops', default=str(ROOT / 'rl' / 'data' / 'local_planner_nav2' / 'dice_legs_navfn.npz'))
p.add_argument('--out', default=str(ROOT / 'rl' / 'data' / 'local_dock' / 'dock_legs_v2.npz'))
p.add_argument('--per_stop', type=int, default=300)
p.add_argument('--stop', type=float, nargs=2, default=(0.2525, 0.2675), help='정차 거리 범위 (박스 중심 → 로봇 중심) [m]')
p.add_argument('--seed', type=int, default=20261106)
a = p.parse_args()
rng = np.random.default_rng(a.seed)

m = yaml.safe_load((REPO / 'maps' / 'dice_map.yaml').read_text())
box_names = list(m['boxes']) + list(m['places'])
box_centers = np.array([[v['center'][0] - OX, v['center'][1] - OY] for sec in ('boxes', 'places') for v in m[sec].values()])
faces = [str(nm) for nm in np.load(a.stops)['stop_names'] if str(nm) != 'START']
occ, res, origin, _ = mpb.load_map('dice')
df = ndimage.distance_transform_edt(~occ) * res - res / 2
H, W = df.shape


ys_, xs_ = np.mgrid[0:H, 0:W]
WX, WY = origin[0] + (xs_ + 0.5) * res, origin[1] + (ys_ + 0.5) * res
MINUS = []                                               # 박스마다 그 박스 칸을 지운 거리장 (env World.enable_boxes 와 같음)
for cx_, cy_ in box_centers:
    o_ = occ & ~((np.abs(WX - cx_) <= BOX_HALF + res) & (np.abs(WY - cy_) <= BOX_HALF + res))
    MINUS.append(ndimage.distance_transform_edt(~o_) * res - res / 2)


def dist(xy, bi=None):
    """지도 거리. bi 주면 그 박스는 래스터 대신 해석식 (지도 래스터는 박스가 E · N 쪽으로 약 1.7 cm 큼)"""
    xy = np.asarray(xy)
    f = df if bi is None else MINUS[bi]
    ij = np.floor((xy - origin) / res).astype(int)
    ok = (ij[..., 0] >= 0) & (ij[..., 0] < W) & (ij[..., 1] >= 0) & (ij[..., 1] < H)
    out = np.full(ij.shape[:-1], -1.0)
    out[ok] = f[ij[ok][..., 1], ij[ok][..., 0]]
    if bi is not None:
        q = np.abs(xy - box_centers[bi]) - BOX_HALF
        out = np.minimum(out, np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0))
    return out


def fp_clear(x, y, yaw, half=BODY_HALF, n=12, bi=None):
    t = np.linspace(-half, half, n)
    e = np.full(n, half)
    pts = np.concatenate([np.stack([e, t], 1), np.stack([-e, t], 1), np.stack([t, e], 1), np.stack([t, -e], 1)])
    c, s = math.cos(yaw), math.sin(yaw)
    return float(dist(np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1), bi).min())


def sample_start(mode):
    if mode == 'pre':
        return rng.uniform(0.30, 0.45), rng.uniform(-0.03, 0.03), math.radians(rng.uniform(-10, 10))
    if mode == 'fix':
        return rng.uniform(-0.01, 0.04), rng.uniform(-0.03, 0.03), math.radians(rng.uniform(-5, 5))
    d = rng.uniform(0.03, 0.15) if rng.random() < 0.5 else rng.uniform(0.15, 0.8)
    lat = float(np.clip(rng.normal(0, 0.05), -0.15, 0.15))
    return d, lat, math.radians(rng.uniform(-10, 10) if rng.random() < 0.5 else rng.uniform(-60, 60))


items, drop = [], {'start': 0, 'path': 0}
for name in faces:
    bname, side = name.split('.')
    bi = box_names.index(bname)
    cx, cy = box_centers[bi]
    dx, dy = SIDES[side]
    gyaw = math.atan2(-dy, -dx)                                       # 박스를 보는 방향
    fwd = np.array([math.cos(gyaw), math.sin(gyaw)])
    left = np.array([-fwd[1], fwd[0]])
    got, tries = 0, 0
    while got < a.per_stop and tries < a.per_stop * 30:
        tries += 1
        mode = MODES[int(rng.choice(3, p=[0.35, 0.35, 0.30]))]
        D = rng.uniform(*a.stop)
        gap = D - BOX_HALF - BODY_HALF                                # 차체 앞 끝 ↔ 박스 면
        goal = np.array([cx + dx * D, cy + dy * D])
        d, lat, e = sample_start(mode)
        sxy = goal - d * fwd + lat * left
        syaw = gyaw + e
        if fp_clear(sxy[0], sxy[1], syaw, bi=bi) < 0.005:
            drop['start'] += 1
            continue
        pts = mpb.resample(np.stack([sxy, goal]))
        if len(pts) < 2:
            pts = np.stack([sxy, goal])
        near = np.linalg.norm(pts - goal, axis=1) < 0.15                 # 정차점 근처는 박스 면이 0.15–0.165 m (내접 반경보다 가까움)
        if (dist(pts, bi) < np.where(near, 0.145, 0.17)).any():
            drop['path'] += 1
            continue
        items.append((f'{name}#{got}', pts, gyaw, syaw, bi, MODES.index(mode), gap))
        got += 1
    print(f'  {name}: {got} 개 (시도 {tries})')

N = len(items)
names = mpb.TRAIN_MAPS + mpb.EVAL_MAPS
P = np.zeros((N, mpb.MAX_PTS, 2), np.float32)
n = np.zeros(N, np.int32)
out = dict(goal_yaw=np.zeros(N, np.float32), start_yaw=np.zeros(N, np.float32), start=np.zeros((N, 2), np.float32),
           box_id=np.zeros(N, np.int16), mode=np.zeros(N, np.int8), gap=np.zeros(N, np.float32))
for i, (_, pts, gy, sy, bi, md, gp) in enumerate(items):
    k = len(pts)
    P[i, :k] = pts - pts[0]
    P[i, k:] = pts[-1] - pts[0]
    n[i] = k
    out['goal_yaw'][i], out['start_yaw'][i], out['start'][i] = gy, sy, pts[0]
    out['box_id'][i], out['mode'][i], out['gap'][i] = bi, md, gp
dst = Path(a.out)
dst.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(dst, pts=P, n=n, level=np.ones(N, np.int8), split=np.zeros(N, np.int8),
                    src=np.full(N, names.index('dice'), np.int16), names=np.array(names),
                    legs=np.array([it[0] for it in items]), box_centers=box_centers.astype(np.float32), box_half=np.float32(BOX_HALF),
                    box_names=np.array(box_names), modes=np.array(MODES), **out)
L = (n - 1) * mpb.STEP
print(f'접근 구간 {N} 개 (정차 면 {len(faces)} 곳), 모드 ' + ' · '.join(f'{md} {int((out["mode"] == i).sum())}' for i, md in enumerate(MODES))
      + f', 길이 중앙 {np.median(L):.2f} m · 최대 {L.max():.2f}, 정차 {a.stop[0]:.4f}–{a.stop[1]:.4f} m (차체 틈 {np.min(out["gap"]) * 100:.1f}–{np.max(out["gap"]) * 100:.1f} cm), 버림 {drop} → {dst}')
