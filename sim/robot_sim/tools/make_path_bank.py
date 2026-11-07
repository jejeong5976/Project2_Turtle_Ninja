#!/usr/bin/env python3
"""스워브 경로 추종 RL 용 경로 은행 → rl/data/path_bank.npz

(a) 지도 경로 (NavFn 흉내): 미로 배치 → tools/layout_to_occupancy.py 점유 격자 (0.025 m) →
    Nav2 식 비용지도 (config/nav2.yaml global_costmap: footprint ±0.16 + padding 0.01 → 내접 반경 0.17,
    inflation_radius 0.25 · cost_scaling_factor 10. 장애물 254 · 내접 안 253 · 그 밖 252·exp(−10·(d−0.17))) →
    NavFn 식 칸 비용 (neutral 50 + 0.8 × 비용, 253 이상은 막힘) Dijkstra 퍼텐셜 (8 이웃) → 퍼텐셜 경사 하강 (반 칸 걸음) →
    0.05 m 재표본 → 경로 점 모두 비용 < 253 (중심이 내접 영역 밖) 확인.
    실제 Nav2 NavFn 과의 비교는 아직 안 함 (sim Nav2 로 확인할 것).
(b) 합성 경로: 꺾은선 1–4 구간 + 모서리 원호, U턴 포함. 난이도 등급 0 (≤45°, ≤2 m) / 1 (≤135°) / 2 (U턴).

지도 나눔 (학습 · 평가 분리): 학습 = real + rand_00..06, 평가 = dice + rand_07. 합성 경로는 시드 다르게 학습 · 평가 둘 다.
저장: pts (N,256,2) 경로 시작 = 원점 기준 [m], 256 점 못 채우면 마지막 점 반복 / n (N,) 점 수 / goal_yaw (N,) 지도 좌표 기준 /
      start (N,2) 지도 좌표 시작점 /
      level (N,) / split (N,) 0 학습 · 1 평가 / src (N,) 지도 이름 번호 (−1 = 합성) / names.
예) env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python tools/make_path_bank.py
"""
import argparse
import heapq  # noqa: F401  (scipy 없을 때 대비 표시용)
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

ROOT = Path(__file__).resolve().parents[1]
STEP = 0.05
MAX_PTS = 256
INSCRIBED, INFL_R, SCALE = 0.17, 0.25, 10.0
TRAIN_MAPS = ['real'] + [f'rand_{i:02d}' for i in range(7)]
EVAL_MAPS = ['dice', 'rand_07']

p = argparse.ArgumentParser()
p.add_argument('--out', default=str(ROOT / 'rl' / 'data' / 'path_bank.npz'))
p.add_argument('--per_map', type=int, default=600, help='지도 하나당 경로 수')
p.add_argument('--goals', type=int, default=60, help='지도 하나당 목표 수 (목표마다 Dijkstra 한 번)')
p.add_argument('--synthetic', type=int, default=3000, help='합성 경로 수 (학습). 평가는 1/5')
p.add_argument('--seed', type=int, default=20261030)
a = p.parse_args() if __name__ == '__main__' else p.parse_args([])   # 다른 도구가 함수만 가져다 쓸 때는 인자 안 읽음
rng = np.random.default_rng(a.seed)


# ---------------------------------------------------------------- 공통
def resample(xy, step=STEP):
    """꺾은선 → 호 길이 step 간격 점 (끝점 포함). 0 길이 선분 · 중복점 제거"""
    xy = np.asarray(xy, float)
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    keep = np.concatenate([[True], d > 1e-9])
    xy = xy[keep]
    if len(xy) < 2:
        return xy
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    n = int(math.floor(s[-1] / step))
    ss = np.concatenate([np.arange(n + 1) * step, [s[-1]]]) if s[-1] - n * step > 1e-6 else np.arange(n + 1) * step
    return np.stack([np.interp(ss, s, xy[:, 0]), np.interp(ss, s, xy[:, 1])], 1)


def end_tangent_yaw(pts):
    d = pts[-1] - pts[max(0, len(pts) - 5)]
    return math.atan2(d[1], d[0])


# ---------------------------------------------------------------- (a) 지도 경로
def load_map(name):
    out_dir = ROOT / 'rl' / 'data' / 'maps'
    stem = 'maze_real_map' if name == 'real' else f'maze_{name}_map'
    if not (out_dir / f'{stem}.pgm').exists():
        subprocess.run([sys.executable, str(ROOT / 'tools' / 'layout_to_occupancy.py'), '--maze', name, '--out_dir', str(out_dir)], check=True)
    raw = (out_dir / f'{stem}.pgm').read_bytes()
    parts = raw.split(b'\n', 4)          # P5 / 주석 / W H / 255 / 데이터
    W, H = map(int, parts[2].split())
    img = np.frombuffer(parts[4], np.uint8)[:W * H].reshape(H, W)[::-1]   # 행 0 = 아래 (y 최소)
    meta = {k.strip(): v.strip() for k, v in (l.split(':', 1) for l in (out_dir / f'{stem}.yaml').read_text().splitlines()
                                             if ':' in l and not l.startswith('#'))}
    res = float(meta['resolution'])
    origin = json.loads(meta['origin'])[:2]
    layout = json.loads(((ROOT / 'maze_real_layout.json') if name == 'real' else (ROOT / 'mazes' / f'{name}_layout.json')).read_text())
    return img == 0, res, np.array(origin), layout


def costmap(occ, res):
    d = ndimage.distance_transform_edt(~occ) * res
    c = np.where(d <= INSCRIBED, 253.0, np.where(d <= INFL_R, 252.0 * np.exp(-SCALE * (d - INSCRIBED)), 0.0))
    c[occ] = 254.0
    return c


def build_graph(cost):
    H, W = cost.shape
    free = cost < 253
    cell = 50.0 + 0.8 * cost
    idx = np.arange(H * W).reshape(H, W)
    rows, cols, wts = [], [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        y0, y1 = max(0, -dy), H - max(0, dy)
        x0, x1 = max(0, -dx), W - max(0, dx)
        A = (slice(y0, y1), slice(x0, x1))
        Bs = (slice(y0 + dy, y1 + dy), slice(x0 + dx, x1 + dx))
        ok = free[A] & free[Bs]
        if dx != 0 and dy != 0:   # 대각선은 양옆 칸도 비어야 함 (모서리 통과 금지)
            ok &= free[y0 + dy:y1 + dy, x0:x1] & free[y0:y1, x0 + dx:x1 + dx]
        w = 0.5 * (cell[A] + cell[Bs]) * math.hypot(dx, dy)
        rows.append(idx[A][ok]); cols.append(idx[Bs][ok]); wts.append(w[ok])
    r, c, w = np.concatenate(rows), np.concatenate(cols), np.concatenate(wts)
    return coo_matrix((np.concatenate([w, w]), (np.concatenate([r, c]), np.concatenate([c, r]))), shape=(H * W, H * W)).tocsr()


def descend(pot, start, goal, max_iter=20000):
    """퍼텐셜 경사 하강 (NavFn gradient path 비슷하게): 반 칸 걸음, 막히면 8 이웃 최솟값으로"""
    H, W = pot.shape
    finite = np.isfinite(pot)
    big = np.nanmax(np.where(finite, pot, np.nan)) * 1.5
    P = np.where(finite, pot, big)
    gy, gx = np.gradient(P)
    p = np.array(start, float)        # (행, 열)
    path = [p.copy()]
    for _ in range(max_iter):
        if np.hypot(*(p - goal)) < 1.0:
            path.append(np.array(goal, float))
            return np.array(path)
        r, c = int(round(p[0])), int(round(p[1]))
        r0, c0 = int(math.floor(p[0])), int(math.floor(p[1]))
        fr, fc = p[0] - r0, p[1] - c0
        r1, c1 = min(r0 + 1, H - 1), min(c0 + 1, W - 1)

        def bil(G):
            return (G[r0, c0] * (1 - fr) * (1 - fc) + G[r1, c0] * fr * (1 - fc) + G[r0, c1] * (1 - fr) * fc + G[r1, c1] * fr * fc)
        g = np.array([bil(gy), bil(gx)])
        n = np.linalg.norm(g)
        nb = [P[r0, c0], P[r1, c0], P[r0, c1], P[r1, c1]]
        if n < 1e-9 or not all(np.isfinite(pot[rr, cc]) for rr, cc in ((r0, c0), (r1, c0), (r0, c1), (r1, c1))) or max(nb) >= big:
            best, bp = P[r, c], None    # 장애물 근처 · 평평: 이웃 칸 최솟값으로
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W and P[rr, cc] < best:
                        best, bp = P[rr, cc], (rr, cc)
            if bp is None:
                return None
            p = np.array(bp, float)
        else:
            p = p - 0.5 * g / n
        path.append(p.copy())
    return None


def map_paths(name, n_paths, n_goals, split):
    occ, res, origin, layout = load_map(name)
    cost = costmap(occ, res)
    H, W = cost.shape
    sx, sy = layout['size_m'][:2]
    ys, xs = np.mgrid[0:H, 0:W]
    wx, wy = origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res
    inside = (np.abs(wx) < sx / 2 - 0.05) & (np.abs(wy) < sy / 2 - 0.05)
    cand = np.argwhere((cost < 128) & inside)     # 로봇이 설 만한 칸 (내접 영역에서 조금 떨어짐)
    G = build_graph(cost)
    out = []
    per_goal = int(math.ceil(n_paths / n_goals))
    tries = 0
    while len(out) < n_paths and tries < n_goals * 4:
        tries += 1
        g = cand[rng.integers(len(cand))]
        pot = dijkstra(G, indices=g[0] * W + g[1]).reshape(H, W)
        starts = cand[rng.choice(len(cand), per_goal * 3, replace=False)]
        got = 0
        for s in starts:
            if got >= per_goal or len(out) >= n_paths:
                break
            if not np.isfinite(pot[s[0], s[1]]):
                continue
            cells = descend(pot, s, g)
            if cells is None:
                continue
            xy = np.stack([origin[0] + (cells[:, 1] + 0.5) * res, origin[1] + (cells[:, 0] + 0.5) * res], 1)
            pts = resample(xy)
            L = (len(pts) - 1) * STEP
            if len(pts) < 7 or len(pts) > MAX_PTS:
                continue
            ci = np.clip(((pts - origin) / res - 0.5).round().astype(int), 0, [W - 1, H - 1])
            if (cost[ci[:, 1], ci[:, 0]] >= 253).any():
                continue
            gy_ = end_tangent_yaw(pts) if rng.random() < 0.5 else rng.uniform(-math.pi, math.pi)
            out.append((pts, gy_, 1 if L <= 4.0 else 2, split))
            got += 1
    print(f'  {name}: 경로 {len(out)} 개 (목표 {tries} 번)', flush=True)
    return out


# ---------------------------------------------------------------- (b) 합성 경로
def fillet_polyline(corners, radius):
    """꺾은선 모서리를 원호로 (반경은 이웃 구간 길이의 45 % 까지 줄임)"""
    pts = [corners[0]]
    for i in range(1, len(corners) - 1):
        P0, P, P1 = corners[i - 1], corners[i], corners[i + 1]
        d1, d2 = P - P0, P1 - P
        l1, l2 = np.linalg.norm(d1), np.linalg.norm(d2)
        u1, u2 = d1 / l1, d2 / l2
        phi = math.acos(max(-1.0, min(1.0, float(u1 @ u2))))   # 꺾임각
        if phi < 1e-3 or radius <= 0:
            pts.append(P)
            continue
        t = radius * math.tan(phi / 2)
        tmax = 0.45 * min(l1, l2)
        r = radius if t <= tmax else tmax / math.tan(phi / 2)
        t = r * math.tan(phi / 2)
        A, Bp = P - u1 * t, P + u2 * t
        side = np.sign(u1[0] * u2[1] - u1[1] * u2[0])
        nrm = np.array([-u1[1], u1[0]]) * side
        C = A + nrm * r
        a0 = math.atan2(A[1] - C[1], A[0] - C[0])
        k = max(2, int(math.ceil(phi * r / 0.01)))
        for j in range(k + 1):
            ang = a0 + side * phi * j / k
            pts.append(C + r * np.array([math.cos(ang), math.sin(ang)]))
    pts.append(corners[-1])
    return np.array(pts)


def synthetic(level):
    if level == 0:
        n_seg, turns, seg_len, max_len = rng.integers(1, 3), [0, 15, 30, 45], (0.3, 1.2), 2.0
    elif level == 1:
        n_seg, turns, seg_len, max_len = rng.integers(1, 5), [0, 45, 90, 135], (0.2, 2.0), 6.0
    else:
        n_seg, turns, seg_len, max_len = rng.integers(2, 5), [90, 135, 180], (0.2, 2.0), 8.0
    heading = rng.uniform(-math.pi, math.pi)
    corners = [np.zeros(2)]
    for i in range(n_seg):
        if i > 0:
            t = rng.choice(turns)
            if t == 180:   # U턴 = 같은 쪽 90° 두 번 + 짧은 옆 구간
                side = rng.choice([-1, 1])
                heading += side * math.pi / 2
                corners.append(corners[-1] + rng.uniform(0.15, 0.6) * np.array([math.cos(heading), math.sin(heading)]))
                heading += side * math.pi / 2
            else:
                heading += math.radians(t) * rng.choice([-1, 1])
        corners.append(corners[-1] + rng.uniform(*seg_len) * np.array([math.cos(heading), math.sin(heading)]))
    radius = rng.choice([0.0, rng.uniform(0.05, 0.3)])
    pts = resample(fillet_polyline(np.array(corners), radius))
    pts = pts[: int(max_len / STEP) + 1]
    gy_ = end_tangent_yaw(pts) if rng.random() < 0.5 else rng.uniform(-math.pi, math.pi)
    return pts, gy_


# ---------------------------------------------------------------- 실행
def main():
    items, srcs, names = [], [], TRAIN_MAPS + EVAL_MAPS
    for name in TRAIN_MAPS + EVAL_MAPS:
        split = 0 if name in TRAIN_MAPS else 1
        for it in map_paths(name, a.per_map if split == 0 else a.per_map // 2, a.goals, split):
            items.append(it); srcs.append(names.index(name))
    for split, n in ((0, a.synthetic), (1, a.synthetic // 5)):
        for i in range(n):
            lv = (0, 0, 1, 1, 1, 2)[i % 6]
            pts, gy_ = synthetic(lv)
            if len(pts) >= 3:
                items.append((pts, gy_, lv, split)); srcs.append(-1)

    N = len(items)
    P = np.zeros((N, MAX_PTS, 2), np.float32)
    n = np.zeros(N, np.int32)
    gyaw = np.zeros(N, np.float32)
    lvl = np.zeros(N, np.int8)
    start = np.zeros((N, 2), np.float32)
    spl = np.zeros(N, np.int8)
    for i, (pts, gy_, lv, sp) in enumerate(items):
        start[i] = pts[0]                      # 지도 좌표 시작점 (그림용)
        pts = pts - pts[0]                     # 시작 = 원점 (방향은 지도 그대로, env 가 무작위 회전)
        k = len(pts)
        P[i, :k] = pts
        P[i, k:] = pts[-1]
        n[i], gyaw[i], lvl[i], spl[i] = k, gy_, lv, sp
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, pts=P, n=n, goal_yaw=gyaw, start=start, level=lvl, split=spl, src=np.array(srcs, np.int16), names=np.array(names))
    L = (n - 1) * STEP
    for sp in (0, 1):
        m = spl == sp
        print(f'{"학습" if sp == 0 else "평가"}: {m.sum()} 개, 등급 {np.bincount(lvl[m], minlength=3).tolist()}, '
              f'길이 중앙 {np.median(L[m]):.2f} m · 최대 {L[m].max():.2f} m, 지도 경로 {(np.array(srcs)[m] >= 0).sum()}')
    print(f'→ {out}')


if __name__ == '__main__':
    main()
