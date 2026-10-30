#!/usr/bin/env python3
"""무작위 미로 N 개 -> 시뮬레이션용 레이아웃 + USD (YOLO 데이터용: 대회 맵은 시연 1주 전 공개, 바깥 크기만 같음).

바깥벽 = 실제 A 경기장과 같은 크기 (바깥벽 중심선 ±2.202 × ±1.576 m, maze_real_layout.json 의 inner_half_m).
칸 수·칸 크기·통로는 미로마다 무작위:
  - 칸 nx × ny (기본 4–7 × 3–5) -> 칸 크기 = 바깥 크기 / 칸 수 (가장 작으면 약 0.63 m, 로봇 제자리 회전 약 0.40 m 보다 큼)
  - 통로: 모든 칸이 이어지게 무작위 깊이 우선 탐색으로 길을 뚫고(완전 미로), 남은 안쪽 벽을 p_extra 확률로 더 뚫음(순환 길·넓은 곳)
  - 벽판: 실물과 같은 두께 3 mm·높이 380 mm·아래 5 mm 띄움, 양 끝 7 mm 간격. 0.75 m 넘는 칸 변은 가운데 기둥으로 둘로 나눔 (실물 788 mm 와 같은 방식)
  - 기둥: 벽판이 닿는 격자점마다. 바깥 네 모서리 = 3030 알루미늄(D60 마운트), 나머지는 p_alu 확률로 3030, 아니면 R3 출력 기둥(R5 마운트)
가정: 벽판 길이를 칸 크기에 맞춰 자름 (실물 부품은 720 / 380 mm 고정 -> 대회 맵이 다른 칸 크기면 부품도 다를 것)
출력: mazes/<이름>_layout.json (maze_real_layout.json 과 같은 형식), usd/mazes/<이름>.usd
  시뮬에서 쓰기: 환경 변수 TN_MAZE=<이름> (turtleninja_maze_env.py), make_yolo_dataset.py --maze <이름>
실행: usd-core 가 있는 파이썬 (또는 Isaac Lab 파이썬) tools/random_maze.py --count 8 --seed 0
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from layout_to_maze_usd import (HERE, MOUNT_H, MOUNT_R, PANEL_H, PANEL_T, PANEL_Z0, POST_H, R5_H,  # noqa: E402
                                write_maze_usd)

REF = json.loads((HERE / 'maze_real_layout.json').read_text())
X0, Y0 = REF['inner_half_m']           # 바깥벽 중심선 (실제 A 경기장과 같음)
GAP = 0.007                            # 벽판 양 끝 ~ 기둥 중심 간격 (실물)
SPLIT_M = 0.75                         # 이보다 긴 칸 변은 가운데 기둥으로 나눔


def carve(nx, ny, p_extra, rng):
    """벽 집합 반환. 벽 = ('h', i, j): 칸 (i, j-1)·(i, j) 사이 가로벽 (j = 0..ny), ('v', i, j): 칸 (i-1, j)·(i, j) 사이 세로벽."""
    walls = {('h', i, j) for i in range(nx) for j in range(ny + 1)} | {('v', i, j) for i in range(nx + 1) for j in range(ny)}
    stack = [(int(rng.integers(nx)), int(rng.integers(ny)))]
    seen = {stack[0]}
    while stack:
        i, j = stack[-1]
        nb = [(i + di, j + dj, w) for di, dj, w in ((1, 0, ('v', i + 1, j)), (-1, 0, ('v', i, j)), (0, 1, ('h', i, j + 1)), (0, -1, ('h', i, j)))
              if 0 <= i + di < nx and 0 <= j + dj < ny and (i + di, j + dj) not in seen]
        if not nb:
            stack.pop()
            continue
        ii, jj, w = nb[int(rng.integers(len(nb)))]
        walls.discard(w)
        seen.add((ii, jj))
        stack.append((ii, jj))
    inner = [w for w in walls if (w[0] == 'h' and 0 < w[2] < ny) or (w[0] == 'v' and 0 < w[1] < nx)]
    for w in inner:
        if rng.random() < p_extra:
            walls.discard(w)
    return walls


def build(name, nx, ny, p_extra, p_alu, rng):
    cx, cy = 2 * X0 / nx, 2 * Y0 / ny
    node_xy = lambda i, j: (round(-X0 + i * cx, 4), round(Y0 - j * cy, 4))   # j = 0 이 북쪽
    walls = carve(nx, ny, p_extra, rng)
    cells = [{'i': i, 'j': j, 'xy': [round(-X0 + (i + 0.5) * cx, 4), round(Y0 - (j + 0.5) * cy, 4)], 'region': 1}
             for j in range(ny) for i in range(nx)]
    passable = []
    for j in range(ny):
        for i in range(nx):
            c = j * nx + i
            if i + 1 < nx and ('v', i + 1, j) not in walls:
                passable.append([c, c + 1])
            if j + 1 < ny and ('h', i, j + 1) not in walls:
                passable.append([c, c + nx])

    boxes, posts = [], {}                  # posts: 좌표 -> 이어진 벽 방향 집합 ('x' / 'y')
    for k, w in enumerate(sorted(walls)):
        kind, i, j = w
        a, b = (node_xy(i, j), node_xy(i + 1, j)) if kind == 'h' else (node_xy(i, j), node_xy(i, j + 1))
        n_seg = 2 if (cx if kind == 'h' else cy) > SPLIT_M else 1
        pts = [tuple(round(a[d] + (b[d] - a[d]) * t / n_seg, 4) for d in (0, 1)) for t in range(n_seg + 1)]
        for s in range(n_seg):
            p, q = pts[s], pts[s + 1]
            L = abs(q[0] - p[0]) + abs(q[1] - p[1]) - 2 * GAP
            ctr = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, PANEL_Z0 + PANEL_H / 2]
            boxes.append({'name': f'W{len(boxes) + 1:02d}', 'kind': 'panel', 'center': ctr,
                          'size': [L, PANEL_T, PANEL_H] if kind == 'h' else [PANEL_T, L, PANEL_H]})
        for p in pts:
            posts.setdefault(p, set()).add('x' if kind == 'h' else 'y')
    corners = {node_xy(0, 0), node_xy(nx, 0), node_xy(0, ny), node_xy(nx, ny)}
    mounts, n_alu = [], 0
    for k, (xy, dirs) in enumerate(sorted(posts.items())):
        alu = xy in corners or rng.random() < p_alu
        if alu:
            boxes.append({'name': f'post_{k}', 'kind': 'post_3030', 'center': [xy[0], xy[1], POST_H / 2], 'size': [0.030, 0.030, POST_H]})
            n_alu += 1
        else:
            along_x = dirs == {'x'} or (len(dirs) > 1 and rng.random() < 0.5)
            boxes.append({'name': f'post_{k}', 'kind': 'post_R3', 'center': [xy[0], xy[1], POST_H / 2],
                          'size': [0.036, 0.0095, POST_H] if along_x else [0.0095, 0.036, POST_H]})
        mtype = 'D60' if alu else 'R5'
        mounts.append({'node': f'N{k}', 'type': mtype, 'xy': list(xy), 'r': MOUNT_R, 'h': MOUNT_H,
                       'top_h': R5_H if mtype == 'R5' else MOUNT_H})
    n_panels = sum(b['kind'] == 'panel' for b in boxes)
    layout = {
        'source': f'tools/random_maze.py ({name}, 바깥 크기 = 실제 A 경기장)', 'units': 'm, Z-up, 미로 중심 = 원점, 바닥 z=0',
        'size_m': REF['size_m'], 'inner_half_m': [X0, Y0],
        'grid': {'nx': nx, 'ny': ny, 'cell_m': [round(cx, 4), round(cy, 4)], 'j0_is': 'north (+y)'},
        'regions': {'1': nx * ny}, 'cells': cells, 'passable_pairs': passable,
        'post_half_m': MOUNT_R, 'wall_thickness_m': PANEL_T, 'recommended_env_spacing_m': REF['recommended_env_spacing_m'],
        'collision_boxes': boxes, 'mounts': mounts,
        'counts': {'panels': n_panels, 'nodes': len(posts), 'posts_3030': n_alu, 'posts_R3': len(posts) - n_alu},
        'random': {'p_extra': round(p_extra, 3), 'p_alu': p_alu},
        'assumptions': '벽판 길이 = 칸 변 - 14 mm (실물 부품 길이 고정 아님), 벽판 아래 5 mm, R5 충돌 Ø60×25 mm 원통, R3 36×9.5 상자',
    }
    return layout


def ascii_map(L):
    nx, ny = L['grid']['nx'], L['grid']['ny']
    pas = {tuple(p) for p in L['passable_pairs']}
    out = []
    for j in range(ny):
        top, mid = '+', '|'
        for i in range(nx):
            c = j * nx + i
            top += ('   ' if j and (c - nx, c) in pas else '---') + '+'
            mid += '   ' + (' ' if i + 1 < nx and (c, c + 1) in pas else '|')
        out += [top, mid]
    out.append('+' + '---+' * nx)
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--prefix', default='rand')
    ap.add_argument('--nx', type=int, nargs=2, default=[4, 7], help='가로 칸 수 범위 (포함)')
    ap.add_argument('--ny', type=int, nargs=2, default=[3, 5], help='세로 칸 수 범위 (포함)')
    ap.add_argument('--p_extra', type=float, nargs=2, default=[0.1, 0.35], help='완전 미로에서 안쪽 벽을 더 뚫을 확률 범위')
    ap.add_argument('--p_alu', type=float, default=0.35, help='모서리 밖 기둥이 3030 알루미늄일 확률 (실물 A 경기장 16/45)')
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    (HERE / 'mazes').mkdir(exist_ok=True)
    (HERE / 'usd' / 'mazes').mkdir(parents=True, exist_ok=True)
    for k in range(a.count):
        name = f'{a.prefix}_{k:02d}'
        nx, ny = int(rng.integers(a.nx[0], a.nx[1] + 1)), int(rng.integers(a.ny[0], a.ny[1] + 1))
        L = build(name, nx, ny, float(rng.uniform(*a.p_extra)), a.p_alu, rng)
        (HERE / 'mazes' / f'{name}_layout.json').write_text(json.dumps(L, indent=1, ensure_ascii=False))
        write_maze_usd(HERE / 'usd' / 'mazes' / f'{name}.usd', L['collision_boxes'], L['mounts'])
        print(f"{name}: 칸 {nx}×{ny} ({L['grid']['cell_m'][0]:.3f} × {L['grid']['cell_m'][1]:.3f} m), 통로 {len(L['passable_pairs'])}, "
              f"벽판 {L['counts']['panels']}, 기둥 {L['counts']['nodes']} (3030 {L['counts']['posts_3030']})")
        print(ascii_map(L))


if __name__ == '__main__':
    main()
