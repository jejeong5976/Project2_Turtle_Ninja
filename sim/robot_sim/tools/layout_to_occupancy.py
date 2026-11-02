#!/usr/bin/env python3
"""미로 배치 파일(*_layout.json) -> Nav2 지도 (PGM + YAML). 시뮬 Nav2 시험용 (실물은 Cartographer 지도를 씀).

라이다 스캔 높이(배포판 URDF base_scan_front, 바닥 위 60 mm)에서 자른 단면을 그림:
  collision_boxes(벽판 · 기둥)는 z 범위가 스캔 높이를 지나면 점유, mounts(벽 연결 부품 원통)는 윗면이 스캔 높이보다 높으면 점유.
지도 좌표 = 미로 좌표 (미로 중심 원점, 시뮬 env 원점과 같음) -> 시뮬 정답 위치를 그대로 map 좌표로 씀.

예) python3 tools/layout_to_occupancy.py                 # 실제 A 경기장 -> maps/maze_real_map.{pgm,yaml}
    python3 tools/layout_to_occupancy.py --maze rand_03  # 무작위 미로
    python3 tools/layout_to_occupancy.py --maze dice     # 발제 v5 주사위 경기장 (박스 · 받침대 · 신호등 · 바코드 기둥 포함)
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]   # sim/robot_sim

p = argparse.ArgumentParser()
p.add_argument('--maze', default='real', help='real = maze_real_layout.json, 그 밖 = mazes/<이름>_layout.json')
p.add_argument('--res', type=float, default=0.025, help='지도 해상도 [m/칸]')
p.add_argument('--scan_z', type=float, default=0.060, help='라이다 스캔 높이 (바닥 기준) [m]')
p.add_argument('--pad', type=float, default=0.30, help='미로 바깥 여백 [m]')
p.add_argument('--out_dir', default=str(ROOT / 'maps'))
a = p.parse_args()

layout_path = ROOT / 'maze_real_layout.json' if a.maze == 'real' else ROOT / 'mazes' / f'{a.maze}_layout.json'
L = json.loads(layout_path.read_text())
sx, sy = L['size_m'][:2]
x0, y0 = -sx / 2 - a.pad, -sy / 2 - a.pad
W, H = int(math.ceil((sx + 2 * a.pad) / a.res)), int(math.ceil((sy + 2 * a.pad) / a.res))
occ = np.zeros((H, W), bool)                 # [행 = y, 열 = x], 행 0 = 아래 (y0)
xs = x0 + (np.arange(W) + 0.5) * a.res
ys = y0 + (np.arange(H) + 0.5) * a.res
X, Y = np.meshgrid(xs, ys)
half_cell = a.res / 2

n_box = n_mount = 0
for b in L['collision_boxes']:
    cz, hz = b['center'][2], b['size'][2] / 2
    if not (cz - hz <= a.scan_z <= cz + hz):
        continue
    yaw = b.get('yaw', 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = X - b['center'][0], Y - b['center'][1]
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    # 얇은 벽판(3 mm)도 칸 하나는 막히게 반 칸만큼 넓혀 그림
    occ |= (np.abs(lx) <= b['size'][0] / 2 + half_cell) & (np.abs(ly) <= b['size'][1] / 2 + half_cell)
    n_box += 1
for m in L.get('mounts', []):
    if m.get('top_h', m.get('h', 0.0)) < a.scan_z:
        continue
    occ |= np.hypot(X - m['xy'][0], Y - m['xy'][1]) <= m['r'] + half_cell
    n_mount += 1

# 주사위 경기장(dice_layout.json, tools/make_dice_map.py): 바닥에 놓인 소품도 스캔 높이를 지나면 점유
#   시작 박스 · L1/L3 배치대 205 mm, 중앙 받침대 820 mm (높이 70 mm), 신호등 받침 Ø60 (높이 30 mm, 위 기둥 Ø40 까지 149 mm), 바코드 기둥 20×20
n_prop = 0
M = L.get('mission')
if M:
    def add_square(cx, cy, w):
        global occ, n_prop
        occ |= (np.abs(X - cx) <= w / 2 + half_cell) & (np.abs(Y - cy) <= w / 2 + half_cell)
        n_prop += 1
    for b in M['start_boxes'].values():
        add_square(*b['sim_m'], 0.205)
    for k in ('L1', 'L3'):
        add_square(*M['places'][k]['center_sim_m'], 0.205)
    add_square(*M['platform']['sim_m'][:2], M['platform']['size_m'][0])
    if M['traffic_light']['top_z_m'] >= a.scan_z:
        occ |= np.hypot(X - M['traffic_light']['sim_m'][0], Y - M['traffic_light']['sim_m'][1]) <= 0.030 + half_cell
        n_prop += 1
    add_square(*M['barcode']['sim_m'][:2], 0.020)

out = Path(a.out_dir)
out.mkdir(parents=True, exist_ok=True)
name = f'maze_{a.maze}_map' if a.maze != 'real' else 'maze_real_map'
img = np.where(occ, 0, 254).astype(np.uint8)[::-1]   # PGM 첫 줄 = 지도 위쪽 (y 최대)
with open(out / f'{name}.pgm', 'wb') as f:
    f.write(f'P5\n# {layout_path.name}, scan_z {a.scan_z} m\n{W} {H}\n255\n'.encode())
    f.write(img.tobytes())
(out / f'{name}.yaml').write_text(
    f'# 시뮬 Nav2 시험용 지도: tools/layout_to_occupancy.py 가 {layout_path.name} 에서 만듦 (스캔 높이 {a.scan_z} m)\n'
    f'image: {name}.pgm\nmode: trinary\nresolution: {a.res}\norigin: [{x0:.4f}, {y0:.4f}, 0.0]\n'
    f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print(f'{out / name}.{{pgm,yaml}}: {W}x{H} 칸 ({a.res} m), 벽 상자 {n_box}, 연결 부품 {n_mount}, 소품 {n_prop}, 점유 {occ.mean() * 100:.1f}%')
