"""dice 정차 후보: 출발점 + 박스 B1–B6 · 장소 L1–L3 의 네 면 (N · E · S · W) 앞 자리.

면 = 로봇이 박스의 어느 쪽에 서는지 (N = 박스 북쪽에 서서 −Y 를 봄). 이름 'B1.S'.
정차 거리 = 박스 반 폭 0.1025 + Nav2 외곽 반 폭 0.16 + gap (기본 0.02 → 중심에서 0.2825 m, 박스 면 ↔ 로봇 중심 0.18 m).
  내접 반경 0.17 (외곽 0.16 + padding 0.01) 보다 멀어야 planner 가 목표로 받음 (0.18 → 비용 228 < 253).
못 서는 면 = 그 자리 외곽 (±0.16) 이 박스 말고 다른 것 (벽 · 기둥) 과 gap − 1.5 cm 보다 가까우면 뺌 (지도 칸 2.5 cm 오차 감안).
별칭: 'B1' = 레포 maps/dice_map.yaml 의 지금 정차 면 (B1 = 'B1.S') → local_planner/play.py 의 옛 구간 이름 그대로 씀.
좌표: 경기장 m (dice_map.yaml) → 지도 = 경기장 − (2.202, 3.152).
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]                                   # sim/robot_sim
sys.path.insert(0, str(ROOT / 'tools'))
import make_path_bank as mpb  # noqa: E402

DICE_MAP = ROOT.parents[1] / 'maps' / 'dice_map.yaml'
OX, OY = 2.202, 3.152
BOX_HALF = 0.1025                                        # 205 mm 박스 (시작 박스 · 장소 같음)
FP_HALF = 0.16                                           # Nav2 외곽 (config/nav2.yaml footprint)
SIDES = {'N': (0.0, 1.0), 'E': (1.0, 0.0), 'S': (0.0, -1.0), 'W': (-1.0, 0.0)}


def _footprint_clear(df, res, origin, x, y, yaw, n=12):
    """외곽 둘레 점의 지도 거리 최솟값 (장애물 칸 중심까지 − 반 칸)"""
    t = np.linspace(-FP_HALF, FP_HALF, n)
    e = np.full(n, FP_HALF)
    pts = np.concatenate([np.stack([e, t], 1), np.stack([-e, t], 1), np.stack([t, e], 1), np.stack([t, -e], 1)])
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1)
    ij = np.floor((w - origin) / res).astype(int)
    H, W = df.shape
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
    if not ok.all():
        return -1.0
    return float(df[ij[:, 1], ij[:, 0]].min())


def candidates(gap=0.02, verbose=True):
    """→ (stops {이름: (x, y, yaw) 지도 좌표}, aliases {옛 이름: 면 이름}, dropped {이름: 여유})"""
    m = yaml.safe_load(DICE_MAP.read_text())
    occ, res, origin, _ = mpb.load_map('dice')
    D = BOX_HALF + FP_HALF + gap
    sx, sy, syaw = m['start']
    stops, aliases, dropped = {'START': (sx - OX, sy - OY, syaw)}, {}, {}
    for sec in ('boxes', 'places'):
        for k, v in m[sec].items():
            cx, cy = v['center'][0] - OX, v['center'][1] - OY
            # 이 박스 칸을 지운 지도로 여유 (박스 자신은 gap 만큼 떨어져 있는 게 정상)
            box = np.zeros_like(occ)
            ys, xs = np.mgrid[0:occ.shape[0], 0:occ.shape[1]]
            wx, wy = origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res
            box[(np.abs(wx - cx) < BOX_HALF + res) & (np.abs(wy - cy) < BOX_HALF + res)] = True
            df = ndimage.distance_transform_edt(~(occ & ~box)) * res - res / 2
            st = v['stop']
            dxs, dys = st[0] - v['center'][0], st[1] - v['center'][1]
            aliases[k] = f"{k}.{'E' if dxs > abs(dys) else 'W' if -dxs > abs(dys) else 'N' if dys > 0 else 'S'}"
            for side, (dx, dy) in SIDES.items():
                x, y, yaw = cx + dx * D, cy + dy * D, math.atan2(-dy, -dx)
                clr = _footprint_clear(df, res, origin, x, y, yaw)
                if clr < gap - 0.015:
                    dropped[f'{k}.{side}'] = clr
                    continue
                stops[f'{k}.{side}'] = (x, y, yaw)
    if verbose:
        print(f'[stops] 정차 거리 {D:.4f} m (gap {gap * 100:.1f} cm), 후보 {len(stops)} 개, 뺀 면 '
              + (', '.join(f'{k} (여유 {v * 100:.1f} cm)' for k, v in dropped.items()) or '없음'))
        miss = [a for a in aliases.values() if a not in stops]
        if miss:
            print(f'[stops] 경고: 지금 정차 면 {miss} 가 후보에서 빠짐')
    return stops, aliases, dropped


if __name__ == '__main__':
    s, al, dr = candidates()
    for k, (x, y, yaw) in s.items():
        print(f'  {k:8s} 지도 ({x:+.4f}, {y:+.4f}) yaw {math.degrees(yaw):+6.1f}°  경기장 ({x + OX:.4f}, {y + OY:.4f})')
    print('  별칭', al)
