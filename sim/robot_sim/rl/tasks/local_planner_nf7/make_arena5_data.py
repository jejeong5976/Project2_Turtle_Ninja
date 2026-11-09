"""nf7 학습 데이터: arena5 지도 · 정차점 구간 · 박스 목록 (지도 ↔ 실제 어긋남용).

받는 것
  - --arena_map: 실물 SLAM 지도 yaml (Jetson ~/maps/arena5.yaml, map_saver 형식)
  - --dice_arena5: arena5 에서 잰 박스 · 장소 자리 (~/Downloads/dice_map_arena5.yaml, map_from_arena 맞춤값 포함)
  - --pairs: 정차 면 쌍마다 Nav2 planner_server 경로 (make_arena5_pairs.py 출력, 기본 rl/data/local_planner_nf7/arena5_pairs.json)
  - --drawing: 레포 maps/dice_map.yaml (도면 박스 자리 → 시뮬 dice 지도 박스 목록)
주는 것 (rl/data)
  - maps/maze_arena5_map.pgm · yaml: world.load_pgm_map 형식 (주석 줄 있는 P5)
  - local_planner_nf7/arena5_legs.npz: dice_legs 형식 (names = ['arena5'], 정차 면 쌍 구간, 정차점 = 박스 면 2 cm = 중심 0.2825 m)
  - local_planner_nf7/map_boxes.json: {'dice': 도면 박스 9 개 (시뮬 지도 좌표), 'arena5': 잰 박스 9 개 (arena5 map 좌표)} [cx, cy, 회전]
실행: python3 rl/tasks/local_planner_nf7/make_arena5_data.py   (sim/robot_sim 에서, 시스템 python3)
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

RL = Path(__file__).resolve().parents[2]
REPO = RL.parents[2]
STEP = 0.05                        # swerve_follow.paths.STEP (경로 점 간격)
M = 256                            # 구간 점 수 (dice_legs_navfn 과 같게)
D_STOP, GAP_BACK = 0.2575, 0.025   # 정차 거리 · Nav2 목표 = 정차점 뒤 2.5 cm (dock_nav_back, nf 구간 끝)
SIM_ARENA_ORIGIN = (2.202, 3.152)  # 시뮬 dice 지도 좌표 = 경기장 − 이것 (dice_legs stop_poses 와 같음)


def resample(p):
    """점 목록 → STEP 간격 (끝점 포함)"""
    p = np.asarray(p, float)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-6])
    p, s = p[keep], np.concatenate([[0.0], np.cumsum(seg[seg > 1e-6])])
    L = s[-1]
    t = np.append(np.arange(0.0, L, STEP), L)
    return np.stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])], 1), L


def write_map(src_yaml: Path, out_dir: Path, name: str):
    m = yaml.safe_load(src_yaml.read_text())
    raw = (src_yaml.parent / m['image']).read_bytes()
    parts = raw.split(b'\n', 3)                    # map_saver: P5 / W H / 255 / 데이터 (주석 없음)
    assert parts[0] == b'P5' and not parts[1].startswith(b'#'), 'map_saver P5 (주석 없는) 형식만'
    stem = f'maze_{name}_map'
    (out_dir / f'{stem}.pgm').write_bytes(b'P5\n# ' + src_yaml.name.encode() + b' (slam_toolbox, make_arena5_data.py)\n'
                                         + parts[1] + b'\n' + parts[2] + b'\n' + parts[3])
    o = m['origin']
    (out_dir / f'{stem}.yaml').write_text(
        f'# {src_yaml} 복사 (nf7 학습용)\nimage: {stem}.pgm\nmode: trinary\nresolution: {m["resolution"]}\n'
        f'origin: [{o[0]}, {o[1]}, {o[2] if len(o) > 2 else 0.0}]\nnegate: 0\noccupied_thresh: {m["occupied_thresh"]}\nfree_thresh: {m["free_thresh"]}\n')
    return out_dir / f'{stem}.pgm'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arena_map', default=str(Path.home() / 'Downloads' / 'arena5.yaml'))
    ap.add_argument('--dice_arena5', default=str(Path.home() / 'Downloads' / 'dice_map_arena5.yaml'))
    ap.add_argument('--pairs', default=str(RL / 'data' / 'local_planner_nf7' / 'arena5_pairs.json'))
    ap.add_argument('--drawing', default=str(REPO / 'maps' / 'dice_map.yaml'))
    ap.add_argument('--name', default='arena5')
    a = ap.parse_args()
    out = RL / 'data' / 'local_planner_nf7'
    out.mkdir(parents=True, exist_ok=True)
    pgm = write_map(Path(a.arena_map), RL / 'data' / 'maps', a.name)
    print(f'지도 → {pgm}')

    # 구간: 정차 면 쌍 (planner_server 경로) → 정차점 0.2825 m 끼리
    J = json.loads(Path(a.pairs).read_text())
    N = {k: (np.array(v[0], float), float(v[1])) for k, v in J['nodes'].items()}
    def stop(k):                                   # 박스 · 장소 면 = 정차점 뒤 2.5 cm, S = 출발점
        p, yw = N[k]
        return (p, yw) if k == 'S' else (p - GAP_BACK * np.array([math.cos(yw), math.sin(yw)]), yw)
    pts, ns, starts, gy, sy, legs = [], [], [], [], [], []
    for key, P in J['P'].items():
        A, Bn = key.split('>')
        P = np.asarray(P, float)
        (sa, ya), (gb, yb) = stop(A), stop(Bn)
        body = P[1:-1]                             # planner 경로 (출발 = 정차점 또는 빠져나온 점 … 끝 = 정차점 뒤 2.5 cm)
        path, L = resample(np.vstack([[sa], body[1:-1], [gb]]) if len(body) > 2 else np.vstack([[sa], [gb]]))
        if len(path) > M:
            print(f'  {key}: {len(path)} 점 > {M} → 뺌')
            continue
        pad = np.vstack([path, np.repeat(path[-1:], M - len(path), 0)])
        pts.append(pad - path[0])
        ns.append(len(path))
        starts.append(path[0])
        gy.append(yb)
        sy.append(ya)
        legs.append(key.replace('>', '->'))
    np.savez(out / 'arena5_legs.npz', pts=np.array(pts, np.float32), n=np.array(ns, np.int32), start=np.array(starts, np.float32),
             level=np.ones(len(ns), np.int8), split=np.zeros(len(ns), np.int8), src=np.zeros(len(ns), np.int16),
             names=np.array([a.name]), goal_yaw=np.array(gy, np.float32), start_yaw=np.array(sy, np.float32),
             legs=np.array(legs), gap=np.float32(0.02))
    print(f'구간 {len(ns)} 개 → {out / "arena5_legs.npz"} (길이 {min(ns) * STEP:.2f}–{max(ns) * STEP:.2f} m)')

    # 박스 목록: 도면 (시뮬 dice 지도) · arena5 (잰 자리, map 좌표)
    dr = yaml.safe_load(Path(a.drawing).read_text())
    da = yaml.safe_load(Path(a.dice_arena5).read_text())
    T = da['map_from_arena']
    c, s = math.cos(T[2]), math.sin(T[2])
    boxes = {'dice': [], a.name: []}
    for sec in ('boxes', 'places'):
        for k in sorted(dr[sec]):
            cx, cy = dr[sec][k]['center']
            boxes['dice'].append([round(cx - SIM_ARENA_ORIGIN[0], 4), round(cy - SIM_ARENA_ORIGIN[1], 4), 0.0])
        for k in sorted(da[sec]):
            cx, cy = da[sec][k]['center']
            rot = da[sec][k]['stop'][2] + T[2]     # 박스 회전 = 정차 방향 (정사각형이라 90° 단위 무관)
            rot = (rot + math.pi / 4) % (math.pi / 2) - math.pi / 4
            boxes[a.name].append([round(T[0] + c * cx - s * cy, 4), round(T[1] + s * cx + c * cy, 4), round(rot, 4)])
    (out / 'map_boxes.json').write_text(json.dumps(boxes, indent=1))
    print(f'박스 → {out / "map_boxes.json"} (dice {len(boxes["dice"])} · {a.name} {len(boxes[a.name])})')


if __name__ == '__main__':
    main()
