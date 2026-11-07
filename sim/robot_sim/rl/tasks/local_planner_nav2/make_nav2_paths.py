#!/usr/bin/env python3
"""실제 Nav2 planner_server 경로로 경로 은행 · dice 구간 만들기 → rl/data/local_planner_nav2/

local_planner 의 경로 은행 (tools/make_path_bank.py) 은 NavFn 을 파이썬으로 흉내 낸 것 → 배포 때 받는 경로와 다를 수 있음.
여기선 같은 시작 · 목표 쌍을 planner_server 에 planner_id 별로 요청 (기본 GridBased = NavFn, Smac2D, Smac2DFar) + 흉내 경로도 같은 쌍으로 만듦.
→ 세 은행의 경로 쌍이 같아서 경로 모양 비교 (compare_paths.py) · 정책 평가를 같은 기준으로 할 수 있음.

받는 것: compute_path_to_pose (planner_server), map_server/load_map, /global_costmap/costmap (지도 바뀐 뒤 반영 확인)
만드는 것 (형식 = path_bank.npz · dice_legs.npz 와 같음, 지도 경로만 · 합성 경로 없음):
  path_bank_<이름>.npz  지도 경로 (학습 = real + rand_00..06, 평가 = dice + rand_07)
  dice_legs_<이름>.npz  dice 정차 후보끼리 모든 구간 (stops.py: 출발 + 박스 · 장소 네 면 중 설 수 있는 것, 박스 면 ↔ 외곽 gap 2 cm)
                        + stop_names · stop_poses · aliases ('B1=B1.S' = dice_map.yaml 의 지금 면)
  stats.json            planner 별 실패 수 · 계획 시간
  <이름> = navfn (GridBased) · smac2d (Smac2D) · smac2d_far (Smac2DFar, 벽 피하기 세게) · imitation (파이썬 흉내), --planners 로 바꿈
쌍 고르기 = make_path_bank 와 같음 (비용 < 128 칸, 목표마다 시작 여러 개). 모든 planner 가 성공하고 경로 점 비용 < 253 인 쌍만 남김.
목표 yaw = 쌍마다 한 번 뽑음: 50 % 경로 끝 접선 (planner 마다 자기 경로 끝) · 50 % 무작위 (모두 같은 값).

예) run_make_paths.sh 가 planner_server 를 띄우고 이것을 부름. 직접 부를 때:
    source /opt/ros/lyrical/setup.bash; export ROS_DOMAIN_ID=17 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 rl/tasks/local_planner_nav2/make_nav2_paths.py [--per_map 600] [--maps dice,rand_07] [--no_dice]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.sparse.csgraph import dijkstra

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]                                   # sim/robot_sim
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))
import make_path_bank as mpb  # noqa: E402
sys.path.insert(0, str(HERE))
import stops as dstops  # noqa: E402

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from nav2_msgs.srv import LoadMap  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402

NAMES = mpb.TRAIN_MAPS + mpb.EVAL_MAPS                   # 지도 번호 순서 (env 가 은행 · 구간 순서 같은지 확인)

p = argparse.ArgumentParser()
p.add_argument('--out_dir', default=str(ROOT / 'rl' / 'data' / 'local_planner_nav2'))
p.add_argument('--maps_dir', default=str(ROOT / 'rl' / 'data' / 'maps'))
p.add_argument('--maps', default=','.join(NAMES), help='쉼표로 (기본 전체)')
p.add_argument('--per_map', type=int, default=600, help='학습 지도 하나당 경로 수 (평가 지도는 절반)')
p.add_argument('--goals', type=int, default=60, help='지도 하나당 목표 수')
p.add_argument('--planners', default='navfn=GridBased,smac2d=Smac2D,smac2d_far=Smac2DFar', help='파일 이름=planner_id (smac2d_planners.yaml)')
p.add_argument('--no_dice', action='store_true', help='dice 구간 안 만듦')
p.add_argument('--gap', type=float, default=0.02, help='dice 정차: Nav2 외곽 (±0.16) ↔ 박스 면 거리 [m] (stops.py)')
p.add_argument('--timeout', type=float, default=5.0, help='요청 하나 기다리는 시간 [s]')
p.add_argument('--seed', type=int, default=20261104)
a = p.parse_args()
rng = np.random.default_rng(a.seed)
PLANNERS = dict(kv.split('=') for kv in a.planners.split(','))   # 파일 이름 → planner_id


def to_ps(x, y, yaw=0.0):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x, ps.pose.position.y = float(x), float(y)
    ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return ps


class Client(Node):
    def __init__(self):
        super().__init__('make_nav2_paths')
        self.act = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.load = self.create_client(LoadMap, 'map_server/load_map')
        self.cm = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self._cm, qos)

    def _cm(self, m):
        self.cm = m

    def wait(self, fut, timeout):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def switch_map(self, yaml_path, W, H, origin):
        """지도 바꾸고 global_costmap 이 같은 크기 · 원점으로 다시 나올 때까지 기다림"""
        assert self.load.wait_for_service(timeout_sec=10.0), 'map_server/load_map 없음 (paths.launch.py 떴는지)'
        r = self.wait(self.load.call_async(LoadMap.Request(map_url=str(yaml_path))), 10.0)
        assert r is not None and r.result == 0, f'지도 못 읽음: {yaml_path}'
        self.cm, t0 = None, time.time()
        while time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.2)
            m = self.cm
            if m is not None and m.info.width == W and m.info.height == H \
                    and abs(m.info.origin.position.x - origin[0]) < 1e-3 and abs(m.info.origin.position.y - origin[1]) < 1e-3:
                for _ in range(10):                      # 팽창 층 반영까지 조금 더
                    rclpy.spin_once(self, timeout_sec=0.1)
                return
        raise RuntimeError(f'global_costmap 이 새 지도로 안 바뀜: {yaml_path}')

    def plan(self, planner_id, s, g):
        """s, g = (x, y, yaw) 지도 좌표 → (경로 점 (N,2), 계획 시간 s) 또는 (None, 이유)"""
        if not self.act.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('compute_path_to_pose 없음')
        goal = ComputePathToPose.Goal(goal=to_ps(*g), start=to_ps(*s), planner_id=planner_id, use_start=True)
        h = self.wait(self.act.send_goal_async(goal), a.timeout)
        if h is None or not h.accepted:
            return None, 'rejected'
        r = self.wait(h.get_result_async(), a.timeout)
        if r is None:
            return None, 'timeout'
        poses = r.result.path.poses
        if len(poses) < 2:
            return None, f'error {r.result.error_code}'
        pt = r.result.planning_time
        return np.array([(q.pose.position.x, q.pose.position.y) for q in poses]), pt.sec + pt.nanosec * 1e-9


def cell_xy(c, origin, res):
    return origin[0] + (c[1] + 0.5) * res, origin[1] + (c[0] + 0.5) * res


def imitation(pot, s_cell, g_cell, origin, res):
    cells = mpb.descend(pot, s_cell, g_cell)
    if cells is None:
        return None
    return np.stack([origin[0] + (cells[:, 1] + 0.5) * res, origin[1] + (cells[:, 0] + 0.5) * res], 1)


def free_cell(cost, c, res, tol=0.1):
    """정차 칸이 내접 영역 (비용 253 이상) 이면 tol 안 가장 가까운 빈 칸 (NavFn · Smac2D tolerance 0.1 m 와 같은 처리).
    박스 면 2 cm 앞 정차점은 지도 칸 2.5 cm 오차로 내접 영역 칸에 들기도 함 → 흉내 경로도 그 칸에서 끝내고 끝점만 정확한 점으로"""
    if cost[c] < 253:
        return c
    r = int(math.ceil(tol / res))
    best = None
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            rr, cc = c[0] + dr, c[1] + dc
            if 0 <= rr < cost.shape[0] and 0 <= cc < cost.shape[1] and cost[rr, cc] < 253 and math.hypot(dr, dc) * res <= tol:
                if best is None or math.hypot(dr, dc) < best[0]:
                    best = (math.hypot(dr, dc), (rr, cc))
    return c if best is None else best[1]


def finish(xy, s, g):
    """시작 · 끝을 정확한 점으로 맞추고 0.05 m 재표본"""
    xy = np.asarray(xy, float).copy()
    xy[0], xy[-1] = s[:2], g[:2]
    return mpb.resample(xy)


def cost_ok(pts, cost, origin, res, skip_end=0):
    H, W = cost.shape
    ci = np.clip(((pts - origin) / res - 0.5).round().astype(int), 0, [W - 1, H - 1])
    c = cost[ci[:, 1], ci[:, 0]]
    if skip_end:
        c = c[skip_end:-skip_end] if len(c) > 2 * skip_end else c[:0]
    return not (c >= 253).any()


def save_bank(path, items, extra=None):
    """items = [(지도 좌표 pts, goal_yaw, level, split, src, 이름 · 구간, start_yaw)] → path_bank 형식"""
    N = len(items)
    P = np.zeros((N, mpb.MAX_PTS, 2), np.float32)
    n = np.zeros(N, np.int32)
    out = {k: np.zeros(N, np.float32) for k in ('goal_yaw', 'start_yaw')}
    start = np.zeros((N, 2), np.float32)
    lvl, spl, src = np.zeros(N, np.int8), np.zeros(N, np.int8), np.zeros(N, np.int16)
    for i, (pts, gy, lv, sp, sr, _, sy) in enumerate(items):
        k = len(pts)
        start[i] = pts[0]
        P[i, :k] = pts - pts[0]
        P[i, k:] = pts[-1] - pts[0]
        n[i], out['goal_yaw'][i], out['start_yaw'][i], lvl[i], spl[i], src[i] = k, gy, sy, lv, sp, sr
    np.savez_compressed(path, pts=P, n=n, start=start, level=lvl, split=spl, src=src, names=np.array(NAMES), **out, **(extra or {}))


def map_pairs(cli, name, stats, banks):
    occ, res, origin, layout = mpb.load_map(name)
    cost = mpb.costmap(occ, res)
    H, W = cost.shape
    stem = 'maze_real_map' if name == 'real' else f'maze_{name}_map'
    cli.switch_map(Path(a.maps_dir) / f'{stem}.yaml', W, H, origin)
    split = 0 if name in mpb.TRAIN_MAPS else 1
    n_paths = a.per_map if split == 0 else a.per_map // 2
    sx, sy = layout['size_m'][:2]
    ys, xs = np.mgrid[0:H, 0:W]
    wx, wy = origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res
    cand = np.argwhere((cost < 128) & (np.abs(wx) < sx / 2 - 0.05) & (np.abs(wy) < sy / 2 - 0.05))
    G = mpb.build_graph(cost)
    per_goal = int(math.ceil(n_paths / a.goals))
    got, tries, t0 = 0, 0, time.time()
    while got < n_paths and tries < a.goals * 4:
        tries += 1
        gc = cand[rng.integers(len(cand))]
        pot = dijkstra(G, indices=gc[0] * W + gc[1]).reshape(H, W)
        g = (*cell_xy(gc, origin, res), 0.0)
        k = 0
        for sc in cand[rng.choice(len(cand), per_goal * 3, replace=False)]:
            if k >= per_goal or got >= n_paths:
                break
            s = (*cell_xy(sc, origin, res), 0.0)
            u, ry = rng.random(), rng.uniform(-math.pi, math.pi)        # 목표 yaw 규칙 (쌍마다 한 번)
            if np.hypot(s[0] - g[0], s[1] - g[1]) < 0.3 or not np.isfinite(pot[sc[0], sc[1]]):
                continue
            paths, bad = {}, None
            xy = imitation(pot, sc, gc, origin, res)
            paths['imitation'] = None if xy is None else finish(xy, s, g)
            for fn, pid in PLANNERS.items():
                xy, info = cli.plan(pid, s, g)
                if xy is None:
                    stats[fn]['fail'][info] = stats[fn]['fail'].get(info, 0) + 1
                    paths[fn] = None
                    continue
                stats[fn]['plan_s'].append(info)
                paths[fn] = finish(xy, s, g)
            for fn, pts in paths.items():
                if pts is None:
                    bad = bad or f'{fn} 없음'
                elif len(pts) < 7 or len(pts) > mpb.MAX_PTS:
                    bad = bad or f'{fn} 길이'
                elif not cost_ok(pts, cost, origin, res):
                    bad = bad or f'{fn} 비용'
                    stats[fn]['cost_reject'] = stats[fn].get('cost_reject', 0) + 1
            if bad:
                stats['pair_reject'][bad] = stats['pair_reject'].get(bad, 0) + 1
                continue
            for fn, pts in paths.items():
                gy = mpb.end_tangent_yaw(pts) if u < 0.5 else ry
                L = (len(pts) - 1) * mpb.STEP
                banks[fn].append((pts, gy, 1 if L <= 4.0 else 2, split, NAMES.index(name), name, np.nan))
            k += 1
            got += 1
    print(f'  {name}: 쌍 {got} 개 (목표 {tries} 번, {time.time() - t0:.0f} s)', flush=True)


def dice_legs(cli, stats):
    occ, res, origin, _ = mpb.load_map('dice')
    cost = mpb.costmap(occ, res)
    H, W = cost.shape
    cli.switch_map(Path(a.maps_dir) / 'maze_dice_map.yaml', W, H, origin)
    G = mpb.build_graph(cost)
    stops, aliases, _ = dstops.candidates(a.gap)                   # 출발 + 박스 · 장소 네 면 (지도 좌표)
    cell = {k: free_cell(cost, (int(round((y - origin[1]) / res - 0.5)), int(round((x - origin[0]) / res - 0.5))), res)
            for k, (x, y, _) in stops.items()}
    unreachable = set()                                            # 출발점에서 어느 planner 로도 경로가 없는 면 = 갈 수 없는 자리
    for k, g in stops.items():
        if k != 'START' and all(cli.plan(pid, stops['START'], g)[0] is None for pid in PLANNERS.values()):
            unreachable.add(k)
    if unreachable:
        print(f'  갈 수 없는 면 (출발점에서 경로 없음) 뺌: {sorted(unreachable)}')
    stops = {k: v for k, v in stops.items() if k not in unreachable}
    legs = {fn: [] for fn in ['imitation', *PLANNERS]}
    for gk, g in stops.items():
        pot = dijkstra(G, indices=cell[gk][0] * W + cell[gk][1]).reshape(H, W)
        for sk, s in stops.items():
            if sk == gk:
                continue
            paths = {}
            xy = imitation(pot, cell[sk], cell[gk], origin, res)
            paths['imitation'] = None if xy is None else finish(xy, s, g)
            for fn, pid in PLANNERS.items():
                xy, info = cli.plan(pid, s, g)
                if xy is None:
                    stats[fn]['dice_fail'].append(f'{sk}->{gk}: {info}')
                    paths[fn] = None
                else:
                    paths[fn] = finish(xy, s, g)
            if any(v is None or len(v) > mpb.MAX_PTS for v in paths.values()):
                print(f'  {sk}->{gk}: 뺌 ({", ".join(k for k, v in paths.items() if v is None or len(v) > mpb.MAX_PTS)})')
                continue
            for fn, pts in paths.items():
                legs[fn].append((pts, g[2], 1, 0, NAMES.index('dice'), f'{sk}->{gk}', s[2]))
    for fn, items in legs.items():
        dst = Path(a.out_dir) / f'dice_legs_{fn}.npz'
        save_bank(dst, items, {'legs': np.array([it[5] for it in items]), 'gap': np.float32(a.gap),
                               'stop_names': np.array(list(stops)), 'stop_poses': np.array(list(stops.values()), np.float32),
                               'aliases': np.array([f'{k}={v}' for k, v in aliases.items()])})
        print(f'  dice 구간 {fn}: {len(items)} 개 → {dst}')
    stats['dice_unreachable'] = sorted(unreachable)


def main():
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    cli = Client()
    stats = {fn: {'fail': {}, 'plan_s': [], 'dice_fail': []} for fn in PLANNERS}
    stats['pair_reject'] = {}
    banks = {fn: [] for fn in ['imitation', *PLANNERS]}
    try:
        for name in [m for m in a.maps.split(',') if m]:
            assert name in NAMES, f'모르는 지도 {name}'
            map_pairs(cli, name, stats, banks)
        for fn, items in banks.items():
            if items:
                save_bank(out / f'path_bank_{fn}.npz', items)
                sp = np.array([it[3] for it in items])
                print(f'경로 은행 {fn}: 학습 {(sp == 0).sum()} · 평가 {(sp == 1).sum()} → {out / f"path_bank_{fn}.npz"}')
        if not a.no_dice:
            dice_legs(cli, stats)
    finally:
        for fn in PLANNERS:
            t = np.array(stats[fn].pop('plan_s')) if stats[fn]['plan_s'] else np.zeros(1)
            stats[fn]['plan_ms'] = {'median': float(np.median(t) * 1e3), 'p95': float(np.percentile(t, 95) * 1e3), 'n': int(len(t))}
        (out / 'stats.json').write_text(json.dumps({'seed': a.seed, 'maps': a.maps, **stats}, indent=1, ensure_ascii=False))
        print(json.dumps(stats, ensure_ascii=False))
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
