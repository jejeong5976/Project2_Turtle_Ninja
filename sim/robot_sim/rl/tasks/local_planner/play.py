#!/usr/bin/env python3
"""지역 계획기 평가: dice 경로 (Nav2 시험과 같은 10 구간 START → B1 → L1 → B2 → B6 → L3 → B3 → B5 → B4 → L2 → START) 를
구간마다 env N 개 (DR 다름) 로 돌려 시간 · 성공 · 충돌 · 도착 오차. 정책 없으면 기준 추종기 (swerve_follow.follower).
간이 플랜트 결과라 Isaac 과 숫자는 다름 → 정책 · 추종기 비교용. 사람 없음 (--people 로 단계 2 사람).
예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/play.py --checkpoint rl/logs/local_planner/<run>/model_3000.pt
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner/play.py            # 기준 추종기
그림: --plot out.png (구간마다 env 0 궤적, dice 지도 위)
실제 Nav2 경로로 평가: --dice_legs rl/data/local_planner_nav2/dice_legs_navfn.npz --path_bank rl/data/local_planner_nav2/path_bank_navfn.npz
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402
from swerve_follow.follower import follow  # noqa: E402

ROUTE = ['START', 'B1', 'L1', 'B2', 'B6', 'L3', 'B3', 'B5', 'B4', 'L2', 'START']

p = argparse.ArgumentParser()
p.add_argument('--checkpoint', default='')
p.add_argument('--num_envs', type=int, default=64)
p.add_argument('--people', action='store_true')
p.add_argument('--level', type=int, default=2, help='성공 판정 단계 (2 = 2 cm · 2°)')
p.add_argument('--plot', default='')
p.add_argument('--device', default='cuda')
p.add_argument('--dice_legs', default='', help='dice 구간 파일 (기본 cfg 값). 실제 Nav2 경로 = rl/data/local_planner_nav2/dice_legs_<navfn|smac2d|...>.npz')
p.add_argument('--path_bank', default='', help='경로 은행 (dice 구간과 지도 이름 순서가 같아야 함, 기본 cfg 값)')
a = p.parse_args()

cfg = Cfg()
if a.checkpoint:
    saved = json.loads((Path(a.checkpoint).parent / 'cfg.json').read_text())
    for sec in ('env', 'dr', 'reward'):
        for k, v in saved[sec].items():
            if hasattr(getattr(cfg, sec), k) and k not in ('path_bank', 'dice_legs', 'maps_dir', 'swerve_yaml'):
                setattr(getattr(cfg, sec), k, tuple(v) if isinstance(v, list) else v)
    if 'std_range' in saved.get('train', {}):              # local_planner_nav2 실행 = log std · 상한 (체크포인트 매개변수 이름이 다름)
        from local_planner_nav2.cfg import TrainCfg as _NavTrain
        cfg.train = _NavTrain(std_range=tuple(saved['train']['std_range']))
cfg.env.num_envs, cfg.env.device, cfg.env.compile = a.num_envs, a.device, True
if a.dice_legs:
    cfg.env.dice_legs = a.dice_legs
if a.path_bank:
    cfg.env.path_bank = a.path_bank
cfg.env.curriculum, cfg.env.dice_prob = False, 1.0
cfg.env.timeout_slack_s = 30.0
if not a.people:
    cfg.env.people_prob = (0.0, 0.0, 0.0)
env = LocalPlannerEnv(cfg)
env.level = a.level
_z = np.load(cfg.env.dice_legs)
ALIAS = dict(x.split('=') for x in map(str, _z['aliases'])) if 'aliases' in _z else {}   # 네 면 구간 (local_planner_nav2): 'B1' → 지금 정차 면 'B1.S'
B, dev = a.num_envs, a.device

policy = None
if a.checkpoint:
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, cfg.train.runner_dict(), None, dev)
    runner.load(a.checkpoint, map_location=dev)
    policy = runner.get_inference_policy(dev)
lim = (torch.full((B,), cfg.env.vmax, device=dev), torch.full((B,), cfg.env.amax, device=dev), torch.full((B,), cfg.env.wmax, device=dev))

rows, tracks = [], []
with torch.inference_mode():
    for s, g in zip(ROUTE, ROUTE[1:]):
        k = env.dice_first + env.dice_legs.index(f'{ALIAS.get(s, s)}->{ALIAS.get(g, g)}')
        env.forced = torch.full((B,), k, dtype=torch.long, device=dev)
        env._reset_idx(torch.arange(B, device=dev))
        env.obs = env._compute_obs(torch.arange(B, device=dev))
        env.forced = torch.full((B,), -1, dtype=torch.long, device=dev)
        leg_len = float(env.path.length[0])                       # 끝난 env 는 무작위 구간으로 다시 리셋되므로 지금 잡아 둠
        leg_path = env.path.pts[0, :int(env.path.n[0])].cpu().numpy().copy()
        res = {}
        tr = [env.plant.pose[0].cpu().numpy().copy()]
        for _ in range(int(env.max_episode_length_t.max()) + 2):
            if policy is not None:
                act = policy(env.obs)
            else:
                act = follow(env.path, env._est_pose(), env.s_est, *lim, 0.20, 1.5) / env.lim3
            if 0 not in res:
                tr.append(env.plant.pose[0].cpu().numpy().copy())
            env.step(act)
            d = env.last_done
            if d is not None:
                for j, i in enumerate(d['ids'].tolist()):
                    if i not in res:
                        res[i] = {kk: float(d[kk][j]) for kk in ('success', 'collision', 'lost', 'timeout', 'time_s', 'final_dist_true', 'final_dyaw_true', 'clear_min', 'travel')}
            if len(res) == B:
                break
        r = list(res.values())
        ok = [x for x in r if x['success']]
        row = dict(leg=f'{s}->{g}', length=leg_len, success=np.mean([x['success'] for x in r]),
                   collision=np.mean([x['collision'] for x in r]), timeout=np.mean([x['timeout'] for x in r]),
                   time_s=np.median([x['time_s'] for x in ok]) if ok else float('nan'),
                   err_cm=100 * np.median([x['final_dist_true'] for x in ok]) if ok else float('nan'),
                   yaw_deg=57.3 * np.median([x['final_dyaw_true'] for x in ok]) if ok else float('nan'),
                   clear_cm=100 * np.median([x['clear_min'] for x in r]), steer_rad=np.median([x['travel'] for x in r]))
        rows.append(row)
        tracks.append((f'{s}->{g}', np.array(tr), leg_path))
        print(f"{row['leg']:10s} {row['length']:4.2f} m  성공 {row['success']:.2f} 충돌 {row['collision']:.2f} 시간초과 {row['timeout']:.2f}  "
              f"시간 {row['time_s']:5.1f} s  오차 {row['err_cm']:4.1f} cm · {row['yaw_deg']:4.1f}°  최소여유 {row['clear_cm']:4.1f} cm  조향이동 {row['steer_rad']:5.1f} rad", flush=True)

tot = sum(r['time_s'] for r in rows)
print(f"합계: 성공 {np.mean([r['success'] for r in rows]):.3f} · 충돌 {np.mean([r['collision'] for r in rows]):.3f} · 시간 (성공 중앙값 합) {tot:.1f} s · "
      f"조향 이동 합 {sum(r['steer_rad'] for r in rows):.0f} rad")

if a.plot:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    w = env.world
    mi = env.map_names.index('dice')
    D = w.dfield[mi].cpu().numpy()
    o = w.origin[mi].cpu().numpy()
    fig, ax = plt.subplots(figsize=(7, 9))
    ax.imshow(D <= 0, origin='lower', cmap='Greys', extent=[o[0], o[0] + D.shape[1] * w.res, o[1], o[1] + D.shape[0] * w.res])
    for name, tr, path in tracks:
        ax.plot(path[:, 0], path[:, 1], '--', lw=0.8, color='tab:blue')
        ax.plot(tr[:, 0], tr[:, 1], lw=1.2, color='tab:red')
        for q in tr[::20]:
            ax.arrow(q[0], q[1], 0.08 * np.cos(q[2]), 0.08 * np.sin(q[2]), head_width=0.03, color='tab:red', lw=0.5)
    ax.set_title(('policy ' + Path(a.checkpoint).parent.name) if a.checkpoint else 'baseline follower')
    ax.set_aspect('equal')
    fig.savefig(a.plot, dpi=130, bbox_inches='tight')
    print('그림 →', a.plot)
