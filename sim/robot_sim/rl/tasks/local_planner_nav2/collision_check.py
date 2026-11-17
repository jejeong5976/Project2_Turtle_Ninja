#!/usr/bin/env python3
"""충돌 시험: 체크포인트를 학습 env (마지막 커리큘럼 단계, 사람 있음) 에서 평균 행동으로 돌려 충돌 원인 · 회전 중 충돌을 셈.
원인: 목표 0.3 m 안 (정밀 정차) · 사람 (사람 빼면 안 닿음) · 주행 중 벽 · 박스. 주행 중 벽 충돌은 직전 회전 속도도 봄. 목표 아닌 곳 멈춤 시간도.
local_planner · local_planner_nav2 체크포인트 둘 다 (cfg.json 의 env · dr · reward 그대로, 경로 은행 · dice 구간 포함).
예) env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/local_planner_nav2/collision_check.py rl/logs/local_planner_nav2/<run>/model_300.pt
    [--people 0] (사람 없이) [--steps 1500] [--num_envs 4096]
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from local_planner.cfg import Cfg  # noqa: E402
from local_planner.env import LocalPlannerEnv  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('checkpoint')
p.add_argument('--steps', type=int, default=1500)
p.add_argument('--num_envs', type=int, default=4096)
p.add_argument('--people', type=float, default=-1, help='사람 확률 (기본 = 학습 마지막 단계 값)')
p.add_argument('--dice_only', action='store_true')
a = p.parse_args()

ck = Path(a.checkpoint)
saved = json.loads((ck.parent / 'cfg.json').read_text())
c = Cfg()
for sec in ('env', 'dr', 'reward'):
    for k, v in saved[sec].items():
        if hasattr(getattr(c, sec), k):
            setattr(getattr(c, sec), k, tuple(v) if isinstance(v, list) else v)
if 'std_range' in saved.get('train', {}):
    from local_planner_nav2.cfg import TrainCfg
    c.train = TrainCfg(std_range=tuple(saved['train']['std_range']))
c.env.num_envs, c.env.curriculum = a.num_envs, False
if a.people >= 0:
    c.env.people_prob = (a.people,) * 3
if a.dice_only:
    c.env.dice_prob = 1.0
env = LocalPlannerEnv(c)
env.level = 2
r = OnPolicyRunner(env, c.train.runner_dict(), None, 'cuda')
r.load(str(ck))
pol = r.get_inference_policy('cuda')

tot = dict(done=0, success=0, collision=0, near=0, person=0, wall=0, wall_rot=0, timeout=0, lost=0)
wz_wall, cap, stall = [], [], []
with torch.inference_mode():
    obs = env.get_observations()
    for _ in range(a.steps):
        pre_pose, pre_w = env.plant.pose.clone(), env.plant.odom[:, 2].abs().clone()
        obs, _, _, ex = env.step(pol(obs))
        if '/act/rot_capped' in ex['log']:
            cap.append(float(ex['log']['/act/rot_capped']))
        d = env.last_done
        if d is None:
            continue
        tot['done'] += len(d['ids'])
        stall += d['stall_s'].tolist()
        for k in ('success', 'collision', 'timeout', 'lost'):
            tot[k] += int(d[k].sum())
        if d['collision'].any():
            ids = d['ids'][d['collision']]
            near = d['final_dist_true'][d['collision']] < 0.3
            mapc = env.world.clearance(pre_pose[ids], env.fp_collide, env.world.map_id[ids], people=False)
            by_person = (mapc >= 0.02) & ~near
            wall = ~by_person & ~near
            tot['near'] += int(near.sum())
            tot['person'] += int(by_person.sum())
            tot['wall'] += int(wall.sum())
            tot['wall_rot'] += int((wall & (pre_w[ids] > 0.2)).sum())
            wz_wall += pre_w[ids][wall].tolist()
n = max(tot['done'], 1)
print(f'{ck.parent.name}/{ck.name}: 에피소드 {tot["done"]}, 성공 {tot["success"] / n * 100:.1f} %, 충돌 {tot["collision"] / n * 100:.1f} % '
      f'(목표 근처 {tot["near"] / n * 100:.1f} · 사람 {tot["person"] / n * 100:.1f} · 주행 중 벽 {tot["wall"] / n * 100:.1f} %, '
      f'그중 0.2 rad/s 넘게 돌던 것 {tot["wall_rot"] / n * 100:.1f} %), 시간 초과 {tot["timeout"] / n * 100:.1f} %, 이탈 {tot["lost"] / n * 100:.1f} %')
if wz_wall:
    w = torch.tensor(wz_wall)
    print(f'  주행 중 벽 충돌 직전 |wz| 중앙 {float(w.median()):.3f} rad/s')
if stall:
    st = torch.tensor(stall)
    print(f'  목표 아닌 곳 멈춤 (이동 < 0.02 m/s, 사람 0.6 m 안 · 출발 0.5 s 뺌): 에피소드 평균 {float(st.mean()):.2f} s, '
          f'1 s 넘게 멈춘 에피소드 {float((st > 1.0).float().mean()) * 100:.1f} %')
if cap:
    print(f'  회전 상한이 걸린 스텝 비율 {sum(cap) / len(cap) * 100:.1f} %')
