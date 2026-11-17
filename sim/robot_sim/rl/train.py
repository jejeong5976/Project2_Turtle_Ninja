#!/usr/bin/env python3
"""주행 RL 정책 학습 (torch 간이 플랜트, Isaac 안 띄움). rsl_rl PPO.

과제 = 배포 정책 넷 (rl/tasks/<과제>/cfg.py). 바탕 cfg · env 는 local_planner (env · 보상 · DR) · local_planner_nav2 (실제 Nav2 경로) · local_dock (정밀 접근 구간)
  주행     local_planner_nf5 (config/rl/nav_nf5) → local_planner_nf7 (nav_nf7)
  정밀 접근 local_dock3 (config/rl/dock3) → local_dock5 (dock5)

예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task local_planner_nf7 --paths navfn --run_name nf7
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task local_dock5 --run_name dock5
결과: rl/logs/<과제>/<시각>_<이름>/ (model_*.pt, TensorBoard, cfg.json)
"""
import argparse
import dataclasses
import importlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # sim/robot_sim
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

# 과제 → (환경 모듈, 클래스). 환경은 local_planner 하나 (dock5 만 측정기 흉내 · 앞쪽 라이다가 붙은 DockEnv)
TASKS = {
    'local_planner_nf5': ('local_planner.env', 'LocalPlannerEnv'),
    'local_planner_nf7': ('local_planner.env', 'LocalPlannerEnv'),
    'local_dock3': ('local_planner.env', 'LocalPlannerEnv'),
    'local_dock5': ('local_dock5.env', 'DockEnv'),
}
PATH_TASKS = ('local_planner_nf5', 'local_planner_nf7')   # 경로 은행 고름

p = argparse.ArgumentParser()
p.add_argument('--task', choices=list(TASKS), default='local_planner_nf7')
p.add_argument('--num_envs', type=int, default=None, help='기본 = 과제 cfg 값')
p.add_argument('--max_iterations', type=int, default=None, help='기본 = 과제 cfg 값')
p.add_argument('--device', default='cuda')
p.add_argument('--seed', type=int, default=1)
p.add_argument('--paths', default='navfn', help='주행 과제: 경로 은행 (navfn · smac2d · smac2d_far · imitation)')
p.add_argument('--run_name', default='')
p.add_argument('--set', nargs='*', default=[], help='설정 덮어쓰기 sec.key=값 (예: env.rot_limit=False reward.goal_yaw_far=0)')
p.add_argument('--resume', default='', help='이어 학습할 model_*.pt')
a = p.parse_args()

Cfg = importlib.import_module(f'{a.task}.cfg').Cfg
env_mod, env_cls = TASKS[a.task]
Env = getattr(importlib.import_module(env_mod), env_cls)

cfg = Cfg(paths=a.paths) if a.task in PATH_TASKS else Cfg()
if a.num_envs is not None:
    cfg.env.num_envs = a.num_envs
cfg.env.device, cfg.env.seed = a.device, a.seed
for kv in a.set:                                       # 설정 덮어쓰기 (cfg.json 에 그대로 남음)
    key, val = kv.split('=', 1)
    sec, k = key.split('.', 1)
    obj = getattr(cfg, sec)
    old = getattr(obj, k)
    setattr(obj, k, (val.lower() == 'true') if isinstance(old, bool) else type(old)(val))
    print(f'[train] {key} = {getattr(obj, k)} (기본 {old})')
if a.max_iterations is not None:
    cfg.train.max_iterations = a.max_iterations
cfg.train.run_name = a.run_name
torch.manual_seed(a.seed)

log_dir = ROOT / 'rl' / 'logs' / a.task / (time.strftime('%Y%m%d_%H%M%S') + (f'_{a.run_name}' if a.run_name else ''))
log_dir.mkdir(parents=True, exist_ok=True)
(log_dir / 'cfg.json').write_text(json.dumps(dataclasses.asdict(cfg), indent=1, ensure_ascii=False))

env = Env(cfg)
runner = OnPolicyRunner(env, cfg.train.runner_dict(), str(log_dir), a.device)
if a.resume:
    runner.load(a.resume)
print(f'[train] 관측 actor {env.obs["policy"].shape[1]} · critic 추가 {env.obs["privileged"].shape[1]}, env {cfg.env.num_envs}, → {log_dir}', flush=True)
runner.learn(cfg.train.max_iterations)
runner.save(str(log_dir / f'model_{cfg.train.max_iterations}.pt'))
runner.export_policy_to_jit(str(log_dir / 'exported'), 'policy.pt')
print(f'[train] 끝 → {log_dir}')
