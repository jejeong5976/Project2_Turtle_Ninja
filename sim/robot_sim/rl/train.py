#!/usr/bin/env python3
"""스워브 경로 추종 정책 학습 (torch 간이 플랜트, Isaac 안 띄움). rsl_rl PPO.

예) cd sim/robot_sim
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --max_iterations 1500          # 기본 16384 env, 약 3.2 s/iter
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --residual_lin 0.03 --residual_ang 0.15 --run_name res003
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task swerve_track --run_name track   # 속도 추종 (명령 은행 필요)
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task local_planner_nav2 --paths navfn --run_name nf1   # 실제 Nav2 경로
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task local_dock --run_name dock1   # 정밀 접근 (rl/tasks/local_dock)
    env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/train.py --task local_dock3 --run_name dock3  # 정밀 접근 성공 1 cm (rl/tasks/local_dock3)
결과: rl/logs/<과제>/<시각>_<이름>/ (model_*.pt, TensorBoard, cfg.json)
"""
import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # sim/robot_sim
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402


p = argparse.ArgumentParser()
p.add_argument('--task', choices=['swerve_follow', 'swerve_track', 'local_planner', 'local_planner_nav2', 'local_dock', 'local_dock3', 'local_planner_nf4', 'local_planner_nf5', 'local_planner_nf6', 'local_planner_nf7'], default='swerve_follow')
p.add_argument('--num_envs', type=int, default=None, help='기본 = 과제 cfg 값')
p.add_argument('--max_iterations', type=int, default=None, help='기본 = 과제 cfg 값')
p.add_argument('--device', default='cuda')
p.add_argument('--seed', type=int, default=1)
p.add_argument('--action_mode', choices=['residual', 'direct'], default='residual')
p.add_argument('--residual_lin', type=float, default=None)
p.add_argument('--residual_ang', type=float, default=None)
p.add_argument('--hardware_profile', action='store_true', help='속도 한계 프로필에 0.2612 m/s · 1.0 m/s² 추가')
p.add_argument('--no_curriculum', action='store_true')
p.add_argument('--paths', default='navfn', help='local_planner_nav2: 경로 은행 (navfn · smac2d · smac2d_far · imitation)')
p.add_argument('--run_name', default='')
p.add_argument('--set', nargs='*', default=[], help='설정 덮어쓰기 sec.key=값 (예: env.rot_limit=False reward.goal_yaw_far=0)')
p.add_argument('--resume', default='', help='이어 학습할 model_*.pt (같은 설계 실험 이어가기용)')
a = p.parse_args()

if a.task == 'swerve_track':
    from swerve_track.cfg import Cfg
    from swerve_track.env import SwerveTrackEnv as Env
elif a.task == 'local_planner':
    from local_planner.cfg import Cfg
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_dock':
    from local_dock.cfg import Cfg
    from local_planner.env import LocalPlannerEnv as Env      # 환경 같음, 구간 · 속도 · 보상만 다름
elif a.task == 'local_dock3':
    from local_dock3.cfg import Cfg                             # dock2 + 성공 1 cm (rl/tasks/local_dock3)
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_planner_nf6':
    from local_planner_nf6.cfg import Cfg                       # nf5 + 헛회전 · 회전 반전 벌점 (rl/tasks/local_planner_nf6)
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_planner_nf7':
    from local_planner_nf7.cfg import Cfg                       # nf6 + arena5 구간 · 박스 어긋남 · yaw 흐름 · 큰 튐 (rl/tasks/local_planner_nf7)
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_planner_nf5':
    from local_planner_nf5.cfg import Cfg                       # nf4 + PolygonStop 구역 학습 (rl/tasks/local_planner_nf5)
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_planner_nf4':
    from local_planner_nf4.cfg import Cfg                       # nf3 + 목표 앞 정지 고침 (rl/tasks/local_planner_nf4)
    from local_planner.env import LocalPlannerEnv as Env
elif a.task == 'local_planner_nav2':
    from local_planner_nav2.cfg import Cfg
    from local_planner.env import LocalPlannerEnv as Env      # 환경은 local_planner 그대로, 경로 은행만 다름
else:
    from swerve_follow.cfg import Cfg
    from swerve_follow.env import SwerveFollowEnv as Env

cfg = Cfg(paths=a.paths) if a.task in ('local_planner_nav2', 'local_planner_nf4', 'local_planner_nf5', 'local_planner_nf6', 'local_planner_nf7') else Cfg()
if a.num_envs is not None:
    cfg.env.num_envs = a.num_envs
cfg.env.device, cfg.env.seed = a.device, a.seed
if a.task == 'swerve_follow':
    cfg.env.action_mode = a.action_mode
if a.residual_lin is not None:
    cfg.env.residual_lin = a.residual_lin
if a.residual_ang is not None:
    cfg.env.residual_ang = a.residual_ang
if a.hardware_profile and a.task == 'swerve_follow':
    cfg.env.limit_profiles = tuple(cfg.env.limit_profiles) + ((0.2612, 1.0),)
if a.no_curriculum and a.task == 'swerve_follow':
    cfg.env.curriculum = False
for kv in a.set:                                       # 비교 실험용 (cfg.json 에 그대로 남음)
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
