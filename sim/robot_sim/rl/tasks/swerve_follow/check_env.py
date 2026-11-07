#!/usr/bin/env python3
"""swerve_follow 환경 검사 (Codex 검토 항목). 실행: cd sim/robot_sim && env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python rl/tasks/swerve_follow/check_env.py
1) 경로 투영: 끝 너머 · 시작 앞 이탈 거리, 헤어핀에서 창이 다른 가지로 안 넘어감
2) 플랜트: 직진 · 옆 이동 · 제자리 회전 속도 (조향 정렬 뒤 명령 속도 ±3 %)
3) 명령 지연: 계단 명령 뒤 컨트롤러 출력이 d_cmd 틱 늦게 움직임
4) 관측 지연: 관측 자세가 d_obs 틱 전 추정 자세와 같음 (현재 자세 누출 없음)
"""
import math
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'rl' / 'tasks'))
from sim_core.swerve_torch import SwerveGeometry, SwerveTorch  # noqa: E402
from swerve_follow.cfg import Cfg  # noqa: E402
from swerve_follow.env import SwerveFollowEnv  # noqa: E402
from swerve_follow.paths import PathBatch  # noqa: E402
from swerve_follow.plant import SwervePlant  # noqa: E402

DEV = 'cpu'
fails = []


def check(name, ok, msg=''):
    print(f'  {"OK " if ok else "실패"} {name} {msg}')
    if not ok:
        fails.append(name)


print('[1] 경로 투영')
pb = PathBatch(3, 64, DEV)
line = torch.zeros(64, 2); n = 21; line[:n, 0] = torch.linspace(0, 1, n); line[n:, 0] = 1.0
hair = torch.zeros(64, 2)   # 헤어핀: (0,0)→(1,0)→(1,0.1)→(0,0.1)
pts = torch.cat([torch.stack([torch.linspace(0, 1, 21), torch.zeros(21)], 1), torch.stack([torch.ones(2), torch.tensor([0.05, 0.1])], 1),
                 torch.stack([torch.linspace(0.95, 0, 20), torch.full((20,), 0.1)], 1)])
hair[:len(pts)] = pts; hair[len(pts):] = pts[-1]
pb.set(torch.arange(3), torch.stack([line, line, hair]), torch.tensor([n, n, len(pts)]), torch.zeros(3))
s, lat, dist, _ = pb.project(torch.tensor([[2.0, 0.0], [-1.0, 0.0], [0.3, 0.02]]), torch.tensor([1.0, 0.0, 0.3]), 0.10, 0.30)
check('끝 너머 1 m → 거리 1', abs(dist[0] - 1.0) < 1e-5, f'(거리 {dist[0]:.3f}, 횡오차 {lat[0]:.3f})')
check('시작 앞 1 m → 거리 1', abs(dist[1] - 1.0) < 1e-5, f'({dist[1]:.3f})')
check('헤어핀: 0.1 m 옆 가지로 안 넘어감', abs(s[2] - 0.3) < 1e-5 and abs(dist[2] - 0.02) < 1e-5, f'(s {s[2]:.3f}, 거리 {dist[2]:.3f})')

print('[2] 플랜트')
g = SwerveGeometry(DEV)
for name, cmd in (('직진 0.1', [0.1, 0.0, 0.0]), ('옆 0.1', [0.0, 0.1, 0.0]), ('회전 0.5', [0.0, 0.0, 0.5])):
    ctrl, pl = SwerveTorch(1, DEV, geom=g), SwervePlant(1, g, 0.01)
    pl.rate_max[:] = 4.8; pl.tau_s[:] = 0.05; pl.tau_w[:] = 0.03; pl.tau_b[:] = 0.03
    c = torch.tensor([cmd])
    for _ in range(300):
        sg, wg = ctrl.update(c, pl.q, pl.w, pl.qd, 0.01)
        pl.step(sg, wg)
    v = pl.body_vel()[0]
    err = (v - c[0]).abs().max().item()
    check(name, err < 0.03 * max(abs(x) for x in cmd) + 1e-3, f'(몸 속도 {v.tolist()})')

print('[3] 명령 지연 · [4] 관측 지연')
cfg = Cfg(); cfg.env.num_envs, cfg.env.device, cfg.env.compile = 8, DEV, False
cfg.env.curriculum = False; cfg.env.split = 0
cfg.dr.cmd_delay_ticks = (0, 3); cfg.dr.obs_delay_ticks = (0, 4); cfg.dr.map_jump_prob = 0.0
env = SwerveFollowEnv(cfg)
# 3) 기준 추종기 명령이 0 이 아니게 → 출발 순간 controller.out 이 처음 움직이는 틱 = d_cmd
env.ctrl.reset(torch.arange(8)); env.plant.reset_motion(torch.arange(8), torch.zeros(8, 3)); env.plant.q.zero_()   # 조향 0 (게이트 안 걸리게)
env.cmd_ring.zero_()
first_move = torch.full((8,), -1)
cmd = torch.tensor([[0.05, 0.0, 0.0]]).repeat(8, 1)
ar = torch.arange(8)
for k in range(8):
    env.cmd_ring[:, env.tick % 8] = cmd
    cd = env.cmd_ring[ar, (env.tick - env.d_cmd) % 8]
    env.ctrl.update(cd, env.plant.q, env.plant.w, env.plant.qd, 0.01)
    env.tick += 1
    moved = env.ctrl.out[:, 0].abs() > 0
    first_move = torch.where((first_move < 0) & moved, torch.tensor(k), first_move)
check('명령 지연 = d_cmd 틱', bool((first_move == env.d_cmd).all()), f'(d_cmd {env.d_cmd.tolist()}, 첫 움직임 {first_move.tolist()})')
# 4) 상태 링에 틱마다 다른 자세를 넣고 관측 자세 확인
for k in range(8):
    env.plant.pose[:, 0] = float(k)
    env.tick += 1
    env.state_ring[:, (env.tick - 1) % 8] = env._state_now()
st = env._delayed(env.state_ring, env.d_obs)
check('관측 = d_obs 틱 전 자세', bool(((7 - env.d_obs).float() - (st[:, 0] - env.map_err[:, 0])).abs().max() < 1e-5),
      f'(d_obs {env.d_obs.tolist()}, 관측 x {[round(v, 3) for v in (st[:, 0] - env.map_err[:, 0]).tolist()]})')
print('통과' if not fails else f'실패: {fails}')
sys.exit(1 if fails else 0)
