#!/usr/bin/env python3
"""주사위 미션 경기장에서 노드 사이 구간 주행 시간 측정 (env 1개 = 구간 1개, 동시에 달림).

입력 --paths: 구간별 웨이포인트 JSON ({'paths': {'A>B': {'wp': [[x, y], ...]}}, 'heading': {노드: yaw}}, 경기장 m 좌표)
주행: SwerveAction (배포판 nt_swerve = 팀 swerve.cpp 규칙, 가속 제한 · 조향 속도 · 조향 범위 포함) 에
      전방 주시 추종기 (몸 방향 유지한 채 평행 이동 + 도착 방향으로 천천히 회전, 끝에서 감속) 로 (vx, vy, wz) 명령.
      Nav2 MPPI 와 다른 추종기라 실제 Nav2 시간과는 차이가 있음.
도착: 목표 2 cm · 방향 0.03 rad 안 + 속도 1 cm/s 아래. 시간 초과 = 막힘으로 기록.
속도 프로필 --profile: confirmed = 실물 확인 범위 (0.13 m/s · 0.2 m/s²), hardware = 하드웨어 끝 (0.261 m/s · 1.0 m/s²)
예) OMNI_KIT_ACCEPT_EULA=YES env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/measure_dice_routes.py \
      --paths results/dice_paths.json --profile confirmed --out results/dice_route_times_confirmed.json
"""
import argparse
import json
import math
import sys
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--paths', required=True)
parser.add_argument('--out', required=True)
parser.add_argument('--profile', choices=['confirmed', 'hardware'], default='confirmed')
parser.add_argument('--steer_tol', type=float, default=None, help='조향 정렬 허용각 [rad] (기본 = 환경 기본값)')
parser.add_argument('--timeout_s', type=float, default=150.0)
parser.add_argument('--lookahead', type=float, default=0.20)
add_launcher_args(parser)
args = parser.parse_args()

import torch

from isaaclab.utils import instantiate

from sim_core import dice_mission as dm
from sim_core import turtleninja_maze_env as tme

PROFILES = {   # max_linear, max_linear_accel, max_angular, max_angular_accel (회전은 직선과 같은 비율로 줄임)
    'confirmed': (0.13, 0.2, 1.675 * 0.13 / 0.2612, 6.0 * 0.2),
    'hardware': (0.2612, 1.0, 1.675, 6.0),
}
VMAX, ACC, WMAX, WACC = PROFILES[args.profile]
P = json.loads(_P(args.paths).read_text())
keys = sorted(P['paths'])
n = len(keys)
o = dm.LAYOUT['arena_origin_in_sim_m']   # 시뮬 = 경기장 + o

cfg = dm.DiceMissionEnvCfg()
cfg.scene.num_envs = n
b = cfg.actions.base
b.max_linear, b.max_linear_accel, b.max_angular, b.max_angular_accel = VMAX, ACC, WMAX, WACC
if args.steer_tol is not None:
    b.steer_tolerance = args.steer_tol

with launch_simulation(cfg, args):
    env = instantiate(cfg)
    dev = env.device
    dt = cfg.sim.dt * cfg.decimation
    env.reset()
    robot = env.scene['robot']
    org = env.scene.env_origins
    wps = [torch.tensor([[x + o[0], y + o[1]] for x, y in P['paths'][k]['wp']], device=dev) for k in keys]
    goal_yaw = torch.tensor([P['heading'][k.split('>')[1]] for k in keys], device=dev)
    start_yaw = torch.tensor([P['heading'][k.split('>')[0]] for k in keys], device=dev)
    # 구간 시작 자세로 옮김
    pos = org.clone()
    pos[:, :2] += torch.stack([w[0] for w in wps])
    pos[:, 2] += tme.SPAWN_Z
    z = torch.zeros(n, device=dev)
    ids = torch.arange(n, device=dev)
    robot.write_root_pose_to_sim_index(root_pose=torch.cat([pos, tme.quat_from_euler_xyz(z, z, start_yaw)], -1), env_ids=ids)
    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    action = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
    for _ in range(50):   # 1 s 정착 (명령 0)
        env.step(action)

    seg = [0] * n                         # 현재 웨이포인트 구간 번호
    done_t = [None] * n
    travel = torch.zeros(n, device=dev)
    prev = robot.data.root_pos_w.torch[:, :2].clone()
    gate_steps = torch.zeros(n, device=dev)
    t, last_print = 0.0, 0.0
    while env.sim.is_running() and t < args.timeout_s and any(d is None for d in done_t):
        with torch.inference_mode():
            xy = robot.data.root_pos_w.torch[:, :2] - org[:, :2]
            yaw = tme.euler_xyz_from_quat(robot.data.root_quat_w.torch)[2]
            vel = robot.data.root_lin_vel_w.torch[:, :2].norm(dim=1)
            cmd = torch.zeros(n, 3, device=dev)
            for e in range(n):
                if done_t[e] is not None:
                    continue
                w = wps[e]
                p = xy[e]
                # 지난 웨이포인트 넘기기: 다음 점에 가까우면 구간 진행
                while seg[e] < len(w) - 2 and (p - w[seg[e] + 1]).norm() < args.lookahead:
                    seg[e] += 1
                # 남은 거리 (경로 따라)
                rem = (w[seg[e] + 1] - p).norm() + sum((w[k + 1] - w[k]).norm() for k in range(seg[e] + 1, len(w) - 1))
                # 전방 주시점: 현재 구간 위 투영점에서 lookahead 앞
                a, c = w[seg[e]], w[seg[e] + 1]
                ab = c - a
                L = ab.norm().clamp(min=1e-6)
                s = ((p - a) @ ab / L).clamp(0, float(L))
                tgt = a + ab / L * min(float(L), float(s) + args.lookahead)
                if seg[e] == len(w) - 2 and rem < args.lookahead:
                    tgt = w[-1]
                d = tgt - p
                dn = d.norm().clamp(min=1e-6)
                v = min(VMAX, math.sqrt(max(0.0, 2 * 0.8 * ACC * float(rem))), max(0.02, 2.0 * float(rem)))
                vw = d / dn * v
                yerr = math.remainder(float(goal_yaw[e] - yaw[e]), 2 * math.pi)
                wz = max(-WMAX, min(WMAX, 1.5 * yerr))
                cy, sy = math.cos(float(yaw[e])), math.sin(float(yaw[e]))
                cmd[e, 0] = (cy * vw[0] + sy * vw[1]) / VMAX
                cmd[e, 1] = (-sy * vw[0] + cy * vw[1]) / VMAX
                cmd[e, 2] = wz / WMAX
                if float(rem) < 0.02 and abs(yerr) < 0.03 and float(vel[e]) < 0.01:
                    done_t[e] = t
                    cmd[e] = 0.0
            action[:, :3] = cmd.clamp(-1, 1)
            env.step(action)
            t += dt
            now = robot.data.root_pos_w.torch[:, :2]
            travel += (now - prev).norm(dim=1)
            prev = now.clone()
            gate_steps += (~env.action_manager.get_term('base').aligned).float()
            if t - last_print > 10:
                print(f'[routes] t {t:.0f} s, 도착 {sum(d is not None for d in done_t)}/{n}', flush=True)
                last_print = t

    res = {'profile': args.profile, 'limits': {'max_linear': VMAX, 'max_linear_accel': ACC, 'max_angular': WMAX, 'max_angular_accel': WACC,
                                                'steer_tolerance': cfg.actions.base.steer_tolerance},
           'segments': {}}
    for e, k in enumerate(keys):
        res['segments'][k] = {'time_s': None if done_t[e] is None else round(done_t[e], 2), 'path_len_m': P['paths'][k]['len_m'],
                              'travel_m': round(float(travel[e]), 3), 'gate_blocked_s': round(float(gate_steps[e]) * dt, 2)}
    _P(args.out).parent.mkdir(parents=True, exist_ok=True)
    _P(args.out).write_text(json.dumps(res, indent=1))
    ok = [v for v in res['segments'].values() if v['time_s'] is not None]
    print(f"[routes] {args.profile}: 도착 {len(ok)}/{n}, 평균 속도 {sum(v['path_len_m'] for v in ok) / max(1e-6, sum(v['time_s'] for v in ok)):.3f} m/s, "
          f"막힘 {[k for k, v in res['segments'].items() if v['time_s'] is None]}", flush=True)
    env.close()
