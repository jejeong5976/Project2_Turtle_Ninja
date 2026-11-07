"""스워브 속도 추종 학습 환경 (rsl_rl VecEnv, torch 만). 설계 README.md.

한 정책 스텝 (20 Hz) = 컨트롤러 · 플랜트 5 틱 (100 Hz):
  명령 은행의 이번 목표 c_t (MPPI /cmd_vel 대용) + 잔차 a → 속도 한계 → 명령 큐 (지연 d_cmd) → swerve_torch (nt_swerve) → 플랜트.
  목표가 정지 (|c| < stop) 면 잔차 0 = 정지 명령은 그대로 통과 (collision_monitor · /mission/hold 가 항상 이김).
보상 = 참 몸 속도가 c_t 를 따라가는 정도 (지수) − 조향 이동량 − 조향 범위 끝 − 행동 크기 · 변화.
에피소드 = 명령 열 하나 (최대 20 s) 끝까지, 끝 = truncation (연속 주행 과제라 bootstrap). 실패 종료 없음.
플랜트 · 컨트롤러 · DR = swerve_follow 것 그대로 (자세 · 경로는 안 씀).
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import torch
from tensordict import TensorDict

from rsl_rl.env import VecEnv
from sim_core.swerve_torch import SwerveGeometry, SwerveTorch
from swerve_follow.plant import SwervePlant

from .cfg import Cfg

W_PER_V = 1.675 / 0.2612
ANG_ACC_PER_LIN = 6.0
RING = 8
FRAME = 23          # 조향 sin · cos · 범위 위치 12, 바퀴 4, odom 3, 자이로 z 1, 보낸 명령 3


class SwerveTrackEnv(VecEnv):
    def __init__(self, cfg: Cfg):
        self.full_cfg = cfg
        c = cfg.env
        self.c, self.dr, self.rw = c, cfg.dr, cfg.reward
        self.cfg = dataclasses.asdict(cfg)
        self.device = dev = c.device
        self.num_envs = B = c.num_envs
        self.num_actions = 3
        self.gen = torch.Generator(device=dev).manual_seed(c.seed)
        self.dt = c.ctrl_dt * c.decimation
        self.geom = SwerveGeometry(dev, torch.float32)
        self.ctrl = SwerveTorch(B, dev, geom=self.geom)
        self.plant = SwervePlant(B, self.geom, c.ctrl_dt)
        self._load_bank(c.cmd_bank, c.split, c.sources)

        z = lambda *s: torch.zeros(*s, device=dev)  # noqa: E731
        self.seq = torch.zeros(B, dtype=torch.long, device=dev)       # 은행 열 번호
        self.t = torch.zeros(B, dtype=torch.long, device=dev)         # 열 안 위치
        self.vmax, self.amax, self.wmax = z(B), z(B), z(B)
        self.d_cmd = torch.zeros(B, dtype=torch.long, device=dev)
        self.d_obs = torch.zeros(B, dtype=torch.long, device=dev)
        self.state_ring = z(B, RING, 12)          # 조향 4 · 바퀴 4 · odom 3 · 자이로 1
        self.cmd_ring = z(B, RING, 3)
        self.tick = 0
        self.hist = z(B, c.history, FRAME)
        self.last_action = z(B, 3)
        self.last_cmd = z(B, 3)
        self.target = z(B, 3)
        self.prev_steer_goal = z(B, 4)
        self.episode_length_buf = torch.zeros(B, dtype=torch.long, device=dev)
        self.max_episode_length = 10 ** 6
        self.first = torch.ones(B, dtype=torch.bool, device=dev)
        self.ep = {k: z(B) for k in ('ret', 'n', 'se_lin', 'se_ang', 'n_chg', 'se_lin_chg', 'se_ang_chg', 'travel', 'flips',
                                      'end_steps', 'gate', 'res_lin', 'res_ang')}
        self.extras = {}
        self.last_done = None
        self._physics = torch.compile(self._physics_impl, dynamic=False) if c.compile else self._physics_impl
        ids = torch.arange(B, device=dev)
        self._reset_idx(ids)
        self.obs = self._compute_obs(ids)

    # ------------------------------------------------------------------ 은행
    def _load_bank(self, path, split, sources):
        d = np.load(path)
        m = (d['split'] == split) & np.isin(d['src'], sources)
        dev = self.device
        self.bank_cmd = torch.tensor(d['cmd'][m], device=dev)           # (N, SEQ, 3)
        self.bank_n = torch.tensor(d['n'][m], dtype=torch.long, device=dev)
        self.bank_vmax = torch.tensor(d['vmax'][m], device=dev)
        self.bank_amax = torch.tensor(d['amax'][m], device=dev)
        self.bank_src = torch.tensor(d['src'][m], dtype=torch.long, device=dev)
        self.N, self.SEQ = self.bank_cmd.shape[0], self.bank_cmd.shape[1]
        assert self.N > 0, '명령 은행이 비었음 (tools/make_cmd_bank.py)'

    def _rand(self, n, lo, hi):
        return lo + (hi - lo) * torch.rand(n, device=self.device, generator=self.gen)

    def _cmd_at(self, k):
        """열 안 위치 k (B,) 의 명령 (끝 넘으면 0)"""
        kk = k.clamp(max=self.SEQ - 1)
        c = self.bank_cmd[self.seq, kk]
        return torch.where((k < self.bank_n[self.seq]).unsqueeze(-1), c, torch.zeros_like(c))

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, ids):
        if len(ids) == 0:
            return
        c, dr, n, dev = self.c, self.dr, len(ids), self.device
        if c.sequential:
            self.seq[ids] = ids % self.N
        else:
            self.seq[ids] = torch.randint(0, self.N, (n,), device=dev, generator=self.gen)
        self.t[ids] = 0
        k = self.seq[ids]
        self.vmax[ids], self.amax[ids] = self.bank_vmax[k], self.bank_amax[k]
        self.wmax[ids] = self.vmax[ids] * W_PER_V
        self.ctrl.max_linear[ids], self.ctrl.max_linear_accel[ids] = self.vmax[ids], self.amax[ids]
        self.ctrl.max_angular[ids], self.ctrl.max_angular_accel[ids] = self.wmax[ids], self.amax[ids] * ANG_ACC_PER_LIN
        self.plant.reset_motion(ids, torch.zeros(n, 3, device=dev))
        rnd = self.first[ids] | (self._rand(n, 0, 1) < c.random_steer_prob)
        rid = ids[rnd]
        if len(rid):
            g = self.geom
            self.plant.q[rid] = g.rmin + (g.rmax - g.rmin) * torch.rand(len(rid), 4, device=dev, generator=self.gen)
            self.ctrl.reset(rid)
        self.ctrl.stop(ids[~rnd])
        self.first[ids] = False
        lv = torch.tensor(dr.steer_rate_levels, device=dev)
        self.plant.rate_max[ids] = lv[torch.randint(0, len(lv), (n,), device=dev, generator=self.gen)] * self._rand(n, *dr.steer_rate_scale)
        self.plant.tau_s[ids] = self._rand(n, *dr.tau_steer)
        self.plant.tau_w[ids] = self._rand(n, *dr.tau_wheel)
        self.plant.tau_b[ids] = self._rand(n, *dr.tau_body)
        self.plant.slip[ids] = dr.slip[0] + (dr.slip[1] - dr.slip[0]) * torch.rand(n, 4, device=dev, generator=self.gen)
        self.d_cmd[ids] = torch.randint(dr.cmd_delay_ticks[0], dr.cmd_delay_ticks[1] + 1, (n,), device=dev, generator=self.gen)
        self.d_obs[ids] = torch.randint(dr.obs_delay_ticks[0], dr.obs_delay_ticks[1] + 1, (n,), device=dev, generator=self.gen)
        self.state_ring[ids] = self._state_now()[ids].unsqueeze(1)
        self.cmd_ring[ids] = 0.0
        self.last_action[ids] = 0.0
        self.last_cmd[ids] = 0.0
        self.prev_steer_goal[ids] = self.plant.q[ids]
        self.episode_length_buf[ids] = 0
        for v in self.ep.values():
            v[ids] = 0.0

    # ------------------------------------------------------------------ 상태 · 관측
    def _state_now(self):
        return torch.cat([self.plant.q, self.plant.w, self.plant.odom, self.plant.vel_w[:, 2:3]], -1)

    def _delayed(self, ring, d):
        return ring[torch.arange(self.num_envs, device=self.device), (self.tick - 1 - d) % RING]

    def _compute_obs(self, reset_ids=None):
        c, dr, g = self.c, self.dr, self.geom
        B, dev = self.num_envs, self.device
        st = self._delayed(self.state_ring, self.d_obs) if self.tick > 0 else self._state_now()
        q, w, odom, gyro = st[:, :4], st[:, 4:8], st[:, 8:11], st[:, 11:12]
        q = q + torch.randn(B, 4, device=dev, generator=self.gen) * dr.noise_steer
        nz = torch.tensor([dr.noise_lin_vel, dr.noise_lin_vel, dr.noise_ang_vel], device=dev)
        odom = odom + torch.randn(B, 3, device=dev, generator=self.gen) * nz
        gyro = gyro + torch.randn(B, 1, device=dev, generator=self.gen) * dr.noise_ang_vel
        lim3 = torch.stack([self.vmax, self.vmax, self.wmax], -1)
        self.target = self._cmd_at(self.t)
        rpos = (q - g.rmin) / (g.rmax - g.rmin) * 2 - 1
        frame = torch.cat([q.sin(), q.cos(), rpos, w / self.plant.max_wheel, odom / lim3, gyro / self.wmax.unsqueeze(-1),
                           self.last_cmd / lim3], -1)
        self.hist = torch.cat([frame.unsqueeze(1), self.hist[:, :-1]], 1)
        if reset_ids is not None and len(reset_ids):
            self.hist[reset_ids] = frame[reset_ids].unsqueeze(1)
        parts = [self.target / lim3, (self.target - odom) / lim3]
        for j in range(1, c.preview + 1):
            parts.append(self._cmd_at(self.t + j * c.preview_every) / lim3)
        parts += [self.hist.reshape(B, -1), torch.stack([self.vmax / 0.26, self.amax / 1.0], -1)]
        policy = torch.cat(parts, -1)
        priv = torch.cat([self.plant.body_vel() / lim3,
                          torch.stack([self.plant.rate_max / 4.8, self.plant.tau_s / 0.15, self.plant.tau_w / 0.06, self.plant.tau_b / 0.08,
                                       self.d_cmd / 3.0, self.d_obs / 4.0, self.ctrl.blend_ratio, self.ctrl.gate_blocked.float()], -1),
                          self.plant.slip, self._cmd_at(self.t + 5) / lim3, self._cmd_at(self.t + 10) / lim3], -1)
        return TensorDict({'policy': policy, 'privileged': priv}, batch_size=[B], device=dev)

    def get_observations(self) -> TensorDict:
        return self.obs

    # ------------------------------------------------------------------ 명령
    def _command(self, a):
        c, tg = self.c, self.target
        rl = a[:, :2] * c.residual_lin
        n = rl.norm(dim=-1, keepdim=True)
        rl = rl * torch.where(n > c.residual_lin, c.residual_lin / n.clamp(min=1e-9), 1.0)
        res = torch.cat([rl, a[:, 2:3] * c.residual_ang], -1)
        stop = (tg[:, :2].norm(dim=-1) < c.stop_lin) & (tg[:, 2].abs() < c.stop_ang)
        res = torch.where(stop.unsqueeze(-1), torch.zeros_like(res), res)
        cmd = tg + res
        lin = cmd[:, :2]
        n = lin.norm(dim=-1)
        lin = lin * torch.where(n > self.vmax, self.vmax / n.clamp(min=1e-9), torch.ones_like(n)).unsqueeze(-1)
        wz = torch.maximum(torch.minimum(cmd[:, 2], self.wmax), -self.wmax)
        return torch.cat([lin, wz.unsqueeze(-1)], -1), res

    def _physics_impl(self, cmd_d):
        sg, wg = self.ctrl.update(cmd_d, self.plant.q, self.plant.w, self.plant.qd, self.c.ctrl_dt)
        flips = ((sg - self.prev_steer_goal).abs() > math.pi / 2).float().sum(-1)
        self.prev_steer_goal = sg
        q0 = self.plant.q
        self.plant.step(sg, wg)
        travel = (self.plant.q - q0).abs().sum(-1)
        return flips, travel, self.ctrl.gate_blocked.float(), self._state_now()

    # ------------------------------------------------------------------ 스텝
    def step(self, actions):
        c, rw, dev, g = self.c, self.rw, self.device, self.geom
        B = self.num_envs
        a = actions.clamp(-1.0, 1.0)
        cmd, res = self._command(a)
        ar = torch.arange(B, device=dev)
        flips = torch.zeros(B, device=dev)
        travel = torch.zeros(B, device=dev)
        gate = torch.zeros(B, device=dev)
        for _ in range(c.decimation):
            self.cmd_ring[:, self.tick % RING] = cmd
            f, tr, gt, st = self._physics(self.cmd_ring[ar, (self.tick - self.d_cmd) % RING])
            flips += f
            travel += tr
            gate += gt
            self.tick += 1
            self.state_ring[:, (self.tick - 1) % RING] = st
        self.last_cmd = cmd
        self.episode_length_buf += 1

        tg = self.target
        bv = self.plant.body_vel()
        e_lin2 = ((bv[:, :2] - tg[:, :2]) ** 2).sum(-1)
        e_ang2 = (bv[:, 2] - tg[:, 2]) ** 2
        q = self.plant.q
        end_d = torch.minimum(q - g.rmin, g.rmax - q)
        end_pen = ((c.steer_end_zone - end_d) / c.steer_end_zone).clamp(0, 1).mean(-1)
        rew = (rw.lin * torch.exp(-e_lin2 / rw.sigma_lin ** 2) + rw.ang * torch.exp(-e_ang2 / rw.sigma_ang ** 2)
               - rw.steer_travel * travel - rw.steer_end * end_pen
               - rw.action * (a ** 2).sum(-1) - rw.action_rate * ((a - self.last_action) ** 2).sum(-1))
        self.last_action = a

        # 지표: 변화 구간 = 0.5 s 전 명령과 차이가 큰 스텝
        prev = self._cmd_at((self.t - c.change_window).clamp(min=0))
        chg = ((tg[:, :2] - prev[:, :2]).norm(dim=-1) > c.change_lin) | ((tg[:, 2] - prev[:, 2]).abs() > c.change_ang)
        e = self.ep
        e['ret'] += rew
        e['n'] += 1
        e['se_lin'] += e_lin2
        e['se_ang'] += e_ang2
        e['n_chg'] += chg.float()
        e['se_lin_chg'] += e_lin2 * chg
        e['se_ang_chg'] += e_ang2 * chg
        e['travel'] += travel
        e['flips'] += flips
        e['end_steps'] += (end_d.min(-1).values < 0.0087).float()      # 0.5° 안
        e['gate'] += gate
        e['res_lin'] += res[:, :2].norm(dim=-1)
        e['res_ang'] += res[:, 2].abs()

        self.t += 1
        dones = self.t >= self.bank_n[self.seq]
        timeout = dones.clone()
        log = {}
        done_ids = torch.nonzero(dones).squeeze(-1)
        if len(done_ids):
            d = {k: v[done_ids].clone() for k, v in e.items()}
            n_ = d['n']
            nc = d['n_chg'].clamp(min=1)
            stats = {'rmse_lin': (d['se_lin'] / n_).sqrt(), 'rmse_ang': (d['se_ang'] / n_).sqrt(),
                     'rmse_lin_chg': (d['se_lin_chg'] / nc).sqrt(), 'rmse_ang_chg': (d['se_ang_chg'] / nc).sqrt(),
                     'has_chg': d['n_chg'] > 0, 'time_s': n_ * self.dt, 'travel_per_s': d['travel'] / (n_ * self.dt),
                     'flips_per_s': d['flips'] / (n_ * self.dt), 'end_frac': d['end_steps'] / n_, 'gate_s': d['gate'] * 0.01,
                     'res_lin': d['res_lin'] / n_, 'res_ang': d['res_ang'] / n_, 'src': self.bank_src[self.seq[done_ids]]}
            log = {'/ep/return': d['ret'].mean(), '/ep/rmse_lin': stats['rmse_lin'].mean(), '/ep/rmse_ang': stats['rmse_ang'].mean(),
                   '/ep/rmse_lin_chg': stats['rmse_lin_chg'].mean(), '/ep/travel_per_s': stats['travel_per_s'].mean(),
                   '/ep/flips_per_s': stats['flips_per_s'].mean(), '/ep/end_frac': stats['end_frac'].mean(),
                   '/ep/res_lin': stats['res_lin'].mean(), '/ep/res_ang': stats['res_ang'].mean()}
            self.last_done = {'ids': done_ids, **stats}
        else:
            self.last_done = None
        self._reset_idx(done_ids)
        self.obs = self._compute_obs(done_ids)
        self.extras = {'time_outs': timeout, 'log': log}
        return self.obs, rew, dones, self.extras
