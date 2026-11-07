"""스워브 경로 추종 학습 환경 (rsl_rl VecEnv, torch 만, Isaac 안 띄움).

한 정책 스텝 (20 Hz) = 컨트롤러 · 플랜트 5 틱 (100 Hz):
  정책 행동 a → 명령 = 기준 추종기 + 잔차 (또는 direct) → 속도 한계 → 명령 큐 (지연 d_cmd 틱) → swerve_torch → 플랜트.
  관측은 d_obs 틱 지난 상태 (자세 = 참 자세에 map→odom 오차를 얹은 '추정 자세').
보상 · 종료는 '현재 추정 자세'(지연 없음) 기준 = Nav2 goal checker 와 같은 판정 (map→odom 오차는 컨트롤러가 못 고침, 범위 안에서 계단으로 바뀜).
진행도: 보상용 s_true (현재 추정 자세) 와 관측용 s_est (d_obs 지연 자세) 를 따로 추적. 참 자세 · DR 값은 critic 입력에만.
설계 = README.md (Claude · Codex 토의 결과).
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import torch
from tensordict import TensorDict

from rsl_rl.env import VecEnv
from sim_core.swerve_torch import SwerveGeometry, SwerveTorch

from .cfg import Cfg
from .follower import follow
from .paths import PathBatch, to_body, wrap
from .plant import SwervePlant

W_PER_V = 1.675 / 0.2612       # 각속도 한계 / 선속도 한계 (하드웨어 끝 비율)
ANG_ACC_PER_LIN = 6.0          # 각가속 / 선가속
RING = 8                       # 지연 버퍼 길이 (틱)


class SwerveFollowEnv(VecEnv):
    def __init__(self, cfg: Cfg):
        self.full_cfg = cfg
        c, dr = cfg.env, cfg.dr
        self.c, self.dr, self.rw = c, dr, cfg.reward
        self.cfg = dataclasses.asdict(cfg)          # rsl_rl 로거가 저장
        self.device = c.device
        self.num_envs = B = c.num_envs
        self.num_actions = 3
        dev = self.device
        self.gen = torch.Generator(device=dev).manual_seed(c.seed)
        self.dt = c.ctrl_dt * c.decimation

        self.geom = SwerveGeometry(dev, torch.float32)
        self.ctrl = SwerveTorch(B, dev, geom=self.geom)
        self.plant = SwervePlant(B, self.geom, c.ctrl_dt)
        self._load_bank(c.path_bank, c.split)
        self.path = PathBatch(B, self.bank_pts.shape[1], dev)

        z = lambda *s: torch.zeros(*s, device=dev)  # noqa: E731
        self.vmax, self.amax, self.wmax = z(B), z(B), z(B)
        self.s_true, self.s_est = z(B), z(B)
        self.map_err = z(B, 3)
        self.d_cmd = torch.zeros(B, dtype=torch.long, device=dev)
        self.d_obs = torch.zeros(B, dtype=torch.long, device=dev)
        self.state_ring = z(B, RING, 14)          # 추정 자세 3 · 조향 4 · 바퀴 4 · odom 3
        self.cmd_ring = z(B, RING, 3)
        self.tick = 0
        self.hist = z(B, c.history, 22)
        self.last_action = z(B, 3)
        self.last_cmd = z(B, 3)
        self.base_cmd = z(B, 3)
        self.prev_steer_goal = z(B, 4)
        self.hold = z(B)
        self.soft = z(B)
        self.episode_length_buf = torch.zeros(B, dtype=torch.long, device=dev)
        self.max_episode_length_t = torch.ones(B, dtype=torch.long, device=dev)
        self.max_episode_length = 10 ** 6          # rsl_rl init_at_random_ep_len 용 스칼라 (env 별 실제 값은 _t)
        self.first = torch.ones(B, dtype=torch.bool, device=dev)
        # 에피소드 지표
        self.ep = {k: z(B) for k in ('ret', 'flips', 'travel', 'lat_max', 'lat_sum', 'gate')}
        self.level = c.curriculum_start if c.curriculum else 2
        self.outcomes = torch.zeros(c.curriculum_window, device=dev)
        self.outcome_ptr = 0
        self.outcome_count = 0
        self.extras = {}
        # 작은 연산 수천 개라 커널 띄우는 시간이 대부분 → torch.compile 로 한 틱을 묶음 (c.compile)
        self._physics = torch.compile(self._physics_impl, dynamic=False) if c.compile else self._physics_impl
        self._reset_idx(torch.arange(B, device=dev))
        self.obs = self._compute_obs(torch.arange(B, device=dev))

    # ------------------------------------------------------------------ 경로 은행
    def _load_bank(self, path, split):
        d = np.load(path)
        m = d['split'] == split
        order = np.argsort(d['level'][m], kind='stable')
        dev = self.device
        self.bank_pts = torch.tensor(d['pts'][m][order], device=dev)
        self.bank_n = torch.tensor(d['n'][m][order], dtype=torch.long, device=dev)
        self.bank_yaw = torch.tensor(d['goal_yaw'][m][order], device=dev)
        lv = d['level'][m][order]
        self.bank_count_le = [int((lv <= k).sum()) for k in range(3)]
        assert self.bank_count_le[0] > 0, '등급 0 경로가 없음'

    def _rand(self, n, lo, hi):
        return lo + (hi - lo) * torch.rand(n, device=self.device, generator=self.gen)

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, ids):
        if len(ids) == 0:
            return
        c, dr, n, dev = self.c, self.dr, len(ids), self.device
        lvl = self.level
        # 경로: 등급 ≤ 현재 단계에서 균등, 시작점 기준 무작위 회전
        if c.sequential_paths:
            k = ids % self.bank_count_le[lvl]
        else:
            k = torch.randint(0, self.bank_count_le[lvl], (n,), device=dev, generator=self.gen)
        th = self._rand(n, -math.pi, math.pi) * float(c.random_rotation)
        R = torch.stack([torch.stack([th.cos(), -th.sin()], -1), torch.stack([th.sin(), th.cos()], -1)], -2)
        pts = self.bank_pts[k] @ R.transpose(1, 2)
        self.path.set(ids, pts, self.bank_n[k], wrap(self.bank_yaw[k] + th))
        # 속도 한계 프로필
        prof = torch.tensor(c.limit_profiles, device=dev)
        pi = torch.randint(0, len(c.limit_profiles), (n,), device=dev, generator=self.gen)
        self.vmax[ids], self.amax[ids] = prof[pi, 0], prof[pi, 1]
        self.wmax[ids] = self.vmax[ids] * W_PER_V
        self.ctrl.max_linear[ids], self.ctrl.max_linear_accel[ids] = self.vmax[ids], self.amax[ids]
        self.ctrl.max_angular[ids], self.ctrl.max_angular_accel[ids] = self.wmax[ids], self.amax[ids] * ANG_ACC_PER_LIN
        # 초기 자세: 경로 시작 ± 위치, 목표 yaw ± 범위
        pos = torch.randn(n, 2, device=dev, generator=self.gen) * c.init_pos_std[lvl]
        yr = math.radians(c.init_yaw_range_deg[lvl])
        yaw = wrap(self.path.goal_yaw[ids] + self._rand(n, -yr, yr))
        self.plant.reset_motion(ids, torch.cat([pos, yaw.unsqueeze(-1)], -1))
        # 조향 상태: 처음 · 일부는 범위 안 무작위 (컨트롤러 이력도 지움), 나머지는 직전 에피소드 끝 상태 그대로
        rnd = self.first[ids] | (self._rand(n, 0, 1) < c.random_steer_prob)
        rid = ids[rnd]
        if len(rid):
            g = self.geom
            self.plant.q[rid] = g.rmin + (g.rmax - g.rmin) * torch.rand(len(rid), 4, device=dev, generator=self.gen)
            self.ctrl.reset(rid)
        self.ctrl.stop(ids[~rnd])
        self.first[ids] = False
        # DR
        lv = torch.tensor(dr.steer_rate_levels, device=dev)
        self.plant.rate_max[ids] = lv[torch.randint(0, len(lv), (n,), device=dev, generator=self.gen)] * self._rand(n, *dr.steer_rate_scale)
        self.plant.tau_s[ids] = self._rand(n, *dr.tau_steer)
        self.plant.tau_w[ids] = self._rand(n, *dr.tau_wheel)
        self.plant.tau_b[ids] = self._rand(n, *dr.tau_body)
        self.plant.slip[ids] = dr.slip[0] + (dr.slip[1] - dr.slip[0]) * torch.rand(n, 4, device=dev, generator=self.gen)
        self.d_cmd[ids] = torch.randint(dr.cmd_delay_ticks[0], dr.cmd_delay_ticks[1] + 1, (n,), device=dev, generator=self.gen)
        self.d_obs[ids] = torch.randint(dr.obs_delay_ticks[0], dr.obs_delay_ticks[1] + 1, (n,), device=dev, generator=self.gen)
        self.map_err[ids] = torch.stack([self._rand(n, -dr.map_err_pos, dr.map_err_pos), self._rand(n, -dr.map_err_pos, dr.map_err_pos),
                                         self._rand(n, -1, 1) * math.radians(dr.map_err_yaw_deg)], -1)
        # 버퍼 · 추적 · 지표
        self.state_ring[ids] = self._state_now()[ids].unsqueeze(1)
        self.cmd_ring[ids] = 0.0
        self.s_true[ids] = 0.0
        self.s_est[ids] = 0.0
        self.last_action[ids] = 0.0
        self.last_cmd[ids] = 0.0
        self.prev_steer_goal[ids] = self.plant.q[ids]
        self.hold[ids] = 0.0
        self.soft[ids] = 0.0
        self.episode_length_buf[ids] = 0
        T = (2.0 * self.path.length[ids] / self.vmax[ids] + c.timeout_slack_s) / self.dt
        self.max_episode_length_t[ids] = T.ceil().long()
        for v in self.ep.values():
            v[ids] = 0.0

    def force_path(self, ids, pts, n, goal_yaw, start_yaw):
        """평가 · 그림용: 지정 경로 (env 좌표, 시작 = 원점) 로 에피소드 다시 시작 (DR · 조향 상태는 그대로)"""
        self.path.set(ids, pts, n, goal_yaw)
        self.plant.reset_motion(ids, torch.cat([torch.zeros(len(ids), 2, device=self.device), start_yaw.unsqueeze(-1)], -1))
        self.ctrl.stop(ids)
        self.state_ring[ids] = self._state_now()[ids].unsqueeze(1)
        self.cmd_ring[ids] = 0.0
        for t in (self.s_true, self.s_est, self.hold, self.soft, self.last_action, self.last_cmd):
            t[ids] = 0.0
        self.episode_length_buf[ids] = 0
        self.max_episode_length_t[ids] = ((2.0 * self.path.length[ids] / self.vmax[ids] + self.c.timeout_slack_s) / self.dt).ceil().long()
        for v in self.ep.values():
            v[ids] = 0.0
        self.obs = self._compute_obs(ids)
        return self.obs

    # ------------------------------------------------------------------ 상태 · 관측
    def _est_pose(self):
        """참 자세 + 위치 추정 오차 (dx, dy, dθ) = 추정 자세 (AMCL 오차 흉내). 로봇 자리에서 더함 → 위치 오차 = ±map_err_pos 그대로"""
        p, e = self.plant.pose, self.map_err
        return torch.stack([p[:, 0] + e[:, 0], p[:, 1] + e[:, 1], wrap(p[:, 2] + e[:, 2])], -1)

    def _state_now(self):
        return torch.cat([self._est_pose(), self.plant.q, self.plant.w, self.plant.odom], -1)

    def _delayed(self, ring, d):
        idx = (self.tick - 1 - d) % RING
        return ring[torch.arange(self.num_envs, device=self.device), idx]

    def _compute_obs(self, reset_ids=None):
        c, dr, g = self.c, self.dr, self.geom
        B, dev = self.num_envs, self.device
        st = self._delayed(self.state_ring, self.d_obs) if self.tick > 0 else self._state_now()
        pose, q, w, odom = st[:, :3], st[:, 3:7], st[:, 7:11], st[:, 11:14]
        q = q + torch.randn(B, 4, device=dev, generator=self.gen) * dr.noise_steer
        odom = odom + torch.randn(B, 3, device=dev, generator=self.gen) * torch.tensor(
            [dr.noise_lin_vel, dr.noise_lin_vel, dr.noise_ang_vel], device=dev)
        self.s_est, _, _, _ = self.path.project(pose[:, :2], self.s_est, c.proj_back, c.proj_fwd)
        lim3 = torch.stack([self.vmax, self.vmax, self.wmax], -1)
        # 경로 앞 점 · 목표
        sk = self.s_est.unsqueeze(-1) + c.path_obs_spacing * torch.arange(c.path_obs_points, device=dev)
        pk, _ = self.path.point_at(sk)
        pk = to_body(pk, pose).reshape(B, -1)
        gb = to_body(self.path.goal.unsqueeze(1), pose).squeeze(1)
        dyaw = wrap(self.path.goal_yaw - pose[:, 2])
        rem = (self.path.length - self.s_est)
        goal = torch.stack([gb[:, 0] / 2, gb[:, 1] / 2, dyaw.sin(), dyaw.cos(), rem / 3], -1)
        # 기준 추종기 (지연 · 오차 자세로 계산 = 실물 Nav2 controller 와 같은 입력)
        self.base_cmd = follow(self.path, pose, self.s_est, self.vmax, self.amax, self.wmax,
                               c.follower_lookahead, c.follower_yaw_gain)
        # 고유 상태 프레임
        rpos = (q - g.rmin) / (g.rmax - g.rmin) * 2 - 1
        frame = torch.cat([q.sin(), q.cos(), rpos, w / self.plant.max_wheel, odom / lim3, self.last_cmd / lim3], -1)
        self.hist = torch.cat([frame.unsqueeze(1), self.hist[:, :-1]], 1)
        if reset_ids is not None and len(reset_ids):
            self.hist[reset_ids] = frame[reset_ids].unsqueeze(1)
        lims = torch.stack([self.vmax / 0.26, self.amax / 1.0], -1)
        policy = torch.cat([pk, goal, self.base_cmd / lim3, self.hist.reshape(B, -1), lims], -1)
        # 비대칭 critic 용 (참 값 · DR 값)
        pe = self._est_pose()
        s_t, lat_t, _, _ = self.path.project(pe[:, :2], self.s_true, c.proj_back, c.proj_fwd)
        dy_t = wrap(self.path.goal_yaw - pe[:, 2])
        priv = torch.cat([self.plant.body_vel() / lim3, (lat_t / 0.1).unsqueeze(-1), dy_t.sin().unsqueeze(-1), dy_t.cos().unsqueeze(-1),
                          ((self.path.length - s_t) / 3).unsqueeze(-1),
                          torch.stack([self.plant.rate_max / 4.8, self.plant.tau_s / 0.15, self.plant.tau_w / 0.06, self.plant.tau_b / 0.08,
                                       self.d_cmd / 3.0, self.d_obs / 4.0, self.ctrl.blend_ratio, self.ctrl.gate_blocked.float()], -1),
                          self.map_err * torch.tensor([50.0, 50.0, 57.3], device=dev), self.plant.slip], -1)
        return TensorDict({'policy': policy, 'privileged': priv}, batch_size=[B], device=dev)

    def get_observations(self) -> TensorDict:
        return self.obs

    # ------------------------------------------------------------------ 명령
    def _command(self, a):
        c = self.c
        if c.action_mode == 'residual':
            rl = a[:, :2] * c.residual_lin
            n = rl.norm(dim=-1, keepdim=True)
            rl = rl * torch.where(n > c.residual_lin, c.residual_lin / n.clamp(min=1e-9), 1.0)
            cmd = self.base_cmd + torch.cat([rl, a[:, 2:3] * c.residual_ang], -1)
        else:
            cmd = a * torch.stack([self.vmax, self.vmax, self.wmax], -1)
        lin = cmd[:, :2]
        n = lin.norm(dim=-1)
        lin = lin * torch.where(n > self.vmax, self.vmax / n.clamp(min=1e-9), torch.ones_like(n)).unsqueeze(-1)
        wz = torch.maximum(torch.minimum(cmd[:, 2], self.wmax), -self.wmax)
        return torch.cat([lin, wz.unsqueeze(-1)], -1)

    # ------------------------------------------------------------------ 한 틱 물리 (컨트롤러 + 플랜트)
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
        c, dr, rw, dev = self.c, self.dr, self.rw, self.device
        B = self.num_envs
        a = actions.clamp(-1.0, 1.0)
        # 위치 추정 계단 변화 (AMCL 보정). 틱 전에 바꿔서 상태 링 · 보상이 같은 오차를 씀
        jump = self._rand(B, 0, 1) < dr.map_jump_prob * self.dt / dr.map_jump_every_s
        if jump.any():
            dj = torch.stack([self._rand(B, -1, 1) * dr.map_jump_pos, self._rand(B, -1, 1) * dr.map_jump_pos,
                              self._rand(B, -1, 1) * math.radians(dr.map_jump_yaw_deg)], -1)
            lim = torch.tensor([dr.map_err_pos, dr.map_err_pos, math.radians(dr.map_err_yaw_deg)], device=dev)
            err = torch.maximum(torch.minimum(self.map_err + dj, lim), -lim)       # AMCL 오차는 쌓이지 않음 → 범위 안으로
            self.map_err = torch.where(jump.unsqueeze(-1), err, self.map_err)
        cmd = self._command(a)
        ar = torch.arange(B, device=dev)
        flips = torch.zeros(B, device=dev)
        travel = torch.zeros(B, device=dev)
        gate = torch.zeros(B, device=dev)
        for _ in range(c.decimation):
            self.cmd_ring[:, self.tick % RING] = cmd
            cmd_d = self.cmd_ring[ar, (self.tick - self.d_cmd) % RING]
            f, tr, gt, st = self._physics(cmd_d)
            flips += f
            travel += tr
            gate += gt
            self.tick += 1
            self.state_ring[:, (self.tick - 1) % RING] = st
        self.last_cmd = cmd
        self.episode_length_buf += 1

        # 보상 (현재 추정 자세, 정지 판정 속도 = odom)
        pose = self._est_pose()
        s_new, _, lat, _ = self.path.project(pose[:, :2], self.s_true, c.proj_back, c.proj_fwd)   # lat = 경로까지 거리 (부호 없음)
        ds = s_new - self.s_true
        ds = torch.where(ds.abs() > self.vmax * self.dt + rw.progress_jump, torch.zeros_like(ds), ds)
        self.s_true = s_new
        rem = (self.path.length - s_new).clamp(min=0)
        dyaw = wrap(self.path.goal_yaw - pose[:, 2])
        dist = (self.path.goal - pose[:, :2]).norm(dim=-1)
        dist_true = (self.path.goal - self.plant.pose[:, :2]).norm(dim=-1)
        bv = self.plant.odom
        r_prog = rw.progress * ds
        r_lat = -rw.lateral * ((lat / 0.05) ** 2).clamp(max=9.0)
        r_head = -rw.heading * (1 - dyaw.cos()) * (1 - rem / rw.heading_zone).clamp(0, 1)
        r_act = -rw.action_rate * ((a - self.last_action) ** 2).sum(-1)
        rew = r_prog + r_lat + r_head + r_act - rw.time - rw.branch_flip * flips - rw.steer_travel * travel
        self.last_action = a

        at_goal = (dist < c.goal_pos_tol) & (dyaw.abs() < c.goal_yaw_tol) & (bv[:, :2].norm(dim=-1) < c.goal_lin_vel) & (bv[:, 2].abs() < c.goal_ang_vel)
        self.hold = torch.where(at_goal, self.hold + 1, torch.zeros_like(self.hold))
        success = self.hold * self.dt >= c.goal_hold_s - 1e-6
        self.soft = torch.where(lat.abs() > c.fail_lat_soft, self.soft + 1, torch.zeros_like(self.soft))
        failure = (lat.abs() > c.fail_lat_hard) | (self.soft * self.dt >= c.fail_lat_soft_s - 1e-6)
        failure &= ~success
        timeout = (self.episode_length_buf >= self.max_episode_length_t) & ~success & ~failure
        rew = rew + rw.success * success.float() - rw.failure * failure.float()
        dones = success | failure | timeout

        e = self.ep
        e['ret'] += rew
        e['flips'] += flips
        e['travel'] += travel
        e['gate'] += gate
        e['lat_max'] = torch.maximum(e['lat_max'], lat.abs())
        e['lat_sum'] += lat.abs()
        log = {}
        done_ids = torch.nonzero(dones).squeeze(-1)
        if len(done_ids):
            steps = self.episode_length_buf[done_ids].float()
            log = {'/ep/success': success[done_ids].float().mean(), '/ep/failure': failure[done_ids].float().mean(),
                   '/ep/timeout': timeout[done_ids].float().mean(), '/ep/return': e['ret'][done_ids].mean(),
                   '/ep/flips': e['flips'][done_ids].mean(), '/ep/steer_travel_rad': e['travel'][done_ids].mean(),
                   '/ep/gate_s': (e['gate'][done_ids] * 0.01).mean(), '/ep/lat_max_m': e['lat_max'][done_ids].mean(),
                   '/ep/lat_mean_m': (e['lat_sum'][done_ids] / steps).mean(), '/ep/final_dist_m': dist[done_ids].mean()}
            if success[done_ids].any():   # 성공한 것만 평균 (경로 길이 다르므로 m/s 도 같이)
                ok = done_ids[success[done_ids]]
                log['/ep/time_s'] = (self.episode_length_buf[ok].float() * self.dt).mean()
                log['/ep/speed_mps'] = (self.path.length[ok] / (self.episode_length_buf[ok].float() * self.dt)).mean()
            self._record_outcomes(success[done_ids].float())
            log['/curriculum/level'] = torch.tensor(float(self.level))
            log['/curriculum/success_window'] = self.outcomes[:min(self.outcome_count, len(self.outcomes))].mean()
            self.last_done = {'ids': done_ids, 'success': success[done_ids], 'failure': failure[done_ids], 'timeout': timeout[done_ids],
                              'time_s': steps * self.dt, 'flips': e['flips'][done_ids].clone(), 'travel': e['travel'][done_ids].clone(),
                              'lat_max': e['lat_max'][done_ids].clone(), 'lat_mean': e['lat_sum'][done_ids] / steps,
                              'gate_s': e['gate'][done_ids] * 0.01, 'final_dist': dist[done_ids], 'final_dist_true': dist_true[done_ids], 'final_dyaw': dyaw[done_ids].abs(),
                              'length': self.path.length[done_ids].clone()}
        else:
            self.last_done = None
        self._reset_idx(done_ids)
        self.obs = self._compute_obs(done_ids)
        self.extras = {'time_outs': timeout, 'log': log}
        return self.obs, rew, dones, self.extras

    def _record_outcomes(self, s):
        W = len(self.outcomes)
        s = s[-W:]
        k = len(s)
        self.outcomes[(self.outcome_ptr + torch.arange(k, device=self.device)) % W] = s
        self.outcome_ptr = (self.outcome_ptr + k) % W
        self.outcome_count += k
        c = self.c
        if c.curriculum and self.level < 2 and self.outcome_count >= W and float(self.outcomes.mean()) >= c.curriculum_success:
            self.level += 1
            self.outcome_count = 0
            self.outcomes.zero_()
            print(f'[커리큘럼] 단계 {self.level} 로', flush=True)
