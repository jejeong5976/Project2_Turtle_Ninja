"""dock5 환경 = local_planner env + 박스 측정기 흉내 + 앞쪽 촘촘한 라이다 (설명 cfg.py).

정책이 보는 목표 · 경로 = '노드 목표' (obs_goal · opath): 측정 전 = 지도 정차점, 측정 뒤 = 측정 목표, 바뀌면 지금 자리 → 새 목표 직선.
보상 · 성공 · 이탈 = 참 목표 · 참 경로 (self.path, local_planner env 그대로).
관측 = local_planner 관측 (경로 16 점 · 목표 5 · 라이다 36 · 이력) + [측정 표시 1] + 앞쪽 라이다 (2·fine_scan_half + 1)."""
from __future__ import annotations

import math

import torch
from tensordict import TensorDict

from local_planner.env import LocalPlannerEnv
from swerve_follow.paths import STEP as STEP_PTS, PathBatch, to_body, wrap


class DockEnv(LocalPlannerEnv):
    def __init__(self, cfg):
        c = cfg.env
        assert c.box_dynamic and c.lidar_model == 'real', 'dock5 = box_dynamic · 실물형 라이다'
        B, dev = c.num_envs, c.device
        z = lambda *s: torch.zeros(*s, device=dev)  # noqa: E731
        self.est_on = torch.zeros(B, dtype=torch.bool, device=dev)
        self.est_t, self.est_delay = z(B), z(B)
        self.est_bias = z(B, 3)
        self.est_goal = z(B, 3)                   # 측정 목표 (켜진 뒤)
        self.nom_goal = z(B, 3)                   # 지도 정차점 (측정 전 목표)
        self.obs_goal = z(B, 3)                   # 노드 목표 (정책이 봄)
        self.s_obs = z(B)
        self.opath = None
        self._lidar_fine = None
        super().__init__(cfg)

    # ------------------------------------------------------------------ 노드 목표 · 경로
    def _straight(self, ids, st, goal3):
        """opath[ids] = st (n,2) → goal3 (n,3) 직선 (STEP_PTS 점)"""
        M = self.opath.M
        g = goal3[:, :2]
        L = (g - st).norm(dim=-1, keepdim=True)
        frac = (torch.arange(M, device=self.device) * STEP_PTS / L.clamp(min=1e-6)).clamp(max=1.0)
        pts = st.unsqueeze(1) + (g - st).unsqueeze(1) * frac.unsqueeze(-1)
        n = (torch.ceil(L.squeeze(-1) / STEP_PTS).long() + 1).clamp(2, M)
        self.opath.set(ids, pts, n, goal3[:, 2])
        self.s_obs[ids] = 0.0

    def _true_goal(self, ids=None):
        g = torch.cat([self.path.goal, self.path.goal_yaw.unsqueeze(-1)], -1)
        return g if ids is None else g[ids]

    def _reset_idx(self, ids):
        super()._reset_idx(ids)
        if len(ids) == 0:
            return
        c, n, dev = self.c, len(ids), self.device
        if self.opath is None:
            self.opath = PathBatch(self.num_envs, self.path.M, dev)
        k = self.cur_k[ids]
        self.nom_goal[ids] = torch.cat([self.bank_pts[k, self.bank_n[k] - 1], self.bank_yaw[k].unsqueeze(-1)], -1)
        fast = self._rand(n, 0, 1) < c.est_fast_prob
        self.est_delay[ids] = torch.where(fast, self._rand(n, *c.est_delay_fast), self._rand(n, *c.est_delay_slow))
        self.est_t[ids] = 0.0
        self.est_on[ids] = False
        g = torch.randn(n, 3, device=dev, generator=self.gen)
        self.est_bias[ids] = g * torch.tensor([c.est_bias_xy, c.est_bias_xy, math.radians(c.est_bias_yaw_deg)], device=dev)
        self.est_goal[ids] = self.nom_goal[ids]
        self.obs_goal[ids] = self.nom_goal[ids]
        self._straight(ids, self._est_pose()[ids, :2], self.nom_goal[ids])

    def _measure(self, ids):
        c, n = self.c, len(ids)
        g = torch.randn(n, 3, device=self.device, generator=self.gen)
        noise = g * torch.tensor([c.est_noise_xy, c.est_noise_xy, math.radians(c.est_noise_yaw_deg)], device=self.device)
        m = self._true_goal(ids) + self.est_bias[ids] + noise
        m[:, 2] = wrap(m[:, 2])
        return m

    def _est_step(self, skip_ids):
        """정책 스텝마다 측정기 한 번 (방금 리셋한 env 는 뺌)"""
        c, B, dev = self.c, self.num_envs, self.device
        act = torch.ones(B, dtype=torch.bool, device=dev)
        if skip_ids is not None and len(skip_ids):
            act[skip_ids] = False
        self.est_t += self.dt * act.float()
        pose = self._est_pose()
        near = (self.obs_goal[:, :2] - pose[:, :2]).norm(dim=-1) < c.est_freeze      # 배포: 지금 목표 freeze 안이면 안 잼
        turn_on = act & ~self.est_on & (self.est_t >= self.est_delay)
        upd = (act & self.est_on & ~near & (self.episode_length_buf % c.est_every == 0)
               & (torch.rand(B, device=dev, generator=self.gen) >= c.est_drop))
        new = turn_on | upd
        if new.any():
            ids = torch.nonzero(new).squeeze(-1)
            m = self._measure(ids)
            e = self.est_goal[ids]
            mix = torch.cat([e[:, :2] + c.est_gain * (m[:, :2] - e[:, :2]), wrap(e[:, 2] + c.est_gain * wrap(m[:, 2] - e[:, 2])).unsqueeze(-1)], -1)
            self.est_goal[ids] = torch.where(turn_on[ids].unsqueeze(-1), m, mix)
        self.est_on |= turn_on
        tgt = torch.where(self.est_on.unsqueeze(-1), self.est_goal, self.nom_goal)
        moved = ((tgt[:, :2] - self.obs_goal[:, :2]).norm(dim=-1) > c.est_replan_xy) | \
                (wrap(tgt[:, 2] - self.obs_goal[:, 2]).abs() > math.radians(c.est_replan_yaw_deg))
        moved &= act
        if moved.any():                                # 배포 노드: 목표 바뀌면 지금 자리 → 새 목표 직선
            ids = torch.nonzero(moved).squeeze(-1)
            self.obs_goal[ids] = tgt[ids]
            self._straight(ids, pose[ids, :2], tgt[ids])

    # ------------------------------------------------------------------ 명령 (목표 근처 상한 = 노드 목표 기준, 배포와 같음)
    def _command(self, a):
        true_path = self.path
        self.path = self.opath
        try:
            return super()._command(a)
        finally:
            self.path = true_path

    # ------------------------------------------------------------------ 관측
    def _compute_obs(self, reset_ids=None):
        c, dr, g = self.c, self.dr, self.geom
        B, dev = self.num_envs, self.device
        if self._lidar_fine is None:
            self._lidar_fine = torch.compile(self.world.lidar_real_fine, dynamic=False) if c.compile else self.world.lidar_real_fine
        if self.tick > 0:
            self._est_step(reset_ids)
        st = self._delayed(self.state_ring, self.d_obs) if self.tick > 0 else self._state_now()
        pose, q, w, odom = st[:, :3], st[:, 3:7], st[:, 7:11], st[:, 11:14]
        q = q + torch.randn(B, 4, device=dev, generator=self.gen) * dr.noise_steer
        odom = odom + torch.randn(B, 3, device=dev, generator=self.gen) * torch.tensor(
            [dr.noise_lin_vel, dr.noise_lin_vel, dr.noise_ang_vel], device=dev)
        op = self.opath
        self.s_obs, _, _, _ = op.project(pose[:, :2], self.s_obs, c.proj_back, c.proj_fwd)
        sk = self.s_obs.unsqueeze(-1) + c.path_obs_spacing * torch.arange(c.path_obs_points, device=dev)
        pk, _ = op.point_at(sk)
        pk = to_body(pk, pose).reshape(B, -1)
        gb = to_body(self.obs_goal[:, None, :2], pose).squeeze(1)
        dyaw = wrap(self.obs_goal[:, 2] - pose[:, 2])
        rem = op.length - self.s_obs
        goal = torch.stack([gb[:, 0] / 2, gb[:, 1] / 2, dyaw.sin(), dyaw.cos(), (rem / 3).clamp(max=2.0)], -1)
        nb = self.occ.beam.numel()
        noise = torch.randn(B, nb, device=dev, generator=self.gen) * dr.lidar_noise
        drop = torch.rand(B, nb, device=dev, generator=self.gen) < dr.lidar_dropout
        scan, fine = self._lidar_fine(self.plant.pose, self.plant.q, self.occ, c.lidar_rays, self.lidar_trace_max, noise, drop,
                                      c.fine_scan_half, math.radians(c.fine_scan_step_deg), c.fine_scan_clip)
        self.scan, self.fine = scan, fine
        lid = scan.clamp(0, c.lidar_obs_clip) / c.lidar_obs_clip
        rpos = (q - g.rmin) / (g.rmax - g.rmin) * 2 - 1
        frame = torch.cat([q.sin(), q.cos(), rpos, w / self.plant.max_wheel, odom / self.lim3, self.last_cmd / self.lim3], -1)
        self.hist = torch.cat([frame.unsqueeze(1), self.hist[:, :-1]], 1)
        if reset_ids is not None and len(reset_ids):
            self.hist[reset_ids] = frame[reset_ids].unsqueeze(1)
        extra = [self.est_on.float().unsqueeze(-1)] if c.est_flag else []
        policy = torch.cat([pk, goal, lid, self.hist.reshape(B, -1)] + extra + [fine / c.fine_scan_clip], -1)
        # 비대칭 critic (참 값 · DR 값 · 측정 오차)
        pt = self.plant.pose
        s_t, lat_t, _, _ = self.path.project(pt[:, :2], self.s_true, c.proj_back, c.proj_fwd)
        dist_t, dy_t = self._goal_err(pt)
        clr = self.world.clearance(pt, self.fp_clear)
        rel = self.world.people_c - pt[:, None, :2]
        dpp = torch.where(self.world.people_on, rel.norm(dim=-1) - self.world.people_r, torch.full_like(self.world.people_r, 5.0))
        tg = self._true_goal()
        gerr = torch.cat([self.obs_goal[:, :2] - tg[:, :2], wrap(self.obs_goal[:, 2] - tg[:, 2]).unsqueeze(-1)], -1)
        priv = torch.cat([self.plant.body_vel() / self.lim3, (lat_t / 0.1).unsqueeze(-1), dy_t.sin().unsqueeze(-1), dy_t.cos().unsqueeze(-1),
                          (dist_t / 3).unsqueeze(-1), (clr / 0.1).clamp(-1, 3).unsqueeze(-1), (dpp.min(-1).values / 1.0).clamp(max=3).unsqueeze(-1),
                          torch.stack([self.plant.rate_max / 4.8, self.plant.tau_s / 0.15, self.plant.tau_w / 0.06, self.plant.tau_b / 0.08,
                                       self.d_cmd / 3.0, self.d_obs / 4.0, self.ctrl.blend_ratio, self.ctrl.gate_blocked.float()], -1),
                          self._pose_err() * torch.tensor([60.0, 60.0, 57.3], device=dev), self.plant.slip,
                          torch.full((B, 1), float(self.level), device=dev),
                          (gerr * torch.tensor([10.0, 10.0, 10.0], device=dev)).clamp(-3, 3), self.est_on.float().unsqueeze(-1)], -1)
        return TensorDict({'policy': policy, 'privileged': priv}, batch_size=[B], device=dev)

    def step(self, actions):
        obs, rew, dones, extras = super().step(actions)
        tg = self._true_goal()
        extras['log']['/est/on'] = self.est_on.float().mean()
        extras['log']['/est/goal_err_cm'] = ((self.obs_goal[:, :2] - tg[:, :2]).norm(dim=-1)[self.est_on].mean() * 100
                                             if self.est_on.any() else torch.zeros((), device=self.device))
        extras['log']['/est/box_shift_cm'] = (self.nom_goal[:, :2] - tg[:, :2]).norm(dim=-1).mean() * 100
        return obs, rew, dones, extras
