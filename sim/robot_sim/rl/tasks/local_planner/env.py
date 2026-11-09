"""지역 계획기 학습 환경 (rsl_rl VecEnv, torch 만, Isaac 안 띄움). MPPI 자리를 대신할 정책.

한 정책 스텝 (20 Hz) = 컨트롤러 · 플랜트 5 틱 (100 Hz):
  행동 a ∈ [−1, 1]³ → 몸 속도 (vx, vy) · wz (실물 한계로 자름) → 명령 큐 (지연) → 스워브 컨트롤러 (팀 swerve.yaml) → 간이 플랜트.
관측 (정책): 전역 경로 앞 16 점 · 목표 (몸 좌표) · 라이다 36 방향 · 바퀴 상태 이력 4 프레임. 경로 · 목표는 '추정 자세'(지연 + 위치 오차) 기준.
보상 · 성공 = 추정 자세 (실물 goal checker 와 같음), 충돌 · 여유 · 라이다 = 참 자세 (실제 몸).
세계 = 점유 격자 지도 (env 마다 하나) + 사람 원기둥 (world.py). 경로 = 지도 경로 은행 (NavFn 흉내) + dice 정차점 구간.
플랜트 · 경로 묶음은 swerve_follow 것 그대로 씀.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import torch
from tensordict import TensorDict

from rsl_rl.env import VecEnv
from sim_core.swerve_torch import SwerveGeometry, SwerveTorch
from swerve_follow.paths import STEP as STEP_PTS, PathBatch, to_body, wrap
from swerve_follow.plant import SwervePlant

from .cfg import Cfg
from .world import World

RING = 8                       # 지연 버퍼 길이 (틱)


class LocalPlannerEnv(VecEnv):
    def __init__(self, cfg: Cfg):
        self.full_cfg = cfg
        c, dr = cfg.env, cfg.dr
        self.c, self.dr, self.rw = c, dr, cfg.reward
        self.cfg = dataclasses.asdict(cfg)
        self.device = dev = c.device
        self.num_envs = B = c.num_envs
        self.num_actions = 3
        self.gen = torch.Generator(device=dev).manual_seed(c.seed)
        self.dt = c.ctrl_dt * c.decimation

        self.geom = SwerveGeometry(dev, torch.float32, path=c.swerve_yaml)
        self.ctrl = SwerveTorch(B, dev, geom=self.geom)
        self.ctrl.max_linear[:], self.ctrl.max_angular[:] = c.vmax, c.wmax
        self.ctrl.max_linear_accel[:], self.ctrl.max_angular_accel[:] = c.amax, c.awmax
        self.plant = SwervePlant(B, self.geom, c.ctrl_dt)
        self._load_paths()
        self.world = World(self.map_names, c.maps_dir, B, dev, c.max_people)
        if c.box_dynamic:
            e = np.load(c.dice_legs)
            assert 'box_centers' in e, 'box_dynamic 은 box_id · box_centers 가 있는 구간 파일 필요 (local_dock/make_dock_legs.py)'
            self.world.enable_boxes(self.map_names.index('dice'), e['box_centers'], float(e['box_half']))
        if c.map_boxes:                                 # 지도 ↔ 실제 어긋남 (nf7): 박스를 해석식으로, 에피소드마다 옮김
            assert not c.box_dynamic, 'box_dynamic (접근 박스 하나) 과 map_boxes (여러 박스) 는 같이 못 씀'
            import json
            mb = json.loads(open(c.map_boxes).read())
            self.world.enable_multi_boxes({self.map_names.index(k): v for k, v in mb.items() if k in self.map_names},
                                          c.mbox_half, c.mbox_erase_margin)
            print(f'[local_planner] 박스 해석식: ' + ', '.join(f'{k} {len(v)} 개' for k, v in mb.items() if k in self.map_names), flush=True)
        self.fp_collide = World.footprint_points(c.collide_half, c.footprint_per_side, dev)
        self.fp_clear = World.footprint_points(c.footprint_half, c.footprint_per_side, dev)
        self._check_goals()
        self.path = PathBatch(B, self.bank_pts.shape[1], dev)
        self.lim3 = torch.tensor([c.vmax, c.vmax, c.wmax], device=dev)

        z = lambda *s: torch.zeros(*s, device=dev)  # noqa: E731
        self.s_true, self.s_est = z(B), z(B)
        self.map_err = z(B, 3)
        self.yaw_drift, self.drift_rate = z(B), z(B)      # nf7 yaw 흐름 [rad] · 속도 [rad/s]
        self.big_err = z(B, 3)                            # nf7 큰 튐 (줄어드는 중)
        self.d_cmd = torch.zeros(B, dtype=torch.long, device=dev)
        self.d_obs = torch.zeros(B, dtype=torch.long, device=dev)
        self.state_ring = z(B, RING, 14)          # 추정 자세 3 · 조향 4 · 바퀴 4 · odom 3
        self.cmd_ring = z(B, RING, 3)
        self.tick = 0
        self.hist = z(B, c.history, 22)
        self.last_action = z(B, 3)
        self.last_cmd = z(B, 3)
        self.prev_steer_goal = z(B, 4)
        self.phi = z(B)
        self.hold = z(B)
        self.prev_yaw = z(B)                      # 헛회전 계산용 (참 자세 yaw · 목표 방향 오차)
        self.prev_dyaw_abs = z(B)
        self.is_dice = torch.zeros(B, dtype=torch.bool, device=dev)
        self.episode_length_buf = torch.zeros(B, dtype=torch.long, device=dev)
        self.max_episode_length_t = torch.ones(B, dtype=torch.long, device=dev)
        self.max_episode_length = 10 ** 6
        self.first = torch.ones(B, dtype=torch.bool, device=dev)
        self.ep = {k: z(B) for k in ('ret', 'travel', 'clear_min', 'dev_max', 'stall', 'rot', 'waste', 'rev')}
        self.level = c.curriculum_start if c.curriculum else 2
        self.outcomes = torch.zeros(c.curriculum_window, device=dev)
        self.outcome_ptr = 0
        self.outcome_count = 0
        self.forced = None                         # 평가: 다음 리셋에 쓸 경로 번호 (B,) (−1 = 무작위)
        self.cur_k = torch.zeros(B, dtype=torch.long, device=dev)   # env 마다 지금 경로 번호 (평가 · 분석용)
        self.start_xy = torch.zeros(B, 2, device=dev)                # 에피소드 출발 자리 (정지 구역 출발 예외)
        self.extras = {}
        self.last_done = None
        self._physics = torch.compile(self._physics_impl, dynamic=False) if c.compile else self._physics_impl
        self._lidar = torch.compile(self.world.lidar, dynamic=False) if c.compile else self.world.lidar
        if c.lidar_model == 'real':
            from local_planner.world import LidarOcclusion
            self.occ = LidarOcclusion(c.lidar_occlusion, dev, c.lidar_beam_stride)
            import yaml
            steer = yaml.safe_load(open(c.swerve_yaml))['joints']['steer']
            assert self.occ.steer_names == list(steer), f'가림표 조향 순서 {self.occ.steer_names} ≠ {steer}'
            self._lidar_real = torch.compile(self.world.lidar_real, dynamic=False) if c.compile else self.world.lidar_real
            self.lidar_trace_max = min(c.lidar_max, c.lidar_obs_clip + float(self.occ.origin.norm(dim=-1).max()) + 0.03)
        else:
            assert c.lidar_model == 'ideal', c.lidar_model
        self._reset_idx(torch.arange(B, device=dev))
        self.obs = self._compute_obs(torch.arange(B, device=dev))

    # ------------------------------------------------------------------ 경로
    def _load_paths(self):
        c, dev = self.c, self.device
        d = np.load(c.path_bank)
        names = [str(x) for x in d['names']]
        m = (d['split'] == c.split) & (d['src'] >= 0)                    # 지도 경로만
        e = np.load(c.dice_legs)
        assert [str(x) for x in e['names']] == names, '경로 은행 · dice 구간 지도 이름 순서가 다름'
        pts = np.concatenate([d['pts'][m] + d['start'][m][:, None], e['pts'] + e['start'][:, None]], 0)   # 지도 좌표
        self.map_names = names
        self.bank_pts = torch.tensor(pts, dtype=torch.float32, device=dev)
        self.bank_n = torch.tensor(np.concatenate([d['n'][m], e['n']]), dtype=torch.long, device=dev)
        self.bank_yaw = torch.tensor(np.concatenate([d['goal_yaw'][m], e['goal_yaw']]), dtype=torch.float32, device=dev)
        self.bank_start_yaw = torch.tensor(np.concatenate([np.full(m.sum(), np.nan), e['start_yaw']]), dtype=torch.float32, device=dev)
        self.bank_map = torch.tensor(np.concatenate([d['src'][m], e['src']]), dtype=torch.long, device=dev)
        self.bank_dice = torch.zeros(len(pts), dtype=torch.bool, device=dev)
        self.bank_dice[m.sum():] = True
        self.dice_legs = [str(x) for x in e['legs']]
        self.dice_first = int(m.sum())
        bid = e['box_id'] if 'box_id' in e else np.full(len(e['n']), -1)        # 접근 구간의 박스 번호 (−1 = 없음)
        self.bank_box = torch.tensor(np.concatenate([np.full(m.sum(), -1), bid]), dtype=torch.long, device=dev)
        self.bank_extra = torch.zeros(len(pts), dtype=torch.bool, device=dev)
        if c.extra_legs:                                 # 추가 구간 (nf7): 다른 지도 (arena5 등) 정차점 구간, 지도 이름은 파일 안 names
            x = np.load(c.extra_legs)
            xnames = [str(v) for v in x['names']]
            for nm in xnames:
                if nm not in self.map_names:
                    self.map_names.append(nm)
            remap = np.array([self.map_names.index(nm) for nm in xnames])
            xp = x['pts'] + x['start'][:, None]
            assert xp.shape[1] == pts.shape[1], f'추가 구간 점 수 {xp.shape[1]} ≠ {pts.shape[1]}'
            nx, tt = len(xp), lambda a, dt: torch.tensor(a, dtype=dt, device=dev)   # noqa: E731
            self.bank_pts = torch.cat([self.bank_pts, tt(xp, torch.float32)])
            self.bank_n = torch.cat([self.bank_n, tt(x['n'], torch.long)])
            self.bank_yaw = torch.cat([self.bank_yaw, tt(x['goal_yaw'], torch.float32)])
            self.bank_start_yaw = torch.cat([self.bank_start_yaw, tt(x['start_yaw'], torch.float32)])
            self.bank_map = torch.cat([self.bank_map, tt(remap[x['src']], torch.long)])
            self.bank_dice = torch.cat([self.bank_dice, torch.ones(nx, dtype=torch.bool, device=dev)])     # 정차점 구간 (출발 yaw 있음)
            self.bank_box = torch.cat([self.bank_box, torch.full((nx,), -1, dtype=torch.long, device=dev)])
            self.bank_extra = torch.cat([self.bank_extra, torch.ones(nx, dtype=torch.bool, device=dev)])
            self.dice_legs += [f'{xnames[s_]}:{lg}' for s_, lg in zip(x['src'], x['legs'])]

    def _check_goals(self):
        """목표 자세에서 외곽이 지도와 겹치는 지도 경로는 끝 접선 yaw 로 바꾸고, 그래도 겹치면 안 씀"""
        N, dev = len(self.bank_n), self.device
        ar = torch.arange(N, device=dev)
        goal = self.bank_pts[ar, self.bank_n - 1]
        prev = self.bank_pts[ar, (self.bank_n - 5).clamp(min=0)]
        tan = torch.atan2(goal[:, 1] - prev[:, 1], goal[:, 0] - prev[:, 0])
        clr = lambda yaw: self.world.clearance(torch.cat([goal, yaw.unsqueeze(-1)], -1), self.fp_collide, self.bank_map, people=False)  # noqa: E731
        bad = (clr(self.bank_yaw) < 0.005) & ~self.bank_dice
        self.bank_yaw = torch.where(bad, tan, self.bank_yaw)
        self.bank_ok = (clr(self.bank_yaw) >= 0.005) | self.bank_dice
        dice_clr = clr(self.bank_yaw)[self.bank_dice]
        self.random_ids = torch.nonzero(self.bank_ok & ~self.bank_dice).squeeze(-1)
        self.dice_ids = torch.nonzero(self.bank_dice & ~self.bank_extra).squeeze(-1)
        self.extra_ids = torch.nonzero(self.bank_extra).squeeze(-1)
        print(f'[local_planner] 지도 경로 {len(self.random_ids)} 개 (목표 yaw 바꿈 {int(bad.sum())} · 뺌 {int((~self.bank_ok).sum())}), '
              f'dice 구간 {len(self.dice_ids)} 개 · 추가 구간 {len(self.extra_ids)} 개 (목표 여유 최소 {float(dice_clr.min()) * 100:.1f} cm, 래스터 기준)', flush=True)

    def _rand(self, n, lo, hi):
        return lo + (hi - lo) * torch.rand(n, device=self.device, generator=self.gen)

    # ------------------------------------------------------------------ 리셋
    def _reset_idx(self, ids):
        if len(ids) == 0:
            return
        c, dr, n, dev = self.c, self.dr, len(ids), self.device
        # 경로 고르기: dice 구간 (dice_prob) · 지도 경로
        u = self._rand(n, 0, 1)
        use_extra = (u < c.extra_prob) if len(self.extra_ids) else torch.zeros(n, dtype=torch.bool, device=dev)
        use_dice = ~use_extra & (u < c.extra_prob + c.dice_prob)
        kd = self.dice_ids[torch.randint(0, len(self.dice_ids), (n,), device=dev, generator=self.gen)]
        kr = self.random_ids[torch.randint(0, len(self.random_ids), (n,), device=dev, generator=self.gen)]
        k = torch.where(use_dice, kd, kr)
        if len(self.extra_ids):
            kx = self.extra_ids[torch.randint(0, len(self.extra_ids), (n,), device=dev, generator=self.gen)]
            k = torch.where(use_extra, kx, k)
        if self.forced is not None:
            f = self.forced[ids]
            k = torch.where(f >= 0, f, k)
        self.is_dice[ids] = self.bank_dice[k]
        self.world.map_id[ids] = self.bank_map[k]
        self.path.set(ids, self.bank_pts[k], self.bank_n[k], self.bank_yaw[k])
        start = self.bank_pts[k, 0]
        # 시작 yaw: dice = 출발 정차점 yaw ± 잡음, 지도 경로 = 외곽이 안 겹치는 무작위 yaw (8 후보 중 처음, 없으면 경로 접선)
        cand = self._rand(n * 8, -math.pi, math.pi).view(n, 8)
        pose8 = torch.cat([start.unsqueeze(1).expand(-1, 8, -1), cand.unsqueeze(-1)], -1).reshape(-1, 3)
        clr8 = self.world.clearance(pose8, self.fp_collide, self.bank_map[k].repeat_interleave(8), people=False).view(n, 8)
        okc = clr8 > 0.01
        first = torch.where(okc.any(1), okc.float().argmax(1), torch.zeros(n, dtype=torch.long, device=dev))
        p1 = self.bank_pts[k, 2]
        tan = torch.atan2(p1[:, 1] - start[:, 1], p1[:, 0] - start[:, 0])
        yaw_r = torch.where(okc.any(1), cand.gather(1, first.unsqueeze(1)).squeeze(1), tan)
        yaw_d = self.bank_start_yaw[k] + self._rand(n, -1, 1) * math.radians(c.dice_start_yaw_noise_deg)
        yaw = wrap(torch.where(self.is_dice[ids], yaw_d, yaw_r))
        pos = start + torch.randn(n, 2, device=dev, generator=self.gen) * c.init_pos_std
        # 시작 자세가 벽 · 박스와 겹치면 잡음 없이 (dice 정차점은 박스 면 2 cm 라 yaw ±5° · 위치 1 cm 잡음이면 겹침 → 0 s 충돌, 2026-11-04 A/B 에서 벽 충돌의 68 %)
        pose0 = torch.cat([pos, yaw.unsqueeze(-1)], -1)
        bad = self.world.clearance(pose0, self.fp_collide, self.bank_map[k], people=False) < 0.005
        if bad.any():
            exact = torch.cat([start, torch.where(self.is_dice[ids], wrap(self.bank_start_yaw[k]), yaw).unsqueeze(-1)], -1)
            pose0 = torch.where(bad.unsqueeze(-1), exact, pose0)
        self.cur_k[ids] = k
        self.start_xy[ids] = pose0[:, :2]
        if c.box_dynamic:
            self._shift_boxes(ids, k, pose0)
        if c.map_boxes:
            self._shift_multi_boxes(ids, pose0)
        self.plant.reset_motion(ids, pose0)
        self._place_people(ids)
        # 조향 상태
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
        self.drift_rate[ids] = self._rand(n, -1, 1) * math.radians(dr.yaw_drift_dps)
        self.yaw_drift[ids] = 0.0
        self.big_err[ids] = 0.0
        if c.start_speed_prob > 0:
            self._moving_start(ids)
        # 버퍼 · 추적
        self.state_ring[ids] = self._state_now()[ids].unsqueeze(1)
        self.cmd_ring[ids] = 0.0
        self.s_true[ids] = 0.0
        self.s_est[ids] = 0.0
        self.last_action[ids] = 0.0
        self.last_cmd[ids] = 0.0
        self.prev_steer_goal[ids] = self.plant.q[ids]
        self.hold[ids] = 0.0
        self.phi[ids] = self._potential()[ids]
        yaw0 = self.plant.pose[ids, 2]
        self.prev_yaw[ids] = yaw0
        self.prev_dyaw_abs[ids] = wrap(self.path.goal_yaw[ids] - yaw0).abs()
        self.episode_length_buf[ids] = 0
        T = (2.0 * self.path.length[ids] / c.vmax + c.timeout_slack_s) / self.dt
        self.max_episode_length_t[ids] = T.ceil().long()
        for v in self.ep.values():
            v[ids] = 0.0
        self.ep['clear_min'][ids] = 1.0

    def _moving_start(self, ids):
        """start_speed_prob 확률로 경로 첫 방향 병진 v 로 이미 가는 중인 상태로 (회전 0). 조향 = 컨트롤러 역기구학 각, 바퀴 = 그 속도"""
        c, dev, n = self.c, self.device, len(ids)
        sel = (self._rand(n, 0, 1) < c.start_speed_prob) & (self.path.length[ids] >= c.start_speed_min_len)
        mid = ids[sel]
        if len(mid) == 0:
            return
        m = len(mid)
        v = self._rand(m, *c.start_speed)
        pts = self.path.pts[mid]                                       # 박스 밀림 뒤 경로
        d = pts[:, 2] - pts[:, 0]                                       # 첫 0.1 m 방향
        th = torch.atan2(d[:, 1], d[:, 0])
        yaw = self.plant.pose[mid, 2]
        vw = torch.stack([v * th.cos(), v * th.sin(), torch.zeros_like(v)], -1)
        cb, sb = yaw.cos(), yaw.sin()
        vb = torch.stack([cb * vw[:, 0] + sb * vw[:, 1], -sb * vw[:, 0] + cb * vw[:, 1], torch.zeros_like(v)], -1)
        ctl = self.ctrl
        ang, wheel, ok = ctl.inverse(vb, self.plant.q[mid], False, ctl.prev[mid], ctl.have_prev[mid],
                                     torch.zeros(m, dtype=torch.bool, device=dev))
        mid, ang, wheel, vw, vb = mid[ok], ang[ok], wheel[ok], vw[ok], vb[ok]
        self.plant.q[mid], self.plant.w[mid], self.plant.qd[mid] = ang, wheel, 0.0
        self.plant.vel_w[mid], self.plant.odom[mid] = vw, vb
        ctl.out[mid], ctl.translating[mid] = vb, True
        ctl.prev[mid], ctl.have_prev[mid] = ang, True

    def _shift_boxes(self, ids, k, pose0):
        """접근 구간의 박스를 밀고 목표를 같이 옮김. 박스가 벽에 닿거나 출발 외곽과 겹치면 안 밂"""
        c, w, dev, n = self.c, self.world, self.device, len(ids)
        bb = self.bank_box[k]
        shift = (bb >= 0) & (self._rand(n, 0, 1) < c.box_shift_prob)
        dxy = torch.stack([self._rand(n, -1, 1), self._rand(n, -1, 1)], -1) * c.box_shift_xy
        dth = self._rand(n, -1, 1) * math.radians(c.box_shift_yaw_deg)
        c0 = w.box_c[bb.clamp(min=0)]
        bpose = torch.cat([c0 + dxy, dth.unsqueeze(-1)], -1)
        # 옮긴 박스 둘레가 다른 장애물 (벽 · 기둥) 과 겹치면 안 밂
        t = torch.linspace(-w.box_half, w.box_half, 9, device=dev)
        h = torch.full_like(t, w.box_half)
        per = torch.cat([torch.stack([t, -h], -1), torch.stack([h, t], -1), torch.stack([-t, h], -1), torch.stack([-h, -t], -1)], 0)
        cs, sn = dth.cos().unsqueeze(-1), dth.sin().unsqueeze(-1)
        px = bpose[:, None, 0] + cs * per[:, 0] - sn * per[:, 1]
        py = bpose[:, None, 1] + sn * per[:, 0] + cs * per[:, 1]
        wall = w._lookup(w.dfield_minus, bb.clamp(min=0), torch.stack([px, py], -1), w.origin[self.bank_map[k]]).min(-1).values
        shift &= wall >= 0.005
        # 접근 구간의 박스는 밀지 않아도 해석식 (205 mm 사각형) 으로: 지도 래스터는 박스가 E · N 쪽으로 약 1.7 cm 커서
        #   정차 0.2575 (차체 앞 끝 ↔ 박스 면 1 cm) 에서 래스터면 닿은 것으로 나옴 (2026-11-06)
        nominal = torch.cat([c0, torch.zeros_like(dth).unsqueeze(-1)], -1)
        w.box_k[ids] = bb                                              # 접근 구간 아니면 −1 (bank_box)
        w.box_pose[ids] = torch.where(shift.unsqueeze(-1), bpose, nominal)
        # 출발 차체가 옮긴 박스와 겹치면 안 밂 (원래 자리로)
        clr = w.clearance(pose0, self.fp_collide, people=False, ids=ids)
        back = shift & (clr < 0.005)
        shift &= ~back
        w.box_pose[ids] = torch.where(shift.unsqueeze(-1), bpose, nominal)
        if not shift.any():
            return
        # 목표 = 박스에 붙여 옮김, 경로 = 출발 → 옮긴 목표 직선 (0.05 m 점)
        si = ids[shift]
        g0, gy0 = self.path.goal[si], self.path.goal_yaw[si]
        rel = g0 - c0[shift]
        cs, sn = cs[shift], sn[shift]
        g = c0[shift] + dxy[shift] + torch.cat([cs * rel[:, :1] - sn * rel[:, 1:], sn * rel[:, :1] + cs * rel[:, 1:]], -1)
        st = self.bank_pts[k[shift], 0]
        L = (g - st).norm(dim=-1, keepdim=True)
        M = self.path.M
        frac = (torch.arange(M, device=dev) * STEP_PTS / L.clamp(min=1e-6)).clamp(max=1.0)        # (n,M)
        pts = st.unsqueeze(1) + (g - st).unsqueeze(1) * frac.unsqueeze(-1)
        npts = (torch.ceil(L.squeeze(-1) / STEP_PTS).long() + 1).clamp(2, M)
        self.path.set(si, pts, npts, wrap(gy0 + dth[shift]))

    def _shift_multi_boxes(self, ids, pose0):
        """지도 박스 (해석식) 를 에피소드마다 옮김 (nf7). 출발 · 목표 mbox_exempt 안 박스는 그대로.
        옮긴 박스가 벽 (박스 지운 래스터) 과 겹치거나 출발 · 목표 외곽이 박스와 겹치면 그 env 는 하나도 안 옮김"""
        c, w, dev, n = self.c, self.world, self.device, len(ids)
        mid = w.map_id[ids]
        nom, valid = w.mb_nom[mid], w.mb_valid[mid]                     # (n,K,3) · (n,K)
        K = nom.shape[1]
        w.mb_pose[ids], w.mb_on[ids] = nom, valid
        start, goal = self.path.pts[ids, 0], self.path.goal[ids]
        far = ((nom[..., :2] - start.unsqueeze(1)).norm(dim=-1) > c.mbox_exempt) & ((nom[..., :2] - goal.unsqueeze(1)).norm(dim=-1) > c.mbox_exempt)
        sh = valid & far & (self._rand(n, 0, 1) < c.mbox_shift_prob).unsqueeze(-1)
        if not sh.any():
            return
        dxy = (torch.rand(n, K, 2, device=dev, generator=self.gen) * 2 - 1) * c.mbox_shift_xy
        dth = (torch.rand(n, K, device=dev, generator=self.gen) * 2 - 1) * math.radians(c.mbox_shift_yaw_deg)
        moved = nom + torch.cat([dxy, dth.unsqueeze(-1)], -1) * sh.unsqueeze(-1).float()
        # 옮긴 박스 둘레가 벽 (박스 지운 래스터) 과 겹치면 그 박스는 안 옮김
        h = w.mb_half
        t = torch.linspace(-h, h, 9, device=dev)
        hh = torch.full_like(t, h)
        per = torch.cat([torch.stack([t, -hh], -1), torch.stack([hh, t], -1), torch.stack([-t, hh], -1), torch.stack([-hh, -t], -1)], 0)   # (36,2)
        cs, sn = moved[..., 2].cos().unsqueeze(-1), moved[..., 2].sin().unsqueeze(-1)
        px = moved[..., 0:1] + cs * per[:, 0] - sn * per[:, 1]
        py = moved[..., 1:2] + sn * per[:, 0] + cs * per[:, 1]
        wall = w._lookup(w.dfield_nobox, mid, torch.stack([px, py], -1).view(n, -1, 2), w.origin[mid]).view(n, K, -1).min(-1).values
        sh &= wall >= 0.005
        w.mb_pose[ids] = torch.where(sh.unsqueeze(-1), moved, nom)
        # 출발 · 목표 외곽이 (옮긴) 박스와 겹치면 그 env 는 원래 자리로
        gpose = torch.cat([goal, self.path.goal_yaw[ids].unsqueeze(-1)], -1)
        clr_s = w.clearance(pose0, self.fp_collide, people=False, ids=ids)
        clr_g = w.clearance(gpose, self.fp_collide, people=False, ids=ids)
        clr_s0 = w.clearance(pose0, self.fp_collide, map_id=mid, people=False)       # 래스터 기준 (원래 겹치던 자리는 탓하지 않음)
        clr_g0 = w.clearance(gpose, self.fp_collide, map_id=mid, people=False)
        back = ((clr_s < 0.005) & (clr_s < clr_s0 - 1e-4)) | ((clr_g < 0.005) & (clr_g < clr_g0 - 1e-4))
        if back.any():
            w.mb_pose[ids[back]] = nom[back]

    def _place_people(self, ids):
        """사람 원기둥: 경로 옆 (중심 = 경로점 + 법선 × offset), 벽 · 시작 · 목표에서 떨어진 자리만"""
        c, n, dev, w = self.c, len(ids), self.device, self.world
        P = c.max_people
        w.people_on[ids] = False
        on = self._rand(n * P, 0, 1).view(n, P) < c.people_prob[self.level]
        L = self.path.length[ids]
        s = (0.4 + self._rand(n * P, 0, 1).view(n, P) * (L.unsqueeze(-1) - 1.0)).clamp(min=0)
        pk = torch.zeros(self.num_envs, P, device=dev)
        pk[ids] = s
        pt, tan = self.path.point_at(pk)
        pt, tan = pt[ids], tan[ids]
        nrm = torch.stack([-tan[..., 1], tan[..., 0]], -1)
        side = torch.where(self._rand(n * P, 0, 1).view(n, P) < 0.5, -1.0, 1.0)
        off = self._rand(n * P, *c.people_offset).view(n, P)
        r = self._rand(n * P, *c.people_r).view(n, P)
        cen = pt + nrm * (side * off).unsqueeze(-1)
        mid = self.world.map_id[ids].unsqueeze(-1).expand(-1, P).reshape(-1)
        dmap = w.map_dist(cen.reshape(-1, 1, 2), mid).view(n, P)
        # 지나갈 자리: 사람 반대편 (사람 가장자리 + 차체 반 폭 + 3 cm) 에 로봇 중심이 벽에서 0.19 m 떨어져 설 수 있어야 함
        q = cen - nrm * (side * (r + 0.19)).unsqueeze(-1)
        dq = w.map_dist(q.reshape(-1, 1, 2), mid).view(n, P)
        start = self.path.pts[ids, 0].unsqueeze(1)
        goal = self.path.goal[ids].unsqueeze(1)
        ok = on & (L.unsqueeze(-1) > 1.2) & (dmap > r) & (dq > 0.19) & ((cen - start).norm(dim=-1) > r + 0.45) & ((cen - goal).norm(dim=-1) > r + 0.45)
        w.people_c[ids], w.people_r[ids], w.people_on[ids] = cen, r, ok

    # ------------------------------------------------------------------ 상태 · 관측
    def _pose_err(self):
        """추정 − 참 (B,3): 위치 추정 오차 + 큰 튐 (nf7) + yaw 흐름 (nf7)"""
        e = self.map_err + self.big_err
        return torch.stack([e[:, 0], e[:, 1], e[:, 2] + self.yaw_drift], -1)

    def _est_pose(self):
        p, e = self.plant.pose, self._pose_err()
        return torch.stack([p[:, 0] + e[:, 0], p[:, 1] + e[:, 1], wrap(p[:, 2] + e[:, 2])], -1)

    def _state_now(self):
        return torch.cat([self._est_pose(), self.plant.q, self.plant.w, self.plant.odom], -1)

    def _delayed(self, ring, d):
        idx = (self.tick - 1 - d) % RING
        return ring[torch.arange(self.num_envs, device=self.device), idx]

    def _goal_err(self, pose):
        dist = (self.path.goal - pose[:, :2]).norm(dim=-1)
        dyaw = wrap(self.path.goal_yaw - pose[:, 2])
        return dist, dyaw

    def _potential(self):
        rw = self.rw
        dist, dyaw = self._goal_err(self._est_pose())
        return -(rw.goal_pos * dist.clamp(max=rw.goal_zone) + rw.goal_yaw * self._yaw_weight(dist) * dyaw.abs())

    def _yaw_weight(self, dist):
        """목표 방향 가중 (목표 근처 1, 멀면 0). goal_yaw_far ≤ 0 이면 늘 1"""
        rw = self.rw
        if rw.goal_yaw_far <= 0:
            return torch.ones_like(dist)
        return ((rw.goal_yaw_far - dist) / (rw.goal_yaw_far - rw.goal_yaw_near)).clamp(0.0, 1.0)

    def _compute_obs(self, reset_ids=None):
        c, dr, g = self.c, self.dr, self.geom
        B, dev = self.num_envs, self.device
        st = self._delayed(self.state_ring, self.d_obs) if self.tick > 0 else self._state_now()
        pose, q, w, odom = st[:, :3], st[:, 3:7], st[:, 7:11], st[:, 11:14]
        q = q + torch.randn(B, 4, device=dev, generator=self.gen) * dr.noise_steer
        odom = odom + torch.randn(B, 3, device=dev, generator=self.gen) * torch.tensor(
            [dr.noise_lin_vel, dr.noise_lin_vel, dr.noise_ang_vel], device=dev)
        self.s_est, _, _, _ = self.path.project(pose[:, :2], self.s_est, c.proj_back, c.proj_fwd)
        sk = self.s_est.unsqueeze(-1) + c.path_obs_spacing * torch.arange(c.path_obs_points, device=dev)
        pk, _ = self.path.point_at(sk)
        pk = to_body(pk, pose).reshape(B, -1)
        gb = to_body(self.path.goal.unsqueeze(1), pose).squeeze(1)
        dyaw = wrap(self.path.goal_yaw - pose[:, 2])
        rem = self.path.length - self.s_est
        goal = torch.stack([gb[:, 0] / 2, gb[:, 1] / 2, dyaw.sin(), dyaw.cos(), (rem / 3).clamp(max=2.0)], -1)
        # 라이다 (참 자세, 잡음 · 빠짐)
        if c.lidar_model == 'real':                    # 빔마다 잡음 · 빠짐 → 몸 가림 · 몸 상자 · 칸 최솟값 (배포 /scan 과 같은 처리)
            nb = self.occ.beam.numel()
            noise = torch.randn(B, nb, device=dev, generator=self.gen) * dr.lidar_noise
            drop = torch.rand(B, nb, device=dev, generator=self.gen) < dr.lidar_dropout
            # 추적 = 관측 자르기 (2 m) + 라이다 원점 오프셋 (0.115 m) + 여유 까지만: 원점이 몸 중심에서 떨어져 있어 라이다에서 2.1 m 인 점도
            #   몸 중심에선 2 m 안일 수 있음. 그 밖 점은 관측에서 어차피 2 m 로 잘림 → 3 m 추적과 같은 관측 (8192 env 에서 차이 0 확인), 18.9 → 16.5 ms
            scan = self._lidar_real(self.plant.pose, self.plant.q, self.occ, c.lidar_rays, self.lidar_trace_max, noise, drop)
        else:
            scan = self._lidar(self.plant.pose, c.lidar_rays, c.lidar_max)
            scan = scan + torch.randn_like(scan) * dr.lidar_noise
            scan = torch.where(torch.rand(scan.shape, device=dev, generator=self.gen) < dr.lidar_dropout, torch.full_like(scan, c.lidar_max), scan)
        self.scan = scan
        lid = scan.clamp(0, c.lidar_obs_clip) / c.lidar_obs_clip
        rpos = (q - g.rmin) / (g.rmax - g.rmin) * 2 - 1
        frame = torch.cat([q.sin(), q.cos(), rpos, w / self.plant.max_wheel, odom / self.lim3, self.last_cmd / self.lim3], -1)
        self.hist = torch.cat([frame.unsqueeze(1), self.hist[:, :-1]], 1)
        if reset_ids is not None and len(reset_ids):
            self.hist[reset_ids] = frame[reset_ids].unsqueeze(1)
        policy = torch.cat([pk, goal, lid, self.hist.reshape(B, -1)], -1)
        # 비대칭 critic (참 값 · DR 값)
        pt = self.plant.pose
        s_t, lat_t, _, _ = self.path.project(pt[:, :2], self.s_true, c.proj_back, c.proj_fwd)
        dist_t, dy_t = self._goal_err(pt)
        clr = self.world.clearance(pt, self.fp_clear)
        rel = self.world.people_c - pt[:, None, :2]
        dpp = torch.where(self.world.people_on, rel.norm(dim=-1) - self.world.people_r, torch.full_like(self.world.people_r, 5.0))
        priv = torch.cat([self.plant.body_vel() / self.lim3, (lat_t / 0.1).unsqueeze(-1), dy_t.sin().unsqueeze(-1), dy_t.cos().unsqueeze(-1),
                          (dist_t / 3).unsqueeze(-1), (clr / 0.1).clamp(-1, 3).unsqueeze(-1), (dpp.min(-1).values / 1.0).clamp(max=3).unsqueeze(-1),
                          torch.stack([self.plant.rate_max / 4.8, self.plant.tau_s / 0.15, self.plant.tau_w / 0.06, self.plant.tau_b / 0.08,
                                       self.d_cmd / 3.0, self.d_obs / 4.0, self.ctrl.blend_ratio, self.ctrl.gate_blocked.float()], -1),
                          self._pose_err() * torch.tensor([60.0, 60.0, 57.3], device=dev), self.plant.slip,
                          torch.full((B, 1), float(self.level), device=dev)], -1)
        return TensorDict({'policy': policy, 'privileged': priv}, batch_size=[B], device=dev)

    def get_observations(self) -> TensorDict:
        return self.obs

    # ------------------------------------------------------------------ 명령 · 물리
    def _command(self, a):
        c = self.c
        lin = a[:, :2] * c.vmax
        n = lin.norm(dim=-1, keepdim=True)
        lin = lin * torch.where(n > c.vmax, c.vmax / n.clamp(min=1e-9), torch.ones_like(n))
        wz = a[:, 2:3] * c.wmax
        if c.rot_limit:                                # 벽 가까우면 회전 상한 (라이다 최솟값, 배포 노드 command 와 같은 식)
            dmin = self.scan.min(-1, keepdim=True).values
            t = ((dmin - c.rot_clear_lo) / (c.rot_clear_hi - c.rot_clear_lo)).clamp(0.0, 1.0)
            lim = c.rot_wz_lo + (c.wmax - c.rot_wz_lo) * t
            self.rot_capped = (wz.abs() > lim).squeeze(-1)
            self.rot_excess = (wz.abs() - lim).clamp(min=0.0).squeeze(-1)      # 상한을 넘은 요청 [rad/s] (보상용)
            wz = torch.maximum(torch.minimum(wz, lim), -lim)
        if c.approach_limit:                           # 목표 근처 속도 상한 (추정 자세 거리, 배포 노드 command 와 같은 식)
            dist, _ = self._goal_err(self._est_pose())
            vlim = (c.approach_v0 + c.approach_k * dist).clamp(max=c.vmax).unsqueeze(-1)
            n = lin.norm(dim=-1, keepdim=True)
            self.approach_capped = (n > vlim).squeeze(-1)
            lin = lin * torch.where(n > vlim, vlim / n.clamp(min=1e-9), torch.ones_like(n))
        return torch.cat([lin, wz], -1)

    def _physics_impl(self, cmd_d):
        sg, wg = self.ctrl.update(cmd_d, self.plant.q, self.plant.w, self.plant.qd, self.c.ctrl_dt)
        self.prev_steer_goal = sg
        q0 = self.plant.q
        self.plant.step(sg, wg)
        travel = (self.plant.q - q0).abs().sum(-1)
        return travel, self._state_now()

    # ------------------------------------------------------------------ 스텝
    def step(self, actions):
        c, dr, rw, dev = self.c, self.dr, self.rw, self.device
        B = self.num_envs
        a = actions.clamp(-1.0, 1.0)
        jump = self._rand(B, 0, 1) < dr.map_jump_prob * self.dt / dr.map_jump_every_s
        if jump.any():
            dj = torch.stack([self._rand(B, -1, 1) * dr.map_jump_pos, self._rand(B, -1, 1) * dr.map_jump_pos,
                              self._rand(B, -1, 1) * math.radians(dr.map_jump_yaw_deg)], -1)
            lim = torch.tensor([dr.map_err_pos, dr.map_err_pos, math.radians(dr.map_err_yaw_deg)], device=dev)
            err = torch.maximum(torch.minimum(self.map_err + dj, lim), -lim)
            self.map_err = torch.where(jump.unsqueeze(-1), err, self.map_err)
        if dr.yaw_drift_dps > 0:                       # yaw 흐름 (nf7)
            lim_d = math.radians(dr.yaw_drift_max_deg)
            self.yaw_drift = (self.yaw_drift + self.drift_rate * self.dt).clamp(-lim_d, lim_d)
        if dr.big_jump_rate > 0:                       # 큰 튐 (nf7): 줄어들다가 드물게 새로 튐, 튈 때 yaw 흐름 0 (SLAM 보정)
            self.big_err = self.big_err * math.exp(-self.dt / dr.big_jump_decay_s)
            big = self._rand(B, 0, 1) < dr.big_jump_rate * self.dt
            if big.any():
                ang = self._rand(B, -math.pi, math.pi)
                mag = self._rand(B, *dr.big_jump_pos)
                yj = self._rand(B, *dr.big_jump_yaw_deg) * torch.where(self._rand(B, 0, 1) < 0.5, -1.0, 1.0) * (math.pi / 180)
                bj = torch.stack([mag * ang.cos(), mag * ang.sin(), yj], -1)
                self.big_err = torch.where(big.unsqueeze(-1), bj, self.big_err)
                self.yaw_drift = torch.where(big, torch.zeros_like(self.yaw_drift), self.yaw_drift)
        prev_wz = self.last_cmd[:, 2].clone()
        cmd = self._command(a)
        ar = torch.arange(B, device=dev)
        travel = torch.zeros(B, device=dev)
        for _ in range(c.decimation):
            self.cmd_ring[:, self.tick % RING] = cmd
            cmd_d = self.cmd_ring[ar, (self.tick - self.d_cmd) % RING]
            tr, st = self._physics(cmd_d)
            travel += tr
            self.tick += 1
            self.state_ring[:, (self.tick - 1) % RING] = st
        self.last_cmd = cmd
        self.episode_length_buf += 1

        # 진행 · 목표 (추정 자세)
        pose = self._est_pose()
        s_new, _, dev_path, _ = self.path.project(pose[:, :2], self.s_true, c.proj_back, c.proj_fwd)
        ds = s_new - self.s_true
        ds = torch.where(ds.abs() > c.vmax * self.dt + 0.05, torch.zeros_like(ds), ds)
        self.s_true = s_new
        dist, dyaw = self._goal_err(pose)
        phi = self._potential()
        r_pot = phi - self.phi
        self.phi = phi
        # 몸 (참 자세)
        clr = self.world.clearance(self.plant.pose, self.fp_clear)
        collide = self.world.clearance(self.plant.pose, self.fp_collide) < 0
        zone_hit = torch.zeros_like(collide)
        if self.c.stop_zone > 0:                                       # 실물 PolygonStop 구역 (사람 뺌, 목표 · 출발 근처 뺌)
            clr_np = self.world.clearance(self.plant.pose, self.fp_clear, people=False)
            exempt = (dist < self.c.stop_zone_goal_exempt) | ((self.plant.pose[:, :2] - self.start_xy).norm(dim=-1) < self.c.stop_zone_start_exempt)
            zone_hit = (clr_np < self.c.stop_zone) & ~exempt & ~collide
            collide = collide | zone_hit
        near_goal = (dist < 0.3).float()
        r_clear = -rw.clearance * ((rw.clear_margin - clr) / rw.clear_margin).clamp(0, 1) * (1 - near_goal)
        r_dev = -rw.deviation * ((dev_path - 0.1) / 0.1).clamp(0, 4) ** 2
        r_act = -rw.action_rate * ((a - self.last_action) ** 2).sum(-1)
        near_mult = torch.where(dist < rw.steer_near_dist, torch.full_like(dist, rw.steer_near_mult), torch.ones_like(dist))
        r_prog, r_steer = rw.progress * ds, -rw.steer_travel * near_mult * travel
        # 멈춤 (목표 아닌 곳): 이동 속도 거의 0 (제자리 회전 포함), 출발 직후 · 사람 가까이는 뺌
        w = self.world
        dpp = torch.where(w.people_on, (w.people_c - self.plant.pose[:, None, :2]).norm(dim=-1) - w.people_r, torch.full_like(w.people_r, 9.0))
        stalled = ((dist > rw.stall_goal_dist) & (self.plant.odom[:, :2].norm(dim=-1) < rw.stall_speed)
                   & (self.episode_length_buf.float() * self.dt > rw.stall_grace_s) & (dpp.min(-1).values > rw.stall_person_dist))
        r_stall = -rw.stall * stalled.float()
        r_bound = -rw.action_bound * ((actions.abs() - 1.0).clamp(min=0.0) ** 2).sum(-1)       # 잘리기 전 행동
        r_rot = -rw.rot_cost * (1.0 - self._yaw_weight(dist)) * cmd[:, 2].abs() / c.wmax
        r_cap = -rw.rot_cap_excess * self.rot_excess / c.wmax if c.rot_limit else torch.zeros_like(r_rot)
        r_near = -rw.rot_near * (dist < rw.rot_near_dist).float() * cmd[:, 2].abs() / c.wmax
        r_hold = -rw.hold_cmd * (dist < rw.hold_dist).float() * ((cmd[:, :2].norm(dim=-1) / c.vmax) ** 2 + (cmd[:, 2] / c.wmax) ** 2)
        # 헛회전 (참 자세): 이번 스텝 몸 yaw 변화 중 목표 방향 오차를 줄이지 않은 만큼
        yaw_now = self.plant.pose[:, 2]
        dyaw_abs = wrap(self.path.goal_yaw - yaw_now).abs()
        turned = wrap(yaw_now - self.prev_yaw).abs()
        waste = (turned - (self.prev_dyaw_abs - dyaw_abs).clamp(min=0.0)).clamp(min=0.0)
        self.prev_yaw, self.prev_dyaw_abs = yaw_now.clone(), dyaw_abs
        m = rw.rot_reversal_min
        reversed_ = ((prev_wz >= m) & (cmd[:, 2] <= -m)) | ((prev_wz <= -m) & (cmd[:, 2] >= m))
        r_waste = -rw.rot_waste * waste
        r_rev = -rw.rot_reversal * reversed_.float()
        rew = (r_prog + r_pot + r_clear + r_dev + r_act - rw.time + r_steer + r_stall + r_bound + r_rot + r_cap + r_near + r_hold
               + r_waste + r_rev)
        self.last_action = a

        bv = self.plant.odom
        tol_p, tol_y = c.goal_pos_tol[self.level], math.radians(c.goal_yaw_tol_deg[self.level])
        at_goal = (dist < tol_p) & (dyaw.abs() < tol_y) & (bv[:, :2].norm(dim=-1) < c.goal_lin_vel) & (bv[:, 2].abs() < c.goal_ang_vel)
        self.hold = torch.where(at_goal, self.hold + 1, torch.zeros_like(self.hold))
        success = self.hold * self.dt >= c.goal_hold_s - 1e-6
        lost = (dev_path > c.fail_dev) & ~success
        collide &= ~success
        timeout = (self.episode_length_buf >= self.max_episode_length_t) & ~success & ~collide & ~lost
        rew = rew + rw.success * success.float() - rw.collision * collide.float() - rw.lost * lost.float()
        dones = success | collide | lost | timeout

        # 진단 (매 스텝): 보상 항별 평균 · 행동 포화 (샘플 행동이 [-1, 1] 밖 = 잘림 → 같은 행동, 탐색 왜곡). 튜터 PPO 글 8.5 · 8.6
        sat = (actions.abs() > 1.0).float()
        cap = self.rot_capped.float().mean() if c.rot_limit else torch.zeros((), device=dev)
        diag = {'/rew/progress': r_prog.mean(), '/rew/potential': r_pot.mean(), '/rew/clearance': r_clear.mean(),
                '/rew/deviation': r_dev.mean(), '/rew/stall': r_stall.mean(), '/rew/action_bound': r_bound.mean(), '/rew/rot_cost': r_rot.mean(), '/rew/rot_cap_excess': r_cap.mean(), '/rew/rot_near': r_near.mean(), '/rew/hold_cmd': r_hold.mean(), '/rew/rot_waste': r_waste.mean(), '/rew/rot_reversal': r_rev.mean(),
                '/act/approach_capped': self.approach_capped.float().mean() if c.approach_limit else torch.zeros((), device=dev),
                '/act/mean_out': (actions.abs() > 1.0).float().mean(), '/act/wz_abs': cmd[:, 2].abs().mean(), '/stall/frac': stalled.float().mean(), '/rew/action_rate': r_act.mean(), '/rew/steer_travel': r_steer.mean(),
                '/rew/terminal': (rw.success * success.float() - rw.collision * collide.float() - rw.lost * lost.float()).mean(),
                '/act/sat_frac': sat.mean(), '/act/sat_vx': sat[:, 0].mean(), '/act/sat_vy': sat[:, 1].mean(), '/act/sat_wz': sat[:, 2].mean(),
                '/act/raw_abs_mean': actions.abs().mean(), '/act/rot_capped': cap}
        e = self.ep
        e['ret'] += rew
        e['travel'] += travel
        e['stall'] += stalled.float() * self.dt
        e['rot'] += turned
        e['waste'] += waste
        e['rev'] += reversed_.float()
        e['clear_min'] = torch.minimum(e['clear_min'], clr)
        e['dev_max'] = torch.maximum(e['dev_max'], dev_path)
        log = dict(diag)
        done_ids = torch.nonzero(dones).squeeze(-1)
        dist_true, dyaw_true = self._goal_err(self.plant.pose)
        if len(done_ids):
            steps = self.episode_length_buf[done_ids].float()
            dd = self.is_dice[done_ids]
            log |= {'/ep/success': success[done_ids].float().mean(), '/ep/collision': collide[done_ids].float().mean(),
                   '/ep/lost': lost[done_ids].float().mean(), '/ep/timeout': timeout[done_ids].float().mean(),
                   '/ep/return': e['ret'][done_ids].mean(), '/ep/steer_travel_rad': e['travel'][done_ids].mean(),
                   '/ep/clear_min_m': e['clear_min'][done_ids].mean(), '/ep/stall_s': e['stall'][done_ids].mean(), '/ep/dev_max_m': e['dev_max'][done_ids].mean(),
                   '/ep/final_dist_m': dist[done_ids].mean(), '/ep/final_dyaw_deg': dyaw[done_ids].abs().mean() * 57.3,
                   '/ep/rot_total_rad': e['rot'][done_ids].mean(), '/ep/rot_waste_rad': e['waste'][done_ids].mean(),
                   '/ep/rot_reversals': e['rev'][done_ids].mean()}
            if dd.any():
                log['/dice/success'] = success[done_ids][dd].float().mean()
                log['/dice/collision'] = collide[done_ids][dd].float().mean()
            xd = self.bank_extra[self.cur_k[done_ids]]
            if xd.any():
                log['/extra/success'] = success[done_ids][xd].float().mean()
                log['/extra/collision'] = collide[done_ids][xd].float().mean()
            # 충돌 원인 (끝난 에피소드 중 비율): 목표 0.3 m 안 (정밀 정차) · 사람 (사람 빼면 안 닿음) · 그 밖 (주행 중 벽 · 박스)
            cids = done_ids[collide[done_ids]]
            if len(cids):
                n_done = float(len(done_ids))
                near = dist_true[cids] < 0.3
                by_person = self.world.clearance(self.plant.pose[cids], self.fp_collide, self.world.map_id[cids], people=False) >= 0
                log['/col/near_goal'] = torch.tensor(float(near.sum()) / n_done)
                log['/col/person'] = torch.tensor(float((by_person & ~near).sum()) / n_done)
                log['/col/travel_wall'] = torch.tensor(float((~by_person & ~near).sum()) / n_done)
                log['/col/stop_zone'] = torch.tensor(float(zone_hit[cids].sum()) / n_done)
                dc = self.is_dice[cids]
                log['/col/dice_near_goal'] = torch.tensor(float((near & dc).sum()) / n_done)
            ok = done_ids[success[done_ids]]
            if len(ok):
                log['/ep/time_s'] = (self.episode_length_buf[ok].float() * self.dt).mean()
                log['/ep/speed_mps'] = (self.path.length[ok] / (self.episode_length_buf[ok].float() * self.dt)).mean()
                okd = ok[self.is_dice[ok]]
                if len(okd):
                    log['/dice/time_per_m_s'] = (self.episode_length_buf[okd].float() * self.dt / self.path.length[okd]).mean()
            self._record_outcomes(success[done_ids].float())
            log['/curriculum/level'] = torch.tensor(float(self.level))
            log['/curriculum/success_window'] = self.outcomes[:min(self.outcome_count, len(self.outcomes))].mean()
            self.last_done = {'ids': done_ids, 'success': success[done_ids], 'collision': collide[done_ids], 'lost': lost[done_ids],
                              'timeout': timeout[done_ids], 'time_s': steps * self.dt, 'travel': e['travel'][done_ids].clone(),
                              'clear_min': e['clear_min'][done_ids].clone(), 'final_dist': dist[done_ids], 'final_dyaw': dyaw[done_ids].abs(),
                              'final_dist_true': dist_true[done_ids], 'final_dyaw_true': dyaw_true[done_ids].abs(),
                              'length': self.path.length[done_ids].clone(), 'dice': dd.clone(), 'stall_s': e['stall'][done_ids].clone(),
                              'rot': e['rot'][done_ids].clone(), 'rot_waste': e['waste'][done_ids].clone(), 'rot_rev': e['rev'][done_ids].clone()}
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
