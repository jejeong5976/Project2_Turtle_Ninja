"""스워브 간이 플랜트 (torch, env B 개) — 학습용. 실제 접촉 물리는 Isaac 평가로 확인.

한 틱 (dt = 0.01 s, 컨트롤러와 같은 주기):
  조향: θ̇ = clip((θ_goal − θ) / τ_s, ±rate_max), θ ← clamp(θ + θ̇·dt, 범위)  → 실제 θ̇ = (θ_new − θ)/dt (범위 끝에서 0)
  바퀴: ω ← ω + α_w (ω_goal − ω), α = 1 − exp(−dt/τ),  |ω| ≤ 최고 속도
  실제 몸 속도 목표 = 순기구학(θ, 접지 구름 = slip·ω·r + θ̇·offset) (최소제곱 = 조향이 어긋나면 바퀴끼리 타협한 속도, 미끄럼 근사)
  몸 속도: world 좌표에서 1차 지연 α_b (관성) → 자세 적분
  odom (관측) = 순기구학(θ, ω·r + θ̇·offset) — 엔코더 기준이라 slip 안 걸림 (실물 /odom 과 같은 식)
"""
import math

import torch

from sim_core.swerve_torch import SwerveGeometry


class SwervePlant:
    def __init__(self, num_envs: int, geom: SwerveGeometry, dt: float):
        self.g, self.B, self.dt = geom, num_envs, dt
        dev, dty = geom.device, geom.dtype
        z = lambda *s: torch.zeros(*s, device=dev, dtype=dty)  # noqa: E731
        self.q, self.qd, self.w = z(num_envs, 4), z(num_envs, 4), z(num_envs, 4)
        self.pose = z(num_envs, 3)          # world (x, y, yaw)
        self.vel_w = z(num_envs, 3)         # world (vx, vy, wz)
        self.odom = z(num_envs, 3)          # body (vx, vy, wz) 엔코더 순기구학
        # DR (env 별)
        self.rate_max = z(num_envs) + 4.8
        self.tau_s = z(num_envs) + 0.08
        self.tau_w = z(num_envs) + 0.04
        self.tau_b = z(num_envs) + 0.04
        self.slip = z(num_envs, 4) + 1.0
        self.max_wheel = float(geom.hw['wheel_speed'])

    def reset_motion(self, ids, pose):
        """정지 상태로 자세 지정. 조향각은 그대로 둠 (이전 주행 끝 상태)"""
        self.pose[ids] = pose
        self.vel_w[ids] = 0.0
        self.odom[ids] = 0.0
        self.w[ids] = 0.0
        self.qd[ids] = 0.0

    def step(self, steer_goal, wheel_goal):
        g, dt = self.g, self.dt
        rate = ((steer_goal - self.q) / self.tau_s.unsqueeze(-1))
        rate = torch.maximum(torch.minimum(rate, self.rate_max.unsqueeze(-1)), -self.rate_max.unsqueeze(-1))
        qn = torch.minimum(torch.maximum(self.q + rate * dt, g.rmin), g.rmax)
        self.qd = (qn - self.q) / dt
        self.q = qn
        aw = 1.0 - torch.exp(-dt / self.tau_w).unsqueeze(-1)
        self.w = (self.w + aw * (wheel_goal - self.w)).clamp(-self.max_wheel, self.max_wheel)
        # 참 몸 속도 (접지)
        lin_true = self.slip * self.w * g.r + self.qd * g.off
        tb = g.forward(self.q, lin_true)                       # body
        c, s = torch.cos(self.pose[:, 2]), torch.sin(self.pose[:, 2])
        tw = torch.stack([c * tb[:, 0] - s * tb[:, 1], s * tb[:, 0] + c * tb[:, 1], tb[:, 2]], -1)
        ab = (1.0 - torch.exp(-dt / self.tau_b)).unsqueeze(-1)
        self.vel_w = self.vel_w + ab * (tw - self.vel_w)
        p = self.pose + self.vel_w * dt
        p[:, 2] = torch.remainder(p[:, 2] + math.pi, 2 * math.pi) - math.pi
        self.pose = p
        # odom (엔코더)
        self.odom = g.forward(self.q, self.w * g.r + self.qd * g.off)

    def body_vel(self):
        """참 몸 속도 (body 좌표)"""
        c, s = torch.cos(self.pose[:, 2]), torch.sin(self.pose[:, 2])
        v = self.vel_w
        return torch.stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], v[:, 2]], -1)
