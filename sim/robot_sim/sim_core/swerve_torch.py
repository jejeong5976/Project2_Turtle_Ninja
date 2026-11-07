"""스워브 컨트롤러 torch 벡터화 판 — robot_description/swerve/nt_swerve.py(SwerveDrive.update) 와 같은 규칙을 env 수천 개에 한 번에.

nt_swerve 는 env 마다 Python 루프라 학습에 못 씀 → 같은 식을 (B, 4) 텐서로 옮김. 일치 검사 tools/check_swerve_torch.py.
옮긴 범위 = 이 로봇 설정(config/swerve.yaml)이 쓰는 갈래만:
  모듈별 조향 범위 있음 · coherent 조향 · blend_steer_drive 켬(blend_stall_s > 0) · cosine_scale 켬 → '균일 cos 감속'.
  명령은 매 주기 새로 줌 (fresh 항상 참, cmd_timeout 흉내 안 냄).
순서: 명령 크기 제한 → 실행 가능 검사 → 가속 제한 → blend 이분 탐색(14회) → 역기구학 → 균일 cos 감속
      → 이전 목표 갱신 → 조향 중 굴림 보상 → 조향 정렬 게이트 → 바퀴 최고 속도 정규화 → blend 최종 조향각.
단위: m, rad, s. 바퀴 속도 = 바퀴 각속도 [rad/s]. 모듈 순서 fl, fr, rl, rr.
"""
from __future__ import annotations

import math
from pathlib import Path

import torch
import yaml

PI = math.pi
STOP_EPS = 1e-4
SWERVE_YAML = Path(__file__).resolve().parents[1] / 'robot_description' / 'config' / 'swerve.yaml'


def wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.remainder(a + PI, 2 * PI) - PI


def cpp_round(x: torch.Tensor) -> torch.Tensor:
    """std::round (0.5 는 0 에서 먼 쪽)"""
    return torch.sign(x) * torch.floor(x.abs() + 0.5)


class SwerveGeometry:
    """swerve.yaml 기하 · 범위 · 컨트롤러 값 (텐서)"""

    def __init__(self, device, dtype=torch.float32, path: str | Path = SWERVE_YAML):
        c = yaml.safe_load(Path(path).read_text())
        g, rg, ctl, lim = c['geometry'], c['steer_range_rad'], c['controller'], c['limits_for_training']
        t = lambda v: torch.tensor(v, device=device, dtype=dtype)  # noqa: E731
        self.mx, self.my, self.off = t(g['module_x']), t(g['module_y']), t(g['module_offset'])
        self.r = float(g['wheel_radius'])
        self.rmin, self.rmax = t(rg['min']), t(rg['max'])
        assert ctl['steering_policy'] == 'coherent' and ctl['cosine_scale'] and ctl['blend_steer_drive'] and ctl['blend_stall_s'] > 0, \
            'swerve_torch 는 coherent · cosine_scale · blend 켬 설정만 옮김'
        self.margin = float(ctl['steer_branch_margin'])
        self.slack = float(ctl['steer_end_slack'])
        self.steer_then_drive = bool(ctl['steer_then_drive'])
        self.steer_tolerance = float(ctl['steer_tolerance'])
        self.roll_while_steering = bool(ctl['roll_while_steering'])
        self.blend_margin = float(ctl['blend_margin'])
        self.blend_stall_s = float(ctl['blend_stall_s'])
        self.max_wheel_speed = float(lim['max_wheel_speed'])
        self.hw = c['hardware_max']
        self.limits = lim
        self.device, self.dtype = device, dtype

    def forward(self, q: torch.Tensor, lin: torch.Tensor) -> torch.Tensor:
        """순기구학 (nt_swerve.SwerveKinematics.forward): 조향각 q (B,4), 바퀴 접지 구름 속도 lin = ω·r + θ̇·offset (B,4)
        → 몸 속도 (vx, vy, wz) 최소제곱 (B,3)."""
        c, s = torch.cos(q), torch.sin(q)
        roll = torch.stack([c, s, self.mx * s - self.my * c - self.off], -1)       # (B,4,3) 구름 방향
        side = torch.stack([-s, c, self.mx * c + self.my * s], -1)                  # (B,4,3) 옆 방향 (속도 0)
        A = torch.cat([roll, side], 1)                                               # (B,8,3)
        b = torch.cat([lin, torch.zeros_like(lin)], 1).unsqueeze(-1)                 # (B,8,1)
        ata = A.transpose(1, 2) @ A
        atb = A.transpose(1, 2) @ b
        return torch.linalg.solve(ata, atb).squeeze(-1)


class SwerveTorch:
    """env B 개의 SwerveDrive 상태 기계. 한계(max_linear 등)는 env 마다 다를 수 있음 (B,) 텐서."""

    def __init__(self, num_envs: int, device, dtype=torch.float32, steer_tolerance: float = PI,
                 geom: SwerveGeometry | None = None):
        self.g = geom or SwerveGeometry(device, dtype)
        self.B, self.device, self.dtype = num_envs, device, dtype
        self.steer_tolerance = steer_tolerance    # 사용자 결정: π = 연속 조향 (swerve.yaml 0.17 덮어씀, SwerveActionCfg 와 같음)
        z = lambda *s: torch.zeros(*s, device=device, dtype=dtype)  # noqa: E731
        lim = self.g.limits
        self.max_linear = z(num_envs) + lim['max_linear']
        self.max_angular = z(num_envs) + lim['max_angular']
        self.max_linear_accel = z(num_envs) + lim['max_linear_accel']
        self.max_angular_accel = z(num_envs) + lim['max_angular_accel']
        self.out = z(num_envs, 3)
        self.translating = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.prev = z(num_envs, 4)
        self.have_prev = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.blend_since = z(num_envs) - 1.0
        self.blend_ratio = z(num_envs) + 1.0
        self.blend_stalled = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.gate_blocked = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.feasible = torch.ones(num_envs, dtype=torch.bool, device=device)
        self.now = z(num_envs)

    def reset(self, ids):
        self.out[ids] = 0.0
        self.translating[ids] = False
        self.prev[ids] = 0.0
        self.have_prev[ids] = False
        self.blend_since[ids] = -1.0
        self.blend_ratio[ids] = 1.0
        self.blend_stalled[ids] = False
        self.gate_blocked[ids] = False
        self.feasible[ids] = True
        self.now[ids] = 0.0

    def stop(self, ids):
        """정지 상태로 (새 경로 시작 전): 출력 속도만 0. 조향 분기 이력은 실물처럼 남김"""
        self.out[ids] = 0.0
        self.translating[ids] = False

    # ------------------------------------------------------------ 역기구학 (범위 있음 · coherent 갈래)
    def inverse(self, cmd, now, cosine_scale: bool, prev, have_prev, prefer_out):
        """cmd (B,3), now = 측정 조향각 (B,4), prev (B,4) · have_prev (B,) = 이전 목표, prefer_out (B,)
        → (조향각 (B,4), 바퀴 각속도 (B,4), 실행 가능 (B,))"""
        g = self.g
        dt = cmd.dtype
        vx, vy, wz = cmd[:, 0:1], cmd[:, 1:2], cmd[:, 2:3]
        hp = have_prev.unsqueeze(-1)                                   # (B,1)
        hyst = 20.0 * PI / 180.0

        # 공통 각 (coherent): 몸 병진 방향 + nπ 중 모든 모듈 범위 안, 점수 최소
        lin = torch.hypot(vx, vy).squeeze(-1)
        heading = torch.atan2(vy, vx)                                  # (B,1)
        first = torch.ceil((g.rmin[0] - heading) / PI)
        last = torch.floor((g.rmax[0] - heading) / PI)
        n = first + torch.arange(4, device=cmd.device, dtype=dt)       # (B,K) 범위 폭 ≤ 2π → 후보 ≤ 3
        a = heading + n * PI                                           # (B,K)
        aj = a.unsqueeze(-1)                                           # (B,K,1)
        ret = hp.unsqueeze(1) & ((aj - prev.unsqueeze(1)).abs() < PI / 2)   # (B,K,4)
        guard = torch.where(ret, 0.0, g.margin)
        inside = (aj >= g.rmin + guard - g.slack) & (aj <= g.rmax - guard + g.slack)
        valid = (n <= last) & inside.all(-1)
        score = aj.abs() + torch.where(hp.unsqueeze(1) & ~ret, hyst, 0.0)
        cost = torch.where(valid, score.sum(-1), torch.inf)
        kbest = cost.argmin(1, keepdim=True)
        common = a.gather(1, kbest)                                    # (B,1)
        common_ok = torch.isfinite(cost.gather(1, kbest)) & (lin >= STOP_EPS).unsqueeze(-1)

        # 모듈별 후보 2 개 (정 · 역)
        mvx = vx - wz * g.my
        mvy = vy + wz * g.mx                                           # (B,4)
        speed = torch.hypot(mvx, mvy)
        stopped = speed < STOP_EPS
        head = torch.atan2(mvy, mvx)
        base = torch.stack([head, head + PI], -1)                      # (B,4,2)
        rmin, rmax = g.rmin.unsqueeze(-1), g.rmax.unsqueeze(-1)
        fk = torch.ceil((rmin - base - g.slack - 1e-12) / (2 * PI))
        lk = torch.floor((rmax - base + g.slack + 1e-12) / (2 * PI))
        exist = fk <= lk
        nowe = now.unsqueeze(-1)
        k = torch.minimum(torch.maximum(cpp_round((nowe - base) / (2 * PI)), fk), lk)
        raw = base + k * 2 * PI
        ang = torch.minimum(torch.maximum(raw, rmin), rmax)
        sgn = torch.tensor([1.0, -1.0], device=cmd.device, dtype=dt)
        v = sgn * speed.unsqueeze(-1) * torch.cos(raw - ang)
        retm = hp.unsqueeze(-1) & ((ang - prev.unsqueeze(-1)).abs() < PI / 2)
        safe = retm | ((ang >= rmin + g.margin) & (ang <= rmax - g.margin))
        safe_exists = (exist & safe).any(-1, keepdim=True)
        mxe, mye, offe = g.mx.unsqueeze(-1), g.my.unsqueeze(-1), g.off.unsqueeze(-1)
        outward = -offe * torch.sin(ang) * mxe + offe * torch.cos(ang) * mye
        dead = (g.off.abs() * torch.hypot(g.mx, g.my) * math.sin(10.0 * PI / 180.0)).unsqueeze(-1)
        og = 10.0 * PI / 180.0
        out_ok = exist & (~safe_exists | safe) & (outward >= -dead) & (ang >= rmin + og) & (ang <= rmax - og)
        po = prefer_out.view(-1, 1, 1)
        outward_alt = po & out_ok.any(-1, keepdim=True)
        allowed = exist & ~(safe_exists & ~safe)
        not_ret = hp.unsqueeze(-1) & ~retm
        cst_common = (ang - common.unsqueeze(-1)).abs() + torch.where(not_ret, hyst, 0.0) + torch.where(
            po, torch.where(outward_alt & (outward < -dead), 2 * PI, 0.0), torch.where(not_ret, 2 * PI, 0.0))
        cst_free = ang.abs() + torch.where(not_ret, hyst, 0.0) + torch.where(~po & not_ret, 2 * PI, 0.0)
        cst = torch.where(common_ok.unsqueeze(-1), cst_common, cst_free)
        cst = torch.where(allowed, cst, torch.inf)
        pick = (cst[..., 1] < cst[..., 0]).long().unsqueeze(-1)       # 같으면 앞(정방향) — Python 의 '<' 와 같음
        best = ang.gather(-1, pick).squeeze(-1)
        bv = v.gather(-1, pick).squeeze(-1)
        mod_ok = torch.isfinite(cst.gather(-1, pick).squeeze(-1))
        wheel_lin = bv - wz * g.off
        if cosine_scale:
            wheel_lin = wheel_lin * torch.clamp(torch.cos(best - now), min=0.0)
        angle = torch.where(stopped, now, best)
        wheel = torch.where(stopped, -wz * g.off / g.r, wheel_lin / g.r)
        finite = torch.isfinite(cmd).all(-1)
        ok = finite & (mod_ok | stopped).all(-1)
        angle = torch.where(ok.unsqueeze(-1), angle, now)
        wheel = torch.where(ok.unsqueeze(-1), wheel, 0.0)
        return angle, wheel, ok

    # ------------------------------------------------------------ 한 주기
    def _limit_accel(self, cur, tgt, dt):
        d = tgt - cur
        t_lin = torch.where(self.max_linear_accel > 0, torch.hypot(d[:, 0], d[:, 1]) / self.max_linear_accel, 0.0)
        t_ang = torch.where(self.max_angular_accel > 0, d[:, 2].abs() / self.max_angular_accel, 0.0)
        t_need = torch.maximum(t_lin, t_ang)
        f = torch.where(t_need <= dt, 1.0, dt / t_need.clamp(min=1e-12))
        return torch.where((t_need <= dt).unsqueeze(-1), tgt, cur + d * f.unsqueeze(-1))

    def update(self, cmd, meas_q, meas_w, meas_rate, dt: float):
        """cmd (B,3) 몸 속도 명령, 측정 조향각 · 바퀴 각속도 · 조향 각속도 (B,4) → (조향 목표 (B,4), 바퀴 각속도 목표 (B,4))"""
        g = self.g
        self.now += dt
        # 명령 크기 제한 (set_command)
        lin = torch.hypot(cmd[:, 0], cmd[:, 1])
        sc = torch.where(lin > self.max_linear, self.max_linear / lin.clamp(min=1e-12), 1.0)
        wanted = torch.stack([cmd[:, 0] * sc, cmd[:, 1] * sc,
                              torch.maximum(torch.minimum(cmd[:, 2], self.max_angular), -self.max_angular)], -1)
        wl = torch.hypot(wanted[:, 0], wanted[:, 1])
        ol = torch.hypot(self.out[:, 0], self.out[:, 1])
        self.translating &= ~((wl < STOP_EPS) & (ol < STOP_EPS))
        po = ~self.translating
        no_prev = torch.zeros_like(self.have_prev)
        hold_a, hold_w, ok0 = self.inverse(wanted, meas_q, True, self.prev, no_prev, po)

        before = self.out.clone()
        out = self._limit_accel(self.out, wanted, min(max(dt, 0.0), 0.1))
        trans = torch.hypot(before[:, 0], before[:, 1])
        moving = (trans >= STOP_EPS) & (trans >= before[:, 2].abs() * 0.13)
        meas_ok = torch.isfinite(meas_q).all(-1)
        tol = self.steer_tolerance * min(max(g.blend_margin, 0.0), 1.0)

        def lerp(a):
            return before + a.unsqueeze(-1) * (out - before)

        def fits(a):
            ang, _, f = self.inverse(lerp(a), meas_q, True, self.prev, self.have_prev, po)
            return f & ((ang - meas_q).abs() <= tol).all(-1)

        one = torch.ones_like(trans)
        zero = torch.zeros_like(trans)
        cand = moving & meas_ok & ok0          # 실행 불가(ok0 거짓)면 Python 은 여기 오기 전에 돌아감
        need = cand & ~fits(one) & fits(zero)
        blend_ratio = torch.ones_like(trans)
        final_a = meas_q.clone()
        have_final = torch.zeros_like(need)
        # 이분 탐색은 항상 돌리고 need 인 env 에만 적용 (CPU 동기화 없이 → torch.compile 한 덩어리로)
        lo, hi = zero.clone(), one.clone()
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            f = fits(mid)
            lo = torch.where(f, mid, lo)
            hi = torch.where(f, hi, mid)
        out = torch.where(need.unsqueeze(-1), lerp(lo), out)
        blend_ratio = torch.where(need, lo, blend_ratio)
        fa, _, fok = self.inverse(wanted, meas_q, False, self.prev, self.have_prev, po)
        have_final = need & fok
        final_a = torch.where(have_final.unsqueeze(-1), fa, final_a)
        # blend 멈춤 판정 (조향이 실제로 안 돌면서 blend 가 1 s 넘게 이어짐)
        part = cand & (blend_ratio < 1.0)
        turning = (meas_rate.abs() > 0.1).any(-1)
        restart = part & ((self.blend_since < 0) | turning)
        self.blend_since = torch.where(restart, self.now, self.blend_since)
        stalled = part & ((self.now - self.blend_since) > g.blend_stall_s)
        self.blend_since = torch.where(part, self.blend_since, -1.0)

        ang, wheel, ok = self.inverse(out, meas_q, False, self.prev, self.have_prev, po)
        smin = torch.clamp(torch.cos(wrap(ang - meas_q)), min=0.0).amin(-1, keepdim=True)
        wheel = torch.where(ok.unsqueeze(-1), wheel * smin, wheel)
        ok = ok & ok0
        out = torch.where(ok.unsqueeze(-1), out, 0.0)
        ang = torch.where(ok0.unsqueeze(-1), ang, hold_a)
        wheel = torch.where(ok0.unsqueeze(-1), wheel, hold_w)

        moved = ok & ((torch.hypot(out[:, 0], out[:, 1]) >= STOP_EPS) | (out[:, 2].abs() >= STOP_EPS))
        self.prev = torch.where(moved.unsqueeze(-1), torch.where(have_final.unsqueeze(-1), final_a, ang), self.prev)
        self.have_prev |= moved
        roll = -meas_rate * g.off / g.r if g.roll_while_steering else torch.zeros_like(wheel)
        wheel = torch.where(ok.unsqueeze(-1), wheel + roll, wheel)
        aligned = ((ang - meas_q).abs() <= self.steer_tolerance).all(-1)
        gate = ok & (stalled | ~aligned) if g.steer_then_drive else ok & stalled
        wheel = torch.where(gate.unsqueeze(-1), roll, wheel)
        out = torch.where(gate.unsqueeze(-1), 0.0, out)
        self.translating |= torch.hypot(out[:, 0], out[:, 1]) >= STOP_EPS
        peak = wheel.abs().amax(-1, keepdim=True)
        if g.max_wheel_speed > 0:
            wheel = torch.where((peak > g.max_wheel_speed) & ok0.unsqueeze(-1), wheel * g.max_wheel_speed / peak.clamp(min=1e-12), wheel)
        ang = torch.where(have_final.unsqueeze(-1), final_a, ang)

        self.out = out
        self.blend_ratio = blend_ratio
        self.blend_stalled = stalled
        self.gate_blocked = gate
        self.feasible = ok
        return ang, wheel
