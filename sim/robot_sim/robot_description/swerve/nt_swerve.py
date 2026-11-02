"""닌자터틀 스워브 계산 — 팀 nt_control/swerve.cpp(2026-10-27 로컬 판)를 줄 단위로 옮긴 Python 판.

시뮬·학습 환경에서 "몸 속도 명령(vx, vy, wz)" → "바퀴 4개의 조향 각도·구동 속도"로 바꿀 때 씁니다.
실물 컨트롤러와 같은 규칙(coherent 조향 선택, 멈춤→출발 때 바퀴 바깥쪽 선호, 이동 중 분기 유지,
범위 끝 1° 여유, 동시 조향·구동(blend), 조향 먼저 게이트, 바퀴 최고 속도 정규화)을 그대로 따릅니다.
C++ 과의 일치 검사: tools/check_swerve_port.py (같은 입력 → 같은 출력).

단위: m, rad, s. 바퀴 속도는 바퀴 각속도 [rad/s]. 모듈 순서는 fl, fr, rl, rr(URDF 관절 이름 순서).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

PI = math.pi
STOP_EPS = 1e-4          # 이보다 느리면 "멈춤" [m/s]  (swerve.cpp kStopEps)


def wrap_angle(a: float) -> float:
    a = math.fmod(a + PI, 2.0 * PI)
    if a < 0.0:
        a += 2.0 * PI
    return a - PI


@dataclass
class Module:
    x: float
    y: float
    offset: float = 0.0   # 바퀴 접지점이 조향축에서 떨어진 거리 [m] (좌 +, 우 −)


@dataclass
class Range:
    min: float
    max: float


@dataclass
class ModuleState:
    steer_angle: float = 0.0
    wheel_speed: float = 0.0
    steer_rate: float = 0.0


@dataclass
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def limit_accel(cur: Twist, tgt: Twist, max_lin: float, max_ang: float, dt: float) -> Twist:
    if dt <= 0.0:
        return cur
    dvx, dvy, dw = tgt.vx - cur.vx, tgt.vy - cur.vy, tgt.wz - cur.wz
    t_lin = math.hypot(dvx, dvy) / max_lin if max_lin > 0.0 else 0.0
    t_ang = abs(dw) / max_ang if max_ang > 0.0 else 0.0
    t_need = max(t_lin, t_ang)
    if t_need <= dt:
        return Twist(tgt.vx, tgt.vy, tgt.wz)
    f = dt / t_need
    return Twist(cur.vx + dvx * f, cur.vy + dvy * f, cur.wz + dw * f)


def steering_aligned(cmd: Sequence[ModuleState], meas: Sequence[ModuleState], tol: float) -> bool:
    if len(cmd) != len(meas):
        return False
    return all(abs(c.steer_angle - m.steer_angle) <= tol for c, m in zip(cmd, meas))


class SwerveKinematics:
    def __init__(self, modules: List[Module], wheel_radius: float, steer_limit: float = PI,
                 ranges: Optional[List[Range]] = None, coherent: bool = False,
                 branch_margin: float = 3.0 * PI / 180.0, end_slack: float = 0.0):
        if not modules:
            raise ValueError("모듈이 하나도 없음")
        if wheel_radius <= 0.0:
            raise ValueError("바퀴 반지름은 0 보다 커야 함")
        self.modules = modules
        self.r = wheel_radius
        self.steer_limit = max(steer_limit, PI / 2.0)
        self.ranges = list(ranges or [])
        if self.ranges and len(self.ranges) != len(modules):
            raise ValueError("모듈별 조향 범위 개수가 다름")
        for rg in self.ranges:
            if not (math.isfinite(rg.min) and math.isfinite(rg.max)) or rg.min > rg.max or rg.max - rg.min > 2 * PI:
                raise ValueError("조향 범위는 유한한 연속 구간(min <= max, 폭 <= 2pi)이어야 함")
        if not (math.isfinite(end_slack) and 0.0 <= end_slack <= 2.0 * PI / 180.0):
            raise ValueError("steer_end_slack 은 0 ~ 2° 이어야 함")
        self.coherent = coherent
        self.branch_margin = branch_margin
        self.end_slack = end_slack

    def size(self) -> int:
        return len(self.modules)

    def has_module_ranges(self) -> bool:
        return bool(self.ranges)

    def roll_while_steering(self, i: int, steer_rate: float) -> float:
        return -steer_rate * self.modules[i].offset / self.r

    # ---------------------------------------------------------------- 역기구학
    def inverse(self, cmd: Twist, current: Sequence[ModuleState] = (), cosine_scale: bool = True,
                previous: Sequence[ModuleState] = (), prefer_outward: bool = True):
        """반환: (모듈 명령 리스트, feasible)"""
        n = len(self.modules)
        have_current = len(current) == n
        out = [ModuleState() for _ in range(n)]
        if self.coherent and not self.ranges:
            k = SwerveKinematics(self.modules, self.r, self.steer_limit,
                                 [Range(-self.steer_limit, self.steer_limit)] * n, True, self.branch_margin, self.end_slack)
            return k.inverse(cmd, current, cosine_scale, previous, prefer_outward)
        if self.ranges:
            margin = self.branch_margin
            hysteresis = 20.0 * PI / 180.0
            have_previous = len(previous) == n

            def retained(i, angle):
                return have_previous and abs(angle - previous[i].steer_angle) < PI / 2

            def score(i, angle):
                return abs(angle) + (hysteresis if have_previous and not retained(i, angle) else 0.0)

            common = math.nan
            if self.coherent and math.hypot(cmd.vx, cmd.vy) >= STOP_EPS:
                heading = math.atan2(cmd.vy, cmd.vx)
                cost = math.inf
                first = math.ceil((self.ranges[0].min - heading) / PI)
                last = math.floor((self.ranges[0].max - heading) / PI)
                nn = first
                while nn <= last:
                    a = heading + nn * PI
                    c = 0.0
                    valid = True
                    for j, rg in enumerate(self.ranges):
                        guard = 0.0 if retained(j, a) else margin
                        if a < rg.min + guard - self.end_slack or a > rg.max - guard + self.end_slack:
                            valid = False
                            break
                        c += score(j, a)
                    if valid and c < cost:
                        cost, common = c, a
                    nn += 1

            def reject():
                for j in range(n):
                    out[j] = ModuleState(current[j].steer_angle if have_current else min(max(0.0, self.ranges[j].min), self.ranges[j].max), 0.0)
                return out, False

            if not (math.isfinite(cmd.vx) and math.isfinite(cmd.vy) and math.isfinite(cmd.wz)):
                return reject()
            if len(current) != 0 and not have_current:
                return reject()
            for i, rg in enumerate(self.ranges):
                mod = self.modules[i]
                now = current[i].steer_angle if have_current else min(max(0.0, rg.min), rg.max)
                if not math.isfinite(now):
                    return reject()
                vx = cmd.vx - cmd.wz * mod.y
                vy = cmd.vy + cmd.wz * mod.x
                speed = math.hypot(vx, vy)
                if speed < STOP_EPS:
                    out[i] = ModuleState(now, -cmd.wz * mod.offset / self.r)
                    continue
                heading = math.atan2(vy, vx)
                cands = []   # (angle, speed, safe)
                for reverse in (0, 1):
                    base = heading + reverse * PI
                    first = math.ceil((rg.min - base - self.end_slack - 1e-12) / (2 * PI))
                    last = math.floor((rg.max - base + self.end_slack + 1e-12) / (2 * PI))
                    if first > last:
                        continue
                    k = min(max(_cpp_round((now - base) / (2 * PI)), first), last)
                    angle = min(max(base + k * 2 * PI, rg.min), rg.max)
                    v = (-speed if reverse else speed) * math.cos(base + k * 2 * PI - angle)
                    cands.append((angle, v, retained(i, angle) or (rg.min + margin <= angle <= rg.max - margin)))
                safe_exists = any(c[2] for c in cands)

                def outward(a, mod=mod):
                    return -mod.offset * math.sin(a) * mod.x + mod.offset * math.cos(a) * mod.y

                out_deadband = abs(mod.offset) * math.hypot(mod.x, mod.y) * math.sin(10.0 * PI / 180.0)
                out_guard = 10.0 * PI / 180.0
                outward_alt = prefer_outward and any(
                    ((not self.coherent) or (not safe_exists) or d[2]) and outward(d[0]) >= -out_deadband
                    and rg.min + out_guard <= d[0] <= rg.max - out_guard for d in cands)
                best, best_speed, distance = 0.0, 0.0, math.inf
                for angle, v, safe in cands:
                    if self.coherent and safe_exists and not safe:
                        continue
                    if not self.coherent:
                        cst = abs(angle - now)
                    elif math.isfinite(common):
                        cst = abs(angle - common) + (hysteresis if have_previous and not retained(i, angle) else 0.0)
                        if prefer_outward:
                            cst += 2.0 * PI if (outward_alt and outward(angle) < -out_deadband) else 0.0
                        else:
                            cst += 2.0 * PI if (have_previous and not retained(i, angle)) else 0.0
                    else:
                        cst = score(i, angle) + (2.0 * PI if (not prefer_outward and have_previous and not retained(i, angle)) else 0.0)
                    if cst < distance:
                        distance, best, best_speed = cst, angle, v
                if not math.isfinite(distance):
                    return reject()
                wheel_lin = best_speed - cmd.wz * mod.offset
                if cosine_scale and have_current:
                    wheel_lin *= max(0.0, math.cos(best - now))
                out[i] = ModuleState(best, wheel_lin / self.r)
            return out, True
        # 범위 없는 옛 방식(실물에서는 쓰지 않음, 호환용)
        for i, mod in enumerate(self.modules):
            vx = cmd.vx - cmd.wz * mod.y
            vy = cmd.vy + cmd.wz * mod.x
            speed = math.hypot(vx, vy)
            now_angle = current[i].steer_angle if have_current else 0.0
            if speed < STOP_EPS:
                out[i] = ModuleState(now_angle, -cmd.wz * mod.offset / self.r)
                continue
            angle = math.atan2(vy, vx)
            if have_current and abs(wrap_angle(angle - now_angle)) > PI / 2.0:
                angle = wrap_angle(angle + PI)
                speed = -speed
            if have_current:
                angle = now_angle + wrap_angle(angle - now_angle)
                if angle > self.steer_limit:
                    angle -= PI
                    speed = -speed
                elif angle < -self.steer_limit:
                    angle += PI
                    speed = -speed
            wheel_lin = speed - cmd.wz * mod.offset
            if cosine_scale and have_current:
                wheel_lin *= math.cos(wrap_angle(angle - now_angle))
            out[i] = ModuleState(angle, wheel_lin / self.r)
        return out, True

    # ---------------------------------------------------------------- 순기구학 (odom)
    def forward(self, states: Sequence[ModuleState]) -> Twist:
        if len(states) != len(self.modules):
            raise ValueError("모듈 상태 개수가 모듈 수와 다름")
        ata = [[0.0] * 3 for _ in range(3)]
        atb = [0.0] * 3

        def add(a, b):
            for r in range(3):
                for c in range(3):
                    ata[r][c] += a[r] * a[c]
                atb[r] += a[r] * b
        for st, mod in zip(states, self.modules):
            lin = st.wheel_speed * self.r + st.steer_rate * mod.offset
            c, s = math.cos(st.steer_angle), math.sin(st.steer_angle)
            add((c, s, mod.x * s - mod.y * c - mod.offset), lin)
            add((-s, c, mod.x * c + mod.y * s), 0.0)
        m = [ata[r] + [atb[r]] for r in range(3)]
        for col in range(3):
            piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
            if abs(m[piv][col]) < 1e-12:
                return Twist(0.0, 0.0, 0.0)
            m[col], m[piv] = m[piv], m[col]
            for r in range(3):
                if r != col:
                    f = m[r][col] / m[col][col]
                    for c in range(col, 4):
                        m[r][c] -= f * m[col][c]
        return Twist(m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2])


def _cpp_round(x: float) -> float:
    """std::round: 0.5 는 0 에서 먼 쪽 (Python round 는 짝수 쪽)"""
    return math.floor(x + 0.5) if x >= 0 else -math.floor(-x + 0.5)


@dataclass
class DriveConfig:
    max_wheel_speed: float = 0.0
    max_linear: float = 0.10
    max_angular: float = 0.5
    max_linear_accel: float = 0.2
    max_angular_accel: float = 0.8
    cmd_timeout_s: float = 0.3
    cosine_scale: bool = True
    steer_then_drive: bool = True
    steer_tolerance: float = 0.17
    roll_while_steering: bool = True
    blend_steer_drive: bool = False
    blend_margin: float = 1.0
    blend_stall_s: float = 0.0


class SwerveDrive:
    """실물 swerve_controller 의 한 주기 계산(SwerveDrive::update)과 같은 상태 기계.
    시뮬에서는 매 제어 주기마다 set_command → update(실측 모듈 상태) → 조향 위치·바퀴 속도 목표를 관절에 줍니다.
    (팔 차례/motion_mode 잠금은 시뮬용으로 항상 '주행 허용'으로 둡니다.)"""

    def __init__(self, kin: SwerveKinematics, cfg: DriveConfig):
        self.kin, self.cfg = kin, cfg
        self.out = Twist()
        self.cmd = Twist()
        self.cmd_time = -1e9
        self.have_cmd = False
        self.previous_targets: List[ModuleState] = []
        self.translating = False
        self.blend_since = -1.0
        self.blend_ratio = 1.0
        self.blend_stalled = False
        self.gate_blocked = False
        self.command_feasible = True

    def set_command(self, vx: float, vy: float, wz: float, now_s: float) -> bool:
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            self.have_cmd = False
            return False
        lin = math.hypot(vx, vy)
        if self.cfg.max_linear > 0.0 and lin > self.cfg.max_linear:
            vx *= self.cfg.max_linear / lin
            vy *= self.cfg.max_linear / lin
        if self.cfg.max_angular > 0.0:
            wz = min(max(wz, -self.cfg.max_angular), self.cfg.max_angular)
        self.cmd = Twist(vx, vy, wz)
        self.cmd_time = now_s
        self.have_cmd = True
        return True

    def reset(self):
        self.__init__(self.kin, self.cfg)

    def update(self, now_s: float, dt: float, measured: Sequence[ModuleState]) -> List[ModuleState]:
        cfg, kin = self.cfg, self.kin
        fresh = self.have_cmd and 0.0 <= (now_s - self.cmd_time) + 0.05 and (now_s - self.cmd_time) < cfg.cmd_timeout_s
        wanted = Twist(self.cmd.vx, self.cmd.vy, self.cmd.wz) if fresh else Twist()
        if (not fresh or math.hypot(wanted.vx, wanted.vy) < STOP_EPS) and math.hypot(self.out.vx, self.out.vy) < STOP_EPS:
            self.translating = False
        prefer_outward = not self.translating
        self.command_feasible = True
        if kin.has_module_ranges():
            hold, ok = kin.inverse(wanted, measured, cfg.cosine_scale, (), prefer_outward)
            if not ok:
                self.command_feasible = False
                self.out = Twist()
                return hold
        before = Twist(self.out.vx, self.out.vy, self.out.wz)
        self.out = limit_accel(self.out, wanted, cfg.max_linear_accel, cfg.max_angular_accel, min(max(dt, 0.0), 0.1))
        self.blend_ratio = 1.0
        self.gate_blocked = False
        trans = math.hypot(before.vx, before.vy)
        moving = trans >= STOP_EPS and trans >= abs(before.wz) * 0.13
        measured_ok = len(measured) == kin.size() and all(math.isfinite(m.steer_angle) for m in measured)
        blend_on = cfg.blend_steer_drive and cfg.blend_stall_s > 0.0
        self.blend_stalled = False
        final_targets: List[ModuleState] = []
        if blend_on and fresh and moving and measured_ok:
            tol = cfg.steer_tolerance * min(max(cfg.blend_margin, 0.0), 1.0)
            target = Twist(self.out.vx, self.out.vy, self.out.wz)

            def lerp(a):
                return Twist(before.vx + a * (target.vx - before.vx), before.vy + a * (target.vy - before.vy),
                             before.wz + a * (target.wz - before.wz))

            def fits(a):
                m, feas = kin.inverse(lerp(a), measured, cfg.cosine_scale, self.previous_targets, prefer_outward)
                return feas and steering_aligned(m, measured, tol)
            if not fits(1.0) and fits(0.0):
                lo, hi = 0.0, 1.0
                for _ in range(14):
                    mid = 0.5 * (lo + hi)
                    if fits(mid):
                        lo = mid
                    else:
                        hi = mid
                self.out = lerp(lo)
                self.blend_ratio = lo
                ft, ok = kin.inverse(wanted, measured, False, self.previous_targets, prefer_outward)
                final_targets = ft if ok else []
            if self.blend_ratio < 1.0:
                turning = any(math.isfinite(m.steer_rate) and abs(m.steer_rate) > 0.1 for m in measured)
                if self.blend_since < 0.0 or turning:
                    self.blend_since = now_s
                if now_s - self.blend_since > cfg.blend_stall_s:
                    self.blend_stalled = True
            else:
                self.blend_since = -1.0
        else:
            self.blend_since = -1.0
        uniform = blend_on and cfg.cosine_scale and measured_ok
        modules, ok = kin.inverse(self.out, measured, False if uniform else cfg.cosine_scale, self.previous_targets, prefer_outward)
        self.command_feasible = ok
        if uniform and ok:
            s_min = 1.0
            for i, m in enumerate(modules):
                s_min = min(s_min, max(0.0, math.cos(wrap_angle(m.steer_angle - measured[i].steer_angle))))
            for m in modules:
                m.wheel_speed *= s_min
        if not ok:
            self.out = Twist()
            return modules
        if not fresh:
            aligned = steering_aligned(modules, measured, cfg.steer_tolerance)
            if not aligned:
                self.out = Twist()
            for i in range(min(len(modules), len(measured))):
                modules[i].steer_angle = measured[i].steer_angle
                if not aligned:
                    modules[i].wheel_speed = 0.0
        elif math.hypot(self.out.vx, self.out.vy) >= STOP_EPS or abs(self.out.wz) >= STOP_EPS:
            src = final_targets if final_targets else modules
            self.previous_targets = [ModuleState(m.steer_angle, m.wheel_speed, m.steer_rate) for m in src]
        if blend_on and fresh and cfg.roll_while_steering:
            for i in range(min(len(modules), len(measured))):
                if math.isfinite(measured[i].steer_rate):
                    modules[i].wheel_speed += kin.roll_while_steering(i, measured[i].steer_rate)
        if self.blend_stalled or (cfg.steer_then_drive and not steering_aligned(modules, measured, cfg.steer_tolerance)):
            self.gate_blocked = True
            for i in range(len(modules)):
                rate = measured[i].steer_rate if i < len(measured) else 0.0
                modules[i].wheel_speed = kin.roll_while_steering(i, rate) if cfg.roll_while_steering else 0.0
            self.out = Twist()
        if math.hypot(self.out.vx, self.out.vy) >= STOP_EPS:
            self.translating = True
        if cfg.max_wheel_speed > 0.0:
            peak = max(abs(m.wheel_speed) for m in modules)
            if peak > cfg.max_wheel_speed:
                for m in modules:
                    m.wheel_speed *= cfg.max_wheel_speed / peak
        if final_targets and len(final_targets) == len(modules):
            for i in range(len(modules)):
                modules[i].steer_angle = final_targets[i].steer_angle
        return modules


# ---------------------------------------------------------------- 이 로봇 설정 불러오기
def from_config(path: str):
    """config/swerve.yaml → (SwerveKinematics, DriveConfig, 관절 이름)"""
    import yaml
    c = yaml.safe_load(open(path))
    g = c["geometry"]
    mods = [Module(x, y, o) for x, y, o in zip(g["module_x"], g["module_y"], g["module_offset"])]
    rng = [Range(lo, hi) for lo, hi in zip(c["steer_range_rad"]["min"], c["steer_range_rad"]["max"])]
    ctl = c["controller"]
    kin = SwerveKinematics(mods, g["wheel_radius"], math.pi, rng, ctl["steering_policy"] == "coherent",
                           ctl["steer_branch_margin"], ctl["steer_end_slack"])
    lim = c["limits_for_training"]
    cfg = DriveConfig(max_wheel_speed=lim["max_wheel_speed"], max_linear=lim["max_linear"], max_angular=lim["max_angular"],
                      max_linear_accel=lim["max_linear_accel"], max_angular_accel=lim["max_angular_accel"],
                      cmd_timeout_s=ctl["cmd_timeout_s"], cosine_scale=ctl["cosine_scale"], steer_then_drive=ctl["steer_then_drive"],
                      steer_tolerance=ctl["steer_tolerance"], roll_while_steering=ctl["roll_while_steering"],
                      blend_steer_drive=ctl["blend_steer_drive"], blend_margin=ctl["blend_margin"], blend_stall_s=ctl["blend_stall_s"])
    return kin, cfg, c["joints"]
