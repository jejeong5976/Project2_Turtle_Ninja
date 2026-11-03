#!/usr/bin/env python3
"""팔 자세 · 경로 안전 검사 (2026-10-30, 계산만 · 포트 안 씀). 모터로 움직이는 도구(arm_pose_runner) 가 실행 전에 반드시 통과시켜야 함.

  $NT_PY pose_safety.py poses.json            # 자세 · 자세 사이 경로 검사
  $NT_PY pose_safety.py --calib out.json       # 손눈 보정용 자세 자동 생성 (검사 통과 + 정확도 최대)

자세 파일 (JSON): {"poses": [{"name": "p1", "ticks": [J1, J2, J3, J4, J5], "gripper": 3504(선택), "check": true}, …]}
  틱 순서 = ID 11 12 13 14 16 (J1 · 그리퍼는 확장 위치 모드: 받침 자세에서 전원 켠 기준).
검사 (자세마다 + 이웃 자세 사이를 관절 공간 직선으로 2° 간격):
  1. 관절 범위: URDF 한계 안쪽 5° (J1 은 지도 정면 4088 기준 ±150°)
  2. 차체 · J1 몸통 · 받침대 · 카메라 기둥 · 프로파일 판: 15 mm 이상 (받침 자세 출발/도착 구간은 받침대 접촉 허용 — 정해진 세우기 경유점만)
  3. 적재함: 칸 무리 두 곳(사진 어림, 가로세로 15 mm 여유 · 세우기 구간 3 mm) 위로 퍽 윗면(상판 + 60 mm) + 20 mm 아래는 금지 (어깨 · 팔꿈치 · 손목 · 손가락)
  4. 바닥: 바닥 위 25 mm 이상
  5. 자기 충돌: 손목 · 손가락 ↔ 어깨 · J1 쪽 15 mm 이상
"""
import argparse
import json
import os
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import arm_pose_fk as fk  # noqa: E402
from reach_lib import PLATE_TOP_Z, apply  # noqa: E402
from reach_model import ADJACENT, Model  # noqa: E402

BASE_Z = 0.072
REST = [4084, 1102, 2668, 1971, 2063]
# 받침 → P0 세우기 경유점 (ChatGPT 팔 엔진 run_task_calib_poses_demo1.sh 의 검증된 순서: J2 +512 · J3 −160 → J4 2034 → J2 −448 → J3 2380 → +77)
# ChatGPT 팔 엔진 run_task_calib_poses_demo1.sh 의 실물 검증 순서 그대로 (2026-10-30 저녁 교체 — 첫 실물 실행에서 복귀 끝에 걸림)
#   세우기: S:12:d+64/13:d-20 … 8 조각 (J2 +496 · J3 −206 동시) → 14:2034 → 12:d-128 ×3, d-64 → 13:2380 → 13:d+77
#   복귀  : 11:d-32 → 12:d+128 ×3, d+64 → 11:d+32 → 14:d-62 → S:12:d-48/13:d+24 … 8 조각
#   복귀 중 J2 올릴 때 J1 −32: 10-25 A9 실물에서 '왼쪽 돌출부 걸림' (ChatGPT CHANGES.md 67~70 행) 뒤 사용자 결정.
_UP_J2 = [64, 64, 64, 64, 64, 64, 64, 48]
_UP_J3 = [-20, -22, -24, -28, -28, -30, -30, -24]


def _standup():
    out, t = [], list(REST)
    for k, (d2, d3) in enumerate(zip(_UP_J2, _UP_J3)):
        t = [t[0], t[1] + d2, t[2] + d3, t[3], t[4]]; out.append((f'up{k + 1}', list(t)))
    t[3] = 2034; out.append(('upJ4', list(t)))
    for k, d in enumerate([-128, -128, -128, -64]):
        t[1] += d; out.append((f'upJ2_{k + 1}', list(t)))
    t[2] = 2380; out.append(('upJ3a', list(t)))
    t[2] += 77; out.append(('P0', list(t)))
    return out


def _return(p0):
    out, t = [], list(p0)
    t[0] -= 32; out.append(('rJ1a', list(t)))
    for k, d in enumerate([128, 128, 128, 64]):
        t[1] += d; out.append((f'rJ2_{k + 1}', list(t)))
    t[0] += 32; out.append(('rJ1b', list(t)))
    t[3] -= 62; out.append(('rJ4', list(t)))
    for k, (d2, d3) in enumerate(zip(reversed(_UP_J2), reversed(_UP_J3))):
        t = [t[0], t[1] - d2, t[2] - d3, t[3], t[4]]; out.append((f'r{k + 1}', list(t)))
    return out


STANDUP = _standup()
RETURN = _return(STANDUP[-1][1])
STAND_MARGIN = 0.003                            # 받침 근처 세우기 구간: 받침대 · 차체와 원래 가까움 → 3 mm (실물에서 검증된 순서만)
MARGIN = 0.015
# 10-30 손눈 보정: 손목 굽힘 합(J2+J3+J4)이 모델보다 약 +5.3° → 어느 관절인지 모름 → 세 가설 + 원래 모델 모두에서 통과해야 함 (관절 번호 0 부터)
#   J3−5.3 은 뺌: 그 가설이면 받침 자세(실물로 놓여 있음)에서 팔이 상판을 8 mm 뚫어야 함 → 불가능 (10-30 저녁 검사)
#   (17:40 교체) 처짐 모델 확정(sag_model.py, c 3.07 °/N·m, 실행기가 앞먹임) → 남는 오차 가설 = 어깨 · 팔꿈치 ±2° (처짐 3° 안팎 · 보정 덜 맞음 · 앞먹임 없는 옛 자세 파일 포함)
HYPS = [('', {}), ('J2+2', {1: 2.0}), ('J2−2', {1: -2.0}), ('J3+2', {2: 2.0}), ('J3−2', {2: -2.0})]
EXTRA_OBS = []          # 자세 파일 'obstacles' (박스 · 퍽 등, base_link 기준) → reach_lib.Analytic
OBS_MARGIN = {'cup': 0.006, 'L_box': 0.005}   # 이름이 이것으로 시작하는 장애물은 이 여유 (10-30 저녁 적재: 수직 손 손가락 끝이 컵 윗면 위 10 mm) · 박스 윗면 5 mm (11-01 L8 · L7: 35 · 40 mm 깊게 쥐면 손끝이 박스 위 모델 20 · 15 mm, 실물은 약 5 mm 더 높음)
PLACED_PUCK_MARGIN = float(os.environ.get('NT_PLACED_PUCK_MARGIN', '0.002'))   # 한 번만 낮출 때 NT_PLACED_PUCK_MARGIN=0.001 (눈으로 보며, 11-01)    · 찬 칸에서 컵 위로 나온 퍽('cup X puck') 여유 — 이웃 두 칸이 차면 6 mm 로는 어느 손목 방향도 안 됨 (10-31 실물 B: 최악 2~3 mm, 퍽 크기에 밀림 2 mm 포함) · 10-31 19:3x 3 → 2 mm: 스캔 C–D 간격 38.3 mm 로 컵(40 mm) 보다 좁게 잼 = 실제 ≥ 1.7 mm 숨은 여유, D 가 C 퍽과 [J3−2] 2 mm
FLOOR_MARGIN = 0.025
RACK_TOP = PLATE_TOP_Z + 0.060 + 0.020          # 퍽 윗면(상판 + 홀더 30 + 퍽 노출 30) + 여유 20 mm
RACK_LINKS = ('link2', 'link3', 'link4', 'link5', 'finger_a', 'finger_b')
# 적재함 칸 무리 (10-30 위에서 찍은 사진으로 어림, ±10 mm → 가로세로 15 mm 여유 포함): (x_min, x_max, y_min, y_max) base_link 기준 m
#   왼쪽 2 열 × 4 줄 (칸 가운데 y +67 / +107, x +95 / +32 / −29 / −93) · 오른쪽 2 열 × 2 줄 (y −64 / −104, x +96 / +36), 칸 37.8 mm
RACK_ZONES = [(-0.112, 0.114, 0.048, 0.126), (0.017, 0.115, -0.123, -0.045)]          # 칸 바깥 모서리 (여유 없음)
RACK_XY_MARGIN, RACK_XY_MARGIN_STAND = 0.015, 0.003
RACK_TOP_STAND = PLATE_TOP_Z + 0.060 + 0.003     # 세우기 구간: 퍽 윗면 + 3 mm (모델상 받침 자세 손목 브래킷이 앞 칸 퍽 위 약 5 mm — 실물 확인 필요)
# 세우기 구간은 J1 · J5 고정이라 손가락이 좌우로 안 움직임 (손가락 y −38 ~ +39 mm, 칸 안쪽 모서리 ±45~48 mm → 실제 6~8 mm) — 받침 자세 자체가 그렇게 놓여 있음 (사진)
SELF_PAIRS = [('link5', 'link1'), ('finger_a', 'link1'), ('finger_b', 'link1'), ('link5', 'link2'), ('finger_a', 'link2'), ('finger_b', 'link2')]
STATIC_OK_NEAR_REST = ('park_stand',)


# ---------- 자세 검사 결과 저장 (11-01) ----------
# 같은 틱 · 그리퍼 · 장애물 · 여유 · 모델 파일이면 결과가 같음 → 디스크에 저장해 다시 씀 (받침 복귀 길처럼 매번 같은 구간이 대부분).
#   키에 장애물 목록 전체(이름 · 위치 · 크기)와 여유 값, 모델 · 보정 파일 내용 해시를 넣으므로 하나라도 바뀌면 예전 결과를 안 씀.
#   NT_POSE_CACHE=0 이면 끔.
import atexit, hashlib, pickle  # noqa: E401,E402
_CACHE_ON = os.environ.get('NT_POSE_CACHE', '1') != '0'
_CACHE = {}
_CACHE_NEW = {}


def _code_key():
    import reach_lib
    files = [Path(__file__), HERE / 'arm_pose_fk.py', HERE / 'sag_model.py', HERE / 'camera_calib.json',
             Path(reach_lib.__file__), Path(reach_lib.__file__).with_name('reach_model.py'), reach_lib.URDF_PATH,
             Path(reach_lib.URDF_PATH).parent.parent / 'config' / 'joint_map.json']
    h = hashlib.sha1(repr((PLACED_PUCK_MARGIN, OBS_MARGIN, MARGIN, STAND_MARGIN, FLOOR_MARGIN, HYPS)).encode())
    for f in files:
        h.update(Path(f).read_bytes() if Path(f).exists() else str(f).encode())
    return h.hexdigest()[:16]


_CACHE_FILE = HERE / '.pose_cache' / f'{_code_key()}.pkl' if _CACHE_ON else None


def _cache_load():
    try:
        _CACHE.update(pickle.loads(_CACHE_FILE.read_bytes()))
    except Exception:
        pass


def _cache_save():
    if not _CACHE_NEW:
        return
    try:
        _CACHE_FILE.parent.mkdir(exist_ok=True)
        old = {}
        try:
            old = pickle.loads(_CACHE_FILE.read_bytes())          # 다른 프로세스가 그새 쓴 것과 합침
        except Exception:
            pass
        old.update(_CACHE_NEW)
        tmp = _CACHE_FILE.with_suffix(f'.{os.getpid()}.tmp')
        tmp.write_bytes(pickle.dumps(old))
        tmp.replace(_CACHE_FILE)
    except Exception as e:
        print('자세 검사 저장 실패 (계산엔 영향 없음):', e)


if _CACHE_ON:
    _cache_load()
    atexit.register(_cache_save)


def _obs_key():
    return hashlib.sha1(repr([(o.name, o.kind, np.round(o.T, 6).tolist(), np.round(o.dims, 6).tolist()) for o in EXTRA_OBS]).encode()).hexdigest()[:16]


class Checker:
    def __init__(self, spacing=0.004):
        self.spacing = spacing
        self.M = Model(spacing=spacing)
        self.mesh_obs = dict(self.M.static)                       # chassis · j1_housing · park_stand · profile_plate · camera_column
        jm = fk.JM['arm']
        self.tick_rng = []
        for k, a in enumerate(jm):
            lo, hi = math.degrees(self.M.lo[k]) + 5, math.degrees(self.M.hi[k]) - 5
            t = sorted([a['ref_tick'] + a['sign'] * (d - a['q_deg_at_ref']) * 4096 / 360 for d in (lo, hi)])
            self.tick_rng.append(t)
        j4 = list(self.tick_rng[3])                                      # CAD 모델 J4 한계 [틱] (아래에서 그대로 씀)
        # J1: 예전엔 ±150° 로 따로 묶었음 → 11-01 사용자 '리밋을 적게 잡지 마' · 손으로 L7 에 넣을 때 J1 5781 틱(148.6°) — 아래 ROS 범위(±π, 끝 2° 여유) 만 씀
        # 팀 ROS (nt_opencr) 로 움직일 때 추가 한계: 일반 위치 모드는 펌웨어가 목표를 0~4095 로 잘라냄,
        # robot.urdf.xacro 한계 J1 ±π · J2 ±1.5 · J3 −1.5~1.4 · J4 −1.7~1.97 · J5 ±1.5708 rad (rad = (틱 − 2048)·2π/4096), 끝에서 2° 여유
        #   (10-30 20:05) 영점은 ros/hardware_map_arm_only.yaml (arm_map) — J1 영점 4088 · 확장 모드면 4095 상한 없음 (arm_tests/fw_ext 펌웨어 필요)
        import arm_map
        zs, ext = arm_map.zeros(), arm_map.extended()
        pad = 2 * 4096 / 360
        lims = [(-3.14159, 3.14159), (-1.5, 1.5), (-1.5, 1.4), (-1.7, 1.97), (-1.5708, 1.5708)]
        self.ros_rng = []
        for k, (lo, hi) in enumerate(lims):
            a_, b_ = zs[k] + lo * 4096 / (2 * math.pi) + pad, zs[k] + hi * 4096 / (2 * math.pi) - pad
            if not ext[k]:                                               # 일반 위치 모드: 펌웨어가 목표를 0 ~ 4095 로 자름
                a_, b_ = max(a_, 0 + pad), min(b_, 4095 - 10)
            self.ros_rng.append([a_, b_])
        # 11-01 사용자: 손으로 L7 · L8 에 넣어 보니 선 · 기구에 지장 없음 → '더 크게 풀어 줘'. 관절 범위 = 팀 ROS · URDF 범위(끝 2° 여유) 만.
        #   예전 CAD 모델 한계(J4 52° = 2430 틱 등) 는 IK 탐색 범위로만 넓힘 (충돌은 그대로 이 검사가 봄)
        self.tick_rng = [list(r) for r in self.ros_rng]
        for j in range(3):
            self.M.lo[j] -= math.radians(30)
            self.M.hi[j] += math.radians(30)
        # J4 는 CAD 한계 그대로 (위 −5°, 2430 틱): 11-01 12:15 넓혔더니 L1 칸 위에서 J4 2495 틱 · 전류 1.06 A → 8 s 뒤 과부하 멈춤 (기구 끝에 닿음)
        self.tick_rng[3] = [max(self.ros_rng[3][0], j4[0]), min(self.ros_rng[3][1], j4[1])]
        rest_bad = self.M.clearance(*self._qf(REST), {}, [], self_pairs=SELF_PAIRS)
        self.rest_close = {k for k, d in rest_bad.items() if d < MARGIN}
        self.rack_ok_stand = False                                          # 세우기 구간도 적재함 검사함 (팔은 가운데 통로)      # 받침 자세에서 이미 가까운 자기 쌍 (실물 = 접어 둔 상태)

    def _qf(self, ticks, grip=3504):
        q, gap, fq = self.q_of(ticks, grip)
        return q, fq

    def q_of(self, ticks, grip=3504, off=None):
        t = {'11': ticks[0], '12': ticks[1], '13': ticks[2], '14': ticks[3], '16': ticks[4], '15': grip}
        q, gap, fq = fk.ticks_to_q(t)
        for j, o in (off or {}).items():
            q[j] += math.radians(o)
        return q, gap, fq

    def pose(self, ticks, grip=3504, allow_stand=False, rack_zone=True, hyps=True, j4_hi=None):
        """관절 기준 가설(HYPS) 모두에서 검사 — 하나라도 걸리면 문제. 반환: 문제 목록, 최소 여유"""
        key = None
        if _CACHE_ON:
            key = (tuple(int(round(t)) for t in ticks), int(grip), bool(allow_stand), bool(rack_zone), bool(hyps), self.spacing, _obs_key(), j4_hi)
            if key in _CACHE:
                bad, worst = _CACHE[key]
                return list(bad), worst
        bad, worst = self._pose_all(ticks, grip, allow_stand, rack_zone, hyps, j4_hi)
        if key is not None and all(float(t).is_integer() for t in ticks):   # 정수 틱만 저장 (반올림한 키와 정확히 같은 자세)
            _CACHE[key] = _CACHE_NEW[key] = (tuple(bad), worst)
        return bad, worst

    def _pose_all(self, ticks, grip, allow_stand, rack_zone, hyps, j4_hi=None):
        bad_all, worst_all = [], 1.0
        for lab, off in (HYPS[:1] if allow_stand or not hyps else HYPS):          # 세우기 · 복귀 구간 = 실물 검증 순서 → 원래 모델만 (가설을 넣으면 받침 자세 자체가 관통으로 나옴)
            bad, worst = self._pose1(ticks, grip, allow_stand, off, rack_zone, j4_hi)
            bad_all += [f'[{lab}] {b}' if lab else b for b in bad if (f'[{lab}] {b}' if lab else b) not in bad_all]
            worst_all = min(worst_all, worst)
        return bad_all, worst_all

    def _pose1(self, ticks, grip, allow_stand, off, rack_zone=True, j4_hi=None):
        """문제 목록 (빈 목록 = 통과) 과 최소 여유"""
        bad = []
        for k, (lo, hi) in enumerate(self.tick_rng):
            if k == 3 and j4_hi:                                          # 시연 재생 구간만 J4 상한을 올림 (11-01 L7 · L8 꺼내기 — 기구 끝 약 2485 틱)
                hi = j4_hi
            if not lo <= ticks[k] <= hi:
                bad.append(f'J{k + 1} 틱 {ticks[k]} 범위 밖 ({lo:.0f}~{hi:.0f})')
            rlo, rhi = self.ros_rng[k]
            if not rlo <= ticks[k] <= rhi:
                bad.append(f'J{k + 1} 틱 {ticks[k]} ROS 범위 밖 ({rlo:.0f}~{rhi:.0f})')
        q, gap, fq = self.q_of(ticks, grip, off)
        cl = self.M.clearance(q, fq, self.mesh_obs, EXTRA_OBS, self_pairs=SELF_PAIRS)
        worst = 1.0
        for (a, b), d in cl.items():
            if (a, b) in ADJACENT or (b, a) in ADJACENT:            # 붙어 있는 이웃 (link1 ↔ J1 몸통 등) 은 원래 0
                continue
            if allow_stand and ((a, b) in self.rest_close or b in STATIC_OK_NEAR_REST):
                continue
            if b == 'plate_below':
                bad.append(f'{a} 상판 아래 관통 {d * 1000:.0f} mm'); continue
            mg = PLACED_PUCK_MARGIN if str(b).startswith('cup') and str(b).endswith(' puck') else \
                next((v for k_, v in OBS_MARGIN.items() if str(b).startswith(k_)), None)
            if j4_hi and str(b).startswith('cup') and mg is not None:       # 사람 시연 재생 구간: 실물로 안 닿은 자세라 컵 여유 3 mm (11-01 L7 쥐기 · 꺼내기)
                mg = min(mg, 0.003)
            if d < (mg if mg is not None else (STAND_MARGIN if allow_stand else MARGIN)):
                bad.append(f'{a} ↔ {b} {d * 1000:.0f} mm')
            worst = min(worst, d)
        Ts = self.M.fk(q, fq)
        zmin = 1.0
        for n, body in self.M.bodies.items():
            P = apply(Ts[self.M.link_of[n]], body.pts)
            zmin = min(zmin, P[:, 2].min() + BASE_Z)
            if n in RACK_LINKS and rack_zone:                         # rack_zone False = 적재 자세: 사진 어림 영역 대신 실측 컵 장애물(cup*) 로 검사
                mg = RACK_XY_MARGIN_STAND if allow_stand else RACK_XY_MARGIN
                for x0, x1, y0, y1 in RACK_ZONES:
                    x0, x1, y0, y1 = x0 - mg, x1 + mg, y0 - mg, y1 + mg
                    over = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
                    top = RACK_TOP_STAND if allow_stand else RACK_TOP
                    if over.any() and P[over, 2].min() < top:
                        bad.append(f'{n} 적재함 위 (상판 위 높이 {(P[over, 2].min() - PLATE_TOP_Z) * 1000:.0f} mm < {(top - PLATE_TOP_Z) * 1000:.0f})'); break
        if zmin < FLOOR_MARGIN:
            bad.append(f'바닥 위 {zmin * 1000:.0f} mm')
        return bad, worst

    def path(self, a, b, grip=3504, step_deg=2.0, allow_stand_ends=False, stand_a=False, stand_b=False, rack_zone=True, j4_hi=None):
        """stand_a / stand_b: 그 끝이 세우기 구간(P0 등) — 그 끝에서 4 칸(약 8°) 안은 원래 모델만 (P0 는 접힌 자세라 가설을 넣으면 자기 쌍이 늘 가까움)"""
        a, b = np.array(a, float), np.array(b, float)
        n = max(2, int(np.abs(b - a).max() * 360 / 4096 / step_deg) + 1)
        for i in range(1, n):
            t = (a + (b - a) * i / n).round().astype(int).tolist()
            near = (stand_a and i <= 4) or (stand_b and n - i <= 4)
            bad, _ = self.pose(t, grip, allow_stand=allow_stand_ends, rack_zone=rack_zone, j4_hi=j4_hi)
            if bad and near and not allow_stand_ends:
                bad = [x for x in bad if not x.startswith('[')]          # 가설 결과는 빼고 원래 모델만
            if bad:
                return f'{i}/{n} 지점 {t}: ' + '; '.join(bad[:3])
        return None


def set_obstacles(obs):
    from reach_lib import Analytic, make_T
    EXTRA_OBS.clear()
    for o in obs:
        EXTRA_OBS.append(Analytic(o['name'], o['kind'], make_T(tuple(o['xyz'])), o['dims']))


GRIP_JM = {'closed': 3504, 'open': 1755, 'held': 2245, 'release': 2098}   # 검사용 그리퍼 틱 (joint_map 간격 표: 닫힘 16 · 열림 약 50 · 네모 퍽 쥠 약 30 · 놓기 약 36 mm)


def full_sequence(poses, start_here=False, end_here=False):
    """실행기가 그대로 따라갈 전체 순서: 받침 → 세우기 경유점 → 자세들 → P0 → 세우기 역순 → 받침.
    (이름, 틱, 세우기 구간?, 멈춤?, enc, 덧붙임{grip · held_kg}) — 자세 파일의 'stop' (기본 true) · 'grip' (open | close, 도착 뒤) · 'held_kg' · 'speed' (그 자세로 가는 최고 °/s) 그대로"""
    # start_here · end_here (10-31, 두 번에 나눈 적재 load_cycle.sh): 받침 세우기 없이 첫 자세(지금 자세 = 들기) 에서 시작 · 마지막 자세에서 끝 (복귀 없음)
    seq = [] if start_here else [('받침', REST, True, False, None, {})] + [(n, t, True, False, None, {}) for n, t in STANDUP]
    seq += [(p['name'], [int(x) for x in p['ticks']], False, p.get('stop', True), p.get('enc'),
             {k: p[k] for k in ('grip', 'held_kg', 'speed', 'rack', 'j4_hi') if k in p}) for p in poses]
    if not end_here:
        seq += [('P0_복귀', STANDUP[-1][1], True, False, None, {})] + [(n + '_복귀', t, True, False, None, {}) for n, t in RETURN[:-1]] + [('받침_복귀', RETURN[-1][1], True, False, None, {})]
    return seq


def check_file(C, poses, stamp_path=None, start_here=False, end_here=False):
    """전체 순서 검사. 통과하면 stamp_path 에 순서 · 해시 저장 (실행기는 이 파일만 따라감).
    그리퍼 상태(닫힘 · 열림 · 쥠) 를 순서대로 따라가며 그 손가락 간격으로 검사 — 자세에 도착한 뒤 grip 이 바뀌므로 그 자세 자체는 두 상태 모두."""
    import hashlib
    ok = True
    seq = full_sequence(poses, start_here, end_here)
    print(f"{'자세':10s} {'틱 J1..J5':32s} 결과")
    g0 = 'held' if start_here and poses and poses[0].get('held_kg') else 'closed'   # 퍽을 쥔 채 시작 (넣기 실행)
    # 경로 검사(가장 오래 걸림, 34 s)를 먼저 여러 스레드로 (11-01: 구간끼리 독립 · Open3D 거리 계산은 스레드 병렬. NT_SAFETY_THREADS=1 이면 예전처럼 차례로)
    jobs, gq, pv = [], g0, None
    for name, t, std, stop, enc, ex in seq:
        if pv is not None:
            jobs.append((pv[1], t, GRIP_JM[pv[3]], (pv[2] and std), pv[2], std, not (ex.get('rack') or pv[4]), max(ex.get('j4_hi') or 0, pv[5] or 0) or None))
        if ex.get('grip'):
            gq = ex['grip'] if ex['grip'] in ('open', 'release') else ('held' if ex.get('held_kg') or name.startswith('grasp') else 'closed')
        pv = (name, t, std, gq, bool(ex.get('rack')), ex.get('j4_hi'))
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=max(1, int(os.environ.get('NT_SAFETY_THREADS', '6')))) as ex_:
        path_res = list(ex_.map(lambda j: C.path(j[0], j[1], grip=j[2], allow_stand_ends=j[3], stand_a=j[4], stand_b=j[5], rack_zone=j[6], j4_hi=j[7]), jobs))
    prev = None
    g = g0
    for name, t, std, stop, enc, ex in seq:
        gs = [g]
        if ex.get('grip'):
            g = ex['grip'] if ex['grip'] in ('open', 'release') else ('held' if ex.get('held_kg') or name.startswith('grasp') else 'closed')
            gs.append(g)
        for gi, gg in enumerate(dict.fromkeys(gs)):
            bad, worst = C.pose(t, grip=GRIP_JM[gg], allow_stand=std, rack_zone=not ex.get('rack'), j4_hi=ex.get('j4_hi'))
            msg = '통과 (여유 %.0f mm)' % (worst * 1000) if not bad else '✗ ' + '; '.join(bad)
            tag = f" [{ {'closed': '닫힘', 'open': '열림', 'held': '쥠', 'release': '놓기'}[gg] }]" if len(gs) > 1 or gg != 'closed' else ''
            print(f"{name if gi == 0 else '':10s} {str(t):32s} {msg}{'  [멈춤]' if stop and gi == 0 else ''}{tag}")
            ok &= not bad
        if enc is not None:                                               # 처짐 앞먹임 명령 자세 (실제 팔은 t 근처) 도 검사
            # 명령 자세 = 처짐이 없다고 본 자리 (실제 팔은 t 근처) → 처짐 가설(±2°) 없이 원래 모델만. t 자체는 위에서 가설까지 검사 (11-01: 둘 다 얹으면 이중으로 셈)
            bad2, w2 = C.pose(enc, grip=GRIP_JM[g], allow_stand=std, rack_zone=not ex.get('rack'), hyps=False, j4_hi=ex.get('j4_hi'))
            print(f"{'  (명령)':10s} {str(enc):32s} {'통과 (여유 %.0f mm)' % (w2 * 1000) if not bad2 else '✗ ' + '; '.join(bad2)}")
            ok &= not bad2
        if prev is not None:
            pb = path_res.pop(0)
            if pb:
                print(f'   └ {prev[0]} → {name} 경로 ✗ {pb}'); ok = False
        prev = (name, t, std, g, bool(ex.get('rack')))
    print('\n전체:', '통과' if ok else '✗ 실행 금지 — 위 항목 고칠 것')
    if ok and stamp_path:
        import sag_model
        # pgap = 명령에 더할 모터 위치 오차 앞먹임 (P 게인만인 팀 ROS 경로, sag_model.pgap_ticks) — 실제 목표 자세(t) 의 중력 토크로 (쥔 퍽 무게 포함)
        body = [dict(name=n, ticks=t, stand=s_, stop=st, pgap=sag_model.pgap_ticks(C.M, *C._qf(t), held_kg=ex.get('held_kg', 0.0)),
                     **({'enc': e} if e is not None else {}), **ex)
                for n, t, s_, st, e, ex in seq]
        if EXTRA_OBS:
            body[0]['obstacles'] = [dict(name=o.name, kind=o.kind, xyz=o.T[:3, 3].round(4).tolist(), dims=o.dims.round(4).tolist()) for o in EXTRA_OBS]
        h = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
        flags = {k: True for k, v in (('start_here', start_here), ('end_here', end_here)) if v}   # 실행기가 받침 시작 · 끝 규칙을 건너뛰는 표시 (load_cycle 두 단계)
        Path(stamp_path).write_text(json.dumps(dict(checked_by='pose_safety.py', sha256=h, sequence=body, **flags,
                                                    note='세우기 구간(stand) 은 받침대 · 적재함 여유를 줄여 검사 (J1 · J5 고정 구간만). 실행기는 이 순서만 따름'),
                                               ensure_ascii=False, indent=1))
        print('실행 순서 저장', stamp_path, h[:12])
    return ok


# ---------- 손눈 보정 자세 만들기 ----------
def calib_poses(C, n_pick=10):
    import handeye_floor as he
    rig = he.Rig()
    T5d = rig.prior[-22.0]                                               # 지금 추정 (광축 ↔ 손가락 약 22~24°)
    base = [4080, 1667, 1992, 2066, 1996]
    deg = lambda x: round(x * 4096 / 360)
    cands = []
    for d1 in (-35, -25, -15, -8, 0):                                   # J1: ROS 위치 모드에서 4095 위로 못 감 → 틱 감소 쪽만
        for d4 in (-25, -12, 0, 12, 20):
            for d5 in (-60, -30, 0, 30, 60):
                for d23 in ((0, 0), (-12, 15), (10, -12)):
                    cands.append([base[0] + deg(d1), base[1] + deg(d23[0]), base[2] + deg(d23[1]), base[3] + deg(d4), base[4] + deg(d5)])
    fx = 430.4
    rays = np.array([[(u - 424) / fx, (v - 240) / fx, 1.0] for u in (60, 424, 788) for v in (40, 240, 440)])
    good = []
    ch = C.M.static['chassis']
    for t in cands:
        q, gap, fq = C.q_of(t)
        T5 = C.M.fk(q, fq)['arm_link5_wrist_roll']
        Twd = T5 @ T5d
        n, d = he.predict(T5, T5d)
        tilt = math.degrees(math.acos(min(1, abs(n[2]))))
        if not (0.22 <= d <= 0.55 and tilt <= 45):
            continue
        R, p = Twd[:3, :3], Twd[:3, 3]
        hits = []
        okv = True
        for r in rays:
            w = R @ r
            if w[2] >= -0.05:
                okv = False; break
            s = (-BASE_Z - p[2]) / w[2]; hits.append(p + s * w)
        if not okv:
            continue
        hits = np.array(hits)
        inside = (hits[:, 0] > ch.lo[0] - 0.03) & (hits[:, 0] < ch.hi[0] + 0.03) & (hits[:, 1] > ch.lo[1] - 0.03) & (hits[:, 1] < ch.hi[1] + 0.03)
        if inside.sum() > 2:                                             # 화면에 차체가 많이 보이면 바닥 평면이 흐려짐
            continue
        bad, worst = C.pose(t)
        if bad:
            continue
        good.append(dict(ticks=t, T5=T5, d=d, tilt=tilt))
    print(f'후보 {len(cands)} 개 → 안전 · 바닥 보임 통과 {len(good)} 개')

    def jac(s):
        from scipy.spatial.transform import Rotation as Rot
        J = []
        for k in range(6):
            x = np.zeros(6); x[k] = 1e-4
            dT = he.T_of(Rot.from_rotvec(x[:3]).as_matrix(), x[3:])
            n1, d1 = he.predict(s['T5'], T5d @ dT)
            n0, d0 = he.predict(s['T5'], T5d)
            J.append(np.r_[(n1 - n0) / he.SIG_N, (d1 - d0) / he.SIG_D] / 1e-4)
        return np.array(J).T
    for s in good:
        s['J'] = jac(s)
    pick = [min(good, key=lambda s: sum(abs(a - b) for a, b in zip(s['ticks'], base)))]
    while len(pick) < min(n_pick, len(good)):
        A = sum(s['J'].T @ s['J'] for s in pick)
        best = max((s for s in good if s not in pick), key=lambda s: np.linalg.slogdet(A + s['J'].T @ s['J'] + 1e-6 * np.eye(6))[1])
        pick.append(best)
    A = sum(s['J'].T @ s['J'] for s in pick)
    std = np.sqrt(np.diag(np.linalg.inv(A)))
    print(f'고른 {len(pick)} 개 · 예상 불확실성(1σ) 회전 {np.degrees(std[:3]).round(2).tolist()}° · 위치 {(std[3:] * 1000).round(1).tolist()} mm')
    # 이동 순서: 가까운 것부터 (관절 틱 거리)
    order = [pick[0]]; rest = pick[1:]
    while rest:
        nxt = min(rest, key=lambda s: max(abs(a - b) for a, b in zip(s['ticks'], order[-1]['ticks'])))
        order.append(nxt); rest.remove(nxt)
    return [dict(name=f'c{i + 1}', ticks=s['ticks'], floor_mm=round(s['d'] * 1000), tilt_deg=round(s['tilt'], 1)) for i, s in enumerate(order)]


def kin_poses(C, n_pick=10):
    """어느 관절(J2 · J3 · J4) 기준이 틀렸는지 가리는 자세: 어깨 · 팔꿈치를 크게 바꾸며 바닥을 봄. 보정된 카메라(camera_calib.json) 로 바닥 보임 판정,
    D-최적(카메라 6 + 관절 오프셋 3) 으로 고름"""
    import handeye_floor as he
    from scipy.spatial.transform import Rotation as Rot
    cal = json.loads((HERE / 'camera_calib.json').read_text())
    T5d = np.array(cal['T_5d']); off0 = {1: cal['joint_offset_deg']['J2']}
    deg = lambda x: round(x * 4096 / 360)
    base = [3950, 1667, 1992, 2066, 1996]
    cands = [[base[0], base[1] + deg(a), base[2] + deg(b), base[3] + deg(c), base[4] + deg(e)]
             for a in (-26, -13, 0, 13, 26) for b in (-30, -15, 0, 15, 30) for c in (-25, -12, 0, 12, 25) for e in (-45, 0, 45)]
    ch = C.M.static['chassis']
    fx = 430.4
    rays = np.array([[(u - 424) / fx, (v - 240) / fx, 1.0] for u in (60, 424, 788) for v in (40, 240, 440)])

    def T5_of(t, extra=None):
        off = dict(off0)
        for j, o in (extra or {}).items():
            off[j] = off.get(j, 0) + o
        q, gap, fq = C.q_of(t, off=off)
        return C.M.fk(q, fq)['arm_link5_wrist_roll']
    good = []
    for t in cands:
        T5 = T5_of(t)
        n, d = he.predict(T5, T5d)
        tilt = math.degrees(math.acos(min(1, abs(n[2]))))
        if not (0.20 <= d <= 0.60 and tilt <= 45):
            continue
        Twd = T5 @ T5d; R, p = Twd[:3, :3], Twd[:3, 3]
        hits = []
        for r in rays:
            w = R @ r
            if w[2] >= -0.05:
                break
            hits.append(p + (-BASE_Z - p[2]) / w[2] * w)
        if len(hits) < len(rays):
            continue
        hits = np.array(hits)
        inside = (hits[:, 0] > ch.lo[0] - 0.03) & (hits[:, 0] < ch.hi[0] + 0.03) & (hits[:, 1] > ch.lo[1] - 0.03) & (hits[:, 1] < ch.hi[1] + 0.03)
        if inside.sum() > 2:
            continue
        good.append(dict(ticks=t, d=d, tilt=tilt))
    print(f'후보 {len(cands)} 개 → 바닥 보임 {len(good)} 개 → 안전 검사 (가설 4 개) 중 …')
    good = [g for g in good if not C.pose(g['ticks'])[0]]
    print(f'   안전 통과 {len(good)} 개')

    def jac(t):
        J = []
        n0, d0 = he.predict(T5_of(t), T5d)
        for k in range(9):
            if k < 6:
                x = np.zeros(6); x[k] = 1e-4
                n1, d1 = he.predict(T5_of(t), T5d @ he.T_of(Rot.from_rotvec(x[:3]).as_matrix(), x[3:])); h = 1e-4
            else:
                n1, d1 = he.predict(T5_of(t, {k - 5: 0.05}), T5d); h = math.radians(0.05)
            J.append(np.r_[(n1 - n0) / he.SIG_N, (d1 - d0) / he.SIG_D] / h)
        return np.array(J).T
    for g in good:
        g['J'] = jac(g['ticks'])
    pick = [min(good, key=lambda g: sum(abs(a - b) for a, b in zip(g['ticks'], base)))]
    while len(pick) < min(n_pick, len(good)):
        A = sum(g['J'].T @ g['J'] for g in pick)
        pick.append(max((g for g in good if g not in pick), key=lambda g: np.linalg.slogdet(A + g['J'].T @ g['J'] + 1e-6 * np.eye(9))[1]))
    A = sum(g['J'].T @ g['J'] for g in pick)
    std = np.sqrt(np.diag(np.linalg.inv(A)))
    print(f'고른 {len(pick)} 개 · 예상 1σ: 카메라 회전 {np.degrees(std[:3]).round(2).tolist()}° · 위치 {(std[3:6] * 1000).round(1).tolist()} mm · 관절 J2/J3/J4 오프셋 {np.degrees(std[6:]).round(2).tolist()}°')
    order = [pick[0]]; rest = pick[1:]
    while rest:
        nxt = min(rest, key=lambda g: max(abs(a - b) for a, b in zip(g['ticks'], order[-1]['ticks'])))
        order.append(nxt); rest.remove(nxt)
    return [dict(name=f'k{i + 1}', ticks=g['ticks'], floor_mm=round(g['d'] * 1000), tilt_deg=round(g['tilt'], 1)) for i, g in enumerate(order)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('poses', nargs='?')
    ap.add_argument('--calib', help='손눈 보정 자세를 만들어 이 파일로 저장 (검사까지)')
    ap.add_argument('--kin', help='관절 기준 가리기 자세를 만들어 이 파일로 저장 (검사까지)')
    ap.add_argument('--n', type=int, default=10)
    a = ap.parse_args()
    C = Checker()
    if a.calib:
        poses = calib_poses(C, a.n)
        Path(a.calib).write_text(json.dumps({'note': '손눈 보정 (바닥 깊이 평면) 자세 · pose_safety.py --calib 로 생성', 'poses': poses}, ensure_ascii=False, indent=1))
        print('저장', a.calib)
        check_file(C, poses, a.calib.replace('.json', '') + '.checked.json')
        return
    if a.kin:
        poses = kin_poses(C, a.n)
        Path(a.kin).write_text(json.dumps({'note': '관절 기준(J2 · J3 · J4) 가리기 + 손눈 보정 자세 · pose_safety.py --kin 로 생성', 'poses': poses}, ensure_ascii=False, indent=1))
        print('저장', a.kin)
        check_file(C, poses, a.kin.replace('.json', '') + '.checked.json')
        return
    if not a.poses:
        sys.exit(__doc__)
    d = json.loads(Path(a.poses).read_text())
    set_obstacles(d.get('obstacles', []))
    sys.exit(0 if check_file(C, d['poses'], a.poses.replace('.json', '') + '.checked.json', d.get('start_here', False), d.get('end_here', False)) else 1)


if __name__ == '__main__':
    main()
