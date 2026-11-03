#!/usr/bin/env python3
"""퍽 집어서 적재함 칸에 넣기 — 자세 만들기 (2026-10-30 저녁, 계산만 · 포트 안 씀). puck_locate.py --load <칸> 이 부름.

순서: 위 40 열기 → 10 → 쥘 자리(윗면 − 25 mm) 닫기[멈춤] → 10 → 들기 40 → (필요하면 경유점: 칸 옆 한 점, 또는 적재함 앞에서 올려 들어가는 꺾은선)
      → 칸 위 30 → 칸 안(퍽 바닥이 칸 바닥 6 mm 위) 놓기 36 mm[멈춤] → 칸 위 30 → 닫기 → (경유점 거꾸로) → 받침.
손은 수직으로 쥠 (15° 기울여 쥐면 적재함 앞 열 A · B 가 관절 한계로 안 됨 — 10-30 계산). 네모 퍽은 닫힘축 = 퍽 면 법선 → 칸 벽과 나란하게 넣음.
손목 방향(닫힘축 x · y) 은 IK 갈래로 나옴 — 손가락이 옆 칸 컵 · 든 퍽에 6 mm 안 걸리는 갈래를 고름 (C 칸은 D 가 찬 칸이라 어느 갈래도 안 됨, 10-31).
칸 위치 = rack_cells.json (오른쪽 2 × 2 실측). 검사: pose_safety (팔 · 손가락, 적재 자세는 실측 컵 장애물 여유 6 mm) +
  여기서 쥔 퍽(30 × 30 × 60) 바닥이 이동 내내 박스 · 상판 · 컵 윗면 · 든 퍽 위 5 mm 이상인지 (pose_safety 모델엔 쥔 퍽이 없음).
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import json
import math
import sys
from contextvars import ContextVar  # L7/L8 설정을 호출 단위로 분리한다.
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import grasp_test as gt  # noqa: E402
import pose_safety as ps  # noqa: E402
import sag_model  # noqa: E402

PUCK_H, PUCK_HALF, GRIP_DEPTH = 0.060, 0.015, 0.025
CUP_HALF = 0.0205
PUCK_SLOP = 0.002                 # 컵 안 퍽이 가운데에서 벗어난 만큼. 최대 (안쪽 약 37.8 − 퍽 30) / 2 = 4 mm 이지만 우리가 놓은 퍽은 칸 안 도착 4 틱 · 놓기 한쪽 3.5 mm 라 거의 가운데 → 2 mm (10-31 실물 A)
CLEAR = 0.005                     # 쥔 퍽 바닥 ↔ 아래 장애물(상판 · 박스) 높이 여유
CLEAR_CUP = 0.015                 # 컵 · 찬 칸 퍽 위는 더 (11-01 실물 B: 5 mm 로 A · D 퍽 위를 지나다 닿음 — 쥔 자리 치우침 · 처짐 오차)
OUT_DZ = 0.005       # 놓은 뒤 그리퍼 닫는 높이 = 칸 위 + 5 mm (11-01)
APPROACH_DZ = 0.030  # 칸 위 대기 = 놓는 높이 + 30 mm (35 면 C 칸 J4 가 한계 2430 에 붙어 처짐 앞먹임 명령이 넘음, 10-31) — 퍽 바닥이 컵 윗면 + 8 mm
PLACE_GAP = 0.010   # 컵 바닥 두께 모름 (상판 실측 0.094 기준) → 10 mm 위에서 놓음. 11-01 L3: 6 mm 에서 두 번 J3 26 틱 덜 내려가 멈춤 = 실제 쥔 깊이가 얕아(약 20 mm) 퍽이 칸 바닥에 닿음
BOX = dict(xy=(0.2775, 0.0), half=(0.1025, 0.1025), top=-0.002)        # 시작 박스 (grasp_test 와 같음)
CHASSIS = dict(xy=(0.0, 0.0), half=(0.141, 0.133), top=0.094)            # 상판 (깊이 실측 약 0.094)


# 현행 J4 복구 모델의 통과 설정만 제안한다. 원본 숫자·안전 기준은 바꾸지 않는다.
# 계산 검토용: 방금 놓은 target 퍽의 퇴출 보충검사가 실패하므로 실물 실행용 완성안이 아니다.
# 기준 스냅숏 SHA-256: 997b67d3190675e428063b1a1c2ec45bf003307963573a135ab27de75f04e203
_RACK_PROFILE = ContextVar('rack_load_l78_profile', default=None)
_L78_PROFILES = {('L7', False): {'approach_dz': 0.02,
                 'grip_depth': 0.035,
                 'held_hyps_required': True,
                 'place_gap': 0.02,
                 'preferred_j5': 2684,
                 'vias': [[4302, 1664, 2038, 1872, 2228], [5858, 1965, 1675, 2205, 2134]]},
 ('L7', True): {'approach_dz': 0.02,
                'grip_depth': 0.04,
                'held_hyps_required': True,
                'place_gap': 0.025,
                'preferred_j5': 2684,
                'vias': [[4302, 1664, 2038, 1872, 2228],
                         [5858, 1935, 1675, 2205, 2134],
                         [5832, 2053, 1569, 2425, 2722]]},
 ('L8', False): {'approach_dz': 0.01,
                 'exit_slide': (0.0, -1.0),   # 11-01: 놓은 뒤 손끝이 퍽 윗면보다 4.8 mm 낮아 그 자리에서 못 닫음 → 연 채 L7(빈 칸) 쪽으로 손가락 면 따라 미끄러진 뒤 닫음
                 'grip_depth': 0.035,
                 'held_hyps_required': True,
                 'place_gap': 0.02,
                 'preferred_j5': 2550,
                 'vias': [[4304, 1688, 1782, 2044, 2011], [5242, 1386, 1815, 2135, 2293]]},
 ('L8', True): {'approach_dz': 0.01,
                'grip_depth': 0.035,
                'held_hyps_required': True,
                'place_gap': 0.02,
                'preferred_j5': 2550,
                'vias': [[4304, 1688, 1782, 2044, 2011], [5242, 1386, 1815, 2135, 2293]]}}

def _rack_value(name, default):
    profile = _RACK_PROFILE.get()
    return default if profile is None else profile[name]  # 기존 칸에는 원래 상수를 쓴다.


# 보정용 (10-31): NT_RACK_HOVER_STOP=1 이면 칸 위(over_cell, 퍽 바닥이 컵 테두리 위 약 8 mm) 에서 퍽을 든 채 멈춤 → 위에서 퍽 네 변 ↔ 컵 안쪽 벽 틈을 줄자로
#   (걸친 뒤에 재면 퍽이 기울어 손 위치가 안 나옴). Enter 면 그대로 내려가 놓음
import os  # noqa: E402
HOVER_STOP = os.environ.get('NT_RACK_HOVER_STOP') == '1'

def load_cells():
    return json.loads((HERE / 'rack_cells.json').read_text())


def cell_yaw(rc, cell):
    """base_link 기준 칸 방향 [도]. 개별 실측이 없을 때만 적재함 공통값 사용."""
    yaw = float(cell['yaw_deg'] if 'yaw_deg' in cell else rc['yaw_deg'])
    if not math.isfinite(yaw):
        raise ValueError('칸 yaw_deg 는 유한한 수여야 함')
    return yaw


def cell_footprints(rc, cell):
    """컵 · 찬 칸 퍽의 보수적 xy 반너비. 퍽 중심 밀림은 칸 축마다 2 mm 포함."""
    # 컵은 기울기만큼 키우지 않음 (10-31 17:46 실물: 칸별 스캔으로 C–D 간격 38 mm 로 잼 → 6° 감싸는 상자면 두 컵이 겹쳐 C 로 내리는 길이 막힘.
    #   모서리는 최대 CUP_HALF·(cos+sin−1) ≈ 2 mm 덜 잡히지만 위아래 CLEAR 5 mm · 손가락 여유 6 mm 가 그대로). 찬 칸의 퍽은 감싸는 상자 그대로
    # 찬 칸의 퍽도 기울기만큼 키우지 않음: 우리가 칸 방향(칸별 yaw)으로 손을 돌려 놓으므로 퍽 면이 컵 벽과 나란 (10-31 19:2x: D 가 C 퍽과 1 mm 모자라 막힘)
    return CUP_HALF, PUCK_HALF + PUCK_SLOP


def puck_corners(M, q):
    """쥔 퍽 8 꼭짓점 (base_link). 손끝은 퍽 축 위, 활성 설정의 깊이만큼 윗면 아래."""
    tip, app, close, _ = M.grip(q)
    side = np.cross(app, close)
    c = tip + app * (PUCK_H / 2 - _rack_value('grip_depth', GRIP_DEPTH))                            # 퍽 가운데 (app = 아래쪽)
    return np.array([c + sa * PUCK_HALF * close + sb * PUCK_HALF * side + sc * PUCK_H / 2 * app
                     for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)])


def obstacles(rc, target):
    obs = [dict(name='box', **BOX), dict(name='plate', **CHASSIS)]
    for n, c in rc['cells'].items():
        full = c['state'].startswith('full')
        if n == target:
            continue
        cup_half, puck_half = cell_footprints(rc, c)
        obs.append(dict(name=f'cup {n}', xy=tuple(c['xy']), half=(cup_half,) * 2, top=rc['rim_z']))
        if full:                                                          # 찬 칸 = 컵(테두리까지) + 그 위로 솟은 퍽 (컵 크기 상자로 보면 옆 칸에서 5 mm 손해)
            obs.append(dict(name=f'cup {n} puck', xy=tuple(c['xy']), half=(puck_half,) * 2, top=rc['floor_z'] + PUCK_H))
    return obs


def puck_hits(M, q, obs, skip=()):
    P = puck_corners(M, q)
    lo, hi = P[:, :2].min(0), P[:, :2].max(0)
    zmin = P[:, 2].min()
    for o in obs:
        if o['name'] in skip:
            continue
        ox, oy = o['xy']; hx, hy = o['half']
        if hi[0] < ox - hx or lo[0] > ox + hx or hi[1] < oy - hy or lo[1] > oy + hy:
            continue
        clear = CLEAR_CUP if o['name'].endswith(' puck') else CLEAR   # 찬 칸에서 솟은 퍽 위만 15 mm, 빈 컵 테두리 · 상판은 5 mm (11-01: 빈 컵까지 15 mm 면 왼쪽 L3~L6 길 없음)
        if zmin < o['top'] + clear:
            return f"쥔 퍽 바닥 {zmin * 1000:.0f} mm < {o['name']} 윗면 {o['top'] * 1000:.0f} + {clear * 1000:.0f}"
    return None


def path_hits(C, a, b, obs, skip=(), step_deg=1.5):
    a, b = np.array(a, float), np.array(b, float)
    n = max(2, int(np.abs(b - a).max() * 360 / 4096 / step_deg) + 1)
    profile = _RACK_PROFILE.get()  # L7/L8만 추가 검사하며 기존 칸의 판정은 유지한다.
    for i in range(n + 1):
        t = (a + (b - a) * i / n).round().astype(int).tolist()
        q, fq = C._qf(t, grip=ps.GRIP_JM['held'])
        h = puck_hits(C.M, q, obs, skip)
        if h:
            return f'{i}/{n} {t}: {h}'
        if profile is not None and profile['held_hyps_required']:
            for label, off in ps.HYPS:  # 선정 시 요구한 경우에만 쥔 퍽도 같은 가설로 검사한다.
                qh, _, _ = C.q_of(t, grip=ps.GRIP_JM['held'], off=off)
                h = puck_hits(C.M, qh, obs, skip)
                if h:
                    return f'{i}/{n} {t} [{label or "nominal"}]: {h}'
    if profile is not None:
        for label, endpoint in (('start', a), ('end', b)):
            bad, _ = C.pose(endpoint.tolist(), grip=ps.GRIP_JM['held'], rack_zone=False)
            if bad:
                return f'{label} pose: ' + '; '.join(bad[:3])  # C.path가 제외하는 끝점도 검사한다.
        bad = C.path(a.tolist(), b.tolist(), grip=ps.GRIP_JM['held'], rack_zone=False)
        if bad:
            return 'arm path: ' + bad  # 퍽만 통과하고 팔/손가락은 걸리는 경유점을 배제한다.
    return None


def in_range(C, t):
    return all(max(C.tick_rng[k][0], C.ros_rng[k][0]) <= t[k] <= min(C.tick_rng[k][1], C.ros_rng[k][1]) for k in range(5))


def ik_vertical(C, p, yaw_deg, seed=None):
    """수직 손 IK, 닫힘축 = 칸 벽과 나란 (yaw_deg). reach_model 의 IK 롤 잔차가 90° 주기(정사각 퍽) 라 닫힘축 x · y · ±는 한 번의 IK 가 다 덮음
    (10-31: 네 방향을 따로 풀면 똑같은 해가 4 번 나와 4 배 느렸음). 반환 [(틱, q)] — 같은 틱은 하나만.
    씨앗 없으면 씨앗 63 개 (약 0.6 s), 있으면 그 하나 (약 1 ms)."""
    M = C.M
    app = np.array([0.0, 0.0, -1.0])
    y = math.radians(yaw_deg)
    close = np.array([math.cos(y), math.sin(y), 0.0])
    sols = M.ik(np.asarray(p, float), app, close, seed=seed) if seed is not None else M.ik(np.asarray(p, float), app, close)
    out, seen = [], set()
    for q, info in sols:
        if info['pos_mm'] < 1 and info['app_deg'] < 1 and info['roll_deg'] < 2:
            t = gt.q_to_ticks(q)
            if tuple(t) not in seen and in_range(C, t):
                seen.add(tuple(t)); out.append((t, q))
    return out


def _lazy(*fs):
    """fs 의 결과를 차례로 — 앞 것에서 끝나면 뒤 것(비싼 IK) 은 안 부름"""
    for f in fs:
        yield from f()


def _dense(C, t_from, pts, step):
    """들기 손끝 → 꺾은선 pts 를 step 간격으로 나눈 점들 (시작점 제외)"""
    p_prev = C.M.grip(C._qf(t_from, grip=ps.GRIP_JM['held'])[0])[0]
    dense = []
    for p in pts:
        p = np.asarray(p, float)
        n = max(1, int(math.ceil(np.linalg.norm(p - p_prev) / step)))
        dense += [p_prev + (p - p_prev) * i / n for i in range(1, n + 1)]
        p_prev = p
    return dense


def _next(C, pp, yaw_deg, seed, last, full=False):
    """점 pp 의 수직 손 IK 중 앞 자세 last 와 250 틱 안에서 이어지는 가장 가까운 해 — (틱, q) 또는 None.
    먼저 씨앗 IK (1 ms), 안 이어지면 full 일 때만 씨앗 없는 IK (0.6 s) 로 다른 갈래를 찾음."""
    for s in ((seed,) if not full else (seed, None)):
        sols = [s_ for s_ in ik_vertical(C, pp, yaw_deg, seed=s) if max(abs(u - v) for u, v in zip(s_[0], last)) <= 250]
        if sols:
            return min(sols, key=lambda s_: np.abs(np.array(s_[0]) - np.array(last)).max())
    return None


def _chain(C, t_from, t_to, pts, yaw_deg, step=0.02):
    """꺾은선 pts (손끝 위치들, 마지막 = 칸 위) 를 2 cm 간격으로 나눠 수직 손 IK (앞 해 씨앗). 반환 틱 목록 (마지막 = t_to) 또는 None
    (다른 꺾임으로 뛰면 끊음 — 끝에서 칸 위 자세 t_to 와도 250 틱 안에 이어져야 함: 10-31 전엔 이 검사가 없어 J5 가 90° 다른 갈래를 타고 와
    칸 바로 위에서 손목을 크게 돌리는 경로가 나왔음)"""
    seed, last, out = C._qf(t_from)[0], list(t_from), []
    for pp in _dense(C, t_from, pts, step)[:-1]:
        nx = _next(C, pp, yaw_deg, seed, last)
        if nx is None:
            return None
        out.append(nx[0]); seed, last = nx[1], nx[0]
    if max(abs(u - v) for u, v in zip(last, t_to)) > 250:
        return None
    return out + [list(t_to)]


def _chain_back(C, t_from, t_to, pts, yaw_deg, step=0.02):
    """_chain 과 같은 꺾은선을 칸 위(t_to)에서 거꾸로 풀기 (10-31): 칸 가까이는 IK 갈래가 하나뿐이라 (C 칸: J5 ≈ 2000~2500 쪽, J4 한계 근처)
    앞에서부터 가까운 해를 고르면 다른 갈래(J5 ≈ 1400~1600)를 탔다가 칸 앞 6 cm 에서 끊김. 반환 = _chain 과 같은 모양 (마지막 = 칸 위) 또는 None.
    들기 자세 → 첫 점은 손목 롤(J5) 이 크게 돌 수 있음 (그 구간도 path_hits · pose_safety 로 검사).
    씨앗 없는 IK 는 씨앗 IK 가 안 이어질 때만 (10-31: 점마다 늘 부르면 후보 하나에 6~8 s, 결과는 같음)."""
    dense = _dense(C, t_from, pts, step)
    seed, last, back = C._qf(t_to)[0], list(t_to), []
    for pp in reversed(dense[:-1]):
        nx = _next(C, pp, yaw_deg, seed, last, full=True)
        if nx is None:
            return None
        back.append(nx[0]); seed, last = nx[1], nx[0]
    return list(reversed(back)) + [list(t_to)]


def cart_route(C, t_from, t_to, p_to, yaw_deg, obs):
    """들기 → 칸 위 경유점 (쥔 퍽이 다른 칸 · 상판 위 5 mm 이상). 수직 손은 J4 한계 때문에 몸에서 멀면 높이 못 듦
    (10-30: 박스 근처 손끝 ≤ 0.12~0.14 m, 0.17 m 는 x ≤ 약 0.115) → '적재함 앞까지 낮게 → 앞에서 올림 → 그 높이로 칸 위' 꺾은선 후보를 차례로.
    반환 경유점 틱 목록 (칸 위 자세 제외) 또는 None"""
    profile = _RACK_PROFILE.get()
    if profile is not None:
        vias = [list(t) for t in profile['vias']]  # 선정 경유점을 현재 시작/칸 위 자세로 다시 검사한다.
        chain = [list(t_from)] + vias + [list(t_to)]
        for i, (a_, b_) in enumerate(zip(chain, chain[1:])):
            if path_hits(C, a_, b_, obs, skip=('box',) if i == 0 else ()):
                return None  # 저장된 틱도 현재 컵 상태와 안전 기준을 통과해야 한다.
        return vias
    M = C.M
    p0 = M.grip(C._qf(t_from, grip=ps.GRIP_JM['held'])[0])[0]
    p_to = np.asarray(p_to, float)
    cands = []
    # (나) 적재함 앞 x1 까지 z1 → x2 에서 칸 위 높이 → 칸 위. 상판 가장자리(x 0.141 + 퍽 반 0.015) 를 넘을 때 퍽 바닥(손끝 − 35 mm) 이
    #   상판 0.094 + 5 mm 위여야 하므로 z1 ≥ z_edge = 0.134 (10-31: 들기 + 0.03 = 0.128 은 1 mm 모자라 모든 후보가 걸렸음). 높은 z1 · 바깥 x1 먼저.
    z_edge = CHASSIS['top'] + CLEAR + PUCK_H - _rack_value('grip_depth', GRIP_DEPTH)
    for z1 in (z_edge + 0.01, z_edge + 0.025, z_edge):
        for x1 in (0.16, 0.14, 0.13):
            for x2 in (0.115, 0.125, 0.105):
                y1 = p0[1] + (p_to[1] - p0[1]) * 0.5
                cands.append([np.array([x1, y1, max(z1, p0[2])]), np.array([x2, p_to[1], p_to[2]])])
    cands.append([np.array([p0[0], p0[1], max(p0[2], p_to[2])])])         # (가) 그 자리에서 올려 수평 — 수직 손은 몸에서 멀면 거의 안 됨 → 맨 뒤
    for key in cands:
        ticks = _chain(C, t_from, t_to, key + [p_to], yaw_deg) or _chain_back(C, t_from, t_to, key + [p_to], yaw_deg)
        if ticks is None:
            continue
        chain = [t_from] + ticks[:-1] + [t_to]
        sk = lambda i: ('box',) if i == 0 else ()
        if any(path_hits(C, a_, b_, obs, skip=sk(i)) for i, (a_, b_) in enumerate(zip(chain, chain[1:]))):
            continue
        keep, i = [chain[0]], 0                                            # 경유점 줄이기 (건너뛰어도 통과하면 뺌)
        while i < len(chain) - 1:
            j = len(chain) - 1
            while j > i + 1 and path_hits(C, chain[i], chain[j], obs, skip=sk(i)):
                j -= 1
            keep.append(chain[j]); i = j
        return keep[1:-1]
    return None


def cell_fingers(C, ta, tp, rc):
    """칸 위(쥠) · 칸 안(쥠 · 놓기) · 내리는 길을 pose_safety 로 실측 컵 장애물(여유 6 mm) 에 대해 검사 — 손가락이 옆 칸 컵 · 든 퍽에 닿는지.
    반환 (걸림 설명 또는 None, 최소 여유 m). 전역 EXTRA_OBS 는 쓰고 되돌림."""
    saved = list(ps.EXTRA_OBS)
    ps.set_obstacles(cup_obstacles(rc))
    try:
        worst, why = 1.0, None
        for t, g in ((ta, 'held'), (tp, 'held'), (tp, 'release')):
            bad, w = C.pose(t, grip=ps.GRIP_JM[g], rack_zone=False)
            worst = min(worst, w)
            if bad and why is None:
                why = f'{t} [{g}] ' + '; '.join(bad[:2])
        pb = why or C.path(ta, tp, grip=ps.GRIP_JM['held'], rack_zone=False)
        return (why or (f'내리는 길 {pb}' if pb else None)), worst
    finally:
        ps.EXTRA_OBS[:] = saved


def shifted(rc, cell):
    """칸 넣기 자세 보정 (place_calib.py, 10-31): 그 칸의 place_offset 만큼 적재함 전체를 옮긴 사본 — 그 자세에서 계산(처짐 · FK) 이 어긋난 만큼
    모델 안의 컵들을 같이 옮겨 IK · 안전 검사 → 실제 팔은 스캔한 자리로 감. 보정이 없으면 그대로"""
    off = rc['cells'].get(cell, {}).get('place_offset')
    if not off:
        return rc
    rc = json.loads(json.dumps(rc))
    for c in rc['cells'].values():
        c['xy'] = [c['xy'][0] + off[0], c['xy'][1] + off[1]]
    return rc


def _plan_rack_impl(C, cell, grasp_lift_ticks, rc=None):
    """들기 자세 → 칸 넣기 자세들. 반환 (poses dict 목록, 설명) — 실패면 예외"""
    rc = shifted(rc or load_cells(), cell)
    c = rc['cells'][cell]
    if c['state'].startswith('full'):
        raise SystemExit(f'칸 {cell} 은 찬 칸 ({c["state"]})')
    x, y = c['xy']
    yaw_deg = cell_yaw(rc, c)
    z_place = rc['floor_z'] + _rack_value('place_gap', PLACE_GAP) + PUCK_H - _rack_value('grip_depth', GRIP_DEPTH)          # 퍽 바닥이 칸 바닥(상판) 위 PLACE_GAP 에서 놓음
    z_above = z_place + _rack_value('approach_dz', APPROACH_DZ)  # L7/L8 접근 높이만 개별 적용한다.
    obs = obstacles(rc, cell)
    # 칸 위 · 칸 안 한 쌍을 IK 갈래마다 (갈래 = 손목 방향: 손끝이 J5 축에서 약 12 mm 옆에 있어 닫힘축을 90° 돌리면 J1 도 같이 달라짐 — J5 만 ±1024
    #   돌리면 손끝이 17 mm 옮겨져 퍽이 칸 가운데를 벗어남, 10-31). 손가락이 옆 칸 컵 · 든 퍽에 안 걸리는 갈래만, 손목 덜 도는 순서.
    cands, fails = [], []
    profile = _RACK_PROFILE.get()
    for ta, qa in ik_vertical(C, (x, y, z_above), yaw_deg):
        if profile is not None and abs(ta[4] - profile['preferred_j5']) >= 100:
            continue  # 검증한 손목 갈래만 선택하며 관절 한계는 바꾸지 않는다.
        pl = ik_vertical(C, (x, y, z_place), yaw_deg, seed=qa)
        if not pl:
            continue
        tp = min(pl, key=lambda s_: np.abs(np.array(s_[0]) - np.array(ta)).max())[0]
        if max(abs(u - v) for u, v in zip(tp, ta)) > 300:
            continue
        why, worst = cell_fingers(C, ta, tp, rc)
        if why:
            fails.append(f'{why} (여유 {worst * 1000:.0f} mm)'); continue
        cands.append((abs(ta[4] - grasp_lift_ticks[4]), ta, qa, tp, worst))
    if not cands:
        raise SystemExit(f'칸 {cell}: ' + ('수직 손 IK 없음' if not fails else '손가락이 옆 칸 컵에 걸리는 손목 방향뿐 — ' + ' · '.join(fails)))
    # 들기에서 칸 위까지 쥔 퍽이 다른 칸 · 상판을 안 스치는 길: 곧장 → 칸 둘레 경유점 하나 → 꺾은선(cart_route)
    hit, route = None, None
    side = [(0.0, 0.0, dz) for dz in (0.03, 0.05, 0.07)] + \
           [(dx, dy, dz) for dz in (0.0, 0.02, 0.04) for dx, dy in   # 앞 셋: 칸 바로 위 높게 (11-01 꺼내기 C · D: 퍽 든 채 상판 모서리 1~5 mm 모자람 — 팔 가까운 칸은 위로 더 올라감)
            ((0.06, 0.0), (0.06, 0.03), (0.06, -0.03), (0.0, 0.05), (0.04, 0.05), (-0.04, 0.05), (0.09, 0.0), (0.0, -0.05),
                         (0.11, -0.04), (0.09, -0.06))]   # 마지막 둘: 11-01 L4 (박스 쪽 안쪽으로 비스듬히 — 바깥 줄 칸은 이 쪽으로만 상판을 넘음)
    if profile is not None:
        side = []  # L7/L8는 검증된 고정 경유점을 우선 사용한다.
    for _, ta, qa, tp, clear in sorted(cands, key=lambda c_: c_[0]):
        h = path_hits(C, grasp_lift_ticks, ta, obs, skip=('box',))
        if not h:
            route = (ta, tp, clear, []); break
        hit = hit or h
        found = None
        # 경유점 후보: 칸 둘레(앞 · 옆, 다른 칸이 없는 쪽부터) 같은 높이 또는 조금 위 → 거기서 수평으로 칸 위로
        #   IK 는 칸 위 해(qa) 씨앗으로 먼저 (1 ms), 그게 안 되면 씨앗 없이 (0.6 s, 다른 갈래)
        for dx, dy, dz in side:
            pv = (x + dx, y + dy, z_above + dz)
            for tv, qv in _lazy(lambda: ik_vertical(C, pv, yaw_deg, seed=qa), lambda: ik_vertical(C, pv, yaw_deg)):
                if path_hits(C, grasp_lift_ticks, tv, obs, skip=('box',)) or path_hits(C, tv, ta, obs):
                    continue
                found = tv
                break
            if found:
                break
        vias = [found] if found else cart_route(C, grasp_lift_ticks, ta, (x, y, z_above), yaw_deg, obs)
        if vias is not None:
            route = (ta, tp, clear, vias); break
    if route is None:
        # 11-01 L2 (L1 찬 뒤): 들기에서 곧장 가면 퍽이 상판 모서리를 2 mm 차로 스침 → 들기 자리에서 수직으로 먼저 올린 경유점 하나 (손목 덜 도는 해부터)
        tip0 = np.asarray(C.M.grip(C._qf(list(grasp_lift_ticks))[0])[0])
        for dz in (0.02, 0.03, 0.04, 0.06, 0.08):           # 11-01: 관절 범위를 ROS 범위로 넓힌 뒤 L7 은 더 높이 올려야 상판을 넘음
            ups = sorted(ik_vertical(C, tip0 + [0, 0, dz], yaw_deg), key=lambda s_: abs(s_[0][4] - grasp_lift_ticks[4]))
            for tu, _ in ups:
                if path_hits(C, grasp_lift_ticks, tu, obs, skip=('box',)):
                    continue
                for _, ta, qa, tp, clear in sorted(cands, key=lambda c_: c_[0]):
                    if not path_hits(C, tu, ta, obs, skip=('box',)):
                        route = (ta, tp, clear, [tu]); break
                if route:
                    break
            if route:
                break
    if route is None:
        raise SystemExit(f'칸 {cell}: 쥔 퍽이 지나갈 길 없음 (첫 걸림 {hit})')
    ta, tp, clear, vias = route
    h2 = path_hits(C, ta, tp, obs, skip=('plate',))                    # 칸 안으로 내릴 때 퍽 아래는 칸 바닥 (상판 판정 뺌)
    if h2:
        raise SystemExit(f'칸 {cell}: 내리는 길 ✗ {h2}')
    held = dict(held_kg=0.017)
    sl = dict(speed=15.0)

    def enc(t, kg):                                                       # 처짐 앞먹임 (엔코더가 가리켜야 할 틱) — grasp_test 와 같은 식
        q, fq = C._qf(t)
        return gt.q_to_ticks(sag_model.encoder_goal_q(C.M, q, fq, held_kg=kg))

    for t in vias + [ta, tp]:                                             # 처짐 앞먹임 명령 틱도 관절 범위 안이어야 (pose_safety 가 (명령) 자세도 검사)
        if not in_range(C, enc(t, 0.017)):
            raise SystemExit(f'칸 {cell}: 처짐 앞먹임 명령 {enc(t, 0.017)} (자세 {t}) 이 관절 범위 밖')
    # 놓은 뒤 그리퍼를 닫는 자리 = 칸 위보다 OUT_DZ 더 위 (11-01 실물: 칸 위에서 닫으면 손가락이 놓은 퍽에 닿음). 팔이 못 가면 칸 위 그대로
    to = ta
    for dz in (OUT_DZ, 0.004, 0.003):                                     # 5 mm 가 안 되면 4 · 3 mm (L3 는 4 mm 까지, 11-01)
        ups = [t_ for t_, _ in ik_vertical(C, (x, y, z_above + dz), yaw_deg, seed=qa) + ik_vertical(C, (x, y, z_above + dz), yaw_deg)
               if abs(t_[4] - ta[4]) < 200 and abs(t_[0] - ta[0]) < 200 and in_range(C, t_) and in_range(C, enc(t_, 0.0))]   # 손목 · 몸통 같은 갈래만
        ups = [t_ for t_ in ups if not path_hits(C, tp, t_, obstacles(rc, cell), skip=('plate', f'target cup {cell}'))]
        if ups:
            to = ups[0]; break
    poses = [dict(name=f'carry{i + 1}', ticks=v, enc=enc(v, 0.017), stop=False, rack=True, **held, speed=25.0) for i, v in enumerate(vias)]
    poses += [dict(name='over_cell', ticks=ta, enc=enc(ta, 0.017), stop=HOVER_STOP, rack=True, **held, speed=25.0),
              dict(name='in_cell', ticks=tp, enc=enc(tp, 0.017), stop=True, rack=True, grip='release', **held, **sl),   # 놓기 = 약 36 mm 만 (옆 칸 퍽에 안 닿게)
              dict(name='out_cell', ticks=to, enc=enc(to, 0.0), stop=False, rack=True, grip='close', **sl)]
    if profile is not None and profile.get('exit_slide'):
        # 연 채(놓기 간격) 손가락 면을 따라 옆으로 → 놓은 퍽에서 벗어난 뒤 닫음. 끝에서 닫을 때는 놓은 퍽까지 장애물로 검사 (11-01 L8)
        poses[-1].pop('grip')
        rc_full = json.loads(json.dumps(rc)); rc_full['cells'][cell]['state'] = 'full'
        saved = list(ps.EXTRA_OBS); ps.set_obstacles(cup_obstacles(rc_full, cell))
        try:
            tip0 = np.asarray(C.M.grip(C._qf(to)[0])[0]); d_ = np.r_[profile['exit_slide'], 0.0]
            slide = None
            for dist in (0.035, 0.040, 0.045):
                ln, seed, last = [], C._qf(to)[0], list(to)
                for k in range(1, int(round(dist / 0.005)) + 1):
                    nx = _next(C, tip0 + d_ * 0.005 * k, yaw_deg, seed, last)
                    if nx is None:
                        ln = None; break
                    ln.append(nx[0]); seed, last = nx[1], nx[0]
                if not ln:
                    continue
                for dz in (0.015, 0.010, 0.005, 0.0):                  # 옆으로 나온 뒤 더 올라가서 닫음 (11-01 L8 실물: 그 높이에서 닫을 때 살짝 걸림)
                    up = [ln[-1]]
                    tip1 = np.asarray(C.M.grip(C._qf(ln[-1])[0])[0]); sd, lt = C._qf(ln[-1])[0], list(ln[-1])
                    for k in range(1, int(round(dz / 0.005)) + 1):
                        nx = _next(C, tip1 + np.r_[0.0, 0.0, 0.005 * k], yaw_deg, sd, lt)
                        if nx is None:
                            up = None; break
                        up.append(nx[0]); sd, lt = nx[1], nx[0]
                    if up and not C.pose(up[-1], grip=ps.GRIP_JM['closed'], rack_zone=False)[0]:
                        slide = ln + up[1:]; break
                if slide:
                    break
        finally:
            ps.EXTRA_OBS[:] = saved
        if slide is None:
            raise SystemExit(f'칸 {cell}: 놓은 뒤 옆으로 빠져나와 닫을 자리 없음')
        poses += [dict(name=f'slide{i + 1}', ticks=v, enc=enc(v, 0.0), stop=False, rack=True, speed=10.0) for i, v in enumerate(slide)]
        poses[-1]['grip'] = 'close'
    # 나올 때는 들어온 경유점을 거꾸로 (칸에서 받침으로 곧장 가면 손가락이 팔 밑동(link1) 에 붙음 — C 칸 15 mm)
    poses += [dict(name=f'back{i + 1}', ticks=v, enc=enc(v, 0.0), stop=False, rack=True, speed=25.0) for i, v in enumerate(reversed(vias))]
    # 마지막으로 올 때의 들기 자세(박스 위)를 거쳐 받침으로 — 경유점에서 P0 로 곧장 가면 손가락이 팔 밑동(link1) 에 15 mm 안으로 붙는 칸이 있음
    #   (10-31 C 보정 뒤 [J3−2] 14 mm). 들기 → 칸 은 앞에서 검사한 길을 거꾸로, 들기 ↔ P0 도 집기 때 지나온 길
    # 그리퍼는 칸 위(out_cell) 에서 닫음 — 연 채(36 mm) 로 빠져나오면 칸이 다 찬 뒤 벌어진 손가락이 이웃 퍽에 걸림 (11-01 실물).
    #   out_cell 에서 손끝은 제대로 들어간 퍽 윗면보다 약 11 mm 위 → 다시 쥐지 않음 (테두리에 걸쳐 높이 뜬 퍽만 잡힐 수 있음 — 칸 위 멈춤으로 확인)
    poses.append(dict(name='back_lift', ticks=list(grasp_lift_ticks), enc=enc(list(grasp_lift_ticks), 0.0), stop=False, speed=25.0))
    return poses, f'칸 {cell} ({x * 1000:.1f}, {y * 1000:.1f}) · 방향 {yaw_deg:.1f}° · 칸 위 J5 {ta[4]} · 손가락 ↔ 컵 최소 여유 {clear * 1000:.0f} mm · 경유 {len(vias)}'


def plan_rack(C, cell, grasp_lift_ticks, rc=None):
    """기존 10칸은 그대로, L7/L8만 선정된 상태별 프로파일로 동기 계산한다."""
    if cell not in ('L7', 'L8'):
        return _plan_rack_impl(C, cell, grasp_lift_ticks, rc)
    raw_rc = rc or load_cells()  # 기존 함수의 기본 입력 동작을 유지한다.
    other = 'L8' if cell == 'L7' else 'L7'
    full = raw_rc['cells'][other]['state'].startswith('full')
    if cell == 'L8' and full:
        raise SystemExit('칸 L8: L7 이 차 있으면 놓은 뒤 빠져나올 길 없음 (손가락이 L7 퍽에 닿음) — L8 먼저, L7 나중')
    profile = _L78_PROFILES[(cell, full)]
    saved_obstacles = list(ps.EXTRA_OBS)
    token = _RACK_PROFILE.set(profile)
    try:
        # 원본 호출부는 계획 전에 컵 장애물을 설정하지 않으므로 여기서 준비한다.
        # raw_rc를 각각 한 번 shift: 검사와 구현의 좌표가 같고 중복 보정은 없다.
        ps.set_obstacles(cup_obstacles(raw_rc, cell))
        return _plan_rack_impl(C, cell, grasp_lift_ticks, raw_rc)
    finally:
        ps.EXTRA_OBS[:] = saved_obstacles  # 전역 장애물은 기존 동기 CLI 사용 조건에서 복원한다.
        _RACK_PROFILE.reset(token)  # 예외가 나도 다음 칸의 기본 설정에 영향을 주지 않는다.


def cup_obstacles(rc=None, cell=None):
    """pose_safety 용 칸별 회전 컵 · 찬 칸 퍽을 감싼 상자 (퍽 밀림 2 mm 포함). cell 을 주면 그 칸의 place_offset 만큼 옮김 (plan_rack 과 같게)"""
    rc = shifted(rc or load_cells(), cell)
    out = []
    for n, c in rc['cells'].items():
        cup_half, puck_half = cell_footprints(rc, c)
        boxes = [(f'cup {n}', cup_half, rc['rim_z'])]
        if c['state'].startswith('full'):
            boxes.append((f'cup {n} puck', puck_half, rc['floor_z'] + PUCK_H))
        for name, half, top in boxes:
            out.append(dict(name=name, kind='box', xyz=[c['xy'][0], c['xy'][1], (rc['floor_z'] + top) / 2],
                            dims=[half, half, (top - rc['floor_z']) / 2]))
    return out
