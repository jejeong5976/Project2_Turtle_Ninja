#!/usr/bin/env python3
"""손눈 보정 — 바닥 깊이 평면으로 손목 D435 의 실제 장착 위치 · 각도 구하기 (계산만 · 포트 안 씀).

  $ARM_PY handeye_floor.py p1_Depth.raw p2_Depth.raw …      # 자세마다 깊이 스냅샷 1 장
  $ARM_PY handeye_floor.py --selftest                       # 가짜 자료로 풀이 확인

원리: 자세마다 깊이에서 바닥 평면을 뽑으면 (카메라 기준 바닥 법선 2 자유도 + 바닥까지 거리 1) = 3 개 식.
  같은 순간의 팔 관절값(run_pose_read 의 02_track, 0.29 s 간격) → FK 로 link5 자세를 알므로, 손목 자세가 서로 다른 5~6 장이면
  "link5 → 깊이 카메라" 6 자유도(회전 3 · 위치 3)가 풀림. 태그 · 박스 필요 없음 (바닥만 평평하면 됨).
가정: 바닥 = base_link 아래 0.072 m 평면 (차체가 바닥에 수평), 깊이 단위 0.001 m, 팔 관절 읽기 정상 (받침 자세 그리퍼 37° 실측 = FK 37°).
시각 맞춤: 깊이 메타데이터 Timestamp(ms, 호스트 시각) ↔ 02_track started_at_utc. 그 앞뒤 1 s 관절이 흔들리면 그 장은 뺌.
결과: link5 기준 깊이 · 컬러 카메라 자세, 팀원 PR URDF(−12° 추정) 와의 차이, URDF `d435_cad_front_fixed` 에 넣을 origin.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import csv
import glob
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
sys.path.insert(0, os.environ.get('PLACE_SPOTS', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'place_spots')))
import arm_pose_fk as fk  # noqa: E402
import camera_angle_ref as car  # noqa: E402
import place_ik as pk  # noqa: E402
from reach_model import Model  # noqa: E402

POSE_GLOB = os.path.join(os.environ.get('POSE_MEASURE', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'arm_workspace_measure_20261024')), 'runs', 'POSE_*', '**', '*.json')
KS = ('11', '12', '13', '14', '15', '16')
BASE_Z = 0.072
# 공장 extrinsic (S/N 261722072635, rs_calib_261722072635.txt): p_color = R p_depth + t
R_DC = np.array([[0.999909, 0.0101139, 0.0088869], [-0.0100548, 0.999927, -0.00666748], [-0.00895368, 0.00657752, 0.999938]])
t_DC = np.array([0.0150923375, -0.000127987645, 0.000309382624])
DEPTH_IN_PROXY_X = -0.0325 + 0.0151        # 깊이 원점(왼쪽 IR) = 케이스 가운데에서 광학 −x 약 17.4 mm (컬러 −32.5 mm 에서 15.1 mm 안쪽)
SIG_N, SIG_D = math.radians(0.3), 0.003     # 바닥 법선 · 거리 측정 흔들림 어림 (잔차 가중)


def T_of(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def inv(T):
    return T_of(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


# ---------- 깊이 → 바닥 평면 ----------
def read_depth(raw):
    raw = Path(raw)
    meta = {}
    with open(str(raw)[:-4] + '_metadata.csv', newline='') as f:
        for row in csv.reader(f):
            if len(row) >= 2:
                meta[row[0].strip()] = row[1].strip()
    W, H = int(meta['Resolution x']), int(meta['Resolution y'])
    Z = np.fromfile(raw, dtype='<u2').reshape(H, W).astype(float) * 0.001
    K = (float(meta['Fx']), float(meta['Fy']), float(meta['PPx']), float(meta['PPy']))
    return Z, K, float(meta['Timestamp (ms)']) / 1000.0


def _ransac(P, thr, iters, rng):
    best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    return best


def _refine(Q):
    for _ in range(3):
        c0 = Q.mean(0)
        n = np.linalg.svd(Q - c0, full_matrices=False)[2][2]       # ⚠ full_matrices=False (기본값이면 N×N 행렬로 메모리 폭발)
        keep = np.abs((Q - c0) @ n) < 2.5 * max(np.std((Q - c0) @ n), 0.0015)
        Q = Q[keep]
    if n @ c0 > 0:
        n = -n                                                     # 카메라 쪽을 향하게 (카메라 원점이 평면 위쪽)
    return n, -(c0 @ n), Q, c0


def floor_plane(Z, K, step=2, thr=0.004, iters=400, seed=0, min_frac=0.12):
    """바닥 = 화면의 큰 평면들(점 12 % 이상) 중 카메라에서 가장 먼 것 (박스 윗면 · 판자 · 차체는 더 가까움).
    반환: 카메라 쪽을 향한 단위 법선 n, 카메라 → 평면 거리 d, 전체 점 중 바닥 점 비율, 흔들림(mm)"""
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:Z.shape[0]:step, 0:Z.shape[1]:step]
    z = Z[::step, ::step]
    ok = (z > 0.15) & (z < 2.0)
    P = np.c_[(u[ok] - cx) / fx * z[ok], (v[ok] - cy) / fy * z[ok], z[ok]]
    rng = np.random.default_rng(seed)
    rest, planes = P, []
    for _ in range(4):
        if len(rest) < 0.05 * len(P):
            break
        inl = _ransac(rest, thr, iters, rng)
        if inl.sum() < min_frac * len(P):
            break
        n, d, Q, c0 = _refine(rest[inl])
        planes.append((d, n, Q, c0))
        rest = rest[~inl]
    d, n, Q, c0 = max(planes, key=lambda x: x[0])
    return n, d, len(Q) / len(P), float(np.std((Q - c0) @ n) * 1000)


# ---------- 관절 기록에서 그 순간 틱 ----------
_TRACK = None


def track():
    """POSE 폴더의 모든 읽기 (01_rest · 02_track · 02_pose_* · 03_rest_after) 를 시각순으로"""
    global _TRACK
    if _TRACK is None:
        rows = []
        for f in glob.glob(POSE_GLOB, recursive=True):
            try:
                d = json.loads(Path(f).read_text())
                t = datetime.strptime(d['started_at_utc'], '%Y-%m-%dT%H:%M:%S.%fZ').replace(tzinfo=timezone.utc).timestamp()
                p = {str(m['id']): m['registers']['present_position']['value'] for m in d['motors']}
                if all(k in p for k in KS):
                    src = 'track' if '/02_track/' in f else Path(f).stem
                    rows.append((t, np.array([p[k] for k in KS], float), src))
            except Exception:
                continue
        rows.sort(key=lambda r: r[0])
        _TRACK = rows
    return _TRACK


RUNNER_STOPS = []          # arm_pose_runner.py 기록: (도착 UTC, 떠남 UTC, 실측 틱) — 그 사이에 찍힌 스냅샷은 이 틱 (팔이 토크로 서 있음)


def load_runner(path):
    for line in Path(path).read_text().splitlines():
        r = json.loads(line)
        if r.get('event') == 'stop':
            f = lambda x: datetime.strptime(x, '%Y-%m-%dT%H:%M:%S.%fZ').replace(tzinfo=timezone.utc).timestamp()
            m = r['measured']
            RUNNER_STOPS.append((f(r['arrive_utc']), f(r['leave_utc']), {'11': m[0], '12': m[1], '13': m[2], '14': m[3], '16': m[4], '15': 3504}, r['name']))


def ticks_at(ts, guess_gap=0.0):
    for t0, t1, tk, name in RUNNER_STOPS:
        if t0 + 0.5 <= ts <= t1:
            return {k: int(round(v)) for k, v in tk.items()}, f'실행기 자세 {name} (토크 유지)', False
    """그 순간 틱. ① 앞뒤 0.6 s 안에 읽기가 있으면 시각으로 보간 (손 떨림 속도도 봄)
    ② 없고 --guess_gap 을 주면, 그 시간 안의 가장 가까운 읽기를 '추정' 으로 (그 주변 읽기가 안정적이고 받침 자세가 아닐 때만)"""
    rows = track()
    T = np.array([r[0] for r in rows])
    i = int(np.searchsorted(T, ts))
    lo, hi = (rows[i - 1] if i > 0 else None), (rows[i] if i < len(rows) else None)
    if lo is not None and hi is not None and ts - lo[0] <= 0.6 and hi[0] - ts <= 0.6:
        w = (ts - lo[0]) / max(hi[0] - lo[0], 1e-6)
        v = lo[1] + w * (hi[1] - lo[1])
        speed = np.abs(hi[1] - lo[1]) / max(hi[0] - lo[0], 1e-6)
        if speed[1:5].max() > 25 or speed[0] > 35:
            return None, f'찍는 순간 팔이 움직이는 중 (틱/s {speed.round().astype(int).tolist()})', False
        return {k: int(round(x)) for k, x in zip(KS, v)}, f'보간 · 움직임 틱/s 최대 {speed[:5].max():.0f}', False
    if guess_gap <= 0:
        return None, '그 시각 관절 기록 없음 (자세를 잡고 있는 동안 기록이 안 돌았음)', False
    cand = [(abs(r[0] - ts), j) for j, r in enumerate(rows) if abs(r[0] - ts) <= guess_gap]
    if not cand:
        return None, f'앞뒤 {guess_gap:.0f} s 안에 관절 기록 없음', False
    gap, j = min(cand)
    side = rows[max(0, j - 2):j + 3]
    arr = np.array([r[1] for r in side if abs(r[0] - rows[j][0]) < 1.5])
    spread = arr.max(0) - arr.min(0)
    v = rows[j][1]
    if v[1] < 1250:                                           # J2 ≈ 1102 = 받침 자세
        return None, f'가장 가까운 읽기({gap:.1f} s 차, {rows[j][2]}) 가 받침 자세', False
    if spread[1:5].max() > 6 or spread[0] > 8:
        return None, f'가장 가까운 읽기({gap:.1f} s 차) 주변이 흔들림 {spread.astype(int).tolist()}', False
    return {k: int(x) for k, x in zip(KS, v)}, f'⚠ 추정 — {gap:.1f} s 떨어진 읽기({rows[j][2]}), 그동안 자세를 유지했다고 가정', True


# ---------- 모델 ----------
class Rig:
    def __init__(self):
        self.M = Model(spacing=0.01)
        hg = car.Hinge()
        self.prior = {}
        for h in (-12.0, -22.0):
            T = hg.T(h).copy()
            T[:3, 3] += T[:3, 0] * DEPTH_IN_PROXY_X
            self.prior[h] = T                                       # link5 → 깊이 광학 프레임 (모델 · 경첩 h)
        # 컬러 광학 프레임 (깊이 기준): p_c = R p_d + t → 컬러 자세 = inv
        self.T_d_c = inv(T_of(R_DC, t_DC))
        pr = pk.Chain(car.PR_URDF, 'd435_color_optical_frame').T({})
        self.pr_5c = inv(pk.Chain(car.PR_URDF, 'arm_link5_wrist_roll').T({})) @ pr
        fr = pk.Chain(car.PR_URDF, 'd435_cad_front_frame').T({})
        self.front_c = inv(fr) @ pr                                 # 앞판 → 컬러 (URDF 고정)

    def T5(self, ticks):
        q, gap, fq = fk.ticks_to_q(ticks)
        return self.M.fk(q, fq)['arm_link5_wrist_roll'], q

    def approach(self, T5):
        return T5[:3, :3] @ self.M.app5


def predict(T5, T5d, base_z=BASE_Z):
    Twd = T5 @ T5d
    n = Twd[:3, :3].T @ np.array([0, 0, 1.0])                     # 바닥 위쪽 방향 = 카메라 기준 (카메라 쪽 법선)
    d = Twd[2, 3] + base_z
    return n, d


def solve(samples, T_init, free_floor=False):
    def unpack(x):
        dT = T_of(Rot.from_rotvec(x[:3]).as_matrix(), x[3:6])
        return T_init @ dT, (BASE_Z + x[6] if free_floor else BASE_Z)

    def res(x):
        T5d, bz = unpack(x)
        r = []
        for s in samples:
            n, d = predict(s['T5'], T5d, bz)
            r += list((s['n'] - n) / SIG_N) + [(s['d'] - d) / SIG_D]
        return np.array(r)

    x0 = np.zeros(7 if free_floor else 6)
    sol = least_squares(res, x0)
    T5d, bz = unpack(sol.x)
    J = sol.jac
    dof = max(1, len(sol.fun) - len(sol.x))
    s2 = float(sol.fun @ sol.fun) / dof
    try:
        cov = np.linalg.inv(J.T @ J) * max(s2, 1.0)
        std = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        std = np.full(len(sol.x), np.inf)
    return T5d, bz, sol, std, s2


def rpy(R):
    return Rot.from_matrix(R).as_euler('xyz')


def report(rig, samples, T5d, bz, sol, std, s2, free_floor):
    print(f'\n== 결과 ({len(samples)} 장) · 잔차 크기 {math.sqrt(s2):.2f} (1 ≈ 법선 0.3° · 거리 3 mm 수준)' + (f' · 바닥 높이 풀림 base_link 바닥 위 {bz*1000:.1f} mm' if free_floor else ''))
    print(f'   불확실성(1σ): 회전 {np.degrees(std[:3]).round(2).tolist()}° · 위치 {(std[3:6]*1000).round(1).tolist()} mm'
          + ('  ⚠ 자세 다양성 부족 — 손목 롤 · 기울기 · J1 을 더 다르게' if (np.degrees(std[:3]).max() > 0.7 or std[3:6].max() > 0.006) else ''))
    for i, s in enumerate(samples):
        n, d = predict(s['T5'], T5d, bz)
        print(f"   {s['name']:28s} 법선 차 {math.degrees(math.acos(np.clip(n @ s['n'], -1, 1))):4.2f}° · 거리 차 {(s['d'] - d)*1000:+5.1f} mm")
    T5c = T5d @ rig.T_d_c
    app = rig.M.app5
    ang = lambda T: math.degrees(math.acos(np.clip(app @ T[:3, 2], -1, 1)))
    dR = T5c[:3, :3] @ rig.pr_5c[:3, :3].T
    print(f'\n   컬러 카메라 (link5 기준): 위치 {(T5c[:3, 3]*1000).round(1).tolist()} mm · rpy {np.degrees(rpy(T5c[:3, :3])).round(2).tolist()}°')
    print(f'   팀원 PR URDF(−12° 추정) 대비: 회전 {math.degrees(Rot.from_matrix(dR).magnitude()):.1f}° · 위치 {np.linalg.norm(T5c[:3, 3] - rig.pr_5c[:3, 3])*1000:.1f} mm '
          f'(차이 벡터 {((T5c[:3, 3] - rig.pr_5c[:3, 3])*1000).round(1).tolist()} mm)')
    print(f'   광축 ↔ 손가락(접근축) 각도: 실측 {ang(T5c):.1f}° · PR URDF {ang(rig.pr_5c):.1f}°')
    Tf = T5c @ inv(rig.front_c)
    r_ = rpy(Tf[:3, :3])
    print(f'   URDF d435_cad_front_fixed 에 넣을 값 (parent arm_link5_wrist_roll):\n     <origin xyz="{Tf[0,3]:.6f} {Tf[1,3]:.6f} {Tf[2,3]:.6f}" rpy="{r_[0]:.6f} {r_[1]:.6f} {r_[2]:.6f}" />')
    return T5c


def selftest(rig):
    print('== 가짜 자료로 풀이 확인: 참값 = 경첩 −22° 모델에서 위치 (+4, −3, +6) mm 더 · 측정 잡음 0.3° · 2 mm')
    rng = np.random.default_rng(1)
    truth = rig.prior[-22.0] @ T_of(Rot.from_euler('xyz', [1.0, -0.5, 0.8], degrees=True).as_matrix(), [0.004, -0.003, 0.006])
    base = {'11': 4026, '12': 1667, '13': 1992, '14': 2066, '15': 3504, '16': 1996}
    samples = []
    deg = lambda x: round(x * 4096 / 360)
    # 권장 자세 다양성 (J1 · J4 · J5 각도 차, °): 20 번 반복 모의에서 회전 0.3° · 위치 2.6 mm (J4 ±10 · J5 ±30 정도면 0.5° · 5.6 mm)
    for k, (d1, d4, d5) in enumerate([(0, 0, 0), (0, -25, 0), (0, 20, 0), (0, 0, -60), (0, 0, 60), (-30, 10, 45), (30, -10, -45)]):
        t = dict(base); t['11'] += deg(d1); t['14'] += deg(d4); t['16'] += deg(d5)
        T5, _ = rig.T5(t)
        n, d = predict(T5, truth)
        n = Rot.from_rotvec(rng.normal(0, math.radians(0.3), 3)).as_matrix() @ n
        samples.append(dict(name=f'가짜 {k+1}', T5=T5, n=n / np.linalg.norm(n), d=d + rng.normal(0, 0.002)))
    T5d, bz, sol, std, s2 = solve(samples, rig.prior[-12.0])
    dR = math.degrees(Rot.from_matrix(T5d[:3, :3] @ truth[:3, :3].T).magnitude())
    dt = np.linalg.norm(T5d[:3, 3] - truth[:3, 3]) * 1000
    print(f'   참값과 차이: 회전 {dR:.2f}° · 위치 {dt:.1f} mm  → {"통과" if dR < 0.6 and dt < 6 else "⚠ 실패"}')
    report(rig, samples, T5d, bz, sol, std, s2, False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('raw', nargs='*', help='realsense-viewer 깊이 스냅샷 *_Depth.raw (옆에 *_metadata.csv 필요)')
    ap.add_argument('--free_floor', action='store_true', help='바닥 높이(base_link 0.072)도 같이 풀기 (검증용, 6 장 이상)')
    ap.add_argument('--guess_gap', type=float, default=0.0, help='그 순간 기록이 없을 때 이 초 안의 가장 가까운 읽기로 추정 (예: 12). 추정 장은 이름에 (추정), 한 장씩 빼기로 검증')
    ap.add_argument('--runner_log', help='arm_pose_runner.py 의 runs/runner_*/log.jsonl — 팔이 스스로 선 자세의 실측 틱 (그리퍼 틱은 FK 에 영향 거의 없음)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    rig = Rig()
    if a.selftest:
        return selftest(rig)
    if not a.raw:
        sys.exit(__doc__)
    if a.runner_log:
        load_runner(a.runner_log)
    samples = []
    for f in a.raw:
        name = Path(f).name
        Z, K, ts = read_depth(f)
        ticks, msg, guessed = ticks_at(ts, a.guess_gap)
        when = datetime.fromtimestamp(ts, timezone.utc).strftime('%H:%M:%S')
        if ticks is None:
            print(f'   {name}: 뺌 — {msg} (깊이 시각 {when} UTC)')
            continue
        n, d, frac, rough = floor_plane(Z, K)
        T5, q = rig.T5(ticks)
        tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
        print(f'   {name}: {when} UTC · 틱 J1..J5 {[ticks[k] for k in ("11", "12", "13", "14", "16")]} ({msg}) · 바닥 {d*1000:.0f} mm · 광축 수직에서 {tilt:.1f}° · 평면 점 {frac*100:.0f} % · 흔들림 {rough:.1f} mm'
              + ('  ⚠ 평면 점 비율 낮음 (바닥 말고 다른 면?)' if frac < 0.4 else ''))
        samples.append(dict(name=name + (' (추정)' if guessed else ''), T5=T5, n=n, d=d))
    if len(samples) < 3:
        sys.exit(f'쓸 수 있는 장 {len(samples)} 개 — 최소 3 장 (권장 5~6 장, 손목 롤 · 기울기 · J1 을 다르게)')
    for h in (-12.0, -22.0):
        n_err = [math.degrees(math.acos(np.clip(predict(s['T5'], rig.prior[h])[0] @ s['n'], -1, 1))) for s in samples]
        print(f'   (참고) 모델 경첩 {h:+.0f}° 그대로면 법선 차 평균 {np.mean(n_err):.1f}°')
    T5d, bz, sol, std, s2 = solve(samples, rig.prior[-12.0], a.free_floor)
    report(rig, samples, T5d, bz, sol, std, s2, a.free_floor)
    if len(samples) >= 4:
        print('\n   한 장씩 빼고 다시 (결과가 크게 바뀌는 장 = 의심):')
        for i in range(len(samples)):
            sub = samples[:i] + samples[i + 1:]
            T2, *_ = solve(sub, rig.prior[-12.0], a.free_floor)
            print(f"     {samples[i]['name']:28s} 빼면 회전 {math.degrees(Rot.from_matrix(T2[:3, :3] @ T5d[:3, :3].T).magnitude()):.2f}° · 위치 {np.linalg.norm(T2[:3, 3] - T5d[:3, 3])*1000:.1f} mm 바뀜")


if __name__ == '__main__':
    main()
