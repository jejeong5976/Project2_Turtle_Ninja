#!/usr/bin/env python3
"""손목 D435 관찰(LOOK) 자세 찾기 — URDF 순기구학 + 그리퍼 가림 (Isaac 없이 numpy · OpenCV).

조건: 선반 윗면 8×8 cm 영역 (+ 선 퍽 높이 6 cm 까지 = 상자 8×8×6 cm) 이
      ① 화면 안 (가장자리 margin 안쪽) ② 그리퍼(손가락 · 크랭크 · 링크) 가림 영역과 안 겹침 ③ D435 깊이 최소 거리(min_z) 보다 멂
      + arm_poses 와 같은 몸체 · 바닥 · 기둥 · 도달 · 선반 상자 · 뒤 벽 검사, 영상 기울기 ±max_roll
      ④ 로봇 차체 · 팔 링크(손목 아래 제외)가 영역을 가리지 않음 (URDF 메시 점을 투영, 영역보다 카메라에 가까운 점이 영역 안이면 탈락)
점수 (작을수록 좋음, mm 비슷한 단위): 시뮬 seg8 평가(2026-10-29)에서 맞춘 중심 오차 예측 = a + b × 기울기(수직에서) + c × 거리
      + 차체 · 팔이 화면을 차지한 비율 × --w_body + 영역 중심이 화면 가운데에서 떨어진 정도(반폭 = 1) × --w_center
      + 관절 한계 여유 (0.15 rad 안쪽이면 +1)
      (2026-10-29 Isaac 렌더 확인: 차체 검사 없이 고른 수직 LOOK 은 화면 아래 40 % 를 차체가 가림 -> ④ · 차체 비율 점수 추가)
그리퍼 가림: 카메라 · 그리퍼가 같은 손목 링크에 붙어 있어 화면 속 가림 영역은 팔 자세와 무관, 손가락 벌림(--finger)만으로 정해짐
      손가락 관절 = URDF gripper_finger_*_joint (아래 −0.021254 = 닫힘 16 mm, 0 = 58.5 mm, 위 0.010247 = 최대 79 mm)

예) cd sim/robot_sim && python3 tools/find_look_pose.py --out /tmp/look
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim_core import arm_poses as ap   # noqa: E402  (numpy 만 씀)

URDF = ROOT / 'robot_description' / 'robot.urdf'
CAM = 'd435_color_optical_frame'
WRIST = 'arm_link5_wrist_roll'
K = np.array([[605.9, 0, 324.0], [0, 605.4, 245.9], [0, 0, 1.0]])
W, H = 640, 480
SHELF = (0.27, 0.27, 0.07)                 # turtleninja_maze_env.SHELF_SIZE
SHELF_FRONT = 0.133 + 0.03                 # ROBOT_FRONT + SHELF_STAND_GAP (base_link -> 선반 앞면)
ZTOP = ap.FLOOR_Z + SHELF[2]               # 선반 윗면 높이 (base_link 기준)
# 시뮬 seg8 평가 (view_sim, 잘린 퍽 뺌): 기울기 0–60° · 거리 0.20–0.60 m 에서 중심 오차 중앙값 (선형 맞춤은 main 에서 다시 함)
SIM_ERR = [(7.5, 2.9), (20, 3.2), (30, 3.6), (40, 4.0), (52.5, 4.9)]       # (기울기 °, mm)
SIM_RNG = [(0.23, 2.6), (0.28, 2.9), (0.32, 3.2), (0.37, 3.9), (0.45, 4.5)]  # (거리 m, mm)


def load_obj(path: Path):
    v, f = [], []
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            f.append([int(x.split('/')[0]) - 1 for x in line.split()[1:4]])
    return np.array(v), np.array(f)


def load_mesh_vertices(path: Path, scale=(1.0, 1.0, 1.0)) -> np.ndarray:
    """OBJ · STL(이진 · 문자) 정점 (N, 3) × scale"""
    if path.suffix.lower() == '.obj':
        v = load_obj(path)[0]
    else:
        b = path.read_bytes()
        if b[:5] == b'solid' and b'facet' in b[:400]:
            v = np.array([[float(x) for x in l.split()[1:4]] for l in b.decode(errors='ignore').splitlines() if l.strip().startswith('vertex')])
        else:
            n = int.from_bytes(b[80:84], 'little')
            v = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3).astype(float)
    return v * np.asarray(scale, float)


ARM_LINKS = ['arm_base_link', 'arm_link1', 'arm_link2', 'arm_link3', 'arm_link4_wrist']   # 관절에 따라 움직이는 팔 링크 (손목 이하는 gripper_mask)
SKIP_LINKS = {WRIST, 'gripper_finger_a', 'gripper_finger_b'}


def load_body_points(fk, max_per_mesh: int = 1500, seed: int = 0):
    """차체 · 팔 링크 메시 점 (손목 링크 · 손가락 제외). -> (차체 점 base_link 좌표 (N, 3), {팔 링크: 링크 좌표 점})
    차체 링크는 관절 0 자세(조향 0)로 고정, 팔 링크는 자세마다 변환"""
    import xml.etree.ElementTree as ET
    rng = np.random.default_rng(seed)
    root = ET.parse(str(URDF)).getroot()
    body, arm = [], {}
    for L in root.findall('link'):
        name = L.get('name')
        if name in SKIP_LINKS:
            continue
        pts = []
        for vis in L.findall('visual'):
            m = vis.find('geometry/mesh')
            if m is None:
                continue
            o = vis.find('origin')
            xyz = np.array([float(x) for x in (o.get('xyz') if o is not None else '0 0 0').split()])
            rpy = [float(x) for x in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()]
            sc = [float(x) for x in m.get('scale').split()] if m.get('scale') else (1.0, 1.0, 1.0)
            v = load_mesh_vertices(URDF.parent / m.get('filename'), sc)
            if len(v) > max_per_mesh:
                v = v[rng.choice(len(v), max_per_mesh, replace=False)]
            pts.append(v @ ap._rpy(*rpy).T + xyz)
        if not pts:
            continue
        pts = np.concatenate(pts)
        if name in ARM_LINKS:
            arm[name] = pts
        else:
            t, R = ap.link_pose(fk, {}, name)
            body.append(pts @ R.T + t)
    return np.concatenate(body), arm


def rel_pose(fk, q, link, ref):
    """ref 링크 기준 link 자세 (t, R)"""
    tl, Rl = ap.link_pose(fk, q, link)
    tr, Rr = ap.link_pose(fk, q, ref)
    return Rr.T @ (tl - tr), Rr.T @ Rl


def hinge(tilt_deg: float, pivot):
    """경첩 각도 -> URDF 카메라 광학 프레임 기준 (t 이동, R 회전). 경첩 축 = 광학 x (D435 긴 축 = 손가락 벌림 방향, URDF 에서 1.3° 차이),
    회전 중심 = pivot (광학 좌표, 브래킷 STL 로 어림: 렌즈에서 영상 아래쪽 28 mm · 뒤 17 mm). − = 그리퍼 쪽으로 숙임 (d435_tilt_R 과 같은 부호)"""
    a = math.radians(tilt_deg)
    R = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    p = np.asarray(pivot, float)
    return p - R @ p, R


def gripper_mask(fk, finger: float, cam_off):
    """손가락 벌림 -> 화면 속 그리퍼 가림 마스크 (H, W bool). 손목 링크 · 손가락 메시를 카메라에 투영해 삼각형을 채움"""
    import xml.etree.ElementTree as ET
    root = ET.parse(str(URDF)).getroot()
    links = {x.get('name'): x for x in root.findall('link')}
    q = {f: finger for f in ap.FINGERS}
    tc, Rc = rel_pose(fk, q, CAM, WRIST)
    tc, Rc = tc + Rc @ cam_off[0], Rc @ cam_off[1]
    mask = np.zeros((H, W), np.uint8)
    for ln in (WRIST, 'gripper_finger_a', 'gripper_finger_b'):
        tl, Rl = rel_pose(fk, q, ln, WRIST) if ln != WRIST else (np.zeros(3), np.eye(3))
        for vis in links[ln].findall('visual'):
            m = vis.find('geometry/mesh')
            if m is None or 'D435' in m.get('filename'):
                continue
            o = vis.find('origin')
            xyz = np.array([float(x) for x in (o.get('xyz') if o is not None else '0 0 0').split()])
            rpy = [float(x) for x in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()]
            v, f = load_obj(URDF.parent / m.get('filename'))
            pw = (v @ ap._rpy(*rpy).T + xyz) @ Rl.T + tl          # 손목 링크 좌표
            pc = (pw - tc) @ Rc                                    # 카메라 광학 좌표
            ok = pc[:, 2] > 0.015
            uv = np.stack([K[0, 0] * pc[:, 0] / np.maximum(pc[:, 2], 1e-6) + K[0, 2],
                           K[1, 1] * pc[:, 1] / np.maximum(pc[:, 2], 1e-6) + K[1, 2]], 1)
            for tri in f:
                if ok[tri].all():
                    p = uv[tri]
                    if np.abs(p).max() < 1e5:
                        cv2.fillConvexPoly(mask, np.round(p).astype(np.int32), 1)
    return mask.astype(bool)


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument('--finger', type=float, default=0.010247, help='손가락 관절값 (기본 = 최대 벌림 79 mm)')
    ap_.add_argument('--tilt_deg', type=float, default=0.0, help='경첩 각도 URDF 대비 추가분 (− = 그리퍼 쪽). URDF = 실물 추정 −12° 반영 (2026-10-29, 광축 ↔ 손가락 31°). 배포판 camera40 = +12')
    ap_.add_argument('--pivot', default='0,0.028,-0.017', help='경첩 핀 위치 (URDF 광학 좌표 m, 브래킷 STL 어림값)')
    ap_.add_argument('--targets', default='0.26,0.30,0.34', help='8×8 영역 중심의 base_link x (m). 선반 앞면 0.163, 뒷면 0.433 (0.22 는 차체가 화면에 크게 들어와 뺌)')
    ap_.add_argument('--fingers', default='-0.021254,0.010247', help='가림 검사에 합칠 손가락 관절값들 (기본 = 닫힘 16 mm + 최대 79 mm)')
    ap_.add_argument('--w_body', type=float, default=4.0, help='차체 · 팔이 화면을 차지한 비율 벌점 (비율 0.25 -> +1)')
    ap_.add_argument('--w_center', type=float, default=1.0, help='영역 중심이 화면 가운데에서 반폭만큼 떨어지면 +이 값')
    ap_.add_argument('--n', type=int, default=300000)
    ap_.add_argument('--margin_px', type=int, default=15)
    ap_.add_argument('--mask_dilate_px', type=int, default=15, help='그리퍼 가림 영역 여유')
    ap_.add_argument('--min_z', type=float, default=0.20, help='D435 깊이 최소 거리 (640×480, m)')
    ap_.add_argument('--max_roll', type=float, default=10.0)
    ap_.add_argument('--seed', type=int, default=0)
    ap_.add_argument('--out', default=str(ROOT / 'results' / 'look_pose'))
    a = ap_.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    fk = ap.ArmFK(URDF)
    cam_off = hinge(a.tilt_deg, [float(x) for x in a.pivot.split(',')])

    # 중심 오차 예측: 기울기 · 거리 각각 선형 (두 맞춤의 기울기만 씀, 절편은 평균)
    bt = np.polyfit(*np.array(SIM_ERR).T, 1)[0]
    br = np.polyfit(*np.array(SIM_RNG).T, 1)[0]
    pred = lambda tilt, d: 3.4 + bt * (tilt - 25) + br * (d - 0.32)

    mask = np.zeros((H, W), bool)
    for g in [float(x) for x in a.fingers.split(',')]:
        mask |= gripper_mask(fk, g, cam_off)
    body_pts, arm_pts = load_body_points(fk)
    print(f'차체 점 {len(body_pts)}, 팔 링크 점 {sum(len(v) for v in arm_pts.values())}')
    maskd = cv2.dilate(mask.astype(np.uint8), np.ones((2 * a.mask_dilate_px + 1,) * 2, np.uint8)).astype(bool)
    print(f'그리퍼 가림 (손가락 {a.fingers}): 화면 {mask.mean() * 100:.1f} %  (여유 {a.mask_dilate_px} px 포함 {maskd.mean() * 100:.1f} %)')
    cv2.imwrite(str(out / 'gripper_mask.png'), (mask * 255).astype(np.uint8))

    tuck = {**json.loads((ROOT / 'robot_description' / 'config' / 'initial_pose.json').read_text())['poses']['C1_P0']['q_rad'],
            **{f: 0.0 for f in ap.FINGERS}}
    tuck_pts = fk.points(tuck)
    inside = (np.abs(tuck_pts[:, 0]) < ap.BODY_HALF) & (np.abs(tuck_pts[:, 1]) < ap.BODY_HALF)
    body_min_z = float(tuck_pts[inside, 2].min()) - 0.01
    pole_xy = ap.link_pose(fk, {}, 'profile_camera_bracket_TEMP')[0][:2]
    seed = dict(json.loads((ROOT / 'robot_description' / 'config' / 'initial_pose.json').read_text())['poses']['LOOK_PUCK']['q_rad'])   # 현재 LOOK_PUCK
    targets = [float(x) for x in a.targets.split(',')]
    box = np.array([[sx * 0.04, sy * 0.04, sz] for sx in (-1, 0, 1) for sy in (-1, 0, 1) for sz in (0.0, 0.03, 0.06)])  # 8×8×6 cm 상자 점

    def evaluate(q, tx):
        tl, Rl = ap.link_pose(fk, q, CAM)
        tl, R = tl + Rl @ cam_off[0], Rl @ cam_off[1]
        T = np.array([tx, 0.0, ZTOP])
        pc = (T + box - tl) @ R                             # 카메라 좌표
        if (pc[:, 2] < a.min_z).any():
            return None
        uv = np.stack([K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2], K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]], 1)
        m = a.margin_px
        if (uv[:, 0] < m).any() or (uv[:, 0] > W - m).any() or (uv[:, 1] < m).any() or (uv[:, 1] > H - m).any():
            return None
        hull = cv2.convexHull(np.round(uv).astype(np.int32))
        reg = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(reg, hull, 1)
        if (reg.astype(bool) & maskd).any():
            return None
        # 차체 · 팔 링크 점 투영: 화면 차지 비율, 영역 가림
        P = [body_pts] + [v @ Rk.T + tk for k, v in arm_pts.items() for tk, Rk in [ap.link_pose(fk, q, k)]]
        bc = (np.concatenate(P) - tl) @ R
        bc = bc[bc[:, 2] > 0.02]
        bu = np.stack([K[0, 0] * bc[:, 0] / bc[:, 2] + K[0, 2], K[1, 1] * bc[:, 1] / bc[:, 2] + K[1, 2]], 1)
        on = (bu[:, 0] >= 0) & (bu[:, 0] < W) & (bu[:, 1] >= 0) & (bu[:, 1] < H)
        bu, bz = bu[on].astype(int), bc[on, 2]
        grid = np.zeros((H // 8, W // 8), np.uint8)
        grid[bu[:, 1] // 8, bu[:, 0] // 8] = 1
        body_frac = float(cv2.dilate(grid, np.ones((3, 3), np.uint8)).mean())
        regd = cv2.dilate(reg, np.ones((21, 21), np.uint8)).astype(bool)
        if (regd[bu[:, 1], bu[:, 0]] & (bz < pc[:, 2].max())).any():
            return None
        z = R[:, 2]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -z[2]))))           # 광축 ↔ 수직 아래
        roll = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
        if abs(roll) > a.max_roll:
            return None
        d = float(np.linalg.norm(T - tl))
        gap = cv2.distanceTransform((~maskd).astype(np.uint8), cv2.DIST_L2, 3)[reg.astype(bool)].min()   # 영역 ↔ 가림 최소 화소
        lim = min(min(q[n] - fk.limits[n][0], fk.limits[n][1] - q[n]) for n in ap.ARM_JOINTS)
        cen = np.hypot(uv[13, 0] - W / 2, uv[13, 1] - H / 2) / (W / 2)
        score = pred(tilt, d) + a.w_body * body_frac + a.w_center * cen + (0.0 if lim > 0.15 else 1.0)
        return dict(q={k: round(q[k], 4) for k in ap.ARM_JOINTS}, target_x=tx, tilt=round(tilt, 1), roll=round(roll, 1), dist=round(d, 3),
                    min_z=round(float(pc[:, 2].min()), 3), uv_center=[round(float(x)) for x in uv[13]], gap_px=round(float(gap), 1),
                    joint_margin=round(lim, 3), cam_xyz=[round(float(x), 3) for x in tl], pred_mm=round(score, 2), uv=uv.tolist(),
                    err_mm=round(pred(tilt, d), 2), body_frac=round(body_frac, 3), center_off=round(float(cen), 2), body_uv=bu[::7].tolist())

    found = []
    for i in range(a.n):
        q = {}
        for name in ap.ARM_JOINTS:
            lo, hi = fk.limits[name]
            if name == 'arm_joint5':
                lo, hi = max(lo, -ap.J5_LIMIT), min(hi, ap.J5_LIMIT)
            lo, hi = lo + ap.JOINT_MARGIN, hi - ap.JOINT_MARGIN
            q[name] = float(np.clip(seed[name] + rng.uniform(-0.6, 0.6), lo, hi)) if i % 2 else float(rng.uniform(lo, hi))
        q.update({f: 0.0 for f in ap.FINGERS})
        pts = fk.points(q)
        if not fk.ok(pts, body_min_z, pole_xy):
            continue
        x0, x1 = SHELF_FRONT, SHELF_FRONT + SHELF[0]
        if ((pts[:, 0] > x0) & (pts[:, 0] < x1) & (np.abs(pts[:, 1]) < SHELF[1] / 2) & (pts[:, 2] < ZTOP + 0.02)).any():
            continue
        if (pts[:, 0] > x1 - 0.02).any():
            continue
        for tx in targets:
            r = evaluate(q, tx)
            if r is not None:
                found.append(r)
    print(f'시도 {a.n}, 조건 맞는 (자세, 목표) {len(found)} 개')
    if not found:
        return
    found.sort(key=lambda r: r['pred_mm'])
    print('\n순위  목표x   기울기  거리    최소깊이  영역중심(px)  차체비율  중심벗어남  관절여유  오차예측  점수   관절 (J1..J5)')
    for k, r in enumerate(found[:10]):
        print(f'{k + 1:3}  {r["target_x"]:.2f}  {r["tilt"]:5.1f}°  {r["dist"]:.3f}  {r["min_z"]:.3f}    {str(r["uv_center"]):12} {r["body_frac"]:6.2f}   {r["center_off"]:6.2f}    '
              f'{r["joint_margin"]:.2f}    {r["err_mm"]:5.2f}   {r["pred_mm"]:5.2f}  {[r["q"][n] for n in ap.ARM_JOINTS]}')
    by_tilt = {}
    for r in found:
        b = int(r['tilt'] // 10) * 10
        if b not in by_tilt or r['pred_mm'] < by_tilt[b]['pred_mm']:
            by_tilt[b] = r
    print('\n기울기 구간별 최선 (가능한 범위 확인용)')
    for b in sorted(by_tilt):
        r = by_tilt[b]
        print(f'  {b:2}–{b + 10:2}°: 기울기 {r["tilt"]:.1f}° 거리 {r["dist"]:.3f} 목표x {r["target_x"]:.2f} 예측 {r["pred_mm"]:.2f} mm 관절 {[r["q"][n] for n in ap.ARM_JOINTS]}')
    best = found[0]
    vis = np.full((H, W, 3), 230, np.uint8)
    vis[maskd] = (150, 150, 150)
    vis[mask] = (60, 60, 60)
    for u_, v_ in best['body_uv']:
        cv2.circle(vis, (u_, v_), 2, (200, 120, 60), -1)
    uv = np.array(best['uv'])
    cv2.polylines(vis, [cv2.convexHull(np.round(uv).astype(np.int32))], True, (0, 160, 0), 2)
    base = np.round(uv[0::3]).astype(np.int32)                             # 선반 면 (z = 0) 점 9 개
    cv2.polylines(vis, [cv2.convexHull(base)], True, (0, 0, 200), 2)
    cv2.putText(vis, f'hinge {a.tilt_deg:+.0f}, tilt {best["tilt"]} deg, dist {best["dist"]} m, body {best["body_frac"]:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(out / 'best_view.png'), vis)
    (out / 'look_candidates.json').write_text(json.dumps([{k: v for k, v in r.items() if k not in ('uv', 'body_uv')} for r in found[:50]], indent=1))
    print(f'\n그림: {out / "best_view.png"} (빨강 = 선반 면 8×8 cm, 초록 = +6 cm 까지 상자, 진회색 = 그리퍼, 연회색 = 여유, 파랑 점 = 차체 · 팔)\n후보: {out / "look_candidates.json"}')


if __name__ == '__main__':
    main()
