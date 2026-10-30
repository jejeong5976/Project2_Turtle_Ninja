"""팔 무작위 자세 후보 (YOLO 데이터: 움직이는 팔이 광각 화면에 보이기도·안 보이기도 하게). Isaac 없이 numpy 만 씀.

robot.urdf 의 팔 사슬(base_link -> arm_link1..5 -> 손가락)로 순기구학을 계산해 말이 되는 자세만 남긴다.
  - 관절 한계 안 (끝에서 margin 만큼 안쪽), 손목 비틀기(arm_joint5) ±90°
  - 모든 점이 바닥 위 (base_link z > FLOOR_Z + 여유)
  - 차체 위 (차체 외곽 안쪽 점은 접은 자세의 가장 낮은 점 높이 이상) -> 팔이 차체를 뚫지 않음 (시뮬은 자기 충돌 꺼짐)
  - 상단 카메라 기둥(세로 프로파일, turtleninja_maze_env.PROFILE_XY)과 떨어짐
  - 수평으로 REACH_MAX 안 (반환값 reach 로 로봇 자리 검사 반경을 키움 -> 벽에 부딪히지 않게)
손끝 = 손가락 원점에서 손목->손가락 방향으로 TIP_EXT 배 더 나간 점 (손가락 메시 길이 미측정 -> 가정)
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ARM_JOINTS = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
FINGERS = ['gripper_finger_a_joint', 'gripper_finger_b_joint']
FLOOR_Z = -0.072          # base_link 기준 바퀴 바닥 (turtleninja_maze_env.SPAWN_Z 설명)
FLOOR_MARGIN = 0.03
BODY_HALF = 0.15          # 차체 외곽 ±0.133 + 여유
POLE_R = 0.06
REACH_MAX = 0.45
TIP_EXT = 0.7
JOINT_MARGIN = 0.1
J5_LIMIT = math.pi / 2


def _rot(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def _rpy(r, p, y):
    return _rot((0, 0, 1), y) @ _rot((0, 1, 0), p) @ _rot((1, 0, 0), r)


class ArmFK:
    def __init__(self, urdf: Path):
        root = ET.parse(str(urdf)).getroot()
        self.by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        self.limits = {}
        for name in ARM_JOINTS + FINGERS:
            lim = self.joints[name].find('limit')
            self.limits[name] = (float(lim.get('lower')), float(lim.get('upper')))
        self.chains = {f: self._chain(self.joints[f].find('child').get('link')) for f in FINGERS}

    def _chain(self, link):
        out = []
        while link in self.by_child:
            j = self.by_child[link]
            o = j.find('origin')
            xyz = np.array([float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()])
            rpy = [float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()]
            ax = j.find('axis')
            axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1]
            out.append((j.get('name'), j.get('type'), xyz, _rpy(*rpy), axis))
            link = j.find('parent').get('link')
            if link == 'base_link':
                break
        return out[::-1]

    def points(self, q: dict) -> np.ndarray:
        """관절값 -> 팔 링크 원점들 + 손끝 2개 (base_link 기준, (N, 3))."""
        pts = []
        for f in FINGERS:
            R, t = np.eye(3), np.zeros(3)
            frames = []
            for name, typ, xyz, R0, axis in self.chains[f]:
                t = t + R @ xyz
                R = R @ R0
                v = q.get(name, 0.0)
                if typ == 'revolute':
                    R = R @ _rot(axis, v)
                elif typ == 'prismatic':
                    t = t + R @ (np.asarray(axis, float) * v)
                frames.append(t.copy())
            wrist, finger = frames[-2], frames[-1]
            pts += frames[1:] + [finger + TIP_EXT * (finger - wrist)]
        return np.array(pts)

    def ok(self, pts: np.ndarray, body_min_z: float, pole_xy) -> bool:
        if (pts[:, 2] < FLOOR_Z + FLOOR_MARGIN).any():
            return False
        inside = (np.abs(pts[:, 0]) < BODY_HALF) & (np.abs(pts[:, 1]) < BODY_HALF)
        if (pts[inside, 2] < body_min_z).any():
            return False
        if (np.hypot(pts[:, 0] - pole_xy[0], pts[:, 1] - pole_xy[1]) < POLE_R).any():
            return False
        return float(np.hypot(pts[:, 0], pts[:, 1]).max()) <= REACH_MAX


def make_bank(urdf: Path, tuck: dict, n: int, rng: np.random.Generator, pole_xy, max_tries: int = 200000,
              body_ref: dict | None = None):
    """무작위 자세 n 개: [(관절값 dict, 수평 도달 거리 m)]. 첫 항목 = 접은 자세.
    body_ref: 차체 위 팔 최저 높이 기준 자세 (없으면 tuck). 받침 자세는 차체 안으로 내려가 있어 P0 를 기준으로 줌."""
    fk = ArmFK(urdf)
    tuck_pts = fk.points(tuck)
    ref_pts = fk.points(body_ref) if body_ref is not None else tuck_pts
    inside = (np.abs(ref_pts[:, 0]) < BODY_HALF) & (np.abs(ref_pts[:, 1]) < BODY_HALF)
    body_min_z = float(ref_pts[inside, 2].min()) - 0.01 if inside.any() else 0.0
    bank = [(dict(tuck), float(np.hypot(tuck_pts[:, 0], tuck_pts[:, 1]).max()))]
    for _ in range(max_tries):
        if len(bank) > n:
            break
        q = {}
        for name in ARM_JOINTS:
            lo, hi = fk.limits[name]
            if name == 'arm_joint5':
                lo, hi = max(lo, -J5_LIMIT), min(hi, J5_LIMIT)
            q[name] = float(rng.uniform(lo + JOINT_MARGIN, hi - JOINT_MARGIN))
        g = float(rng.uniform(*fk.limits[FINGERS[0]]))
        q.update({f: g for f in FINGERS})
        pts = fk.points(q)
        if fk.ok(pts, body_min_z, pole_xy):
            bank.append((q, float(np.hypot(pts[:, 0], pts[:, 1]).max())))
    return bank, body_min_z


# ---------- 손목 D435 관찰 자세 후보 (make_wrist_dataset.py) ----------
def link_pose(fk: ArmFK, q: dict, link: str):
    """관절값 -> base_link 기준 link 자세 (t (3,), R (3, 3)). 고정 · 회전 · 직선 관절 모두."""
    chain = []
    while link in fk.by_child and link != 'base_link':
        j = fk.by_child[link]
        chain.append(j)
        link = j.find('parent').get('link')
    R, t = np.eye(3), np.zeros(3)
    for j in chain[::-1]:
        o = j.find('origin')
        xyz = np.array([float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()])
        rpy = [float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()]
        t = t + R @ xyz
        R = R @ _rpy(*rpy)
        ax = j.find('axis')
        axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [1, 0, 0]
        v = q.get(j.get('name'), 0.0)
        if j.get('type') in ('revolute', 'continuous'):
            R = R @ _rot(axis, v)
        elif j.get('type') == 'prismatic':
            t = t + R @ (np.asarray(axis, float) * v)
    return t, R


def view_of(fk: ArmFK, q: dict, cam_link: str, cam_off, target, K, size):
    """카메라(광학: x 오른쪽 · y 아래 · z 앞) 자세와 target(base_link 점)을 보는 정도.
    반환 dict: cam_t, dist (m), pitch (아래로 숙인 각 °), roll (영상 기울기 °), uv (target 투영 화소), off_axis (광축과 target 사이 °)."""
    tl, Rl = link_pose(fk, q, cam_link)
    t = tl + Rl @ np.asarray(cam_off[0], float)
    R = Rl @ np.asarray(cam_off[1], float)
    d = R.T @ (np.asarray(target, float) - t)                 # 카메라 좌표의 target
    z = R[:, 2]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -z[2]))))
    # 영상 기울기: 카메라 x 축(오른쪽)이 수평에서 기운 각
    roll = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
    uv = (K[0][0] * d[0] / d[2] + K[0][2], K[1][1] * d[1] / d[2] + K[1][2]) if d[2] > 1e-3 else (float('nan'),) * 2
    off = math.degrees(math.acos(max(-1.0, min(1.0, d[2] / max(np.linalg.norm(d), 1e-9)))))
    return {'cam_t': t, 'R': R, 'dist': float(np.linalg.norm(d)), 'pitch': pitch, 'roll': roll, 'uv': uv, 'off_axis': off}


def make_view_bank(urdf: Path, tuck: dict, n: int, rng: np.random.Generator, pole_xy, cam_link: str, cam_off, K, size,
                   target, shelf_box, seed_pose: dict | None = None, dist=(0.20, 0.45), pitch=(35.0, 80.0), max_roll=20.0,
                   margin_px=80, max_tries: int = 400000):
    """선반 윗면을 보는 관찰 자세 n 개: [(관절값, 보기 정보)].
    조건: make_bank 와 같은 몸체 · 바닥 · 기둥 · 도달 검사, 팔 점이 선반 상자(shelf_box = (xmin, xmax, ymin, ymax, ztop)) 안 아님 · 선반 뒤 벽(xmax) 안 넘음,
          카메라가 선반 윗면보다 5 cm 이상 위, target 까지 거리 dist, 아래로 pitch, 영상 기울기 ±max_roll, target 투영이 화면 가장자리 margin_px 안쪽.
    후보 뽑기: 절반은 관절 한계 전체에서 무작위, 절반은 seed_pose 근처(±0.5 rad) -> 쓸 만한 자세가 드문 공간을 빨리 채움."""
    fk = ArmFK(urdf)
    tuck_pts = fk.points(tuck)
    inside = (np.abs(tuck_pts[:, 0]) < BODY_HALF) & (np.abs(tuck_pts[:, 1]) < BODY_HALF)
    body_min_z = float(tuck_pts[inside, 2].min()) - 0.01 if inside.any() else 0.0
    W, H = size
    x0, x1, y0, y1, ztop = shelf_box
    bank, tries = [], 0
    for tries in range(max_tries):
        if len(bank) >= n:
            break
        q = {}
        for name in ARM_JOINTS:
            lo, hi = fk.limits[name]
            if name == 'arm_joint5':
                lo, hi = max(lo, -J5_LIMIT), min(hi, J5_LIMIT)
            lo, hi = lo + JOINT_MARGIN, hi - JOINT_MARGIN
            if seed_pose is not None and tries % 2:
                q[name] = float(np.clip(seed_pose[name] + rng.uniform(-0.5, 0.5), lo, hi))
            else:
                q[name] = float(rng.uniform(lo, hi))
        q.update({f: 0.0 for f in FINGERS})
        pts = fk.points(q)
        if not fk.ok(pts, body_min_z, pole_xy):
            continue
        if ((pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1) & (pts[:, 2] < ztop + 0.02)).any():
            continue
        if (pts[:, 0] > x1 - 0.02).any():                   # 선반 뒤 = 벽 (선반을 벽에 붙여 놓음)
            continue
        v = view_of(fk, q, cam_link, cam_off, target, K, size)
        u, w = v['uv']
        if not (dist[0] <= v['dist'] <= dist[1] and pitch[0] <= v['pitch'] <= pitch[1] and abs(v['roll']) <= max_roll
                and v['cam_t'][2] > ztop + 0.05 and margin_px <= u <= W - margin_px and margin_px <= w <= H - margin_px):
            continue
        bank.append(({k: round(val, 4) for k, val in q.items() if k in ARM_JOINTS}, v))
    return bank, tries + 1
