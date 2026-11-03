"""로봇 모델(팔 FK·IK) + 장면 거리 계산."""
import math, sys
import numpy as np
import trimesh
from scipy.optimize import least_squares
from shapely.geometry import Polygon, MultiPoint

from reach_lib import *

sys.path.insert(0, str(REL))
from config.nt_train_kit import Replay  # isaaclab 없이 import 됨

MOVING = ["arm_link1", "arm_link2", "arm_link3", "arm_link4_wrist", "arm_link5_wrist_roll", "gripper_finger_a", "gripper_finger_b"]
SHORT = {"arm_link1": "link1", "arm_link2": "link2", "arm_link3": "link3", "arm_link4_wrist": "link4", "arm_link5_wrist_roll": "link5",
         "gripper_finger_a": "finger_a", "gripper_finger_b": "finger_b", "puck": "held_puck"}
STATIC_GROUPS = {
    "chassis": ["base_link", "arm_top_plate_link", "bottom_plate_link", "lidar_front_link", "lidar_rear_link", "battery_link", "jetson_planned_link"],
    "j1_housing": ["arm_base_link"],
    "park_stand": ["gripper_park_stand_TEMP"],
    "profile_plate": ["profile_fixing_plate_TEMP"],
    "camera_column": ["profile_column_link", "profile_camera_bracket_TEMP", "birdeye_camera_link"],
}
CURRENT_HOLDERS = ["holder_front_left", "holder_front_right", "holder_middle_left", "holder_middle_right", "holder_rear_left", "holder_rear_center"]
ADJACENT = {("j1_housing", "link1"), ("link1", "link2"), ("link2", "link3"), ("link3", "link4"), ("link4", "link5"), ("link5", "finger_a"), ("link5", "finger_b"),
            ("finger_a", "finger_b"), ("held_puck", "finger_a"), ("held_puck", "finger_b"), ("held_puck", "link5")}
HOLD_GAP_MM = 28.1      # 실측 쥔 상태 간격(재생 기록)
OPEN_GAP_MM = 49.8      # 실측 놓은 뒤 간격(재생 기록)
TEMPLATE_T = 16.194     # '적재함으로' 끝
FLOOR = 0.003           # 칸 바닥 두께(지금 홀더 메시: 지느러미가 z=3 mm 에서 시작)


def moving_mesh(u, link):
    """충돌 형상 ∪ 보이는 형상(링크막대 CAD 자세 시각 제외) — 보수적."""
    parts = [link_mesh(u, link, "collision")]
    for it in u.links[link]["visual"]:
        if "gripper_crank_visual_5" in it["name"]:
            continue
        m = load_mesh_scaled(it["file"], it["scale"]); m.apply_transform(it["T"]); parts.append(m)
    return trimesh.util.concatenate([p for p in parts if p is not None])


class Model:
    def __init__(self, spacing=SPACING, puck_p_mm=18.0):
        self.u = URDF(); self.fk = ArmFK(self.u); self.rp = Replay()
        self.lo, self.hi = self.u.limits()
        a5 = self.u.joints["arm_joint5"]["axis"]; self.app5 = -a5 / np.linalg.norm(a5)
        c = self.u.joints["gripper_finger_a_joint"]["axis"]; self.close5 = c / np.linalg.norm(c)
        self.w5 = np.cross(self.app5, self.close5)
        F0 = self.fk(np.zeros(5), 0.0); T5i = inv(F0["arm_link5_wrist_roll"]); tips = []
        for L in ["gripper_finger_a", "gripper_finger_b"]:
            P = apply(T5i @ F0[L], link_mesh(self.u, L, "visual").vertices); pr = P @ self.app5
            tips.append(P[pr > pr.max() - 0.001].mean(0))
        self.tip5 = (tips[0] + tips[1]) / 2
        self.puck_p = puck_p_mm / 1000.0
        # 움직이는 몸
        self.bodies = {SHORT[L]: Body(SHORT[L], moving_mesh(self.u, L), spacing) for L in MOVING}
        self.link_of = {SHORT[L]: L for L in MOVING}
        self.puck_bodies = {s: Body("held_puck", puck_mesh(s), spacing) for s in ("square", "cyl")}
        # 고정 장애물 (base 좌표)
        F = self.u.fk({}); self.F_static = F
        self.static = {}
        for g, links in STATIC_GROUPS.items():
            ms = []
            for L in links:
                m = link_mesh(self.u, L, "collision")
                if m is not None:
                    m.apply_transform(F[L]); ms.append(m)
            self.static[g] = Body(g, trimesh.util.concatenate(ms), spacing)
        self.holder_bodies = {}
        for h in CURRENT_HOLDERS:
            m = link_mesh(self.u, h, "collision"); m.apply_transform(F[h]); self.holder_bodies[h] = Body(h, m, spacing)
        self.holder_T = {h: F[h] for h in CURRENT_HOLDERS}
        self.j1_xy = F["arm_base_link"][:3, 3][:2].copy()
        # 상판 외곽
        m = link_mesh(self.u, "arm_top_plate_link", "collision"); m.apply_transform(F["arm_top_plate_link"])
        s = m.section(plane_origin=[0, 0, 0.0905], plane_normal=[0, 0, 1])
        self.plate_hull = MultiPoint(np.vstack([d[:, :2] for d in s.discrete])).convex_hull
        self.template = self.make_template()

    # ------------------------------------------------------------ 자세
    def frames(self, q, fq):
        return self.fk(q, fq)

    def grip(self, q):
        T5 = self.fk(q, 0.0)["arm_link5_wrist_roll"]; R = T5[:3, :3]
        return R @ self.tip5 + T5[:3, 3], R @ self.app5, R @ self.close5, T5

    def puck_T_in5(self):
        """쥔 퍽 중심(link5 좌표). 퍽 축 = 접근축, 퍽 x = 손가락 닫힘축. 퍽 바닥이 손끝보다 p 아래."""
        T = np.eye(4); T[:3, :3] = np.column_stack([self.close5, self.w5, -self.app5])
        T[:3, 3] = self.tip5 + (self.puck_p - 0.030) * self.app5
        return T

    # ------------------------------------------------------------ 템플릿
    def make_template(self, t=TEMPLATE_T):
        q, g = self.rp.at(t)
        tip, a, c, T5 = self.grip(q)
        dists = {h: float(np.linalg.norm(tip[:2] - self.holder_T[h][:2, 3])) for h in CURRENT_HOLDERS}
        h = min(dists, key=dists.get); Th = self.holder_T[h]
        yaw = math.atan2(Th[1, 0], Th[0, 0])
        rel = inv(Th) @ np.r_[tip, 1]
        radial = math.atan2(Th[1, 3] - self.j1_xy[1], Th[0, 3] - self.j1_xy[0])
        tilt = math.degrees(math.acos(-a[2]))
        app_az = math.atan2(a[1], a[0])
        cp = c - c[2] * np.array([0, 0, 1.0]); close_az = math.atan2(cp[1], cp[0])
        wrapd = lambda x: (math.degrees(x) + 180) % 360 - 180
        # 접근축: 방사 좌표(J1→칸)에서 표현.  위치·roll: 칸 좌표.
        Rr = rpy_matrix(0, 0, -radial); a_rad = Rr @ a
        Rc = rpy_matrix(0, 0, -yaw); c_cell = Rc @ c; a_cell = Rc @ a
        return dict(t=t, q=q, finger_q=g, gap_mm=G0_MM + 2000 * g, holder=h, holder_dist_mm={k: v * 1000 for k, v in dists.items()},
                    tip_base=tip, app_base=a, close_base=c, tip_in_cell=rel[:3], cell_yaw=yaw, radial_az=radial,
                    tilt_deg=tilt, app_az_deg=math.degrees(app_az), app_rel_radial_deg=wrapd(app_az - radial), app_rel_cell_deg=wrapd(app_az - yaw),
                    close_az_deg=math.degrees(close_az), roll_rel_cell_deg=wrapd(close_az - yaw), roll_rel_cell_mod90=((math.degrees(close_az - yaw)) % 90),
                    a_radial=a_rad, c_cell=c_cell, a_cell=a_cell)

    def target_for_cell(self, T_cell, roll_mode="template", tilt_deg=None):
        """칸(T_cell: 바닥면 중심, z=상판 윗면) → (손끝 위치, 접근축, 닫힘축 목표)."""
        tp = self.template
        p = apply(T_cell, tp["tip_in_cell"][None])[0]
        radial = math.atan2(T_cell[1, 3] - self.j1_xy[1], T_cell[0, 3] - self.j1_xy[0])
        ar = tp["a_radial"]
        if tilt_deg is not None:       # 민감도: 접근축 기울기만 바꿈(방위는 템플릿 그대로)
            az = math.atan2(ar[1], ar[0]); t = math.radians(tilt_deg)
            ar = np.array([math.sin(t) * math.cos(az), math.sin(t) * math.sin(az), -math.cos(t)])
        a = rpy_matrix(0, 0, radial) @ ar
        c = T_cell[:3, :3] @ tp["c_cell"]
        if roll_mode == "aligned":   # 닫힘축을 칸 벽에 평행(칸 기준 0°/90°)
            yaw = math.atan2(T_cell[1, 0], T_cell[0, 0]); c = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        return p, a, c

    # ------------------------------------------------------------ IK
    def residual(self, q, p_t, a_t, c_t):
        tip, a, c, _ = self.grip(q)
        r_pos = (tip - p_t) * 1000.0
        r_app = (a - a_t) * (180 / math.pi)
        cp = c - (c @ a_t) * a_t; cp /= np.linalg.norm(cp) + 1e-12
        ct = c_t - (c_t @ a_t) * a_t; ct /= np.linalg.norm(ct) + 1e-12
        ang = math.atan2(np.cross(ct, cp) @ a_t, ct @ cp)
        r_roll = math.degrees(math.sin(4 * ang) / 4)       # 90° 마다 같음(정사각 칸·퍽)
        return np.r_[r_pos, r_app, r_roll]

    def errors(self, q, p_t, a_t, c_t):
        tip, a, c, _ = self.grip(q)
        cp = c - (c @ a_t) * a_t; cp /= np.linalg.norm(cp); ct = c_t - (c_t @ a_t) * a_t; ct /= np.linalg.norm(ct)
        ang = math.degrees(math.atan2(np.cross(ct, cp) @ a_t, ct @ cp)); ang = (ang + 45) % 90 - 45
        return dict(pos_mm=float(np.linalg.norm(tip - p_t) * 1000), app_deg=float(math.degrees(math.acos(np.clip(a @ a_t, -1, 1)))), roll_deg=float(abs(ang)))

    def ik(self, p_t, a_t, c_t, seed=None, n_rand=24, rng=0):
        q0 = self.template["q"]
        seeds = []
        if seed is not None:
            seeds.append(np.asarray(seed))
        else:
            r = np.random.default_rng(rng)
            for j1 in np.linspace(self.lo[0], self.hi[0], 13):
                for j5 in (q0[4], q0[4] + math.pi / 2, q0[4] - math.pi / 2):
                    seeds.append(np.r_[j1, q0[1:4], j5])
            for _ in range(n_rand):
                seeds.append(r.uniform(self.lo, self.hi))
        sols = []
        for s in seeds:
            s = np.clip(s, self.lo + 1e-6, self.hi - 1e-6)
            try:
                res = least_squares(self.residual, s, bounds=(self.lo, self.hi), args=(p_t, a_t, c_t), xtol=1e-10, ftol=1e-10, max_nfev=400)
            except Exception:
                continue
            e = self.errors(res.x, p_t, a_t, c_t)
            sols.append((res.x, e))
        return sols

    def best_ik(self, p_t, a_t, c_t, tol=(1.0, 1.0, 2.0)):
        sols = self.ik(p_t, a_t, c_t)
        ok = [s for s in sols if s[1]["pos_mm"] <= tol[0] and s[1]["app_deg"] <= tol[1] and s[1]["roll_deg"] <= tol[2]]
        q0 = self.template["q"]
        if ok:
            # 실물 성공 자세와 같은 꺾임(J2~J5 가까운 것) 우선
            ok.sort(key=lambda s: np.linalg.norm(s[0][1:] - q0[1:]))
            uniq = []
            for s in ok:
                if all(np.abs(s[0] - u_[0]).max() > 0.05 for u_ in uniq):
                    uniq.append(s)
            return True, uniq
        sols.sort(key=lambda s: s[1]["pos_mm"] + s[1]["app_deg"] + s[1]["roll_deg"])
        return False, sols[:1]

    def path_ik(self, q_start, p0, a_t, c_t, dzs, tol=(1.0, 1.0, 2.0)):
        """손끝을 p0 에서 위로 dz 만큼 수직 이동한 점들의 IK(앞 해를 씨앗으로)."""
        out, q = [], q_start
        for dz in dzs:
            p = p0 + np.array([0, 0, dz])
            sols = self.ik(p, a_t, c_t, seed=q)
            qn, e = sols[0]
            ok = e["pos_mm"] <= tol[0] and e["app_deg"] <= tol[1] and e["roll_deg"] <= tol[2]
            jump = float(np.degrees(np.abs(qn - q).max()))
            out.append(dict(dz=dz, q=qn, err=e, ok=ok, jump_deg=jump)); q = qn
        return out

    # ------------------------------------------------------------ 거리
    def clearance(self, q, fq, mesh_obs, ana_obs, held=None, cap=CAP, self_pairs=None):
        """한 자세의 (움직이는 몸, 장애물) 쌍별 최소 거리 [m] (cap 에서 자름).
        mesh_obs: {이름: Body(base 좌표)} · ana_obs: [Analytic] · held: None 또는 퍽 모양('square'/'cyl')."""
        Ts = self.fk(q, fq)
        movers = [(n, self.bodies[n], Ts[self.link_of[n]]) for n in self.bodies]
        if held:
            movers.append(("held_puck", self.puck_bodies[held], Ts["arm_link5_wrist_roll"] @ self.puck_T_in5()))
        out = {}
        W = {}
        for n, b, T in movers:
            P = apply(T, b.pts); W[n] = P
            lo, hi = P.min(0), P.max(0)
            for g, ob in mesh_obs.items():
                m = np.all((P > ob.lo - cap) & (P < ob.hi + cap), axis=1)
                d1 = scene_dist(ob.scene, P[m]).min() if m.any() else cap
                if g == "chassis":      # 넓고 평평한 차체·상판: 한 방향(움직이는 점 → 차체 면)만
                    d2 = cap
                else:
                    m2 = np.all((ob.pts > lo - cap) & (ob.pts < hi + cap), axis=1)
                    d2 = scene_dist(b.scene, apply(inv(T), ob.pts[m2])).min() if m2.any() else cap
                out[(n, g)] = min(d1, d2, cap)
            for a in ana_obs:
                c = a.T[:3, 3]
                m = np.sum((P - c) ** 2, axis=1) < (a.radius + cap) ** 2
                d = a.dist(P[m]).min() if m.any() else cap
                key = (n, a.group); out[key] = min(out.get(key, cap), d, cap)
        # 자기 충돌
        if self_pairs:
            mv = {n: (b, T) for n, b, T in movers}
            for n1, n2 in self_pairs:
                if n1 not in mv or n2 not in mv:
                    continue
                b1, T1 = mv[n1]; b2, T2 = mv[n2]; P1, P2 = W[n1], W[n2]
                lo2, hi2 = P2.min(0), P2.max(0); lo1, hi1 = P1.min(0), P1.max(0)
                m1 = np.all((P1 > lo2 - cap) & (P1 < hi2 + cap), axis=1); m2 = np.all((P2 > lo1 - cap) & (P2 < hi1 + cap), axis=1)
                d1 = scene_dist(b2.scene, apply(inv(T2), P1[m1])).min() if m1.any() else cap
                d2 = scene_dist(b1.scene, apply(inv(T1), P2[m2])).min() if m2.any() else cap
                out[(n1, n2)] = min(d1, d2, cap)
        # 상판 아래로 들어간 점(관통) 표시
        allP = np.vstack(list(W.values()))
        inside = allP[:, 2] < PLATE_TOP_Z - 0.0003
        if inside.any():
            from shapely import contains_xy
            sel = contains_xy(self.plate_hull, allP[inside, 0], allP[inside, 1])
            if sel.any():
                out[("arm", "plate_below")] = float((allP[inside][sel, 2] - PLATE_TOP_Z).min())
        return out
