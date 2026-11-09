"""적재함 12칸 도달 검사 공용 함수 (CPU 전용, 읽기 전용 자료 사용).

URDF 파싱·순기구학·충돌 형상 적재·거리 계산. isaaclab 은 쓰지 않는다.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh

REL = Path(os.environ.get('URDF_RELEASE', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nt_urdf_release_20261027_layout')))
URDF_PATH = REL / "urdf/robot.urdf"
PKG = "package://nt_release_description/"
ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
FINGER_JOINTS = ["gripper_finger_a_joint", "gripper_finger_b_joint"]
G0_MM = 58.50723799617533
PLATE_TOP_Z = 0.0915


def gap_to_q(gap_mm):
    return (gap_mm - G0_MM) / 2000.0


# ---------------------------------------------------------------- 변환
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def make_T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = rpy_matrix(*rpy); T[:3, 3] = xyz
    return T


def axis_angle(axis, ang):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    T = np.eye(4); T[:3, :3] = R
    return T


def yaw_T(x, y, z, yaw):
    return make_T((x, y, z), (0, 0, yaw))


def apply(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


# ---------------------------------------------------------------- URDF
def _origin(el):
    o = el.find("origin")
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return make_T(xyz, rpy)


class URDF:
    def __init__(self, path=URDF_PATH):
        root = ET.parse(path).getroot()
        self.links, self.joints = {}, {}
        for L in root.findall("link"):
            geo = {"collision": [], "visual": []}
            for kind in ("collision", "visual"):
                for g in L.findall(kind):
                    ge = g.find("geometry")
                    if ge is None:
                        continue
                    item = {"name": g.get("name"), "T": _origin(g)}
                    m, b, c = ge.find("mesh"), ge.find("box"), ge.find("cylinder")
                    if m is not None:
                        item["type"] = "mesh"; item["file"] = str(REL / m.get("filename").replace(PKG, ""))
                        item["scale"] = [float(v) for v in m.get("scale", "1 1 1").split()]
                    elif b is not None:
                        item["type"] = "box"; item["size"] = [float(v) for v in b.get("size").split()]
                    elif c is not None:
                        item["type"] = "cyl"; item["r"] = float(c.get("radius")); item["l"] = float(c.get("length"))
                    else:
                        continue
                    geo[kind].append(item)
            self.links[L.get("name")] = geo
        for J in root.findall("joint"):
            d = {"name": J.get("name"), "type": J.get("type"), "parent": J.find("parent").get("link"),
                 "child": J.find("child").get("link"), "T": _origin(J)}
            a = J.find("axis"); d["axis"] = np.array([float(v) for v in a.get("xyz").split()]) if a is not None else np.array([1., 0, 0])
            lim = J.find("limit")
            if lim is not None:
                d["lower"], d["upper"] = float(lim.get("lower", 0)), float(lim.get("upper", 0))
            mm = J.find("mimic")
            if mm is not None:
                d["mimic"] = (mm.get("joint"), float(mm.get("multiplier", 1)), float(mm.get("offset", 0)))
            self.joints[d["name"]] = d
        self.child_joint = {j["child"]: j for j in self.joints.values()}

    def fk(self, q):
        """q: {joint: value} → {link: 4x4 base_link 기준}"""
        out = {"base_link": np.eye(4)}

        def get(link):
            if link in out:
                return out[link]
            j = self.child_joint[link]
            Tp = get(j["parent"])
            T = Tp @ j["T"]
            v = q.get(j["name"], 0.0)
            if "mimic" in j:
                src, mul, off = j["mimic"]; v = q.get(src, 0.0) * mul + off
            if j["type"] in ("revolute", "continuous"):
                T = T @ axis_angle(j["axis"], v)
            elif j["type"] == "prismatic":
                Tt = np.eye(4); Tt[:3, 3] = np.asarray(j["axis"]) / np.linalg.norm(j["axis"]) * v; T = T @ Tt
            out[link] = T
            return T

        for L in self.links:
            get(L)
        return out

    def limits(self, names=ARM_JOINTS):
        return np.array([self.joints[n]["lower"] for n in names]), np.array([self.joints[n]["upper"] for n in names])


def load_mesh_scaled(path, scale):
    m = trimesh.load(path, force="mesh").copy()
    if any(abs(s - 1) > 1e-12 for s in scale):
        S = np.diag([scale[0], scale[1], scale[2], 1.0]); m.apply_transform(S)
    return m


def link_mesh(urdf, link, kind="collision"):
    parts = []
    for it in urdf.links[link][kind]:
        if it["type"] == "mesh":
            m = load_mesh_scaled(it["file"], it["scale"])
        elif it["type"] == "box":
            m = trimesh.creation.box(extents=it["size"])
        else:
            m = trimesh.creation.cylinder(radius=it["r"], height=it["l"])
        m.apply_transform(it["T"])
        parts.append(m)
    return trimesh.util.concatenate(parts) if parts else None


# ---------------------------------------------------------------- 빠른 팔 FK (5관절 + 손가락)
class ArmFK:
    """base_link → arm_link1..5, 손가락 a/b 의 빠른 FK."""
    CHAIN = ["arm_link1", "arm_link2", "arm_link3", "arm_link4_wrist", "arm_link5_wrist_roll"]

    def __init__(self, urdf):
        self.u = urdf
        f = urdf.fk({})
        self.T_base_armbase = f["arm_base_link"]
        self.js = [urdf.child_joint[l] for l in self.CHAIN]
        self.fa, self.fb = urdf.joints["gripper_finger_a_joint"], urdf.joints["gripper_finger_b_joint"]

    def __call__(self, q5, fq=0.0):
        T = self.T_base_armbase.copy(); out = {"arm_base_link": T}
        for j, v in zip(self.js, q5):
            T = T @ j["T"] @ axis_angle(j["axis"], v); out[j["child"]] = T
        T5 = T
        for jf, link in ((self.fa, "gripper_finger_a"), (self.fb, "gripper_finger_b")):
            Tt = np.eye(4); Tt[:3, 3] = jf["axis"] / np.linalg.norm(jf["axis"]) * fq
            out[link] = T5 @ jf["T"] @ Tt
        return out


# ---------------------------------------------------------------- 해석적 거리 (상자·원기둥)
def dist_points_box(P, T_box, half):
    """점 P (n,3) 에서 상자(중심 T_box, 반크기 half) 까지 거리. 안쪽이면 음수(가장 가까운 면까지)."""
    L = (P - T_box[:3, 3]) @ T_box[:3, :3]
    d = np.abs(L) - half
    outside = np.linalg.norm(np.maximum(d, 0), axis=1)
    inside = np.minimum(np.max(d, axis=1), 0)
    return outside + inside


def dist_points_cyl(P, T_cyl, r, h):
    """원기둥(축 z, 중심 T_cyl, 반지름 r, 높이 h) 까지 부호 거리."""
    L = (P - T_cyl[:3, 3]) @ T_cyl[:3, :3]
    dr = np.hypot(L[:, 0], L[:, 1]) - r
    dz = np.abs(L[:, 2]) - h / 2
    outside = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0))
    inside = np.minimum(np.maximum(dr, dz), 0)
    return outside + inside


# ---------------------------------------------------------------- 표면 점
def sample_surface(mesh, spacing=0.001, seed=0):
    area = mesh.area
    n = int(max(200, area / spacing ** 2 * 1.3))
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=seed)
    return np.vstack([pts, mesh.vertices]).astype(np.float64)


# ================================================================ 충돌·거리 엔진
import open3d as o3d
from shapely.geometry import Polygon, MultiPoint, Point
from shapely import affinity

CAP = 0.03          # 이보다 먼 거리는 정확히 재지 않음(30 mm 이상 = 충분)
SPACING = 0.001     # 표면 점 간격 1 mm


def o3d_scene(mesh):
    s = o3d.t.geometry.RaycastingScene()
    s.add_triangles(o3d.core.Tensor(np.asarray(mesh.vertices, np.float32)), o3d.core.Tensor(np.asarray(mesh.faces, np.uint32)))
    return s


def scene_dist(scene, P):
    if len(P) == 0:
        return np.array([CAP])
    return scene.compute_distance(o3d.core.Tensor(np.ascontiguousarray(P, dtype=np.float32))).numpy().astype(float)


class Body:
    """메시 하나(자기 좌표계) + 표면 점 + open3d 장면."""
    def __init__(self, name, mesh, spacing=SPACING):
        self.name, self.mesh = name, mesh
        self.pts = sample_surface(mesh, spacing)
        self.scene = o3d_scene(mesh)
        self.lo, self.hi = mesh.bounds
        self.center = (self.lo + self.hi) / 2; self.radius = np.linalg.norm(self.hi - self.lo) / 2


class Analytic:
    """해석적 볼록 장애물: 상자 또는 원기둥 (base 좌표계 T)."""
    def __init__(self, name, kind, T, dims, group=None):
        self.name, self.kind, self.T, self.dims = name, kind, T, np.asarray(dims, float)
        self.group = group or name
        if kind == "box":
            self.radius = np.linalg.norm(self.dims)  # dims = 반크기
        else:
            self.radius = math.hypot(self.dims[0], self.dims[1] / 2)

    def dist(self, P):
        if self.kind == "box":
            return dist_points_box(P, self.T, self.dims)
        return dist_points_cyl(P, self.T, self.dims[0], self.dims[1])


def puck_shapes(T_center, shape):
    """퍽(중심 T, z=퍽 축) → Analytic 하나. shape: 'square' 30x30x60 | 'cyl' Ø30x60"""
    if shape == "square":
        return Analytic("puck", "box", T_center, [0.015, 0.015, 0.030])
    return Analytic("puck", "cyl", T_center, [0.015, 0.060])


def puck_mesh(shape):
    if shape == "square":
        return trimesh.creation.box(extents=[0.030, 0.030, 0.060])
    return trimesh.creation.cylinder(radius=0.015, height=0.060, sections=64)


def tube_cell(name, T_cell, outer=0.040, hole=0.033, height=0.026, floor=0.003):
    """새 칸(바깥 40, 구멍 33, 높이 26, 바닥 3 mm 가정) → Analytic 상자 5개. T_cell 원점 = 칸 바닥면 중심(상판 윗면)."""
    w = (outer - hole) / 2; out = []
    def add(sub, cx, cy, cz, hx, hy, hz):
        T = T_cell @ make_T((cx, cy, cz)); out.append(Analytic(f"{name}:{sub}", "box", T, [hx, hy, hz], group=name))
    add("floor", 0, 0, floor / 2, outer / 2, outer / 2, floor / 2)
    zc, hz = (floor + height) / 2, (height - floor) / 2
    add("wx+", outer / 2 - w / 2, 0, zc, w / 2, outer / 2, hz)
    add("wx-", -outer / 2 + w / 2, 0, zc, w / 2, outer / 2, hz)
    add("wy+", 0, outer / 2 - w / 2, zc, hole / 2, w / 2, hz)
    add("wy-", 0, -outer / 2 + w / 2, zc, hole / 2, w / 2, hz)
    return out
