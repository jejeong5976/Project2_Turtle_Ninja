"""parked 자세 + 그리퍼 16 mm 에서 손가락 충돌 모양이 받침에 얼마나 들어가는지 계산 (읽기만, 결과 config/park_stand_check.json).
비교: (A) 지금 받침 충돌 = 임시 상자(바닥+벽 4) (B) 실제 받침 모델 mani_home.stl(보이는 모양, 같은 위치·배율).
URDF FK(고정 관절 포함), 손가락 prismatic 은 축 방향 이동. 실행: /home/pa25/IsaacLab-course/.venv/bin/python tools/check_park_stand.py [gap_mm]"""
import json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, trimesh

HERE = Path(__file__).resolve().parents[1]
GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 16.0
r = ET.parse(HERE / "urdf/robot.urdf").getroot()
links = {l.get("name"): l for l in r.findall("link")}
J = {j.find("child").get("link"): j for j in r.findall("joint")}
pose = json.loads((HERE / "config/initial_pose.json").read_text())["positions"]
qf = (GAP - 58.50723799617533) / 2000.0
Q = dict(pose); Q["gripper_finger_a_joint"] = qf; Q["gripper_finger_b_joint"] = qf


def rpyR(rr, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(rr), math.sin(rr), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def axisR(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def origin(e):
    o = e.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()])
    rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
    return rpyR(*rpy), xyz


def T(link):
    chain = []
    while link in J:
        chain.append(J[link]); link = J[link].find("parent").get("link")
    R, t = np.eye(3), np.zeros(3)
    for j in reversed(chain):
        Ro, to = origin(j); t = t + R @ to; R = R @ Ro
        n, typ = j.get("name"), j.get("type")
        if typ in ("revolute", "continuous", "prismatic") and n in Q:
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            if typ == "prismatic":
                t = t + R @ (ax * Q[n])
            else:
                R = R @ axisR(ax, Q[n])
    return R, t


def geoms(link, tag):
    out = []
    for e in links[link].findall(tag):
        Ro, to = origin(e); g = e.find("geometry")
        R, t = T(link); Rw, tw = R @ Ro, t + R @ to
        if g.find("mesh") is not None:
            m = g.find("mesh"); f = m.get("filename").replace("package://nt_release_description/", str(HERE) + "/")
            sc = np.array([float(v) for v in (m.get("scale") or "1 1 1").split()])
            mesh = trimesh.load(f, force="mesh"); mesh.vertices = mesh.vertices * sc
        else:
            mesh = trimesh.creation.box(extents=[float(v) for v in g.find("box").get("size").split()])
        mesh.vertices = (Rw @ mesh.vertices.T).T + tw
        out.append((e.get("name"), mesh))
    return out


stand = "gripper_park_stand_TEMP"
fingers = [l for l in links if l.startswith("gripper_finger_")]
fcols = [(l, n, m) for l in fingers for n, m in geoms(l, "collision")]
res = {"gap_mm": GAP, "finger_q_m": qf, "finger_collision_pieces": len(fcols)}
# (A) temp boxes: penetration = max depth of finger vertices inside each box
boxes = geoms(stand, "collision")
A = []
for bn, bm in boxes:
    for fl, fn, fm in fcols:
        inside = bm.contains(fm.vertices) if bm.is_watertight else np.zeros(len(fm.vertices), bool)
        if inside.any():
            d = -trimesh.proximity.signed_distance(bm, fm.vertices[inside]) if False else trimesh.proximity.signed_distance(bm, fm.vertices[inside])
            A.append({"box": bn, "finger": fn, "verts_inside": int(inside.sum()), "max_depth_mm": round(float(np.max(d)) * 1000, 2)})
res["A_temp_box_penetrations"] = A
# (B) real stand visual mesh
vis = geoms(stand, "visual")[0][1]
res["B_stand_mesh_watertight"] = bool(vis.is_watertight)
B = []
if vis.is_watertight:
    for fl, fn, fm in fcols:
        inside = vis.contains(fm.vertices)
        if inside.any():
            d = trimesh.proximity.signed_distance(vis, fm.vertices[inside])
            B.append({"finger": fn, "verts_inside": int(inside.sum()), "max_depth_mm": round(float(np.max(d)) * 1000, 2)})
res["B_real_stand_penetrations"] = B
# lateral clearance: finger collision outer faces vs stand mesh along finger travel
for fl, fn, fm in fcols:
    pq = trimesh.proximity.ProximityQuery(vis)
    d = pq.signed_distance(fm.vertices)
    res.setdefault("B_min_signed_distance_mm", {})[fn] = round(float(-np.max(d)) * 1000, 2)   # + = gap, − = inside
R, t = T(stand)
res["stand_origin_base_mm"] = (t * 1000).round(1).tolist()
res["stand_mesh_bounds_mm"] = (vis.bounds * 1000).round(1).tolist()
for fl in fingers:
    res.setdefault("finger_link_origin_mm", {})[fl] = (T(fl)[1] * 1000).round(1).tolist()
(HERE / "config/park_stand_check.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res, ensure_ascii=False, indent=1))
