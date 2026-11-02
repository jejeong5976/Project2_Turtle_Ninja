"""손가락 a·b 충돌 조각을 보이는 모양(visual 메시)에서 CoACD 로 다시 만듦 → meshes/collision_finger_layout/, config/finger_collision.json.
이유: 기존 충돌(minimal_component 등)이 거칠어 parked+16 mm 에서 손가락끼리 4.8 mm, 받침 벽을 관통(ㄱ자 모서리를 볼록 하나로 채움).
조각 꼭짓점은 링크 좌표계(visual origin·scale 적용 후)라 URDF 에는 origin 없이 넣음. apply_layout.py 가 이 json 을 읽어 교체.
실행: /home/pa25/IsaacLab-course/.venv/bin/python tools/make_finger_collision.py [threshold=0.02]"""
import json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, trimesh, coacd

HERE = Path(__file__).resolve().parents[1]
TH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.02
SRC = Path("/home/pa25/Documents/Codex/2026-10-23/nt_urdf_release_20261027_swerve/urdf/robot.urdf")   # 원래 손가락 정의
OUT = HERE / "meshes/collision_finger_layout"; OUT.mkdir(exist_ok=True)
r = ET.parse(SRC).getroot()
links = {l.get("name"): l for l in r.findall("link")}


def rpyR(rr, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(rr), math.sin(rr), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


res = {"threshold": TH, "source_urdf": str(SRC), "links": {}}
for link in ("gripper_finger_a", "gripper_finger_b"):
    v = links[link].find("visual"); o = v.find("origin"); m = v.find("geometry/mesh")
    xyz = np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()])
    R = rpyR(*[float(x) for x in (o.get("rpy") if o is not None else "0 0 0").split()])
    sc = np.array([float(x) for x in (m.get("scale") or "1 1 1").split()])
    mesh = trimesh.load(m.get("filename").replace("package://nt_release_description/", str(HERE) + "/"), force="mesh")
    mesh.vertices = (R @ (mesh.vertices * sc).T).T + xyz
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=TH, max_convex_hull=int(sys.argv[2]) if len(sys.argv) > 2 else 48, preprocess_mode="auto", resolution=2000, seed=0)
    files = []
    for i, (vv, ff) in enumerate(parts):
        f = OUT / f"{link}_{i:03d}.obj"; trimesh.Trimesh(vv, ff).export(f); files.append(f"meshes/collision_finger_layout/{f.name}")
    res["links"][link] = {"visual_mesh": m.get("filename"), "pieces": files, "old_collision_count": len(links[link].findall("collision"))}
    print(link, "pieces", len(files))
(HERE / "config/finger_collision.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
