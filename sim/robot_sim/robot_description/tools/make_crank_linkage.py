"""그리퍼 크랭크·링크 막대 보이는 모양 움직임 자료 → config/crank_linkage.json, meshes/generated/crank_anim_*.obj (보이는 것만, 물리 무관).
크랭크(회색 틀 + 핀) 는 혼 축 a 를 중심으로 실물 모터 각도만큼 회전: 손가락 q → 간격 → 틱(실측표 역보간, 2680 쪽 닫힘) → (틱 − CAD 자세 틱)·2π/4096.
막대 a·b 는 크랭크 핀 끝을 핀에 붙이고 손가락 끝 쪽을 손가락 연결점(손가락과 같이 이동)으로 향하게 돌림. 막대 길이는 CAD(52 mm, 실측 55 mm) 그대로라
닫힘 쪽에서 손가락 끝이 몇 mm 어긋날 수 있음(보고값 rod_end_error_mm).
CAD 크랭크 축·중심은 막대 핀 끝 두 점(180° 반대)의 중점과 크랭크 틀 축으로 계산. 회전 방향은 막대 길이 오차가 작은 쪽을 고름.
실행: /home/pa25/IsaacLab-course/.venv/bin/python tools/make_crank_linkage.py"""
import json, math, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np, trimesh

HERE = Path(__file__).resolve().parents[1]
U = ET.parse(HERE / "urdf/robot.urdf").getroot()
links = {l.get("name"): l for l in U.findall("link")}
J = {j.get("name"): j for j in U.findall("joint")}
OUT = HERE / "meshes/generated"; OUT.mkdir(exist_ok=True)
G0 = 58.50723799617533
TABLE = [[1045, 79.0], [2270, 29.0], [2658, 16.0]]   # 실측 틱 → 패드 간격 mm (joint_map.json 과 같음)


def vis(key):
    for v in links["arm_link5_wrist_roll"].findall("visual"):
        if key in v.get("name"):
            o = np.array([float(x) for x in v.find("origin").get("xyz").split()])
            m = trimesh.load(HERE / v.find("geometry/mesh").get("filename").replace("package://nt_release_description/", ""), force="mesh")
            m.vertices = m.vertices + o
            return v.get("name"), m


def axisR(a, t):
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def tick_of_gap(g):   # 실측표 역보간(간격이 줄면 틱 증가), 표 밖은 끝값
    t = np.array(TABLE)[::-1]
    return float(np.interp(g, t[:, 1], t[:, 0]))


crank_name, crank = vis("gripper_crank_visual_4")
rods_name, rods = vis("gripper_crank_visual_5")
parts = sorted(rods.split(only_watertight=False), key=lambda p: -len(p.faces))
rod_parts, pin = parts[:2], parts[2]
ends = []
for p in rod_parts:
    V = p.vertices; c = V.mean(0); _, ev = np.linalg.eigh(np.cov((V - c).T)); u = ev[:, -1]; s = (V - c) @ u
    ends.append((c + u * s.min(), c + u * s.max()))
ij = min(((i, j) for i in range(2) for j in range(2)), key=lambda k: np.linalg.norm(ends[0][k[0]] - ends[1][k[1]]))
P0 = [ends[0][ij[0]], ends[1][ij[1]]]; F0 = [ends[0][1 - ij[0]], ends[1][1 - ij[1]]]
C = (P0[0] + P0[1]) / 2
_, ev = np.linalg.eigh(np.cov((crank.vertices - crank.vertices.mean(0)).T))
a = ev[:, 0]                                    # 크랭크 틀의 가장 얇은 방향 = 혼 축
a = a - (a @ (P0[0] - C)) / ((P0[0] - C) @ (P0[0] - C)) * (P0[0] - C); a /= np.linalg.norm(a)   # 핀 방향과 직교
ax = {n: np.array([float(x) for x in J[n].find("axis").get("xyz").split()]) for n in ("gripper_finger_a_joint", "gripper_finger_b_joint")}
fa = [n for n in ax][0 if (F0[0] - C) @ ax["gripper_finger_a_joint"] > 0 else 1]
rod_for = {"a": 0 if fa == "gripper_finger_a_joint" else 1}; rod_for["b"] = 1 - rod_for["a"]
L0 = [float(np.linalg.norm(F0[i] - P0[i])) for i in range(2)]
tick_cad = tick_of_gap(G0)


def solve(sign, q):
    th = sign * (tick_of_gap(G0 + 2000 * q) - tick_cad) * 2 * math.pi / 4096
    R = axisR(a, th); out = {}
    for f, n in (("a", "gripper_finger_a_joint"), ("b", "gripper_finger_b_joint")):
        i = rod_for[f]; P = C + R @ (P0[i] - C); F = F0[i] + ax[n] * q
        out[f] = (P, F, abs(np.linalg.norm(F - P) - L0[i]))
    return th, out


qs = np.linspace(-0.021254, 0.010247, 40)
err = {s: max(max(solve(s, q)[1][f][2] for f in "ab") for q in qs) for s in (1, -1)}
sign = min(err, key=err.get)
files = {}
for nm, m in (("crank", trimesh.util.concatenate([crank, pin])), ("rod_a", rod_parts[rod_for["a"]]), ("rod_b", rod_parts[rod_for["b"]])):
    f = OUT / f"crank_anim_{nm}.obj"; m.export(f); files[nm] = str(f.relative_to(HERE))
rep = {q: {f: round(solve(sign, q)[1][f][2] * 1000, 2) for f in "ab"} for q in (0.010247, 0.0, (29 - G0) / 2000, -0.021254)}
res = {"note": __doc__.splitlines()[0], "hide_visuals": [crank_name, rods_name], "meshes": files,
       "colors": {"crank": [0.62, 0.66, 0.7], "rod_a": [0.95, 0.8, 0.15], "rod_b": [0.95, 0.8, 0.15]},
       "axis": a.round(9).tolist(), "center_m": C.round(9).tolist(), "sign": sign, "tick_table_mm": TABLE, "gap0_mm": G0, "tick_cad": round(tick_cad, 2),
       "rod_pin_end_m": {f: P0[rod_for[f]].round(9).tolist() for f in "ab"}, "rod_finger_end_m": {f: F0[rod_for[f]].round(9).tolist() for f in "ab"},
       "finger_axis": {f: ax[f"gripper_finger_{f}_joint"].tolist() for f in "ab"},
       "rod_length_cad_mm": [round(x * 1000, 2) for x in L0], "crank_radius_cad_mm": round(float(np.linalg.norm(P0[0] - C)) * 1000, 2),
       "rod_end_error_mm": {f"gap {G0 + 2000 * q:.1f} mm": v for q, v in rep.items()}, "max_error_both_signs_mm": {str(k): round(v * 1000, 2) for k, v in err.items()}}
(HERE / "config/crank_linkage.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps({k: res[k] for k in ("sign", "tick_cad", "rod_length_cad_mm", "crank_radius_cad_mm", "rod_end_error_mm", "max_error_both_signs_mm")}, ensure_ascii=False, indent=1))
