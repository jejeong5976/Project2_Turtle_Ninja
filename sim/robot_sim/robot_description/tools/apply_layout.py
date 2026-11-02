#!/usr/bin/env python3
"""스워브 배포본 URDF(6e1a…, apply_swerve.py 결과) → 배치 수정판 (2026-10-27).
build_release.py → apply_swerve.py → **apply_layout.py** → isaac_import.py → apply_initial_pose.py 순서로 실행.

바꾸는 것 (값과 근거는 config/layout_20261027.yaml):
  1) 라이다판(bottom_plate_link)과 앞·뒤 라이다를 같은 양만큼 위로 — 바닥 기준 판 밑면 실측 20 mm.
     base_scan_* 오프셋(라이다 링크 → 스캔면) — 바닥 기준 스캔면 실측 60 mm.
     base_link·바퀴·다른 판은 그대로(로봇 전체를 옮기지 않음).
  2) 보드(OpenCR 2개·Jetson·imu_link·라이다↔Jetson 연결 보드): config 의 boards 항목 그대로.
배터리판(base_link 메시 안)은 실측 80 mm와 모순이 있어 바꾸지 않음 → config 의 open_items 참고.
입력 해시가 다르면 거부. 실행: python3 tools/apply_layout.py"""
import hashlib, re, sys
from pathlib import Path
import yaml

HERE = Path(__file__).resolve().parents[1]
U = HERE / "urdf/robot.urdf"
SRC_SHA = "6e1a9b437827e29f88c2ef609ea13e9a0b43ccc89825e92164c3515aaa0de423"
MARK = "layout_release_20261027"
src = U.read_text()
sha = hashlib.sha256(src.encode()).hexdigest()
if sha != SRC_SHA:
    sys.exit("이미 적용됨" if MARK in src else f"입력 URDF 해시가 스워브 배포본과 다름: {sha}")
cfg = yaml.safe_load((HERE / "config/layout_20261027.yaml").read_text())
out = src
log = []


def set_origin(joint, xyz=None, rpy=None):
    """Replace <origin .../> inside <joint name="joint">…</joint>; returns old (xyz, rpy)."""
    global out
    m = re.search(r'(<joint name="%s"[^>]*>.*?<origin xyz=")([^"]*)(" rpy=")([^"]*)(" />)' % re.escape(joint), out, re.S)
    if not m:
        sys.exit("관절 없음: " + joint)
    old = (m.group(2), m.group(4))
    new_xyz = xyz if xyz is not None else old[0]
    new_rpy = rpy if rpy is not None else old[1]
    out = out[:m.start()] + m.group(1) + new_xyz + m.group(3) + new_rpy + m.group(5) + out[m.end():]
    log.append({"joint": joint, "xyz": [old[0], new_xyz], "rpy": [old[1], new_rpy]})
    return old


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


# ---- 1) lidar deck + lidars + scan planes
L = cfg["lidar"]
ground = L["ground_z_in_base_m"]                       # wheel contact in base_link frame
dz = (L["measured_plate_underside_ground_m"] + ground) - L["model_plate_underside_base_m"]
set_origin("bottom_plate_mount", xyz=fmt([0, 0, dz]))
for j in ("lidar_front_mount", "lidar_rear_mount"):
    m = re.search(r'<joint name="%s".*?<origin xyz="([^"]*)"' % j, out, re.S)
    x, y, z = map(float, m.group(1).split())
    set_origin(j, xyz=fmt([x, y, z + dz]))
mount_z = L["model_lidar_mount_base_m"] + dz            # lidar link origin = plate top
scan_off = (L["measured_scan_plane_ground_m"] + ground) - mount_z
for j in ("base_scan_front_fixed", "base_scan_rear_fixed"):
    set_origin(j, xyz=fmt([0, 0, scan_off]))

# ---- 1b) battery plate: move only its connected mesh components (+ battery) so the plate TOP matches the measurement
B = cfg["battery_plate"]
bdz = (B["measured_top_ground_m"] + ground) - B["model_top_base_m"]
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
lines = (HERE / B["mesh_in"]).read_text().splitlines()
vidx = [i for i, l in enumerate(lines) if l.startswith("v ")]
V = np.array([[float(x) for x in lines[i].split()[1:4]] for i in vidx])
F = np.array([[int(x.split("/")[0]) - 1 for x in l.split()[1:4]] for l in lines if l.startswith("f ")])
g = coo_matrix((np.ones(len(F) * 2), (np.r_[F[:, 0], F[:, 1]], np.r_[F[:, 1], F[:, 2]])), shape=(len(V), len(V)))
nc, lab = connected_components(g, directed=False)
zmin = np.full(nc, np.inf); zmax = np.full(nc, -np.inf)
np.minimum.at(zmin, lab, V[:, 2]); np.maximum.at(zmax, lab, V[:, 2])
lo, hi = B["component_z_band_base_m"]
sel = np.where((zmin >= lo) & (zmax <= hi))[0]
if len(sel) != B["expected_components"]:
    sys.exit(f"배터리판 조각 수가 예상과 다름: {len(sel)}")
move = np.isin(lab, sel)
for k in np.where(move)[0]:
    x, y, z = V[k]
    lines[vidx[k]] = f"v {x:.12g} {y:.12g} {z + bdz:.12g}"
(HERE / B["mesh_out"]).write_text("\n".join(lines) + "\n")
n_ref = out.count(B["mesh_in"])
out = out.replace(B["mesh_in"], B["mesh_out"])
m = re.search(r'<joint name="%s".*?<origin xyz="([^"]*)"' % B["battery_joint"], out, re.S)
bx, by, bz = map(float, m.group(1).split())
set_origin(B["battery_joint"], xyz=fmt([bx, by, bz + bdz]))
log.append({"battery_plate": {"dz_m": round(bdz, 6), "components": int(len(sel)), "vertices_moved": int(move.sum()), "urdf_mesh_refs": n_ref}})

# ---- 2) boards
for b in cfg.get("boards", []):
    xyz, rpy = fmt(b["xyz"]), fmt(b.get("rpy", [0, 0, 0]))
    if b.get("joint"):
        set_origin(b["joint"], xyz=xyz, rpy=rpy)
    elif b.get("visual"):              # base_link visual whose mesh vertices are already in base_link frame
        tag = '<visual name="%s">' % b["visual"]
        if out.count(tag) != 1:
            sys.exit("visual 없음/중복: " + b["visual"])
        out = out.replace(tag, tag + f'\n      <origin xyz="{xyz}" rpy="{rpy}" />', 1)
        log.append({"visual": b["visual"], "origin": [xyz, rpy]})
    elif b.get("new_link"):            # display-only marker: visual box, no inertial, no collision
        n = b["new_link"]
        block = (f'  <link name="{n}">\n    <visual name="{n}__visual">\n      <geometry><box size="{fmt(b["box"])}" /></geometry>\n'
                 f'      <material name="{n}_color"><color rgba="0.1 0.6 0.2 1.0" /></material>\n    </visual>\n  </link>\n'
                 f'  <joint name="{n}_mount" type="fixed">\n    <parent link="base_link" />\n    <child link="{n}" />\n'
                 f'    <origin xyz="{xyz}" rpy="{rpy}" />\n  </joint>\n')
        out = out.replace("</robot>", block + "</robot>", 1)
        log.append({"new_link": n, "origin": [xyz, rpy]})

# ---- 3a) finger collision pieces (collision only; visual unchanged)
fc = cfg.get("finger_collision")
if fc:
    import json as _json
    spec = _json.loads((HERE / fc["file"]).read_text())
    for link, info in spec["links"].items():
        a = out.index('<link name="%s"' % link); b = out.index("</link>", a)
        blk = out[a:b]
        n_old = blk.count("<collision")
        blk = re.sub(r"\s*<collision\b.*?</collision>", "", blk, flags=re.S)
        add = "".join(f'\n    <collision name="{link}__collision__coacd_{k:03d}">\n      <geometry>\n        <mesh filename="package://nt_release_description/{f}" scale="1 1 1" />\n      </geometry>\n    </collision>'
                      for k, f in enumerate(info["pieces"]))
        blk = blk.rstrip() + add + "\n  "
        out = out[:a] + blk + out[b:]
        log.append({"finger_collision": link, "old": n_old, "new": len(info["pieces"])})

# ---- 3b) gripper limits in model metal-gap terms (pad-to-pad measurement + 2 × pad)
G = cfg.get("gripper")
if G:
    padmm = G["pad_per_side_m"] * 1000
    import math
    # round OUTWARD to 1e-6 m (same as the original release −0.021254 / 0.010246) so the exact 16/79 mm values stay inside
    lo = math.floor((G["closed_pad_gap_mm"] + 2 * padmm - G["gap0_mm"]) / 2000 * 1e6) / 1e6
    hi = math.ceil((G["open_pad_gap_mm"] + 2 * padmm - G["gap0_mm"]) / 2000 * 1e6) / 1e6
    for jn in G["joints"]:
        pat = r'(<joint name="%s".*?<limit lower=")[^"]*(" upper=")[^"]*(")' % re.escape(jn)
        out, n = re.subn(pat, lambda m: m.group(1) + f"{lo:.6f}" + m.group(2) + f"{hi:.6f}" + m.group(3), out, count=1, flags=re.S)
        if n != 1:
            sys.exit("손가락 관절 한계 없음: " + jn)
        log.append({"gripper_limit": jn, "lower": round(lo, 7), "upper": round(hi, 7)})
PS = cfg.get("park_stand_collision", {})
if PS.get("mount_joint"):
    m = re.search(r'<joint name="%s".*?<origin xyz="([^"]*)"' % PS["mount_joint"], out, re.S)
    x, y, z = map(float, m.group(1).split()); d = PS["mount_shift_m"]
    set_origin(PS["mount_joint"], xyz=fmt([x + d[0], y + d[1], z + d[2]]))

# ---- 3) park stand collision walls (collision only)
for w in PS.get("walls", []):
    pat = r'(<collision name="%s"><origin xyz=")[^"]*(" rpy="[^"]*"/><geometry><box size=")[^"]*("/>)' % re.escape(w["name"])
    out, n = re.subn(pat, lambda m: m.group(1) + fmt(w["xyz"]) + m.group(2) + fmt(w["size"]) + m.group(3), out)
    if n != 1:
        sys.exit("받침 벽 충돌 없음: " + w["name"])
    log.append({"collision": w["name"], "xyz": w["xyz"], "size": w["size"]})

out = out.replace("<robot ", f"<!-- {MARK}: tools/apply_layout.py, config/layout_20261027.yaml -->\n<robot ", 1)
U.write_text(out)
print("lidar dz %.4f m, scan offset %.4f m" % (dz, scan_off))
for e in log:
    print(e)
print("sha256", hashlib.sha256(out.encode()).hexdigest())
