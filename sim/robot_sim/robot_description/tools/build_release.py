#!/usr/bin/env python3
"""배포용 URDF 만들기 (2026-10-26, Claude). 원본 nt_urdf_candidate_v1 은 읽기만 함.

바꾸는 것(근거는 README_KO.md 와 config/release_changes.json):
  1) 팔 관절 위치 범위 = 실측(손 측정 + 실제 실행) · 위치 모드 엔코더 창
  2) 팔 관절 velocity / effort = XM430-W350 데이터시트(12 V)
  3) 그리퍼 손가락 범위 = 실측 간격 16~79 mm
  4) 받침대 = 실측 상자 50×53×33 mm, 실측 위치
  5) 총질량 3.9 kg 실측 → 차이를 base_link 에 배분(질량중심·관성 비율 유지)
  6) 패키지 이름 nt_release_description, 참조하는 메시만 복사
"""
import hashlib
import json
import math
import re
import shutil
from pathlib import Path

LATEST = Path("/home/pa25/Documents/Codex/2026-10-23/task-2/audit_work/holders6_20261024/inside_plate_proposal/side_columns_adjustment/full_review_h4_followup/candidate")   # 2026-10-25 최신(퍽 홀더 6칸 재배치 + 손가락 충돌 정밀화)
SRC_URDF = LATEST / "simulation/robot_sim_candidate.urdf"
V1 = Path("/home/pa25/Documents/Codex/2026-10-23/nt_urdf_candidate_v1")   # 최신 폴더에 없는 같은 체크섬 자산은 여기서
SRC = LATEST
OUT = Path(__file__).resolve().parents[1]
INP = Path("/home/pa25/Documents/Codex/2026-10-23/task/model_inputs_20261026")
SRC_PKG, PKG = "nt_candidate_description", "nt_release_description"
SRC_SHA = "7d482f2162004e786269409e84a9ca510d6eec4c2ef3227d03defd22e728de1d"

V_MAX = 46 * 2 * math.pi / 60      # XM430-W350 무부하 46 rpm @12 V
T_MAX = 4.1                         # XM430-W350 멈춤 토크 4.1 N·m @12 V
GAP0, GAP_MIN, GAP_MAX = 58.50723799617533, 16.0, 79.0   # 손가락 q = (간격 − GAP0)/2000 m (손가락마다)
STAND = dict(sx=0.053, sy=0.050, h=0.033, wall=0.003, floor=0.006, cx=0.1003, cy=0.0, z0=0.0915)
MASS_TOTAL = 3.9


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def set_limit(s, joint, lower, upper, effort, velocity):
    pat = re.compile(r'(<joint name="%s"[^>]*>.*?)<limit [^>]*/>' % re.escape(joint), re.S)
    new = f'<limit lower="{lower:.6f}" upper="{upper:.6f}" effort="{effort:g}" velocity="{velocity:.4f}" />'
    s2, n = pat.subn(lambda m: m.group(1) + new, s, count=1)
    assert n == 1, joint
    return s2



def orient_fixed_j1_body(s):
    """Apply the user-approved housing correction in the local output-shaft frame.

    Input is the pinned, unflipped CAD source (SRC_SHA). Rotate the
    arm_base_link visual/collision/inertial frames, then derive the mount X
    from user-confirmed housing direction and the measured 40mm face gap.
    The inertia tensor remains in its rotated inertia frame; mass is unchanged.
    """
    import xml.etree.ElementTree as ET
    pattern = re.compile(r'<link name="arm_base_link">.*?</link>', re.S)
    match = pattern.search(s)
    assert match is not None
    link = ET.fromstring(match.group(0))
    for tag in ("visual", "collision", "inertial"):
        for element in link.findall(tag):
            origin = element.find("origin")
            if origin is None:
                origin = ET.Element("origin")
                element.insert(0, origin)
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            assert rpy == [0, 0, 0], "Unexpected source J1 body orientation; inspect before rebuilding"
            origin.set("xyz", f"{-xyz[0]:.12g} {-xyz[1]:.12g} {xyz[2]:.12g}")
            origin.set("rpy", f"0 0 {math.pi:.16g}")
    ET.indent(link, space="  ", level=1)
    return s[:match.start()] + ET.tostring(link, encoding="unicode").rstrip() + s[match.end():]


def main():
    assert sha(SRC_URDF) == SRC_SHA, "원본 URDF 해시가 다름 — 중단"
    s = SRC_URDF.read_text()
    rng = json.loads((INP / "joint_range_20261026.json").read_text())["joints"]
    changes = {"source_urdf": str(SRC_URDF), "source_sha256": SRC_SHA, "joints": {}}

    # 1·2) 팔 관절
    jm = {j["joint"]: j for j in json.loads((OUT / "config/joint_map.json").read_text())["arm"]}
    for name, v in rng.items():
        j = jm[name]
        ticks = v.get("controllable_tick_mode3") or v["combined_tick"]
        qs = sorted(math.radians(j["q_deg_at_ref"]) + j["sign"] * (t - j["ref_tick"]) * 2 * math.pi / 4096 for t in ticks)
        lo, hi = math.floor(qs[0] * 1e4) / 1e4, math.ceil(qs[1] * 1e4) / 1e4   # 틱 끝값을 바깥쪽 0.0001 rad 로 반올림(넓히지 않음 수준)
        deg = [round(math.degrees(lo), 2), round(math.degrees(hi), 2)]
        s = set_limit(s, name, lo, hi, T_MAX, V_MAX)
        om = re.search(r'<joint name="%s".*?<limit lower="([^"]+)" upper="([^"]+)"' % name, SRC_URDF.read_text(), re.S)
        old = [round(math.degrees(float(om.group(1))), 1), round(math.degrees(float(om.group(2))), 1)]
        changes["joints"][name] = {"limit_deg": deg, "from_ticks": ticks, "limit_rad": [round(lo, 6), round(hi, 6)], "old_urdf_deg": old,
                                   "basis": "손 측정(POSE_20261026T024707Z·025459Z) + 실제 실행 틱" + (" · 위치 모드 엔코더 32~4063" if v.get("controllable_deg") else ""),
                                   "effort_Nm": T_MAX, "velocity_rad_s": round(V_MAX, 4), "effort_velocity_basis": "XM430-W350 데이터시트 12 V(무부하 46 rpm, 멈춤 4.1 N·m)"}
    # 3) 그리퍼
    # 사용자 실측 16~79mm를 기구 표시 범위로 반영. 충돌 정합/접촉 안정성은 별도 검사하며 충돌 제외하지 않음.
    qlo, qhi = (GAP_MIN - GAP0) / 2000, (GAP_MAX - GAP0) / 2000
    for j in ("gripper_finger_a_joint", "gripper_finger_b_joint"):
        m = re.search(r'<joint name="%s".*?<limit [^>]*effort="([^"]+)" velocity="([^"]+)"' % j, s, re.S)
        s = set_limit(s, j, qlo, qhi, float(m.group(1)), float(m.group(2)))
    changes["gripper"] = {"finger_limit_m": [round(qlo, 6), round(qhi, 6)], "gap_mm": [GAP_MIN, GAP_MAX],
                          "basis": "사용자 실측 패드 간격16~79mm; q=(gap-58.50723799617533)/2000. 하드웨어 제한/틱 지도 변경 없음. 모델 접촉 정합은 미완료.", "old_m": [-0.0194393656, 0.00649013437],
                          "effort_velocity": "원본 유지(미측정)"}

    # 4) 받침대
    st = STAND
    def box(n, x, y, z, a, b, c):
        return (f'<collision name="gripper_park_stand_TEMP__collision__{n}"><origin xyz="{x:.5f} {y:.5f} {z:.5f}" rpy="0 0 0"/>'
                f'<geometry><box size="{a:.5f} {b:.5f} {c:.5f}"/></geometry></collision>')
    wz, wh = st["floor"] + (st["h"] - st["floor"]) / 2, st["h"] - st["floor"]
    cols = (box("floor", 0, 0, st["floor"] / 2, st["sx"], st["sy"], st["floor"]) + box("wall0", st["sx"] / 2 - st["wall"] / 2, 0, wz, st["wall"], st["sy"], wh)
            + box("wall1", -st["sx"] / 2 + st["wall"] / 2, 0, wz, st["wall"], st["sy"], wh)
            + box("wall2", 0, st["sy"] / 2 - st["wall"] / 2, wz, st["sx"] - 2 * st["wall"], st["wall"], wh)
            + box("wall3", 0, -st["sy"] / 2 + st["wall"] / 2, wz, st["sx"] - 2 * st["wall"], st["wall"], wh))
    pat = re.compile(r'(<link name="gripper_park_stand_TEMP">).*?(</link>)', re.S)
    vis = (f'<visual name="gripper_park_stand__visual__mani_home"><origin xyz="0 0 {st["h"] / 2:.5f}" rpy="0 0 0"/><geometry>'
           f'<mesh filename="package://{SRC_PKG}/meshes/added/mani_home.stl" scale="{st["sx"] / 50:.6f} {st["sy"] / 50:.6f} {st["h"] / 30:.6f}"/></geometry>'
           f'<material name="mani_home_mat"><color rgba="0.45 0.55 0.7 0.9"/></material></visual>')
    s, n = pat.subn(lambda m: m.group(1) + "<!--2026-10-26 실측 상자 50(좌우)×53(앞뒤)×33 mm, 바닥이 상판 윗면-->" + vis + cols + m.group(2), s)
    assert n == 1
    s, n = re.subn(r'(<child link="gripper_park_stand_TEMP" />\s*<origin xyz=")[^"]*', lambda m: m.group(1) + f'{st["cx"]} {st["cy"]} {st["z0"]}', s)
    assert n == 1
    changes["park_stand"] = {**st, "basis": "사용자 사진 실측(stand_photos_20261026): 앞면이 상판 앞 돌기 끝에서 약 6 mm 안쪽, 좌우 가운데(±10 mm)",
                             "old": "50×50×30 mm, 중심 (0.086, 0, 0.1065)",
                             "known_issue": "받침 자세에서 모델 집게가 로봇 쪽 벽을 1~2 cm 파고듦(실물은 벽 테두리에 기대는 정상 접촉). 충돌 제외로 숨기지 말 것. 수정 장착/16mm에서 잔여 앞벽 교차가 있어 물리 초기화 미검증"}

    # 5) 총질량
    masses = [(m.start(), float(m.group(1))) for m in re.finditer(r'<mass value="([^"]+)"', s)]
    total = sum(v for _, v in masses)
    blk = re.search(r'<link name="base_link">.*?</link>', s, re.S)
    inr = re.search(r'<inertial>.*?</inertial>', blk.group(0), re.S).group(0)
    m0 = float(re.search(r'<mass value="([^"]+)"', inr).group(1))
    m1 = m0 + (MASS_TOTAL - total)
    k = m1 / m0
    inr2 = re.sub(r'<mass value="[^"]+"', f'<mass value="{m1:.6f}"', inr)
    inr2 = re.sub(r'(i(?:xx|xy|xz|yy|yz|zz))="([^"]+)"', lambda mm: f'{mm.group(1)}="{float(mm.group(2)) * k:.9g}"', inr2)
    s = s.replace(blk.group(0), blk.group(0).replace(inr, inr2), 1)
    changes["mass"] = {"model_total_before_kg": round(total, 6), "measured_total_kg": MASS_TOTAL, "base_link_kg": [m0, round(m1, 6)],
                       "basis": "사용자 체중계 3.9 kg(로봇 전체, 홀더의 퍽 6개 포함 — 모델의 test_puck 6개와 같은 조건). 차이는 출처 미확정 → 여러 부품 합산 링크인 base_link 에 배분, 질량중심 유지·관성은 질량 비율로"}

    # Remove only the six fixed example payloads AFTER the historical loaded-mass
    # reconciliation. Their mass must not be redistributed into base_link.
    import xml.etree.ElementTree as ET
    payload_tree = ET.fromstring(s)
    removed_names = [f"test_puck_H{i}" for i in range(1, 7)]
    payloads = [payload_tree.find(f"link[@name='{name}']") for name in removed_names]
    assert all(link is not None for link in payloads)
    removed_mass = sum(float(link.find("inertial/mass").get("value")) for link in payloads)
    removed_joints = []
    for joint in payload_tree.findall("joint"):
        if joint.find("child").get("link") in removed_names:
            assert joint.get("type") == "fixed"
            removed_joints.append(joint.get("name"))
        assert joint.find("parent").get("link") not in removed_names
    assert len(removed_joints) == 6
    for tag, names in (("link", removed_names), ("joint", removed_joints)):
        for name in names:
            pattern = re.compile(r'<'+tag+r' name="'+re.escape(name)+r'"[^>]*>.*?</'+tag+r'>', re.S)
            s, count = pattern.subn("", s)
            assert count == 1, name
    changes["fixed_example_pucks"] = {
        "included": False,
        "removed_links": removed_names,
        "removed_fixed_joints": removed_joints,
        "removed_payload_mass_kg": round(removed_mass, 6),
        "payload_composition": "three cuboids at17g and three cylinders at13g",
        "robot_authored_mass_sum_kg": round(MASS_TOTAL - removed_mass, 6),
        "base_link_inertial_unchanged_by_removal": True,
        "basis": "User requests robot without fixed example pucks. 3.9kg is the prior loaded measurement; 3.81kg is model payload subtraction, not a new empty-robot measurement.",
        "independent_dynamic_objects": "Not created here; spawn separately in the learning environment.",
        "holder_and_stand_preserved": True
    }
    changes["mass"]["robot_authored_mass_sum_kg"] = round(MASS_TOTAL - removed_mass, 6)
    changes["mass"]["removed_example_payload_mass_kg"] = round(removed_mass, 6)

    # 5b) 바퀴 관절 이름 = 팀 swerve_controller 규칙("steer_<모듈>_joint", "drive_<모듈>_joint"). 위치·축·범위는 그대로
    for mod in ("fl", "fr", "rl", "rr"):
        for a, b in ((f"{mod}_steer_joint", f"steer_{mod}_joint"), (f"{mod}_wheel_joint", f"drive_{mod}_joint")):
            n0 = s.count(f'"{a}"')
            assert n0 >= 1, a
            s = s.replace(f'"{a}"', f'"{b}"')
    changes["wheel_joint_names"] = {"rule": "팀 swerve_controller.cpp: steer_<fl|fr|rl|rr>_joint / drive_<..>_joint", "from": "Codex fl_steer_joint / fl_wheel_joint"}
    # Persist the previously approved fixed-housing direction after every rebuild.
    s = orient_fixed_j1_body(s)
    # Measured exterior-face gap, not shaft-to-stand-center distance.
    # CAD housing: 46.500011488 mm fore-aft; after reversal the front face
    # is 11.250005849 mm ahead of the vertical output shaft. Derived from
    # pinned CAD motor visual, retained arm_mount yaw -1.208484306590865.
    motor_front_from_axis_m = 0.0112500058488773
    measured_face_gap_m = 0.040
    stand_rear_m = STAND["cx"] - STAND["sx"] / 2
    corrected_axis_x_m = stand_rear_m - measured_face_gap_m - motor_front_from_axis_m
    mount_pattern = re.compile(r'(<joint name="arm_mount_fixed".*?<origin xyz=")([^" ]+) ([^" ]+) ([^"]+)', re.S)
    mount_match = mount_pattern.search(s)
    assert mount_match is not None
    old_axis_x_m = float(mount_match.group(2))
    s, mount_count = mount_pattern.subn(lambda m: m.group(1) + f'{corrected_axis_x_m:.12g} ' + m.group(3) + ' ' + m.group(4), s, count=1)
    assert mount_count == 1
    changes["fixed_J1_body_orientation"] = {
        "body_local_rotation_about_shaft_rad": math.pi,
        "scope": "arm_base_link rigid body reversal plus derived arm_mount_fixed X; stand and actuated joints unchanged",
        "actuated_joint_origins_axes_limits_zero_changed": False,
        "arm_mount_x_m": [old_axis_x_m, corrected_axis_x_m],
        "arm_mount_delta_x_m": corrected_axis_x_m - old_axis_x_m,
        "motor_front_from_axis_m": motor_front_from_axis_m,
        "measured_motor_front_to_stand_rear_m": measured_face_gap_m,
        "stand_rear_x_m": stand_rear_m,
        "basis_formula": "axis_x = stand_rear_x - measured_face_gap - CAD_corrected_front_offset",
        "body_center_shift_m": corrected_axis_x_m - old_axis_x_m - 0.024,
        "lateral_height_yaw_preserved": True,
        "fastener_hole_correspondence": "not independently quantified; measured exterior-face placement and confirmed orientation used",
        "basis": "User-confirmed output-shaft-forward installation and 40mm housing-face-to-stand-rear measurement; 2026-10-26 correction",
        "old_unflipped_URDF_face_gap_mm": 42.80035026080132,
        "old_face_gap_status": "superseded: calculated with incorrect housing orientation; not the corrected model",
        "derived_USD_status": "existing USD not regenerated by this URDF-only builder"
    }
    # Photo-backed housing orientation only; sensor scan frame/interface unchanged.
    import xml.etree.ElementTree as ET
    for link_name in ("lidar_front_link", "lidar_rear_link"):
        pattern = re.compile(r'<link name="' + link_name + r'">.*?</link>', re.S)
        match = pattern.search(s)
        link = ET.fromstring(match.group(0))
        assert link.find("inertial") is None, "New sensor inertia needs matching rigid transform"
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                o = e.find("origin")
                if o is None:
                    o = ET.Element("origin"); e.insert(0, o)
                x,y,z = [float(v) for v in o.get("xyz", "0 0 0").split()]
                roll,pitch,yaw = [float(v) for v in o.get("rpy", "0 0 0").split()]
                assert abs(roll)<1e-9 and abs(pitch)<1e-9
                o.set("xyz", f"{-y:.12g} {x:.12g} {z:.12g}")
                o.set("rpy", f"0 0 {yaw+math.pi/2:.16g}")
        ET.indent(link, space="  ", level=1)
        s = s[:match.start()] + ET.tostring(link, encoding="unicode").rstrip() + s[match.end():]
    # Model underside at54.117mm, OpenCR layer supporting plate top46.5mm.
    # Lower complete rigid link, preserving XY and local visual/collision relationship.
    jetson_dz = 0.0465 - 0.054117
    pattern = re.compile(r'(<joint name="jetson_planned_mount".*?<origin xyz=")([^" ]+) ([^" ]+) ([^"]+)', re.S)
    match = pattern.search(s); assert match is not None and abs(float(match.group(4)))<1e-12
    s,n = pattern.subn(lambda m:m.group(1)+m.group(2)+" "+m.group(3)+f" {jetson_dz:.12g}",s,count=1);assert n==1
    changes["lidar_housing_orientation"]={"local_visual_collision_yaw_delta_rad":math.pi/2,"world_housing_yaw_deg":{"front":180,"rear":360},"basis":"IMG_9624.jpg / IMG_9625.jpg: lateral paired ears toward exterior; single ear inward. Provided CAD ear arrangement.","mount_positions_and_scan_transforms_changed":False,"actual_device_scan_zero_confirmed":False}
    changes["jetson_height"]={"mount_dz_m":jetson_dz,"bottom_z_m":[.054117,.0465],"top_z_m":[.088883,.081266],"support_plate_top_z_m":.0465,"arm_plate_lower_z_m":.0825,"upper_vertical_clearance_m":.001234,"basis":"User requests lower placement on OpenCR third layer; mesh underside aligned to CAD plate top, not bbox centre. Ground-relative measured120mm kept separate from base_link Z46.5mm.","standoff_height":"not specified; no invented spacer","mass_inertia":"no standalone inertial on this link; no new mass or assumed inertia applied","full_mount_and_contact_validation":False}
    # 6) 패키지 이름
    s = s.replace(f"package://{SRC_PKG}/", f"package://{PKG}/")
    s = s.replace('<robot name="nt_practice_candidate_v1">', '<robot name="nt_ninja_turtle_release_20261026">', 1)
    head = ("<?xml version=\"1.0\"?>\n<!-- 닌자터틀 배포용 URDF 2026-10-26. 원본 2026-10-25 최신 후보 full_review_h4_followup (sha256 " + SRC_SHA[:16] +
            "…) + 실측 반영. 바뀐 점: config/release_changes.json, 관절 각도 ↔ 실물 모터 틱: config/joint_map.yaml -->\n")
    s = re.sub(r'^<\?xml[^>]*\?>\s*', '', s)
    (OUT / "urdf").mkdir(parents=True, exist_ok=True)
    (OUT / "urdf/robot.urdf").write_text(head + s)

    # 메시 복사(참조하는 것만)
    refs = sorted(set(re.findall(r'package://%s/([^"]+)' % PKG, s)))
    for r in refs:
        dst = OUT / r
        dst.parent.mkdir(parents=True, exist_ok=True)
        src = SRC / r if (SRC / r).exists() else V1 / r
        shutil.copy2(src, dst)
    shutil.copytree(V1 / "rviz", OUT / "rviz", dirs_exist_ok=True)
    changes["meshes_copied"] = len(refs)
    changes["output_sha256"] = sha(OUT / "urdf/robot.urdf")
    (OUT / "config").mkdir(exist_ok=True)
    (OUT / "config/release_changes.json").write_text(json.dumps(changes, ensure_ascii=False, indent=1))
    print(json.dumps({k: v for k, v in changes.items() if k != "joints"}, ensure_ascii=False, indent=1)[:1500])
    for n_, v in changes["joints"].items():
        print(n_, v["old_urdf_deg"], "→", v["limit_deg"])


if __name__ == "__main__":
    main()
