#!/usr/bin/env python3
"""Isaac USD 충돌 쌍 제외 — isaac_import.py 다음(apply_initial_pose.py 와 같이) 실행. 형상은 바꾸지 않음.
  gripper_finger_a <-> gripper_finger_b : 손가락 충돌 근사(minimal_component)가 실물보다 두꺼워 16 mm(관절 한계)에서 서로 4.8 mm 겹침
  → 시뮬에서 21.3 mm 에서 멈춤(tools/check_park_stand.py 로 확인). 실물은 관절 한계 16 mm 라 손가락끼리 닿지 않음.
  손가락–퍽·손가락–받침·손가락–다른 링크 충돌은 그대로(파지·받침 지지 유지).
실행: /home/pa25/env_isaacsim62/bin/python tools/apply_collision_filters.py"""
from pathlib import Path
from pxr import Usd, UsdPhysics, Sdf
HERE = Path(__file__).resolve().parents[1]
PHYS = HERE / "usd/robot/payloads/Physics/physics.usda"
PAIRS = [("gripper_finger_a", "gripper_finger_b")]
layer = Sdf.Layer.FindOrOpen(str(PHYS)); st = Usd.Stage.Open(layer)
bodies = {p.GetName(): p for p in st.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
for a, b in PAIRS:
    pa, pb = bodies[a], bodies[b]
    rel = UsdPhysics.FilteredPairsAPI.Apply(pa).CreateFilteredPairsRel()
    if pb.GetPath() not in rel.GetTargets():
        rel.AddTarget(pb.GetPath())
    print("filtered:", pa.GetPath(), "<->", pb.GetPath())
layer.Save()
