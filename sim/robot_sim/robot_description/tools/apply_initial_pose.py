#!/usr/bin/env python3
"""Isaac USD 에 주행 기본자세(parked, 그리퍼 16 mm)를 초기값으로 적음 — isaac_import.py 다음에 실행.
URDF 관절 영점·한계·관절 종류는 그대로(팔·그리퍼는 계속 움직일 수 있는 관절).
  - 팔 5개: PhysicsJointStateAPI 'state:angular:physics:position'(도) + 'drive:angular:physics:targetPosition'(도)
  - 손가락 a·b 둘 다: 'state:linear:physics:position'(m) + 'drive:linear:physics:targetPosition'(m)
    (b 는 NewtonMimicAPI 구속이라 상태가 자동 복사되지 않음 → 둘 다 적음)
  - 속도 0, 루트 프림에 Physics 변형 기본 선택 = physx (선택 없이 열면 관절이 없어서)
값: config/initial_pose.json 'positions'(A9F5 토크 OFF 기록) + 손가락 −0.021253619 m = (16 − 58.507)/2000.
Isaac Lab 은 USD 관절 상태를 읽지 않음 → config/isaac_lab_init_state.py 의 init_state 를 같이 써야 리셋도 parked.
실행: /home/pa25/env_isaacsim62/bin/python tools/apply_initial_pose.py  (pxr 만 사용, 시뮬레이터 안 띄움)"""
import json, math, sys
from pathlib import Path
from pxr import Usd, UsdPhysics, Sdf

HERE = Path(__file__).resolve().parents[1]
ROOT_USD = HERE / "usd/robot/robot.usda"
PHYS = HERE / "usd/robot/payloads/Physics/physics.usda"
pose = json.loads((HERE / "config/initial_pose.json").read_text())
q = dict(pose["positions"])
import yaml
G = yaml.safe_load((HERE / "config/layout_20261027.yaml").read_text())["gripper"]
q["gripper_finger_a_joint"] = q["gripper_finger_b_joint"] = (G["closed_pad_gap_mm"] + 2000 * G["pad_per_side_m"] - G["gap0_mm"]) / 2000   # 패드 16 mm = 금속 16 + 2·패드

layer = Sdf.Layer.FindOrOpen(str(PHYS))
st = Usd.Stage.Open(layer)
done = {}
for p in st.TraverseAll():   # joints live under an "over" parent in this payload layer
    name = p.GetName()
    if name not in q:
        continue
    if p.IsA(UsdPhysics.RevoluteJoint):
        kind, val = "angular", math.degrees(q[name])
    elif p.IsA(UsdPhysics.PrismaticJoint):
        kind, val = "linear", q[name]
    else:
        sys.exit(f"관절 종류 예상 밖: {p.GetPath()} {p.GetTypeName()}")
    j = UsdPhysics.RevoluteJoint(p) if kind == "angular" else UsdPhysics.PrismaticJoint(p)
    lo, hi = j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()
    if not (lo - 1e-9 <= val <= hi + 1e-9):
        sys.exit(f"{name}: 초기값 {val} 이 한계 [{lo}, {hi}] 밖")
    p.CreateAttribute(f"state:{kind}:physics:position", Sdf.ValueTypeNames.Float).Set(val)
    p.CreateAttribute(f"state:{kind}:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)
    if "PhysicsJointStateAPI:" + kind not in p.GetAppliedSchemas():
        p.AddAppliedSchema("PhysicsJointStateAPI:" + kind)
    drv = UsdPhysics.DriveAPI.Get(p, kind)
    if drv:
        drv.CreateTargetPositionAttr().Set(val)
        drv.CreateTargetVelocityAttr().Set(0.0)
    done[name] = (kind, val, bool(drv))
missing = set(q) - set(done)
if missing:
    sys.exit(f"USD 에서 못 찾은 관절: {missing}")
layer.Save()

root = Sdf.Layer.FindOrOpen(str(ROOT_USD))
rst = Usd.Stage.Open(root)
prim = rst.GetDefaultPrim()
vs = prim.GetVariantSets()
if "Physics" in vs.GetNames():
    vs.GetVariantSet("Physics").SetVariantSelection("physx")
    root.Save()
for k, v in done.items():
    print(f"{k:24s} {v[0]:7s} {v[1]: .6f} drive_target={'yes' if v[2] else 'no drive'}")
print("Physics variant default:", vs.GetVariantSet("Physics").GetVariantSelection() if "Physics" in vs.GetNames() else "없음")
