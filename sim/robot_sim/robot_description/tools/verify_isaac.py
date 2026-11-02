#!/usr/bin/env python3
"""배치 수정판 USD 를 Isaac Sim(PhysX)에서 실제로 열어 확인 — headless, 결과 config/verify_isaac.json.
USD 를 그대로 열고(Physics 기본 변형 physx), 바닥(base_link 아래 0.072 m)만 세션 레이어에 추가해 4 s 재생한 뒤:
  관절값(JointStateAPI 초기값이 재생 시 적용되는지), 그리퍼 간격, base_link 높이, 스캔면 높이(URDF 오프셋), 라이다판 밑면,
  팔·손가락 충돌체 vs Jetson 충돌 상자 AABB 겹침(정착 후 자세).
GUI 로 보려면 --gui (창 하나, 재생 상태 유지). 실행: /home/pa25/env_isaacsim62/bin/python tools/verify_isaac.py [--gui]"""
import json, math, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
GUI = "--gui" in sys.argv
from isaacsim import SimulationApp
rt = Path("/tmp") / "nt_layout_verify_runtime"; rt.mkdir(exist_ok=True)
app = SimulationApp({"headless": not GUI, "limit_cpu_threads": 4,
                     "extra_args": ["--/app/extensions/registryEnabled=false", "--/app/enableTelemetry=false", "--/app/settings/persistent=false", "--portable-root=" + str(rt)]})
import omni.usd, omni.timeline
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import numpy as np
res = {"usd": str(HERE / "usd/robot/robot.usda")}
try:
    ctx = omni.usd.get_context()
    ctx.open_stage(res["usd"])
    for _ in range(10):
        app.update()
    st = ctx.get_stage()
    st.SetEditTarget(st.GetSessionLayer())
    root = st.GetDefaultPrim()
    res["physics_variant"] = root.GetVariantSets().GetVariantSet("Physics").GetVariantSelection()
    if not any(p.IsA(UsdPhysics.Scene) for p in st.Traverse()):
        sc = UsdPhysics.Scene.Define(st, "/PhysicsScene"); sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); sc.CreateGravityMagnitudeAttr(9.81)
    g = UsdGeom.Cube.Define(st, "/GroundCheck"); g.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(g).SetScale(Gf.Vec3f(3, 3, 0.02)); UsdGeom.XformCommonAPI(g).SetTranslate(Gf.Vec3d(0, 0, -0.072 - 0.01 - 0.001))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    tl = omni.timeline.get_timeline_interface(); tl.play()
    for _ in range(5):
        app.update()
    import omni.physics.tensors as tensors
    sv = tensors.create_simulation_view("numpy")
    art_root = next(str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    av = sv.create_articulation_view(art_root)
    names = av.shared_metatype.dof_names
    q0 = av.get_dof_positions()[0].copy()
    t0 = time.time()
    for _ in range(240):
        app.update()
    q1 = av.get_dof_positions()[0].copy()
    rootpos = av.get_root_transforms()[0][:3].copy()
    pose = json.loads((HERE / "config/initial_pose.json").read_text())["positions"]
    rows = {}
    for i, n in enumerate(names):
        want = pose.get(n, pose.get("gripper_finger_a_joint") if n == "gripper_finger_b_joint" else None)
        rows[n] = {"start": round(float(q0[i]), 6), "after_4s": round(float(q1[i]), 6), "parked_target": None if want is None else round(float(want), 6)}
    res["joints"] = rows
    qa, qb = q1[names.index("gripper_finger_a_joint")], q1[names.index("gripper_finger_b_joint")]
    res["gripper_gap_mm_after_4s"] = round(58.50723799617533 + 1000 * (qa + qb), 2)
    res["base_link_world_z_m"] = round(float(rootpos[2]), 5)
    res["base_above_ground_mm"] = round((float(rootpos[2]) + 0.072) * 1000, 1)
    # scan plane: URDF chain base_link -> lidar_*_mount (z −0.043) -> base_scan (+0.031); fixed frames are merged by the importer
    res["scan_plane_above_ground_mm"] = round((float(rootpos[2]) - 0.043 + 0.031 + 0.072) * 1000, 1)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy], useExtentsHint=False)
    def aabb(prim):
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange(); return np.array(r.GetMin()), np.array(r.GetMax())
    bp = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if "bottom_plate_link" in p.GetName() and p.IsA(UsdGeom.Imageable)]
    if bp:
        lo, _ = aabb(bp[0]); res["lidar_plate_bottom_above_ground_mm"] = round((lo[2] - (rootpos[2] - 0.072)) * 1000, 1)
    jet = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.GetName().startswith("jetson_planned_link__collision")]
    arm = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI) and any(k in str(p.GetPath()) for k in ("/arm_link", "/gripper_"))]
    if jet:
        jlo, jhi = aabb(jet[0]); base_lo = rootpos[2] - 0.072
        res["jetson_collision_aabb_mm_rel_ground"] = [((jlo - [rootpos[0], rootpos[1], base_lo]) * 1000).round(1).tolist(), ((jhi - [rootpos[0], rootpos[1], base_lo]) * 1000).round(1).tolist()]
        over = []; mind = 1e9
        for p in arm:
            lo, hi = aabb(p)
            gap = np.maximum(np.maximum(jlo - hi, lo - jhi), 0)
            d = float(np.linalg.norm(gap)); mind = min(mind, d)
            if d == 0: over.append(str(p.GetPath()).split("/Geometry/")[-1])
        res["arm_colliders_checked"] = len(arm)
        res["arm_vs_jetson_aabb_overlaps"] = over
        res["arm_vs_jetson_min_aabb_gap_mm"] = round(mind * 1000, 1)
    res["note"] = "AABB 는 보수적(겹침 0 이면 실제 겹침도 없음; 겹침이 나오면 정밀 확인 필요). 받침 자세 손가락–받침 1~3 mm 겹침은 별개."
    (HERE / "config/verify_isaac.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
    print(json.dumps(res, ensure_ascii=False, indent=1))
    if GUI:
        while app.is_running():
            app.update()
finally:
    app.close()
