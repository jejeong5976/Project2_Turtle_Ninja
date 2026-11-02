#!/usr/bin/env python3
"""Isaac Sim 6.2 RTX 보기용 렌더 (Codex render_no_pins.py 방식). 시뮬레이션 실행 없음 — 링크마다 FK 자세를 직접 놓고 찍음.
  import : 배포 URDF → USD (usd/ 에 새로 만듦)
  images : P0 · 받침 자세 · 관절 범위 이미지
  video RUN : 실물 실행 기록(03_session.jsonl)을 10 fps 로 재생한 영상
실행: /home/pa25/env_isaacsim62/bin/python tools/isaac_render.py all ARM_20261026T023249Z"""
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parents[1]
URDF = OUT / "urdf/robot.urdf"
JM = json.loads((OUT / "config/joint_map.json").read_text())
POSES = json.loads((OUT / "config/initial_pose.json").read_text())["poses"]
RUNS = "/home/pa25/Documents/Codex/2026-10-20/task-3/arm_staged_trial_claude_20261021/runs"
W, H = 1400, 1050
NAVY, FOOT = (14, 20, 30), (238, 242, 246)


def origin(e):
    t = np.eye(4)
    if e is not None:
        t[:3, 3] = np.fromstring(e.get("xyz", "0 0 0"), sep=" ")
        t[:3, :3] = Rotation.from_euler("xyz", np.fromstring(e.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return t


def fk(robot, values):
    world = {"base_link": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        for j in pending[:]:
            parent = j.find("parent").get("link")
            if parent not in world:
                continue
            a = np.eye(4)
            v = values.get(j.get("name"), 0.0)
            mm = j.find("mimic")
            if mm is not None:
                v = values.get(mm.get("joint"), 0.0) * float(mm.get("multiplier", "1")) + float(mm.get("offset", "0"))
            if j.get("type") != "fixed":
                ax = np.fromstring(j.find("axis").get("xyz"), sep=" ")
                ax /= np.linalg.norm(ax)
                if j.get("type") == "prismatic":
                    a[:3, 3] = ax * v
                else:
                    a[:3, :3] = Rotation.from_rotvec(ax * v).as_matrix()
            world[j.find("child").get("link")] = world[parent] @ origin(j.find("origin")) @ a
            pending.remove(j)
    return world


def q_from_ticks(tk, lo_grip):
    q = {}
    for j in JM["arm"]:
        t = tk.get(str(j["id"]))
        if t is not None:
            q[j["joint"]] = math.radians(j["q_deg_at_ref"]) + j["sign"] * (t - j["ref_tick"]) * 2 * math.pi / 4096
    g = tk.get("15")
    if g is not None:
        gp = JM["gripper"]["gap_table_tick_mm"]
        gap = float(np.interp(g, [a for a, _ in gp], [b for _, b in gp]))
        q["gripper_finger_a_joint"] = max(lo_grip, (gap - JM["gripper"]["gap0_mm"]) / 2000)
    return q


def main():
    mode = sys.argv[1]
    run = sys.argv[2] if len(sys.argv) > 2 else None
    usd_dir = OUT / "usd"
    rt = Path("/tmp") / "nt_release_isaac_runtime"
    rt.mkdir(exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "multi_gpu": False, "width": W, "height": H, "limit_cpu_threads": 4,
                         "extra_args": ["--/app/extensions/registryEnabled=false", "--/app/enableTelemetry=false", "--/app/settings/persistent=false",
                                        "--portable-root=" + str(rt)]})
    try:
        robot = ET.parse(URDF).getroot()
        joints = {j.get("name"): j for j in robot.findall("joint")}
        lo_grip = float(joints["gripper_finger_a_joint"].find("limit").get("lower"))
        usd = next(iter(sorted(usd_dir.rglob("*.usda"))), None) if usd_dir.exists() else None
        if usd is None or mode in ("import", "all"):
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension("isaacsim.asset.importer.urdf")
            from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
            st_ = json.loads((OUT / "config/isaac_import_fixed.json").read_text())
            usd_dir.mkdir(exist_ok=True)
            st_.update(urdf_path=str(URDF), usd_path=str(usd_dir), ros_package_paths=[{"name": "nt_release_description", "path": str(OUT)}])
            usd = Path(URDFImporter(URDFImporterConfig(**st_)).import_urdf())
            print("USD", usd, flush=True)
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image, ImageDraw, ImageFont
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        fb = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 28)
        fr = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 21)
        ctx = omni.usd.get_context()
        ctx.open_stage(str(usd))
        for _ in range(5):
            app.update()
        st = ctx.get_stage()
        st.SetEditTarget(st.GetSessionLayer())
        root = st.GetDefaultPrim()
        vs = root.GetVariantSet("Physics")
        vs.SetVariantSelection("physics")
        mapping = {"base_link": str(root.GetPath()) + "/Geometry/base_link"}
        for prim in st.Traverse():
            if prim.GetName() in joints and prim.IsA(UsdPhysics.Joint):
                tg = UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()
                if tg:
                    mapping[joints[prim.GetName()].find("child").get("link")] = str(tg[0])
        vs.SetVariantSelection("none")
        ops = {}
        for n, path in mapping.items():
            xf = UsdGeom.Xformable(st.GetPrimAtPath(path))
            xf.ClearXformOpOrder()
            xf.SetResetXformStack(True)
            ops[n] = xf.AddTransformOp(opSuffix="displayPose")
        UsdLux.DomeLight.Define(st, "/ReviewLight").CreateIntensityAttr(1000)
        g = UsdGeom.Cube.Define(st, "/ReviewGround")
        g.CreateSizeAttr(1)
        gx = UsdGeom.Xformable(g)
        gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.084))
        gx.AddScaleOp().Set(Gf.Vec3f(3, 3, 0.02))
        cam = UsdGeom.Camera.Define(st, "/ReviewCamera")
        cam.CreateFocalLengthAttr(20)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(20.955 * H / W)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.001, 100))
        cop = UsdGeom.Xformable(cam).AddTransformOp()
        rp = rep.create.render_product("/ReviewCamera", (W, H))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])

        def pose(q):
            w = fk(robot, q)
            for n, op in ops.items():
                op.Set(Gf.Matrix4d(w[n].T.tolist()))

        def snap(eye, target, sub=8):
            cop.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
            for _ in range(sub):
                rep.orchestrator.step(rt_subframes=2, pause_timeline=True, delta_time=0)
            return Image.fromarray(rgb.get_data()).convert("RGB")

        def label(im, t1, t2, t3=""):
            d = ImageDraw.Draw(im)
            d.rectangle((0, 0, W, 86), fill=NAVY)
            d.text((20, 8), t1, font=fb, fill="white")
            d.text((20, 48), t2, font=fr, fill=(255, 210, 100))
            if t3:
                d.rectangle((0, H - 44, W, H), fill=(14, 20, 30))
                d.text((20, H - 38), t3, font=fr, fill=(220, 225, 230))
            return im

        img = OUT / "images"
        img.mkdir(exist_ok=True)
        ISO, ISO_T = (0.90, 0.72, 0.72), (0.03, 0, 0.24)
        if mode in ("images", "all"):
            p0 = dict(POSES["C1_P0"]["q_rad"])
            p0.update({"gripper_finger_a_joint": -0.021253618998087663, "gripper_finger_b_joint": -0.021253618998087663})
            pose(p0)
            label(snap(ISO, ISO_T), "닌자터틀 배포 URDF · 선택 작업 대기 자세 P0 (기본 표시: parked)",
                  "퍽 홀더 6칸 최신 배치(10-25) + 오늘 실측: 관절 범위·속도·힘·그리퍼·받침대·총질량 3.9 kg",
                  "nt_release_description / urdf/robot.urdf").save(img / "release_p0.png")
            label(snap((-0.75, -0.85, 0.75), (0.0, 0.0, 0.22)), "준비 자세 P0 · 뒤쪽에서", "프로파일 기둥·버드아이 카메라·홀더 6칸").save(img / "release_p0_back.png")
            pk = dict(json.loads((OUT / "config/initial_pose.json").read_text())["positions"])
            pk["gripper_finger_b_joint"] = pk["gripper_finger_a_joint"]
            pose(pk)
            label(snap((0.42, 0.30, 0.30), (0.09, 0.0, 0.13)), "받침 자세 · 실측 받침대 50×53×33 mm",
                  "기본 표시 parked·개도16mm / 장착 수정 후 잔여 벽 교차와 물리 초기화 미검증").save(img / "release_park.png")
            for name, lab in [("arm_joint1", "J1 몸통"), ("arm_joint2", "J2 어깨"), ("arm_joint3", "J3 팔꿈치"), ("arm_joint4", "J4 손목"), ("arm_joint5", "J5 손목 회전")]:
                lim = joints[name].find("limit")
                for tag, v in (("min", float(lim.get("lower"))), ("max", float(lim.get("upper")))):
                    q = dict(p0)
                    q[name] = v
                    pose(q)
                    label(snap(ISO, ISO_T, 6), f"관절 범위 · {lab} {math.degrees(v):+.1f}°", "나머지 관절은 P0 · 손 측정(프로파일 장착)+실제 실행+엔코더 창").save(img / f"range_{name}_{tag}.png")
            print("images done", flush=True)
        if mode in ("video", "all") and run:
            rows, last = [], -1e9
            for l in open(f"{RUNS}/{run}/03_session.jsonl"):
                if '"event":"snapshot"' not in l or '"state":"REPLAY' not in l:
                    continue
                x = json.loads(l)
                if x["monotonic_ms"] - last >= 100:
                    last = x["monotonic_ms"]
                    rows.append((x["monotonic_ms"], {k: v["pos"] for k, v in x["ids"].items()}))
            fr_dir = OUT / "images" / f"_frames_{run}"
            fr_dir.mkdir(exist_ok=True)
            t0 = rows[0][0]
            for k, (t, tk) in enumerate(rows):
                pose(q_from_ticks(tk, lo_grip))
                label(snap(ISO, ISO_T, 4), f"실물 실행 재생 · A9F5 상자 왼쪽 끝 퍽 → 적재함 · {(t - t0) / 1000:5.1f} s",
                      f"실제 모터 기록을 배포 URDF 에 그대로 · J1~J5 {tk['11']}/{tk['12']}/{tk['13']}/{tk['14']}/{tk['16']} · 그리퍼 {tk['15']}",
                      "1배속 · 관절 각도 = config/joint_map.json").save(fr_dir / f"f{k:05d}.png")
                if k % 25 == 0:
                    print("frame", k, len(rows), flush=True)
            out = OUT / "images" / f"release_motion_{run}.mp4"
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "10", "-i", str(fr_dir / "f%05d.png"), "-vf", "scale=1400:-2",
                            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", str(out)], check=True)
            print("video", out, flush=True)
        rgb.detach([rp])
        rp.destroy()
    finally:
        app.close()


if __name__ == "__main__":
    main()
