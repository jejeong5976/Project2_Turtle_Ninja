#!/usr/bin/env python3
"""배포본 보기용 그림·영상 (MuJoCo 렌더, 포트·실물 없음)
  images/release_p0.png, images/release_parked.png  — 정지 그림
  images/release_views.png  — 받침 자세 6방향(정면·후면·왼쪽·오른쪽·위·비스듬히)
  images/drive_swerve_sim.mp4  — swerve/nt_swerve.py 로 MuJoCo 바닥 주행(물리) : S자 → 정면 유지 전방향 → 제자리 회전 → 회전→전진
  images/arm_replay_A9F5.mp4   — 실물 실행 기록(ARM_20261026T023249Z, 27.3 s)의 관절 값을 이 모델에 그대로(차체 고정, 물리 아님)
실행: MUJOCO_GL=egl <isaac env python> tools/render_media.py [images|drive|arm|all]"""
import json, math, re, subprocess, sys, tempfile
from pathlib import Path
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "swerve"))
from nt_swerve import ModuleState, SwerveDrive, from_config  # noqa: E402
IMG = HERE / "images"; IMG.mkdir(exist_ok=True)
FB = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 30)
FR = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 22)
kin, cfg, J = from_config(str(HERE / "config/swerve.yaml"))
IP = json.loads((HERE / "config/initial_pose.json").read_text())
JM = json.loads((HERE / "config/joint_map.json").read_text())
RUNS = "/home/pa25/Documents/Codex/2026-10-20/task-3/arm_staged_trial_claude_20261021/runs"
W, H = 1280, 720


def build():
    s = (HERE / "urdf/robot.urdf").read_text().replace("package://nt_release_description/", str(HERE) + "/")
    s = re.sub(r'(<robot name="[^"]+">)', r'\1<mujoco><compiler discardvisual="false" fusestatic="false" strippath="false" inertiafromgeom="false" balanceinertia="true"/></mujoco>', s, count=1)
    _tmp = tempfile.TemporaryDirectory(); t = _tmp.name   # 컴파일 때 메시를 읽으므로 build 끝까지 유지
    if True:
        # MuJoCo 는 메시를 파일 이름으로 구분 → profile_holder/body.obj 와 puck_holder/body.obj 가 같은 'body' 로 합쳐짐(퍽 홀더 모양이 프로파일 받침에 그려짐).
        #   경로마다 고유 이름의 링크를 만들어 읽음 (URDF 자체는 바꾸지 않음)
        def uniq(mm):
            src = Path(mm.group(1)); dst = Path(t) / ("_".join(src.relative_to(HERE).parts))
            if not dst.exists(): dst.symlink_to(src)
            return f'filename="{dst}"'
        s = re.sub(r'filename="(' + re.escape(str(HERE)) + r'/[^"]+)"', uniq, s)
        f = Path(t) / "r.urdf"; f.write_text(s)
        spec = mujoco.MjSpec.from_file(str(f))
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    spec.visual.quality.shadowsize = 4096
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]; spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    root = spec.worldbody.first_body(); root.add_freejoint(); root.pos = [0, 0, 0.0725]
    for g in spec.geoms:
        if g.contype or g.conaffinity:          # 충돌 형상: 바닥하고만
            g.contype, g.conaffinity = 0, 1; g.friction = [1.0, 0.005, 0.0001]; g.group = 3
    tex = spec.add_texture(); tex.name = "grid"; tex.type = mujoco.mjtTexture.mjTEXTURE_2D; tex.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    tex.rgb1 = [0.86, 0.87, 0.89]; tex.rgb2 = [0.78, 0.8, 0.83]; tex.width = tex.height = 512
    sky = spec.add_texture(); sky.name = "sky"; sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX; sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.93, 0.94, 0.96]; sky.rgb2 = [0.80, 0.83, 0.88]; sky.width = sky.height = 256
    mat = spec.add_material(); mat.name = "grid"; mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"; mat.texrepeat = [20, 20]
    fl = spec.worldbody.add_geom(); fl.type = mujoco.mjtGeom.mjGEOM_PLANE; fl.size = [6, 6, 0.1]; fl.contype = fl.conaffinity = 1
    fl.friction = [1.0, 0.005, 0.0001]; fl.material = "grid"
    lt = spec.worldbody.add_light(); lt.pos = [0.5, -0.8, 2.5]; lt.dir = [-0.2, 0.3, -1]; lt.castshadow = True; lt.diffuse = [0.55, 0.55, 0.55]
    for j in spec.joints:
        if j.name.startswith("drive_"): j.damping = [0.002, 0, 0]; j.armature = 0.005
        elif j.name.startswith("steer_"): j.damping = [0.002, 0, 0]; j.armature = 0.012
        elif j.name.startswith("arm_joint"):
            j.armature = 0.012; j.frictionloss = 0.3; j.damping = [0.5, 0, 0]   # [추정] 토크 OFF XM430 감속기 역구동 저항
        elif j.name.startswith("gripper_finger"):
            j.armature = 0.012; j.frictionloss = 2.0; j.damping = [5.0, 0, 0]   # [추정] 그리퍼 크랭크·감속기 저항 [N]

    def act(name, kind, kp=0.0, kv=0.0, frc=1.0):
        a = spec.add_actuator(); a.name = "a_" + name; a.target = name; a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        if kind == "pos": a.set_to_position(kp=kp, kv=kv)
        else: a.set_to_velocity(kv=kv)
        a.forcelimited = True; a.forcerange = [-frc, frc]
    # URDF mimic(손가락 b = 손가락 a)을 MuJoCo 가 읽지 않음(equality 0개, 확인) → 직접 연결. 실물은 크랭크로 두 손가락이 같이 움직임
    eq = spec.add_equality(); eq.type = mujoco.mjtEq.mjEQ_JOINT; eq.name1 = "gripper_finger_b_joint"; eq.name2 = "gripper_finger_a_joint"
    eq.data[0:5] = [0, 1, 0, 0, 0]
    for j in J["steer"]: act(j, "pos", kp=20.0, kv=0.3, frc=4.1)
    for j in J["drive"]: act(j, "vel", kv=1.0, frc=3.0)
    for j in spec.joints:
        pass   # 주행 중 팔·그리퍼 토크 OFF(실물): 액추에이터 없음. 그리퍼는 받침 상자 벽이 막음(아래 충돌 설정)
    # 받침에 끼운 그리퍼 = 받침에 고정(용접). 실물: 주행 중 팔·그리퍼 토크 OFF, 받침 벽이 손목·손가락을 막아 못 움직임.
    #   접촉으로 안 하는 이유: 지금 형상은 실제 받침 자세에서 손가락이 벽을 3.9 mm 관통하고 바닥에서 3~5 mm 떠 있음(10-27 확인)
    #   → 접촉만으로는 시작하자마자 밀려나 팔이 접힘. 형상 수정 전까지 용접으로 대신함. 상대 자세는 park_lock() 이 받침 자세에서 잼.
    for b in ("arm_link5_wrist_roll", "gripper_finger_a"):
        w = spec.add_equality(); w.name = "park_" + b; w.type = mujoco.mjtEq.mjEQ_WELD; w.objtype = mujoco.mjtObj.mjOBJ_BODY
        w.name1, w.name2 = b, "gripper_park_stand_TEMP"
    return spec.compile()


def park_lock(m, d):
    """지금 자세(받침 자세)에서 손목·손가락 ↔ 받침 상대 자세를 용접 목표로 기록. mj_forward 뒤에 부름."""
    st = m.body("gripper_park_stand_TEMP").id
    for b in ("arm_link5_wrist_roll", "gripper_finger_a"):
        e = m.equality("park_" + b).id; bi = m.body(b).id
        R1 = d.xmat[bi].reshape(3, 3)
        m.eq_data[e, 0:3] = 0.0
        m.eq_data[e, 3:6] = R1.T @ (d.xpos[st] - d.xpos[bi])
        q = np.zeros(4); mujoco.mju_negQuat(q, d.xquat[bi]); r = np.zeros(4); mujoco.mju_mulQuat(r, q, d.xquat[st])
        m.eq_data[e, 6:10] = r; m.eq_data[e, 10] = 1.0


m = build(); d = mujoco.MjData(m); R = mujoco.Renderer(m, H, W); RH = mujoco.Renderer(m, H, W // 2)
A = {m.actuator(i).name[2:]: i for i in range(m.nu)}
qa = lambda n: m.jnt_qposadr[m.joint(n).id]
va = lambda n: m.jnt_dofadr[m.joint(n).id]
OPT = mujoco.MjvOption(); OPT.geomgroup[3] = 0     # 충돌 형상 숨김


def set_arm(q):
    for n, v in q.items():
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0:
            d.qpos[qa(n)] = v                            # 팔·그리퍼 토크 OFF → 위치만 놓음
            if n in A: d.ctrl[A[n]] = v
        if n == "gripper_finger_a_joint":
            d.qpos[qa("gripper_finger_b_joint")] = v


def shot(az, el, dist, look, half=False):
    cam = mujoco.MjvCamera(); cam.lookat[:] = look; cam.distance, cam.azimuth, cam.elevation = dist, az, el
    r = RH if half else R
    r.update_scene(d, cam, OPT); return Image.fromarray(r.render())


def label(im, t1, t2=""):
    dr = ImageDraw.Draw(im); dr.rectangle((0, 0, W, 78 if t2 else 48), fill=(18, 36, 58))
    dr.text((18, 6), t1, font=FB, fill="white")
    if t2: dr.text((18, 44), t2, font=FR, fill=(255, 214, 120))
    return im


def images():
    for key, title in (("C1_P0", "준비 자세 P0"), (None, "받침 자세(주행 자세)")):
        mujoco.mj_resetData(m, d)
        set_arm(IP["poses"][key]["q_rad"] if key else IP["positions"])
        mujoco.mj_forward(m, d)
        tiles = [shot(225, -22, 0.75, d.qpos[0:3] + [0.02, 0, 0.14], True), shot(320, -14, 0.7, d.qpos[0:3] + [0.03, 0, 0.14], True)]
        im = Image.new("RGB", (W, H)); im.paste(tiles[0], (0, 0)); im.paste(tiles[1], (W // 2, 0))
        label(im, f"닌자터틀 배포 URDF 2026-10-27 · {title}", "J1 축 수정 · 고정 퍽 제거(빈 홀더 6칸) · 바퀴 관절 한계 = 모터 최대값").save(IMG / ("release_p0.png" if key else "release_parked.png"))
    print("images ok")


def views():
    """받침 자세 6방향. MuJoCo 방위각: 0 = 카메라가 뒤에서 +x(앞) 를 봄 → 정면은 180."""
    mujoco.mj_resetData(m, d); set_arm(IP["positions"]); mujoco.mj_forward(m, d)
    tw, th = 760, 570; rr = mujoco.Renderer(m, th, tw)
    V = [("정면 (앞 +x)", 180, -6, 0.80, 0.17), ("후면", 0, -6, 0.80, 0.17), ("왼쪽 (+y) — 앞이 화면 왼쪽", 270, -6, 0.80, 0.17),
         ("오른쪽 — 앞이 화면 오른쪽", 90, -6, 0.80, 0.17), ("위 — 앞이 화면 위쪽", 0, -89.9, 1.25, 0.10), ("비스듬히 (앞 왼쪽 위)", 215, -25, 0.72, 0.15)]
    big = Image.new("RGB", (tw * 3, th * 2 + 60), (18, 36, 58))
    for i, (name, az, el, dist, lz) in enumerate(V):
        cam = mujoco.MjvCamera(); cam.lookat[:] = d.qpos[0:3] + [0.0, 0.0, lz]; cam.distance, cam.azimuth, cam.elevation = dist, az, el
        rr.update_scene(d, cam, OPT); im = Image.fromarray(rr.render())
        dr = ImageDraw.Draw(im); dr.rectangle((0, 0, tw, 40), fill=(18, 36, 58)); dr.text((14, 3), name, font=FR, fill="white")
        big.paste(im, ((i % 3) * tw, 60 + (i // 3) * th))
    ImageDraw.Draw(big).text((18, 10), "닌자터틀 스워브 배포본 2026-10-27 · 받침 자세(주행 자세) · J1 축 수정 · 고정 퍽 제거 · 바퀴 한계 = 모터 최대값", font=FB, fill="white")
    big.save(IMG / "release_views.png"); rr.close(); print("views ok")


def ffmpeg(frames_dir, out, fps):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(frames_dir / "f%05d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", str(out)], check=True)


def drive():
    V, Wz = cfg.max_linear, cfg.max_angular; Dg = V / math.sqrt(2)
    segs = [("S자", (V, 0, 0), 1.5), ("S자", (Dg, Dg, 0), 1.5), ("S자", (V, 0, 0), 1.5), ("S자", (Dg, -Dg, 0), 1.5), ("S자", (V, 0, 0), 1.5),
            ("정지", (0, 0, 0), 1.0)]
    segs += [("정면 유지 전방향", (math.cos(k * math.pi / 4) * V, math.sin(k * math.pi / 4) * V, 0), 1.5) for k in range(9)]
    segs += [("정지", (0, 0, 0), 1.0), ("제자리 회전", (0, 0, Wz), 2.5), ("회전 → 바로 전진", (V, 0, 0), 2.0), ("정지", (0, 0, 0), 1.0)]
    mujoco.mj_resetData(m, d); set_arm(IP["positions"]); mujoco.mj_forward(m, d); park_lock(m, d)
    for _ in range(300): mujoco.mj_step(m, d)
    drv = SwerveDrive(kin, cfg); tgt = [d.qpos[qa(n)] for n in J["steer"]]
    fr = IMG / "_frames_drive"; fr.mkdir(exist_ok=True); [p.unlink() for p in fr.glob("*.png")]
    t, k, fps = 0.0, 0, 30; dt = 0.01; nxt = 0.0; trail = []
    for name, cmd, secs in segs:
        for _ in range(int(secs / dt)):
            t += dt
            meas = [ModuleState(d.qpos[qa(s)], d.qvel[va(w)], d.qvel[va(s)]) for s, w in zip(J["steer"], J["drive"])]
            drv.set_command(*cmd, t); out = drv.update(t, dt, meas)
            for i, (s, w) in enumerate(zip(J["steer"], J["drive"])):
                e = out[i].steer_angle - tgt[i]; tgt[i] += max(-4.7962 * dt, min(4.7962 * dt, e))
                d.ctrl[A[s]] = tgt[i]; d.ctrl[A[w]] = max(-7.9137, min(7.9137, out[i].wheel_speed))
            for _ in range(5): mujoco.mj_step(m, d)
            if t >= nxt:
                nxt += 1.0 / fps
                Rm = d.xmat[1].reshape(3, 3); v = Rm.T @ d.qvel[0:3]
                im = shot(150, -32, 1.25, d.qpos[0:3] + [0, 0, 0.05])
                st = "조향 대기(조향 먼저)" if drv.gate_blocked else ("동시 조향 중" if drv.blend_ratio < 1 else "주행")
                label(im, f"스워브 시뮬 주행 (MuJoCo 물리) · {name} · {t:5.1f} s",
                      f"명령 vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} m/s, wz {cmd[2]:+.2f} rad/s → 실제 {v[0]:+.2f} {v[1]:+.2f} m/s, {d.qvel[5]:+.2f} rad/s · {st}").save(fr / f"f{k:05d}.png")
                k += 1
    ffmpeg(fr, IMG / "drive_swerve_sim.mp4", fps); print("drive ok", k, "frames")


def arm(run="ARM_20261026T023249Z"):
    rows, last = [], -1e9
    for l in open(f"{RUNS}/{run}/03_session.jsonl"):
        if '"event":"snapshot"' not in l or '"state":"REPLAY' not in l: continue
        x = json.loads(l)
        if x["monotonic_ms"] - last >= 100:
            last = x["monotonic_ms"]; rows.append((x["monotonic_ms"], {k: v["pos"] for k, v in x["ids"].items()}))
    gp = JM["gripper"]["gap_table_tick_mm"]
    fr = IMG / "_frames_arm"; fr.mkdir(exist_ok=True); [p.unlink() for p in fr.glob("*.png")]
    lo = m.jnt_range[m.joint("gripper_finger_a_joint").id][0]
    for k, (tms, tk) in enumerate(rows):
        mujoco.mj_resetData(m, d)
        q = {}
        for a in JM["arm"]:
            q[a["joint"]] = math.radians(a["q_deg_at_ref"]) + a["sign"] * (tk[str(a["id"])] - a["ref_tick"]) * 2 * math.pi / 4096
        gap = float(np.interp(tk["15"], [x for x, _ in gp], [y for _, y in gp]))
        q["gripper_finger_a_joint"] = max(lo, (gap - JM["gripper"]["gap0_mm"]) / 2000)
        set_arm(q); mujoco.mj_forward(m, d)
        im = shot(205, -24, 0.95, d.qpos[0:3] + [0.04, -0.02, 0.17])
        label(im, f"실물 팔 실행 재생 · A9F5(퍽 잡아 적재함) · {(tms - rows[0][0]) / 1000:5.1f} s",
              f"실제 모터 기록을 이 배포 URDF 에 그대로 · J1~J5 {tk['11']}/{tk['12']}/{tk['13']}/{tk['14']}/{tk['16']} · 그리퍼 {tk['15']} (1배속)").save(fr / f"f{k:05d}.png")
    ffmpeg(fr, IMG / "arm_replay_A9F5.mp4", 10); print("arm ok", len(rows), "frames")


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("images", "all"): images()
    if what in ("views", "images", "all"): views()
    if what in ("drive", "all"): drive()
    if what in ("arm", "all"): arm()
