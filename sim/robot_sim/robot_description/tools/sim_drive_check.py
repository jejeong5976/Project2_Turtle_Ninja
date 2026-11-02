#!/usr/bin/env python3
"""MuJoCo 바닥 주행 확인: 배포 URDF + swerve/nt_swerve.py(실물 컨트롤러와 같은 규칙)로 바퀴를 실제로 굴려 봄. 포트·실물 없음.
  - 차체 자유 이동(바닥 위). 팔·그리퍼 토크 OFF(실물 주행과 같음) — 받침에 끼운 손목·손가락은 받침에 용접(받침이 막음), 로봇 충돌은 바닥과만
  - 조향: 위치 서보 + 목표를 조향 최고 속도(4.796 rad/s)로 따라가게, 힘 ±4.1 N·m
  - 구동: 속도 서보, 힘 ±3.0 N·m, 속도 ±7.9137 rad/s
  - 제어 100 Hz, 물리 500 Hz
결과: config/sim_drive_check.json (+ 콘솔 표). 실행: MUJOCO_GL=egl <isaac env python> tools/sim_drive_check.py"""
import json, math, re, sys, tempfile
from pathlib import Path
import numpy as np, mujoco
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "swerve"))
from nt_swerve import ModuleState, SwerveDrive, from_config  # noqa: E402

kin, cfg, J = from_config(str(HERE / "config/swerve.yaml"))
init = json.loads((HERE / "config/initial_pose.json").read_text())["positions"]

def build():
    s = (HERE / "urdf/robot.urdf").read_text().replace("package://nt_release_description/", str(HERE) + "/")
    s = re.sub(r'(<robot name="[^"]+">)', r'\1<mujoco><compiler discardvisual="true" fusestatic="false" strippath="false" inertiafromgeom="false" balanceinertia="true"/></mujoco>', s, count=1)
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
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST   # 서보 감쇠를 암시적으로 — 작은 바퀴 관성에서도 안정
    root = spec.worldbody.first_body()
    root.add_freejoint()
    root.pos = [0, 0, 0.0725]
    for g in spec.geoms:
        g.contype, g.conaffinity = 0, 1
        g.friction = [1.0, 0.005, 0.0001]
    fl = spec.worldbody.add_geom(); fl.type = mujoco.mjtGeom.mjGEOM_PLANE; fl.size = [5, 5, 0.1]; fl.contype = fl.conaffinity = 1
    fl.friction = [1.0, 0.005, 0.0001]
    for j in spec.joints:
        # 감속기 반사 관성(armature) [추정]: XM430 회전자 관성 ~1e-7 kg·m² × 감속비²(W210 ≈213, W350 ≈353) → 0.005~0.012
        if j.name.startswith("drive_"):
            j.damping = [0.002, 0, 0]; j.armature = 0.005
        elif j.name.startswith("steer_"):
            j.damping = [0.002, 0, 0]; j.armature = 0.012
        elif j.name.startswith(("arm_joint", "gripper_finger")):
            j.armature = 0.012
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

m = build(); d = mujoco.MjData(m)
jid = lambda n: m.joint(n).id
QA = {n: m.jnt_qposadr[jid(n)] for n in J["steer"] + J["drive"]}
VA = {n: m.jnt_dofadr[jid(n)] for n in J["steer"] + J["drive"]}
A = {m.actuator(i).name[2:]: i for i in range(m.nu)}

def reset():
    mujoco.mj_resetData(m, d)
    for n, v in init.items():
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0 and m.joint(n).type != mujoco.mjtJoint.mjJNT_FREE:
            d.qpos[m.jnt_qposadr[jid(n)]] = v            # 팔·그리퍼는 토크 OFF → 위치만 놓고 받침이 받침
            if n in A: d.ctrl[A[n]] = v
    mujoco.mj_forward(m, d); park_lock(m, d)
    for _ in range(300): mujoco.mj_step(m, d)    # 내려앉기

def body_state():
    R = d.xmat[1].reshape(3, 3)
    v = R.T @ d.qvel[0:3]; yaw = math.atan2(R[1, 0], R[0, 0])
    return v[0], v[1], d.qvel[5], yaw, d.qpos[0:2].copy()

V, W = cfg.max_linear, cfg.max_angular
Dg = V / math.sqrt(2)
SCEN = {
    "전진": [((V, 0, 0), 2.0)], "옆(왼)": [((0, V, 0), 2.0)], "대각(앞왼)": [((Dg, Dg, 0), 2.0)], "대각(앞오른, rr 끝)": [((Dg, -Dg, 0), 2.0)],
    "후진": [((-V, 0, 0), 2.0)], "제자리 회전": [((0, 0, W), 2.0)],
    "S자": [((V, 0, 0), 1.5), ((Dg, Dg, 0), 1.5), ((V, 0, 0), 1.5), ((Dg, -Dg, 0), 1.5), ((V, 0, 0), 1.5)],
    "정면 유지 전방향": [((math.cos(k * math.pi / 4) * V, math.sin(k * math.pi / 4) * V, 0), 1.5) for k in range(9)],
    "곡선": [((V, 0, 1.0), 2.0), ((V, 0, -1.0), 2.0)],
    "회전→전진": [((0, 0, W), 1.5), ((V, 0, 0), 1.5)],
}
ctrl_dt = 0.01; sub = int(round(ctrl_dt / m.opt.timestep))
report = {}
for name, segs in SCEN.items():
    reset(); drv = SwerveDrive(kin, cfg); t = 0.0
    tgt_steer = [d.qpos[QA[n]] for n in J["steer"]]
    segrep = []
    for cmd, secs in segs:
        p0 = d.qpos[0:2].copy(); yaw0 = body_state()[3]; vs = []; gated = 0; steps = int(secs / ctrl_dt)
        ARM = [n for n in init if n.startswith(("arm_joint", "gripper_finger")) and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0]
        armdev = {n: 0.0 for n in ARM}
        for k in range(steps):
            t += ctrl_dt
            meas = [ModuleState(d.qpos[QA[s]], d.qvel[VA[w]], d.qvel[VA[s]]) for s, w in zip(J["steer"], J["drive"])]
            drv.set_command(*cmd, t)
            out = drv.update(t, ctrl_dt, meas)
            gated += drv.gate_blocked
            for i, (s, w) in enumerate(zip(J["steer"], J["drive"])):
                e = out[i].steer_angle - tgt_steer[i]
                tgt_steer[i] += max(-4.7962 * ctrl_dt, min(4.7962 * ctrl_dt, e))
                d.ctrl[A[s]] = tgt_steer[i]
                d.ctrl[A[w]] = max(-7.9137, min(7.9137, out[i].wheel_speed))
            for _ in range(sub): mujoco.mj_step(m, d)
            if k >= steps // 2: vs.append(body_state()[:3])
            for n in ARM: armdev[n] = max(armdev[n], abs(d.qpos[m.jnt_qposadr[jid(n)]] - init[n]))
        vx, vy, wz, yaw, p = body_state()
        dp = p - p0; c, s_ = math.cos(yaw0), math.sin(yaw0)
        disp = (c * dp[0] + s_ * dp[1], -s_ * dp[0] + c * dp[1])
        want = math.hypot(cmd[0], cmd[1]) * secs
        vm = np.mean(vs, axis=0)
        seg = {"cmd": [round(x, 4) for x in cmd], "secs": secs,
               "move_mm": [round(disp[0] * 1000), round(disp[1] * 1000)], "yaw_deg": round(math.degrees(yaw - yaw0), 1),
               "late_vel": [round(vm[0], 4), round(vm[1], 4), round(vm[2], 3)], "gate_blocked_s": round(gated * ctrl_dt, 2),
               "arm_max_dev": {n.replace("_joint", ""): round(math.degrees(v), 2) if n.startswith("arm") else round(v * 1000, 2) for n, v in armdev.items()}}   # 팔 [°] · 손가락 [mm] (받침 자세 대비)
        if want > 1e-9:
            seg["progress"] = round((disp[0] * cmd[0] + disp[1] * cmd[1]) / (math.hypot(cmd[0], cmd[1]) * want), 2)
            seg["late_speed_ratio"] = round(math.hypot(vm[0], vm[1]) / math.hypot(cmd[0], cmd[1]), 3)
        else:
            seg["late_wz_ratio"] = round(vm[2] / cmd[2], 3) if cmd[2] else None
        segrep.append(seg)
    report[name] = segrep
    print(f"== {name}")
    for sgm in segrep:
        print("   ", sgm)
(HERE / "config/sim_drive_check.json").write_text(json.dumps({"engine": f"MuJoCo {mujoco.__version__}", "control_hz": 100, "physics_hz": 500,
    "limits": {"max_linear": V, "max_angular": W, "accel": [cfg.max_linear_accel, cfg.max_angular_accel], "max_wheel_speed": cfg.max_wheel_speed},
    "note": "시뮬 결과. 바퀴 마찰 1.0·팔/그리퍼 토크 OFF·받침 안 손목/손가락은 받침에 용접(형상 관통 3.9 mm 때문에 접촉 대신)·로봇 충돌은 바닥과만. 실물 결과 아님.", "scenarios": report}, ensure_ascii=False, indent=1))
