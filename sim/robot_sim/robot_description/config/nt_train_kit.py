"""학습용 묶음 (Isaac Lab 3.0) — 이 배포본의 로봇을 학습 환경에 넣을 때 쓰는 검증된 설정 한곳 모음.

    import sys; sys.path.insert(0, "<이 폴더>")
    from config import nt_train_kit as kit
    robot = kit.robot_cfg("{ENV_REGEX_NS}/Robot")          # ArticulationCfg (USD·액추에이터·parked 시작·자기 충돌 켬)
    ...  scene 만든 뒤, sim.reset() 전에:
    kit.apply_collision_filters(stage)                     # 차체 (c) + 손가락 a↔b (모든 env)
    drive = kit.swerve()                                   # 몸 속도 → 조향 각·바퀴 속도 (env 하나당 하나)
    rp = kit.Replay()                                      # 실물 성공 실행 팔 궤적: rp.segment("out"/"back"/"full"), rp.pose("parked"/"p0"/"grasp")

검증(tools/check_train_kit.py, env 여러 개): parked 유지 그리퍼 16 mm, 실측 꺼내기→복귀 끝 자세 오차, 스워브 직진 속도.
값 근거: README '최종 점검' · CHANGES. 팀 sim(robot_sim) 의 TURTLENINJA_CFG 와 다른 점:
  - 자기 충돌 켬: 받침(gripper_park_stand)이 base_link 에 합쳐져 있어, 끄면 손가락이 받침을 통과 → 꺼내기·넣기 학습 불가.
  - 그리퍼: 두 손가락 모두 구동 stiffness 1000 · damping 2 · 20 N · 0.15 m/s (1 N·200 이면 16 mm 유지·빠른 개폐가 안 됨).
  - 시작 = parked(받침에 넣은 주행 자세) + 그리퍼 16 mm. P0 시작은 pose("p0").
  - 차체 볼록 덮개가 조향·바퀴에 닿는 가짜 접촉 제거((c) 필터), 손가락끼리 충돌 제외.
"""
import json, math, os, sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
USD = HERE / "usd/robot/robot.usda"
sys.path.insert(0, str(HERE / "swerve"))
G0_MM = 58.50723799617533   # 손가락 관절 0 일 때 패드 간격


def actuators():
    """검증한 액추에이터(키보드 시연·verify_* 와 같은 값). 팔은 USD 의 구동값(stiffness/damping None)."""
    from isaaclab.actuators import ImplicitActuatorCfg
    return {"steer": ImplicitActuatorCfg(joint_names_expr=["steer_.*"], joint_effort_limit=4.1, joint_velocity_limit=4.7962, stiffness=20.0, damping=0.3, armature=0.012),
            "drive": ImplicitActuatorCfg(joint_names_expr=["drive_.*"], joint_effort_limit=3.0, joint_velocity_limit=7.9137, stiffness=0.0, damping=1.0, armature=0.005),
            "arm": ImplicitActuatorCfg(joint_names_expr=["arm_joint[1-5]"], stiffness=None, damping=None),
            "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0, joint_effort_limit=20.0, joint_velocity_limit=0.15)}


def robot_cfg(prim_path="{ENV_REGEX_NS}/Robot", root_z=0.075, self_collision=True):
    """ArticulationCfg. root_z: base_link 높이(바퀴 접지면 −0.072 m → 0.072 이 딱 닿음, 0.075 는 3 mm 위에서 내려놓음)."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg
    from config.isaac_lab_init_state import parked_init_state
    kw = {}
    try:
        from isaaclab_physx.sim.schemas import PhysxArticulationCfg   # Isaac Lab 3.0 PhysX 전용 설정
        kw["articulation_props"] = PhysxArticulationCfg(enabled_self_collisions=self_collision, solver_position_iteration_count=32, solver_velocity_iteration_count=4)
    except Exception:
        try:
            kw["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=self_collision, solver_position_iteration_count=32, solver_velocity_iteration_count=4)
        except Exception:
            pass   # USD 기본값(변환 때 allow_self_collision=True)
    return ArticulationCfg(prim_path=prim_path,
                           spawn=sim_utils.UsdFileCfg(usd_path=str(USD), variants={"Physics": "physx"}, **kw),
                           init_state=parked_init_state(root_z), actuators=actuators())


def apply_collision_filters(stage, root="/World"):
    """sim.reset() 전에 한 번. root 아래 모든 로봇에:
    (c) 차체 chassis_remainder 충돌체 ↔ 같은 로봇의 조향·바퀴 8 링크 충돌 제외(볼록 덮개가 빈 공간을 채워 생기는 가짜 접촉, 실물 간섭 없음 기록),
    손가락 a ↔ b 충돌 제외(USD 에 이미 있으면 그대로). 반환: 처리한 로봇 수."""
    from pxr import Usd, UsdPhysics
    robots = {}
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        n = p.GetName()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and n in ("base_link",):
            robots[str(p.GetPath())] = p
    count = 0
    for bpath, base in robots.items():
        rroot = bpath.rsplit("/Geometry/", 1)[0] if "/Geometry/" in bpath else bpath.rsplit("/", 1)[0]
        rp = stage.GetPrimAtPath(rroot)
        bodies = [q for q in Usd.PrimRange(rp) if q.HasAPI(UsdPhysics.RigidBodyAPI) and ("_steer_link" in q.GetName() or "_wheel_link" in q.GetName())]
        for x in Usd.PrimRange(rp):
            if "chassis_remainder" in x.GetName() and x.IsInstance():
                x.SetInstanceable(False)
        n_ch = 0
        for x in Usd.PrimRange(rp):
            if "chassis_remainder" in str(x.GetPath()) and x.HasAPI(UsdPhysics.CollisionAPI):
                rel = UsdPhysics.FilteredPairsAPI.Apply(x).CreateFilteredPairsRel()
                for b in bodies:
                    rel.AddTarget(b.GetPath())
                n_ch += 1
        fa = [q for q in Usd.PrimRange(rp) if q.GetName() == "gripper_finger_a" and q.HasAPI(UsdPhysics.RigidBodyAPI)]
        fb = [q for q in Usd.PrimRange(rp) if q.GetName() == "gripper_finger_b" and q.HasAPI(UsdPhysics.RigidBodyAPI)]
        if fa and fb:
            rel = UsdPhysics.FilteredPairsAPI.Apply(fa[0]).CreateFilteredPairsRel()
            if fb[0].GetPath() not in rel.GetTargets():
                rel.AddTarget(fb[0].GetPath())
        count += 1 if n_ch and len(bodies) == 8 else 0
    return count


def swerve(entry_margin_deg=5.0):
    """팀 swerve.cpp 이식 + 목표각 일관성 + 진입 여유(기본 5°, collision_c_sweep 검증). 최고 0.26 m/s · 1.675 rad/s (실물 하드웨어 끝)."""
    os.environ.setdefault("ENTRY_MARGIN_DEG", str(entry_margin_deg)); os.environ.setdefault("RETENTION_DEG", "0"); os.environ["CONTROL_VARIANT"] = "candidate"
    import nt_candidate as nt
    kin, cfg, _ = nt.from_config(str(HERE / "config/swerve.yaml"))
    cfg.max_linear, cfg.max_angular, cfg.max_linear_accel, cfg.max_angular_accel = 0.26, 1.675, 1.0, 6.0   # swerve.yaml limits_for_training
    return nt.SwerveDrive(kin, cfg), nt


class Replay:
    """실물 성공 실행의 팔 궤적(config/arm_replays/<실행>.json, 50 ms Present 틱 → 관절값). 기본 A9F5 실물 최단 022813Z.
    받침 안에서는 이 실측 경로(out/back)만 쓰는 것을 권장(받침 안 직선 보간은 받침 벽에 걸렸다 말았다 함)."""
    def __init__(self, run="ARM_20261026T022813Z"):
        import numpy as np
        self.np = np
        d = json.loads((HERE / f"config/arm_replays/{run}.json").read_text())
        self.run, self.t, self.q, self.g = run, np.array(d["t_s"]), np.array(d["arm_q_rad"]), np.array(d["finger_q_m"])
        self.joint_names = d["joint_names"]; self.seg = d["segments"]
        io = int(np.argmax(self.g)); self.t_grasp = float(self.t[io + int(np.argmax(self.g[io:] < self.g[io] - 0.001))])

    def at(self, t):
        """(팔 5 관절 [rad], 손가락 q [m]) at 시각 t [s]"""
        np = self.np
        return np.array([np.interp(t, self.t, self.q[:, i]) for i in range(5)]), float(np.interp(t, self.t, self.g))

    def segment(self, name):
        """'out' 받침→P0, 'back' P0→받침, 'full' 미션 전체 → (시작 s, 끝 s). 1배 = 실물 속도."""
        return {"out": tuple(self.seg[0]["wall_s"]), "back": tuple(self.seg[-1]["wall_s"]), "full": (self.seg[0]["wall_s"][0], self.seg[-1]["wall_s"][1])}[name]

    def pose(self, name):
        """'parked' 받침 자세, 'p0' 대기자세, 'grasp' 퍽 집기 자세(그리퍼 닫히기 시작한 순간) → (팔 q, 손가락 q)"""
        return self.at({"parked": self.seg[0]["wall_s"][0], "p0": self.seg[0]["wall_s"][1], "grasp": self.t_grasp}[name])


def gap_mm(finger_q):
    return G0_MM + 2000.0 * finger_q


def load_obj(path):
    """크랭크 OBJ 읽기 → (.vertices (n,3) float, .faces (m,3) int). trimesh 있으면 그것, 없으면 v/f 줄만 직접 읽음(Isaac Lab 기본 설치에 trimesh 가 없을 수 있음)."""
    import numpy as np
    from types import SimpleNamespace
    try:
        import trimesh
        m = trimesh.load(str(path), force="mesh"); return SimpleNamespace(vertices=np.asarray(m.vertices), faces=np.asarray(m.faces))
    except ImportError:
        pass
    V, F = [], []
    for ln in Path(path).read_text().splitlines():
        t = ln.split()
        if not t:
            continue
        if t[0] == "v":
            V.append([float(x) for x in t[1:4]])
        elif t[0] == "f":
            ids = [int(x.split("/")[0]) for x in t[1:]]; ids = [i - 1 if i > 0 else len(V) + i for i in ids]
            F += [[ids[0], ids[j], ids[j + 1]] for j in range(1, len(ids) - 1)]
    return SimpleNamespace(vertices=np.array(V, float), faces=np.array(F, int))


class CrankVisuals:
    """그리퍼 크랭크(회색)·링크 막대(노랑)를 손가락 위치에 맞춰 움직이는 보이는 모양(물리 무관). USD 의 막대는 CAD 자세(58.5 mm)로 고정돼 있어
    그리퍼를 16 mm 로 닫으면 막대 끝이 옆으로 튀어나와 보임 → 카메라(손목 D435 등)를 쓰는 학습·영상에서는 이것을 붙이고 렌더 전에 update().
    sim.reset() 전에 만들 것. 크랭크 각 = 손가락 → 간격 → 실측 틱표(1045 열림·2270 29 mm·2658 16 mm) → 모터 각(config/crank_linkage.json)."""
    def __init__(self, stage, root="/World"):
        import numpy as np
        from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt
        self.np, self.Gf = np, Gf
        ck = json.loads((HERE / "config/crank_linkage.json").read_text()); self.ck = ck
        self.a, self.C = np.array(ck["axis"]), np.array(ck["center_m"]); self.tab = np.array(ck["tick_table_mm"])[::-1]
        meshes = {nm: load_obj(HERE / f) for nm, f in ck["meshes"].items()}
        self.ops, self.last = [], None
        for w in [p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.GetName() == "arm_link5_wrist_roll" and p.HasAPI(UsdPhysics.RigidBodyAPI)]:
            for c in w.GetChildren():
                if c.GetName() in ck["hide_visuals"]:
                    UsdGeom.Imageable(c).MakeInvisible()
            ops = {}
            for nm, m in meshes.items():
                g = UsdGeom.Mesh.Define(stage, f"{w.GetPath()}/crank_anim_{nm}")
                g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(m.vertices.astype(np.float32)))
                g.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces))); g.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.reshape(-1).tolist()))
                g.CreateDisplayColorAttr([Gf.Vec3f(*ck["colors"][nm])]); ops[nm] = g.AddTransformOp()
            self.ops.append(ops)

    def _axR(self, a, t):
        np = self.np; K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]); return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K

    def _set(self, op, R, t):
        M = self.np.eye(4); M[:3, :3] = R.T; M[3, :3] = t; op.Set(self.Gf.Matrix4d(M.tolist()))

    def update(self, finger_q):
        """finger_q: env 별 손가락 관절값 [m] (a·b 평균), 길이 = 로봇 수. 바뀐 env 만 씀."""
        np = self.np; ck = self.ck
        q_all = np.asarray(finger_q, float).reshape(-1)
        from pxr import Sdf
        with Sdf.ChangeBlock():
            for e, ops in enumerate(self.ops):
                q = float(q_all[e])
                if self.last is not None and abs(self.last[e] - q) < 2e-5:
                    continue
                tick = float(np.interp(ck["gap0_mm"] + 2000 * q, self.tab[:, 1], self.tab[:, 0]))
                R = self._axR(self.a, ck["sign"] * (tick - ck["tick_cad"]) * 2 * math.pi / 4096)
                self._set(ops["crank"], R, self.C - R @ self.C)
                for f in "ab":
                    P0, F0 = np.array(ck["rod_pin_end_m"][f]), np.array(ck["rod_finger_end_m"][f])
                    P = self.C + R @ (P0 - self.C); F = F0 + np.array(ck["finger_axis"][f]) * q
                    u0, u = (F0 - P0) / np.linalg.norm(F0 - P0), (F - P) / np.linalg.norm(F - P); v = np.cross(u0, u); s = np.linalg.norm(v)
                    Rr = np.eye(3) if s < 1e-12 else self._axR(v / s, math.atan2(s, float(u0 @ u)))
                    self._set(ops[f"rod_{f}"], Rr, P - Rr @ P0)
        self.last = q_all.copy()
