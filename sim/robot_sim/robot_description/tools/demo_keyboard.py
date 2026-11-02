"""키보드 시연 (Isaac Sim GUI 창 하나, Isaac Lab) — 배치 수정판 모델로 주행·팔 동작을 직접 조종.

주행(스워브, 실물과 같은 계산 nt_candidate: 목표각 일관성 수정 + 진입 여유 5°, 최고 0.26 m/s):
  ↑ ↓ 앞뒤   ← → 왼쪽·오른쪽 옆으로   Z / X 제자리 회전   L 정지
팔(실물 성공 실행 14개의 측정 궤적 재생, 9 / 0 으로 실행 고르기, 기본 ARM_20261026T022813Z A9F5 실물 최단):
     2 받침→P0 꺼내기   1 P0→받침 복귀(그 실행의 실측 복귀)   M 미션 전체
     P 대기자세(P0)   O 퍽 집기 자세(그리퍼 열림, G 로 닫기) — 둘 다 고른 실행의 실측 자세(꺼내기→그리퍼 열며 올리기→잡기→접어 들기→적재함→빠지기→닫으며 복귀→넣기, 29 s)
     [ / ] 재생 배속 낮춤·높임(기본 2배, 0.5~4배; 1배 = 실물 최단 실행 속도)
     G 그리퍼 열기(79 mm)/닫기(16 mm)   R(또는 Backspace·Home) 로봇을 처음 자리로
팔 자유 조작: 3 4 5 6 7 = J1~J5 고르기, W / S 누르고 있는 동안 + / − 방향(0.8 rad/s, 관절 범위 안). 재생 중에는 안 움직임.
환경(NT_WORLD=maze 기본): 팀장님 sim(NT_TEAM_SIM, 기본 /home/pa25/sim/robot_sim; 없으면 빈 바닥) 의 실제 경기장 미로 maze_real.usd + 카펫 바닥 + 강의실 파노라마 + 천장 조명 6개 + 퍽 3개.
  NT_WORLD=empty 면 예전 빈 바닥(60 mm 라이다 스캔면 원판 표시). 시작 자리 NT_START="x,y,yaw°"(기본 2번 칸 −0.367,0.394,0°).
카메라 창 2개가 따로 뜸(움직이면서 실시간, C 로 끄기/켜기 — 끄면 화면이 부드러워짐): birdeye = 팀 실측 K + 어안 왜곡 D(config/camera_wide.yaml 값, 팀 sim 과 같음), wrist_d435 = D435 공장 K.
차체 충돌: chassis_remainder 1개 ↔ 조향·바퀴 8 링크만 충돌 제외((c), 실물 간섭 없음 기록 근거). 나머지 충돌 유지.
그리퍼 크랭크(회색)·막대(노랑)는 손가락 위치 → 실물 모터 틱(1045 열림·2658 닫힘) 각도로 같이 움직임(보이는 모양만).
창 3D 화면을 한 번 클릭해야 키 입력이 들어감.
실행: OMNI_KIT_ACCEPT_EULA=YES /home/pa25/IsaacLab-course/.venv/bin/python tools/demo_keyboard.py"""
import json, math, os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
SW = HERE / "swerve"   # nt_candidate.py = 팀 swerve.cpp 이식 + 목표각 일관성 수정 + 진입 여유(collision_c_sweep_20261027 에서 검증한 것)
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(SW))
os.environ.setdefault("ENTRY_MARGIN_DEG", "5"); os.environ.setdefault("RETENTION_DEG", "0"); os.environ["CONTROL_VARIANT"] = "candidate"
from isaaclab.app import AppLauncher
app = AppLauncher({"headless": False, "visualizer": ["kit"]}).app   # Isaac Lab 3.0: Kit GUI 창은 visualizer kit 로 열림
import numpy as np, torch, carb, omni, omni.usd
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
from pxr import Usd, UsdPhysics, Sdf
import nt_candidate as nt
from config.isaac_lab_init_state import parked_init_state, parked_joint_pos

DT = 0.005
DEV = os.environ.get("NT_SIM_DEVICE", "cpu")   # 로봇 1대는 CPU 물리가 GPU 보다 단계당 부담이 작음(화면 끊김 줄임)
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=DEV))
TEAM = Path(os.environ.get("NT_TEAM_SIM", "/home/pa25/sim/robot_sim"))   # 팀장님 sim (읽기만). 레포의 sim/robot_sim 경로를 NT_TEAM_SIM 으로
WORLD = os.environ.get("NT_WORLD", "maze")
if WORLD == "maze" and not (TEAM / "usd/maze_real.usd").exists():
    print(f"[demo] 팀 sim 미로 없음({TEAM}) → 빈 바닥. NT_TEAM_SIM=<Project2_Turtle_Ninja/sim/robot_sim> 로 지정", flush=True); WORLD = "empty"
sx_, sy_, syaw_ = [float(v) for v in os.environ.get("NT_START", "-0.367,0.394,0").split(",")] if WORLD == "maze" else (0.0, 0.0, 0.0)
START = (sx_, sy_, 0.075, 0.0, 0.0, math.sin(math.radians(syaw_) / 2), math.cos(math.radians(syaw_) / 2))   # x y z qx qy qz qw
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg(visible=WORLD != "maze"))
if WORLD == "maze":   # 팀 sim 의 use_real_look + add_ceiling_lights + 미로 (값 그대로)
    sim_utils.DomeLightCfg(intensity=1800.0, texture_file=str(TEAM / "usd/textures/pano_191331.jpg"), texture_format="latlong").func(
        "/World/light", sim_utils.DomeLightCfg(intensity=1800.0, texture_file=str(TEAM / "usd/textures/pano_191331.jpg"), texture_format="latlong"))
    sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/floor_carpet.usda")).func("/World/Floor", sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/floor_carpet.usda")), translation=(0.0, 0.0, 0.0005))
    sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/maze_real.usd")).func("/World/Maze", sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/maze_real.usd")))
    from pxr import UsdLux, UsdGeom as _G, Gf as _Gf
    _st = omni.usd.get_context().get_stage()
    for i_, (x_, y_) in enumerate([(x, y) for x in (-1.2, 1.2) for y in (-2.0, 0.0, 2.0)]):
        lt = UsdLux.RectLight.Define(_st, f"/World/CeilingLights/l{i_}")
        lt.CreateWidthAttr(0.6); lt.CreateHeightAttr(1.2); lt.CreateColorAttr(_Gf.Vec3f(1.0, 0.97, 0.92)); lt.CreateIntensityAttr(10000.0)
        _G.XformCommonAPI(lt.GetPrim()).SetTranslate(_Gf.Vec3d(x_, y_, 2.6))
    c_, s_ = math.cos(math.radians(syaw_)), math.sin(math.radians(syaw_))
    for i_, (d_, l_) in enumerate([(0.30, -0.08), (0.36, 0.06), (0.45, -0.02)]):   # 로봇 앞 퍽 3개(보기용)
        sim_utils.UsdFileCfg(usd_path=str(TEAM / f"usd/props/puck_{i_}.usda")).func(f"/World/Puck_{i_}", sim_utils.UsdFileCfg(usd_path=str(TEAM / f"usd/props/puck_{i_}.usda")),
                                                                                  translation=(sx_ + c_ * d_ - s_ * l_, sy_ + s_ * d_ + c_ * l_, 0.031))
else:
    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2000.0))
    scan = sim_utils.CylinderCfg(radius=0.25, height=0.0008, axis="Z", visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2), opacity=0.3))
    scan.func("/World/ScanPlane_60mm", scan, translation=(0.0, 0.0, 0.060))
robot = Articulation(ArticulationCfg(prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=str(HERE / "usd/robot/robot.usda"), variants={"Physics": "physx"}),
    init_state=parked_init_state(0.075).replace(pos=START[:3], rot=START[3:]),
    actuators={"steer": ImplicitActuatorCfg(joint_names_expr=["steer_.*"], joint_effort_limit=4.1, joint_velocity_limit=4.7962, stiffness=20.0, damping=0.3, armature=0.012),
               "drive": ImplicitActuatorCfg(joint_names_expr=["drive_.*"], joint_effort_limit=3.0, joint_velocity_limit=7.9137, stiffness=0.0, damping=1.0, armature=0.005),
               "arm": ImplicitActuatorCfg(joint_names_expr=["arm_joint[1-5]"], stiffness=None, damping=None),
               "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0, joint_effort_limit=20.0, joint_velocity_limit=0.15)}))   # USD maxForce 1 N 이라 느림 → 20 N

# --- (c) chassis collider filter, before physics parse
stage = omni.usd.get_context().get_stage()
bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and ("_steer_link" in p.GetName() or "_wheel_link" in p.GetName())]
for x in stage.Traverse():
    if "chassis_remainder" in x.GetName() and x.IsInstance():
        x.SetInstanceable(False)
for p in Usd.PrimRange(stage.GetPseudoRoot()):
    if "chassis_remainder" in str(p.GetPath()) and p.HasAPI(UsdPhysics.CollisionAPI):
        rel = UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel()
        for b in bodies:
            rel.AddTarget(b.GetPath())
        print("[demo] (c) filter:", p.GetPath(), "->", len(bodies), "bodies", flush=True)

# --- viewport cameras (team values): birdeye on base_link, D435 on wrist
import xml.etree.ElementTree as ET
U = ET.parse(HERE / "urdf/robot.urdf").getroot(); JS = {j.find("child").get("link"): j for j in U.findall("joint")}
def rpyR(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def pose(link, root="base_link"):
    ch = []
    while link != root:
        j = JS[link]; ch.append(j); link = j.find("parent").get("link")
    R, t = np.eye(3), np.zeros(3)
    for j in reversed(ch):
        o = j.find("origin"); t = t + R @ np.array([float(v) for v in o.get("xyz").split()]); R = R @ rpyR(*[float(v) for v in o.get("rpy").split()])
    return R, t
def quat_xyzw(R):   # Isaac Lab 3.0 spawner orientation is (x, y, z, w)
    from scipy.spatial.transform import Rotation
    return tuple(float(v) for v in Rotation.from_matrix(R).as_quat())
CV2GL = np.diag([1.0, -1.0, -1.0])           # ROS optical (z fwd, y down) -> USD camera (−z fwd, y up)
WORLD2GL = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)   # columns = USD camera x,y,z in a world-convention cam frame (x fwd, z up)
R, t = pose("birdeye_camera_link"); t = t + R[:, 0] * 0.021
bk = dict(fx=853.374030, fy=852.156716, cx=675.548425, cy=371.440164)   # 팀 실측 camera_wide.yaml (1280x720)
bcfg = sim_utils.PinholeCameraCfg.from_intrinsic_matrix([bk["fx"], 0, bk["cx"], 0, bk["fy"], bk["cy"], 0, 0, 1], 1280, 720, clipping_range=(0.02, 20.0))
try:   # 팀 sim 과 같은 OpenCV 어안 왜곡 D
    from isaaclab.sim.spawners.sensors import OpenCvFisheyeDistortionCfg
    bcfg.distortion = OpenCvFisheyeDistortionCfg(**bk, k1=-0.128704, k2=0.012667, k3=-0.039807, k4=0.022852, image_size=(1280, 720))
except Exception as e:
    print("[demo] birdeye 왜곡 생략:", e, flush=True)
bcfg.func("/World/Robot/Geometry/base_link/birdeye", bcfg, translation=tuple(t), orientation=quat_xyzw(R @ WORLD2GL))
wrist = next(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "arm_link5_wrist_roll" and p.HasAPI(UsdPhysics.RigidBodyAPI))
try:
    R, t = pose("d435_color_optical_frame", root="arm_link5_wrist_roll")   # 컬러 렌즈 (2026-10-29)
    dk = sim_utils.PinholeCameraCfg.from_intrinsic_matrix([605.9, 0, 324.0, 0, 605.4, 245.9, 0, 0, 1], 640, 480, clipping_range=(0.03, 10.0))
    dk.func(wrist + "/wrist_d435", dk, translation=tuple(t), orientation=quat_xyzw(R @ CV2GL))
except Exception as e:
    print("[demo] D435 camera skipped:", e, flush=True)

# --- 크랭크·링크 막대 보이는 모양 움직임 (config/crank_linkage.json, tools/make_crank_linkage.py; 물리 무관)
from pxr import UsdGeom, Gf, Vt
from config.nt_train_kit import load_obj   # trimesh 없어도 됨
CK = json.loads((HERE / "config/crank_linkage.json").read_text())
wp = stage.GetPrimAtPath(wrist)
for c in wp.GetChildren():
    if c.GetName() in CK["hide_visuals"]:
        UsdGeom.Imageable(c).MakeInvisible()
ck_xf = {}
for nm, f in CK["meshes"].items():
    m = load_obj(HERE / f)
    g = UsdGeom.Mesh.Define(stage, f"{wrist}/crank_anim_{nm}")
    g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(m.vertices.astype(np.float32)))
    g.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces))); g.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.reshape(-1).tolist()))
    g.CreateDisplayColorAttr([Gf.Vec3f(*CK["colors"][nm])])
    ck_xf[nm] = g.AddTransformOp()
CK_a = np.array(CK["axis"]); CK_C = np.array(CK["center_m"]); CK_tab = np.array(CK["tick_table_mm"])[::-1]
def _axR(a, t):
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]); return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
def _align(u0, u):
    u0 = u0 / np.linalg.norm(u0); u = u / np.linalg.norm(u); v = np.cross(u0, u); s = np.linalg.norm(v)
    return np.eye(3) if s < 1e-12 else _axR(v / s, math.atan2(s, float(u0 @ u)))
def _set(op, R, t):
    M = np.eye(4); M[:3, :3] = R.T; M[3, :3] = t; op.Set(Gf.Matrix4d(M.tolist()))
def crank_update(q):   # q = 손가락 관절값 [m] (a·b 평균)
    tick = float(np.interp(CK["gap0_mm"] + 2000 * q, CK_tab[:, 1], CK_tab[:, 0]))
    R = _axR(CK_a, CK["sign"] * (tick - CK["tick_cad"]) * 2 * math.pi / 4096)
    _set(ck_xf["crank"], R, CK_C - R @ CK_C)
    for f in "ab":
        P0 = np.array(CK["rod_pin_end_m"][f]); F0 = np.array(CK["rod_finger_end_m"][f])
        P = CK_C + R @ (P0 - CK_C); F = F0 + np.array(CK["finger_axis"][f]) * q
        Rr = _align(F0 - P0, F - P); _set(ck_xf[f"rod_{f}"], Rr, P - Rr @ P0)
    return tick
crank_update(parked_joint_pos()["gripper_finger_a_joint"])
print("[demo] 크랭크·막대 움직임 켬 (실물 틱 각도, 회전 부호", CK["sign"], ")", flush=True)

sim.set_camera_view(eye=(sx_ + 0.7, sy_ - 0.8, 0.6), target=(sx_, sy_, 0.1))
sim.reset()
try:   # 카메라 화면 창 2개(실시간)
    from omni.kit.viewport.utility import create_viewport_window
    cam_windows = [create_viewport_window("birdeye (team K + fisheye D)", width=640, height=360, position_x=1270, position_y=60, camera_path="/World/Robot/Geometry/base_link/birdeye"),
                   create_viewport_window("wrist_d435 (factory K)", width=640, height=480, position_x=1270, position_y=460, camera_path=wrist + "/wrist_d435")]
    print("[demo] 카메라 창 2개 열림", flush=True)
except Exception as e:
    cam_windows = []; print("[demo] 카메라 창 실패:", e, flush=True)
T = lambda x: x.torch if hasattr(x, "torch") else x
names = robot.joint_names
si = [names.index(f"steer_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
wi = [names.index(f"drive_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
ai = [names.index(f"arm_joint{i}") for i in range(1, 6)]; gi = [names.index(n) for n in ("gripper_finger_a_joint", "gripper_finger_b_joint")]
park = parked_joint_pos()
p0 = json.loads((HERE / "config/initial_pose.json").read_text())["poses"]["C1_P0"]["q_rad"]
qpark = np.array([park[f"arm_joint{i}"] for i in range(1, 6)]); qp0 = np.array([p0[f"arm_joint{i}"] for i in range(1, 6)])
g_closed = park["gripper_finger_a_joint"]; g_open = 0.010246; G0_MM = 58.50723799617533
# 실물 전체 성공 실행 14개(tools/make_all_replays.sh). 9 / 0 키로 고름. 기본 = A9F5 실물 최단 022813Z
RUN_LABEL = {"ARM_20261025T122354Z": "A9 1/5", "ARM_20261025T124745Z": "A9 2/5", "ARM_20261025T131012Z": "A9 3/5", "ARM_20261026T014434Z": "A9 4/5",
             "ARM_20261026T014742Z": "A9 5/5", "ARM_20261026T020137Z": "A9F 1.5x", "ARM_20261026T021016Z": "A9F2", "ARM_20261026T021426Z": "A9F3",
             "ARM_20261026T021752Z": "A9F4", "ARM_20261026T022813Z": "A9F5 최단", "ARM_20261026T023047Z": "A9F5", "ARM_20261026T023249Z": "A9F5",
             "ARM_20261025T133451Z": "A10 오른쪽 끝 1/2", "ARM_20261025T134033Z": "A10 오른쪽 끝 2/2"}
RUNS = [r for r in RUN_LABEL if (HERE / f"config/arm_replays/{r}.json").exists()]
cur_run = [RUNS.index("ARM_20261026T022813Z") if "ARM_20261026T022813Z" in RUNS else 0]
ASCII_LABEL = {"최단": "fastest", "오른쪽 끝": "right-end"}
import omni.ui as ui   # 화면 안내 창(지금 고른 실행 번호·마지막 동작). 글꼴 때문에 영문
info_win = ui.Window("Arm replay", width=560, height=96, position_x=130, position_y=880)
with info_win.frame:
    with ui.VStack():
        info_run = ui.Label("", style={"font_size": 20}); info_last = ui.Label("", style={"font_size": 18})
def info(last=None):
    lab = RUN_LABEL[RUNS[cur_run[0]]]
    for k_, v_ in ASCII_LABEL.items():
        lab = lab.replace(k_, v_)
    info_run.text = f"Run {cur_run[0] + 1}/{len(RUNS)}: {lab}  ({RUNS[cur_run[0]][4:]})   speed x{state['speed']:.2f}   [9/0 change]"
    if last is not None:
        info_last.text = last
def load_run(i):
    global RT, RQ, RG, SEG
    RP = json.loads((HERE / f"config/arm_replays/{RUNS[i]}.json").read_text())
    RT = np.array(RP["t_s"]); RQ = np.array(RP["arm_q_rad"]); RG = np.array(RP["finger_q_m"]); SEG = RP["segments"]
    global T_P0, T_GRASP
    T_P0 = SEG[0]["wall_s"][1]                      # 대기자세(P0) = 꺼내기 끝
    io = int(np.argmax(RG)); ic = io + int(np.argmax(RG[io:] < RG[io] - 0.001))   # 가장 열린 뒤 닫히기 시작하는 순간 = 퍽 집기 자세
    T_GRASP = float(RT[ic])
    print(f"[demo] 팔 궤적 = {RUNS[i]} ({RUN_LABEL[RUNS[i]]}): 꺼내기 {SEG[0]['wall_s'][1] - SEG[0]['wall_s'][0]:.1f} s · 복귀 {SEG[-1]['wall_s'][1] - SEG[-1]['wall_s'][0]:.1f} s · 전체 {SEG[-1]['wall_s'][1]:.1f} s (1배 기준)", flush=True)
load_run(cur_run[0])
def rp_at(t):
    return np.array([np.interp(t, RT, RQ[:, i]) for i in range(5)]), float(np.interp(t, RT, RG))
kin, cfg, _ = nt.from_config(str(HERE / "config/swerve.yaml"))
cfg.max_linear, cfg.max_angular, cfg.max_linear_accel, cfg.max_angular_accel = 0.26, 1.675, 1.0, 6.0   # swerve.yaml limits_for_training
drv = nt.SwerveDrive(kin, cfg)
kb = Se2Keyboard(Se2KeyboardCfg(v_x_sensitivity=0.26, v_y_sensitivity=0.26, omega_z_sensitivity=1.2, sim_device=DEV))
state = {"arm_from": qpark.copy(), "arm_to": qpark.copy(), "arm_t": 1.0, "grip": g_closed, "reset": False, "play": None, "speed": 2.0,   # 재생 배속(기본 2배, [ ] 로 0.5~4)
         "sel": 2, "jog": 0}   # 팔 자유 조작: 고른 관절(0~4 = J1~J5), W/S 방향
_lim = getattr(robot.data, "joint_pos_limits", None)
_lim = T(_lim if _lim is not None else robot.data.soft_joint_pos_limits)[0, ai].cpu().numpy()   # (5, 2) rad
JOG_RATE = 0.8   # rad/s
def on_key(e, *a):
    n = getattr(e.input, "name", e.input)   # 문자 입력 이벤트는 input 이 str
    if n in ("W", "S"):   # 누르고 있는 동안만
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            state["jog"] = 1 if n == "W" else -1
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            state["jog"] = 0
        return True
    if e.type != carb.input.KeyboardEventType.KEY_PRESS:
        return True
    if n == "C" and cam_windows:   # 카메라 창 끄기/켜기(끄면 화면이 빨라짐)
        v_ = not cam_windows[0].visible
        for w_ in cam_windows:
            w_.visible = v_
        print("[demo] 카메라 창", "켬" if v_ else "끔", flush=True); return True
    if n in ("KEY_3", "KEY_4", "KEY_5", "KEY_6", "KEY_7"):
        state["sel"] = int(n[-1]) - 3; print(f"[demo] 팔 조작 관절 J{state['sel'] + 1} (W +, S −)", flush=True); return True
    cur = np.array([float(v) for v in T(robot.data.joint_pos)[0, ai]])
    if n in ("KEY_9", "KEY_0") and state["play"] is None:   # 궤적(성공 실행) 바꾸기
        cur_run[0] = (cur_run[0] + (1 if n == "KEY_0" else -1)) % len(RUNS); load_run(cur_run[0]); info(); return True
    if n in ("P", "O") and state["play"] is None:   # P 대기자세(P0), O 퍽 집기 자세(그 실행에서 그리퍼가 닫히기 시작한 순간, 그리퍼 열림) → G 로 닫기
        q_t, g_t = rp_at(T_P0 if n == "P" else T_GRASP)
        in_stand = min(np.abs(cur - rp_at(SEG[0]["wall_s"][0])[0]).max(), np.abs(cur - rp_at(SEG[-1]["wall_s"][1])[0]).max()) < 0.12
        if in_stand:   # 받침 안이면 실측 꺼내기 먼저, 끝나면 그 자세로
            a_, b_ = SEG[0]["wall_s"]; q_s, _ = rp_at(a_)
            state.update(arm_from=cur, arm_to=q_s, arm_t=0.0, arm_T=0.3, play={"t": a_, "end": b_, "grip": False, "queue": [], "after": (q_t, g_t if n == "O" else None)})
        else:
            state.update(arm_from=cur, arm_to=q_t, arm_t=0.0, arm_T=max(0.3, float(np.abs(q_t - cur).max()) / 1.2))
            if n == "O":
                state["grip"] = g_t
        info(f"{'P: standby (P0)' if n == 'P' else 'O: puck grasp pose (gripper open, G = close)'}  from run {cur_run[0] + 1}")
        print("[demo]", "대기자세 P0" if n == "P" else f"퍽 집기 자세(t={T_GRASP:.2f} s, 그리퍼 {G0_MM + 2000 * g_t:.0f} mm)", RUNS[cur_run[0]], flush=True); return True
    if n in ("KEY_1", "KEY_2", "M"):   # 1 = 고른 실행의 실측 P0→받침 복귀
        # 받침 안에서는 항상 실측 경로로만 움직임: 받침에 있는데 1 → 꺼내기 먼저, 받침 밖에서 2·M → 복귀 먼저
        #   (받침 안에서 P0 로 직선 보간하면 기록에 없는 길로 빠져나오다 받침 벽에 걸렸다 말았다 함)
        OUT, BACK, FULL = (*SEG[0]["wall_s"], False), (*SEG[-1]["wall_s"], False), (SEG[0]["wall_s"][0], SEG[-1]["wall_s"][1], True)
        in_stand = min(np.abs(cur - rp_at(SEG[0]["wall_s"][0])[0]).max(), np.abs(cur - rp_at(SEG[-1]["wall_s"][1])[0]).max()) < 0.12
        queue = {"KEY_1": [OUT, BACK] if in_stand else [BACK], "KEY_2": [OUT] if in_stand else [BACK, OUT], "M": [FULL] if in_stand else [BACK, FULL]}[n]
        a_, b_, g_ = queue[0]; q_start, _ = rp_at(a_)
        blend = max(0.3, float(np.abs(q_start - cur).max()) / 1.2)   # 시작 자세까지 관절 최대 1.2 rad/s
        state.update(arm_from=cur, arm_to=q_start, arm_t=0.0, arm_T=blend, play={"t": a_, "end": b_, "grip": g_, "queue": queue[1:]})
        dur = sum(q_[1] - q_[0] for q_ in queue) / state["speed"]   # 이어 붙인 구간 합(표시용)
        play_no[0] += 1
        info(f"{dict(KEY_2='2: stand -> P0 (take out)', KEY_1='1: P0 -> stand (return)', M='M: full mission')[n]}  from run {cur_run[0] + 1}  {dur:.1f} s" + ("  (out first)" if n == "KEY_1" and len(queue) > 1 else "  (return first)" if len(queue) > 1 else ""))
        print(f"[demo] 재생 #{play_no[0]}:", {"KEY_2": SEG[0]["name"], "KEY_1": SEG[-1]["name"], "M": "미션 전체"}[n], f"실행 {cur_run[0] + 1}/{len(RUNS)}", RUNS[cur_run[0]], f"{dur:.1f} s ({state['speed']:g}배)", "+".join("꺼내기" if q_[0] == SEG[0]["wall_s"][0] and not q_[2] else "복귀" if q_[0] == SEG[-1]["wall_s"][0] else "전체" for q_ in queue), flush=True)
    elif n in ("LEFT_BRACKET", "RIGHT_BRACKET"):
        state["speed"] = min(4.0, max(0.5, state["speed"] * (1.25 if n == "RIGHT_BRACKET" else 0.8))); print(f"[demo] 재생 배속 {state['speed']:.2f}배", flush=True); info()
    elif n == "G":
        state["grip"] = g_open if abs(state["grip"] - g_closed) < 1e-6 else g_closed; print("[demo] 그리퍼:", "열기 79 mm" if state["grip"] == g_open else "닫기 16 mm", flush=True)
    elif n in ("R", "BACKSPACE", "HOME"):
        state["reset"] = True; print("[demo] 리셋 요청", flush=True)
    return True
play_no = [0]; info("press 2 / 1 / M")
sub = kb._input.subscribe_to_keyboard_events(kb._keyboard, on_key)   # same keyboard handle the Se2Keyboard device acquired
print(__doc__, flush=True)
steer_t = torch.zeros((1, 4), device=DEV); wheel_t = torch.zeros((1, 4), device=DEV)
import time
RENDER_EVERY = 4; ck_last = [1e9]
k = 0; t_sim = 0.0; wall0 = fps0 = time.perf_counter(); since_draw = 0; frames = 0; t_fps = 0.0
while app.is_running():
    if state["reset"]:
        robot.write_root_pose_to_sim_index(root_pose=torch.tensor([list(START)], device=DEV))
        robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=DEV))
        dj = T(robot.data.default_joint_pos).clone()
        robot.write_joint_position_to_sim_index(position=dj); robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(dj))
        drv = nt.SwerveDrive(kin, cfg); state.update(arm_from=qpark.copy(), arm_to=qpark.copy(), arm_t=1.0, grip=g_closed, reset=False, play=None); print("[demo] 처음 자리로(재생도 멈춤)", flush=True); ck_last[0] = 1e9
    if k % 4 == 0:   # 50 Hz command
        c = kb.advance(); c = c.tolist() if hasattr(c, "tolist") else list(c)
        drv.set_command(float(c[0]), float(c[1]), float(c[2]), t_sim)
    if k % 2 == 0:   # 100 Hz swerve controller
        jp, jv = T(robot.data.joint_pos)[0], T(robot.data.joint_vel)[0]
        ms = drv.update(t_sim, 0.01, [nt.ModuleState(float(jp[si[i]]), float(jv[wi[i]]), float(jv[si[i]])) for i in range(4)])
        steer_t[0] = torch.tensor([m.steer_angle for m in ms]); wheel_t[0] = torch.tensor([m.wheel_speed for m in ms])
    pl = state["play"]
    if state["arm_t"] < 1.0:          # 시작 자세로 짧게 맞추기(0.3 s)
        state["arm_t"] = min(state["arm_t"] + DT / state.get("arm_T", 0.3), 1.0)
        s = 0.5 - 0.5 * math.cos(math.pi * state["arm_t"])
        qa = state["arm_from"] + (state["arm_to"] - state["arm_from"]) * s
    elif pl is not None and pl.get("delay", 0) > 0:   # 복귀 전 그리퍼 닫힐 때까지 대기
        pl["delay"] -= DT; qa = state["arm_to"]
    elif pl is not None:              # 실측 궤적 재생(dir −1 = 거꾸로)
        qa, gq = rp_at(pl["t"])
        if pl["grip"]:
            state["grip"] = gq
        d_ = pl.get("dir", 1); pl["t"] += DT * state["speed"] * d_
        if (pl["t"] - pl["end"]) * d_ >= 0:
            if pl.get("queue"):   # 이어서 다음 구간(시작 자세로 짧게 맞춘 뒤)
                a_, b_, g_ = pl["queue"].pop(0); q_n, _ = rp_at(a_)
                state.update(arm_from=qa.copy(), arm_to=q_n, arm_t=0.0, arm_T=max(0.3, float(np.abs(q_n - qa).max()) / 1.2))
                pl.update(t=a_, end=b_, grip=g_)
            elif pl.get("after") is not None:   # P·O: 꺼내기 뒤 목표 자세로
                q_n, g_n = pl["after"]; state["play"] = None
                state.update(arm_from=qa.copy(), arm_to=q_n, arm_t=0.0, arm_T=max(0.3, float(np.abs(q_n - qa).max()) / 1.2))
                if g_n is not None:
                    state["grip"] = g_n
            else:
                state["play"] = None; state["arm_from"] = state["arm_to"] = qa.copy(); print("[demo] 재생 끝", flush=True)
    else:
        if state["jog"]:   # 팔 자유 조작
            j_ = state["sel"]; state["arm_to"] = state["arm_to"].copy()
            state["arm_to"][j_] = float(np.clip(state["arm_to"][j_] + state["jog"] * JOG_RATE * DT, _lim[j_, 0], _lim[j_, 1]))
            if k % 100 == 0:
                print(f"[demo] J{j_ + 1} 목표 {math.degrees(state['arm_to'][j_]):.1f}° (범위 {math.degrees(_lim[j_, 0]):.0f}~{math.degrees(_lim[j_, 1]):.0f}°)", flush=True)
        qa = state["arm_to"]
    tgt = T(robot.data.default_joint_pos).clone()
    tgt[0, si] = steer_t[0]
    for i, j in enumerate(ai):
        tgt[0, j] = float(qa[i])
    tgt[0, gi[0]] = tgt[0, gi[1]] = state["grip"]
    robot.set_joint_position_target(tgt)
    robot.set_joint_velocity_target_index(target=wheel_t, joint_ids=wi)
    # 물리는 항상 실제 시간(200 Hz)을 따라가고, 화면은 따라잡았을 때만 그림(최대 50 Hz). 화면이 느려도 움직임 속도는 실물 그대로
    since_draw += 1
    draw = (t_sim + DT >= time.perf_counter() - wall0 and since_draw >= RENDER_EVERY) or since_draw >= 40
    if draw:
        qf = float(T(robot.data.joint_pos)[0, gi].mean())
        if abs(qf - ck_last[0]) > 2e-5:   # 손가락이 움직일 때만 크랭크·막대 갱신(실제 손가락 위치 기준)
            with Sdf.ChangeBlock():
                crank_update(qf)
            ck_last[0] = qf
    robot.write_data_to_sim(); sim.step(render=draw); robot.update(DT)
    k += 1; t_sim += DT
    if draw:   # 실제 시간에 맞춤(빠르면 기다림)
        since_draw = 0; frames += 1
        lag = t_sim - (time.perf_counter() - wall0)
        if lag > 0:
            time.sleep(lag)
        elif lag < -0.25:
            wall0 = time.perf_counter() - t_sim   # 그래도 많이 밀리면(물리 자체가 느림) 기준 재설정
    if k % 200 == 0 and (abs(c[0]) + abs(c[1]) + abs(c[2])) > 1e-6:   # 1 s 마다 명령 vs 실제 속도
        vb = T(robot.data.root_lin_vel_b)[0]; wb = T(robot.data.root_ang_vel_b)[0, 2]; jv = T(robot.data.joint_vel)[0, wi].abs()
        print(f"[demo] 속도 명령 {math.hypot(c[0], c[1]):.3f} m/s·{c[2]:+.2f} rad/s → 실제 {float(vb[:2].norm()):.3f} m/s·{float(wb):+.2f} rad/s, 바퀴 {float(jv.mean()):.2f}/{7.9137:.2f} rad/s", flush=True)
    if k % 2000 == 0:   # 화면 fps · 실제 시간 대비 물리 속도
        el = time.perf_counter() - fps0
        print(f"[demo] 화면 {frames / el:.0f} fps · 실제 시간의 {(t_sim - t_fps) / el:.2f}배", flush=True); fps0 = time.perf_counter(); frames = 0; t_fps = t_sim
app.close()
