"""학습 묶음 동작을 영상으로 (Isaac Lab 3.0, 팀 미로 안 env 8개) — 정책 학습이 아니라 '스크립트 명령 재생'으로 같은 로봇 설정(nt_train_kit)이
여러 동작(팔 실측 궤적·그리퍼·스워브 주행)에 버티는지 보여주는 것.
GUI 창의 뷰포트 2개를 프레임마다 파일로 캡처 → 글자 붙여 mp4 (headless RTX 카메라는 흰 화면만 나와서 GUI 뷰포트로 찍음. 다른 창은 찍히지 않음).
환경: 팀 sim(NT_TEAM_SIM, 기본 /home/pa25/sim/robot_sim, 읽기만)의 maze_real.usd + 카펫 바닥 + 파노라마 돔 + 천장 RectLight 6개 (demo_keyboard.py 와 같은 값).
배치: maze_real_layout.json 칸 중심(칸 번호 j*6+i)에 env 하나씩, 열린 이웃 쪽을 보게. 주행 env 는 0.45 s 뒤 자세에서 로봇 바닥 외곽(parked 직사각형,
팔·그리퍼가 오른쪽 위로 나와 y −0.31 m)과 벽판·기둥·받침 사이가 3 cm 아래로 줄어드는 명령이면 이동(필요하면 회전도)을 0 으로 → 벽에 안 박음.
또 base 중심이 자기 칸·열린 통로(이웃 env 와 안 겹치게 고른 것) 밖으로 나가지 않게.
env 별 동작(PLAN): 미션 전체 · 꺼내기→집기 자세→그리퍼 닫기/열기→복귀(크랭크 확인) · 직진+제자리 회전 · 옆으로 · 대각+회전 · 주행 자세 S자 ·
1 s 마다 바뀌는 무작위 몸 속도 · 받침 밖 무작위 팔 목표. 크랭크·링크 막대는 kit.CrankVisuals 로 손가락 따라 움직임.
왼쪽 = 미로 전체(env 번호 표시), 오른쪽 = env 하나씩 돌아가며 가까이(그리퍼 단계는 그리퍼 가까이). 위 글자 = 측정값.
결과: images/train_kit_check.mp4, images/train_kit_{1_maze,2_crank_closed,3_crank_open,4_mission,5_side,6_random_arm}.png, config/record_train_kit.json
실행: OMNI_KIT_ACCEPT_EULA=YES <Isaac Lab 3.0 python> tools/record_train_kit.py   (NT_REC_SECONDS=6 이면 짧게 시험)"""
import json, math, os, sys, subprocess, shutil, time
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(HERE))
TEAM = Path(os.environ.get("NT_TEAM_SIM", "/home/pa25/sim/robot_sim"))
DUR = float(os.environ.get("NT_REC_SECONDS", "39"))
if not (TEAM / "maze_real_layout.json").exists() or not (TEAM / "usd/maze_real.usd").exists():
    sys.exit(f"[rec] 팀 sim 미로 없음: {TEAM} — NT_TEAM_SIM=<Project2_Turtle_Ninja/sim/robot_sim> 로 지정 (maze_real_layout.json, usd/maze_real.usd 필요)")
DEV = "cuda:0"
from isaaclab.app import AppLauncher
app = AppLauncher({"headless": False, "visualizer": ["kit"]}).app
import numpy as np, torch, omni.usd
from PIL import Image, ImageDraw, ImageFont
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from config import nt_train_kit as kit

L = json.loads((TEAM / "maze_real_layout.json").read_text())
CELL = {c["j"] * 6 + c["i"]: np.array(c["xy"], float) for c in L["cells"]}
OPEN = {tuple(sorted(p)) for p in L["passable_pairs"]}
HALF = np.array(L["grid"]["cell_m"]) / 2
# 벽판·기둥(상자) + 바닥 받침(원) — 벽 여유 계산용
WALLS = [(np.array(b["center"][:2]), np.array(b["size"][:2]) / 2) for b in L["collision_boxes"]]
MOUNTS = [(np.array(m["xy"]), m["r"]) for m in L["mounts"]]
# env 별 계획: cell = 시작 칸, face = 바라볼 열린 이웃(없으면 yaw°), edges = 주행 허용 통로(열린 칸 쌍)
# center=True: base 를 칸 중심에(제자리 회전용). False: 바닥 외곽 중심을 칸 중심에(직선 주행 양옆 여유 같게)
PLAN = [dict(cell=7, face=6, kind="mission", edges=[], label="미션 전체 재생(실측 29 s)"),
        dict(cell=8, face=2, kind="grasp", edges=[], label="꺼내기→집기 자세→그리퍼 닫기·열기→복귀"),
        dict(cell=22, face=23, kind="fwd_turn", edges=[(22, 23)], center=True, label="직진 + 제자리 회전"),
        dict(cell=13, yaw=90.0, kind="side", edges=[(12, 13), (13, 14)], label="옆으로(스워브 평행 이동)"),
        dict(cell=10, face=11, kind="diag", edges=[(10, 11)], center=True, label="대각 + 제자리 회전"),
        dict(cell=0, face=1, kind="s_curve", edges=[(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)], label="주행 자세 S자(앞·뒤)"),
        dict(cell=19, face=20, kind="rand_body", edges=[(18, 19)], center=True, label="무작위 몸 속도(1 s 마다)"),
        dict(cell=21, face=20, kind="rand_arm", edges=[], label="받침 밖 무작위 팔 목표")]
N = len(PLAN)
for p in PLAN:
    for e in p["edges"]:
        assert tuple(sorted(e)) in OPEN, e
    if "face" in p:
        assert tuple(sorted((p["cell"], p["face"]))) in OPEN
        d = CELL[p["face"]] - CELL[p["cell"]]; p["yaw"] = math.atan2(d[1], d[0])
    else:
        p["yaw"] = math.radians(p["yaw"])


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(visible=False))
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=1800.0, texture_file=str(TEAM / "usd/textures/pano_191331.jpg"), texture_format="latlong"))
    floor = AssetBaseCfg(prim_path="/World/Floor", spawn=sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/floor_carpet.usda")), init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0005)))
    maze = AssetBaseCfg(prim_path="/World/Maze", spawn=sim_utils.UsdFileCfg(usd_path=str(TEAM / "usd/maze_real.usd")))
    # 처음 생성 자리는 미로 밖(y 3.5 m), reset 직후 칸 중심으로 옮김
    robot = kit.robot_cfg("{ENV_REGEX_NS}/Robot").replace(init_state=kit.robot_cfg().init_state.replace(pos=(0.0, 3.5, 0.075)))


DT, EVERY, W, H = 0.005, 8, 960, 540   # 25 fps 영상
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=DEV))
scene = InteractiveScene(SceneCfg(num_envs=N, env_spacing=0.8, replicate_physics=False))
stage = omni.usd.get_context().get_stage()
n_f = kit.apply_collision_filters(stage)
crank = kit.CrankVisuals(stage)
from pxr import UsdGeom, UsdLux, Gf, Usd
for i_, (x_, y_) in enumerate([(x, y) for x in (-1.2, 1.2) for y in (-2.0, 0.0, 2.0)]):   # 팀 sim add_ceiling_lights
    lt = UsdLux.RectLight.Define(stage, f"/World/CeilingLights/l{i_}")
    lt.CreateWidthAttr(0.6); lt.CreateHeightAttr(1.2); lt.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92)); lt.CreateIntensityAttr(10000.0)
    UsdGeom.XformCommonAPI(lt.GetPrim()).SetTranslate(Gf.Vec3d(x_, y_, 2.6))
# 로봇 바닥 외곽(parked, 바퀴 0°, 로봇 기준) — 팔·그리퍼·D435 가 오른쪽(−y) 위(벽 높이 안)로 나와 있음
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeUntransformedBound(stage.GetPrimAtPath("/World/envs/env_0/Robot")).ComputeAlignedRange()
lo, hi = np.array(bb.GetMin()), np.array(bb.GetMax())
R_FOOT = float(max(math.hypot(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])))   # base 중심 제자리 회전 반지름
C_FOOT = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])
def _ring(a, b):   # 직사각형 둘레 점
    ex = [np.linspace(a[0], b[0], 15), np.linspace(a[1], b[1], 24)]
    return np.array([(x, y) for x in ex[0] for y in (a[1], b[1])] + [(x, y) for y in ex[1] for x in (a[0], b[0])])
FOOT = _ring(lo, hi)                                            # 전체(팔·그리퍼 포함) → 벽판·기둥
LOW = _ring((lo[0], -hi[1]), (hi[0], hi[1]))                    # 바닥 가까운 차체만(좌우 대칭, 높이 7 cm 아래) → 바닥 받침
WC = np.array([c for c, _ in WALLS]); WH = np.array([h for _, h in WALLS]); MC = np.array([c for c, _ in MOUNTS]); MR = np.array([r for _, r in MOUNTS])
ROOM = HALF - 0.05   # base 중심이 있을 수 있는 칸 안 반폭(이웃 env 와 겹침 방지용, 벽은 아래 외곽 검사로)
print(f"[rec] 로봇 바닥 외곽 x {lo[0]:+.3f}~{hi[0]:+.3f} y {lo[1]:+.3f}~{hi[1]:+.3f} m (제자리 회전 반지름 {R_FOOT:.3f} m)", flush=True)
for p in PLAN:   # base 중심 허용 상자: 칸 + 열린 통로
    rects = [(CELL[c] - ROOM, CELL[c] + ROOM) for c in {p["cell"], *[c for e in p["edges"] for c in e]}]
    for a, b in p["edges"]:
        rects.append((np.minimum(CELL[a], CELL[b]) - ROOM, np.maximum(CELL[a], CELL[b]) + ROOM))
    p["rects"] = rects


def inside(p, xy):
    return any(np.all(xy >= a) and np.all(xy <= b) for a, b in p["rects"])


def clearance(xy, yaw):
    """base 위치 xy·yaw 에서 외곽(직사각형 둘레 점) ↔ 벽판·기둥, 차체 ↔ 바닥 받침(높이 ≤ 7.4 cm) 사이 최소 거리 [m] (음수면 겹침)"""
    c, s = math.cos(yaw), math.sin(yaw); Rm = np.array([[c, s], [-s, c]]); P = xy + FOOT @ Rm; Q = xy + LOW @ Rm
    dw = np.linalg.norm(np.maximum(np.abs(P[:, None, :] - WC[None]) - WH[None], 0.0), axis=2).min()
    dm = (np.linalg.norm(Q[:, None, :] - MC[None], axis=2) - MR[None]).min()
    return float(min(dw, dm))


cams, xops = {}, {}
for k, f in (("wide", 15.0), ("close", 20.0)):
    c = UsdGeom.Camera.Define(stage, f"/World/Cam_{k}"); c.CreateFocalLengthAttr(f); c.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    c.CreateHorizontalApertureAttr(20.955); c.CreateVerticalApertureAttr(20.955 * H / W)
    xops[k] = c.AddTransformOp(); cams[k] = (c, f)
sim.reset()
robot = scene["robot"]
T = lambda x: x.torch if hasattr(x, "torch") else x
# 칸 중심으로 옮기기(물리 단계 전). Isaac Lab 3.0 쿼터니언 = (x, y, z, w)
def start_xy(p):
    c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
    return CELL[p["cell"]] if p.get("center") else CELL[p["cell"]] - np.array([c * C_FOOT[0] - s * C_FOOT[1], s * C_FOOT[0] + c * C_FOOT[1]])


pose0 = torch.tensor([[*start_xy(p), 0.075, 0.0, 0.0, math.sin(p["yaw"] / 2), math.cos(p["yaw"] / 2)] for p in PLAN], device=DEV, dtype=torch.float32)
dj = T(robot.data.default_joint_pos).clone()
robot.write_root_pose_to_sim_index(root_pose=pose0); robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((N, 6), device=DEV))
robot.write_joint_position_to_sim_index(position=dj); robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(dj))
robot.set_joint_position_target(dj); scene.write_data_to_sim()
import omni.ui as ui
from omni.kit.viewport.utility import get_active_viewport, create_viewport_window, capture_viewport_to_file
vp_main = get_active_viewport(); vp_main.camera_path = "/World/Cam_wide"
vp_close = create_viewport_window("close-up", width=W, height=H, position_x=60, position_y=60, camera_path="/World/Cam_close").viewport_api
for v_ in (vp_main, vp_close):
    try:
        v_.resolution = (W, H)
    except Exception:
        pass


def look(k, eye, target):
    f = np.asarray(target, float) - np.asarray(eye, float); f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f)
    M = np.eye(4); M[0, :3], M[1, :3], M[2, :3], M[3, :3] = r, u, -f, eye   # USD 행벡터: 행 = 카메라 x·y·z 축, 마지막 = 위치
    xops[k].Set(Gf.Matrix4d(M.tolist())); cams[k] = (cams[k][0], cams[k][1], np.asarray(eye, float), r, u, f)


def project(k, pw):
    _, fl, eye, r, u, f = cams[k]; d = np.asarray(pw) - eye; z = float(d @ f)
    if z <= 0.05:
        return None
    s = W * fl / 20.955
    return W / 2 + s * float(d @ r) / z, H / 2 - s * float(d @ u) / z


WIDE_EYE, WIDE_TGT = np.array([0.0, -3.35, 3.0]), np.array([0.0, -0.12, 0.0])
look("wide", WIDE_EYE, WIDE_TGT)
for _ in range(60):
    app.update()


def grab(vp, path):
    path.unlink(missing_ok=True); capture_viewport_to_file(vp, str(path))
    for _ in range(200):
        app.update()
        if path.exists() and path.stat().st_size > 0:
            time.sleep(0.02); return Image.open(path).convert("RGB").resize((W, H))
    return Image.new("RGB", (W, H))


names = robot.joint_names
ai = [names.index(f"arm_joint{i}") for i in range(1, 6)]
gi = [names.index(n) for n in ("gripper_finger_a_joint", "gripper_finger_b_joint")]
si = [names.index(f"steer_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
wi = [names.index(f"drive_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
bi_wrist = robot.body_names.index("arm_link5_wrist_roll")
lim = T(getattr(robot.data, "joint_pos_limits", None) if getattr(robot.data, "joint_pos_limits", None) is not None else robot.data.soft_joint_pos_limits)[0, ai].cpu().numpy()
rp = kit.Replay()
q_park, g_closed = dj[0, ai].cpu().numpy(), float(dj[0, gi[0]])
q_grasp, g_open = rp.pose("grasp")
OUT, BACK, FULL = rp.segment("out"), rp.segment("back"), rp.segment("full")
q_back0 = rp.at(BACK[0])[0]
rng = np.random.default_rng(20261027)


def rand_arm_q():   # 기록된 받침 밖 자세(P0·올리기·집기·접어 들기·빈 그리퍼 닫기 구간) + 관절마다 N(0, 0.12 rad), 관절 범위 안
    w = [(OUT[1], 11.8), (20.75, BACK[0])]; a, b = w[int(rng.integers(2))]
    return np.clip(rp.at(float(rng.uniform(a, b)))[0] + rng.normal(0, 0.12, 5), lim[:, 0] + 0.02, lim[:, 1] - 0.02)


def blend(q, g, dur=None):
    return ("blend", q, g, dur)


GRIP_SHOW = [blend(q_grasp, g_open, 1.5), ("hold", 0.6), ("grip", g_closed, 1.4), ("grip", g_open, 1.4), ("grip", g_closed, 1.4), blend(q_back0, g_closed, 1.5), ("replay", *BACK, False), ("hold", 1.5)]
ARM_SEQ = {"mission": [("hold", 1.0), ("replay", *FULL, True)],
           "grasp": [("hold", 1.0), ("replay", *OUT, False)] + GRIP_SHOW + [("replay", *OUT, False)] + GRIP_SHOW,
           "rand_arm": [("hold", 1.0), ("replay", *OUT, False)] + [blend(rand_arm_q(), [g_open, g_closed][int(rng.integers(2))]) for _ in range(14)] + [blend(q_back0, g_closed, 1.5), ("replay", *BACK, False)]}


class ArmSeq:
    """팔 목표 순서: hold(유지) · replay(실측 궤적 a→b, 그리퍼도 기록대로/닫힌 채) · blend(자세로 코사인 보간, 관절 최대 1 rad/s) · grip(그리퍼 목표 바꾸고 기다림)"""
    def __init__(self, seq):
        self.seq, self.i, self.t, self.q, self.g = list(seq), 0, 0.0, q_park.copy(), g_closed; self.q0, self.label = None, ""

    def step(self, dt):
        while self.i < len(self.seq):
            a = self.seq[self.i]
            if a[0] == "hold" and self.t < a[1]:
                self.label = "유지"; break
            if a[0] == "grip" and self.t < a[2]:
                self.g = a[1]; self.label = "그리퍼 닫기" if a[1] == g_closed else "그리퍼 열기"; break
            if a[0] == "replay" and self.t <= a[2] - a[1]:
                self.q, g = rp.at(a[1] + self.t)
                if a[3]:
                    self.g = g
                self.label = "실측 꺼내기" if a[1] == OUT[0] and a[2] == OUT[1] else "실측 복귀" if a[1] == BACK[0] else "실측 미션"; break
            if a[0] == "blend":
                if self.q0 is None:
                    self.q0 = self.q.copy(); self.dur = a[3] or max(1.2, float(np.abs(a[1] - self.q0).max()) / 1.0)
                if self.t < self.dur:
                    s = 0.5 - 0.5 * math.cos(math.pi * self.t / self.dur); self.q = self.q0 + (a[1] - self.q0) * s; self.g = a[2]
                    self.label = "자세로 이동"; break
                self.q = np.array(a[1]); self.q0 = None
            self.i += 1; self.t = 0.0
        else:
            self.label = "끝(유지)"
        self.t += dt
        return self.q, self.g


arms = [ArmSeq(ARM_SEQ.get(p["kind"], [])) for p in PLAN]
rand_cmd = {}


def body_cmd(e, t):
    """env e 의 몸 속도 명령(몸 기준 vx, vy, ω) — 시간표"""
    k = PLAN[e]["kind"]
    if t < 1.0 or k in ARM_SEQ:
        return 0.0, 0.0, 0.0
    t -= 1.0
    if k == "fwd_turn":   # 앞 0.32 m → 멈춤 → 반 바퀴 → 다시 앞(가운데로) → 반 바퀴 …
        c = t % 5.42
        return (0.2, 0.0, 0.0) if c < 1.6 else (0.0, 0.0, 1.2) if 2.2 <= c < 2.2 + math.pi / 1.2 else (0.0, 0.0, 0.0)
    if k == "side":
        c = t % 6.0
        return (0.0, 0.2 if c < 1.5 or c >= 4.5 else -0.2, 0.0)
    if k == "diag":   # 몸 기준 대각으로 갔다 오기 → 제자리 90° 회전 → (방향 바뀐 채) 다시 대각 …
        c = t % (3.0 + math.pi / 2)
        return (0.13, 0.13, 0.0) if c < 1.2 else (-0.13, -0.13, 0.0) if c < 2.4 else (0.0, 0.0, 0.0) if c < 2.7 else (0.0, 0.0, 1.0) if c < 2.7 + math.pi / 2 else (0.0, 0.0, 0.0)
    if k == "s_curve":   # 앞으로 S자 11 s → 뒤로 S자 11 s … (줄 방향 그대로, 회전은 ±0.32 rad 흔들기만)
        c = t % 22.0
        return (0.2 if c < 11.0 else -0.2, 0.0, 0.5 * math.cos(2 * math.pi * (c % 11.0) / 4.0))
    if k == "rand_body":
        s = int(t)
        if s not in rand_cmd:
            rand_cmd[s] = (float(rng.uniform(-0.2, 0.2)), float(rng.uniform(-0.2, 0.2)), float(rng.uniform(-1.2, 1.2)))
        return rand_cmd[s]
    return 0.0, 0.0, 0.0


drives = [kit.swerve()[0] for _ in range(N)]; nt = kit.swerve()[1]
tgt = dj.clone()
FONT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 24)
FONT_S = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 19)
FONT_B = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 22)
FR = HERE / "images/_frames_train_kit"; shutil.rmtree(FR, ignore_errors=True); FR.mkdir(parents=True)
# 가까이 카메라 시간표: (시작 s, env, 모드)
TOUR = [(0.0, 1, "robot"), (4.6, 1, "grip"), (11.0, 0, "robot"), (15.0, 2, "robot"), (19.0, 3, "robot"), (23.0, 4, "robot"),
        (27.0, 5, "robot"), (31.0, 6, "robot"), (35.0, 7, "robot")]
STILLS = {1.6: "train_kit_1_maze", 6.5: "train_kit_2_crank_closed", 8.0: "train_kit_3_crank_open", 13.5: "train_kit_4_mission", 21.0: "train_kit_5_side", 37.0: "train_kit_6_random_arm"}
MOVING = [e for e, p in enumerate(PLAN) if p["kind"] not in ARM_SEQ]
GUARD_M, LOOK_S = 0.03, 0.45
st = {"k": 0, "f": 0, "guard": [0] * N, "clr": [9.0] * N, "tilt": [0.0] * N, "tour": None, "eye_off": None, "stills": {}}
grip_log = []


def yaw_of(q):   # (x, y, z, w)
    x, y, z, w = [float(v) for v in q]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def frame():
    t = st["k"] * DT
    root = T(robot.data.root_pos_w).cpu().numpy(); quat = T(robot.data.root_quat_w).cpu().numpy()
    fq = T(robot.data.joint_pos)[:, gi].mean(1).cpu().numpy()
    crank.update(fq)
    _, e, mode = [x for x in TOUR if x[0] <= t][-1]
    if st["tour"] != (e, mode):   # 새 구간: 카메라 방향은 그 env 의 지금 yaw 기준으로 고정(회전하는 로봇 따라 돌지 않게)
        st["tour"] = (e, mode); st["yaw_cam"] = yaw_of(quat[e])
    c, s = math.cos(st["yaw_cam"]), math.sin(st["yaw_cam"]); Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    if mode == "grip":
        g = T(robot.data.body_pos_w)[e, bi_wrist].cpu().numpy()
        look("close", g + Rz @ [0.10, -0.24, 0.14], g + [0, 0, -0.02])
    else:
        look("close", root[e] + Rz @ [0.30, -0.28, 0.58], root[e] + [0.03, 0.0, 0.12])
    for v_, cp in ((vp_main, "/World/Cam_wide"), (vp_close, "/World/Cam_close")):   # 창을 건드려 카메라가 바뀌어도(Top 등) 다시 고정
        if str(v_.camera_path) != cp:
            v_.camera_path = cp
    sim.render()
    im = [grab(vp_main, FR / "_w.png"), grab(vp_close, FR / "_c.png")]
    TOP, BOT = 96, 112
    out = Image.new("RGB", (2 * W, TOP + H + BOT), (20, 22, 26)); out.paste(im[0], (0, TOP)); out.paste(im[1], (W, TOP))
    d = ImageDraw.Draw(out)
    d.text((14, 4), f"학습 환경 동작 확인 — 정책 학습 아님(스크립트 명령 재생) · 팀 미로 · env {N}개 GPU · nt_train_kit 설정 · t={t:5.2f} s", font=FONT, fill=(235, 235, 235))
    gap1 = kit.gap_mm(fq[1]); tick1 = float(np.interp(gap1, crank.tab[:, 1], crank.tab[:, 0]))
    ce = min(MOVING, key=lambda i: st["clr"][i])
    d.text((14, 38), f"측정: env1 그리퍼 {gap1:4.1f} mm → 크랭크 틱 {tick1:4.0f} · 주행 env 벽 여유 최소 {100 * st['clr'][ce]:.1f} cm (env {ce}, 바닥 외곽 기준) · "
                     f"최대 기울기 {max(st['tilt']):.1f}° · 벽 막기 {sum(st['guard'])}회", font=FONT_S, fill=(170, 220, 255))
    d.text((14, 66), f"오른쪽 가까이: env {e} — {PLAN[e]['label']}  [{arms[e].label if PLAN[e]['kind'] in ARM_SEQ else '주행'}]" + ("  · 그리퍼 가까이(노란 막대·회색 크랭크가 손가락 따라 움직임)" if mode == "grip" else ""),
           font=FONT_S, fill=(255, 210, 90))
    for i in range(N):   # 미로 전체 화면에 env 번호
        pp = project("wide", root[i] + [0, 0, 0.42])
        if pp:
            x, y = pp[0], pp[1] + TOP; col = (255, 210, 90) if i == e else (255, 255, 255)
            d.rounded_rectangle((x - 14, y - 15, x + 14, y + 15), 6, fill=(20, 22, 26), outline=col, width=2); d.text((x - 6, y - 15), str(i), font=FONT_B, fill=col)
    for i in range(N):   # 아래 범례
        x, y = 14 + (i % 4) * 476, TOP + H + 8 + (i // 4) * 32
        d.text((x, y), f"{i}  {PLAN[i]['label']}", font=FONT_S, fill=(255, 210, 90) if i == e else (210, 210, 210))
    d.text((14, TOP + H + 76), "벽 막기: 0.45 s 뒤 로봇 외곽(팔 포함)↔벽 여유가 3 cm 아래로 줄 명령이면 이동·회전을 0 으로. 팔 = 실물 성공 실행 A9F5 022813Z 실측 궤적.",
           font=FONT_S, fill=(150, 150, 150))
    out.save(FR / f"{st['f']:05d}.png"); st["f"] += 1
    for ts, nm in STILLS.items():
        if nm not in st["stills"] and t >= ts:
            out.save(HERE / f"images/{nm}.png"); st["stills"][nm] = round(t, 2)


k_ctl = 0; sa = np.zeros((N, 4)); wv = np.zeros((N, 4))
for k in range(int(DUR / DT)):
    t = k * DT
    if k % 2 == 0:   # 100 Hz 스워브
        jp, jv = T(robot.data.joint_pos).cpu().numpy(), T(robot.data.joint_vel).cpu().numpy()
        root = T(robot.data.root_pos_w).cpu().numpy(); quat = T(robot.data.root_quat_w).cpu().numpy()
        for e, d_ in enumerate(drives):
            vx, vy, w = body_cmd(e, t)
            if vx or vy or w:   # 벽 막기: 0.45 s 뒤 외곽 여유가 3 cm 아래이고 지금보다 줄면 이동 0, 회전도 그러면 회전 0
                yw = yaw_of(quat[e]); c, s = math.cos(yw), math.sin(yw); xy = root[e, :2]
                nxt = xy + LOOK_S * np.array([c * vx - s * vy, s * vx + c * vy]); now = clearance(xy, yw)
                if (vx or vy) and (not inside(PLAN[e], nxt) or (clearance(nxt, yw + LOOK_S * w) < min(GUARD_M, now))):
                    vx = vy = 0.0; st["guard"][e] += 1
                if w and clearance(xy, yw + LOOK_S * w) < min(GUARD_M, now):
                    w = 0.0; st["guard"][e] += 1
            d_.set_command(vx, vy, w, t)
            ms = d_.update(t, 0.01, [nt.ModuleState(float(jp[e, si[i]]), float(jv[e, wi[i]]), float(jv[e, si[i]])) for i in range(4)])
            sa[e] = [m.steer_angle for m in ms]; wv[e] = [m.wheel_speed for m in ms]
        tgt[:, si] = torch.tensor(sa, device=DEV, dtype=tgt.dtype)
        robot.set_joint_velocity_target_index(target=torch.tensor(wv, device=DEV, dtype=tgt.dtype), joint_ids=wi)
    for e, a in enumerate(arms):
        q, g = a.step(DT); tgt[e, ai] = torch.tensor(q, device=DEV, dtype=tgt.dtype); tgt[e, gi] = g
    robot.set_joint_position_target(tgt); scene.write_data_to_sim(); sim.step(render=False); scene.update(DT); st["k"] += 1
    if st["k"] % 20 == 0:   # 측정(10 Hz): 벽 여유·기울기·env1 그리퍼
        root = T(robot.data.root_pos_w).cpu().numpy(); gz = T(robot.data.projected_gravity_b)[:, 2].cpu().numpy(); quat = T(robot.data.root_quat_w).cpu().numpy()
        for e in range(N):
            if e in MOVING:
                st["clr"][e] = min(st["clr"][e], clearance(root[e, :2], yaw_of(quat[e])))
            st["tilt"][e] = max(st["tilt"][e], math.degrees(math.acos(min(1.0, max(-1.0, -float(gz[e]))))))
        grip_log.append((round(st["k"] * DT, 2), arms[1].label, round(float(kit.gap_mm(T(robot.data.joint_pos)[1, gi].mean())), 2)))
    if st["k"] % EVERY == 0:
        frame()
(FR / "_w.png").unlink(missing_ok=True); (FR / "_c.png").unlink(missing_ok=True)
root = T(robot.data.root_pos_w).cpu().numpy()
gaps_by = {}
for t_, lab, gmm in grip_log:
    gaps_by.setdefault(lab, []).append(gmm)
summary = {"note": "record_train_kit.py — 스크립트 명령 재생(정책 학습 아님). 벽 여유 = 주행 env 의 parked 바닥 외곽 직사각형(USD 경계 상자) 과 maze_real_layout.json 벽판·기둥(상자)·받침(원) 사이 최소 거리, 10 Hz 측정.",
           "seconds": DUR, "frames": st["f"], "foot_xy_m": [lo[:2].round(4).tolist(), hi[:2].round(4).tolist()], "spin_radius_m": round(R_FOOT, 4), "filtered_robots": n_f, "stills_t_s": st["stills"],
           "env1_gripper_gap_mm_by_phase": {k_: [min(v), max(v), v[-1]] for k_, v in gaps_by.items()},
           "envs": [{"env": e, "kind": p["kind"], "label": p["label"], "cell": p["cell"], "yaw_deg": round(math.degrees(p["yaw"]), 1),
                     "min_wall_clearance_cm": round(100 * st["clr"][e], 1) if e in MOVING else None, "max_tilt_deg": round(st["tilt"][e], 2), "guard_stops": st["guard"][e],
                     "end_xy": root[e, :2].round(3).tolist(), "end_in_own_cells": bool(inside(p, root[e, :2]))} for e, p in enumerate(PLAN)]}
(HERE / "config/record_train_kit.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "25", "-i", str(FR / "%05d.png"), "-vf", "scale=1600:-2",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(HERE / "images/train_kit_check.mp4")], check=True)
shutil.rmtree(FR, ignore_errors=True)
print(json.dumps(summary, ensure_ascii=False), flush=True)
app.close()
