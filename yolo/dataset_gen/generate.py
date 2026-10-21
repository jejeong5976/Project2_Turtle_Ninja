"""퍽/신호등 합성 데이터 생성 (Isaac Sim 6.2, headless, 실제 미로 맵).

실행 (ROS 경로가 섞이지 않게 PYTHONPATH를 비우고 Isaac Sim venv의 python으로, 프로젝트 루트에서):

    env -u PYTHONPATH ~/env_isaacsim62/bin/python yolo/dataset_gen/generate.py \
        --camera wide --frames 200 --out yolo/datasets/sim_raw/wide_000

장면:
    - 벽: Gazebo에서 쓰던 Twin Closed Maze B STL (maze.py)
    - 벽 연결 부품 83개: 파란 퍽과 같은 색. `wall_joint`로 라벨링해서 퍽과 구분하도록 학습
    - 선반: 높이 8cm. 위치 미정이라 벽에 붙여 무작위 배치 [확인 필요]
    - 퍽: 주로 선반 위, 일부는 선반 앞 바닥에 넘어진 상태
    - 신호등: 통로 바닥 (일부는 기둥 위)

출력 (BasicWriter):
    rgb_XXXX.png
    bounding_box_2d_tight_XXXX.npy / _labels_XXXX.json / _prim_paths_XXXX.json
    meta_XXXX.json   ← 이 스크립트가 따로 기록 (퍽 색, 신호등 상태 등 후처리 검증용 정답)

카메라 내부 파라미터는 실측값(config/camera_wide.yaml, config/camera_wrist.yaml)을 읽어 쓴다.
    - wide : Arducam 광각, OpenCV fisheye (docs/perception/camera_calibration.md)
    - wrist: D435 컬러, 공장 intrinsics (pinhole)

fisheye 모서리 잘림 우회: Isaac Sim RTX는 fisheye 결과를 주점 중심, 반지름 imageSize 폭/2인 원으로
잘라 버린다 (https://github.com/isaac-sim/IsaacSim/issues/838, 6.1.0-rc.26에서 확인).
그래서 좌우로 넓혀 렌더링한 뒤 원래 크기로 잘라내고, bbox도 같은 만큼 옮긴다 (crop_outputs).
"""

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

parser = argparse.ArgumentParser()
parser.add_argument("--camera", choices=["wide", "wrist"], default="wide")
parser.add_argument("--frames", type=int, default=50)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--rt-subframes", type=int, default=8)
parser.add_argument("--max-pucks", type=int, default=10)
parser.add_argument("--maze-stl", type=str, default=None, help="미로 STL 경로 (기본: waffle-lyrical-slam의 maze.stl)")
parser.add_argument("--seg", action="store_true", help="instance segmentation도 저장 (seg 모델용)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"

CAMERA_PROFILES = {
    # Arducam 광각: 바닥에서 렌즈까지 614mm (±1cm 조립 오차). 앞쪽 아래로 기울여 장착
    # [확인 필요] 기울기 각도는 장착 후 측정 (가이드 11.4)
    "wide": dict(calib="camera_wide.yaml",
                 height=(0.604, 0.624), pitch_deg=(20.0, 45.0), target_dist=(0.4, 1.8)),
    # D435 컬러: 선반 위 퍽을 0.15~0.45m 거리에서 봄
    "wrist": dict(calib="camera_wrist.yaml",
                  distance=(0.15, 0.45), elevation_deg=(10.0, 75.0)),
}


def load_calib(name: str) -> dict:
    """ROS camera_info yaml → 해상도, K(fx, fy, cx, cy), 왜곡 모델·계수."""
    info = yaml.safe_load((CONFIG_DIR / name).read_text())
    k = info["camera_matrix"]["data"]
    return dict(width=info["image_width"], height=info["image_height"],
                fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                model=info["distortion_model"], dist=info["distortion_coefficients"]["data"])


def horizontal_fov(c: dict) -> float:
    """수평 화각 [rad]. 영상 좌우 끝 픽셀이 광축과 이루는 각의 합."""
    def edge_angle(r_px):
        x = r_px / c["fx"]
        if c["model"] != "equidistant":
            return math.atan(x)
        # fisheye: θd = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸) = x 를 θ에 대해 풂 (뉴턴법)
        k1, k2, k3, k4 = c["dist"]
        t = x
        for _ in range(20):
            t2 = t * t
            f = t * (1 + k1 * t2 + k2 * t2**2 + k3 * t2**3 + k4 * t2**4) - x
            df = 1 + 3 * k1 * t2 + 5 * k2 * t2**2 + 7 * k3 * t2**3 + 9 * k4 * t2**4
            t -= f / df
        return t
    return edge_angle(c["cx"]) + edge_angle(c["width"] - c["cx"])


def fisheye_pad(c: dict) -> int:
    """잘림 원(반지름 = 렌더 폭/2)이 네 모서리를 덮도록 좌우에 더할 폭 [px]. pinhole이면 0."""
    if c["model"] != "equidistant":
        return 0
    r_max = max(math.hypot(x - c["cx"], y - c["cy"]) for x in (0, c["width"]) for y in (0, c["height"]))
    return 8 * math.ceil((r_max - c["width"] / 2) / 8)


profile = CAMERA_PROFILES[args.camera]
calib = load_calib(profile["calib"])
W, H = calib["width"], calib["height"]                     # 저장되는 영상 크기 (실제 카메라와 같음)
hfov = horizontal_fov(calib)
PAD = fisheye_pad(calib)                                    # wide: 136px → 1552x720으로 렌더링
render_calib = dict(calib, width=W + 2 * PAD, cx=calib["cx"] + PAD)

simulation_app = SimulationApp({"headless": True, "width": render_calib["width"], "height": H})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from assets import (  # noqa: E402
    PUCK_BASE_HSV, PUCK_HEIGHT, TOWER_HEIGHT, TOWER_STATES, Puck, SignalTower,
    add_label, bind, jitter_puck_color, make_omnipbr, set_material_inputs, set_pose,
)
from maze import DEFAULT_STL, WALL_HEIGHT, load_maze, split_joints  # noqa: E402

SHELF_HEIGHT = (0.075, 0.085)   # 8cm [확인 필요: 정확한 높이]
SHELF_LENGTH = (0.15, 0.30)     # 벽을 따라가는 길이 [확인 필요]
SHELF_DEPTH = (0.07, 0.15)      # 벽에서 튀어나온 깊이 [확인 필요]
PANEL_HALF_T = 0.0015           # 벽 패널 두께 3mm의 절반
MAX_SHELVES = 4

rng = np.random.default_rng(args.seed)
out_dir = Path(args.out).resolve()
out_dir.mkdir(parents=True, exist_ok=True)
maze = load_maze(Path(args.maze_stl) if args.maze_stl else DEFAULT_STL)

# ---------------------------------------------------------------------------
# 장면
# ---------------------------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")
for p in ("/World/Looks", "/World/Pucks", "/World/Towers", "/World/Env", "/World/Lights"):
    UsdGeom.Scope.Define(stage, p)
rep.orchestrator.set_capture_on_play(False)


def make_box(path, size_xyz, material):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
    xf.AddScaleOp().Set(Gf.Vec3f(*size_xyz))
    bind(cube.GetPrim(), material)
    return cube.GetPrim()


def set_box(prim, center, size):
    set_pose(prim, center)
    ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    ops["xformOp:scale"].Set(Gf.Vec3f(*map(float, size)))


def set_visible(prim, visible: bool):
    UsdGeom.Imageable(prim).GetVisibilityAttr().Set(
        UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)


floor_mat = make_omnipbr("Floor", (0.3, 0.3, 0.3), 0.9)
floor = make_box("/World/Env/Floor", (12.0, 12.0, 0.02), floor_mat)
set_pose(floor, (0.0, 0.0, -0.01))

# 미로 벽 (STL 메시): 패널 + 기둥 자리 연결 부품 83개
def make_mesh(path, points, faces, material):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces) // 3, 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr([tuple(map(float, points.min(0))), tuple(map(float, points.max(0)))])
    bind(mesh.GetPrim(), material)
    return mesh.GetPrim()


(panel_pts, panel_faces), joints = split_joints(maze.points, maze.faces)
wall_mat = make_omnipbr("Wall", (0.65, 0.72, 0.8), 0.25)
make_mesh("/World/Env/Maze/Panels", panel_pts, panel_faces, wall_mat)
# 연결 부품은 파란 퍽과 같은 색 → 오탐을 막기 위해 wall_joint로 라벨링 (hard negative)
joint_mat = make_omnipbr("WallJoint", (0.1, 0.2, 0.65), 0.6)
for k, (_, jp, jf) in enumerate(joints):
    add_label(make_mesh(f"/World/Env/Maze/Joint_{k:02d}", jp, jf, joint_mat), "wall_joint")

# 선반 (라벨 없음)
shelves = []
for i in range(MAX_SHELVES):
    mat = make_omnipbr(f"Shelf_{i}", (0.6, 0.6, 0.6), 0.7)
    shelves.append((make_box(f"/World/Env/Shelf_{i}", (0.2, 0.1, 0.08), mat), mat))

# 방해물: 퍽과 모양·크기가 다른 색깔 물체 (라벨 없음)
distractors = []
for i in range(4):
    mat = make_omnipbr(f"Distractor_{i}", (0.5, 0.5, 0.5), 0.6)
    if i % 2 == 0:
        sph = UsdGeom.Sphere.Define(stage, f"/World/Env/Distractor_{i}")
        sph.CreateRadiusAttr(float(rng.uniform(0.03, 0.06)))
        UsdGeom.Xformable(sph).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        UsdGeom.Xformable(sph).AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
        prim = sph.GetPrim()
        bind(prim, mat)
    else:
        # 퍽(3x3x6cm)과 헷갈리지 않게 모든 변을 10cm 이상으로
        prim = make_box(f"/World/Env/Distractor_{i}", tuple(rng.uniform(0.10, 0.2, size=3)), mat)
    distractors.append((prim, mat))

# 신호등 설치대 (높이 올려 설치하는 경우용 기둥, 라벨 없음)
pole_mat = make_omnipbr("Pole", (0.2, 0.2, 0.2), 0.6)
pole = make_box("/World/Env/TowerPole", (0.02, 0.02, 0.1), pole_mat)

pucks = [Puck(stage, i) for i in range(args.max_pucks)]
tower = SignalTower(stage, 0)

# 조명
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
ceiling = [UsdLux.SphereLight.Define(stage, f"/World/Lights/Ceiling_{i}") for i in range(3)]
for light in [dome] + ceiling:
    light.CreateEnableColorTemperatureAttr(True)
for light in ceiling:
    light.CreateRadiusAttr(0.15)
    UsdGeom.Xformable(light).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)

# 카메라: 실측 K로 pinhole 값을 맞추고, fisheye면 OpenCV fisheye 렌즈 왜곡을 적용
def make_camera(path: str, c: dict):
    cam = UsdGeom.Camera.Define(stage, path)
    px = 20.955 / c["width"]                          # 센서 픽셀 크기 [mm] (가로 aperture 20.955mm 기준)
    cam.CreateFocalLengthAttr(c["fx"] * px)
    cam.CreateHorizontalApertureAttr(c["width"] * px)
    cam.CreateVerticalApertureAttr(c["height"] * px * c["fx"] / c["fy"])
    cam.CreateHorizontalApertureOffsetAttr((c["width"] / 2 - c["cx"]) * px)
    cam.CreateVerticalApertureOffsetAttr((c["cy"] - c["height"] / 2) * px)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    if c["model"] == "equidistant":
        prim = cam.GetPrim()
        prim.AddAppliedSchema("OmniLensDistortionOpenCvFisheyeAPI")
        ns = "omni:lensdistortion:opencvFisheye:"
        for key in ("fx", "fy", "cx", "cy"):
            prim.CreateAttribute(ns + key, Sdf.ValueTypeNames.Float).Set(float(c[key]))
        for i, k in enumerate(c["dist"], start=1):
            prim.CreateAttribute(f"{ns}k{i}", Sdf.ValueTypeNames.Float).Set(float(k))
        prim.CreateAttribute(ns + "imageSize", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(c["width"], c["height"]))
    return cam


cam = make_camera("/World/Camera", render_calib)
cam_op = UsdGeom.Xformable(cam).AddTransformOp()

rp = rep.create.render_product(str(cam.GetPath()), (render_calib["width"], H))
backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir=str(out_dir))
writer = rep.writers.get("BasicWriter")
writer.initialize(backend=backend, rgb=True, bounding_box_2d_tight=True,
                  instance_segmentation=args.seg,
                  distance_to_image_plane=(args.camera == "wrist"))
writer.attach(rp)


# ---------------------------------------------------------------------------
# 배치 계산
# ---------------------------------------------------------------------------
def look_at(eye, target):
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    cam_op.Set(view.GetInverse())


def sample_shelf():
    """벽 구간 하나를 골라 한쪽 면에 붙인 선반. 미로 안쪽 면만 사용."""
    for _ in range(100):
        seg = maze.segments[int(rng.integers(len(maze.segments)))]
        side = float(rng.choice([-1.0, 1.0]))
        length = rng.uniform(*SHELF_LENGTH)
        depth = rng.uniform(*SHELF_DEPTH)
        height = rng.uniform(*SHELF_HEIGHT)
        margin = 0.035 + length / 2   # 기둥(6cm)을 피함
        if seg.length < 2 * margin:
            continue
        t = rng.uniform(margin, seg.length - margin)
        if seg.axis == "x":
            cx, cy = seg.a[0] + t, seg.a[1] + side * (PANEL_HALF_T + depth / 2)
            size = (length, depth, height)
            normal = (0.0, side)
        else:
            cx, cy = seg.a[0] + side * (PANEL_HALF_T + depth / 2), seg.a[1] + t
            size = (depth, length, height)
            normal = (side, 0.0)
        front = (cx + normal[0] * (depth / 2 + 0.1), cy + normal[1] * (depth / 2 + 0.1))
        if maze.clearance_at(front) < 0.05:   # 바깥쪽 면이거나 막힌 곳
            continue
        return dict(center=(cx, cy, height / 2), size=size, normal=normal, top=height,
                    axis=seg.axis, length=length, depth=depth)
    return None


def shelf_overlaps(s, others):
    for o in others:
        if (abs(s["center"][0] - o["center"][0]) < (s["size"][0] + o["size"][0]) / 2 + 0.02 and
                abs(s["center"][1] - o["center"][1]) < (s["size"][1] + o["size"][1]) / 2 + 0.02):
            return True
    return False


def puck_slots_on_shelf(s, n):
    """선반 위면에서 서로 4.5cm 이상 떨어진 위치 n개."""
    slots = []
    for _ in range(60):
        if len(slots) >= n:
            break
        along = rng.uniform(-s["length"] / 2 + 0.02, s["length"] / 2 - 0.02)
        across = rng.uniform(-max(0.0, s["depth"] / 2 - 0.02), max(0.0, s["depth"] / 2 - 0.02))
        if s["axis"] == "x":
            xy = (s["center"][0] + along, s["center"][1] + across)
        else:
            xy = (s["center"][0] + across, s["center"][1] + along)
        if all(math.dist(xy, q) >= 0.045 for q in slots):
            slots.append(xy)
    return slots


def sample_camera_wide(shelf):
    """선반 앞쪽 통로에서 선반을 바라보는 로봇 상단 카메라 위치."""
    sx, sy, _ = shelf["center"]
    base_dir = math.atan2(shelf["normal"][1], shelf["normal"][0])
    for _ in range(200):
        d = rng.uniform(*profile["target_dist"])
        a = base_dir + rng.uniform(-1.2, 1.2)
        xy = (sx + d * math.cos(a), sy + d * math.sin(a))
        if maze.clearance_at(xy) < 0.15:   # 로봇이 갈 수 없는 곳
            continue
        h = rng.uniform(*profile["height"])
        if not maze.line_of_sight(xy, (sx, sy), h, shelf["top"] + 0.03):
            continue
        yaw = math.atan2(sy - xy[1], sx - xy[0]) + rng.uniform(-0.35, 0.35) * hfov
        pitch = math.radians(rng.uniform(*profile["pitch_deg"]))
        eye = (xy[0], xy[1], h)
        target = (xy[0] + math.cos(yaw) * math.cos(pitch), xy[1] + math.sin(yaw) * math.cos(pitch),
                  h - math.sin(pitch))
        return eye, target, yaw
    return None


def sample_camera_wrist(shelf, focus_xyz):
    """선반 앞쪽에서 퍽을 가까이 보는 손목 카메라 위치."""
    base_dir = math.atan2(shelf["normal"][1], shelf["normal"][0])
    for _ in range(200):
        d = rng.uniform(*profile["distance"])
        elev = math.radians(rng.uniform(*profile["elevation_deg"]))
        az = base_dir + rng.uniform(-1.2, 1.2)
        eye = (focus_xyz[0] + d * math.cos(elev) * math.cos(az),
               focus_xyz[1] + d * math.cos(elev) * math.sin(az),
               focus_xyz[2] + d * math.sin(elev))
        if eye[2] < WALL_HEIGHT + 0.02 and maze.clearance_at(eye[:2]) < 0.03:
            continue   # 벽 속
        target = (focus_xyz[0] + rng.normal(0, 0.015), focus_xyz[1] + rng.normal(0, 0.015), focus_xyz[2])
        return eye, target
    return None


def free_point_in_view(eye, yaw, d_range, bearing_half, min_clear, taken, gap):
    for _ in range(100):
        d = rng.uniform(*d_range)
        b = yaw + rng.uniform(-bearing_half, bearing_half)
        xy = (eye[0] + d * math.cos(b), eye[1] + d * math.sin(b))
        if maze.clearance_at(xy) < min_clear:
            continue
        if not maze.line_of_sight(eye[:2], xy, eye[2], 0.05):
            continue
        if all(math.dist(xy, q) >= gap for q in taken):
            taken.append(xy)
            return xy
    return None


# ---------------------------------------------------------------------------
# 랜덤화
# ---------------------------------------------------------------------------
def randomize_lights():
    dome.GetIntensityAttr().Set(float(rng.uniform(150, 900)))
    dome.GetColorTemperatureAttr().Set(float(rng.uniform(3000, 6500)))
    n_on = int(rng.integers(1, len(ceiling) + 1))
    for i, light in enumerate(ceiling):
        on = i < n_on
        light.GetIntensityAttr().Set(float(rng.uniform(4e4, 1.5e5)) if on else 0.0)
        light.GetColorTemperatureAttr().Set(float(rng.uniform(3000, 6500)))
        UsdGeom.XformCommonAPI(light).SetTranslate(
            Gf.Vec3d(*rng.uniform((-2.2, -3.1, 2.2), (2.2, 3.1, 2.8))))


def randomize_materials():
    g = rng.uniform(0.12, 0.6)
    set_material_inputs(floor_mat, diffuse_color_constant=tuple(np.clip(g * rng.uniform(0.92, 1.08, 3), 0, 1)),
                        reflection_roughness_constant=float(rng.uniform(0.6, 1.0)))
    # [확인 필요] 실제 벽 색. Gazebo 색(하늘빛 회색) 주변 + 흰색·회색
    # 벽은 살짝 유광 (확인됨): roughness를 낮게 잡아 조명·퍽·켜진 신호등이 벽에 비치도록
    base = [(0.65, 0.72, 0.8), (0.85, 0.85, 0.85), (0.5, 0.5, 0.5)][int(rng.integers(3))]
    set_material_inputs(wall_mat, diffuse_color_constant=tuple(np.clip(np.array(base) * rng.uniform(0.85, 1.15), 0, 1)),
                        reflection_roughness_constant=float(rng.uniform(0.15, 0.4)))
    # 연결 부품: 실물 확인됨 (3D 프린팅, 전체 파란색). 실제 색 측정(가이드 11.9) 전까지 파란 퍽과 같은 분포로 흔듦
    set_material_inputs(joint_mat, diffuse_color_constant=jitter_puck_color(rng, "blue"),
                        reflection_roughness_constant=float(rng.uniform(0.4, 0.9)))
    for _, mat in shelves:
        sg = rng.uniform(0.3, 0.95)
        set_material_inputs(mat, diffuse_color_constant=tuple(np.clip(sg * rng.uniform(0.9, 1.1, 3), 0, 1)))


def place_pucks(entries):
    """entries: [(xy, base_z, fallen, on_shelf)] → 퍽 배치, 나머지는 숨김."""
    meta = []
    for i, puck in enumerate(pucks):
        if i >= len(entries):
            puck.hide()
            continue
        xy, base_z, fallen, on_shelf = entries[i]
        puck.show()
        shape = str(rng.choice(["cyl", "box"]))
        color = str(rng.choice(list(PUCK_BASE_HSV)))
        puck.set(rng, shape, color, fallen, xy, float(rng.uniform(0, 360)),
                 roughness=float(rng.uniform(0.6, 0.9)), base_z=base_z)
        meta.append(dict(prim=str(puck.shapes[shape].GetPath()), shape=shape, color=color,
                         fallen=fallen, on_shelf=on_shelf, xy=list(xy), base_z=base_z))
    return meta


def randomize_frame():
    randomize_lights()
    randomize_materials()

    # 1) 선반: 첫 번째가 이번 프레임의 주 대상
    active = []
    while len(active) < int(rng.integers(1, MAX_SHELVES + 1)):
        s = sample_shelf()
        if s and not shelf_overlaps(s, active):
            active.append(s)
    for i, (prim, _) in enumerate(shelves):
        set_visible(prim, i < len(active))
        if i < len(active):
            set_box(prim, active[i]["center"], active[i]["size"])

    # 2) 퍽: 선반 위(대부분 서 있음) + 선반 앞 바닥(넘어짐)
    entries, taken = [], []
    for s in active:
        for xy in puck_slots_on_shelf(s, int(rng.integers(1, 4))):
            fallen = bool(rng.random() < 0.1)
            entries.append((xy, s["top"], fallen, True))
            taken.append(xy)
        if rng.random() < 0.4:   # 선반에서 떨어진 퍽
            off = s["depth"] / 2 + rng.uniform(0.03, 0.15)
            xy = (s["center"][0] + s["normal"][0] * off + rng.normal(0, 0.03),
                  s["center"][1] + s["normal"][1] * off + rng.normal(0, 0.03))
            if maze.clearance_at(xy) > 0.02 and all(math.dist(xy, q) >= 0.08 for q in taken):
                entries.append((xy, 0.0, bool(rng.random() < 0.8), False))
                taken.append(xy)
    entries = entries[:len(pucks)]

    main = active[0]
    tower_meta = None
    set_visible(pole, False)

    if args.camera == "wide":
        cam_pose = sample_camera_wide(main)
        if cam_pose is None:
            return None
        eye, target, yaw = cam_pose
        look_at(eye, target)

        # 3) 신호등: 시야 안 통로 바닥
        tower.hide()
        if rng.random() < 0.5:
            xy = free_point_in_view(eye, yaw, (0.6, 2.5), hfov / 2 * 0.8, 0.05, taken, 0.1)
            if xy is not None:
                tower.show()
                # [확인 필요] 설치 높이. 70%는 바닥에 직접, 30%는 기둥 위
                base_z = 0.0 if rng.random() < 0.7 else float(rng.uniform(0.05, 0.3))
                if base_z > 0:
                    set_visible(pole, True)
                    set_box(pole, (xy[0], xy[1], base_z / 2), (0.02, 0.02, base_z))
                state = str(rng.choice(TOWER_STATES))
                tower.set(rng, state, (xy[0], xy[1], base_z), float(rng.uniform(0, 360)))
                tower_meta = dict(prim=str(tower.root.GetPath()), state=state, xy=list(xy),
                                  base_z=base_z, height=TOWER_HEIGHT)

        # 방해물
        for prim, mat in distractors:
            xy = free_point_in_view(eye, yaw, (0.3, 2.0), hfov / 2, 0.08, taken, 0.1) if rng.random() < 0.4 else None
            set_visible(prim, xy is not None)
            if xy is not None:
                set_pose(prim, (xy[0], xy[1], 0.05), (0, 0, float(rng.uniform(0, 360))))
                set_material_inputs(mat, diffuse_color_constant=tuple(rng.uniform(0.05, 0.95, size=3)))
    else:
        tower.hide()
        for prim, _ in distractors:
            set_visible(prim, False)
        shelf_pucks = [e for e in entries if e[3]]
        focus_xy, focus_z = (shelf_pucks[0][0], main["top"] + PUCK_HEIGHT / 2) if shelf_pucks else \
            (main["center"][:2], main["top"])
        cam_pose = sample_camera_wrist(main, (focus_xy[0], focus_xy[1], focus_z))
        if cam_pose is None:
            return None
        eye, target = cam_pose
        look_at(eye, target)

    puck_meta = place_pucks(entries)
    return dict(camera=args.camera, camera_pose=dict(eye=list(eye), target=list(target)),
                shelves=[dict(center=list(s["center"]), size=list(s["size"])) for s in active],
                pucks=puck_meta, tower=tower_meta)


def crop_outputs(out_dir: Path, pad: int, width: int):
    """넓혀 렌더링한 결과를 가운데 width만 남기고, bbox x좌표를 pad만큼 옮김. 다 잘려 나간 bbox는 버림."""
    from PIL import Image

    for name in ("rgb", "instance_segmentation"):
        for png in out_dir.glob(f"{name}_*.png"):
            with Image.open(png) as im:
                im.crop((pad, 0, pad + width, im.height)).save(png)
    for npy in out_dir.glob("bounding_box_2d_tight_[0-9]*.npy"):
        boxes = np.load(npy)
        boxes["x_min"] = np.clip(boxes["x_min"] - pad, 0, width - 1)
        boxes["x_max"] = np.clip(boxes["x_max"] - pad, 0, width - 1)
        keep = boxes["x_max"] > boxes["x_min"]
        np.save(npy, boxes[keep])
        paths = npy.with_name(npy.stem.replace("tight_", "tight_prim_paths_") + ".json")
        if paths.exists():
            prims = json.loads(paths.read_text())
            paths.write_text(json.dumps([p for p, k in zip(prims, keep) if k]))


# ---------------------------------------------------------------------------
# 캡처 루프
# ---------------------------------------------------------------------------
# 셰이더·머티리얼 컴파일이 끝나도록 몇 프레임 미리 렌더링 (기록 안 함)
for _ in range(5):
    simulation_app.update()

try:
    i = 0
    while i < args.frames:
        meta = randomize_frame()
        if meta is None:   # 카메라 위치를 못 찾으면 다른 배치로 재시도
            continue
        # 라벨·보이기 변경을 렌더러가 등록한 뒤에 캡처 (바로 캡처하면 바뀐 퍽의 bbox가 빠짐)
        for _ in range(2):
            simulation_app.update()
        rep.orchestrator.step(delta_time=0.0, rt_subframes=args.rt_subframes)
        (out_dir / f"meta_{i:04d}.json").write_text(json.dumps(meta, indent=1))
        i += 1
        if i % 10 == 0:
            print(f"[generate] {i}/{args.frames}", flush=True)
    rep.orchestrator.wait_until_complete()
    if PAD:
        crop_outputs(out_dir, PAD, W)
finally:
    writer.detach()
    rp.destroy()
    simulation_app.close()
