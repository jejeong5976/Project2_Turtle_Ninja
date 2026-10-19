"""퍽, 신호등(타워램프) USD 생성 함수.

Isaac Sim이 실행된 상태(SimulationApp 생성 이후)에서 import해서 사용한다.
단위는 m, 위쪽 축은 +Z.

치수 출처: docs/perception/yolo26_isaac_sim_guide.md 11.5, 11.6
"""

import colorsys

import omni.replicator.core as rep
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSemantics

# ---------------------------------------------------------------------------
# 치수 (m)
# ---------------------------------------------------------------------------
PUCK_SIZE = 0.03      # 원기둥 지름 / 사각기둥 한 변
PUCK_HEIGHT = 0.06

TOWER_RADIUS = 0.02   # 한영넉스 STL040, Ø40
TOWER_CAP_H = 0.012
TOWER_LINE_H = 0.001  # 칸 사이 검은 구분선
TOWER_LENS_H = 0.035  # 칸 높이 36mm = 구분선 1mm + 렌즈 35mm
TOWER_HEIGHT = TOWER_CAP_H + 3 * (TOWER_LINE_H + TOWER_LENS_H)   # 0.120

# ---------------------------------------------------------------------------
# 색 (선형 RGB). 실측 전 기본값 — 11.9 색 측정 후 교체
# ---------------------------------------------------------------------------
PUCK_BASE_HSV = {      # (h 0~1, s, v)
    "red": (0.0, 0.85, 0.75),
    "green": (0.36, 0.80, 0.55),
    "blue": (0.62, 0.85, 0.65),
}
LENS_OFF_RGB = {       # 꺼진 렌즈: 색 있는 반투명 플라스틱을 불투명 재질로 근사
    "red": (0.45, 0.03, 0.03),
    "yellow": (0.65, 0.35, 0.02),
    "green": (0.02, 0.35, 0.10),
}
LENS_ON_RGB = {        # 켜진 렌즈 발광색
    "red": (1.0, 0.08, 0.05),
    "yellow": (1.0, 0.55, 0.05),
    "green": (0.10, 1.0, 0.30),
}
TOWER_CAP_RGB = (0.62, 0.62, 0.58)
TOWER_LINE_RGB = (0.02, 0.02, 0.02)

TOWER_STATES = ("off", "red", "yellow", "green")

# 안 쓰는 물체를 치워 두는 위치 (바닥 아래).
# visibility로 숨겼다 다시 보이게 하면 렌더러가 semantic label을 잃어 bbox가 빠지므로,
# 라벨이 있는 물체는 숨기지 않고 이 위치로 옮긴다.
PARK = (0.0, 0.0, -50.0)


def add_label(prim: Usd.Prim, label: str) -> None:
    """class 분류 체계로 semantic label을 붙인다 (기존 라벨은 덮어씀)."""
    UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr().Set([label])


def make_omnipbr(name: str, rgb, roughness: float, parent: str = "/World/Looks") -> Usd.Prim:
    return rep.functional.create.material(
        mdl="OmniPBR.mdl",
        name=name,
        parent=parent,
        diffuse_color_constant=tuple(rgb),
        reflection_roughness_constant=float(roughness),
    )


def set_material_inputs(material: Usd.Prim, **inputs) -> None:
    for key, value in inputs.items():
        rep.functional.modify.attribute(material, key, value)


def bind(prim: Usd.Prim, material: Usd.Prim) -> None:
    rep.functional.modify.material(prim, material)


def set_pose(prim: Usd.Prim, position, rot_xyz_deg=(0.0, 0.0, 0.0)) -> None:
    """translate + rotateXYZ xform op을 한 번만 만들고 이후에는 값만 바꾼다."""
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate") or xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    r = ops.get("xformOp:rotateXYZ") or xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
    t.Set(Gf.Vec3d(*map(float, position)))
    r.Set(Gf.Vec3d(*map(float, rot_xyz_deg)))


def jitter_puck_color(rng, color: str, dh=0.015, ds=0.15, dv=0.20):
    """기준 HSV 주변으로만 흔든다 (가이드 4.4: 무작위 색은 쓰지 않음)."""
    h, s, v = PUCK_BASE_HSV[color]
    h = (h + rng.uniform(-dh, dh)) % 1.0
    s = min(1.0, max(0.0, s * (1 + rng.uniform(-ds, ds))))
    v = min(1.0, max(0.0, v * (1 + rng.uniform(-dv, dv))))
    return colorsys.hsv_to_rgb(h, s, v)


# ---------------------------------------------------------------------------
# 퍽
# ---------------------------------------------------------------------------
class Puck:
    """원기둥/사각기둥 형상을 둘 다 만들어 두고, 쓰지 않는 형상은 PARK 위치로 옮긴다.

    semantic label은 생성할 때 한 번만 붙이고 바꾸지 않는다 (Cyl=puck_cyl, Box=puck_box).
    실행 중에 라벨을 바꾸거나 visibility를 전환하면 렌더러가 라벨을 잃어 bbox가 빠진다.
    넘어짐 여부는 meta_XXXX.json의 fallen 값으로 convert_to_yolo.py가 붙인다.
    """

    def __init__(self, stage: Usd.Stage, index: int):
        self.root = UsdGeom.Xform.Define(stage, f"/World/Pucks/Puck_{index:02d}").GetPrim()
        base = self.root.GetPath()

        cyl = UsdGeom.Cylinder.Define(stage, base.AppendChild("Cyl"))
        cyl.CreateRadiusAttr(PUCK_SIZE / 2)
        cyl.CreateHeightAttr(PUCK_HEIGHT)
        cyl.CreateAxisAttr(UsdGeom.Tokens.z)
        cyl.CreateExtentAttr([(-PUCK_SIZE / 2, -PUCK_SIZE / 2, -PUCK_HEIGHT / 2),
                              (PUCK_SIZE / 2, PUCK_SIZE / 2, PUCK_HEIGHT / 2)])
        cyl_t = UsdGeom.Xformable(cyl).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)

        box = UsdGeom.Cube.Define(stage, base.AppendChild("Box"))
        box.CreateSizeAttr(1.0)
        box.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
        box_t = UsdGeom.Xformable(box).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        UsdGeom.Xformable(box).AddScaleOp().Set(Gf.Vec3f(PUCK_SIZE, PUCK_SIZE, PUCK_HEIGHT))

        self.shapes = {"cyl": cyl.GetPrim(), "box": box.GetPrim()}
        self.shape_offsets = {"cyl": cyl_t, "box": box_t}
        self.hide()
        for name, prim in self.shapes.items():
            add_label(prim, f"puck_{name}")
        self.material = make_omnipbr(f"PuckMat_{index:02d}", (0.5, 0.5, 0.5), 0.75)
        for p in self.shapes.values():
            bind(p, self.material)

    def set(self, rng, shape: str, color: str, fallen: bool, xy, yaw_deg: float, roughness: float,
            base_z: float = 0.0):
        """base_z: 퍽이 놓인 면의 높이 (바닥 0, 선반 위면 선반 높이)."""
        for name, op in self.shape_offsets.items():
            op.Set(Gf.Vec3d(0, 0, 0) if name == shape else Gf.Vec3d(*PARK))
        if fallen:
            # 옆으로 눕힘: 높이 방향이 수평이 되고 중심 높이는 반지름(반 변)
            set_pose(self.root, (xy[0], xy[1], base_z + PUCK_SIZE / 2), (90.0, 0.0, yaw_deg))
        else:
            set_pose(self.root, (xy[0], xy[1], base_z + PUCK_HEIGHT / 2), (0.0, 0.0, yaw_deg))
        set_material_inputs(self.material,
                            diffuse_color_constant=jitter_puck_color(rng, color),
                            reflection_roughness_constant=roughness)

    def hide(self):
        set_pose(self.root, PARK)

    def show(self):
        pass   # set()이 위치를 정한다


# ---------------------------------------------------------------------------
# 신호등 (한영넉스 STL040 3단, 받침 없음)
# ---------------------------------------------------------------------------
class SignalTower:
    """아래에서부터 초록 렌즈, 구분선, 노랑, 구분선, 빨강, 구분선, 캡 순서로 원통 7개를 쌓는다.

    semantic label `signal_tower`는 루트 Xform에 붙여서 bbox가 신호등 전체를 덮게 한다.
    """

    def __init__(self, stage: Usd.Stage, index: int = 0):
        self.root = UsdGeom.Xform.Define(stage, f"/World/Towers/Tower_{index:02d}").GetPrim()
        add_label(self.root, "signal_tower")
        base = self.root.GetPath()

        cap_mat = make_omnipbr(f"TowerCap_{index:02d}", TOWER_CAP_RGB, 0.5)
        line_mat = make_omnipbr(f"TowerLine_{index:02d}", TOWER_LINE_RGB, 0.8)
        self.lens_mats = {}

        z = 0.0
        layout = [("green", TOWER_LENS_H), ("line0", TOWER_LINE_H),
                  ("yellow", TOWER_LENS_H), ("line1", TOWER_LINE_H),
                  ("red", TOWER_LENS_H), ("line2", TOWER_LINE_H),
                  ("cap", TOWER_CAP_H)]
        for name, h in layout:
            cyl = UsdGeom.Cylinder.Define(stage, base.AppendChild(name.capitalize()))
            cyl.CreateRadiusAttr(TOWER_RADIUS)
            cyl.CreateHeightAttr(h)
            cyl.CreateAxisAttr(UsdGeom.Tokens.z)
            cyl.CreateExtentAttr([(-TOWER_RADIUS, -TOWER_RADIUS, -h / 2), (TOWER_RADIUS, TOWER_RADIUS, h / 2)])
            UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0, 0, z + h / 2))
            z += h
            if name == "cap":
                bind(cyl.GetPrim(), cap_mat)
            elif name.startswith("line"):
                bind(cyl.GetPrim(), line_mat)
            else:
                mat = make_omnipbr(f"TowerLens_{name}_{index:02d}", LENS_OFF_RGB[name], 0.4)
                set_material_inputs(mat, enable_emission=False,
                                    emissive_color=LENS_ON_RGB[name], emissive_intensity=0.0)
                bind(cyl.GetPrim(), mat)
                self.lens_mats[name] = mat

    def set(self, rng, state: str, position, yaw_deg: float = 0.0,
            on_intensity=(2000.0, 8000.0), off_scale=(0.7, 1.3)):
        """state: off / red / yellow / green. 켜진 칸 발광 세기, 꺼진 칸 밝기를 랜덤화한다."""
        set_pose(self.root, position, (0.0, 0.0, yaw_deg))
        for name, mat in self.lens_mats.items():
            k = rng.uniform(*off_scale)
            off_rgb = tuple(min(1.0, c * k) for c in LENS_OFF_RGB[name])
            lit = name == state
            set_material_inputs(mat,
                                diffuse_color_constant=off_rgb,
                                enable_emission=lit,
                                emissive_intensity=float(rng.uniform(*on_intensity)) if lit else 0.0)

    def hide(self):
        set_pose(self.root, PARK)

    def show(self):
        pass   # set()이 위치를 정한다
