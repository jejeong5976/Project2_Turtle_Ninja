#!/usr/bin/env python3
"""랜덤 소품 환경용 USD 생성 -> usd/props/ (먼저 python3 make_tag_barcode_images.py 로 usd/props/tex/*.png 생성).

  퍽 8개 : 빨강 R1–R4 (AprilTag ID 0–3), 초록 G1–G4 (ID 4–7). 파랑은 빠짐 (팀 결정 2026-10-23).
           모양: 색마다 직육면체 30×30×60 mm 2개 + 원기둥 Ø30×60 mm 2개 (형태별 개수는 제작 후 공지 -> 가정)
           태그: 직육면체 6면 모두, 원기둥 위·아래·옆면 1장. 원기둥 옆면은 곡면을 따라 감은 스티커 (호 길이 20 mm = 76°).
           라벨 20 mm (검은 16 mm + 흰 여백). 동적 강체, 질량 직육면체 17 g·원기둥 13 g (실측), PLA 마찰 0.35/0.3 (가정), 퍽 색 무광
  장소 태그 3개 : L1–L3 (ID 12–14), 라벨 60 mm (검은 48 mm). 벽에 붙임. kinematic, 충돌 없음
  바코드 4개 : Code128 P2-01–P2-04 (팀 배포 이미지에서 복원). 판 70 × 24 mm (흰 여백 포함 실측, 2026-10-28) -> 모듈 70/110 mm. kinematic, 충돌 없음
  신호등 : 한영넉스 STL040 3단 세로 + 알루미늄 프로파일 20×20×450 기둥 (이전과 같음)
  로컬 +X = 벽에서 칸 안쪽(바코드·태그 앞면). 퍽 원점 = 퍽 중심.
실행: usd-core 가 있는 파이썬 (예: python3 -m pip install usd-core 후 python3) tools/make_props_usd.py
"""
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parents[1]   # sim/robot_sim
OUT = HERE / 'usd' / 'props'
TEX = OUT / 'tex'
assert (TEX / 'tag36h11_00.png').exists(), 'python3 make_tag_barcode_images.py 먼저'

PUCK_D, PUCK_H = 0.03, 0.06
PUCK_MASS = {'box': 0.017, 'cyl': 0.013}        # 실측 (저울, 2026-10-24): 직육면체 17 g, 원기둥 13 g
PUCK_LABEL, PLACE_LABEL = 0.020, 0.060
BAR_MODULE, BAR_MODULES, BAR_H = 0.070 / 110, 110, 0.024   # 흰 여백 포함 70 × 24 mm (실측 23–25, 2026-10-28)
LIFT = 0.0002                                  # 라벨을 면에서 띄움 (겹쳐 그려짐 방지). 스티커 두께 정도 (이전 0.4 mm)


def srgb_to_linear(rgb255):
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in (v / 255.0 for v in rgb255))


VGAIN = 2.4   # 세로면 보정 배율: 목표 색을 그대로 넣으면 시뮬 조명에서 어둡게 나옴 -> 렌더가 목표 색이 되게 (layout_to_maze_usd.py 와 같음, 돔 세기 1800 기준)
_lift = lambda rgb: tuple(min(1.0, VGAIN * v) for v in srgb_to_linear(rgb))
PUCK_COLORS = {'red': _lift((157, 34, 53)),      # 팀 지정 색 sRGB (157,34,53)
               'green': _lift((45, 150, 72))}    # 실물 사진 카펫 위 중앙값 (real_photos/20261023_2059*.jpg)
PUCK_ROUGHNESS = 0.55                                     # PLA 출력물: 옆면에 약한 광택 (사진), 이전 0.85 무광
# (이름, 색, 모양, 태그 ID)
PUCKS = [('R1', 'red', 'box', 0), ('R2', 'red', 'box', 1), ('R3', 'red', 'cyl', 2), ('R4', 'red', 'cyl', 3),
         ('G1', 'green', 'box', 4), ('G2', 'green', 'box', 5), ('G3', 'green', 'cyl', 6), ('G4', 'green', 'cyl', 7)]
PLACES = [('L1', 12), ('L2', 13), ('L3', 14)]
BARCODES = ['P2-01', 'P2-02', 'P2-03', 'P2-04']

def new_stage(path: Path, root_name: str):
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f'/{root_name}')
    stage.SetDefaultPrim(root.GetPrim())
    # kinematic 강체 (env 설정에서 reset 마다 위치를 씀). 충돌 형상 없음
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)
    return stage, root


def preview_material(stage, path: str, color, roughness=0.6, texture: str | None = None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f'{path}/Surface')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    if texture:
        st = UsdShade.Shader.Define(stage, f'{path}/st')
        st.CreateIdAttr('UsdPrimvarReader_float2')
        st.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
        tex = UsdShade.Shader.Define(stage, f'{path}/tex')
        tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
        tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('clamp')
        tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('clamp')
        tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), 'result')
        tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    else:
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat



def label_quad(stage, path, center, normal, up, w, h, texture):
    """평평한 라벨 사각형. normal = 바깥 방향, up = 그림의 위쪽. 앞에서 보면 텍스처가 바로 보이게 st 배치."""
    n, u = Gf.Vec3d(*normal).GetNormalized(), Gf.Vec3d(*up).GetNormalized()
    r = Gf.Cross(-n, u)                        # 앞에서 보는 사람 기준 오른쪽
    c = Gf.Vec3d(*center) + n * LIFT
    pts = [c - r * w / 2 - u * h / 2, c + r * w / 2 - u * h / 2, c + r * w / 2 + u * h / 2, c - r * w / 2 + u * h / 2]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(*n)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateDoubleSidedAttr(False)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    looks = path.rsplit('/', 1)[0].split('/')[1]
    mat = preview_material(stage, f'/{looks}/Looks/{Path(texture).stem}', None, roughness=0.7, texture=texture)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


def smooth_cylinder(stage, path, radius, height, seg=64):
    """옆면 매끈한 원기둥 메쉬 (옆면 꼭짓점 법선 = 반지름 방향, 위·아래 뚜껑은 따로 평평한 법선)."""
    pts, nrm, cnt, idx = [], [], [], []
    for zz in (-height / 2, height / 2):                       # 옆면: 아래 줄 0..seg-1, 위 줄 seg..2seg-1
        for i in range(seg):
            a = 2 * math.pi * i / seg
            pts.append(Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), zz)); nrm.append(Gf.Vec3f(math.cos(a), math.sin(a), 0))
    for i in range(seg):
        j = (i + 1) % seg
        cnt.append(4); idx += [i, j, seg + j, seg + i]
    for zz, sgn in ((height / 2, 1), (-height / 2, -1)):        # 뚜껑: 꼭짓점 따로 (법선 ±z)
        base = len(pts)
        for i in range(seg):
            a = 2 * math.pi * i / seg
            pts.append(Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), zz)); nrm.append(Gf.Vec3f(0, 0, sgn))
        cnt.append(seg); idx += [base + i for i in (range(seg) if sgn > 0 else range(seg - 1, -1, -1))]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts); mesh.CreateFaceVertexCountsAttr(cnt); mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateNormalsAttr(nrm); mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    return mesh


def label_cyl(stage, path, radius, phi, z, w, h, texture, seg=24):
    """원기둥 옆면에 감은 라벨 (스티커). phi = 라벨 가운데 방위각, 호 길이 w, 높이 h. 바깥에서 보면 텍스처가 바로 보임."""
    r = radius + LIFT
    half = w / 2 / radius                       # 호 길이 -> 각도
    pts, nrm, st = [], [], []
    for row, zz in ((0, z - h / 2), (1, z + h / 2)):
        for i in range(seg + 1):
            a = phi - half + 2 * half * i / seg
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), zz))
            nrm.append(Gf.Vec3f(math.cos(a), math.sin(a), 0.0))
            st.append(Gf.Vec2f(i / seg, row))   # 각도가 커지는 쪽 = 바깥에서 볼 때 오른쪽
    idx = []
    for i in range(seg):
        idx += [i, i + 1, seg + 2 + i, seg + 1 + i]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * seg)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateNormalsAttr(nrm)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateDoubleSidedAttr(False)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray(st))
    looks = path.rsplit('/', 1)[0].split('/')[1]
    mat = preview_material(stage, f'/{looks}/Looks/{Path(texture).stem}', None, roughness=0.5, texture=texture)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


# ---------- 퍽 (동적 강체 + 태그) ----------
def write_puck(path: Path, name: str, color: str, shape: str, tid: int, tex_dir: str = './tex'):
    """퍽 USD 1개. tex_dir = 이 USD 기준 태그 그림 폴더 (dice 퍽은 usd/props/dice/ 에 있어 '../tex'). make_dice_map.py 도 씀"""
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/Puck')
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(PUCK_MASS[shape])
    if shape == 'box':
        body = UsdGeom.Cube.Define(stage, '/Puck/body')
        body.CreateSizeAttr(1.0)
        body.AddScaleOp().Set(Gf.Vec3f(PUCK_D, PUCK_D, PUCK_H))
    else:
        # 충돌 = 원기둥 도형 (렌더 안 함, guide), 보이는 몸 = 64각 매끈한 메쉬 (RTX 가 Cylinder 도형을 각지게 그려서)
        body = UsdGeom.Cylinder.Define(stage, '/Puck/body')
        body.CreateRadiusAttr(PUCK_D / 2); body.CreateHeightAttr(PUCK_H); body.CreateAxisAttr('Z')
        body.CreatePurposeAttr(UsdGeom.Tokens.guide)
        smooth_cylinder(stage, '/Puck/visual', PUCK_D / 2, PUCK_H)
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    pmat = UsdShade.Material.Define(stage, '/Puck/PhysicsMaterial')
    pma = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
    pma.CreateStaticFrictionAttr(0.35); pma.CreateDynamicFrictionAttr(0.3); pma.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(pmat, materialPurpose='physics')
    body_look = preview_material(stage, '/Puck/Looks/body', PUCK_COLORS[color], roughness=PUCK_ROUGHNESS)
    UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(body_look)
    if shape == 'cyl':
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/Puck/visual')).Bind(body_look)
    tex = f'{tex_dir}/tag36h11_{tid:02d}.png'
    hd, hh = PUCK_D / 2, PUCK_H / 2
    faces = [('top', (0, 0, hh), (0, 0, 1), (1, 0, 0)), ('bottom', (0, 0, -hh), (0, 0, -1), (1, 0, 0))]
    if shape == 'box':
        faces += [('px', (hd, 0, 0), (1, 0, 0), (0, 0, 1)), ('nx', (-hd, 0, 0), (-1, 0, 0), (0, 0, 1)),
                  ('py', (0, hd, 0), (0, 1, 0), (0, 0, 1)), ('ny', (0, -hd, 0), (0, -1, 0), (0, 0, 1))]
    for fname, c, nrm, up in faces:
        label_quad(stage, f'/Puck/tag_{fname}', c, nrm, up, PUCK_LABEL, PUCK_LABEL, tex)
    if shape == 'cyl':
        label_cyl(stage, '/Puck/tag_side', hd, 0.0, 0.0, PUCK_LABEL, PUCK_LABEL, tex)   # 옆면: 곡면 따라 감음
    root.GetPrim().SetCustomDataByKey('puck', {'name': name, 'color': color, 'shape': shape, 'tag_id': tid})
    stage.GetRootLayer().Save()


def main():
    for k, (name, color, shape, tid) in enumerate(PUCKS):
        write_puck(OUT / f'puck_{k}.usda', name, color, shape, tid)
    print('퍽 8개:', [(p[0], p[2], f'ID{p[3]}') for p in PUCKS])

    # ---------- 장소 태그 (벽, 60 mm) ----------
    for k, (name, tid) in enumerate(PLACES):
        stage, root = new_stage(OUT / f'place_tag_{k}.usda', 'PlaceTag')
        label_quad(stage, '/PlaceTag/label', (0, 0, 0), (1, 0, 0), (0, 0, 1), PLACE_LABEL, PLACE_LABEL, f'./tex/tag36h11_{tid:02d}.png')
        root.GetPrim().SetCustomDataByKey('tag', {'name': name, 'tag_id': tid, 'label_m': PLACE_LABEL})
        stage.GetRootLayer().Save()
    print('장소 태그 3개:', PLACES)

    # ---------- 바코드 (벽, Code128) ----------
    W = BAR_MODULE * BAR_MODULES
    for k, code in enumerate(BARCODES):
        stage, root = new_stage(OUT / f'barcode_{k}.usda', 'Barcode')
        label_quad(stage, '/Barcode/plate', (0, 0, 0), (1, 0, 0), (0, 0, 1), W, BAR_H, f'./tex/barcode_{k}.png')
        root.GetPrim().SetCustomDataByKey('code128', code)
        root.GetPrim().SetCustomDataByKey('size_m', Gf.Vec2d(W, BAR_H))
        stage.GetRootLayer().Save()
    print(f'바코드 4개: {BARCODES}, 판 {W * 1000:.0f} × {BAR_H * 1000:.0f} mm')

    # ---------- 가짜 종이 (오탐 대비, 라벨 없음): 줄무늬·QR·글자·체크무늬, 88 × 60 mm ----------
    for k in range(4):
        stage, root = new_stage(OUT / f'distract_paper_{k}.usda', 'DistractPaper')
        label_quad(stage, '/DistractPaper/plate', (0, 0, 0), (1, 0, 0), (0, 0, 1), 0.088, 0.060, f'./tex/distract_{k}.png')
        stage.GetRootLayer().Save()
    print('가짜 종이 4장: distract_paper_0..3 (줄무늬, QR, 글자, 체크무늬)')

    # ---------- 신호등 기둥 (프로파일 20×20×450 + STL040 3단 세로) ----------
    PROFILE_W, PROFILE_H = 0.020, 0.450
    R = 0.020
    SEGMENTS = [('cap', 0.012, (0.55, 0.55, 0.55), 0.5),
                ('div0', 0.001, (0.02, 0.02, 0.02), 0.9), ('red', 0.035, (0.45, 0.04, 0.04), 0.4),
                ('div1', 0.001, (0.02, 0.02, 0.02), 0.9), ('yellow', 0.035, (0.50, 0.38, 0.04), 0.4),
                ('div2', 0.001, (0.02, 0.02, 0.02), 0.9), ('green', 0.035, (0.04, 0.32, 0.10), 0.4)]
    total = sum(s[1] for s in SEGMENTS)
    stage, root = new_stage(OUT / 'traffic_light_post.usda', 'TrafficLightPost')
    prof = UsdGeom.Cube.Define(stage, '/TrafficLightPost/profile')
    prof.CreateSizeAttr(1.0)
    prof.AddTranslateOp().Set(Gf.Vec3d(0, 0, PROFILE_H / 2))
    prof.AddScaleOp().Set(Gf.Vec3f(PROFILE_W, PROFILE_W, PROFILE_H))
    prof.CreateDisplayColorAttr([Gf.Vec3f(0.70, 0.71, 0.73)])
    UsdPhysics.CollisionAPI.Apply(prof.GetPrim())
    alu = preview_material(stage, '/TrafficLightPost/Looks/aluminum', (0.70, 0.71, 0.73), 0.4)
    alu.GetPrim().GetChild('Surface').GetAttribute('inputs:metallic').Set(0.8)
    UsdShade.MaterialBindingAPI.Apply(prof.GetPrim()).Bind(alu)
    x_tl = PROFILE_W / 2 + R  # 프로파일 앞면(칸 쪽)에 붙임
    z = PROFILE_H  # 캡 위 끝 = 프로파일 위 끝
    for name, h, color, rough in SEGMENTS:
        c = UsdGeom.Cylinder.Define(stage, f'/TrafficLightPost/{name}')
        c.CreateRadiusAttr(R)
        c.CreateHeightAttr(h)
        c.CreateAxisAttr('Z')
        c.AddTranslateOp().Set(Gf.Vec3d(x_tl, 0, z - h / 2))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(preview_material(stage, f'/TrafficLightPost/Looks/{name}', color, rough))
        z -= h
    root.GetPrim().SetCustomDataByKey('tower_z_m', Gf.Vec2d(PROFILE_H - total, PROFILE_H))
    stage.GetRootLayer().Save()
    print(f'traffic_light_post: 프로파일 {PROFILE_W * 1000:.0f}×{PROFILE_W * 1000:.0f}×{PROFILE_H * 1000:.0f} mm, '
          f'신호등 Ø{2 * R * 1000:.0f}×{total * 1000:.0f} mm 세로, 바닥에서 {(PROFILE_H - total) * 1000:.0f}–{PROFILE_H * 1000:.0f} mm, '
          f'기둥 앞으로 {(PROFILE_W / 2 + 2 * R) * 1000:.0f} mm')



if __name__ == '__main__':
    main()
