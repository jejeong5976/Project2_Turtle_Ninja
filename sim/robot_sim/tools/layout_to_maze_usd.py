#!/usr/bin/env python3
"""실제 미로(팀 배포 '미로_전체부품_R3_R5_M4_2026-10-23')의 배치 JSON -> 시뮬레이션용 레이아웃 + USD.

입력: source/twin-90-B-A1-R3-layout.json (팀 배포 '미로_전체부품_R3_R5_M4_2026-10-23/06_전체배치_1대5_A3' 의 배치 파일 복사본)
  - A·B 경기장 2개가 같은 모양 (B = A + y 3152 mm). 시뮬레이션은 A 경기장 1개 (원점: A 왼쪽 아래 기둥 중심, +x 오른쪽 · +y 위쪽)
  - 기둥 노드: 3030 알루미늄(30×30, D60 마운트 Ø60×25 mm 위·아래) / R3 출력 기둥(36×9.5, R5 마운트 Ø60, 높이 73.55 mm 위·아래)
  - 벽판: 두께 3 mm, 높이 380 mm, 길이 720(가로 칸) / 380(세로 칸 788 mm 를 중간 기둥으로 둘로 나눔), 양 끝 간격 7 mm
출력 (미로 중심 = 원점, 칸 격자는 기존 시뮬레이션과 같은 734 × 788 mm, j=0 이 북쪽):
  maze_real_layout.json   칸·통로·크기 + collision_boxes(벽판·기둥, 라이다 계산용) + mounts
  usd/maze_real.usd       보이는 형상 + 충돌 (벽판 회색, 3030 기둥 진회색, R3·R5 주황, D60 진회색)
가정 (도면에 없음): 벽판 아래 끝 높이 5 mm, R5 마운트 충돌은 Ø60 × 25 mm 원통으로 단순화 (팔은 기둥 쪽으로 붙어 있어 생략),
  R3 기둥은 36 × 9.5 mm 상자 (splice_yaw 방향). 위쪽 마운트는 보이기만 하고 충돌은 포함 (로봇 높이에는 영향 없음)
실행: usd-core 가 있는 파이썬 (예: python3 -m pip install usd-core 후 python3, 또는 Isaac Lab 파이썬) tools/layout_to_maze_usd.py
  write_maze_usd 는 tools/random_maze.py 도 씀 (import 만 하면 파일 안 씀)
"""
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parents[1]   # sim/robot_sim (입출력 기준)
SRC = HERE / 'source' / 'twin-90-B-A1-R3-layout.json'
CX, CY = 0.734, 0.788
NX, NY = 6, 4
X0, Y0 = 3 * CX, 2 * CY                    # A 경기장 중심 (post 중심 좌표계) -> 원점
PANEL_T, PANEL_H, PANEL_Z0 = 0.003, 0.380, 0.005
POST_H = 0.390
MOUNT_R, MOUNT_H, R5_H = 0.030, 0.025, 0.07355
REAL = HERE / 'usd' / 'textures' / 'real_colors.json'   # photos_to_textures.py (실제 사진에서 뽑은 색, 선형). 있으면 덮어씀
PANEL_ROUGH = 0.25   # 벽판: 살짝 유광, 사람·로봇이 비침 (사용자 확인 2026-10-24). 이전 0.6 (무광)



def write_maze_usd(path: Path, boxes: list, mounts: list):
    """벽판·기둥 상자(boxes)와 마운트(mounts) -> 미로 USD (보이는 형상 + 충돌). random_maze.py 도 씀."""
    COL = {'panel': (0.78, 0.80, 0.82), 'post_3030': (0.22, 0.22, 0.24), 'post_R3': (0.80, 0.50, 0.12),
           'D60': (0.25, 0.25, 0.27), 'R5': (0.85, 0.55, 0.15)}
    if REAL.exists():
        rc = json.loads(REAL.read_text())['colors']
        # 사진 색은 '조명 받은 겉보기'라 그대로 반사율로 쓰면 세로면이 어두움 (배율 없이 렌더하면 벽판 29 vs 사진 76).
        # 파노라마 돔 조명(2026-10-23, 세기 1800) 렌더에서 벽판이 사진과 같아지도록 세로면 반사율에 배율을 곱함 (보정값, 측정 아님)
        VGAIN = 2.4
        g = lambda c: tuple(min(1.0, v * VGAIN) for v in c)
        COL.update({'panel': g(rc['panel']['linear']), 'post_3030': g(rc['aluminum']['linear']),
                    'post_R3': g(rc['blue_print']['linear']), 'D60': g(rc['blue_print']['linear']),
                    'R5': g(rc['blue_print']['linear'])})   # 실물: R3 이음판·마운트 모두 파란 출력물, 3030 은 은색
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/Maze')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, '/Maze/Collision')
    mat = UsdShade.Material.Define(stage, '/Maze/PhysicsMaterial_PLA')
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.35); pm.CreateDynamicFrictionAttr(0.3); pm.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/Maze/Collision')).Bind(
        mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose='physics')
    looks = {}
    for k, c in COL.items():
        m = UsdShade.Material.Define(stage, f'/Maze/Looks/{k}')
        sh = UsdShade.Shader.Define(stage, f'/Maze/Looks/{k}/Surface')
        sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*c))
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(PANEL_ROUGH if k == 'panel' else 0.45)
        sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.3 if k == 'post_3030' else 0.0)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        looks[k] = m


    def add_box(path, center, size, look):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*COL[look])])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(looks[look])


    def add_cyl(path, xy, z0, h, r, look):
        cy = UsdGeom.Cylinder.Define(stage, path)
        cy.CreateRadiusAttr(r); cy.CreateHeightAttr(h); cy.CreateAxisAttr('Z')
        cy.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], z0 + h / 2))
        cy.CreateDisplayColorAttr([Gf.Vec3f(*COL[look])])
        UsdPhysics.CollisionAPI.Apply(cy.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cy.GetPrim()).Bind(looks[look])


    for b in boxes:
        add_box(f"/Maze/Collision/{b['name']}", b['center'], b['size'], b['kind'])
    for k, m in enumerate(mounts):
        add_cyl(f"/Maze/Collision/mount_bot_{k}", m['xy'], 0.0, MOUNT_H, MOUNT_R, m['type'])
        add_cyl(f"/Maze/Collision/mount_top_{k}", m['xy'], POST_H - MOUNT_H, MOUNT_H, MOUNT_R, m['type'])
    stage.GetRootLayer().Save()



def write_carpet():
    CARPET = HERE / 'usd' / 'textures' / 'carpet_tile.jpg'
    if CARPET.exists():
        tile_m = json.loads(REAL.read_text())['carpet']['tile_m'] if REAL.exists() else 0.4
        S = 60.0                                      # 모든 env 를 덮는 한 장 (/World/Floor)
        (HERE / 'usd' / 'floor_carpet.usda').unlink(missing_ok=True)
        fs = Usd.Stage.CreateNew(str(HERE / 'usd' / 'floor_carpet.usda'))
        UsdGeom.SetStageUpAxis(fs, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(fs, 1.0)
        fr = UsdGeom.Xform.Define(fs, '/Floor'); fs.SetDefaultPrim(fr.GetPrim())
        mesh = UsdGeom.Mesh.Define(fs, '/Floor/Carpet')
        mesh.CreatePointsAttr([Gf.Vec3f(-S / 2, -S / 2, 0), Gf.Vec3f(S / 2, -S / 2, 0), Gf.Vec3f(S / 2, S / 2, 0), Gf.Vec3f(-S / 2, S / 2, 0)])
        mesh.CreateFaceVertexCountsAttr([4]); mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4); mesh.SetNormalsInterpolation('vertex')
        n = S / tile_m
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(n, 0), Gf.Vec2f(n, n), Gf.Vec2f(0, n)])
        fm = UsdShade.Material.Define(fs, '/Floor/Looks/Carpet')
        sh = UsdShade.Shader.Define(fs, '/Floor/Looks/Carpet/Surface'); sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.95)
        rd = UsdShade.Shader.Define(fs, '/Floor/Looks/Carpet/st'); rd.CreateIdAttr('UsdPrimvarReader_float2')
        rd.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
        tx = UsdShade.Shader.Define(fs, '/Floor/Looks/Carpet/Tex'); tx.CreateIdAttr('UsdUVTexture')
        tx.CreateInput('file', Sdf.ValueTypeNames.Asset).Set('./textures/carpet_tile.jpg')
        tx.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat'); tx.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
        tx.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
        tx.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), 'result')
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), 'rgb')
        fm.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(fm)
        fs.GetRootLayer().Save()
        print(f'usd/floor_carpet.usda: {S} m 정사각, 카펫 {tile_m} m 반복')


def main():
    D = json.loads(SRC.read_text())
    nodes = {n['id']: n for n in D['nodes'] if n['xz_mm'][1] <= 3152 + 1}
    panels = []
    for p in D['panels']:
        if p['a'] in nodes and p['b'] in nodes:
            a, b = nodes[p['a']]['xz_mm'], nodes[p['b']]['xz_mm']
            if max(a[1], b[1]) <= 3152 + 1:
                panels.append((p, a, b))
    to_c = lambda xz: (xz[0] / 1000 - X0, xz[1] / 1000 - Y0)

    # 칸 · 통로: 인접 칸 사이 경계에 벽판이 있으면 막힘
    cells = []
    for j in range(NY):
        for i in range(NX):
            cells.append({'i': i, 'j': j, 'xy': [round((i + 0.5) * CX - X0, 4), round(Y0 - (j + 0.5) * CY, 4)], 'region': 1})
    blocked = set()
    for p, a, b in panels:
        (ax, ay), (bx, by) = to_c(a), to_c(b)
        mx, my = (ax + bx) / 2, (ay + by) / 2
        if abs(ay - by) < 1e-6:                  # 가로 벽판 (y 일정): 위·아래 칸 사이
            i = int(math.floor((mx + X0) / CX)); jl = round((Y0 - my) / CY)   # 경계선 번호 (0 = 북쪽 바깥벽)
            if 0 < jl < NY and 0 <= i < NX:
                blocked.add(tuple(sorted(((jl - 1) * NX + i, jl * NX + i))))
        else:                                    # 세로 벽판 (x 일정): 왼·오른쪽 칸 사이
            j = int(math.floor((Y0 - my) / CY)); il = round((mx + X0) / CX)
            if 0 < il < NX and 0 <= j < NY:
                blocked.add(tuple(sorted((j * NX + il - 1, j * NX + il))))
    passable = []
    for j in range(NY):
        for i in range(NX):
            c = j * NX + i
            for d in ((1, 0), (0, 1)):
                ii, jj = i + d[0], j + d[1]
                if ii < NX and jj < NY and tuple(sorted((c, jj * NX + ii))) not in blocked:
                    passable.append([c, jj * NX + ii])

    # 충돌 상자 (라이다·시각화용, 축 정렬)
    boxes = []
    for p, a, b in panels:
        (ax, ay), (bx, by) = to_c(a), to_c(b)
        L = p['length_mm'] / 1000
        horiz = abs(ay - by) < 1e-6
        sx, sy = (L, PANEL_T) if horiz else (PANEL_T, L)
        boxes.append({'name': p['id'], 'kind': 'panel', 'center': [(ax + bx) / 2, (ay + by) / 2, PANEL_Z0 + PANEL_H / 2], 'size': [sx, sy, PANEL_H]})
    mounts = []
    for nid, n in nodes.items():
        x, y = to_c(n['xz_mm'])
        if n['support'] == 'aluminum':
            boxes.append({'name': f'post_{nid}', 'kind': 'post_3030', 'center': [x, y, POST_H / 2], 'size': [0.030, 0.030, POST_H]})
            mtype = 'D60'
        else:
            along_x = abs(math.sin(n.get('splice_yaw', 0.0))) < 0.5
            boxes.append({'name': f'post_{nid}', 'kind': 'post_R3', 'center': [x, y, POST_H / 2],
                          'size': [0.036, 0.0095, POST_H] if along_x else [0.0095, 0.036, POST_H]})
            mtype = 'R5'
        mounts.append({'node': nid, 'type': mtype, 'xy': [x, y], 'r': MOUNT_R, 'h': MOUNT_H, 'top_h': R5_H if mtype == 'R5' else MOUNT_H})

    layout = {
        'source': f'{SRC.name} (A 경기장, layout_to_maze_usd.py)', 'units': 'm, Z-up, 미로 중심 = 원점, 바닥 z=0',
        'size_m': [4.464, 3.212, 0.39], 'inner_half_m': [X0, Y0],
        'grid': {'nx': NX, 'ny': NY, 'cell_m': [CX, CY], 'j0_is': 'north (+y)'},
        'regions': {'1': NX * NY}, 'cells': cells, 'passable_pairs': passable,
        'post_half_m': MOUNT_R,                  # 칸 안 스폰 여유 계산용 = 바닥 마운트 반지름
        'wall_thickness_m': PANEL_T, 'recommended_env_spacing_m': 5.5,
        'collision_boxes': [b for b in boxes], 'mounts': mounts,
        'counts': {'panels': len(panels), 'nodes': len(nodes), 'posts_3030': sum(n['support'] == 'aluminum' for n in nodes.values()),
                   'posts_R3': sum(n['support'] == 'splice' for n in nodes.values())},
        'assumptions': '벽판 아래 5 mm, R5 충돌 Ø60×25 mm 원통, R3 36×9.5 상자',
    }
    (HERE / 'maze_real_layout.json').write_text(json.dumps(layout, indent=1, ensure_ascii=False))

    write_maze_usd(HERE / 'usd' / 'maze_real.usd', boxes, mounts)
    write_carpet()
    print(f"maze_real_layout.json: 칸 {len(cells)}, 통로 {len(passable)}, 벽판 {len(panels)}, 기둥 {len(nodes)} "
          f"(3030 {layout['counts']['posts_3030']}, R3 {layout['counts']['posts_R3']}) / usd/maze_real.usd: 상자 {len(boxes)} + 마운트 {2 * len(mounts)}")


if __name__ == '__main__':
    main()
