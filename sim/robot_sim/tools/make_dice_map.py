#!/usr/bin/env python3
"""발제 v5 '주사위 기억 로봇' 경기장 (4464 × 6364 mm) -> 시뮬레이션용 레이아웃 + USD. 랜덤 미로(random_maze.py)와 따로 둠.

좌표 (발제 그대로): 경기장 좌표 = 왼쪽 아래 기둥 중심 원점, +X 오른쪽 · +Y 위쪽(지도 위), mm. 바닥 z = 0.
  시뮬 좌표 = 경기장 중심 원점 (다른 미로와 같은 규칙): sim = arena − (2202, 3152) mm.
벽 (발제 p.2 그림의 래스터에서 뽑음, 2026-10-28):
  바깥 기둥 = 위·아래 734 간격 7개, 왼·오른쪽 394 간격 17개 (짧은 벽판 380 × 32장).
  안쪽 기둥 = x 367 + 734 i, y 583 + 734 k 격자 (긴 벽판 720 × 18장, 바깥 12장과 합쳐 30장)
  -> 발제 부품표 '긴 벽판 30 · 짧은 벽판 32 · 기둥 30 + 40 = 70' 과 개수 일치.
  그림에서 x 1101 세로벽 아래 끝은 주석 상자에 가려짐 -> y 583 기둥까지로 둠 (그래야 긴 벽판 30장이 맞음)
소품 (발제 p.2·p.7 값):
  시작 박스 6 (205×205×70) · L1/L3 배치대 (205 박스) · 중앙 받침대 820×820×70 (410 박스 4개, 중심 (2202, 3495) 은 그림 실측,
  L2 = 북쪽 두 박스 중심선 y 3700 과 맞음) · 장소 태그 ID 12–14 (접근 방향 면, 중심 높이 35 mm) ·
  신호등 (하단 (80, 950, 0), +X 방향) · 바코드 판 (중심 (80, 650, 450), +X 방향)
가정 (발제에 없음, 바꾸려면 아래 상수):
  - 3030 / R3 기둥 구분: 바깥 734·788 격자점 = 3030 (28개), 나머지 = R3 (실물 30 / 40 과 2개 차이, 겉모습만 다름)
  - 신호등 = 팀 STL040 (Ø40 3단) 을 바닥에 세움, 받침 Ø60×30 mm. 실물 높이 · 발광면 높이 확인 필요
  - 바코드 받침 = 20×20 프로파일 기둥, 판 70 × 24 mm (실측)
  - L1 태그 = 박스 서쪽 면, L2 = 받침대 북쪽 면, L3 = 박스 남쪽 면 (그림의 관찰 원 위치 = 접근 방향)
  - 출발 구역 = 그림 실측 약 600 × 600 mm, 중심 (734, 950)
  - 퍽 ID · 색 · 모양 = 문제 파일 (mazes/dice_problem_*.json). placement.csv 받으면 그 값으로
출력: mazes/dice_layout.json, usd/mazes/dice.usd (벽 + 박스 + 태그 + 바코드 + 신호등, 정적), usd/props/dice/puck_<ID>.usda (동적 퍽 12개)
실행 (usd-core 있는 파이썬): env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python tools/make_dice_map.py [--problem mazes/dice_problem_q1.json]
"""
import argparse
import json
import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

sys.path.insert(0, str(Path(__file__).resolve().parent))
from layout_to_maze_usd import HERE, MOUNT_H, MOUNT_R, PANEL_H, PANEL_T, PANEL_Z0, POST_H, R5_H, write_maze_usd  # noqa: E402
import make_props_usd as mp  # noqa: E402

# ---------- 경기장 (mm, 발제 좌표) ----------
ARENA_W, ARENA_H = 4404, 6304            # 바깥 기둥 중심선 (전체 4464 × 6364 = + 기둥 60)
CENTER = (ARENA_W / 2, ARENA_H / 2)       # 시뮬 원점
GAP = 7                                   # 벽판 끝 ~ 기둥 중심
IX = [367 + 734 * i for i in range(6)]    # 안쪽 기둥 x
IY = [583 + 734 * k for k in range(8)]    # 안쪽 기둥 y
# 안쪽 벽: ((i0, k0), (i1, k1)) 격자 번호, 한 칸 = 긴 벽판 1장
INNER = [((1, 6), (1, 7)),                                        # 시작 박스 4 오른쪽
         ((2, 6), (3, 6)), ((3, 6), (3, 7)),                      # 시작 박스 5 아래 · 오른쪽
         ((4, 6), (4, 7)),                                        # L3 왼쪽
         ((0, 5), (1, 5)), ((1, 4), (1, 5)),                      # 왼쪽 ㄱ자
         ((4, 5), (5, 5)),                                        # 시작 박스 6 위
         ((3, 2), (4, 2)), ((4, 2), (4, 3)), ((4, 3), (4, 4)), ((4, 4), (5, 4)),   # 오른쪽 가운데
         ((1, 2), (1, 3)), ((1, 2), (2, 2)), ((2, 1), (2, 2)), ((1, 1), (2, 1)), ((1, 0), (1, 1)),   # 왼쪽 아래
         ((3, 1), (4, 1)), ((4, 1), (5, 1))]                      # L1 위

BOX_S, BOX_L, BOX_HT = 205, 410, 70
START_BOXES = {1: (1468, 950), 2: (3670, 3152), 3: (2936, 5354), 4: (734, 5354), 5: (2202, 5354), 6: (3670, 3886)}
PUCK_DX = 40                              # 시작 박스 중심 X ±40, Y 0
PLATFORM_C = (2202, 3495)                 # 중앙 받침대 (410 × 4) 중심, 그림 실측
LOCS = {   # 이름: 격자 중심, 태그 ID, 태그 면 중심 (z 35), 태그 바깥 법선, 접근 관찰점 (그림의 원, 대략)
    'L1': {'center': (3670, 950), 'tag_id': 12, 'tag_xy': (3670 - BOX_S / 2, 950), 'normal': (-1, 0), 'approach': (3230, 950)},
    'L2': {'center': (2202, 3700), 'tag_id': 13, 'tag_xy': (2202, PLATFORM_C[1] + BOX_L), 'normal': (0, 1), 'approach': (2202, 4270)},
    'L3': {'center': (3670, 5354), 'tag_id': 14, 'tag_xy': (3670, 5354 - BOX_S / 2), 'normal': (0, -1), 'approach': (3670, 4900)},
}
TAG_Z = 35
GRID_STEP = 60
START_POSE = (734, 950, 180.0)            # 로봇 중심, −X 방향
START_ZONE = (434, 650, 1034, 1250)       # x0, y0, x1, y1 (그림 실측, 가정)
TRAFFIC_LIGHT = (80, 950)                 # 하단 중심, +X 방향
BARCODE = (80, 650, 450)                  # 판 중심, +X 방향
TL_BASE = (0.030, 0.030)                  # 받침 반지름 · 높이 [m] (가정)
TL_R = 0.020                              # STL040 반지름
TL_SEG = [('green', 0.035), ('div1', 0.001), ('yellow', 0.035), ('div2', 0.001), ('red', 0.035), ('cap', 0.012)]   # 아래 -> 위
LAMP_COLOR = {'red': (0.45, 0.04, 0.04), 'yellow': (0.50, 0.38, 0.04), 'green': (0.04, 0.32, 0.10)}   # 꺼진 색 (make_props_usd 와 같음)
BOX_SRGB = (191, 166, 134)                # 골판지 (turtleninja_maze_env.SHELF_SRGB)
BARCODES = ['P2-01', 'P2-02', 'P2-03', 'P2-04']

m = lambda v: v / 1000.0
to_sim = lambda x, y: (round(m(x - CENTER[0]), 4), round(m(y - CENTER[1]), 4))


def walls():
    """벽판 상자 · 기둥 · 마운트 (시뮬 좌표, random_maze.py 와 같은 형식)."""
    segs = []                                                     # 경기장 mm 좌표 (a, b)
    for k in range(6):                                            # 아래 · 위 (734 간격)
        segs += [((734 * k, 0), (734 * (k + 1), 0)), ((734 * k, ARENA_H), (734 * (k + 1), ARENA_H))]
    for k in range(16):                                           # 왼 · 오른 (394 간격)
        segs += [((0, 394 * k), (0, 394 * (k + 1))), ((ARENA_W, 394 * k), (ARENA_W, 394 * (k + 1)))]
    for (i0, k0), (i1, k1) in INNER:
        segs.append(((IX[i0], IY[k0]), (IX[i1], IY[k1])))
    boxes, posts = [], {}
    for a, b in segs:
        horiz = a[1] == b[1]
        L = m(abs(b[0] - a[0]) + abs(b[1] - a[1]) - 2 * GAP)
        c = to_sim((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        boxes.append({'name': f'W{len(boxes) + 1:02d}', 'kind': 'panel', 'center': [c[0], c[1], PANEL_Z0 + PANEL_H / 2],
                      'size': [L, PANEL_T, PANEL_H] if horiz else [PANEL_T, L, PANEL_H]})
        for p in (a, b):
            posts.setdefault(p, set()).add('x' if horiz else 'y')
    mounts, n_alu = [], 0
    for k, (p, dirs) in enumerate(sorted(posts.items())):
        xy = to_sim(*p)
        outer = p[0] in (0, ARENA_W) or p[1] in (0, ARENA_H)
        alu = outer and p[0] % 734 == 0 and p[1] % 788 == 0       # 가정: 바깥 734·788 격자점만 3030
        if alu:
            boxes.append({'name': f'post_{k}', 'kind': 'post_3030', 'center': [xy[0], xy[1], POST_H / 2], 'size': [0.030, 0.030, POST_H]})
            n_alu += 1
        else:
            along_x = 'x' in dirs and 'y' not in dirs
            boxes.append({'name': f'post_{k}', 'kind': 'post_R3', 'center': [xy[0], xy[1], POST_H / 2],
                          'size': [0.036, 0.0095, POST_H] if along_x else [0.0095, 0.036, POST_H]})
        mtype = 'D60' if alu else 'R5'
        mounts.append({'node': f'N{k}', 'type': mtype, 'xy': list(xy), 'r': MOUNT_R, 'h': MOUNT_H,
                       'top_h': R5_H if mtype == 'R5' else MOUNT_H})
    n_short = sum(1 for a, b in segs if abs(b[0] - a[0]) + abs(b[1] - a[1]) == 394)
    counts = {'panels_long_720': len(segs) - n_short, 'panels_short_380': n_short, 'nodes': len(posts),
              'posts_3030': n_alu, 'posts_R3': len(posts) - n_alu}
    return boxes, mounts, counts


def mission_boxes():
    """정적 박스 (이름, 시뮬 중심 xy, 한 변 m). 위 면 z = 0.070."""
    out = [(f'start_{k}', to_sim(*c), m(BOX_S)) for k, c in START_BOXES.items()]
    out += [('place_L1', to_sim(*LOCS['L1']['center']), m(BOX_S)), ('place_L3', to_sim(*LOCS['L3']['center']), m(BOX_S))]
    for k, (dx, dy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        out.append((f'platform_{k}', to_sim(PLATFORM_C[0] + dx * BOX_L / 2, PLATFORM_C[1] + dy * BOX_L / 2), m(BOX_L)))
    return out


def add_mission_props(path: Path, barcode: str):
    """미로 USD 에 박스 · 장소 태그 · 바코드 · 신호등 추가 (모두 정적: 강체 아님, 박스 · 신호등 · 바코드 기둥만 충돌)."""
    stage = Usd.Stage.Open(str(path))
    UsdGeom.Scope.Define(stage, '/Maze/Mission')
    box_look = mp.preview_material(stage, '/Maze/Looks/cardboard', mp.srgb_to_linear(BOX_SRGB), roughness=0.8)
    for name, (x, y), s in mission_boxes():
        cube = UsdGeom.Cube.Define(stage, f'/Maze/Collision/{name}')     # /Maze/Collision 아래 = 미로 물리 재질 같이 받음
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(x, y, m(BOX_HT) / 2))
        cube.AddScaleOp().Set(Gf.Vec3f(s, s, m(BOX_HT)))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(box_look)
    for name, L in LOCS.items():                                        # 장소 태그 (라벨 60 mm, 검은 48 mm)
        x, y = to_sim(*L['tag_xy'])
        nx, ny = L['normal']
        mp.label_quad(stage, f'/Maze/Mission/tag_{name}', (x, y, m(TAG_Z)), (nx, ny, 0), (0, 0, 1),
                      mp.PLACE_LABEL, mp.PLACE_LABEL, f"../props/tex/tag36h11_{L['tag_id']:02d}.png")
    # 바코드: 기둥 (20×20, 판 뒤) + 판 (+X 방향)
    bx, by = to_sim(*BARCODE[:2])
    bz = m(BARCODE[2])
    post = UsdGeom.Cube.Define(stage, '/Maze/Collision/barcode_post')
    post.CreateSizeAttr(1.0)
    post_h = bz + mp.BAR_H / 2 + 0.010
    post.AddTranslateOp().Set(Gf.Vec3d(bx - 0.010, by, post_h / 2))
    post.AddScaleOp().Set(Gf.Vec3f(0.020, 0.020, post_h))
    UsdPhysics.CollisionAPI.Apply(post.GetPrim())
    alu = mp.preview_material(stage, '/Maze/Looks/aluminum', (0.70, 0.71, 0.73), 0.4)
    UsdShade.MaterialBindingAPI.Apply(post.GetPrim()).Bind(alu)
    k = BARCODES.index(barcode)
    mp.label_quad(stage, '/Maze/Mission/barcode', (bx, by, bz), (1, 0, 0), (0, 0, 1),
                  mp.BAR_MODULE * mp.BAR_MODULES, mp.BAR_H, f'../props/tex/barcode_{k}.png')
    stage.GetPrimAtPath('/Maze/Mission/barcode').SetCustomDataByKey('code128', barcode)
    # 신호등: 받침 + STL040 3단. 등 재질 = /Maze/Looks/lamp_<색> (inputs:emissiveColor 를 실행 중 바꿔 점멸, scripts/view_dice.py)
    tx, ty = to_sim(*TRAFFIC_LIGHT)
    z = 0.0
    parts = [('base', TL_BASE[0], TL_BASE[1], (0.10, 0.10, 0.11))] + \
            [(n, TL_R, h, LAMP_COLOR.get(n, (0.02, 0.02, 0.02) if n.startswith('div') else (0.55, 0.55, 0.55))) for n, h in TL_SEG]
    for name, r, h, col in parts:
        c = UsdGeom.Cylinder.Define(stage, f'/Maze/Collision/tl_{name}')
        c.CreateRadiusAttr(r); c.CreateHeightAttr(h); c.CreateAxisAttr('Z')
        c.AddTranslateOp().Set(Gf.Vec3d(tx, ty, z + h / 2))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        look = f'lamp_{name}' if name in LAMP_COLOR else f'tl_{name}'
        mat = mp.preview_material(stage, f'/Maze/Looks/{look}', col, 0.4)
        if name in LAMP_COLOR:
            mat.GetPrim().GetChild('Surface').CreateAttribute('inputs:emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0, 0, 0))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
        z += h
    stage.GetPrimAtPath('/Maze/Mission').SetCustomDataByKey('traffic_light_top_z_m', z)
    stage.GetRootLayer().Save()
    return z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--problem', default=str(HERE / 'mazes' / 'dice_problem_q1.json'))
    a = ap.parse_args()
    prob = json.loads(Path(a.problem).read_text())
    assert prob['barcode'] in BARCODES, f"바코드 {prob['barcode']} 는 {BARCODES} 중 하나"
    ids = sorted(p['id'] for p in prob['pucks'])
    assert ids == list(range(12)), f'퍽 ID 0–11 이 하나씩 있어야 함: {ids}'
    assert len(prob['signal']) == 12 and set(prob['signal']) <= set('RYG'), '신호는 R/Y/G 12개'

    boxes, mounts, counts = walls()
    (HERE / 'usd' / 'mazes').mkdir(parents=True, exist_ok=True)
    usd = HERE / 'usd' / 'mazes' / 'dice.usd'
    write_maze_usd(usd, boxes, mounts)
    tl_top = add_mission_props(usd, prob['barcode'])

    pdir = HERE / 'usd' / 'props' / 'dice'
    pdir.mkdir(parents=True, exist_ok=True)
    pucks = []
    for p in sorted(prob['pucks'], key=lambda p: p['id']):
        bx, by = START_BOXES[p['box']]
        xy = (bx + p['side'] * PUCK_DX, by)
        mp.write_puck(pdir / f"puck_{p['id']:02d}.usda", f"ID{p['id']}", p['color'], p['shape'], p['id'], tex_dir='../tex')
        pucks.append({**p, 'arena_mm': list(xy), 'sim_m': [*to_sim(*xy), m(BOX_HT) + mp.PUCK_H / 2]})

    sim_pose = lambda x, y, yaw: [*to_sim(x, y), yaw]
    layout = {
        'source': 'tools/make_dice_map.py (발제 v5 주사위 기억 로봇, 2026-11-04 판)',
        'units': 'sim_m = m, 경기장 중심 원점 (= 경기장 mm − (2202, 3152)). arena_mm = 발제 좌표 (왼쪽 아래 기둥 원점). 바닥 z=0',
        'arena_origin_in_sim_m': [-m(CENTER[0]), -m(CENTER[1])],
        'size_m': [4.464, 6.364, 0.39], 'inner_half_m': [m(CENTER[0]), m(CENTER[1])],
        'post_half_m': MOUNT_R, 'wall_thickness_m': PANEL_T, 'recommended_env_spacing_m': 8.0,
        'collision_boxes': boxes, 'mounts': mounts, 'counts': counts,
        'problem': {'file': Path(a.problem).name, 'signal': prob['signal'], 'barcode': prob['barcode']},
        'mission': {
            'start_pose': {'arena_mm_deg': list(START_POSE), 'sim_m_deg': sim_pose(*START_POSE)},
            'start_zone': {'arena_mm': list(START_ZONE), 'sim_m': [*to_sim(*START_ZONE[:2]), *to_sim(*START_ZONE[2:])]},
            'traffic_light': {'arena_mm': list(TRAFFIC_LIGHT), 'sim_m': list(to_sim(*TRAFFIC_LIGHT)), 'facing': '+X',
                              'lamp_z_m': {'green': [TL_BASE[1], TL_BASE[1] + 0.035], 'yellow': [TL_BASE[1] + 0.036, TL_BASE[1] + 0.071],
                                           'red': [TL_BASE[1] + 0.072, TL_BASE[1] + 0.107]}, 'top_z_m': round(tl_top, 4)},
            'barcode': {'arena_mm': list(BARCODE), 'sim_m': [*to_sim(*BARCODE[:2]), m(BARCODE[2])], 'facing': '+X', 'code': prob['barcode']},
            'start_boxes': {str(k): {'arena_mm': list(c), 'sim_m': list(to_sim(*c))} for k, c in START_BOXES.items()},
            'platform': {'arena_mm': list(PLATFORM_C), 'sim_m': list(to_sim(*PLATFORM_C)), 'size_m': [0.82, 0.82, 0.07]},
            'places': {n: {'tag_id': L['tag_id'], 'center_arena_mm': list(L['center']), 'center_sim_m': list(to_sim(*L['center'])),
                           'tag_arena_mm': [*L['tag_xy'], TAG_Z], 'tag_sim_m': [*to_sim(*L['tag_xy']), m(TAG_Z)],
                           'tag_normal': list(L['normal']), 'approach_arena_mm': list(L['approach']),
                           'grid_step_m': m(GRID_STEP), 'top_z_m': m(BOX_HT)} for n, L in LOCS.items()},
            'pucks': pucks,
        },
        'assumptions': '3030/R3 구분, 신호등 크기 · 받침, 바코드 기둥, 태그 면, 출발 구역 크기, 중앙 받침대 중심 = 그림 실측. 퍽 배정 = 문제 파일',
    }
    (HERE / 'mazes' / 'dice_layout.json').write_text(json.dumps(layout, indent=1, ensure_ascii=False))
    print(f"dice.usd: 벽판 긴 {counts['panels_long_720']} · 짧은 {counts['panels_short_380']}, 기둥 {counts['nodes']} "
          f"(3030 {counts['posts_3030']}, R3 {counts['posts_R3']}), 박스 {len(mission_boxes())}, 신호등 높이 {tl_top * 1000:.0f} mm")
    print(f"퍽 12개 -> usd/props/dice/ ({prob['name']}: 빨강 {sum(p['color'] == 'red' for p in pucks)}, "
          f"원기둥 {sum(p['shape'] == 'cyl' for p in pucks)}), 신호 {prob['signal']}, 바코드 {prob['barcode']}")


if __name__ == '__main__':
    main()
