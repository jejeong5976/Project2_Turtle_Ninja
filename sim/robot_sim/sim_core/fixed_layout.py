"""소품 환경에서 선반·로봇을 원하는 자리에 고정 배치 (view_maze.py --shelf / --robot / --map 에서 사용).

랜덤 reset 이 끝난 뒤 덮어쓴다: 지정한 선반 k 는 지정 자리로, 나머지 선반은 미로 밖 대기 자리로.
선반 위 퍽은 선반과 같이 옮긴다 (선반 기준 상대 위치 유지). 신호등 기둥이 고정 선반과 가까우면 대기 자리로 치운다.
바코드·장소 태그는 벽 높이 0.15 m 이상이라 선반(0.07 m)과 겹치지 않아 그대로 둔다.

선반 지정 'CELL:SIDE[:GAP[:ALONG[:YAW]]]'
  CELL  칸 번호 (--map 으로 확인, j=0 이 북쪽 줄)
  SIDE  N/S/E/W = 그 칸의 어느 벽 쪽에 놓을지
  GAP   벽 안쪽 면 ~ 선반 가장 가까운 점 (m, 기본 0 = 붙임)
  ALONG 벽을 따라 칸 중심에서 이동 (m, 기본 0). 벽을 바라볼 때 +는 왼쪽
  YAW   선반 회전 (도, 기본 0). 45 면 모서리가 벽 쪽
로봇 지정 'CELL[:DX:DY:YAW]' = 칸 중심 + (DX, DY) m, 정면(+x, 라이다 쪽) yaw 도 (0 = 동쪽 +x, 90 = 북쪽 +y)
"""
from __future__ import annotations

import math

import torch

from . import turtleninja_maze_env as tme

NX, NY = tme.LAYOUT['grid']['nx'], tme.LAYOUT['grid']['ny']
WALL_T = tme.LAYOUT.get('wall_thickness_m', 0.003)
SIDES = {'N': (0.0, -1.0), 'S': (0.0, 1.0), 'E': (-1.0, 0.0), 'W': (1.0, 0.0)}   # 벽 -> 칸 안쪽 방향
NEIGHBOR = {'N': (0, -1), 'S': (0, 1), 'E': (1, 0), 'W': (-1, 0)}                 # (di, dj)
_PASS = {tuple(sorted(p)) for p in tme.LAYOUT['passable_pairs']}


def _open(c: int, side: str) -> bool:
    """칸 c 의 side 쪽이 뚫려 있는지 (바깥벽이면 False)."""
    i, j = c % NX, c // NX
    di, dj = NEIGHBOR[side]
    ii, jj = i + di, j + dj
    return 0 <= ii < NX and 0 <= jj < NY and tuple(sorted((c, jj * NX + ii))) in _PASS


def maze_map() -> str:
    """칸 번호와 벽을 글자로. 위 = 북쪽(+y), 오른쪽 = 동쪽(+x)."""
    lines = []
    for j in range(NY):
        top = '+'
        mid = '|'
        for i in range(NX):
            c = j * NX + i
            top += ('     ' if _open(c, 'N') else '-----') + '+'
            mid += f' {c:>2}  ' + (' ' if _open(c, 'E') else '|')
        lines += [top, mid]
    lines.append('+' + '-----+' * NX)
    cx, cy = tme.CELL_X, tme.CELL_Y
    lines.append(f'칸 {cx * 1000:.0f} (동서) × {cy * 1000:.0f} (남북) mm, 원점 = 미로 중심. 위 = 북(+y), 오른쪽 = 동(+x)')
    return '\n'.join(lines)


def parse_shelf(spec: str) -> dict:
    f = spec.split(':')
    c, side = int(f[0]), f[1].upper()
    assert 0 <= c < tme.NUM_CELLS and side in SIDES, f'선반 지정 오류: {spec}'
    gap = float(f[2]) if len(f) > 2 else 0.0
    along = float(f[3]) if len(f) > 3 else 0.0
    yaw_e = math.radians(float(f[4])) if len(f) > 4 else 0.0
    nx_, ny_ = SIDES[side]
    cxy = tme.LAYOUT['cells'][c]['xy']
    half_cell = (tme.CELL_Y if side in 'NS' else tme.CELL_X) / 2
    half_ext = tme.SHELF_SIZE[0] / 2 * (abs(math.cos(yaw_e)) + abs(math.sin(yaw_e)))   # 벽 방향 반폭 (정사각형 가정)
    depth = gap + half_ext                                      # 벽 안쪽 면 -> 선반 중심
    wall_face = half_cell - WALL_T / 2
    tx, ty = -ny_, nx_                                          # 벽을 바라볼 때(−법선) 왼쪽이 +along
    x = cxy[0] - nx_ * wall_face + nx_ * depth - tx * along
    y = cxy[1] - ny_ * wall_face + ny_ * depth - ty * along
    yaw = math.atan2(ny_, nx_) + yaw_e
    cell_w = 2 * half_cell
    clear = cell_w - WALL_T - (gap + 2 * half_ext)
    if _open(c, {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[side]):
        note = f'맞은편이 뚫려 있음 (칸 안 남는 폭 {clear:.3f} m, 실제 통로는 더 넓음)'
    else:
        note = f'맞은편 벽까지 남는 폭 {clear:.3f} m'
    return {'spec': spec, 'cell': c, 'xy': (x, y), 'yaw': yaw, 'clear': clear, 'note': note}


def parse_robot(spec: str) -> tuple[float, float, float]:
    f = spec.split(':')
    c = int(f[0])
    dx = float(f[1]) if len(f) > 1 else 0.0
    dy = float(f[2]) if len(f) > 2 else 0.0
    yaw = math.radians(float(f[3])) if len(f) > 3 else 0.0
    cxy = tme.LAYOUT['cells'][c]['xy']
    return cxy[0] + dx, cxy[1] + dy, yaw


def describe(shelves: list[dict]) -> str:
    out = []
    for k, s in enumerate(shelves):
        out.append(f"  shelf_{k}: {s['spec']:<18} 중심 ({s['xy'][0]:+.3f}, {s['xy'][1]:+.3f}) m, "
                   f"yaw {math.degrees(s['yaw']):+.0f}°, {s['note']}")
    for a in range(len(shelves)):
        for b in range(a + 1, len(shelves)):
            d = math.dist(shelves[a]['xy'], shelves[b]['xy'])
            if d < tme.SHELF_SIZE[0] * math.sqrt(2):
                out.append(f'  ! shelf_{a} 과 shelf_{b} 중심 거리 {d:.3f} m < {tme.SHELF_SIZE[0] * math.sqrt(2):.3f} -> 겹칠 수 있음 (ALONG 으로 띄우기)')
    out.append('  (참고) 칸 모서리 기둥 아래 마운트 Ø60 이 3 cm 튀어나와 모서리 쪽은 더 좁음. '
               '로봇 회전 반경 약 0.20 m (추정) -> 제자리 회전에 약 0.40 m 필요')
    return '\n'.join(out)


def _yaw_quat(yaw: torch.Tensor) -> torch.Tensor:
    z = torch.zeros_like(yaw)
    return tme.quat_from_euler_xyz(z, z, yaw)


def apply(env, shelves: list[dict], robot: tuple[float, float, float] | None):
    """env.reset() 바로 뒤에 호출. 모든 env 에 같은 배치."""
    st = tme.maze_state(env)
    n, dev = env.num_envs, env.device
    ids = torch.arange(n, device=dev)
    org = env.scene.env_origins
    half = tme.SHELF_SIZE[0] / 2 + 0.01

    # 선반 k: 새 자세 (미로 좌표). 지정 안 한 선반은 대기 자리
    new_xy, new_yaw, new_active = [], [], []
    for k in range(tme.NUM_SHELVES):
        if k < len(shelves):
            new_xy.append(torch.tensor(shelves[k]['xy'], device=dev).expand(n, 2))
            new_yaw.append(torch.full((n,), shelves[k]['yaw'], device=dev))
            new_active.append(torch.ones(n, dtype=torch.bool, device=dev))
        else:
            p = tme._prop_init(8 + k).pos
            new_xy.append(torch.tensor(p[:2], device=dev).expand(n, 2))   # 미로 좌표 (env 원점 기준)
            new_yaw.append(torch.zeros(n, device=dev))
            new_active.append(torch.zeros(n, dtype=torch.bool, device=dev))
    new_xy, new_yaw, new_active = torch.stack(new_xy, 1), torch.stack(new_yaw, 1), torch.stack(new_active, 1)
    old_xy, old_yaw, old_active = st['shelf_xy'], st['shelf_yaw'], st['shelf_active']
    # 대기 선반의 옛 자리는 랜덤 reset 이 쓴 대기 위치 (shelf_xy 에는 벽 자리가 남아 있음)
    for k in range(tme.NUM_SHELVES):
        park = torch.tensor(tme._prop_init(8 + k).pos[:2], device=dev)
        old_xy[:, k] = torch.where(old_active[:, k:k + 1], old_xy[:, k], park.expand(n, 2))
        old_yaw[:, k] = torch.where(old_active[:, k], old_yaw[:, k], torch.zeros(n, device=dev))

    # 퍽: 어느 선반 위에 있는지 찾아 선반과 같이 이동
    for p in range(len(tme.PUCKS)):
        obj = env.scene[f'puck_{p}']
        pos = obj.data.root_pos_w.torch.clone()
        quat = obj.data.root_quat_w.torch.clone()
        pxy = pos[:, :2] - org[:, :2]
        for k in range(tme.NUM_SHELVES):
            c, s = torch.cos(old_yaw[:, k]), torch.sin(old_yaw[:, k])
            d = pxy - old_xy[:, k]
            lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
            on = (lx.abs() < half) & (ly.abs() < half)
            if not on.any():
                continue
            c2, s2 = torch.cos(new_yaw[:, k]), torch.sin(new_yaw[:, k])
            nxy = new_xy[:, k] + torch.stack([c2 * lx - s2 * ly, s2 * lx + c2 * ly], -1)
            pyaw = tme.euler_xyz_from_quat(quat)[2] + new_yaw[:, k] - old_yaw[:, k]
            pos[:, :2] = torch.where(on[:, None], nxy + org[:, :2], pos[:, :2])
            quat = torch.where(on[:, None], _yaw_quat(pyaw), quat)
            pxy = torch.where(on[:, None], torch.full_like(pxy, 1e6), pxy)   # 한 번만 옮김
        obj.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], -1), env_ids=ids)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)

    for k in range(tme.NUM_SHELVES):
        pos = torch.cat([new_xy[:, k] + org[:, :2], torch.full((n, 1), tme.SHELF_SIZE[2] / 2, device=dev)], -1)
        env.scene[f'shelf_{k}'].write_root_pose_to_sim_index(root_pose=torch.cat([pos, _yaw_quat(new_yaw[:, k])], -1),
                                                            env_ids=ids)

    # 신호등 기둥이 고정 선반 가까이 있으면 치움
    tl = env.scene['traffic_light']
    tl_xy = tl.data.root_pos_w.torch[:, :2] - org[:, :2]
    near = torch.zeros(n, dtype=torch.bool, device=dev)
    for k in range(len(shelves)):
        near |= (tl_xy - new_xy[:, k]).norm(dim=-1) < 0.30
    if near.any():
        p = tme._prop_init(7).pos
        park = torch.tensor([p[0], p[1], 0.0, 0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 7).clone()
        park[:, :2] += org[:, :2]
        cur = torch.cat([tl.data.root_pos_w.torch, tl.data.root_quat_w.torch], -1)
        tl.write_root_pose_to_sim_index(root_pose=torch.where(near[:, None], park, cur), env_ids=ids)

    # 상태 갱신 (다른 코드가 읽는 값)
    st['shelf_xy'][:], st['shelf_yaw'][:], st['shelf_active'][:] = new_xy, new_yaw, new_active
    st['blocked'][:] = False
    for s in shelves:
        st['blocked'][:, s['cell']] = True

    robot_asset = env.scene['robot']
    if robot is None:   # 랜덤 시작 칸을 고정 선반 칸을 피해 다시 뽑음
        tme.reset_robot_in_maze(env, ids, tme.FOOTPRINT_RADIUS, 0.02)
        return
    x, y, yaw = robot
    pos = org.clone()
    pos[:, 0] += x
    pos[:, 1] += y
    pos[:, 2] += tme.SPAWN_Z
    robot_asset.write_root_pose_to_sim_index(root_pose=torch.cat([pos, _yaw_quat(torch.full((n,), yaw, device=dev))], -1),
                                             env_ids=ids)
    robot_asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
