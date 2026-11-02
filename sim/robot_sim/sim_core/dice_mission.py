"""주사위 기억 미션 경기장 환경 (발제 v5). 랜덤 소품 환경(TurtleNinjaMazePropsEnvCfg)과 따로 둠.

맵 · 소품은 tools/make_dice_map.py 가 만든 것을 그대로 씀:
  usd/mazes/dice.usd (벽 + 박스 + 장소 태그 + 바코드 + 신호등, 정적), mazes/dice_layout.json (좌표), usd/props/dice/puck_<ID>.usda (퍽 12개)
reset 마다 무작위 없이 같은 배치: 퍽 = 시작 박스 위 (문제 파일 자리), 로봇 = 출발점 (734, 950) mm · −X 방향, 팔 = 접은 자세.
좌표: 시뮬 = 경기장 중심 원점 (= 발제 mm − (2202, 3152)), env 원점 기준.
신호등 점멸은 SignalPlayer (등 재질 inputs:emissiveColor 를 시간에 따라 바꿈). 1회 = 1 s 켜짐 + 1 s 꺼짐, 12번째 뒤 꺼짐 유지 (발제 p.3).
"""
from __future__ import annotations

import json
import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_from_euler_xyz

from . import turtleninja_maze_env as tme

LAYOUT = json.loads((tme.ROOT / 'mazes' / 'dice_layout.json').read_text())
MISSION = LAYOUT['mission']
DICE_USD = tme.ROOT / 'usd' / 'mazes' / 'dice.usd'
PUCK_DIR = tme.PROPS_DIR / 'dice'
NUM_PUCKS = 12
START_SIM = MISSION['start_pose']['sim_m_deg']          # (x, y, yaw 도)
# 켜진 등 발광색 = make_yolo_dataset.py TL_GLOW 와 같음 (너무 크면 하얗게 번짐)
LAMP_ON = {'R': 'red', 'Y': 'yellow', 'G': 'green'}   # 신호 글자 -> 등. 겉모습(렌즈 색 · 발광 · 번짐)은 tme.TL_LAMP (실제 영상에 맞춘 값)


def _puck_cfg(pid: int) -> RigidObjectCfg:
    p = next(p for p in MISSION['pucks'] if p['id'] == pid)
    return RigidObjectCfg(prim_path=f'{{ENV_REGEX_NS}}/Puck_{pid:02d}',
                          init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p['sim_m'])),
                          spawn=sim_utils.UsdFileCfg(usd_path=str(PUCK_DIR / f'puck_{pid:02d}.usda')))


@configclass
class DiceSceneCfg(tme.MazeSceneCfg):
    maze = AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Maze', spawn=sim_utils.UsdFileCfg(usd_path=str(DICE_USD)))
    puck_00 = _puck_cfg(0)
    puck_01 = _puck_cfg(1)
    puck_02 = _puck_cfg(2)
    puck_03 = _puck_cfg(3)
    puck_04 = _puck_cfg(4)
    puck_05 = _puck_cfg(5)
    puck_06 = _puck_cfg(6)
    puck_07 = _puck_cfg(7)
    puck_08 = _puck_cfg(8)
    puck_09 = _puck_cfg(9)
    puck_10 = _puck_cfg(10)
    puck_11 = _puck_cfg(11)


def reset_dice_mission(env, env_ids: torch.Tensor):
    """퍽 12개 = 시작 박스 위 (z = 박스 윗면 + 퍽 반 높이 + 2 mm), 로봇 = 출발점. 무작위 없음."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n, dev = len(env_ids), env.device
    org = env.scene.env_origins[env_ids]
    unit = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
    for p in MISSION['pucks']:
        pos = org + torch.tensor([p['sim_m'][0], p['sim_m'][1], p['sim_m'][2] + 0.002], device=dev)
        obj = env.scene[f"puck_{p['id']:02d}"]
        obj.write_root_pose_to_sim_index(root_pose=torch.cat([pos, unit], -1), env_ids=env_ids)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
    robot = env.scene['robot']
    pos = org.clone()
    pos[:, 0] += START_SIM[0]
    pos[:, 1] += START_SIM[1]
    pos[:, 2] += tme.SPAWN_Z
    yaw = torch.full((n,), math.radians(START_SIM[2]), device=dev)
    zeros = torch.zeros(n, device=dev)
    robot.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat_from_euler_xyz(zeros, zeros, yaw)], -1), env_ids=env_ids)
    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
    q = robot.data.default_joint_pos.torch[env_ids].clone()
    robot.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)


@configclass
class DiceMissionEnvCfg(tme.TurtleNinjaMazeEnvCfg):
    """주사위 미션 경기장 + 퍽 12 + 로봇 (출발점). 행동 = 차체 (vx, vy, wz), 팔은 접은 자세 고정 (기존 환경과 같음)."""
    scene: DiceSceneCfg = DiceSceneCfg(num_envs=1, env_spacing=LAYOUT['recommended_env_spacing_m'])

    def __post_init__(self):
        super().__post_init__()
        self.events.reset_in_maze = EventTerm(func=reset_dice_mission, mode='reset')


def puck_states(env, env_id: int = 0) -> list[dict]:
    """퍽 ID · 위치 (경기장 mm) · 기울기 (도). 시뮬 정답 확인용 (재검증 로그 아님)."""
    o = LAYOUT['arena_origin_in_sim_m']
    org = env.scene.env_origins[env_id]
    out = []
    for p in MISSION['pucks']:
        obj = env.scene[f"puck_{p['id']:02d}"]
        xyz = (obj.data.root_pos_w.torch[env_id] - org).tolist()
        qx, qy, qz, qw = obj.data.root_quat_w.torch[env_id].tolist()
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))
        out.append({'id': p['id'], 'color': p['color'], 'shape': p['shape'],
                    'arena_mm': [round((xyz[0] - o[0]) * 1000, 1), round((xyz[1] - o[1]) * 1000, 1), round(xyz[2] * 1000, 1)],
                    'tilt_deg': round(tilt, 1)})
    return out


class SignalPlayer:
    """신호등 점멸 재생. start(t) 후 update(t) 를 매 스텝 부름. 1회 = on_s 켜짐 + off_s 꺼짐, 마지막 뒤 계속 꺼짐."""

    def __init__(self, stage, num_envs: int, sequence: str, on_s: float = 1.0, off_s: float = 1.0, gain: float = 1.0):
        self.seq, self.on_s, self.off_s, self.gain = sequence, on_s, off_s, gain
        self.looks = []
        for e in range(num_envs):
            m = f'/World/envs/env_{e}/Maze'
            for c in ('red', 'yellow', 'green'):
                assert stage.GetPrimAtPath(f'{m}/Looks/lamp_{c}/Surface'), f'등 재질 없음: lamp_{c} (tools/make_dice_map.py 다시 실행)'
            self.looks.append(tme.TrafficLightLook(stage, m, {c: (f'{m}/Collision/tl_{c}', f'{m}/Looks/lamp_{c}/Surface')
                                                              for c in ('red', 'yellow', 'green')}))
        self.t0, self.cur = None, None
        self._set(None)

    def start(self, t: float):
        self.t0 = t

    def state(self, t: float):
        """(현재 켜진 색 'R'/'Y'/'G' 또는 None, 몇 번째 점멸 0-based 또는 None)."""
        if self.t0 is None or t < self.t0:
            return None, None
        k, r = divmod(t - self.t0, self.on_s + self.off_s)
        k = int(k)
        if k >= len(self.seq) or r >= self.on_s:
            return None, (k if k < len(self.seq) else None)
        return self.seq[k], k

    def update(self, t: float):
        c, _ = self.state(t)
        if c != self.cur:
            self._set(c)
            self.cur = c
        return c

    def _set(self, c):
        for look in self.looks:
            look.set(LAMP_ON[c] if c is not None else None, self.gain)

    @property
    def duration(self) -> float:
        return len(self.seq) * (self.on_s + self.off_s)
