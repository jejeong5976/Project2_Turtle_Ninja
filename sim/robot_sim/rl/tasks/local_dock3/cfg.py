"""정밀 접근 정책 dock3 (local_dock3): local_dock + 성공 1 cm.

성공 2 cm 판정 + 3 cm 안 명령 크기 벌점이면 2 cm 경계에 들자마자 멈추는 게 이득 → 짧게 섬. 바뀐 것:
  - goal_pos_tol 커리큘럼 0.05 → 0.03 → 0.01
  - hold_dist 0.01: 명령 크기 벌점은 1 cm 안에서만
  - max_iterations 2000
  - 구간 dock_legs_v3.npz: 정차 거리 0.2525–0.2875 (틈 0.5–4 cm). 목표는 관측으로 받으므로 이 안이면 dice_map 정차점만 바꿔 씀
  - vmax 0.255, 움직이며 출발 0.05–0.255 m/s (주행 상한)
배포: 노드 --tol_pos 0.01"""
from dataclasses import dataclass, field

from local_dock.cfg import RL_DIR
from local_dock.cfg import Cfg as _DockCfg
from local_dock.cfg import TrainCfg as _DockTrainCfg


@dataclass
class TrainCfg(_DockTrainCfg):
    experiment_name: str = 'local_dock3'
    max_iterations: int = 2000


@dataclass
class Cfg(_DockCfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        self.env.goal_pos_tol = (0.05, 0.03, 0.01)
        self.env.dice_legs = str(RL_DIR / 'data' / 'local_dock' / 'dock_legs_v3.npz')   # 정차 0.2525–0.2875 (틈 0.5–4 cm)
        self.reward.hold_dist = 0.01
        self.env.vmax = 0.255                  # 주행 상한 0.255: 넘겨받는 속도 · 처음 몇 cm. 박스 앞은 approach_limit 가 정함
        self.env.start_speed = (0.05, 0.255)   # 움직이며 출발 = Nav2 (0.255) 에서 넘겨받음
