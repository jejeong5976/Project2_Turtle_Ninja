"""정밀 접근 정책 dock3 (local_dock3): dock2 (local_dock) 그대로 + 마지막 단계 성공 1 cm.

dock2 결과 (2026-11-06, 학습 플랜트): 성공 99.8 % · 충돌 0.19 % 인데 1.4 cm 짧게 섬 (박스 면 틈 중앙 2.46 cm, 목표 1 cm)
  원인 = 성공 2 cm 판정 + 3 cm 안 명령 크기 벌점 (hold_cmd) → 2 cm 경계 안에 들어오자마자 멈추는 게 이득
바뀐 것:
  - goal_pos_tol 마지막 단계 0.02 → 0.01 (커리큘럼 0.05 → 0.03 → 0.01)
  - hold_dist 0.03 → 0.01: 명령 크기 벌점은 1 cm 안에서만 (그 밖에서는 끝까지 붙어도 벌점 없음)
  - max_iterations 1500 → 2000 (판정이 좁아 늦게 수렴할 수 있음)
  - 구간 dock_legs_v3.npz: 정차 거리 0.2525–0.2875 (틈 0.5–4 cm, dock2 는 0.5–2 cm). 목표는 관측으로 받으니 이 안이면
    dice_map.yaml 정차점만 바꿔 쓸 수 있음 (먼 줄 2 단계 놓기 범위 정해지면)
  - vmax 0.2 → 0.255, 움직이며 출발 0.05–0.255 m/s (주행 상한 0.255, 2026-11-06)
배포: 노드 --tol_pos 0.01 (나머지 dock2 와 같음)"""
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
        self.env.vmax = 0.255                  # 2026-11-06 주행 상한 0.255: 넘겨받는 속도 · 처음 몇 cm. 박스 앞은 approach_limit 가 정함
        self.env.start_speed = (0.05, 0.255)   # 움직이며 출발 = Nav2 (0.255) 에서 넘겨받음
