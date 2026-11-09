"""주행 정책 nf4 (local_planner_nf4): local_planner_nav2 (nf3 설정) + 목표 앞 정지 고침.

nf3 결과 (2026-11-06): 목표 약 31 cm 앞에 멈춰 섬 (시간 초과 97 %, 학습 중 성공 0)
  원인 = 목표 0.3 m 안 조향 벌점 × 10 ↔ 0.3 m 밖 멈춤 벌점 → 0.3 m 경계 바로 밖에 서는 게 가장 덜 손해
바뀐 것:
  - stall_goal_dist 0.3 → 0.05: 멈춤 벌점을 목표 5 cm (커리큘럼 0 단계 성공 범위) 안에 들어올 때까지 줌
  - steer_near_mult 10 → 3: 목표 0.3 m 안 조향 이동 벌점 배수 줄임
  - vmax 0.2 → 0.255 (실물 controllers.yaml max_linear 0.255, 2026-11-06)
경로 = navfn (local_planner_nav2 기본). 박스 · 장소는 정차점 0.40 m 안에서 dock 으로 넘기지만, 출발점 · 바코드 자리는 끝까지 감"""
from dataclasses import dataclass, field

from local_planner_nav2.cfg import Cfg as _NavCfg
from local_planner_nav2.cfg import TrainCfg as _NavTrainCfg


@dataclass
class TrainCfg(_NavTrainCfg):
    experiment_name: str = 'local_planner_nf4'


@dataclass
class Cfg(_NavCfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        self.reward.stall_goal_dist = 0.05
        self.reward.steer_near_mult = 3.0
        self.env.vmax = 0.255                  # 2026-11-06 실물 주행 상한 0.2 → 0.255 (바퀴 모터 끝 0.261 의 98 %)
