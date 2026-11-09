"""주행 정책 nf5 (local_planner_nf5): nf4 (vmax 0.255 · 목표 앞 정지 고침) + 실물 PolygonStop 구역 학습.

nf4 Isaac (2026-11-07): 8/10. B1 · L2 = 0.255 에서 벽 · 기둥에 바짝 붙어 지나다 PolygonStop ±0.18 에 걸림 → 모든 명령 0 → 교착
  (nf 는 collision_monitor 없이 외곽 0.16 기준으로 학습해서 0.18 정지 구역을 모름)
바뀐 것:
  - env.stop_zone 0.02: 외곽 0.16 에서 2 cm 안 (= PolygonStop ±0.18) 에 지도 장애물이 들어오면 충돌로 끝냄 (사람 뺌)
    목표 0.40 m 안 (dock 넘김) · 출발 0.30 m 안 (도킹 자리 출발 = 노드가 영역 끔) 은 예외
  - reward.clear_margin 0.04 → 0.06: 벽 여유 벌점이 외곽 6 cm 부터 (정지 구역 2 cm 보다 앞에서 기울기)"""
from dataclasses import dataclass, field

from local_planner_nf4.cfg import Cfg as _NF4Cfg
from local_planner_nf4.cfg import TrainCfg as _NF4TrainCfg


@dataclass
class TrainCfg(_NF4TrainCfg):
    experiment_name: str = 'local_planner_nf5'


@dataclass
class Cfg(_NF4Cfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        self.env.stop_zone = 0.02
        self.reward.clear_margin = 0.06
