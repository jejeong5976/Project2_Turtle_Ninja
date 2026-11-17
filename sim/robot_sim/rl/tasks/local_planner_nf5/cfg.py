"""주행 정책 nf5 (local_planner_nf5, 배포 config/rl/nav_nf5): local_planner_nav2 + 목표 앞 정지 막기 · 주행 상한 0.255 + PolygonStop 구역 학습.

바뀐 것:
  - reward.stall_goal_dist 0.05: 멈춤 벌점을 목표 5 cm (커리큘럼 0 단계 성공 범위) 안에 들어올 때까지 줌
    (목표 0.3 m 안 조향 벌점과 0.3 m 밖 멈춤 벌점이 만나면 0.3 m 경계 바로 밖에 서는 게 이득 → 막음)
  - reward.steer_near_mult 3: 목표 0.3 m 안 조향 이동 벌점 배수
  - env.vmax 0.255 (controllers.yaml max_linear, 바퀴 모터 끝 0.261 의 98 %)
  - env.stop_zone 0.02: 외곽 0.16 에서 2 cm 안 (= collision_monitor PolygonStop ±0.18) 에 지도 장애물이 들어오면 충돌로 끝냄 (사람 뺌)
    목표 0.40 m 안 (dock 넘김) · 출발 0.30 m 안 (도킹 자리 출발 = 노드가 영역 끔) 은 예외. 외곽 0.16 기준으로만 배우면
    벽 · 기둥에 바짝 붙어 지나다 PolygonStop 에 걸려 모든 명령 0 (교착)
  - reward.clear_margin 0.06: 벽 여유 벌점이 외곽 6 cm 부터 (정지 구역 2 cm 보다 앞에서 기울기)
경로 = navfn. 박스 · 장소는 정차점 0.40 m 안에서 dock 으로 넘기고, 출발점 · 바코드 자리는 끝까지 감"""
from dataclasses import dataclass, field

from local_planner_nav2.cfg import Cfg as _NavCfg
from local_planner_nav2.cfg import TrainCfg as _NavTrainCfg


@dataclass
class TrainCfg(_NavTrainCfg):
    experiment_name: str = 'local_planner_nf5'


@dataclass
class Cfg(_NavCfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        self.reward.stall_goal_dist = 0.05
        self.reward.steer_near_mult = 3.0
        self.env.vmax = 0.255                  # 실물 주행 상한 (바퀴 모터 끝 0.261 의 98 %)
        self.env.stop_zone = 0.02
        self.reward.clear_margin = 0.06
