"""주행 정책 nf6 (local_planner_nf6): nf5 (PolygonStop 구역 학습) + 헛회전 · 회전 방향 반전 벌점.

목적: 몸 회전을 필요한 만큼만 (odom yaw 오차 누적 대비, 2026-11-09 사용자)
  - 스워브라 이동 중 몸을 돌릴 필요가 없음 → 필요한 회전 하한 = 출발 yaw ↔ 정차 yaw 차이뿐
  - nf5 까지는 회전 크기 벌점 (rot_cost, 목표 2 m 밖만) 뿐이라 목표 근처에서 갔다 오는 회전 · 좌우 번갈아 도는 회전을 따로 안 벌줌
바뀐 것:
  - reward.rot_waste 2.0: 스텝마다 몸 yaw 변화 중 목표 방향 오차를 줄이지 않은 만큼 [rad] × 2 (참 자세)
    최대 회전 0.5 rad/s · 0.05 s 면 스텝당 최대 −0.05 (= stall 벌점과 같은 크기, progress 최고 +0.064)
  - reward.rot_reversal 0.02: wz 명령 부호가 바뀌면 (앞 · 지금 둘 다 |wz| ≥ 0.05 rad/s) 한 번에 −0.02
  - max_iterations 1500 (nf4 곡선: 1500 뒤 거의 안 늘어남, 모자라면 --resume)
그 밖 (rot_cost · goal_yaw_far 2.0 · stop_zone · vmax 0.255) 은 nf5 그대로
비교 지표 (학습 로그 · play): /ep/rot_total_rad · /ep/rot_waste_rad · /ep/rot_reversals (nf5 도 같은 env 라 같은 지표가 나옴)"""
from dataclasses import dataclass, field

from local_planner_nf5.cfg import Cfg as _NF5Cfg
from local_planner_nf5.cfg import TrainCfg as _NF5TrainCfg


@dataclass
class TrainCfg(_NF5TrainCfg):
    experiment_name: str = 'local_planner_nf6'
    max_iterations: int = 1500


@dataclass
class Cfg(_NF5Cfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        self.reward.rot_waste = 2.0
        self.reward.rot_reversal = 0.02
