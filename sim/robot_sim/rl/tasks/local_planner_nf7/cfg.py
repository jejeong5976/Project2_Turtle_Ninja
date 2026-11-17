"""주행 정책 nf7 (local_planner_nf7, 배포 config/rl/nav_nf7): nf5 + 헛회전 · 회전 반전 벌점 + 실물과 시뮬 차이 랜덤화.

바뀐 것 (env · dr 새 항목은 기본 꺼짐, 여기서만 켬):
  - reward.rot_waste 2.0: 스텝마다 몸 yaw 변화 중 목표 방향 오차를 줄이지 않은 만큼 [rad] × 2 (참 자세). 스워브라 이동 중 몸을
      돌릴 필요가 없음 → 필요한 회전 하한 = 출발 yaw ↔ 정차 yaw 차이뿐 (odom yaw 오차 누적 대비)
  - reward.rot_reversal 0.02: wz 명령 부호가 바뀌면 (앞 · 지금 둘 다 |wz| ≥ 0.05 rad/s) 한 번에 −0.02
  - 헛회전 벌점은 라이다 0.25 m 안 (장애물 피하는 중) 이면 안 줌 (reward.rot_waste_clear)
  - arena5 구간 (env.extra_legs · extra_prob 0.3): 실물 SLAM 지도 arena5 + 정차 면 19 개 · 출발점 쌍 380 구간 (실제 planner_server 경로)
      데이터 = make_arena5_data.py (지도 → rl/data/maps/maze_arena5_map, 구간 · 박스 → rl/data/local_planner_nf7). dice 구간 0.3
  - 지도 ↔ 실제 어긋남 (env.map_boxes · mbox_shift_prob 0.5): 박스 · 장소 9 개를 해석식으로, 에피소드 절반은 박스마다 ±12 cm · ±10° 옮김
      (경로 · 지도는 지도 자리 그대로). 출발 · 목표 0.5 m 안 박스는 안 옮김 (목표 박스는 정밀 접근이 재서 맞춤)
  - yaw 흐름 (dr.yaw_drift_dps 0.2, 최대 3°): 자이로 치우침
  - 큰 위치 튐 (dr.big_jump_rate 0.003 /s, 5–10 cm · 2–5°, 3 s 에 걸쳐 줄어듦): SLAM 큰 보정
  - max_iterations 1500 (모자라면 --resume)
지표: /extra/success · /extra/collision (arena5 구간), /dice/*, /ep/rot_total_rad · /ep/rot_waste_rad · /ep/rot_reversals"""
from dataclasses import dataclass, field

from local_planner.cfg import RL_DIR
from local_planner_nf5.cfg import Cfg as _NF5Cfg
from local_planner_nf5.cfg import TrainCfg as _NF5TrainCfg

DATA = RL_DIR / 'data' / 'local_planner_nf7'


@dataclass
class TrainCfg(_NF5TrainCfg):
    experiment_name: str = 'local_planner_nf7'
    max_iterations: int = 1500


@dataclass
class Cfg(_NF5Cfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        e, dr = self.env, self.dr
        self.reward.rot_waste = 2.0
        self.reward.rot_reversal = 0.02
        e.extra_legs = str(DATA / 'arena5_legs.npz')
        e.extra_prob = 0.3
        e.dice_prob = 0.3
        e.map_boxes = str(DATA / 'map_boxes.json')
        e.mbox_shift_prob = 0.5
        e.mbox_shift_xy = 0.12
        e.mbox_shift_yaw_deg = 10.0
        dr.yaw_drift_dps = 0.2
        dr.yaw_drift_max_deg = 3.0
        dr.big_jump_rate = 0.003
        self.reward.rot_waste_clear = 0.25
