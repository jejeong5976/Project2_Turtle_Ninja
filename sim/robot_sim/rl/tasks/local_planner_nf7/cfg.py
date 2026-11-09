"""주행 정책 nf7 (local_planner_nf7): nf6 (헛회전 · 회전 반전 벌점) + 실물에서 본 차이 랜덤화 (2026-11-09).

바뀐 것 (env · dr 새 항목은 기본 꺼짐, 여기서만 켬):
  - arena5 구간 (env.extra_legs · extra_prob 0.3): 실물 SLAM 지도 arena5 + 정차 면 19 개 · 출발점 쌍 380 구간 (실제 planner_server 경로)
      데이터 = make_arena5_data.py (지도 → rl/data/maps/maze_arena5_map, 구간 · 박스 → rl/data/local_planner_nf7). dice 구간 0.4 → 0.3
  - 지도 ↔ 실제 어긋남 (env.map_boxes · mbox_shift_prob 0.5): 박스 · 장소 9 개를 해석식으로, 에피소드 절반은 박스마다 ±12 cm · ±10° 옮김
      (경로 · 지도는 옛 자리 그대로). 출발 · 목표 0.5 m 안 박스는 안 옮김 (목표 박스는 정밀 접근이 재고 맞춤)
      실물: arena5 박스가 도면에서 8–19 cm, 11-07 실패 주원인
  - yaw 흐름 (dr.yaw_drift_dps 0.2, 최대 3°): 실물 자이로 치우침 추정 −0.05 ↔ −0.21 °/s
  - 큰 위치 튐 (dr.big_jump_rate 0.003 /s ≈ 20 s 구간당 6 %, 5–10 cm · 2–5°, 3 s 에 걸쳐 줄어듦): 실물 pose_guard 'SLAM 보정 9–13 cm'
그 밖 (nf6 회전 벌점 · stop_zone · vmax 0.255 · 1500 iteration) 은 nf6 그대로
비교 지표: /extra/success · /extra/collision (arena5 구간), /dice/*, /ep/rot_*"""
from dataclasses import dataclass, field

from local_planner.cfg import RL_DIR
from local_planner_nf6.cfg import Cfg as _NF6Cfg
from local_planner_nf6.cfg import TrainCfg as _NF6TrainCfg

DATA = RL_DIR / 'data' / 'local_planner_nf7'


@dataclass
class TrainCfg(_NF6TrainCfg):
    experiment_name: str = 'local_planner_nf7'


@dataclass
class Cfg(_NF6Cfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        e, dr = self.env, self.dr
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
