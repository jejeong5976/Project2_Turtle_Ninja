"""정밀 접근 정책 (local_dock): 박스 · 장소 면 앞 0.03–0.8 m 에서 2 cm · 2° 로 붙기. MPPI · nf 정책 어느 쪽 뒤에도 붙이는 마지막 단계.

env · 관측 (161) · 라이다 (real) · 플랜트 · DR = local_planner_nav2 그대로. 바뀐 것:
  - 구간 = make_dock_legs.py (정차 면 22 곳 × 출발 분포, 직선 경로), dice 구간만 씀 (dice_prob 1)
  - 속도 상한 0.2 m/s (2026-11-06, 옛 0.08) · 0.3 rad/s, 목표 근처는 approach_limit 0.02 + 0.5 d (10 cm 0.07 · 3 cm 0.035)
  - 움직이며 출발 50 % (0.05–0.2 m/s, 경로 첫 방향): planning 이 Nav2 중간 정차 전에 넘겨줌 (서지 않음)
  - 방향 보상 늘 켬 (goal_yaw_far 0) + 가중 0.5 → 1.0: 방향을 먼저 · 먼 곳에서 맞춤
  - 목표 0.3 m 안 회전 벌점 (rot_near 0.05): 마지막 0.3 m 는 직선 접근, 2° 안 미세 보정만
  - 멈춤 벌점 · 회전 비용 끔: 출발점에서 제자리로 방향부터 맞추는 것은 허용 (박스에서 0.3 m 밖)
  - (dock2) 박스 밀림 · 짧은 구간 · 틈 2–4 cm (make_dock_legs.py v2, env box_dynamic)
배포: local_planner/deploy/local_planner_node.py 그대로 (--action_name dock_to_pose 로 nf · MPPI 의 navigate_to_pose 와 안 겹치게)"""
from dataclasses import dataclass, field
from pathlib import Path

from local_planner_nav2.cfg import Cfg as _NavCfg
from local_planner_nav2.cfg import TrainCfg as _NavTrainCfg

HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1]


@dataclass
class TrainCfg(_NavTrainCfg):
    experiment_name: str = 'local_dock'
    max_iterations: int = 1500


@dataclass
class Cfg(_NavCfg):
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()                # 실제 NavFn 경로 은행 (이름 순서용) · 실물형 라이다 · 회전 상한 · 범위 벌점 · std 상한
        e, r = self.env, self.reward
        # dock2 (2026-11-06): 출발 모드 pre · fix · broad, 박스 면 틈 2–4 cm, 박스 밀림 ±6 cm · ±10° (목표 같이 옮김, 지도 · 출발은 원래 자리)
        e.dice_legs = str(RL_DIR / 'data' / 'local_dock' / 'dock_legs_v2.npz')
        e.box_dynamic = True
        # 정차 0.2575 (박스 · 장소 둘 다, 2026-11-06): 차체 앞 끝 0.145 ↔ 박스 면 1 cm
        e.collide_half = 0.145                 # 충돌 판정 = 실제 차체 (바퀴 포함). Nav2 외곽 ±0.16 은 박스와 겹침
        d = self.dr                            # correct 모드 목표 = 라이다로 잰 박스 기준 (로봇 상대) → 지도 위치 오차가 대부분 빠짐
        d.map_err_pos, d.map_err_yaw_deg = 0.005, 0.3
        d.map_jump_pos, d.map_jump_yaw_deg = 0.003, 0.2
        e.dice_prob = 1.0                      # 접근 구간만
        e.vmax, e.wmax = 0.2, 0.3               # 2026-11-06: 0.08 → 0.2 (중간 정차 없이 Nav2 속도로 넘겨받음). 박스 앞은 approach_limit (0.02 + 0.5 d) 가 줄임
        e.start_speed_prob = 0.5               # 절반은 움직이며 출발 (Nav2 에서 서지 않고 넘겨받기)
        e.start_speed = (0.05, 0.2)
        e.timeout_slack_s = 10.0
        e.dice_start_yaw_noise_deg = 3.0
        e.people_prob = (0.0, 0.0, 0.1)        # 박스 바로 앞이라 사람은 드묾 (collision_monitor 가 먼저 세움)
        r.goal_yaw_far = 0.0                   # 방향 보상 늘 켬
        r.goal_yaw = 1.0
        r.rot_near, r.rot_near_dist = 0.05, 0.3
        r.stall = 0.0
        r.rot_cost = 0.0
        r.progress = 10.0                      # 경로가 짧아 (중앙 0.15 m) 진행 보상 키움
