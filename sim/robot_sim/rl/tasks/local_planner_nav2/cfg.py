"""local_planner 와 같은 환경 · 보상 · PPO, 경로만 실제 Nav2 planner_server 경로 (make_nav2_paths.py).
PATHS 이름 = navfn (GridBased) · smac2d · smac2d_far · imitation (local_planner 와 같은 흉내, 같은 쌍 → 비교 기준).
train.py --task local_planner_nav2 --paths <이름> 으로 고름 (기본 navfn = 지금 배포 설정과 같은 경로).
라이다 = 'real' (앞 · 뒤 원점 · 자기 몸 가림 · scan_merger 몸 상자 · 칸 최솟값)."""
from dataclasses import dataclass, field
from pathlib import Path

from local_planner.cfg import Cfg as _LPCfg
from local_planner.cfg import TrainCfg as _LPTrainCfg

HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1]
DATA = RL_DIR / 'data' / 'local_planner_nav2'
PATHS = ('navfn', 'smac2d', 'smac2d_far', 'imitation')


@dataclass
class TrainCfg(_LPTrainCfg):
    experiment_name: str = 'local_planner_nav2'
    # 행동 표준편차 상한 (튜터 PPO 글 8.5 행동 클리핑 포화): lp2 는 std 0.24 → 16 으로 커져 샘플 행동이 거의 ±1 로 잘림
    #   (잘린 행동은 같은 행동이라 std 를 줄일 이유가 사라지고 엔트로피 보너스만 남음). log 매개변수 + [0.05, 0.8] 로 막음
    std_range: tuple = (0.05, 0.8)

    def runner_dict(self) -> dict:
        d = super().runner_dict()
        d['actor']['distribution_cfg'] |= {'std_type': 'log', 'std_range': tuple(self.std_range)}
        return d


@dataclass
class Cfg(_LPCfg):
    train: TrainCfg = field(default_factory=TrainCfg)
    paths: str = 'navfn'

    def __post_init__(self):
        self.use_paths(self.paths)
        self.env.lidar_model = 'real'          # 배포 /scan (scan_merger) 과 같은 라이다 (make_lidar_occlusion.py 가림표)
        self.env.rot_limit = True              # 벽 가까우면 회전 상한 (0.32 m ↑ 0.5 · 0.24 m ↓ 0.25 rad/s), 배포 노드도 cfg.json 으로 같게
        self.reward.goal_yaw_far = 2.0         # 목표 방향 퍼텐셜은 2.0 m 안에서만, 0.8 m 안 전부 (2026-11-04 A/B: 1.5 · 0.5 면 0.5 m 안에서 몰아 돌아
        self.reward.goal_yaw_near = 0.8        #   박스 앞 충돌 0.5 → 1.1 % → 박스 0.3 m 들어가기 전에 다 돌게)
        self.reward.action_bound = 0.05        # 잘리기 전 행동 |a| > 1 벌점 (nf1 bang-bang: vy · wz 평균 93 % 가 ±1 밖)
        self.reward.rot_cap_excess = 0.1       # 회전 상한 넘긴 요청 벌점 (nf1: 상한 걸린 스텝 62 %, 잘려도 결과 같아 안 줄임). 0.25 rad/s 넘기면 스텝마다 −0.05 = 초당 −1.0
        self.reward.rot_cost = 0.02            # 목표 근처 아닐 때 회전 비용 (nf1: 주행 내내 0.35 rad/s, 조향 이동 +63 %)
        # nf3 (2026-11-06): nf2 도착 직전 흔들림 (1–2 cm 에서 최고 속도 명령을 이리저리 → 바퀴만 80° · 20 cm 앞 접근 방향 바꿔 165°)
        self.env.approach_limit = True         # 병진 ≤ 0.02 + 0.5 × 목표 거리 (배포 노드도 cfg.json 으로 같게)
        self.reward.steer_near_mult = 10.0     # 목표 0.3 m 안 조향 이동 벌점 10 배
        self.reward.hold_cmd = 0.05            # 목표 3 cm 안 명령 크기 벌점
        self.reward.stall = 0.05               # 목표 0.3 m 밖 멈춤 (이동 < 0.02 m/s, 제자리 회전 포함) 스텝마다 −0.05 = 초당 −1.0. 출발 0.5 s · 사람 0.6 m 안은 뺌

    def use_paths(self, name: str):
        assert name in PATHS, f'paths = {PATHS} 중 하나'
        self.paths = name
        self.env.path_bank = str(DATA / f'path_bank_{name}.npz')
        self.env.dice_legs = str(DATA / f'dice_legs_{name}.npz')
