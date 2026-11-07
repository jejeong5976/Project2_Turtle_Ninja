"""스워브 경로 추종 학습 설정 (환경 · 도메인 랜덤화 · 보상 · PPO). 설계 근거 = rl/tasks/swerve_follow/README.md"""
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1]


@dataclass
class EnvCfg:
    num_envs: int = 16384                # 작은 연산 위주라 env 를 늘려도 스텝 시간이 거의 그대로 (4096: 16 ms · 16384: 21 ms)
    device: str = 'cuda'
    compile: bool = True                 # 컨트롤러 + 플랜트 한 틱 torch.compile (첫 스텝 컴파일 약 1 분)
    seed: int = 1
    path_bank: str = str(RL_DIR / 'data' / 'path_bank.npz')
    split: int = 0                       # 0 학습 · 1 평가 (경로 은행 split)
    random_rotation: bool = True         # 경로를 시작점 기준 무작위 회전 (False = 지도 방향 그대로, 그림용)
    sequential_paths: bool = False       # 평가: 첫 에피소드 경로 = 은행 순서 (env i → 경로 i mod N), 정책 · 기준선 같은 조건
    # 주기: 컨트롤러 · 플랜트 100 Hz, 정책 20 Hz (= Nav2 controller_frequency)
    ctrl_dt: float = 0.01
    decimation: int = 5
    # 행동: residual = 기준 추종기 + 잔차, direct = 정책이 몸 속도 전부
    action_mode: str = 'residual'
    residual_lin: float = 0.05           # [m/s] 병진 잔차 벡터 노름 한계 (Codex 안 0.03 과 비교 실험 대상)
    residual_ang: float = 0.3            # [rad/s]
    # 속도 한계 프로필 (max_linear, max_linear_accel). 각속도 한계 = 1.675 × v / 0.2612, 각가속 = 6 × a (선속도에 묶음)
    limit_profiles: tuple = ((0.10, 0.2), (0.13, 0.2))
    # 경로 · 관측
    path_obs_points: int = 16
    path_obs_spacing: float = 0.10
    history: int = 4                     # 고유 상태 프레임 수 (현재 + 이전 3, 정책 주기)
    proj_back: float = 0.10              # 최근접 탐색 창: 이전 진행 위치 뒤 · 앞 [m]
    proj_fwd: float = 0.30
    # 기준 추종기 (scripts/measure_dice_routes.py 와 같은 식)
    follower_lookahead: float = 0.20
    follower_yaw_gain: float = 1.5
    # 에피소드 · 초기 상태 (커리큘럼 단계별 [0, 1, 2])
    init_pos_std: tuple = (0.02, 0.035, 0.05)
    init_yaw_range_deg: tuple = (15.0, 45.0, 180.0)   # 목표 yaw 기준 ±
    random_steer_prob: float = 0.2       # 조향 상태를 범위 안 무작위로 (나머지 = 직전 에피소드 끝 상태 이어받음)
    timeout_slack_s: float = 15.0        # 시간 제한 = 2 L / v_max + 이 값 (학습 편의용 = truncation)
    # 성공 · 실패
    goal_pos_tol: float = 0.05
    goal_yaw_tol: float = 0.10
    goal_lin_vel: float = 0.02
    goal_ang_vel: float = 0.05
    goal_hold_s: float = 0.5
    fail_lat_hard: float = 0.25
    fail_lat_soft: float = 0.15
    fail_lat_soft_s: float = 0.5
    # 커리큘럼: 최근 성공률이 넘으면 다음 단계
    curriculum: bool = True
    curriculum_start: int = 0
    curriculum_success: float = 0.9
    curriculum_window: int = 2000        # 에피소드


@dataclass
class DRCfg:
    steer_rate_levels: tuple = (0.96, 2.44, 4.80)     # [rad/s] 실물 미측정 → 3 조건 층화
    steer_rate_scale: tuple = (0.8, 1.2)
    tau_steer: tuple = (0.05, 0.15)      # [s] 조향 1차 지연 (속도 한계와 같이)
    tau_wheel: tuple = (0.02, 0.06)
    tau_body: tuple = (0.02, 0.08)       # 몸 관성 · 미끄럼 근사 (world 좌표 속도 필터)
    slip: tuple = (0.90, 1.0)            # 바퀴별 실제 접지 구름 비율 (odom 엔코더에는 안 걸림)
    cmd_delay_ticks: tuple = (0, 3)      # 컨트롤러 틱 (10 ms) 0–30 ms
    obs_delay_ticks: tuple = (0, 4)      # 0–40 ms
    noise_steer: float = 0.005           # [rad]
    noise_lin_vel: float = 0.005         # [m/s]
    noise_ang_vel: float = 0.02          # [rad/s]
    map_err_pos: float = 0.02            # 위치 추정 오차 (자세에 직접 더함) ± [m], 이 범위 안에서만 움직임
    map_err_yaw_deg: float = 1.0
    map_jump_every_s: float = 2.0        # 이 주기마다 확률 p 로 계단 변화 (AMCL 보정)
    map_jump_prob: float = 0.3
    map_jump_pos: float = 0.01
    map_jump_yaw_deg: float = 0.5


@dataclass
class RewardCfg:
    progress: float = 5.0                # × Δs [m] (참 자세 기준, 부호 있음)
    progress_jump: float = 0.05          # 한 정책 스텝 |Δs| 가 v_max·dt 보다 이만큼 넘으면 0 처리
    lateral: float = 0.02                # × min((e/0.05)², 9)
    heading: float = 0.02                # × (1 − cos dyaw) × clip(1 − 남은 길이/0.3, 0, 1)
    heading_zone: float = 0.30
    action_rate: float = 0.01            # × ‖a_t − a_{t−1}‖²
    time: float = 0.005
    success: float = 20.0
    failure: float = 20.0
    branch_flip: float = 0.0             # 지표만 (가중 0)
    steer_travel: float = 0.0


@dataclass
class TrainCfg:
    max_iterations: int = 1500
    num_steps_per_env: int = 64
    save_interval: int = 100
    experiment_name: str = 'swerve_follow'
    run_name: str = ''

    def runner_dict(self) -> dict:
        return {
            'num_steps_per_env': self.num_steps_per_env,
            'save_interval': self.save_interval,
            'experiment_name': self.experiment_name,
            'run_name': self.run_name,
            'logger': 'tensorboard',
            'obs_groups': {'actor': ['policy'], 'critic': ['policy', 'privileged']},
            'algorithm': {
                'class_name': 'PPO', 'num_learning_epochs': 5, 'num_mini_batches': 4, 'clip_param': 0.2,
                'gamma': 0.995, 'lam': 0.95, 'value_loss_coef': 1.0, 'entropy_coef': 0.001,
                'learning_rate': 3e-4, 'max_grad_norm': 1.0, 'schedule': 'adaptive', 'desired_kl': 0.01,
            },
            'actor': {'class_name': 'MLPModel', 'hidden_dims': [128, 128], 'activation': 'elu', 'obs_normalization': True,
                      'distribution_cfg': {'class_name': 'GaussianDistribution', 'init_std': 0.3}},
            'critic': {'class_name': 'MLPModel', 'hidden_dims': [256, 256], 'activation': 'elu', 'obs_normalization': True},
        }


@dataclass
class Cfg:
    env: EnvCfg = field(default_factory=EnvCfg)
    dr: DRCfg = field(default_factory=DRCfg)
    reward: RewardCfg = field(default_factory=RewardCfg)
    train: TrainCfg = field(default_factory=TrainCfg)
