"""스워브 속도 추종 학습 설정. 플랜트 · 컨트롤러 · DR 은 swerve_follow 것 그대로. 설계 근거 = rl/tasks/swerve_track/README.md"""
from dataclasses import dataclass, field

from swerve_follow.cfg import RL_DIR, DRCfg


@dataclass
class EnvCfg:
    num_envs: int = 16384
    device: str = 'cuda'
    compile: bool = True
    seed: int = 1
    cmd_bank: str = str(RL_DIR / 'data' / 'swerve_track' / 'cmd_bank.npz')
    split: int = 0                       # 0 학습 · 1 평가
    sources: tuple = (0, 1)              # 0 follower (기준 추종기 명령) · 1 synth (합성)
    sequential: bool = False             # 평가: env i → 열 i mod N
    ctrl_dt: float = 0.01
    decimation: int = 5                  # 정책 20 Hz (= /cmd_vel 주기)
    residual_lin: float = 0.03           # [m/s] 병진 잔차 노름 한계 (Codex 권고값부터)
    residual_ang: float = 0.15           # [rad/s]
    stop_lin: float = 0.005              # 명령이 이보다 작으면 (각속도도) 잔차 0 = 정지 명령은 그대로 통과
    stop_ang: float = 0.02
    preview: int = 0                     # 앞으로 올 명령 몇 개를 관측에 넣을지 (0.25 s 간격). 실물에선 MPPI 예측 궤적이 필요 → 기본 0
    preview_every: int = 5
    history: int = 3                     # 고유 상태 프레임 (현재 + 이전 2)
    random_steer_prob: float = 0.2
    change_window: int = 10              # 지표: 0.5 s 전 명령과 차이로 '변화 구간' 판정
    change_lin: float = 0.03
    change_ang: float = 0.10
    steer_end_zone: float = 0.10         # [rad] 조향 범위 끝 근처


@dataclass
class RewardCfg:
    lin: float = 1.0                     # × exp(−‖e_xy‖² / σ_lin²), 참 몸 속도 기준
    sigma_lin: float = 0.05
    ang: float = 0.5                     # × exp(−e_wz² / σ_ang²)
    sigma_ang: float = 0.15
    steer_travel: float = 0.2            # × 한 정책 스텝 조향 이동량 합 [rad] (기준선 약 0.05 rad/스텝)
    steer_end: float = 0.05              # × 바퀴 평균 clip((zone − 끝까지 거리)/zone, 0, 1)
    action: float = 0.01                 # × ‖a‖²
    action_rate: float = 0.01            # × ‖Δa‖²


@dataclass
class TrainCfg:
    max_iterations: int = 1000
    num_steps_per_env: int = 64
    save_interval: int = 100
    experiment_name: str = 'swerve_track'
    run_name: str = ''
    init_std: float = 0.3
    min_std: float = 0.02                # 탐색이 일찍 죽는 것 막음 (swerve_follow 본학습 1: std 0.005 까지 줄어듦)

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
                'gamma': 0.99, 'lam': 0.95, 'value_loss_coef': 1.0, 'entropy_coef': 0.002,
                'learning_rate': 3e-4, 'max_grad_norm': 1.0, 'schedule': 'adaptive', 'desired_kl': 0.01,
            },
            'actor': {'class_name': 'MLPModel', 'hidden_dims': [128, 128], 'activation': 'elu', 'obs_normalization': True,
                      'distribution_cfg': {'class_name': 'GaussianDistribution', 'init_std': self.init_std,
                                           'std_range': (self.min_std, 1.0)}},
            'critic': {'class_name': 'MLPModel', 'hidden_dims': [256, 256], 'activation': 'elu', 'obs_normalization': True},
        }


@dataclass
class Cfg:
    env: EnvCfg = field(default_factory=EnvCfg)
    dr: DRCfg = field(default_factory=DRCfg)
    reward: RewardCfg = field(default_factory=RewardCfg)
    train: TrainCfg = field(default_factory=TrainCfg)
