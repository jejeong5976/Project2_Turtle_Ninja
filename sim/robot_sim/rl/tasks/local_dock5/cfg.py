"""정밀 접근 정책 dock5 (local_dock5, 배포 config/rl/dock5): dock3 + 회전하며 넘겨받기 + 지도 ↔ 실제 박스 크게 어긋남 (±18 cm · ±15°)
+ 박스 측정기 흉내 + 새 관측.

왜: 박스 없이 매핑한 지도면 박스 자리는 도면 좌표뿐이고 실제 박스는 도면에서 10–20 cm 벗어날 수 있음.
    배포 rl_dock 은 FaceTrack (지도 자리 ±0.20 m · 벽 거르기 · 처음 3 스캔 확인) 으로 박스를 재므로 목표가 출발 뒤 늦게 · 크게 바뀜.
바뀐 것 (env = DockEnv, env.py):
  - 회전하며 넘겨받기: 넘김 순간 주행 정책은 아직 도는 중 (|wz| 최대 약 0.5 rad/s, 병진 방향이 목표에서 ±30°)
      env.start_wz (0, 0.55) · start_wz_toward 0.9 · start_dir_noise_deg 30, start_speed_prob 0.8 · start_speed (0.12, 0.255)
  - reward.rot_waste 2.0 · rot_reversal 0.02: 방향 오차를 줄이지 않는 회전 · wz 부호 바꿈 벌점
  - 박스 밀림 반지름 0.18 m 원 안 · ±15°, 확률 0.85, 옮긴 목표 차체가 벽과 겹치면 안 밂 (box_goal_check)
  - 박스 측정기 흉내: 출발 뒤 est_delay 동안 목표 = 지도 정차점, 그 뒤 = 참 목표 + 고정 치우침 + 측정 잡음,
    est_every 스텝마다 갱신 (빠짐 est_drop), 목표 est_freeze 안이면 고정. 목표가 3 mm · 0.2° 넘게 바뀌면 경로 = 지금 자리 → 새 목표 직선
    (배포 노드와 같음). 목표 근처 속도 상한도 이 목표 기준. 성공 · 보상은 참 목표
  - 관측 추가 (뒤에 붙임): 측정 표시 1 (0 = 아직 지도 목표) + 앞쪽 촘촘한 라이다 41 칸 (±60°, 3°, 1.0 m 자름)
  - 비대칭 critic 에 측정 오차 (지금 목표 − 참 목표) · 측정 표시 추가
배포: rl_dock 노드 = cfg.json env 의 fine_scan_half · est_flag 를 읽어 같은 관측 (rl_core build_obs), --box_search 0.20"""
from dataclasses import dataclass, field

from local_dock3.cfg import Cfg as _Dock3Cfg
from local_dock3.cfg import TrainCfg as _Dock3TrainCfg
from local_planner.cfg import EnvCfg as _EnvCfg


@dataclass
class DockEnvCfg(_EnvCfg):
    # 앞쪽 촘촘한 라이다: 몸 기준 각 k × fine_scan_step_deg (k = −fine_scan_half … fine_scan_half) 칸 최솟값, fine_scan_clip 으로 자름
    fine_scan_half: int = 20
    fine_scan_step_deg: float = 3.0
    fine_scan_clip: float = 1.0
    est_flag: bool = True                 # 관측에 측정 표시 (배포 노드가 cfg 로 앎)
    # 박스 측정기 흉내 (배포 FaceTrack): 처음 측정까지 지연 = est_fast_prob 확률로 est_delay_fast, 아니면 est_delay_slow [s]
    est_fast_prob: float = 0.5
    est_delay_fast: tuple = (0.15, 0.35)  # 연속 3 스캔 확인 (스캔 약 10 Hz)
    est_delay_slow: tuple = (0.35, 1.0)   # 처음 몇 스캔은 벽 · 가림으로 못 찾음 (배포는 10 스캔 못 찾으면 실패)
    est_bias_xy: float = 0.003            # 에피소드마다 고정 치우침 σ [m] (박스 맞춤 목표 오차 90 % 약 5 mm)
    est_bias_yaw_deg: float = 0.5
    est_noise_xy: float = 0.004           # 측정마다 잡음 σ
    est_noise_yaw_deg: float = 0.6
    est_every: int = 2                    # 정책 스텝 (20 Hz) 마다 → 10 Hz
    est_drop: float = 0.2                 # 측정 빠짐 (가림 · 확인 실패)
    est_gain: float = 0.5                 # 새 측정 반영 비율 (배포 = 최근 5 개 중앙값)
    est_freeze: float = 0.10              # 지금 목표까지 이 안이면 고정 (배포 FaceTrack freeze_dist)
    est_replan_xy: float = 0.003          # 목표가 이만큼 넘게 바뀌면 경로 다시 (배포 노드와 같음)
    est_replan_yaw_deg: float = 0.2


@dataclass
class TrainCfg(_Dock3TrainCfg):
    experiment_name: str = 'local_dock5'
    max_iterations: int = 2500            # 관측 · 박스 밀림이 커져 dock3 (2000) 보다 조금 더. 8192 env 반복 약 5 s


@dataclass
class Cfg(_Dock3Cfg):
    env: DockEnvCfg = field(default_factory=DockEnvCfg)
    train: TrainCfg = field(default_factory=TrainCfg)

    def __post_init__(self):
        super().__post_init__()
        e, r = self.env, self.reward
        e.start_speed_prob = 0.8
        e.start_speed = (0.12, 0.255)
        e.start_wz = (0.0, 0.55)
        e.start_wz_toward = 0.9
        e.start_dir_noise_deg = 30.0
        r.rot_waste = 2.0
        r.rot_reversal = 0.02
        e.box_shift_prob = 0.85
        e.box_shift_xy = 0.18                 # 원 반지름 (배포 찾기 0.20 보다 조금 안쪽)
        e.box_shift_yaw_deg = 15.0
        e.box_shift_disc = True
        e.box_goal_check = True
        e.lidar_beam_stride = 2               # 2° 빔 (1° 면 반복 시간 2 배). 앞쪽 3° 칸은 두 라이다로 채움
        e.timeout_slack_s = 12.0              # 목표가 뒤늦게 옮겨지면 길어짐
