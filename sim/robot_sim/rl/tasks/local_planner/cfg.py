"""지역 계획기 (MPPI 대체) 학습 설정. 정책 = 전역 경로 · 목표 · 라이다 · 바퀴 상태 → 몸 속도 (vx, vy, wz) → 스워브 컨트롤러.
속도 · 가속 한계 = 실물 config/controllers.yaml swerve_controller (0.2 m/s · 0.5 rad/s, 0.5 m/s² · 2.0 rad/s²)."""
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1]


@dataclass
class EnvCfg:
    num_envs: int = 8192
    device: str = 'cuda'
    compile: bool = True
    seed: int = 1
    path_bank: str = str(RL_DIR / 'data' / 'path_bank.npz')       # 지도 경로만 씀 (합성 경로는 장애물 없음)
    dice_legs: str = str(RL_DIR / 'data' / 'local_planner' / 'dice_legs.npz')       # dice 정차점끼리 90 구간 (make_dice_legs.py)
    maps_dir: str = str(RL_DIR / 'data' / 'maps')
    swerve_yaml: str = str(RL_DIR / 'data' / 'team_swerve.yaml')  # 팀 로봇 (최종판) 기하 · 조향 범위
    # 조향 범위 덮어쓰기 [rad] 바퀴 4 개 (fl, fr, rl, rr). 빈 값 = swerve_yaml. 컨트롤러 · 플랜트 둘 다 씀 (실물 steer_ranges.yaml 값)
    steer_min: tuple = ()
    steer_max: tuple = ()
    split: int = 0                       # 경로 은행 split (0 학습 · 1 평가)
    dice_prob: float = 0.4               # 에피소드 중 dice 구간 비율 (시연 경기장)
    # 주기: 컨트롤러 · 플랜트 100 Hz, 정책 20 Hz
    ctrl_dt: float = 0.01
    decimation: int = 5
    # 실물 한계
    vmax: float = 0.2
    wmax: float = 0.5
    amax: float = 0.5
    awmax: float = 2.0
    # 관측
    path_obs_points: int = 16
    path_obs_spacing: float = 0.10
    history: int = 4
    lidar_rays: int = 36
    lidar_max: float = 3.0
    lidar_obs_clip: float = 2.0
    # 'ideal' = 몸 중심 광선, 'real' = 앞 · 뒤 원점 · 몸 가림 · scan_merger 몸 상자 · 칸 최솟값 (배포 /scan 과 같음)
    lidar_model: str = 'ideal'
    lidar_occlusion: str = str(RL_DIR / 'data' / 'local_planner_nav2' / 'lidar_occlusion.npz')   # local_planner_nav2/make_lidar_occlusion.py
    lidar_beam_stride: int = 2           # 360 빔 → 180 (2°), 10° 칸마다 라이다당 5 빔
    # 벽 가까우면 회전 속도 상한 (정책 뒤 고정 규칙, 배포 노드와 같은 식): 라이다 최솟값 ≥ rot_clear_hi 면 wmax, ≤ rot_clear_lo 면 rot_wz_lo, 사이 직선.
    #   빨리 돌며 달리면 차체 모서리 (0.212 m) 가 벽을 쓸고 감. 0.25 rad/s = 모서리 0.053 m/s → 지연 0.24 s 에 1.3 cm
    rot_limit: bool = False
    rot_clear_lo: float = 0.24
    rot_clear_hi: float = 0.32
    rot_wz_lo: float = 0.25
    # 목표 근처 속도 상한 (정책 뒤 고정 규칙, 배포 노드와 같은 식): 병진 ≤ approach_v0 + approach_k × 목표 거리 (추정 자세).
    #   도착 1–2 cm 에서 큰 명령을 이리저리 바꾸면 몸은 서 있고 바퀴만 돎 → 큰 명령 자체를 못 내게. 0.02 + 0.5 d = 2 cm 0.03 · 10 cm 0.07 · 36 cm 0.2 m/s
    approach_limit: bool = False
    approach_v0: float = 0.02
    approach_k: float = 0.5
    # 움직이며 출발 (local_dock): Nav2 중간 정차에서 서지 않고 넘겨받음 → 출발 때 이미 경로 첫 방향으로 v 로 가는 중.
    #   start_speed_prob 확률 · 경로 길이 ≥ start_speed_min_len 일 때, v ~ U(start_speed), 조향 · 바퀴 · 몸 속도 · 컨트롤러 출력 맞춤.
    #   명령 지연 버퍼는 0 (실물: Nav2 취소 → 0 속도 → 접근 노드 첫 명령까지 감속)
    start_speed_prob: float = 0.0
    start_speed: tuple = (0.05, 0.2)
    start_speed_min_len: float = 0.15
    # 회전하며 출발 (local_dock5): 넘김 순간 주행 정책이 아직 도는 중 (|wz| 최대 약 0.5 rad/s, 대부분 방향 오차 줄이는 쪽,
    #   병진 방향은 목표 방향에서 ±30° 안). 움직이며 출발한 env 에서 |wz| ~ U(start_wz), start_wz_toward 확률로 오차 줄이는 쪽 부호,
    #   병진 방향 = 경로 첫 방향 + U(±start_dir_noise_deg). 기본 (0, 0) · 0 = 끔
    start_wz: tuple = (0.0, 0.0)
    start_wz_toward: float = 0.9
    start_dir_noise_deg: float = 0.0
    # 박스 밀림 (local_dock): 접근 구간 (box_id ≥ 0) 에서 box_shift_prob 확률로 박스를 ±box_shift_xy · ±box_shift_yaw_deg 밀고
    #   목표도 박스에 붙여 같이 옮김. 충돌 · 여유 · 라이다 = 밀린 박스, 지도 · 출발 = 그대로 (실물: 지도는 늦게 갱신),
    #   경로 = 출발 → 옮긴 목표 직선 (배포 노드: planner 실패 시 직선 대체와 같음)
    box_dynamic: bool = False
    box_shift_prob: float = 0.7
    box_shift_xy: float = 0.06
    box_shift_yaw_deg: float = 10.0
    # box_shift_disc: 밀기 = 반지름 box_shift_xy 원 안 고르게 (기본 = ±box_shift_xy 정사각형).
    #   box_goal_check: 옮긴 목표 자세의 차체가 벽과 겹치면 안 밂 (기본 끔, 6 cm 밀기에선 거의 안 생김)
    box_shift_disc: bool = False
    box_goal_check: bool = False
    # 지도 ↔ 실제 어긋남: map_boxes json {지도 이름: [[cx, cy, 회전], …]} 의 박스를 래스터에서 지우고 해석식으로.
    #   mbox_shift_prob 확률 에피소드에서 박스마다 ±mbox_shift_xy · ±mbox_shift_yaw_deg 옮김 (경로 · 지도는 래스터 그대로 = 지도 자리).
    #   출발 · 목표에서 mbox_exempt 안 박스는 안 옮김 (목표 박스는 정밀 접근이 재서 맞춤). 벽 · 출발 · 목표 외곽과 겹치면 그 env 는 안 옮김
    #   실물 박스는 도면에서 10–20 cm 벗어날 수 있고, 시연 중 사람이 밀 수 있음
    map_boxes: str = ''
    mbox_half: float = 0.1025
    mbox_erase_margin: float = 0.035     # 래스터 박스 칸 지울 여유 (라이다 칸 번짐 1–2.5 cm)
    mbox_shift_prob: float = 0.0
    mbox_shift_xy: float = 0.12
    mbox_shift_yaw_deg: float = 10.0
    mbox_exempt: float = 0.5
    # 추가 구간: 다른 지도의 정차점 구간 (형식 = dice_legs, 지도 이름은 파일 안 names). extra_prob 확률로 이 구간, 그다음 dice_prob 로 dice 구간
    extra_legs: str = ''
    extra_prob: float = 0.0
    lidar_body_half: tuple = (0.17, 0.19)  # scan_merger body_half (ros2_ws/src/navigation/config/navigation.yaml) — 'real' 은 가림표 값 씀
    proj_back: float = 0.10
    proj_fwd: float = 0.50
    # 외곽: 충돌 판정 ±0.15 (실제 차체 + 1 cm 여유 정도), 여유 벌점은 Nav2 외곽 ±0.16 기준
    collide_half: float = 0.15
    footprint_half: float = 0.16
    # 정지 구역: 외곽 (footprint_half) 에서 stop_zone 안에 지도 장애물이 들어오면 충돌로 끝냄 (사람 뺌).
    #   = 실물 collision_monitor PolygonStop ±0.18 (0.16 + 0.02): 들어가면 모든 명령 0 → 교착.
    #   목표 stop_zone_goal_exempt 안 (dock 넘김 0.40) · 출발 stop_zone_start_exempt 안 (도킹 자리 출발 = 노드가 영역 끔) 은 뺌. 0 = 끔
    stop_zone: float = 0.0
    stop_zone_goal_exempt: float = 0.40
    stop_zone_start_exempt: float = 0.30
    footprint_per_side: int = 12
    # 에피소드
    init_pos_std: float = 0.01
    dice_start_yaw_noise_deg: float = 5.0
    random_steer_prob: float = 0.2
    timeout_slack_s: float = 15.0
    # 성공 판정 (커리큘럼 단계별 [0, 1, 2]) = 추정 자세 기준 (실물 goal checker 와 같음)
    goal_pos_tol: tuple = (0.05, 0.03, 0.02)
    goal_yaw_tol_deg: tuple = (6.0, 3.0, 2.0)
    goal_lin_vel: float = 0.02
    goal_ang_vel: float = 0.05
    goal_hold_s: float = 0.3
    fail_dev: float = 0.6                # 경로에서 이만큼 벗어나면 실패
    # 사람 (원기둥, 경로 옆에 놓아 일부 막음)
    people_prob: tuple = (0.0, 0.3, 0.5)
    max_people: int = 2
    people_r: tuple = (0.12, 0.18)
    people_offset: tuple = (0.28, 0.45)  # 경로에서 중심까지
    # 커리큘럼
    curriculum: bool = True
    curriculum_start: int = 0
    curriculum_success: float = 0.85
    curriculum_window: int = 2000


@dataclass
class DRCfg:
    steer_rate_levels: tuple = (2.4, 4.8)                # [rad/s] 실물 profile_vel 200 = 4.8, 느린 경우도 섞음
    steer_rate_scale: tuple = (0.8, 1.1)
    tau_steer: tuple = (0.05, 0.15)
    tau_wheel: tuple = (0.02, 0.06)
    tau_body: tuple = (0.02, 0.08)
    slip: tuple = (0.90, 1.0)
    # 조향 돌림 (θ̇ · 바퀴 축 오프셋) 이 몸을 미는 비율, env 마다 [lo, hi]. 1 = 접지 그대로 밀림 (컨트롤러 보정과 같음).
    #   1 로만 배우면 이 몫을 상쇄하는 회전 명령을 배워, 밀림이 다른 곳에서 끝 yaw 가 한쪽으로 치우침 → 범위로 섞음
    scrub: tuple = (1.0, 1.0)
    cmd_delay_ticks: tuple = (0, 3)
    obs_delay_ticks: tuple = (0, 4)
    noise_steer: float = 0.005
    noise_lin_vel: float = 0.005
    noise_ang_vel: float = 0.02
    lidar_noise: float = 0.01            # [m]
    lidar_dropout: float = 0.02          # 이 비율로 최대 거리
    map_err_pos: float = 0.015           # 위치 추정 오차 ± [m]
    map_err_yaw_deg: float = 1.0
    map_jump_every_s: float = 2.0
    map_jump_prob: float = 0.3
    map_jump_pos: float = 0.01
    map_jump_yaw_deg: float = 0.5
    # yaw 흐름: 에피소드마다 ±yaw_drift_dps [°/s] 로 추정 yaw 가 흘러 ±yaw_drift_max_deg 까지 (자이로 치우침)
    yaw_drift_dps: float = 0.0
    yaw_drift_max_deg: float = 3.0
    # 큰 위치 튐: 초당 big_jump_rate 확률로 추정 자세가 pos · yaw 만큼 튀고 big_jump_decay_s 로 줄어듦, 그때 yaw 흐름은 0 으로 (SLAM 큰 보정)
    big_jump_rate: float = 0.0
    big_jump_pos: tuple = (0.05, 0.10)
    big_jump_yaw_deg: tuple = (2.0, 5.0)
    big_jump_decay_s: float = 3.0
    # odom 오차 누적 → SLAM 되돌림. 추정 xy 오차 acc_err 가 쌓이다가 snap 때 한 번에 줄어듦 (yaw 는 IMU 라 안 쌓음)
    #   odom_slip_err: 엔코더 순기구학 (odom) − 참 몸 속도 (slip 반영) 차이를 적분 = 실물 /odom 이 미끄럼만큼 틀어지는 것
    #   fight_*: 바퀴끼리 안 맞을 때 (최소제곱 잔차 fit_res) 참 몸이 엔코더에 안 보이게 밀림 (큰 조향 회전 때 몸 비틀림)
    #            밀림 속도 = k × fit_res, 방향은 에피소드마다 무작위. odom 은 못 봄 → acc_err 로 쌓임
    #   snap_*: SLAM 보정 주기 (에피소드마다 snap_every_s 안 값 × 0.5–1.5 마다), 보정 뒤 남는 비율 snap_keep
    odom_slip_err: bool = False
    fight_prob: float = 0.0              # 에피소드 중 바퀴 싸움 밀림이 있는 비율
    fight_k: tuple = (0.0, 0.0)          # 밀림 속도 / fit_res
    fight_yaw_k: tuple = (0.0, 0.0)      # 몸 비틀림 [rad/s] / fit_res [m/s], 방향 에피소드마다 무작위. 참 yaw 만 (IMU 가 봄 → 추정 yaw 오차 아님)
    snap_every_s: tuple = (0.0, 0.0)     # 0 = 보정 없음 (odom_slip_err · fight 를 켜면 반드시 줌)
    snap_keep: tuple = (0.0, 0.2)
    # 추정 자세가 튄 스텝 (큰 튐 · snap) 은 진행 · 퍼텐셜 보상 0 (튐으로 생긴 가짜 진행 막음)
    jump_reward_mask: bool = False


@dataclass
class RewardCfg:
    progress: float = 5.0                # × Δs [m]
    time: float = 0.01                   # 정책 스텝마다
    goal_pos: float = 2.0                # 퍼텐셜 −(goal_pos·min(거리, goal_zone) + goal_yaw·|dyaw|) 의 변화
    goal_yaw: float = 0.5
    goal_zone: float = 0.5
    # 목표 방향 퍼텐셜을 목표 근처에서만 (거리 ≥ goal_yaw_far 면 0, ≤ goal_yaw_near 면 1, 사이 직선). 0 이하 = 늘 켬
    #   늘 켜면 출발하자마자 최대 각속도로 돌며 달림 → 복도 벽 충돌
    goal_yaw_far: float = 0.0
    goal_yaw_near: float = 0.5
    clearance: float = 0.05              # × clip((clear_margin − 여유) / clear_margin, 0, 1)
    clear_margin: float = 0.04
    deviation: float = 0.02              # × clip((경로 거리 − 0.1) / 0.1, 0, 4)²
    deviation_person_dist: float = 0.0   # 사람 가장자리가 몸 중심에서 이 거리 안이면 경로 이탈 벌점 0. 0 = 끔
    action_rate: float = 0.01
    steer_travel: float = 0.02           # × 조향 이동량 [rad] (0.002 면 조향 이동이 기준 추종기의 1.5 배)
    # 목표 아닌 곳에서 멈춤 벌점 (정책 스텝마다): 목표 stall_goal_dist 밖 · 이동 속도 < stall_speed (제자리 회전도 멈춤) ·
    #   출발 stall_grace_s 뒤 · 사람 stall_person_dist 밖일 때. 0 = 끔
    stall: float = 0.0
    stall_goal_dist: float = 0.3
    stall_speed: float = 0.02
    stall_grace_s: float = 0.5
    stall_person_dist: float = 0.6
    # 행동 범위 벌점: −w × Σ max(|a| − 1, 0)² (샘플 행동, 잘리기 전). 0 = 끔. 없으면 평균 행동이 ±1 밖으로 나가 bang-bang
    action_bound: float = 0.0
    # 회전 비용: −w × (1 − 목표 방향 가중) × |wz 명령| / wmax. 목표 근처 (방향 맞출 때) 는 안 셈. 0 = 끔. 없으면 주행 내내 돎
    rot_cost: float = 0.0
    # 회전 상한 넘김 벌점: −w × max(|요청 wz| − 상한, 0) / wmax (rot_limit 켰을 때). 0 = 끔.
    #   상한에 잘려도 결과가 같으면 줄일 이유가 없어 늘 상한에 걸림
    rot_cap_excess: float = 0.0
    # 목표 가까이 회전 벌점: −w × [목표 rot_near_dist 안] × |wz 명령| / wmax. 0 = 끔 (local_dock: 방향은 먼 곳에서, 마지막 0.3 m 는 직선 접근)
    rot_near: float = 0.0
    rot_near_dist: float = 0.3
    # 목표 근처 조향 이동 벌점 배수: 목표 steer_near_dist 안에서 steer_travel × steer_near_mult (1 = 끔).
    #   목표 앞에서 이동 방향을 자주 바꾸면 몸은 조금 가는데 바퀴가 크게 쓸림
    steer_near_mult: float = 1.0
    steer_near_dist: float = 0.3
    # 도착 근처 명령 크기 벌점: −w × [목표 hold_dist 안] × ((병진 명령 / vmax)² + (wz 명령 / wmax)²). 0 = 끔
    hold_cmd: float = 0.0
    hold_dist: float = 0.03
    # 헛회전 벌점: −w × max(|Δ몸 yaw| − (목표 방향 오차가 줄어든 양), 0) [rad] (참 자세, 정책 스텝마다). 0 = 끔.
    #   목표 방향으로 도는 회전은 안 셈 → 필요한 만큼만 돌고 왕복 · 갔다 오는 회전만 벌줌 (odom yaw 오차 누적 대비)
    rot_waste: float = 0.0
    # 헛회전 벌점 빼는 거리: 라이다 최솟값이 이보다 가까우면 (벽 · 사람 피하는 중) 헛회전 벌점 안 줌. 0 = 늘 줌
    #   헛회전 벌점이 피하기 회전까지 막으면 충돌이 늘어서 뺌. 0.25 m 안 = 목표 0.3 m 밖 스텝의 약 12 % (0.35 면 대부분이라 사실상 끔)
    rot_waste_clear: float = 0.0
    # 회전 방향 반전 벌점: −w × [wz 명령 부호가 바뀜, 앞 · 지금 둘 다 |wz| ≥ rot_reversal_min]. 0 = 끔
    rot_reversal: float = 0.0
    rot_reversal_min: float = 0.05
    success: float = 20.0
    collision: float = 20.0
    lost: float = 10.0


@dataclass
class TrainCfg:
    max_iterations: int = 3000
    num_steps_per_env: int = 64
    save_interval: int = 200
    experiment_name: str = 'local_planner'
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
                'gamma': 0.99, 'lam': 0.95, 'value_loss_coef': 1.0, 'entropy_coef': 0.003,
                'learning_rate': 3e-4, 'max_grad_norm': 1.0, 'schedule': 'adaptive', 'desired_kl': 0.01,
            },
            'actor': {'class_name': 'MLPModel', 'hidden_dims': [256, 256, 128], 'activation': 'elu', 'obs_normalization': True,
                      'distribution_cfg': {'class_name': 'GaussianDistribution', 'init_std': 0.5}},
            'critic': {'class_name': 'MLPModel', 'hidden_dims': [512, 256, 128], 'activation': 'elu', 'obs_normalization': True},
        }


@dataclass
class Cfg:
    env: EnvCfg = field(default_factory=EnvCfg)
    dr: DRCfg = field(default_factory=DRCfg)
    reward: RewardCfg = field(default_factory=RewardCfg)
    train: TrainCfg = field(default_factory=TrainCfg)
