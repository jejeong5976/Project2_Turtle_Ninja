// rl_core.hpp — RL 지역 계획기 계산 (ROS 없음, 시험 대상). local_planner_node.cpp 가 씀
//
// 역할
//   - RlPath: 경로 점 · 호 길이 · 창 안 투영 (학습 swerve_follow.paths.PathBatch 와 같은 식)
//   - NpzPolicy: export.py 의 policy_<N>.npz (평균 · 표준편차 정규화 + MLP · ELU) 실행
//   - BoxTracker: 정밀 접근 중 스캔으로 박스 앞면을 재서 정차 목표를 박스 기준으로 고침
//   - 관측 · 명령: 학습 env (_compute_obs · _command) 와 같은 순서 · 정규화 · 한계
// 받는 것
//   - 정책 npz · 학습 cfg.json (env) · team_swerve.yaml 값, 스캔 점 · 자세 · 관절 값 (노드가 넣음)
// 주는 것
//   - 관측 벡터 (161), 몸 속도 명령 (vx, vy, wz), 고친 정차 목표
// note
//   - 파이썬 판 (scripts/local_planner_node.py · box_tracker.py · numpy_policy.py) 과 같은 식. sim (Isaac) 은 파이썬 판을 씀
//   - 라이다 칸 번호는 numpy round 와 같게 짝수 쪽 반올림 (nearbyint)

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace navigation::rl
{

struct Vec2
{
  double x = 0.0, y = 0.0;
};

struct Pose2
{
  double x = 0.0, y = 0.0, yaw = 0.0;
};

double wrap(double a);                                  // [−π, π) (파이썬 (a + π) % 2π − π 와 같음)
Pose2 compose(const Pose2 & T, const Pose2 & p);        // T · p (평면)
Vec2 apply(const Pose2 & T, const Vec2 & p);
Pose2 inverse(const Pose2 & T);

// 경로 (점 · 호 길이)
class RlPath
{
public:
  explicit RlPath(std::vector<Vec2> pts);
  double length() const {return L_;}
  const std::vector<Vec2> & points() const {return p_;}
  Vec2 point_at(double s) const;
  // 가장 가까운 발 (호 길이, 거리). s_prev 있으면 s_prev − back ~ s_prev + fwd 안으로 자름
  std::pair<double, double> project(const Vec2 & xy, std::optional<double> s_prev = std::nullopt,
    double back = 0.1, double fwd = 0.5) const;

private:
  std::vector<Vec2> p_;
  std::vector<double> s_;
  double L_ = 0.0;
};

// export.py npz 정책: x = (obs − mean) / (std + eps) → [W x + b → ELU] × (층 − 1) → W x + b
class NpzPolicy
{
public:
  explicit NpzPolicy(const std::string & path);         // 실패하면 std::runtime_error
  std::array<double, 3> operator()(const std::vector<float> & obs) const;
  int input_dim() const {return static_cast<int>(mean_.size());}

private:
  std::vector<double> mean_, std_;
  double eps_ = 0.0;
  struct Layer
  {
    int out = 0, in = 0;
    std::vector<double> W, b;                           // W 행 우선 (out × in)
  };
  std::vector<Layer> layers_;
};

// 박스 앞면 맞춤 (box_tracker.py 와 같음)
class BoxTracker
{
public:
  BoxTracker(Vec2 goal_xy, double goal_yaw, double box_dist = 0.2575, double half = 0.1025, double max_shift = 0.08,
    double max_yaw_deg = 15.0, double win = 0.08, int min_pts = 8, double freeze_dist = 0.10, int med_n = 5,
    double min_len = 0.06);
  Pose2 goal() const;                                   // 잰 박스 중심에서 box_dist, 박스 정면
  bool update(const std::vector<Vec2> & pts, Vec2 robot_xy);   // 고쳤으면 true
  // 상태 (노드 로그 · 직선 경로 검사)
  double yaw0, h, freeze, th = 0.0;
  Vec2 c0, c;
  int n_ok = 0, n_wall = 0;
  std::string last_why;

private:
  std::optional<std::array<double, 3>> measure(const std::vector<Vec2> & pts, Vec2 robot_xy);
  double dist_, max_shift_, max_yaw_, win_, min_len_;
  int min_pts_, med_n_;
  std::vector<std::array<double, 3>> hist_;
};

// 학습 cfg.json env 중 노드가 쓰는 값
struct EnvCfg
{
  double vmax = 0.2, wmax = 0.5, ctrl_dt = 0.01;
  int decimation = 5, path_obs_points = 16, lidar_rays = 36, history = 4;
  double path_obs_spacing = 0.1, lidar_max = 3.0, lidar_obs_clip = 2.0;
  double goal_hold_s = 0.3, goal_lin_vel = 0.02, goal_ang_vel = 0.05;
  double lidar_body_x = 0.17, lidar_body_y = 0.19;
  bool approach_limit = false, rot_limit = false;
  double approach_v0 = 0.02, approach_k = 0.5;
  double rot_clear_lo = 0.24, rot_clear_hi = 0.32, rot_wz_lo = 0.25;
  std::string swerve_yaml;
};
EnvCfg load_env_cfg(const std::string & cfg_json);      // {"env": {...}}

struct SwerveCfg
{
  std::vector<std::string> steer, drive;
  std::array<double, 4> rmin{}, rmax{};
  double max_wheel = 1.0;
};
SwerveCfg load_swerve(const std::string & yaml);

// 스캔 점 (몸 기준 xy) → 10° 칸 최솟값 (빈 칸 = lidar_max), 몸 상자 안 점 뺌
std::vector<double> lidar_bins(const std::vector<Vec2> & body_pts, const EnvCfg & c);

// 관측 한 장 (경로 16 점 · 목표 5 · 라이다 · 이력). hist = 최근 프레임 먼저, 비었으면 같은 프레임으로 채움
struct JointObs
{
  std::array<double, 4> q{}, w{};                       // 조향 위치 · 바퀴 속도
  std::array<double, 3> odom{};                         // 몸 속도 vx vy wz
};
std::vector<float> build_obs(const EnvCfg & c, const SwerveCfg & sw, const RlPath & path, const Pose2 & pose, double s,
  const Pose2 & goal, const std::vector<double> & scan, const JointObs & j, const std::array<double, 3> & last_cmd,
  std::vector<std::vector<double>> & hist);

// 행동 → 몸 속도 명령 (병진 크기 ≤ vmax · 목표 근처 상한 · 추가 상한 · 벽 근처 회전 상한)
//   extra_vlim ≤ 0 이면 안 씀, scan_min 없으면 (스캔 없음) 0 으로 봄
std::array<double, 3> command(const EnvCfg & c, std::array<double, 3> a, std::optional<double> goal_dist,
  double extra_vlim, std::optional<double> scan_min);

// 정밀 접근 직선 (pose → 목표) 을 몸 (body_half + 1 cm) 이 쓸고 갈 때 박스 아닌 점 수 · 처음 걸린 거리
//   박스 점 = 잰 앞면 평면 (로봇 쪽 +3 cm) 뒤 · 옆 반 변 + 8 cm 안
std::pair<int, double> straight_blocked(const Pose2 & pose, const Pose2 & goal, const BoxTracker & tr,
  const std::vector<Vec2> & pts, Vec2 body_half);

}  // namespace navigation::rl
