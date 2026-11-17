// nudge.hpp — 칸 안 삐뚤어진 퍽 툭 건드리기 (팀 RL nudge_n1b) 의 순수 계산 (test/test_nudge.cpp)
// 정책 (config/rl/nudge_n1b.md): 30 Hz, 관측 86 · 행동 6 (a[0:5] 관절 1–5 속도 비율, a[5] 안 씀), 목표 관절 += a × 0.70 rad/s × 1/30 s
// 여기: ① 칸 안 퍽 자세 (측정 · 손으로 준 값) ② 관측 86 만들기 ③ 한 걸음 (목표 적분 · 안전 확인 · 끝 판정). ROS 는 control_master
//
// 좌표: 모두 우리 base_link (x 앞 · y 왼쪽 · z 위). 관측의 손끝 절대 높이 (12–14 의 z) 만 시뮬 좌표로 tip_z_offset 을 더함
//   (시뮬 base 원점이 우리 base_link 보다 약 33–35 mm 아래, xy 는 같음). 상대 값 (21–32 등) 은 영향 없음

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/grasp_guard.hpp"   // SeatCellRef

namespace manipulation
{

constexpr int kNudgeObsDim = 86;
constexpr int kNudgeActDim = 6;
constexpr int kNudgeSimSteps = 120;   // 시뮬 에피소드 4 s × 30 Hz (남은 시간 관측의 기준)

struct NudgePuck
{
  Eigen::Vector3d centre{Eigen::Vector3d::Zero()};   // 퍽 중심 [m, base_link]
  Eigen::Vector3d up{Eigen::Vector3d::UnitZ()};      // 퍽 위쪽 축 (단위)
  double face_rad{0.0};                              // 퍽 yaw − 칸 yaw (원기둥 0), 관측은 sin · cos(4 ×)
  std::string source;                                // seat (넣은 뒤 보기 측정) · manual (손으로 줌) · assumed (칸 + 기울기만)
  double tilt_deg() const {return std::acos(std::clamp(up.normalized().z(), -1.0, 1.0)) * 180.0 / M_PI;}
};

// 칸 안 바른 자리 = 칸 중심 위 바로 앉은 퍽 중심 (윗면 − 퍽 높이 / 2)
Eigen::Vector3d nudge_target(const SeatCellRef & c, double puck_h);

// 손으로 준 퍽 자세 (벤치 시험 · 측정 없을 때). dx · dy = 퍽 중심 xy − 칸 중심 [mm, base_link 축]
//   dir_deg: 위쪽 축이 기운 방향 [°, base_link] — NaN 이면 (dx, dy) 쪽 (둘 다 0 이면 칸 x 축 = +x 쪽)
//   dz_mm: 퍽 중심 − 바른 자리 높이 — NaN 이면 한 아래 모서리가 바닥에 닿아 기운 꼴: half_w · sin θ − h/2 · (1 − cos θ)
NudgePuck nudge_puck_manual(const SeatCellRef & c, double puck_h, double half_w, double tilt_deg, double face_deg,
  double dx_mm, double dy_mm, double dir_deg = std::nan(""), double dz_mm = std::nan(""));
// 퍽이 손끝을 따라 내려감 (실물엔 정책 도는 동안 퍽 측정이 없음 → 처음 값 p0 에서 고침, nudge_puck_follow):
//   손끝이 퍽 중심에서 수평 reach_m 안이고 퍽 윗면 아래로 내려가면 퍽 중심 높이 = 손끝 − h/2 · cos(기울기) (바른 자리 밑으론 안 감, 올라가지 않음).
//   높이가 처음 높이 대비 비율 r 로 줄면 기울기 · 중심 xy 어긋남도 × r (눌려 앉으며 바로 섬 — 가정). 처음 높이가 바른 자리 이하면 안 바꿈. 바꿨으면 true
bool nudge_puck_follow(NudgePuck & p, const NudgePuck & p0, const Eigen::Vector3d & tip, const SeatCellRef & c, double puck_h, double reach_m);

// 넣은 뒤 보기 측정 (control_master ~/seat_pose 와 같은 값, 칸 축): 윗면 태그 중심 − 칸 중심 (x_mm, y_mm), 윗면 − 바로 앉은 윗면 (height_mm),
//   R_cell = 태그 자세 (칸 축, z = 윗면 법선), face_deg = 면 (네모, 원기둥은 0)
NudgePuck nudge_puck_from_seat(const SeatCellRef & c, double puck_h, double x_mm, double y_mm, double height_mm,
  const Eigen::Matrix3d & R_cell, double face_deg);

struct NudgeObsIn
{
  std::array<double, 5> q{}, qd{}, home{};    // 관절 (팀 명령 rad, /joint_states) · 속도 [rad/s] · poses.home
  double grip_open{0.0};                      // 그리퍼 열림 비율 0 닫힘 ~ 1 열림
  double grip_force_n{0.0};                   // 그리퍼 힘 [N] (모름 = 0)
  Eigen::Vector3d tip{Eigen::Vector3d::Zero()}, wrist_z{Eigen::Vector3d::UnitZ()}, wrist_y{Eigen::Vector3d::UnitY()};   // base_link
  NudgePuck puck;
  SeatCellRef cell;
  double puck_h{0.060};
  std::array<double, 12> occupied{};          // S1–S12 찬 칸 (1/0)
  int task_index{-1};                         // 작업 칸 0..11 (S1 = 0)
  std::array<double, 6> prev_action{};
  int step{0};                                // 지금까지 한 걸음 수 (남은 시간 = 1 − step / 120)
  double tip_z_offset{0.0};                   // 관측 손끝 z 에만 더함 (시뮬 좌표, 머리말)
};
std::vector<double> nudge_obs(const NudgeObsIn & in);

struct NudgeConfig
{
  double speed_rad_s{0.70};       // 시뮬 최대 관절 속도
  double speed_scale{0.5};        // 실물 속도 = speed_rad_s × 이것
  double dt_s{1.0 / 30.0};
  int max_steps{120};
  std::array<double, 5> lo{}, hi{};   // 목표 관절 한계 (팀 rad, 여유 뺀 값)
  Eigen::Vector2d cell_xy{Eigen::Vector2d::Zero()};
  double top_z{0.1925};               // 바로 앉은 윗면 [m, base_link]
  double box_xy_m{0.040};             // 손끝이 칸 중심에서 수평으로 이 안
  double below_top_m{0.005};          // 손끝이 바로 앉은 윗면 − 이것 아래로 안 감
  double above_top_m{0.080};          // 손끝이 윗면 + 이것 위로 안 감
  double current_a{0.30};             // J2–J4 전류 변화 (시작 대비) 문턱 [A], ≤ 0 = 안 봄
  int current_hold{2};                // 연속 이만큼 걸음 넘으면 멈춤
  double max_lag_rad{0.15};           // 목표 − 실측 이보다 크면 멈춤 (막힘), ≤ 0 = 안 봄
  double idle_action{0.10};           // |a[0:5]| 최대가 이보다 작으면 쉼
  int idle_steps{10};                 // 쉼이 이만큼 이어지고 · 손끝이 윗면 + idle_clear_m 위면 끝 (정책이 다 했다고 봄)
  int min_steps{10};
  double idle_clear_m{0.015};
};

// LEFT: 손끝이 윗면 + idle_clear_m 위에서 다음 목표가 상자 (수평 · 위) 밖 = 정책이 일을 끝내고 칸을 떠남 (시뮬 학습: 끝나면 HOME 쪽) → 보통 끝
enum class NudgeEnd : uint8_t { NONE, IDLE, MAX_STEPS, SAFETY, LEFT };
const char * nudge_end_name(NudgeEnd e);

struct NudgeSense
{
  std::array<double, 5> q{};                 // 실측 관절 (팀 rad)
  std::array<double, 5> effort{};            // 실측 전류 [A] (NaN = 모름)
};

struct NudgeStepOut
{
  std::array<double, 5> target{};   // 보낼 목표 (SAFETY 면 보내지 말 것 — 실측으로 멈춤)
  NudgeEnd end{NudgeEnd::NONE};
  std::string why;
};

// 손끝 FK (팀 rad → base_link 손끝). control_master = arm_grip(team_to_model_q) + z 변환, 시험 = 가짜
using NudgeTipFn = std::function<Eigen::Vector3d(const std::array<double, 5> &)>;

class NudgeRunner
{
public:
  // q0: 시작 실측 (= 첫 목표), effort0: 시작 전류 (멈춘 채 잰 평균, NaN 이면 그 관절 전류 안 봄)
  void start(const NudgeConfig & c, const std::array<double, 5> & q0, const std::array<double, 5> & effort0, NudgeTipFn tip);
  // 실측 안전 (걸음 전): 관절 한계 · 손끝 상자 · 전류 · 따라가기 오차. "" = 괜찮음
  std::string check(const NudgeSense & s);
  // 실측 확인 + 행동 → 다음 목표. SAFETY 면 target = 실측 (멈춤)
  NudgeStepOut step(const NudgeSense & s, const std::vector<double> & action);
  // 목표 자세 하나가 한계 · 상자 안인가 ("" = 괜찮음)
  std::string target_ok(const std::array<double, 5> & t) const;
  int steps() const {return steps_;}
  const std::array<double, 6> & prev_action() const {return prev_;}
  const std::array<double, 5> & target() const {return target_;}
  double min_tip_z() const {return min_tip_z_;}

private:
  NudgeConfig c_;
  NudgeTipFn tip_;
  std::array<double, 5> target_{}, effort0_{};
  std::array<double, 6> prev_{};
  int steps_{0}, idle_{0}, cur_over_{0};
  double min_tip_z_{1e9};
};

// 도중 멈춤 (취소 · 하드웨어 이상) 에 실측 자세로 붙잡을 때: 그 자세를 지나온 길 (home_retreat trace, 명령 점만 있음) 끝에 더함
//   → 팔이 마지막 명령보다 home_retreat_snap_rad (0.06) 넘게 뒤처져도 HOME 의 거꾸로 가기가 지금 자세에서 시작
void nudge_hold_trace(std::vector<std::vector<double>> & trace, const std::array<double, 5> & q_meas, std::size_t keep_before);

// ArmTask NUDGE 받기 (control_master on_arm_goal → build_plan): "" = 받음, 아니면 거절 이유 (robot_cli · 로그에 그대로)
struct NudgeGateIn
{
  bool enabled{false};       // nudge_enable
  bool carrying{false};      // 퍽을 쥔 채
  bool slot_known{false};
  bool model_ok{false};      // 팔 기구학
  bool policy_ok{false};
  std::string policy_why;    // 정책 못 읽은 이유
};
std::string nudge_gate(const NudgeGateIn & in);

// 손으로 준 퍽 자세 (ArmTask.puck_id = "nudge:<기울기°>,<면°>,<dx mm>,<dy mm>[,<기운 방향°>[,<dz mm>]]", robot_cli nudge 인자)
struct NudgeManual
{
  bool given{false};
  double tilt_deg{0.0}, face_deg{0.0}, dx_mm{0.0}, dy_mm{0.0};
  double dir_deg{std::nan("")}, dz_mm{std::nan("")};
};
inline constexpr const char * kNudgeManualTag = "nudge:";
// "" · 태그 없음 = given false (true 돌려줌). 태그가 있는데 숫자가 틀리면 false + why
bool parse_nudge_manual(const std::string & s, NudgeManual & out, std::string & why);

}  // namespace manipulation
