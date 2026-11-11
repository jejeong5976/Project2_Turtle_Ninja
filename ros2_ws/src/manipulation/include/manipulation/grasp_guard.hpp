// grasp_guard.hpp — 쥐기 · 넣기 확인 판단 (순수 C++, test/test_grasp_guard.cpp). 쓰는 곳: control_master.cpp 의 grasp_guard 묶음
// =====================================================================
// 11-11 시연 전 (11-10 밤 실물: 비뚤게 쥔 퍽 · 칸에 덜 들어간 퍽을 사람이 손으로 밀어 넣음). 모두 기본값은 지금 동작 그대로 (로그 또는 1 번만 다시)
//   ① 쥔 폭: PICK 닫기 뒤 그리퍼 실측 → 손가락 사이 [mm] (모델 2 · (inner0 + q) + 보정) ↔ 퍽 종류 폭 (원기둥 30 · 네모 34.8)
//   ② 손 안 퍽: 손목 카메라 쥔 퍽 태그 (wrist_link) → 접근축에서 옆으로 [mm] · 윗면 기울기 [°] · 쥔 깊이 [mm]
//      표본은 지난 PICK 의 목표 퍽 ID · 쥔 뒤 ~ 놓기 전만 (다른 퍽 · 놓은 뒤 태그는 버림)
//   ③ 넣기 걸림: 칸 안 마지막 내려가기 동안 J2–J4 추종 오차 · 전류 · 손끝 높이 모자람이 내려가기 직전보다 커졌나
//   ④ 넣은 뒤: 놓은 퍽 윗면 태그 (base_link) 높이 ↔ 칸 바닥 + 60 mm, 칸 중심 ↔ 컵 안 여유
//   ⑤ 다시 쥐기 동작: 그 자리에서 (열기 → 위 → 쥐기 → 닫기) · 들기를 되돌려 내려놓고 다시 쥐기 · 칸 안에서 들어 옮기기

#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm.hpp"
#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_path.hpp"

namespace manipulation
{

constexpr double kGuardNaN = std::numeric_limits<double>::quiet_NaN();

// ============================================================================ ① 쥔 폭
enum class PuckShape {Unknown, Cylinder, Square};
const char * puck_shape_name(PuckShape s);

struct GripWidthConfig
{
  double inner0_m{0.0091};    // q = 0 일 때 손가락 안쪽 면 반 간격 (grasp_select FingerTip inner0 · robot_cli gripper_gap)
  double gap_bias_mm{7.4};    // 모델 간격에 더해 실제로: 실측 네모 (34.8) 쥔 채 q 약 0.0046 → 모델 27.4 mm (park_loose_grip_m 주석)
  double cylinder_mm{30.0};
  double square_mm{34.8};
  double tol_mm{5.0};         // |간격 − 기대| 가 이보다 크면 잘못 쥠
};

// 그리퍼 실측 q [m] → 손가락 사이 [mm]. NaN 이면 NaN
double finger_gap_mm(double q_m, const GripWidthConfig & c);

struct GripWidthVerdict
{
  bool judged{false};       // 실측 · 설정이 숫자라 판단함
  bool bad{false};          // 기대 폭에서 tol 넘게 다름
  double gap_mm{kGuardNaN};
  double expect_mm{kGuardNaN};   // 모양 모름이면 NaN (30 ~ 34.8 사이면 맞음)
  double err_mm{kGuardNaN};      // 간격 − 기대 (모양 모름: 범위 밖으로 나간 만큼, 안이면 0)
  std::string text;
};
GripWidthVerdict judge_grip_width(double q_m, PuckShape shape, const GripWidthConfig & c);

// ============================================================================ ② 손 안 퍽 (쥔 퍽 태그)
// 쥔 퍽 태그 표본을 받을지. 받으면 nullptr, 아니면 버리는 이유
//   frame: 표본 좌표 (rack_tag_node 는 wrist_link), want_id: 지난 PICK (RETRIEVE) 의 목표 퍽 ID ("" = 모름 → 다 버림),
//   after_s: 쥔 뒤 (닫기 · 기준점 도착), before_s: 놓기 (안 놓았으면 무한)
const char * held_sample_reject(const std::string & id, const std::string & frame, double stamp_s, const std::string & want_id, double after_s,
  double before_s);

struct HeldVerdict
{
  bool seen{false};
  int frames{0};
  double lateral_mm{kGuardNaN};   // 퍽 윗면 태그 중심 ↔ 손끝 접근축 거리
  double close_mm{kGuardNaN};     // 그중 닫힘축 방향 (손가락이 가운데로 미는 방향, 보통 작음)
  double width_mm{kGuardNaN};     // 손가락 폭 방향 (손가락 따라 미끄러진 양)
  double depth_mm{kGuardNaN};     // 태그 ↔ 손끝 접근축 거리 = 실제 쥔 깊이 추정 (카메라 · 처짐 치우침 포함)
  double face_deg{kGuardNaN};     // 윗면 법선 ↔ 접근축
  bool bad{false};                // lateral > max_mm 또는 face > max_face_deg
  std::string text;
};
// p · n: 쥔 퍽 태그 중심 · 면 방향 (wrist_link, 중앙값), tip · app · close: 손끝 · 접근축 · 닫힘축 (wrist_link = 팔 모델 link5 좌표)
HeldVerdict judge_held_in_hand(bool seen, const Eigen::Vector3d & p, const Eigen::Vector3d & n, int frames,
  const Eigen::Vector3d & tip, const Eigen::Vector3d & app, const Eigen::Vector3d & close, double max_mm, double max_face_deg);

// ============================================================================ ③ 넣기 걸림
// 팀 명령 rad (관절 1..5) ↔ 모델 관절 (arm_state · cell_shift 와 같은 틱 변환)
Q5 team_to_model_q(const ArmModel & m, const std::vector<double> & team_rad);
std::vector<double> model_to_team(const ArmModel & m, const Q5 & q);
// 시간표 t 의 명령 (점 사이 직선, 앞은 첫 점 · 뒤는 끝 점)
std::vector<double> command_at(const std::vector<TrajPoint> & pts, double t);
// 마지막 내려가기 시작 시각: 명령 손끝이 끝 점보다 zone_m 이상 높은 마지막 점의 시각 (없으면 0)
double descent_start_time(const ArmModel & m, const std::vector<TrajPoint> & pts, double zone_m);

struct ContactConfig
{
  double base_s{0.3};        // 내려가기 직전 기준 구간 [s]
  double err_deg{1.5};       // 끝 (멈춘 뒤) J2–J4 추종 오차 변화 문턱 [°]
  double current_a{0.30};    // 내려가는 동안 J2–J4 전류 변화 최대 문턱 [A] (opencr effort = 전류)
  double height_mm{4.0};     // 끝 손끝 높이 모자람 (실측 − 명령) 변화 문턱 [mm]
};

struct ContactVerdict
{
  bool judged{false};
  bool suspect{false};
  int n_base{0}, n_eval{0}, n_end{0};
  std::array<double, 3> err_end_deg{{kGuardNaN, kGuardNaN, kGuardNaN}};   // J2 · J3 · J4 끝 − 기준 (명령 − 실측)
  std::array<double, 3> cur_peak_a{{kGuardNaN, kGuardNaN, kGuardNaN}};    // 내려가는 동안 |전류 − 기준| 최대
  double height_mm{kGuardNaN};   // 끝 (실측 − 명령 손끝 z) − 기준. + = 명령보다 덜 내려감 (걸림)
  std::string text;              // 숫자 한 줄
};

// 한 팔 동작 (칸 안으로 놓는 점까지) 동안 관절 실측을 모아 판단. 시각은 ROS 초
class ContactWatch
{
public:
  // pts: 보낸 시간표, t0: 보낸 시각, t_desc: 마지막 내려가기 시작 (시간표 기준), attempt: 다시 넣기 번호 (0 = 처음)
  void start(const std::vector<TrajPoint> & pts, double t0, double t_desc, int attempt);
  void stop() {active_ = false;}
  bool active() const {return active_;}
  int attempt() const {return attempt_;}
  const std::vector<TrajPoint> & points() const {return pts_;}
  // 관절 실측 (관절 1..5 팀 rad · 전류 A, 없으면 NaN)
  void feed(double stamp_s, const std::array<double, 5> & q, const std::array<double, 5> & cur);
  // m 이 있으면 손끝 높이도 (없으면 NaN)
  ContactVerdict judge(const ContactConfig & c, const ArmModel * m) const;
  std::size_t samples() const {return s_.size();}

private:
  struct Sample {double t; std::array<double, 5> q, cur;};
  bool active_{false};
  int attempt_{0};
  std::vector<TrajPoint> pts_;
  double t0_{0.0}, t_desc_{0.0}, hold_from_{0.0}, t_end_{0.0};
  std::vector<Sample> s_;
};

// 명령 자세 (팀 rad) 의 손끝을 base 방향 dp [m] 만큼 옮긴 자세 (접근축 · 닫힘축 유지, 감쇠 최소제곱 — cell_shift 와 같은 무게).
//   손끝이 0.5 mm 안으로 안 맞거나 관절 범위 (틱) 밖이면 false
bool tip_offset_command(const ArmModel & m, const std::vector<double> & cmd, const Eigen::Vector3d & dp_m, std::vector<double> & out);

// ============================================================================ ④ 넣은 뒤 (착좌)
enum class SeatState {UNKNOWN, SEATED, NOT_SEATED};
const char * seat_state_name(SeatState s);
struct SeatConfig
{
  double floor_z{0.1325};    // 칸 바닥 [m, base_link] (config/rack_cells.yaml floor_z)
  double puck_h{0.060};
  double tol_mm{6.0};        // 윗면 높이 ± (카메라 · 팔 자세 오차 포함)
  double cup_x{0.044}, cup_y{0.047};   // 컵 안 폭 (base_link x · y)
  double slack_mm{2.0};      // 컵 여유에 더하는 측정 여유
  int min_samples{3};
};
struct SeatSample
{
  double stamp{0.0};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};   // 퍽 윗면 태그 중심 [m, base_link]
  bool has_z{false};                             // false = 높이를 가정해 잰 것 (rack_puck) → xy 만
};
struct SeatVerdict
{
  SeatState state{SeatState::UNKNOWN};
  int n3{0}, n2{0};
  Eigen::Vector3d rel_mm{kGuardNaN, kGuardNaN, kGuardNaN};   // 윗면 − (칸 중심, 바닥 + 60 mm)
  std::string text;
};
// cell_xy: 칸 바닥 중심 [m, base_link], puck_w: 퍽 폭 [m] (네모 0.0348, 원기둥 0.030)
SeatVerdict judge_seat(const std::vector<SeatSample> & samples, const Eigen::Vector2d & cell_xy, double puck_w, const SeatConfig & c);

// ============================================================================ ⑤ 다시 쥐기 동작
// PICK 경로의 퍽 쥐는 닫기 하나 (control_master set_grasp_checks 와 같은 순서 · 같은 고르기)
struct RegripPoint
{
  std::string close_name, up_name;
  std::vector<double> q_close, q_up;      // 쥐는 점 · 다음 점 명령 [팀 rad]
  double close_speed_dps{0.0}, up_speed_dps{0.0};
  double open_m{kGuardNaN};               // 이 닫기 전 마지막으로 연 폭 (open · release)
  bool retry_ok{false};                   // 다음 점이 곧게 위 (이름 *up) + 연 폭 앎 → 그 자리에서 다시 쥐기 가능
  std::vector<PathNode> after;            // 닫기 뒤 경로 끝까지 (쥔 채 들기)
  int tries{0};                           // 이 닫기를 다시 쥔 횟수
};
// 퍽을 쥐는 닫기 (grip close, 가며 아님, 받침 꽂기 · 밀기 전 빈손 닫기 park_press_turn · park_push_over 는 뺌) 마다 경로 순서
std::vector<RegripPoint> regrip_points(const ArmPath & path, const GripperValues & grip);
// 그 자리에서 다시 쥐기: [그리퍼 열기 (open_m), 팔 쥐는 점 → 위 → 쥐는 점 (+ 그리퍼 앞 멈춤), 그리퍼 닫기]. retry_ok 아니면 빈 목록
std::vector<ArmAction> regrip_in_place(const RegripPoint & r, const PathTiming & t, const GripperValues & grip);
// 들기를 되돌려 내려놓고 다시 쥐기 (PICK 끝 = 들어 올린 자세에서): after 가 [*up, *lift] 일 때만, 아니면 빈 목록
//   [팔 들기 → 위 → 쥐는 점 (내려놓음), 열기, 팔 위 → 쥐는 점, 닫기, 팔 위 → 들기]
bool put_back_ok(const RegripPoint & r);
std::vector<ArmAction> put_back_regrip(const RegripPoint & r, const PathTiming & t, const GripperValues & grip);
// 칸 안에서 걸렸을 때 다시 넣기: 끝 자세 cmd_end 에서 곧게 lift_m 올림 → 수평 d 옮김 → lift_m 내림 (+ 그리퍼 앞 멈춤). IK 안 되면 빈 동작 (points 빔)
ArmAction contact_retry_action(const ArmModel & m, const std::vector<double> & cmd_end, double lift_m, const Eigen::Vector2d & d,
  double speed_dps, const PathTiming & t, std::string & why);

// ============================================================================ ⑥ 넣기 걸림 보정 (store_recover, 11-11)
// 명령 자세에서 손끝을 곧게 위로 step_m 씩 (tip_offset_command 를 이어서, 접근축 · 닫힘축 유지) 최대 lift_m 까지 올린 자세.
//   관절 범위 (J4 멈춤 등) · IK 로 더 못 올리면 거기까지. 손끝 옆 어긋남 1 mm · 접근축 2° 안. 돌려줌 = 올린 높이 [m] (0 = 못 올림, out 그대로 cmd)
//   11-11 계산: store_S9 놓는 점에서 수직 손은 30 mm 까지 (J4 2430 틱)
double tip_lift_max(const ArmModel & m, const std::vector<double> & cmd, double lift_m, double step_m, std::vector<double> & out);
// 칸 경로에서 퍽을 놓는 첫 점 (grip release · open) 번호. 없으면 nodes.size()
std::size_t store_release_index(const std::vector<PathNode> & nodes);
// 칸 안 놓는 점 nodes[release] 에서 퍽을 쥔 채 온 길로 nodes[0] (박스 위 기준점 · 집은 자리) 까지 (retrace_nodes: 고리 빼고 · 지나온 구간 속도 ·
//   그리퍼 · 멈춤 없음). 결과 [0] = 놓는 점 (출발), 끝 = nodes[0]. 놓는 점 바로 위 점이 lift_m 보다 낮으면 (칸 안 곧은 구간이 짧음)
//   놓는 점에서 곧게 올린 점 (recover_lift, lift_m 까지 · 관절 범위 안에서 되는 만큼, 그 구간 속도 = 내려온 속도) 을 먼저 넣고, 그보다 낮고
//   옆으로 15 mm 안 (같은 곧은 구간) 인 점은 건너뜀. 곧게 올린 높이가 바로 위 기록 점보다 낮으면 올리기 없이 온 길 그대로 (note 에 이유).
//   note = 올린 높이 한 줄 (로그). 온 길이 안 되면 빈 목록 (why)
std::vector<PathNode> store_recover_out_nodes(const ArmModel & m, const std::vector<PathNode> & nodes, std::size_t release, double lift_m,
  std::string & why, std::string & note);
// 결과 글 토큰 (미션이 읽음): "STORE_RETURNED_TO_BOX x y" (박스에 내려놓은 자리, base_link m, 소수 4 자리)
inline constexpr const char * kStoreReturnedTag = "STORE_RETURNED_TO_BOX";
std::string store_returned_token(double x, double y);
// 결과 글에서 토큰 찾기 (없거나 숫자가 아니면 false)
bool parse_store_returned(const std::string & message, double & x, double & y);

}  // namespace manipulation
