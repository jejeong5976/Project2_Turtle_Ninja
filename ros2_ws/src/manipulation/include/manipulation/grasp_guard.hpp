// grasp_guard.hpp — 쥐기 · 넣기 확인 판단 (순수 C++, test/test_grasp_guard.cpp). 쓰는 곳: control_master.cpp 의 grasp_guard 묶음
//   ① 쥔 폭: PICK 닫기 뒤 그리퍼 실측 → 손가락 사이 [mm] (모델 2 · (inner0 + q) + 보정) ↔ 퍽 종류 폭 (원기둥 30 · 네모 34.8)
//   ② 손 안 퍽: 손목 카메라 쥔 퍽 태그 (wrist_link) → 접근축에서 옆으로 [mm] · 윗면 기울기 [°] · 쥔 깊이 [mm]
//   ③ 넣기 걸림: 칸 안 마지막 내려가기 동안 J2–J4 추종 오차 · 전류 · 손끝 높이 모자람이 내려가기 직전보다 커졌나
//   ④ 넣은 뒤: 놓은 퍽 윗면 태그 (base_link) 높이 ↔ 칸 바닥 + 60 mm, 칸 중심 ↔ 컵 안 여유
//   ④-2 넣은 뒤 보기: 칸 위 관찰 자세에서 퍽 윗면 태그 PnP → 칸 기준 위치 · 높이 · 기울기 · 면 방향 → 잘못 앉음 판단
//   ⑤ 다시 쥐기 동작: 그 자리에서 · 들기를 되돌려 내려놓고 다시 쥐기 · 칸 안에서 들어 옮기기
//   ⑥ 넣기 걸림 보정 (store_recover): 칸 안에서 놓지 않고 들어 나와 박스로 되돌리기

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
  double gap_bias_mm{7.4};    // 모델 간격에 더할 보정 (네모 34.8 을 쥔 채 q 약 0.0046 → 모델 27.4 mm)
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
//   frame: 표본 좌표 (wrist_link 만), want_id: 지난 PICK (RETRIEVE) 의 목표 퍽 ID ("" = 모름 → 다 버림),
//   after_s: 쥔 시각, before_s: 놓은 시각 (안 놓았으면 무한) [s] — 이 사이 표본만 받음
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

// ============================================================================ ④-2 넣은 뒤 보기 (seat look)
// STORE 가 놓고 P0 로 빠진 뒤 그 칸을 내려다보는 자세 (LOOK_RACK 관찰 자세) 에서 퍽 윗면 태그 PnP
//   (/rack_tags/puck_pnp → control_master 가 drift 되돌려 base_link) 여러 장 → 칸 기준 자세 → 판단. RL nudge 입력으로도 씀
// 칸 기준 = 칸 바닥 중심 · 칸 방향 (적재함 CAD 축 + 스캔 맞춤 회전) · 바로 앉은 퍽 윗면 높이 (칸 바닥 + 60 mm + seat_top_offset_mm)
struct SeatCellRef
{
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};   // 칸 바닥 중심 [m, base_link]
  double yaw{0.0};                               // 칸 x 축 방향 [rad, base_link]
  double top_z{0.1925};                          // 바로 앉은 퍽 윗면 [m, base_link]
};
struct SeatPose   // 한 장, 칸 축 (x = 칸 yaw 방향 · y = 그 왼쪽)
{
  double x_mm{kGuardNaN}, y_mm{kGuardNaN};   // 윗면 태그 중심 − 칸 중심
  double height_mm{kGuardNaN};               // 윗면 − 바로 앉은 윗면 (+ = 떠 있음)
  double tilt_deg{kGuardNaN};                // 윗면 법선 ↔ 위
  double yaw_deg{kGuardNaN};                 // 태그 x 축 ↔ 칸 x 축, 90° 로 접음 [−45, 45] (네모 면 방향)
};
// p: 태그 중심 [m, base_link], R: 태그 자세 (열 = x · y · z 축, z = 윗면 법선) [base_link]
SeatPose seat_pose_in_cell(const Eigen::Vector3d & p, const Eigen::Matrix3d & R, const SeatCellRef & c);

// 기준 (control_master seat_* 값)
struct SeatLookConfig
{
  double height_tol_mm{6.0};                   // 이보다 높이 뜨면 잘못 앉음 (위로만 — 바닥 아래로는 못 감)
  double tilt_tol_deg{8.0};
  double yaw_tol_deg{15.0};                    // 네모만
  double xy_tol_x_mm{6.25}, xy_tol_y_mm{6.25}; // 칸 축 x · y (seat_cup_clearance_mm)
  int min_frames{3};
  double top_max_tilt_deg{60.0};               // 이보다 기운 태그 = 옆면 태그 (윗면 아님) → 안 셈
  double outlier_mm{4.0}, outlier_deg{6.0};    // 가운데값에서 이보다 먼 장은 버림
  double spread_mm{2.5}, spread_deg{4.0};      // 남은 장 흩어짐 (RMS) 이 이보다 크면 믿지 않음
};
enum class SeatLookState {MISSING, NOISY, SEATED, BAD};
const char * seat_look_state_name(SeatLookState s);
struct SeatLookVerdict
{
  SeatLookState state{SeatLookState::MISSING};
  SeatPose pose;           // 남은 장 가운데값 (면 방향은 90° 주기 평균)
  int used{0}, total{0};
  double spread_mm{kGuardNaN}, spread_deg{kGuardNaN};
  std::string bad;         // BAD: 넘은 기준 (높이 · 기울기 · 위치 · 면), MISSING · NOISY: 믿지 않는 이유
  std::string text;        // 로그 한 줄 (칸 기준 자세 · 장 수 · 흩어짐)
};
// square: 네모 (면 방향도 봄). 높이가 선 자리보다 height_tol_mm 넘게 낮으면 기준 높이 (seat_top_offset_mm) · 측정이 틀린 것 → NOISY
SeatLookVerdict judge_seat_look(const std::vector<SeatPose> & frames, bool square, const SeatLookConfig & c);
// 컵 위치 허용 [mm] = (컵 입구 − 퍽 폭) / 2 + 측정 여유. CAD 컵 입구 (테두리, 27 mm 높이) 39.3 mm.
//   윗면 중심이 이보다 멀면 퍽이 테두리에 기대 있음. 측정 여유 4 = 기준 스캔 1.5 + 카메라 · drift 1–2 + 태그 붙인 자리 1–2 mm
double seat_cup_clearance_mm(double puck_w_mm, double cup_mm = 39.3, double slack_mm = 4.0);
// 꺼내 다시 넣을지: BAD (믿을 만한 측정에서만 나옴) · 이 퍽에서 done < max
bool seat_should_recover(const SeatLookVerdict & v, int done, int max);

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

// ============================================================================ ⑥ 넣기 걸림 보정 (store_recover)
// 명령 자세에서 손끝을 곧게 위로 step_m 씩 (tip_offset_command 를 이어서, 접근축 · 닫힘축 유지) 최대 lift_m 까지 올린 자세.
//   관절 범위 (J4 멈춤 등) · IK 로 더 못 올리면 거기까지. 손끝 옆 어긋남 1 mm · 접근축 2° 안. 돌려줌 = 올린 높이 [m] (0 = 못 올림, out 그대로 cmd)
double tip_lift_max(const ArmModel & m, const std::vector<double> & cmd, double lift_m, double step_m, std::vector<double> & out);
// 칸 경로에서 퍽을 놓는 첫 점 (grip release · open) 번호. 없으면 nodes.size()
std::size_t store_release_index(const std::vector<PathNode> & nodes);
// 칸 안 놓는 점 nodes[release] 에서 퍽을 쥔 채 온 길로 nodes[0] (박스 위 기준점) 까지 (retrace_nodes: 고리 빼고 · 지나온 구간 속도 ·
//   그리퍼 · 멈춤 없음). 결과 [0] = 놓는 점, 끝 = nodes[0]. 놓는 점 바로 위 점이 lift_m 보다 낮으면 놓는 점에서 곧게 올린 점
//   (recover_lift, 되는 만큼) 을 먼저 넣고, 그보다 낮고 옆으로 15 mm 안인 점은 건너뜀. 올린 높이가 바로 위 기록 점보다 낮으면 온 길 그대로.
//   note = 올린 높이 한 줄 (로그). 온 길이 안 되면 빈 목록 (why)
std::vector<PathNode> store_recover_out_nodes(const ArmModel & m, const std::vector<PathNode> & nodes, std::size_t release, double lift_m,
  std::string & why, std::string & note);
// 관찰 자세에서 온 칸 경로 (store_look_<칸>) 의 들어 나오기 (out, 끝 = 관찰 자세) 뒤에 방금 간 look_rack 경로 (look, 실행한 그대로) 를
//   거꾸로 붙임 → 끝 = look 첫 점 (집은 자리 · 기준점). out 끝이 look 끝과 관절마다 tol [rad] 안이 아니거나 거꾸로 못 가면 false + why (out 그대로)
bool store_look_recover_extend(std::vector<PathNode> & out, const std::vector<PathNode> & look, double tol, std::string & why);
// 결과 글 토큰 (미션이 읽음): "STORE_RETURNED_TO_BOX x y" (박스에 내려놓은 자리, base_link m, 소수 4 자리)
inline constexpr const char * kStoreReturnedTag = "STORE_RETURNED_TO_BOX";
std::string store_returned_token(double x, double y);
// 결과 글에서 토큰 찾기 (없거나 숫자가 아니면 false)
bool parse_store_returned(const std::string & message, double & x, double & y);

}  // namespace manipulation
