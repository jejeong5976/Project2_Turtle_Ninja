// barcode_search.hpp — 출발 구역 바코드 찾기 · 읽기 (순수 계산 · test/test_barcode_search.cpp)
//
// 왜
//   - 바코드 자리가 무작위 (barcode_view 고정 자리 아님). 고정 LOOK_BARCODE 자세는 광축이 바코드에서 28° 벗어나 벽 위를 봄
//   - 신호등 점멸 (STANDBY · HOME · 첫 READ_BARCODE · OBSERVE_SIGNAL) 을 광각으로 보는 중 → 몸을 돌려도 신호등이 광각 시야 안
// 흐름 (BarcodeSearch::next 가 다음 할 일 하나를 줌, mission_manager 가 단계로 바꿈)
//   1) 광각 barcode 상자 → 어안 역투영 광선 (URDF TF + bc_wide_pitch_offset_deg) → 벽 거리와 만나는 점 = 바코드 xyz
//        벽 거리 차례: 손목 깊이 (바코드가 손목에 보이면) > 앞 라이다 + bc_lidar_range_offset_m > 상자 폭 · 바코드 폭 > bc_default_range_m
//        높이는 잰 값 (고정 45 cm 아님). 신호등 · 바코드는 볼 때마다 다시 잼 (사람이 신호등을 옮길 수 있음)
//      벽 법선 = 맞은 점 둘레 라이다 점 직선 → 보는 방향 (정면)
//   2) LOOK_AT (팔 작업 9) 으로 그 점을 standoff 에서 봄 → 머묾 (판독 기다림, 손목 바코드 상자 모음)
//   3) 못 읽고 손목 상자가 있으면: 상자 가운데 광선 ↔ 벽 면 → 목표 다시 · 크기 (화소/모듈) 로 거리 다시 → LOOK_AT (refine_max 번)
//   4) 그래도 못 읽으면 흔들기 (J1 ±4°, J4 ±3°, 거리 ±5 cm)
//   5) 광각에 없으면: 몸 제자리 돌기 (rotate_steps, 신호등이 광각 시야 안 + light_margin 일 때만) → 손목 훑기 (sweep_az × sweep_heights)
//   6) 닿지 않음 (팔 거절 "범위" · "거리") → 출발 구역 안 몸 옮기기 (신호등 시야 조건, max_base_moves)
//   7) 팔이 신호등 시야를 가려 거절 + 기록 중 → 그 목표는 미룸 (신호 기록이 끝나면 다시). 다 해도 없으면 기다림 → 시간 초과 GiveUp
//   몸 멈춤 (World::hold_base): 신호 기록이 아직 안 끝났으면 몸을 옮기거나 돌리지 않음 (5 · 6 · 다가가기 보류).
//      몸을 돌리면 가까이 낮게 선 신호등이 어안 구석으로 가 노랑 · 초록 등이 잘림 → 12 점멸을 못 채움.
//      그동안은 지금 자리에서 팔만 (광각 목표 LOOK_AT · 손목 훑기, 가림 검사 그대로). 닿지 않는 목표는 미룸 (hold)
//      기록이 끝나면 (Done · 끝남) 미룬 목표 먼저 → 다가가기 · 돌기 · 옮기기 그대로. 몸 멈춤 때문에 기다린 시간은 max_s 에서 뺌
//      (어차피 OBSERVE_SIGNAL 에서 기다릴 시간), 그 합이 hold_max_s (mission signal_wait_s) 를 넘으면 멈춤 풀림
//   벽 판 (SearchParams::panel): 실제 경기장은 바코드가 늘 출발 구역 앞 벽 판 하나 (폭 36 · 높이 39 cm, 파란 기둥 둘 사이) 위.
//      광각 상자는 그 판 (+ panel_margin) 안 목표만 씀 (다른 곳 상자 버림), 라이다가 없으면 판 면과 광선이 만나는 점.
//      제자리 돌기 없음. 광각에 없으면 판 따라 팔만 훑기 (panel_sweep_m, 광각이 본 자리 먼저).
//      몸은 팔이 못 닿을 때만 그 목표 정면 approach_dist 자리로 작게 (앞 · 옆, 방향 그대로).
//      옮긴 뒤에도 [REACH] 면 renudge_m 더 가까이 (approach_min_m 까지, max_base_moves 안)
//      판 면은 라이다로 맞춤 (판 가운데 쪽 벽 거리: 지도보다 가까우면 늘, 멀면 panel_depth_tol 안일 때만 — 벽 속을 보지 않게)
//   GiveUp → mission 이 예비 고정 경로 (barcode_view + LOOK_BARCODE) 로 (bc_fallback_fixed)
// 좌표
//   - 목표 · 신호등 = map. 로봇 자세 (map 2D), 광각 · 손목 카메라 = 광학 프레임 (z 앞, x 오른쪽, y 아래)
// note
//   - 모든 기준값은 SearchParams (mission_manager bc_* 파라미터). 읽기 범위 (거리 · 비스듬 · 화소/모듈 · 가운데 허용) 는 자리표시 — 실측으로 바꿀 것

#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mission
{
namespace bc
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// mission 단계 kind (common.hpp StepKind 다음 번호, 겹치지 않게) · 팔 작업 번호
constexpr int kStepBcPlan = 11;    // 다음 할 일 정하기 (BarcodeSearch::next)
constexpr int kStepBcDwell = 12;   // 머묾 (판독 · 광각 기다림)

struct V3
{
  double x{0.0}, y{0.0}, z{0.0};
};
inline V3 operator+(V3 a, V3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline V3 operator-(V3 a, V3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline V3 operator*(V3 a, double k) {return {a.x * k, a.y * k, a.z * k};}
inline double dot(V3 a, V3 b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
inline V3 cross(V3 a, V3 b) {return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};}
inline double norm(V3 a) {return std::sqrt(dot(a, a));}
inline V3 unit(V3 a) {const double n = norm(a); return n > 1e-12 ? a * (1.0 / n) : V3{};}

// 강체 변환 parent ← child: p_parent = R p_child + t
struct Pose3
{
  std::array<std::array<double, 3>, 3> R{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  V3 t;
};
V3 rotate(const Pose3 & T, V3 v);
V3 apply(const Pose3 & T, V3 p);
Pose3 compose(const Pose3 & a, const Pose3 & b);
Pose3 planar(double x, double y, double yaw, double z = 0.0);
Pose3 from_quat(double qx, double qy, double qz, double qw, V3 t);
// 광학 프레임 카메라를 더 아래로 down_rad 숙임 (광학 x 축 둘레, URDF 를 안 고치고 bc_wide_pitch_offset_deg 로 보정)
Pose3 pitch_optical(const Pose3 & T, double down_rad);
// 광학 카메라를 왼쪽으로 left_rad 돌림 (광학 y 축 둘레, bc_wide_yaw_offset_deg: + = 실제 카메라가 URDF 보다 왼쪽을 봄 → 추정이 왼쪽으로)
Pose3 yaw_optical(const Pose3 & T, double left_rad);

// 어안 (equidistant, OpenCV fisheye: θd = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸))
struct Fisheye
{
  double fx{853.374}, fy{852.157}, cx{675.548}, cy{371.440};
  std::array<double, 4> k{{-0.128704, 0.012667, -0.039807, 0.022852}};
  int w{1280}, h{720};
};
V3 fisheye_ray(const Fisheye & c, double u, double v);              // 광학 프레임 단위 광선
bool fisheye_project(const Fisheye & c, V3 ray, double & u, double & v);

struct Pinhole
{
  double fx{606.321}, fy{606.188}, cx{321.425}, cy{248.200};
  int w{640}, h{480};
};
V3 pinhole_ray(const Pinhole & c, double u, double v);

// 광각 시야 끝 각 [rad]: 가로 (cy 줄) 왼 · 오른, 세로 (cx 열) 위 · 아래 (광학 x = 오른 +, y = 아래 +)
struct FovEdges
{
  double left{0.0}, right{0.0}, top{0.0}, bottom{0.0};
};
FovEdges fisheye_edges(const Fisheye & c);

// 신호등 (map) 이 로봇 자세 (rx, ry, ryaw) 에서 광각 시야 안 + margin 인가. margin_deg 결과 = 가장 가까운 끝까지 여유 [°]
struct FovCheck
{
  bool ok{false};
  double margin_deg{-180.0};
};
//   돌기는 가로만 바꾸므로 margin 은 가로 (왼 · 오른 끝) 여유, 세로는 화면 안이기만 하면 됨. base_z = base_link 바닥 위 높이
FovCheck light_in_fov(const Fisheye & c, const Pose3 & T_base_cam, double rx, double ry, double ryaw, V3 light_map, double margin_deg,
  double base_z = 0.033);

// 라이다 점 (같은 프레임 xy) 중 원점 (ox, oy) 에서 방위 az ± half 안 · 거리 [rmin, rmax] 점들의 수평 거리 가운데값. 3 점 미만이면 NaN
double scan_range(const std::vector<std::array<double, 2>> & pts, double ox, double oy, double az, double half, double rmin, double rmax);
// 맞은 점 (hx, hy) 둘레 radius 안 점들의 직선 → 원점 쪽을 향한 법선의 반대 = 벽을 보는 방향 yaw [rad]. 점 5 개 미만이면 NaN
double wall_view_yaw(const std::vector<std::array<double, 2>> & pts, double ox, double oy, double hx, double hy, double radius);
// 광선 (origin, dir) 위 수평 거리 range_h 인 점
bool ray_at_range(V3 origin, V3 dir, double range_h, V3 & out);
// 광선과 수직 면 (점 p0, 수평 법선 yaw n_yaw) 의 만나는 점
bool ray_plane(V3 origin, V3 dir, V3 p0, double n_yaw, V3 & out);

// 출발 구역 (map 다각형 4 점)
struct Zone
{
  std::array<std::array<double, 2>, 4> c{};
  bool valid{false};
};
Zone zone_from_arena(double x0, double x1, double y0, double y1, double mfa_x, double mfa_y, double mfa_yaw);
bool in_zone(const Zone & z, double x, double y, double margin);

// 바코드 화소/모듈 (상자 폭 / 모듈 수) 이 min_ppm 이 되는 거리 = 지금 거리 × ppm / min_ppm
double standoff_for_ppm(double bbox_w_px, int modules, double dist_now, double min_ppm);

// ================================================================ 찾기 상태
struct SearchParams
{
  double standoff_m{0.15};         // 바라는 손목 카메라 ↔ 바코드 (24 cm = 2 px/모듈 못 읽음, 15 cm ≈ 3.5 px/모듈 읽음)
  double standoff_min_m{0.15};      // 15 cm 아래 = 손끝이 벽에 닿을 수 있음 · 24 cm = 못 읽음 → 읽기 보기는 [0.15, 0.18] 만
  double standoff_max_m{0.18};
  double sweep_standoff_m{0.30};   // 손목 훑기 때 (넓게 보기, 읽기 아님 — 자리 찾기)
  // 두 걸음 보기: 광각 목표를 먼저 locate_standoff (넓게 · J2 작게) 에서 봐서 손목 상자 (라벨) 를 찾고 → 그 상자로 한 번 0.15 m 로.
  //   먼 보기에서 아무것도 없으면 훑기 · 돌기. 먼 보기가 [REACH] 로 거절되면 바로 0.15 m 보기. false = 바로 0.15 m 보기
  bool two_step{true};
  double locate_standoff_m{0.28};
  double default_range_m{0.70};    // 라이다 · 크기 둘 다 없을 때 광각 → 벽 수평 거리
  double range_min_m{0.25}, range_max_m{1.60};
  double scan_half_deg{3.0};
  double lidar_range_offset_m{-0.03};   // 라이다 벽 거리에 더함 (앞 라이다가 벽을 1.7–4.5 cm 멀게 봄)
  double barcode_width_m{0.08};    // 상자 폭으로 거리 잴 때 (약 8 cm)
  int modules{90};                 // Code128 'P2-0x' 모듈 수 (여백 뺌) — 화소/모듈 계산
  double min_px_per_module{3.0};   // 2 px/모듈은 못 읽음
  double min_width_px{0.0};        // > 0 이면 손목 상자 폭이 이보다 작으면 다가감 (ppm 대신 · 같이)
  double center_tol_px{60.0};      // 손목 상자 가운데가 겨눔 화소에서 이보다 멀면 다시 겨눔
  double aim_du_px{-60.0}, aim_dv_px{-50.0};   // 겨눔 화소 = 화면 가운데 + 이것 (control_master look_at_aim_offset_px 와 같게, 손가락 피하기)
  double edge_margin_px{15.0};     // 상자가 화면 끝 이만큼 안이면 잘린 것 → 다시 겨눔
  double max_obliquity_deg{40.0};  // 보는 방향이 광선 방위에서 이보다 틀어지면 이만큼만 (팔 look_at_max_obliquity_deg 보다 작게)
  double light_margin_deg{8.0};    // 신호등 ↔ 광각 시야 끝 최소 여유
  double light_default_range_m{0.80};
  double light_default_height_m{0.30};   // 바닥 위 (못 봤을 때 가정)
  std::vector<double> rotate_steps_deg{25.0, -25.0};   // 광각에 바코드가 없을 때 제자리 돌기 (신호등 조건 통과만)
  std::vector<double> sweep_az_deg{0.0, 35.0, -35.0, 70.0, -70.0, 110.0, -110.0};   // 손목 훑기 방위 (base, 왼 +)
  std::vector<double> sweep_heights_m{0.45};           // 바닥 위. 광각이 바코드를 본 적 있으면 그 높이 하나만 (머리가 위로 안 감), 못 봤으면 이것 (발제 450 mm)
  double wide_fresh_s{1.5};
  double dwell_s{6.0};             // LOOK_AT 뒤 판독 기다림 (자세 도착 뒤 3–5 s 에 읽힘, 15 s 넘기지 말 것)
  double sweep_dwell_s{1.5};
  double settle_s{0.4};            // LOOK_AT 끝난 뒤 이 시간 뒤 손목 상자만 씀
  int refine_max{2};
  std::vector<std::array<double, 3>> perturbs{};   // dj1° dj4° dd m. 기본 없음 (팔 움직임 줄임 — mission bc_perturb_* 로 켬)
  bool perturb_need_box{true};     // 그 목표에서 손목 상자를 한 번도 못 봤으면 흔들기 안 함 (틀린 목표에 60 s 쓰지 않게)
  double depth_plane_tol_m{0.04};  // 손목 깊이 점은 지금 목표 벽 면에서 이만큼 안일 때만 (D435 최소 거리 · 막대 겹침)
  double depth_min_m{0.08};        //   + 카메라에서 이 거리 [min, max] 안
  double depth_max_m{0.60};
  double light_wait_s{20.0};       // 신호 기록 중 · 신호등을 아직 못 봤으면 이만큼 기다림 (팔 접고), 그 뒤엔 광각 바코드 + 다가가기만 (돌기 · 훑기 안 함)
  double max_s{75.0};              // 찾기 전체 시간 → GiveUp (몸 멈춤으로 기다린 시간은 뺌)
  double hold_max_s{90.0};         // 몸 멈춤으로 기다린 시간 합 상한 (mission 이 signal_wait_s 로 채움) → 넘으면 멈춤 풀림
  int max_base_moves{2};
  double approach_dist_m{0.38};    // base ↔ 바코드 수평 거리 목표 (벽 38 cm 에서 접은 자세로 읽음 — J2 토크 작게)
  bool approach_first{true};       // 바코드가 approach_dist + approach_tol 보다 멀면 LOOK_AT 전에 몸부터 (출발 구역 · 신호등 조건 안)
  double approach_tol_m{0.04};
  double robot_half_m{0.17};       // 출발 구역 안 판정 여유 (차체 반)
  double dedupe_m{0.08};           // 이미 본 목표와 이만큼 안이면 같은 것
  double z_min_m{0.03}, z_max_m{1.0};   // 바코드 높이 (바닥 위) 범위 — 밖이면 버림
  double base_z_m{0.033};          // base_link 높이 (바닥 위)
  // 벽 판 (mission bc_panel_*). panel false = 돌기 · 방위 훑기
  bool panel{false};
  V3 panel_c;                      // 판 가운데 (map, z = 바닥 위)
  double panel_n_yaw{0.0};         // 판 법선 (벽 → 경기장 안쪽) map yaw
  double panel_w_m{0.36}, panel_h_m{0.39};
  double panel_margin_m{0.08};     // 판 밖 허용 (지도 맞춤 · 광각 추정 오차)
  double panel_depth_tol_m{0.20};  // 광각 목표 · 라이다 판 면 맞춤이 판 면에서 이만큼 안
  std::vector<double> panel_sweep_m{0.0, -0.12, 0.12};   // 판 가운데에서 벽 따라 (로봇에서 보아 왼 +) 훑기 자리
  double renudge_m{0.04};          // 몸 옮긴 뒤에도 닿지 않음 → 이만큼 더 가까이
  double approach_min_m{0.30};     //   base ↔ 바코드 수평 거리 최소 (차체 앞 끝 0.145 + 여유)
};

struct Box
{
  double u{0.0}, v{0.0}, w{0.0}, h{0.0};   // 가운데 · 크기 [px]
  double stamp{-1.0};
  std::string text;                         // 손목: 읽은 원문 ("" = 자리만)
};

// 찾기에 넘기는 지금 상태 (mission_manager 가 TF · 토픽에서 채움)
struct World
{
  double now{0.0};
  bool have_pose{false};
  double rx{0.0}, ry{0.0}, ryaw{0.0};      // base_link in map
  bool have_wide_tf{false};
  Pose3 T_base_wide;                        // 광각 광학 in base_link
  std::optional<Box> wide_barcode, wide_light;
  std::optional<Box> wrist_barcode;         // 마지막 LOOK_AT 뒤 settle 지난 손목 상자
  std::optional<V3> wrist_barcode_map;      // 그 상자의 손목 깊이 3D 점 (map) — 벽 거리 1 순위
  std::optional<V3> wrist_center_map;       // 손목 화면 가운데 깊이 점 (map) — 훑기 때 벽 자리
  bool have_wrist_tf{false};
  Pose3 T_map_wrist;                        // 손목 광학 in map (그 상자를 찍을 때 — 멈춘 팔)
  std::vector<std::array<double, 2>> scan_base;   // 라이다 점 (base_link xy)
  bool signal_done{false};                  // 신호 기록 끝 → 신호등 조건 끔
  bool hold_base{false};                    // 신호 기록 진행 중 (Done · 끝남 · 시간 초과 아님) → 몸 옮기기 · 돌기 보류 (mission 이 채움)
  bool wide_ready{true};                    // 찾기 시작 · 몸 옮긴 뒤 광각 상자를 충분히 받았나 (아니면 '광각에 없음' 판단을 미룸)
  double start_yaw{kNaN};                   // 출발 자세 yaw (신호등을 못 봤을 때 그 앞으로 가정)
};

enum class Act { LookAt, MoveBase, Dwell, GiveUp };

struct Action
{
  Act kind{Act::GiveUp};
  V3 target;                     // map (LookAt)
  double standoff{0.25};
  double view_yaw{kNaN};         // map 보는 방향 (카메라 → 목표)
  double dj1_deg{0.0}, dj4_deg{0.0};
  bool sweep{false};
  double bx{0.0}, by{0.0}, byaw{0.0};   // MoveBase (map)
  double dwell_s{0.0};
  bool light{false};             // LOOK_AT 에 신호등 시야선 조건 (기록 중)
  V3 light_map;
  std::string note;
};

class BarcodeSearch
{
public:
  void reset(const SearchParams & p, const Fisheye & wide, const Pinhole & wrist, const Zone & zone, double now);
  // 다음 할 일. 먼저 observe 로 광각 · 신호등을 반영
  Action next(const World & w);
  // 마지막 LookAt 이 팔에서 거절 · 실패 (why = control_master 글)
  void on_look_failed(const std::string & why, const World & w);
  // 마지막 MoveBase 실패
  void on_move_failed(const std::string & why);
  bool light_known() const {return light_known_;}
  V3 light_map() const {return light_;}
  double elapsed(double now) const {return now - t0_;}
  const std::string & last_note() const {return note_;}
  int looks() const {return looks_;}
  // 몸 멈춤 중인가 (신호 기록 중 + 멈춤으로 기다린 합 < hold_max_s)
  bool holding(const World & w) const {return w.hold_base && held_s_ < p_.hold_max_s;}
  // 몸 멈춤 때문의 기다림 (이 시간은 max_s 에서 뺌, 다음 next 에서 셈). mission 도 씀 (예비 고정 경로 포기를 기록 끝까지 미룰 때)
  Action hold_wait(const World & w, double secs, const std::string & note);
  double held_s() const {return held_s_;}
  // max_s 에 세는 시간 (몸 멈춤 기다림 뺌)
  double search_s(double now) const {return now - t0_ - held_s_;}

  // 시험 · 로그용
  std::optional<V3> target_from_wide(const World & w, double * view_yaw) const;
  std::optional<V3> target_from_wrist(const World & w, double * dist) const;
  void update_light(const World & w);
  // 벽 판: 라이다로 맞춘 판 가운데 (map) · 판 안인가 (lat = 벽 따라 로봇에서 보아 왼 +)
  V3 panel_center(const World & w) const;
  bool in_panel(V3 t, V3 c, double * lat = nullptr) const;
  int panel_rejects() const {return panel_rejects_;}

private:
  struct Cand
  {
    V3 target;
    double view_yaw{kNaN};
    std::string src;
    double standoff{0.25};
    int refines{0};
    std::size_t perturb_i{0};
    bool base_moved{false};
    bool saw_box{false};       // 이 목표 LOOK_AT 에서 손목 상자를 본 적 있나 (S4)
    bool locate{false};        // 두 걸음 보기의 먼 보기 (훑기처럼 다룸)
    bool held{false};          // 몸 멈춤으로 미룸 (닿지 않음 · 다가가기) — 멈춤이 풀리면 다시 (신호 끝 아니어도)
    bool panel_sweep{false};   // 벽 판 훑기 자리 (훑기처럼 보되 닿지 않으면 몸 옮김 · 미룸)
  };
  Action look(const Cand & c, const World & w, double dj1, double dj4, double dd, bool sweep, const std::string & note);
  bool seen_before(V3 t) const;
  std::optional<Action> base_move_for(const Cand & c, const World & w, bool * light_blocked = nullptr, double want = -1.0);
  Action wait(double secs, const std::string & note);
  std::optional<Action> maybe_approach(Cand & c, const World & w);

  SearchParams p_;
  Fisheye wide_;
  Pinhole wrist_;
  Zone zone_;
  double t0_{0.0};
  bool light_known_{false};
  V3 light_;
  std::vector<Cand> queue_, deferred_;
  std::vector<V3> tried_;
  std::optional<Cand> cur_;
  bool cur_sweep_{false};
  std::size_t rot_i_{0}, sweep_i_{0};
  int base_moves_{0};
  double last_look_s_{-1.0};
  int looks_{0};
  std::string note_;
  double rot_base_yaw_{kNaN};
  double last_wide_z_{kNaN};          // 광각이 본 바코드 높이 (훑기 높이)
  bool cur_perturb_{false};          // 마지막 LookAt 이 흔들기
  std::optional<Action> pending_move_;   // 닿지 않아 몸 옮기기 (다음 next 가 먼저 줌)
  double held_s_{0.0};               // 몸 멈춤으로 기다린 시간 합
  double hold_since_{-1.0};          // 마지막 hold_wait 시각 (다음 next 에서 held_s_ 에 더함)
  void defer_held(Cand c);
  double last_wide_lat_{kNaN};        // 광각이 본 바코드의 판 가로 자리 (판 훑기 첫 자리)
  mutable int panel_rejects_{0};      // 판 밖이라 버린 광각 상자 수 (로그)
};

}  // namespace bc
}  // namespace mission
