// simple_dock_core.hpp — 규칙으로 하는 정밀 접근 (박스 · 장소 정차) 계산, ROS 없음. simple_dock_node.cpp 가 씀
//
// 역할
//   - DockCore: 돌기 (목표 방향과 align_deg 넘게 다르면 제자리 회전, 처음 한 번) → 다가가기 (몸 좌표로 곧게, 최고 v_max,
//     남은 거리에 비례해 느려짐, 방향은 w_fine 까지만) → 끝 (tol_pos · tol_yaw 안, 또는 tol_ok · tol_yaw_ok 안에서 stall_s 동안 1 mm 도 안 줄면)
//   - FaceTrack: 스캔마다 박스 앞면 = 직선 조각 (길이 0.12 m ~ 한 변 + 3 cm, 연속, 끝 밖으로 같은 직선이 안 이어짐 = 벽 아님)
//       처음 = 지도 박스 자리 ±search (0.25 m) · 방향 ±15° 안에서 박스 중심이 지도 자리에 가장 가까운 조각
//       (실물 arena5: 사람이 놓은 박스 · 지도 각도 오차로 8–25 cm 어긋남)
//       그 뒤 = 지금 추정 ±track (6 cm) · 방향 ±8° 안 조각만 → 최근 5 개 중앙값. 박스 옆면 (90° 다름) 은 방향으로 걸러짐
//       양 모서리가 다 보이면 옆 위치도 고치고, 아니면 옆은 이전 추정 유지. 목표 freeze_dist 안에서는 고정
//   - 안전 (매 주기, 라이다 원래 점 = scan_merger 몸 상자에 안 지워진 점)
//       가까운 점 = 몸 사각형 (±half_x · ±half_y) 바깥 띠 안, 박스 앞면 띠가 아닌 점. 몸 안쪽 (±self_in, 바퀴 · 기둥) 은 안 셈
//       로봇 중심이 박스 앞면 정면 (옆 ±몸 반폭) 밖이면 앞면 띠도 막는 점 (박스 옆에서 넘어오면 모서리를 쓸고 감)
//       명령대로 움직이면 그 점 쪽으로 다가가는 몸 부분이 있고 틈 < margin + 다가가는 속도 × (t_react + 스캔 나이) 면 그 명령은 안 씀
//       돌기 막힘 → 그 점에서 v_esc 로 떨어진 뒤 다시 / 다가가기 막힘 → 회전 빼고, 그래도 막히면 그 점 쪽 성분 빼고 미끄러지듯
//       다 막히면 멈추고 실패 (이유 = 막은 점 자리)
// 받는 것
//   - 목표 자세 (odom, 박스를 봄 · 박스 중심에서 box_dist), 로봇 자세 (odom), 합친 스캔 점 (odom), 라이다 원래 점 (몸 좌표)
// 주는 것
//   - 몸 좌표 속도 (vx, vy, wz), 상태 (Run · Done · Fail) + 이유
// note
//   - 앞면 찾기의 두 점 뽑기는 고정 난수 (LCG, 시드 0) → 같은 입력이면 늘 같은 결과

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "navigation/rl_core.hpp"

namespace navigation::dock
{

using rl::Pose2;
using rl::Vec2;

// 두 점 뽑기용 고정 난수 (64 비트 LCG, 위 31 비트)
struct Lcg
{
  uint64_t s = 0;
  uint32_t next() {s = s * 6364136223846793005ULL + 1442695040888963407ULL; return static_cast<uint32_t>(s >> 33);}
  std::pair<int, int> pick2(int n)                      // 서로 다른 두 번호 (0 ~ n − 1), n ≥ 2
  {
    const int i = static_cast<int>(next() % static_cast<uint32_t>(n));
    int j = static_cast<int>(next() % static_cast<uint32_t>(n - 1));
    if (j >= i) {++j;}
    return {i, j};
  }
};

class FaceTrack
{
public:
  FaceTrack(Vec2 goal_xy, double goal_yaw, double box_dist, double half, double search = 0.25, double track = 0.06,
    double track_deg = 8.0, double max_deg = 15.0, double freeze_dist = 0.10, int med_n = 5, double tol = 0.012, int iters = 80);
  Pose2 goal() const;
  bool update(const std::vector<Vec2> & P, Vec2 robot_xy);   // 고쳤으면 true
  double yaw0, h, th = 0.0;
  Vec2 c0, c;
  int n_ok = 0;
  std::string last_why;
  Lcg rng;

private:
  std::optional<std::array<double, 3>> find(const std::vector<Vec2> & P, Vec2 robot_xy, Vec2 center, double radius,
    double th_ref, double th_tol, bool first);
  double dist_, search_, track_, track_th_, max_th_, freeze_, tol_;
  int med_n_, iters_;
  std::vector<std::array<double, 3>> hist_;
};

struct DockParams
{
  bool box_track = true;
  double box_dist = 0.2575, box_half = 0.1025;
  double v_max = 0.15, v_min = 0.012, k_v = 1.0, w_max = 0.3, w_min = 0.06, w_fine = 0.1, k_w = 1.0, align_deg = 6.0;
  double tol_pos = 0.005, tol_yaw_deg = 2.0, tol_ok = 0.015, tol_yaw_ok_deg = 3.0, stall_s = 1.5;
  double half_x = 0.16, half_y = 0.16, margin = 0.02, self_in = 0.165, box_excl = 0.03;
  int min_hits = 3;
  double v_esc = 0.02, toward_eps = 0.002, t_react = 0.25, search = 0.25, face_depth = 0.05;
};

enum class DockState {Run, Done, Fail};

class DockCore
{
public:
  DockCore(Vec2 goal_xy, double goal_yaw, const DockParams & p = DockParams());
  Pose2 target() const;                                 // 지금 정차 목표 (odom)
  void update_box(const std::vector<Vec2> & pts_odom, Vec2 robot_xy);
  // pose odom, t 시작부터 [s], raw_age 원래 스캔 나이 [s]
  std::pair<std::array<double, 3>, DockState> step(const Pose2 & pose, const std::vector<Vec2> & raw_body, double t,
    double raw_age = 0.0);
  const std::optional<FaceTrack> & tracker() const {return tr_;}
  std::optional<FaceTrack> & tracker() {return tr_;}
  std::string mode;                                     // 지난 주기 명령 종류 (로그)
  std::string why;                                      // 끝 · 실패 이유

  // 안전 검사 (시험용으로 공개)
  std::vector<Vec2> near_points(const std::vector<Vec2> & raw_body, const Pose2 & pose, bool keep_box = false) const;
  std::vector<Vec2> blockers(const std::vector<Vec2> & near, const std::array<double, 3> & cmd) const;
  bool safe(const std::vector<Vec2> & near, const std::array<double, 3> & cmd) const;

private:
  std::pair<Vec2, double> box_pose() const;
  std::string describe(const std::vector<Vec2> & P) const;
  Vec2 g_xy_;
  double g_yaw_;
  DockParams p_;
  std::optional<FaceTrack> tr_;
  double align_, tol_yaw_, tol_yaw_ok_, band_;
  double age_ = 0.0, best_ = 1e300, t_best_ = 0.0;
  bool aligned_ = false;
};

}  // namespace navigation::dock
