// rack_tags.hpp — 적재함 칸 바닥 AprilTag 로 칸 위치 재기 (순수 C++, test/test_rack_tags.cpp)
// =====================================================================
// 칸 바닥 높이를 아니까 깊이를 추정하지 않음: 태그 모서리 화소의 광선 ∩ 칸 바닥 평면 → base_link 의 xy
//   (시뮬에서 태그 자세 추정 solvePnP 는 2~10 mm, 평면 교점은 1 mm 안)
//   네 모서리를 평면에 내려 놓으니 그 평균이 태그 중심 · 한 변 길이가 태그 크기와 맞아야 함 (높이 · TF 가 틀리면 크기가 틀어짐)
// 여러 칸을 재면 적재함 전체 이동 (평행 이동 + 회전) 을 맞춰 안 잰 칸의 옮길 양도 냄

#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace manipulation
{

struct TagPlaneConfig
{
  double plane_z = 0.1325;     // 칸 바닥 (base_link z) [m]
  double tag_size_m = 0.026;   // 태그 검은 사각 한 변
  double size_tol = 0.15;      // 잰 한 변이 tag_size_m 에서 이 비율 넘게 다르면 버림 (평면 높이 · TF 가 틀림)
  // 광선이 수직에서 20° (cos 0.94) 안이어야 씀: 비스듬할수록 카메라 각도 오차가 크게 번짐 (40° 에서 2° 틀리면 약 10 mm)
  //   스캔 자세는 제 칸 두 개를 약 7° 로 봄 (비스듬히 본 옆 칸이 섞이면 칸끼리 크게 어긋남)
  double min_view_cos = 0.94;
};

struct TagOnPlane
{
  bool ok = false;
  Eigen::Vector2d center{0.0, 0.0};   // base_link xy [m]
  double yaw = 0.0;                   // 태그 x 축 (모서리 0 → 1) 방향 [rad]
  double side_m = 0.0;                // 네 변 평균
  const char * why = "";
};

// corners_n: 왜곡을 푼 정규 좌표 (x/z, y/z) 의 태그 모서리 4 개 (aruco 순서), T_base_cam: base_link ← 카메라 광학 프레임
TagOnPlane tag_on_plane(
  const std::array<Eigen::Vector2d, 4> & corners_n, const Eigen::Isometry3d & T_base_cam, const TagPlaneConfig & c);

// 적재함 전체 이동: 잰 = R(dyaw) · (표 − pivot) + pivot + (dx, dy). pivot = 적재함 중심 (12 칸 평균, 잰 칸과 상관없이 고정)
//   잰 칸끼리 가장 먼 거리가 min_span_m 보다 짧으면 평행 이동만 (dyaw = 0, 가까운 두 칸의 1 mm 오차가 회전 1° 넘게 됨)
struct RackFit
{
  bool ok = false;
  bool rotation = false;                   // 회전도 맞췄는지
  double dx = 0.0, dy = 0.0, dyaw = 0.0;   // [m] · [rad]
  double rms = 0.0;                        // 맞춘 뒤 남는 오차 [m]
  int n = 0;
};

RackFit fit_rack(const std::vector<Eigen::Vector2d> & expected, const std::vector<Eigen::Vector2d> & measured,
  const Eigen::Vector2d & pivot, double min_span_m = 0.06);

// fit 을 칸 하나 (표 위치) 에 적용했을 때 옮길 양 [m]
Eigen::Vector2d rack_shift(const RackFit & f, const Eigen::Vector2d & expected, const Eigen::Vector2d & pivot);

// 값들의 가운데 (짝수면 가운데 둘 평균). 비면 NaN
double median(std::vector<double> v);

// 퍽 윗면 태그 PnP 두 해 고르기 (rack_tag_stage /rack_tags/puck_pnp, 넣은 뒤 보기): e1 ≤ e2 = IPPE 두 해 재투영 RMS [px],
//   up1 · up2 = 두 해 윗면 법선의 base_link z 성분. 두 번째 해가 뚜렷이 나쁘면 (e2 ≥ max(2 e1, e1 + 0.3 px)) 첫 해 (0), 아니면 모호 →
//   법선이 위쪽에 더 가까운 해 (작은 태그를 거의 수직으로 보면 기울기 부호가 갈림 — 크게 잘못 읽어 멀쩡한 퍽을 다시 넣지 않게, 보수적)
int pick_pnp_solution(double e1_px, double e2_px, double up1, double up2, bool & ambiguous);

}  // namespace manipulation
