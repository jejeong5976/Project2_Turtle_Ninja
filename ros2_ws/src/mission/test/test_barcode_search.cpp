// test_barcode_search.cpp — 출발 구역 바코드 찾기 계산 (barcode_search.hpp)
//   광각 카메라 = 배포 URDF birdeye (base_link 기준) + 아래 숙임 보정, 손목 = D435 컬러 내부값

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "mission/barcode_search.hpp"

using namespace mission::bc;

namespace
{
constexpr double kDeg = M_PI / 180.0;

Pose3 rz(double a) {return planar(0, 0, a, 0);}
Pose3 ry(double a)
{
  Pose3 T;
  T.R = {{{std::cos(a), 0, std::sin(a)}, {0, 1, 0}, {-std::sin(a), 0, std::cos(a)}}};
  return T;
}
Pose3 rx(double a)
{
  Pose3 T;
  T.R = {{{1, 0, 0}, {0, std::cos(a), -std::sin(a)}, {0, std::sin(a), std::cos(a)}}};
  return T;
}

// URDF: bracket yaw 0.2745 · 카메라 pitch 30° · 광학 (rpy −π/2, 0, −π/2) — base_link 위치 (−0.0463, −0.0797, 0.598)
Pose3 wide_urdf(double extra_down_deg)
{
  Pose3 T = compose(rz(0.274506446118), compose(ry(0.523598775598), compose(rz(-M_PI / 2), rx(-M_PI / 2))));
  T.t = {-0.0463, -0.0797, 0.598};
  return pitch_optical(T, extra_down_deg * kDeg);
}

// 벽: base x = wx 인 직선 (y −0.6 ~ 0.6, 1 cm 간격) — 라이다 점 (base_link)
std::vector<std::array<double, 2>> wall_x(double wx)
{
  std::vector<std::array<double, 2>> p;
  for (double y = -0.6; y <= 0.6; y += 0.01) {p.push_back({wx, y});}
  return p;
}

Box project_box(const Fisheye & f, const Pose3 & T_base_cam, V3 p, double width_m)
{
  const Pose3 T_map_cam = compose(planar(0, 0, 0, 0.033), T_base_cam);   // base_link 바닥 위 0.033
  // 광학 프레임으로
  const V3 d = p - T_map_cam.t;
  const V3 q{T_map_cam.R[0][0] * d.x + T_map_cam.R[1][0] * d.y + T_map_cam.R[2][0] * d.z,
    T_map_cam.R[0][1] * d.x + T_map_cam.R[1][1] * d.y + T_map_cam.R[2][1] * d.z,
    T_map_cam.R[0][2] * d.x + T_map_cam.R[1][2] * d.y + T_map_cam.R[2][2] * d.z};
  Box b;
  fisheye_project(f, q, b.u, b.v);
  b.w = f.fx * width_m / norm(q);
  b.h = b.w / 3;
  b.stamp = 100.0;
  return b;
}
World world_at_start()
{
  World w;
  w.now = 100.0;
  w.have_pose = true;
  w.rx = 0.0;
  w.ry = 0.0;
  w.ryaw = 0.0;   // map = base (시험)
  w.have_wide_tf = true;
  w.T_base_wide = wide_urdf(4.0);
  w.start_yaw = 0.0;
  w.wide_light = project_box(Fisheye{}, w.T_base_wide, {0.8, 0.0, 0.30}, 0.05);   // 신호등을 광각에서 봄 (못 보면 기다림)
  return w;
}

}  // namespace

TEST(BarcodeSearch, FisheyeRoundTrip)
{
  const Fisheye f;
  for (double u : {10.0, 300.0, 675.0, 1000.0, 1270.0}) {
    for (double v : {5.0, 200.0, 371.0, 700.0}) {
      const V3 r = fisheye_ray(f, u, v);
      double u2, v2;
      ASSERT_TRUE(fisheye_project(f, r, u2, v2));
      EXPECT_NEAR(u2, u, 0.01);
      EXPECT_NEAR(v2, v, 0.01);
    }
  }
  const auto e = fisheye_edges(f);
  std::printf("광각 시야: 왼 %.1f° 오른 %.1f° 위 %.1f° 아래 %.1f°\n", e.left / kDeg, e.right / kDeg, e.top / kDeg, e.bottom / kDeg);
  EXPECT_NEAR((e.right - e.left) / kDeg, 94.0, 6.0);   // camera_wide.yaml: 수평 약 94°
}

// 실물 사진: 바코드 상자 가운데 (656, 375) 폭 93 px, 줄자: 바코드 base (0.46, 0.08) 바닥 위 0.27 m
//   pitch 30° (URDF) 로는 높이가 틀리고, +4° 보정이면 라이다 벽 거리 (0.46) 로 높이 27 cm 근처
TEST(BarcodeSearch, WideRayToTargetReal)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.lidar_range_offset_m = 0.0;   // 벽 점을 정확히 줌
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  for (double extra : {0.0, 4.0}) {
    World w = world_at_start();
    w.T_base_wide = wide_urdf(extra);
    w.scan_base = wall_x(0.47);   // 바코드 면 (바코드 x 0.46 + 반 두께)
    Box b;
    b.u = 656;
    b.v = 375;
    b.w = 93;
    b.h = 30;
    b.stamp = 100.0;
    w.wide_barcode = b;
    double vy = 0.0;
    const auto t = s.target_from_wide(w, &vy);
    ASSERT_TRUE(t.has_value());
    std::printf("보정 +%.0f°: 바코드 (%.3f, %.3f) 바닥 위 %.3f m, 보는 방향 %.1f°\n", extra, t->x, t->y, t->z, vy / kDeg);
    if (extra == 4.0) {
      EXPECT_NEAR(t->z, 0.27, 0.04);
      EXPECT_NEAR(t->y, 0.08, 0.05);
      EXPECT_NEAR(vy, 0.0, 3 * kDeg);   // 벽 정면
    }
  }
}

// 라이다 없을 때: 상자 폭 (바코드 8 cm) 으로 거리
TEST(BarcodeSearch, WideRangeFromBoxWidth)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  const V3 truth{0.55, 0.10, 0.25};
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, truth, 0.08);
  const auto t = s.target_from_wide(w, nullptr);
  ASSERT_TRUE(t.has_value());
  EXPECT_LT(norm(*t - truth), 0.04) << t->x << " " << t->y << " " << t->z;
}

// 무작위 바코드 (출발 구역 둘레 벽, 광각 시야 안) → 라이다 벽 거리로 3D: 오차 < 2 cm
TEST(BarcodeSearch, RandomWallBarcodesFromWide)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  const Fisheye f;
  int n = 0, ok = 0;
  for (int i = 0; i < 200 && n < 60; ++i) {
    const double wx = 0.40 + 0.5 * U(rng), y = -0.5 + 1.0 * U(rng), z = 0.10 + 0.45 * U(rng);
    World w = world_at_start();
    const V3 truth{wx, y, z};
    // 광각 시야 안인지 (투영 화면 안)
    Box b = project_box(f, w.T_base_wide, truth, 0.08);
    if (!(b.u > 20 && b.u < f.w - 20 && b.v > 20 && b.v < f.h - 20)) {continue;}
    ++n;
    w.wide_barcode = b;
    w.scan_base = wall_x(wx);
    BarcodeSearch s;
    SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
    p.lidar_range_offset_m = 0.0;
    s.reset(p, f, Pinhole{}, Zone{}, 100.0);
    const auto t = s.target_from_wide(w, nullptr);
    if (t && norm(*t - truth) < 0.02) {++ok;}
  }
  std::printf("무작위 벽 바코드 %d 개 중 2 cm 안 %d\n", n, ok);
  EXPECT_GE(n, 20);
  EXPECT_EQ(ok, n);
}

// 신호등 시야: 앞 0.8 m 신호등 → 지금 자리 통과. 광각이 왼쪽 15.7° 돌아 있어 신호등은 화면 오른쪽 → 왼쪽 돌기 여유가 작음
//   (±25° 는 둘 다 8° 여유 통과, 왼쪽 +40° 는 실패, 90° 는 화면 밖)
TEST(BarcodeSearch, LightInFov)
{
  const Fisheye f;
  const Pose3 T = wide_urdf(4.0);
  const V3 light{0.8, 0.0, 0.30};
  const auto c0 = light_in_fov(f, T, 0, 0, 0, light, 8.0);
  const auto cl = light_in_fov(f, T, 0, 0, 25 * kDeg, light, 8.0);
  const auto cr = light_in_fov(f, T, 0, 0, -25 * kDeg, light, 8.0);
  const auto cb = light_in_fov(f, T, 0, 0, 90 * kDeg, light, 8.0);
  const auto cl40 = light_in_fov(f, T, 0, 0, 40 * kDeg, light, 8.0);
  std::printf("신호등 여유: 지금 %.1f° · 왼 25° %.1f° · 오른 25° %.1f° · 90° %.1f°\n", c0.margin_deg, cl.margin_deg, cr.margin_deg, cb.margin_deg);
  EXPECT_TRUE(c0.ok);
  EXPECT_TRUE(cl.ok);
  EXPECT_LT(cl.margin_deg, c0.margin_deg);
  EXPECT_TRUE(cr.ok);
  EXPECT_FALSE(cl40.ok);
  EXPECT_FALSE(cb.ok);
}

TEST(BarcodeSearch, ScanRangeAndWallYaw)
{
  auto pts = wall_x(0.6);
  EXPECT_NEAR(scan_range(pts, 0, 0, 0, 3 * kDeg, 0.2, 2), 0.6, 0.005);
  EXPECT_NEAR(wall_view_yaw(pts, 0, 0, 0.6, 0.1, 0.15), 0.0, 1e-6);
  // 30° 기운 벽
  std::vector<std::array<double, 2>> q;
  for (double t = -0.5; t <= 0.5; t += 0.01) {q.push_back({0.6 + t * std::sin(30 * kDeg), t * std::cos(30 * kDeg)});}
  EXPECT_NEAR(wall_view_yaw(q, 0, 0, 0.6, 0.0, 0.15), -30 * kDeg, 1e-3);
  EXPECT_TRUE(std::isnan(scan_range({}, 0, 0, 0, 0.1, 0.1, 2)));
}

TEST(BarcodeSearch, ZoneFromArena)
{
  // dice_map_arena6: 출발 구역 434–1034 × 650–1250 mm, map_from_arena [0.5759, 0.8937, -3.2071]
  const Zone z = zone_from_arena(0.434, 1.034, 0.650, 1.250, 0.5759, 0.8937, -3.2071);
  // 출발 (0.734, 0.950) arena → map
  const double c = std::cos(-3.2071), sn = std::sin(-3.2071);
  const double sx = 0.5759 + c * 0.734 - sn * 0.950, sy = 0.8937 + sn * 0.734 + c * 0.950;
  EXPECT_TRUE(in_zone(z, sx, sy, 0.17));
  EXPECT_FALSE(in_zone(z, sx + 0.25, sy, 0.17));   // 반 변 0.3 − 0.17 = 0.13 넘음
  EXPECT_TRUE(in_zone(z, sx + 0.10, sy - 0.10, 0.17));
}

// 흐름: 광각에 보임 → LookAt (정면) → 못 읽음 · 손목 상자 없음 → 흔들기 없이 다음 (돌기 · 훑기)
//       손목 상자는 봤는데 못 읽음 → 흔들기 4 번 (J1 ±4 · J4 ±3, 거리 흔들기 없음)
TEST(BarcodeSearch, FlowWideThenPerturbThenSweep)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.perturbs = {{{4, 0, 0}, {-4, 0, 0}, {0, 3, 0}, {0, -3, 0}}};   // 기본은 없음 → 켜서 시험
  p.lidar_range_offset_m = 0.0;
  p.approach_first = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.47);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.08, 0.27}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_NEAR(a.target.z, 0.27, 0.02);
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  EXPECT_TRUE(a.light);
  // 손목 상자 (겨눔 화소 = 가운데 + (−60, −50), 크게 = 다시 겨눔 없음) → 흔들기
  World wb = w;
  wb.have_wrist_tf = true;
  wb.T_map_wrist.R = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  wb.T_map_wrist.t = a.target - V3{0.15, 0, 0};
  Box b;
  b.u = 261;
  b.v = 198;
  b.w = 320;
  b.h = 100;
  wb.wrist_barcode = b;
  for (int i = 0; i < 4; ++i) {
    a = s.next(i == 0 ? wb : w);
    ASSERT_EQ(a.kind, Act::LookAt) << i << " " << a.note;
    EXPECT_FALSE(a.sweep);
    EXPECT_TRUE(a.dj1_deg != 0 || a.dj4_deg != 0) << i;
    EXPECT_GE(a.standoff, p.standoff_min_m - 1e-9);
  }
  a = s.next(w);   // 같은 광각 목표는 이미 봄 → 돌기 +25 · −25 (신호등 통과)
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(a.byaw, 25 * kDeg, 1e-9);
  a = s.next(w);
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(a.byaw, -25 * kDeg, 1e-9);
  w.wide_barcode.reset();
  a = s.next(w);
  EXPECT_EQ(a.kind, Act::LookAt);
  EXPECT_TRUE(a.sweep) << a.note;
}

// 상자를 한 번도 못 본 목표는 흔들지 않고 바로 다음
TEST(BarcodeSearch, NoWristBoxNoPerturb)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.approach_first = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.47);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.08, 0.27}, 0.08);
  ASSERT_EQ(s.next(w).kind, Act::LookAt);
  const auto a = s.next(w);
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;   // 흔들기 아님 → 돌기
}

// 신호 기록 중 · 신호등 못 봄 → light_wait_s 동안 기다림, 그 뒤엔 광각 바코드만 (돌기 · 훑기 안 함)
TEST(BarcodeSearch, LightUnknownGate)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.approach_first = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.wide_light.reset();
  w.scan_base = wall_x(0.47);
  auto a = s.next(w);
  EXPECT_EQ(a.kind, Act::Dwell) << a.note;
  w.now = 100.0 + p.light_wait_s + 1.0;
  a = s.next(w);   // 광각 바코드 없음 → 돌기 · 훑기 대신 기다림
  EXPECT_EQ(a.kind, Act::Dwell) << a.note;
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.08, 0.27}, 0.08);
  w.wide_barcode->stamp = w.now;
  a = s.next(w);
  EXPECT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_FALSE(s.light_known());
  // 신호 끝 → 제한 풀림 (돌기)
  w.wide_barcode.reset();
  w.signal_done = true;
  a = s.next(w);
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;
}

// 광각 상자를 아직 못 받음 → '광각에 없음' 판단 (돌기) 을 미룸
TEST(BarcodeSearch, WaitForWideFrames)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.wide_ready = false;
  EXPECT_EQ(s.next(w).kind, Act::Dwell);
  w.wide_ready = true;
  EXPECT_EQ(s.next(w).kind, Act::MoveBase);
}

// 닿지 않음 + 신호등 조건으로 몸을 못 옮김 → 버리지 않고 미룸 → 신호 끝 뒤 다시
TEST(BarcodeSearch, LightBlockedMoveDeferred)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.approach_first = false;
  p.rotate_steps_deg.clear();
  p.sweep_az_deg.clear();
  p.light_margin_deg = 80.0;   // 어떤 자리도 신호등 조건 실패
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.9);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.9, 0.1, 0.3}, 0.08);
  ASSERT_EQ(s.next(w).kind, Act::LookAt);
  s.on_look_failed("LOOK_AT[REACH] 거리 0.300 m 가 바란 0.150 ± 0.030 밖", w);
  w.wide_barcode.reset();
  auto a = s.next(w);
  EXPECT_EQ(a.kind, Act::Dwell) << a.note;   // 미룬 것만 남음
  w.signal_done = true;
  a = s.next(w);
  EXPECT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.target.x, 0.9, 0.03);
}

// 손목 깊이 점이 벽 면에서 멀면 (D435 최소 거리 · 막대 겹침) 안 쓰고 광선 ↔ 면
TEST(BarcodeSearch, ImplausibleDepthIgnored)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.approach_first = false;
  p.rotate_steps_deg.clear();
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.6);
  auto a = s.next(w);
  ASSERT_TRUE(a.sweep);
  w.have_wrist_tf = true;
  w.T_map_wrist.R = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  w.T_map_wrist.t = a.target - V3{0.30, 0, 0};
  Box b;
  b.u = 320;
  b.v = 240;
  b.w = 60;
  b.h = 20;
  w.wrist_barcode = b;
  w.wrist_barcode_map = V3{a.target.x - 0.20, a.target.y, a.target.z};   // 벽보다 20 cm 앞 = 틀린 깊이
  const auto a2 = s.next(w);
  ASSERT_EQ(a2.kind, Act::LookAt);
  EXPECT_NEAR(a2.target.x, a.target.x, 0.02);   // 깊이 대신 벽 면
}

// 훑기 중 손목 상자 (가운데에서 멂) → 다시 겨눔 (훑기 아님, standoff 기본)
TEST(BarcodeSearch, SweepThenWristRefine)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.rotate_steps_deg.clear();
  p.approach_first = false;   // 다시 겨눔 계산만 (몸 다가가기는 ApproachFirstThenCloseLook)
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.6);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  ASSERT_TRUE(a.sweep);
  // 손목 카메라: 목표에서 0.35 m 뒤, 앞 (+x) 을 봄. 광학: z = +x, x = −y, y = −z
  w.have_wrist_tf = true;
  w.T_map_wrist.R = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  w.T_map_wrist.t = a.target - V3{0.35, 0, 0};
  Box b;
  b.u = 500;
  b.v = 240;
  b.w = 60;
  b.h = 20;
  w.wrist_barcode = b;
  const auto a2 = s.next(w);
  ASSERT_EQ(a2.kind, Act::LookAt);
  EXPECT_FALSE(a2.sweep);
  EXPECT_NEAR(a2.target.x, a.target.x, 0.02);   // 같은 벽 면
  EXPECT_LT(a2.target.y, a.target.y - 0.05);    // 화면 오른쪽 = 로봇 오른쪽 (−y)
  // 손목 깊이 점이 있으면 그대로 씀
  w.wrist_barcode_map = V3{0.61, -0.2, 0.3};
  const auto a3 = s.next(w);
  ASSERT_EQ(a3.kind, Act::LookAt);
  EXPECT_LT(norm(a3.target - V3{0.61, -0.2, 0.3}), 1e-9);
}

// 닿지 않음 → 구역 안 몸 옮기기 → 같은 목표 다시. 신호등 가림 → 미룸 → 신호 기록 끝 뒤
TEST(BarcodeSearch, ReachMoveAndOcclusionDefer)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.approach_first = false;
  p.approach_dist_m = 0.55;
  p.standoff_max_m = 0.35;
  p.rotate_steps_deg.clear();
  p.sweep_az_deg = {0.0};
  p.sweep_heights_m = {0.3};
  p.lidar_range_offset_m = 0.0;
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.9);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.9, 0.1, 0.3}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  s.on_look_failed("LOOK_AT: 거리 0.520 m 가 [0.20, 0.35] 밖", w);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << s.last_note();
  EXPECT_TRUE(in_zone(z, a.bx, a.by, p.robot_half_m - 1e-9));
  EXPECT_GT(a.bx, 0.05);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_NEAR(a.target.x, 0.9, 0.03);
  // 이번엔 신호등 가림 → 미룸 → 훑기 1 → (흔들기 없음) → 기다림 → 신호 끝 → 미룬 목표
  s.on_look_failed("LOOK_AT: 손끝 이 광각 → 신호등 시야를 가림 (3.0°)", w);
  w.wide_barcode.reset();
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_TRUE(a.sweep);
  a = s.next(w);   // 훑기 뒤 상자 없음 → 미룬 것만 → 기다림
  EXPECT_EQ(a.kind, Act::Dwell) << s.last_note();
  w.signal_done = true;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_FALSE(a.light);
  EXPECT_NEAR(a.target.x, 0.9, 0.03);
}

TEST(BarcodeSearch, GiveUpAfterMaxTime)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.now = 100.0 + p.max_s + 1.0;
  EXPECT_EQ(s.next(w).kind, Act::GiveUp);
}

TEST(BarcodeSearch, StandoffForPpm)
{
  // 8 cm · 90 모듈, 0.35 m 에서 fx 606 → 상자 138 px = 1.54 px/모듈 → 2 px/모듈 거리 0.27 m
  const double w = 606.3 * 0.08 / 0.35;
  EXPECT_NEAR(standoff_for_ppm(w, 90, 0.35, 2.0), 0.269, 0.003);
}

// 바코드가 0.46 m 앞 → LOOK_AT 전에 몸을 0.38 m 까지 (출발 구역 안) → 같은 목표를 0.16 m 에서
TEST(BarcodeSearch, ApproachFirstThenCloseLook)
{
  BarcodeSearch s;
  SearchParams p;
  p.two_step = false;   // 한 걸음 흐름 (0.15 m 먼저) 시험 — 두 걸음은 TwoStepLook
  p.lidar_range_offset_m = 0.0;
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.47);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.08, 0.27}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(std::hypot(0.46 - a.bx, 0.08 - a.by), 0.38, 0.03);
  EXPECT_TRUE(in_zone(z, a.bx, a.by, p.robot_half_m - 1e-9));
  w.rx = a.bx;
  w.ry = a.by;
  w.ryaw = a.byaw;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  EXPECT_NEAR(a.target.x, 0.46, 0.03);
}

// 두 걸음: 광각 → 먼 보기 (0.28, 넓게) → 손목 라벨 상자 → 한 번 0.15 m (가운데) → 못 읽고 가운데서 멂 → 한 번 다시 가운데 = LOOK_AT 3
TEST(BarcodeSearch, TwoStepLook)
{
  BarcodeSearch s;
  SearchParams p;
  p.approach_first = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.40);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.39, 0.09, 0.21}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_TRUE(a.sweep);   // 먼 보기 = 훑기처럼 (vpitch 20 · 짧은 머묾)
  EXPECT_NEAR(a.standoff, 0.28, 1e-9);
  // 먼 보기에서 라벨 상자 (가운데에서 오른쪽 아래, 작음 1.8 px/모듈)
  World wb = w;
  wb.have_wrist_tf = true;
  wb.T_map_wrist.R = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};
  wb.T_map_wrist.t = a.target - V3{0.28, 0, 0};
  Box b;
  b.u = 420;
  b.v = 300;
  b.w = 160;
  b.h = 55;
  wb.wrist_barcode = b;
  wb.wrist_barcode_map = V3{0.39, 0.012, 0.183};   // 손목 깊이 (그 벽 면 위)
  a = s.next(wb);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_FALSE(a.sweep);
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  EXPECT_LT(norm(a.target - V3{0.39, 0.012, 0.183}), 1e-9);
  // 0.15 m 보기: 못 읽음, 상자가 가운데에서 100 px → 한 번 다시 가운데
  World wc = wb;
  wc.T_map_wrist.t = a.target - V3{0.15, 0, 0};
  wc.wrist_barcode->u = 420;
  wc.wrist_barcode->v = 248;
  wc.wrist_barcode->w = 300;
  wc.wrist_barcode_map = V3{0.39, -0.01, 0.185};
  a = s.next(wc);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  EXPECT_EQ(s.looks(), 3);
  // 그래도 못 읽음 → (흔들기 기본 없음) 이 목표 끝 → 광각 목표는 이미 봄 → 돌기
  a = s.next(wc);
  EXPECT_NE(a.kind, Act::LookAt) << a.note;
}

// 먼 보기에서 아무것도 없음 → 이 목표 끝 (0.15 m 안 감) → 돌기 · 훑기. 먼 보기 [REACH] 거절 → 바로 0.15 m
TEST(BarcodeSearch, TwoStepFallbacks)
{
  BarcodeSearch s;
  SearchParams p;
  p.approach_first = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.40);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.39, 0.09, 0.21}, 0.08);
  auto a = s.next(w);
  ASSERT_TRUE(a.sweep);
  a = s.next(w);   // 상자 없음
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;
  BarcodeSearch s2;
  s2.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  a = s2.next(w);
  ASSERT_TRUE(a.sweep);
  s2.on_look_failed("LOOK_AT[REACH] J2 토크 1.28 N·m > 1.25", w);
  a = s2.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_FALSE(a.sweep);
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
}

// bc_wide_yaw_offset_deg: + = 왼쪽 → 같은 화소의 추정이 왼쪽 (y +)
TEST(BarcodeSearch, WideYawOffset)
{
  const Pose3 T = pitch_optical(Pose3{}, 0.0);
  const V3 z0 = rotate(yaw_optical(T, 0.0), {0, 0, 1});
  const V3 zl = rotate(yaw_optical(T, 10 * kDeg), {0, 0, 1});
  EXPECT_NEAR(z0.x, 0.0, 1e-12);
  EXPECT_NEAR(zl.x, -std::sin(10 * kDeg), 1e-12);   // 광학 x = 오른쪽 → − = 왼쪽
  // 실제 로봇 광각에서: +12° 면 같은 상자의 바코드 추정 y 가 커짐
  BarcodeSearch s;
  SearchParams p;
  p.lidar_range_offset_m = 0.0;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.47);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.0, 0.27}, 0.08);
  const auto t0 = s.target_from_wide(w, nullptr);
  w.T_base_wide = yaw_optical(w.T_base_wide, 12 * kDeg);
  const auto t1 = s.target_from_wide(w, nullptr);
  ASSERT_TRUE(t0 && t1);
  EXPECT_GT(t1->y, t0->y + 0.05);
}

// 다가가기 뒤에도 먼 보기부터
TEST(BarcodeSearch, ApproachThenLocate)
{
  BarcodeSearch s;
  SearchParams p;
  p.lidar_range_offset_m = 0.0;
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.47);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.46, 0.08, 0.27}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  w.rx = a.bx;
  w.ry = a.by;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt);
  EXPECT_TRUE(a.sweep);
  EXPECT_NEAR(a.standoff, 0.28, 1e-9);
}

// 몸 멈춤 (몸을 돌리거나 옮기면 신호등이 광각 구석으로 가 12 점멸을 못 채울 수 있음)
//   신호 기록 중 (hold_base): 다가가기 · 몸 옮기기 · 돌기 없음, 팔만 (광각 목표 LOOK_AT · 훑기), 닿지 않으면 미룸 → 기다림.
//   기다린 시간은 max_s 에서 뺌. 기록 끝 → 미룬 목표 다가가기 (MoveBase) → 같은 목표 LOOK_AT → (못 읽으면) 남겨 둔 돌기
TEST(BarcodeSearch, HoldBaseWhileSignalRecording)
{
  BarcodeSearch s;
  SearchParams p;   // 기본 (두 걸음 · 다가가기 · 돌기 ±25 · 훑기 7 방위)
  p.lidar_range_offset_m = 0.0;
  p.hold_max_s = 1000.0;   // 상한은 HoldBaseCapReleases
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.hold_base = true;
  w.scan_base = wall_x(0.62);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.61, 0.05, 0.27}, 0.08);   // 멂 → 몸 멈춤이 없으면 먼저 몸 옮김
  int looks = 0, sweeps = 0, waits = 0;
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;   // 다가가지 않고 지금 자리에서 먼 보기
  EXPECT_TRUE(a.light);
  EXPECT_NEAR(a.standoff, 0.28, 1e-9);
  s.on_look_failed("LOOK_AT[REACH] J2 토크 1.40 N·m > 1.25", w);   // 먼 보기 안 됨 → 0.15 m
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  s.on_look_failed("LOOK_AT[REACH] 관절 · 틱 범위 밖", w);       // 몸 멈춤 없으면 몸 옮김 → 멈춤 중이라 미룸
  EXPECT_NE(s.last_note().find("몸 멈춤"), std::string::npos) << s.last_note();
  // 기록이 길어도 (max_s 의 3 배) 몸은 안 움직이고 포기도 안 함 (기다린 시간은 max_s 에서 뺌)
  for (int i = 0; i < 200 && w.now < 100.0 + 3 * p.max_s; ++i) {
    w.now += 1.0;
    a = s.next(w);
    ASSERT_NE(a.kind, Act::MoveBase) << i << " " << a.note;
    ASSERT_NE(a.kind, Act::GiveUp) << i << " " << a.note;
    if (a.kind == Act::LookAt) {++looks; if (a.sweep) {++sweeps;} EXPECT_TRUE(a.light);} else {++waits; w.now += a.dwell_s;}
  }
  EXPECT_EQ(sweeps, static_cast<int>(p.sweep_az_deg.size())) << "훑기는 팔만이라 함";
  EXPECT_GT(waits, 10);
  EXPECT_GT(s.held_s(), p.max_s);
  EXPECT_LT(s.search_s(w.now), p.max_s);
  // 기록 끝 → 미룬 목표: 다가가기 → 그 목표 LOOK_AT
  w.hold_base = false;
  w.signal_done = true;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_GT(a.bx, 0.05);   // 바코드 쪽으로 (출발 구역 안까지)
  EXPECT_TRUE(in_zone(z, a.bx, a.by, p.robot_half_m - 1e-9));
  w.rx = a.bx;
  w.ry = a.by;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.target.x, 0.61, 0.03);
  EXPECT_FALSE(a.light);
  // 못 읽음 (상자 없음) → 남겨 둔 돌기 +25 (광각 새 상자 없음)
  w.wide_barcode.reset();
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NE(a.note.find("제자리"), std::string::npos) << a.note;
}

// 기록 중 광각에 바코드 없음: 제자리 돌기 대신 훑기 (팔) 먼저, 돌기는 기록 끝 뒤 그대로 (안 씀 · 안 버림)
TEST(BarcodeSearch, HoldBaseRotationKeptForLater)
{
  BarcodeSearch s;
  SearchParams p;
  p.sweep_az_deg = {0.0, 35.0};
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.hold_base = true;
  w.scan_base = wall_x(0.5);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_TRUE(a.sweep);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_TRUE(a.sweep);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::Dwell) << a.note;   // 돌기 2 남음 → 기록 끝 기다림
  EXPECT_NE(a.note.find("돌기 2"), std::string::npos) << a.note;
  w.hold_base = false;   // 끝남 (Done · 불완전 끝 · 시간 초과 — signal_done 은 아직 거짓이어도 몸 멈춤만 풀림)
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(a.byaw, 25 * kDeg, 1e-9);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(a.byaw, -25 * kDeg, 1e-9);
}

// 몸 멈춤 기다림 합이 hold_max_s (mission signal_wait_s) 를 넘으면 멈춤 풀림 → 몸 멈춤 없는 흐름 (신호등 시야 조건 안 돌기)
TEST(BarcodeSearch, HoldBaseCapReleases)
{
  BarcodeSearch s;
  SearchParams p;
  p.sweep_az_deg.clear();
  p.hold_max_s = 10.0;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.hold_base = true;
  int waits = 0;
  Action a;
  for (int i = 0; i < 30; ++i) {
    a = s.next(w);
    if (a.kind != Act::Dwell) {break;}
    ++waits;
    w.now += a.dwell_s;
  }
  EXPECT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_FALSE(s.holding(w));
  EXPECT_GE(waits, 5);
  EXPECT_NEAR(s.held_s(), 10.0, 2.01);
}

// hold_base 거짓 (신호 끝 · signal_fixed · 기록 안 함) = 몸 멈춤 없는 흐름: 멀면 먼저 몸 옮김
TEST(BarcodeSearch, NoHoldUnchanged)
{
  BarcodeSearch s;
  SearchParams p;
  p.lidar_range_offset_m = 0.0;
  const Zone z = zone_from_arena(-0.3, 0.3, -0.3, 0.3, 0, 0, 0);
  s.reset(p, Fisheye{}, Pinhole{}, z, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.62);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.61, 0.05, 0.27}, 0.08);
  EXPECT_EQ(s.next(w).kind, Act::MoveBase);
}

// ================================================================ 벽 판 하나 (폭 36 · 높이 39 cm)
//   시험 자리: map = base (출발 0, 0, 0, +x 를 봄), 벽 = base x wx, 판 가운데 (0.55 · 라이다로 맞춤, 0, 높이 0.20), 법선 −x (π)
namespace
{
SearchParams panel_params()
{
  SearchParams p;
  p.lidar_range_offset_m = 0.0;
  p.panel = true;
  p.panel_c = {0.55, 0.0, 0.20};   // 지도 맞춤이 몇 cm 틀려도 라이다 벽으로 맞춤 (panel_depth_tol 0.20 안)
  p.panel_n_yaw = M_PI;
  return p;
}
}  // namespace

// 판 가운데는 라이다 벽 면으로 맞추고, 광각 상자는 판 (+ 여유 8 cm) 안 목표만
TEST(BarcodeSearch, PanelCenterAndWideAcceptance)
{
  BarcodeSearch s;
  const SearchParams p = panel_params();
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.scan_base = wall_x(0.65);
  const V3 c = s.panel_center(w);
  EXPECT_NEAR(c.x, 0.65, 0.01);
  EXPECT_NEAR(c.y, 0.0, 1e-9);
  {   // 라이다 벽이 지도 판보다 훨씬 가까움 (지도 0.55, 벽 0.30) → 늘 라이다 (판 뒤 벽 속을 안 봄). 훨씬 멂 (0.90) → 지도 그대로
    World w2 = w;
    w2.scan_base = wall_x(0.30);
    EXPECT_NEAR(s.panel_center(w2).x, 0.30, 0.01);
    w2.scan_base = wall_x(0.90);
    EXPECT_NEAR(s.panel_center(w2).x, 0.55, 1e-9);
  }
  double lat = 0.0;
  EXPECT_TRUE(s.in_panel({0.65, 0.20, 0.30}, c, &lat));
  EXPECT_NEAR(lat, 0.20, 1e-9);   // 왼 + (map +y)
  EXPECT_FALSE(s.in_panel({0.65, -0.30, 0.20}, c));
  EXPECT_FALSE(s.in_panel({0.65, 0.0, 0.50}, c));
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.65, 0.10, 0.18}, 0.08);   // 판 위
  auto t = s.target_from_wide(w, nullptr);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(t->y, 0.10, 0.02);
  EXPECT_NEAR(t->z, 0.18, 0.02);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.65, -0.45, 0.20}, 0.08);  // 옆 벽 (판 밖)
  EXPECT_FALSE(s.target_from_wide(w, nullptr).has_value());
  EXPECT_EQ(s.panel_rejects(), 1);
  // 라이다가 없으면 판 면 (지도 0.55) 과 광선
  w.scan_base.clear();
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.55, 0.05, 0.20}, 0.08);
  t = s.target_from_wide(w, nullptr);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(t->x, 0.55, 0.005);
  EXPECT_NEAR(t->y, 0.05, 0.01);
}

// 광각에 없음 (신호 끝, 판이 가까움): 돌기 없이 판 따라 팔만 훑기 (0, −0.12, +0.12), 그 뒤에도 몸은 안 움직임
TEST(BarcodeSearch, PanelSweepNoRotation)
{
  BarcodeSearch s;
  const SearchParams p = panel_params();
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.signal_done = true;
  w.scan_base = wall_x(0.40);
  std::vector<double> ys;
  for (int i = 0; i < 80; ++i) {
    w.now += 1.0;
    const auto a = s.next(w);
    ASSERT_NE(a.kind, Act::MoveBase) << i << " " << a.note;
    if (a.kind == Act::GiveUp) {break;}
    if (a.kind == Act::LookAt) {
      EXPECT_TRUE(a.sweep);
      EXPECT_NEAR(a.target.x, 0.40, 0.01);
      EXPECT_NEAR(a.target.z, 0.20, 1e-9);
      EXPECT_NEAR(std::fabs(a.view_yaw), 0.0, 1e-9);   // 판 정면 (+x)
      if (ys.size() < 3) {ys.push_back(a.target.y);}
    } else {
      w.now += a.dwell_s;
    }
  }
  ASSERT_EQ(ys.size(), 3u);
  EXPECT_NEAR(ys[0], 0.0, 1e-9);
  EXPECT_NEAR(ys[1], -0.12, 1e-9);
  EXPECT_NEAR(ys[2], 0.12, 1e-9);
}

// 판이 멂 (출발 자세, 벽 0.65): 판 정면 approach_dist 자리로 앞으로만 (방향 그대로) → 그 자리에서 판 훑기
TEST(BarcodeSearch, PanelFarNudgeForwardThenSweep)
{
  BarcodeSearch s;
  const SearchParams p = panel_params();
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.signal_done = true;
  w.ryaw = 2.0 * kDeg;
  w.scan_base = wall_x(0.65);   // (2° 는 벽 거리에 거의 영향 없음)
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(a.bx, 0.65 - p.approach_dist_m, 0.02);
  EXPECT_NEAR(a.by, 0.0, 0.03);
  EXPECT_NEAR(a.byaw, 2.0 * kDeg, 1e-9);   // 돌지 않음
  w.rx = a.bx;
  w.ry = a.by;
  w.ryaw = 0.0;
  w.scan_base = wall_x(0.65 - a.bx);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_TRUE(a.sweep);
  EXPECT_NEAR(a.target.x, 0.65, 0.01);
}

// 광각 목표로 다가갔는데 4 cm 덜 감 → 0.15 m 보기 [REACH] (J2 1.45) → renudge 로 한 번 더 작게
TEST(BarcodeSearch, PanelRenudgeAfterShortMove)
{
  BarcodeSearch s;
  const SearchParams p = panel_params();
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.signal_done = true;
  w.scan_base = wall_x(0.62);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.62, 0.03, 0.18}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;   // 다가가기 (판 정면: 옆으로도 3 cm)
  EXPECT_NEAR(a.bx, 0.62 - p.approach_dist_m, 0.02);
  EXPECT_NEAR(a.by, 0.03, 0.02);
  w.rx = a.bx - 0.04;   // 4 cm 덜 감
  w.ry = a.by;
  w.scan_base = wall_x(0.62 - w.rx);
  w.wide_barcode.reset();   // 팔 · 몸 움직임 뒤 새 광각 상자 없음
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.standoff, 0.28, 1e-9);
  s.on_look_failed("LOOK_AT[REACH] J2 토크 1.30 N·m > 1.25", w);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  s.on_look_failed("LOOK_AT[REACH] J2 토크 1.45 N·m > 1.25 (팔을 너무 뻗음 — 몸을 가까이)", w);
  EXPECT_NE(s.last_note().find("옮긴 뒤에도"), std::string::npos) << s.last_note();
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  EXPECT_NEAR(0.62 - a.bx, p.approach_dist_m, 0.02);   // 바코드에서 approach_dist 자리
  EXPECT_GT(a.bx, w.rx + 0.02);
  EXPECT_LT(a.bx - w.rx, 0.10);   // 작게
  EXPECT_NEAR(a.byaw, 0.0, 1e-9);
  w.rx = a.bx;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;   // 같은 목표 0.15 m
  EXPECT_NEAR(a.standoff, 0.15, 1e-9);
  EXPECT_NEAR(a.target.y, 0.03, 0.02);
  // 관절 범위 거절 (너무 가까울 수도) 은 renudge 안 함 (base_moves_ 와 상관없이 같은 판정) — 아래는 2 번 다 써서도 버림
  // 또 안 닿음 → max_base_moves (2) 다 씀 → 더 안 옮김
  s.on_look_failed("LOOK_AT[REACH] J2 토크 1.30 N·m > 1.25", w);
  EXPECT_NE(s.last_note().find("목표 버림"), std::string::npos) << s.last_note();
}

// 신호 기록 중 (몸 멈춤): 판이 멀어도 팔만 판 훑기 → 닿지 않으면 미룸 → 기다림. 기록 끝 → 판 앞으로 작게 → 판 훑기
TEST(BarcodeSearch, PanelHoldThenNudge)
{
  BarcodeSearch s;
  SearchParams p = panel_params();
  p.hold_max_s = 1000.0;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.hold_base = true;
  w.scan_base = wall_x(0.65);
  int looks = 0;
  Action a;
  for (int i = 0; i < 40; ++i) {
    w.now += 0.5;
    a = s.next(w);
    ASSERT_NE(a.kind, Act::MoveBase) << i << " " << a.note;
    ASSERT_NE(a.kind, Act::GiveUp) << i << " " << a.note;
    if (a.kind == Act::LookAt) {
      ++looks;
      EXPECT_TRUE(a.sweep);
      s.on_look_failed("LOOK_AT[REACH] J2 토크 1.60 N·m > 1.25", w);
      EXPECT_NE(s.last_note().find("몸 멈춤"), std::string::npos) << s.last_note();
    } else {
      w.now += a.dwell_s;
    }
  }
  EXPECT_EQ(looks, 3);   // 판 훑기 3 자리 모두 팔로 해 봄
  EXPECT_EQ(a.kind, Act::Dwell);
  EXPECT_NE(a.note.find("미룬 목표 3"), std::string::npos) << a.note;
  EXPECT_NE(a.note.find("돌기 0"), std::string::npos) << a.note;
  w.hold_base = false;
  w.signal_done = true;
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;   // 미룬 판 가운데 → 판 앞으로
  EXPECT_NEAR(a.bx, 0.65 - p.approach_dist_m, 0.02);
  EXPECT_NEAR(a.by, 0.0, 0.02);
  EXPECT_NEAR(a.byaw, 0.0, 1e-9);
  w.rx = a.bx;
  w.scan_base = wall_x(0.65 - a.bx);
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_TRUE(a.sweep);
  EXPECT_NEAR(a.target.y, 0.0, 0.01);
  a = s.next(w);   // 못 읽음 (손목 상자 없음 → 다음): 다음 미룬 자리 (−0.12) 는 몸 안 옮기고 팔만
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  EXPECT_NEAR(a.target.y, -0.12, 0.01);
}

// 옮긴 뒤 [REACH] 가 J2 토크가 아니면 (관절 범위 · 훑기) 더 가까이 가지 않음 (너무 가까워 거절일 수 있음)
TEST(BarcodeSearch, PanelNoRenudgeOnJointRange)
{
  BarcodeSearch s;
  SearchParams p = panel_params();
  p.two_step = false;
  s.reset(p, Fisheye{}, Pinhole{}, Zone{}, 100.0);
  World w = world_at_start();
  w.signal_done = true;
  w.scan_base = wall_x(0.62);
  w.wide_barcode = project_box(Fisheye{}, w.T_base_wide, {0.62, 0.0, 0.20}, 0.08);
  auto a = s.next(w);
  ASSERT_EQ(a.kind, Act::MoveBase) << a.note;
  w.rx = a.bx - 0.04;
  w.scan_base = wall_x(0.62 - w.rx);
  w.wide_barcode.reset();
  a = s.next(w);
  ASSERT_EQ(a.kind, Act::LookAt) << a.note;
  s.on_look_failed("LOOK_AT[REACH] 관절 · 틱 범위 밖", w);
  EXPECT_NE(s.last_note().find("목표 버림"), std::string::npos) << s.last_note();
}
