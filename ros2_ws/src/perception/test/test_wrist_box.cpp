// test_wrist_box.cpp — 박스 윗면 중심 · 방향 시험: 광선 추적 깊이 (잡음 · 구멍) 로 오차 · 실패 판정
//   카메라 = feat/moveit-config URDF 의 LOOK 자세 (mock control_master, 처짐 보정 TF) → 박스 앞쪽 절반만 보임
//   실행: colcon build → ./build/perception/test_wrist_box

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "perception/wrist_box.hpp"

using namespace perception::wrist;

namespace
{
constexpr int W = 640, H = 480;
const Intrinsics K{605.9, 605.4, 324.0, 245.9};

// 바닥 프레임 ← 카메라 광학 (LOOK, 카메라 차체 앞 0.227 m · 높이 0.322 m, 거의 수직)
Mat3 cam_R()
{
  Mat3 R;
  R << -0.025177, -0.988666, -0.148005,
       -0.999353, 0.028696, -0.021691,
       0.025692, 0.147363, -0.988749;
  return R;
}
const Vec3 CAM_T(0.226527, 0.018615, 0.321958);

// 새 LOOK 후보 (박스 전체가 보임, 순기구학 탐색): 박스 중심 0.2575 m (차체 앞 0.145 와 틈 10 mm) 기준
//   관절 [0.0078, -0.0942, -0.731, 0.3447, -0.0386], 카메라 높이 0.373 m, 광축 수직에서 4.8°
Mat3 new_R()
{
  Mat3 R;
  R << -0.001179, -0.996457, 0.084095,
       -0.999996, 0.000951, -0.002756,
       0.002666, -0.084098, -0.996454;
  return R;
}
const Vec3 NEW_T(0.226006, 0.034632, 0.372756);

struct Puck {double x, y, h;};   // 박스 좌표 중심, 높이
struct Scene
{
  double cx{0.2935}, cy{0.0}, yaw{0.0}, size{0.205}, top{0.07};
  std::vector<Puck> pucks;
  bool box{true};
};

// 축 맞춘 상자와 광선 (o + t d) 교차 → 들어가는 t, 없으면 inf
double slab(const Vec3 & o, const Vec3 & d, const Vec3 & lo, const Vec3 & hi)
{
  double t0 = 0.0, t1 = 1e9;
  for (int k = 0; k < 3; ++k) {
    if (std::fabs(d(k)) < 1e-12) {
      if (o(k) < lo(k) || o(k) > hi(k)) {return INFINITY;}
      continue;
    }
    double a = (lo(k) - o(k)) / d(k), b = (hi(k) - o(k)) / d(k);
    if (a > b) {std::swap(a, b);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
    if (t0 > t1) {return INFINITY;}
  }
  return t0 > 0 ? t0 : INFINITY;
}

cv::Mat1f render(const Scene & sc, double noise, double holes, unsigned seed, const Mat3 & R = cam_R(), const Vec3 & T = CAM_T)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> nd(0.0, noise);
  std::uniform_real_distribution<double> ud(0.0, 1.0);
  const double co = std::cos(-sc.yaw), si = std::sin(-sc.yaw);
  Mat3 Rb;   // 바닥 → 박스 좌표
  Rb << co, -si, 0, si, co, 0, 0, 0, 1;
  const Vec3 ob = Rb * (T - Vec3(sc.cx, sc.cy, 0));
  cv::Mat1f depth(H, W, 0.0f);
  for (int v = 0; v < H; ++v) {
    for (int u = 0; u < W; ++u) {
      const Vec3 dc((u + 0.5 - K.cx) / K.fx, (v + 0.5 - K.cy) / K.fy, 1.0);   // 카메라 z = 1
      const Vec3 d = Rb * (R * dc);
      double best = d.z() < 0 ? -ob.z() / d.z() : INFINITY;   // 바닥
      if (sc.box) {
        const double h = sc.size / 2;
        best = std::min(best, slab(ob, d, Vec3(-h, -h, 0), Vec3(h, h, sc.top)));
        for (const auto & pk : sc.pucks) {
          best = std::min(best, slab(ob, d, Vec3(pk.x - 0.015, pk.y - 0.015, sc.top),
            Vec3(pk.x + 0.015, pk.y + 0.015, sc.top + pk.h)));
        }
      }
      if (!std::isfinite(best) || ud(rng) < holes) {continue;}
      depth(v, u) = static_cast<float>(best + nd(rng));
    }
  }
  return depth;
}

BoxFit fit(const Scene & sc, double noise = 0.001, double holes = 0.03, unsigned seed = 1)
{
  return fit_place_box(render(sc, noise, holes, seed), K, cam_R(), CAM_T, BoxParams{}, Vec3(0.2935, 0.0, 0.0));
}

double ang_deg(double a) {return std::fabs(std::remainder(a, 2 * M_PI)) * 180.0 / M_PI;}
}  // namespace

TEST(WristBox, FoldQuarter)
{
  EXPECT_NEAR(fold_quarter(0.1, 0.0), 0.1, 1e-12);
  EXPECT_NEAR(fold_quarter(M_PI / 2 + 0.1, 0.0), 0.1, 1e-12);
  EXPECT_NEAR(fold_quarter(-M_PI + 0.05, 0.0), 0.05, 1e-12);
  EXPECT_NEAR(fold_quarter(0.3, M_PI / 2), M_PI / 2 + 0.3, 1e-12);
}

// 정 위치 · 빈 박스: 앞 변 · 양옆이 보여 중심이 정해짐 (뒤 변은 화면 밖)
TEST(WristBox, CenteredEmptyBox)
{
  const auto r = fit(Scene{});
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_NEAR(r.x, 0.2935, 0.003);
  EXPECT_NEAR(r.y, 0.0, 0.003);
  EXPECT_LT(ang_deg(r.yaw), 0.5);
  EXPECT_GE(r.side_pts[0], 40);   // 앞
  EXPECT_LT(r.side_pts[1], 40);   // 뒤 = 화면 밖
}

// 밀리고 돈 박스 + 눈 칸 퍽 (퍽이 가린 경계는 테두리로 안 셈)
TEST(WristBox, ShiftedRotatedWithPucks)
{
  for (const double yaw_deg : {-9.0, -4.0, 3.0, 8.0}) {
    Scene sc;
    sc.cx = 0.2935 + 0.004;
    sc.cy = yaw_deg > 0 ? 0.022 : -0.018;
    sc.yaw = yaw_deg * M_PI / 180.0;
    sc.pucks = {{-0.06, -0.06, 0.06}, {-0.06, 0.06, 0.06}, {0.0, 0.0, 0.03}, {0.06, -0.06, 0.06}, {0.06, 0.06, 0.06}};
    const auto r = fit(sc, 0.001, 0.03, static_cast<unsigned>(10 + yaw_deg));
    ASSERT_TRUE(r.ok) << yaw_deg << ": " << r.why;
    EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.003) << yaw_deg;
    EXPECT_LT(ang_deg(r.yaw - sc.yaw), 0.5) << yaw_deg;
  }
}

// 시작 박스 퍽 2 개 (가운데 ±40 mm)
TEST(WristBox, StartBoxTwoPucks)
{
  Scene sc;
  sc.cy = 0.01;
  sc.pucks = {{0.0, -0.04, 0.06}, {0.0, 0.04, 0.06}};
  const auto r = fit(sc);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.003);
}

// 박스가 차체 쪽으로 2.8 cm 밀림 → 앞 변이 차체에 가려 틀린 답 대신 실패
TEST(WristBox, BoxAgainstBodyRejected)
{
  Scene sc;
  sc.cx = 0.2935 - 0.028;
  const auto r = fit(sc);
  EXPECT_FALSE(r.ok);
}

// 박스 없음 (바닥만) → 실패
TEST(WristBox, NoBox)
{
  Scene sc;
  sc.box = false;
  const auto r = fit(sc);
  EXPECT_FALSE(r.ok);
}

// 새 LOOK · 틈 10 mm: 앞 변은 차체 · 좁은 틈에 가려도 뒤 변 · 양옆으로 정해짐 (앞 변 검사 안 함)
TEST(WristBox, NewLookFullBoxNearStop)
{
  for (const double yaw_deg : {-8.0, 0.0, 6.0}) {
    Scene sc;
    sc.cx = 0.2575;
    sc.cy = yaw_deg * 0.002;
    sc.yaw = yaw_deg * M_PI / 180.0;
    sc.pucks = {{-0.06, -0.06, 0.06}, {0.06, 0.06, 0.06}, {0.06, -0.06, 0.06}, {0.0, 0.06, 0.06}};
    const auto r = fit_place_box(render(sc, 0.001, 0.03, 7, new_R(), NEW_T), K, new_R(), NEW_T, BoxParams{},
      Vec3(0.2575, 0.0, 0.0));
    ASSERT_TRUE(r.ok) << yaw_deg << ": " << r.why;
    EXPECT_GE(r.side_pts[1], 40);   // 뒤 변 보임
    EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.003) << yaw_deg;
    EXPECT_LT(ang_deg(r.yaw - sc.yaw), 0.5) << yaw_deg;
  }
}

// 넓게 찾기 (box.search): 처음 추정 (정차 0.2575 m 앞) 에서 10–15 cm 어긋나고 돈 시작 박스 (퍽 2 개)
//   search 0 = 변 부족 또는 변 하나 건너 뒤집힌 답, search 켬 = 맞는 답 (2 cm · 3° 안)
TEST(WristBox, WideSearchShiftedRotated)
{
  struct C {double dx, dy, yaw_deg;};
  for (const C c : {C{0.10, 0.0, 0.0}, C{0.0, 0.10, 20.0}, C{-0.0707, 0.0707, 45.0}, C{0.0, -0.10, 45.0}, C{0.06, 0.06, 30.0}}) {
    Scene sc;
    sc.cx = 0.2575 + c.dx;
    sc.cy = c.dy;
    sc.yaw = c.yaw_deg * M_PI / 180.0;
    sc.pucks = {{0.04, 0.0, 0.06}, {-0.04, 0.0, 0.06}};
    const auto depth = render(sc, 0.002, 0.05, 5, new_R(), NEW_T);
    const auto r = fit_place_box(depth, K, new_R(), NEW_T, BoxParams{}, Vec3(0.2575, 0.0, 0.0));
    ASSERT_TRUE(r.ok) << c.dx << "," << c.dy << "," << c.yaw_deg << ": " << r.why;
    EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.02) << c.dx << "," << c.dy << "," << c.yaw_deg;
    EXPECT_LT(ang_deg(std::remainder(r.yaw - sc.yaw, M_PI / 2)), 3.0) << c.dx << "," << c.dy << "," << c.yaw_deg;
  }
}

// 넓게 찾기는 처음 추정 답이 맞으면 그 답 그대로 (PLACE 정 위치 측정 안 바뀜)
TEST(WristBox, WideSearchKeepsGuessAnswer)
{
  Scene sc;
  sc.cx = 0.2575;
  sc.cy = 0.004;
  sc.yaw = 3.0 * M_PI / 180.0;
  sc.pucks = {{-0.06, -0.06, 0.06}, {0.06, 0.06, 0.06}};
  const auto depth = render(sc, 0.001, 0.03, 7, new_R(), NEW_T);
  BoxParams off;
  off.search = 0.0;
  off.max_inside = -1;
  const auto a = fit_place_box(depth, K, new_R(), NEW_T, off, Vec3(0.2575, 0.0, 0.0));
  const auto b = fit_place_box(depth, K, new_R(), NEW_T, BoxParams{}, Vec3(0.2575, 0.0, 0.0));
  ASSERT_TRUE(a.ok && b.ok) << a.why << " / " << b.why;
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.yaw, b.yaw);
}

// 현재 LOOK URDF+처짐 자세: 정사각형 45°를 0°부터만 풀면 ok 이지만 방향이 17.6° 틀렸다.
TEST(WristBox, QuarterTurnBoundaryEscapesLocalMinimum)
{
  Mat3 R;
  R << -0.2107724516, -0.9722967735, 0.1010641273,
       -0.9699015031, 0.1951105958, -0.1456809177,
       0.1219264041, -0.1287277732, -0.9841560407;
  const Vec3 T(0.1725724038, -0.0030702076, 0.4389838876);
  for (const double deg : {-45.0, 45.0}) {
    Scene sc;
    sc.cx = 0.2575;
    sc.yaw = deg * M_PI / 180.0;
    sc.pucks = {{0.04, 0.0, 0.06}, {-0.04, 0.0, 0.06}};
    const auto r = fit_place_box(render(sc, 0.002, 0.05, 5, R, T), K, R, T,
      BoxParams{}, Vec3(0.2575, 0.0, 0.0));
    ASSERT_TRUE(r.ok) << r.why;
    EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.003);
    EXPECT_LT(ang_deg(std::remainder(r.yaw - sc.yaw, M_PI / 2)), 3.0);
  }
}

// 박스가 거의 화면 밖 (25 cm 옆) → 틀린 답 대신 실패
TEST(WristBox, WideSearchFarOutOfViewFails)
{
  Scene sc;
  sc.cx = 0.2575;
  sc.cy = 0.25;
  sc.yaw = 20.0 * M_PI / 180.0;
  const auto r = fit_place_box(render(sc, 0.002, 0.05, 3, new_R(), NEW_T), K, new_R(), NEW_T, BoxParams{}, Vec3(0.2575, 0.0, 0.0));
  if (r.ok) {EXPECT_LT(std::hypot(r.x - sc.cx, r.y - sc.cy), 0.03);}
}

// 넓게 찾기 상한: 시작 수 상한에 걸리면 멈추고 그때까지 답 (시간 · 시작 수 기록). search 0 = 넓게 찾기 · 안쪽 바닥 점 거르기 둘 다 안 함
TEST(WristBox, WideSearchCapAndRollback)
{
  Scene sc;
  sc.cx = 0.3775 - 0.15;
  sc.cy = 0.15;
  sc.pucks.push_back({0.04, 0.0, 0.06});
  sc.pucks.push_back({-0.04, 0.0, 0.06});
  const auto depth = render(sc, 0.001, 0.03, 1, new_R(), NEW_T);
  BoxParams cap;
  cap.search_max_starts = 3;
  const auto a = fit_place_box(depth, K, new_R(), NEW_T, cap, Vec3(0.2575, 0.0, 0.0));
  EXPECT_LE(a.search_starts, 3);
  EXPECT_TRUE(a.search_capped);
  EXPECT_GE(a.search_ms, 0.0);
  // 기본 시작 수 상한 (150) 이면 3 개를 넘게 봄. 시간 상한은 CPU · 빌드 형식에 따라 먼저 걸릴 수 있어 끄고 시작 수만 봄
  BoxParams no_time;
  no_time.search_budget_ms = 1e9;
  const auto b = fit_place_box(depth, K, new_R(), NEW_T, no_time, Vec3(0.2575, 0.0, 0.0));
  EXPECT_GT(b.search_starts, 3);
  EXPECT_FALSE(b.search_capped);
  // search 0: 넓게 찾기 · 뒤집힘 거르기 다 안 함 → 뒤집힌 답도 ok 로 나옴
  BoxParams off;
  off.search = 0.0;
  const auto c = fit_place_box(depth, K, new_R(), NEW_T, off, Vec3(0.2575, 0.0, 0.0));
  EXPECT_EQ(c.search_starts, 0);
  if (c.ok) {EXPECT_GT(c.inside_low, 0);}   // 안쪽 바닥 점이 있어도 거르지 않음
}
