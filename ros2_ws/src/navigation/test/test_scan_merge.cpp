#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "navigation/scan_merge.hpp"

using navigation::BodyBox;
using navigation::Mount;
using navigation::Point2;

TEST(ScanMerge, DropsBodyAndOutOfRangePoints)
{
  const BodyBox body{0.17, 0.19};
  const std::vector<float> r{0.10f, 1.0f, std::numeric_limits<float>::quiet_NaN(), 9.0f};
  std::vector<Point2> pts;
  navigation::scan_to_points(r, 0.0, 0.0, 0.05, 8.0, Mount{0.0, 0.0, 0.0}, body, pts);
  ASSERT_EQ(pts.size(), 1u);   // 0.10 m = 몸 안, NaN, 9 m = 범위 밖
  EXPECT_NEAR(pts[0].x, 1.0, 1e-9);
}

TEST(ScanMerge, BodyBoxMatchesFootprintEdge)
{
  // 왼앞 자기 선 (0.10, 0.18) 은 버리고, 몸 상자 바로 밖 (0.10, 0.20) 은 남김
  const BodyBox body{0.17, 0.19};
  EXPECT_TRUE(navigation::inside_body(Point2{0.10, 0.18}, body));
  EXPECT_FALSE(navigation::inside_body(Point2{0.10, 0.20}, body));
}

TEST(ScanMerge, RearMountRotatesPoints)
{
  // 앞 라이다처럼 180° 돌려 붙인 장착: 라이다 0° 방향 1 m → 몸 기준 뒤쪽
  std::vector<Point2> pts;
  navigation::scan_to_points({1.0f}, 0.0, 0.0, 0.05, 8.0, Mount{0.1145, 0.0, M_PI}, BodyBox{0.17, 0.19}, pts);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 0.1145 - 1.0, 1e-9);
  EXPECT_NEAR(pts[0].y, 0.0, 1e-9);
}

TEST(ScanMerge, BinsAreCenteredWithoutBias)
{
  // 칸 가운데 각 = −π + k · inc. 칸 경계 바로 안쪽 점이 옆 칸으로 밀리지 않음 (floor 면 평균 반 칸 돌아감)
  const int bins = 720;
  const double inc = 2.0 * M_PI / bins;
  std::vector<float> ranges;
  for (double a : {0.0, 0.4 * inc, -0.4 * inc}) {
    navigation::bin_points({Point2{std::cos(a), std::sin(a)}}, bins, ranges);
    const long k = std::lround(M_PI / inc);   // 0 rad 의 칸
    EXPECT_NEAR(ranges[static_cast<std::size_t>(k)], 1.0f, 1e-6) << a;
  }
}

TEST(ScanMerge, KeepsNearestPerBinAndWrapsPi)
{
  std::vector<float> ranges;
  navigation::bin_points({Point2{-2.0, 1e-9}, Point2{-1.0, -1e-9}}, 720, ranges);   // ±π 는 같은 칸 0
  EXPECT_NEAR(ranges[0], 1.0f, 1e-6);
}

// ---------- 자기 몸 거르기 wheels (차체 상자 + 조향 따라 도는 바퀴 사각형) ----------
namespace
{
navigation::SelfFilter wheels_filter(std::vector<double> steer = {0.0, 0.0, 0.0, 0.0})
{
  navigation::SelfFilter f;
  f.box_mode = false;
  f.modules = {{-0.070, 0.112, 1.0}, {0.060, 0.112, 1.0}, {-0.070, -0.112, -1.0}, {0.060, -0.112, -1.0}};
  f.steer = steer;
  return f;
}
}  // namespace

TEST(SelfFilter, BoxModeIsOldBehaviour)
{
  navigation::SelfFilter f;   // 기본 = box ±0.17 × ±0.19
  EXPECT_TRUE(navigation::inside_self(Point2{0.10, 0.18}, f));
  EXPECT_TRUE(navigation::inside_self(Point2{0.0, 0.175}, f));   // 옆 사람 자리도 지움 (고친 이유)
  EXPECT_FALSE(navigation::inside_self(Point2{0.10, 0.20}, f));
  // BodyBox 판 scan_to_points 와 같은 결과
  std::vector<Point2> a, b;
  const std::vector<float> r{0.10f, 0.2f, 0.3f, 1.0f};
  navigation::scan_to_points(r, 0.0, 0.5, 0.05, 8.0, Mount{0.1145, 0.0, M_PI}, BodyBox{0.17, 0.19}, a);
  navigation::scan_to_points(r, 0.0, 0.5, 0.05, 8.0, Mount{0.1145, 0.0, M_PI}, f, b);
  ASSERT_EQ(a.size(), b.size());
}

TEST(SelfFilter, WheelsKeepsPersonBesideBodyInsidePolygonStop)
{
  const auto f = wheels_filter();
  // 차체 안 (기둥 · 배터리 · 왼쪽 선 0.14–0.15) 은 버림
  EXPECT_TRUE(navigation::inside_self(Point2{-0.02, 0.145}, f));
  EXPECT_TRUE(navigation::inside_self(Point2{0.14, -0.10}, f));
  // 조향 0° 바퀴 바깥면 (0.112 + 0.027 + 0.010 = 0.149) · 라이다 오차까지
  EXPECT_TRUE(navigation::inside_self(Point2{0.06, 0.158}, f));
  // 옆 사람: 몸 옆 가운데 y 0.16 · 0.17 (PolygonStop ±0.18 안) 은 남김 — 상자 방식은 지움
  for (double y : {0.155, 0.165, 0.175}) {
    EXPECT_FALSE(navigation::inside_self(Point2{0.0, y}, f)) << y;
    EXPECT_FALSE(navigation::inside_self(Point2{0.0, -y}, f)) << y;
  }
  // 바퀴 앞 · 뒤 끝 너머 (x 0.12, 조향 0°) 의 옆 점도 남김
  EXPECT_FALSE(navigation::inside_self(Point2{0.12, 0.16}, f));
  // 앞뒤: 차체 폭 안은 앞뒤 면 띠 ±0.17 까지 버림 (박스 면 1 cm 정차에서 출발 막힘), 그 너머는 남김
  EXPECT_TRUE(navigation::inside_self(Point2{0.15, 0.0}, f));
  EXPECT_TRUE(navigation::inside_self(Point2{-0.14, 0.0}, f));
  EXPECT_FALSE(navigation::inside_self(Point2{0.175, 0.0}, f));
  EXPECT_FALSE(navigation::inside_self(Point2{-0.175, 0.0}, f));
}

TEST(SelfFilter, WheelsDropsDockedBoxFaceButKeepsSideBand)
{
  // dock5 정차: 박스 면 = 앞 끝 0.141 + 1 cm = 0.151 (± 정밀 접근 오차 1 cm), 박스 폭 ±0.1025
  //   → FootprintApproach (±0.16) 안. 안 지우면 rl_nav 출발 명령이 collision_monitor APPROACH 로 다 0
  const auto f = wheels_filter();
  for (double x : {0.146, 0.151, 0.156, 0.165}) {
    for (double y : {-0.1025, 0.0, 0.1025}) {
      EXPECT_TRUE(navigation::inside_self(Point2{x, y}, f)) << x << " " << y;
    }
  }
  // 뒤 면도 같음 (뒤로 붙은 정차 · 뒤 라이다)
  EXPECT_TRUE(navigation::inside_self(Point2{-0.16, 0.05}, f));
  // 옆 띠는 그대로: 판 옆 2 · 4 cm 사람 (앞 모서리 · 가운데 · 뒤 모서리)
  for (double x : {-0.16, -0.005, 0.12, 0.16}) {
    EXPECT_FALSE(navigation::inside_self(Point2{x, 0.133 + 0.04}, f)) << x;
    EXPECT_FALSE(navigation::inside_self(Point2{x, -(0.133 + 0.04)}, f)) << x;
  }
  // face_clear_x 를 차체 끝으로 두면 앞뒤 띠 거르기 없음
  auto g = f;
  g.face_clear_x_min = g.chassis_x_min;
  g.face_clear_x_max = g.chassis_x_max;
  EXPECT_FALSE(navigation::inside_self(Point2{0.151, 0.0}, g));
}

TEST(SelfFilter, WheelRectangleTurnsWithSteer)
{
  // fr 조향축 (0.06, 0.112). 사각형 = 굴림 ±0.05 · 바퀴축 −0.05~+0.05 → 조향 따라 모서리 (0.0707) 가 돎
  const Point2 diag{0.06 + 0.048, 0.112 + 0.048};   // 조향축에서 45° 방향 0.068
  const Point2 up{0.06, 0.112 + 0.068};              // 조향축에서 +y 방향 0.068 (y 0.180, PolygonStop 끝)
  const double a = M_PI / 4;
  EXPECT_TRUE(navigation::inside_self(diag, wheels_filter({0.0, 0.0, 0.0, 0.0})));    // 조향 0°: 모서리
  EXPECT_FALSE(navigation::inside_self(diag, wheels_filter({0.0, a, 0.0, 0.0})));     // 45°: 굴림 방향 0.068 > 0.05
  EXPECT_FALSE(navigation::inside_self(up, wheels_filter({0.0, 0.0, 0.0, 0.0})));     // 0°: 바깥 0.068 > 0.05 → 사람 봄
  EXPECT_TRUE(navigation::inside_self(up, wheels_filter({0.0, a, 0.0, 0.0})));        // 45°: 바퀴 모서리가 거기 → 버림
  // 오른쪽 모듈은 바깥 = −y (rr 조향축 (0.06, −0.112), 조향 0°)
  EXPECT_TRUE(navigation::inside_self(Point2{0.06, -0.158}, wheels_filter()));
  EXPECT_FALSE(navigation::inside_self(Point2{0.06, -0.165}, wheels_filter()));
}

// 실물 bag: 옆으로 갈 때 (조향 −97°) rl 모듈 (−0.07, −0.112) 바퀴 굴림 끝 점이 몸 (−0.08~−0.097, −0.161~−0.170) 에 남아
//   PolygonStop ±0.18 이 8 s 막음 (B4 · B5 이동 4 번 실패). 굴림 반길이 0.05 → 0.07 (navigation.yaml) 이면 버리고, 옆 사람 · 벽은 그대로 봄
TEST(SelfFilter, SidewaysSteerDropsWheelTipSeenOnRobot)
{
  const double st = -97.0 * M_PI / 180.0;
  auto f = wheels_filter({0.0, 0.0, st, st});
  const std::vector<Point2> tip{{-0.079, -0.166}, {-0.086, -0.165}, {-0.097, -0.165}, {-0.081, -0.170}, {-0.074, -0.167}};
  for (const auto & p : tip) {
    EXPECT_FALSE(navigation::inside_self(p, f)) << "0.05 에서는 남음 " << p.x << "," << p.y;
  }
  f.wheel_half_u = 0.07;
  for (const auto & p : tip) {
    EXPECT_TRUE(navigation::inside_self(p, f)) << p.x << "," << p.y;
  }
  // 그대로 보이는 것: 바퀴 사이 옆 사람 (조향 ±90° 도), 옆 벽 0.19, 조향 0° 옆 띠, 앞뒤 면 띠 밖
  for (double a : {0.0, M_PI / 2, -M_PI / 2, st}) {
    auto g = wheels_filter({a, a, a, a});
    g.wheel_half_u = 0.07;
    EXPECT_FALSE(navigation::inside_self(Point2{0.0, -0.175}, g)) << a;
    EXPECT_FALSE(navigation::inside_self(Point2{0.0, 0.175}, g)) << a;
    EXPECT_FALSE(navigation::inside_self(Point2{-0.07, -0.19}, g)) << a;
    EXPECT_FALSE(navigation::inside_self(Point2{0.175, 0.0}, g)) << a;
    EXPECT_FALSE(navigation::inside_self(Point2{-0.175, 0.0}, g)) << a;
  }
}

TEST(SelfFilter, UnknownSteerUsesSweepCircle)
{
  auto f = wheels_filter({});   // 조향 각 모름 → 모듈 둘레 원 (반지름 = 사각형 모서리 0.0707)
  EXPECT_NEAR(f.sweep_radius(), std::hypot(0.05, 0.05), 1e-12);
  EXPECT_TRUE(navigation::inside_self(Point2{0.06, 0.112 + 0.07}, f));    // 어느 각도든 바퀴가 닿을 수 있는 곳
  EXPECT_FALSE(navigation::inside_self(Point2{0.0, 0.165}, f));           // 모듈 사이 옆은 그대로 봄
  auto g = wheels_filter({0.0, NAN, 0.0, 0.0});                           // 한 모듈만 NaN → 그 모듈만 원
  EXPECT_TRUE(navigation::inside_self(Point2{0.06, 0.112 + 0.07}, g));
  EXPECT_FALSE(navigation::inside_self(Point2{-0.07, 0.112 + 0.07}, g));  // fl 은 조향 0° 사각형 (바깥 0.05 까지)
}

TEST(SelfFilter, ScanToPointsDropsWheelHitsButKeepsSidePerson)
{
  // 앞 라이다 (0.1145, 0, yaw π): 몸 왼쪽 옆 사람 (0.0, 0.17) 과 fr 바퀴 안쪽 면 (0.06, 0.09)
  const Mount front{0.1145, 0.0, M_PI};
  auto ray = [&](const Point2 & p, double & ang, float & r) {
      const double dx = p.x - front.x, dy = p.y - front.y;
      r = static_cast<float>(std::hypot(dx, dy));
      ang = std::atan2(dy, dx) - front.yaw;
    };
  double a1, a2;
  float r1, r2;
  ray(Point2{0.0, 0.17}, a1, r1);
  ray(Point2{0.06, 0.09}, a2, r2);
  std::vector<Point2> pts;
  navigation::scan_to_points({r1}, a1, 0.0, 0.05, 8.0, front, wheels_filter(), pts);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].y, 0.17, 1e-6);
  navigation::scan_to_points({r2}, a2, 0.0, 0.05, 8.0, front, wheels_filter(), pts);
  EXPECT_TRUE(pts.empty());
  navigation::SelfFilter box;   // 상자 방식은 옆 사람도 지움
  navigation::scan_to_points({r1}, a1, 0.0, 0.05, 8.0, front, box, pts);
  EXPECT_TRUE(pts.empty());
}
