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
