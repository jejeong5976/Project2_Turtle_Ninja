// test_standup_table.cpp — 세우기 경로 표: 읽기 · 틀린 줄 거절 · 가장 가까운 경로 (180° 대칭 · 크기)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "manipulation/standup_table.hpp"

using manipulation::StandupPath;

namespace
{
std::string write_tmp(const std::string & body)
{
  const std::string f = ::testing::TempDir() + "standup_paths_test.txt";
  std::ofstream(f) << body;
  return f;
}

// side x y axis rate n release end_x end_y end_face above[5] grasp[5] q[n × 5]
std::string row(double side, double x, double y, double axis, int n = 2, int release = 1)
{
  std::string s = std::to_string(side) + " " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(axis) +
    " 0.9 " + std::to_string(n) + " " + std::to_string(release) + " 0.23 0.01 12.5 1 2 3 4 5 6 7 8 9 10";
  for (int i = 0; i < n * 5; ++i) {s += " " + std::to_string(2000 + i);}
  return s + "\n";
}
}  // namespace

TEST(StandupTable, LoadsRowsAndSkipsComments)
{
  const auto f = write_tmp("# 머리말\n\n" + row(0.0348, 0.23, 0.0, 15.0, 3, 1) + row(0.030, 0.245, -0.01, -30.0));
  std::vector<StandupPath> t;
  std::string why;
  ASSERT_TRUE(manipulation::load_standup_table(f, t, why)) << why;
  ASSERT_EQ(t.size(), 2u);
  EXPECT_DOUBLE_EQ(t[0].x, 0.23);
  EXPECT_EQ(t[0].release, 1);
  ASSERT_EQ(t[0].q.size(), 3u);
  EXPECT_EQ(t[0].q[2][4], 2014);
  EXPECT_EQ(t[0].grasp[0], 6);
  EXPECT_DOUBLE_EQ(t[0].end_face_deg, 12.5);
}

TEST(StandupTable, RejectsWrongPointCount)
{
  std::string bad = row(0.0348, 0.23, 0.0, 15.0, 2, 1);
  bad.insert(bad.size() - 1, " 7");   // 점 하나에 칸 하나 더
  const auto f = write_tmp(bad);
  std::vector<StandupPath> t;
  std::string why;
  EXPECT_FALSE(manipulation::load_standup_table(f, t, why));
  EXPECT_NE(why.find("칸 수"), std::string::npos) << why;
}

TEST(StandupTable, RejectsReleaseOutOfRange)
{
  const auto f = write_tmp(row(0.0348, 0.23, 0.0, 15.0, 2, 2));
  std::vector<StandupPath> t;
  std::string why;
  EXPECT_FALSE(manipulation::load_standup_table(f, t, why));
}

TEST(StandupTable, NearestUsesSizeAndHalfTurnSymmetry)
{
  std::vector<StandupPath> t(4);
  t[0].side = 0.0348; t[0].x = 0.23; t[0].y = 0.0; t[0].axis_deg = 80.0;
  t[1].side = 0.0348; t[1].x = 0.23; t[1].y = 0.0; t[1].axis_deg = 0.0;
  t[2].side = 0.030; t[2].x = 0.231; t[2].y = 0.001; t[2].axis_deg = -85.0;
  t[3].side = 0.0348; t[3].x = 0.26; t[3].y = 0.05; t[3].axis_deg = -85.0;
  double dx = 0, dy = 0, da = 0;
  // −85° 는 95° 와 같아서 80° (차 15°) 가 0° (차 85°) 보다 가까움. 크기가 다른 t[2] 는 안 고름
  EXPECT_EQ(manipulation::nearest_standup(t, 0.232, 0.0, -85.0, 0.0348, dx, dy, da), 0);
  EXPECT_NEAR(dx, 0.002, 1e-9);
  EXPECT_NEAR(da, 15.0, 1e-9);   // −85 − 80 = −165 → 180° 대칭으로 +15
  // 30 mm 퍽은 30 mm 경로만
  EXPECT_EQ(manipulation::nearest_standup(t, 0.26, 0.05, -85.0, 0.030, dx, dy, da), 2);
  EXPECT_EQ(manipulation::nearest_standup({}, 0.26, 0.05, -85.0, 0.030, dx, dy, da), -1);
}
