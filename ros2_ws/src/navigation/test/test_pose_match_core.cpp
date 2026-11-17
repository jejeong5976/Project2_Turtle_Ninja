// test_pose_match_core.cpp — test_auto_initial_pose.py (MatchTests) 를 옮김. 지도 raycast 합성 스캔으로 초기 위치 보정 계산 검증. ROS/장비 없음.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#include "navigation/py_json.hpp"
#include "startup_fixture.hpp"

using namespace navigation::pose_match;   // NOLINT
using startup_test::fixture;
using startup_test::raycast;

namespace
{
double rad(double d) {return d * M_PI / 180.0;}
double xy_err(const Pose & q, const Pose & t) {return std::hypot(q[0] - t[0], q[1] - t[1]);}
}  // namespace

TEST(PoseMatch, OffsetsAndHeading)
{
  const auto model = fixture();
  const Pose truth{.15, -.15, .08};
  const Points points = raycast(*model, truth);
  std::vector<double> elapsed;
  const double dxy[][2] = {{0, 0}, {.1, 0}, {.2, 0}, {.3, 0}, {.4, 0}, {-.4, 0}, {0, -.4}, {.25, -.2}, {-.3, .2}};
  for (const auto & o : dxy) {
    for (double da : {-20.0, -10.0, 0.0, 10.0, 20.0}) {
      SCOPED_TRACE(testing::Message() << "dx=" << o[0] << " dy=" << o[1] << " yaw=" << da);
      const Pose current{truth[0] + o[0], truth[1] + o[1], truth[2] + rad(da)};
      const auto result = match(*model, points, current, current);
      elapsed.push_back(result.elapsed_s);
      ASSERT_TRUE(result.pose.has_value());
      const Pose q = *result.pose;
      EXPECT_GE(result.score, .95);
      EXPECT_LT(xy_err(q, truth), .035);
      EXPECT_LT(std::fabs(wrap(q[2] - truth[2])), rad(1.1));
      const auto decision = decide_publication(result, *model, points, current);
      EXPECT_EQ(decision.publish, o[0] != 0 || o[1] != 0 || da != 0) << decision.reason;
    }
  }
  std::sort(elapsed.begin(), elapsed.end());
  std::printf("synthetic_match_seconds: cases=%zu min=%.3f max=%.3f median=%.3f\n", elapsed.size(),
    elapsed.front(), elapsed.back(), elapsed[elapsed.size() / 2]);
}

TEST(PoseMatch, OffGridRangeNoiseAndMissingBeams)
{
  // 파이썬은 numpy default_rng(71) — 같은 분포 (거리 잡음 σ 4 mm, 빔 10 % 빠짐) 를 mt19937 로
  const auto model = fixture();
  const Pose truth{.173, -.137, .137};
  const Points clean = raycast(*model, truth);
  std::mt19937 rng(71);
  std::normal_distribution<double> noise(0.0, .004);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  Points points;
  for (std::size_t i = 0; i < clean.size(); ++i) {
    const double n = std::hypot(clean.x[i], clean.y[i]);
    const double k = 1 + noise(rng) / n;
    if (uni(rng) > .1) {
      points.push(clean.x[i] * k, clean.y[i] * k);
    }
  }
  const Pose current{truth[0] + .273, truth[1] - .194, truth[2] + rad(13.7)};
  const auto result = match(*model, points, current, current);
  ASSERT_TRUE(result.pose.has_value());
  EXPECT_LT(xy_err(*result.pose, truth), .035);
  EXPECT_LT(std::fabs(wrap((*result.pose)[2] - truth[2])), rad(1.1));
  EXPECT_TRUE(decide_publication(result, *model, points, current).publish);
}

TEST(PoseMatch, MapOriginRotation)
{
  const double yaw = .63;
  const auto model = fixture(yaw);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double lx = 3.15, ly = 2.35;
  const Pose truth{-3. + (c * lx - s * ly), -2.5 + (s * lx + c * ly), yaw + .08};
  const Points points = raycast(*model, truth);
  const Pose current{truth[0] + .25, truth[1] - .30, truth[2] - .25};
  const auto result = match(*model, points, current, current);
  ASSERT_TRUE(result.pose.has_value());
  EXPECT_LT(xy_err(*result.pose, truth), .035);
  EXPECT_LT(std::fabs(wrap((*result.pose)[2] - truth[2])), .02);
  EXPECT_TRUE(decide_publication(result, *model, points, current).publish);
}

TEST(PoseMatch, CorridorAmbiguityRetries)
{
  startup_test::Grid grid(600, 240);
  grid.fill(0, 600, 79, 82, 100);
  grid.fill(0, 600, 159, 162, 100);
  const auto model = MapModel::create(grid.g, grid.w, grid.h, .025, Pose{-3., -7.5, 0.});
  const Points points = raycast(*model, Pose{0., 0., 0.});
  const Pose current{.2, .2, .1};
  const auto result = match(*model, points, current, current);
  const auto decision = decide_publication(result, *model, points, current);
  EXPECT_TRUE(result.ambiguous) << result.reason;
  EXPECT_FALSE(decision.publish);
  EXPECT_TRUE(decision.retry);
}

TEST(PoseMatch, InvalidAndSparseInputs)
{
  EXPECT_THROW(MapModel::create(std::vector<int16_t>(400, 0), 20, 20, .02, Pose{0, 0, 0}), std::invalid_argument);
  EXPECT_THROW(MapModel::create(std::vector<int16_t>(4, 100), 2, 2, 0., Pose{0, 0, 0}), std::invalid_argument);
  const MatchConfig cfg;
  const std::vector<double> ranges{NAN, INFINITY, 0., -.2, .3, 5.};
  const Points points = scan_points(ranges, -.5, .1);
  EXPECT_EQ(points.size(), 1u);
  EXPECT_THROW(prepare_points(points, cfg), std::invalid_argument);
  MatchConfig bad;
  bad.coarse_xy = 0;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
}

TEST(PoseMatch, OutsideMapCountsAsMiss)
{
  const auto model = fixture();
  Points points;
  for (int i = 0; i < 40; ++i) {
    points.push(50., 50.);
  }
  EXPECT_EQ(model->score(points, Pose{.15, -.15, 0}, .04), 0.);
}

TEST(PoseMatch, FreshCurrentPoseComparison)
{
  const auto model = fixture();
  const Pose truth{.15, -.15, .08};
  const Points points = raycast(*model, truth);
  const Pose old_pose{.45, -.15, .08};
  const auto result = match(*model, points, old_pose, old_pose);
  // 계산 중 AMCL이 이미 맞췄다면 같은 initialpose를 덮어쓰지 않음.
  const auto decision = decide_publication(result, *model, points, truth);
  EXPECT_FALSE(decision.publish);
  EXPECT_EQ(decision.reason, "already_aligned");
  // 계산 이후 전혀 다른 스캔이면 이전 후보를 발행하지 않음.
  Points wrong = points;
  for (std::size_t i = 0; i < wrong.size(); ++i) {
    wrong.x[i] += 3.;
    wrong.y[i] += 3.;
  }
  EXPECT_FALSE(decide_publication(result, *model, wrong, old_pose).publish);
}

TEST(PoseMatch, MotionEpochGeometry)
{
  EXPECT_FALSE(pose_changed(Pose{0, 0, 3.14}, Pose{.001, .002, -3.14}));
  EXPECT_TRUE(pose_changed(Pose{0, 0, 0}, Pose{.02, 0, 0}));
  EXPECT_TRUE(pose_changed(Pose{0, 0, 0}, Pose{0, 0, .04}));
}

// C++ 판 추가: 파이썬과 같은 글자 (json.dumps · float repr) · 같은 offsets 격자
TEST(PoseMatch, PythonFloatRepr)
{
  auto f = [](double v) {
      std::string s;
      navigation::pyjson::append_float(s, v);
      return s;
    };
  // 기대값 = python3 repr()
  EXPECT_EQ(f(0.0), "0.0");
  EXPECT_EQ(f(-0.0), "-0.0");
  EXPECT_EQ(f(1.0), "1.0");
  EXPECT_EQ(f(-1.0), "-1.0");
  EXPECT_EQ(f(0.1), "0.1");
  EXPECT_EQ(f(0.9583333333333334), "0.9583333333333334");
  EXPECT_EQ(f(1e-4), "0.0001");
  EXPECT_EQ(f(1e-5), "1e-05");
  EXPECT_EQ(f(1.5e-7), "1.5e-07");
  EXPECT_EQ(f(1e15), "1000000000000000.0");
  EXPECT_EQ(f(1e16), "1e+16");
  EXPECT_EQ(f(123456789012345678.0), "1.2345678901234568e+17");
  EXPECT_EQ(f(12.5), "12.5");
  EXPECT_EQ(f(0.123456789), "0.123456789");
  EXPECT_EQ(f(1e100), "1e+100");
  EXPECT_THROW(f(NAN), std::invalid_argument);
  std::string s;
  navigation::pyjson::append_string(s, "a\"b\\c\n\x01한글");
  EXPECT_EQ(s, "\"a\\\"b\\\\c\\n\\u0001한글\"");
  double v = 0;
  EXPECT_TRUE(navigation::pyjson::parse_python_float(" 0.5 ", v));
  EXPECT_EQ(v, .5);
  EXPECT_TRUE(navigation::pyjson::parse_python_float("nan", v));
  EXPECT_TRUE(std::isnan(v));
  EXPECT_FALSE(navigation::pyjson::parse_python_float("", v));
  EXPECT_FALSE(navigation::pyjson::parse_python_float("0.5x", v));
  EXPECT_FALSE(navigation::pyjson::parse_python_float("0x10", v));
}

TEST(PoseMatch, OffsetsLikeNumpyUnique)
{
  // 기대값 = python pose_match_core.offsets 길이 (31 · 27 · 27)
  EXPECT_EQ(offsets(0.6, 0.04).size(), 31u);
  EXPECT_EQ(offsets(0.5, 0.04).size(), 27u);
  EXPECT_EQ(offsets(rad(25.0), rad(2.0)).size(), 27u);
  const auto o = offsets(0.04, 0.01);
  EXPECT_EQ(o.front(), -0.04);
  EXPECT_EQ(o.back(), 0.04);
}
