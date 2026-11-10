// test_simple_dock.cpp — 규칙 정밀 접근 (simple_dock_core DockCore) 시험, ROS 없음
//   점 로봇 운동 시뮬 (명령 0.1 s 1 차 지연, 20 Hz) + 박스 · 벽 · 사람 선분을 라이다 점으로
//   보는 것: 박스 면 틈 · 옆 위치 · 방향, 몸 사각형 (앞 0.145 · 옆 · 뒤 0.16) 이 벽 · 박스 · 사람에 안 닿음, 막히면 멈추고 실패
//   실행: colcon build → ./build/navigation/test_simple_dock

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "navigation/simple_dock_core.hpp"

using namespace navigation::dock;
using navigation::rl::wrap;

namespace
{
// 몸: 앞 0.145 (box_dist = 반 변 + 0.145 + 1 cm) · 옆 · 뒤 0.16 (footprint)
constexpr double kBoxHalf = 0.1025, kBoxDist = 0.2575, kHalf = 0.16, kFront = 0.145, kDt = 0.05;

using Seg = std::pair<Vec2, Vec2>;

double rad(double d) {return d * M_PI / 180.0;}

// 박스 중심 c · 로봇이 볼 방향 yaw → 네 변
std::vector<Seg> box_segs(Vec2 c, double yaw)
{
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const double k[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  Vec2 v[4];
  for (int i = 0; i < 4; ++i) {
    const double x = k[i][0] * kBoxHalf, y = k[i][1] * kBoxHalf;
    v[i] = {c.x + cs * x - sn * y, c.y + sn * x + cs * y};
  }
  std::vector<Seg> out;
  for (int i = 0; i < 4; ++i) {out.push_back({v[i], v[(i + 1) % 4]});}
  return out;
}

std::vector<Vec2> sample(const std::vector<Seg> & segs, double step = 0.005)
{
  std::vector<Vec2> out;
  for (const auto & [a, b] : segs) {
    const int n = std::max(2, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / step));
    for (int i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / (n - 1);
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  return out;
}

// 로봇 중심에서 보이는 점 (가림 없음) → 몸 좌표
std::vector<Vec2> visible(const std::vector<Vec2> & P, const Pose2 & pose, double r_max = 1.5)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  std::vector<Vec2> out;
  for (const auto & p : P) {
    const double dx = p.x - pose.x, dy = p.y - pose.y;
    const Vec2 b{c * dx + s * dy, -s * dx + c * dy};
    if (std::hypot(b.x, b.y) < r_max) {out.push_back(b);}
  }
  return out;
}

// 몸 사각형 (앞 kFront · 옆 · 뒤 kHalf) 과 점들 사이 가장 짧은 거리 (안쪽이면 음수)
double body_gap(const std::vector<Vec2> & P, const Pose2 & pose)
{
  const auto B = visible(P, pose, 50.0);
  if (B.empty()) {return 9.0;}
  const double cx = (kFront - kHalf) / 2;
  double best = 1e9;
  for (const auto & b : B) {
    const double dx = std::abs(b.x - cx) - (kFront + kHalf) / 2, dy = std::abs(b.y) - kHalf;
    const double g = (dx < 0 && dy < 0) ? std::max(dx, dy) : std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
    best = std::min(best, g);
  }
  return best;
}

// 로봇 쪽을 보는 박스 변만 (가림 대신 뒷면 버리기)
std::vector<Vec2> box_visible(Vec2 c, double yaw, const Pose2 & pose)
{
  std::vector<Seg> out;
  for (const auto & [a, b] : box_segs(c, yaw)) {
    const Vec2 m{(a.x + b.x) / 2, (a.y + b.y) / 2};
    const Vec2 n{m.x - c.x, m.y - c.y};
    if (n.x * (pose.x - m.x) + n.y * (pose.y - m.y) > 0) {out.push_back({a, b});}
  }
  return sample(out);
}

struct Person
{
  double t;
  Seg seg;
};

struct Result
{
  DockState st = DockState::Run;
  std::string why;
  Pose2 pose;
  double gap = 9.0, face_gap = 0.0, side = 0.0, dyaw = 0.0;
  std::set<std::string> modes;
};

void step_pose(Pose2 & p, const std::array<double, 3> & v)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  p = {p.x + kDt * (c * v[0] - s * v[1]), p.y + kDt * (s * v[0] + c * v[1]), wrap(p.yaw + kDt * v[2])};
}

// map_box = 지도 (목표) 박스, box = 실제 박스
Result run(Pose2 start, Vec2 box_c, double box_yaw, std::optional<std::pair<Vec2, double>> map_box = std::nullopt,
  const std::vector<Seg> & walls = {}, std::optional<Person> person = std::nullopt, bool self_pts = true, double t_max = 60.0)
{
  const Vec2 mc = map_box ? map_box->first : box_c;
  const double my = map_box ? map_box->second : box_yaw;
  DockCore core({mc.x - kBoxDist * std::cos(my), mc.y - kBoxDist * std::sin(my)}, my);
  const auto walls_p = sample(walls);
  const auto box_all = sample(box_segs(box_c, box_yaw));
  Pose2 pose = start;
  std::array<double, 3> v{0, 0, 0};
  std::mt19937 rng(0);
  std::normal_distribution<double> noise(0.0, 0.003);
  const std::vector<Vec2> selfp{{0.01, 0.04}, {0.08, 0.1}, {0.16, -0.11}, {-0.07, -0.1}, {0.09, 0.14}, {-0.08, 0.1}};
  Result r;
  std::vector<Vec2> W;
  const auto all_with_box = [&]() {auto a = W; a.insert(a.end(), box_all.begin(), box_all.end()); return a;};
  for (double t = 0.0; t < t_max; t += kDt) {
    W = box_visible(box_c, box_yaw, pose);
    W.insert(W.end(), walls_p.begin(), walls_p.end());
    if (person && t >= person->t) {
      const auto pp = sample({person->seg});
      W.insert(W.end(), pp.begin(), pp.end());
    }
    auto raw = visible(W, pose);
    for (auto & p : raw) {p.x += noise(rng); p.y += noise(rng);}
    if (self_pts) {raw.insert(raw.end(), selfp.begin(), selfp.end());}
    std::vector<Vec2> odom_pts;                         // scan_merger 몸 상자 (±0.17 × ±0.19) 지움 → odom
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    for (const auto & p : raw) {
      if (std::abs(p.x) < 0.17 && std::abs(p.y) < 0.19) {continue;}
      odom_pts.push_back({c * p.x - s * p.y + pose.x, s * p.x + c * p.y + pose.y});
    }
    core.update_box(odom_pts, {pose.x, pose.y});
    const auto [cmd, st] = core.step(pose, raw, t);
    r.st = st;
    r.modes.insert(core.mode);
    if (st != DockState::Run) {break;}
    for (int k = 0; k < 3; ++k) {v[k] += (cmd[k] - v[k]) * std::min(1.0, kDt / 0.1);}
    step_pose(pose, v);
    r.gap = std::min(r.gap, body_gap(all_with_box(), pose));
  }
  for (int i = 0; i < 4; ++i) {                         // 끝난 뒤 남은 속도로 미끄러짐 (지연 0.1 s)
    for (auto & x : v) {x *= 0.5;}
    step_pose(pose, v);
    r.gap = std::min(r.gap, body_gap(all_with_box(), pose));
  }
  r.why = core.why;
  r.pose = pose;
  const Vec2 fwd{std::cos(box_yaw), std::sin(box_yaw)}, rel{box_c.x - pose.x, box_c.y - pose.y};
  r.face_gap = rel.x * fwd.x + rel.y * fwd.y - kBoxHalf - kFront;   // 박스 면 틈 (목표 1 cm)
  r.side = rel.x * -fwd.y + rel.y * fwd.x;
  r.dyaw = wrap(box_yaw - pose.yaw) * 180.0 / M_PI;
  return r;
}

std::string info(const Result & r)
{
  char b[256];
  std::snprintf(b, sizeof(b), "why '%s' 면 틈 %.4f · 옆 %.4f · yaw %.2f° · 몸 틈 %.4f · 자세 (%.3f, %.3f)", r.why.c_str(), r.face_gap,
    r.side, r.dyaw, r.gap, r.pose.x, r.pose.y);
  return b;
}
}  // namespace

TEST(SimpleDock, StraightBoxAhead)
{
  const auto r = run({0.0, 0.0, 0.0}, {0.55, 0.0}, 0.0);
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(0.004 < r.face_gap && r.face_gap < 0.016 && std::abs(r.side) < 0.008 && std::abs(r.dyaw) < 2.5) << info(r);
}

// 출발 30° 틀어짐 · 박스가 지도보다 옆 4 cm · 5° 돌아감 → 실제 박스에 맞춰 섬
TEST(SimpleDock, TurnFirstThenApproachWithShiftedBox)
{
  const auto r = run({0.0, 0.05, rad(30)}, {0.58, -0.04}, rad(5), std::make_pair(Vec2{0.58, 0.0}, 0.0));
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(r.modes.count("돌기")) << info(r);
  EXPECT_TRUE(0.004 < r.face_gap && r.face_gap < 0.016 && std::abs(r.side) < 0.01 && std::abs(r.dyaw) < 3.0) << info(r);
}

// 박스 왼쪽 옆에 벽 (도착 자리 몸 옆면에서 4.5 · 3 cm). 8° 틀어져 출발 → 뒤 왼쪽 모서리가 벽에서 2.5 · 1 cm (띠 안)
class WallBeside : public ::testing::TestWithParam<double> {};
TEST_P(WallBeside, NeverTouched)
{
  const double wy = GetParam();
  const auto r = run({0.0, 0.0, rad(-8)}, {0.56, 0.0}, 0.0, std::nullopt, {{{-0.6, wy}, {0.9, wy}}});
  EXPECT_GT(r.gap, 0.005) << info(r);
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(0.004 < r.face_gap && r.face_gap < 0.016) << info(r);
}
INSTANTIATE_TEST_SUITE_P(SimpleDock, WallBeside, ::testing::Values(0.205, 0.19));

// 벽에 2 cm 붙은 채 40° 틀어져 출발 → 모서리가 벽을 칠 회전이라 먼저 떨어진 뒤 돌기
TEST(SimpleDock, RotationBlockedByWallEscapesThenDocks)
{
  const double th = rad(40);
  const double y0 = 0.40 - 0.02 - kHalf * (std::cos(th) + std::sin(th));   // 돌아간 몸의 위쪽 모서리가 벽에서 2 cm
  const auto r = run({0.0, y0, th}, {0.58, 0.0}, 0.0, std::nullopt, {{{-0.8, 0.40}, {0.9, 0.40}}});
  EXPECT_GT(r.gap, 0.003) << info(r);
  ASSERT_EQ(r.st, DockState::Done) << info(r);
}

// 다가가는 중 사람 다리가 몸 앞 약 15 cm 에 들어옴 (0.6 s, 최고 0.15 m/s) → 멈추고 실패 (부딪히지 않음)
TEST(SimpleDock, PersonStepsInFrontStops)
{
  const auto r = run({-0.15, 0.0, 0.0}, {0.55, 0.0}, 0.0, std::nullopt, {}, Person{0.6, {{0.25, -0.05}, {0.25, 0.0}}});
  ASSERT_EQ(r.st, DockState::Fail) << info(r);
  EXPECT_NE(r.why.find("막힘"), std::string::npos) << info(r);
  EXPECT_GT(r.gap, 0.0) << info(r);
}

// 박스 면 (몸 앞 1 cm) 은 막는 점이 아님 · 지도 박스가 7 cm 어긋나도
TEST(SimpleDock, BoxPointsDoNotBlockFinalApproach)
{
  const auto r = run({0.0, 0.0, 0.0}, {0.55, 0.07}, 0.0, std::make_pair(Vec2{0.55, 0.0}, 0.0));
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(std::abs(r.side) < 0.015 && r.gap > 0.0) << info(r);
}

// 장소 정차 (앞에 박스 없음, 박스 맞춤 켠 채) → 지도 정차점 그대로
TEST(SimpleDock, PlaceStopWithoutBox)
{
  const Vec2 goal{0.6 - kBoxDist, 0.0};
  const auto r = run({0.0, 0.1, rad(-20)}, {5.0, 5.0}, 0.0, std::make_pair(Vec2{0.6, 0.0}, 0.0), {}, std::nullopt, false);
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_LT(std::hypot(r.pose.x - goal.x, r.pose.y - goal.y), 0.02) << info(r);
}

// 옆 3 cm 에 다리 (앞으로 가는 길 밖) → 안 멈춤
TEST(SimpleDock, LegBesideDoesNotStopForward)
{
  const auto r = run({-0.15, 0.0, 0.0}, {0.55, 0.0}, 0.0, std::nullopt, {}, Person{1.0, {{0.05, -0.19}, {0.12, -0.19}}});
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_GT(r.gap, 0.005) << info(r);
}

// 박스가 지도 자리에서 10–22 cm 옆 · 앞뒤로, 돌아가 있음 (실물 arena5: 8–25 cm) → 넓게 찾기로 실제 박스 앞에 섬
class FarBox : public ::testing::TestWithParam<std::array<double, 3>> {};
TEST_P(FarBox, FoundByWideSearch)
{
  const auto [dx, dy, yd] = GetParam();
  const auto r = run({0.0, 0.0, 0.0}, {0.58 + dx, dy}, rad(yd), std::make_pair(Vec2{0.58, 0.0}, 0.0));
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(0.004 < r.face_gap && r.face_gap < 0.02 && std::abs(r.side) < 0.015 && std::abs(r.dyaw) < 3.0) << info(r);
  EXPECT_GT(r.gap, 0.0) << info(r);
}
INSTANTIATE_TEST_SUITE_P(SimpleDock, FarBox, ::testing::Values(
    std::array<double, 3>{0.10, 0.15, 5.0}, std::array<double, 3>{-0.12, -0.18, -8.0}, std::array<double, 3>{0.0, 0.22, 0.0}));

// 지도 자리 옆 (20 cm) 에 지나가는 벽, 박스는 지도 자리에서 12 cm 옆 → 벽이 아니라 박스에 섬
TEST(SimpleDock, WideSearchIgnoresWallBesideMapSpot)
{
  const auto r = run({0.0, 0.0, 0.0}, {0.58, -0.12}, 0.0, std::make_pair(Vec2{0.58, 0.0}, 0.0), {{{0.25, 0.37}, {1.0, 0.37}}});
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(std::abs(r.side) < 0.015 && 0.004 < r.face_gap && r.face_gap < 0.02) << info(r);
  EXPECT_GT(r.gap, 0.0) << info(r);
}

// 박스 뒤가 바로 벽 (긴 직선) → 벽이 아니라 박스 앞면에 섬
TEST(SimpleDock, BoxAgainstWallBehind)
{
  const auto r = run({0.0, 0.0, 0.0}, {0.6975, 0.1}, 0.0, std::make_pair(Vec2{0.58, 0.0}, 0.0), {{{0.8, -0.6}, {0.8, 0.6}}});
  ASSERT_EQ(r.st, DockState::Done) << info(r);
  EXPECT_TRUE(std::abs(r.side) < 0.015 && 0.004 < r.face_gap && r.face_gap < 0.02) << info(r);
}

// 고정 난수: 같은 입력이면 늘 같은 결과 (두 번 돌려 같음)
TEST(SimpleDock, Deterministic)
{
  const auto a = run({0.0, 0.05, rad(30)}, {0.58, -0.04}, rad(5), std::make_pair(Vec2{0.58, 0.0}, 0.0));
  const auto b = run({0.0, 0.05, rad(30)}, {0.58, -0.04}, rad(5), std::make_pair(Vec2{0.58, 0.0}, 0.0));
  EXPECT_EQ(a.pose.x, b.pose.x);
  EXPECT_EQ(a.pose.y, b.pose.y);
  EXPECT_EQ(a.why, b.why);
}
