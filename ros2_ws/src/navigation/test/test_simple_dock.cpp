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

// 박스가 지도 자리에서 10–22 cm 옆 · 앞뒤로, 돌아가 있음 → 넓게 찾기로 실제 박스 앞에 섬
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

// ---------------------------------------------------------------- FaceTrack 넓게 찾기 규칙 (rl_dock 이 켬)
namespace
{
// 지도 박스 (0.85, 0) · 로봇 원점에서 +x 를 봄 → 목표 = 박스 중심에서 kBoxDist 앞
FaceTrack wide_track()
{
  FaceTrack t({0.85 - kBoxDist, 0.0}, 0.0, kBoxDist, kBoxHalf, 0.20);
  t.confirm_n = 3;
  t.end_check = true;
  return t;
}
}  // namespace

// 실제 박스 (지도에서 +15 cm 옆 · 앞 5 cm, 6° 돌아감) → 연속 3 스캔 뒤 받음, 중심 1 cm 안
TEST(FaceTrackWide, ConfirmThenAcceptShiftedBox)
{
  auto t = wide_track();
  const Vec2 c{0.80, 0.15};
  const auto P = sample(box_segs(c, rad(6)));
  EXPECT_FALSE(t.update(P, {0.0, 0.0}));
  EXPECT_TRUE(t.pending());
  EXPECT_FALSE(t.update(P, {0.0, 0.0}));
  ASSERT_TRUE(t.update(P, {0.0, 0.0})) << t.last_why;
  EXPECT_EQ(t.n_ok, 1);
  EXPECT_LT(std::hypot(t.c.x - c.x, t.c.y - c.y), 0.01);
  EXPECT_NEAR(t.th, rad(6), rad(1.0));
}

// 확인 중 한 장 못 찾으면 처음부터 다시 셈
TEST(FaceTrackWide, ConfirmResetsOnMiss)
{
  auto t = wide_track();
  const auto P = sample(box_segs({0.85, 0.0}, 0.0));
  EXPECT_FALSE(t.update(P, {0.0, 0.0}));
  EXPECT_FALSE(t.update({}, {0.0, 0.0}));
  EXPECT_FALSE(t.pending());
  EXPECT_FALSE(t.update(P, {0.0, 0.0}));
  EXPECT_FALSE(t.update(P, {0.0, 0.0}));
  EXPECT_TRUE(t.update(P, {0.0, 0.0}));
}

// 앞 기둥 두 개 사이로 벽이 박스 앞면 길이 (21 cm) 만큼 보임 → 양 끝 가려짐으로 버림 (end_check 끄면 박스로 잡음)
TEST(FaceTrackWide, WallSeenThroughGapRejected)
{
  std::vector<Vec2> P = sample({{{0.75, -0.105}, {0.75, 0.105}}});          // 기둥에 안 가린 벽
  const auto posts = sample({{{0.5, 0.07}, {0.5, 0.30}}, {{0.5, -0.30}, {0.5, -0.07}}});
  P.insert(P.end(), posts.begin(), posts.end());
  auto t = wide_track();
  for (int i = 0; i < 5; ++i) {EXPECT_FALSE(t.update(P, {0.0, 0.0}));}
  EXPECT_EQ(t.n_ok, 0);
  EXPECT_EQ(t.last_why, "양 끝 가려짐");
  auto off = wide_track();
  off.end_check = false;
  bool got = false;
  for (int i = 0; i < 5 && !got; ++i) {got = off.update(P, {0.0, 0.0});}
  EXPECT_TRUE(got);
}

// 한 스캔 면 각 잡음 (라이다 1 cm 면 20 cm 면에서 ±3–11°) → 처음 확인은 각 10° 안이면 같은 자리로 봄 (3° 면 계속 리셋)
TEST(FaceTrackWide, ConfirmAcceptsYawJitter)
{
  auto t = wide_track();
  const Vec2 c{0.85, 0.0};
  EXPECT_FALSE(t.update(sample(box_segs(c, rad(0))), {0.0, 0.0}));
  EXPECT_FALSE(t.update(sample(box_segs(c, rad(6))), {0.0, 0.0}));
  ASSERT_TRUE(t.update(sample(box_segs(c, rad(-2))), {0.0, 0.0})) << t.last_why;
  EXPECT_EQ(t.n_ok, 1);
}

namespace
{
// 앞면만 (x = 면, y 6.5 mm 간격 = 0.75 m 에서 라이다 0.5°), 거리 방향 잡음 σ → 한 스캔 점
std::vector<Vec2> noisy_face(std::mt19937 & g, double sigma)
{
  std::normal_distribution<double> n(0.0, sigma);
  std::vector<Vec2> P;
  for (double y = -0.10; y <= 0.10 + 1e-9; y += 0.0065) {
    const double x = 0.85 - kBoxHalf;
    const double r = std::hypot(x, y) + n(g);
    const double a = std::atan2(y, x);
    P.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return P;
}

// 스캔 40 장 → 추정 면 각 오차 [°] (맞춤 창 fit_n 이 찬 뒤 매 장)
std::vector<double> yaw_errors(bool fit, int seed)
{
  std::mt19937 g(seed);
  auto t = wide_track();
  t.fit_face = fit;
  std::vector<double> e;
  for (int i = 0; i < 40; ++i) {
    if (t.update(noisy_face(g, 0.01), {0.0, 0.0}) && t.n_ok >= t.fit_n) {e.push_back(std::abs(t.th) * 180.0 / M_PI);}
  }
  return e;
}
}  // namespace

// 도착 방향: 최근 fit_n 스캔 앞면 띠 점을 합쳐 직선 맞춤 → 스캔마다 잰 각의 중앙값보다 정확 (잡음 1 cm, 같은 난수 스캔)
//   0.75 m 에서 서서 봄 (RMS 맞춤 약 0.8° · 중앙값 1.4°). 다가가며 보면 더 작음 (오프라인 폐루프 99 % 1.2°)
TEST(FaceTrackWide, FitFaceYawMoreAccurate)
{
  const auto rms = [](bool fit, double & mx, int & n) {
      double s = 0;
      mx = 0;
      n = 0;
      for (int seed = 0; seed < 20; ++seed) {
        for (double e : yaw_errors(fit, seed)) {s += e * e; mx = std::max(mx, e); ++n;}
      }
      return std::sqrt(s / std::max(n, 1));
    };
  double max_fit = 0, max_med = 0;
  int n_fit = 0, n_med = 0;
  const double rms_fit = rms(true, max_fit, n_fit), rms_med = rms(false, max_med, n_med);
  ASSERT_GT(n_fit, 200);
  ASSERT_GT(n_med, 200);
  EXPECT_LT(rms_fit, 0.65 * rms_med) << "맞춤 " << rms_fit << "° · 중앙값 " << rms_med << "°";
  EXPECT_LT(max_fit, 2.5) << "맞춤 최대 " << max_fit << "° · 중앙값 최대 " << max_med << "°";
}

// ---------------------------------------------------------------- 돌아간 · 옮겨진 B 박스 (경기 규칙: 무작위 회전, 10–15 cm 이동)
namespace
{
double fold_deg(double d) {return fold_quarter(rad(d)) * 180.0 / M_PI;}

// 끝 자세에서 가장 가까운 박스 면 (정사각 대칭) 기준 면 틈 · 옆 · 방향
void face_metrics(Result & r, Vec2 box_c, double box_yaw)
{
  const double by = r.pose.yaw + fold_quarter(box_yaw - r.pose.yaw);
  const Vec2 fwd{std::cos(by), std::sin(by)}, rel{box_c.x - r.pose.x, box_c.y - r.pose.y};
  r.face_gap = rel.x * fwd.x + rel.y * fwd.y - kBoxHalf - kFront;
  r.side = rel.x * -fwd.y + rel.y * fwd.x;
  r.dyaw = wrap(by - r.pose.yaw) * 180.0 / M_PI;
}
}  // namespace

// FaceTrack: 0–45° 아무 회전 · ±15 cm 옆 이동 → 접은 면 방향 (±45° 안) · 중심 1 cm 안 (로봇에서 보이는 면만)
class RotatedTrack : public ::testing::TestWithParam<std::array<double, 2>> {};
TEST_P(RotatedTrack, FoldsToNearestFace)
{
  const auto [yd, dy] = GetParam();
  auto t = wide_track();
  const Vec2 c{0.85 + 0.05, dy};
  const Pose2 robot{0.0, 0.0, 0.0};
  bool ok = false;
  for (int i = 0; i < 8; ++i) {ok = t.update(box_visible(c, rad(yd), robot), {0.0, 0.0}) || ok;}
  ASSERT_TRUE(ok) << yd << "° " << t.last_why;
  EXPECT_LT(std::hypot(t.c.x - c.x, t.c.y - c.y), 0.01) << yd << "° (" << t.c.x << ", " << t.c.y << ")";
  EXPECT_LT(std::abs(fold_quarter(t.th - rad(yd))), rad(1.5)) << yd << "° → " << t.th * 180 / M_PI;
  EXPECT_LE(std::abs(t.th), rad(46.0));
}
INSTANTIATE_TEST_SUITE_P(FaceTrackRot, RotatedTrack, ::testing::Values(
    std::array<double, 2>{15, 0.0}, std::array<double, 2>{30, 0.0}, std::array<double, 2>{39, 0.0}, std::array<double, 2>{-39, 0.0},
    std::array<double, 2>{45, 0.0}, std::array<double, 2>{60, 0.10}, std::array<double, 2>{-75, -0.15}, std::array<double, 2>{39, 0.15},
    std::array<double, 2>{30, -0.15}, std::array<double, 2>{89, 0.05}));

// 45° 근처 (44 · 46° 번갈아) → 면이 ±45° 경계에서 안 갈림 (중앙값이 0° 로 섞이지 않음)
TEST(FaceTrackRot, NoFlipAt45)
{
  auto t = wide_track();
  const Vec2 c{0.85, 0.0};
  for (int i = 0; i < 20; ++i) {t.update(box_visible(c, rad(i % 2 ? 44.0 : 46.0), {0.0, 0.0, 0.0}), {0.0, 0.0});}
  ASSERT_GT(t.n_ok, 5) << t.last_why;
  EXPECT_NEAR(std::abs(t.th), rad(45.0), rad(2.0)) << t.th * 180 / M_PI;
}

// 규칙 정밀 접근 (dock:=simple): 돌아간 · 옮겨진 박스 → 가까운 면 정면 1 cm 에 섬, 몸이 박스에 안 닿음
class RotatedDock : public ::testing::TestWithParam<std::array<double, 3>> {};
TEST_P(RotatedDock, DocksSquareToFace)
{
  const auto [yd, dx, dy] = GetParam();
  const Vec2 bc{0.58 + dx, dy};
  auto r = run({0.0, 0.0, 0.0}, bc, rad(yd), std::make_pair(Vec2{0.58, 0.0}, 0.0));
  face_metrics(r, bc, rad(yd));
  ASSERT_EQ(r.st, DockState::Done) << yd << "° " << info(r);
  EXPECT_TRUE(0.004 < r.face_gap && r.face_gap < 0.02 && std::abs(r.side) < 0.015 && std::abs(r.dyaw) < 3.0) << yd << "° " << info(r);
  EXPECT_GT(r.gap, 0.0) << info(r);
}
INSTANTIATE_TEST_SUITE_P(SimpleDockRot, RotatedDock, ::testing::Values(
    std::array<double, 3>{15, 0.0, 0.0}, std::array<double, 3>{30, 0.0, 0.0}, std::array<double, 3>{39, 0.0, 0.0},
    std::array<double, 3>{-39, 0.05, 0.10}, std::array<double, 3>{45, 0.0, 0.0}, std::array<double, 3>{39, -0.10, -0.12},
    std::array<double, 3>{30, 0.10, 0.15}));

// ---------------------------------------------------------------- rl_dock 기하 (정책 대신 직선 따라가기): FaceTrack · straight_blocked · box_via
//   노드 (local_planner_node run) 와 같은 순서: 스캔 → FaceTrack 갱신 → 정차점 (먼저 갈 곳) → 직선 막힘 검사 (연속 3 스캔이면 실패)
namespace
{
struct RlRes
{
  bool ok = false;
  std::string why;
  Pose2 pose;
  double gap = 9.0;
  bool via = false;
  int n_ok = 0;
};

// start 에서 출발, 0.12 m/s · 0.4 rad/s 로 지금 목표까지 곧게 (yaw 는 남은 거리 비례). 박스 bc · byaw, 지도 박스 (0.58, 0)
RlRes rl_run(Pose2 start, Vec2 bc, double byaw, const std::vector<Seg> & walls = {}, double search = 0.20)
{
  FaceTrack tr({0.58 - kBoxDist, 0.0}, 0.0, kBoxDist, kBoxHalf, search);
  tr.confirm_n = 3;
  tr.end_check = true;
  const Vec2 body{0.145, 0.153};
  Pose2 pose = start, goal = tr.goal(), final_goal = goal;
  bool via_on = false;
  double via_back = 0.0;
  int n_block = 0;
  const auto wp = sample(walls);
  const auto box_all = sample(box_segs(bc, byaw));
  RlRes r;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.003);
  for (int it = 0; it < 600; ++it) {                    // 20 Hz, 스캔 = 매 주기
    std::vector<Vec2> W = box_visible(bc, byaw, pose);
    W.insert(W.end(), wp.begin(), wp.end());
    std::vector<Vec2> odom;
    for (auto p : visible(W, pose)) {
      p.x += noise(rng); p.y += noise(rng);
      if (std::abs(p.x) < 0.17 && std::abs(p.y) < 0.15) {continue;}
      const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
      odom.push_back({c * p.x - s * p.y + pose.x, s * p.x + c * p.y + pose.y});
    }
    if (tr.update(odom, {pose.x, pose.y})) {
      const Pose2 g = tr.goal();
      if (std::hypot(g.x - final_goal.x, g.y - final_goal.y) > 0.003 || std::abs(wrap(g.yaw - final_goal.yaw)) > rad(0.2)) {
        final_goal = g;
        if (!via_on) {
          if (const auto v = navigation::rl::box_via(pose, g, tr.c, tr.yaw0 + tr.th, tr.h, body)) {
            via_on = true;
            r.via = true;
            via_back = std::hypot(v->x - g.x, v->y - g.y);
          }
        }
        goal = via_on ? Pose2{g.x - via_back * std::cos(g.yaw), g.y - via_back * std::sin(g.yaw), g.yaw} : g;
      }
    }
    const double unk = tr.n_ok == 0 ? search + kBoxHalf * std::sqrt(2.0) + 0.03 : 0.0;
    const auto [nb, at] = navigation::rl::straight_blocked(pose, goal, tr.c, tr.yaw0 + tr.th, tr.h, odom, body, unk);
    n_block = nb ? n_block + 1 : 0;
    if (n_block >= 3) {
      r.why = "막힘 " + std::to_string(nb) + " 점 " + std::to_string(at);
      break;
    }
    if (via_on && std::hypot(goal.x - pose.x, goal.y - pose.y) < 0.03 && std::abs(wrap(goal.yaw - pose.yaw)) < rad(5)) {
      via_on = false;
      goal = final_goal;
    }
    const double ex = goal.x - pose.x, ey = goal.y - pose.y, d = std::hypot(ex, ey), dpsi = wrap(goal.yaw - pose.yaw);
    if (!via_on && d < 0.005 && std::abs(dpsi) < rad(1.0)) {r.ok = true; break;}
    const double v = std::min(0.12, 2.0 * d), T = std::max(d / std::max(v, 1e-6), std::abs(dpsi) / 0.4);
    const double k = std::min(1.0, kDt / std::max(T, kDt));   // 직선 위 · yaw 고르게 (직선 경로를 따라가는 정책 흉내)
    pose = {pose.x + k * ex, pose.y + k * ey, wrap(pose.yaw + k * dpsi)};
    std::vector<Vec2> all = box_all;
    all.insert(all.end(), wp.begin(), wp.end());
    r.gap = std::min(r.gap, body_gap(all, pose));
  }
  r.pose = pose;
  r.n_ok = tr.n_ok;
  return r;
}
}  // namespace

// 출발 = Nav2 넘김 (정차점 뒤 0.33 m). 0–45° 회전 · ±15 cm → 막힘 없이 가까운 면 정면, 몸 틈 > 0 (모서리 안 침)
class RlRotated : public ::testing::TestWithParam<std::array<double, 4>> {};
TEST_P(RlRotated, DocksWithoutCorner)
{
  const auto [yd, dx, dy, back] = GetParam();
  const Vec2 bc{0.58 + dx, dy};
  const auto r = rl_run({-back, 0.0, 0.0}, bc, rad(yd));
  Result m;
  m.pose = r.pose;
  face_metrics(m, bc, rad(yd));
  ASSERT_TRUE(r.ok) << yd << "° " << r.why << " 측정 " << r.n_ok;
  EXPECT_TRUE(0.006 < m.face_gap && m.face_gap < 0.016 && std::abs(m.side) < 0.012 && std::abs(m.dyaw) < 2.5)
    << yd << "° 면 " << m.face_gap << " 옆 " << m.side << " yaw " << m.dyaw;
  EXPECT_GT(r.gap, 0.004) << yd << "° 몸 틈 " << r.gap << (r.via ? " (먼저 갈 곳)" : "");
}
INSTANTIATE_TEST_SUITE_P(RlDockRot, RlRotated, ::testing::Values(
    std::array<double, 4>{0, 0.0, 0.0, 0.33}, std::array<double, 4>{15, 0.0, 0.0, 0.33}, std::array<double, 4>{30, 0.0, 0.0, 0.33},
    std::array<double, 4>{39, 0.0, 0.0, 0.33}, std::array<double, 4>{-39, 0.0, 0.0, 0.33}, std::array<double, 4>{45, 0.0, 0.0, 0.33},
    std::array<double, 4>{39, 0.0, 0.14, 0.33}, std::array<double, 4>{-30, 0.05, -0.15, 0.33}, std::array<double, 4>{45, -0.10, 0.10, 0.33},
    std::array<double, 4>{39, 0.0, 0.0, 0.05}, std::array<double, 4>{45, 0.0, 0.10, 0.02}, std::array<double, 4>{-39, 0.0, -0.10, 0.10}));

// 재현: B2 (39° 돎 · 박스 옆 10 cm · 앞 9 cm) — 막힘 없이 섬
TEST(RlDockRot, B2Like)
{
  const Vec2 bc{0.58 - 0.09, 0.10};
  const auto r = rl_run({-0.33, 0.0, 0.0}, bc, rad(-39));
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_GT(r.gap, 0.004);
}

// 박스 옆 벽 (박스 옆면에서 12 cm, 접근 방향과 나란히) · 박스 뒤 벽 → 벽은 박스로 안 봄 · 몸이 벽 · 박스에 안 닿음
TEST(RlDockRot, WallsNearRotatedBox)
{
  const Vec2 bc{0.58, 0.0};
  const std::vector<Seg> walls{{{-0.6, 0.47}, {1.2, 0.47}}, {{0.83, -0.6}, {0.83, 0.47}}};
  const auto r = rl_run({-0.33, 0.0, 0.0}, bc, rad(30), walls);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_GT(r.gap, 0.004);
}

// 정차점으로 가는 직선 위 진짜 벽 (박스 아님) → 연속 3 스캔 막힘으로 멈춤 (박스 원 · 정사각 밖)
TEST(RlDockRot, RealObstacleStillStops)
{
  const auto r = rl_run({-0.33, 0.0, 0.0}, {0.58, 0.0}, rad(30), {{{-0.02, -0.03}, {-0.02, 0.03}}});
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.why.find("막힘"), std::string::npos) << r.why;
  EXPECT_GT(r.gap, 0.0);
}

namespace
{
// rl_dock 과 같은 설정 (넓게 찾기 0.30 · 확인 3 · 끝 가림 검사) + 가까이 확인. 목표 = 원점 +x
FaceTrack near_track(double near_shift)
{
  FaceTrack t({0.0, 0.0}, 0.0, 0.2575, kBoxHalf, 0.30);
  t.confirm_n = 3;
  t.end_check = true;
  t.near_shift = near_shift;
  return t;
}

// 로봇 (몸 좌표 = map, yaw 0) 에서 본 박스 점 + scan_merger 자기 몸 거르기 (앞 x ≤ 0.17 · |y| ≤ 0.15 버림)
std::vector<Vec2> scan_at(Vec2 c, Vec2 robot)
{
  std::vector<Vec2> out;
  for (const auto & p : box_visible(c, 0.0, {robot.x, robot.y, 0.0})) {
    const double bx = p.x - robot.x, by = p.y - robot.y;
    if (bx <= 0.17 && bx >= -0.17 && std::abs(by) <= 0.15) {continue;}
    out.push_back(p);
  }
  return out;
}
}  // namespace

// 정차가 실제 박스보다 6 cm 멂 (면 틈 7 cm) → freeze 안이어도 면이 보임 → 확인 3 장 뒤 목표를 고쳐 더 붙음
TEST(FaceTrackNear, FartherBoxSeenInsideFreezeMovesGoal)
{
  auto t = near_track(0.02);
  const Vec2 real{0.2575 + 0.06, 0.03};
  EXPECT_FALSE(t.update(scan_at(real, {0.0, 0.0}), {0.0, 0.0}));
  EXPECT_EQ(t.last_why, "freeze");                     // 받기 전엔 freeze 그대로 (노드가 '버림' 으로 세지 않게)
  EXPECT_FALSE(t.update(scan_at(real, {0.0, 0.0}), {0.0, 0.0}));
  ASSERT_TRUE(t.update(scan_at(real, {0.0, 0.0}), {0.0, 0.0})) << t.near_why;
  EXPECT_EQ(t.n_near, 1);
  const Pose2 g = t.goal();
  EXPECT_NEAR(g.x, 0.06, 0.005);
  EXPECT_NEAR(g.y, 0.03, 0.01);
  EXPECT_NEAR(g.yaw, 0.0, 2.0 * M_PI / 180.0);
  // 고친 목표에 붙으면 면이 몸 거르기 안 → 더 안 고침
  for (int i = 0; i < 5; ++i) {EXPECT_FALSE(t.update(scan_at(real, {g.x, g.y}), {g.x, g.y}));}
  EXPECT_EQ(t.n_near, 1);
}

// 맞는 박스: 다가오는 중 (목표 5 cm 앞, 면 보임) · 정차 (면 안 보임) 둘 다 목표 안 바꿈. 가까이 확인 끔 (simple_dock) 도 안 바꿈
TEST(FaceTrackNear, CorrectBoxOrDisabledKeepsGoal)
{
  const Vec2 real{0.2575, 0.0};
  for (const double r : {-0.05, 0.0}) {
    auto t = near_track(0.02);
    for (int i = 0; i < 6; ++i) {EXPECT_FALSE(t.update(scan_at(real, {r, 0.0}), {r, 0.0})) << r << " " << t.near_why;}
    EXPECT_EQ(t.n_near, 0);
  }
  auto off = near_track(0.0);
  const Vec2 far{0.2575 + 0.06, 0.0};
  for (int i = 0; i < 6; ++i) {EXPECT_FALSE(off.update(scan_at(far, {0.0, 0.0}), {0.0, 0.0}));}
  EXPECT_EQ(off.n_near, 0);
}

// 면 바꾸기: 45° 박스에서 고른 면을 +90° 옆 면으로 바꾸면 그 뒤 측정도 새 면 기준 (원래 면으로 안 돌아감), 중심은 그대로
TEST(FaceTrackRot, SwitchFaceKeepsNewFace)
{
  auto t = wide_track();
  const Vec2 c{0.85, 0.0};
  for (int i = 0; i < 6; ++i) {t.update(box_visible(c, rad(44.0), {0.0, 0.0, 0.0}), {0.0, 0.0});}
  ASSERT_GT(t.n_ok, 2) << t.last_why;
  const double th0 = t.th;
  t.switch_face(th0 > 0 ? -M_PI / 2 : M_PI / 2);
  for (int i = 0; i < 6; ++i) {t.update(box_visible(c, rad(44.0), {0.0, 0.0, 0.0}), {0.0, 0.0});}
  EXPECT_NEAR(std::abs(t.th - th0), M_PI / 2, rad(2.0)) << th0 * 180 / M_PI << " → " << t.th * 180 / M_PI;
  EXPECT_LT(std::hypot(t.c.x - c.x, t.c.y - c.y), 0.01);
}
