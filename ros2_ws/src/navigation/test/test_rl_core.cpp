// test_rl_core.cpp — RL 지역 계획기 계산 시험 (ROS 없음): 경로 투영 · npz 정책 · 라이다 칸 · 명령 한계 · 관측 · 박스 맞춤
//   정책 기준값 = 파이썬 판 scripts/numpy_policy.py 로 같은 관측을 넣은 결과
//   실행: colcon build → ./build/navigation/test_rl_core

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "navigation/rl_core.hpp"

using namespace navigation::rl;

namespace
{
const std::string kDir = RL_CONFIG_DIR;

double deg2rad(double d) {return d * M_PI / 180.0;}

// 박스 (중심 c, yaw = 로봇이 볼 방향) 의 로봇 쪽 앞면 점 (간격 1 cm, 반 변 half)
std::vector<Vec2> face(Vec2 c, double yaw, double half = 0.1025, double step = 0.01)
{
  const Vec2 n{-std::cos(yaw), -std::sin(yaw)}, t{-n.y, n.x};
  std::vector<Vec2> pts;
  for (double s = -half; s <= half + 1e-9; s += step) {
    pts.push_back({c.x + half * n.x + s * t.x, c.y + half * n.y + s * t.y});
  }
  return pts;
}
}  // namespace

TEST(RlCore, Wrap)
{
  EXPECT_NEAR(wrap(3 * M_PI / 2), -M_PI / 2, 1e-12);
  EXPECT_NEAR(wrap(-M_PI), -M_PI, 1e-12);            // [−π, π)
  EXPECT_NEAR(wrap(M_PI), -M_PI, 1e-12);
  const Pose2 T{1.0, 2.0, 0.7};
  const Pose2 p = compose(inverse(T), compose(T, {0.3, -0.4, 0.2}));
  EXPECT_NEAR(p.x, 0.3, 1e-12);
  EXPECT_NEAR(p.y, -0.4, 1e-12);
  EXPECT_NEAR(p.yaw, 0.2, 1e-12);
}

TEST(RlCore, PathPointAndProject)
{
  const RlPath p({{0, 0}, {1, 0}, {1, 0}, {1, 1}});   // 길이 0 구간 포함
  EXPECT_DOUBLE_EQ(p.length(), 2.0);
  EXPECT_NEAR(p.point_at(0.5).x, 0.5, 1e-12);
  EXPECT_NEAR(p.point_at(1.5).y, 0.5, 1e-12);
  EXPECT_NEAR(p.point_at(5.0).y, 1.0, 1e-12);        // 끝에서 자름
  EXPECT_NEAR(p.point_at(-1.0).x, 0.0, 1e-12);
  auto [s, d] = p.project({0.3, 0.2});
  EXPECT_NEAR(s, 0.3, 1e-12);
  EXPECT_NEAR(d, 0.2, 1e-12);
  // 창 안 투영: 이전 s 0.2 면 0.2 + 0.5 = 0.7 까지만
  std::tie(s, d) = p.project({1.1, 0.8}, 0.2);
  EXPECT_NEAR(s, 0.7, 1e-12);
  std::tie(s, d) = p.project({1.1, 0.8});
  EXPECT_NEAR(s, 1.8, 1e-12);
  EXPECT_NEAR(d, 0.1, 1e-12);
}

TEST(RlCore, PolicyMatchesPython)
{
  struct Ref
  {
    const char * file;
    std::array<double, 3> zero, ramp;
  };
  const std::vector<Ref> refs{
    {"nav_nf5.npz", {-0.10425144404332384, 0.7182991170358316, -0.1529275171779417},
      {-0.9253160581362398, 0.6037912651394062, 0.986669640883779}},
    {"dock3.npz", {-1.5040869728931108, -0.6226160277955339, 0.09224585289358908},
      {-1.5264149064994286, -0.9709759062021162, 0.25202828850999676}}};
  for (const auto & r : refs) {
    const NpzPolicy pol(kDir + "/" + r.file);
    ASSERT_EQ(pol.input_dim(), 161) << r.file;
    std::vector<float> zero(161, 0.0f), ramp(161);
    for (int i = 0; i < 161; ++i) {ramp[i] = static_cast<float>(i % 7 - 3) * 0.1f;}
    const auto a = pol(zero), b = pol(ramp);
    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(a[k], r.zero[k], 1e-9) << r.file;
      EXPECT_NEAR(b[k], r.ramp[k], 1e-9) << r.file;
    }
  }
  EXPECT_THROW(NpzPolicy(kDir + "/없음.npz"), std::runtime_error);
  EXPECT_THROW(NpzPolicy(kDir + "/nav_nf5.json"), std::runtime_error);
}

TEST(RlCore, LoadConfigs)
{
  const auto c = load_env_cfg(kDir + "/nav_nf5.json");
  EXPECT_DOUBLE_EQ(c.vmax, 0.255);
  EXPECT_DOUBLE_EQ(c.wmax, 0.5);
  EXPECT_EQ(c.lidar_rays, 36);
  EXPECT_EQ(c.history, 4);
  EXPECT_TRUE(c.approach_limit);
  EXPECT_TRUE(c.rot_limit);
  EXPECT_DOUBLE_EQ(c.lidar_body_x, 0.17);
  EXPECT_DOUBLE_EQ(load_env_cfg(kDir + "/dock3.json").wmax, 0.3);
  const auto sw = load_swerve(kDir + "/team_swerve.yaml");
  EXPECT_EQ(sw.steer[0], "steer_fl_joint");
  EXPECT_EQ(sw.drive[3], "drive_rr_joint");
  EXPECT_GT(sw.max_wheel, 0.0);
  EXPECT_LT(sw.rmin[0], sw.rmax[0]);
}

TEST(RlCore, LidarBins)
{
  EnvCfg c;
  // 칸 10°: 5° = 0.5 칸 → 0 (짝수 쪽), 15° = 1.5 칸 → 2, −5° → 0, 몸 상자 안 점 뺌, 빈 칸 = lidar_max
  const auto at = [](double deg, double r) {return Vec2{r * std::cos(deg2rad(deg)), r * std::sin(deg2rad(deg))};};
  const auto b = lidar_bins({at(5, 1.0), at(15, 0.8), at(-5, 0.6), at(180, 2.0), {0.1, 0.1}}, c);
  ASSERT_EQ(b.size(), 36u);
  EXPECT_NEAR(b[0], 0.6, 1e-12);
  EXPECT_NEAR(b[2], 0.8, 1e-12);
  EXPECT_NEAR(b[18], 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(b[1], c.lidar_max);
  EXPECT_DOUBLE_EQ(b[9], c.lidar_max);
}

TEST(RlCore, CommandLimits)
{
  const auto c = load_env_cfg(kDir + "/nav_nf5.json");
  // 행동 자르기 · 병진 크기 ≤ vmax
  auto u = command(c, {2.0, 2.0, 0.0}, std::nullopt, 0.0, 5.0);
  EXPECT_NEAR(std::hypot(u[0], u[1]), c.vmax, 1e-12);
  // 목표 근처 상한 0.02 + 0.5 d
  u = command(c, {1.0, 0.0, 0.0}, 0.1, 0.0, 5.0);
  EXPECT_NEAR(u[0], 0.07, 1e-12);
  // 추가 상한 (출발 자리 영역 끈 동안)
  u = command(c, {1.0, 0.0, 0.0}, 5.0, 0.10, 5.0);
  EXPECT_NEAR(u[0], 0.10, 1e-12);
  // 벽 가까우면 회전 상한: 0.24 이하 = rot_wz_lo, 0.32 이상 = wmax, 스캔 없음 = 0 으로 봄
  EXPECT_NEAR(command(c, {0, 0, 1.0}, 5.0, 0.0, 0.20)[2], c.rot_wz_lo, 1e-12);
  EXPECT_NEAR(command(c, {0, 0, 1.0}, 5.0, 0.0, 0.40)[2], c.wmax, 1e-12);
  EXPECT_NEAR(command(c, {0, 0, -1.0}, 5.0, 0.0, 0.28)[2], -(c.rot_wz_lo + (c.wmax - c.rot_wz_lo) * 0.5), 1e-9);
  EXPECT_NEAR(command(c, {0, 0, 1.0}, 5.0, 0.0, std::nullopt)[2], c.rot_wz_lo, 1e-12);
}

TEST(RlCore, ObservationLayoutAndHistory)
{
  const auto c = load_env_cfg(kDir + "/nav_nf5.json");
  const auto sw = load_swerve(kDir + "/team_swerve.yaml");
  const RlPath path({{0, 0}, {2, 0}});
  const std::vector<double> scan(c.lidar_rays, 1.0);
  JointObs j;
  j.odom = {0.1, 0.0, 0.2};
  std::vector<std::vector<double>> hist;
  auto o = build_obs(c, sw, path, {0.5, 0.1, 0.0}, 0.5, {2.0, 0.0, M_PI / 2}, scan, j, {0.05, 0, 0}, hist);
  ASSERT_EQ(o.size(), 161u);
  EXPECT_NEAR(o[0], 0.0, 1e-6);                         // 첫 경로 점 = 몸 기준 (0, −0.1)
  EXPECT_NEAR(o[1], -0.1, 1e-6);
  EXPECT_NEAR(o[2], 0.1, 1e-6);                         // 다음 점 0.1 m 앞
  EXPECT_NEAR(o[32], 0.75, 1e-6);                       // 목표 몸 x / 2
  EXPECT_NEAR(o[34], 1.0, 1e-6);                        // sin(dyaw)
  EXPECT_NEAR(o[36], 0.5, 1e-6);                        // 남은 1.5 m / 3
  EXPECT_NEAR(o[37], 0.5, 1e-6);                        // 라이다 1.0 / clip 2.0
  ASSERT_EQ(hist.size(), 4u);
  EXPECT_EQ(hist[0], hist[3]);                          // 처음엔 같은 프레임으로 채움
  const int f0 = 2 * c.path_obs_points + 5 + c.lidar_rays;
  EXPECT_NEAR(o[f0 + 16], 0.1 / c.vmax, 1e-6);          // odom vx / vmax
  EXPECT_NEAR(o[f0 + 19], 0.05 / c.vmax, 1e-6);         // last_cmd vx / vmax
  j.odom = {0.2, 0.0, 0.0};
  o = build_obs(c, sw, path, {0.6, 0.1, 0.0}, 0.6, {2.0, 0.0, M_PI / 2}, scan, j, {0, 0, 0}, hist);
  EXPECT_NEAR(o[f0 + 16], 0.2 / c.vmax, 1e-6);          // 최신 프레임 먼저
  EXPECT_NEAR(o[f0 + 22 + 16], 0.1 / c.vmax, 1e-6);     // 한 칸 밀린 옛 프레임
}

TEST(RlCore, BoxTrackerFollowsShiftedBox)
{
  // 지도 정차점 (0, 0) yaw 0 → 지도 박스 중심 (0.2575, 0). 실제 박스는 (+2 cm, −3 cm) · +5° 돌아감
  BoxTracker tr({0.0, 0.0}, 0.0);
  EXPECT_NEAR(tr.c0.x, 0.2575, 1e-12);
  const Vec2 real{0.2775, -0.03};
  const double ry = deg2rad(5.0);
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.002);
  for (int k = 0; k < 6; ++k) {
    auto pts = face(real, ry);
    for (auto & p : pts) {p.x += noise(rng); p.y += noise(rng);}
    EXPECT_TRUE(tr.update(pts, {-0.3, 0.0})) << tr.last_why;
  }
  EXPECT_NEAR(tr.c.x, real.x, 0.005);
  EXPECT_NEAR(tr.c.y, real.y, 0.005);
  EXPECT_NEAR(tr.th, ry, deg2rad(1.0));
  const Pose2 g = tr.goal();                            // 잰 박스 중심에서 0.2575, 박스 정면
  EXPECT_NEAR(std::hypot(real.x - g.x, real.y - g.y), 0.2575, 0.006);
  EXPECT_NEAR(g.yaw, ry, deg2rad(1.0));
  // 목표 0.10 m 안이면 안 고침
  EXPECT_FALSE(tr.update(face(real, ry), {g.x - 0.05, g.y}));
  EXPECT_EQ(tr.last_why, "freeze");
}

TEST(RlCore, BoxTrackerRejectsWallAndFar)
{
  BoxTracker tr({0.0, 0.0}, 0.0);
  // 박스 자리에 긴 벽 (40 cm) → 면 길이 (벽)
  std::vector<Vec2> wall;
  for (double y = -0.2; y <= 0.2 + 1e-9; y += 0.01) {wall.push_back({0.155, y});}
  EXPECT_FALSE(tr.update(wall, {-0.3, 0.0}));
  EXPECT_NE(tr.last_why.find("벽"), std::string::npos) << tr.last_why;
  EXPECT_EQ(tr.n_wall, 1);
  // 점 없음
  EXPECT_FALSE(tr.update({}, {-0.3, 0.0}));
  EXPECT_EQ(tr.last_why, "점 0");
  // 20° 돌아간 면 → 기울기 (한계 15°)
  EXPECT_FALSE(tr.update(face(tr.c0, deg2rad(20.0)), {-0.3, 0.0}));
  EXPECT_EQ(tr.last_why, "기울기 20°");
  EXPECT_EQ(tr.n_ok, 0);
  // 옆으로 10 cm 밀린 면은 창 안에 한쪽 모서리만 → 옆은 이전 추정 유지 → 받음 (파이썬 판과 같음)
  BoxTracker side({0.0, 0.0}, 0.0);
  EXPECT_TRUE(side.update(face({0.2575, 0.10}, 0.0), {-0.3, 0.0})) << side.last_why;
  EXPECT_NEAR(side.c.y, 0.0, 1e-9);
  // 앞으로 7 cm → 받음, 그다음 9 cm → 지도에서 9 cm (한계 8 cm)
  BoxTracker far({0.0, 0.0}, 0.0);
  EXPECT_TRUE(far.update(face({0.3275, 0.0}, 0.0), {-0.3, 0.0})) << far.last_why;
  EXPECT_FALSE(far.update(face({0.3475, 0.0}, 0.0), {-0.3, 0.0}));
  EXPECT_EQ(far.last_why, "지도에서 9 cm");
  EXPECT_EQ(far.n_ok, 1);
}

TEST(RlCore, StraightBlockedIgnoresBox)
{
  BoxTracker tr({0.0, 0.0}, 0.0);
  const auto box = face(tr.c0, 0.0);
  // 박스 앞면 점만 → 안 막힘 (박스는 늘 정차 자리에 걸리므로 뺌)
  EXPECT_EQ(straight_blocked({-0.3, 0.0, 0.0}, {0.0, 0.0, 0.0}, tr, box, {0.145, 0.153}).first, 0);
  // 가는 길 옆 10 cm 에 벽 점 → 막힘
  auto pts = box;
  pts.push_back({-0.15, 0.16});
  const auto [n, at] = straight_blocked({-0.3, 0.0, 0.0}, {0.0, 0.0, 0.0}, tr, pts, {0.145, 0.153});
  EXPECT_EQ(n, 1);
  EXPECT_GE(at, 0.0);
  EXPECT_LE(at, 0.3);
}
