// test_arm_kinematics.cpp — C++ 팔 기구학이 실물 집기 계산기(grasp_test · sag_model)와 같은 답을 내나
// 정답 = test/arm_kin_data.hpp (실물 계산기 결과)

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm_kinematics.hpp"
#include "arm_kin_data.hpp"

using manipulation::Ticks;

namespace
{
int max_diff(const Ticks & a, const Ticks & b)
{
  int d = 0;
  for (int j = 0; j < 5; ++j) {d = std::max(d, std::abs(a[j] - b[j]));}
  return d;
}
}  // namespace

// 1) 틱 → 관절 → 틱 왕복
TEST(ArmKinematics, TickRoundTrip)
{
  const auto m = kin_data::model();
  const Ticks t{{4084, 1102, 2668, 1971, 2063}};
  EXPECT_EQ(manipulation::q_to_ticks(m, manipulation::ticks_to_q(m, t)), t);
}

// 2) FK 손끝 · 접근축 · 닫힘축, 중력 토크, 처짐 앞먹임 명령이 실물 계산기와 같음
TEST(ArmKinematics, ForwardAndSagMatchRealCalculator)
{
  const auto m = kin_data::model();
  for (const auto & c : kin_data::fk_cases) {
    const auto q = manipulation::ticks_to_q(m, c.ticks);
    const auto g = manipulation::arm_grip(m, q);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(g.tip(i), c.tip[i], 1e-9);
      EXPECT_NEAR(g.app(i), c.app[i], 1e-9);
      EXPECT_NEAR(g.close(i), c.close[i], 1e-9);
    }
    const auto tau = manipulation::sag_torques(m, q, 0.017);
    for (int i = 0; i < 3; ++i) {EXPECT_NEAR(tau[i], c.tau[i], 1e-9);}
    EXPECT_EQ(manipulation::q_to_ticks(m, manipulation::encoder_goal_q(m, q, 0.017)), c.enc_held);
  }
}

// 3) 퍽 집기 자세 (위 40 · 10 · 쥐기 −30 mm): 실물 계산기와 ±2 틱. 실물 계산기가 못 푸는 자리는 C++ 도 못 풂
TEST(ArmKinematics, PickPosesMatchRealCalculator)
{
  const auto m = kin_data::model();
  const double top = 0.070 - 0.072 + 0.060;   // 박스 윗면 + 퍽 높이 (모델 base 기준, grasp_test 와 같음)
  for (const auto & c : kin_data::pick_cases) {
    std::string why;
    const auto P = manipulation::pick_poses(m, c.x, c.y, top, c.yaw, {40, 10, -30}, why);
    if (!c.ok) {
      EXPECT_TRUE(P.empty()) << c.x << ", " << c.y;
      continue;
    }
    ASSERT_EQ(P.size(), 3u) << why;
    for (int k = 0; k < 3; ++k) {
      EXPECT_LE(max_diff(P[k].ticks, c.ticks[k]), 2) << c.x << ", " << c.y << " " << P[k].name;
      EXPECT_LE(max_diff(P[k].enc, c.enc[k]), 2) << c.x << ", " << c.y << " " << P[k].name;
    }
  }
}

// 4) 팀 명령 rad = (틱 − 영점) · 2π/4096
TEST(ArmKinematics, TeamCommand)
{
  const auto m = kin_data::model();
  const auto c = manipulation::ticks_to_command(m, Ticks{{4084, 1102, 2668, 1971, 2063}});
  EXPECT_NEAR(c[0], -0.0061, 1e-4);
  EXPECT_NEAR(c[1], -1.4511, 1e-4);
  EXPECT_NEAR(c[2], 0.9511, 1e-4);
}

// 닫힘 방향을 직접 주면 손가락 방향(180° 주기)을 정확히 맞춤: 기본 방향 = 예전과 같은 해, 90° 돌리면 J5 가 약 90° 돎
TEST(ArmKinematics, ExplicitCloseAxisTurnsWrist)
{
  const auto m = kin_data::model();
  std::string why;
  const double def = manipulation::default_close_rad(m, 0.2102, 0.038, -4.0) * 180.0 / M_PI;
  const auto a = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why);
  const auto b = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why, def);
  const auto c = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why, def - 90.0);
  ASSERT_EQ(a.size(), 3u);
  ASSERT_EQ(b.size(), 3u);
  ASSERT_EQ(c.size(), 3u) << why;
  for (int j = 0; j < 5; ++j) {EXPECT_EQ(a[2].ticks[j], b[2].ticks[j]);}
  EXPECT_NEAR(std::abs(c[2].ticks[4] - a[2].ticks[4]), 1024, 80);   // 90° = 1024 틱 (손끝 위치를 맞추느라 조금 다름)
}

// 먼 퍽: 수직 손은 안 닿고, 기울이면 닿음 (10° 씩, 가장 작은 기울임). 가까운 퍽은 기울임 허용해도 예전과 같은 수직 해
TEST(ArmKinematics, TiltedHandReachesFarther)
{
  const auto m = kin_data::model();
  std::string why;
  EXPECT_TRUE(manipulation::pick_poses(m, 0.30, 0.0, 0.058, -4.0, {40, 10, -30}, why).empty());
  EXPECT_FALSE(why.empty());
  const auto far = manipulation::pick_poses(m, 0.30, 0.0, 0.058, -4.0, {40, 10, -30}, why, std::nan(""), 30.0);
  ASSERT_EQ(far.size(), 3u) << why;
  EXPECT_GT(far.back().tilt_deg, 0.0);
  EXPECT_LE(far.back().tilt_deg, 30.0);
  const auto a = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why);
  const auto b = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why, std::nan(""), 30.0);
  ASSERT_EQ(b.size(), 3u);
  EXPECT_DOUBLE_EQ(b.back().tilt_deg, 0.0);
  for (int j = 0; j < 5; ++j) {EXPECT_EQ(a[2].ticks[j], b[2].ticks[j]);}
}

// 선호 기울임 (PLACE: 쥘 때 기울임) 에 가까운 것부터 · 차이 한도 밖은 안 씀
TEST(ArmKinematics, TiltPreferenceAndLimit)
{
  const auto m = kin_data::model();
  std::string why;
  // 가까운 자리: 기울임 20° 선호 · 한도 10° → 10~30° 중 되는 것, 수직(0°)은 안 씀
  const auto p = manipulation::pick_poses(m, 0.21, 0.0, 0.058, std::nan(""), {40, 10, -23}, why, std::nan(""), 30.0, 20.0, 10.0);
  if (!p.empty()) {EXPECT_GE(p.back().tilt_deg, 10.0);}
  // 먼 자리 (수직 안 닿음) · 선호 0° · 한도 0° → 거절
  EXPECT_TRUE(manipulation::pick_poses(m, 0.30, 0.0, 0.058, std::nan(""), {40, 10, -23}, why, std::nan(""), 30.0, 0.0, 0.0).empty());
}

// 쥐기 −40 mm (control 기본, 11-01 실물 넣기 깊이): 박스 자리 · 옆 자리 · 먼 자리(기울인 손)에서 풀리고 −30 보다 손끝이 10 mm 낮음
TEST(ArmKinematics, PickDepth40Reachable)
{
  const auto m = kin_data::model();
  std::string why, why30;
  struct P {double x, y, yaw, tilt;};
  for (const P & c : {P{0.2102, 0.038, -4.0, 20.0}, P{0.22, 0.08, std::nan(""), 20.0}, P{0.22, -0.08, std::nan(""), 20.0},
                      P{0.18, 0.0, std::nan(""), 20.0}, P{0.30, 0.0, std::nan(""), 20.0}, P{0.33, 0.0, std::nan(""), 30.0}}) {
    const auto d40 = manipulation::pick_poses(m, c.x, c.y, 0.058, c.yaw, {40, 10, -40}, why, std::nan(""), c.tilt);
    ASSERT_EQ(d40.size(), 3u) << c.x << ", " << c.y << ": " << why;
    const auto d30 = manipulation::pick_poses(m, c.x, c.y, 0.058, c.yaw, {40, 10, -30}, why30, std::nan(""), c.tilt);
    ASSERT_EQ(d30.size(), 3u) << why30;
    EXPECT_DOUBLE_EQ(d40.back().tilt_deg, d30.back().tilt_deg);
    const auto g40 = manipulation::arm_grip(m, manipulation::ticks_to_q(m, d40[2].ticks));
    const auto g30 = manipulation::arm_grip(m, manipulation::ticks_to_q(m, d30[2].ticks));
    EXPECT_NEAR(g30.tip.z() - g40.tip.z(), 0.010, 0.0015) << c.x << ", " << c.y;
  }
}
