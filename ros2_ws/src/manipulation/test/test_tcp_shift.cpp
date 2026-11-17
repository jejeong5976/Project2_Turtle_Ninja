// test_tcp_shift.cpp — 그리퍼 손끝 (TCP) 옮김 (arm_kinematics with_tcp_shift, control_master gripper_tcp_shift_mm)
//   [0, 0, 0] = 계획 관절 목표가 옮김 없는 모델과 비트까지 같음 · 옮김은 손목 돌림 · 기울임을 따라감 (수직 · 기울임 30° · J5 90° 돌림 · 받침 · 세우기 표)

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/park_stand.hpp"
#include "manipulation/standup_table.hpp"
#include "arm_kin_data.hpp"

using namespace manipulation;

namespace
{
const double kTop = 0.037 + 0.060 + 0.033 - 0.072;   // 박스 위 선 퍽 윗면 (모델 z, control_master 기본값)
const Eigen::Vector3d kShift(-0.0055, 0.008, 0.0);    // 제안 값 [m, 손 좌표] = [-5.5, 8, 0] mm

bool same_bits(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {return std::memcmp(a.data(), b.data(), sizeof(double) * 3) == 0;}

void expect_same_poses(const std::vector<PickPose> & a, const std::vector<PickPose> & b)
{
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].ticks, b[i].ticks) << i;
    EXPECT_EQ(a[i].enc, b[i].enc) << i;
    EXPECT_EQ(a[i].tilt_deg, b[i].tilt_deg) << i;
  }
}

// 옮긴 모델로 푼 쥐기 자세: 옮긴 손끝 = 목표, 옮기기 전 손끝 = 목표 − (손 좌표 옮김을 그 자세 손목으로 돌린 것)
void check_shifted(const ArmModel & raw, const ArmModel & sh, const PickPose & p, const Eigen::Vector3d & target, Eigen::Vector3d & d_base)
{
  const Q5 q = ticks_to_q(raw, p.ticks);
  d_base = tcp_shift_base(raw, q, kShift);
  EXPECT_NEAR(d_base.norm(), kShift.norm(), 1e-9);   // 돌리기만 함
  EXPECT_LT((arm_grip(sh, q).tip - target).norm(), 1.2e-3);              // IK 허용 1 mm + 틱 반올림
  EXPECT_LT((arm_grip(raw, q).tip - (target - d_base)).norm(), 1.2e-3);   // 실물 손가락 = 모델 손끝 + d → 명령 손끝 = 목표 − d
  EXPECT_LT((arm_grip(sh, q).tip - arm_grip(raw, q).tip - d_base).norm(), 1e-12);
}
}  // namespace

// 손 좌표: 수직 손 · J5 기본 (정면 퍽) 에서 x = base 앞 · y = 왼쪽 · z = 위
TEST(TcpShift, HandFrameAtVerticalPick)
{
  const auto m = kin_data::model();
  std::string why;
  const auto P = pick_poses(m, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, why);
  ASSERT_EQ(P.size(), 3u) << why;
  const Q5 q = ticks_to_q(m, P[2].ticks);
  const Eigen::Vector3d d = tcp_shift_base(m, q, Eigen::Vector3d(0.001, 0.002, 0.003));
  EXPECT_NEAR(d.x(), 0.001, 3e-5);
  EXPECT_NEAR(d.y(), 0.002, 3e-5);
  EXPECT_NEAR(d.z(), 0.003, 3e-5);
}

// [0, 0, 0]: tip5 비트까지 같고 계획 관절 목표 (실제 틱 · 처짐 앞먹임 명령 틱) 가 옮김 없는 모델과 같음
TEST(TcpShift, ZeroIsByteIdentical)
{
  const auto raw = kin_data::model();
  const auto z = with_tcp_shift(raw, Eigen::Vector3d::Zero());
  EXPECT_TRUE(same_bits(raw.tip5, z.tip5));
  EXPECT_TRUE(tcp_shift_base(raw, raw.template_q, Eigen::Vector3d::Zero()).isZero(0.0));
  std::string w1, w2;
  // 수직
  expect_same_poses(pick_poses(raw, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, w1),
    pick_poses(z, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, w2));
  // 기울임 30°
  expect_same_poses(pick_poses(raw, 0.25, 0.0, kTop, std::nan(""), {40.0, 10.0, -30.0}, w1, std::nan(""), 30.0, 30.0, 0.5),
    pick_poses(z, 0.25, 0.0, kTop, std::nan(""), {40.0, 10.0, -30.0}, w2, std::nan(""), 30.0, 30.0, 0.5));
  // J5 90° 돌림 (닫힘축 0° = 반지름 방향)
  expect_same_poses(pick_poses(raw, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, w1, 0.0),
    pick_poses(z, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, w2, 0.0));
  // 표 기록 자세 다시 맞추기 (세우기 · 먼 퍽 · 누운 퍽 표가 쓰는 계산)
  StandupRetarget rt;
  rt.from = Eigen::Vector2d(0.20, 0.0);
  rt.to = Eigen::Vector2d(0.205, 0.004);
  RetargetResult a, b;
  const Q5 q0 = ticks_to_q(raw, Ticks{{4088, 2371, 2053, 1742, 2019}});
  ASSERT_TRUE(standup_retarget(raw, q0, rt, false, a));
  ASSERT_TRUE(standup_retarget(z, q0, rt, false, b));
  EXPECT_EQ(std::memcmp(a.q.data(), b.q.data(), sizeof(double) * 5), 0);
  // 누운 퍽 받침 쥐기
  LyingGraspCfg gc;
  LyingGrasp ga, gb;
  const double top_lying = 0.070 + 0.0348 + 0.0024 - 0.072;
  ASSERT_TRUE(lying_grasp(raw, 0.205, 0.0, top_lying, 0.0, {40.0, 10.0, -45.0}, gc, ga, w1, 0.0)) << w1;
  ASSERT_TRUE(lying_grasp(z, 0.205, 0.0, top_lying, 0.0, {40.0, 10.0, -45.0}, gc, gb, w2, 0.0)) << w2;
  expect_same_poses(ga.poses, gb.poses);
}

// 수직 손 · J5 기본: 옮김 [-5.5, 8, 0] mm → 명령 손끝이 base (+5.5, −8) mm (pick_offset 을 그만큼 바꾼 것과 같음)
TEST(TcpShift, VerticalPick)
{
  const auto raw = kin_data::model();
  const auto sh = with_tcp_shift(raw, kShift);
  std::string why;
  const auto P = pick_poses(sh, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, why);
  ASSERT_EQ(P.size(), 3u) << why;
  EXPECT_EQ(P.back().tilt_deg, 0.0);
  Eigen::Vector3d d;
  check_shifted(raw, sh, P.back(), Eigen::Vector3d(0.20, 0.0, kTop - 0.040), d);
  EXPECT_NEAR(d.x(), -0.0055, 1e-4);
  EXPECT_NEAR(d.y(), 0.008, 1e-4);
  EXPECT_NEAR(d.z(), 0.0, 1e-4);
  const Eigen::Vector3d cmd = arm_grip(raw, ticks_to_q(raw, P.back().ticks)).tip;   // 옮김 없는 모델로 본 명령 손끝
  EXPECT_NEAR((cmd.x() - 0.20) * 1000.0, 5.5, 1.2);
  EXPECT_NEAR(cmd.y() * 1000.0, -8.0, 1.2);
}

// 기울임 30° (먼 퍽): 옮김이 기울인 손을 따라 돎 → base 로 z 성분이 생기고 수직과 다름
TEST(TcpShift, TiltedPick30)
{
  const auto raw = kin_data::model();
  const auto sh = with_tcp_shift(raw, kShift);
  std::string why;
  const auto P = pick_poses(sh, 0.25, 0.0, kTop, std::nan(""), {40.0, 10.0, -30.0}, why, std::nan(""), 30.0, 30.0, 0.5, {}, false);
  ASSERT_EQ(P.size(), 3u) << why;
  EXPECT_EQ(P.back().tilt_deg, 30.0);
  Eigen::Vector3d d;
  check_shifted(raw, sh, P.back(), Eigen::Vector3d(0.25, 0.0, kTop - 0.030), d);
  // 손 x (반지름 바깥) 가 30° 기울어 위로: −5.5 mm · (cos 30°, 0, sin 30°) 쪽
  EXPECT_NEAR(d.x(), -0.0055 * std::cos(M_PI / 6.0), 3e-4);
  EXPECT_NEAR(d.y(), 0.008, 3e-4);
  EXPECT_NEAR(d.z(), -0.0055 * std::sin(M_PI / 6.0), 3e-4);
}

// J5 90° 돌림 (닫힘축 = 반지름): 옮김이 손목과 같이 90° 돎 → base (+8, +5.5) mm
TEST(TcpShift, WristRotated90)
{
  const auto raw = kin_data::model();
  const auto sh = with_tcp_shift(raw, kShift);
  std::string why;
  const auto P = pick_poses(sh, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, why, 0.0);
  ASSERT_EQ(P.size(), 3u) << why;
  const auto P0 = pick_poses(sh, 0.20, 0.0, kTop, std::nan(""), {40.0, 10.0, -40.0}, why);
  ASSERT_EQ(P0.size(), 3u) << why;
  EXPECT_GT(std::abs(P.back().ticks[4] - P0.back().ticks[4]), 900);   // J5 약 90° (1024 틱) 다름
  Eigen::Vector3d d;
  check_shifted(raw, sh, P.back(), Eigen::Vector3d(0.20, 0.0, kTop - 0.040), d);
  EXPECT_NEAR(std::abs(d.x()), 0.008, 2e-4);
  EXPECT_NEAR(std::abs(d.y()), 0.0055, 2e-4);
  EXPECT_NEAR(d.z(), 0.0, 1e-4);
}

// 받침 (누운 퍽) 쥐기 · 받침 넣기: 옮긴 모델로 풀면 실제 손가락 (옮긴 손끝) 이 옮김 없는 모델의 손끝 자리에 옴
TEST(TcpShift, ParkGraspFollowsShift)
{
  const auto raw = kin_data::model();
  const auto sh = with_tcp_shift(raw, kShift);
  LyingGraspCfg gc;
  ParkCfg pk;
  const double top_lying = 0.070 + 0.0348 + 0.0024 - 0.072;
  LyingGrasp ga, gb;
  std::string w;
  ASSERT_TRUE(lying_grasp(raw, 0.205, 0.0, top_lying, 0.0, {40.0, 10.0, -45.0}, gc, ga, w, 0.0)) << w;
  ASSERT_TRUE(lying_grasp(sh, 0.205, 0.0, top_lying, 0.0, {40.0, 10.0, -45.0}, gc, gb, w, 0.0)) << w;
  const Q5 qb = ticks_to_q(raw, gb.poses.back().ticks);
  EXPECT_LT((arm_grip(sh, qb).tip - ga.tip).norm(), 1.2e-3);   // 같은 손끝 목표
  EXPECT_GT((arm_grip(raw, qb).tip - ga.tip).norm(), 8e-3);    // 관절은 옮김만큼 다름
  // 받침 자세: 닫힘축 (y) 옮김 8 mm 는 받침 위 J5 한계에 걸려 안 풀림 → control_master 는 받침 자세만 옮기기 전 모델로 (park_raw).
  //   앞뒤 (x) 옮김만이면 받침 자세도 옮긴 손끝으로 풀리고 손끝 목표가 같음
  std::vector<ParkNode> na, nb;
  double ea = 0.0, eb = 0.0;
  ASSERT_TRUE(park_nodes(raw, pk, ga, ticks_to_q(raw, ga.poses[0].ticks), na, ea, w)) << w;
  EXPECT_FALSE(park_nodes(sh, pk, gb, qb, nb, eb, w));
  const auto shx = with_tcp_shift(raw, Eigen::Vector3d(-0.0055, 0.0, 0.0));
  LyingGrasp gx;
  ASSERT_TRUE(lying_grasp(shx, 0.205, 0.0, top_lying, 0.0, {40.0, 10.0, -45.0}, gc, gx, w, 0.0)) << w;
  ASSERT_TRUE(park_nodes(shx, pk, gx, ticks_to_q(raw, gx.poses[0].ticks), nb, eb, w)) << w;
  ASSERT_EQ(na.size(), nb.size());
  for (size_t i = 0; i < na.size(); ++i) {
    EXPECT_LT((arm_grip(shx, nb[i].q).tip - arm_grip(raw, na[i].q).tip).norm(), 1.5e-3) << na[i].name;
  }
}
