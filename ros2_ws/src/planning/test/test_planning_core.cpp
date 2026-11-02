// test_planning_core.cpp — planning 실행 층 순수 계산 시험, ROS 없이: 기록부 · grasp · 적재함 · 감시
//   실행: colcon build → source install/setup.bash → ./build/planning/test_planning_core

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <map>
#include <set>

#include "planning/cargo.hpp"
#include "planning/grasp.hpp"
#include "planning/health.hpp"
#include "planning/puck_registry.hpp"

using namespace planning;


// ============================================================================ types
TEST(Types, ValidQuaternion)
{
  EXPECT_TRUE(valid_quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_TRUE(valid_quaternion(0.0, 0.0, std::sin(0.4), std::cos(0.4)));
  EXPECT_FALSE(valid_quaternion(0.0, 0.0, 0.0, 0.0));            // 방향 모름
  EXPECT_FALSE(valid_quaternion(0.0, 0.0, 0.0, std::nan("")));   // 못 읽음
  EXPECT_FALSE(valid_quaternion(0.0, 0.0, 0.0, 2.0));            // 정규화 안 됨
}

// ============================================================================ puck_registry
PuckRecord rec(const std::string & id, double x, double y, double t, const std::string & color = "red")
{
  PuckRecord r;
  r.id = id;
  r.color = color;
  r.x = x;
  r.y = y;
  r.z = 0.03;
  r.last_seen = t;
  return r;
}

TEST(PuckRegistry, UpdatesByIdAndKeepsState)
{
  PuckRegistry reg;
  reg.update(rec("7", 1.0, 2.0, 0.0));
  reg.set_state("7", "in_gripper");
  reg.update(rec("7", 1.1, 2.0, 1.0));   // 다시 봄 → 위치 갱신, 상태 유지
  const auto r = reg.find("7");
  ASSERT_TRUE(r.has_value());
  EXPECT_DOUBLE_EQ(r->x, 1.1);
  EXPECT_EQ(r->state, "in_gripper");
}

TEST(PuckRegistry, CandidateMergesAndIsReplacedById)
{
  PuckRegistry reg(0.06);
  reg.add_candidate(rec("", 1.00, 1.00, 0.0));
  reg.add_candidate(rec("", 1.02, 1.01, 0.5));   // 같은 퍽 → 합침
  EXPECT_EQ(reg.candidates().size(), 1u);
  reg.update(rec("9", 1.01, 1.00, 1.0));          // ID 확인 → 후보 삭제
  EXPECT_TRUE(reg.candidates().empty());
  EXPECT_TRUE(reg.find("9").has_value());
}

TEST(PuckRegistry, CandidateNearKnownPuckRefreshesIt)
{
  PuckRegistry reg(0.06);
  reg.update(rec("3", 2.0, 2.0, 0.0));
  reg.add_candidate(rec("", 2.02, 2.0, 5.0));     // 멀리서 ID 없이 다시 봄
  EXPECT_TRUE(reg.candidates().empty());
  EXPECT_DOUBLE_EQ(reg.find("3")->last_seen, 5.0);
}

TEST(PuckRegistry, PrunesOldCandidates)
{
  PuckRegistry reg;
  reg.add_candidate(rec("", 0.0, 0.0, 0.0));
  reg.add_candidate(rec("", 3.0, 0.0, 9.0));
  reg.prune_candidates(10.0, 5.0);                 // 10 s 전 후보 삭제
  ASSERT_EQ(reg.candidates().size(), 1u);
  EXPECT_DOUBLE_EQ(reg.candidates().front().x, 3.0);
}

// ============================================================================ grasp
TEST(Grasp, ApproachPoseFacesPuck)
{
  const Pose2D p = approach_pose(0.0, 0.0, 1.0, 0.0, 0.3);
  EXPECT_NEAR(p.x, 0.7, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.yaw, 0.0, 1e-9);
  const Pose2D q = approach_pose(0.0, 0.0, 0.0, 2.0, 0.5);
  EXPECT_NEAR(q.y, 1.5, 1e-9);
  EXPECT_NEAR(q.yaw, M_PI / 2.0, 1e-9);
}

TEST(Grasp, GraspAxisPicksUpperOrNearEnd)
{
  const double h = std::sqrt(0.5);
  // 방향 모름 → 똑바로 섬, 위쪽 끝
  auto g = choose_grasp_axis(false, {0, 0, 0, 0}, {0.5, 0.0, 0.03});
  EXPECT_DOUBLE_EQ(g.axis.z, 1.0);
  EXPECT_FALSE(g.flipped);
  // 똑바로 섬
  g = choose_grasp_axis(true, {0, 0, 0, 1}, {0.5, 0.0, 0.03});
  EXPECT_NEAR(g.axis.z, 1.0, 1e-9);
  EXPECT_FALSE(g.flipped);
  // 뒤집혀 섬 (x 축 180°) → 위쪽 끝 = 자세 z 반대
  g = choose_grasp_axis(true, {1, 0, 0, 0}, {0.5, 0.0, 0.03});
  EXPECT_NEAR(g.axis.z, 1.0, 1e-9);
  EXPECT_TRUE(g.flipped);
  // 30° 기움 → 아직 서 있음으로 봄
  g = choose_grasp_axis(true, {std::sin(M_PI / 12), 0, 0, std::cos(M_PI / 12)}, {0.5, 0.0, 0.03});
  EXPECT_GT(g.axis.z, 0.8);
  // 누움 (y 축 90°): 긴축 = +x (로봇에서 먼 쪽) → 로봇 쪽 끝 -x 로 뒤집음
  g = choose_grasp_axis(true, {0, h, 0, h}, {0.5, 0.0, 0.015});
  EXPECT_NEAR(g.axis.x, -1.0, 1e-9);
  EXPECT_NEAR(g.axis.z, 0.0, 1e-9);
  EXPECT_TRUE(g.flipped);
  // 누움 (x 축 90°): 긴축 = -y. 퍽이 왼쪽(+y)에 있으면 -y 가 로봇 쪽 → 그대로
  g = choose_grasp_axis(true, {h, 0, 0, h}, {0.5, 0.2, 0.015});
  EXPECT_NEAR(g.axis.y, -1.0, 1e-9);
  EXPECT_FALSE(g.flipped);
  //   퍽이 오른쪽(-y)이면 +y 로 뒤집음
  g = choose_grasp_axis(true, {h, 0, 0, h}, {0.5, -0.2, 0.015});
  EXPECT_NEAR(g.axis.y, 1.0, 1e-9);
  EXPECT_TRUE(g.flipped);

  // 위 자세 {0, h, 0, h} 를 x 축으로 180° 돌린 자세 → z 축 = 반대 방향
  const Quat f{h, 0, -h, 0};
  const auto gf = choose_grasp_axis(true, f, {0.5, 0.0, 0.015});
  EXPECT_NEAR(gf.axis.x, -1.0, 1e-9);
  EXPECT_FALSE(gf.flipped);   // 이미 로봇 쪽 끝을 가리킴

  // 서 있는 퍽: 중심 높이 0.03 → 잡을 점 0.045 (위쪽 3 cm 구간의 가운데)
  const Vec3 up = grasp_point({0.25, 0.0, 0.03}, {0.0, 0.0, 1.0}, 0.015);
  EXPECT_NEAR(up.x, 0.25, 1e-9);
  EXPECT_NEAR(up.z, 0.045, 1e-9);
  // 앞뒤로 누운 퍽: 중심 (0.5, 0, 0.015), 로봇 쪽 끝 → 잡을 점 x = 0.485
  const Vec3 lying = grasp_point({0.5, 0.0, 0.015}, {-1.0, 0.0, 0.0}, 0.015);
  EXPECT_NEAR(lying.x, 0.485, 1e-9);
  EXPECT_NEAR(lying.z, 0.015, 1e-9);
}

// 쿼터니언 → 회전 행렬 열 (테스트 확인용)
static Vec3 col_x(const Quat & q)
{
  return {1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)};
}
static Vec3 col_z(const Quat & q)
{
  return {2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)};
}
// z 축 a 도 회전 → (x, y, z, w)
static Quat rot_z(double a) {return {0, 0, std::sin(a / 2), std::cos(a / 2)};}
// q1 · q2
static Quat mul(const Quat & a, const Quat & b)
{
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

TEST(Grasp, PlanarFromPoseUprightBoxFace)
{
  const double d2r = M_PI / 180.0;
  const Vec3 c{0.3, 0.0, 0.11};
  // 방향 모름 → 선 퍽, θ 모름
  auto p = planar_from_pose(false, {0, 0, 0, 0}, c, false, true);
  EXPECT_TRUE(p.upright);
  EXPECT_FALSE(p.theta_known);
  // 선 사각기둥, 면 방향 30° (perception 이 구함)
  p = planar_from_pose(true, rot_z(30 * d2r), c, true, true);
  EXPECT_TRUE(p.upright);
  ASSERT_TRUE(p.theta_known);
  EXPECT_NEAR(p.theta, 30 * d2r, 1e-9);
  // 면 방향 80° → 90° 주기라 -10°
  p = planar_from_pose(true, rot_z(80 * d2r), c, true, true);
  EXPECT_NEAR(p.theta, -10 * d2r, 1e-9);
  // 면 방향 못 구함 (yaw_known false) · 원기둥 → θ 모름
  EXPECT_FALSE(planar_from_pose(true, rot_z(30 * d2r), c, false, true).theta_known);
  EXPECT_FALSE(planar_from_pose(true, rot_z(30 * d2r), c, true, false).theta_known);
}

TEST(Grasp, PlanarFromPoseLying)
{
  const double d2r = M_PI / 180.0;
  const double h = std::sqrt(0.5);
  const Quat lie{0, h, 0, h};                       // y 축 90°: 긴축 = +x
  // 누운 퍽, 긴축 30° (z 축 30° 돌린 뒤 눕힘 = qz · qy)
  auto p = planar_from_pose(true, mul(rot_z(30 * d2r), lie), {0.3, 0.1, 0.095}, false, true);
  EXPECT_FALSE(p.upright);
  ASSERT_TRUE(p.theta_known);
  EXPECT_NEAR(p.theta, 30 * d2r, 1e-9);
  // 긴축 150° → 180° 주기라 -30° (원기둥도 같음)
  p = planar_from_pose(true, mul(rot_z(150 * d2r), lie), {0.3, 0.1, 0.095}, false, false);
  EXPECT_NEAR(p.theta, -30 * d2r, 1e-9);
}

TEST(Grasp, GraspOrientationRules)
{
  const double d2r = M_PI / 180.0;
  // 선 사각기둥 · 면 방향 30°: z = 위, x = 면 방향
  PuckPlanar up_box{{0.3, 0.0, 0.11}, 30 * d2r, true, true};
  Quat q = grasp_orientation(up_box, {{0, 0, 1}, false});
  Vec3 x = col_x(q), z = col_z(q);
  EXPECT_NEAR(z.z, 1.0, 1e-9);
  EXPECT_NEAR(x.x, std::cos(30 * d2r), 1e-9);
  EXPECT_NEAR(x.y, std::sin(30 * d2r), 1e-9);
  // 선 퍽 · 면 방향 모름: x = 로봇 → 퍽 수평 (0.5, 0.2)
  PuckPlanar up_unknown{{0.5, 0.2, 0.11}, 0.0, true, false};
  q = grasp_orientation(up_unknown, {{0, 0, 1}, false});
  x = col_x(q);
  const double n = std::hypot(0.5, 0.2);
  EXPECT_NEAR(x.x, 0.5 / n, 1e-9);
  EXPECT_NEAR(x.y, 0.2 / n, 1e-9);
  EXPECT_NEAR(x.z, 0.0, 1e-9);
  // 누운 퍽: z = 쥘 끝 (로봇 쪽 -x), x = 위쪽
  PuckPlanar lying{{0.5, 0.0, 0.095}, 0.0, false, true};
  q = grasp_orientation(lying, {{-1, 0, 0}, true});
  x = col_x(q);
  z = col_z(q);
  EXPECT_NEAR(z.x, -1.0, 1e-9);
  EXPECT_NEAR(x.z, 1.0, 1e-9);
  // 단위 쿼터니언
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-9);
  // 기울어진 쥘 끝 (서 있음 판정 안의 30° 기움): x 는 z 에 수직
  const Vec3 tilt{std::sin(30 * d2r), 0.0, std::cos(30 * d2r)};
  q = grasp_orientation(up_box, {tilt, false});
  x = col_x(q);
  z = col_z(q);
  EXPECT_NEAR(x.x * z.x + x.y * z.y + x.z * z.z, 0.0, 1e-9);
  EXPECT_NEAR(z.x, tilt.x, 1e-9);
}

// ============================================================================ cargo
CargoItem item(const std::string & id)
{
  CargoItem it;
  it.puck_id = id;
  it.color = "beige";
  return it;
}

TEST(Cargo, PickStoreRetrievePlace)
{
  Cargo c({"R1", "R2"});
  EXPECT_EQ(c.free_slot().value(), "R1");
  ASSERT_TRUE(c.on_pick(item("7")));
  EXPECT_TRUE(c.holding());
  ASSERT_TRUE(c.on_store("R1"));
  EXPECT_FALSE(c.holding());
  EXPECT_EQ(c.slots()[0].second->puck_id, "7");
  EXPECT_EQ(c.free_slot().value(), "R2");
  ASSERT_TRUE(c.on_pick(item("12")));
  ASSERT_TRUE(c.on_store("R2"));
  EXPECT_FALSE(c.free_slot().has_value());
  EXPECT_EQ(c.stored_count(), 2u);
  ASSERT_TRUE(c.on_retrieve("R1"));
  const auto placed = c.on_place();
  ASSERT_TRUE(placed.has_value());
  EXPECT_EQ(placed->puck_id, "7");
  EXPECT_EQ(c.stored_count(), 1u);
}

TEST(Cargo, RejectsInconsistentTransitions)
{
  Cargo c({"R1"});
  std::string err;
  EXPECT_FALSE(c.on_store("R1", &err));        // 빈 그리퍼
  EXPECT_FALSE(c.on_retrieve("R1", &err));     // 빈 칸
  EXPECT_FALSE(c.on_place(&err).has_value());  // 빈 그리퍼
  ASSERT_TRUE(c.on_pick(item("7")));
  EXPECT_FALSE(c.on_pick(item("8"), &err));    // 이미 쥐고 있음
  EXPECT_FALSE(c.on_store("L1", &err));        // 없는 칸
  ASSERT_TRUE(c.on_store("R1"));
  ASSERT_TRUE(c.on_pick(item("8")));
  EXPECT_FALSE(c.on_store("R1", &err));        // 찬 칸
  EXPECT_FALSE(err.empty());
}

TEST(Cargo, SaveLoadRoundTrip)
{
  namespace fs = std::filesystem;
  const std::string path = (fs::temp_directory_path() / "planning_test_cargo.yaml").string();
  Cargo a({"R1", "R2"});
  a.on_pick(item("7"));
  a.on_store("R2");
  a.on_pick(item("3"));
  std::string err;
  ASSERT_TRUE(a.save(path, &err)) << err;

  Cargo b({"R1", "R2"});
  ASSERT_TRUE(b.load(path, &err)) << err;
  ASSERT_TRUE(b.slots()[1].second.has_value());
  EXPECT_EQ(b.slots()[1].second->puck_id, "7");
  ASSERT_TRUE(b.holding());
  EXPECT_EQ(b.gripper()->puck_id, "3");

  Cargo wrong({"R1", "R3"});                   // 칸 이름이 다르면 복원 거부
  EXPECT_FALSE(wrong.load(path, &err));
  fs::remove(path);
}

// ============================================================================ health
TEST(Health, BlocksImmediatelyAndRecoversAfterStableGood)
{
  ConditionMonitor m(2.0, 2, 10.0);
  EXPECT_EQ(m.update(false, 0.0), Health::Ok);
  EXPECT_EQ(m.update(true, 1.0), Health::Waiting);    // 나빠지면 바로 차단
  EXPECT_TRUE(m.active());
  EXPECT_EQ(m.update(false, 2.0), Health::Waiting);   // 좋아져도 2 s 유지 전엔 차단
  EXPECT_EQ(m.update(true, 2.5), Health::Waiting);    // 다시 나빠지면 회복 시간 처음부터
  EXPECT_EQ(m.update(false, 3.0), Health::Waiting);
  EXPECT_EQ(m.update(false, 5.0), Health::Ok);        // 2 s 유지 → 해제
  EXPECT_FALSE(m.active());
}

TEST(Health, FaultsAfterTimeoutIncludingRecoveryWait)
{
  ConditionMonitor m(2.0, 2, 10.0);
  m.update(true, 0.0);
  EXPECT_EQ(m.update(true, 9.0), Health::Waiting);
  EXPECT_EQ(m.update(false, 9.5), Health::Waiting);   // 회복 확인 중
  EXPECT_EQ(m.update(false, 10.5), Health::Fault);    // 켜진 뒤 10 s 넘음
  EXPECT_EQ(m.update(false, 20.0), Health::Fault);    // reset 전까지 유지
  m.reset();
  EXPECT_EQ(m.update(false, 21.0), Health::Ok);
}

TEST(Health, FaultsWhenRecoveriesUsedUp)
{
  ConditionMonitor m(1.0, 2, 10.0);
  for (int i = 0; i < 2; ++i) {
    const double t = i * 10.0;
    EXPECT_EQ(m.update(true, t), Health::Waiting);
    EXPECT_EQ(m.update(false, t + 0.5), Health::Waiting);
    EXPECT_EQ(m.update(false, t + 1.6), Health::Ok);
  }
  EXPECT_EQ(m.update(true, 30.0), Health::Fault);     // 세 번째 → 자동 해제 안 함
}

TEST(Health, LocalizationHealthLevels)
{
  // pose_guard /localization_health: 0 정상 · 1 경고 · 2 정지 · 3 STALE, timeout 2 s
  LocHealth h;
  EXPECT_TRUE(loc_health_lost(h, 10.0, 2.0));    // 아직 못 받음
  EXPECT_FALSE(loc_health_ready(h, 10.0, 2.0));
  h = LocHealth{0, 9.0};
  EXPECT_FALSE(loc_health_lost(h, 10.0, 2.0));
  EXPECT_TRUE(loc_health_ready(h, 10.0, 2.0));
  h = LocHealth{1, 9.0};                          // 경고: 계속 가지만 출발은 안 함
  EXPECT_FALSE(loc_health_lost(h, 10.0, 2.0));
  EXPECT_FALSE(loc_health_ready(h, 10.0, 2.0));
  for (const int lv : {2, 3}) {
    h = LocHealth{lv, 9.0};
    EXPECT_TRUE(loc_health_lost(h, 10.0, 2.0));
    EXPECT_FALSE(loc_health_ready(h, 10.0, 2.0));
  }
  h = LocHealth{0, 7.5};                          // 2.5 s 끊김 = 상실
  EXPECT_TRUE(loc_health_lost(h, 10.0, 2.0));
  EXPECT_FALSE(loc_health_ready(h, 10.0, 2.0));
}

TEST(Health, CovarianceStd)
{
  double c[36] = {0.0};
  c[0] = 0.01;      // x 표준편차 0.1
  c[7] = 0.04;      // y 0.2
  c[35] = 0.0121;   // yaw 0.11
  const auto s = covariance_std(c);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->x, 0.1, 1e-9);
  EXPECT_NEAR(s->y, 0.2, 1e-9);
  EXPECT_NEAR(s->yaw, 0.11, 1e-9);
  c[7] = -1.0;
  EXPECT_FALSE(covariance_std(c).has_value());
  c[7] = std::nan("");
  EXPECT_FALSE(covariance_std(c).has_value());
}
