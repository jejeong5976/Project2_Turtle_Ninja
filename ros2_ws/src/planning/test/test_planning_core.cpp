// test_planning_core.cpp — planning 실행 층 순수 계산 시험, ROS 없이: 기록부 · grasp · 적재함 · 재고 불확실 · 감시
//   실행: colcon build → source install/setup.bash → ./build/planning/test_planning_core

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <map>
#include <set>

#include "planning/approach.hpp"
#include "planning/cargo.hpp"
#include "planning/grasp.hpp"
#include "planning/health.hpp"
#include "planning/inventory_policy.hpp"
#include "planning/puck_registry.hpp"
#include "planning/types.hpp"

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

// ============================================================================ inventory_policy (재고 불확실)
// STORE 7 → R1 이 수락 뒤 실패한 상태 (기록 = 그리퍼 7, R1 빔)
Cargo store_doubt()
{
  Cargo c({"R1", "R2"});
  c.on_pick(item("7"));
  c.set_doubt(make_doubt(c, kArmStore, "7", "R1", CargoItem{}, "arm:STORE:7>R1", "팔 작업 실패"));
  return c;
}

TEST(InventoryPolicy, DoubtOnlyWhenRiskyTaskWasSent)
{
  for (const uint8_t t : {kArmPick, kArmStore, kArmRetrieve, kArmPlace, kArmLookRack}) {
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::NotSent)) << int(t);     // 보내기 전 실패 → 재시도 · 재계획 그대로
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::Rejected)) << int(t);    // 목표 거절 → 팔 안 움직임
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::Succeeded)) << int(t);
    EXPECT_TRUE(inventory_doubt_after(t, ArmEnd::Failed)) << int(t);       // 수락 뒤 실패 · 취소
    EXPECT_TRUE(inventory_doubt_after(t, ArmEnd::Unknown)) << int(t);      // 시간 초과 · FAULT 취소 결과 모름
    EXPECT_TRUE(inventory_doubt_after(t, ArmEnd::Mismatch)) << int(t);     // 최종 성공인데 기록 모순
  }
  for (const uint8_t t : {kArmHome, kArmLook, kArmLookBarcode}) {   // 재고 안 바꿈
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::Failed)) << int(t);
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::Unknown)) << int(t);
    EXPECT_FALSE(inventory_doubt_after(t, ArmEnd::Mismatch)) << int(t);
  }
}

TEST(InventoryPolicy, DoubtKeepsLastConfirmedRecordAndFirstCause)
{
  Cargo c = store_doubt();
  ASSERT_TRUE(c.doubt().has_value());
  EXPECT_EQ(c.doubt()->item.puck_id, "7");
  EXPECT_EQ(c.gripper()->puck_id, "7");          // lost 로 지우지 않음
  EXPECT_FALSE(c.slots()[0].second.has_value());
  EXPECT_FALSE(c.set_doubt(make_doubt(c, kArmPlace, "7", "", CargoItem{}, "arm:PLACE", "x")));   // 첫 원인 유지
  EXPECT_EQ(c.doubt()->arm_task, kArmStore);
  c.clear_gripper();                             // 그리퍼 기록을 지워도 불확실 표시는 남음
  EXPECT_TRUE(c.doubt().has_value());
}

TEST(InventoryPolicy, BeforeKeepsRecord)
{
  Cargo c = store_doubt();
  std::string id, state, err;
  ASSERT_TRUE(resolve_doubt(c, DoubtFix::Before, &id, &state, &err)) << err;
  EXPECT_FALSE(c.doubt().has_value());
  EXPECT_EQ(c.gripper()->puck_id, "7");
  EXPECT_FALSE(c.slots()[0].second.has_value());
  EXPECT_EQ(id, "7");
  EXPECT_EQ(state, "");                          // 기록부 상태 안 바꿈
}

TEST(InventoryPolicy, DoneAppliesSuccessOnce)
{
  Cargo c = store_doubt();
  std::string id, state, err;
  ASSERT_TRUE(resolve_doubt(c, DoubtFix::Done, &id, &state, &err)) << err;
  EXPECT_FALSE(c.holding());
  ASSERT_TRUE(c.slots()[0].second.has_value());
  EXPECT_EQ(c.slots()[0].second->puck_id, "7");
  EXPECT_EQ(state, "in_cargo:R1");
  const Cargo after = c;
  EXPECT_FALSE(resolve_doubt(c, DoubtFix::Done, &id, &state, &err));   // 표시 없음 → 두 번 반영 안 함
  EXPECT_EQ(c.stored_count(), after.stored_count());

  // PICK · RETRIEVE · PLACE 도 성공 전환 그대로
  Cargo p({"R1"});
  CargoItem pk = item("4");
  pk.color = "red";
  p.set_doubt(make_doubt(p, kArmPick, "4", "", pk, "arm:PICK:4", "팔 작업 실패"));
  ASSERT_TRUE(resolve_doubt(p, DoubtFix::Done, &id, &state, &err)) << err;
  EXPECT_EQ(p.gripper()->color, "red");
  EXPECT_EQ(state, "in_gripper");
  p.on_store("R1");
  p.set_doubt(make_doubt(p, kArmRetrieve, "4", "R1", CargoItem{}, "arm:RETRIEVE:R1>4", "시간 초과"));
  EXPECT_EQ(p.doubt()->item.color, "red");      // RETRIEVE = 그 칸 내용
  ASSERT_TRUE(resolve_doubt(p, DoubtFix::Done, &id, &state, &err)) << err;
  EXPECT_EQ(p.gripper()->puck_id, "4");
  EXPECT_EQ(state, "in_gripper");
  p.set_doubt(make_doubt(p, kArmPlace, "4", "", CargoItem{}, "arm:PLACE:4>L1#0", "FAULT 로 취소"));
  ASSERT_TRUE(resolve_doubt(p, DoubtFix::Done, &id, &state, &err)) << err;
  EXPECT_FALSE(p.holding());
  EXPECT_EQ(state, "delivered");                 // 사람 확인이 있을 때만 delivered
}

TEST(InventoryPolicy, DoneRefusedWhenRecordDisagreesOrNoTransition)
{
  Cargo c({"R1"});
  c.on_pick(item("3"));
  c.on_store("R1");                              // R1 이미 참
  c.on_pick(item("7"));
  c.set_doubt(make_doubt(c, kArmStore, "7", "R1", CargoItem{}, "arm:STORE:7>R1", "팔 작업 실패"));
  std::string id, state, err;
  EXPECT_FALSE(resolve_doubt(c, DoubtFix::Done, &id, &state, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(c.doubt().has_value());            // 실패하면 표시 · 기록 그대로
  EXPECT_EQ(c.gripper()->puck_id, "7");
  EXPECT_EQ(c.slots()[0].second->puck_id, "3");

  Cargo r({"R1"});
  r.on_pick(item("5"));
  r.set_doubt(make_doubt(r, kArmLookRack, "5", "R1", CargoItem{}, "arm:LOOK_RACK:R1", "팔 작업 실패"));
  EXPECT_FALSE(resolve_doubt(r, DoubtFix::Done, &id, &state, &err));   // 전환 없는 작업
  EXPECT_TRUE(r.doubt().has_value());
}

TEST(InventoryPolicy, RemovedClearsOnlyThatPuck)
{
  Cargo c({"R1", "R2"});
  c.on_pick(item("3"));
  c.on_store("R1");
  c.on_pick(item("7"));
  c.on_store("R2");
  c.set_doubt(make_doubt(c, kArmRetrieve, "7", "R2", CargoItem{}, "arm:RETRIEVE:R2>7", "팔 작업 실패"));
  std::string id, state, err;
  ASSERT_TRUE(resolve_doubt(c, DoubtFix::Removed, &id, &state, &err)) << err;
  EXPECT_FALSE(c.slots()[1].second.has_value());
  EXPECT_EQ(c.slots()[0].second->puck_id, "3");  // 다른 칸은 그대로
  EXPECT_FALSE(c.holding());
  EXPECT_EQ(id, "7");
  EXPECT_EQ(state, "removed");                   // lost 아님 = 사람이 뺀 것

  Cargo s = store_doubt();
  ASSERT_TRUE(resolve_doubt(s, DoubtFix::Removed, &id, &state, &err)) << err;
  EXPECT_FALSE(s.holding());
}

TEST(InventoryPolicy, LateSuccessMatchesCause)
{
  const Cargo c = store_doubt();
  EXPECT_TRUE(doubt_matches(*c.doubt(), kArmStore, "7", "R1"));
  EXPECT_FALSE(doubt_matches(*c.doubt(), kArmStore, "7", "R2"));
  EXPECT_FALSE(doubt_matches(*c.doubt(), kArmPlace, "7", ""));
}

// 기록 = 그리퍼 7, R1 빔, R2 = 3
Cargo holding7_r2_3()
{
  Cargo c({"R1", "R2"});
  c.on_pick(item("3"));
  c.on_store("R2");
  c.on_pick(item("7"));
  return c;
}

TEST(InventoryPolicy, PreflightStoreLookRackPlaceNeedThatPuckInGripper)
{
  const Cargo c = holding7_r2_3();
  std::string err;
  EXPECT_TRUE(arm_preflight_ok(c, kArmStore, "7", "R1", &err)) << err;
  EXPECT_TRUE(arm_preflight_ok(c, kArmLookRack, "7", "R1", &err)) << err;
  EXPECT_TRUE(arm_preflight_ok(c, kArmPlace, "7", "", &err)) << err;
  for (const uint8_t t : {kArmStore, kArmLookRack, kArmPlace}) {   // 그리퍼 기록과 다른 퍽
    err.clear();
    EXPECT_FALSE(arm_preflight_ok(c, t, "5", "R1", &err)) << int(t);
    EXPECT_FALSE(err.empty()) << int(t);
    EXPECT_FALSE(arm_preflight_ok(c, t, "", "R1", &err)) << int(t);   // 단계 퍽 ID 없음
  }
  EXPECT_FALSE(arm_preflight_ok(c, kArmStore, "7", "R2", &err));      // 칸 기록 이미 참
  EXPECT_FALSE(arm_preflight_ok(c, kArmStore, "7", "R9", &err));      // 없는 칸
  const Cargo empty({"R1"});
  EXPECT_FALSE(arm_preflight_ok(empty, kArmStore, "7", "R1", &err));  // 빈 그리퍼
  EXPECT_FALSE(arm_preflight_ok(empty, kArmPlace, "7", "", &err));
}

TEST(InventoryPolicy, PreflightRetrieveNeedsThatPuckInSlotAndEmptyGripper)
{
  Cargo c({"R1", "R2"});
  c.on_pick(item("3"));
  c.on_store("R2");
  std::string err;
  EXPECT_TRUE(arm_preflight_ok(c, kArmRetrieve, "3", "R2", &err)) << err;
  EXPECT_FALSE(arm_preflight_ok(c, kArmRetrieve, "4", "R2", &err));   // 칸에 다른 퍽
  EXPECT_FALSE(arm_preflight_ok(c, kArmRetrieve, "3", "R1", &err));   // 빈 칸
  EXPECT_FALSE(arm_preflight_ok(c, kArmRetrieve, "3", "R9", &err));   // 없는 칸
  c.on_pick(item("8"));
  EXPECT_FALSE(arm_preflight_ok(c, kArmRetrieve, "3", "R2", &err));   // 그리퍼 기록 참
}

TEST(InventoryPolicy, PreflightPickNeedsEmptyGripperAndNoDuplicate)
{
  const Cargo c = holding7_r2_3();
  std::string err;
  EXPECT_FALSE(arm_preflight_ok(c, kArmPick, "9", "", &err));          // 그리퍼 기록 참
  Cargo d({"R1", "R2"});
  d.on_pick(item("3"));
  d.on_store("R2");
  EXPECT_TRUE(arm_preflight_ok(d, kArmPick, "9", "", &err)) << err;
  EXPECT_FALSE(arm_preflight_ok(d, kArmPick, "3", "", &err));          // 이미 칸에 실린 ID
  EXPECT_FALSE(arm_preflight_ok(d, kArmPick, "", "", &err));
  for (const uint8_t t : {kArmHome, kArmLook, kArmLookBarcode}) {   // 재고 무관 → 확인 없음
    EXPECT_TRUE(arm_preflight_ok(c, t, "", "", &err)) << int(t);
  }
}

TEST(InventoryPolicy, SuccessAppliesTransitionAndState)
{
  Cargo c = holding7_r2_3();
  ArmSuccessApplied r;
  std::string err;
  ASSERT_TRUE(apply_arm_success(c, kArmStore, "7", "R1", CargoItem{}, &r, &err)) << err;
  EXPECT_TRUE(r.changed);
  EXPECT_FALSE(r.doubt_cleared);
  EXPECT_EQ(r.puck_state, "in_cargo:R1");
  EXPECT_FALSE(c.holding());
  EXPECT_EQ(c.slots()[0].second->puck_id, "7");

  ASSERT_TRUE(apply_arm_success(c, kArmRetrieve, "3", "R2", CargoItem{}, &r, &err)) << err;
  EXPECT_EQ(r.puck_state, "in_gripper");
  ASSERT_TRUE(apply_arm_success(c, kArmPlace, "3", "", CargoItem{}, &r, &err)) << err;
  EXPECT_EQ(r.puck_state, "delivered");
  ASSERT_TRUE(r.placed.has_value());
  EXPECT_EQ(r.placed->puck_id, "3");
  ASSERT_TRUE(apply_arm_success(c, kArmPick, "9", "", item("9"), &r, &err)) << err;
  EXPECT_EQ(c.gripper()->puck_id, "9");

  const Cargo before = c;
  ASSERT_TRUE(apply_arm_success(c, kArmHome, "", "", CargoItem{}, &r, &err)) << err;   // 전환 없음
  EXPECT_FALSE(r.changed);
  EXPECT_EQ(r.puck_state, "");
  EXPECT_EQ(c.gripper()->puck_id, before.gripper()->puck_id);
}

// 예전: 전환이 거부돼도 기록부를 in_gripper · in_cargo · delivered 로 바꾸고 경고만 → 이제 아무것도 안 바꾸고 false
TEST(InventoryPolicy, SuccessMismatchChangesNothing)
{
  Cargo c = holding7_r2_3();
  ArmSuccessApplied r;
  r.puck_state = "untouched";
  std::string err;
  EXPECT_FALSE(apply_arm_success(c, kArmStore, "7", "R2", CargoItem{}, &r, &err));   // R2 기록 참
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(r.puck_state, "untouched");          // 기록부에 넣을 상태 안 나옴
  EXPECT_EQ(c.gripper()->puck_id, "7");
  EXPECT_EQ(c.slots()[1].second->puck_id, "3");
  EXPECT_FALSE(c.slots()[0].second.has_value());
  EXPECT_FALSE(apply_arm_success(c, kArmPlace, "5", "", CargoItem{}, &r, &err));    // 다른 퍽을 놓았다고 함
  EXPECT_TRUE(c.holding());
  EXPECT_FALSE(apply_arm_success(c, kArmPick, "9", "", item("9"), &r, &err));       // 그리퍼 기록 참
  EXPECT_EQ(c.gripper()->puck_id, "7");
  Cargo e({"R1"});
  EXPECT_FALSE(apply_arm_success(e, kArmPlace, "7", "", CargoItem{}, &r, &err));    // 빈 그리퍼 PLACE
  EXPECT_FALSE(apply_arm_success(e, kArmRetrieve, "7", "R1", CargoItem{}, &r, &err));
  EXPECT_FALSE(e.holding());
}

// 늦은 최종 성공: 전환과 표시 해소가 함께 (전환 실패면 표시 유지 = 지운 뒤 실패하는 창 없음)
TEST(InventoryPolicy, LateSuccessClearsMatchingDoubtOnlyWithTransition)
{
  Cargo c = store_doubt();
  ArmSuccessApplied r;
  std::string err;
  ASSERT_TRUE(apply_arm_success(c, kArmStore, "7", "R1", CargoItem{}, &r, &err)) << err;
  EXPECT_TRUE(r.doubt_cleared);
  EXPECT_FALSE(c.doubt().has_value());
  EXPECT_EQ(c.slots()[0].second->puck_id, "7");
  EXPECT_FALSE(apply_arm_success(c, kArmStore, "7", "R1", CargoItem{}, &r, &err));   // 같은 결과 두 번 반영 안 됨

  Cargo m({"R1"});                               // 원인 단계와 같지만 기록이 안 맞음 (R1 이미 참)
  m.on_pick(item("3"));
  m.on_store("R1");
  m.on_pick(item("7"));
  m.set_doubt(make_doubt(m, kArmStore, "7", "R1", CargoItem{}, "arm:STORE:7>R1", "FAULT 로 취소"));
  EXPECT_FALSE(apply_arm_success(m, kArmStore, "7", "R1", CargoItem{}, &r, &err));
  ASSERT_TRUE(m.doubt().has_value());            // 표시 그대로
  EXPECT_EQ(m.gripper()->puck_id, "7");

  Cargo o = store_doubt();                       // 다른 단계 표시는 안 지움
  o.on_store("R2");
  ASSERT_TRUE(apply_arm_success(o, kArmRetrieve, "7", "R2", CargoItem{}, &r, &err)) << err;
  EXPECT_FALSE(r.doubt_cleared);
  EXPECT_TRUE(o.doubt().has_value());

  Cargo k({"R1"});                               // LOOK_RACK: 전환 없이 표시만 해소
  k.on_pick(item("5"));
  k.set_doubt(make_doubt(k, kArmLookRack, "5", "R1", CargoItem{}, "arm:LOOK_RACK:R1", "FAULT 로 취소"));
  ASSERT_TRUE(apply_arm_success(k, kArmLookRack, "5", "R1", CargoItem{}, &r, &err)) << err;
  EXPECT_TRUE(r.doubt_cleared);
  EXPECT_TRUE(r.changed);
  EXPECT_EQ(r.puck_state, "");
  EXPECT_EQ(k.gripper()->puck_id, "5");
}

TEST(InventoryPolicy, DoubtSurvivesSaveLoad)
{
  namespace fs = std::filesystem;
  const std::string path = (fs::temp_directory_path() / "planning_test_cargo_doubt.yaml").string();
  const Cargo a = store_doubt();
  std::string err;
  ASSERT_TRUE(a.save(path, &err)) << err;
  Cargo b({"R1", "R2"});
  ASSERT_TRUE(b.load(path, &err)) << err;
  ASSERT_TRUE(b.doubt().has_value());            // 재시작 (cargo_restore) 해도 불확실 유지
  EXPECT_EQ(b.doubt()->arm_task, kArmStore);
  EXPECT_EQ(b.doubt()->slot, "R1");
  EXPECT_EQ(b.doubt()->step_name, "arm:STORE:7>R1");
  EXPECT_EQ(b.doubt()->item.puck_id, "7");
  ASSERT_TRUE(b.holding());

  Cargo plain({"R1", "R2"});                     // 표시 없는 파일 → 표시 없음 (예전 파일 호환)
  ASSERT_TRUE(plain.save(path, &err)) << err;
  ASSERT_TRUE(b.load(path, &err)) << err;
  EXPECT_FALSE(b.doubt().has_value());
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

TEST(Health, LocalizationLostBlocksOnlyMapDependentArmStart)
{
  // 위치 상실: PICK · map 의존 목표 (Puck · Custom) 의 PLACE 차단 = LOOK 직후 위치 점프 중 미션 PLACE 시작 방지
  EXPECT_TRUE(arm_start_blocked(kArmPick, Target::Puck, false, true));
  EXPECT_TRUE(arm_start_blocked(kArmPlace, Target::Custom, false, true));
  EXPECT_TRUE(arm_start_blocked(kArmPlace, Target::Puck, false, true));
  // 로봇 자체 기준 동작은 목표 종류와 상관없이 허용 (arm_target 이 목표를 안 씀)
  for (const uint8_t t : {kArmHome, kArmLook, kArmLookBarcode, kArmLookRack, kArmStore, kArmRetrieve}) {
    for (const Target g : {Target::None, Target::Puck, Target::Custom}) {
      EXPECT_FALSE(arm_start_blocked(t, g, false, true)) << int(t) << " " << int(g);
      EXPECT_FALSE(arm_goal_uses_map(t, g)) << int(t) << " " << int(g);
    }
  }
  // PLACE 라도 목표가 없거나 고정이면 위치 의존 아님
  EXPECT_FALSE(arm_start_blocked(kArmPlace, Target::None, false, true));
  EXPECT_FALSE(arm_start_blocked(kArmPlace, Target::Fixed, false, true));
  // 감시가 꺼져 있으면 (require_localization false 포함) 막지 않음
  EXPECT_FALSE(arm_start_blocked(kArmPick, Target::Puck, false, false));
  EXPECT_FALSE(arm_start_blocked(kArmPlace, Target::Custom, false, false));
}

TEST(Health, DetectionLostBlocksOnlyPickStart)
{
  EXPECT_TRUE(arm_start_blocked(kArmPick, Target::Puck, true, false));
  EXPECT_FALSE(arm_start_blocked(kArmPlace, Target::Custom, true, false));   // 검출 끊김은 PLACE 와 무관
  EXPECT_FALSE(arm_start_blocked(kArmStore, Target::Puck, true, false));
  EXPECT_FALSE(arm_start_blocked(kArmHome, Target::None, true, false));
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

// 첫 면은 연속 두 스캔이 맞아야 받음 → 같은 관측 두 번
static void see_face(planning::ApproachState & st, const planning::ApproachConfig & c, double face, double front, double travel)
{
  planning::approach_observe(st, c, face, front, travel);
  planning::approach_observe(st, c, face, front, travel);
}

// 박스 앞 마지막 접근: Nav2 목표는 보는 방향 반대로 물러남, 그 뒤 라이다 면 거리로 멈춤 (가까워져 안 보이면 마지막 관측 + odom)
TEST(Approach, BackedOffAndStopsAtFace)
{
  const planning::Pose2D stop{1.0, 2.0, M_PI / 2};
  const auto g = planning::backed_off(stop, 0.10);
  EXPECT_NEAR(g.x, 1.0, 1e-9);
  EXPECT_NEAR(g.y, 1.9, 1e-9);
  EXPECT_NEAR(g.yaw, M_PI / 2, 1e-9);

  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;
  see_face(st, c, 0.291, 0.291, 0.0);   // 10 cm 남음 → 최고 속도
  auto a = planning::approach_step(st, c, 0.0, 0.0);
  EXPECT_FALSE(a.done);
  EXPECT_NEAR(a.vx, c.speed, 1e-9);
  planning::approach_observe(st, c, std::nan(""), std::nan(""), 0.090);   // 가까워져 안 보임 → 관측 기준 그대로
  a = planning::approach_step(st, c, 0.095, 0.095);   // 0.291 − 0.095 − 0.191 = 5 mm
  EXPECT_NEAR(a.remaining, 0.005, 1e-9);
  a = planning::approach_step(st, c, 0.097, 0.097);
  EXPECT_TRUE(a.done);
  EXPECT_DOUBLE_EQ(a.vx, 0.0);
  a = planning::approach_step(st, c, 0.096, 0.096);   // 같은 관측을 여러 주기 써도 기준점은 안 바뀜
  EXPECT_NEAR(a.remaining, 0.004, 1e-9);
}

TEST(Approach, WaitsForFaceAndStopsForObstacles)
{
  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;   // 면을 본 적 없음 → 움직이지 않음
  planning::approach_observe(st, c, std::nan(""), 0.5, 0.0);
  auto a = planning::approach_step(st, c, 0.0, 0.0);
  EXPECT_TRUE(a.waiting);
  EXPECT_DOUBLE_EQ(a.vx, 0.0);
  planning::ApproachState st2;
  see_face(st2, c, 0.18, 0.18, 0.0);   // 이미 목표보다 가까움 → 바로 끝 (뒤로 안 감)
  EXPECT_TRUE(planning::approach_step(st2, c, 0.0, 0.0).done);
  planning::ApproachState st3;   // 차체 폭 띠에 박스 면보다 가까운 물체 → 멈춤
  see_face(st3, c, 0.29, 0.24, 0.0);
  EXPECT_TRUE(planning::approach_step(st3, c, 0.0, 0.0).fail);
  planning::ApproachState st5;   // 면이 사라진 뒤 보인 앞 물체는 마지막 면 관측 기준으로 (이동량을 두 번 빼지 않음)
  see_face(st5, c, 0.29, 0.29, 0.0);
  planning::approach_observe(st5, c, std::nan(""), 0.26, 0.03);
  a = planning::approach_step(st5, c, 0.03, 0.03);
  EXPECT_FALSE(a.fail);
  EXPECT_NEAR(a.remaining, 0.29 - 0.03 - 0.191, 1e-9);
  planning::ApproachState st4;   // 많이 가면 실패
  see_face(st4, c, 0.40, 0.40, 0.0);
  EXPECT_TRUE(planning::approach_step(st4, c, 0.15, 0.15).fail);
}

// 면을 본 뒤 먼 배경으로 뛴 면은 버리고 (마지막 면 + odom 으로 이어감), 가까워진 면은 받음
TEST(Approach, IgnoresFarJumpAfterFace)
{
  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;
  see_face(st, c, 0.29, 0.29, 0.0);
  planning::approach_observe(st, c, 0.70, 0.70, 0.06);   // 박스 반사가 사라지고 옆 배경 (예측 0.23)
  EXPECT_DOUBLE_EQ(st.face_m, 0.29);
  EXPECT_DOUBLE_EQ(st.front_m, 0.29);                     // 안전용 앞 거리도 먼 값으로 안 덮음
  auto a = planning::approach_step(st, c, 0.06, 0.06);
  EXPECT_FALSE(a.fail);
  EXPECT_NEAR(a.remaining, 0.29 - 0.06 - 0.191, 1e-9);
  EXPECT_LT(a.vx, c.speed);                               // 0.70 기준 최고 속도가 아님
  planning::approach_observe(st, c, 0.245, 0.245, 0.06);  // 예측 0.23 + 1.5 cm (잡음 안) → 받음
  EXPECT_DOUBLE_EQ(st.face_m, 0.245);
  planning::approach_observe(st, c, 0.20, 0.20, 0.08);    // 예측보다 가까움 → 받음 (일찍 멈출 뿐)
  EXPECT_DOUBLE_EQ(st.face_m, 0.20);
  EXPECT_DOUBLE_EQ(st.travel_m, 0.08);
}

// 첫 면은 연속 두 스캔이 맞아야 받음: 첫 스캔이 튀었으면 (실제 0.29 인데 0.24) 그 값으로 움직이지 않음
TEST(Approach, FirstFaceNeedsTwoScans)
{
  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;
  planning::approach_observe(st, c, 0.29, 0.29, 0.0);     // 면 → 면 없음 → 같은 면: 연속이 아님 → 아직 면 아님
  planning::approach_observe(st, c, std::nan(""), std::nan(""), 0.0);
  planning::approach_observe(st, c, 0.29, 0.29, 0.0);
  EXPECT_TRUE(planning::approach_step(st, c, 0.0, 0.0).waiting);
  planning::approach_observe(st, c, 0.24, 0.24, 0.0);     // 튄 값 → 아직 면 아님
  EXPECT_TRUE(planning::approach_step(st, c, 0.0, 0.0).waiting);
  planning::approach_observe(st, c, 0.29, 0.29, 0.0);     // 5 cm 다름 → 이것을 다음 비교 대상으로
  EXPECT_TRUE(planning::approach_step(st, c, 0.0, 0.0).waiting);
  planning::approach_observe(st, c, 0.288, 0.288, 0.0);   // 맞음 → 받음
  const auto a = planning::approach_step(st, c, 0.0, 0.0);
  EXPECT_FALSE(a.waiting);
  EXPECT_NEAR(a.remaining, 0.288 - 0.191, 1e-9);
}

// 한 스캔만 튄 가까운 면 (잡음 · 지나간 물체) 뒤 실제 면은 그 앞 면 예측에 맞아 받음 → 일찍 끝내지 않음
TEST(Approach, RecoversAfterOneCloseGlitch)
{
  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;
  see_face(st, c, 0.29, 0.29, 0.0);
  planning::approach_observe(st, c, 0.24, 0.24, 0.01);    // 튄 값 (예측 0.28) → 일단 받음
  planning::approach_observe(st, c, 0.27, 0.27, 0.02);    // 예측 0.23 보다 4 cm 멂, 그 앞 면 (0.29) 예측 0.27 에 맞음 → 받음
  EXPECT_DOUBLE_EQ(st.face_m, 0.27);
  auto a = planning::approach_step(st, c, 0.055, 0.055);
  EXPECT_FALSE(a.done);
  EXPECT_NEAR(a.remaining, 0.27 - 0.035 - 0.191, 1e-9);
  planning::approach_observe(st, c, 0.70, 0.70, 0.06);    // 그 뒤 먼 배경은 둘 다에 안 맞음 → 버림
  EXPECT_DOUBLE_EQ(st.face_m, 0.27);
  planning::ApproachState st2;   // 앞 면 0.35 · 지금 면 0.23 (가까운 물체) 뒤 0.28: 지금보다 5 cm 멀고 앞 면보다 7 cm 가까움 → 버림
  see_face(st2, c, 0.35, 0.35, 0.0);
  planning::approach_observe(st2, c, 0.23, 0.23, 0.0);
  planning::approach_observe(st2, c, 0.28, 0.28, 0.0);
  EXPECT_DOUBLE_EQ(st2.face_m, 0.23);
  EXPECT_DOUBLE_EQ(st2.front_m, 0.23);                    // 안전용 앞 거리도 그대로
}

// 직진 거리는 부호 있음 (시작점보다 뒤에서 찍힌 스캔), 안전 상한은 직선 거리로 따로
TEST(Approach, SignedTravelAndStraightLineLimit)
{
  planning::ApproachConfig c;
  c.target_face_m = 0.191;
  c.max_travel_m = 0.14;
  planning::ApproachState st;
  see_face(st, c, 0.31, 0.31, -0.02);   // 시작점 2 cm 뒤에서 찍힘 → 시작점에서는 0.29
  auto a = planning::approach_step(st, c, 0.0, 0.0);
  EXPECT_NEAR(a.remaining, 0.31 - 0.02 - 0.191, 1e-9);
  a = planning::approach_step(st, c, 0.05, 0.15);          // 앞으로는 5 cm 지만 시작점에서 15 cm 벗어남 → 실패
  EXPECT_TRUE(a.fail);
}
