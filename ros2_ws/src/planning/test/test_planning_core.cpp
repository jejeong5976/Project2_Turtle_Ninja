// test_planning_core.cpp — planning 실행 층 순수 계산 시험, ROS 없이: 기록부 · grasp · 적재함 · 재고 불확실 · 감시
//   실행: colcon build → source install/setup.bash → ./build/planning/test_planning_core

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <map>
#include <set>

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

// 색 · 모양 = 현장에 있을 때 본 장의 다수결. 한 장 오검출은 기록을 못 바꿈, 표가 많아지면 바뀜
TEST(PuckRegistry, KindIsMajorityWhileOnField)
{
  PuckRegistry reg;
  auto r = rec("11", 1.0, 1.0, 0.0, "green");
  r.shape = "cylinder";
  for (int i = 0; i < 3; ++i) {reg.update(r);}
  auto b = r;
  b.shape = "box";
  b.x = 1.01;
  reg.update(b);                                   // 한 장 box → 그대로 원기둥, 위치는 새 값
  EXPECT_EQ(reg.find("11")->shape, "cylinder");
  EXPECT_DOUBLE_EQ(reg.find("11")->x, 1.01);
  reg.update(b);
  reg.update(b);                                   // 3 : 3 → 지금 값 유지
  EXPECT_EQ(reg.find("11")->shape, "cylinder");
  reg.update(b);                                   // 4 : 3 → box
  EXPECT_EQ(reg.find("11")->shape, "box");
  auto u = b;
  u.shape = "";
  u.color = "";
  reg.update(u);                                   // 모르는 값은 표가 아님
  EXPECT_EQ(reg.find("11")->shape, "box");
  EXPECT_EQ(reg.find("11")->color, "green");
}

// 다루는 중 (vote_kind false) · 손 · 칸 · 놓은 뒤 (on_field 아님) 검출은 위치만 바꿈
TEST(PuckRegistry, KindFrozenWhileHandledOrOffField)
{
  PuckRegistry reg;
  auto r = rec("11", 1.0, 1.0, 0.0, "green");
  r.shape = "cylinder";
  reg.update(r);
  auto b = r;
  b.shape = "box";
  b.color = "red";
  for (int i = 0; i < 5; ++i) {reg.update(b, false);}   // PICK 중
  EXPECT_EQ(reg.find("11")->shape, "cylinder");
  EXPECT_EQ(reg.find("11")->color, "green");
  reg.set_state("11", "in_cargo:S11");
  b.x = 2.0;
  for (int i = 0; i < 5; ++i) {reg.update(b);}          // 칸 안
  EXPECT_EQ(reg.find("11")->shape, "cylinder");
  EXPECT_DOUBLE_EQ(reg.find("11")->x, 2.0);
  // 처음 본 장이 다루는 중이어도 빈 기록은 채움 (모르는 것보다 나음), 다음 현장 표가 바로 이김
  reg.update([] {auto q = rec("12", 0.0, 0.0, 0.0, "red"); q.shape = "box"; return q;}(), false);
  EXPECT_EQ(reg.find("12")->shape, "box");
  reg.update([] {auto q = rec("12", 0.0, 0.0, 1.0, "red"); q.shape = "cylinder"; return q;}());
  EXPECT_EQ(reg.find("12")->shape, "cylinder");
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

// 칸 제약: 넣기는 제약을 지키는 첫 빈 칸 (없으면 첫 빈 칸), 빼기 가능 여부
TEST(Cargo, SlotRulesPickStoreAndRetrieveOrder)
{
  Cargo c({"S12", "S7", "S9"});
  c.set_rules({{"S12", "S7"}}, {{"S7", "S12"}});
  auto put = [&](const std::string & id, const std::string & slot) {
      ASSERT_TRUE(c.on_pick(CargoItem{id, "red", "circle", "", false}));
      ASSERT_TRUE(c.on_store(slot));
    };
  EXPECT_EQ(c.free_slot().value(), "S12");   // S7 비어 있음 → S12 그대로
  put("1", "S12");
  put("2", "S7");
  EXPECT_FALSE(c.retrieve_ok("S7"));          // S12 가 참
  EXPECT_TRUE(c.retrieve_ok("S12"));
  ASSERT_TRUE(c.on_retrieve("S12"));
  ASSERT_TRUE(c.on_place().has_value());
  EXPECT_TRUE(c.retrieve_ok("S7"));
  // S12 가 비었지만 S7 이 참 → 넣기는 S9 먼저
  EXPECT_EQ(c.free_slot().value(), "S9");
  put("3", "S9");
  EXPECT_EQ(c.free_slot().value(), "S12");   // 지키는 칸이 없으면 첫 빈 칸 (막지 않음)
}

// 빼기 짝 칸 (S7 · S12): 짝에 다른 색 퍽이 있으면 그 칸은 미루고, 같은 종류 퍽이 오면 그 칸. 막는 칸 알려 줌
TEST(Cargo, FreeSlotForKeepsPairsSameKind)
{
  Cargo c({"S12", "S7", "S9"});
  c.set_rules({}, {{"S7", "S12"}});
  const CargoItem red{"1", "red", "cylinder", "", false}, green{"2", "green", "box", "", false}, red2{"3", "red", "cylinder", "", false};
  ASSERT_TRUE(c.on_pick(red));
  ASSERT_TRUE(c.on_store(*c.free_slot_for(red)));     // S12
  EXPECT_EQ(c.free_slot_for(green).value(), "S9");     // S7 짝 (S12) 이 빨강 → 미룸
  EXPECT_EQ(c.free_slot_for(red2).value(), "S7");
  ASSERT_TRUE(c.on_pick(red2));
  ASSERT_TRUE(c.on_store("S7"));
  EXPECT_EQ(c.retrieve_blocker("S7").value(), "S12");
  EXPECT_FALSE(c.retrieve_blocker("S12").has_value());
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

// 빈손 확인은 PICK 이 수락 뒤 실패 + 결과 글에 EMPTY_GRASP 일 때만 (맨 앞 · ArmRunner "팔 작업 중단: " 뒤 모두)
TEST(InventoryPolicy, EmptyGraspOnlyForFailedPickWithToken)
{
  EXPECT_TRUE(empty_grasp_confirmed(kArmPick, ArmEnd::Failed, "EMPTY_GRASP 빈손: 그리퍼가 -0.0010 m 까지 닫힘"));
  EXPECT_TRUE(empty_grasp_confirmed(kArmPick, ArmEnd::Failed, "팔 작업 중단: EMPTY_GRASP 빈손"));
  EXPECT_FALSE(empty_grasp_confirmed(kArmPick, ArmEnd::Failed, "팔 작업 중단: 빈손: 그리퍼가 닫힘"));   // 표시 없음 → 불확실
  EXPECT_FALSE(empty_grasp_confirmed(kArmPick, ArmEnd::Failed, ""));
  EXPECT_FALSE(empty_grasp_confirmed(kArmPick, ArmEnd::Unknown, "EMPTY_GRASP"));    // 결과 모름 · 시간 초과
  EXPECT_FALSE(empty_grasp_confirmed(kArmPick, ArmEnd::Mismatch, "EMPTY_GRASP"));
  EXPECT_FALSE(empty_grasp_confirmed(kArmPick, ArmEnd::Rejected, "EMPTY_GRASP"));
  for (const uint8_t t : {kArmStore, kArmRetrieve, kArmPlace, kArmLookRack, kArmHome, kArmLook}) {   // PICK 만
    EXPECT_FALSE(empty_grasp_confirmed(t, ArmEnd::Failed, "EMPTY_GRASP")) << int(t);
  }
}

// STORE 박스 되돌림 (control store_recover): STORE 수락 뒤 실패 + 결과 글에 STORE_RETURNED_TO_BOX x y 일 때만
TEST(InventoryPolicy, StoreReturnedOnlyForFailedStoreWithTokenAndNumbers)
{
  double x = 0.0, y = 0.0;
  EXPECT_TRUE(store_returned_confirmed(kArmStore, ArmEnd::Failed, "STORE_RETURNED_TO_BOX 0.2149 -0.0312 · STORE 보정: 칸 넣기 걸림", &x, &y));
  EXPECT_DOUBLE_EQ(x, 0.2149);
  EXPECT_DOUBLE_EQ(y, -0.0312);
  EXPECT_TRUE(store_returned_confirmed(kArmStore, ArmEnd::Failed, "EMPTY_GRASP: 팔 작업 중단: 빈손 · STORE_RETURNED_TO_BOX 0.2 0.0"));
  EXPECT_FALSE(store_returned_confirmed(kArmStore, ArmEnd::Failed, "팔 동작 실패: 'release' 동작이 성공하지 못함"));   // 표시 없음 → 불확실
  EXPECT_FALSE(store_returned_confirmed(kArmStore, ArmEnd::Failed, "STORE_RETURNED_TO_BOX"));         // 숫자 없음
  EXPECT_FALSE(store_returned_confirmed(kArmStore, ArmEnd::Failed, "STORE_RETURNED_TO_BOX nan 0.1"));
  EXPECT_FALSE(store_returned_confirmed(kArmStore, ArmEnd::Unknown, "STORE_RETURNED_TO_BOX 0.2 0.0"));   // 결과 모름
  EXPECT_FALSE(store_returned_confirmed(kArmStore, ArmEnd::Succeeded, "STORE_RETURNED_TO_BOX 0.2 0.0"));
  for (const uint8_t t : {kArmPick, kArmRetrieve, kArmPlace, kArmLookRack, kArmHome}) {   // STORE 만
    EXPECT_FALSE(store_returned_confirmed(t, ArmEnd::Failed, "STORE_RETURNED_TO_BOX 0.2 0.0")) << int(t);
  }
}

TEST(InventoryPolicy, StoreReturnedClearsGripperOnlyForThatPuck)
{
  Cargo c({"R1", "R2"});
  ASSERT_TRUE(c.on_pick(item("7")));
  std::string state, err;
  EXPECT_FALSE(apply_store_returned(c, "8", &state, &err)) << "다른 퍽";
  EXPECT_TRUE(c.holding());
  EXPECT_FALSE(err.empty());
  ASSERT_TRUE(apply_store_returned(c, "7", &state, &err)) << err;
  EXPECT_FALSE(c.holding());
  EXPECT_EQ(state, "on_field");
  EXPECT_FALSE(c.slots()[0].second.has_value());
  EXPECT_FALSE(c.slots()[1].second.has_value());
  EXPECT_FALSE(c.doubt().has_value());
  EXPECT_FALSE(apply_store_returned(c, "7", &state, &err)) << "그리퍼 빔 (두 번 반영 안 함)";
  std::string pre;
  EXPECT_TRUE(arm_preflight_ok(c, kArmPick, "7", "", &pre)) << pre << " — 다시 집을 수 있음";
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

// 빈손 LOOK_RACK (RETRIEVE 빈손 자동 정합 확인): 퍽 ID 없음 + 그리퍼 기록 빔 = 보기만 → 통과. 그리퍼에 퍽 있으면 ID 필요
TEST(InventoryPolicy, PreflightEmptyHandLookRackOnlyWhenGripperEmpty)
{
  Cargo c({"R1", "R2"});
  c.on_pick(item("3"));
  c.on_store("R2");
  std::string err;
  EXPECT_TRUE(arm_preflight_ok(c, kArmLookRack, "", "R2", &err)) << err;
  EXPECT_FALSE(arm_preflight_ok(holding7_r2_3(), kArmLookRack, "", "R1", &err));
  EXPECT_FALSE(arm_preflight_ok(c, kArmStore, "", "R1", &err));   // STORE 는 그대로 ID 필요
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

// 전환이 거부되면 기록부를 안 바꾸고 false
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

  Cargo plain({"R1", "R2"});                     // 표시 없는 파일 → 표시 없음 (표시 없는 파일 호환)
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

// 실행기 위치 감시 = ConditionMonitor(0.0, loc_max_recoveries, loc_fault_s)
//   기본 3: 짧은 끊김 3 번까지 넘기고 4 번째에 FAULT. -1: 횟수 제한 없음, loc_fault_s 넘게 이어질 때만 FAULT
TEST(Health, LocMonitorRecoveriesParam)
{
  ConditionMonitor def(0.0, 3, 5.0);
  ConditionMonitor unl(0.0, -1, 5.0);
  for (int i = 0; i < 3; ++i) {
    const double t = i * 10.0;
    EXPECT_EQ(def.update(true, t), Health::Waiting);
    EXPECT_EQ(def.update(false, t + 1.0), Health::Ok);   // 회복 대기 0 s → 좋아지면 바로 해제
  }
  EXPECT_EQ(def.update(true, 30.0), Health::Fault);     // 4 번째 끊김

  for (int i = 0; i < 50; ++i) {
    const double t = i * 10.0;
    EXPECT_EQ(unl.update(true, t), Health::Waiting) << i;
    EXPECT_EQ(unl.update(true, t + 4.0), Health::Waiting) << i;   // loc_fault_s 안
    EXPECT_EQ(unl.update(false, t + 4.5), Health::Ok) << i;
  }
  EXPECT_EQ(unl.update(true, 600.0), Health::Waiting);
  EXPECT_EQ(unl.update(true, 605.5), Health::Fault);   // 끊김이 loc_fault_s 넘게 이어지면 그대로 FAULT
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

// ---------------------------------------------------------------- 위치 추정 상실 복구 (LocRecovery)

namespace
{
// t0 부터 t1 까지 dt 간격으로 update, 물러나기 요청 횟수
int run_loc(LocRecovery & r, double t0, double t1, bool lost, bool stable, bool can_back, double dt = 0.1)
{
  int n = 0;
  for (double t = t0; t < t1 - 1e-9; t += dt) {
    if (r.update(lost, stable, can_back, t)) {++n;}
  }
  return n;
}
}  // namespace

// 잠깐 상실 → 대기 → 정상 3 s 유지 뒤 정상 (FAULT 개념 없음)
TEST(LocRecovery, ShortLossWaitsThenRecovers)
{
  LocRecovery r;
  EXPECT_EQ(run_loc(r, 0.0, 6.0, true, false, true), 0);   // wait_s 10 전
  EXPECT_EQ(r.mode(), LocMode::Wait);
  EXPECT_TRUE(r.blocks());
  run_loc(r, 6.0, 8.9, false, false, true);
  EXPECT_EQ(r.mode(), LocMode::Wait);                       // 정상 3 s 안 됨
  run_loc(r, 8.9, 9.3, false, false, true);
  EXPECT_EQ(r.mode(), LocMode::Ok);
  EXPECT_FALSE(r.blocks());
}

// 대기 10 s 뒤 물러나기 요청, 끝나면 다시 대기 10 s 뒤 두 번째, 최대 2 번
TEST(LocRecovery, BackoffAfterWaitUpToMax)
{
  LocRecovery r;
  EXPECT_EQ(run_loc(r, 0.0, 10.15, true, false, true), 1);
  EXPECT_EQ(r.mode(), LocMode::Backoff);
  EXPECT_EQ(run_loc(r, 10.15, 14.0, true, false, true), 0);  // 물러나는 중엔 새 요청 없음
  r.backoff_done(14.0);
  EXPECT_EQ(r.mode(), LocMode::Wait);
  EXPECT_EQ(run_loc(r, 14.0, 24.15, true, false, true), 1);
  r.backoff_done(26.0);
  EXPECT_EQ(run_loc(r, 26.0, 29.9, true, false, true), 0);   // 2 번 다 씀
  EXPECT_EQ(r.backoffs(), 2);
}

// 물러나기 못 함 (박스 앞 팔 작업 중) + 불안정이어도 force_s 60 s 넘으면 조건부 계속 = 무한 대기 없음
TEST(LocRecovery, ForceDegradeWhenStuck)
{
  LocRecovery r;
  EXPECT_EQ(run_loc(r, 0.0, 59.9, true, false, false), 0);
  EXPECT_EQ(r.mode(), LocMode::Wait);
  run_loc(r, 59.9, 60.2, true, false, false);
  EXPECT_EQ(r.mode(), LocMode::Degraded);
  EXPECT_FALSE(r.blocks());
}

// 상실 30 s + map→odom 안정 → 조건부 계속, 불안정이면 물러나기 다 쓸 때까지 아님
TEST(LocRecovery, DegradeWhenStableAfter30s)
{
  LocRecovery a;
  run_loc(a, 0.0, 30.2, true, true, false);
  EXPECT_EQ(a.mode(), LocMode::Degraded);
  LocRecovery b;
  run_loc(b, 0.0, 30.2, true, false, false);
  EXPECT_EQ(b.mode(), LocMode::Wait);
  // 조건부 계속 중 정상 3 s → 정상, 다음 상실은 물러나기 횟수 새로
  run_loc(a, 30.2, 33.5, false, true, true);
  EXPECT_EQ(a.mode(), LocMode::Ok);
  EXPECT_EQ(run_loc(a, 33.5, 43.6, true, false, true), 1);
}

// 물러나는 중 정상 회복 → 정상 (실행기는 이동을 멈춤)
TEST(LocRecovery, RecoverDuringBackoff)
{
  LocRecovery r;
  run_loc(r, 0.0, 10.15, true, false, true);
  ASSERT_EQ(r.mode(), LocMode::Backoff);
  run_loc(r, 10.15, 13.3, false, false, true);
  EXPECT_EQ(r.mode(), LocMode::Ok);
  r.backoff_done(13.4);   // 늦게 와도 상태 안 바뀜
  EXPECT_EQ(r.mode(), LocMode::Ok);
}


// ============================================================================ 검출 끊김 구간 (perception 멈춤 · respawn)
TEST(DetectionGap, StartEndAndOverlap)
{
  DetectionGap g(1.0);
  EXPECT_EQ(g.update(false, 0.0), 0);
  EXPECT_FALSE(g.overlaps(0.0));
  EXPECT_EQ(g.update(true, 10.0), 1);          // 시작
  EXPECT_EQ(g.update(true, 12.0), 0);
  EXPECT_TRUE(g.active());
  EXPECT_TRUE(g.overlaps(100.0));               // 지금 끊김 = 언제부터 봐도 겹침
  EXPECT_EQ(g.update(false, 20.0), 0);          // 좋음 시작 (아직 1 s 안)
  EXPECT_EQ(g.update(true, 20.5), 0);           // 다시 나쁨 → 같은 끊김
  EXPECT_EQ(g.update(false, 21.0), 0);
  EXPECT_EQ(g.update(false, 22.1), -1);         // 1 s 이어짐 → 끝, 끝 시각 = 좋음 시작 21
  EXPECT_FALSE(g.active());
  EXPECT_DOUBLE_EQ(g.started(), 10.0);
  EXPECT_DOUBLE_EQ(g.ended(), 21.0);
  EXPECT_EQ(g.count(), 1);
  EXPECT_TRUE(g.overlaps(5.0));                 // LOOK 5 s → 끊김 10–21 과 겹침
  EXPECT_TRUE(g.overlaps(15.0));                // 끊김 도중 시작한 LOOK
  EXPECT_FALSE(g.overlaps(21.5));               // 회복 뒤 LOOK
  g.reset();
  EXPECT_FALSE(g.overlaps(0.0));
  EXPECT_EQ(g.count(), 0);
}

TEST(DetectionGap, Action)
{
  DetectionGap g(1.0);
  EXPECT_EQ(gap_action(g, 5.0, 0, 2), GapAction::Proceed);      // 끊김 없음
  EXPECT_EQ(gap_action(g, -1.0, 0, 2), GapAction::Proceed);     // 이 정차 LOOK 없음
  g.update(true, 10.0);
  EXPECT_EQ(gap_action(g, 5.0, 0, 2), GapAction::Wait);         // 끊김 중 → 미룸
  EXPECT_EQ(gap_action(g, 5.0, 2, 2), GapAction::Wait);         // 다 써도 끊김 중이면 기다림 (FAULT 는 감시)
  g.update(false, 30.0);
  g.update(false, 31.5);
  EXPECT_EQ(gap_action(g, 5.0, 0, 2), GapAction::Redo);         // 끝난 끊김과 겹침 → LOOK 다시
  EXPECT_EQ(gap_action(g, 5.0, 1, 2), GapAction::Redo);
  EXPECT_EQ(gap_action(g, 5.0, 2, 2), GapAction::Proceed);      // 다 씀 → 본 만큼으로
  EXPECT_EQ(gap_action(g, 5.0, 0, 2, false), GapAction::Proceed);   // 다시 끼우면 안 되는 단계 (재고 정합)
  EXPECT_EQ(gap_action(g, 31.0, 0, 2), GapAction::Proceed);     // 다시 한 LOOK (회복 뒤) → 그대로
}
