// test_transfer.cpp — mission 운반 계획 시험, ROS 없이: transfer_solver · transfer_planner · dice_map · 재검증 · 기록 파일
//   실행: colcon build → source install/setup.bash → ./build/mission/test_transfer

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <tuple>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/dice_steps.hpp"
#include "mission/dice_record.hpp"
#include "mission/verify_image.hpp"
#include "mission/dice_verify.hpp"
#include "mission/hand_state.hpp"
#include "mission/place_box.hpp"
#include "mission/transfer_planner.hpp"
#include "mission/transfer_solver.hpp"

using namespace mission;

namespace
{

// 시험용 지도: 남쪽 덩어리 {S, B1, B2, L1}, 북쪽 덩어리 {B3–B6, L2, L3} (실제 dice 맵 측정 시간표와 비슷한 모양)
TransferCosts test_costs()
{
  // 노드 좌표 [m] → 이동 시간 = 맨해튼 거리 / 0.13 m/s
  const double xy[kNumNodes][2] = {
    {0.7, 0.9}, {1.5, 0.7}, {3.7, 2.9}, {2.9, 5.1}, {0.7, 5.1}, {2.2, 5.6}, {3.4, 3.9},
    {3.3, 0.9}, {2.2, 4.1}, {3.7, 4.9}};
  TransferCosts c;
  for (int a = 0; a < kNumNodes; ++a) {
    for (int b = 0; b < kNumNodes; ++b) {
      c.travel_s[a][b] = (std::fabs(xy[a][0] - xy[b][0]) + std::fabs(xy[a][1] - xy[b][1])) / 0.13;
    }
  }
  return c;
}

std::vector<std::string> slot_names(int n)
{
  std::vector<std::string> v;
  for (int i = 1; i <= n; ++i) {v.push_back("S" + std::to_string(i));}
  return v;
}

// Q1 배치 (sim/robot_sim/mazes/dice_problem_q1.json 과 같음): 박스마다 (빨강, 초록), 모양 번갈아
std::vector<TransferPuck> q1_pucks()
{
  const PuckKind k[12] = {
    PuckKind::RedCylinder, PuckKind::GreenBox, PuckKind::RedBox, PuckKind::GreenCylinder,
    PuckKind::RedCylinder, PuckKind::GreenBox, PuckKind::RedBox, PuckKind::GreenCylinder,
    PuckKind::RedCylinder, PuckKind::GreenBox, PuckKind::RedBox, PuckKind::GreenCylinder};
  std::vector<TransferPuck> v;
  for (int i = 0; i < 12; ++i) {v.push_back(TransferPuck{std::to_string(i), k[i], i / 2 + 1});}
  return v;
}

// Q1: RGRRGRGGYGYG → R4 Y2 G6, P2-01 오름차순 → L1 = Y(2), L2 = R(4), L3 = G(6)
SolverInput q1_input()
{
  SolverInput in;
  in.place_group = {{'Y', 'R', 'G'}};
  in.slots_left = {{2, 4, 6}};
  return in;
}

// 계획대로 실행해 보고 장소별 퍽 · 그룹 조건 검사
void check_plan_rules(
  const TransferPlan & plan, const std::vector<TransferPuck> & all, const SolverInput & in)
{
  std::map<std::string, PuckKind> kind;
  for (const auto & p : all) {kind[p.id] = p.kind;}
  std::array<int, kNumPlaces> placed{{0, 0, 0}};
  std::set<std::string> seen;
  bool holding = in.gripper.has_value();
  for (const auto & s : plan.stops) {
    // 퍽 쥔 채 주행 안 함: 이동하는 정차는 그리퍼가 빈 채 시작
    if (s.move) {EXPECT_FALSE(holding) << "퍽 쥔 채 이동: " << s.name;}
    for (const auto & a : s.actions) {
      if (a.op == ArmOp::Pick || a.op == ArmOp::Retrieve) {holding = true;}
      if (a.op == ArmOp::Store || a.op == ArmOp::Place) {holding = false;}
      if (a.op != ArmOp::Place) {continue;}
      ASSERT_TRUE(seen.insert(a.puck_id).second) << "같은 퍽 두 번 놓음: " << a.puck_id;
      ASSERT_EQ(s.node, place_node(a.place)) << "다른 장소 정차에서 놓음";
      const PuckKind k = kind.at(a.puck_id);
      const char g = in.place_group[a.place];
      if (g == 'R') {EXPECT_TRUE(kind_is_red(k));}
      if (g == 'G') {EXPECT_FALSE(kind_is_red(k));}
      if (g == 'Y') {EXPECT_EQ(kind_is_box(k), plan.yellow_shape == "box");}
      EXPECT_EQ(a.cell, placed[a.place]);
      ++placed[a.place];
    }
  }
  for (int p = 0; p < kNumPlaces; ++p) {EXPECT_EQ(placed[p], in.slots_left[p]);}
}

}  // namespace

TEST(Transfer, KindStrings)
{
  EXPECT_EQ(puck_kind("red", "cylinder"), PuckKind::RedCylinder);
  EXPECT_EQ(puck_kind("green", "box"), PuckKind::GreenBox);
  EXPECT_FALSE(puck_kind("blue", "box").has_value());
  EXPECT_FALSE(puck_kind("red", "").has_value());
  EXPECT_STREQ(kind_shape(PuckKind::GreenCylinder), "cylinder");
  EXPECT_EQ(node_name(0), "S");
  EXPECT_EQ(node_name(3), "B3");
  EXPECT_EQ(node_name(place_node(1)), "L2");
}

TEST(Transfer, CollectActionsStoreEveryPuck)
{
  Cargo cargo(slot_names(12));
  const auto pucks = q1_pucks();
  std::string err;
  // 첫 박스: 그리퍼 빔 → PICK, STORE, PICK, STORE (퍽 쥔 채 주행 안 함 → 마지막 퍽도 넣음)
  auto a = collect_box_actions({pucks[0], pucks[1]}, cargo, &err);
  ASSERT_EQ(a.size(), 4u) << err;
  EXPECT_EQ(a[0].op, ArmOp::Pick);
  EXPECT_EQ(a[0].puck_id, "0");
  EXPECT_EQ(a[1].op, ArmOp::Store);
  EXPECT_EQ(a[1].puck_id, "0");
  EXPECT_EQ(a[1].slot, "S1");
  EXPECT_EQ(a[1].color, "red");
  EXPECT_EQ(a[1].shape, "cylinder");
  EXPECT_EQ(a[2].op, ArmOp::Pick);
  EXPECT_EQ(a[2].puck_id, "1");
  EXPECT_EQ(a[3].op, ArmOp::Store);
  EXPECT_EQ(a[3].puck_id, "1");
  EXPECT_EQ(a[3].slot, "S2");
  // 그리퍼에 퍽이 있으면 (재계획 등) 먼저 STORE
  ASSERT_TRUE(cargo.on_pick(CargoItem{"9", "green", "box", "", false}));
  a = collect_box_actions({pucks[2], pucks[3]}, cargo, &err);
  ASSERT_EQ(a.size(), 5u);
  EXPECT_EQ(a[0].op, ArmOp::Store);
  EXPECT_EQ(a[0].puck_id, "9");
  // 칸이 모자라면 실패
  Cargo small(slot_names(1));
  ASSERT_TRUE(small.on_pick(CargoItem{"9", "green", "box", "", false}));
  EXPECT_TRUE(collect_box_actions({pucks[2], pucks[3]}, small, &err).empty());
}

TEST(Transfer, AllLoadedPlansThreePlaceStops)
{
  // A안 계획 시점: B5 정차 중, 11개 실림 + 1개 그리퍼 (B5 의 오른쪽 퍽)
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 11; ++i) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{all[i].id, kind_color(all[i].kind), kind_shape(all[i].kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  ASSERT_TRUE(cargo.on_pick(CargoItem{all[11].id, kind_color(all[11].kind), kind_shape(all[11].kind), "", false}));
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  std::string err;
  ASSERT_TRUE(fill_carried(cargo, in, &err)) << err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  // 장소 3곳만 정차 (박스 재방문 없음)
  int place_stops = 0;
  for (const auto & s : plan->stops) {
    EXPECT_FALSE(is_box_node(s.node) && s.move) << "박스 재방문";
    if (s.move && is_place_node(s.node)) {++place_stops;}
  }
  EXPECT_EQ(place_stops, 3);
  check_plan_rules(*plan, all, in);
  EXPECT_EQ(plan->place_of.size(), 12u);
}

TEST(Transfer, AllLoadedMatchesBestPlaceOrder)
{
  // 모두 실린 경우의 비용은 장소 순서 6가지 중 최소 + 팔 동작 고정분과 같아야 함
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 12; ++i) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{all[i].id, kind_color(all[i].kind), kind_shape(all[i].kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  const TransferCosts c = test_costs();
  const auto sol = solve_transfer(in, c);
  ASSERT_TRUE(sol.has_value());

  double best = 1e18;
  std::array<int, 3> p{{0, 1, 2}};
  do {
    double t = c.travel_s[5][place_node(p[0])] + c.travel_s[place_node(p[0])][place_node(p[1])] +
      c.travel_s[place_node(p[1])][place_node(p[2])] + 3 * (c.stop_s + c.look_s);
    // 재검증 순회 (마지막 장소에서 시작, 최소)
    double v = 1e18;
    std::array<int, 3> q{{0, 1, 2}};
    do {
      v = std::min(v, c.travel_s[place_node(p[2])][place_node(q[0])] + c.travel_s[place_node(q[0])][place_node(q[1])] +
        c.travel_s[place_node(q[1])][place_node(q[2])] + c.travel_s[place_node(q[2])][kNodeStart]);
    } while (std::next_permutation(q.begin(), q.end()));
    best = std::min(best, t + v + 3 * (c.stop_s + c.look_s));
  } while (std::next_permutation(p.begin(), p.end()));
  best += 12 * (c.retrieve_s + c.place_s);
  EXPECT_NEAR(sol->cost_s, best, 1e-6);
}

TEST(Transfer, GripperPuckStoredBeforeDriving)
{
  // 재계획 등으로 그리퍼에 퍽이 있는 채 계획하면 장소로 떠나기 전에 STORE (퍽 쥔 채 주행 안 함)
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 11; ++i) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{all[i].id, kind_color(all[i].kind), kind_shape(all[i].kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  ASSERT_TRUE(cargo.on_pick(CargoItem{"11", "green", "cylinder", "", false}));
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}});
  ASSERT_TRUE(plan.has_value());
  check_plan_rules(*plan, all, in);
  // 첫 정차 (이동 없이 B5 에서 이어서) 의 첫 동작 = 그리퍼 퍽 11 STORE
  ASSERT_FALSE(plan->stops.empty());
  ASSERT_FALSE(plan->stops.front().move);
  ASSERT_FALSE(plan->stops.front().actions.empty());
  EXPECT_EQ(plan->stops.front().actions.front().op, ArmOp::Store);
  EXPECT_EQ(plan->stops.front().actions.front().puck_id, "11");
}

TEST(Transfer, ReplanPicksPuckLeftOnBox)
{
  // 집기 실패로 B3 에 퍽 1개가 남음 → 계획에 B3 방문 · 집기가 들어가야 함
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 12; ++i) {
    if (i == 4) {continue;}   // 퍽 4 (B3 왼쪽) 를 못 집음
    ASSERT_TRUE(cargo.on_pick(CargoItem{all[i].id, kind_color(all[i].kind), kind_shape(all[i].kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  in.on_boxes = {all[4]};
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  bool picked = false;
  for (const auto & s : plan->stops) {
    for (const auto & a : s.actions) {
      if (a.op == ArmOp::Pick && a.puck_id == "4") {picked = true; EXPECT_EQ(s.node, 3);}
    }
  }
  EXPECT_TRUE(picked);
  check_plan_rules(*plan, all, in);
}

TEST(Transfer, PartialProgressKeepsCellNumbers)
{
  // L3(G) 에 이미 2개 놓음 → 남은 초록 4개의 칸 번호는 2 부터
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  std::vector<TransferPuck> left;
  int skipped_green = 0;
  for (const auto & p : all) {
    if (!kind_is_red(p.kind) && skipped_green < 2) {++skipped_green; continue;}   // 이미 놓은 초록 2개
    left.push_back(p);
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = place_node(2);
  in.stopped = true;
  in.slots_left = {{2, 4, 4}};
  ASSERT_TRUE(fill_carried(cargo, in));
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 2}});
  ASSERT_TRUE(plan.has_value());
  int first_g_cell = -1;
  for (const auto & s : plan->stops) {
    for (const auto & a : s.actions) {
      if (a.op == ArmOp::Place && a.place == 2 && first_g_cell < 0) {first_g_cell = a.cell;}
    }
  }
  EXPECT_EQ(first_g_cell, 2);
}

TEST(Transfer, RejectsMismatchAndUnsolvable)
{
  std::string err;
  SolverInput in = q1_input();
  in.on_boxes = q1_pucks();
  in.on_boxes.pop_back();   // 퍽 11개, 칸 12개
  EXPECT_FALSE(solve_transfer(in, test_costs(), &err).has_value());
  EXPECT_NE(err.find("남은 퍽"), std::string::npos);

  // 개수로 할당 불가: 빨강 6 · 초록 6 인데 Y 2 · R 5 · G 5 → Y 에 빨강 1 + 초록 1 이 같은 모양이어야 함.
  // Y 가 cylinder 면 빨강 원기둥 3 · 초록 원기둥 3 이라 가능 → 대신 초록을 전부 사각기둥으로 바꿔 Y 원기둥 · 사각 둘 다 막음
  SolverInput bad = q1_input();
  bad.on_boxes = q1_pucks();
  for (auto & pk : bad.on_boxes) {
    if (!kind_is_red(pk.kind)) {pk.kind = PuckKind::GreenBox;}            // 초록 6개 모두 사각
    else if (kind_is_box(pk.kind)) {pk.kind = PuckKind::RedCylinder;}     // 빨강 6개 모두 원기둥
  }
  bad.slots_left = {{2, 5, 5}};    // Y 에 빨강 1 + 초록 1 필요 → 같은 모양 없음
  EXPECT_FALSE(solve_transfer(bad, test_costs(), &err).has_value());

  SolverInput wrong = q1_input();
  wrong.place_group = {{'R', 'X', 'G'}};
  EXPECT_FALSE(solve_transfer(wrong, test_costs(), &err).has_value());
}

TEST(Transfer, FullProblemFromStartUsesCollectFallback)
{
  // 출발점에서 12개 전부 남음 → 대체 해 (가까운 박스 순 수집 → 배달 정확해). 규칙을 지키고 박스는 한 번씩만 감
  SolverInput in = q1_input();
  in.on_boxes = q1_pucks();
  Cargo cargo(slot_names(12));
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  check_plan_rules(*plan, q1_pucks(), in);
  std::set<int> boxes;
  for (const auto & s : plan->stops) {
    if (is_box_node(s.node)) {EXPECT_TRUE(boxes.insert(s.node).second) << "박스 두 번 방문";}
  }
  EXPECT_EQ(boxes.size(), 6u);
}

TEST(Transfer, ExactSolverWithHalfRemaining)
{
  // 남은 퍽 6개 (B4–B6) + 실은 6개: 한도를 6 으로 올리면 정확해로 풀림 (기본 4 는 Jetson 시간 보호)
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  SolverInput in = q1_input();
  for (const auto & p : all) {
    if (p.box >= 4) {in.on_boxes.push_back(p); continue;}
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  in.position = 3;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err, 6);
  ASSERT_TRUE(plan.has_value()) << err;
  check_plan_rules(*plan, all, in);
  EXPECT_GT(plan->expanded, 0u);   // 정확해를 썼음 (대체 해면 배달만 풀어 수가 작음)
}

TEST(Transfer, SolverReportsStateLimit)
{
  SolverInput in = q1_input();
  in.on_boxes = q1_pucks();
  std::string err;
  EXPECT_FALSE(solve_transfer(in, test_costs(), &err, 1000).has_value());
  EXPECT_NE(err.find("한도"), std::string::npos);
}

TEST(Transfer, LoadsCostsYaml)
{
  const auto path = std::filesystem::temp_directory_path() / "transfer_costs_test.yaml";
  {
    std::ofstream f(path);
    f << "nodes: [S, B1, B2, B3, B4, B5, B6, L1, L2, L3]\ntravel_s:\n";
    for (int a = 0; a < kNumNodes; ++a) {
      f << "  - [";
      for (int b = 0; b < kNumNodes; ++b) {f << (a == b ? 0 : 10 + a + b) << (b + 1 < kNumNodes ? ", " : "");}
      f << "]\n";
    }
    f << "stop_s: 12.5\nverify_tour: false\n";
  }
  TransferCosts c;
  std::string err;
  ASSERT_TRUE(load_transfer_costs(path.string(), c, &err)) << err;
  EXPECT_DOUBLE_EQ(c.travel_s[2][5], 17.0);
  EXPECT_DOUBLE_EQ(c.stop_s, 12.5);
  EXPECT_FALSE(c.verify_tour);
  EXPECT_DOUBLE_EQ(c.pick_s, 30.0);   // 없는 값은 기본값
  std::filesystem::remove(path);
}

TEST(Transfer, Q1WithMeasuredTravelTable)
{
  // 레포 maps/dice_travel_times.yaml (Isaac 측정) 로 A안 전체: 수집 6곳 → 계획 → 배달
  TransferCosts c;
  std::string err;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, c, &err)) << err;
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  double collect = 0.0;
  int pos = kNodeStart;
  for (const int b : {1, 2, 6, 3, 4, 5}) {
    collect += c.travel_s[pos][b] + c.stop_s + c.look_s;
    pos = b;
    if (b == 5) {break;}   // 마지막 박스는 LOOK 뒤 계획이 집는 순서까지 정함
    std::vector<TransferPuck> here;
    for (const auto & p : all) {if (p.box == b) {here.push_back(p);}}
    for (const auto & a : collect_box_actions(here, cargo, &err)) {
      if (a.op == ArmOp::Store) {ASSERT_TRUE(cargo.on_store(a.slot)); collect += c.store_s;}
      if (a.op == ArmOp::Pick) {ASSERT_TRUE(cargo.on_pick(CargoItem{a.puck_id, a.color, a.shape, "", false})); collect += c.pick_s;}
    }
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  for (const auto & p : all) {if (p.box == 5) {in.on_boxes.push_back(p);}}
  ASSERT_TRUE(fill_carried(cargo, in, &err)) << err;
  const auto plan = plan_transfer(in, c, cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  check_plan_rules(*plan, all, in);
  const double total = collect + plan->cost_s;
  // 평가기 (문제 100개 평균) A안 약 1600 s 와 같은 범위
  EXPECT_GT(total, 1300.0);
  EXPECT_LT(total, 1900.0);
  std::printf("  Q1 A안 예상 %.0f s (수집 %.0f + 계획 이후 %.0f), Y 모양 %s, 정차 %zu\n",
    total, collect, plan->cost_s, plan->yellow_shape.c_str(), plan->stops.size());
}

TEST(Transfer, RetrievesNearestSlotFirst)
{
  // 칸 목록 = 먼 칸부터 (S1 가장 멂). 12 개를 S1 → S12 로 다 실은 뒤 배달하면
  // 같은 종류 안에서는 목록 뒤 칸 (가까운 칸) 부터 꺼내야 함
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (const auto & p : all) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  std::map<std::string, PuckKind> kind_of;
  for (const auto & p : all) {kind_of[p.id] = p.kind;}
  std::map<int, int> last_no;               // 종류별 직전에 꺼낸 칸 번호
  int n = 0;
  for (const auto & s : plan->stops) {
    for (const auto & a : s.actions) {
      if (a.op != ArmOp::Retrieve) {continue;}
      const int no = std::stoi(a.slot.substr(1));
      const int k = static_cast<int>(kind_of.at(a.puck_id));
      if (last_no.count(k)) {EXPECT_LT(no, last_no[k]) << "같은 종류는 번호 큰 칸 (가까운 칸) 부터: " << a.slot;}
      last_no[k] = no;
      ++n;
    }
  }
  EXPECT_EQ(n, 12);
}

// 채우는 순서 + 칸 제약 (planning.yaml): 다 실은 뒤 배달할 때 S12 가 찬 채 S7 · S5 가 찬 채 S10 · S2 가 찬 채 S1 을 안 뺌
//   (같은 장소 안 순서 바꾸기 → 그래도 걸리면 막는 퍽을 빈 칸으로 옮김: RETRIEVE → STORE)
TEST(Transfer, RetrieveKeepsRackRules)
{
  const auto all = q1_pucks();
  Cargo cargo({"S1", "S2", "S3", "S8", "S11", "S6", "S12", "S7", "S10", "S5", "S9", "S4"});
  cargo.set_rules({{"S1", "S2"}, {"S9", "S4"}, {"S10", "S5"}, {"S11", "S6"}, {"S12", "S7"}}, {{"S1", "S2"}, {"S7", "S12"}, {"S10", "S5"}});
  for (const auto & p : all) {
    const CargoItem it{p.id, kind_color(p.kind), kind_shape(p.kind), "", false};
    ASSERT_TRUE(cargo.on_pick(it));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot_for(it)));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  std::set<std::string> full;
  for (const auto & s : cargo.slots()) {full.insert(s.first);}
  const std::map<std::string, std::string> needs = {{"S1", "S2"}, {"S7", "S12"}, {"S10", "S5"}};
  int placed = 0;
  for (const auto & s : plan->stops) {
    for (const auto & a : s.actions) {
      if (a.op == ArmOp::Store) {full.insert(a.slot);}
      if (a.op == ArmOp::Place) {++placed;}
      if (a.op != ArmOp::Retrieve) {continue;}
      const auto it = needs.find(a.slot);
      if (it != needs.end()) {EXPECT_EQ(full.count(it->second), 0u) << a.slot << " 를 " << it->second << " 가 찬 채 뺌";}
      full.erase(a.slot);
    }
  }
  EXPECT_EQ(placed, 12);
  EXPECT_TRUE(full.empty());
}

// 한 장소에 놓을 두 종류: 먼저 꺼낼 종류의 칸이 제약에 걸리면 (S7 ← S12) 같은 장소의 다른 종류부터 꺼냄
TEST(Transfer, SamePlaceRetrieveReorderedForRackRules)
{
  Cargo cargo({"S12", "S7"});
  cargo.set_rules({}, {{"S7", "S12"}});
  ASSERT_TRUE(cargo.on_pick(CargoItem{"a", "red", "cylinder", "", false}));
  ASSERT_TRUE(cargo.on_store("S12"));
  ASSERT_TRUE(cargo.on_pick(CargoItem{"b", "red", "box", "", false}));
  ASSERT_TRUE(cargo.on_store("S7"));
  SolverInput in;
  in.place_group = {{'R', 'G', 'Y'}};
  in.slots_left = {{2, 0, 0}};
  in.cargo_capacity = 2;
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  std::vector<std::string> order;
  for (const auto & st : plan->stops) {
    for (const auto & a : st.actions) {
      if (a.op == ArmOp::Retrieve) {order.push_back(a.slot);}
    }
  }
  EXPECT_EQ(order, (std::vector<std::string>{"S12", "S7"}));
}

TEST(Transfer, PicksNearestFirstAtBox)
{
  // 마지막 박스 B5 에 종류가 다른 퍽 2 개 → on_boxes 순서 (노드가 가까운 순으로 넣음) 대로만 집어야 함
  // 순서를 뒤집어도 계획의 첫 PICK 이 항상 on_boxes[0]
  const auto all = q1_pucks();
  for (const bool swap : {false, true}) {
    Cargo cargo(slot_names(12));
    for (const auto & p : all) {
      if (p.box == 5) {continue;}
      ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
      ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
    }
    SolverInput in = q1_input();
    in.position = 5;
    in.stopped = true;
    for (const auto & p : all) {if (p.box == 5) {in.on_boxes.push_back(p);}}
    if (swap) {std::swap(in.on_boxes[0], in.on_boxes[1]);}
    ASSERT_TRUE(fill_carried(cargo, in));
    std::string err;
    const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
    ASSERT_TRUE(plan.has_value()) << err;
    std::vector<std::string> picks;
    for (const auto & s : plan->stops) {
      for (const auto & a : s.actions) {if (a.op == ArmOp::Pick) {picks.push_back(a.puck_id);}}
    }
    ASSERT_EQ(picks.size(), 2u);
    EXPECT_EQ(picks[0], in.on_boxes[0].id) << (swap ? "뒤집은 순서" : "기본 순서");
    EXPECT_EQ(picks[1], in.on_boxes[1].id);
  }
}

TEST(Transfer, RetrieveOnlyAtPlaceBeforePlacing)
{
  // 꺼내기는 장소 정차 안에서 바로 놓을 퍽만 → 정차 마지막 동작이 RETRIEVE 이면 안 됨 (퍽 쥔 채 다음 장소로 가지 않음)
  const auto all = q1_pucks();
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 12; ++i) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{all[i].id, kind_color(all[i].kind), kind_shape(all[i].kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}});
  ASSERT_TRUE(plan.has_value());
  for (const auto & s : plan->stops) {
    for (size_t i = 0; i < s.actions.size(); ++i) {
      if (s.actions[i].op != ArmOp::Retrieve) {continue;}
      EXPECT_TRUE(is_place_node(s.node));
      ASSERT_LT(i + 1, s.actions.size()) << "RETRIEVE 뒤 PLACE 없음 (" << s.name << ")";
      EXPECT_EQ(s.actions[i + 1].op, ArmOp::Place);
      EXPECT_EQ(s.actions[i + 1].puck_id, s.actions[i].puck_id);
    }
  }
}

// ============================================================================ dice_map · dice_steps

TEST(DiceMap, LoadsRepoFileWithStartOrigin)
{
  // map_from_arena: start → 출발 자세가 map 원점 (SLAM 매핑을 출발 자리에서 시작)
  DiceMap a;
  std::string err;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a, &err)) << err;
  EXPECT_NEAR(a.start.x, 0.0, 1e-9);
  EXPECT_NEAR(a.start.y, 0.0, 1e-9);
  EXPECT_NEAR(a.start.yaw, 0.0, 1e-4);
  // B1 중심 (1.468, 0.950) = 출발에서 +X 0.734 → 출발이 −X 를 보므로 map 에서는 뒤쪽 (−0.734, 0)
  EXPECT_NEAR(a.boxes[0].center[0], -0.734, 1e-3);
  EXPECT_NEAR(a.boxes[0].center[1], 0.0, 1e-3);
  // L1 보는 방향 (남쪽 정차): 경기장 +Y → map −Y
  EXPECT_NEAR(a.places[0].view_dir[0], 0.0, 1e-3);
  EXPECT_NEAR(a.places[0].view_dir[1], -1.0, 1e-3);
  EXPECT_NEAR(a.places[0].top_z, 0.07, 1e-9);
  EXPECT_NEAR(a.box_radius, 0.15, 1e-9);
  // L2 = 205 박스, 정차점이 격자 중심에서 0.2575 m (남쪽 정차 · 북쪽을 봄)
  EXPECT_NEAR(std::hypot(a.places[1].stop.x - a.places[1].center[0], a.places[1].stop.y - a.places[1].center[1]), 0.2575, 1e-3);
}

TEST(DiceMap, ExplicitTransformAndInverse)
{
  const auto path = std::filesystem::temp_directory_path() / "dice_map_sim_test.yaml";
  {
    std::ofstream f(path);
    f << "map_from_arena: [-2.202, -3.152, 0.0]\nstart: [0.734, 0.950, 3.1416]\nbarcode_view: [0.734, 0.950, 3.1416]\n"
      << "boxes:\n";
    for (int i = 1; i <= 6; ++i) {f << "  B" << i << ": {center: [1.0, 1.0], stop: [1.0, 0.7, 1.5708]}\n";}
    f << "places:\n";
    for (int i = 1; i <= 3; ++i) {f << "  L" << i << ": {center: [2.0, 2.0], stop: [2.0, 1.7, 1.5708]}\n";}
  }
  DiceMap a;
  std::string err;
  ASSERT_TRUE(load_dice_map(path.string(), a, &err)) << err;
  EXPECT_NEAR(a.start.x, 0.734 - 2.202, 1e-9);     // 시뮬 지도 = 경기장 − (2.202, 3.152)
  EXPECT_NEAR(a.boxes[2].center[1], 1.0 - 3.152, 1e-9);
  EXPECT_NEAR(a.places[1].view_dir[1], 1.0, 1e-4);
  std::filesystem::remove(path);

  const Pose2D p{1.2, -0.4, 0.7};
  const auto q = arena_to_map(inverse_pose(p), p);   // 자기 역변환을 거치면 원점
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.yaw, 0.0, 1e-9);

  DiceMap bad;
  EXPECT_FALSE(load_dice_map("/nonexistent/dice_map.yaml", bad, &err));
}

TEST(DiceSteps, CollectStepsPerBox)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  ASSERT_EQ(steps.size(), 18u);   // 박스마다 이동 · LOOK · Plan
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(steps[3 * i].type, StepType::Navigate);
    EXPECT_EQ(steps[3 * i + 1].arm_task, kArmLook);
    EXPECT_EQ(steps[3 * i + 2].type, StepType::Custom);
    EXPECT_EQ(steps[3 * i + 2].kind, i == 5 ? kPlanDelivery : kPlanCollectBox);
  }
  EXPECT_EQ(box_of(steps[17]), 5);
  EXPECT_NEAR(steps[3].pose.x, a.boxes[1].stop.x, 1e-9);   // 두 번째 = B2 정차
  std::string err;
  EXPECT_TRUE(collect_steps(a, {1, 1}, &err).empty());
  EXPECT_TRUE(collect_steps(a, {7}, &err).empty());
  EXPECT_TRUE(collect_steps(a, {}, &err).empty());
}

// 옆 보기: 박스 Plan 에서 퍽이 side_look_expected 보다 적으면 LOOK right → 기다리기 → LOOK left → 같은 Plan
TEST(DiceSteps, SideLookWhenPucksMissing)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  const Step & front = steps[4];   // arm:LOOK:B2
  const Step & plan = steps[5];    // plan:collect:B2
  ASSERT_TRUE(is_custom(plan, kPlanCollectBox));
  EXPECT_FALSE(is_side_look(front));   // 앞 LOOK 은 slot "" 그대로
  EXPECT_TRUE(front.slot.empty());

  const SideLookConfig cfg;   // 기본 = 켬, 2 개, right → left
  const auto more = side_look_steps(plan, 1, cfg);
  ASSERT_EQ(more.size(), 4u);
  EXPECT_EQ(more[0].type, StepType::Arm);
  EXPECT_EQ(more[0].arm_task, kArmLook);
  EXPECT_EQ(more[0].slot, "right");   // control_master / robot_cli 와 같은 글자
  EXPECT_EQ(box_of(more[0]), 2);
  EXPECT_EQ(place_of(more[0]), -1);
  EXPECT_EQ(more[0].name, "arm:LOOK:B2:right");
  EXPECT_TRUE(is_custom(more[1], kLookDwell));
  EXPECT_EQ(box_of(more[1]), 2);
  EXPECT_EQ(more[2].arm_task, kArmLook);
  EXPECT_EQ(more[2].slot, "left");
  EXPECT_TRUE(is_custom(more[3], kPlanCollectBox));   // 같은 Plan 다시
  EXPECT_EQ(box_of(more[3]), 2);
  EXPECT_EQ(more[3].name, "plan:collect:B2:side");
  for (const auto & s : more) {
    EXPECT_NE(s.type, StepType::Navigate);   // 같은 정차 안 (dice_recover 의 정차 구간이 그대로)
    if (s.type == StepType::Arm) {
      EXPECT_TRUE(is_side_look(s));
      EXPECT_TRUE(is_look_task(s.arm_task));   // 실행기: LOOK 류 = 펴진 채 → 다음 이동 전 HOME
      EXPECT_TRUE(s.puck_id.empty());          // 다시 보기 LOOK 아님
    }
  }
  EXPECT_EQ(side_look_steps(plan, 0, cfg).size(), 4u);   // 하나도 못 봄

  // 2 개 이상 보면 아무것도 안 끼움
  EXPECT_TRUE(side_look_steps(plan, 2, cfg).empty());
  EXPECT_TRUE(side_look_steps(plan, 3, cfg).empty());
  // 끄면 안 함
  SideLookConfig off = cfg;
  off.enabled = false;
  EXPECT_TRUE(side_look_steps(plan, 0, off).empty());
  // 기대 수 바꿈
  SideLookConfig three = cfg;
  three.expected = 3;
  EXPECT_EQ(side_look_steps(plan, 2, three).size(), 4u);
  SideLookConfig zero = cfg;
  zero.expected = 0;
  EXPECT_TRUE(side_look_steps(plan, 0, zero).empty());

  // 마지막 박스 = 배달 Plan 도 같은 Plan 다시
  const auto last = side_look_steps(steps[17], 1, cfg);
  ASSERT_EQ(last.size(), 4u);
  EXPECT_TRUE(is_custom(last.back(), kPlanDelivery));
  EXPECT_EQ(box_of(last.back()), 5);
  EXPECT_EQ(last[0].name, "arm:LOOK:B5:right");

  // 순서 · 글자: right · left 만 (rack = 적재함 스캔 경로라 절대 안 보냄), 한쪽만이면 기다리기 없음
  SideLookConfig one = cfg;
  one.order = {"left"};
  const auto l = side_look_steps(plan, 1, one);
  ASSERT_EQ(l.size(), 2u);
  EXPECT_EQ(l[0].slot, "left");
  EXPECT_TRUE(is_custom(l[1], kPlanCollectBox));
  SideLookConfig bad = cfg;
  bad.order = {"rack", "Right", ""};
  EXPECT_TRUE(side_look_steps(plan, 0, bad).empty());
  SideLookConfig mixed = cfg;
  mixed.order = {"left", "rack", "right"};
  const auto m = side_look_steps(plan, 0, mixed);
  ASSERT_EQ(m.size(), 4u);
  EXPECT_EQ(m[0].slot, "left");
  EXPECT_EQ(m[2].slot, "right");

  // 박스 Plan 이 아니면 안 함 (LOOK · 재검증 · 지금 자리 재계획)
  EXPECT_TRUE(side_look_steps(front, 0, cfg).empty());
  Step here = steps[17];
  box_of(here) = -1;
  EXPECT_TRUE(side_look_steps(here, 0, cfg).empty());
  const auto ver = verify_steps({{0, 1, 2}}, a);
  EXPECT_TRUE(side_look_steps(ver[2], 0, cfg).empty());
}

// PICK 빈손 (EMPTY_GRASP) 뒤 같은 박스 Plan 다시 앞에 끼울 앞 LOOK: 이동 전까지 뒤로 찾음, 옆 보기 · 다른 박스 LOOK 은 안 씀
TEST(DiceSteps, BoxRelookAfterEmptyGrasp)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  // B2 정차: [3] 이동 → [4] LOOK:B2 → [5] Plan → 옆 보기 LOOK right · 기다리기 · LOOK left → PICK
  std::vector<Step> st(steps.begin(), steps.begin() + 6);
  ASSERT_EQ(st[3].type, StepType::Navigate);
  const auto side = side_look_steps(st[5], 0, SideLookConfig{});
  ASSERT_EQ(side.size(), 4u);
  st.insert(st.end(), side.begin(), side.end() - 1);
  Step pick;
  pick.type = StepType::Arm;
  pick.arm_task = kArmPick;
  pick.puck_id = "P7";
  st.push_back(pick);
  const std::size_t at = st.size() - 1;

  const auto lk = box_relook_step(st, at, 2, "P7");
  ASSERT_TRUE(lk.has_value());
  EXPECT_EQ(lk->type, StepType::Arm);
  EXPECT_EQ(lk->arm_task, kArmLook);
  EXPECT_EQ(box_of(*lk), 2);
  EXPECT_EQ(place_of(*lk), -1);
  EXPECT_TRUE(lk->slot.empty());          // 앞 LOOK (옆 보기 아님)
  EXPECT_FALSE(is_side_look(*lk));
  EXPECT_EQ(lk->puck_id, "P7");           // 다시 보기 LOOK → 미션이 옆 보기 상태를 안 되돌림
  EXPECT_EQ(lk->name, "arm:LOOK:B2(빈손 뒤 다시 보기:P7)");

  EXPECT_FALSE(box_relook_step(st, at, 1, "P7").has_value());   // B1 LOOK 은 이동 너머 (앞 정차)
  EXPECT_FALSE(box_relook_step(st, 4, 2, "P7").has_value());    // 앞 LOOK 보다 앞 = 이동뿐
  EXPECT_FALSE(box_relook_step(st, at, 0, "P7").has_value());   // 박스 번호 잘못
  EXPECT_FALSE(box_relook_step(st, at, 7, "P7").has_value());
  EXPECT_EQ(box_relook_step(st, at, 2, "")->puck_id, "?");      // ID 없어도 다시 보기 LOOK
  EXPECT_EQ(box_relook_step(st, st.size() + 5, 2, "P7")->name, lk->name);   // at 이 목록 밖이어도 끝에서 찾음

  // 실행기 다시 보기 LOOK (이름 · puck_id 붙음) 이 앞에 있어도 같은 앞 LOOK 자세 · 새 이름
  Step prev = st[4];
  prev.puck_id = "P3";
  prev.name += "(다시 보기:P3)";
  st.insert(st.begin() + static_cast<std::ptrdiff_t>(at), prev);
  const auto lk2 = box_relook_step(st, at + 1, 2, "P7");
  ASSERT_TRUE(lk2.has_value());
  EXPECT_EQ(lk2->puck_id, "P7");
  EXPECT_EQ(lk2->name, "arm:LOOK:B2(빈손 뒤 다시 보기:P7)");
}

// 관측 구간 (look_started_) · 옆 보기 상태를 새로 잡는 LOOK = 새 정차 앞 LOOK 만
//   빈손 뒤 다시 보기 · 실행기 다시 보기 · 옆 보기 LOOK 은 그대로 둠 → 옆에서만 본 퍽이 Plan 다시에 남음
TEST(DiceSteps, RelookKeepsBoxLookWindow)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  std::vector<Step> st(steps.begin(), steps.begin() + 6);
  ASSERT_EQ(st[4].arm_task, kArmLook);
  ASSERT_EQ(box_of(st[4]), 2);
  EXPECT_TRUE(starts_box_look(st[4]));            // 새 정차 앞 LOOK
  for (const auto & s : side_look_steps(st[5], 0, SideLookConfig{})) {
    if (is_side_look(s)) {EXPECT_FALSE(starts_box_look(s));}   // 옆 보기 LOOK
  }
  const auto lk = box_relook_step(st, st.size(), 2, "P7");
  ASSERT_TRUE(lk.has_value());
  EXPECT_FALSE(starts_box_look(*lk));             // 빈손 뒤 다시 보기 LOOK
  const auto lk_noid = box_relook_step(st, st.size(), 2, "");
  ASSERT_TRUE(lk_noid.has_value());
  EXPECT_FALSE(starts_box_look(*lk_noid));        // ID 없어도 ("?") 그대로
  Step ex = st[4];
  ex.puck_id = "P3";                              // 실행기 다시 보기 LOOK (insert_relook)
  EXPECT_FALSE(starts_box_look(ex));
  EXPECT_FALSE(starts_box_look(st[5]));           // Plan (팔 아님)
  EXPECT_FALSE(starts_box_look(st[3]));           // 이동
  Step bad = st[4];
  box_of(bad) = 0;                                // 박스 번호 없음
  EXPECT_FALSE(starts_box_look(bad));
}

// 옆 보기 slot near · far (control_master LOOK slot: J3 +look_near_tilt_deg · J4 −look_far_tilt_deg) 도 받음
//   관측 구간을 안 바꾸고 (starts_box_look 거짓), 빈손 뒤 다시 보기는 그 너머 앞 LOOK 을 찾음
TEST(DiceSteps, SideLookAcceptsNearFar)
{
  for (const char * ok : {"right", "left", "near", "far"}) {EXPECT_TRUE(is_side_slot(ok)) << ok;}
  for (const char * no : {"", "rack", "Near", "front", "barcode"}) {EXPECT_FALSE(is_side_slot(no)) << no;}

  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  SideLookConfig cfg;
  cfg.order = {"left", "near"};   // 노드 기본 side_look_order
  const auto more = side_look_steps(steps[5], 1, cfg);
  ASSERT_EQ(more.size(), 4u);
  EXPECT_EQ(more[0].slot, "left");
  EXPECT_TRUE(is_custom(more[1], kLookDwell));
  EXPECT_EQ(more[2].slot, "near");
  EXPECT_EQ(more[2].name, "arm:LOOK:B2:near");
  EXPECT_TRUE(is_side_look(more[2]));
  EXPECT_FALSE(starts_box_look(more[2]));   // 관측 구간 · 옆 보기 상태 그대로
  EXPECT_FALSE(is_pick_relook(more[2]));
  EXPECT_TRUE(is_custom(more[3], kPlanCollectBox));
  cfg.order = {"far"};
  const auto far = side_look_steps(steps[5], 0, cfg);
  ASSERT_EQ(far.size(), 2u);
  EXPECT_EQ(far[0].slot, "far");
  EXPECT_TRUE(is_side_look(far[0]));

  // 빈손 뒤 다시 보기: near 옆 보기를 건너뛰고 그 정차 앞 LOOK
  std::vector<Step> st(steps.begin(), steps.begin() + 6);
  st.insert(st.end(), more.begin(), more.end() - 1);
  const auto lk = box_relook_step(st, st.size(), 2, "P7");
  ASSERT_TRUE(lk.has_value());
  EXPECT_TRUE(lk->slot.empty());
}

// LOOK 자세 구간 (LOOK 성공 ~ 다음 팔 작업) 안에서 찍힌 퍽 → 본 slot. 구간 밖 (팔이 움직이는 중) · ID 없는 관측은 안 씀
TEST(DiceSteps, PuckViewsFromLookWindows)
{
  const std::vector<LookWindow> w{{"", 10.0, 12.0}, {"left", 13.0, 15.0}, {"near", 16.0, -1.0}};
  const std::vector<SeenAt> seen{
    {"P1", 10.5}, {"P1", 11.0}, {"P1", 13.5},   // 앞 · left
    {"P2", 12.5},                               // 움직이는 중 → 안 씀
    {"P2", 14.0},                               // left 만
    {"P3", 20.0},                               // near (아직 그 자세)
    {"", 11.0},                                 // ID 없음
    {"P4", 9.0}};                               // 앞 LOOK 전
  const auto v = puck_views(w, seen);
  ASSERT_EQ(v.count("P1"), 1u);
  EXPECT_EQ(v.at("P1"), (std::vector<std::string>{"", "left"}));
  EXPECT_EQ(v.at("P2"), (std::vector<std::string>{"left"}));
  EXPECT_EQ(v.at("P3"), (std::vector<std::string>{"near"}));
  EXPECT_EQ(v.count("P4"), 0u);
  EXPECT_EQ(v.count(""), 0u);
  EXPECT_EQ(relook_slot(v, "P1"), "");       // 앞에서 봄 → 앞 LOOK
  EXPECT_EQ(relook_slot(v, "P2"), "left");   // 옆에서만 → 그 slot
  EXPECT_EQ(relook_slot(v, "P3"), "near");
  EXPECT_EQ(relook_slot(v, "P9"), "");       // 기록 없음 → 앞 LOOK
}

namespace
{
// B2 정차 앞 LOOK 뒤 Plan 이 끼울 집기 단계 (P1 → STORE → P2 → STORE)
std::vector<Step> b2_pick_steps(bool rack_look = false)
{
  std::vector<ArmAction> acts;
  for (const auto & [id, slot] : std::vector<std::pair<std::string, std::string>>{{"P1", "S1"}, {"P2", "S2"}}) {
    ArmAction p;
    p.op = ArmOp::Pick;
    p.puck_id = id;
    p.box = 2;
    acts.push_back(p);
    ArmAction s;
    s.op = ArmOp::Store;
    s.puck_id = id;
    s.slot = slot;
    acts.push_back(s);
  }
  return arm_steps(acts, rack_look, false);
}

bool is_pick_of(const Step & s, const std::string & id)
{
  return s.type == StepType::Arm && s.arm_task == kArmPick && s.puck_id == id;
}
}  // namespace

// PICK 앞 다시 보기: 박스 LOOK 바로 뒤 첫 퍽은 그대로, STORE 뒤 (P0) 둘째 퍽은 앞 LOOK 다시 → 기다리기 → PICK
TEST(DiceSteps, PickRelookAfterStore)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  const Step & front = steps[4];   // arm:LOOK:B2
  ASSERT_TRUE(starts_box_look(front));
  const PuckViews views{{"P1", {""}}, {"P2", {""}}};

  const auto out = with_pick_relooks(b2_pick_steps(), front, views);
  ASSERT_EQ(out.size(), 6u);
  EXPECT_TRUE(is_pick_of(out[0], "P1"));      // 첫 퍽: 앞 LOOK 자세에서 바로
  EXPECT_EQ(out[1].arm_task, kArmStore);
  const Step & lk = out[2];                   // 둘째 퍽: STORE 뒤 → 다시 보기
  EXPECT_EQ(lk.type, StepType::Arm);
  EXPECT_EQ(lk.arm_task, kArmLook);
  EXPECT_EQ(box_of(lk), 2);
  EXPECT_EQ(place_of(lk), -1);
  EXPECT_TRUE(lk.slot.empty());               // 앞에서 본 퍽 → 앞 LOOK
  EXPECT_TRUE(lk.puck_id.empty());            // 실행기 다시 보기 기다림 (relook_wait_s 거절) 안 씀
  EXPECT_EQ(lk.target, Target::None);
  EXPECT_TRUE(is_pick_relook(lk));
  EXPECT_FALSE(is_side_look(lk));
  EXPECT_FALSE(starts_box_look(lk));          // 관측 구간 look_started_ · 옆 보기 상태 그대로
  EXPECT_EQ(lk.name, "arm:LOOK:B2(집기 전 다시 보기:P2)");
  Step copied = lk;                           // 실행기 insert_relook 복사본 (puck_id 붙음) → 보통 다시 보기 LOOK 처리
  copied.puck_id = "P2";
  EXPECT_FALSE(is_pick_relook(copied));
  EXPECT_FALSE(starts_box_look(copied));
  EXPECT_TRUE(is_custom(out[3], kRelookDwell));
  EXPECT_EQ(box_of(out[3]), 2);
  EXPECT_EQ(out[3].puck_id, "P2");
  EXPECT_TRUE(is_pick_of(out[4], "P2"));
  EXPECT_EQ(out[5].arm_task, kArmStore);
  for (const auto & s : out) {EXPECT_NE(s.type, StepType::Navigate);}   // 같은 정차 안

  // LOOK_RACK 이 STORE 앞에 있어도 같음 (PICK 바로 앞 팔 단계 = STORE)
  const auto rk = with_pick_relooks(b2_pick_steps(true), front, views);
  ASSERT_EQ(rk.size(), 8u);
  EXPECT_TRUE(is_pick_of(rk[0], "P1"));
  EXPECT_EQ(rk[1].arm_task, kArmLookRack);
  EXPECT_TRUE(is_pick_relook(rk[3]));
  EXPECT_TRUE(is_custom(rk[4], kRelookDwell));
  EXPECT_TRUE(is_pick_of(rk[5], "P2"));

  // 기록 없는 퍽 = 앞에서 본 것으로 봄 (같은 결과)
  EXPECT_EQ(with_pick_relooks(b2_pick_steps(), front, {}).size(), 6u);
  // 앞 자세를 모름 (HOME 뒤 · 이동 뒤) → 첫 퍽도 다시 보기
  const auto cold = with_pick_relooks(b2_pick_steps(), std::nullopt, views);
  ASSERT_EQ(cold.size(), 8u);
  EXPECT_TRUE(is_pick_relook(cold[0]));
  Step home;
  home.type = StepType::Arm;
  home.arm_task = kArmHome;
  EXPECT_TRUE(is_pick_relook(with_pick_relooks(b2_pick_steps(), home, views)[0]));
  // 다른 박스 LOOK 자세 → 다시 보기
  Step other = front;
  box_of(other) = 3;
  EXPECT_TRUE(is_pick_relook(with_pick_relooks(b2_pick_steps(), other, views)[0]));
  // 실행기 · 빈손 뒤 다시 보기 LOOK (puck_id 있음) 도 같은 앞 LOOK 자세
  const auto again = box_relook_step(std::vector<Step>(steps.begin(), steps.begin() + 6), 6, 2, "P9");
  ASSERT_TRUE(again.has_value());
  EXPECT_TRUE(is_pick_of(with_pick_relooks(b2_pick_steps(), *again, views)[0], "P1"));
}

// 옆 보기로만 본 퍽은 그 slot 으로 다시 봄. 마지막 옆 보기 자세가 그 퍽을 봤으면 첫 퍽은 그대로
TEST(DiceSteps, PickRelookSideOnlyPuck)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  SideLookConfig cfg;
  cfg.order = {"left", "near"};
  const auto side = side_look_steps(steps[5], 1, cfg);
  ASSERT_EQ(side.size(), 4u);
  const Step & near = side[2];   // 마지막 옆 보기 = 지금 팔 자세
  ASSERT_EQ(near.slot, "near");
  const Step & front = steps[4];

  // P1 near 에서만, P2 앞 · left
  const PuckViews views{{"P1", {"near"}}, {"P2", {"", "left"}}};
  const auto out = with_pick_relooks(b2_pick_steps(), near, views);
  ASSERT_EQ(out.size(), 6u);
  EXPECT_TRUE(is_pick_of(out[0], "P1"));      // near 자세가 P1 을 봄 → 그대로
  EXPECT_TRUE(is_pick_relook(out[2]));
  EXPECT_TRUE(out[2].slot.empty());           // P2 는 앞에서도 봄 → 앞 LOOK
  EXPECT_TRUE(is_pick_of(out[4], "P2"));

  // 앞 LOOK 자세에서 near 에서만 본 P1 → near 로 다시 보기 (옆 보기 LOOK 이지만 관측 구간 그대로)
  const auto f = with_pick_relooks(b2_pick_steps(), front, views);
  ASSERT_EQ(f.size(), 8u);
  const Step & lk = f[0];
  EXPECT_TRUE(is_pick_relook(lk));
  EXPECT_EQ(lk.slot, "near");
  EXPECT_TRUE(is_side_look(lk));
  EXPECT_FALSE(starts_box_look(lk));
  EXPECT_EQ(lk.name, "arm:LOOK:B2:near(집기 전 다시 보기:P1)");
  EXPECT_TRUE(is_custom(f[1], kRelookDwell));
  EXPECT_EQ(f[1].puck_id, "P1");
  EXPECT_TRUE(is_pick_of(f[2], "P1"));
  EXPECT_TRUE(is_pick_relook(f[4]));
  EXPECT_TRUE(f[4].slot.empty());

  // 마지막 옆 보기 (near) 가 앞에서만 본 퍽 → 앞 LOOK 으로 다시 보기 (그 자세에선 안 보였을 수 있음)
  const PuckViews front_only{{"P1", {""}}, {"P2", {""}}};
  const auto n = with_pick_relooks(b2_pick_steps(), near, front_only);
  ASSERT_EQ(n.size(), 8u);
  EXPECT_TRUE(is_pick_relook(n[0]));
  EXPECT_TRUE(n[0].slot.empty());

  // PICK 앞 다시 보기 LOOK 을 빈손 뒤 다시 보기가 복사해도 표시는 안 물려받음
  std::vector<Step> st(steps.begin(), steps.begin() + 6);
  Step rl = f[4];   // 앞 슬롯 다시 보기
  st.push_back(rl);
  const auto eg = box_relook_step(st, st.size(), 2, "P2");
  ASSERT_TRUE(eg.has_value());
  EXPECT_FALSE(is_pick_relook(*eg));
  EXPECT_EQ(eg->puck_id, "P2");
}

// 배달 단계: 이동 → 박스 LOOK 뒤 첫 퍽은 그대로, 이어 하는 마지막 박스는 지금 팔 자세부터
TEST(DiceSteps, PickRelookTransferSteps)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  TransferPlan plan;
  Stop here;   // 마지막 박스 B5 에서 이어서 (이동 없음)
  here.node = 5;
  here.move = false;
  ArmAction p;
  p.op = ArmOp::Pick;
  p.box = 5;
  p.puck_id = "P1";
  ArmAction st;
  st.op = ArmOp::Store;
  st.slot = "S1";
  st.puck_id = "P1";
  here.actions = {p, st};
  Stop b3;   // B3 로 가서 LOOK → 두 개
  b3.node = 3;
  b3.move = true;
  ArmAction look;
  look.op = ArmOp::Look;
  look.box = 3;
  ArmAction p2 = p, p3 = p, s2 = st, s3 = st;
  p2.box = p3.box = 3;
  p2.puck_id = s2.puck_id = "P2";
  p3.puck_id = s3.puck_id = "P3";
  s2.slot = "S2";
  s3.slot = "S3";
  b3.actions = {look, p2, s2, p3, s3};
  plan.stops = {here, b3};
  const auto steps = transfer_steps(plan, a);
  ASSERT_EQ(steps.size(), 8u);

  const PuckViews views{{"P1", {""}}, {"P2", {""}}, {"P3", {""}}};
  Step b5_look;   // 마지막 박스 앞 LOOK (COLLECT 목록의 지금 팔 자세)
  b5_look.type = StepType::Arm;
  b5_look.arm_task = kArmLook;
  box_of(b5_look) = 5;
  const auto out = with_pick_relooks(steps, b5_look, views);
  // [PICK P1, STORE] [이동 B3, LOOK B3, PICK P2, STORE, 다시 보기 LOOK, 기다리기, PICK P3, STORE]
  ASSERT_EQ(out.size(), 10u);
  EXPECT_TRUE(is_pick_of(out[0], "P1"));
  EXPECT_EQ(out[2].type, StepType::Navigate);
  EXPECT_TRUE(starts_box_look(out[3]));       // 그 정차 LOOK (새 관측 구간)
  EXPECT_TRUE(is_pick_of(out[4], "P2"));
  EXPECT_TRUE(is_pick_relook(out[6]));
  EXPECT_EQ(box_of(out[6]), 3);
  EXPECT_TRUE(is_custom(out[7], kRelookDwell));
  EXPECT_TRUE(is_pick_of(out[8], "P3"));
  int relooks = 0;
  for (const auto & s : out) {relooks += is_pick_relook(s) ? 1 : 0;}
  EXPECT_EQ(relooks, 1);

  // 지금 자리에서 다시 계획 (서 있지 않음) → 첫 퍽도 다시 보기
  EXPECT_TRUE(is_pick_relook(with_pick_relooks(steps, std::nullopt, views)[0]));
}

// 정차마다 한 번: 첫 Plan 이 옆 보기를 끼우면 두 번째 (합친) Plan 은 모자라도 그대로 계획, 다시 안 끼움
TEST(DiceSteps, SideLookOncePerStop)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto steps = collect_steps(a, {1, 2, 6, 3, 4, 5});
  const Step & plan = steps[5];
  const SideLookConfig cfg;

  SideLookState st = SideLookState::Unchecked;
  const auto first = side_look_on_plan(st, plan, 1, cfg);
  EXPECT_EQ(first.size(), 4u);
  EXPECT_EQ(st, SideLookState::Pending);
  EXPECT_TRUE(side_look_on_plan(st, first.back(), 1, cfg).empty());   // 옆에서도 못 봄 → 본 만큼 계획
  EXPECT_EQ(st, SideLookState::Done);
  EXPECT_TRUE(side_look_on_plan(st, plan, 0, cfg).empty());   // 복구 Plan (집기 실패 뒤 등) 도 다시 안 함
  EXPECT_EQ(st, SideLookState::Done);

  // 처음부터 2 개 보이면 바로 Done (옆 보기 없이 계획)
  st = SideLookState::Unchecked;
  EXPECT_TRUE(side_look_on_plan(st, plan, 2, cfg).empty());
  EXPECT_EQ(st, SideLookState::Done);
  EXPECT_TRUE(side_look_on_plan(st, plan, 1, cfg).empty());   // 그 뒤 복구 Plan 에서 모자라도 안 함 (실패 처리 그대로)

  // 꺼짐: 언제나 바로 계획
  SideLookConfig off = cfg;
  off.enabled = false;
  st = SideLookState::Unchecked;
  EXPECT_TRUE(side_look_on_plan(st, plan, 0, off).empty());
  EXPECT_EQ(st, SideLookState::Done);
}

// 실행기가 Plan 성공 뒤 끼우는 모양 그대로: 옆 보기는 그 정차 안, 다음 이동 전에 끝남
TEST(DiceSteps, SideLookStaysInsideStop)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  auto steps = collect_steps(a, {1, 2, 6});
  const auto more = side_look_steps(steps[5], 1, SideLookConfig{});
  steps.insert(steps.begin() + 6, more.begin(), more.end());   // insert_after_current = Plan 바로 뒤
  std::vector<std::string> names;
  for (const auto & s : steps) {names.push_back(s.name);}
  const std::vector<std::string> want{
    "nav:B1", "arm:LOOK:B1", "plan:collect:B1",
    "nav:B2", "arm:LOOK:B2", "plan:collect:B2",
    "arm:LOOK:B2:right", "wait:B2:right", "arm:LOOK:B2:left", "plan:collect:B2:side",
    "nav:B6", "arm:LOOK:B6", "plan:delivery:B6"};
  EXPECT_EQ(names, want);
}

TEST(DiceSteps, Q1CollectThenDeliverSteps)
{
  // 노드와 같은 순서로 순수 계산만: 박스 5 곳 수집 (collect_box_actions → Cargo 반영) → 마지막 박스에서 계획 → 단계
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  TransferCosts costs;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));
  const auto all = q1_pucks();
  const std::vector<int> order{1, 2, 6, 3, 4, 5};
  Cargo cargo(slot_names(12));
  auto at_box = [&all](int b) {
      std::vector<TransferPuck> v;
      for (const auto & p : all) {if (p.box == b) {v.push_back(p);}}
      return v;
    };
  for (size_t i = 0; i + 1 < order.size(); ++i) {
    std::string err;
    const auto acts = collect_box_actions(at_box(order[i]), cargo, &err);
    ASSERT_FALSE(acts.empty()) << err;
    const auto st = arm_steps(acts);
    ASSERT_EQ(st.size(), acts.size());
    for (const auto & a2 : acts) {   // 팔이 성공했다고 보고 기록 반영
      const auto & p = *std::find_if(all.begin(), all.end(), [&](const TransferPuck & q) {return q.id == a2.puck_id;});
      if (a2.op == ArmOp::Pick) {ASSERT_TRUE(cargo.on_pick({p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));}
      if (a2.op == ArmOp::Store) {ASSERT_TRUE(cargo.on_store(a2.slot));}
    }
  }
  EXPECT_FALSE(cargo.holding());   // 박스마다 다 넣음
  EXPECT_EQ(cargo.stored_count(), 10u);

  // 마지막 박스 (B5): 노드 plan_delivery 와 같은 계산
  SolverInput in;
  in.position = 5;
  in.stopped = true;
  in.on_boxes = at_box(5);
  in.cargo_capacity = 12;
  ASSERT_TRUE(fill_carried(cargo, in));
  const SignalCounts counts{4, 2, 6};   // RGRRGRGGYGYG
  std::string err;
  auto dp = dice_layout(counts, DiceOrder::Ascending, DiceConfig{}, &err);
  ASSERT_TRUE(dp.has_value()) << err;
  apply_view_directions(*dp, place_view_dirs(a));
  for (int i = 0; i < kNumPlaces; ++i) {
    in.place_group[i] = dp->places[i].group;
    in.slots_left[i] = dp->places[i].pips;
  }
  const auto tp = plan_transfer(in, costs, cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(tp.has_value()) << err;
  const auto steps = transfer_steps(*tp, a);
  int places = 0, navs = 0;
  std::set<std::pair<int, int>> cells;
  for (const auto & st : steps) {
    if (st.type == StepType::Navigate) {++navs;}
    if (st.type == StepType::Arm && st.arm_task == kArmPlace) {
      ++places;
      EXPECT_TRUE(is_dice_cell(st));
      EXPECT_TRUE(cells.insert({place_of(st), cell_of(st)}).second) << "같은 칸 두 번";
      const auto pt = dice_cell_point(*dp, a, place_of(st), cell_of(st));
      ASSERT_TRUE(pt.has_value());
      // 칸은 장소 중심에서 격자 대각 안 (60 mm 간격 3×3 → 최대 0.085 m)
      EXPECT_LT(std::hypot((*pt)[0] - a.places[place_of(st)].center[0], (*pt)[1] - a.places[place_of(st)].center[1]), 0.09);
    }
    if (st.type == StepType::Arm && (st.arm_task == kArmStore || st.arm_task == kArmRetrieve)) {
      EXPECT_FALSE(st.slot.empty());
    }
  }
  EXPECT_EQ(places, 12);
  EXPECT_EQ(navs, 3);   // 장소 3 곳 (첫 정차 = 마지막 박스에서 이어서)
  EXPECT_EQ(steps.front().type, StepType::Arm);   // 이동 없이 B5 퍽 집기부터
  const auto ver = verify_steps(tp->verify_order, a);
  EXPECT_EQ(ver.size(), 9u);   // 장소마다 이동 · LOOK · Verify
  for (size_t i = 0; i < 3; ++i) {   // 세 장소 모두 LOOK
    EXPECT_EQ(ver[3 * i + 1].arm_task, kArmLook) << "L" << place_of(ver[3 * i + 1]) + 1;
  }
  EXPECT_FALSE(dice_cell_point(*dp, a, 0, 99).has_value());
}

namespace
{
// Q1 계획 (RGRRGRGGYGYG, P2-01) + 계획 칸에 정확히 놓았다고 본 관측
struct Q1Verify
{
  DiceMap arena;
  DicePlan plan;
  std::array<std::vector<std::string>, 3> ids;   // 장소별 놓은 퍽 ID (칸 순서)
  std::array<std::vector<VerifyObs>, 3> obs;
};

Q1Verify q1_verify()
{
  Q1Verify v;
  load_dice_map(TEST_DICE_MAP, v.arena);
  v.plan = *dice_layout(SignalCounts{4, 2, 6}, DiceOrder::Ascending);
  apply_view_directions(v.plan, place_view_dirs(v.arena));
  // L1 = Y 2 (빨강 상자), L2 = R 4 (빨강), L3 = G 6 (초록)
  v.ids = {{{"2", "6"}, {"0", "4", "8", "10"}, {"1", "3", "5", "7", "9", "11"}}};
  for (int pl = 0; pl < 3; ++pl) {
    for (size_t k = 0; k < v.ids[pl].size(); ++k) {
      const auto & id = v.ids[pl][k];
      const auto p = q1_pucks()[std::stoi(id)];
      const auto pt = *dice_cell_point(v.plan, v.arena, pl, static_cast<int>(k));
      v.obs[pl].push_back({id, kind_color(p.kind), kind_shape(p.kind), pt[0] + 0.004, pt[1] - 0.003});
    }
  }
  return v;
}
}  // namespace

TEST(DiceVerify, AllCorrectPasses)
{
  auto v = q1_verify();
  std::vector<PlaceVerify> pv;
  for (int pl : {0, 2, 1}) {
    pv.push_back(judge_place(pl, v.plan, v.arena, v.obs[pl]));
    EXPECT_TRUE(pv.back().ok) << pl;
  }
  EXPECT_FALSE(summarize({pv[0]}).complete);   // 한 곳만 → 아직
  const auto r = summarize(pv);
  EXPECT_TRUE(r.complete);
  EXPECT_TRUE(r.pass);
  EXPECT_TRUE(r.missing_ids.empty());
  const auto y = verify_to_yaml(r, v.plan, v.arena, "P2-01", "RGRRGRGGYGYG");
  EXPECT_NE(y.find("status: PASS"), std::string::npos);
  EXPECT_NE(y.find("place: L3"), std::string::npos);
}

TEST(DiceVerify, OffCellWrongColorCountAndIds)
{
  auto v = q1_verify();
  // L2 (R 4): 한 개 모양 밖으로 14 cm 밀림 → 칸 밖 (허용 5 cm, 모양을 옮기고 돌려도 못 덮음)
  v.obs[1][0].x += 0.10;
  v.obs[1][0].y += 0.10;
  auto l2 = judge_place(1, v.plan, v.arena, v.obs[1]);
  EXPECT_FALSE(l2.ok);
  EXPECT_EQ(l2.seen[0].cell, -1);
  // L3 (G 6): 빨강 섞임
  v.obs[2][0].color = "red";
  auto l3 = judge_place(2, v.plan, v.arena, v.obs[2]);
  EXPECT_FALSE(l3.ok);
  // L1 (Y 2): 하나 못 봄 → 개수 틀림, 그 ID 누락
  const std::string gone = v.obs[0].back().id;
  v.obs[0].pop_back();
  auto l1 = judge_place(0, v.plan, v.arena, v.obs[0]);
  EXPECT_FALSE(l1.ok);
  // 같은 ID 를 두 곳에서 봄 → 중복
  auto l3dup = v.obs[2];
  l3dup[1].id = v.obs[1][1].id;
  const auto r = summarize({l1, l2, judge_place(2, v.plan, v.arena, l3dup)});
  EXPECT_TRUE(r.complete);
  EXPECT_FALSE(r.pass);
  EXPECT_NE(std::find(r.missing_ids.begin(), r.missing_ids.end(), gone), r.missing_ids.end());
  EXPECT_EQ(r.duplicate_ids.size(), 1u);
}

TEST(DiceVerify, YellowNeedsOneShape)
{
  auto v = q1_verify();
  ASSERT_EQ(v.plan.places[0].group, 'Y');
  v.obs[0][0].shape = v.obs[0][0].shape == "box" ? "cylinder" : "box";
  EXPECT_FALSE(judge_place(0, v.plan, v.arena, v.obs[0]).ok);
}

TEST(DiceVerify, YellowUnknownShapeFailsOnlyThatPuck)
{
  auto v = q1_verify();
  ASSERT_EQ(v.plan.places[0].group, 'Y');
  v.obs[0][0].shape = "";
  const auto pv = judge_place(0, v.plan, v.arena, v.obs[0]);
  EXPECT_FALSE(pv.ok);                 // 확인 못 한 퍽 있음 → 장소 불합격 그대로
  EXPECT_FALSE(pv.seen[0].ok);
  EXPECT_TRUE(pv.seen[1].ok);          // 모양 아는 퍽은 안 떨어짐
  ASSERT_EQ(pv.issues.size(), 1u);
  EXPECT_NE(pv.issues[0].find("모양 모름"), std::string::npos);
}

namespace
{
// 실주행 장면 (P2-02 내림차순, 신호 R6 Y5 G1 → L1 R6 · L2 Y5 · L3 G1). 박스 LOOK 로그의 퍽 (ID, 종류, 박스)
std::vector<TransferPuck> fallen_puck_pucks()
{
  const auto RC = PuckKind::RedCylinder, RB = PuckKind::RedBox, GC = PuckKind::GreenCylinder, GB = PuckKind::GreenBox;
  return {
    {"2", GC, 1}, {"4", RB, 1}, {"1", GC, 2}, {"8", RB, 2}, {"5", RC, 3}, {"3", RC, 3},
    {"7", RC, 4}, {"6", RB, 4}, {"11", GC, 5}, {"0", RC, 5}, {"10", RB, 6}, {"9", GB, 6}};
}

struct FallenPuckVerify
{
  DiceMap arena;
  DicePlan plan;
  VerifyBox box;
  std::vector<VerifyObs> placed;   // L2 에 놓은 순서 5 · 11 · 7 · 1 · 2 (모두 원기둥), 칸 위
};

FallenPuckVerify fallen_puck_verify()
{
  FallenPuckVerify v;
  load_dice_map(TEST_DICE_MAP, v.arena);
  v.plan = *dice_layout(SignalCounts{6, 5, 1}, DiceOrder::Descending);
  apply_view_directions(v.plan, place_view_dirs(v.arena));
  const auto & c = v.arena.places[1].center;
  v.box.x = c[0];
  v.box.y = c[1];
  v.box.yaw = v.arena.map_from_arena.yaw;
  v.box.source = "placed";
  const std::vector<std::pair<std::string, std::string>> put = {
    {"5", "red"}, {"11", "green"}, {"7", "red"}, {"1", "green"}, {"2", "green"}};
  for (size_t k = 0; k < put.size(); ++k) {
    const auto pt = *dice_cell_point(v.plan, v.arena, 1, static_cast<int>(k));
    v.placed.push_back({put[k].first, put[k].second, "cylinder", pt[0] + 0.004, pt[1] - 0.003});
  }
  return v;
}
}  // namespace

// 계획 쪽: 이 퍽 목록이면 Y 는 원기둥만 가능 (초록 상자 1 개뿐) → 계획 "Y 모양 cylinder" 가 발제대로, 색 섞임은 정상
TEST(DiceVerify, FallenPuckPlannerYellowIsCylinder)
{
  SolverInput in;
  in.place_group = {{'R', 'Y', 'G'}};
  in.slots_left = {{6, 5, 1}};
  in.on_boxes = fallen_puck_pucks();
  const auto sol = solve_transfer(in, test_costs());
  ASSERT_TRUE(sol.has_value());
  EXPECT_EQ(sol->yellow_shape, "cylinder");
  int y = 0, y_red = 0;
  for (const auto & s : sol->steps) {
    if (s.act != SolverAct::Place || s.place != 1) {continue;}
    ++y;
    EXPECT_FALSE(kind_is_box(s.kind));
    if (kind_is_red(s.kind)) {++y_red;}
  }
  EXPECT_EQ(y, 5);
  EXPECT_EQ(y_red, 2);   // 빨강 8 − R6 = 2, 초록 4 − G1 = 3 → Y 는 빨강 원기둥 2 + 초록 원기둥 3
  in.yellow_shape = "box";   // 상자로 고정하면 풀 수 없음
  EXPECT_FALSE(solve_transfer(in, test_costs()).has_value());
}

// 재검증 쪽: 다섯 원기둥 (빨강 2 · 초록 3) 이 칸 위면 L2 Y5 합격 (색 섞여도 됨)
TEST(DiceVerify, FallenPuckMixedColorCylindersPass)
{
  auto v = fallen_puck_verify();
  ASSERT_EQ(v.plan.places[1].group, 'Y');
  ASSERT_EQ(v.plan.places[1].pips, 5);
  const auto pv = judge_place(1, v.plan, v.arena, v.placed, 0.05, 0.0, {}, v.box);
  EXPECT_TRUE(pv.ok);
  for (const auto & i : pv.issues) {ADD_FAILURE() << i;}
}

// 재현: ID 1 · 2 · 7 은 칸 위 원기둥, ID 못 읽은 퍽 하나가 박스 가장자리 8 mm 밖 · 상자로 읽힘
//   떨어진 퍽까지 모양 기준에 넣으면 {cylinder, box} 두 가지 → 제자리 원기둥 셋까지 "그룹 Y 조건 안 맞음"
//   떨어진 퍽은 Y 기준 모양에 안 들어감 → 그 퍽만 불합격 (박스 밖 · 칸 밖 · ID), 개수 4 ≠ 5 → 장소는 FAIL 그대로
TEST(DiceVerify, FallenPuckDoesNotFailPlacedCylinders)
{
  auto v = fallen_puck_verify();
  std::vector<VerifyObs> obs = {v.placed[3], v.placed[4], v.placed[2]};   // 1 · 2 · 7
  // 박스 축 (u, w): 가장자리 u = half + 8 mm (떨어짐), w = 9 mm → 가장 가까운 칸 (u 0.06, w 0.06) 에서 72 mm (로그 값)
  const double ax = std::cos(v.box.yaw), ay = std::sin(v.box.yaw);
  const double u = v.box.half + 0.008, w = 0.009;
  VerifyObs fallen{"", "green", "box", v.box.x + u * ax - w * ay, v.box.y + u * ay + w * ax};
  obs.push_back(fallen);
  // 로그에서 모양 맞춤은 1 cm 남짓만 옮김 → 맞춤 없이 기준 칸으로 봄 (맞춤이 떨어진 퍽 쪽으로 끌려가는 경우는 이 시험 밖)
  VerifyFitParams no_fit;
  no_fit.enable = false;
  const auto pv = judge_place(1, v.plan, v.arena, obs, 0.05, 0.0, no_fit, v.box);
  EXPECT_FALSE(pv.ok);
  ASSERT_EQ(pv.seen.size(), 4u);
  for (const auto & o : pv.seen) {
    if (o.id.empty()) {
      EXPECT_FALSE(o.ok);
      EXPECT_EQ(o.on_box, 0);
      EXPECT_EQ(o.cell, -1);   // 로그: 칸 밖 (가장 가까운 칸 72 mm)
    } else {
      EXPECT_TRUE(o.ok) << "ID " << o.id;
    }
  }
  bool off = false, count = false;
  for (const auto & i : pv.issues) {
    EXPECT_EQ(i.find("그룹 Y"), std::string::npos) << i;
    off = off || i.find("박스 윗면 밖") != std::string::npos;
    count = count || i.find("퍽 4 개 보임") != std::string::npos;
  }
  EXPECT_TRUE(off);
  EXPECT_TRUE(count);
}

// 칸 위 퍽끼리 모양이 섞이면 소수 모양 퍽만 불합격 (기준 = 최다 모양)
TEST(DiceVerify, YellowMinorityShapeFailsOnlyThatPuck)
{
  auto v = fallen_puck_verify();
  auto obs = v.placed;
  obs[0].shape = "box";   // ID 5 를 상자로 놓았다고 침
  const auto pv = judge_place(1, v.plan, v.arena, obs, 0.05, 0.0, {}, v.box);
  EXPECT_FALSE(pv.ok);
  for (const auto & o : pv.seen) {EXPECT_EQ(o.ok, o.id != "5") << "ID " << o.id;}
  ASSERT_EQ(pv.issues.size(), 1u);
  EXPECT_NE(pv.issues[0].find("Y 모양 cylinder"), std::string::npos) << pv.issues[0];
}

TEST(DiceVerify, GroupObsVotesColorAndShape)
{
  auto o = [](const std::string & id, const std::string & c, const std::string & s, double x) {
      return VerifyObs{id, c, s, x, 0.0};
    };
  // 퍽 1: box 3 · "" 1 (마지막 장 빈 값) → box
  // 퍽 2: box 1 · cylinder 1 → 표 갈림 → ""
  // ID 없는 장 (퍽 1 에서 1 cm): 퍽 1 에 합침, 따로 안 셈
  // ID 없는 퍽 (0.2 m): 장 2 개 묶어 하나, 색 red 2 · green 1 → 0.67 < 0.7 → ""
  const std::vector<VerifyObs> raw = {
    o("1", "red", "box", 0.000), o("2", "red", "box", 0.100), o("1", "red", "box", 0.001),
    o("", "red", "box", 0.010), o("2", "red", "cylinder", 0.101), o("", "red", "", 0.200),
    o("", "green", "", 0.205), o("", "red", "", 0.203), o("1", "red", "", 0.002)};
  const auto g = group_place_obs(raw, 0.03, 0.7);
  ASSERT_EQ(g.size(), 3u);
  EXPECT_EQ(g[0].id, "1");
  EXPECT_EQ(g[0].shape, "box");
  EXPECT_EQ(g[0].color, "red");
  EXPECT_DOUBLE_EQ(g[0].x, 0.002);     // 위치 = 가장 최근 장
  EXPECT_EQ(g[1].id, "2");
  EXPECT_EQ(g[1].shape, "");
  EXPECT_EQ(g[1].color, "red");
  EXPECT_EQ(g[2].id, "");
  EXPECT_EQ(g[2].color, "");
  EXPECT_EQ(g[2].shape, "");
  EXPECT_DOUBLE_EQ(g[2].x, 0.203);
  // 기준을 낮추면 2 : 1 은 통과
  EXPECT_EQ(group_place_obs(raw, 0.03, 0.6)[2].color, "red");
}

TEST(DiceVerify, AnonObsGoToNearestPuck)
{
  // 퍽 6 (ID 읽힘, x 0) 에 퍽 0 (ID 못 읽음, x 0.04) 이 기대 섬. 둘 다 ID 없는 장이 섞여 옴
  //   처음 걸린 퍽에 붙이면 퍽 6 쪽 묶음이 퍽 0 으로 끌려가 퍽 0 이 둘 → 3 개
  auto o = [](const std::string & id, double x, double t) {
      VerifyObs v{id, "red", id.empty() && x > 0.03 ? "cylinder" : "box", x, 0.0};
      v.stamp = t;
      return v;
    };
  const std::vector<VerifyObs> raw = {
    o("6", 0.000, 1.0), o("", 0.000, 1.1), o("", 0.040, 1.1), o("", 0.022, 1.2), o("", 0.045, 1.3), o("6", 0.001, 1.4)};
  const auto g = group_place_obs(raw, 0.03, 0.7);
  ASSERT_EQ(g.size(), 2u);
  EXPECT_EQ(g[0].id, "6");
  EXPECT_EQ(g[1].id, "");
  EXPECT_DOUBLE_EQ(g[1].x, 0.045);
}

TEST(Replan, LeftoverAllowsPartialPlaces)
{
  // 초록 퍽 3 이 빠져 (못 집음) 초록 5 개 → G 6 은 못 채움. L1 Y · L2 R 만 채우고 초록 5 개는 남김
  auto all = q1_pucks();
  all.erase(all.begin() + 3);
  Cargo cargo(slot_names(12));
  for (const auto & p : all) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput in = q1_input();
  in.position = 5;
  in.stopped = true;
  ASSERT_TRUE(fill_carried(cargo, in));
  std::string err;
  EXPECT_FALSE(plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err).has_value());   // 세 곳 다는 불가
  in.slots_left = {{2, 4, 0}};
  EXPECT_FALSE(plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err).has_value());   // 남는 퍽 허용 안 하면 불가
  in.allow_leftover = true;
  const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  int placed[3] = {0, 0, 0};
  for (const auto & s : plan->stops) {
    for (const auto & a : s.actions) {if (a.op == ArmOp::Place) {++placed[a.place];}}
  }
  EXPECT_EQ(placed[0], 2);
  EXPECT_EQ(placed[1], 4);
  EXPECT_EQ(placed[2], 0);
}

TEST(Replan, LeftoverKeepsYellowFeasible)
{
  // 남는 퍽 허용이어도 Y 는 한 모양: 빨강 4 (모두 원기둥) + 초록 2 (상자 1 · 원기둥 1), R 2 · Y 2 (G 0)
  //   R 에 원기둥 2 를 쓰면 Y = 원기둥 2 (빨강) 로 됨
  std::vector<TransferPuck> p{
    {"0", PuckKind::RedCylinder, 1}, {"1", PuckKind::RedCylinder, 1}, {"2", PuckKind::RedCylinder, 2},
    {"3", PuckKind::RedCylinder, 2}, {"4", PuckKind::GreenBox, 3}, {"5", PuckKind::GreenCylinder, 3}};
  SolverInput in;
  in.on_boxes = p;
  in.place_group = {{'Y', 'R', 'G'}};
  in.slots_left = {{2, 2, 0}};
  in.allow_leftover = true;
  Cargo cargo(slot_names(12));
  std::string err;
  const auto sol = solve_transfer(in, test_costs(), &err);
  ASSERT_TRUE(sol.has_value()) << err;
  in.slots_left = {{3, 2, 0}};   // Y 3 = 한 모양 3 개 필요: 원기둥 (빨강 2 남음 + 초록 1) = 3 → 됨
  EXPECT_TRUE(solve_transfer(in, test_costs(), &err).has_value()) << err;
  in.slots_left = {{4, 2, 0}};   // Y 4: 원기둥 3 · 상자 1 → 안 됨
  EXPECT_FALSE(solve_transfer(in, test_costs(), &err).has_value());
}

// 퍽이 하나 모자람 (11 개, 빨강 원기둥 하나 잃음): L1 G2 · L2 Y4 · L3 R6 은 셋 다 못 채움, R6+Y4 도 안 됨 (Y 한 모양 4 개 모자람)
//   두 곳 조합 R6+G2 (8 퍽) · G2+Y4 (6 퍽) 중 기본 = 시간 짧은 쪽, alloc_prefer_more_pucks = 퍽 많은 쪽 (R6+G2)
TEST(Replan, ChooseDeliveryPrefersMorePucks)
{
  std::vector<TransferPuck> all;
  const PuckKind k[11] = {
    PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox,
    PuckKind::RedCylinder, PuckKind::RedCylinder, PuckKind::RedCylinder,
    PuckKind::GreenBox, PuckKind::GreenBox, PuckKind::GreenCylinder, PuckKind::GreenCylinder};
  for (int i = 0; i < 11; ++i) {all.push_back(TransferPuck{std::to_string(i), k[i], i / 2 + 1});}
  Cargo cargo(slot_names(12));
  for (const auto & p : all) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput base;
  base.position = 3;
  base.stopped = true;
  base.allow_leftover = true;
  base.place_group = {{'G', 'Y', 'R'}};
  ASSERT_TRUE(fill_carried(cargo, base));
  const std::array<int, kNumPlaces> need{{2, 4, 6}};
  TransferCosts costs;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));

  std::string err;
  const auto t = choose_delivery(base, need, 11, costs, cargo, {{0, 0, 0}}, 4, false, &err);
  ASSERT_TRUE(t.plan.has_value()) << err;
  EXPECT_EQ(t.n, 2);
  EXPECT_EQ(t.mask, 0b011);   // 기본: L1 G2 + L2 Y4 (6 퍽) 가 더 빠름
  EXPECT_EQ(t.pucks, 6);
  EXPECT_EQ(t.time_mask, t.mask);

  const auto m = choose_delivery(base, need, 11, costs, cargo, {{0, 0, 0}}, 4, true, &err);
  ASSERT_TRUE(m.plan.has_value()) << err;
  EXPECT_EQ(m.n, 2);
  EXPECT_EQ(m.mask, 0b101);   // 퍽 많은 쪽: L1 G2 + L3 R6 (8 퍽)
  EXPECT_EQ(m.pucks, 8);
  EXPECT_EQ(m.time_mask, 0b011);   // 시간만 보면 G2+Y4 (기록용)
  EXPECT_GT(m.plan->cost_s, t.plan->cost_s);
  SolverInput in = base;
  in.slots_left = {{2, 0, 6}};
  check_plan_rules(*m.plan, all, in);

  // L3 R6 이미 끝남 (need 0) → L3 든 조합은 뺌, 남은 두 곳을 다 채우면 (장소 수가 먼저) 둘 다 같은 답
  std::array<int, kNumPlaces> small{{2, 4, 0}};
  const auto a = choose_delivery(base, small, 11, costs, cargo, {{0, 0, 6}}, 4, false, &err);
  const auto b = choose_delivery(base, small, 11, costs, cargo, {{0, 0, 6}}, 4, true, &err);
  ASSERT_TRUE(a.plan.has_value() && b.plan.has_value()) << err;
  EXPECT_EQ(a.mask, 0b011);
  EXPECT_EQ(b.mask, 0b011);
  EXPECT_EQ(a.n, 2);

  // 채울 장소 없음 (퍽 1 개뿐) → plan 없음
  SolverInput few;
  few.position = 3;
  few.stopped = true;
  few.allow_leftover = true;
  few.place_group = base.place_group;
  few.on_boxes = {TransferPuck{"0", PuckKind::RedBox, 3}};
  Cargo empty(slot_names(12));
  const auto none = choose_delivery(few, need, 1, costs, empty, {{0, 0, 0}}, 4, true, &err);
  EXPECT_FALSE(none.plan.has_value());
}

// 부분 채우기 (fill_partial_places): 같은 11 개 (빨강 원기둥 하나 잃음) → 세 곳 다 놓고 놓는 퍽 11 개 (끝까지 채움 모드는 최대 8)
//   R6 + Y + G2 에서 Y 한 모양 · 빨강 공유 때문에 한 칸이 빔: (R6 Y3 G2) 또는 (R5 Y4 G2), 둘 다 다 채운 장소 2 곳 → 시간 짧은 쪽
// 칸에 둔 채 건너뛴 퍽 (RETRIEVE 빈손 반복 · 확인 못 함, mission rack_stuck_): 계획 입력에서 빼고 펼치기용 적재함 사본은 모르는 종류로
//   → 그 칸에서 꺼내지 않고 · 그 칸에 넣지도 않음, 남은 퍽으로 부분 채우기 그대로
TEST(Replan, StuckRackPuckIsNeverRetrieved)
{
  std::vector<TransferPuck> all;
  const PuckKind k[11] = {
    PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox,
    PuckKind::RedCylinder, PuckKind::RedCylinder, PuckKind::RedCylinder,
    PuckKind::GreenBox, PuckKind::GreenBox, PuckKind::GreenCylinder, PuckKind::GreenCylinder};
  for (int i = 0; i < 11; ++i) {all.push_back(TransferPuck{std::to_string(i), k[i], i / 2 + 1});}
  Cargo cargo(slot_names(12));
  for (const auto & p : all) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  // 퍽 3 (빨강 네모) 을 칸에 둔 채 건너뜀
  std::string stuck_slot;
  Cargo plan_cargo = cargo;
  for (const auto & [slot, it] : cargo.slots()) {
    if (it && it->puck_id == "3") {
      stuck_slot = slot;
      CargoItem x = *it;
      x.color = "stuck";
      ASSERT_TRUE(plan_cargo.replace_slot_item(slot, x));
    }
  }
  ASSERT_FALSE(stuck_slot.empty());
  EXPECT_FALSE(plan_cargo.replace_slot_item("S12", CargoItem{"x", "red", "box", "", false}));   // 빈 칸은 안 바꿈
  SolverInput base;
  base.position = 3;
  base.stopped = true;
  base.allow_leftover = true;
  base.place_group = {{'G', 'Y', 'R'}};
  ASSERT_TRUE(fill_carried(cargo, base));
  base.in_cargo.erase(std::remove_if(base.in_cargo.begin(), base.in_cargo.end(),
    [](const TransferPuck & p) {return p.id == "3";}), base.in_cargo.end());
  base.cargo_capacity -= 1;
  const std::array<int, kNumPlaces> need{{2, 4, 6}};
  TransferCosts costs;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));

  std::string err;
  const auto p = choose_delivery(base, need, 10, costs, plan_cargo, {{0, 0, 0}}, 4, true, &err, true);
  ASSERT_TRUE(p.plan.has_value()) << err;
  int places = 0;
  for (const auto & st : p.plan->stops) {
    for (const auto & a : st.actions) {
      places += a.op == ArmOp::Place ? 1 : 0;
      EXPECT_NE(a.puck_id, "3");
      if (a.op == ArmOp::Retrieve || a.op == ArmOp::Store) {EXPECT_NE(a.slot, stuck_slot) << a.puck_id;}
    }
  }
  EXPECT_EQ(places, 10);
}

TEST(Replan, ChooseDeliveryPartialFillsAllPlaces)
{
  std::vector<TransferPuck> all;
  const PuckKind k[11] = {
    PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox,
    PuckKind::RedCylinder, PuckKind::RedCylinder, PuckKind::RedCylinder,
    PuckKind::GreenBox, PuckKind::GreenBox, PuckKind::GreenCylinder, PuckKind::GreenCylinder};
  for (int i = 0; i < 11; ++i) {all.push_back(TransferPuck{std::to_string(i), k[i], i / 2 + 1});}
  Cargo cargo(slot_names(12));
  for (const auto & p : all) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{p.id, kind_color(p.kind), kind_shape(p.kind), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  SolverInput base;
  base.position = 3;
  base.stopped = true;
  base.allow_leftover = true;
  base.place_group = {{'G', 'Y', 'R'}};
  ASSERT_TRUE(fill_carried(cargo, base));
  const std::array<int, kNumPlaces> need{{2, 4, 6}};
  TransferCosts costs;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));

  std::string err;
  const auto p = choose_delivery(base, need, 11, costs, cargo, {{0, 0, 0}}, 4, true, &err, true);
  ASSERT_TRUE(p.plan.has_value()) << err;
  EXPECT_EQ(p.n, 3);
  EXPECT_EQ(p.mask, 0b111);
  EXPECT_EQ(p.pucks, 11);
  EXPECT_EQ(p.count[0], 2);   // G2 는 다 채움
  EXPECT_EQ(p.count[1] + p.count[2], 9);
  EXPECT_TRUE((p.count[1] == 3 && p.count[2] == 6) || (p.count[1] == 4 && p.count[2] == 5));
  SolverInput in = base;
  in.slots_left = p.count;
  check_plan_rules(*p.plan, all, in);
  int places = 0;
  for (const auto & st : p.plan->stops) {
    for (const auto & a : st.actions) {places += a.op == ArmOp::Place ? 1 : 0;}
  }
  EXPECT_EQ(places, 11);

  // 끝까지 채움 모드 (partial false) 는 그대로 두 곳 · 8 퍽
  const auto f = choose_delivery(base, need, 11, costs, cargo, {{0, 0, 0}}, 4, true, &err);
  ASSERT_TRUE(f.plan.has_value()) << err;
  EXPECT_EQ(f.pucks, 8);

  // 부분 채우기면 퍽 하나라도 놓을 곳이 있음 (끝까지 채움은 없음)
  SolverInput few;
  few.position = 3;
  few.stopped = true;
  few.allow_leftover = true;
  few.place_group = base.place_group;
  few.on_boxes = {TransferPuck{"0", PuckKind::RedBox, 3}};
  Cargo empty(slot_names(12));
  const auto one = choose_delivery(few, need, 1, costs, empty, {{0, 0, 0}}, 4, true, &err, true);
  ASSERT_TRUE(one.plan.has_value()) << err;
  EXPECT_EQ(one.pucks, 1);
  EXPECT_EQ(one.n, 1);
}

TEST(Replan, DiceLayoutWithoutPucks)
{
  const auto lay = dice_layout(SignalCounts{4, 2, 6}, DiceOrder::Ascending);
  ASSERT_TRUE(lay.has_value());
  EXPECT_EQ(lay->places[0].group, 'Y');
  EXPECT_EQ(lay->places[1].group, 'R');
  EXPECT_EQ(lay->places[2].pips, 6);
  EXPECT_EQ(lay->places[2].pip_offsets.size(), 6u);
  EXPECT_FALSE(dice_layout(SignalCounts{4, 2, 5}, DiceOrder::Ascending).has_value());   // 합 11
}

TEST(DiceRecord, AllocationAndSignalJson)
{
  auto v = q1_verify();
  // DELIVER 단계 대신 칸마다 PLACE 단계를 직접 만듦
  std::vector<Step> steps;
  for (int pl = 0; pl < 3; ++pl) {
    for (size_t k = 0; k < v.ids[pl].size(); ++k) {
      Step s;
      s.type = StepType::Arm;
      s.arm_task = kArmPlace;
      s.target = Target::Custom;
      s.kind = kTargetDiceCell;
      place_of(s) = pl;
      cell_of(s) = static_cast<int>(k);
      s.puck_id = v.ids[pl][k];
      steps.push_back(s);
    }
  }
  AllocationRecord r;
  r.barcode = "P2-01";
  r.signal_sequence = "RGRRGRGGYGYG";
  r.counts = {4, 2, 6};
  r.yellow_shape = "cylinder";
  r.filled = {true, true, false};
  r.cells = planned_cells(steps, v.plan, v.arena);
  ASSERT_EQ(r.cells.size(), 12u);
  r.pucks.push_back({"3", "green", "cylinder", "skipped:B2", ""});
  const auto j = allocation_to_json(r, v.plan, v.arena);
  EXPECT_NE(j.find("\"status\": \"partial\""), std::string::npos);
  EXPECT_NE(j.find("\"place\": \"L3\""), std::string::npos);
  EXPECT_NE(j.find("\"where\": \"skipped:B2\""), std::string::npos);
  // L1 중심 (3670, 950) mm 이 경기장 좌표로 나옴 (map_from_arena = start 를 되돌림)
  EXPECT_NE(j.find("\"center_arena_mm\": [3670, 950]"), std::string::npos) << j;

  SignalRecorder rec;
  double t = 0.0;
  for (char c : std::string("RGRRGRGGYGYG")) {
    const auto o = c == 'R' ? SignalObs::Red : (c == 'G' ? SignalObs::Green : SignalObs::Yellow);
    for (int i = 0; i < 10; ++i, t += 0.1) {rec.observe(t, o);}
    for (int i = 0; i < 10; ++i, t += 0.1) {rec.observe(t, SignalObs::Off);}
  }
  for (int i = 0; i < 30; ++i, t += 0.1) {rec.observe(t, SignalObs::Off);}
  const auto sj = signal_to_json(rec, 0.0, false);
  EXPECT_NE(sj.find("\"sequence\": \"RGRRGRGGYGYG\""), std::string::npos) << sj;
  EXPECT_NE(sj.find("\"status\": \"DONE\""), std::string::npos) << sj;
  EXPECT_NE(sj.find("\"n\": 12"), std::string::npos);
}

TEST(DiceSteps, RackLookBeforeStore)
{
  // rack_look: STORE 마다 바로 앞에 같은 칸 LOOK_RACK, 꺼 두면 지금과 같음
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  std::vector<TransferPuck> box1;
  for (const auto & p : q1_pucks()) {if (p.box == 1) {box1.push_back(p);}}
  Cargo cargo(slot_names(12));
  std::string err;
  const auto acts = collect_box_actions(box1, cargo, &err);
  ASSERT_FALSE(acts.empty()) << err;
  const int stores = static_cast<int>(std::count_if(acts.begin(), acts.end(), [](const ArmAction & x) {return x.op == ArmOp::Store;}));
  ASSERT_GT(stores, 0);
  EXPECT_EQ(arm_steps(acts).size(), acts.size());
  const auto st = arm_steps(acts, true);
  ASSERT_EQ(st.size(), acts.size() + static_cast<size_t>(stores));
  int looks = 0;
  for (size_t i = 0; i < st.size(); ++i) {
    if (st[i].arm_task == kArmStore) {
      ASSERT_GT(i, 0u);
      EXPECT_EQ(st[i - 1].arm_task, kArmLookRack);
      EXPECT_EQ(st[i - 1].slot, st[i].slot);
      EXPECT_EQ(st[i - 1].puck_id, st[i].puck_id);
    }
    if (st[i].arm_task == kArmLookRack) {++looks;}
  }
  EXPECT_EQ(looks, stores);
}

// ---------------------------------------------------------------- 장소 박스 측정 (place_box)

TEST(PlaceBox, FoldQuarterDiff)
{
  EXPECT_NEAR(fold_quarter_diff(0.1, 0.0), 0.1, 1e-12);
  EXPECT_NEAR(fold_quarter_diff(M_PI / 2 - 0.1, 0.0), -0.1, 1e-12);   // 90° 돈 정사각형 = 같은 박스
  EXPECT_NEAR(fold_quarter_diff(M_PI + 0.2, 0.0), 0.2, 1e-12);
}

// 중앙값 · 먼 측정 버림 · 지난 측정 안 씀 · 개수 부족
TEST(PlaceBox, FrameFromSamples)
{
  const std::array<double, 2> c{3.0, 1.0};
  std::vector<BoxSample> v;
  v.push_back({0.5, 9.0, 9.0, 0.0});                       // since 전 → 안 씀
  for (int i = 0; i < 5; ++i) {v.push_back({1.0 + i, 3.01 + 0.001 * i, 0.98, M_PI / 2 + 0.05});}   // 90° 주기 → dyaw 0.05
  v.push_back({7.0, 3.3, 1.0, 0.0});                       // 30 cm 먼 측정 → 버림
  std::string why;
  const auto f = place_frame_from(v, 1.0, c, 0.0, 0.06, 3, &why);
  ASSERT_TRUE(f) << why;
  EXPECT_EQ(f->n, 5);
  EXPECT_NEAR(f->x, 3.012, 1e-9);
  EXPECT_NEAR(f->y, 0.98, 1e-9);
  EXPECT_NEAR(f->dyaw, 0.05, 1e-9);
  EXPECT_FALSE(place_frame_from(v, 4.5, c, 0.0, 0.06, 3, &why));   // since 뒤 2 개뿐
  EXPECT_NE(why.find("필요 3"), std::string::npos);
}

// 박스가 돌면 눈 오프셋도 같이 돎, 재검증 사본은 중심 · 오프셋만 바뀜
TEST(PlaceBox, RotatedCellsAndVerifyCopy)
{
  PlaceFrame f;
  f.x = 1.0;
  f.y = 2.0;
  f.dyaw = M_PI / 18;   // 10°
  const auto p = frame_point(f, {0.06, 0.0});
  EXPECT_NEAR(p[0], 1.0 + 0.06 * std::cos(M_PI / 18), 1e-12);
  EXPECT_NEAR(p[1], 2.0 + 0.06 * std::sin(M_PI / 18), 1e-12);
  EXPECT_NEAR(std::hypot(p[0] - 1.0, p[1] - 2.0), 0.06, 1e-12);   // 눈 간격 유지

  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  DicePlan plan;
  plan.places[1].pip_offsets = {{0.06, 0.0}, {-0.06, 0.06}};
  plan.places[0].pip_offsets = {{0.06, 0.0}};
  apply_place_frame(a, plan, 1, f);
  EXPECT_DOUBLE_EQ(a.places[1].center[0], 1.0);
  EXPECT_NEAR(plan.places[1].pip_offsets[0][1], 0.06 * std::sin(M_PI / 18), 1e-12);
  EXPECT_DOUBLE_EQ(plan.places[0].pip_offsets[0][0], 0.06);   // 다른 장소는 그대로
}

// 박스 테두리: 기본 범위 102.5 − 21.2 − 10 = 71.3 mm → 눈 간격 60 mm 칸은 그대로, 넘는 칸만 안쪽으로
TEST(PlaceBox, ClampCellInsideBoxEdge)
{
  const PlaceLimits lim;
  const double r = lim.box_half - lim.puck_half - lim.margin;
  for (const auto & o : dice_pip_offsets(6, 0.06)) {   // 6 = 귀 · 가장자리 칸 전부
    auto c = o;
    EXPECT_FALSE(clamp_offset_in_box(c, 0.0, lim));
    EXPECT_EQ(c, o);
  }
  std::array<double, 2> o{0.09, -0.08};
  EXPECT_TRUE(clamp_offset_in_box(o, 0.0, lim));
  EXPECT_NEAR(o[0], r, 1e-12);
  EXPECT_NEAR(o[1], -r, 1e-12);

  // 박스가 지도에서 30° 돌아 있으면 박스 좌표로 자름: 박스 축 방향 0.09 → r, 옆 0.01 은 그대로
  const double yaw = M_PI / 6;
  std::array<double, 2> q{0.09 * std::cos(yaw) - 0.01 * std::sin(yaw), 0.09 * std::sin(yaw) + 0.01 * std::cos(yaw)};
  EXPECT_TRUE(clamp_offset_in_box(q, yaw, lim));
  EXPECT_NEAR(std::cos(yaw) * q[0] + std::sin(yaw) * q[1], r, 1e-12);
  EXPECT_NEAR(-std::sin(yaw) * q[0] + std::cos(yaw) * q[1], 0.01, 1e-12);

  // 측정 박스가 돌아도 (frame_point) 퍽 중심은 측정 박스 좌표 범위 안
  PlaceFrame f;
  f.x = 1.0;
  f.y = 2.0;
  f.dyaw = 0.3;
  std::array<double, 2> w{0.12, 0.12};
  clamp_offset_in_box(w, 0.0, lim);
  const auto p = frame_point(f, w);
  const double u = std::cos(f.dyaw) * (p[0] - f.x) + std::sin(f.dyaw) * (p[1] - f.y);
  const double v = -std::sin(f.dyaw) * (p[0] - f.x) + std::cos(f.dyaw) * (p[1] - f.y);
  EXPECT_LE(std::fabs(u), r + 1e-12);
  EXPECT_LE(std::fabs(v), r + 1e-12);

  // 퍽 + 여유가 박스보다 크면 가운데
  PlaceLimits tiny;
  tiny.box_half = 0.02;
  std::array<double, 2> z{0.05, 0.0};
  EXPECT_TRUE(clamp_offset_in_box(z, 0.0, tiny));
  EXPECT_DOUBLE_EQ(z[0], 0.0);
}

// box_wait: 장소 LOOK 바로 뒤에만 kPlaceBox, 박스 LOOK 뒤에는 없음, 꺼 두면 지금과 같음
TEST(PlaceBox, WaitStepAfterPlaceLook)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  ArmAction lp;
  lp.op = ArmOp::Look;
  lp.place = 2;
  ArmAction lb;
  lb.op = ArmOp::Look;
  lb.box = 3;
  const std::vector<ArmAction> acts{lb, lp};
  EXPECT_EQ(arm_steps(acts).size(), 2u);
  const auto st = arm_steps(acts, false, true);
  ASSERT_EQ(st.size(), 3u);
  EXPECT_EQ(st[1].arm_task, kArmLook);
  EXPECT_EQ(place_of(st[1]), 2);
  EXPECT_TRUE(is_custom(st[2], kPlaceBox));
  EXPECT_EQ(place_of(st[2]), 2);
}

// 장소별 놓는 점 보정: 로봇 기준 [앞, 왼쪽] mm 를 그 장소 보는 방향으로 map 에 (정차 yaw 회전과 같음). 재검증 눈 점 (dice_cell_point) 은 안 건드림
TEST(PlaceBox, PlaceOffsetFollowsViewDirection)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  for (int p = 0; p < kNumPlaces; ++p) {
    const double yaw = a.places[p].stop.yaw;
    const auto d = place_offset_map(a.places[p].view_dir, {10.0, 5.0});
    EXPECT_NEAR(d[0], 0.010 * std::cos(yaw) - 0.005 * std::sin(yaw), 1e-9) << "L" << p + 1;
    EXPECT_NEAR(d[1], 0.010 * std::sin(yaw) + 0.005 * std::cos(yaw), 1e-9) << "L" << p + 1;
    EXPECT_NEAR(std::hypot(d[0], d[1]), std::hypot(0.010, 0.005), 1e-12);
  }
  // L1: 남쪽 정차에서 경기장 +Y 를 봄 = map −Y (출발이 map 원점, 서쪽 → 남쪽) → 앞 +10 = map y −0.010, 왼쪽 +5 = map x +0.005
  const auto d1 = place_offset_map(a.places[0].view_dir, {10.0, 5.0});
  EXPECT_NEAR(d1[0], 0.005, 1e-4);
  EXPECT_NEAR(d1[1], -0.010, 1e-4);
  const auto z = place_offset_map({0.0, 0.0}, {10.0, 5.0});   // 방향 없음 → 0
  EXPECT_DOUBLE_EQ(z[0], 0.0);
  EXPECT_DOUBLE_EQ(z[1], 0.0);
  const auto z2 = place_offset_map(a.places[2].view_dir, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(z2[0], 0.0);
  EXPECT_DOUBLE_EQ(z2[1], 0.0);
}

// ---------------------------------------------------------------- 적재함 칸 제약 · 쥔 퍽 (전수 시험에서 찾은 경우)
namespace
{

// 실물 넣는 순서 · 칸 제약 (config/planning.yaml) 을 건 적재함
Cargo rack_with_rules()
{
  Cargo c({"S1", "S2", "S3", "S8", "S11", "S6", "S12", "S7", "S10", "S5", "S9", "S4"});
  c.set_rules({{"S1", "S2"}, {"S9", "S4"}, {"S10", "S5"}, {"S11", "S6"}, {"S12", "S7"}}, {{"S1", "S2"}, {"S7", "S12"}, {"S10", "S5"}});
  return c;
}

// "GbGcRc…" (박스 1 부터 2 개씩, 가까운 퍽 먼저) → 퍽 12 개
std::vector<TransferPuck> pucks_from(const std::string & k)
{
  std::vector<TransferPuck> v;
  for (int i = 0; i < 12; ++i) {
    const bool red = k[2 * i] == 'R', box = k[2 * i + 1] == 'b';
    v.push_back(TransferPuck{std::to_string(i), static_cast<PuckKind>((red ? 0 : 2) + (box ? 1 : 0)), i / 2 + 1});
  }
  return v;
}

struct RackRun
{
  int violations = 0;   // 넣기 · 빼기 전에 비어야 할 칸이 찬 채 한 동작 수
  int relocs = 0;       // 같은 정차에서 RETRIEVE 바로 뒤 STORE (막는 퍽 옮기기)
  int placed = 0;
};

// 노드 흐름 그대로: collect_order 앞 5 곳 collect_box_actions → 마지막 박스에서 plan_transfer → 끝까지 재생하며 칸 제약 검사
RackRun run_rack(const std::string & kinds, const SignalCounts & sc, DiceOrder order)
{
  RackRun out;
  TransferCosts costs;
  EXPECT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));
  const auto all = pucks_from(kinds);
  const std::vector<int> collect{1, 2, 4, 5, 6, 3};
  Cargo cargo = rack_with_rules();
  auto apply = [&](const ArmAction & a) {
      if (a.op == ArmOp::Store && !cargo.store_ok(a.slot)) {++out.violations;}
      if (a.op == ArmOp::Retrieve && !cargo.retrieve_ok(a.slot)) {++out.violations;}
      if (a.op == ArmOp::Pick) {EXPECT_TRUE(cargo.on_pick({a.puck_id, a.color, a.shape, "", false}));}
      if (a.op == ArmOp::Store) {EXPECT_TRUE(cargo.on_store(a.slot));}
      if (a.op == ArmOp::Retrieve) {EXPECT_TRUE(cargo.on_retrieve(a.slot));}
      if (a.op == ArmOp::Place) {EXPECT_TRUE(cargo.on_place().has_value()); ++out.placed;}
    };
  for (std::size_t i = 0; i + 1 < collect.size(); ++i) {
    std::vector<TransferPuck> at;
    for (const auto & p : all) {if (p.box == collect[i]) {at.push_back(p);}}
    for (const auto & a : collect_box_actions(at, cargo)) {apply(a);}
  }
  SolverInput in;
  in.position = collect.back();
  in.stopped = true;
  for (const auto & p : all) {if (p.box == collect.back()) {in.on_boxes.push_back(p);}}
  EXPECT_TRUE(fill_carried(cargo, in));
  const auto lay = dice_layout(sc, order);
  EXPECT_TRUE(lay.has_value());
  for (int i = 0; i < kNumPlaces; ++i) {
    in.place_group[i] = lay->places[i].group;
    in.slots_left[i] = lay->places[i].pips;
  }
  std::string err;
  const auto plan = plan_transfer(in, costs, cargo, {{0, 0, 0}}, &err);
  EXPECT_TRUE(plan.has_value()) << err;
  if (!plan) {return out;}
  check_plan_rules(*plan, all, in);
  for (const auto & s : plan->stops) {
    for (std::size_t i = 0; i < s.actions.size(); ++i) {
      if (i + 1 < s.actions.size() && s.actions[i].op == ArmOp::Retrieve && s.actions[i + 1].op == ArmOp::Store) {++out.relocs;}
      apply(s.actions[i]);
    }
  }
  return out;
}

}  // namespace

TEST(RackRules, PicksOtherPuckOfSameGroup)
{
  // L3 G3 에 꼭 갈 초록 사각 S1 퍽 0 · S7 퍽 5 가 S2 · S12 에 막힘 (예비 펼치기 = 같은 종류 뒤 칸만 → 막힌 채 뺌)
  //   → G 의 초록 원기둥으로 S2 퍽 4 를 쓰고 L2 를 먼저 가면 옮기기 없이 됨
  const auto r = run_rack("GbGcGcRbGcGbGcRbRcGcRcRc", SignalCounts{3, 6, 3}, DiceOrder::Descending);
  EXPECT_EQ(r.violations, 0);
  EXPECT_EQ(r.relocs, 0);
  EXPECT_EQ(r.placed, 12);
}

TEST(RackRules, CycleNeedsOneRelocation)
{
  // S7(R) ← S12(Y) = L1 먼저, S10(Y) ← S5(R) = L2 먼저 → 방문 순서로는 못 풂, 막는 퍽 한 번 옮김
  const auto r = run_rack("GcGbGbGbRcRcRbRbGbRcRcRc", SignalCounts{5, 6, 1}, DiceOrder::Descending);
  EXPECT_EQ(r.violations, 0);
  EXPECT_EQ(r.relocs, 1);
  EXPECT_EQ(r.placed, 12);
}

TEST(RackRules, CollectStoresGripperWithNoPucksLeft)
{
  // 박스 마지막 퍽 STORE 가 거절된 뒤 다시 계획: 박스에 남은 퍽이 없어도 쥔 퍽을 넣어야 출발 가능
  Cargo cargo = rack_with_rules();
  ASSERT_TRUE(cargo.on_pick({"3", "red", "box", "", false}));
  std::string err;
  const auto acts = collect_box_actions({}, cargo, &err);
  ASSERT_EQ(acts.size(), 1u) << err;
  EXPECT_EQ(acts[0].op, ArmOp::Store);
  EXPECT_EQ(acts[0].puck_id, "3");
}

TEST(RackRules, ReplanStoresGripperBeforeMoving)
{
  // 배달 중 PLACE 거절 → 그 장소 버림, 퍽은 그리퍼에 남음 → 서 있지 않은 상태 (stopped=false) 로 재계획
  //   정확해 (박스 남은 퍽 ≤ 4) · 대체 해 (> 4) 둘 다 출발 전에 지금 자리에서 넣어야 함
  for (const int left_on_boxes : {0, 6}) {
    const auto all = q1_pucks();
    Cargo cargo = rack_with_rules();
    SolverInput in;
    in.position = place_node(1);
    in.stopped = false;
    in.place_group = {{'Y', 'R', 'G'}};
    in.slots_left = {{2, 0, 6}};   // L2 버림
    in.allow_leftover = true;
    for (const auto & p : all) {
      const CargoItem it{p.id, kind_color(p.kind), kind_shape(p.kind), "", false};
      if (std::stoi(p.id) < 12 - left_on_boxes) {
        if (p.id == "0") {continue;}
        ASSERT_TRUE(cargo.on_pick(it));
        ASSERT_TRUE(cargo.on_store(*cargo.free_slot_for(it)));
      } else {
        in.on_boxes.push_back(p);
      }
    }
    ASSERT_TRUE(cargo.on_pick({"0", kind_color(all[0].kind), kind_shape(all[0].kind), "", false}));   // 쥔 퍽 (빨강)
    ASSERT_TRUE(fill_carried(cargo, in));
    std::string err;
    const auto plan = plan_transfer(in, test_costs(), cargo, {{0, 0, 0}}, &err);
    ASSERT_TRUE(plan.has_value()) << left_on_boxes << ": " << err;
    ASSERT_FALSE(plan->stops.empty());
    const auto & first = plan->stops.front();
    EXPECT_FALSE(first.move) << left_on_boxes;
    EXPECT_EQ(first.node, place_node(1));
    ASSERT_FALSE(first.actions.empty());
    EXPECT_EQ(first.actions.front().op, ArmOp::Store);
    EXPECT_EQ(first.actions.front().puck_id, "0");
    bool holding = true;
    for (const auto & s : plan->stops) {
      if (s.move) {EXPECT_FALSE(holding) << "퍽 쥔 채 이동: " << s.name;}
      for (const auto & a : s.actions) {
        if (a.op == ArmOp::Pick || a.op == ArmOp::Retrieve) {holding = true;}
        if (a.op == ArmOp::Store || a.op == ArmOp::Place) {holding = false;}
      }
    }
  }
}

// ============================================================================ FAULT 뒤 task 이어 하기 (resume_continue_task)
namespace
{
Step arm_step(uint8_t task, const std::string & name, const std::string & puck = "")
{
  Step s;
  s.type = StepType::Arm;
  s.arm_task = task;
  s.name = name;
  s.puck_id = puck;
  return s;
}

std::size_t index_of(const std::vector<Step> & st, const std::string & name)
{
  for (std::size_t i = 0; i < st.size(); ++i) {if (st[i].name == name) {return i;}}
  return st.size();
}
}  // namespace

// COLLECT 박스 안 (B2 Plan 이 끼운 PICK · STORE, 그 앞 HOME) 에서 FAULT → 하던 단계 = STORE, B2 이동부터 다시, 남은 박스 B2 부터
TEST(Resume, CollectMidBoxRestartsFromBoxStop)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  auto st = collect_steps(a, {1, 2, 4, 5, 6, 3});
  const std::size_t plan_b2 = index_of(st, "plan:collect:B2");
  ASSERT_LT(plan_b2, st.size());
  auto pick = arm_step(kArmPick, "arm:PICK:7", "7");
  box_of(pick) = 2;
  auto store = arm_step(kArmStore, "arm:STORE:7>S3", "7");
  store.slot = "S3";
  st.insert(st.begin() + static_cast<std::ptrdiff_t>(plan_b2) + 1, {pick, arm_step(kArmHome, "arm:HOME"), store});
  const std::size_t at = index_of(st, "arm:HOME");   // 끼운 HOME 에서 FAULT (HOME_FAILED)
  const auto r = resume_point(st, at);
  ASSERT_TRUE(r.work.has_value());
  EXPECT_EQ(r.work->name, "arm:STORE:7>S3");   // HOME 은 작업 아님 → 그 뒤 작업
  EXPECT_EQ(st[r.seg].name, "nav:B2");
  EXPECT_EQ(r.boxes, (std::vector<int>{2, 4, 5, 6, 3}));
  ASSERT_FALSE(r.tail.empty());
  EXPECT_EQ(r.tail.front().name, "nav:B2");
  // 다시 만든 수집 단계: B2 부터, 끝 박스 B3 은 배달 계획 (처음과 같은 모양)
  const auto again = collect_steps(a, r.boxes);
  ASSERT_EQ(again.size(), 15u);
  EXPECT_EQ(again.front().name, "nav:B2");
  EXPECT_TRUE(is_custom(again.back(), kPlanDelivery));
  EXPECT_EQ(box_of(again.back()), 3);
  // STORE 는 뺄 것 없음 (재고 작업, 사람 정합 · 횟수만)
  const auto k = resume_skip(*r.work);
  EXPECT_TRUE(k.puck.empty());
  EXPECT_EQ(k.box, -1);
  EXPECT_EQ(k.place, -1);
}

// 이동 중 FAULT (LOCALIZATION_LOST): 그 박스부터. 같은 FAULT 반복이면 그 박스를 뺌
TEST(Resume, NavFaultAndRepeatSkipsBox)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto st = collect_steps(a, {1, 2, 4, 5, 6, 3});
  const std::size_t at = index_of(st, "nav:B4");
  const auto r = resume_point(st, at);
  ASSERT_TRUE(r.work.has_value());
  EXPECT_EQ(r.seg, at);
  EXPECT_EQ(r.boxes, (std::vector<int>{4, 5, 6, 3}));
  const auto k = resume_skip(*r.work);
  EXPECT_EQ(k.box, 4);
  EXPECT_TRUE(k.puck.empty());
  EXPECT_EQ(resume_key("LOCALIZATION_LOST", r.work), resume_key("LOCALIZATION_LOST", resume_point(st, at).work));
  EXPECT_NE(resume_key("LOCALIZATION_LOST", r.work), resume_key("HOME_FAILED", r.work));
  EXPECT_NE(resume_key("LOCALIZATION_LOST", r.work), resume_key("LOCALIZATION_LOST", resume_point(st, at + 1).work));
  // 박스 LOOK 반복도 그 박스
  EXPECT_EQ(resume_skip(st[at + 1]).box, 4);
}

// 마지막 박스 이동 실패 뒤 '지금 자리 배달 계획' 에서 FAULT → 남은 박스 없음 (지난 박스로 되돌아가지 않음)
TEST(Resume, DeliveryHereHasNoBoxesLeft)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  auto st = collect_steps(a, {4});
  st.back().kind = kPlanCollectBox;
  Step here;
  here.type = StepType::Custom;
  here.kind = kPlanDelivery;
  box_of(here) = -1;
  here.name = "plan:delivery:here";
  st.push_back(arm_step(kArmHome, "arm:HOME"));
  st.push_back(here);
  const auto r = resume_point(st, st.size() - 2);
  ASSERT_TRUE(r.work.has_value());
  EXPECT_EQ(r.work->name, "plan:delivery:here");
  EXPECT_TRUE(r.boxes.empty());
  // 목록 끝 (다 함) → 하던 단계 없음, 남은 박스 없음
  const auto end = resume_point(st, st.size());
  EXPECT_FALSE(end.work.has_value());
  EXPECT_TRUE(end.boxes.empty());
  EXPECT_TRUE(end.tail.empty());
  EXPECT_FALSE(resume_point({}, 0).work.has_value());
}

// 뺄 것: PICK · 퍽 다시 보기 → 그 퍽, PLACE · 장소 이동 → 그 장소, "?" 다시 보기 → 그 박스
TEST(Resume, SkipTargets)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  auto pick = arm_step(kArmPick, "arm:PICK:9", "9");
  box_of(pick) = 2;
  EXPECT_EQ(resume_skip(pick).puck, "9");
  EXPECT_EQ(resume_skip(pick).box, -1);
  auto relook = arm_step(kArmLook, "arm:LOOK:B2(다시 보기:9)", "9");
  box_of(relook) = 2;
  EXPECT_EQ(resume_skip(relook).puck, "9");
  relook.puck_id = "?";
  EXPECT_TRUE(resume_skip(relook).puck.empty());
  EXPECT_EQ(resume_skip(relook).box, 2);
  auto place = arm_step(kArmPlace, "arm:PLACE:9>L2#0", "9");
  place.target = Target::Custom;
  place.kind = kTargetDiceCell;
  place_of(place) = 1;
  cell_of(place) = 0;
  EXPECT_EQ(resume_skip(place).place, 1);
  EXPECT_TRUE(resume_skip(place).puck.empty());
  const auto ver = verify_steps({{2, 0, 1}}, a);
  EXPECT_EQ(resume_skip(ver.front()).place, 2);   // 장소 이동
  auto ret = arm_step(kArmRetrieve, "arm:RETRIEVE:S1>9", "9");
  ret.slot = "S1";
  EXPECT_TRUE(resume_skip(ret).puck.empty());
  EXPECT_EQ(resume_skip(ret).place, -1);
  EXPECT_EQ(resume_skip(ret).box, -1);
  EXPECT_EQ(resume_skip(arm_step(kArmHome, "arm:HOME")).box, -1);
}

// 재검증 이어 하기: 지금 장소부터 남은 장소, 반복이면 그 장소 정차를 뺌
TEST(Resume, VerifyTailAndDropPlace)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto st = verify_steps({{0, 1, 2}}, a);
  ASSERT_EQ(st.size() % 3, 0u);
  const std::size_t per = st.size() / 3;
  const auto r = resume_point(st, per + 1);   // L2 LOOK 에서 FAULT
  ASSERT_TRUE(r.work.has_value());
  EXPECT_EQ(r.seg, per);
  EXPECT_EQ(r.tail.size(), 2 * per);
  EXPECT_EQ(place_of(r.tail.front()), 1);
  EXPECT_EQ(resume_skip(*r.work).place, 1);
  const auto dropped = drop_place_stops(r.tail, 1);
  EXPECT_EQ(dropped.size(), per);
  for (const auto & s : dropped) {EXPECT_NE(place_of(s), 1) << s.name;}
  EXPECT_EQ(drop_place_stops(st, 5).size(), st.size());   // 없는 장소면 그대로
}

// 재검증 LOOK 자세: "far" 면 LOOK 에 slot far, 이름 :far. 이상한 값 · "" 는 정면 그대로
TEST(Verify, LookSlot)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto far = verify_steps({{0, 1, 2}}, a, "far");
  const auto front = verify_steps({{0, 1, 2}}, a);
  const auto bad = verify_steps({{0, 1, 2}}, a, "up");
  ASSERT_EQ(far.size(), front.size());
  int looks = 0;
  for (std::size_t i = 0; i < far.size(); ++i) {
    if (far[i].type != StepType::Arm || far[i].arm_task != kArmLook) {continue;}
    ++looks;
    EXPECT_EQ(far[i].slot, "far");
    EXPECT_EQ(far[i].name, front[i].name + ":far");
    EXPECT_EQ(place_of(far[i]), place_of(front[i]));
    EXPECT_TRUE(front[i].slot.empty());
    EXPECT_TRUE(bad[i].slot.empty());
  }
  EXPECT_EQ(looks, 3);
}

// ============================================================================ 장소 박스 다시 붙기 (box_realign_*)
// 거리 기준: 4 cm 안은 그대로, 넘으면 다시 붙기. 횟수를 다 쓰면 넘어도 안 붙음 (그대로 놓음)
TEST(PlaceRealign, ShiftThresholdAndCount)
{
  PlaceFrame e;
  e.x = 2.0;
  e.y = 3.0;
  PlaceFrame f = e;
  f.x += 0.03;
  auto r = realign_check(f, e, 0.04, 8.0 * M_PI / 180.0, 0, 1);
  EXPECT_NEAR(r.shift_m, 0.03, 1e-12);
  EXPECT_FALSE(r.over);
  EXPECT_FALSE(r.realign);
  f.x = e.x + 0.08;                                        // 모의 f4c 8 cm
  r = realign_check(f, e, 0.04, 8.0 * M_PI / 180.0, 0, 1);
  EXPECT_TRUE(r.over);
  EXPECT_TRUE(r.realign);
  r = realign_check(f, e, 0.04, 8.0 * M_PI / 180.0, 1, 1);   // 한 번 붙은 뒤
  EXPECT_TRUE(r.over);
  EXPECT_FALSE(r.realign);
  r = realign_check(f, e, 0.04, 8.0 * M_PI / 180.0, 0, 0);   // box_realign_max 0 = 끔
  EXPECT_FALSE(r.realign);
}

// 방향 기준: 8° 안은 그대로, 넘으면 다시 붙기. 정사각형이라 ±45° 로 접어서 봄 (89° = −1°)
TEST(PlaceRealign, YawThresholdFolded)
{
  PlaceFrame e;
  const double deg = M_PI / 180.0;
  auto at = [&](double d) {PlaceFrame f = e; f.dyaw = d * deg; return realign_check(f, e, 0.04, 8.0 * deg, 0, 1);};
  EXPECT_FALSE(at(5.0).over);
  EXPECT_TRUE(at(10.0).realign);
  EXPECT_TRUE(at(-10.0).realign);
  EXPECT_TRUE(at(15.0).realign);                           // 모의 15° 경우
  EXPECT_NEAR(at(50.0).dyaw, -40.0 * deg, 1e-9);
  EXPECT_FALSE(at(89.0).over);
  // 기대 방향이 이미 돌아 있으면 그 차이만 봄
  e.dyaw = 15.0 * deg;
  EXPECT_FALSE(at(16.0).over);
}

// 다시 붙는 정차점: 측정 박스까지 거리 · 보는 방향은 지도 정차와 같고, 박스가 돈 만큼 같이 돎
TEST(PlaceRealign, RealignedStopKeepsStandoff)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto & pl = a.places[1];
  const double standoff = std::hypot(pl.center[0] - pl.stop.x, pl.center[1] - pl.stop.y);
  PlaceFrame f;
  f.x = pl.center[0] + 0.08;
  f.y = pl.center[1];
  auto s = realigned_stop(f, pl.stop, pl.center);
  EXPECT_NEAR(s.x, pl.stop.x + 0.08, 1e-12);                // 밀린 만큼만 옮김
  EXPECT_NEAR(s.y, pl.stop.y, 1e-12);
  EXPECT_NEAR(s.yaw, std::remainder(pl.stop.yaw, 2 * M_PI), 1e-12);
  f.dyaw = 15.0 * M_PI / 180.0;
  s = realigned_stop(f, pl.stop, pl.center);
  EXPECT_NEAR(std::hypot(f.x - s.x, f.y - s.y), standoff, 1e-9);
  EXPECT_NEAR(std::remainder(s.yaw - pl.stop.yaw - f.dyaw, 2 * M_PI), 0.0, 1e-12);
  // 정차 → 박스 방향이 정차 yaw 와 이루는 각은 지도와 같음 (박스 면을 똑바로 봄)
  const double a0 = std::remainder(std::atan2(pl.center[1] - pl.stop.y, pl.center[0] - pl.stop.x) - pl.stop.yaw, 2 * M_PI);
  const double a1 = std::remainder(std::atan2(f.y - s.y, f.x - s.x) - s.yaw, 2 * M_PI);
  EXPECT_NEAR(a0, a1, 1e-9);
}

// 기대 박스: 지도 정차면 지도 중심 · 0°, 다시 붙은 정차면 그때 측정 박스 → 박스가 그대로면 두 번째 측정은 기준 안 (한 번만 붙음)
TEST(PlaceRealign, ExpectedBoxAndAtMostOnce)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  const auto & pl = a.places[2];
  const auto e0 = expected_box_at(pl.stop, pl.stop, pl.center);
  EXPECT_NEAR(e0.x, pl.center[0], 1e-12);
  EXPECT_NEAR(e0.y, pl.center[1], 1e-12);
  EXPECT_NEAR(e0.dyaw, 0.0, 1e-12);
  const double deg = M_PI / 180.0;
  for (const auto & [dx, dy, dyaw] : std::vector<std::tuple<double, double, double>>{{0.12, 0.0, 0.0}, {0.0, -0.08, 15.0}, {0.05, 0.05, -12.0}}) {
    PlaceFrame f;
    f.x = pl.center[0] + dx;
    f.y = pl.center[1] + dy;
    f.dyaw = dyaw * deg;
    int done = 0;
    auto r = realign_check(f, e0, 0.04, 8.0 * deg, done, 1);
    ASSERT_TRUE(r.realign);
    ++done;
    const auto s = realigned_stop(f, pl.stop, pl.center);
    const auto e1 = expected_box_at(s, pl.stop, pl.center);   // 다시 붙은 정차에서 기대
    EXPECT_NEAR(e1.x, f.x, 1e-9);
    EXPECT_NEAR(e1.y, f.y, 1e-9);
    EXPECT_NEAR(e1.dyaw, f.dyaw, 1e-9);
    r = realign_check(f, e1, 0.04, 8.0 * deg, done, 1);       // 박스 그대로 → 기준 안
    EXPECT_FALSE(r.over);
    PlaceFrame g = f;                                          // 다시 재도 5 cm 다름 → 넘지만 횟수 다 씀 → 그대로 놓음
    g.x += 0.05;
    r = realign_check(g, e1, 0.04, 8.0 * deg, done, 1);
    EXPECT_TRUE(r.over);
    EXPECT_FALSE(r.realign);
  }
}

// 15 cm 까지 측정을 씀 (box_max_shift_m 기본 0.15), 그보다 먼 것은 버림
TEST(PlaceRealign, MaxShiftAccepts12cm)
{
  const std::array<double, 2> c{3.0, 1.0};
  std::vector<BoxSample> v;
  for (int i = 0; i < 4; ++i) {v.push_back({1.0 + i, 3.12, 1.0, 0.0});}
  EXPECT_FALSE(place_frame_from(v, 0.0, c, 0.0, 0.06, 3));   // 한도 6 cm 면 버림
  const auto f = place_frame_from(v, 0.0, c, 0.0, 0.15, 3);
  ASSERT_TRUE(f);
  EXPECT_NEAR(f->x, 3.12, 1e-12);
  std::vector<BoxSample> far(4, BoxSample{1.0, 3.2, 1.0, 0.0});
  EXPECT_FALSE(place_frame_from(far, 0.0, c, 0.0, 0.15, 3));
}

// 재검증 칸 기준 = 박스 측정 (측정 박스로 놓았는데 지도 칸으로 재검증하면 다 '칸 밖')
TEST(PlaceBox, VerifyUsesMeasuredFrame)
{
  auto v = q1_verify();
  const int pl = 2;
  PlaceFrame placed;
  placed.x = v.arena.places[pl].center[0] - 0.039;
  placed.y = v.arena.places[pl].center[1] - 0.003;
  placed.n = 3;
  // 측정 박스 기준 칸에 놓은 퍽 (PLACE 와 같은 frame_point)
  auto obs = v.obs[pl];
  for (size_t k = 0; k < obs.size(); ++k) {
    const auto pt = frame_point(placed, v.plan.places[pl].pip_offsets[k]);
    obs[k].x = pt[0] + 0.004;
    obs[k].y = pt[1] - 0.003;
  }
  VerifyFitParams no_fit;
  no_fit.enable = false;
  EXPECT_FALSE(judge_place(pl, v.plan, v.arena, obs, 0.03, 0.0, no_fit).ok);   // 지도 칸 기준 · 맞춤 없음 · 허용 3 cm → 칸 밖
  const auto fit_map = judge_place(pl, v.plan, v.arena, obs);   // 박스 측정 없어도 모양 맞춤으로 칸 위
  EXPECT_TRUE(fit_map.ok);
  EXPECT_TRUE(fit_map.fitted);
  EXPECT_NEAR(fit_map.fit_dx, -0.035, 0.005);

  // 재검증 칸 기준 = 재검증 LOOK 에서 다시 잰 박스 먼저 (퍽 관측과 같은 때 위치 추정) → 없으면 놓을 때 측정
  PlaceFrame other = placed;
  other.x += 0.03;
  const auto f = verify_place_frame(placed, other);
  ASSERT_TRUE(f.has_value());
  EXPECT_DOUBLE_EQ(f->x, other.x);
  DiceMap a = v.arena;
  DicePlan plan = v.plan;
  apply_place_frame(a, plan, pl, *f);
  EXPECT_TRUE(judge_place(pl, plan, a, obs, 0.05, 0.0, no_fit).ok);   // 3 cm 어긋나도 허용 5 cm 안

  EXPECT_DOUBLE_EQ(verify_place_frame(placed, std::nullopt)->x, placed.x);   // 다시 잰 것 없으면 놓을 때 측정
  EXPECT_FALSE(verify_place_frame(std::nullopt, std::nullopt).has_value());  // 둘 다 없으면 지도
}

// ---------------------------------------------------------------- 박스 다시 찾기 (box_relook)
namespace
{
// 빈 격자 (모두 0), 1 cm 칸, 원점 (ox, oy)
OccGrid free_grid(double ox, double oy, double w_m, double h_m, double res = 0.01)
{
  OccGrid g;
  g.res = res;
  g.ox = ox;
  g.oy = oy;
  g.w = static_cast<int>(std::lround(w_m / res));
  g.h = static_cast<int>(std::lround(h_m / res));
  g.data.assign(static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h), 0);
  return g;
}
void fill(OccGrid & g, double x0, double y0, double x1, double y1, int8_t v = 100)
{
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const double x = g.ox + (i + 0.5) * g.res, y = g.oy + (j + 0.5) * g.res;
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {g.data[static_cast<std::size_t>(j) * g.w + i] = v;}
    }
  }
}
double wrap_deg(double a) {return std::remainder(a, 2 * M_PI) * 180.0 / M_PI;}
}  // namespace

// 45° 돈 박스가 지도 자리에서 25 cm (대각) 떨어짐 → 다시 찾기 LOOK 측정 (±45° 경계에서 갈리는 방향) → 새 중심 · 방향
//   → 로봇 쪽 면 (45°) 정차점 = 중심 − 0.2575 · 법선, yaw = 면을 봄 → 정밀 접근 FaceTrack 이 goal 에서 잡는 박스 = 새 중심, 기울기 0
TEST(BoxRelook, Rotated45Off25cm)
{
  const std::array<double, 2> map_c{1.0, 1.0};
  const Pose2D map_stop{1.0, 1.0 - 0.2575, M_PI / 2};
  const std::array<double, 2> real_c{1.18, 1.18};
  const double real_yaw = M_PI / 4;
  std::vector<BoxSample> v;
  const double yaws[] = {real_yaw + 0.01, real_yaw - 0.012, -real_yaw + 0.008, real_yaw + 0.004, -real_yaw - 0.006, real_yaw};
  for (int i = 0; i < 6; ++i) {v.push_back({10.0 + 0.2 * i, real_c[0] + 0.002 * (i % 2), real_c[1] - 0.002 * (i % 3), yaws[i]});}
  v.push_back({5.0, 1.0, 1.0, 0.0});   // LOOK 전 측정은 안 씀
  const std::vector<std::array<double, 2>> pucks{
    {real_c[0] + 0.04 * std::cos(real_yaw), real_c[1] + 0.04 * std::sin(real_yaw)},
    {real_c[0] - 0.04 * std::cos(real_yaw), real_c[1] - 0.04 * std::sin(real_yaw)}};
  std::string why;
  // 한도 0.25 m 면 25.5 cm 라 측정 · 퍽 다 버림
  EXPECT_FALSE(box_find_from(v, 9.8, map_c, map_stop.yaw, pucks, true, 0.205, 0.25, 3, &why).has_value()) << why;
  const auto f = box_find_from(v, 9.8, map_c, map_stop.yaw, pucks, true, 0.205, 0.30, 3, &why);
  ASSERT_TRUE(f.has_value()) << why;
  EXPECT_TRUE(f->from_detector);
  EXPECT_EQ(f->n, 6);
  EXPECT_NEAR(f->center[0], real_c[0], 0.003);
  EXPECT_NEAR(f->center[1], real_c[1], 0.003);
  EXPECT_LT(std::fabs(fold_quarter_diff(f->yaw, real_yaw)), 0.02);   // 90° 주기 같음 (중앙값이 ±45° 에서 안 갈림)

  // 다시 찾기 LOOK 자리 = 지도 정차 0.12 m 뒤 → 로봇 쪽 면
  const Pose2D stand{map_stop.x, map_stop.y - 0.12, map_stop.yaw};
  const double ref = std::atan2(f->center[1] - stand.y, f->center[0] - stand.x);
  const OccGrid g = free_grid(-1.0, -1.0, 4.0, 4.0);
  const auto face = choose_face(box_face_stops(f->center, f->yaw, 0.2575, ref), [&](const Pose2D & q) {
      return footprint_blocked(g, q, 0.16, 0.16, 0.03, 65, {});
    });
  ASSERT_TRUE(face.has_value());
  EXPECT_EQ(face->blocked, 0);
  EXPECT_NEAR(wrap_deg(face->stop.yaw - real_yaw), 0.0, 1.5);   // 45° 면 (로봇 쪽, 72° 방향에서 가장 덜 돎)
  EXPECT_NEAR(face->stop.x, real_c[0] - 0.2575 * std::cos(real_yaw), 0.004);
  EXPECT_NEAR(face->stop.y, real_c[1] - 0.2575 * std::sin(real_yaw), 0.004);
  // 정밀 접근 (navigation FaceTrack) 은 goal + box_dist · (cos yaw, sin yaw) 를 박스 중심, goal yaw 를 앞면 법선으로 잡음 (±15°)
  const double c0x = face->stop.x + 0.2575 * std::cos(face->stop.yaw), c0y = face->stop.y + 0.2575 * std::sin(face->stop.yaw);
  EXPECT_LT(std::hypot(c0x - real_c[0], c0y - real_c[1]), 0.005);
  EXPECT_LT(std::fabs(fold_quarter_diff(face->stop.yaw, real_yaw)) * 180.0 / M_PI, 1.5);   // 기울기 ≈ 0 (±15° 안)
  // 지도 yaw (90°) 로는 박스 면이 45° 기울어 FaceTrack 이 버림 → 정차 yaw 를 바꿔야 붙음
  EXPECT_GT(std::fabs(fold_quarter_diff(map_stop.yaw, real_yaw)) * 180.0 / M_PI, 15.0);
}

// 벽 옆 박스: 로봇 쪽 면 정차가 벽에 닿으면 다음으로 덜 도는 면 중 빈 면, 모두 막히면 가장 덜 막힌 면 (거절 없음)
TEST(BoxRelook, FaceChoiceNearWall)
{
  const std::array<double, 2> c{1.0, 1.0};
  OccGrid g = free_grid(-1.0, -1.0, 4.0, 4.0);
  fill(g, -1.0, 0.70, 3.0, 0.76);      // 남쪽 벽: 로봇 쪽 (90°) 면 정차 (1, 0.7425) 가 벽 위
  fill(g, -1.0, -1.0, 0.70, 3.0);      // 서쪽 벽: 0° 면 정차 (0.7425, 1) 막힘
  const auto blocked = [&](const Pose2D & q) {return footprint_blocked(g, q, 0.16, 0.16, 0.03, 65, {});};
  const auto cands = box_face_stops(c, 0.0, 0.2575, M_PI / 2);
  ASSERT_EQ(cands.size(), 4u);
  EXPECT_NEAR(wrap_deg(cands[0].stop.yaw), 90.0, 1e-6);   // 로봇 쪽 면 먼저
  EXPECT_GT(blocked(cands[0].stop), 0);
  const auto f = choose_face(cands, blocked);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->blocked, 0);
  EXPECT_NEAR(std::fabs(wrap_deg(f->stop.yaw)), 180.0, 1e-6);   // 동쪽에서 서쪽 (−X) 을 봄
  EXPECT_NEAR(f->stop.x, 1.2575, 1e-9);
  EXPECT_NEAR(f->stop.y, 1.0, 1e-9);

  // 다른 박스 테두리도 막힘으로 셈 (지도에 없는 박스)
  const auto other = box_outline({1.55, 1.0}, 0.0, 0.205);
  const auto f2 = choose_face(cands, [&](const Pose2D & q) {return footprint_blocked(g, q, 0.16, 0.16, 0.03, 65, other);});
  ASSERT_TRUE(f2.has_value());
  EXPECT_NEAR(wrap_deg(f2->stop.yaw), -90.0, 1e-6);   // 북쪽 면 (남쪽을 봄)

  // 모두 막힘 → 가장 덜 막힌 면
  OccGrid all = free_grid(-1.0, -1.0, 4.0, 4.0);
  fill(all, -1.0, -1.0, 3.0, 3.0);
  fill(all, 0.6, 1.1, 1.5, 1.6, 0);   // 북쪽만 조금 빔
  const auto f3 = choose_face(cands, [&](const Pose2D & q) {return footprint_blocked(all, q, 0.16, 0.16, 0.03, 65, {});});
  ASSERT_TRUE(f3.has_value());
  EXPECT_GT(f3->blocked, 0);
  EXPECT_NEAR(wrap_deg(f3->stop.yaw), -90.0, 1e-6);

  // 지도 밖 · 모름 = 막힘, 빈 격자 = 격자 안 봄
  EXPECT_GT(footprint_blocked(g, Pose2D{-5.0, -5.0, 0.0}, 0.16, 0.16, 0.0, 65, {}), 0);
  OccGrid unk = free_grid(-1.0, -1.0, 4.0, 4.0);
  fill(unk, 0.9, 0.9, 1.1, 1.1, -1);
  EXPECT_GT(footprint_blocked(unk, Pose2D{1.0, 1.0, 0.0}, 0.05, 0.05, 0.0, 65, {}), 0);
  EXPECT_EQ(footprint_blocked(OccGrid{}, Pose2D{1.0, 1.0, 0.0}, 0.16, 0.16, 0.03, 65, {}), 0);

  // slam 지도에 찍힌 목표 박스 자신은 ignore 로 안 셈 (몸 사각형 0.19 > 정차 ↔ 면 0.155 라 안 빼면 네 면 다 막힘)
  OccGrid sl = free_grid(-1.0, -1.0, 4.0, 4.0);
  fill(sl, 1.0 - 0.1025, 1.0 - 0.1025, 1.0 + 0.1025, 1.0 + 0.1025);
  const Pose2D south{1.0, 1.0 - 0.2575, M_PI / 2};
  EXPECT_GT(footprint_blocked(sl, south, 0.16, 0.16, 0.03, 65, {}), 0);
  EXPECT_EQ(footprint_blocked(sl, south, 0.16, 0.16, 0.03, 65, {}, {{1.0, 1.0, 0.0}}), 0);
}

// 측정 박스 밖에 본 퍽 = 다른 박스 · 틀린 맞춤 → 측정 안 쓰고 퍽 평균 (방향 = 정차 yaw). 측정 끄기 · 아무것도 없음
TEST(BoxRelook, DetectorFallbacks)
{
  const std::array<double, 2> map_c{1.0, 1.0};
  std::vector<BoxSample> v;
  for (int i = 0; i < 4; ++i) {v.push_back({10.0 + 0.2 * i, 1.10, 1.0, 0.0});}
  const std::vector<std::array<double, 2>> far_pucks{{0.86, 1.0}, {0.94, 1.0}};   // 측정 박스 (1.10) 윗면 밖
  std::string why;
  auto f = box_find_from(v, 9.8, map_c, M_PI / 2, far_pucks, true, 0.205, 0.30, 3, &why);
  ASSERT_TRUE(f.has_value()) << why;
  EXPECT_FALSE(f->from_detector);
  EXPECT_NEAR(f->center[0], 0.90, 1e-9);
  EXPECT_NEAR(f->yaw, M_PI / 2, 1e-12);
  EXPECT_NE(f->note.find("측정 안 씀"), std::string::npos);
  // 측정 끔 → 퍽 평균
  const std::vector<std::array<double, 2>> pucks{{1.06, 1.0}, {1.14, 1.0}};
  f = box_find_from(v, 9.8, map_c, M_PI / 2, pucks, false, 0.205, 0.30, 3, &why);
  ASSERT_TRUE(f.has_value());
  EXPECT_FALSE(f->from_detector);
  EXPECT_NEAR(f->center[0], 1.10, 1e-9);
  // 퍽 없이 측정만 (장소 박스 · 퍽 못 봄) → 측정
  f = box_find_from(v, 9.8, map_c, M_PI / 2, {}, true, 0.205, 0.30, 3, &why);
  ASSERT_TRUE(f.has_value()) << why;
  EXPECT_TRUE(f->from_detector);
  EXPECT_NEAR(f->center[0], 1.10, 1e-9);
  EXPECT_NEAR(std::fabs(fold_quarter_diff(f->yaw, 0.0)), 0.0, 1e-9);
  // 측정 부족 (2 < 3) · 퍽 없음 → 실패 (그 뒤 옆 LOOK)
  const std::vector<BoxSample> few(v.begin(), v.begin() + 2);
  EXPECT_FALSE(box_find_from(few, 9.8, map_c, M_PI / 2, {}, true, 0.205, 0.30, 3, &why).has_value());
  EXPECT_NE(why.find("퍽 못 봄"), std::string::npos);
}

// ================================================================ 신호 기록 · 재검증 묶기 · 박스 방향
namespace
{
// 발제 규칙대로 점멸 (1 s 켜짐 + 1 s 소등, 10 Hz), 앞 · 뒤 소등 따로
void vl_play(SignalRecorder & r, double & t, const std::string & seq)
{
  for (char c : seq) {
    const auto o = c == 'R' ? SignalObs::Red : (c == 'G' ? SignalObs::Green : SignalObs::Yellow);
    for (int i = 0; i < 10; ++i, t += 0.1) {r.observe(t, o);}
    for (int i = 0; i < 10; ++i, t += 0.1) {r.observe(t, SignalObs::Off);}
  }
}
void vl_off(SignalRecorder & r, double & t, double dur)
{
  for (int i = 0; i < static_cast<int>(std::lround(dur / 0.1)); ++i, t += 0.1) {r.observe(t, SignalObs::Off);}
}
void vl_dump(const std::string & name, const std::string & text)
{
  std::ofstream(std::filesystem::temp_directory_path() / ("verifylog_" + name)) << text;   // 손으로 json 검사용
}
}  // namespace

// 11 개 뒤 더 안 옴 → 11 개를 결과로, partial 이력에도 11 개 (색 · 시각)
TEST(VerifyRecord, SignalElevenThenNothingKeepsPartial)
{
  SignalRecorder r;
  double t = 0.0;
  vl_off(r, t, 2.0);
  vl_play(r, t, "RGRRGRGGYGY");
  vl_off(r, t, 3.0);
  ASSERT_EQ(r.partials(), 1);
  ASSERT_EQ(r.partial_history().size(), 1u);
  const auto & h = r.partial_history()[0];
  EXPECT_EQ(h.n, 1);
  EXPECT_EQ(h.sequence, "RGRRGRGGYGY");
  ASSERT_EQ(h.blinks.size(), 11u);
  EXPECT_NEAR(h.blinks[0].start, 2.0, 1e-6);
  EXPECT_NEAR(h.blinks[10].end, 2.0 + 20.0 + 0.9, 1e-6);
  const auto pj = signal_partial_to_json(h, 0.0);
  EXPECT_NE(pj.find("\"kind\": \"signal_partial\""), std::string::npos) << pj;
  EXPECT_NE(pj.find("\"count\": 11"), std::string::npos) << pj;
  EXPECT_NE(pj.find("\"start_s\": 2.00"), std::string::npos) << pj;
  vl_dump("p1.json", pj);
  vl_off(r, t, 20.0);
  r.give_up(t, "signal_wait_s 안에 기록이 안 끝남");
  EXPECT_EQ(r.status(), SignalStatus::Incomplete);
  EXPECT_EQ(r.sequence(), "RGRRGRGGYGY");
  const auto sj = signal_to_json(r, 0.0, true);
  EXPECT_NE(sj.find("\"partials\": [\n    {"), std::string::npos) << sj;
  EXPECT_NE(sj.find("\"sequence\": \"RGRRGRGGYGY\""), std::string::npos) << sj;
  vl_dump("s1.json", sj);
}

// 11 개 → 긴 소등 → 가짜 1 개 → 긴 소등 → 시간 초과. 결과는 마지막 partial (1 개), 앞 11 개는 이력에 남음
//   두 구간을 이어 붙여 12 개로 만들지 않음
TEST(VerifyRecord, SignalElevenThenFakeOneKeepsBoth)
{
  SignalRecorder r;
  double t = 0.0;
  vl_off(r, t, 2.0);
  vl_play(r, t, "RGRRGRGGYGY");
  vl_off(r, t, 3.0);
  vl_play(r, t, "G");
  vl_off(r, t, 3.0);
  ASSERT_EQ(r.partials(), 2);
  ASSERT_EQ(r.partial_history().size(), 2u);
  EXPECT_EQ(r.partial_history()[0].sequence, "RGRRGRGGYGY");
  EXPECT_EQ(r.partial_history()[0].blinks.size(), 11u);
  EXPECT_EQ(r.partial_history()[1].sequence, "G");
  EXPECT_EQ(r.partial_history()[1].n, 2);
  EXPECT_GT(r.partial_history()[1].parked, r.partial_history()[0].parked);
  EXPECT_EQ(r.last_partial(), "G");
  r.give_up(t, "signal_wait_s 안에 기록이 안 끝남");
  EXPECT_EQ(r.status(), SignalStatus::Incomplete);
  EXPECT_EQ(r.sequence(), "G");                // 마지막 partial
  EXPECT_NE(r.sequence().size(), 12u);
  const auto sj = signal_to_json(r, 0.0, true);
  EXPECT_NE(sj.find("\"sequence\": \"RGRRGRGGYGY\""), std::string::npos) << sj;   // 앞 11 개 이력
  EXPECT_NE(sj.find("\"partial\": 2"), std::string::npos) << sj;
  EXPECT_EQ(sj.find("RGRRGRGGYGYG"), std::string::npos) << sj;                    // 이어 붙인 12 개 없음
  EXPECT_NE(sj.find("\"status\": \"INCOMPLETE\""), std::string::npos);
  vl_dump("s2.json", sj);
}

// 12 개 정상 → Done, partial 없음 (빈 목록), 기존 키 그대로
TEST(VerifyRecord, SignalTwelveNormalNoPartials)
{
  SignalRecorder r;
  double t = 0.0;
  vl_off(r, t, 2.0);
  vl_play(r, t, "RGRRGRGGYGYG");
  vl_off(r, t, 3.0);
  EXPECT_EQ(r.status(), SignalStatus::Done);
  EXPECT_TRUE(r.partial_history().empty());
  const auto sj = signal_to_json(r, 0.0, false);
  EXPECT_NE(sj.find("\"sequence\": \"RGRRGRGGYGYG\""), std::string::npos);
  EXPECT_NE(sj.find("\"partials\": [],"), std::string::npos) << sj;
  vl_dump("s3.json", sj);
  // 중간부터 본 시퀀스 (7 개) 뒤 다음 시퀀스 12 개 → Done 은 12 개, 7 개는 이력에만
  SignalRecorder m;
  t = 0.0;
  vl_off(m, t, 1.0);
  vl_play(m, t, "RGGYGYG");
  vl_off(m, t, 3.0);
  vl_play(m, t, "RGRRGRGGYGYG");
  vl_off(m, t, 3.0);
  EXPECT_EQ(m.status(), SignalStatus::Done);
  EXPECT_EQ(m.sequence(), "RGRRGRGGYGYG");
  ASSERT_EQ(m.partial_history().size(), 1u);
  EXPECT_EQ(m.partial_history()[0].sequence, "RGGYGYG");
  vl_dump("s4.json", signal_to_json(m, 0.0, false));
}

// 옮긴 · 돈 박스 기준으로 판정 → 기록 파일 center · cells 가 판정 기준. observed 와 cells 로 다시 잰 오차 = cell_error_mm
//   다른 장소를 나중에 봐도 앞 장소 기록은 그 장소 기준 그대로
TEST(VerifyRecord, VerifyLogUsesJudgeFrame)
{
  auto v = q1_verify();
  const int pl = 2;
  PlaceFrame placed;
  placed.x = v.arena.places[pl].center[0] - 0.039;
  placed.y = v.arena.places[pl].center[1] - 0.003;
  placed.dyaw = 5.0 * M_PI / 180.0;
  placed.n = 3;
  auto obs = v.obs[pl];
  for (size_t k = 0; k < obs.size(); ++k) {
    const auto pt = frame_point(placed, v.plan.places[pl].pip_offsets[k]);
    obs[k].x = pt[0] + 0.004;
    obs[k].y = pt[1] - 0.003;
  }
  DiceMap a = v.arena;
  DicePlan plan = v.plan;
  apply_place_frame(a, plan, pl, placed);
  const auto l3 = judge_place(pl, plan, a, obs);
  EXPECT_TRUE(l3.ok);
  ASSERT_TRUE(l3.has_frame);
  // 판정 중심 = 놓을 때 측정 중심 + 모양 맞춤 이동 (관측이 칸에서 +4 · −3 mm → 맞춤이 그만큼 옮김)
  EXPECT_NEAR(l3.center[0], placed.x + l3.fit_dx, 1e-12);
  EXPECT_NEAR(l3.center[1], placed.y + l3.fit_dy, 1e-12);
  EXPECT_LT(std::hypot(l3.fit_dx, l3.fit_dy), 0.006);
  ASSERT_EQ(l3.cells.size(), v.plan.places[pl].pip_offsets.size());
  const double fcs = std::cos(l3.fit_dyaw), fsn = std::sin(l3.fit_dyaw);
  for (size_t k = 0; k < l3.cells.size(); ++k) {
    // 놓을 때 측정 칸 → 맞춤 (중심 둘레 fit_dyaw 돌리고 fit_dx · fit_dy 옮김)
    const auto pt = frame_point(placed, v.plan.places[pl].pip_offsets[k]);
    const double ox = pt[0] - placed.x, oy = pt[1] - placed.y;
    EXPECT_NEAR(l3.cells[k][0], placed.x + fcs * ox - fsn * oy + l3.fit_dx, 1e-9);
    EXPECT_NEAR(l3.cells[k][1], placed.y + fsn * ox + fcs * oy + l3.fit_dy, 1e-9);
  }
  for (const auto & o : l3.seen) {
    ASSERT_GE(o.cell, 0);
    const auto & c = l3.cells[o.cell];
    EXPECT_NEAR(std::hypot(o.x - c[0], o.y - c[1]), o.cell_error_m, 1e-12);
    EXPECT_LT(o.cell_error_m, 0.006);
  }
  // 그 뒤 L1 을 지도 기준으로 봄 → L3 기록은 측정 기준 그대로
  const auto l1 = judge_place(0, v.plan, v.arena, v.obs[0]);
  const auto r = summarize({l3, l1});
  const auto y = verify_to_yaml(r, v.plan, v.arena, "P2-01", "RGRRGRGGYGYG");
  char buf[96];
  std::snprintf(buf, sizeof(buf), "center: [%.3f, %.3f]", l3.center[0], l3.center[1]);
  EXPECT_NE(y.find(buf), std::string::npos) << y;
  const auto & m3 = v.arena.places[pl].center;
  std::snprintf(buf, sizeof(buf), "map_center: [%.3f, %.3f]", m3[0], m3[1]);
  EXPECT_NE(y.find(buf), std::string::npos) << y;
  std::snprintf(buf, sizeof(buf), "      - [%.3f, %.3f]   # ", l3.cells[0][0], l3.cells[0][1]);
  EXPECT_NE(y.find(buf), std::string::npos) << y;
  const auto & m1 = v.arena.places[0].center;
  std::snprintf(buf, sizeof(buf), "center: [%.3f, %.3f]", m1[0], m1[1]);
  EXPECT_NE(y.find(buf), std::string::npos) << y;
  // 판정 기준이 없는 PlaceVerify → 지도 칸
  PlaceVerify old = l3;
  old.has_frame = false;
  const auto yo = verify_to_yaml(summarize({old}), v.plan, v.arena, "", "");
  std::snprintf(buf, sizeof(buf), "center: [%.3f, %.3f]\n", m3[0], m3[1]);
  EXPECT_NE(yo.find(buf), std::string::npos) << yo;
  EXPECT_EQ(yo.find("map_center"), std::string::npos);
  vl_dump("verify.yaml", y);
}

// 같은 장 두 자리에서 ID 3 → 두 퍽, duplicate_ids 에 3. 여러 장 더 봐도 두 퍽 그대로 (순서 바뀌어도)
TEST(VerifyRecord, SameIdTwoPlacesSameFrameKeptApart)
{
  auto o = [](const std::string & id, const std::string & c, double x, double y, double st) {
      VerifyObs v{id, c, "box", x, y};
      v.stamp = st;
      return v;
    };
  std::vector<VerifyObs> raw;
  raw.push_back(o("3", "red", 0.00, 0.00, 1.0));
  raw.push_back(o("3", "red", 0.10, 0.00, 1.0));
  for (int f = 0; f < 6; ++f) {
    const double st = 1.2 + 0.2 * f;
    const double j = 0.001 * (f % 3);
    if (f % 2) {
      raw.push_back(o("3", "red", 0.10 + j, 0.0, st));
      raw.push_back(o("3", "red", 0.00 + j, 0.0, st));
    } else {
      raw.push_back(o("3", "red", 0.00 + j, 0.0, st));
      raw.push_back(o("3", "red", 0.10 + j, 0.0, st));
    }
    raw.push_back(o("5", "red", 0.05, 0.05, st));
  }
  raw.push_back(o("3", "red", 0.101, 0.0, 3.0));   // 한쪽만 보인 장 → 가까운 퍽에 붙음
  const auto g = group_place_obs(raw, 0.03, 0.7);
  ASSERT_EQ(g.size(), 3u);
  int n3 = 0;
  std::set<int> xs;
  for (const auto & p : g) {
    if (p.id == "3") {++n3; xs.insert(static_cast<int>(std::lround(p.x * 100)));}
  }
  EXPECT_EQ(n3, 2);
  EXPECT_EQ(xs, (std::set<int>{0, 10}));
  // 같은 장, 같은 ID 가 3 cm 안 (한 퍽 두 번 검출) → 한 퍽
  const auto g2 = group_place_obs({o("4", "red", 0.0, 0.0, 1.0), o("4", "red", 0.01, 0.0, 1.0)}, 0.03, 0.7);
  EXPECT_EQ(g2.size(), 1u);
  // 시각 모름 → 같은 ID 는 한 퍽
  EXPECT_EQ(group_place_obs({VerifyObs{"3", "red", "box", 0.0, 0.0}, VerifyObs{"3", "red", "box", 0.1, 0.0}}).size(), 1u);
  // 판정 · 요약으로: L2 (R 4) 에 3 이 두 칸 → duplicate_ids 에 3
  auto v = q1_verify();
  auto l2obs = v.obs[1];
  l2obs[1].id = l2obs[0].id;
  for (auto & x : l2obs) {x.stamp = 7.0;}
  const auto grouped = group_place_obs(l2obs, 0.03, 0.7);
  EXPECT_EQ(grouped.size(), l2obs.size());
  const auto r = summarize({judge_place(1, v.plan, v.arena, grouped)});
  EXPECT_NE(std::find(r.duplicate_ids.begin(), r.duplicate_ids.end(), l2obs[0].id), r.duplicate_ids.end());
}

// ID 가 다 다르면 시각을 넣어도 결과 같음 (GroupObsVotes 입력에 장 시각을 붙여 비교)
TEST(VerifyRecord, UniqueIdsSameAsBefore)
{
  auto o = [](const std::string & id, const std::string & c, const std::string & s, double x) {
      return VerifyObs{id, c, s, x, 0.0};
    };
  std::vector<VerifyObs> raw = {
    o("1", "red", "box", 0.000), o("2", "red", "box", 0.100), o("1", "red", "box", 0.001),
    o("", "red", "box", 0.010), o("2", "red", "cylinder", 0.101), o("", "red", "", 0.200),
    o("", "green", "", 0.205), o("", "red", "", 0.203), o("1", "red", "", 0.002)};
  const auto before = group_place_obs(raw, 0.03, 0.7);
  for (size_t i = 0; i < raw.size(); ++i) {raw[i].stamp = 10.0 + 0.1 * static_cast<double>(i / 2);}   // 장마다 2 개
  const auto after = group_place_obs(raw, 0.03, 0.7);
  ASSERT_EQ(after.size(), before.size());
  for (size_t i = 0; i < after.size(); ++i) {
    EXPECT_EQ(after[i].id, before[i].id);
    EXPECT_EQ(after[i].color, before[i].color);
    EXPECT_EQ(after[i].shape, before[i].shape);
    EXPECT_DOUBLE_EQ(after[i].x, before[i].x);
    EXPECT_DOUBLE_EQ(after[i].y, before[i].y);
  }
  // Q1 전체 (12 개 다른 ID, 한 장) → PASS
  auto v = q1_verify();
  std::vector<PlaceVerify> pv;
  for (int pl = 0; pl < 3; ++pl) {
    auto ob = v.obs[pl];
    for (auto & x : ob) {x.stamp = 3.0;}
    pv.push_back(judge_place(pl, v.plan, v.arena, group_place_obs(ob, 0.03, 0.7)));
  }
  EXPECT_TRUE(summarize(pv).pass);
}

namespace
{
double vl_deg(double d) {return d * M_PI / 180.0;}
std::vector<BoxSample> vl_yaws(const std::vector<double> & deg)
{
  std::vector<BoxSample> v;
  for (size_t i = 0; i < deg.size(); ++i) {v.push_back({1.0 + 0.1 * static_cast<double>(i), 2.0, 1.0, vl_deg(deg[i])});}
  return v;
}
// 비교용 단순 dyaw: nominal 기준으로 접어 중앙값 (±45° 경계에서 틀림)
double vl_old_dyaw(const std::vector<BoxSample> & s, double nominal)
{
  std::vector<double> d;
  for (const auto & x : s) {d.push_back(fold_quarter_diff(x.yaw, nominal));}
  std::sort(d.begin(), d.end());
  const size_t m = d.size() / 2;
  return d.size() % 2 ? d[m] : (d[m - 1] + d[m]) / 2.0;
}
}  // namespace

// ±45° 경계 중앙값. 44/46° 짝수 → 45° (단순 접기는 0°), 3 · 4 · 5 표, 134/136° 같음, 0/90° 동치
TEST(VerifyRecord, PlaceFrameMedianNear45)
{
  const std::array<double, 2> c{2.0, 1.0};
  auto dy = [&](const std::vector<double> & deg, double nominal = 0.0) {
      const auto f = place_frame_from(vl_yaws(deg), 0.0, c, nominal, 0.1, 1);
      EXPECT_TRUE(f.has_value());
      return f ? f->dyaw : 99.0;
    };
  EXPECT_NEAR(vl_old_dyaw(vl_yaws({44, 46, 44, 46}), 0.0), 0.0, 1e-12);   // 단순 접기 반례
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({44, 46, 44, 46}), vl_deg(45))), vl_deg(0.01));
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({44, 46, 44}), vl_deg(44))), vl_deg(0.01));
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({46, 44, 46, 46}), vl_deg(46))), vl_deg(0.01));
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({44, 46, 44, 46, 45}), vl_deg(45))), vl_deg(0.01));
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({134, 136, 134, 136}), vl_deg(45))), vl_deg(0.01));
  EXPECT_LT(std::fabs(fold_quarter_diff(dy({-44, -46, 136, 134}), vl_deg(45))), vl_deg(0.01));
  EXPECT_NEAR(dy({1, 91, -179, 271}), vl_deg(1), 1e-12);   // 0/90° 동치
  for (double d : {dy({44, 46, 44, 46}), dy({134, 136}), dy({-44, -46})}) {   // 계약: ±45° 안
    EXPECT_GT(d, -M_PI / 4 - 1e-12);
    EXPECT_LE(d, M_PI / 4 + 1e-12);
  }
  // nominal 이 돌아 있어도 (다시 찾기 박스 기준) 같은 답
  EXPECT_LT(std::fabs(fold_quarter_diff(vl_deg(30) + dy({74, 76, 74, 76}, vl_deg(30)), vl_deg(75))), vl_deg(0.01));
}

// 0° 근처 (nominal ± 20° 안) 는 단순 접기와 비트까지 같음. box_find_from 도 같음
TEST(VerifyRecord, PlaceFrameNearZeroIdentical)
{
  const std::array<double, 2> c{2.0, 1.0};
  unsigned s = 12345;
  auto rnd = [&s]() {s = s * 1103515245u + 12345u; return static_cast<double>((s >> 8) & 0xFFFF) / 65535.0;};
  for (double nominal : {0.0, 0.3, -1.2, M_PI / 2 + 0.2, M_PI}) {
    for (int trial = 0; trial < 200; ++trial) {
      const int n = 1 + trial % 8;
      const double spread = trial < 100 ? 3.0 : 20.0;
      std::vector<BoxSample> v;
      for (int i = 0; i < n; ++i) {
        const double yaw = nominal + vl_deg((2 * rnd() - 1) * spread) + (trial % 4) * M_PI / 2;
        v.push_back({1.0 + 0.1 * i, 2.0 + 0.001 * i, 1.0, yaw});
      }
      const auto f = place_frame_from(v, 0.0, c, nominal, 0.1, 1);
      ASSERT_TRUE(f.has_value());
      EXPECT_EQ(f->dyaw, vl_old_dyaw(v, nominal)) << nominal << " " << trial;
      // box_find_from: 그 ref 기준 단순 접기 중앙값과 같음
      const auto b = box_find_from(v, 0.0, c, nominal, {}, true, 0.205, 0.1, 1);
      ASSERT_TRUE(b.has_value());
      double s4 = 0.0, c4 = 0.0;
      for (const auto & x : v) {s4 += std::sin(4.0 * x.yaw); c4 += std::cos(4.0 * x.yaw);}
      const double ref = nominal + fold_quarter_diff(std::atan2(s4, c4) / 4.0, nominal);
      EXPECT_EQ(b->yaw, ref + vl_old_dyaw(v, ref)) << nominal << " " << trial;
    }
  }
  // box_find_from 45° 경계 (짝수 표) 도 45°
  const auto b = box_find_from(vl_yaws({44, 46, 44, 46}), 0.0, c, 0.0, {}, true, 0.205, 0.1, 1);
  ASSERT_TRUE(b.has_value());
  EXPECT_LT(std::fabs(fold_quarter_diff(b->yaw, vl_deg(45))), vl_deg(0.01));
}

// 재검증 모양 맞춤: 다시 방문할 때 위치 추정이 틀어져 퍽 전체가 옮겨지고 돌아 보여도 눈 모양이 맞으면 칸 위
TEST(DiceVerify, FitAbsorbsRevisitDrift)
{
  auto v = q1_verify();
  const int pl = 2;   // G 6
  const auto & c = v.arena.places[pl].center;
  const double th = 6.0 * M_PI / 180.0, cs = std::cos(th), sn = std::sin(th);
  auto obs = v.obs[pl];
  for (auto & o : obs) {
    const double x = o.x - c[0], y = o.y - c[1];
    o.x = c[0] + cs * x - sn * y + 0.028;
    o.y = c[1] + sn * x + cs * y - 0.020;
  }
  VerifyFitParams no_fit;
  no_fit.enable = false;
  EXPECT_FALSE(judge_place(pl, v.plan, v.arena, obs, 0.03, 0.0, no_fit).ok);
  const auto pv = judge_place(pl, v.plan, v.arena, obs);
  EXPECT_TRUE(pv.ok);
  EXPECT_NEAR(std::remainder(pv.fit_dyaw - th, M_PI), 0.0, 0.02);   // 6 = 두 열이라 180° 돈 것과 같음
  for (const auto & o : pv.seen) {EXPECT_LT(o.cell_error_m, 0.01) << o.id;}
}

// 한 퍽만 모양 밖으로 14 cm 밀림 → 그 퍽만 칸 밖 (맞춤이 그 퍽 쪽으로 끌려가 다른 퍽까지 떨어뜨리거나 덮어 주지 않음)
TEST(DiceVerify, FitKeepsSingleOutlier)
{
  auto v = q1_verify();
  const int pl = 2;
  auto obs = v.obs[pl];
  obs[3].x += 0.10;
  obs[3].y += 0.10;
  const auto pv = judge_place(pl, v.plan, v.arena, obs);
  EXPECT_FALSE(pv.ok);
  int off = 0;
  for (const auto & o : pv.seen) {
    if (o.cell < 0) {++off; EXPECT_EQ(o.id, obs[3].id);}
  }
  EXPECT_EQ(off, 1);
  EXPECT_LT(std::hypot(pv.fit_dx, pv.fit_dy), 0.01);
}

// 모양이 기준 (박스) 에서 max_shift 넘게 떨어져야 맞으면 박스 위가 아님 → 그만큼 못 옮김, 칸 밖 퍽 남음
//   (칸 몇 개만 겹치는 작은 이동은 찾을 수 있으나 눈 모양 전체는 안 맞음)
TEST(DiceVerify, FitRejectsFarShift)
{
  auto v = q1_verify();
  const int pl = 1;   // R 4
  auto obs = v.obs[pl];
  for (auto & o : obs) {o.x += 0.15;}
  const auto pv = judge_place(pl, v.plan, v.arena, obs);
  EXPECT_FALSE(pv.ok);
  EXPECT_LE(std::hypot(pv.fit_dx, pv.fit_dy), 0.10 + 1e-9);
  int on = 0;
  for (const auto & o : pv.seen) {on += o.cell >= 0;}
  EXPECT_LT(on, 4);
  VerifyFitParams far;
  far.max_shift_m = 0.20;   // 범위를 넓히면 그대로 맞음 (= 막은 건 범위)
  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, obs, 0.03, 0.0, far).ok);
  const auto y = verify_to_yaml(summarize({pv}), v.plan, v.arena, "P2-01", "");
  EXPECT_NE(y.find("frame_source: map"), std::string::npos);
  EXPECT_NE(y.find("fit: {used: "), std::string::npos);
}

// allocation.json: 기록부가 나중 관측으로 색을 바꿔도 계획 값 그대로, 다른 관측은 seen_as
TEST(DiceRecord, AllocationKeepsPlannedKind)
{
  auto v = q1_verify();
  AllocationRecord r;
  r.barcode = "P2-01";
  r.pucks.push_back(AllocPuck{"9", "red", "box", "placed", "green/box"});
  r.pucks.push_back(AllocPuck{"6", "red", "box", "placed", ""});
  const auto j = allocation_to_json(r, v.plan, v.arena);
  EXPECT_NE(j.find("\"id\": \"9\", \"color\": \"red\""), std::string::npos);
  EXPECT_NE(j.find("\"seen_as\": \"green/box\""), std::string::npos);
  EXPECT_NE(j.find("\"seen_as\": \"\""), std::string::npos);
}

// 재검증 그림: 위에서 내려다본 카메라로 본 퍽 · 칸 투영, 맞음 = 초록 · 문제 = 빨강 (bbox 상자 · 없으면 중심 ×), 위 띠
TEST(VerifyImage, ProjectsAndDrawsJudgement)
{
  auto v = q1_verify();
  const int pl = 1;   // R 4
  auto obs = v.obs[pl];
  const auto & c = v.arena.places[pl].center;
  {   // 하나 칸 밖: 박스 중심 쪽으로 8 cm (화면 안, 위 띠에 안 가림)
    const double dx = c[0] - obs[0].x, dy = c[1] - obs[0].y, n = std::hypot(dx, dy);
    obs[0].x += 0.08 * dx / n;
    obs[0].y += 0.08 * dy / n;
  }
  VerifyFitParams no_fit;
  no_fit.enable = false;
  const auto pv = judge_place(pl, v.plan, v.arena, obs, 0.05, 0.0, no_fit);
  ASSERT_FALSE(pv.ok);
  const double top = v.arena.places[pl].top_z + 0.03;
  VerifyCamera cam;   // 장소 중심 0.30 m 위에서 바로 아래 (광학 x = map x, y = −map y, z = −map z)
  cam.fx = cam.fy = 600.0;
  cam.cx = 320.0;
  cam.cy = 240.0;
  cam.R = {1, 0, 0, 0, -1, 0, 0, 0, -1};
  cam.t = {-c[0], c[1], top + 0.30};
  const auto p0 = project_map_point(cam, c[0], c[1], top);
  ASSERT_TRUE(p0.has_value());
  EXPECT_NEAR(p0->x, 320.0, 1e-6);
  EXPECT_NEAR(p0->y, 240.0, 1e-6);
  const auto p1 = project_map_point(cam, c[0] + 0.03, c[1], top);   // 3 cm = 600 · 0.03 / 0.30 = 60 px
  EXPECT_NEAR(p1->x - p0->x, 60.0, 1e-6);
  EXPECT_FALSE(project_map_point(cam, c[0], c[1], top + 0.40).has_value());   // 카메라 뒤

  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_verify(img, pv, cam, top);
  auto has_color = [&](const cv::Rect & r, int ch) {   // 사각 테두리 위에 그 색 (BGR 채널) 이 셈
      int n = 0;
      for (int x = r.x; x < r.x + r.width; ++x) {
        for (int y : {r.y, r.y + r.height - 1}) {
          if (x < 0 || y < 0 || x >= img.cols || y >= img.rows) {continue;}
          const auto px = img.at<cv::Vec3b>(y, x);
          n += px[ch] > 150 && px[ch] > px[(ch + 1) % 3] + 60;
        }
      }
      return n > 8;
    };
  // bbox 없는 퍽 = 투영 중심 × (중심 둘레 15 × 15 px 안에 그 색)
  for (const auto & o : pv.seen) {
    const auto p = *project_map_point(cam, o.x, o.y, top);
    const int ch = o.ok ? 1 : 2;
    int n = 0;
    for (int y = static_cast<int>(p.y) - 7; y <= static_cast<int>(p.y) + 7; ++y) {
      for (int x = static_cast<int>(p.x) - 7; x <= static_cast<int>(p.x) + 7; ++x) {
        if (x < 0 || y < 0 || x >= img.cols || y >= img.rows) {continue;}
        const auto px = img.at<cv::Vec3b>(y, x);
        n += px[ch] > 150 && px[ch] > px[(ch + 1) % 3] + 60;
      }
    }
    EXPECT_GT(n, 8) << o.id;
  }
  // bbox 있는 퍽 = 그 상자 테두리
  {
    auto pb = pv;
    pb.seen[1].bbox = {100.0f, 300.0f, 160.0f, 380.0f};
    cv::Mat img3(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    std::swap(img, img3);
    draw_verify(img, pb, cam, top);
    EXPECT_TRUE(has_color(cv::Rect(100, 300, 60, 80), pb.seen[1].ok ? 1 : 2));
    std::swap(img, img3);
  }
  EXPECT_GT(cv::sum(img.rowRange(0, 48))[0], 0.0);   // 위 띠 글자

  VerifyCamera no_tf = cam;
  no_tf.fx = 0.0;   // TF 없음 → 위 띠만
  cv::Mat img2(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_verify(img2, pv, no_tf, top);
  EXPECT_EQ(cv::sum(img2.rowRange(60, 440))[1], 0.0);   // 위 띠 · 아래 띠 (개수 다름) 밖은 안 그림
}

// 방향 무관: 계획과 90° 돈 배치 · 대각선 반대 (좌우 반전) 배치도 주사위 모양이면 맞음, 칸 허용 5 cm
TEST(DiceVerify, FitIgnoresDiceOrientation)
{
  auto v = q1_verify();
  for (int pl = 0; pl < 3; ++pl) {
    const auto & c = v.arena.places[pl].center;
    for (const double deg : {90.0, 180.0, -37.0}) {
      const double th = deg * M_PI / 180.0, cs = std::cos(th), sn = std::sin(th);
      auto obs = v.obs[pl];
      for (auto & o : obs) {
        const double x = o.x - c[0], y = o.y - c[1];
        o.x = c[0] + cs * x - sn * y + 0.02;
        o.y = c[1] + sn * x + cs * y;
      }
      EXPECT_TRUE(judge_place(pl, v.plan, v.arena, obs).ok) << "L" << pl + 1 << " " << deg;
    }
    auto mir = v.obs[pl];   // 장소 중심 지나는 x 축 거울
    for (auto & o : mir) {o.y = 2 * c[1] - o.y;}
    EXPECT_TRUE(judge_place(pl, v.plan, v.arena, mir).ok) << "L" << pl + 1 << " 거울";
  }
  // 칸 허용: 한 퍽 4.5 cm 어긋남은 맞음, 다른 모양 (2 눈 대각선 → 옆으로 나란히) 은 안 맞음
  auto obs = v.obs[0];   // L1 Y 2
  ASSERT_EQ(obs.size(), 2u);
  auto near = obs;
  near[1].x += 0.045 * 0.7071;
  near[1].y += 0.045 * 0.7071;
  EXPECT_TRUE(judge_place(0, v.plan, v.arena, near).ok);
  const auto & c0 = v.arena.places[0].center;
  auto row = obs;   // 3 눈 칸 간격 (대각 0.17 m) 을 나란히 0.06 m 로
  row[0].x = c0[0] - 0.03; row[0].y = c0[1];
  row[1].x = c0[0] + 0.03; row[1].y = c0[1];
  EXPECT_FALSE(judge_place(0, v.plan, v.arena, row).ok);
}

// 박스 위: 모양은 맞아도 퍽 중심이 잰 윗면 밖이면 그 퍽 불합격 (떨어짐), 맞춘 칸도 윗면 밖으로 안 나감
TEST(DiceVerify, PucksMustStayOnBox)
{
  auto v = q1_verify();
  const int pl = 1;   // R 4 (눈 ±0.06)
  const auto & c = v.arena.places[pl].center;
  VerifyBox box;
  box.x = c[0];
  box.y = c[1];
  box.yaw = v.arena.map_from_arena.yaw;
  box.source = "remeasured";
  EXPECT_NEAR(box_inner_dist(box, c[0], c[1]), 0.1025, 1e-9);

  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, v.obs[pl], 0.05, 0.0, {}, box).ok);   // 제자리면 맞음
  // 모양 그대로 박스 축으로 6 cm 밀림 → 바깥 줄 퍽 중심 0.12 m > 0.1025 = 떨어짐
  const double ax = std::cos(box.yaw), ay = std::sin(box.yaw);
  auto obs = v.obs[pl];
  for (auto & o : obs) {o.x += 0.06 * ax; o.y += 0.06 * ay;}
  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, obs).ok);   // 박스 모르면 모양만 → 맞음
  const auto pv = judge_place(pl, v.plan, v.arena, obs, 0.05, 0.0, {}, box);
  EXPECT_FALSE(pv.ok);
  int off = 0;
  for (const auto & o : pv.seen) {
    if (o.on_box == 0) {++off; EXPECT_FALSE(o.ok);}
  }
  EXPECT_EQ(off, 2);
  for (const auto & q : pv.cells) {EXPECT_GE(box_inner_dist(box, q[0], q[1]), -1e-9);}   // 맞춘 칸은 윗면 안
  const auto y = verify_to_yaml(summarize({pv}), v.plan, v.arena, "P2-01", "");
  EXPECT_NE(y.find("on_box: false"), std::string::npos);
  EXPECT_NE(y.find("box: {source: remeasured"), std::string::npos);
}

// ---------------------------------------------------------------- STORE 실패 뒤 쥔 퍽 다시 계획 (B1)
namespace
{
CargoItem ff_item(const std::string & id, const std::string & color = "red", const std::string & shape = "cylinder")
{
  CargoItem it;
  it.puck_id = id;
  it.color = color;
  it.shape = shape;
  return it;
}

// 실제 cargo_slots 순서 · 칸 제약 (RetrieveKeepsRackRules 와 같음), S1 찬 상태에서 퍽 4 를 쥠 → 첫 칸 S2
Cargo ff_rack_holding()
{
  Cargo cargo({"S1", "S2", "S3", "S8", "S11", "S6", "S12", "S7", "S10", "S5", "S9", "S4"});
  cargo.set_rules({{"S1", "S2"}, {"S9", "S4"}, {"S10", "S5"}, {"S11", "S6"}, {"S12", "S7"}}, {{"S1", "S2"}, {"S7", "S12"}, {"S10", "S5"}});
  EXPECT_TRUE(cargo.on_pick(ff_item("1")));
  EXPECT_TRUE(cargo.on_store("S1"));
  EXPECT_TRUE(cargo.on_pick(ff_item("4")));
  return cargo;
}
}  // namespace

TEST(HeldPuckStore, NoFailureKeepsOldChoice)
{
  const Cargo cargo = ff_rack_holding();
  ASSERT_EQ(cargo.free_slot_for(*cargo.gripper()), std::optional<std::string>("S2"));
  for (int alt : {0, 1, 3}) {
    const auto c = held_puck_choice(cargo, {}, alt, true);
    EXPECT_EQ(c.kind, HeldChoice::Kind::Store);
    EXPECT_EQ(c.slot, "S2");
  }
  // store_alt_cells 0 = 실패한 칸이어도 같은 칸 다시
  const auto old = held_puck_choice(cargo, {"S2"}, 0, true);
  EXPECT_EQ(old.kind, HeldChoice::Kind::Store);
  EXPECT_EQ(old.slot, "S2");
  // 그리퍼 빔 = None
  Cargo empty({"S1", "S2"});
  EXPECT_EQ(held_puck_choice(empty, {"S2"}, 1, true).kind, HeldChoice::Kind::None);
}

TEST(HeldPuckStore, RepeatedRejectChoosesDifferentCell)
{
  const Cargo cargo = ff_rack_holding();
  const auto c = held_puck_choice(cargo, {"S2"}, 1, true);
  ASSERT_EQ(c.kind, HeldChoice::Kind::Store);
  EXPECT_NE(c.slot, "S2");
  EXPECT_TRUE(cargo.store_ok(c.slot));
  bool empty_slot = false;
  for (const auto & [name, content] : cargo.slots()) {
    if (name == c.slot) {empty_slot = !content;}
  }
  EXPECT_TRUE(empty_slot);
  EXPECT_EQ(c.slot, "S3");   // 목록 다음 빈 칸 (제약 지킴)
  // 더 많이 허용하면 실패한 칸 둘을 다 빼고 또 다른 칸
  const auto c2 = held_puck_choice(cargo, {"S2", "S3"}, 2, true);
  ASSERT_EQ(c2.kind, HeldChoice::Kind::Store);
  EXPECT_NE(c2.slot, "S2");
  EXPECT_NE(c2.slot, "S3");
  EXPECT_TRUE(cargo.store_ok(c2.slot));
}

TEST(HeldPuckStore, PutBackAfterAltCellsUsed)
{
  const Cargo cargo = ff_rack_holding();
  const auto c = held_puck_choice(cargo, {"S2", "S3"}, 1, true);   // 다른 칸 하나까지 해 봄 → 되돌림
  EXPECT_EQ(c.kind, HeldChoice::Kind::PutBack);
  EXPECT_TRUE(c.slot.empty());
  EXPECT_EQ(held_puck_choice(cargo, {"S2", "S3"}, 1, false).kind, HeldChoice::Kind::None);   // 되돌림 못 함
}

TEST(HeldPuckStore, PutBackWhenNoAllowedCell)
{
  // S2 실패, 남은 빈 칸 S12 는 S7 이 차서 넣기 제약에 걸림 → 제약 없는 첫 빈 칸 (free_slot 끝) 을 쓰지 않고 되돌림
  Cargo cargo({"S2", "S7", "S12"});
  cargo.set_rules({{"S12", "S7"}}, {{"S7", "S12"}});
  ASSERT_TRUE(cargo.on_pick(ff_item("9", "green")));
  ASSERT_TRUE(cargo.on_store("S7"));
  ASSERT_TRUE(cargo.on_pick(ff_item("4")));
  ASSERT_FALSE(cargo.store_ok("S12"));
  const auto c = held_puck_choice(cargo, {"S2"}, 3, true);
  EXPECT_EQ(c.kind, HeldChoice::Kind::PutBack);
  // 빈 칸이 아예 없음
  Cargo full({"S2", "S3"});
  ASSERT_TRUE(full.on_pick(ff_item("5")));
  ASSERT_TRUE(full.on_store("S3"));
  ASSERT_TRUE(full.on_pick(ff_item("4")));
  EXPECT_EQ(held_puck_choice(full, {"S2"}, 1, true).kind, HeldChoice::Kind::PutBack);
  EXPECT_EQ(held_puck_choice(full, {"S2"}, 1, false).kind, HeldChoice::Kind::None);
}

TEST(HeldPuckStore, HeldFirstStepsStoreOrPutBack)
{
  Step plan;
  plan.type = StepType::Custom;
  plan.kind = kPlanCollectBox;
  box_of(plan) = 1;
  plan.name = "plan:collect:B1:again";
  const CargoItem held = ff_item("4");

  HeldChoice st;
  st.kind = HeldChoice::Kind::Store;
  st.slot = "S3";
  auto s = held_first_steps(st, held, plan, true);
  ASSERT_EQ(s.size(), 3u);
  EXPECT_EQ(s[0].arm_task, kArmLookRack);
  EXPECT_EQ(s[0].slot, "S3");
  EXPECT_EQ(s[1].arm_task, kArmStore);
  EXPECT_EQ(s[1].slot, "S3");
  EXPECT_EQ(s[1].puck_id, "4");
  EXPECT_TRUE(is_custom(s[2], kPlanCollectBox));
  EXPECT_EQ(box_of(s[2]), 1);
  EXPECT_EQ(held_first_steps(st, held, plan, false).size(), 2u);   // LOOK_RACK 끔

  HeldChoice pb;
  pb.kind = HeldChoice::Kind::PutBack;
  s = held_first_steps(pb, held, plan, true);
  ASSERT_EQ(s.size(), 2u);
  EXPECT_TRUE(is_putback(s[0]));
  EXPECT_EQ(s[0].arm_task, kArmPlace);
  EXPECT_EQ(s[0].target, Target::Custom);
  EXPECT_EQ(s[0].puck_id, "4");
  EXPECT_FALSE(is_dice_cell(s[0]));   // 장소 칸 · 놓은 수에 안 셈
  EXPECT_EQ(s[0].name, "arm:PUTBACK:4");
  EXPECT_TRUE(is_custom(s[1], kPlanCollectBox));
  EXPECT_FALSE(is_putback(s[1]));

  EXPECT_TRUE(held_first_steps(HeldChoice{}, held, plan, true).empty());
  // 보통 PLACE 는 되돌림 아님
  ArmAction pl;
  pl.op = ArmOp::Place;
  pl.puck_id = "4";
  pl.place = 0;
  pl.cell = 0;
  const auto ps = arm_steps({pl});
  ASSERT_EQ(ps.size(), 1u);
  EXPECT_FALSE(is_putback(ps[0]));
  EXPECT_TRUE(is_dice_cell(ps[0]));
}

// 재계획 흐름 흉내: S2 거절 → 다른 칸 → 그 칸도 거절 → 되돌림 (같은 칸 6 번 반복 안 함)
TEST(HeldPuckStore, RejectSequenceNeverRepeatsSameCell)
{
  const Cargo cargo = ff_rack_holding();
  std::set<std::string> failed;
  std::vector<std::string> tried;
  HeldChoice c = held_puck_choice(cargo, failed, 1, true);
  for (int i = 0; i < 6 && c.kind == HeldChoice::Kind::Store; ++i) {
    tried.push_back(c.slot);
    failed.insert(c.slot);   // 매번 거절
    c = held_puck_choice(cargo, failed, 1, true);
  }
  EXPECT_EQ(tried, (std::vector<std::string>{"S2", "S3"}));
  EXPECT_EQ(c.kind, HeldChoice::Kind::PutBack);
}

// DELIVER (칸에서 꺼낸 퍽) 은 되돌릴 박스 자리 없음 → 다른 칸만, 다 쓰면 None. 실패 없으면 보통 칸
TEST(HeldPuckStore, DeliverHeldPuckNoPutBack)
{
  const Cargo cargo = ff_rack_holding();
  const auto first = held_puck_choice(cargo, {}, 1, false);   // 포기 · 시간 부족 때 첫 손 비우기 = 보통 고르기
  EXPECT_EQ(first.kind, HeldChoice::Kind::Store);
  EXPECT_EQ(first.slot, "S2");
  const auto alt = held_puck_choice(cargo, {"S2"}, 1, false);
  EXPECT_EQ(alt.kind, HeldChoice::Kind::Store);
  EXPECT_EQ(alt.slot, "S3");
  EXPECT_EQ(held_puck_choice(cargo, {"S2", "S3"}, 1, false).kind, HeldChoice::Kind::None);
  // 손 비우기 실패가 이어지면 칸이 하나씩 빠지고 끝남 (끝없이 돌지 않음): S2 → S3 → (COLLECT) 되돌림
  std::set<std::string> failed;
  std::vector<std::string> seq;
  for (int i = 0; i < 20; ++i) {
    const auto c = held_puck_choice(cargo, failed, 1, true);
    if (c.kind != HeldChoice::Kind::Store) {seq.push_back(c.kind == HeldChoice::Kind::PutBack ? "PUTBACK" : "NONE"); break;}
    seq.push_back(c.slot);
    failed.insert(c.slot);
  }
  EXPECT_EQ(seq, (std::vector<std::string>{"S2", "S3", "PUTBACK"}));
}

// ---------------------------------------------------------------- 그리퍼 실측 → 손 상태 → 재고 불확실 스스로 정합
namespace
{
// 가짜 /joint_states: t0 부터 dt 간격으로 q 값들
std::deque<HandSample> fake_js(double t0, double dt, const std::vector<double> & qs)
{
  std::deque<HandSample> d;
  for (std::size_t i = 0; i < qs.size(); ++i) {d.push_back(HandSample{t0 + dt * static_cast<double>(i), qs[i]});}
  return d;
}
}  // namespace

TEST(HandReconcile, GapFormulaMatchesControl)
{
  const HandConfig c;
  EXPECT_NEAR(hand_gap_mm(0.0046, c), 34.8, 0.05);   // control 로그: 그리퍼 0.0046 m → 34.8 mm
  EXPECT_NEAR(hand_gap_mm(0.0043, c), 34.2, 0.05);
  EXPECT_EQ(classify_hand(0.0046, c), HandState::Holding);    // 네모
  EXPECT_EQ(classify_hand(0.0040, c), HandState::Holding);    // 원기둥 33.5 근처
  EXPECT_EQ(classify_hand(-0.0016, c), HandState::Empty);     // 빈손 닫힘 (gripper_close)
  EXPECT_EQ(classify_hand(0.0003, c), HandState::Empty);      // grasp_empty_m
  EXPECT_EQ(classify_hand(0.0164, c), HandState::Empty);      // 열림 (gripper_open)
  EXPECT_EQ(classify_hand(0.0009, c), HandState::Unknown);    // 닫힘과 퍽 사이
  EXPECT_EQ(classify_hand(0.0085, c), HandState::Unknown);    // 퍽보다 조금 넓음
  EXPECT_EQ(classify_hand(NAN, c), HandState::Unknown);
}

TEST(HandReconcile, ReadHandNeedsStableFresh)
{
  const HandConfig c;   // stable 0.5 s · fresh 0.3 s
  // 10 Hz · 0.7 s 쥠 → Holding
  auto js = fake_js(10.0, 0.1, std::vector<double>(8, 0.0046));
  auto r = read_hand(js, 10.75, c);
  EXPECT_EQ(r.state, HandState::Holding);
  EXPECT_NEAR(r.gap_mm, 34.8, 0.05);
  EXPECT_EQ(r.n, 8);
  EXPECT_NE(r.text.find("34.8 mm"), std::string::npos);
  // 오래됨 (마지막 1 s 전) → 모름
  EXPECT_EQ(read_hand(js, 11.7, c).state, HandState::Unknown);
  // 막 바뀜 (빈손 0.2 s 만) → 모름
  js = fake_js(10.0, 0.1, {0.0046, 0.0046, 0.0046, 0.0046, 0.0046, -0.0016, -0.0016, -0.0016});
  EXPECT_EQ(read_hand(js, 10.75, c).state, HandState::Unknown);
  // 빈손 0.6 s → Empty
  js = fake_js(10.0, 0.1, {0.0046, -0.0016, -0.0016, -0.0016, -0.0016, -0.0016, -0.0016, -0.0016});
  EXPECT_EQ(read_hand(js, 10.75, c).state, HandState::Empty);
  // 열림 0.6 s → Empty
  js = fake_js(10.0, 0.1, std::vector<double>(7, 0.0164));
  EXPECT_EQ(read_hand(js, 10.65, c).state, HandState::Empty);
  // 사이 값 → 모름, 표본 없음 → 모름
  EXPECT_EQ(read_hand(fake_js(10.0, 0.1, std::vector<double>(8, 0.0009)), 10.75, c).state, HandState::Unknown);
  EXPECT_EQ(read_hand({}, 10.0, c).state, HandState::Unknown);
}

// FAULT 중 그리퍼 실측 정합 (inventory_auto) 이 PLACE 빈손 = Done 으로 정하면 놓은 수 · 칸 기록도 갱신해야 함
//   예전: 그 경로만 on_place_confirmed 를 안 불러 재계획이 같은 칸 (0) 에 또 놓음. 세 정합 경로가 이 조건 하나를 씀
TEST(HandReconcile, AutoPlaceDoneUpdatesPlacement)
{
  using planning::DoubtFix;
  std::string why;
  const auto fix = choose_doubt_fix(kArmPlace, HandState::Empty, false, &why);
  ASSERT_TRUE(fix.has_value());
  EXPECT_TRUE(place_confirmed_by(*fix, kArmPlace));
  EXPECT_FALSE(place_confirmed_by(DoubtFix::Before, kArmPlace));
  EXPECT_FALSE(place_confirmed_by(DoubtFix::Removed, kArmPlace));
  for (uint8_t t : {kArmPick, kArmStore, kArmRetrieve, kArmLookRack}) {
    EXPECT_FALSE(place_confirmed_by(DoubtFix::Done, t));
  }
}

TEST(HandReconcile, DoubtFixPerTaskAndHand)
{
  using planning::DoubtFix;
  std::string why;
  // PICK
  EXPECT_EQ(choose_doubt_fix(kArmPick, HandState::Holding, false, &why), DoubtFix::Done);
  EXPECT_EQ(choose_doubt_fix(kArmPick, HandState::Empty, false, &why), DoubtFix::Before);
  // RETRIEVE
  EXPECT_EQ(choose_doubt_fix(kArmRetrieve, HandState::Holding, false, &why), DoubtFix::Done);
  EXPECT_EQ(choose_doubt_fix(kArmRetrieve, HandState::Empty, true, &why), DoubtFix::Before);
  // STORE
  EXPECT_EQ(choose_doubt_fix(kArmStore, HandState::Holding, true, &why), DoubtFix::Before);
  EXPECT_EQ(choose_doubt_fix(kArmStore, HandState::Empty, true, &why), DoubtFix::Done);
  EXPECT_EQ(choose_doubt_fix(kArmStore, HandState::Empty, false, &why), DoubtFix::Removed);
  EXPECT_NE(why.find("놓기 단계 못 봄"), std::string::npos);
  // PLACE
  EXPECT_EQ(choose_doubt_fix(kArmPlace, HandState::Holding, false, &why), DoubtFix::Before);
  EXPECT_EQ(choose_doubt_fix(kArmPlace, HandState::Empty, false, &why), DoubtFix::Done);
  // LOOK_RACK
  EXPECT_EQ(choose_doubt_fix(kArmLookRack, HandState::Holding, false, &why), DoubtFix::Before);
  EXPECT_EQ(choose_doubt_fix(kArmLookRack, HandState::Empty, false, &why), DoubtFix::Removed);
  // 모름 · 재고 작업 아님 → 안 정함
  for (uint8_t t : {kArmPick, kArmStore, kArmPlace, kArmRetrieve}) {
    EXPECT_FALSE(choose_doubt_fix(t, HandState::Unknown, true, &why).has_value());
  }
  EXPECT_FALSE(choose_doubt_fix(kArmHome, HandState::Empty, false, &why).has_value());
}

// 고른 정합이 실제 재고 전환 (planning resolve_doubt) 과 맞는지: 가짜 실측 → 판단 → 정합 → 그리퍼 · 칸
TEST(HandReconcile, ResolveWithFakeJointStates)
{
  using planning::DoubtFix;
  const HandConfig c;
  auto doubt_store = [](Cargo & cg) {
      ASSERT_TRUE(cg.on_pick(ff_item("4")));
      ASSERT_TRUE(cg.set_doubt(planning::make_doubt(cg, kArmStore, "4", "S2", CargoItem{}, "arm:STORE:4>S2", "하드웨어 이상")));
    };
  struct Case {std::vector<double> qs; bool release; bool holding; bool in_s2;};
  const std::vector<Case> cases{
    {std::vector<double>(8, 0.0046), false, true, false},    // 쥠 → before: 손에 그대로
    {std::vector<double>(8, -0.0016), true, false, true},    // 빈손 + 놓기 봄 → done: S2
    {std::vector<double>(8, -0.0016), false, false, false},  // 빈손 + 놓기 못 봄 → removed: 손 · 칸 둘 다 아님
  };
  for (const auto & k : cases) {
    Cargo cg({"S1", "S2", "S3"});
    doubt_store(cg);
    const auto h = read_hand(fake_js(5.0, 0.1, k.qs), 5.75, c);
    const auto fix = choose_doubt_fix(kArmStore, h.state, k.release, nullptr);
    ASSERT_TRUE(fix.has_value());
    std::string id, st, err;
    ASSERT_TRUE(planning::resolve_doubt(cg, *fix, &id, &st, &err)) << err;
    EXPECT_FALSE(cg.doubt().has_value());
    EXPECT_EQ(cg.holding(), k.holding);
    bool s2 = false;
    for (const auto & [n, it] : cg.slots()) {if (n == "S2" && it) {s2 = true;}}
    EXPECT_EQ(s2, k.in_s2);
  }
  // PICK 빈손 → before (그리퍼 빔 그대로), PICK 쥠 → done (그리퍼에 퍽)
  for (bool hold : {false, true}) {
    Cargo cg({"S1", "S2"});
    CargoItem it = ff_item("7");
    ASSERT_TRUE(cg.set_doubt(planning::make_doubt(cg, kArmPick, "7", "", it, "arm:PICK:7", "하드웨어 이상")));
    const auto h = read_hand(fake_js(5.0, 0.1, std::vector<double>(8, hold ? 0.0046 : -0.0016)), 5.75, c);
    const auto fix = choose_doubt_fix(kArmPick, h.state, false, nullptr);
    ASSERT_TRUE(fix.has_value());
    std::string id, st, err;
    ASSERT_TRUE(planning::resolve_doubt(cg, *fix, &id, &st, &err)) << err;
    EXPECT_EQ(cg.holding(), hold);
  }
}

// 적재함 여유 칸: 11 개 실은 상태 + 박스 위 마지막 퍽 1 개 → 그 퍽이 점수에 필요 없으면 안 실음 (한 칸 남김), 필요하면 실음 (마지막 칸 씀)
TEST(HandReconcile, SpareCellKeptWhenPuckNotNeeded)
{
  const PuckKind k[11] = {
    PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox, PuckKind::RedBox,
    PuckKind::RedCylinder, PuckKind::RedCylinder, PuckKind::RedCylinder,
    PuckKind::GreenBox, PuckKind::GreenBox, PuckKind::GreenCylinder, PuckKind::GreenCylinder};
  Cargo cargo(slot_names(12));
  for (int i = 0; i < 11; ++i) {
    ASSERT_TRUE(cargo.on_pick(CargoItem{std::to_string(i), kind_color(k[i]), kind_shape(k[i]), "", false}));
    ASSERT_TRUE(cargo.on_store(*cargo.free_slot()));
  }
  TransferCosts costs;
  ASSERT_TRUE(load_transfer_costs(TEST_TRAVEL_TABLE, costs));
  SolverInput base;
  base.position = 6;
  base.stopped = true;
  base.allow_leftover = true;
  base.place_group = {{'G', 'Y', 'R'}};
  ASSERT_TRUE(fill_carried(cargo, base));
  base.on_boxes = {TransferPuck{"11", PuckKind::RedCylinder, 6}};   // 마지막 퍽 = 빨강 원기둥
  std::string why;
  // 수요 G2 · Y4 · R6 = 12: 빨강 원기둥 없이는 R6 + Y4 (모양 하나) 를 다 못 채움 → 필요 → 마지막 칸 씀
  EXPECT_FALSE(puck_not_needed(base, "11", {{2, 4, 6}}, costs, cargo, {{0, 0, 0}}, 4, &why)) << why;
  // 수요 G2 · R6 만 (Y 0): 실은 빨강 7 개로 충분 → 필요 없음 → 한 칸 남김
  EXPECT_TRUE(puck_not_needed(base, "11", {{2, 0, 6}}, costs, cargo, {{0, 0, 0}}, 4, &why)) << why;
  EXPECT_NE(why.find("안 실으면"), std::string::npos);
  // 박스 위에 없는 퍽 → false (그대로 실음)
  EXPECT_FALSE(puck_not_needed(base, "99", {{2, 0, 6}}, costs, cargo, {{0, 0, 0}}, 4, &why));
  EXPECT_EQ(kRackSpareCells, 1);
}

// 칸 ID 판정 + 박스 밖 거르기 (퍽 4 · 5 · 7 제자리 + 박스 모서리 오검출 초록 둘 (가장자리 −6 · +4 mm) → 개수에 안 넣음)
TEST(DiceVerify, CellIdsAndIgnoreBoxEdge)
{
  auto v = q1_verify();
  const int pl = 1;   // R 4, 칸 ID 0 · 4 · 8 · 10
  const auto & c = v.arena.places[pl].center;
  VerifyBox box;
  box.x = c[0];
  box.y = c[1];
  box.yaw = v.arena.map_from_arena.yaw;
  box.source = "remeasured";
  VerifyFitParams no_fit;
  no_fit.enable = false;
  const auto ids = v.ids[pl];
  // 박스 축 (u, v) 점 → map
  const double ca = std::cos(box.yaw), sa = std::sin(box.yaw);
  auto at = [&](double u, double w) {return std::array<double, 2>{c[0] + ca * u - sa * w, c[1] + sa * u + ca * w};};

  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, v.obs[pl], 0.05, 0.0, no_fit, box, ids, 0.015).ok);   // 제자리

  // 모서리 오검출 둘 (ID 모름) → 15 mm 안쪽이 아니라 뺌 → 합격, 기록에 ignored
  auto obs = v.obs[pl];
  const auto e1 = at(0.1085, 0.095), e2 = at(0.0985, 0.090);   // 가장자리 −6 · +4 mm
  obs.push_back({"", "green", "cylinder", e1[0], e1[1]});
  obs.push_back({"", "green", "cylinder", e2[0], e2[1]});
  EXPECT_FALSE(judge_place(pl, v.plan, v.arena, obs, 0.05, 0.0, no_fit, box, ids, 0.0).ok);   // 안 거르면 개수 6 ≠ 4
  const auto pv = judge_place(pl, v.plan, v.arena, obs, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_TRUE(pv.ok);
  EXPECT_EQ(pv.seen.size(), 4u);
  EXPECT_EQ(pv.ignored.size(), 2u);
  const auto y = verify_to_yaml(summarize({pv}), v.plan, v.arena, "P2-01", "");
  EXPECT_NE(y.find("ignored:"), std::string::npos);
  EXPECT_NE(y.find("# ID 0"), std::string::npos);

  // 같은 색 퍽 둘이 칸만 바뀜 → 그룹 판정은 합격, 칸 ID 판정은 불합격
  auto sw = v.obs[pl];
  std::swap(sw[0].id, sw[1].id);
  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, sw, 0.05, 0.0, no_fit, box).ok);
  EXPECT_FALSE(judge_place(pl, v.plan, v.arena, sw, 0.05, 0.0, no_fit, box, ids, 0.015).ok);

  // 칸 위 퍽 ID 못 읽음 → 불합격
  auto un = v.obs[pl];
  un[2].id.clear();
  const auto pu = judge_place(pl, v.plan, v.arena, un, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_FALSE(pu.ok);

  // 계획 칸에서 박스 중심 쪽으로 6 cm → 불합격, 4 cm → 합격 (박스 중심 기준 칸, 맞춤 없음)
  const double dx = c[0] - v.obs[pl][0].x, dy = c[1] - v.obs[pl][0].y, dn = std::hypot(dx, dy);
  auto far = v.obs[pl];
  far[0].x += 0.06 * dx / dn;
  far[0].y += 0.06 * dy / dn;
  const auto pf = judge_place(pl, v.plan, v.arena, far, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_FALSE(pf.ok);
  EXPECT_EQ(pf.ignored.size(), 0u);
  auto near = v.obs[pl];
  near[0].x += 0.04 * dx / dn;
  near[0].y += 0.04 * dy / dn;
  EXPECT_TRUE(judge_place(pl, v.plan, v.arena, near, 0.05, 0.0, no_fit, box, ids, 0.015).ok);

  // 안 놓은 칸 (부분 채우기) → 그 칸 비어 있음 불합격
  auto part = ids;
  part[3].clear();
  auto po = v.obs[pl];
  po.pop_back();
  const auto pp = judge_place(pl, v.plan, v.arena, po, 0.05, 0.0, no_fit, box, part, 0.015);
  EXPECT_FALSE(pp.ok);
  EXPECT_EQ(pp.seen.size(), 3u);
}

// 놓은 퍽 ID 와 같은 검출은 가장자리 15 mm 규칙에서 안 뺌 (중심이 윗면 밖일 때만 뺌)
//   예: 퍽 6 이 칸에서 36 mm · 가장자리 7 mm 안쪽 → 빼면 4 개 다 놓았는데 불합격
TEST(DiceVerify, PlacedIdNearBoxEdgeIsCounted)
{
  auto v = q1_verify();
  const int pl = 1;   // R 4
  const auto & c = v.arena.places[pl].center;
  VerifyBox box;
  box.x = c[0];
  box.y = c[1];
  box.yaw = v.arena.map_from_arena.yaw;
  box.source = "placed";
  VerifyFitParams no_fit;
  no_fit.enable = false;
  const auto ids = v.ids[pl];
  const double ca = std::cos(box.yaw), sa = std::sin(box.yaw);
  // obs[0] 을 박스 축에서 바깥으로 밀어 가장자리 edge_mm 안쪽에 둠
  auto to_edge = [&](std::vector<VerifyObs> obs, double edge_m) {
    const double rx = obs[0].x - c[0], ry = obs[0].y - c[1];
    double u = ca * rx + sa * ry, w = -sa * rx + ca * ry;
    if (std::fabs(u) >= std::fabs(w)) {u = std::copysign(box.half - edge_m, u);} else {w = std::copysign(box.half - edge_m, w);}
    obs[0].x = c[0] + ca * u - sa * w;
    obs[0].y = c[1] + sa * u + ca * w;
    return obs;
  };

  // 가장자리 7 mm 안쪽 (칸에서 약 36 mm) · 놓은 ID → 셈 → 합격
  const auto in7 = to_edge(v.obs[pl], 0.007);
  const auto p7 = judge_place(pl, v.plan, v.arena, in7, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_TRUE(p7.ok);
  EXPECT_EQ(p7.ignored.size(), 0u);
  EXPECT_EQ(p7.seen.size(), 4u);

  // 같은 자리인데 ID 모름 · 이 장소에 안 놓은 ID → 지금대로 뺌 → 그 칸 비어 불합격
  auto noid = in7;
  noid[0].id.clear();
  const auto pn = judge_place(pl, v.plan, v.arena, noid, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_FALSE(pn.ok);
  EXPECT_EQ(pn.ignored.size(), 1u);
  auto other = in7;
  other[0].id = "99";
  const auto po = judge_place(pl, v.plan, v.arena, other, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_FALSE(po.ok);
  EXPECT_EQ(po.ignored.size(), 1u);

  // 놓은 ID 라도 중심이 윗면 밖 (5 mm 밖) → 뺌 → 불합격
  const auto out5 = to_edge(v.obs[pl], -0.005);
  const auto pout = judge_place(pl, v.plan, v.arena, out5, 0.05, 0.0, no_fit, box, ids, 0.015);
  EXPECT_FALSE(pout.ok);
  EXPECT_EQ(pout.ignored.size(), 1u);
}

// 칸 ID 판정 + 다시 본 색 · 모양: R 은 색만, Y 는 모양 통일 (모르면 불합격), 기록과 다르면 이유에 기록 값
TEST(DiceVerify, CellIdsCheckSeenColorShape)
{
  auto v = q1_verify();
  VerifyFitParams no_fit;
  no_fit.enable = false;
  // L2 = R 4
  {
    const int pl = 1;
    const auto ids = v.ids[pl];
    std::vector<std::array<std::string, 2>> kinds;
    for (const auto & o : v.obs[pl]) {kinds.push_back({o.color, o.shape});}
    EXPECT_TRUE(judge_place(pl, v.plan, v.arena, v.obs[pl], 0.05, 0.0, no_fit, std::nullopt, ids, 0.0, kinds).ok);
    auto sh = v.obs[pl];
    sh[0].shape = sh[0].shape == "box" ? "cylinder" : "box";   // R 은 모양 안 봄
    EXPECT_TRUE(judge_place(pl, v.plan, v.arena, sh, 0.05, 0.0, no_fit, std::nullopt, ids, 0.0, kinds).ok);
    auto col = v.obs[pl];
    col[1].color = "green";
    const auto pc = judge_place(pl, v.plan, v.arena, col, 0.05, 0.0, no_fit, std::nullopt, ids, 0.0, kinds);
    EXPECT_FALSE(pc.ok);
    ASSERT_FALSE(pc.issues.empty());
    EXPECT_NE(pc.issues[0].find("기록 red/"), std::string::npos) << pc.issues[0];
    auto unk = v.obs[pl];
    unk[2].color.clear();
    EXPECT_FALSE(judge_place(pl, v.plan, v.arena, unk, 0.05, 0.0, no_fit, std::nullopt, ids, 0.0, kinds).ok);
  }
  // L1 = Y 2 (빨강 상자 둘)
  {
    const int pl = 0;
    const auto ids = v.ids[pl];
    const auto ok = judge_place(pl, v.plan, v.arena, v.obs[pl], 0.05, 0.0, no_fit, std::nullopt, ids, 0.0);
    EXPECT_TRUE(ok.ok);
    EXPECT_EQ(ok.y_shape, v.obs[pl][0].shape);
    auto mix = v.obs[pl];
    mix[1].shape = mix[0].shape == "box" ? "cylinder" : "box";   // 1 : 1 = 섞임 → 둘 다 불합격
    const auto pm = judge_place(pl, v.plan, v.arena, mix, 0.05, 0.0, no_fit, std::nullopt, ids, 0.0);
    EXPECT_FALSE(pm.ok);
    EXPECT_TRUE(pm.y_shape.empty());
    auto unk = v.obs[pl];
    unk[1].shape.clear();
    const auto pu = judge_place(pl, v.plan, v.arena, unk, 0.05, 0.0, no_fit, std::nullopt, ids, 0.0);
    EXPECT_FALSE(pu.ok);
    EXPECT_TRUE(pu.seen[0].ok);   // 아는 퍽은 그대로 합격
    const auto y = verify_to_yaml(summarize({pu}), v.plan, v.arena, "P2-01", "");
    EXPECT_NE(y.find("y_shape:"), std::string::npos);
  }
}

// ---------------------------------------------------------------- 깊이 틀린 장 때문에 맞게 놓은 퍽이 FAIL
//   실물 L1 (Y5): 퍽 7 이 칸 3 (박스 로봇 쪽 모서리 근처) 에 맞게 놓였지만 far LOOK 마지막 장 3 개가 깊이 점 1 % →
//   깊이 0.22 · 0.73 m → 광선 따라 11 cm 밖 (−3.141, 0.056) → 가장자리 −68 mm 로 '뺌' + 그 퍽의 ID 없는 움직이던 장이 따로 남아 'ID 못 읽음'
#include "verify_l1_depth_outlier_obs.inc"

namespace
{
struct DepthOutlierCase
{
  DicePlan plan;
  DiceMap arena;
  VerifyBox box;
  std::vector<std::string> ids{"11", "0", "3", "7", "5"};
};

DepthOutlierCase depth_outlier_case()
{
  DepthOutlierCase c;
  // 재검증 기록 (verify.yaml): 다시 잰 박스 중심 (−3.310, 0.017) · yaw 181.9°, 칸 (맞춤 없음, 박스 기준)
  const std::array<double, 2> ctr{{-3.310, 0.017}};
  const std::vector<std::array<double, 2>> cells{{{-3.248, -0.041}}, {{-3.368, -0.045}}, {{-3.310, 0.017}}, {{-3.252, 0.079}},
    {{-3.372, 0.075}}};
  c.plan.places[0].group = 'Y';
  c.plan.places[0].pips = 5;
  for (const auto & k : cells) {c.plan.places[0].pip_offsets.push_back({k[0] - ctr[0], k[1] - ctr[1]});}
  c.arena.places[0].center = ctr;
  c.arena.places[0].view_dir = {0.0, -1.0};   // 재검증 정차 (−3.333, 0.269, −89.3°): 로봇이 −y 쪽 박스를 봄
  c.box.x = ctr[0];
  c.box.y = ctr[1];
  c.box.yaw = 181.9 * M_PI / 180.0;
  c.box.source = "remeasured";
  return c;
}

std::vector<VerifyObs> depth_outlier_raw(bool with_z = true)
{
  std::vector<VerifyObs> raw;
  for (const auto & r : kDepthOutlierL1) {
    VerifyObs o{r.id, r.color, r.shape, r.x, r.y};
    o.stamp = 1794653700.0 + r.t;
    o.bbox = {r.b[0], r.b[1], r.b[2], r.b[3]};
    if (with_z) {
      o.z = r.z;
      o.cam = {r.cx, r.cy, r.cz};
    }
    raw.push_back(o);
  }
  return raw;
}

PlaceVerify depth_outlier_judge(const DepthOutlierCase & c, const std::vector<VerifyObs> & raw, double plane_tol)
{
  VerifyFitParams no_fit;
  no_fit.enable = false;   // 노드: 박스 측정 있으면 맞춤 안 함
  return judge_place(0, c.plan, c.arena, group_place_obs(raw, 0.03, 0.7, plane_tol), 0.05, 952.4, no_fit, c.box, c.ids, 0.015);
}

const VerifyObs * find_id(const std::vector<VerifyObs> & v, const std::string & id)
{
  for (const auto & o : v) {
    if (o.id == id) {return &o;}
  }
  return nullptr;
}
}  // namespace

// plane_tol 0 (거르기 끔) = 실물 기록 그대로: FAIL, 퍽 7 뺌 (−68 mm), ID 없는 퍽 (−3.259, 0.069) 칸에서 12 mm
TEST(VerifyDepthOutlier, NoFilterReproducesFail)
{
  const auto c = depth_outlier_case();
  const auto pv = depth_outlier_judge(c, depth_outlier_raw(), 0.0);
  EXPECT_FALSE(pv.ok);
  ASSERT_EQ(pv.ignored.size(), 1u);
  EXPECT_EQ(pv.ignored[0].id, "7");
  EXPECT_NEAR(pv.ignored[0].x, -3.141, 0.002);
  EXPECT_NEAR(pv.ignored[0].y, 0.056, 0.002);
  EXPECT_NEAR(pv.ignored[0].edge_m, -0.068, 0.002);
  // 실물 기록은 ID 없는 퍽 (−3.259, 0.069) 하나 더 셈 (5 개). 묶기를 평균 자리로 하면 그 장들은
  //   퍽 7 ID 장 대부분과 같은 자리라 퍽 7 에 합쳐짐 → 4 개
  EXPECT_EQ(pv.seen.size(), 4u);
  EXPECT_EQ(find_id(pv.seen, ""), nullptr);
  // z 를 모르면 거르기 없이 같은 판정
  const auto pn = depth_outlier_judge(c, depth_outlier_raw(false), 0.03);
  EXPECT_FALSE(pn.ok);
  EXPECT_EQ(pn.ignored.size(), 1u);
}

// 새 판정: 깊이 틀린 장 (윗면 높이 ± 30 mm 밖) 은 위치로 안 씀 → 퍽 7 은 칸 3, ID 없는 장은 퍽 7 에 합쳐짐 → PASS
TEST(VerifyDepthOutlier, DepthOutlierPasses)
{
  const auto c = depth_outlier_case();
  const auto g = group_place_obs(depth_outlier_raw(), 0.03, 0.7, 0.03);
  EXPECT_EQ(g.size(), 5u);   // 퍽 0 · 3 · 5 · 7 · 11, ID 없는 퍽 남지 않음
  const auto * p7 = find_id(g, "7");
  ASSERT_NE(p7, nullptr);
  EXPECT_EQ(p7->off_plane_n, 3);   // 마지막 장 3 개 (z 0.22 · −0.32 · −0.32)
  EXPECT_TRUE(p7->pos_from_earlier);
  EXPECT_FALSE(p7->all_off_plane);
  EXPECT_FLOAT_EQ(p7->bbox[1], 364.0f);   // 그림 상자는 마지막 장 (그림과 같은 장)
  const auto pv = depth_outlier_judge(c, depth_outlier_raw(), 0.03);
  EXPECT_TRUE(pv.ok) << ::testing::PrintToString(pv.issues);
  EXPECT_EQ(pv.seen.size(), 5u);
  EXPECT_TRUE(pv.ignored.empty());
  for (const auto & o : pv.seen) {
    EXPECT_TRUE(o.ok) << o.id;
    EXPECT_LT(o.cell_error_m, 0.03) << o.id;
    EXPECT_GT(o.edge_m, 0.015) << o.id;
  }
  const auto * s7 = find_id(pv.seen, "7");
  ASSERT_NE(s7, nullptr);
  EXPECT_EQ(s7->cell, 3);
  const auto y = verify_to_yaml(summarize({pv}), c.plan, c.arena, "P2-02", "");
  EXPECT_NE(y.find("depth_bad_frames: 3"), std::string::npos);
}

// 반대: 퍽 7 이 정말 박스 밖 바닥에 떨어짐 (모든 장이 바닥 높이, 일부는 깊이도 틀림) → FAIL 유지
TEST(VerifyDepthOutlier, RealDropStillFails)
{
  const auto c = depth_outlier_case();
  auto raw = depth_outlier_raw();
  auto is7 = [](const VerifyObs & o) {return o.id == "7" || (o.id.empty() && o.x > -3.29 && o.y > 0.03);};
  int n7 = 0;
  for (auto & o : raw) {
    if (!is7(o)) {continue;}
    // 칸 3 에서 박스 밖으로 (로봇 왼쪽 앞, +x) 떨어져 바닥 위 (윗면 높이 − 박스 높이 약 0.07 m)
    o.x = -3.165;
    o.y = 0.085;
    o.z = 0.035;
    if (++n7 % 4 == 0) {o.z = -0.3; o.x = -3.10;}   // 바닥 퍽도 깊이가 가끔 틀림
  }
  ASSERT_GT(n7, 10);
  const auto g = group_place_obs(raw, 0.03, 0.7, 0.03);
  const auto * p7 = find_id(g, "7");
  ASSERT_NE(p7, nullptr);
  EXPECT_TRUE(p7->all_off_plane);   // 맞는 장 없음 → 마지막 장 그대로 (바닥)
  const auto pv = depth_outlier_judge(c, raw, 0.03);
  EXPECT_FALSE(pv.ok);
  bool unseen7 = false;
  for (const auto & s : pv.issues) {
    unseen7 |= s.find("L1 가까운 줄 왼쪽 (퍽 7 놓은 자리): 박스 위에서 퍽 7 이 안 보임 (박스 가장자리 밖") != std::string::npos;
  }
  EXPECT_TRUE(unseen7) << ::testing::PrintToString(pv.issues);
  ASSERT_FALSE(pv.ignored.empty());
  EXPECT_LT(pv.ignored[0].edge_m, 0.0);

  // 퍽 7 이 아예 없음 (장 다 뺌) → FAIL (개수 4 ≠ 5, 칸 3 빔)
  std::vector<VerifyObs> no7;
  for (const auto & o : depth_outlier_raw()) {
    if (!is7(o)) {no7.push_back(o);}
  }
  const auto pe = depth_outlier_judge(c, no7, 0.03);
  EXPECT_FALSE(pe.ok);
  EXPECT_EQ(pe.seen.size(), 4u);

  // 퍽 7 이 박스 위지만 칸에서 6 cm 밀림 (모든 장, 깊이 맞음) → 칸 밖 FAIL (가려내기가 놓기 오차를 숨기지 않음)
  auto shifted = depth_outlier_raw();
  for (auto & o : shifted) {
    if (is7(o)) {o.y -= 0.060;}
  }
  EXPECT_FALSE(depth_outlier_judge(c, shifted, 0.03).ok);
}

// 재검증 문구 = 칸 번호 대신 로봇이 볼 때 자리 (실물 L1 판정 문구)
TEST(VerifyWording, DepthOutlierSpotsAndIssues)
{
  const auto c = depth_outlier_case();
  // 퍽 7 은 깊이 틀린 장 (박스 밖) 만 남겨 ID 없는 퍽이 따로 남게 함 (ID 못 읽음 문구 보려고)
  auto raw = depth_outlier_raw();
  raw.erase(std::remove_if(raw.begin(), raw.end(), [](const VerifyObs & o) {return o.id == "7" && o.x < -3.2;}), raw.end());
  const auto pv = depth_outlier_judge(c, raw, 0.0);
  const std::vector<std::string> want{"먼 줄 왼쪽", "먼 줄 오른쪽", "가운데", "가까운 줄 왼쪽", "가까운 줄 오른쪽"};
  EXPECT_EQ(pv.cell_spots, want);
  const std::vector<std::string> want_en{"far-left", "far-right", "center", "near-left", "near-right"};
  EXPECT_EQ(pv.cell_spots_en, want_en);
  const std::vector<std::string> issues{
    "L1 가까운 줄 왼쪽 (퍽 7 놓은 자리): 박스 위에서 퍽 7 이 안 보임 (박스 가장자리 밖 68 mm 에 퍽 7 검출 있음, 박스 밖이라 판정에서 뺌)",
    "L1 가까운 줄 왼쪽 자리 근처 (12 mm) 에 빨강 원기둥 이 보이는데 ID 를 못 읽음"};
  EXPECT_EQ(pv.issues, issues);
  const auto r = summarize({pv});
  const auto y = verify_to_yaml(r, c.plan, c.arena, "P2-02", "");
  EXPECT_NE(y.find("# ID 7 · 가까운 줄 왼쪽"), std::string::npos);
  EXPECT_NE(y.find("spot: far-left"), std::string::npos);
}

// 빈 자리 · 개수 · 자리에서 벗어남 · 어디서도 안 보임 문구
TEST(VerifyWording, EmptyCountFarMissing)
{
  auto c = depth_outlier_case();
  c.ids = {"11", "0", "", "7", "5"};   // 가운데 자리는 퍽 모자라 안 놓음
  auto raw = depth_outlier_raw();
  raw.erase(std::remove_if(raw.begin(), raw.end(), [](const VerifyObs & o) {return o.id == "3" || o.id == "7" || o.id.empty();}),
    raw.end());
  for (auto & o : raw) {
    if (o.id == "11") {o.x -= 0.075;}   // 먼 줄 왼쪽 퍽 11 을 박스 가운데 쪽으로 7.5 cm 밀어 둠 (박스 안, 이웃 자리에서도 5 cm 밖)
  }
  const auto pv = depth_outlier_judge(c, raw, 0.03);
  auto has = [&pv](const std::string & t) {
      return std::any_of(pv.issues.begin(), pv.issues.end(), [&t](const std::string & s) {return s.find(t) != std::string::npos;});
    };
  EXPECT_TRUE(has("L1 가운데 자리: 놓을 퍽이 모자라 비워 둔 자리")) << ::testing::PrintToString(pv.issues);
  EXPECT_TRUE(has("L1 먼 줄 왼쪽: 퍽 11 이 자리에서 ")) << ::testing::PrintToString(pv.issues);
  EXPECT_TRUE(has("L1 가까운 줄 왼쪽 (퍽 7 놓은 자리): 박스 위에서 퍽 7 이 안 보임")) << ::testing::PrintToString(pv.issues);
  EXPECT_TRUE(has("L1 퍽 3 개 보임, 눈 5 개 필요 (놓은 퍽 4 개)")) << ::testing::PrintToString(pv.issues);
  PlaceVerify p2 = pv, p3 = pv;
  p2.place = 1;
  p3.place = 2;
  const auto r = summarize({pv, p2, p3});
  EXPECT_NE(std::find(r.issues.begin(), r.issues.end(), "퍽 1: 세 장소 어디에서도 안 보임"), r.issues.end());
}

// 깊이 틀린 장이 없으면 plane_tol 이 결과를 안 바꿈
TEST(VerifyDepthOutlier, NoOutlierSameAsBefore)
{
  auto raw = depth_outlier_raw();
  raw.erase(std::remove_if(raw.begin(), raw.end(), [](const VerifyObs & o) {return std::fabs(o.z - 0.105) > 0.03;}), raw.end());
  const auto a = group_place_obs(raw, 0.03, 0.7, 0.0);
  const auto b = group_place_obs(raw, 0.03, 0.7, 0.03);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].id, b[i].id);
    EXPECT_DOUBLE_EQ(a[i].x, b[i].x);
    EXPECT_DOUBLE_EQ(a[i].y, b[i].y);
    EXPECT_EQ(b[i].off_plane_n, 0);
  }
}
