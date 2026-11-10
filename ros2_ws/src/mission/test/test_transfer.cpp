// test_transfer.cpp — mission 운반 계획 시험, ROS 없이: transfer_solver · transfer_planner · dice_map · 재검증 · 기록 파일
//   실행: colcon build → source install/setup.bash → ./build/mission/test_transfer

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/dice_steps.hpp"
#include "mission/dice_record.hpp"
#include "mission/dice_verify.hpp"
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

// 박스 정차만 마지막 접근 (dice_map box_approach_m), 멈출 면 거리 = 정차 ↔ 중심 0.2935 − 반 변 0.1025
TEST(DiceSteps, BoxStopsUseFinalApproach)
{
  DiceMap a;
  ASSERT_TRUE(load_dice_map(TEST_DICE_MAP, a));
  a.box_approach_m = 0.10;   // 저장소 설정은 꺼 둠 (0) — 켰을 때 값으로 시험
  int boxes = 0;
  for (const auto & s : collect_steps(a, {1, 2, 6, 3, 4, 5})) {
    if (s.type != StepType::Navigate) {continue;}
    ++boxes;
    EXPECT_DOUBLE_EQ(s.approach_m, a.box_approach_m) << s.name;
    EXPECT_NEAR(s.approach_face_m, 0.2575 - 0.1025, 1e-6) << s.name;   // 정차 거리 0.2575 m
  }
  EXPECT_EQ(boxes, 6);
  a.box_approach_m = 0.0;   // 끄면 예전과 같음
  for (const auto & s : collect_steps(a, {1})) {EXPECT_DOUBLE_EQ(s.approach_m, 0.0);}
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
  // L2 (R 4): 한 개 5 cm 밀림 → 칸 밖
  v.obs[1][0].x += 0.05;
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
  r.pucks.push_back({"3", "green", "cylinder", "skipped:B2"});
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

// 중앙값 · 먼 측정 버림 · 옛 측정 안 씀 · 개수 부족
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
  // L3 G3 에 꼭 갈 초록 사각 S1 퍽 0 · S7 퍽 5 가 S2 · S12 에 막힘 (옛 펼치기 = 같은 종류 뒤 칸만 → 막힌 채 뺌)
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
