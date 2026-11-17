// test_arm_stall_retreat.cpp — 팔 실행기 · 하드웨어 감시의 시각 문제: 주기 멈춤 · 보낸 시각 기한 · 시각 앞섬 · 도중 정지 HOME 되돌아가기
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm.hpp"
#include "manipulation/arm_path.hpp"
#include "manipulation/placed_memory.hpp"

using manipulation::ArmAction;
using manipulation::ArmCommand;
using manipulation::ArmEnd;
using manipulation::ArmPath;
using manipulation::ArmRunner;
using manipulation::HardwareWatch;
using manipulation::PathNode;

namespace
{
std::vector<ArmAction> two_moves()
{
  return {{"approach", false, "pick_above", 0.0}, {"lift", false, "pick_above", 0.0}};
}

bool has(const std::vector<ArmCommand> & out, ArmCommand::Kind k)
{
  for (const auto & c : out) {if (c.kind == k) {return true;}}
  return false;
}

unsigned sent_gen(const std::vector<ArmCommand> & out)
{
  for (const auto & c : out) {if (c.kind == ArmCommand::Kind::SEND) {return c.gen;}}
  return 0;
}
}  // namespace

// ① 실제 로봇 사례 재현: 다시 계획 (2.2 s) 이 tick 시각을 묵혀 SEND → 받아 주기 기한 2 s 가 보내기 전에 지남
//   끄면: 다음 tick 에서 '받아 주지 않음'. 주기 멈춤 늦추기 (set_stall) 만으로도 · 보낸 시각 (mark_sent) 만으로도 안 끊김
TEST(ArmStall, StaleTickSendIsNotAbandoned)
{
  for (int mode = 0; mode < 3; ++mode) {   // 0 = 끔, 1 = set_stall, 2 = mark_sent
    ArmRunner r(2.0, 5.0, 1.0);
    if (mode == 1) {r.set_stall(0.25);}
    r.start(two_moves());
    double t = 10.0;
    auto out = r.tick(t, true);   // 첫 동작 보냄
    const unsigned g1 = sent_gen(out);
    ASSERT_NE(g1, 0u);
    r.on_goal_response(g1, true, t + 0.001);
    r.on_result(g1, true);
    // 이 주기: 다시 계획 2.2 s (tick 에 넘긴 시각은 계획 전 10.01 그대로)
    out = r.tick(10.01, true);
    const unsigned g2 = sent_gen(out);
    ASSERT_NE(g2, 0u);
    const double real = 12.21;   // 실제로 보내는 시각
    if (mode == 2) {EXPECT_TRUE(r.mark_sent(g2, real));}
    out = r.tick(real + 0.01, true);
    if (mode == 0) {
      EXPECT_TRUE(has(out, ArmCommand::Kind::CANCEL_ACTIVE));   // 끔: 받아 주기 전에 포기
    } else {
      EXPECT_FALSE(has(out, ArmCommand::Kind::CANCEL_ACTIVE)) << mode;
      EXPECT_TRUE(r.busy());
      EXPECT_EQ(r.on_goal_response(g2, true, real + 0.02), manipulation::LateGoal::USE);   // 1 ms 뒤 받아들임 = 정상 진행
    }
  }
}

// ② 진짜로 안 받아 주면 여전히 끊김 (늦추기만, 한계 없앰 아님) · 지난 세대 mark_sent 는 아무것도 안 함
TEST(ArmStall, RealAcceptTimeoutStillFails)
{
  ArmRunner r(2.0, 5.0, 1.0);
  r.set_stall(0.25);
  r.start(two_moves());
  auto out = r.tick(0.0, true);
  const unsigned g = sent_gen(out);
  EXPECT_TRUE(r.mark_sent(g, 0.05));
  EXPECT_FALSE(r.mark_sent(g + 7, 0.05));
  bool abandoned = false;
  double t_ab = 0.0;
  for (double t = 0.01; t < 3.0 && !abandoned; t += 0.01) {
    out = r.tick(t, true);
    abandoned = has(out, ArmCommand::Kind::CANCEL_ACTIVE);
    t_ab = t;
  }
  EXPECT_TRUE(abandoned);
  EXPECT_NEAR(t_ab, 2.06, 0.015);   // 보낸 시각 0.05 + 2 s
  ASSERT_TRUE(has(out, ArmCommand::Kind::FINISH));   // 같은 tick 에 실패로 끝
  EXPECT_FALSE(r.mark_sent(g, 3.2));   // 작업 끝 → 안 바꿈
}

// ③ 주기가 멈춘 동안 (재생기 결과 · 측정이 못 들어옴) 끝나기 기한 · 접힘 확인 기한도 늦춤
TEST(ArmStall, MotionAndVerifyDeadlinesShiftByGap)
{
  ArmRunner r(2.0, 1.0, 1.0, 0.5);
  r.set_stall(0.25);
  r.start({{"retreat", false, "home", 0.0}});
  auto out = r.tick(0.0, true);
  const unsigned g = sent_gen(out);
  r.on_goal_response(g, true, 0.01);   // 끝나기 기한 1.01
  r.tick(0.02, true);
  out = r.tick(3.0, true);              // 2.98 s 멈춤 → 기한 + 2.98
  EXPECT_FALSE(has(out, ArmCommand::Kind::CANCEL_ACTIVE));
  EXPECT_GT(r.last_stall_s(), 2.9);
  r.on_result(g, true);
  out = r.tick(3.01, true);             // VERIFY 시작 (기한 3.51)
  EXPECT_TRUE(has(out, ArmCommand::Kind::VERIFY_START));
  out = r.tick(5.0, true, false);       // 멈춤 1.99 s → 기한 5.5
  EXPECT_FALSE(has(out, ArmCommand::Kind::FINISH));
  out = r.tick(5.02, true, true);
  ASSERT_TRUE(has(out, ArmCommand::Kind::FINISH));
  EXPECT_TRUE(out.back().drive_ok);
}

// ④ 끄면 (set_stall 0) 주기가 멈춘 시간도 그대로 세어 시간 초과 → 취소
TEST(ArmStall, OffKeepsOldTimeout)
{
  ArmRunner r(2.0, 1.0, 1.0);
  r.start(two_moves());
  auto out = r.tick(0.0, true);
  r.on_goal_response(sent_gen(out), true, 0.0);
  out = r.tick(3.0, true);
  EXPECT_TRUE(has(out, ArmCommand::Kind::CANCEL_ACTIVE));
}

// ⑤ 하드웨어 감시: 다른 스레드가 판정 시각 뒤 측정 · 답을 넣음 (로그 "잠깐 이상 (정상)" · "(보드 이상: )" 의 원인)
TEST(HardwareWatch, FutureStampWithinToleranceIsHealthy)
{
  HardwareWatch w(0.3, 0.1, 1.0);
  const double t = 100.0;
  w.observe(t + 0.003, true);
  w.observe_components(t + 0.002, true, "");
  EXPECT_FALSE(w.healthy(t));   // 허용 0
  EXPECT_NE(w.status(t).find("앞섬"), std::string::npos);
  w.set_future_tolerance(0.2);
  EXPECT_TRUE(w.healthy(t));
  EXPECT_EQ(w.status(t), "정상");
  // 측정이 끊기면 여전히 이상 (늦음은 그대로 봄)
  EXPECT_FALSE(w.healthy(t + 0.5));
  // 많이 앞선 시각 (시계 이상) 은 이상
  HardwareWatch w2(0.3, 0.1, 1.0);
  w2.set_future_tolerance(0.2);
  w2.observe(t + 1.0, true);
  w2.observe_components(t, true, "");
  EXPECT_FALSE(w2.healthy(t));
  // NaN · 음수는 0
  w2.set_future_tolerance(std::nan(""));
  w2.observe(t + 1.001, true);
  EXPECT_FALSE(w2.healthy(t + 1.0));
}

// ⑥ 지나온 길 기록: 일직선 dt 점은 줄이고 꺾인 점은 남김
TEST(ArmTrace, AppendKeepsCornersOnly)
{
  std::vector<std::vector<double>> tr;
  for (int i = 0; i <= 10; ++i) {manipulation::trace_append(tr, {0.1 * i, 0, 0, 0, 0});}
  ASSERT_EQ(tr.size(), 2u);
  EXPECT_NEAR(tr.back()[0], 1.0, 1e-9);
  for (int i = 1; i <= 5; ++i) {manipulation::trace_append(tr, {1.0, 0.1 * i, 0, 0, 0});}
  ASSERT_EQ(tr.size(), 3u);   // 꺾인 점 (1, 0) 남음
  EXPECT_NEAR(tr[1][0], 1.0, 1e-9);
  EXPECT_NEAR(tr[1][1], 0.0, 1e-9);
  manipulation::trace_append(tr, {1.0, 0.5, 0, 0, 0});   // 같은 점은 안 더함
  EXPECT_EQ(tr.size(), 3u);
  manipulation::trace_append(tr, {1.0, 0.3, 0, 0, 0});   // 되돌아 내려감 = 꺾임 (지움 아님)
  EXPECT_EQ(tr.size(), 4u);
  manipulation::trace_append(tr, {std::nan(""), 0, 0, 0, 0});
  manipulation::trace_append(tr, {1, 2, 3});
  EXPECT_EQ(tr.size(), 4u);
  std::vector<std::vector<double>> big;
  for (int i = 0; i < 50; ++i) {manipulation::trace_append(big, {0.0, (i % 2) * 0.2, 0.01 * i, 0, 0}, 1e-3, 10);}
  EXPECT_EQ(big.size(), 10u);
}

namespace
{
// LOOK (출구) → 퍽 위 A → 내려감 D → 쥠 G → 다시 A → 박스 위 기준점 S → 놓을 자리 위 B → 내려감 P (여기서 FAULT, B–P 도중)
const std::vector<double> L{-0.32, -0.28, -1.06, 0.59, -0.16}, A{0.10, 0.20, -0.90, -0.40, 0.0}, D{0.10, 0.28, -0.95, -0.42, 0.0},
  G{0.10, 0.31, -0.97, -0.43, 0.0}, S{0.0, 0.05, -0.70, -0.30, 0.0}, B{-0.20, 0.20, -0.90, -0.40, 0.3}, P{-0.20, 0.30, -0.96, -0.43, 0.3};

std::vector<std::vector<double>> pick_place_trace()
{
  std::vector<std::vector<double>> tr;
  for (const auto * q : {&L, &A, &D, &G, &D, &A, &S, &B, &P}) {manipulation::trace_append(tr, *q);}
  return tr;
}

bool near(const std::vector<double> & a, const std::vector<double> & b, double tol = 1e-6)
{
  if (a.size() != b.size()) {return false;}
  for (std::size_t j = 0; j < a.size(); ++j) {if (std::fabs(a[j] - b[j]) > tol) {return false;}}
  return true;
}

std::vector<double> mix(const std::vector<double> & a, const std::vector<double> & b, double t)
{
  std::vector<double> q(a.size());
  for (std::size_t j = 0; j < a.size(); ++j) {q[j] = a[j] + t * (b[j] - a[j]);}
  return q;
}
}  // namespace

// ⑦ PLACE 도중 멈춤 → 거꾸로: B → S → A → (A–D–G–D–A 고리 뺌) → L 출구. 이웃 두 점은 모두 지나온 구간 (새 직선 없음)
TEST(HomeRetreat, ReversesToExitSkippingGraspLoop)
{
  const auto tr = pick_place_trace();
  ASSERT_EQ(tr.size(), 9u);
  auto now = mix(B, P, 0.4);
  now[1] += 0.03;   // 따라가기 오차 · 처짐
  std::string why;
  std::size_t ex = 99;
  const auto nodes = manipulation::retreat_from_trace(tr, now, 0.15, 0.01, [](const std::vector<double> & q) {return near(q, L, 0.05);},
    20.0, why, &ex);
  ASSERT_EQ(nodes.size(), 5u) << why;
  EXPECT_TRUE(near(nodes[0].q, now));
  EXPECT_TRUE(near(nodes[1].q, B));
  EXPECT_TRUE(near(nodes[2].q, S));
  EXPECT_TRUE(near(nodes[3].q, A));
  EXPECT_TRUE(near(nodes[4].q, L));
  EXPECT_EQ(nodes.back().name, "retreat_exit");
  EXPECT_TRUE(nodes.back().stop);
  EXPECT_EQ(ex, 0u);
  for (const auto & n : nodes) {
    EXPECT_TRUE(n.grip.empty());
    EXPECT_DOUBLE_EQ(n.speed_dps, 20.0);
  }
  // 이웃 점 (뒤 쪽) 은 trace 에서 이웃한 점 (지나온 구간)
  auto idx = [&tr](const std::vector<double> & q) {for (std::size_t i = 0; i < tr.size(); ++i) {if (near(tr[i], q)) {return static_cast<int>(i);}} return -1;};
  // A 는 1 · 5 두 번 → 고리를 뺀 뒤 A(1) → L(0)
  EXPECT_EQ(idx(nodes[1].q), 7);
  EXPECT_EQ(idx(nodes[2].q), 6);
  // 시간표가 됨 (path_actions 그대로 씀)
  const auto plan = manipulation::path_actions(ArmPath{"home_retreat", nodes, false}, manipulation::PathTiming{}, manipulation::GripperValues{},
    now);
  ASSERT_FALSE(plan.empty());
  for (const auto & a : plan) {EXPECT_FALSE(a.is_gripper);}
  EXPECT_TRUE(near(plan.back().points.back().q, L));
}

// ⑧ 지나온 길에서 벗어남 (손으로 옮김 · 토크 꺼져 처짐) · 출구 없음 · 기록 없음 → 빈 목록 (= 거절)
TEST(HomeRetreat, RefusesWhenOffTraceOrNoExit)
{
  const auto tr = pick_place_trace();
  std::string why;
  auto far = mix(B, P, 0.5);
  far[2] += 0.4;
  EXPECT_TRUE(manipulation::retreat_from_trace(tr, far, 0.15, 0.01, [](const std::vector<double> & q) {return near(q, L, 0.05);}, 20.0, why).empty());
  EXPECT_NE(why.find("벗어남"), std::string::npos);
  why.clear();
  EXPECT_TRUE(manipulation::retreat_from_trace(tr, mix(B, P, 0.5), 0.15, 0.01, [](const std::vector<double> &) {return false;}, 20.0, why).empty());
  EXPECT_NE(why.find("없음"), std::string::npos);
  why.clear();
  EXPECT_TRUE(manipulation::retreat_from_trace({}, P, 0.15, 0.01, [](const std::vector<double> &) {return true;}, 20.0, why).empty());
  EXPECT_FALSE(why.empty());
}

// ⑨ 쥐러 내려가다 멈춤 (A–D 도중) → D 를 지나지 않고 A → L. 되돌아가기가 또 멈춰도 (A–L 도중) 다시 L 로
TEST(HomeRetreat, StopDuringDescentAndRepeatedStop)
{
  std::vector<std::vector<double>> tr;
  for (const auto * q : {&L, &A, &D}) {manipulation::trace_append(tr, *q);}
  std::string why;
  auto ex = [](const std::vector<double> & q) {return near(q, L, 0.05);};
  auto n1 = manipulation::retreat_from_trace(tr, mix(A, D, 0.5), 0.15, 0.01, ex, 20.0, why);
  ASSERT_EQ(n1.size(), 3u) << why;
  EXPECT_TRUE(near(n1[1].q, A));
  EXPECT_TRUE(near(n1[2].q, L));
  auto n2 = manipulation::retreat_from_trace(tr, mix(L, A, 0.3), 0.15, 0.01, ex, 20.0, why);
  ASSERT_EQ(n2.size(), 2u) << why;
  EXPECT_TRUE(near(n2[1].q, L));
  // 이미 출구 근처 (L 에서 조금) → 출구 한 점
  auto n3 = manipulation::retreat_from_trace(tr, mix(L, A, 0.001), 0.15, 0.01, ex, 20.0, why);
  ASSERT_EQ(n3.size(), 2u) << why;
  EXPECT_EQ(n3.back().name, "retreat_exit");
}

// ⑩ 실패한 먼 PLACE: 내려놓을 자리 S 에 퍽이 남았을 때만 기억 (놓기 전 · 다시 쥔 뒤 끝나면 안 남음)
TEST(PlacedMemory, FarPutLeftOnlyAfterRelease)
{
  using manipulation::far_put_left_after;
  bool left = false;
  for (const char * p : {"staging", "far_put_above", "far_put_down"}) {left = far_put_left_after(p, left, false);}
  EXPECT_FALSE(left);                                    // 놓기 전 (아직 손에)
  left = far_put_left_after("far_put", left, false);     // 놓기 열기 뒤 다음 팔 동작
  EXPECT_TRUE(left);
  left = far_put_left_after("far_re_above", left, false);
  EXPECT_TRUE(left);
  EXPECT_FALSE(far_put_left_after("far_re_grasp", left, false));   // 다시 쥠 → 손에
  EXPECT_TRUE(far_put_left_after("far_re_grasp", left, true));     // 다시 쥐기 빈손 → S 에 남음
}

// =====================================================================

// ⑪ 쥐고 올라오는 길 (D → A) 에서 멈춤: 같은 구간 (A → D 내려감) 과 겹쳐도 바깥 A 로 → L. D · G 로 안 내려감
TEST(HomeRetreat, StopOnReturnLegGoesOutward)
{
  const auto tr = pick_place_trace();
  const std::vector<std::vector<double>> blocked{G};   // 닫기를 보낸 점
  std::string why;
  for (const auto * bl : {static_cast<const std::vector<std::vector<double>> *>(nullptr), &blocked}) {
    const auto now = mix(D, A, 0.5);
    const auto nodes = manipulation::retreat_from_trace(tr, now, 0.06, 0.01, [](const std::vector<double> & q) {return near(q, L, 0.05);},
      20.0, why, nullptr, bl);
    ASSERT_EQ(nodes.size(), 3u) << why;
    EXPECT_TRUE(near(nodes[1].q, A));
    EXPECT_TRUE(near(nodes[2].q, L));
  }
}

namespace
{
// STORE: P0 (출구) → 박스 위 S → 칸 위 Ca → 칸 안 Ci (놓기 = 막힌 점) → 곧게 빼기 W (Ca 와 다른 자세) → P0
const std::vector<double> E0{0.0, -1.38, 0.63, -0.02, 0.02}, Sx{0.0, 0.05, -0.70, -0.30, 0.0}, Ca{1.20, 0.10, -0.80, 0.50, -0.8},
  Ci{1.20, 0.25, -0.95, 0.55, -0.8}, W{1.20, 0.05, -0.60, 0.40, -0.8};
bool at_p0(const std::vector<double> & q) {return near(q, E0, 0.05);}
}  // namespace

// ⑫ STORE 빼는 중 멈춤 → 칸 안 (넣은 깊이) 으로 안 돌아감: 빼기 끝 W → P0. 빼기 끝이 길의 끝이면 (P0 로 가는 점을 아직 안 보냄) 거절
TEST(HomeRetreat, StoreWithdrawStopNeverReinserts)
{
  std::vector<std::vector<double>> tr;
  for (const auto * q : {&E0, &Sx, &Ca, &Ci, &W, &E0}) {manipulation::trace_append(tr, *q);}
  const std::vector<std::vector<double>> blocked{Ci};
  std::string why;
  const auto now = mix(Ci, W, 0.5);
  auto nodes = manipulation::retreat_from_trace(tr, now, 0.06, 0.01, at_p0, 20.0, why, nullptr, &blocked);
  ASSERT_EQ(nodes.size(), 3u) << why;
  EXPECT_TRUE(near(nodes[1].q, W));
  EXPECT_TRUE(near(nodes[2].q, E0));
  for (const auto & n : nodes) {EXPECT_FALSE(near(n.q, Ci, 0.01));}
  // 빼기 동작이 W 에서 끝 (다음 동작 안 보냄): 칸 안을 지나야만 출구 → 거절 (칸 안으로 다시 넣으며 거꾸로 가지 않음)
  std::vector<std::vector<double>> tr2;
  for (const auto * q : {&E0, &Sx, &Ca, &Ci, &W}) {manipulation::trace_append(tr2, *q);}
  why.clear();
  EXPECT_TRUE(manipulation::retreat_from_trace(tr2, now, 0.06, 0.01, at_p0, 20.0, why, nullptr, &blocked).empty());
  EXPECT_NE(why.find("길이 없음"), std::string::npos);
  // 넣는 중 멈춤 (Ca → Ci 도중): 칸 위 Ca → S → P0 (첫 점)
  nodes = manipulation::retreat_from_trace(tr, mix(Ca, Ci, 0.4), 0.06, 0.01, at_p0, 20.0, why, nullptr, &blocked);
  ASSERT_EQ(nodes.size(), 4u) << why;
  EXPECT_TRUE(near(nodes[1].q, Ca));
  EXPECT_TRUE(near(nodes[2].q, Sx));
  EXPECT_TRUE(near(nodes[3].q, E0));
}

// ⑬ 그리퍼 점은 일직선 줄이기로 안 지워짐 (막힌 점이 길에 남게)
TEST(ArmTrace, KeepBeforeProtectsGripPoint)
{
  std::vector<std::vector<double>> tr;
  manipulation::trace_append(tr, {0, 0, 0, 0, 0});
  manipulation::trace_append(tr, {0.5, 0, 0, 0, 0});   // 여기서 그리퍼
  const std::size_t keep = tr.size();
  manipulation::trace_append(tr, {1.0, 0, 0, 0, 0}, 1e-3, 4000, keep);
  ASSERT_EQ(tr.size(), 3u);
  EXPECT_NEAR(tr[1][0], 0.5, 1e-12);
}

// ⑭ 자동 접기는 되돌아가기 안 씀 · 되돌아간 뒤 실측이 출구 밖이면 접지 않음
TEST(HomeRetreat, OnlyOnHomeRequestAndLandedByMeasurement)
{
  EXPECT_EQ(manipulation::home_retreat_block(true, true, false), "");
  EXPECT_NE(manipulation::home_retreat_block(false, true, false), "");   // 실패 뒤 자동 접기 · P0 자동 접기
  EXPECT_NE(manipulation::home_retreat_block(true, false, false), "");
  EXPECT_NE(manipulation::home_retreat_block(true, true, true), "");
  auto ex = [](const std::vector<double> & q) {return near(q, L, 0.15);};   // path_start_tolerance 0.15
  auto meas = L;
  meas[1] += 0.10;   // 따라가기 오차
  EXPECT_TRUE(manipulation::home_retreat_landed(meas, ex));
  meas[1] += 0.10;   // 처짐 0.2 rad: 명령값 끝점은 L 이지만 실측은 밖 → 접지 않음 (직선 복귀 없음)
  EXPECT_FALSE(manipulation::home_retreat_landed(meas, ex));
  EXPECT_FALSE(manipulation::home_retreat_landed({std::nan(""), 0, 0, 0, 0}, [](const std::vector<double> &) {return true;}));
}

// ⑮ mark_sent 뒤 다음 tick 이 그 틈을 또 늘리지 않음 → 보낸 시각 + 2 s 에 끊김
TEST(ArmStall, MarkSentDoesNotDoubleExtend)
{
  ArmRunner r(2.0, 5.0, 1.0);
  r.set_stall(0.25);
  r.start(two_moves());
  auto out = r.tick(10.0, true);
  const unsigned g = sent_gen(out);
  EXPECT_TRUE(r.mark_sent(g, 12.2));   // 보내기 전 계획 2.2 s
  out = r.tick(12.21, true);
  EXPECT_EQ(r.stalls(), 0);
  EXPECT_FALSE(has(out, ArmCommand::Kind::CANCEL_ACTIVE));
  bool abandoned = false;
  double t_ab = 0.0;
  for (double t = 12.22; t < 16.0 && !abandoned; t += 0.01) {abandoned = has(r.tick(t, true), ArmCommand::Kind::CANCEL_ACTIVE); t_ab = t;}
  EXPECT_TRUE(abandoned);
  EXPECT_NEAR(t_ab, 14.21, 0.015);
}

// ⑯ 계속 굶는 노드: 늘림 합 상한 (5 s) 뒤에는 시간 초과 · 유예는 시작 2 s 뒤 안 늘림
TEST(ArmStall, PermanentStarvationStillTimesOut)
{
  ArmRunner r(2.0, 1.0, 1.0);
  r.set_stall(0.25, 5.0);
  r.start(two_moves());
  auto out = r.tick(0.0, true);
  r.on_goal_response(sent_gen(out), true, 0.0);   // 끝나기 기한 1.0
  bool timed_out = false;
  double t = 0.0;
  while (!timed_out && t < 30.0) {t += 0.5; timed_out = has(r.tick(t, true), ArmCommand::Kind::CANCEL_ACTIVE);}   // 매 주기 0.5 s 멈춤
  EXPECT_TRUE(timed_out);
  EXPECT_NEAR(t, 6.5, 0.01);   // 1.0 + 상한 5.0 넘어 첫 주기

  manipulation::StallGrace sg;
  EXPECT_FALSE(sg.on_step(0.0, 0.25, 0.3));
  for (double s = 0.5; s <= 4.0 + 1e-9; s += 0.5) {sg.on_step(s, 0.25, 0.3, 2.0);}
  EXPECT_FALSE(sg.active(4.0));    // 0.5 에서 시작 · 2.5 까지만 늘림 → 2.8 에 끝
  sg.on_step(4.01, 0.25, 0.3, 2.0);   // 보통 주기
  sg.on_step(4.02, 0.25, 0.3, 2.0);
  EXPECT_TRUE(sg.on_step(5.0, 0.25, 0.3, 2.0));   // 새 멈춤 → 새 유예
  EXPECT_TRUE(sg.active(5.2));
}
