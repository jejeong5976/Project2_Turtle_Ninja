// test_mission_core.cpp — mission 순수 계산 시험, ROS 없이: 바코드 해석 · 신호등 기록 · dice 계산 · 단계
//   실행: colcon build → source install/setup.bash → ./build/mission/test_mission_core
//   운반 계획 · 재검증 · 기록 파일은 test_transfer.cpp

#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "mission/dice.hpp"
#include "mission/dice_steps.hpp"
#include "mission/signal_recorder.hpp"
#include "mission/task_parser.hpp"

using namespace mission;

// ============================================================================ task_parser
TEST(TaskParser, DiceBarcodes)
{
  // 발제 v5: P2-01 · P2-03 오름차순, P2-02 · P2-04 내림차순
  for (const std::string code : {"P2-01", "P2-03"}) {
    const auto t = parse_task(code);
    ASSERT_TRUE(t.has_value()) << code;
    EXPECT_EQ(t->dice_order, DiceOrder::Ascending);
    EXPECT_EQ(t->task_id, code);
    EXPECT_EQ(t->raw_text, code);
  }
  for (const std::string code : {"P2-02", "P2-04"}) {
    const auto t = parse_task(code);
    ASSERT_TRUE(t.has_value()) << code;
    EXPECT_EQ(t->dice_order, DiceOrder::Descending);
  }
}

TEST(TaskParser, TrimsWhitespace)
{
  const auto t = parse_task("  P2-02 \r\n");
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->task_id, "P2-02");
  EXPECT_EQ(t->dice_order, DiceOrder::Descending);
}

TEST(TaskParser, RejectsUnregistered)
{
  std::string err;
  EXPECT_FALSE(parse_task("", &err).has_value());
  EXPECT_FALSE(parse_task("P2-05", &err).has_value());          // 미등록
  EXPECT_NE(err.find("미등록"), std::string::npos);
  EXPECT_FALSE(parse_task("p2-01", &err).has_value());          // 대소문자 다름 = 미등록
  EXPECT_FALSE(parse_task("move;7;shelf_C", &err).has_value()); // 옛 시험 형식 안 받음
}

// ============================================================================ signal_recorder (발제 v5 신호등 12 번 점멸)
namespace
{
// perception 출력 흉내: dt 간격으로 같은 관측을 dur 동안 넣음
struct SignalFeed
{
  SignalRecorder r;
  double t = 0.0;
  double dt = 0.2;   // perception 약 5 Hz
  explicit SignalFeed(double step = 0.2, SignalRecorderConfig c = {}) : r(c), dt(step) {}
  void run(SignalObs o, double dur)
  {
    for (double e = 0.0; e < dur - 1e-9; e += dt) {r.observe(t, o); t += dt;}
  }
  // 발제 규칙대로 재생: 시작 전 소등 2 s, 점멸마다 1 s 켜짐 + 1 s 소등, 끝 소등 tail_s
  void play(const std::string & seq, double tail_s = 3.2)
  {
    run(SignalObs::Off, 2.0);
    for (char c : seq) {
      run(c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green, 1.0);
      run(SignalObs::Off, 1.0);
    }
    run(SignalObs::Off, tail_s - 1.0);
  }
};
}  // namespace

TEST(SignalRecorder, RecordsTwelveBlinksIncludingRepeatedColors)
{
  SignalFeed f;
  f.play("RGRRGRGGYGYG");   // 같은 색 연속 (RR, GG) 도 소등으로 구별
  EXPECT_EQ(f.r.status(), SignalStatus::Done);
  EXPECT_EQ(f.r.sequence(), "RGRRGRGGYGYG");
  EXPECT_EQ(f.r.count('R'), 4);
  EXPECT_EQ(f.r.count('Y'), 2);
  EXPECT_EQ(f.r.count('G'), 6);
  EXPECT_TRUE(f.r.issues().empty());
}

TEST(SignalRecorder, WorksAtTenHzWithOneAmbiguousSamplePerBlink)
{
  SignalFeed f(0.1);
  f.run(SignalObs::Off, 2.0);
  for (char c : std::string("RRYYGGRYGRYG")) {
    f.run(SignalObs::Ambiguous, 0.1);   // 켜지는 순간 한 장은 색 애매
    f.run(c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green, 0.9);
    f.run(SignalObs::Off, 1.0);
  }
  f.run(SignalObs::Off, 2.6);
  EXPECT_EQ(f.r.status(), SignalStatus::Done);
  EXPECT_EQ(f.r.sequence(), "RRYYGGRYGRYG");
}

TEST(SignalRecorder, IgnoresShortFlicker)
{
  SignalFeed f;
  f.run(SignalObs::Off, 1.0);
  f.run(SignalObs::Red, 0.2);   // 반사 등 0.2 s 깜빡임 (min_on_s 0.3 미만)
  f.play("RGRRGRGGYGYG");
  EXPECT_EQ(f.r.status(), SignalStatus::Done);
  EXPECT_EQ(f.r.sequence(), "RGRRGRGGYGYG");
  EXPECT_FALSE(f.r.notes().empty());
}

TEST(SignalRecorder, UnknownColorIsNotGuessed)
{
  SignalFeed f;
  f.run(SignalObs::Off, 2.0);
  const std::string seq = "RGRRGRGGYGYG";
  for (std::size_t i = 0; i < seq.size(); ++i) {
    const char c = seq[i];
    f.run(i == 4 ? SignalObs::Ambiguous : c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green, 1.0);
    f.run(SignalObs::Off, 1.0);
  }
  f.run(SignalObs::Off, 2.6);
  EXPECT_EQ(f.r.status(), SignalStatus::Incomplete);
  EXPECT_EQ(f.r.sequence(), "RGRR?RGGYGYG");   // 5 번째는 추정하지 않음
  EXPECT_EQ(f.r.count('?'), 1);
}

TEST(SignalRecorder, LongGapMakesResultIncomplete)
{
  SignalFeed f;
  f.run(SignalObs::NotSeen, 3.0);   // 시작 전 못 봄 = 문제 아님
  f.run(SignalObs::Off, 1.0);
  const std::string seq = "RGRRGRGGYGYG";
  for (std::size_t i = 0; i < seq.size(); ++i) {
    const char c = seq[i];
    f.run(c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green, 1.0);
    f.run(i == 6 ? SignalObs::NotSeen : SignalObs::Off, 1.0);   // 소등 구간에 1 s 끊김 (max_gap_s 0.8 초과)
  }
  f.run(SignalObs::Off, 2.6);
  // 7 · 8 번째 (G, G) 사이 소등을 못 봤으니 둘을 구별할 근거가 없음 → 하나로 합쳐짐 (추정해 나누지 않음)
  EXPECT_EQ(f.r.sequence(), "RGRRGRGYGYG");
  EXPECT_EQ(f.r.status(), SignalStatus::Incomplete);
  ASSERT_GE(f.r.issues().size(), 2u);   // 관측 끊김 + 개수 11
  EXPECT_NE(f.r.issues()[0].find("끊김"), std::string::npos);
}

TEST(SignalRecorder, ShortGapInsideBlinkIsTolerated)
{
  SignalFeed f;
  f.run(SignalObs::Off, 2.0);
  for (char c : std::string("RGRRGRGGYGYG")) {
    const SignalObs o = c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green;
    f.run(o, 0.4);
    f.run(SignalObs::NotSeen, 0.2);   // 사람이 잠깐 가림
    f.run(o, 0.4);
    f.run(SignalObs::Off, 1.0);
  }
  f.run(SignalObs::Off, 2.6);
  EXPECT_EQ(f.r.status(), SignalStatus::Done);
  EXPECT_EQ(f.r.sequence(), "RGRRGRGGYGYG");
}

TEST(SignalRecorder, EndsOnlyAfterLongOff)
{
  SignalFeed f;
  f.play("RGRRGRGGYGYG", 2.0);   // 마지막 뒤 소등 2 s
  EXPECT_EQ(f.r.status(), SignalStatus::Recording);
  f.run(SignalObs::Off, 0.8);    // 합 2.8 s > end_off_s 2.5
  EXPECT_EQ(f.r.status(), SignalStatus::Done);
}

TEST(SignalRecorder, TooFewBlinksIsIncomplete)
{
  SignalFeed f;
  f.play("RGRRGRGGYGY");   // 11 개
  EXPECT_EQ(f.r.status(), SignalStatus::Incomplete);
  EXPECT_EQ(f.r.sequence().size(), 11u);
}

TEST(SignalRecorder, CountRuleViolationIsIncomplete)
{
  SignalFeed f;
  f.play("RRRRRRRGGGGY");   // R 7 회 (규칙 1-6)
  EXPECT_EQ(f.r.status(), SignalStatus::Incomplete);
  EXPECT_EQ(f.r.sequence(), "RRRRRRRGGGGY");
}

TEST(SignalRecorder, ColorChangeWithoutOffSplitsAndFlags)
{
  SignalFeed f;
  f.run(SignalObs::Off, 2.0);
  f.run(SignalObs::Red, 1.0);
  f.run(SignalObs::Green, 1.0);   // 소등 없이 바로 바뀜 (규칙상 없어야 함)
  f.run(SignalObs::Off, 1.0);
  EXPECT_EQ(f.r.sequence(), "RG");
  EXPECT_FALSE(f.r.issues().empty());
}


// ============================================================================ dice
namespace
{
std::string groups_str(const std::array<char, 3> & g) {return std::string(g.begin(), g.end());}
}  // namespace

TEST(Dice, ParsesBarcodeOrder)
{
  EXPECT_EQ(parse_dice_barcode("P2-01"), DiceOrder::Ascending);
  EXPECT_EQ(parse_dice_barcode(" P2-03\n"), DiceOrder::Ascending);
  EXPECT_EQ(parse_dice_barcode("P2-02"), DiceOrder::Descending);
  EXPECT_EQ(parse_dice_barcode("P2-04"), DiceOrder::Descending);
  std::string err;
  EXPECT_FALSE(parse_dice_barcode("P2-05", &err).has_value());   // 미등록 = 오류 정지
  EXPECT_NE(err.find("미등록"), std::string::npos);
  EXPECT_FALSE(parse_dice_barcode("", &err).has_value());
}

TEST(Dice, SortsGroupsWithRygTieOrder)
{
  const SignalCounts q1{4, 2, 6};   // RGRRGRGGYGYG
  EXPECT_EQ(groups_str(sort_dice_groups(q1, DiceOrder::Ascending)), "YRG");
  EXPECT_EQ(groups_str(sort_dice_groups(q1, DiceOrder::Descending)), "GRY");
  // 동률은 R → Y → G 유지 (오름 · 내림 둘 다)
  EXPECT_EQ(groups_str(sort_dice_groups({4, 4, 4}, DiceOrder::Ascending)), "RYG");
  EXPECT_EQ(groups_str(sort_dice_groups({4, 4, 4}, DiceOrder::Descending)), "RYG");
  EXPECT_EQ(groups_str(sort_dice_groups({2, 5, 5}, DiceOrder::Descending)), "YGR");
  EXPECT_EQ(groups_str(sort_dice_groups({6, 3, 3}, DiceOrder::Ascending)), "YGR");
}

TEST(Dice, ChecksSignalCounts)
{
  std::string err;
  EXPECT_TRUE(valid_signal_counts({4, 2, 6}));
  EXPECT_FALSE(valid_signal_counts({7, 1, 4}, {}, &err));   // 6 초과
  EXPECT_FALSE(valid_signal_counts({6, 0, 6}, {}, &err));   // 0 회
  EXPECT_FALSE(valid_signal_counts({4, 2, 5}, {}, &err));   // 합 11
}

TEST(Dice, PipOffsetsMatchFigure)
{
  using P = std::array<double, 2>;
  const double p = 0.06;
  EXPECT_EQ(dice_pip_offsets(1), (std::vector<P>{{0.0, 0.0}}));
  EXPECT_EQ(dice_pip_offsets(2), (std::vector<P>{{-p, p}, {p, -p}}));   // 왼쪽 위 → 오른쪽 아래
  EXPECT_EQ(dice_pip_offsets(3), (std::vector<P>{{-p, p}, {0.0, 0.0}, {p, -p}}));
  for (int n = 1; n <= 6; ++n) {
    const auto o = dice_pip_offsets(n);
    ASSERT_EQ(static_cast<int>(o.size()), n);
    std::set<std::pair<double, double>> uniq;
    for (const auto & q : o) {
      EXPECT_LE(std::fabs(q[0]), p + 1e-12);
      EXPECT_LE(std::fabs(q[1]), p + 1e-12);
      uniq.insert({q[0], q[1]});
    }
    EXPECT_EQ(static_cast<int>(uniq.size()), n);   // 한 점에 퍽 1개
  }
  for (const auto & q : dice_pip_offsets(6)) {EXPECT_NE(q[0], 0.0);}   // 6 = 좌우 두 열 (가운데 열 없음)
  EXPECT_TRUE(dice_pip_offsets(0).empty());
  EXPECT_TRUE(dice_pip_offsets(7).empty());
}

TEST(Dice, LayoutQ1Example)
{
  std::string err;
  const auto plan = dice_layout({4, 2, 6}, DiceOrder::Ascending, {}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  // 오름차순: L1 = Y(2), L2 = R(4), L3 = G(6)
  EXPECT_EQ(groups_str({plan->places[0].group, plan->places[1].group, plan->places[2].group}), "YRG");
  for (const auto & pl : plan->places) {
    EXPECT_EQ(pl.pips, (pl.group == 'Y' ? 2 : pl.group == 'R' ? 4 : 6));
    EXPECT_EQ(static_cast<int>(pl.pip_offsets.size()), pl.pips);
  }
}

TEST(Dice, LayoutPdfExampleAndBadCounts)
{
  // 발제 p.5: R5 · Y3 · G4 오름차순 → Y · G · R
  std::string err;
  const auto plan = dice_layout({5, 3, 4}, DiceOrder::Ascending, {}, &err);
  ASSERT_TRUE(plan.has_value()) << err;
  EXPECT_EQ(groups_str({plan->places[0].group, plan->places[1].group, plan->places[2].group}), "YGR");
  EXPECT_FALSE(dice_layout({4, 2, 5}, DiceOrder::Ascending, {}, &err).has_value());   // 신호 합 11
  EXPECT_FALSE(err.empty());
}

TEST(Dice, OrdersFarPipsFirst)
{
  DicePlace pl;
  pl.pips = 5;
  pl.pip_offsets = dice_pip_offsets(5);
  order_far_first(pl, 0.0, -1.0);   // 로봇이 아래(−Y) 에서 접근 → 위쪽 행부터
  ASSERT_EQ(pl.pip_offsets.size(), 5u);
  EXPECT_GT(pl.pip_offsets[0][1], 0.0);
  EXPECT_GT(pl.pip_offsets[1][1], 0.0);
  EXPECT_EQ(pl.pip_offsets[2][1], 0.0);
  EXPECT_LT(pl.pip_offsets[4][1], 0.0);
}

TEST(Dice, ViewDirectionRotatesPattern)
{
  using mission::pips_view_to_map;
  const double p = 0.06;
  const auto two = mission::dice_pip_offsets(2, p);   // 그림: 왼쪽 위 (−1, +1), 오른쪽 아래 (+1, −1)
  // L3: 남쪽 원에서 북쪽을 봄 = 그림 그대로
  auto m = pips_view_to_map(two, 0.0, 1.0);
  EXPECT_NEAR(m[0][0], -p, 1e-12); EXPECT_NEAR(m[0][1], p, 1e-12);
  // L1: 서쪽 원에서 동쪽을 봄 → 오른쪽 = −Y, 먼 쪽 = +X. 왼쪽 위 = 먼 쪽(+X) · 왼쪽(+Y)
  m = pips_view_to_map(two, 1.0, 0.0);
  EXPECT_NEAR(m[0][0], p, 1e-12); EXPECT_NEAR(m[0][1], p, 1e-12);
  EXPECT_NEAR(m[1][0], -p, 1e-12); EXPECT_NEAR(m[1][1], -p, 1e-12);
  // L2: 북쪽 원에서 남쪽을 봄 → 180° 회전. 6 은 두 열이 보는 사람 기준 좌우 = 지도 x 로 나뉨
  const auto six = pips_view_to_map(mission::dice_pip_offsets(6, p), 0.0, -1.0);
  std::set<double> xs;
  for (const auto & q : six) {xs.insert(std::round(q[0] * 1000));}
  EXPECT_EQ(xs.size(), 2u);
  // L1 의 6: 두 열이 보는 사람 좌우 = 지도 y 로 나뉨 (x 는 세 줄)
  const auto six_l1 = pips_view_to_map(mission::dice_pip_offsets(6, p), 1.0, 0.0);
  std::set<double> ys;
  for (const auto & q : six_l1) {ys.insert(std::round(q[1] * 1000));}
  EXPECT_EQ(ys.size(), 2u);
}

TEST(Dice, ApplyViewDirectionsOrdersFarFromViewer)
{
  auto plan = mission::dice_layout(mission::SignalCounts{4, 2, 6}, mission::DiceOrder::Ascending);
  ASSERT_TRUE(plan.has_value());
  // 발제 그림 관찰 원: L1 서 → 동, L2 북 → 남, L3 남 → 북
  mission::apply_view_directions(*plan, {{{{1.0, 0.0}}, {{0.0, -1.0}}, {{0.0, 1.0}}}});
  // 각 장소 첫 칸 = 보는 사람에서 가장 먼 칸 (보는 방향 투영 최대)
  const double f[3][2] = {{1, 0}, {0, -1}, {0, 1}};
  for (int i = 0; i < 3; ++i) {
    const auto & o = plan->places[i].pip_offsets;
    ASSERT_FALSE(o.empty());
    for (const auto & q : o) {
      EXPECT_LE(q[0] * f[i][0] + q[1] * f[i][1], o[0][0] * f[i][0] + o[0][1] * f[i][1] + 1e-12);
    }
  }
}


// ============================================================================ dice_steps (바코드 읽기 · 복귀)
TEST(DiceSteps, ReadStepsWithLook)
{
  const auto st = build_read_steps({0.5, 0.8, 3.14}, 5.0, true);
  ASSERT_EQ(st.size(), 4u);   // 이동 → LOOK_BARCODE → 읽기 → HOME
  EXPECT_EQ(st[0].type, StepType::Navigate);
  EXPECT_DOUBLE_EQ(st[0].pose.x, 0.5);
  EXPECT_EQ(st[1].arm_task, kArmLookBarcode);
  EXPECT_TRUE(is_custom(st[2], kReadBarcode));
  EXPECT_DOUBLE_EQ(st[2].timeout_s, 5.0);
  EXPECT_EQ(st[3].arm_task, kArmHome);
  EXPECT_EQ(build_read_steps({0, 0, 0}, 5.0, false).size(), 2u);
  const auto ret = build_return_steps({1.0, 2.0, 0.0});
  ASSERT_EQ(ret.size(), 1u);
  EXPECT_EQ(ret[0].name, "nav:start");
}
