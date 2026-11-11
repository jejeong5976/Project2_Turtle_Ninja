// test_grasp_guard.cpp — 쥔 폭 · 손 안 퍽 · 넣기 걸림 · 넣은 뒤 판단과 다시 쥐기 동작 (grasp_guard.hpp), ArmRunner 동작 끼우기

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "manipulation/grasp_guard.hpp"
#include "arm_kin_data.hpp"

using namespace manipulation;

namespace
{
const double kNaN = std::numeric_limits<double>::quiet_NaN();
// config/arm_paths.yaml store_S9 칸 안 놓는 점 (팀 명령 rad, test_cell_shift 와 같음)
const std::vector<double> kInCell{-0.94493, -0.38503, 0.02301, 0.26998, 0.56144};

Eigen::Vector3d tip_of(const ArmModel & m, const std::vector<double> & c) {return arm_grip(m, team_to_model_q(m, c)).tip;}

PathNode pn(const std::string & name, const std::vector<double> & q, const std::string & grip = "", double speed = 15.0)
{
  PathNode n;
  n.name = name;
  n.q = q;
  n.grip = grip;
  n.speed_dps = speed;
  return n;
}

std::vector<double> qv(double a) {return {a, -0.5 + a, 0.3 - a, 0.2 + a, 0.1};}

// PICK 모양 경로: 펼치기 끝 → 위 (열기) → 내림 → 쥐기 (닫기) → 올림 → 들기
ArmPath pick_path()
{
  ArmPath p{"pick", {}, false};
  p.nodes.push_back(pn("P0", qv(0.00), "", 0.0));
  p.nodes.push_back(pn("above", qv(0.10), "open", 0.0));
  p.nodes.back().grip_early = true;
  p.nodes.push_back(pn("down", qv(0.12)));
  p.nodes.push_back(pn("grasp", qv(0.15), "close"));
  p.nodes.push_back(pn("up", qv(0.12)));
  p.nodes.push_back(pn("lift", qv(0.10)));
  return p;
}

double qdist(const std::vector<double> & a, const std::vector<double> & b)
{
  double d = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {d = std::max(d, std::fabs(a[j] - b[j]));}
  return d;
}

ArmAction arm_act(const std::string & pose) {return ArmAction{"approach", false, pose, 0.0};}
}  // namespace

// ============================================================================ ① 쥔 폭
TEST(GraspWidth, ModelGapMatchesRealSquareContact)
{
  GripWidthConfig c;
  EXPECT_NEAR(finger_gap_mm(0.0046, c), 34.8, 1e-9);   // 실측 네모 34.8 쥔 채 q 약 0.0046 (park_loose_grip_m 주석)
  EXPECT_NEAR(finger_gap_mm(0.0022, c), 30.0, 1e-9);   // 원기둥 30 = 0.0022 (grasp_empty_m 주석 "약 0.002")
  EXPECT_TRUE(std::isnan(finger_gap_mm(kNaN, c)));
}

TEST(GraspWidth, GoodGraspsPass)
{
  GripWidthConfig c;
  const auto s = judge_grip_width(0.0046, PuckShape::Square, c);
  EXPECT_TRUE(s.judged);
  EXPECT_FALSE(s.bad);
  EXPECT_NEAR(s.err_mm, 0.0, 1e-9);
  const auto y = judge_grip_width(0.0024, PuckShape::Cylinder, c);
  EXPECT_FALSE(y.bad);
  EXPECT_NEAR(y.err_mm, 0.4, 1e-9);
}

TEST(GraspWidth, DiagonalOrTurnedSquareIsBad)
{
  GripWidthConfig c;
  const double q_diag = (34.8 * std::sqrt(2.0) - 7.4) / 2000.0 - 0.0091;   // 대각선 49.2 mm
  const auto v = judge_grip_width(q_diag, PuckShape::Square, c);
  EXPECT_TRUE(v.bad);
  EXPECT_NEAR(v.err_mm, 34.8 * std::sqrt(2.0) - 34.8, 1e-6);
  const double q_turn = (34.8 * (std::cos(0.35) + std::sin(0.35)) - 7.4) / 2000.0 - 0.0091;   // 20° 돌아간 채 모서리
  EXPECT_TRUE(judge_grip_width(q_turn, PuckShape::Square, c).bad);
}

TEST(GraspWidth, EdgeGripNarrowerThanCylinderIsBad)
{
  GripWidthConfig c;
  const auto v = judge_grip_width(-0.0005, PuckShape::Cylinder, c);   // 24.6 mm: 손가락 끝 모서리로 쥠
  EXPECT_TRUE(v.bad);
  EXPECT_NEAR(v.err_mm, -5.4, 1e-9);
}

TEST(GraspWidth, ShapeMixupWithinDefaultTolerance)
{
  GripWidthConfig c;   // 원기둥이라 했는데 네모 (+4.8 mm): 기본 5 mm 안 → 다시 쥐지 않음, 4 mm 면 잡힘
  EXPECT_FALSE(judge_grip_width(0.0046, PuckShape::Cylinder, c).bad);
  c.tol_mm = 4.0;
  EXPECT_TRUE(judge_grip_width(0.0046, PuckShape::Cylinder, c).bad);
}

TEST(GraspWidth, UnknownShapeUsesRange)
{
  GripWidthConfig c;
  const auto in = judge_grip_width(0.0035, PuckShape::Unknown, c);   // 32.6 mm
  EXPECT_FALSE(in.bad);
  EXPECT_DOUBLE_EQ(in.err_mm, 0.0);
  EXPECT_TRUE(std::isnan(in.expect_mm));
  EXPECT_TRUE(judge_grip_width(0.0118, PuckShape::Unknown, c).bad);
  EXPECT_FALSE(judge_grip_width(0.0060, PuckShape::Unknown, c).bad);   // 37.6 = 34.8 + 2.8
}

TEST(GraspWidth, NoMeasurementOrBadConfigNeverJudges)
{
  GripWidthConfig c;
  EXPECT_FALSE(judge_grip_width(kNaN, PuckShape::Square, c).judged);
  EXPECT_FALSE(judge_grip_width(kNaN, PuckShape::Square, c).bad);
  c.tol_mm = kNaN;
  EXPECT_FALSE(judge_grip_width(0.02, PuckShape::Square, c).judged);
  c.tol_mm = 0.0;
  EXPECT_FALSE(judge_grip_width(0.02, PuckShape::Square, c).bad);
}

// ============================================================================ ② 손 안 퍽
TEST(HeldFilter, OnlyTargetIdBetweenGraspAndRelease)
{
  const std::string w = "wrist_link";
  EXPECT_EQ(held_sample_reject("3", w, 10.0, "3", 9.0, 1e18), nullptr);
  EXPECT_STREQ(held_sample_reject("6", w, 10.0, "3", 9.0, 1e18), "다른 퍽");   // 11-10 205428 S10: 쥔 퍽 0, 칸 안 이웃 6 을 13 장
  EXPECT_STREQ(held_sample_reject("3", w, 10.0, "", 9.0, 1e18), "PICK 퍽 ID 없음");
  EXPECT_STREQ(held_sample_reject("3", w, 8.9, "3", 9.0, 1e18), "쥐기 전");
  EXPECT_STREQ(held_sample_reject("3", w, 12.0, "3", 9.0, 11.0), "놓은 뒤");
  EXPECT_STREQ(held_sample_reject("3", w, kNaN, "3", 9.0, 11.0), "쥐기 전");
  EXPECT_EQ(held_sample_reject("3", w, 11.0, "3", 9.0, 11.0), nullptr);
  EXPECT_STREQ(held_sample_reject("3", "base_link", 10.0, "3", 9.0, 1e18), "손목 좌표 아님");
}

TEST(HeldVerdictTest, CenteredUprightPuckIsGood)
{
  const auto m = kin_data::model();
  const Eigen::Vector3d p = m.tip5 - m.app5.normalized() * 0.035;
  const auto v = judge_held_in_hand(true, p, -m.app5, 5, m.tip5, m.app5, m.close5, 8.0, 20.0);
  EXPECT_TRUE(v.seen);
  EXPECT_FALSE(v.bad);
  EXPECT_NEAR(v.lateral_mm, 0.0, 1e-9);
  EXPECT_NEAR(v.depth_mm, 35.0, 1e-9);
  EXPECT_NEAR(v.face_deg, 0.0, 1e-6);
  EXPECT_EQ(v.frames, 5);
}

TEST(HeldVerdictTest, SlidAlongFingerOrTiltedIsBad)
{
  const auto m = kin_data::model();
  const Eigen::Vector3d a = m.app5.normalized();
  const Eigen::Vector3d c = (m.close5 - a * m.close5.dot(a)).normalized();
  const Eigen::Vector3d w = a.cross(c);
  const Eigen::Vector3d p0 = m.tip5 - a * 0.035;
  const auto small = judge_held_in_hand(true, p0 + c * 0.006, -a, 4, m.tip5, m.app5, m.close5, 8.0, 20.0);
  EXPECT_FALSE(small.bad);
  EXPECT_NEAR(small.close_mm, 6.0, 1e-6);
  const auto slid = judge_held_in_hand(true, p0 + w * 0.010, -a, 4, m.tip5, m.app5, m.close5, 8.0, 20.0);
  EXPECT_TRUE(slid.bad);
  EXPECT_NEAR(slid.width_mm, 10.0, 1e-6);
  const Eigen::Vector3d n25 = Eigen::AngleAxisd(25.0 * M_PI / 180.0, c) * (-a);
  const auto tilt = judge_held_in_hand(true, p0, n25, 4, m.tip5, m.app5, m.close5, 8.0, 20.0);
  EXPECT_TRUE(tilt.bad);
  EXPECT_NEAR(tilt.face_deg, 25.0, 1e-6);
  const auto flipped = judge_held_in_hand(true, p0, -n25, 4, m.tip5, m.app5, m.close5, 8.0, 20.0);   // 법선 부호는 상관없음
  EXPECT_NEAR(flipped.face_deg, 25.0, 1e-6);
}

TEST(HeldVerdictTest, NotSeenIsNeverBad)
{
  const auto m = kin_data::model();
  const auto v = judge_held_in_hand(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 1, m.tip5, m.app5, m.close5, 8.0, 20.0);
  EXPECT_FALSE(v.seen);
  EXPECT_FALSE(v.bad);
  EXPECT_NE(v.text.find("못 봄"), std::string::npos);
}

// ============================================================================ ③ 넣기 걸림
TEST(ContactCheck, TipOffsetKeepsOrientation)
{
  const auto m = kin_data::model();
  std::vector<double> up;
  ASSERT_TRUE(tip_offset_command(m, kInCell, Eigen::Vector3d(0.0, 0.0, 0.015), up));
  const Eigen::Vector3d d = tip_of(m, up) - tip_of(m, kInCell);
  EXPECT_NEAR(d.z(), 0.015, 0.0005);
  EXPECT_LT(d.head<2>().norm(), 0.0005);
  const Grip g0 = arm_grip(m, team_to_model_q(m, kInCell)), g1 = arm_grip(m, team_to_model_q(m, up));
  EXPECT_GT(g0.app.normalized().dot(g1.app.normalized()), std::cos(1.0 * M_PI / 180.0));
  std::vector<double> far;
  EXPECT_FALSE(tip_offset_command(m, kInCell, Eigen::Vector3d(0.0, 0.0, 1.0), far));
  EXPECT_FALSE(tip_offset_command(m, {0.0, 0.0}, Eigen::Vector3d(0.0, 0.0, 0.01), far));
}

TEST(ContactCheck, TeamModelRoundTrip)
{
  const auto m = kin_data::model();
  const auto back = model_to_team(m, team_to_model_q(m, kInCell));
  EXPECT_LT(qdist(back, kInCell), 1e-9);
}

namespace
{
// 칸 안 30 mm 위 → 놓는 점 (1 s) → 멈춤 0.3 s (path_actions 의 그리퍼 앞 멈춤과 같은 모양)
std::vector<TrajPoint> descent(const ArmModel & m)
{
  std::vector<double> top;
  EXPECT_TRUE(tip_offset_command(m, kInCell, Eigen::Vector3d(0.0, 0.0, 0.030), top));
  std::vector<TrajPoint> pts;
  for (int i = 1; i <= 20; ++i) {
    const double f = i / 20.0;
    std::vector<double> q(5);
    for (int j = 0; j < 5; ++j) {q[j] = top[j] + (kInCell[j] - top[j]) * f;}
    pts.push_back({q, f});
  }
  pts.push_back({kInCell, 1.3});
  return pts;
}

// 실측: free = 명령 + 일정 오차 (처짐), blocked_at ≥ 0 이면 그 시각 뒤로 그 자세에 멈춤 + 전류 오름
void feed(ContactWatch & w, const std::vector<TrajPoint> & pts, double t0, double blocked_at, double lag = 0.0)
{
  for (int k = 0; k <= 170; ++k) {
    const double t = k * 0.01;
    const bool blocked = blocked_at >= 0.0 && t > blocked_at;
    auto c = command_at(pts, blocked ? blocked_at : t - lag);
    std::array<double, 5> q{}, cur{};
    for (int j = 0; j < 5; ++j) {q[j] = c[j] - (j == 1 ? 0.02 : 0.0);}   // J2 처짐 오차 0.02 rad (기준과 끝이 같음)
    for (int j = 0; j < 5; ++j) {cur[j] = blocked ? 0.65 : 0.25;}
    w.feed(t0 + t, q, cur);
  }
}
}  // namespace

TEST(ContactCheck, DescentStartFromGeometry)
{
  const auto m = kin_data::model();
  const auto pts = descent(m);
  const double t = descent_start_time(m, pts, 0.020);   // 30 mm 위에서 내려옴 → 놓는 점 위 20 mm 는 약 1/3 지점
  EXPECT_GT(t, 0.25);
  EXPECT_LT(t, 0.45);
  EXPECT_DOUBLE_EQ(descent_start_time(m, pts, 0.035), 0.0);   // 그만큼 높은 점 없음 → 처음부터
}

TEST(ContactCheck, FreeInsertionIsNormal)
{
  const auto m = kin_data::model();
  const auto pts = descent(m);
  ContactWatch w;
  w.start(pts, 100.0, 0.35, 0);
  feed(w, pts, 100.0, -1.0);
  const auto v = w.judge(ContactConfig{}, &m);
  EXPECT_TRUE(v.judged);
  EXPECT_FALSE(v.suspect) << v.text;
  EXPECT_NEAR(v.err_end_deg[0], 0.0, 1e-6);   // 일정한 처짐 오차는 기준에서 빠짐
  EXPECT_NEAR(v.height_mm, 0.0, 0.3);
  EXPECT_NEAR(v.cur_peak_a[1], 0.0, 1e-9);
}

TEST(ContactCheck, CommandLagIsRemoved)
{
  const auto m = kin_data::model();
  const auto pts = descent(m);
  ContactWatch w;
  w.start(pts, 100.0, 0.35, 0);
  feed(w, pts, 100.0, -1.0, 0.04);   // 실측이 명령보다 40 ms 늦음 (보낸 시각 · 서보 늦음)
  const auto v = w.judge(ContactConfig{}, &m);
  EXPECT_FALSE(v.suspect) << v.text;
  EXPECT_NEAR(v.err_end_deg[0], 0.0, 1e-6);
  EXPECT_NEAR(v.err_end_deg[1], 0.0, 1e-6);
  EXPECT_NEAR(v.height_mm, 0.0, 0.3);
  EXPECT_NE(v.text.find("지연 40 ms"), std::string::npos) << v.text;
}

TEST(ContactCheck, BlockedOnRimIsSuspect)
{
  const auto m = kin_data::model();
  const auto pts = descent(m);
  ContactWatch w;
  w.start(pts, 100.0, 0.35, 0);
  feed(w, pts, 100.0, 0.75);   // 놓는 점 7.5 mm 위에서 턱에 걸림
  const auto v = w.judge(ContactConfig{}, &m);
  EXPECT_TRUE(v.judged);
  EXPECT_TRUE(v.suspect) << v.text;
  EXPECT_NEAR(v.height_mm, 7.5, 1.0);
  EXPECT_NEAR(v.cur_peak_a[1], 0.40, 1e-9);
  // 문턱을 크게 하면 같은 실측도 정상 (로그만 · 문턱 조정)
  ContactConfig loose;
  loose.err_deg = 90.0;
  loose.current_a = 5.0;
  loose.height_mm = 50.0;
  EXPECT_FALSE(w.judge(loose, &m).suspect);
}

TEST(ContactCheck, TooFewSamplesNotJudged)
{
  const auto m = kin_data::model();
  const auto pts = descent(m);
  ContactWatch w;
  w.start(pts, 100.0, 0.35, 0);
  std::array<double, 5> q{}, cur{};
  w.feed(100.5, q, cur);
  w.feed(99.0, q, cur);   // 보내기 전 측정은 버림
  EXPECT_EQ(w.samples(), 1u);
  const auto v = w.judge(ContactConfig{}, &m);
  EXPECT_FALSE(v.judged);
  EXPECT_FALSE(v.suspect);
  w.stop();
  EXPECT_FALSE(w.active());
}

TEST(ContactCheck, RetryLiftsShiftsAndComesBack)
{
  const auto m = kin_data::model();
  PathTiming t;
  t.grip_settle_s = 0.3;
  std::string why;
  const auto a = contact_retry_action(m, kInCell, 0.015, Eigen::Vector2d(0.002, 0.0), 15.0, t, why);
  ASSERT_FALSE(a.points.empty()) << why;
  EXPECT_EQ(a.phase, "release");
  EXPECT_FALSE(a.is_gripper);
  const Eigen::Vector3d p0 = tip_of(m, kInCell);
  const Eigen::Vector3d pe = tip_of(m, a.points.back().q);
  EXPECT_NEAR((pe - p0).x(), 0.002, 0.0005);
  EXPECT_NEAR((pe - p0).z(), 0.0, 0.0005);
  double zmax = -1.0;
  for (const auto & p : a.points) {zmax = std::max(zmax, tip_of(m, p.q).z() - p0.z());}
  EXPECT_NEAR(zmax, 0.015, 0.001);
  EXPECT_NEAR(a.duration_s, a.points.back().t, 1e-12);
  EXPECT_NEAR(a.points.back().t - a.points[a.points.size() - 2].t, 0.3, 1e-9);   // 놓기 전 멈춤
  const auto bad = contact_retry_action(m, kInCell, 1.0, Eigen::Vector2d::Zero(), 15.0, t, why);
  EXPECT_TRUE(bad.points.empty());
  EXPECT_FALSE(why.empty());
}

// ============================================================================ ④ 넣은 뒤
TEST(SeatCheck, SeatedPuck)
{
  SeatConfig c;
  const Eigen::Vector2d cell(0.078, 0.113);
  std::vector<SeatSample> s;
  for (int i = 0; i < 4; ++i) {s.push_back({1.0 + i, Eigen::Vector3d(cell.x() + 0.001, cell.y() - 0.001, c.floor_z + 0.0605), true});}
  const auto v = judge_seat(s, cell, 0.0348, c);
  EXPECT_EQ(v.state, SeatState::SEATED) << v.text;
  EXPECT_NEAR(v.rel_mm.z(), 0.5, 1e-6);
  EXPECT_EQ(v.n3, 4);
}

TEST(SeatCheck, PuckOnRimIsNotSeated)
{
  SeatConfig c;
  const Eigen::Vector2d cell(0.078, 0.113);
  std::vector<SeatSample> s;
  for (int i = 0; i < 3; ++i) {s.push_back({1.0 + i, Eigen::Vector3d(cell.x() + 0.003, cell.y(), c.floor_z + 0.072), true});}
  EXPECT_EQ(judge_seat(s, cell, 0.0348, c).state, SeatState::NOT_SEATED);
}

TEST(SeatCheck, OffCenterDependsOnPuckWidth)
{
  SeatConfig c;
  const Eigen::Vector2d cell(0.0, 0.0);
  std::vector<SeatSample> s;
  for (int i = 0; i < 3; ++i) {s.push_back({1.0 + i, Eigen::Vector3d(0.008, 0.0, c.floor_z + 0.060), true});}
  EXPECT_EQ(judge_seat(s, cell, 0.0348, c).state, SeatState::NOT_SEATED);   // 네모 여유 4.6 + 2
  EXPECT_EQ(judge_seat(s, cell, 0.030, c).state, SeatState::SEATED);       // 원기둥 여유 7 + 2
}

TEST(SeatCheck, UnknownWhenFewOrImplausible)
{
  SeatConfig c;
  const Eigen::Vector2d cell(0.0, 0.0);
  std::vector<SeatSample> s{{1.0, Eigen::Vector3d(0.0, 0.0, c.floor_z + 0.060), true}, {2.0, Eigen::Vector3d(0.0, 0.0, c.floor_z + 0.060), true}};
  EXPECT_EQ(judge_seat(s, cell, 0.0348, c).state, SeatState::UNKNOWN);
  s.push_back({3.0, Eigen::Vector3d(0.0, 0.0, c.floor_z + 0.060), true});
  EXPECT_EQ(judge_seat(s, cell, 0.0348, c).state, SeatState::SEATED);
  std::vector<SeatSample> high;   // 60 + 40 mm: 다른 면 · 다른 퍽 → 판단 안 함
  for (int i = 0; i < 3; ++i) {high.push_back({1.0 + i, Eigen::Vector3d(0.0, 0.0, c.floor_z + 0.100), true});}
  EXPECT_EQ(judge_seat(high, cell, 0.0348, c).state, SeatState::UNKNOWN);
  std::vector<SeatSample> flat;   // 칸 퍽 태그 (높이 가정) 만 → 높이 모름
  for (int i = 0; i < 3; ++i) {flat.push_back({1.0 + i, Eigen::Vector3d(0.002, 0.001, c.floor_z + 0.060), false});}
  const auto v = judge_seat(flat, cell, 0.0348, c);
  EXPECT_EQ(v.state, SeatState::UNKNOWN);
  EXPECT_EQ(v.n2, 3);
  EXPECT_NE(v.text.find("높이 모름"), std::string::npos);
  EXPECT_EQ(judge_seat({}, cell, 0.0348, c).state, SeatState::UNKNOWN);
}

// ============================================================================ ⑤ 다시 쥐기 동작
TEST(Regrip, PointsFromPickPath)
{
  GripperValues g;
  const auto r = regrip_points(pick_path(), g);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].close_name, "grasp");
  EXPECT_EQ(r[0].up_name, "up");
  EXPECT_DOUBLE_EQ(r[0].open_m, g.open);
  EXPECT_TRUE(r[0].retry_ok);
  ASSERT_EQ(r[0].after.size(), 2u);
  EXPECT_TRUE(put_back_ok(r[0]));
}

TEST(Regrip, SkipsEmptyClosesAndUnknownOpen)
{
  GripperValues g;
  ArmPath p = pick_path();
  p.nodes[1].grip = "release";
  EXPECT_DOUBLE_EQ(regrip_points(p, g)[0].open_m, g.release);
  ArmPath press = pick_path();
  press.nodes[3].name = "park_press_turn";   // 받침 꽂기 전 빈손 닫기 = 쥐는 닫기 아님 (set_grasp_checks 와 같음)
  EXPECT_TRUE(regrip_points(press, g).empty());
  ArmPath way = pick_path();
  way.nodes[3].grip_on_way = true;
  EXPECT_TRUE(regrip_points(way, g).empty());
  ArmPath side = pick_path();
  side.nodes[4].name = "far5";   // 다음 점이 곧게 위가 아님 → 그 자리 다시 쥐기 안 함 (폭은 봄)
  const auto r = regrip_points(side, g);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_FALSE(r[0].retry_ok);
  EXPECT_TRUE(regrip_in_place(r[0], PathTiming{}, g).empty());
  ArmPath no_open = pick_path();
  no_open.nodes[1].grip.clear();
  EXPECT_FALSE(regrip_points(no_open, g)[0].retry_ok);
}

TEST(Regrip, InPlaceOpensGoesUpAndClosesAgain)
{
  GripperValues g;
  PathTiming t;
  t.grip_settle_s = 0.3;
  const auto r = regrip_points(pick_path(), g)[0];
  const auto a = regrip_in_place(r, t, g);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_TRUE(a[0].is_gripper);
  EXPECT_DOUBLE_EQ(a[0].gripper, g.open);
  EXPECT_NE(a[0].phase, "release");   // 놓기로 안 봄 (넣기 감시 · 놓은 시각)
  EXPECT_FALSE(a[1].is_gripper);
  EXPECT_EQ(a[1].pose, "regrip_grasp");
  double to_up = 1.0;
  for (const auto & p : a[1].points) {to_up = std::min(to_up, qdist(p.q, r.q_up));}
  EXPECT_LT(to_up, 1e-9);                                   // 위 점을 지나감
  EXPECT_LT(qdist(a[1].points.back().q, r.q_close), 1e-12);  // 쥐는 점으로 돌아옴
  EXPECT_TRUE(a[2].is_gripper);
  EXPECT_DOUBLE_EQ(a[2].gripper, g.close);
}

TEST(Regrip, PutBackReversesLift)
{
  GripperValues g;
  PathTiming t;
  const auto r = regrip_points(pick_path(), g)[0];
  const auto a = put_back_regrip(r, t, g);
  ASSERT_EQ(a.size(), 5u);
  EXPECT_LT(qdist(a[0].points.back().q, r.q_close), 1e-12);   // 들어 올린 자세 → 쥐는 점 (내려놓음)
  EXPECT_TRUE(a[1].is_gripper);
  EXPECT_DOUBLE_EQ(a[1].gripper, g.open);
  EXPECT_NE(a[1].phase, "release");
  EXPECT_TRUE(a[3].is_gripper);
  EXPECT_DOUBLE_EQ(a[3].gripper, g.close);
  EXPECT_LT(qdist(a[4].points.back().q, r.after.back().q), 1e-12);   // 다시 들기 자세로 (STORE · LOOK_RACK 출발 자세)
  RegripPoint other = r;
  other.after.push_back(pn("staging", qv(0.0)));
  EXPECT_FALSE(put_back_ok(other));
  EXPECT_TRUE(put_back_regrip(other, t, g).empty());
}

// ============================================================================ ArmRunner 동작 끼우기
TEST(RunnerInsert, InsertBeforeSentRunsInsertedThenOriginal)
{
  ArmRunner r(2.0, 7.0, 2.0);
  r.start({arm_act("a"), arm_act("b")});
  auto out = r.tick(0.0, true);
  ASSERT_EQ(out.size(), 1u);
  ASSERT_EQ(out[0].kind, ArmCommand::Kind::SEND);
  EXPECT_EQ(out[0].action.pose, "a");
  ASSERT_TRUE(r.insert_before_sent({arm_act("x"), arm_act("y")}));
  std::vector<std::string> sent{"x"};   // 호출한 쪽은 a 대신 x 를 보냄
  unsigned gen = out[0].gen;
  for (int i = 0; i < 10; ++i) {
    r.on_goal_response(gen, true, 0.1 * i);
    r.on_result(gen, true);
    out = r.tick(0.1 * i + 0.05, true);
    ASSERT_EQ(out.size(), 1u);
    if (out[0].kind == ArmCommand::Kind::FINISH) {
      EXPECT_EQ(out[0].end, ArmEnd::SUCCEEDED);
      break;
    }
    sent.push_back(out[0].action.pose);
    gen = out[0].gen;
  }
  EXPECT_EQ(sent, (std::vector<std::string>{"x", "y", "a", "b"}));
}

TEST(RunnerInsert, OnlyRightAfterSend)
{
  ArmRunner r(2.0, 7.0, 2.0);
  EXPECT_FALSE(r.insert_before_sent({arm_act("x")}));   // 작업 없음
  r.start({arm_act("a")});
  EXPECT_FALSE(r.insert_before_sent({arm_act("x")}));   // 아직 안 보냄
  auto out = r.tick(0.0, true);
  EXPECT_FALSE(r.insert_before_sent({}));                // 빈 목록
  r.on_goal_response(out[0].gen, true, 0.1);
  EXPECT_FALSE(r.insert_before_sent({arm_act("x")}));   // 이미 받아들여짐 (이미 다른 동작이 움직이는 중)
}

TEST(RunnerInsert, ReplaceFromSentDropsSentAndRest)
{
  ArmRunner r(2.0, 7.0, 2.0);
  r.start({arm_act("a"), arm_act("release"), arm_act("b")});
  auto out = r.tick(0.0, true);
  unsigned gen = out[0].gen;
  r.on_goal_response(gen, true, 0.0);
  r.on_result(gen, true);
  out = r.tick(0.05, true);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].action.pose, "release");
  ASSERT_TRUE(r.replace_from_sent({arm_act("lift"), arm_act("box"), arm_act("regrip")}));
  std::vector<std::string> sent{"lift"};   // 호출한 쪽은 release 대신 lift 를 보냄
  gen = out[0].gen;
  for (int i = 0; i < 10; ++i) {
    r.on_goal_response(gen, true, 0.1 * i + 0.1);
    r.on_result(gen, true);
    out = r.tick(0.1 * i + 0.15, true);
    ASSERT_EQ(out.size(), 1u);
    if (out[0].kind == ArmCommand::Kind::FINISH) {
      EXPECT_EQ(out[0].end, ArmEnd::SUCCEEDED);
      break;
    }
    sent.push_back(out[0].action.pose);
    gen = out[0].gen;
  }
  EXPECT_EQ(sent, (std::vector<std::string>{"lift", "box", "regrip"})) << "원래 release · b 는 안 보냄";
}

TEST(RunnerInsert, ReplaceFromSentOnlyRightAfterSend)
{
  ArmRunner r(2.0, 7.0, 2.0);
  EXPECT_FALSE(r.replace_from_sent({arm_act("x")}));   // 작업 없음
  r.start({arm_act("a")});
  EXPECT_FALSE(r.replace_from_sent({arm_act("x")}));   // 아직 안 보냄
  auto out = r.tick(0.0, true);
  EXPECT_FALSE(r.replace_from_sent({}));                // 빈 목록
  r.on_goal_response(out[0].gen, true, 0.1);
  EXPECT_FALSE(r.replace_from_sent({arm_act("x")}));   // 이미 움직이는 중
}

// ============================================================================ ⑥ 넣기 걸림 보정 (store_recover)
namespace
{
// 칸 경로 모양: 기준점 → 칸 위 (높이 entry_mm) → 놓는 점 (kInCell, release) → 곧게 올림 → P0
ArmPath store_like(const ArmModel & m, double entry_mm)
{
  std::vector<double> entry, high, up;
  EXPECT_NEAR(tip_lift_max(m, kInCell, entry_mm / 1000.0, 0.005, entry), entry_mm / 1000.0, 1e-9);
  high = qv(0.3);   // 칸에서 먼 기록 점 (기준점 쪽)
  EXPECT_TRUE(tip_offset_command(m, kInCell, Eigen::Vector3d(0.0, 0.0, 0.03), up));
  ArmPath p{"store_S9", {}, false};
  p.nodes.push_back(pn("staging", qv(0.0), "", 0.0));
  p.nodes.push_back(pn("over", high, "", 0.0));
  p.nodes.push_back(pn("entry", entry, "", 20.0));
  p.nodes.back().stop = true;
  p.nodes.back().hold_s = 0.3;
  p.nodes.push_back(pn("release", kInCell, "release", 12.0));
  p.nodes.push_back(pn("lift_up", up, "close", 12.0));
  p.nodes.push_back(pn("P0", qv(0.05), "", 0.0));
  return p;
}
}  // namespace

TEST(StoreRecover, StraightLiftStopsAtJointRange)
{
  const auto m = kin_data::model();
  std::vector<double> up;
  const double got = tip_lift_max(m, kInCell, 0.040, 0.005, up);
  EXPECT_NEAR(got, 0.030, 0.0051) << "store_S9 놓는 점: 수직 손은 J4 범위로 약 30 mm 까지";
  const Eigen::Vector3d d = tip_of(m, up) - tip_of(m, kInCell);
  EXPECT_NEAR(d.z(), got, 0.001);
  EXPECT_LT(d.head<2>().norm(), 0.001);
  std::vector<double> small;
  EXPECT_NEAR(tip_lift_max(m, kInCell, 0.012, 0.005, small), 0.012, 1e-9);
  EXPECT_EQ(tip_lift_max(m, kInCell, 0.0, 0.005, small), 0.0);
  EXPECT_EQ(qdist(small, kInCell), 0.0);
}

TEST(StoreRecover, ReleaseIndex)
{
  const auto m = kin_data::model();
  const auto p = store_like(m, 20.0);
  EXPECT_EQ(store_release_index(p.nodes), 3u);
  EXPECT_EQ(store_release_index(pick_path().nodes), 1u) << "open 도 놓기 (PICK 경로는 안 씀)";
  std::vector<PathNode> none{pn("a", qv(0.0)), pn("b", qv(0.1), "close")};
  EXPECT_EQ(store_release_index(none), none.size());
}

TEST(StoreRecover, ShortStraightGetsStraightLiftThenWayBack)
{
  const auto m = kin_data::model();
  const auto p = store_like(m, 15.0);   // 칸 안 곧은 구간 15 mm < 40 mm
  std::string why, note;
  const auto out = store_recover_out_nodes(m, p.nodes, 3, 0.025, why, note);
  ASSERT_EQ(out.size(), 4u) << why << note;
  EXPECT_NE(note.find("25 mm"), std::string::npos) << note;
  EXPECT_EQ(out[0].name, "release");
  EXPECT_EQ(out[1].name, "recover_lift");
  EXPECT_EQ(out[2].name, "over") << "entry (15 mm, 같은 곧은 구간) 는 건너뜀";
  EXPECT_EQ(out[3].name, "staging");
  const Eigen::Vector3d d = tip_of(m, out[1].q) - tip_of(m, kInCell);
  EXPECT_NEAR(d.z(), 0.025, 0.001);
  EXPECT_LT(d.head<2>().norm(), 0.0005) << "곧게 위";
  EXPECT_DOUBLE_EQ(out[1].speed_dps, 12.0) << "내려온 속도 그대로";
  for (const auto & n : out) {
    EXPECT_TRUE(n.grip.empty()) << n.name << ": 쥔 채 (그리퍼 동작 없음)";
    EXPECT_FALSE(n.stop) << n.name;
  }
  EXPECT_LT(qdist(out.back().q, p.nodes.front().q), 1e-12);
}

TEST(StoreRecover, LongStraightUsesRecordedWayOnly)
{
  const auto m = kin_data::model();
  const auto p = store_like(m, 30.0);   // 곧은 구간 30 mm ≥ 올릴 높이 30 mm → 온 길 그대로
  std::string why, note;
  const auto out = store_recover_out_nodes(m, p.nodes, 3, 0.030, why, note);
  ASSERT_EQ(out.size(), 4u) << why;
  EXPECT_TRUE(note.empty()) << note;
  EXPECT_EQ(out[1].name, "entry");
  EXPECT_EQ(out[2].name, "over");
  EXPECT_EQ(out[3].name, "staging");
  EXPECT_DOUBLE_EQ(out[1].speed_dps, 12.0) << "entry ← release 구간 = 지나온 release 구간 속도";
  const auto plain = store_recover_out_nodes(m, store_like(m, 15.0).nodes, 3, 0.0, why, note);
  ASSERT_EQ(plain.size(), 4u);
  EXPECT_EQ(plain[1].name, "entry") << "lift 0 = 온 길 그대로";
}

TEST(StoreRecover, RefusesWithoutRelease)
{
  const auto m = kin_data::model();
  const auto p = store_like(m, 20.0);
  std::string why, note;
  EXPECT_TRUE(store_recover_out_nodes(m, p.nodes, 0, 0.04, why, note).empty());
  EXPECT_TRUE(store_recover_out_nodes(m, p.nodes, p.nodes.size(), 0.04, why, note).empty());
  EXPECT_FALSE(why.empty());
}

TEST(StoreRecover, ReturnedToken)
{
  const std::string t = store_returned_token(0.2153, -0.0412);
  EXPECT_EQ(t, "STORE_RETURNED_TO_BOX 0.2153 -0.0412");
  EXPECT_EQ(store_returned_token(0.21494, -0.00001), "STORE_RETURNED_TO_BOX 0.2149 0.0000") << "−0 안 씀";
  double x = 0.0, y = 0.0;
  ASSERT_TRUE(parse_store_returned("EMPTY_GRASP: 팔 작업 중단 · " + t, x, y));
  EXPECT_DOUBLE_EQ(x, 0.2153);
  EXPECT_DOUBLE_EQ(y, -0.0412);
  EXPECT_FALSE(parse_store_returned("팔 동작 실패", x, y));
  EXPECT_FALSE(parse_store_returned("STORE_RETURNED_TO_BOX nan 1", x, y));
}
