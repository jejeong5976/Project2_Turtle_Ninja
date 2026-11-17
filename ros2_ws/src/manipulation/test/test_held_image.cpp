// test_held_image.cpp — 쥔 퍽 영상 잰 값 ↔ 기준 판단 · 기준 파일 (실제 로봇 영상 9 장의 값)
#include <gtest/gtest.h>

#include <cstdio>
#include <deque>
#include <map>
#include <string>

#include "manipulation/held_image.hpp"

#include <cmath>

#include "arm_kin_data.hpp"

using namespace manipulation;

namespace
{
HeldImageSample S(double top_y, double top_deg, double left_x, double left_deg, bool left_ok = true, double stamp = 0.0)
{
  HeldImageSample s;
  s.stamp = stamp;
  s.top_y = top_y;
  s.top_deg = top_deg;
  s.left_x = left_x;
  s.left_deg = left_deg;
  s.left_ok = left_ok;
  s.bottom_y = 479;
  return s;
}

HeldImageRef ref164226()
{
  HeldImageRef r;
  r.ok = true;
  r.s = S(204.6, 7.9, 419.5, -1.3);
  r.n = 3;
  return r;
}
}  // namespace

// 실물 표 (perception test_wrist_held 와 같은 값): 기준 = 164226 (pick_offset_x −0.5, 실물에서 잘 쥠)
TEST(HeldImage, RealTableVerdicts)
{
  const auto ref = ref164226();
  HeldImageConfig c;   // 앞 3 · 뒤 6 · 기울기 5°
  struct Row {const char * name; HeldImageSample s; bool bad; double front;};
  const Row rows[] = {
    {"164514 −0.5 처음", S(208.7, 11.7, 414.7, -4.0), false, 0.8},
    {"164735 +4.5 앞", S(230.5, 13.9, 413.9, -3.3), true, 5.0},
    {"165025 −5.5 뒤", S(131.2, 17.7, 426.1, -2.0), false, -5.0},
    {"164534 다시 쥠 −7", S(137.9, 18.0, 426.9, -2.8), false, -4.5},
    {"164244 대각선", S(257.7, -50.5, 570.5, -31.2), true, 10.2},
    {"164015 처음 (흐림)", S(197.5, 9.2, 414.4, -11.9), true, -0.5},   // 흐린 장: 왼쪽 모서리 −10.6° → 기울기 넘음 (정지에서 다시 볼 것)
    {"1422 14:22 +4.5 빨강", S(199.8, 11.5, 414.5, -4.8), false, -0.3},  // 이미지로는 가운데와 같음
  };
  for (const auto & r : rows) {
    const auto v = judge_held_image(r.s, 3, ref, c);
    ASSERT_TRUE(v.judged) << r.name;
    EXPECT_EQ(v.bad, r.bad) << r.name << ": " << v.text;
    EXPECT_NEAR(v.front_mm, r.front, 0.3) << r.name;
  }
  // 164244 는 기울기로도 걸림
  const auto d = judge_held_image(S(257.7, -50.5, 570.5, -31.2), 1, ref, c);
  EXPECT_NEAR(d.tilt_deg, -29.9, 0.1);
  EXPECT_FALSE(d.tilt_from_top);
}

TEST(HeldImage, BackTolDefaultsToTolWhenZero)
{
  const auto ref = ref164226();
  HeldImageConfig c;
  c.back_tol_mm = 0.0;   // = tol_mm 3
  const auto v = judge_held_image(S(131.2, 17.7, 426.1, -2.0), 2, ref, c);
  EXPECT_TRUE(v.bad);
  EXPECT_NE(v.text.find("뒤로 쥠 허용 넘음"), std::string::npos);
}

TEST(HeldImage, TiltFallsBackToTopEdge)
{
  const auto ref = ref164226();
  HeldImageConfig c;
  const auto v = judge_held_image(S(206.0, 15.0, 0.0, 0.0, false), 2, ref, c);   // 왼쪽 모서리 못 잼 → 위 모서리 각 차 7.1°
  ASSERT_TRUE(v.judged);
  EXPECT_TRUE(v.tilt_from_top);
  EXPECT_NEAR(v.tilt_deg, 7.1, 0.01);
  EXPECT_TRUE(v.bad);
}

TEST(HeldImage, NoReferenceOrNoSamples)
{
  HeldImageConfig c;
  EXPECT_FALSE(judge_held_image(S(200, 0, 400, 0), 3, HeldImageRef{}, c).judged);
  EXPECT_FALSE(judge_held_image(S(200, 0, 400, 0), 0, ref164226(), c).judged);
}

TEST(HeldImage, MedianAndWindow)
{
  std::deque<HeldImageSample> buf;
  buf.push_back(S(300, 0, 400, 0, true, 0.9));    // 멈추기 전 (흔들림)
  buf.push_back(S(205, 8, 420, -1, true, 1.15));
  buf.push_back(S(207, 9, 418, -2, false, 1.20));   // 왼쪽 못 잼
  buf.push_back(S(204, 7, 419, -1.5, true, 1.25));
  buf.push_back(S(100, 0, 400, 0, true, 2.0));    // 판단 뒤
  const auto w = samples_between(buf, 1.1, 1.3);
  ASSERT_EQ(w.size(), 3u);
  const auto m = median_sample(w);
  EXPECT_DOUBLE_EQ(m.top_y, 205.0);
  EXPECT_DOUBLE_EQ(m.top_deg, 8.0);
  EXPECT_TRUE(m.left_ok);
  EXPECT_DOUBLE_EQ(m.left_x, 419.5);   // 왼쪽 잰 2 장의 중앙값
  EXPECT_DOUBLE_EQ(m.stamp, 1.25);
  EXPECT_FALSE(median_sample({}).left_ok);
}

TEST(HeldImage, RefFileRoundTrip)
{
  const std::string path = testing::TempDir() + "held_image_ref_test/ref.txt";
  std::map<std::string, HeldImageRef> refs;
  refs["box"] = ref164226();
  refs["box"].when = "2026-11-11T17:00:00";
  HeldImageRef cyl;
  cyl.ok = true;
  cyl.s = S(190.0, 5.0, 0.0, 0.0, false);
  cyl.n = 4;
  refs["cylinder"] = cyl;
  std::string err;
  ASSERT_TRUE(save_held_image_refs(path, refs, err)) << err;
  const auto back = load_held_image_refs(path, err);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back.at("box").s.top_y, 204.6, 1e-6);
  EXPECT_NEAR(back.at("box").s.left_deg, -1.3, 1e-6);
  EXPECT_TRUE(back.at("box").s.left_ok);
  EXPECT_EQ(back.at("box").n, 3);
  EXPECT_EQ(back.at("box").when, "2026-11-11T17:00:00");
  EXPECT_FALSE(back.at("cylinder").s.left_ok);
  std::remove(path.c_str());
  EXPECT_TRUE(load_held_image_refs(path, err).empty());
}

// ---- 쥔 그대로 보정 (held_image_compensate) ----
namespace
{
HeldImageVerdict V(double front_mm, double d_left_px, double tilt_deg, bool from_top = false)
{
  HeldImageVerdict v;
  v.judged = true;
  v.bad = true;
  v.front_mm = front_mm;
  v.d_left_px = d_left_px;
  v.tilt_deg = tilt_deg;
  v.tilt_from_top = from_top;
  return v;
}
// 수직 손 · 닫힘축 base +y (J5 기본, 정면 퍽) 자세: 모델 base (x, 0, z) 손끝. 손 x 가 base +x 인 풀이 (실제 로봇 영상 기준:
//   pick_offset_x + → 위 모서리 +26 px = 영상 아래 = 손 −x 로 퍽 → 그때 손 x = base +x. J5 를 180° 돌린 풀이는 반대)
bool vertical_pose(const manipulation::ArmModel & m, double x, Q5 & q)
{
  for (double z : {0.0, 0.03, 0.06, -0.03}) {
    for (double r : {180.0, 90.0}) {
      const auto sols = arm_ik(m, Eigen::Vector3d(x, 0.0, z), Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 1, 0), {}, r);
      for (const auto & s : sols) {
        const Eigen::Vector3d hx = arm_frames(m, s.q)[4].block<3, 3>(0, 0) * tcp_hand_to_link5(m, Eigen::Vector3d::UnitX());
        if (s.pos_mm < 0.5 && s.app_deg < 0.5 && hx.x() > 0.999) {q = s.q; return true;}
      }
    }
  }
  return false;
}
Eigen::Matrix3d R5(const manipulation::ArmModel & m, const Q5 & q) {return arm_frames(m, q)[4].block<3, 3>(0, 0);}
}  // namespace

// 판단 → 손 좌표 어긋남: 앞으로 쥠 4.2 mm (실제 로봇 S5) = 퍽 중심이 손 −x 로 4.2 mm. 옆 = −왼쪽 모서리 px / px/mm
TEST(HeldImageComp, VerdictToHandOffset)
{
  const auto c = held_image_comp(V(4.2, 0.0, 1.0), 8.5, 10.0);
  ASSERT_TRUE(c.ok);
  EXPECT_NEAR(c.hand_m.x(), -0.0042, 1e-12);
  EXPECT_NEAR(c.hand_m.y(), 0.0, 1e-12);
  EXPECT_EQ(c.hand_m.z(), 0.0);
  EXPECT_TRUE(c.lateral_used);
  const auto b = held_image_comp(V(-6.5, 17.0, 2.0), 8.5, 10.0);   // 뒤로 쥠 · 왼쪽 모서리 영상 오른쪽 17 px → 퍽 손 −y 2 mm
  EXPECT_NEAR(b.hand_m.x(), 0.0065, 1e-12);
  EXPECT_NEAR(b.lateral_mm, -2.0, 1e-12);
  EXPECT_NEAR(b.hand_m.y(), -0.002, 1e-12);
  // 옆을 안 쓰는 경우: px/mm 0 · 왼쪽 모서리 못 잼 · 기울어 쥠 (왼쪽 모서리가 돌아 보임)
  for (const auto & x : {held_image_comp(V(4.2, 17.0, 1.0), 0.0, 10.0), held_image_comp(V(4.2, 17.0, 1.0, true), 8.5, 10.0),
      held_image_comp(V(4.2, 17.0, -12.0), 8.5, 10.0)})
  {
    EXPECT_TRUE(x.ok);
    EXPECT_FALSE(x.lateral_used);
    EXPECT_EQ(x.hand_m.y(), 0.0);
    EXPECT_NEAR(x.hand_m.x(), -0.0042, 1e-12);
    EXPECT_FALSE(x.lateral_note.empty());
  }
  HeldImageVerdict none;   // 기준 없음 · 못 잼
  EXPECT_FALSE(held_image_comp(none, 8.5, 10.0).ok);
}

// 부호를 URDF 카메라로 확인 (camera_joint rpy, wrist_link = 모델 link5): 영상 아래 ≈ 손 −x (위 모서리가 내려가면 = 앞으로 쥠 = 퍽이 손 −x),
//   영상 오른쪽 ≈ 손 −y (왼쪽 모서리가 오른쪽으로 = 퍽이 손 −y). 손 좌표 = tcp_hand_to_link5
TEST(HeldImageComp, SignMatchesUrdfCamera)
{
  const auto m = kin_data::model();
  const Eigen::Matrix3d Rc = (Eigen::AngleAxisd(-1.995706, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.048951, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-2.969292, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Vector3d img_right = Rc.col(0), img_down = Rc.col(1);
  const Eigen::Vector3d hx = tcp_hand_to_link5(m, Eigen::Vector3d::UnitX()), hy = tcp_hand_to_link5(m, Eigen::Vector3d::UnitY());
  std::printf("영상 아래 · 손 x = %.3f · 영상 오른쪽 · 손 y = %.3f\n", img_down.dot(hx), img_right.dot(hy));
  EXPECT_LT(img_down.dot(hx), -0.7);
  EXPECT_LT(img_right.dot(hy), -0.95);
  // 앞으로 쥠 → 퍽 중심이 손 −x = 영상 아래 쪽 (위 모서리 y 커짐)
  const auto c = held_image_comp(V(4.2, 0.0, 0.0), 8.5, 10.0);
  EXPECT_GT(img_down.dot(tcp_hand_to_link5(m, c.hand_m)), 0.0);
  // 왼쪽 모서리가 영상 오른쪽으로 (+px) → 퍽이 영상 오른쪽
  const auto l = held_image_comp(V(0.0, 17.0, 0.0), 8.5, 10.0);
  EXPECT_GT(img_right.dot(tcp_hand_to_link5(m, l.hand_m)), 0.0);
}

// pick_offset_x + (= 손이 퍽보다 base +x 앞) 로 수직 손 · 닫힘축 +y 로 쥔 퍽 = 앞으로 쥠: 손 좌표 어긋남이 (−δ, 0, 0) 과 같고,
//   어떤 놓는 자세 (다른 J1 · 기울임) 에서도 손끝을 shift 만큼 옮기면 퍽 중심 xy 가 원래 손끝 자리 (칸 · 목표) 에 옴
TEST(HeldImageComp, ShiftPutsPuckOnTarget)
{
  const auto m = kin_data::model();
  Q5 qp;
  ASSERT_TRUE(vertical_pose(m, 0.25, qp));
  const Eigen::Matrix3d Rp = R5(m, qp);
  const Eigen::Vector3d hx_base = Rp * tcp_hand_to_link5(m, Eigen::Vector3d::UnitX());
  EXPECT_GT(hx_base.x(), 0.999);   // 정면 수직 손: 손 x = base +x (pick_offset_x + 쪽)
  const double delta = 0.0042;
  // 퍽 중심 = 손끝 − δ x̂ (손이 앞으로 δ) → 손 좌표
  const Eigen::Vector3d puck_in_link5 = Rp.transpose() * Eigen::Vector3d(-delta, 0.0, 0.0);
  const auto c = held_image_comp(V(delta * 1000.0, 0.0, 0.0), 8.5, 10.0);
  EXPECT_NEAR((tcp_hand_to_link5(m, c.hand_m) - puck_in_link5).norm(), 0.0, 5e-5);   // IK 손 방향 0.5° 안
  const Eigen::Vector3d puck_hand = tcp_hand_to_link5(m, c.hand_m);
  // 정면 수직 손에서 놓으면: 손끝을 +x 로 δ
  const Eigen::Vector2d s0 = held_image_comp_shift(m, Rp, c.hand_m);
  EXPECT_NEAR(s0.x(), delta, 5e-5);
  EXPECT_NEAR(s0.y(), 0.0, 5e-5);
  // 다른 놓는 자세 (J1 · J4 · J5 를 돌림): 옮긴 손끝 + 손 안 퍽 = 원래 손끝 (xy)
  for (const auto & dq : std::vector<Q5>{{0.6, 0, 0, 0, 0}, {-1.2, 0.1, -0.1, 0.15, 0.3}, {2.0, 0, 0, -0.2, -0.5}}) {
    Q5 q = qp;
    for (int j = 0; j < 5; ++j) {q[j] += dq[j];}
    const Eigen::Matrix3d R = R5(m, q);
    const Eigen::Vector2d s = held_image_comp_shift(m, R, c.hand_m);
    const Eigen::Vector2d tip(0.1, 0.05), puck = tip + s + (R * puck_hand).head<2>();
    EXPECT_NEAR((puck - tip).norm(), 0.0, 1e-12);
    EXPECT_NEAR(s.norm(), (R * puck_in_link5).head<2>().norm(), 5e-5);
  }
  // 옆도 같은 식 (손 +y 로 2 mm 비킨 퍽)
  const auto l = held_image_comp(V(0.0, -17.0, 0.0), 8.5, 10.0);
  const Eigen::Vector2d sl = held_image_comp_shift(m, Rp, l.hand_m);
  EXPECT_NEAR(sl.y(), -0.002, 1e-4);   // 정면 수직 손: 손 y = base +y → 손끝을 −y
}

// 보정 없음 (0) 이면 옮김도 정확히 0 (기본 false = 보정 없음의 바탕)
TEST(HeldImageComp, ZeroOffsetIsExactlyZero)
{
  const auto m = kin_data::model();
  Q5 qp;
  ASSERT_TRUE(vertical_pose(m, 0.25, qp));
  const Eigen::Vector2d s = held_image_comp_shift(m, R5(m, qp), Eigen::Vector3d::Zero());
  EXPECT_EQ(s.x(), 0.0);
  EXPECT_EQ(s.y(), 0.0);
  const auto c = held_image_comp(V(0.0, 0.0, 0.0), 8.5, 10.0);
  EXPECT_TRUE(c.hand_m.isZero(0.0));
}

// ── 기울기로 다시 쥘 모양 (held_image_regrasp_tilt_shapes = ["cylinder"]): 원기둥 기울기 > 허용 이면 쥔 그대로 보정 대신 박스에서 다시 쥠 ──
//   실제 로봇 값: 원기둥 +8.7 · +9.7 · +11.8 · +12.3 · +23.7°, 네모 −1.3 … +2.6° (허용 held_image_tilt_deg 10, held_image_compensate true)
TEST(HeldImageTiltRegrasp, CylinderTiltOverTolOnly)
{
  HeldImageConfig c;
  c.tol_mm = 10.0;
  c.back_tol_mm = 10.0;
  c.tilt_deg = 10.0;
  const auto ref = ref164226();
  const std::vector<std::string> shapes{"cylinder"};
  struct Row {double tilt; bool want_cyl;};
  int n = 0;
  for (const Row r : {Row{8.7, false}, Row{9.7, false}, Row{11.8, true}, Row{12.3, true}, Row{23.7, true}, Row{-12.0, true}}) {
    const auto v = judge_held_image(S(204.6, 7.9, 419.5, -1.3 + r.tilt), 2, ref, c);   // 앞뒤 0 · 왼쪽 모서리 각 차 = 기울기
    ASSERT_TRUE(v.judged);
    EXPECT_NEAR(v.tilt_deg, r.tilt, 1e-9);
    EXPECT_EQ(v.tilt_bad, std::fabs(r.tilt) > 10.0);
    EXPECT_EQ(v.bad, std::fabs(r.tilt) > 10.0);   // 허용 판단은 바뀌지 않음
    EXPECT_EQ(held_image_tilt_regrasp(v, "cylinder", shapes), r.want_cyl) << r.tilt;
    EXPECT_FALSE(held_image_tilt_regrasp(v, "box", shapes)) << r.tilt;   // 네모 = 쥔 그대로 보정
    n += held_image_tilt_regrasp(v, "cylinder", shapes) ? 1 : 0;
  }
  EXPECT_EQ(n, 4);
}

// 앞뒤만 넘은 원기둥 · 판단 못 함 · 목록 비었음 · 모양 모름 = 다시 쥐지 않음
TEST(HeldImageTiltRegrasp, FrontOnlyUnjudgedOrNoListKeepsCompensate)
{
  HeldImageConfig c;
  c.tol_mm = 10.0;
  c.back_tol_mm = 10.0;
  c.tilt_deg = 10.0;
  const auto ref = ref164226();
  const std::vector<std::string> shapes{"cylinder"};
  const auto front = judge_held_image(S(204.6 + 5.2 * 12.0, 7.9, 419.5, -1.3 + 2.0), 2, ref, c);   // 12 mm 앞 · 기울기 2°
  ASSERT_TRUE(front.bad);
  EXPECT_FALSE(front.tilt_bad);
  EXPECT_FALSE(held_image_tilt_regrasp(front, "cylinder", shapes));
  const auto both = judge_held_image(S(204.6 + 5.2 * 12.0, 7.9, 419.5, -1.3 + 15.0), 2, ref, c);   // 앞 · 기울기 둘 다 넘음 → 기울기로 다시 쥠
  EXPECT_TRUE(held_image_tilt_regrasp(both, "cylinder", shapes));
  const auto none = judge_held_image(S(200, 0, 400, 0), 0, ref, c);   // 장 없음
  EXPECT_FALSE(none.judged);
  EXPECT_FALSE(held_image_tilt_regrasp(none, "cylinder", shapes));
  const auto tilt = judge_held_image(S(204.6, 7.9, 419.5, -1.3 + 15.0), 2, ref, c);
  EXPECT_FALSE(held_image_tilt_regrasp(tilt, "cylinder", {}));
  EXPECT_FALSE(held_image_tilt_regrasp(tilt, "", shapes));
  EXPECT_TRUE(held_image_tilt_regrasp(tilt, "box", {"box", "cylinder"}));   // 목록에 넣으면 네모도
  // 위 모서리로 잰 기울기 (왼쪽 모서리 못 잼) 도 같게
  const auto top = judge_held_image(S(206.0, 7.9 + 13.0, 0.0, 0.0, false), 2, ref, c);
  EXPECT_TRUE(top.tilt_from_top);
  EXPECT_TRUE(held_image_tilt_regrasp(top, "cylinder", shapes));
}
