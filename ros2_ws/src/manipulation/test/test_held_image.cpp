// test_held_image.cpp — 쥔 퍽 영상 잰 값 ↔ 기준 판단 · 기준 파일 (11-11 16:4x 실물 9 장의 값)
#include <gtest/gtest.h>

#include <cstdio>
#include <deque>
#include <map>
#include <string>

#include "manipulation/held_image.hpp"

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

// 실물 표 (perception test_wrist_held 와 같은 값): 기준 = 164226 (pick_offset_x −0.5, 사용자: 잘 쥠)
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
