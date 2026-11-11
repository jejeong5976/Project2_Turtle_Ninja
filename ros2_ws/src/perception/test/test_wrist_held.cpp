// test_wrist_held.cpp — 쥔 퍽 마스크 재기 시험 (합성 + 11-11 실물 분할 마스크 6 장, test/data)
//   실행: colcon build → ./build/perception/test_wrist_held

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/wrist_held.hpp"

using namespace perception::wrist;

namespace
{
cv::Mat1b load(const std::string & name)
{
  cv::Mat m = cv::imread(std::string(HELD_DATA_DIR) + "/held_" + name + "_mask.png", cv::IMREAD_GRAYSCALE);
  return m.empty() ? cv::Mat1b() : cv::Mat1b(m > 127);
}

SegInstance inst(const cv::Mat1b & mask, float conf, const std::string & kind = "green_box_up")
{
  SegInstance s;
  s.mask = mask;
  s.conf = conf;
  s.kind = kind;
  return s;
}
}  // namespace

TEST(WristHeld, SyntheticRectangle)
{
  cv::Mat1b m = cv::Mat1b::zeros(480, 640);
  cv::rectangle(m, cv::Rect(420, 200, 220, 280), 255, cv::FILLED);   // 위 200, 왼쪽 420, 오른쪽 끝까지
  HeldParams p;
  const auto r = measure_held(m, p);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_NEAR(r.top_y, 200.0, 0.5);
  EXPECT_NEAR(r.top_deg, 0.0, 0.1);
  EXPECT_NEAR(r.left_x, 420.0, 0.5);
  EXPECT_NEAR(r.left_deg, 0.0, 0.1);
  EXPECT_NEAR(r.bottom_y, 479.0, 0.5);
  EXPECT_EQ(r.bbox[0], 420);
  EXPECT_EQ(r.bbox[1], 200);
}

TEST(WristHeld, SlantedEdgeAngles)
{
  cv::Mat1b m = cv::Mat1b::zeros(480, 640);
  // 위 모서리: 오른쪽으로 갈수록 10° 아래, 왼쪽 모서리: 아래로 갈수록 10° 오른쪽 (실물처럼 맨 왼쪽 = 위 왼쪽 모서리 —
  //   실물은 손가락이 아래 왼쪽을 가려 위 모서리 열이 왼쪽 모서리에서 시작)
  const double t = std::tan(10.0 * M_PI / 180.0);
  std::vector<cv::Point> pts{{420, 200}, {639, static_cast<int>(std::lround(200 + 219 * t))}, {639, 479},
    {static_cast<int>(std::lround(420 + 279 * t)), 479}};
  cv::fillConvexPoly(m, pts, 255);
  HeldParams p;
  p.finger_y = 480;
  const auto r = measure_held(m, p);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_NEAR(r.top_deg, 10.0, 1.5);
  EXPECT_NEAR(r.left_deg, 10.0, 1.5);
}

TEST(WristHeld, PickHeldChoosesRightLargeMask)
{
  cv::Mat1b held = cv::Mat1b::zeros(480, 640), far = cv::Mat1b::zeros(480, 640), small = cv::Mat1b::zeros(480, 640);
  cv::rectangle(held, cv::Rect(420, 200, 220, 280), 255, cv::FILLED);
  cv::rectangle(far, cv::Rect(50, 50, 200, 200), 255, cv::FILLED);        // 왼쪽 위 (박스 안 다른 퍽)
  cv::rectangle(small, cv::Rect(600, 400, 40, 40), 255, cv::FILLED);      // 오른쪽 끝이지만 작음
  HeldParams p;
  std::vector<SegInstance> v{inst(far, 0.9f), inst(small, 0.9f), inst(held, 0.25f, "green_box_lying")};
  EXPECT_EQ(pick_held(v, p), 2);
  const auto r = measure_held(v, p);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.kind, "green_box_lying");
  p.conf = 0.3f;   // 신뢰도 문턱 위로 → 후보 없음
  EXPECT_EQ(pick_held(v, p), -1);
  EXPECT_FALSE(measure_held(v, p).ok);
}

// 11-11 16:4x 실물 (같은 초록 네모, 박스에서 수직 PICK, 쥔 뒤 1.2 s 움직이는 중): 노트북 Python 분석 표와 같은 값
TEST(WristHeld, RealMasksMatchTable)
{
  HeldParams p;
  struct Row {const char * name; double top_y, left_deg;};
  const Row rows[] = {
    {"164226", 204.6, -1.3},    // pick_offset_x −0.5 (가운데, 사용자: 잘 쥠)
    {"164514", 208.7, -4.0},    // −0.5 처음
    {"164735", 230.5, -3.3},    // +4.5 (5 mm 앞으로 쥠 — 사용자: 대각선과 같게 잡힘)
    {"165025", 131.2, -2.0},    // −5.5 (5 mm 뒤, 넣기 잘 됨)
    {"164534", 137.9, -2.8},    // 박스에서 다시 쥠 (regrasp_offset −7, 성공)
    {"164244", 257.7, -31.2},   // 박스에서 다시 쥠 +10 mm 앞 (대각선)
  };
  for (const auto & row : rows) {
    const auto m = load(row.name);
    ASSERT_FALSE(m.empty()) << row.name;
    const auto r = measure_held(m, p);
    ASSERT_TRUE(r.ok) << row.name << ": " << r.why;
    EXPECT_NEAR(r.top_y, row.top_y, 1.0) << row.name;
    EXPECT_NEAR(r.left_deg, row.left_deg, 1.0) << row.name;
  }
}

// 기준 (164226) 대비: 앞 · 대각선 · 뒤 세 무리가 갈리는가 (control_master 판단과 같은 문턱 — 앞 5.2 px/mm · 뒤 14.7 px/mm)
TEST(WristHeld, RealMasksSeparateGroups)
{
  HeldParams p;
  const auto ref = measure_held(load("164226"), p);
  ASSERT_TRUE(ref.ok);
  auto front_mm = [&](const char * n) {
      const auto r = measure_held(load(n), p);
      const double d = r.top_y - ref.top_y;
      return d >= 0 ? d / 5.2 : d / 14.7;
    };
  auto tilt = [&](const char * n) {return measure_held(load(n), p).left_deg - ref.left_deg;};
  EXPECT_LT(std::fabs(front_mm("164514")), 3.0);   // 가운데끼리
  EXPECT_LT(std::fabs(tilt("164514")), 5.0);
  EXPECT_GT(front_mm("164735"), 3.0);              // 5 mm 앞 → 앞 허용 넘음
  EXPECT_GT(std::fabs(tilt("164244")), 5.0);       // 대각선 → 기울기 넘음
  EXPECT_GT(front_mm("164244"), 3.0);
  EXPECT_LT(front_mm("165025"), -3.0);             // 5 mm 뒤 → 뒤 (−5 mm 근처)
  EXPECT_GT(front_mm("165025"), -6.0);             //   뒤 허용 6 mm 안 (넣기 잘 됨)
  EXPECT_GT(front_mm("164534"), -6.0);
}
