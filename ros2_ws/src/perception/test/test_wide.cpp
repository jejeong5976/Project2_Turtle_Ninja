// test_wide.cpp — 광각 시험: 상자 후처리 · 퍽 태그 · 신호등 칸 판정, 합성 출력 · 영상으로 엔진 · 카메라 없이
//   실행: colcon build → source install/setup.bash → ./build/perception/test_wide

#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/wide_detector.hpp"
#include "perception/wide_tag_reader.hpp"
#include "perception/traffic_light.hpp"

using namespace perception::wide;

namespace
{
cv::Mat tag_image(int id, int side, Box & box)
{
  cv::Mat tag;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, side, tag);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, side, tag);
#endif
  cv::Mat img(480, 640, CV_8UC1, cv::Scalar(255));
  tag.copyTo(img(cv::Rect(300, 200, side, side)));
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  box = {300.f, 200.f, 300.f + side, 200.f + side};
  return bgr;
}
}  // namespace

TEST(Box, LetterboxAndNms)
{
  // 1280x720 → 960x960: 배율 0.75, 위아래 210 px 채움 (ultralytics 와 같음)
  LetterboxInfo lb;
  const cv::Mat out = letterbox(cv::Mat(720, 1280, CV_8UC3, cv::Scalar::all(0)), {960, 960}, lb);
  EXPECT_EQ(out.size(), cv::Size(960, 960));
  EXPECT_DOUBLE_EQ(lb.r, 0.75);
  EXPECT_EQ(lb.pad_x, 0);
  EXPECT_EQ(lb.pad_y, 210);

  // 같은 클래스의 겹친 상자는 점수 높은 것만, 다른 클래스는 남김
  std::vector<BoxDetection> d{{{0, 0, 10, 10}, 0.9f, 0}, {{1, 1, 10, 10}, 0.8f, 0}, {{1, 1, 10, 10}, 0.7f, 1}};
  const auto k = nms(d, 0.7f);
  ASSERT_EQ(k.size(), 2u);
  EXPECT_FLOAT_EQ(k[0].score, 0.9f);
  EXPECT_EQ(k[1].cls, 1);
}

TEST(Box, ToChwIsRgb)
{
  cv::Mat bgr(2, 3, CV_8UC3, cv::Scalar(10, 20, 255));   // B G R
  std::vector<float> chw;
  to_chw(bgr, chw);
  ASSERT_EQ(chw.size(), 18u);
  EXPECT_FLOAT_EQ(chw[0], 1.0f);            // R 판이 먼저
  EXPECT_FLOAT_EQ(chw[6], 20 / 255.f);      // G
  EXPECT_FLOAT_EQ(chw[12], 10 / 255.f);     // B
}

TEST(Box, DecodeEndToEnd)
{
  // [1, 300, 6] (x1, y1, x2, y2, score, class), 입력 960x960 · 원본 1280x720 (배율 0.75, 위 210)
  LetterboxInfo lb{0.75, 0, 210};
  std::vector<float> out(300 * 6, 0.f);
  const float a[6] = {75, 285, 150, 360, 0.9f, 3};   // 원본 (100, 100)–(200, 200)
  const float b[6] = {0, 0, 10, 10, 0.2f, 1};        // conf 미만
  std::copy(a, a + 6, out.begin());
  std::copy(b, b + 6, out.begin() + 6);
  const auto d = decode(out, {1, 300, 6}, lb, {1280, 720}, 0.5f, 0.7f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].cls, 3);
  EXPECT_NEAR(d[0].box.x1, 100, 1e-3);
  EXPECT_NEAR(d[0].box.y1, 100, 1e-3);
  EXPECT_NEAR(d[0].box.x2, 200, 1e-3);
  EXPECT_NEAR(d[0].box.y2, 200, 1e-3);
}

TEST(Box, DecodeChannelsFirstWithNms)
{
  // [1, 4+2, 3]: 후보 0 · 1 은 같은 자리 같은 클래스 → NMS 로 하나, 후보 2 는 점수 미만
  const int C = 6, N = 3;
  std::vector<float> out(C * N, 0.f);
  auto set = [&](int j, float cx, float cy, float w, float h, float s0, float s1) {
      out[0 * N + j] = cx; out[1 * N + j] = cy; out[2 * N + j] = w; out[3 * N + j] = h;
      out[4 * N + j] = s0; out[5 * N + j] = s1;
    };
  set(0, 50, 50, 20, 20, 0.1f, 0.8f);
  set(1, 51, 50, 20, 20, 0.1f, 0.7f);
  set(2, 10, 10, 5, 5, 0.3f, 0.2f);
  const auto d = decode(out, {1, C, N}, LetterboxInfo{1.0, 0, 0}, {100, 100}, 0.5f, 0.7f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].cls, 1);
  EXPECT_FLOAT_EQ(d[0].score, 0.8f);
  EXPECT_NEAR(d[0].box.x1, 40, 1e-3);
}

TEST(Box, DecodeBadShapeThrows)
{
  EXPECT_THROW(decode({}, {1, 3}, LetterboxInfo{}, {10, 10}, 0.5f, 0.7f), std::runtime_error);
}

TEST(Tag, PuckId)
{
  Box box;
  const cv::Mat img = tag_image(7, 120, box);
  const auto [id, corners] = PuckTagReader().read(img, box);
  EXPECT_EQ(id, 7);
  EXPECT_EQ(corners.size(), 4u);
}

TEST(Tag, PlaceTagIgnored)
{
  // 퍽 상자 안에 장소 태그(12–14)만 보이면 퍽 ID 로 쓰지 않음
  Box box;
  const cv::Mat img = tag_image(13, 120, box);
  EXPECT_EQ(PuckTagReader().read(img, box).first, -1);
}

TEST(Tag, Missing)
{
  const cv::Mat img(480, 640, CV_8UC3, cv::Scalar::all(255));
  EXPECT_EQ(PuckTagReader().read(img, {100, 100, 200, 200}).first, -1);
}

// ---------------------------------------------------------------- 신호등 칸 판정 (합성 신호등 이미지)

namespace
{
using perception::TrafficLightTier;
using perception::classify_traffic_light;
const std::vector<TrafficLightTier> TIERS{{"red", 0.10, 0.40}, {"yellow", 0.40, 0.70}, {"green", 0.70, 1.00}};

cv::Mat tower(const std::string & lit)
{
  cv::Mat img(100, 30, CV_8UC3, cv::Scalar::all(60));
  for (const auto & t : TIERS) {
    if (t.name == lit) {img.rowRange(static_cast<int>(t.top * 100), static_cast<int>(t.bottom * 100)) = cv::Scalar::all(230);}
  }
  return img;
}
}  // namespace

TEST(TrafficLight, LitTier)
{
  for (const std::string s : {"red", "yellow", "green"}) {
    EXPECT_EQ(classify_traffic_light(tower(s), TIERS, 40), s);
  }
}

TEST(TrafficLight, AllOffIsOff)
{
  EXPECT_EQ(classify_traffic_light(tower(""), TIERS, 40), "off");
}

TEST(TrafficLight, EmptyRoiIsUnknown)
{
  // 상자를 못 잘랐으면 꺼짐(OFF)으로 보지 않음
  EXPECT_EQ(classify_traffic_light(cv::Mat(), TIERS, 40), "unknown");
}
