// 후처리 함수 테스트 — 합성 이미지로 (카메라 · 모델 없이)
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "postproc.hpp"

using namespace perception;

namespace
{

const ColorCfg COLOR{};   // 기본값 = config/perception.yaml 값
const std::vector<Tier> TIERS{{"red", 0.10, 0.40}, {"orange", 0.40, 0.70}, {"green", 0.70, 1.00}};

cv::Mat hsv_patch(int h, int s, int v)
{
  cv::Mat hsv(40, 40, CV_8UC3, cv::Scalar(h, s, v)), bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

cv::Mat tower(const std::string & lit)
{
  cv::Mat img(100, 30, CV_8UC3, cv::Scalar::all(60));
  for (const auto & t : TIERS) {
    if (t.name == lit) {img.rowRange(static_cast<int>(t.top * 100), static_cast<int>(t.bottom * 100)) = cv::Scalar::all(230);}
  }
  return img;
}

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

const cv::Matx33d K(600, 0, 320, 0, 600, 240, 0, 0, 1);

}  // namespace

TEST(Color, RedGreenBeige)
{
  EXPECT_EQ(classify_color(hsv_patch(2, 200, 180), COLOR).first, "red");
  EXPECT_EQ(classify_color(hsv_patch(176, 200, 180), COLOR).first, "red");
  EXPECT_EQ(classify_color(hsv_patch(60, 200, 180), COLOR).first, "green");
  EXPECT_EQ(classify_color(hsv_patch(20, 50, 200), COLOR).first, "beige");
}

TEST(Color, RedAcrossHueWrap)
{
  // 빨강 픽셀이 0 과 179 양쪽으로 반씩 → 단순 중앙값이면 90 근처가 나와서 틀림
  cv::Mat img;
  cv::hconcat(hsv_patch(2, 200, 180), hsv_patch(177, 200, 180), img);
  EXPECT_EQ(classify_color(img, COLOR).first, "red");
}

TEST(Color, DarkIsUnknown)
{
  EXPECT_EQ(classify_color(hsv_patch(60, 200, 10), COLOR).first, "unknown");
}

TEST(Signal, State)
{
  for (const std::string s : {"red", "orange", "green"}) {
    EXPECT_EQ(signal_state(tower(s), TIERS, 40), s);
  }
  EXPECT_EQ(signal_state(tower(""), TIERS, 40), "off");
}

TEST(Voter, MajorityOfWindow)
{
  Voter<std::string> v(5, 3);
  EXPECT_FALSE(v.push("red").first.has_value());
  v.push("green");
  v.push("red");
  const auto r = v.push("red");
  ASSERT_TRUE(r.first.has_value());
  EXPECT_EQ(*r.first, "red");
}

TEST(Tag, IdAndPose)
{
  Box box;
  const cv::Mat img = tag_image(7, 120, box);
  const auto [id, corners] = TagReader().read(img, box);
  EXPECT_EQ(id, 7);
  const auto pose = tag_pose(corners, 0.025, K, cv::Mat::zeros(1, 5, CV_64F), false);
  ASSERT_TRUE(pose.has_value());
  // 태그 한 변 120px, f=600 → 25mm 태그는 약 0.125m 앞
  EXPECT_NEAR(pose->t[2], 0.125, 0.01);
  const auto q = rot_to_quat(pose->R);
  EXPECT_NEAR(std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), 1.0, 1e-6);
}

TEST(Tag, Missing)
{
  const cv::Mat img(480, 640, CV_8UC3, cv::Scalar::all(255));
  EXPECT_EQ(TagReader().read(img, {100, 100, 200, 200}).first, -1);
}

TEST(Depth, Point)
{
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(2.0));
  depth(cv::Rect(280, 200, 80, 80)) = 0.5;   // 퍽
  const auto p = depth_point(depth, {280, 200, 360, 280}, K);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR((*p)[0], 0.0, 1e-3);
  EXPECT_NEAR((*p)[1], 0.0, 1e-3);
  EXPECT_NEAR((*p)[2], 0.5, 1e-3);
}

TEST(Barcode, Ean13)
{
  // EAN-13 4006381333931 을 직접 그림
  const std::vector<std::string> L{"0001101", "0011001", "0010011", "0111101", "0100011",
    "0110001", "0101111", "0111011", "0110111", "0001011"};
  const std::vector<std::string> G{"0100111", "0110011", "0011011", "0100001", "0011101",
    "0111001", "0000101", "0010001", "0001001", "0010111"};
  const std::vector<std::string> R{"1110010", "1100110", "1101100", "1000010", "1011100",
    "1001110", "1010000", "1000100", "1001000", "1110100"};
  const std::vector<std::string> PARITY{"LLLLLL", "LLGLGG", "LLGGLG", "LLGGGL", "LGLLGG",
    "LGGLLG", "LGGGLL", "LGLGLG", "LGLGGL", "LGGLGL"};
  const std::string code = "4006381333931";
  std::string bits = "101";
  for (int i = 0; i < 6; ++i) {
    const int digit = code[1 + i] - '0';
    bits += (PARITY[code[0] - '0'][i] == 'L' ? L : G)[digit];
  }
  bits += "01010";
  for (int i = 7; i < 13; ++i) {bits += R[code[i] - '0'];}
  bits += "101";

  BarcodeReader reader;
  for (int m : {3, 6}) {   // 6 은 폭 570px 로 화면을 거의 채움
    cv::Mat img(480, 640, CV_8UC1, cv::Scalar(255));
    const int x0 = (640 - static_cast<int>(bits.size()) * m) / 2;
    for (size_t i = 0; i < bits.size(); ++i) {
      if (bits[i] == '1') {img(cv::Rect(x0 + static_cast<int>(i) * m, 100, m, 280)) = 0;}
    }
    cv::GaussianBlur(img, img, cv::Size(0, 0), 1.0);   // 실제 카메라처럼 경계를 약간 흐리게
    cv::Mat bgr;
    cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
    const auto found = reader.read(bgr);
    ASSERT_FALSE(found.empty()) << "m=" << m;
    EXPECT_EQ(found.front().first, code) << "m=" << m;
  }
}

TEST(Letterbox, AndNms)
{
  // 1280x720 → 960x960: 배율 0.75, 위아래 210 px 채움 (ultralytics 와 같음)
  double r;
  int px, py;
  const cv::Mat out = letterbox(cv::Mat(720, 1280, CV_8UC3, cv::Scalar::all(0)), {960, 960}, r, px, py);
  EXPECT_EQ(out.size(), cv::Size(960, 960));
  EXPECT_DOUBLE_EQ(r, 0.75);
  EXPECT_EQ(px, 0);
  EXPECT_EQ(py, 210);

  // 같은 클래스의 겹친 상자는 점수 높은 것만, 다른 클래스는 남김
  std::vector<Detection> d{{{0, 0, 10, 10}, 0.9f, 0}, {{1, 1, 10, 10}, 0.8f, 0}, {{1, 1, 10, 10}, 0.7f, 1}};
  const auto k = nms(d, 0.7f);
  ASSERT_EQ(k.size(), 2u);
  EXPECT_FLOAT_EQ(k[0].score, 0.9f);
  EXPECT_EQ(k[1].cls, 1);
}
