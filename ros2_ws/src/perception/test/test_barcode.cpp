// test_barcode.cpp — 바코드 판독 시험, 합성 EAN-13 · Code128 영상으로 카메라 없이
//   실행: colcon build → source install/setup.bash → ./build/perception/test_barcode

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "barcode.hpp"

#ifdef PERCEPTION_HAVE_ZXING
#include <ZXing/BitMatrix.h>
#include <ZXing/MultiFormatWriter.h>
#endif

using perception::BarcodeReader;

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
    // 자리 (화소 상자) — 그린 막대 가운데 (320, 240) 를 품어야 함
    const auto located = reader.read_located(bgr);
    ASSERT_FALSE(located.empty()) << "m=" << m;
    const auto & b = located.front().bbox;
    if (b[2] > b[0]) {   // 판독기가 자리를 주는 빌드 (zxing · OpenCV)
      EXPECT_LT(b[0], 320.f);
      EXPECT_GT(b[2], 320.f);
      EXPECT_LT(b[1], 240.f + 140.f);
      EXPECT_GT(b[3], 240.f - 140.f);
    }
    if (perception::barcode_locator_available()) {   // 자리만 찾기 (읽기 아님)
      const auto boxes = perception::locate_barcodes(bgr);
      EXPECT_FALSE(boxes.empty()) << "m=" << m;
    }
  }
}

// 바코드 없는 빈 영상 → 자리 없음
TEST(Barcode, LocateNothingOnBlank)
{
  cv::Mat bgr(480, 640, CV_8UC3, cv::Scalar(200, 200, 200));
  EXPECT_TRUE(perception::locate_barcodes(bgr).empty());
  BarcodeReader reader;
  EXPECT_TRUE(reader.read_located(bgr).empty());
}

#ifdef PERCEPTION_HAVE_ZXING
namespace
{
// Code128 "text" 를 모듈 폭 module px 로 그린 640×480 영상 (흰 여백 · 흐림 · 회전)
cv::Mat code128_image(const std::string & text, int module, double blur_sigma, double rot_deg)
{
  ZXing::MultiFormatWriter w(ZXing::BarcodeFormat::Code128);
  w.setMargin(0);
  const auto bm = w.encode(text, 0, 1);   // 크기 0 = 최소 (모듈 1 px)
  const int n = bm.width();
  cv::Mat img(480, 640, CV_8UC1, cv::Scalar(255));
  const int x0 = (640 - n * module) / 2;
  for (int i = 0; i < n; ++i) {
    if (bm.get(i, 0)) {img(cv::Rect(x0 + i * module, 180, module, 120)) = 0;}
  }
  if (rot_deg != 0.0) {
    const cv::Mat R = cv::getRotationMatrix2D({320.f, 240.f}, rot_deg, 1.0);
    cv::warpAffine(img, img, R, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  }
  if (blur_sigma > 0) {cv::GaussianBlur(img, img, cv::Size(0, 0), blur_sigma);}
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}
}  // namespace

TEST(Barcode, Code128DiceCodes)
{
  // 발제 벽 바코드 (Code128): P2-01 – P2-04
  EXPECT_STREQ(BarcodeReader::backend(), "zxing-cpp");
  BarcodeReader reader;
  for (const std::string code : {"P2-01", "P2-02", "P2-03", "P2-04"}) {
    for (int m : {2, 3, 5}) {
      const auto found = reader.read(code128_image(code, m, 0.8, 0.0));
      ASSERT_FALSE(found.empty()) << code << " m=" << m;
      EXPECT_EQ(found.front().first, code);
      EXPECT_EQ(found.front().second, "Code128");
    }
  }
}

TEST(Barcode, Code128TiltedAndBlurred)
{
  BarcodeReader reader;
  const auto found = reader.read(code128_image("P2-03", 3, 1.2, 8.0));   // 8° 기울고 흐림
  ASSERT_FALSE(found.empty());
  EXPECT_EQ(found.front().first, "P2-03");
}

TEST(Barcode, EmptyImageReadsNothing)
{
  BarcodeReader reader;
  EXPECT_TRUE(reader.read(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(255))).empty());
}
#endif

// 흰 라벨 자리 찾기: 회색 벽 위 160×55 흰 라벨 + 막대 → 찾음 (상자가 라벨을 덮음), 막대 없는 흰 종이 · 정사각 흰 판은 안 찾음
TEST(Barcode, WhiteLabelLocator)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(105, 110, 112));   // 회색 벽 (BGR)
  cv::rectangle(img, cv::Rect(300, 200, 160, 55), cv::Scalar(235, 238, 240), cv::FILLED);   // 흰 라벨
  for (int x = 312, i = 0; x < 448; ++i) {   // 막대 (폭 2–5 px)
    const int w = 2 + (i * 7) % 4;
    if (i % 2 == 0) {cv::rectangle(img, cv::Rect(x, 208, w, 32), cv::Scalar(20, 20, 20), cv::FILLED);}
    x += w;
  }
  cv::rectangle(img, cv::Rect(60, 60, 150, 50), cv::Scalar(235, 238, 240), cv::FILLED);    // 막대 없는 흰 종이
  cv::rectangle(img, cv::Rect(60, 330, 90, 90), cv::Scalar(235, 238, 240), cv::FILLED);    // 정사각
  cv::GaussianBlur(img, img, cv::Size(0, 0), 0.8);
  const perception::LabelParams p;
  const auto labels = perception::locate_labels(img, p, 606.3, nullptr);
  ASSERT_EQ(labels.size(), 1u);
  const auto & b = labels.front().bbox;
  EXPECT_NEAR(0.5 * (b[0] + b[2]), 380.0, 8.0);
  EXPECT_NEAR(0.5 * (b[1] + b[3]), 227.0, 8.0);
  EXPECT_GT(labels.front().score, p.min_changes);
  // 깊이로 크기 확인: 0.30 m 면 기대 폭 = 606 · 0.078 / 0.30 = 158 px → 통과, 0.10 m (기대 473 px) 면 너무 작아 안 찾음
  EXPECT_EQ(perception::locate_labels(img, p, 606.3, [](double, double) {return 0.30;}).size(), 1u);
  EXPECT_TRUE(perception::locate_labels(img, p, 606.3, [](double, double) {return 0.10;}).empty());
}
