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
  }
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
  // 발제 v5 벽 바코드 (Code128): P2-01 – P2-04
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
