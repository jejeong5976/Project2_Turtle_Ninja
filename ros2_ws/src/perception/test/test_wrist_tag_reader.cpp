// test_wrist_tag_reader.cpp — 손목 태그 읽기 (wrist_tag_reader) 시험
//   그린 tag36h11 → ID · 모서리 (aruco 순서), 왜곡 없는 camera_info → 정규 좌표 = K⁻¹ 화소, ROI · 비닝 풀기, 쥔 퍽 거리 판정

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/wrist_tag_reader.hpp"

using namespace perception::wrist;

namespace
{
// 흰 640 × 480 위 (x0, y0) 에 한 변 side 화소 태그 (검은 테두리 포함 마커)
cv::Mat draw_tag(int id, int x0, int y0, int side)
{
  cv::Mat img(480, 640, CV_8UC1, cv::Scalar(255));
  cv::Mat m;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, side, m, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, side, m, 1);
#endif
  m.copyTo(img(cv::Rect(x0, y0, side, side)));
  return img;
}

TagCamera pinhole(double f, double cx, double cy)
{
  TagCamera c;
  c.K = cv::Matx33d(f, 0, cx, 0, f, cy, 0, 0, 1);
  c.D = {0, 0, 0, 0, 0};
  c.model = "plumb_bob";
  return c;
}

// 카메라 앞 z [m] 에 놓인 한 변 s [m] 태그 (광축에 수직, 가운데 (x, y)) 의 정규 좌표 모서리
std::vector<cv::Point2f> norm_square(double x, double y, double z, double s)
{
  const double h = s / 2;
  return {{float((x - h) / z), float((y - h) / z)}, {float((x + h) / z), float((y - h) / z)},
    {float((x + h) / z), float((y + h) / z)}, {float((x - h) / z), float((y + h) / z)}};
}
}  // namespace

// 그린 태그를 읽음: ID · 모서리 4 개 (좌상 · 우상 · 우하 · 좌하, 1 px 안)
TEST(WristTagReader, DetectsDrawnTag)
{
  const int side = 160, x0 = 200, y0 = 150;
  const auto img = draw_tag(20, x0, y0, side);
  TagReader r(TagReader::Params{});
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  r.detect(img, corners, ids);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], 20);
  ASSERT_EQ(corners[0].size(), 4u);
  // 좌상이 가장 작은 x + y, 우하가 가장 큼, 순서는 시계 방향 (aruco)
  EXPECT_LT(corners[0][0].x + corners[0][0].y, corners[0][2].x + corners[0][2].y);
  EXPECT_GT(corners[0][1].x, corners[0][0].x);
  EXPECT_LT(corners[0][1].y, corners[0][2].y);
  EXPECT_LT(corners[0][3].x, corners[0][2].x);
  for (const auto & p : corners[0]) {
    EXPECT_GE(p.x, x0 - 1.0f);
    EXPECT_LE(p.x, x0 + side + 1.0f);
    EXPECT_GE(p.y, y0 - 1.0f);
    EXPECT_LE(p.y, y0 + side + 1.0f);
  }
}

// 왜곡 없는 camera_info: 정규 좌표 = (u − cx) / f. ROI · 비닝이면 전체 해상도로 되돌린 뒤 풂
TEST(WristTagReader, UndistortPinholeRoiBinning)
{
  auto cam = pinhole(600.0, 320.0, 240.0);
  std::vector<cv::Point2f> out;
  ASSERT_TRUE(undistort_corners(cam, {{320.0f, 240.0f}, {920.0f, 240.0f}}, out));
  EXPECT_NEAR(out[0].x, 0.0, 1e-6);
  EXPECT_NEAR(out[1].x, 1.0, 1e-6);
  cam.binning_x = cam.binning_y = 2.0f;
  cam.roi_x = 100.0f;
  ASSERT_TRUE(undistort_corners(cam, {{110.0f, 120.0f}}, out));   // 전체 = (320, 240)
  EXPECT_NEAR(out[0].x, 0.0, 1e-6);
  EXPECT_NEAR(out[0].y, 0.0, 1e-6);
  cam.model = "모름";
  cam.D = {0.1, 0, 0, 0, 0};
  std::string why;
  EXPECT_FALSE(undistort_corners(cam, {{1.0f, 1.0f}}, out, &why));
  EXPECT_FALSE(why.empty());
}

// 쥔 퍽 판정: 15.3 mm 태그가 카메라 0.10 m 면 0.15 m 안, 0.30 m 면 밖. 모서리 순서가 반시계여도 같음
TEST(WristTagReader, TagNearByPnP)
{
  EXPECT_TRUE(tag_near(norm_square(0.01, 0.02, 0.10, 0.0153), 0.0153, 0.15));
  EXPECT_FALSE(tag_near(norm_square(0.01, 0.02, 0.30, 0.0153), 0.0153, 0.15));
  auto ccw = norm_square(0.0, 0.0, 0.08, 0.0153);
  std::reverse(ccw.begin() + 1, ccw.end());
  EXPECT_TRUE(tag_near(ccw, 0.0153, 0.15));
}

// 영역 투영: 카메라 앞 0.5 m 점 (±0.05) → f 600 화소에서 ±60 px + 여유, 카메라 뒤 점은 뺌
TEST(WristTagReader, ProjectBoxPinhole)
{
  const cv::Matx33d K(600, 0, 320, 0, 600, 240, 0, 0, 1);
  const auto r = project_box({{-0.05, -0.05, 0.5}, {0.05, 0.05, 0.5}, {0.0, 0.0, -0.2}}, K, 10.0);
  EXPECT_EQ(r.x, 320 - 60 - 10);
  EXPECT_EQ(r.y, 240 - 60 - 10);
  EXPECT_EQ(r.br().x, 320 + 60 + 10);
  EXPECT_EQ(r.br().y, 240 + 60 + 10);
  EXPECT_EQ(project_box({{0.0, 0.0, -0.1}}, K, 10.0).area(), 0);
}

// 영역 정리: 화면에 자르고 겹친 것은 합침, 넓이가 max_frac 넘으면 false
TEST(WristTagReader, MergeRois)
{
  std::vector<cv::Rect> r{{-20, -20, 60, 60}, {30, 30, 40, 40}, {400, 300, 50, 50}, {700, 10, 30, 30}, {0, 0, 0, 0}};
  ASSERT_TRUE(merge_rois(r, {640, 480}, 0.5));
  ASSERT_EQ(r.size(), 2u);                         // 앞 둘 합침 · 화면 밖 · 빈 것 뺌
  EXPECT_EQ(r[0], cv::Rect(0, 0, 70, 70));
  EXPECT_EQ(r[1], cv::Rect(400, 300, 50, 50));
  std::vector<cv::Rect> big{{0, 0, 600, 400}};
  EXPECT_FALSE(merge_rois(big, {640, 480}, 0.5));
}

// 영역 검출 = 전체 검출과 같은 ID · 모서리 (전체 영상 화소), 영역 밖 태그는 안 나옴
TEST(WristTagReader, DetectInMatchesFull)
{
  cv::Mat img = draw_tag(20, 60, 60, 120);
  draw_tag(25, 420, 300, 120)(cv::Rect(420, 300, 120, 120)).copyTo(img(cv::Rect(420, 300, 120, 120)));
  TagReader r(TagReader::Params{});
  std::vector<std::vector<cv::Point2f>> cf, ci;
  std::vector<int> idf, idi;
  r.detect(img, cf, idf);
  ASSERT_EQ(idf.size(), 2u);
  r.detect_in(img, {cv::Rect(30, 30, 180, 180)}, ci, idi);
  ASSERT_EQ(idi.size(), 1u);
  EXPECT_EQ(idi[0], 20);
  const size_t k = idf[0] == 20 ? 0 : 1;
  for (int j = 0; j < 4; ++j) {EXPECT_NEAR(cv::norm(ci[0][j] - cf[k][j]), 0.0, 1.0);}
  r.detect_in(img, {cv::Rect(30, 30, 180, 180), cv::Rect(390, 270, 180, 180), cv::Rect(40, 40, 100, 100)}, ci, idi);
  EXPECT_EQ(idi.size(), 2u);                       // 같은 ID 는 한 번만
}
