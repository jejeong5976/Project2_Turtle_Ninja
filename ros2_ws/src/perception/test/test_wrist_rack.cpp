// test_wrist_rack.cpp — 적재함 컵 바닥 태그 시험: 합성 영상으로 칸 중심 · yaw 오차, ID 약속, 다수결 창
//   실행: colcon build → ./build/perception/test_wrist_rack

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/wrist_rack.hpp"

using namespace perception::wrist;

namespace
{
constexpr int W = 640, H = 480;
const Intrinsics K{605.9, 605.4, 324.0, 245.9};   // 실물 D435 컬러 640×480

Mat3 rot(const Vec3 & axis, double a) {return Eigen::AngleAxisd(a, axis.normalized()).toRotationMatrix();}

// 태그 무늬 (흰 여백 1 칸 포함 10×10 칸, 칸당 px 화소)
cv::Mat tag_texture(int id, int px)
{
  cv::Mat bits;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 8, bits, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 8, bits, 1);
#endif
  cv::Mat big;
  cv::resize(bits, big, cv::Size(8 * px, 8 * px), 0, 0, cv::INTER_NEAREST);
  cv::Mat tex(10 * px, 10 * px, CV_8U, cv::Scalar(255));
  big.copyTo(tex(cv::Rect(px, px, 8 * px, 8 * px)));
  return tex;
}

// 칸 (로봇 좌표 yaw, 중심) 을 카메라 (로봇 → 카메라 R_cw, t_cw) 로 찍은 영상
cv::Mat render(int id, double side_m, const Mat3 & R_ct, const Vec3 & t_ct)
{
  const int px = 40;
  const cv::Mat tex = tag_texture(id, px);
  const double cell = side_m / 8;
  // 무늬 화소 (u, v) → 태그 면 (X 오른쪽, Y 위, m)
  cv::Matx33d A(cell / px, 0, -5 * cell, 0, -cell / px, 5 * cell, 0, 0, 1);
  cv::Matx33d M(R_ct(0, 0), R_ct(0, 1), t_ct.x(), R_ct(1, 0), R_ct(1, 1), t_ct.y(), R_ct(2, 0), R_ct(2, 1), t_ct.z());
  cv::Matx33d Kc(K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
  const cv::Matx33d Hm = Kc * M * A;
  cv::Mat img(H, W, CV_8U, cv::Scalar(70));   // 컵 바닥 바깥 (어두운 벽 · 그늘)
  cv::warpPerspective(tex, img, cv::Mat(Hm), img.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.7);
  cv::Mat noise(img.size(), CV_16S);
  cv::randn(noise, 0, 2.0);
  cv::Mat out;
  img.convertTo(out, CV_16S);
  out += noise;
  out.convertTo(out, CV_8U);
  return out;
}

struct Scene
{
  Mat3 R_cw;   // 로봇 → 카메라
  Vec3 t_cw;
};

// 칸 위 높이 h 에서 내려다보는 카메라, 수직에서 tilt 만큼 기움
Scene camera_above(const Vec3 & cup, double h, double tilt, double cam_yaw)
{
  // 기본: 광학 z = 아래 (−Z), 광학 x = 로봇 −Y, 광학 y = 로봇 −X
  Mat3 R_wc;
  R_wc.col(0) = Vec3(0, -1, 0);
  R_wc.col(1) = Vec3(-1, 0, 0);
  R_wc.col(2) = Vec3(0, 0, -1);
  R_wc = rot(Vec3::UnitZ(), cam_yaw) * rot(Vec3::UnitX(), tilt) * R_wc;
  const Vec3 cam = cup - R_wc.col(2) * (h / std::cos(tilt));   // 광축이 칸 중심을 지나게
  Scene s;
  s.R_cw = R_wc.transpose();
  s.t_cw = -s.R_cw * cam;
  return s;
}

void check_one(double tilt_deg, double cam_yaw_deg, double cup_yaw_deg)
{
  RackParams p;
  const Vec3 cup(0.0843, 0.1073, 0.1325);   // S5 근처
  const double cup_yaw = cup_yaw_deg * M_PI / 180;
  const Scene s = camera_above(cup, 0.18, tilt_deg * M_PI / 180, cam_yaw_deg * M_PI / 180);
  // 칸 프레임 → 태그 프레임: 태그 = 칸 · Rz(−90°)
  const Mat3 R_w_cup = rot(Vec3::UnitZ(), cup_yaw);
  const Mat3 R_w_tag = R_w_cup * rot(Vec3::UnitZ(), -M_PI / 2);
  const Mat3 R_ct = s.R_cw * R_w_tag;
  const Vec3 t_ct = s.R_cw * cup + s.t_cw;
  const int id = rack_tag_of(5, p);   // S5
  const cv::Mat img = render(id, p.tag_side_m, R_ct, t_ct);

  const Vec3 up = s.R_cw * Vec3::UnitZ();
  const auto tags = detect_rack_tags(img, K, cv::Mat(), p, &up);
  ASSERT_EQ(tags.size(), 1u) << "tilt " << tilt_deg;
  const auto & t = tags[0];
  EXPECT_EQ(t.cell, 5);
  EXPECT_EQ(t.name, "S5");
  // 로봇 좌표로 되돌려 비교
  const Mat3 R_wc = s.R_cw.transpose();
  const Vec3 pos = R_wc * (t.t - s.t_cw);
  const double yaw = planar_yaw(R_wc * t.R);
  EXPECT_LT((pos - cup).head<2>().norm(), 0.0005) << "수평 오차 " << (pos - cup).head<2>().norm() * 1000 << " mm";
  EXPECT_LT(std::abs(pos.z() - cup.z()), 0.001);
  EXPECT_LT(std::abs(yaw - cup_yaw) * 180 / M_PI, 0.5);
  EXPECT_LT(t.reproj_px, 0.5);
}
}  // namespace

TEST(RackTag, IdToCell)
{
  RackParams p;   // 실물 표: S1 20 · S4 31 · S12 28
  EXPECT_EQ(rack_cell_of(20, p), 1);
  EXPECT_EQ(rack_cell_of(31, p), 4);
  EXPECT_EQ(rack_cell_of(28, p), 12);
  EXPECT_EQ(rack_tag_of(5, p), 26);
  EXPECT_EQ(rack_tag_of(13, p), -1);
  for (int c = 1; c <= 12; ++c) {EXPECT_EQ(rack_cell_of(rack_tag_of(c, p), p), c);}
  EXPECT_EQ(rack_cell_of(19, p), 0);
  EXPECT_EQ(rack_cell_of(32, p), 0);
  EXPECT_EQ(rack_cell_of(5, p), 0);   // 퍽 태그
  EXPECT_EQ(rack_cell_name(12), "S12");
}

TEST(RackTag, VerticalView) {check_one(0.0, 0.0, 1.5);}
TEST(RackTag, TiltedView) {check_one(6.0, 20.0, -3.0);}
TEST(RackTag, RotatedCup) {check_one(3.0, -35.0, 8.0);}

TEST(RackTag, PuckTagIgnored)
{
  RackParams p;
  const Vec3 cup(0.0, 0.1, 0.13);
  const Scene s = camera_above(cup, 0.18, 0.0, 0.0);
  const Mat3 R_ct = s.R_cw * rot(Vec3::UnitZ(), -M_PI / 2);
  const cv::Mat img = render(7, p.tag_side_m, R_ct, s.R_cw * cup + s.t_cw);
  const Vec3 up = s.R_cw * Vec3::UnitZ();
  EXPECT_TRUE(detect_rack_tags(img, K, cv::Mat(), p, &up).empty());
}

TEST(RackTag, TooTiltedRejected)
{
  RackParams p;
  p.max_view_deg = 10.0;
  const Vec3 cup(0.0, 0.1, 0.13);
  const Scene s = camera_above(cup, 0.18, 20 * M_PI / 180, 0.0);
  const Mat3 R_ct = s.R_cw * rot(Vec3::UnitZ(), -M_PI / 2);
  const cv::Mat img = render(rack_tag_of(1, p), p.tag_side_m, R_ct, s.R_cw * cup + s.t_cw);
  const Vec3 up = s.R_cw * Vec3::UnitZ();
  std::vector<std::pair<RackTag, std::string>> rej;
  EXPECT_TRUE(detect_rack_tags(img, K, cv::Mat(), p, &up, &rej).empty());
  ASSERT_EQ(rej.size(), 1u);
  EXPECT_EQ(rej[0].second, "view");
}

TEST(RackFilter, MedianAndWindow)
{
  RackFilter f(5, 3);
  f.next_frame();
  f.push(5, Vec3(0.10, 0.20, 0.13), 0.01);
  f.next_frame();
  f.push(5, Vec3(0.11, 0.20, 0.13), 0.02);
  EXPECT_FALSE(f.get(5).has_value());   // 2 장 < 3
  f.next_frame();
  f.push(5, Vec3(0.30, 0.20, 0.13), 0.03);   // 튀는 값
  auto e = f.get(5);
  ASSERT_TRUE(e.has_value());
  EXPECT_NEAR(e->position.x(), 0.11, 1e-9);   // 중앙값이 튀는 값을 버림
  EXPECT_NEAR(e->yaw, 0.02, 1e-9);
  EXPECT_EQ(e->hits, 3);
  EXPECT_EQ(f.cells(), std::vector<int>{5});
  for (int i = 0; i < 3; ++i) {f.next_frame();}   // 3 장 못 봄 → 창 안에 1 장만
  EXPECT_FALSE(f.get(5).has_value());
}

TEST(RackFilter, YawAcrossPi)
{
  RackFilter f(5, 3);
  for (double y : {M_PI - 0.01, -M_PI + 0.01, M_PI - 0.02}) {
    f.next_frame();
    f.push(1, Vec3::Zero(), y);
  }
  const auto e = f.get(1);
  ASSERT_TRUE(e.has_value());
  EXPECT_NEAR(std::abs(e->yaw), M_PI - 0.01, 1e-6);
}
