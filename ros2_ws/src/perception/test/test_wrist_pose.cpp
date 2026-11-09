// test_wrist_pose.cpp — 손목 퍽 기하 · 묶음 시험, 합성 점군으로
//   실행: colcon build → source install/setup.bash → ./build/perception/test_wrist_pose

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <opencv2/imgproc.hpp>

#include "perception/wrist_pose.hpp"
#include "perception/wrist_tracker.hpp"

using namespace perception::wrist;

namespace
{
// 월드(z 위) → 카메라: 카메라는 원점, 아래로 tilt 도 숙여 +y(월드)를 봄
struct Cam
{
  Mat3 R_cw;     // 월드 → 카메라 회전
  Vec3 up_cam;   // 카메라 좌표의 월드 위쪽
  explicit Cam(double tilt_deg)
  {
    const double t = tilt_deg * M_PI / 180.0;
    // 카메라 축 (월드 좌표): z 앞 = (0, cos, -sin), x 오른쪽 = (1, 0, 0), y 아래 = z × x
    const Vec3 zc(0, std::cos(t), -std::sin(t)), xc(1, 0, 0);
    const Vec3 yc = zc.cross(xc);
    R_cw.row(0) = xc.transpose();
    R_cw.row(1) = yc.transpose();
    R_cw.row(2) = zc.transpose();
    up_cam = R_cw * Vec3::UnitZ();
  }
  Vec3 to_cam(const Vec3 & pw) const {return R_cw * pw;}
};

// 선 사각기둥 (중심 c, 면 각 yaw) 윗면 + 카메라 쪽 옆면 점 (월드)
std::vector<Vec3> upright_box_points(const Vec3 & c, double yaw, const Cam & cam, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-0.5, 0.5);
  std::normal_distribution<double> nz(0.0, 0.0005);
  const Vec3 ex(std::cos(yaw), std::sin(yaw), 0), ey(-std::sin(yaw), std::cos(yaw), 0);
  std::vector<Vec3> out;
  for (int i = 0; i < 1500; ++i) {         // 윗면
    out.push_back(c + Vec3(0, 0, PUCK_H / 2 + nz(rng)) + ex * u(rng) * PUCK_D + ey * u(rng) * PUCK_D);
  }
  // 카메라(원점, 월드 -y 쪽)를 향한 옆면
  for (const Vec3 & n : std::vector<Vec3>{ex, -ex, ey, -ey}) {
    if (n.dot(-c) <= 0) {continue;}
    const Vec3 t = Vec3::UnitZ().cross(n);
    for (int i = 0; i < 1500; ++i) {
      out.push_back(c + n * (PUCK_D / 2 + nz(rng)) + t * u(rng) * PUCK_D + Vec3::UnitZ() * u(rng) * PUCK_H);
    }
  }
  for (auto & p : out) {p = cam.to_cam(p);}
  return out;
}

// 누운 원기둥 (중심 c, 긴축 수평 방향 yaw) 의 위쪽 반 원통 점 (월드)
std::vector<Vec3> lying_cyl_points(const Vec3 & c, double yaw, const Cam & cam, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-0.5, 0.5), a(0.0, M_PI);
  const Vec3 ax(std::cos(yaw), std::sin(yaw), 0), side(-std::sin(yaw), std::cos(yaw), 0);
  std::vector<Vec3> out;
  for (int i = 0; i < 3000; ++i) {
    const double th = a(rng);
    out.push_back(c + ax * u(rng) * PUCK_H + (side * std::cos(th) + Vec3::UnitZ() * std::sin(th)) * (PUCK_D / 2));
  }
  for (auto & p : out) {p = cam.to_cam(p);}
  return out;
}

// 누운 사각기둥 (중심 c, 긴축 수평 방향 yaw, 단면 회전 roll: 0 = 면이 위, 45° = 모서리가 위) 의 겉면 점 중 위 절반 (월드)
std::vector<Vec3> lying_box_points(const Vec3 & c, double yaw, double roll, const Cam & cam, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(-0.5, 0.5);
  const Vec3 ax(std::cos(yaw), std::sin(yaw), 0), side(-std::sin(yaw), std::cos(yaw), 0);
  const Vec3 n1 = std::cos(roll) * Vec3::UnitZ() + std::sin(roll) * side;   // 단면의 두 면 법선
  const Vec3 n2 = ax.cross(n1);
  std::vector<Vec3> out;
  for (const Vec3 & n : std::vector<Vec3>{n1, -n1, n2, -n2}) {
    const Vec3 t = ax.cross(n);
    for (int i = 0; i < 1500; ++i) {
      const Vec3 p = c + n * (PUCK_D / 2) + t * u(rng) * PUCK_D + ax * u(rng) * PUCK_H;
      if (p.z() >= c.z()) {out.push_back(p);}                         // 위 절반만 보임
    }
  }
  for (auto & p : out) {p = cam.to_cam(p);}
  return out;
}

double ang_deg(const Vec3 & a, const Vec3 & b)
{
  return std::acos(std::min(1.0, std::abs(a.normalized().dot(b.normalized())))) * 180.0 / M_PI;
}

RawPuck raw_at(const Vec3 & c, bool upright, const std::string & kind = "green_box_up", double conf = 0.9)
{
  RawPuck r;
  r.pose.center = c;
  r.pose.axis = upright ? Vec3(0, -1, 0) : Vec3(1, 0, 0);
  r.pose.upright = upright;
  r.pose.n_points = 500;
  r.kind = kind;
  r.info = parse_kind(kind);
  r.conf = conf;
  return r;
}
}  // namespace

TEST(Stats, PercentileLikeNumpy)
{
  const std::vector<double> v{3, 1, 4, 1, 5, 9, 2, 6};
  // numpy: np.percentile(v, [0, 25, 50, 95, 100]) = 1, 1.75, 3.5, 7.95, 9
  EXPECT_DOUBLE_EQ(percentile(v, 0), 1.0);
  EXPECT_DOUBLE_EQ(percentile(v, 25), 1.75);
  EXPECT_DOUBLE_EQ(median(v), 3.5);
  EXPECT_NEAR(percentile(v, 95), 7.95, 1e-12);
  EXPECT_DOUBLE_EQ(percentile(v, 100), 9.0);
}

TEST(Stats, TopHeightIgnoresNoiseAbove)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 0.002);
  std::vector<double> h;
  for (int i = 0; i < 2000; ++i) {h.push_back(0.1 + n(rng));}       // 윗면 0.1 m, 잡음 2 mm
  for (int i = 0; i < 2000; ++i) {h.push_back(0.04 + 0.06 * i / 2000.0);}   // 옆면
  EXPECT_NEAR(top_height(h), 0.1, 0.0015);
  EXPECT_GT(percentile(h, 95) - 0.1, 0.002);                         // 그냥 p95 는 위로 튐
}

TEST(Geometry, UprightBox)
{
  std::mt19937 rng(2);
  const Cam cam(55.0);
  const Vec3 c(0.02, 0.20, -0.25);
  const double yaw = 0.3;
  const auto pts = upright_box_points(c, yaw, cam, rng);
  PuckPose p;
  p.center = Vec3::Zero();
  gravity_refine(p, pts, cam.up_cam);
  ASSERT_TRUE(p.upright.has_value());
  EXPECT_TRUE(*p.upright);
  EXPECT_EQ(p.axis_source, "gravity_up");
  EXPECT_LT((p.center - cam.to_cam(c)).norm(), 0.002);
  ASSERT_TRUE(p.face_dir.has_value());
  // 면 방향 = 월드 ex 와 90° 주기로 같음
  const Vec3 ex_cam = cam.R_cw * Vec3(std::cos(yaw), std::sin(yaw), 0);
  const double d = std::fmod(ang_deg(*p.face_dir, ex_cam), 90.0);
  EXPECT_LT(std::min(d, 90.0 - d), 2.0);
}

TEST(Geometry, LyingCylinder)
{
  std::mt19937 rng(3);
  const Cam cam(50.0);
  const Vec3 c(-0.03, 0.22, -0.27);
  const double yaw = -0.7;
  const auto pts = lying_cyl_points(c, yaw, cam, rng);
  PuckPose p;
  p.center = Vec3::Zero();
  gravity_refine(p, pts, cam.up_cam);
  ASSERT_TRUE(p.upright.has_value());
  EXPECT_FALSE(*p.upright);
  EXPECT_EQ(p.axis_source, "gravity_flat");
  EXPECT_LT(ang_deg(p.axis, cam.R_cw * Vec3(std::cos(yaw), std::sin(yaw), 0)), 2.0);
  EXPECT_LT((p.center - cam.to_cam(c)).norm(), 0.003);
  EXPECT_GE(std::abs(p.axis.x()) >= 0.2 ? p.axis.x() : p.axis.y(), 0.0);   // 부호 = 화면 오른쪽 (orient_axis)
}

TEST(Geometry, LyingBoxFaceUp)
{
  std::mt19937 rng(5);
  const Cam cam(50.0);
  const Vec3 c(0.01, 0.22, -0.27);
  const double yaw = 0.4;
  PuckPose p;
  gravity_refine(p, lying_box_points(c, yaw, 0.1, cam, rng), cam.up_cam);   // 면이 위 (조금 기울어 놓임 0.1 rad)
  ASSERT_TRUE(p.upright.has_value());
  EXPECT_FALSE(*p.upright);
  ASSERT_TRUE(p.face_dir.has_value());
  EXPECT_LT(ang_deg(*p.face_dir, cam.up_cam), 1e-6);                  // 면 방향 = 위쪽
  EXPECT_GT(p.face_size[0], 0.025);
  // 모서리가 위 (45°) → 띠가 좁아 면 방향 모름
  PuckPose q;
  gravity_refine(q, lying_box_points(c, yaw, M_PI / 4, cam, rng), cam.up_cam);
  ASSERT_TRUE(q.upright.has_value());
  EXPECT_FALSE(*q.upright);
  EXPECT_FALSE(q.face_dir.has_value());
  EXPECT_LT(q.face_size[0], LYING_FACE_MIN_W);
}

TEST(Geometry, OrientAxis)
{
  const Vec3 up(0, -1, 0);
  EXPECT_EQ(orient_axis(Vec3(0, 1, 0), &up), Vec3(0, -1, 0));          // 선 퍽 = 위쪽
  EXPECT_EQ(orient_axis(Vec3(-1, 0, 0), &up), Vec3(1, 0, 0));          // 누움 = 화면 오른쪽
  EXPECT_EQ(orient_axis(Vec3(-0.1, -0.5, 0.86), nullptr), Vec3(0.1, 0.5, -0.86));   // 거의 세로 = 화면 아래
}

TEST(Tracker, LyingAxisSignFollowsPrevious)
{
  PuckTracker tr(5, 3, 0.02);
  const Vec3 c(0, 0, 0.3);
  auto a = raw_at(c, false), b = raw_at(c, false), d = raw_at(c, false);
  b.pose.axis = -b.pose.axis;                                         // 가운데 장만 뒤집힘
  tr.update({a});
  tr.update({b});
  const auto out = tr.update({d});
  ASSERT_EQ(out.size(), 1u);
  EXPECT_LT(ang_deg(quat_to_mat(out[0].orientation).col(2), Vec3(1, 0, 0)), 1e-6);   // 평균이 0 으로 깨지지 않음
}

TEST(Geometry, PuckPoseFromDepthImage)
{
  // 깊이 영상에 평평한 사각 판(퍽 윗면 크기) 을 그려 역투영 · 상자 · 잘림 확인
  const Intrinsics K{605.9, 605.4, 324.0, 245.9};
  cv::Mat1f depth(480, 640, 0.0f);
  cv::Mat1b mask(480, 640, uint8_t(0));
  cv::rectangle(depth, cv::Rect(300, 220, 40, 40), cv::Scalar(0.3), cv::FILLED);
  cv::rectangle(mask, cv::Rect(300, 220, 40, 40), cv::Scalar(255), cv::FILLED);
  const auto p = puck_pose(mask, depth, K, {}, nullptr);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->n_points, 1600);
  EXPECT_EQ(p->bbox, (std::array<int, 4>{300, 220, 339, 259}));
  EXPECT_FALSE(p->truncated);
  EXPECT_NEAR(p->center.z(), 0.3, 1e-6);                              // 위쪽 모르면 보이는 점 평균
  cv::rectangle(mask, cv::Rect(0, 0, 10, 10), cv::Scalar(255), cv::FILLED);   // 가장자리 닿으면 잘림 (깊이 없어도 상자는 마스크 기준)
  EXPECT_TRUE(puck_pose(mask, depth, K, {}, nullptr)->truncated);
  cv::Mat1b tiny(480, 640, uint8_t(0));
  cv::rectangle(tiny, cv::Rect(300, 220, 5, 5), cv::Scalar(255), cv::FILLED);
  EXPECT_FALSE(puck_pose(tiny, depth, K, {}, nullptr).has_value());   // 점 < 30
}

TEST(Geometry, SupportHeightDecidesUpright)
{
  // 카메라가 바로 아래를 봄 (위쪽 = 광학 −z). 박스 윗면 깊이 0.30 m, 퍽 윗면은 그보다 퍽 높이만큼 가까움
  const Intrinsics K{600.0, 600.0, 320.0, 240.0};
  const Vec3 up(0, 0, -1);
  auto scene = [&](double top_above, double len_m, double wid_m) {
      cv::Mat1f depth(480, 640, 0.30f);
      cv::Mat1b mask(480, 640, uint8_t(0));
      const double z = 0.30 - top_above;
      const cv::Rect r(320 - static_cast<int>(len_m / 2 * K.fx / z), 240 - static_cast<int>(wid_m / 2 * K.fy / z),
        static_cast<int>(len_m * K.fx / z), static_cast<int>(wid_m * K.fy / z));
      cv::rectangle(depth, r, cv::Scalar(z), cv::FILLED);
      cv::rectangle(mask, r, cv::Scalar(255), cv::FILLED);
      return std::make_pair(mask, depth);
    };
  {   // 누운 네모 퍽 (윗면 34.8 mm 높이, 60 × 34.8)
    auto [mask, depth] = scene(0.0348, 0.060, 0.0348);
    std::vector<double> hs;
    for (const auto & q : deproject(mask, depth, K)) {hs.push_back(q.dot(up));}
    const double top_h = top_height(hs);
    const auto sh = support_height(mask, depth, K, up, top_h);
    ASSERT_TRUE(sh.has_value());
    EXPECT_NEAR(top_h - *sh, 0.0348, 0.002);
    const auto p = puck_pose(mask, depth, K, {}, &up);
    ASSERT_TRUE(p.has_value() && p->upright.has_value());
    EXPECT_FALSE(*p->upright);
  }
  {   // 누운 퍽 윗면이 반만 보임 (발자국 30 mm < 48 → 예전 규칙은 선 퍽) — 놓인 면 높이로는 누움
    auto [mask, depth] = scene(0.0348, 0.030, 0.0348);
    const auto p = puck_pose(mask, depth, K, {}, &up);
    ASSERT_TRUE(p.has_value() && p->upright.has_value());
    EXPECT_FALSE(*p->upright);
  }
  {   // 선 퍽 (윗면 60 mm 높이, 34.8 × 34.8)
    auto [mask, depth] = scene(0.060, 0.0348, 0.0348);
    const auto p = puck_pose(mask, depth, K, {}, &up);
    ASSERT_TRUE(p.has_value() && p->upright.has_value());
    EXPECT_TRUE(*p->upright);
  }
  {   // 둘레 깊이 없음 → 예전 규칙 (발자국 60 mm → 누움)
    auto [mask, depth] = scene(0.0348, 0.060, 0.0348);
    depth.setTo(0.0f, ~mask);
    const auto p = puck_pose(mask, depth, K, {}, &up);
    ASSERT_TRUE(p.has_value() && p->upright.has_value());
    EXPECT_FALSE(*p->upright);
  }
}

TEST(Geometry, FlatPrintIsNotAPuck)
{
  // 실물: 상자에 인쇄된 로고를 퍽으로 분할 → 둘레보다 안 솟았으면 퍽 아님 (min_rise_m)
  const Intrinsics K{600.0, 600.0, 320.0, 240.0};
  const Vec3 up(0, 0, -1);
  cv::Mat1f depth(480, 640, 0.30f);
  cv::Mat1b mask(480, 640, uint8_t(0));
  cv::rectangle(mask, cv::Rect(300, 220, 40, 40), cv::Scalar(255), cv::FILLED);   // 윗면 = 상자 면 (무늬)
  EXPECT_TRUE(puck_pose(mask, depth, K, {}, &up).has_value());                    // 안 보면 그대로
  EXPECT_FALSE(puck_pose(mask, depth, K, {}, &up, 0.012).has_value());
  // 누운 퍽 (윗면이 상자 면 위 34.8 mm) 은 남음
  const double z = 0.30 - 0.0348;
  cv::rectangle(depth, cv::Rect(300, 220, 40, 40), cv::Scalar(z), cv::FILLED);
  EXPECT_TRUE(puck_pose(mask, depth, K, {}, &up, 0.012).has_value());
  // 둘레 깊이가 없으면 판단 못 함 → 남김
  cv::Mat1f d2(480, 640, 0.0f);
  cv::rectangle(d2, cv::Rect(300, 220, 40, 40), cv::Scalar(0.30), cv::FILLED);
  EXPECT_TRUE(puck_pose(mask, d2, K, {}, &up, 0.012).has_value());
}

TEST(Frame, AxisFrameAndQuat)
{
  const Vec3 axis = Vec3(0.2, -0.9, 0.3).normalized();
  const Mat3 R = axis_frame(axis, std::nullopt);
  EXPECT_NEAR((R.transpose() * R - Mat3::Identity()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  EXPECT_NEAR((R.col(2) - axis).norm(), 0.0, 1e-12);
  const auto q = quat_xyzw(R);
  EXPECT_GE(q[3], 0.0);
  EXPECT_NEAR((quat_to_mat(q) - R).norm(), 0.0, 1e-9);
  // trace ≤ 0 쪽 (180° 회전)
  Mat3 F = Vec3(1, -1, -1).asDiagonal();
  EXPECT_NEAR((quat_to_mat(quat_xyzw(F)) - F).norm(), 0.0, 1e-9);
}

TEST(Kind, Parse)
{
  auto k = parse_kind("red_box_lying");
  EXPECT_EQ(k.color, "red");
  EXPECT_EQ(k.shape, "box");
  ASSERT_TRUE(k.lying.has_value());
  EXPECT_TRUE(*k.lying);
  k = parse_kind("green_cyl");
  EXPECT_EQ(k.shape, "cylinder");
  EXPECT_FALSE(k.lying.has_value());
  k = parse_kind("", "green");
  EXPECT_EQ(k.color, "green");
  EXPECT_EQ(k.shape, "");
}

TEST(Tracker, ConfirmAfterMinHits)
{
  PuckTracker tr(5, 3, 0.02);
  const Vec3 c(0.0, 0.05, 0.3);
  EXPECT_TRUE(tr.update({raw_at(c, true)}).empty());
  EXPECT_TRUE(tr.update({raw_at(c + Vec3(0.005, 0, 0), true)}).empty());
  const auto d = tr.update({raw_at(c + Vec3(-0.005, 0, 0), true)});
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].hits, 3);
  EXPECT_EQ(d[0].label, "green");
  EXPECT_EQ(d[0].shape, "box");
  EXPECT_NEAR(d[0].position.x(), 0.0, 1e-12);                       // 중앙값
  EXPECT_TRUE(tr.update({}).empty());                                // 이번 장에 안 보이면 안 냄
  tr.reset();
  EXPECT_TRUE(tr.update({raw_at(c, true)}).empty());
}

TEST(Tracker, RejectedAndSeparatePucks)
{
  PuckTracker tr(5, 3, 0.02);
  auto bad = raw_at(Vec3(0.1, 0, 0.3), true);
  bad.reject = "truncated";
  std::vector<PuckDetection> d;
  for (int i = 0; i < 3; ++i) {
    d = tr.update({raw_at(Vec3(0, 0, 0.3), true), raw_at(Vec3(0.05, 0, 0.3), false, "red_cyl_lying"), bad});
  }
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(tr.n_tracks(), 2u);
}

TEST(Tracker, UprightMajority)
{
  // 선 2 장 · 누움 1 장 → 선 쪽 장만으로 긴축 (위쪽 그대로)
  PuckTracker tr(5, 3, 0.02);
  const Vec3 c(0, 0, 0.3);
  tr.update({raw_at(c, true)});
  tr.update({raw_at(c, false)});
  const auto d = tr.update({raw_at(c, true)});
  ASSERT_EQ(d.size(), 1u);
  ASSERT_TRUE(d[0].upright.has_value());
  EXPECT_TRUE(*d[0].upright);
  EXPECT_LT(ang_deg(quat_to_mat(d[0].orientation).col(2), Vec3(0, -1, 0)), 1e-6);
}

TEST(Tracker, ClassVoteWeighted)
{
  PuckTracker tr(5, 3, 0.02);
  const Vec3 c(0, 0, 0.3);
  tr.update({raw_at(c, true, "red_cyl_up", 0.9)});
  tr.update({raw_at(c, true, "green_cyl_up", 0.3)});
  const auto d = tr.update({raw_at(c, true, "green_cyl_up", 0.3)});
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].label, "red");                                      // 0.9 > 0.6
  EXPECT_NEAR(d[0].confidence, 0.5, 1e-12);
}

TEST(Planar, UprightBoxAndLying)
{
  // 카메라 = 로봇 프레임에서 아래로 90° (광학 z = 로봇 −z, 광학 x = 로봇 −y, 광학 y = 로봇 −x)
  Mat3 R_bc;
  R_bc << 0, -1, 0,
    -1, 0, 0,
    0, 0, -1;
  const Vec3 t_bc(0.2, 0.0, 0.4);
  PuckDetection d;
  d.shape = "box";
  d.position = Vec3(0, 0, 0.3);
  // 선 퍽: 긴축 = 로봇 +z = 광학 −z, 면 방향 = 로봇 x 에서 20° → 광학으로
  const Vec3 face_b(std::cos(0.35), std::sin(0.35), 0);
  d.orientation = quat_xyzw(axis_frame(R_bc.transpose() * Vec3::UnitZ(), R_bc.transpose() * face_b));
  d.yaw_known = true;
  auto p = planar_pose(d, R_bc, t_bc);
  EXPECT_TRUE(p.upright);
  EXPECT_TRUE(p.theta_known);
  EXPECT_NEAR(p.theta, 0.35, 1e-9);
  EXPECT_NEAR(p.x, 0.2, 1e-12);
  EXPECT_NEAR(p.z, 0.1, 1e-12);
  // 누운 퍽: 긴축 = 로봇 (cos 100°, sin 100°) → θ 는 180° 주기로 −80°
  const Vec3 ax_b(std::cos(100 * M_PI / 180), std::sin(100 * M_PI / 180), 0);
  d.orientation = quat_xyzw(axis_frame(R_bc.transpose() * ax_b, std::nullopt));
  d.yaw_known = false;
  p = planar_pose(d, R_bc, t_bc);
  EXPECT_FALSE(p.upright);
  EXPECT_NEAR(p.theta, -80 * M_PI / 180, 1e-9);
}

TEST(Geometry, TagTopRefineUsesTagCentreAndEdges)
{
  // 바로 아래를 보는 카메라 (위쪽 = −z), 윗면 0.28 m. 깊이 평균이 10 mm 치우친 윗면 중심을 태그로 바로잡음
  const Intrinsics K{606.4, 606.2, 325.2, 256.8};
  const Vec3 up(0.0, 0.0, -1.0), centre(0.01, -0.02, 0.28);
  const double yaw = 30.0 * M_PI / 180.0, h = 0.0153 / 2.0;
  const Vec3 ex(std::cos(yaw), std::sin(yaw), 0.0), ey(-std::sin(yaw), std::cos(yaw), 0.0);
  TagPose t;
  t.id = 4;
  const std::array<Vec3, 4> corners{centre - ex * h - ey * h, centre + ex * h - ey * h, centre + ex * h + ey * h, centre - ex * h + ey * h};
  for (int k = 0; k < 4; ++k) {
    t.corners[k] = cv::Point2f(static_cast<float>(K.fx * corners[k].x() / corners[k].z() + K.cx),
      static_cast<float>(K.fy * corners[k].y() / corners[k].z() + K.cy));
  }
  PuckPose p;
  p.upright = true;
  p.tag = t;
  p.top_center = centre + Vec3(0.010, 0.0, 0.0);
  ASSERT_TRUE(tag_top_refine(p, K, up));
  EXPECT_LT((*p.top_center - centre).norm(), 0.0003);
  EXPECT_LT((p.center - (centre - up * 0.030)).norm(), 0.0003);
  ASSERT_TRUE(p.face_dir.has_value());
  EXPECT_NEAR(std::abs(p.face_dir->dot(ex)), 1.0, 1e-3);   // 태그 변 = 면 방향
  EXPECT_TRUE(p.tag_centre);
  // 마스크가 화면 가장자리에 닿아도 태그로 중심을 잡았으면 거르지 않음, 태그 없으면 거름
  PuckPose cut = p;
  cut.truncated = true;
  cut.n_points = 500;
  EXPECT_EQ(reject_reason(cut), "");
  cut.tag_centre = false;
  EXPECT_EQ(reject_reason(cut), "truncated");
  // 다른 퍽 태그 (윗면 중심에서 30 mm) · 누운 퍽 · 태그 없음 → 그대로
  PuckPose far = p;
  far.top_center = centre + Vec3(0.030, 0.0, 0.0);
  EXPECT_FALSE(tag_top_refine(far, K, up));
  PuckPose lying = p;
  lying.upright = false;
  EXPECT_FALSE(tag_top_refine(lying, K, up));
  PuckPose none = p;
  none.tag.reset();
  EXPECT_FALSE(tag_top_refine(none, K, up));
}

// 누운 네모 퍽 윗면 태그: 깊이로 6 mm 치우친 윗면 중심 · 7° 틀린 긴축을 태그로 바로잡음. 옆면 태그 (윗면 평면이 아님) · 선 퍽은 그대로
TEST(Geometry, TagLyingRefineUsesTagCentreAndLongAxis)
{
  const Intrinsics K{606.4, 606.2, 325.2, 256.8};
  const Vec3 up(0.0, 0.0, -1.0), centre(0.02, 0.01, 0.30);
  const double yaw = 20.0 * M_PI / 180.0, h = 0.0153 / 2.0;
  const Vec3 ex(std::cos(yaw), std::sin(yaw), 0.0), ey(-std::sin(yaw), std::cos(yaw), 0.0);   // ex = 긴축
  auto project = [&](const std::array<Vec3, 4> & q) {
      TagPose t;
      t.id = 7;
      for (int k = 0; k < 4; ++k) {
        t.corners[k] = cv::Point2f(static_cast<float>(K.fx * q[k].x() / q[k].z() + K.cx), static_cast<float>(K.fy * q[k].y() / q[k].z() + K.cy));
      }
      return t;
    };
  PuckPose p;
  p.upright = false;
  p.tag = project({centre - ex * h - ey * h, centre + ex * h - ey * h, centre + ex * h + ey * h, centre - ex * h + ey * h});
  p.top_center = centre + ex * 0.006;
  const double bad = yaw + 7.0 * M_PI / 180.0;
  p.axis = Vec3(std::cos(bad), std::sin(bad), 0.0);
  ASSERT_TRUE(tag_lying_refine(p, K, up));
  EXPECT_LT((*p.top_center - centre).norm(), 0.0003);
  EXPECT_LT((p.center - (centre - up * (PUCK_D / 2.0))).norm(), 0.0003);
  EXPECT_NEAR(std::abs(p.axis.dot(ex)), 1.0, 1e-4);
  EXPECT_TRUE(p.tag_centre);
  // 옆면 (수직) 에 붙은 태그: 윗면 평면으로 옮기면 한 변 길이가 안 맞음 → 그대로
  PuckPose side = p;
  side.tag_centre = false;
  side.top_center = centre + ex * 0.006;
  const Vec3 sc = centre + ey * 0.0174 + up * -0.0174;   // 옆면 중심 (위쪽 = −z 이므로 아래 = +z)
  side.tag = project({sc - ex * h + up * h, sc + ex * h + up * h, sc + ex * h - up * h, sc - ex * h - up * h});
  EXPECT_FALSE(tag_lying_refine(side, K, up));
  EXPECT_FALSE(side.tag_centre);
  // 선 퍽 · 태그 없음 → 그대로
  PuckPose stand = p;
  stand.upright = true;
  EXPECT_FALSE(tag_lying_refine(stand, K, up));
  PuckPose none = p;
  none.tag.reset();
  EXPECT_FALSE(tag_lying_refine(none, K, up));
}

// 태그로 중심을 잡은 장이 있으면 그 장들의 중앙값만 (깊이 점 중심 장은 안 섞음) + center_from_tag
TEST(Tracker, TagCentredFramesWin)
{
  PuckTracker tr(5, 3, 0.02);
  const Vec3 c(0.0, 0.05, 0.3);
  auto tagged = raw_at(c, false);
  tagged.pose.tag_centre = true;
  tr.update({raw_at(c + Vec3(0.006, 0, 0), false)});
  tr.update({tagged});
  auto d = tr.update({raw_at(c + Vec3(0.006, 0, 0), false)});
  ASSERT_EQ(d.size(), 1u);
  EXPECT_TRUE(d[0].center_from_tag);
  EXPECT_NEAR(d[0].position.x(), 0.0, 1e-12);
  // 태그 장이 창에서 빠지면 깊이 점 중심
  for (int i = 0; i < 5; ++i) {d = tr.update({raw_at(c + Vec3(0.006, 0, 0), false)});}
  ASSERT_EQ(d.size(), 1u);
  EXPECT_FALSE(d[0].center_from_tag);
  EXPECT_NEAR(d[0].position.x(), 0.006, 1e-12);
}
