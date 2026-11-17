// test_wrist_tag_rescue.cpp — 손목 퍽 태그 다시 찾기 시험: 실제 36h11 사전으로 그린 퍽 윗면 태그 합성 영상
//   기울임 · 작은 태그 · 그늘 (합친 상자에서 CLAHE 건너뜀) · 상자 경계 잘림 · 옆 퍽 태그 · 처리 시간
//   실행: colcon build → ./build/perception/test_wrist_tag_rescue

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/common.hpp"
#include "perception/wrist_tag_rescue.hpp"

using namespace perception::wrist;

namespace
{
constexpr int W = 640, H = 480;
const Intrinsics K{605.9, 605.4, 324.0, 245.9};   // 실물 D435 컬러 640×480

Mat3 rot(const Vec3 & axis, double a) {return Eigen::AngleAxisd(a, axis.normalized()).toRotationMatrix();}

// 태그 검은 사각 8 × 8 칸 (테두리 1 칸 포함), 칸당 1 px
cv::Mat marker_bits(int id)
{
  cv::Mat bits;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 8, bits, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 8, bits, 1);
#endif
  return bits;
}

// 퍽 윗면 하나: 한 변 face_mm 정사각 (색 → 회색 face_gray), 가운데 흰 라벨 (태그 + 둘레 1 칸) + 태그 (검은 사각 tag_mm)
//   c = 면 중심 (카메라 좌표 m), R = 면 → 카메라 (면 x = 무늬 오른쪽, y = 무늬 아래, z = 카메라에서 멀어지는 쪽)
struct Face
{
  int id{-1};                     // −1 = 태그 없음 (뒤집힌 퍽 · 가려진 태그)
  Vec3 c{0, 0, 0.3};
  Mat3 R{Mat3::Identity()};
  double face_mm{30.0};
  double tag_mm{16.0};
  int face_gray{70};              // 빨강 · 초록 퍽 회색값 약 60–90
  double gain{1.0};               // 밝기 배율 (그늘 < 1)
  double lift{0.0};               // 밝기 더함 (gain 과 같이 쓰면 대비만 낮춤: 뿌연 그늘 · 반사)
  double tag_dx_mm{0.0};          // 태그를 면 중심에서 옮김 (면 x, mm)
  double tag_dy_mm{0.0};
};

struct Scene
{
  cv::Mat gray;
  std::vector<cv::Mat1b> masks;   // 면마다 원본 크기, 255 = 퍽
  std::vector<cv::Rect> boxes;    // 마스크 상자
};

cv::Point2f project(const Vec3 & p) {return {static_cast<float>(K.fx * p.x() / p.z() + K.cx), static_cast<float>(K.fy * p.y() / p.z() + K.cy)};}

Scene render(const std::vector<Face> & faces, double blur_sigma = 0.6, double noise = 2.0, int bg = 120, unsigned seed = 1)
{
  Scene s;
  cv::Mat img(H, W, CV_8U, cv::Scalar(bg));   // 박스 바닥
  const int ppm = 12;                          // 무늬 화소 / mm
  for (const auto & f : faces) {
    const int T = static_cast<int>(std::lround(f.face_mm * ppm));
    cv::Mat tex(T, T, CV_8U, cv::Scalar(f.face_gray));
    if (f.id >= 0) {
      const double cell = f.tag_mm / 8.0;
      const int lab = static_cast<int>(std::lround(cell * 10 * ppm));   // 흰 라벨 = 태그 + 둘레 1 칸
      const int tag = static_cast<int>(std::lround(f.tag_mm * ppm));
      const int cx = T / 2 + static_cast<int>(std::lround(f.tag_dx_mm * ppm)), cy = T / 2 + static_cast<int>(std::lround(f.tag_dy_mm * ppm));
      tex(cv::Rect(cx - lab / 2, cy - lab / 2, lab, lab)).setTo(255);
      cv::Mat big;
      cv::resize(marker_bits(f.id), big, cv::Size(tag, tag), 0, 0, cv::INTER_NEAREST);
      big.copyTo(tex(cv::Rect(cx - tag / 2, cy - tag / 2, tag, tag)));
    }
    tex.convertTo(tex, CV_8U, f.gain, f.lift);
    // 무늬 화소 (u, v) → 면 (m) → 카메라
    const double k = 0.001 / ppm;
    const cv::Matx33d A(k, 0, -k * T / 2.0, 0, k, -k * T / 2.0, 0, 0, 1);
    const cv::Matx33d M(f.R(0, 0), f.R(0, 1), f.c.x(), f.R(1, 0), f.R(1, 1), f.c.y(), f.R(2, 0), f.R(2, 1), f.c.z());
    const cv::Matx33d Kc(K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0, 1);
    cv::warpPerspective(tex, img, cv::Mat(Kc * M * A), img.size(), cv::INTER_AREA, cv::BORDER_TRANSPARENT);
    // 마스크 = 면 사각형 투영
    const double h = f.face_mm / 2000.0;
    std::vector<cv::Point> poly;
    for (const auto & [x, y] : std::vector<std::pair<double, double>>{{-h, -h}, {h, -h}, {h, h}, {-h, h}}) {
      const auto q = project(f.c + f.R * Vec3(x, y, 0));
      poly.emplace_back(static_cast<int>(std::lround(q.x)), static_cast<int>(std::lround(q.y)));
    }
    cv::Mat1b m = cv::Mat1b::zeros(H, W);
    cv::fillConvexPoly(m, poly, cv::Scalar(255));
    s.masks.push_back(m);
    s.boxes.push_back(cv::boundingRect(m));
  }
  if (blur_sigma > 0) {cv::GaussianBlur(img, img, cv::Size(0, 0), blur_sigma);}
  cv::Mat n16(img.size(), CV_16S);
  cv::RNG rng(seed);
  rng.fill(n16, cv::RNG::NORMAL, 0, noise);
  cv::Mat out;
  img.convertTo(out, CV_16S);
  out += n16;
  out.convertTo(s.gray, CV_8U);
  return s;
}

// 손목 1차 경로 (wrist_pipeline: 분할 상자 + 24 px, 합친 상자 검출 → puck_pose 의 상자 ±5 px 규칙) 로 퍽마다 ID
struct Primary
{
  std::vector<TagPose> tags;
  std::vector<int> ids;
};
Primary primary_path(const Scene & s, int pad = 24)
{
  Primary p;
  std::vector<cv::Rect> rois;
  for (const auto & b : s.boxes) {rois.emplace_back(b.x - pad, b.y - pad, b.width + 2 * pad, b.height + 2 * pad);}
  p.tags = detect_tags_in(s.gray, rois, K);
  for (const auto & b : s.boxes) {
    const int x0 = b.x, y0 = b.y, x1 = b.x + b.width - 1, y1 = b.y + b.height - 1;
    int id = -1;
    for (const auto & t : p.tags) {
      cv::Point2f c(0, 0);
      for (const auto & q : t.corners) {c += q * 0.25f;}
      const int cx = static_cast<int>(c.x), cy = static_cast<int>(c.y);
      if (x0 - 5 <= cx && cx <= x1 + 5 && y0 - 5 <= cy && cy <= y1 + 5) {id = t.id; break;}
    }
    p.ids.push_back(id);
  }
  return p;
}

std::vector<RescuePuck> pucks_of(const Scene & s, const std::vector<int> & own)
{
  std::vector<RescuePuck> v(s.masks.size());
  for (size_t i = 0; i < v.size(); ++i) {
    v[i].mask = s.masks[i];
    v[i].box = s.boxes[i];
    v[i].own_id = own[i];
    v[i].need = own[i] < 0;
  }
  return v;
}

// 카메라 앞 dist m, 화면 (u, v) 쪽, 면이 x 축으로 tilt_deg 기울고 면 안에서 spin_deg 돈 퍽 윗면
Face face_at(int id, double u, double v, double dist, double tilt_deg, double spin_deg = 0.0)
{
  Face f;
  f.id = id;
  f.c = Vec3((u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0) * dist;
  f.R = rot(Vec3::UnitX(), tilt_deg * M_PI / 180) * rot(Vec3::UnitZ(), spin_deg * M_PI / 180);
  return f;
}
}  // namespace

// AprilTag 3 모서리 순서 = aruco 순서 (좌상 · 우상 · 우하 · 좌하), 좌표 = OpenCV 화소 중심 규약 (참값 투영과 비교)
TEST(TagRescue, At3CornerOrderMatchesAruco)
{
  if (!apriltag3_available()) {GTEST_SKIP() << "AprilTag 3 없이 빌드";}
  TagRescuer r;
  double bias_at3 = 0, bias_ar = 0;
  int n = 0;
  for (double spin : {0.0, 30.0, 100.0, -140.0}) {
    const Face f = face_at(4, 320, 240, 0.25, 20.0, spin);
    const Scene s = render({f}, 0.0, 0.0);
    const cv::Rect roi(0, 0, W, H);
    const auto a = r.detect_once(s.gray, roi, "at3");
    const auto b = r.detect_once(s.gray, roi, "aruco");
    ASSERT_EQ(a.size(), 1u) << spin;
    ASSERT_EQ(b.size(), 1u) << spin;
    EXPECT_EQ(a[0].id, 4);
    EXPECT_EQ(b[0].id, 4);
    const double h = 0.008;   // 검은 사각 반 변
    const std::array<Vec3, 4> obj{Vec3(-h, -h, 0), Vec3(h, -h, 0), Vec3(h, h, 0), Vec3(-h, h, 0)};   // 무늬 좌상 · 우상 · 우하 · 좌하
    for (int i = 0; i < 4; ++i) {
      const auto t = project(f.c + f.R * obj[i]);
      EXPECT_NEAR(a[0].corners[i].x, t.x, 0.6) << spin << " " << i;
      EXPECT_NEAR(a[0].corners[i].y, t.y, 0.6) << spin << " " << i;
      EXPECT_NEAR(b[0].corners[i].x, t.x, 1.5) << spin << " " << i;
      EXPECT_NEAR(b[0].corners[i].y, t.y, 1.5) << spin << " " << i;
      bias_at3 += cv::norm(a[0].corners[i] - t);
      bias_ar += cv::norm(b[0].corners[i] - t);
      ++n;
    }
  }
  std::printf("모서리 참값 대비 평균 거리: AprilTag 3 %.2f px · aruco %.2f px\n", bias_at3 / n, bias_ar / n);
}

// 주인 퍽 규칙: 마스크 안 → 그 퍽, 틈에서 margin 안 가장 가까운 퍽, 같은 거리 → 안 붙임
TEST(TagRescue, OwnerRule)
{
  std::vector<RescuePuck> p(2);
  for (auto & q : p) {q.mask = cv::Mat1b::zeros(H, W);}
  p[0].box = cv::Rect(100, 100, 40, 40);   // x 100–139
  p[1].box = cv::Rect(151, 100, 40, 40);   // x 151–190, 틈 11 px (140–150)
  p[0].mask(p[0].box).setTo(255);
  p[1].mask(p[1].box).setTo(255);
  EXPECT_EQ(tag_owner({120, 120}, p, 8), 0);
  EXPECT_EQ(tag_owner({170, 120}, p, 8), 1);
  EXPECT_EQ(tag_owner({141, 120}, p, 8), 0);    // 왼쪽 마스크에서 2 px
  EXPECT_EQ(tag_owner({149, 120}, p, 8), 1);    // 오른쪽에서 2 px
  EXPECT_EQ(tag_owner({145, 120}, p, 8), -1);   // 양쪽 6 px 같음 → 애매, 안 붙임
  EXPECT_EQ(tag_owner({120, 160}, p, 8), -1);   // 아래로 21 px → 아무도 아님
  EXPECT_EQ(tag_owner({120, 145}, p, 8), 0);    // 아래로 6 px → 왼쪽
  EXPECT_EQ(tag_owner({120, 145}, p, 4), -1);   // 여유 4 px 면 아님
  EXPECT_EQ(tag_owner({-5, -5}, p, 8), -1);     // 영상 밖
  // 겹친 마스크: 상자 중심이 가까운 퍽
  p[1].mask(cv::Rect(130, 100, 21, 40)).setTo(255);
  p[1].box = cv::Rect(130, 100, 61, 40);
  EXPECT_EQ(tag_owner({132, 120}, p, 8), 0);    // 왼쪽 상자 중심 120 ↔ 오른쪽 160
}

// 탐색용: 거리 (태그 화소) · 기울기 · 흐림 · 밝기 격자에서 1차 경로 (상자 + 24 px · + 48 px · 전체 영상) 대비 다시 찾기 성공 ·
//   틀린 ID 합계 (환경변수 TAG_SWEEP=1). 한 장에 퍽 둘 (가까이, 둘째는 그늘) 도 같이
TEST(TagRescue, Sweep)
{
  if (!std::getenv("TAG_SWEEP")) {GTEST_SKIP();}
  for (double mm : {30.0, 20.0}) {
    for (int use_at3 : {0, 1}) {
      TagRescueParams prm;
      prm.at3_min_margin = mm;
      prm.apriltag3 = use_at3 == 1;
      TagRescuer r(prm);
      int n = 0, prim = 0, prim48 = 0, full = 0, resc = 0, wrong = 0;
      double ms = 0;
      for (double dist : {0.30, 0.38, 0.45, 0.55}) {
        for (double tilt : {0.0, 40.0, 55.0, 65.0}) {
          for (double blur : {0.6, 1.0, 1.4}) {
            for (double gain : {1.0, 0.5, 0.3}) {
              for (int k = 0; k < 6; ++k) {
                Face f = face_at(k, 300 + 10 * k, 250, dist, tilt, 37.0 * k);
                Face g = face_at(k + 6, 300 + 10 * k + 70, 250, dist, tilt, 23.0 * k);
                g.gain = gain;
                const Scene s = render({f, g}, blur, 2.0, 120, 7 + k);
                const auto pp = primary_path(s);
                const auto p48 = primary_path(s, 48);
                const auto all = detect_tags(s.gray, K);
                std::vector<RescuePuck> pk = pucks_of(s, pp.ids);
                const auto t0 = std::chrono::steady_clock::now();
                const auto got = r.rescue(s.gray, K, pk, pp.tags);
                ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                for (int q = 0; q < 2; ++q) {
                  const int want = q == 0 ? k : k + 6;
                  ++n;
                  prim += pp.ids[q] == want;
                  prim48 += p48.ids[q] == want;
                  full += std::any_of(all.begin(), all.end(), [&](const TagPose & t) {return t.id == want;});
                  const int id = pp.ids[q] >= 0 ? pp.ids[q] : (got[q] ? got[q]->id : -1);
                  if (id == want) {++resc;} else if (id >= 0) {++wrong;}
                }
              }
            }
          }
        }
      }
      std::printf("min_margin %2.0f at3 %d: 퍽 %d · 1차 (+24 px) %d · 1차 (+48 px) %d · 전체 영상 %d · 다시 찾기 포함 %d · 틀린 ID %d · "
        "다시 찾기 장당 평균 %.2f ms\n", mm, use_at3, n, prim, prim48, full, resc, wrong, ms / (n / 2));
    }
  }
}

// 기울고 작고 흐린 태그: 1차 (aruco) 는 못 읽고 AprilTag 3 다시 찾기는 읽음, 틀린 ID 없음
TEST(TagRescue, TiltedSmallTagAt3)
{
  if (!apriltag3_available()) {GTEST_SKIP() << "AprilTag 3 없이 빌드";}
  TagRescuer r;
  int prim = 0, resc = 0, wrong = 0;
  for (int k = 0; k < 12; ++k) {
    // 0.55 m · 55° 기울기 = 화면 태그 약 18 × 10 px (Sweep 표: 1차 0/6, AprilTag 3 6/6)
    const Face f = face_at(k, 280 + 8 * k, 250, 0.55, 55.0, 29.0 * k);
    const Scene s = render({f}, 0.6, 2.0, 120, 11 + k);
    const auto pp = primary_path(s);
    prim += pp.ids[0] == k;
    if (pp.ids[0] >= 0) {continue;}
    RescueStats st;
    const auto got = r.rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags, &st);
    EXPECT_EQ(st.rois, 1);
    if (got[0]) {
      (got[0]->id == k ? resc : wrong)++;
      EXPECT_EQ(st.how[0], "at3");
    }
  }
  std::printf("기울고 작은 태그 12 개: 1차 %d · 다시 찾기 %d · 틀린 ID %d\n", prim, resc, wrong);
  EXPECT_GE(resc, 4);
  EXPECT_GE(prim + resc, 10);
  EXPECT_EQ(wrong, 0);
}

// 1차는 붙은 두 퍽 상자를 합쳐 한 번 검출 → 밝은 A 태그를 원본에서 읽으면 CLAHE 를 건너뛰어 그늘진 B 태그를 놓침
//   다시 찾기는 퍽마다 넓힌 상자 + CLAHE (AprilTag 3 없이 aruco 만으로도)
TEST(TagRescue, MergedRoiSkipsClahe)
{
  auto scene = [](int a_id, int k) {
      Face a = face_at(a_id, 280, 240, 0.32, 20.0, 10.0 * k);
      Face b = face_at(8, 342, 240, 0.32, 20.0, 25.0 * k);
      b.gain = 0.05;   // 대비 약 13 단계 (그늘 · 뿌연 반사)
      b.lift = 60;
      return render({a, b}, 0.6, 1.5, 120, 3 + k);
    };
  TagRescueParams aruco_only;
  aruco_only.apriltag3 = false;   // aruco 만으로도 고쳐지는지
  TagRescuer ra(aruco_only), rd;  // rd = 기본 (AprilTag 3 있으면 그것부터)
  for (int k = 0; k < 6; ++k) {
    const Scene s = scene(3, k);
    const auto pp = primary_path(s);
    EXPECT_EQ(pp.ids[0], 3) << k;
    EXPECT_EQ(pp.ids[1], -1) << k;                       // 놓침 (원인 재현)
    EXPECT_EQ(primary_path(scene(-1, k)).ids[1], 8) << k;   // A 에 태그가 없으면 1차 CLAHE 가 돌아 B 를 읽음
    for (auto * r : {&ra, &rd}) {
      RescueStats st;
      const auto got = r->rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags, &st);
      EXPECT_FALSE(got[0]);
      ASSERT_TRUE(got[1]) << k;
      EXPECT_EQ(got[1]->id, 8);
      if (r == &ra) {EXPECT_EQ(st.how[1], "aruco_clahe");}
      if (r == &rd && k == 0) {std::printf("그늘진 B: 기본 설정은 %s 로 찾음\n", st.how[1].c_str());}
    }
  }
}

// 분할 마스크가 퍽 일부만 덮어 태그가 상자 + 24 px 밖으로 걸침: 1차는 잘린 태그를 못 읽거나 상자 ±5 px 밖이라 안 붙임
//   다시 찾기는 상자 + 48 px 로 읽고, 태그 중심이 마스크에서 margin (8 px) 안이라 붙임
TEST(TagRescue, RoiCutsTag)
{
  TagRescueParams prm;
  prm.apriltag3 = false;   // 상자 문제만 보려고 같은 검출기 (aruco)
  TagRescuer r(prm);
  for (double spin : {0.0, 15.0}) {
    const Face f = face_at(6, 320, 240, 0.24, 0.0, spin);   // 태그 약 40 px
    Scene s = render({f}, 0.6, 1.5);
    // 마스크 = 태그 중심보다 6 px 아래부터만 (분할이 흰 라벨 쪽을 덜 잡음)
    const auto c = project(f.c);
    s.masks[0](cv::Rect(0, 0, W, static_cast<int>(c.y) + 6)).setTo(0);
    s.boxes[0] = cv::boundingRect(s.masks[0]);
    const auto pp = primary_path(s);
    EXPECT_EQ(pp.ids[0], -1) << spin;
    RescueStats st;
    const auto got = r.rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags, &st);
    ASSERT_TRUE(got[0]) << spin;
    EXPECT_EQ(got[0]->id, 6);
    // 여유 4 px 면 안 붙임 (태그 중심이 마스크에서 6 px)
    TagRescueParams tight = prm;
    tight.margin_px = 4;
    TagRescuer r2(tight);
    EXPECT_FALSE(r2.rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags)[0]) << spin;
  }
}

// 옆 퍽 태그를 뺏지 않음: 태그 없는 퍽 A 바로 옆 B 의 태그
TEST(TagRescue, NeighbourTagNotStolen)
{
  TagRescuer r;
  const Face a = face_at(-1, 290, 240, 0.32, 20.0);   // 태그 안 보이는 퍽
  const Face b = face_at(7, 290 + 58, 240, 0.32, 20.0, 30.0);
  const Scene s = render({a, b});
  const auto pp = primary_path(s);
  ASSERT_EQ(pp.ids[1], 7);
  // 1) B 가 이번 장 태그를 가짐 → A 는 못 받음
  {
    RescueStats st;
    const auto got = r.rescue(s.gray, K, pucks_of(s, {-1, 7}), pp.tags, &st);
    EXPECT_FALSE(got[0]);
    EXPECT_FALSE(got[1]);
    EXPECT_EQ(st.rois, 1);
  }
  // 2) B 도 이번 장 태그 없음 (둘 다 다시 찾기) → A 상자에서 읽힌 7 은 마스크 주인 B 에게만
  {
    RescueStats st;
    const auto got = r.rescue(s.gray, K, pucks_of(s, {-1, -1}), {}, &st);
    EXPECT_FALSE(got[0]);
    ASSERT_TRUE(got[1]);
    EXPECT_EQ(got[1]->id, 7);
    EXPECT_EQ(st.rois, 1);   // B 는 A 상자 검출에서 받아서 따로 안 함
  }
  // 3) B 는 이번 장 태그 없지만 묶음이 ID 7 을 가짐 (need false, own_id 7) → A 는 7 을 못 받음
  {
    auto pk = pucks_of(s, {-1, -1});
    pk[1].need = false;
    pk[1].own_id = 7;
    EXPECT_FALSE(r.rescue(s.gray, K, pk, {})[0]);
  }
  // 4) 태그 중심이 두 마스크에서 같은 거리 (틈 한가운데) → 아무에게도 안 붙임
  {
    auto pk = pucks_of(s, {-1, -1});
    for (auto & q : pk) {q.mask = cv::Mat1b::zeros(H, W);}
    // 7 태그 중심 양쪽 11 px 에 세로 띠 마스크 둘
    const auto t = r.detect_once(s.gray, cv::Rect(0, 0, W, H), apriltag3_available() ? "at3" : "aruco");
    ASSERT_FALSE(t.empty());
    const auto c = t[0].centre();
    const int cx = static_cast<int>(std::lround(c.x));
    pk[0].mask(cv::Rect(cx - 40, 150, 30, 200)).setTo(255);   // 오른쪽 끝 cx − 11
    pk[1].mask(cv::Rect(cx + 11, 150, 30, 200)).setTo(255);   // 왼쪽 끝 cx + 11
    pk[0].box = cv::Rect(cx - 40, 150, 30, 200);
    pk[1].box = cv::Rect(cx + 11, 150, 30, 200);
    TagRescueParams wide;
    wide.margin_px = 12;
    TagRescuer r3(wide);
    const auto got = r3.rescue(s.gray, K, pk, {});   // 태그 중심 (반올림 cx) 에서 양쪽 마스크 다 11 px
    EXPECT_FALSE(got[0]);
    EXPECT_FALSE(got[1]);
  }
}

// 1차에서 읽혔지만 상자 ±5 px 밖이라 안 붙은 태그는 검출 없이 붙임
TEST(TagRescue, UnattachedPrimaryTagUsedFirst)
{
  TagRescuer r;
  const Face f = face_at(2, 320, 240, 0.30, 0.0);
  Scene s = render({f});
  const auto c = project(f.c);
  s.masks[0](cv::Rect(0, 0, W, static_cast<int>(c.y) + 7)).setTo(0);   // 태그 중심 7 px 위까지 마스크 없음
  s.boxes[0] = cv::boundingRect(s.masks[0]);
  const auto all = detect_tags(s.gray, K);   // 전체 영상 1차 (wrist.tag_roi false 와 같음)
  ASSERT_EQ(all.size(), 1u);
  RescueStats st;
  const auto got = r.rescue(s.gray, K, pucks_of(s, {-1}), all, &st);
  ASSERT_TRUE(got[0]);
  EXPECT_EQ(got[0]->id, 2);
  EXPECT_EQ(st.how[0], "primary");
  EXPECT_EQ(st.from_primary, 1);
  EXPECT_EQ(st.rois, 0);
}

// 시간 한도: 남은 퍽은 다음 장으로 (최소 1 개는 함)
TEST(TagRescue, BudgetDefersPucks)
{
  TagRescueParams prm;
  prm.budget_ms = 1e-6;
  TagRescuer r(prm);
  const Scene s = render({face_at(-1, 160, 240, 0.32, 20.0), face_at(-1, 320, 240, 0.32, 20.0), face_at(-1, 480, 240, 0.32, 20.0)});
  RescueStats st;
  r.rescue(s.gray, K, pucks_of(s, {-1, -1, -1}), {}, &st);
  EXPECT_EQ(st.rois, 1);
  EXPECT_EQ(st.over_budget, 3);                   // 첫 상자 안 단계 사이 1 + 넘긴 상자 2
  EXPECT_EQ(st.at3_runs + st.aruco_runs, 1);      // 첫 상자도 첫 단계 뒤 멈춤
  EXPECT_EQ(st.tried[0], 1);
  EXPECT_EQ(st.tried[1], 0);
}

TEST(Tracker, KnownIdNear)
{
  PuckTracker tr(5, 3, 0.02);
  RawPuck r;
  r.pose.center = Vec3(0.01, 0.02, 0.30);
  r.pose.n_points = 500;
  r.info.color = "red";
  TagPose t;
  t.id = 5;
  r.pose.tag = t;
  EXPECT_EQ(tr.known_id_near(r.pose.center), -1);   // 묶음 없음
  tr.update({r});
  EXPECT_EQ(tr.known_id_near(r.pose.center + Vec3(0.01, 0, 0)), 5);
  EXPECT_EQ(tr.known_id_near(r.pose.center + Vec3(0.05, 0, 0)), -1);   // 다른 자리
  // 태그 없는 장이 이어지면 다음 장 기준 창 (5 장) 밖으로 나가 −1
  RawPuck n = r;
  n.pose.tag.reset();
  for (int i = 0; i < 3; ++i) {tr.update({n});}
  EXPECT_EQ(tr.known_id_near(r.pose.center), 5);    // 태그 본 장 0, 다음 장 4 → 창 안
  tr.update({n});
  EXPECT_EQ(tr.known_id_near(r.pose.center), -1);   // 다음 장 5 → 창 밖
}

namespace
{
// 끝까지 (손목 파이프라인과 같은 순서): 분할 → puck_pose → 다시 찾기 → 묶음
struct E2E
{
  std::vector<SegInstance> inst;
  cv::Mat1f depth;
  Vec3 up{0, 0, -1};   // 카메라가 똑바로 내려다봄
};
E2E e2e_of(const Scene & s, const std::vector<Face> & faces, double floor_z)
{
  E2E e;
  e.depth = cv::Mat1f(H, W, static_cast<float>(floor_z));
  for (size_t i = 0; i < faces.size(); ++i) {
    e.depth.setTo(static_cast<float>(faces[i].c.z()), s.masks[i]);   // 면이 카메라와 나란 (기울기 0)
    SegInstance si;
    si.kind = "red_box_up";
    si.conf = 0.9f;
    si.mask = s.masks[i];
    const auto & b = s.boxes[i];
    si.box = {static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.x + b.width - 1), static_cast<float>(b.y + b.height - 1)};
    e.inst.push_back(si);
  }
  return e;
}
void raws_of(const E2E & e, const std::vector<TagPose> & tags, std::vector<RawPuck> & raws, std::vector<int> & raw_of)
{
  raws.clear();
  raw_of.assign(e.inst.size(), -1);
  for (size_t k = 0; k < e.inst.size(); ++k) {
    auto p = puck_pose(e.inst[k].mask, e.depth, K, tags, &e.up, 0.0, false);
    if (!p) {continue;}
    RawPuck r;
    r.pose = std::move(*p);
    r.kind = e.inst[k].kind;
    r.info = parse_kind(r.kind);
    r.reject = reject_reason(r.pose);
    raw_of[k] = static_cast<int>(raws.size());
    raws.push_back(std::move(r));
  }
}
}  // namespace

// 끝까지: 1차가 못 읽은 퍽에 ID 가 붙고, 묶음이 ID 를 기억하는 동안은 다시 찾지 않음. 태그 안 보이는 퍽은 retry 장 쉼
TEST(TagRescue, EndToEndWithTracker)
{
  if (!apriltag3_available()) {GTEST_SKIP() << "AprilTag 3 없이 빌드";}
  TagRescueParams prm;
  prm.budget_ms = 0.0;   // 시간 한도는 따로 시험 (BudgetDefersPucks), 여기선 느린 빌드에서도 장마다 같게
  TagRescuer r(prm);
  RescueCooldown cool(2, 0.02);
  PuckTracker tr(5, 3, 0.02);
  // 퍽 0: 작고 흐린 태그 9 (1차 못 읽음), 퍽 1: 태그 안 보임
  const std::vector<Face> faces{face_at(9, 250, 240, 0.55, 0.0, 20.0), face_at(-1, 400, 240, 0.55, 0.0)};
  std::vector<int> tried_1;
  for (int frame = 0; frame < 8; ++frame) {
    const Scene s = render(faces, 1.0, 2.0, 120, 50 + frame);
    const E2E e = e2e_of(s, faces, 0.61);
    const auto prim = primary_path(s);
    std::vector<RawPuck> raws;
    std::vector<int> raw_of;
    raws_of(e, prim.tags, raws, raw_of);
    ASSERT_EQ(raws.size(), 2u);
    RescueStats st;
    rescue_raw_pucks(r, s.gray, e.depth, K, e.up, e.inst, raw_of, raws, prim.tags, tr, 0.0, false, &cool, &st);
    tried_1.push_back(st.tried[1]);
    if (frame == 0) {
      ASSERT_TRUE(raws[0].pose.tag) << "1차 " << prim.ids[0];
      EXPECT_EQ(raws[0].pose.tag->id, 9);
      EXPECT_EQ(raws[0].reject, "");
      EXPECT_EQ(st.how[0], "at3");
    }
    const auto dets = tr.update(raws);
    if (frame >= 2) {   // min_hits 3 → 셋째 장부터 확정
      ASSERT_EQ(dets.size(), 2u);
      int with9 = 0;
      for (const auto & d : dets) {with9 += d.id == "9";}
      EXPECT_EQ(with9, 1) << frame;   // 퍽 0 은 줄곧 ID 9, 퍽 1 은 ID 없음
    }
    if (frame >= 1 && frame <= 4) {EXPECT_EQ(st.how[0], "") << frame;}   // 묶음이 ID 를 기억하는 동안 퍽 0 은 다시 안 찾음
  }
  // 퍽 1 (태그 안 보임): 0 장 시도 → 1 · 2 쉼 → 3 시도 → 4 · 5 쉼 → 6 시도
  EXPECT_EQ(tried_1, (std::vector<int>{1, 0, 0, 1, 0, 0, 1, 0}));
}

// 화면 끝에 잘린 퍽 (truncated 로 걸림): tag_centre 일 때만 태그를 다시 찾아 태그 중심으로 통과
//   (tag_centre false = 모양 중심 설정에서는 안 찾음 — 다시 찾은 태그 중심 장이 묶음의 모양 중심 장을 밀어내지 않게)
TEST(TagRescue, TruncatedPuckPassesWithRescuedTag)
{
  if (!apriltag3_available()) {GTEST_SKIP() << "AprilTag 3 없이 빌드";}
  TagRescuer r;
  PuckTracker tr;
  const std::vector<Face> faces{face_at(1, 12, 240, 0.55, 0.0)};   // 면 반쯤 화면 밖, 태그는 안
  const Scene s = render(faces, 1.0, 2.0, 120, 5);
  const E2E e = e2e_of(s, faces, 0.61);
  const auto prim = primary_path(s);
  std::vector<RawPuck> raws;
  std::vector<int> raw_of;
  raws_of(e, prim.tags, raws, raw_of);
  ASSERT_EQ(raws.size(), 1u);
  ASSERT_EQ(raws[0].reject, "truncated");
  if (raws[0].pose.tag) {GTEST_SKIP() << "1차가 이미 읽음";}
  {
    auto raws_shape = raws;   // 모양 중심 설정: 안 찾음, 그대로 truncated
    TagRescuer r2;
    EXPECT_EQ(rescue_raw_pucks(r2, s.gray, e.depth, K, e.up, e.inst, raw_of, raws_shape, prim.tags, tr, 0.0, false), 0);
    EXPECT_EQ(raws_shape[0].reject, "truncated");
  }
  EXPECT_EQ(rescue_raw_pucks(r, s.gray, e.depth, K, e.up, e.inst, raw_of, raws, prim.tags, tr, 0.0, true), 1);
  ASSERT_TRUE(raws[0].pose.tag);
  EXPECT_EQ(raws[0].pose.tag->id, 1);
  EXPECT_TRUE(raws[0].pose.tag_centre);
  EXPECT_EQ(raws[0].reject, "");
  EXPECT_NEAR(raws[0].pose.center.x(), faces[0].c.x(), 0.003);   // 태그 중심 = 면 중심
}

// 처리 시간 (노트북): 1차 · 다시 찾기 (못 찾는 최악 · 찾는 경우 · 시간 한도)
TEST(TagRescue, Timing)
{
  TagRescueParams nolimit;
  nolimit.budget_ms = 0.0;
  TagRescuer r(nolimit), rb;
  // 태그 안 보이는 퍽 3 + 태그 퍽 1 (0.32 m · 30°)
  std::vector<Face> faces{face_at(-1, 140, 240, 0.32, 30.0), face_at(-1, 300, 240, 0.32, 30.0), face_at(-1, 460, 240, 0.32, 30.0),
    face_at(4, 320, 380, 0.32, 30.0)};
  const Scene s = render(faces, 0.8, 2.0);
  const int N = 40;
  auto avg = [&](auto && f) {
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < N; ++i) {f();}
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / N;
    };
  Primary pp;
  const double t_prim = avg([&] {pp = primary_path(s);});
  const double t_full = avg([&] {detect_tags(s.gray, K);});
  ASSERT_EQ(pp.ids[3], 4);
  RescueStats st;
  const double t_fail3 = avg([&] {r.rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags, &st);});
  EXPECT_EQ(st.rois, 3);
  const double t_budget = avg([&] {rb.rescue(s.gray, K, pucks_of(s, pp.ids), pp.tags, &st);});
  const int rois_budget = st.rois;
  auto one = pucks_of(s, {-1, 7, 8, 4});
  one[1].need = one[2].need = false;
  const double t_fail1 = avg([&] {r.rescue(s.gray, K, one, pp.tags, &st);});
  // 찾는 경우: 태그 퍽을 태그 없음으로 넘김
  auto ok = pucks_of(s, {0, 7, 8, -1});
  for (int i = 0; i < 3; ++i) {ok[static_cast<size_t>(i)].need = false;}
  const double t_found = avg([&] {r.rescue(s.gray, K, ok, {}, &st);});
  EXPECT_EQ(st.found, 1);
  std::printf("[노트북] 1차 상자 검출 %.2f ms · 전체 영상 aruco %.2f ms | 다시 찾기: 못 찾는 퍽 1 개 %.2f ms · 3 개 %.2f ms "
    "(한도 10 ms 면 %.2f ms, 상자 %d 개) · 찾는 퍽 1 개 %.2f ms (%s)\n",
    t_prim, t_full, t_fail1, t_fail3, t_budget, rois_budget, t_found, st.how[3].c_str());
}

// 탐색용: 단계 묶음별 (1차 실패분 중) 성공 수 · 넓힌 상자 여유별 (TAG_SWEEP=1)
TEST(TagRescue, MethodUnion)
{
  if (!std::getenv("TAG_SWEEP")) {GTEST_SKIP();}
  TagRescuer r;
  for (int pad : {32, 40, 48}) {
    int fail = 0, u1 = 0, u2 = 0, u3 = 0, u4 = 0, only_arc = 0;
    for (double dist : {0.30, 0.38, 0.45, 0.55}) {
      for (double tilt : {0.0, 40.0, 55.0, 65.0}) {
        for (double blur : {0.6, 1.0, 1.4}) {
          for (double gain : {1.0, 0.5, 0.3, 0.1}) {
            for (int k = 0; k < 6; ++k) {
              Face f = face_at(k, 300 + 10 * k, 250, dist, tilt, 37.0 * k);
              f.gain = gain;
              const Scene s = render({f}, blur, 2.0, 120, 7 + k);
              if (primary_path(s).ids[0] == k) {continue;}
              ++fail;
              const cv::Rect roi(s.boxes[0].x - pad, s.boxes[0].y - pad, s.boxes[0].width + 2 * pad, s.boxes[0].height + 2 * pad);
              auto has = [&](const std::string & m) {
                  for (const auto & t : r.detect_once(s.gray, roi, m)) {if (t.id == k) {return true;}}
                  return false;
                };
              const bool a = has("at3"), b = has("aruco"), c = has("at3_clahe"), d = has("aruco_clahe");
              u1 += a; u2 += a || b; u3 += a || b || c; u4 += a || b || c || d;
              only_arc += d && !(a || b || c);
            }
          }
        }
      }
    }
    std::printf("pad %d: 1차 실패 %d 중 at3 %d · +aruco %d · +at3_clahe %d · +aruco_clahe %d (aruco_clahe 만 %d)\n",
      pad, fail, u1, u2, u3, u4, only_arc);
  }
}
