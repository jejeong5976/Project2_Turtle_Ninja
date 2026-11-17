// test_wrist_seg.cpp — 손목 퍽 분할 전후처리 시험, 합성 출력으로 TensorRT 없이
//   실행: colcon build → source install/setup.bash → ./build/perception/test_wrist_seg

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception/wrist_seg.hpp"

using namespace perception::wrist;

TEST(Segmenter, OneToOneMaskInsideBox)
{
  // 입력 64x64 (여백 없음), one2one 출력 [1, A=1, C=4+nc+nm], 마스크 원형 [1, nm=2, 16, 16]
  std::vector<float> chw;
  const cv::Mat img(64, 64, CV_8UC3, cv::Scalar::all(0));
  const auto lb = letterbox(img, {64, 64}, chw);
  EXPECT_EQ(chw.size(), 3u * 64 * 64);

  SegOutputs o;                                            // 앵커 1 개라 shape_outputs 판정(C < A) 대신 직접 넣음
  o.channels_first = false;
  o.n_anchor = 1;
  o.n_ch = 4 + 2 + 2;
  o.nm = 2;
  o.mh = o.mw = 16;
  o.det = {8, 8, 40, 40, 0.1f, 0.9f, 1.0f, 0.0f};          // 상자 xyxy · 클래스 점수 2 · 계수 2
  o.proto.assign(2 * 16 * 16, 0.0f);
  std::fill(o.proto.begin(), o.proto.begin() + 16 * 16, 1.0f);   // 원형 0 = 전부 1 → 로짓 1

  const auto inst = postprocess(o, lb, img.size(), {"red_cyl_up", "green_box_up"}, 0.5f, 0.7f);
  ASSERT_EQ(inst.size(), 1u);
  EXPECT_EQ(inst[0].kind, "green_box_up");
  EXPECT_FLOAT_EQ(inst[0].conf, 0.9f);
  EXPECT_FLOAT_EQ(inst[0].box[0], 8.0f);
  EXPECT_FLOAT_EQ(inst[0].box[3], 40.0f);
  ASSERT_EQ(inst[0].mask.size(), cv::Size(64, 64));
  EXPECT_EQ(cv::countNonZero(inst[0].mask), 32 * 32);   // 상자 밖은 0
  EXPECT_EQ(inst[0].mask(0, 0), 0);
  EXPECT_EQ(inst[0].mask(20, 20), 255);
}

TEST(Segmenter, NmsGroupsJoinUpAndLying)
{
  const std::vector<std::string> n8 = {"red_cyl_up", "green_cyl_up", "red_box_up", "green_box_up",
    "red_cyl_lying", "green_cyl_lying", "red_box_lying", "green_box_lying"};
  const auto g = nms_groups(n8, 8);
  EXPECT_EQ(g[0], g[4]);                                   // red_cyl up · lying
  EXPECT_EQ(g[3], g[7]);
  EXPECT_NE(g[0], g[1]);                                   // 색 다름
  EXPECT_NE(g[0], g[2]);                                   // 모양 다름
  const auto g4 = nms_groups({"red_cyl", "green_cyl"}, 3);  // 이름 모자란 클래스는 혼자
  EXPECT_NE(g4[0], g4[1]);
  EXPECT_NE(g4[2], g4[0]);
  EXPECT_NE(g4[2], g4[1]);
}

// one2many 합성 출력: 앵커마다 상자 xywh · 클래스 하나만 점수 · 계수 1 (원형 1 → 상자 안 마스크)
static SegOutputs one2many(const std::vector<std::array<float, 4>> & box, const std::vector<int> & cls,
  const std::vector<float> & score, int nc)
{
  SegOutputs o;
  o.channels_first = true;
  o.n_anchor = static_cast<int>(box.size());
  o.nm = 1;
  o.n_ch = 4 + nc + o.nm;
  o.mh = o.mw = 16;
  o.det.assign(static_cast<size_t>(o.n_ch) * o.n_anchor, 0.0f);
  for (int a = 0; a < o.n_anchor; ++a) {
    for (int k = 0; k < 4; ++k) {o.det[static_cast<size_t>(k) * o.n_anchor + a] = box[a][k];}
    o.det[static_cast<size_t>(4 + cls[a]) * o.n_anchor + a] = score[a];
    o.det[static_cast<size_t>(4 + nc) * o.n_anchor + a] = 1.0f;
  }
  o.proto.assign(16 * 16, 1.0f);
  return o;
}

TEST(Segmenter, NmsRemovesUpLyingDuplicateOnly)
{
  const std::vector<std::string> n8 = {"red_cyl_up", "green_cyl_up", "red_box_up", "green_box_up",
    "red_cyl_lying", "green_cyl_lying", "red_box_lying", "green_box_lying"};
  std::vector<float> chw;
  const cv::Mat img(64, 64, CV_8UC3, cv::Scalar::all(0));
  const auto lb = letterbox(img, {64, 64}, chw);
  // 같은 자리 (IoU 0.8) red_cyl_up 0.9 · red_cyl_lying 0.8 → 하나 (점수 높은 up)
  auto o = one2many({{20, 20, 20, 20}, {20, 21, 20, 22}}, {0, 4}, {0.9f, 0.8f}, 8);
  auto inst = postprocess(o, lb, img.size(), n8, 0.5f, 0.7f);
  ASSERT_EQ(inst.size(), 1u);
  EXPECT_EQ(inst[0].kind, "red_cyl_up");
  // 같은 자리라도 색이 다르면 둘 다
  o = one2many({{20, 20, 20, 20}, {20, 21, 20, 22}}, {0, 5}, {0.9f, 0.8f}, 8);
  EXPECT_EQ(postprocess(o, lb, img.size(), n8, 0.5f, 0.7f).size(), 2u);
  // 같은 클래스라도 떨어진 두 퍽 (IoU 0) 은 둘 다
  o = one2many({{14, 20, 12, 12}, {44, 20, 12, 12}}, {0, 4}, {0.9f, 0.8f}, 8);
  EXPECT_EQ(postprocess(o, lb, img.size(), n8, 0.5f, 0.7f).size(), 2u);
}

TEST(Segmenter, ShapeOutputsHeads)
{
  SegOutputs o;
  shape_outputs(o, {1, 4 + 8 + 32, 6300}, {1, 32, 120, 160});   // one2many [1, C, A]
  EXPECT_TRUE(o.channels_first);
  EXPECT_EQ(o.n_anchor, 6300);
  EXPECT_EQ(o.n_ch, 44);
  EXPECT_EQ(o.mh, 120);
  shape_outputs(o, {1, 300, 44}, {1, 32, 120, 160});            // one2one [1, A, C]
  EXPECT_FALSE(o.channels_first);
  EXPECT_EQ(o.n_anchor, 300);
  EXPECT_THROW(shape_outputs(o, {1, 300}, {1, 32, 120, 160}), std::runtime_error);
}

TEST(Segmenter, LetterboxKeepsAspect)
{
  // 640x480 → 640x640: 배율 1, 위아래 80 px (ultralytics 와 같음)
  std::vector<float> chw;
  const auto lb = letterbox(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(0)), {640, 640}, chw);
  EXPECT_EQ(lb.new_w, 640);
  EXPECT_EQ(lb.new_h, 480);
  EXPECT_EQ(lb.pad_x, 0);
  EXPECT_EQ(lb.pad_y, 80);
  EXPECT_EQ(chw.size(), 3u * 640 * 640);
  EXPECT_FLOAT_EQ(chw[0], 114 / 255.0f);   // 여백 회색
}
