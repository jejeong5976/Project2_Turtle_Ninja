// test_wrist_seg.cpp — 손목 퍽 분할 전후처리 시험, 합성 출력으로 TensorRT 없이
//   실행: colcon build → source install/setup.bash → ./build/perception/test_wrist_seg

#include <gtest/gtest.h>

#include <algorithm>
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
