// wide_tag_reader.hpp — 광각 퍽 상자 안의 AprilTag ID 읽기
//
// 역할
//   - 퍽 상자를 pad 만큼 넓혀 자르고, 짧은 변이 min_side_px 보다 작으면 키워서 태그 검출
//   - 퍽 태그 ID 0–11 만 받고, 여러 개면 가장 큰 것
// 입력
//   - BGR 영상, 퍽 상자 Box, pad 기본 0.15
// 출력
//   - 퍽 태그 ID 와 원본 좌표 모서리 4 개, 못 찾으면 -1 과 빈 목록
// 실행
//   - perception_core 라이브러리, WidePipeline 이 wide.read_tags 가 true 일 때 부름. 시험 test_wide
// note
//   - 장소 태그 48 mm 는 퍽 태그 16 mm 보다 커서 같이 잡히면 뽑히기 쉬움 → 버림
//   - 검출은 common.hpp detect_apriltags, 손목과 같은 설정
//   - 16 mm 태그는 광각 1 m 거리에서 칸당 2 px 미만이라 거의 안 읽힘 → wide.read_tags 기본 false

#pragma once

#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/wide_detector.hpp"

namespace perception::wide
{

class PuckTagReader
{
public:
  // min_side_px: 잘라 낸 영역의 짧은 변이 이보다 작으면 키워서 검출
  explicit PuckTagReader(int min_side_px = 60)
  : min_side_(min_side_px) {}

  // → (퍽 태그 ID, 원본 이미지 좌표의 코너 4개). 못 찾으면 (-1, 빈 벡터)
  std::pair<int, std::vector<cv::Point2f>> read(const cv::Mat & img_bgr, const Box & box, double pad = 0.15) const;

private:
  int min_side_;
};

}  // namespace perception::wide
