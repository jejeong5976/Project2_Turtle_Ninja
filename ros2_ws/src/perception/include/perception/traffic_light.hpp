// traffic_light.hpp — 신호등 상자 한 장에서 켜진 칸 판정
//
// 역할
//   - 상자를 세로 칸으로 나누고 칸별 밝기를 비교해 켜진 칸을 고름
//   - 칸 밝기 = 가운데 절반 열의 V 채널 90 분위수
//   - 가장 밝은 칸이 나머지보다 v_margin 넘게 밝으면 그 칸, 아니면 off
// 입력
//   - 신호등 상자 영역 BGR
//   - 칸 목록 TrafficLightTier: 이름 · 상자 높이 대비 위아래 비율
//   - v_margin
// 출력
//   - "red" · "yellow" · "green" · "off", 영역이 비면 "unknown"
//   - score 를 넘기면 칸별 밝기
// 실행
//   - perception_core 라이브러리, WidePipeline 이 부름. 시험 test_wide
// note
//   - 꺼진 렌즈도 색이 보여서 색이 아니라 밝기로 판단함
//   - 여러 장 안정화는 WidePipeline 의 Voter 가 함

#pragma once

#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct TrafficLightTier
{
  std::string name;    // "red" | "yellow" | "green"
  double top, bottom;  // 상자 높이 대비 위→아래 비율
};

// → 칸 이름 또는 "off". roi 가 비면 "unknown"
std::string classify_traffic_light(
  const cv::Mat & roi_bgr, const std::vector<TrafficLightTier> & tiers, double v_margin,
  std::map<std::string, double> * score = nullptr);

}  // namespace perception
