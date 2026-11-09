// traffic_light.hpp — 신호등 상자 한 장에서 켜진 칸 판정
//
// 역할
//   - 상자를 세로 칸으로 나누고 칸별 흰색 비율 · 밝기를 비교해 켜진 칸을 고름
//   - 1) 흰색 비율: 흰 화소(V ≥ white_v · S ≤ white_s) 비율이 가장 높은 칸이 white_min 이상이고
//        다른 칸보다 white_margin 넘게 높으면 그 칸
//   - 2) 밝기: 1) 로 못 정하면 칸 밝기(V 90 분위수)가 가장 높은 칸이 다른 칸보다 v_margin 넘게 밝으면 그 칸
//   - 둘 다 아니면 off
// 입력
//   - 신호등 상자 영역 BGR
//   - 칸 목록 TrafficLightTier: 이름 · 상자 높이 대비 위아래 비율
//   - 판정 기준 TrafficLightRule
// 출력
//   - "red" · "yellow" · "green" · "off", 영역이 비면 "unknown"
//   - score 를 넘기면 칸별 밝기 · 흰색 비율
// 실행
//   - perception_core 라이브러리, WidePipeline 이 부름. 시험 test_wide
// note
//   - 칸마다 가운데 절반 열만 봄 (가장자리 반사 제외)
//   - 꺼진 렌즈도 색이 보여서 색 이름이 아니라 흰색 · 밝기로 판단함
//   - 흰색 비율을 먼저 보는 이유: 실내 자동 노출에서 꺼진 렌즈까지 V 255 로 포화 → 밝기 차이 0 (실측).
//     켜진 칸 가운데는 색 없이 하얗게 찍히고 (S ≈ 0), 꺼진 렌즈는 색이 진함 (S ≈ 150)
//   - 신호등 위 흰 뚜껑도 흰색이라 칸 비율에서 빼야 함 (traffic_light.red · yellow · green)
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

struct TrafficLightRule
{
  double v_margin{40.0};      // 밝기: 가장 밝은 칸 − 다음 칸 > 이 값
  double white_min{0.30};     // 흰색: 켜진 칸 흰색 비율 최소
  double white_margin{0.20};  // 흰색: 가장 높은 칸 − 다음 칸 > 이 값
  double white_v{240.0};      // 흰 화소: V ≥ 이 값
  double white_s{60.0};       //          S ≤ 이 값
};

struct TrafficLightScore
{
  double v{0.0};      // V 90 분위수
  double white{0.0};  // 흰 화소 비율 0~1
};

// → 칸 이름 또는 "off". roi 가 비면 "unknown"
std::string classify_traffic_light(
  const cv::Mat & roi_bgr, const std::vector<TrafficLightTier> & tiers, const TrafficLightRule & rule,
  std::map<std::string, TrafficLightScore> * score = nullptr);

}  // namespace perception
