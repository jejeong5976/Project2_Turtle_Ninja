// wrist_held.hpp — 손에 쥔 퍽을 손목 컬러 분할 마스크로 잼 (깊이 안 씀, ROS 없음)
//
// 역할
//   - 손목 카메라와 그리퍼는 같은 링크 → 쥔 퍽의 영상 속 자리는 팔 자세와 상관없이 "손 안 위치" 로만 정해짐
//   - 분할 결과 중 쥔 퍽 (영상 오른쪽 아래 ROI 안, 오른쪽 끝에 닿는 큰 마스크) 하나를 골라
//     퍽 먼 끝 모서리 (마스크 위 모서리) 높이 · 각도, 손가락 쪽 모서리 (마스크 왼쪽, 손가락 위만) 자리 · 각도, 아래 끝을 픽셀로
// 입력
//   - SegInstance 목록 (wrist_seg.hpp, 신뢰도 문턱은 held.conf 로 낮춰 받음 — 쥔 퍽은 가까워 0.2–0.9)
// 출력
//   - HeldMeasure (픽셀 · 도). 기준 (잘 쥔 상태) 과의 차 · mm 환산은 control_master (held_image_*) 가 함
// note
//   - 11-11 실물 9 장 (held_imgs): 5 mm 앞으로 쥠 → 위 모서리 +26 px, 5 mm 뒤 → −73 px, 대각선 (다시 쥔 뒤 +10 mm) → 왼쪽 모서리 −30°
//   - 검정 손가락이 y ≈ 290 아래에서 퍽 왼쪽을 가림 → 왼쪽 모서리는 finger_y 위 줄만 (아래 줄을 넣으면 각도가 10° 넘게 틀어짐)
//   - 깊이는 173 mm 보다 가까우면 안 나와 쥔 퍽 (60–130 mm) 에는 못 씀

#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/wrist_seg.hpp"

namespace perception::wrist
{

struct HeldParams
{
  float conf{0.2f};          // 쥔 퍽 후보 분할 신뢰도 문턱 (일반 검출 wrist.conf 와 따로)
  int roi_x0{300};           // 이 왼쪽 · 위는 안 봄 (쥔 퍽은 영상 오른쪽 아래)
  int roi_y0{60};
  int min_area_px{8000};     // ROI 안 마스크 넓이 최소 (쥔 퍽 3–5 만 px)
  int right_touch_px{200};   // 영상 오른쪽 10 px 띠에 닿은 화소 수 최소 (쥔 퍽은 늘 오른쪽 끝까지)
  int finger_y{290};         // 이 줄부터 아래는 손가락이 퍽 왼쪽을 가림 → 왼쪽 모서리에 안 씀
  int top_ref_x{520};        // 위 모서리 높이를 재는 열
  int left_rows{90};         // 위 모서리 아래 몇 줄까지 왼쪽 모서리로 (8 줄 띄고)
  int min_rows{15};          // 왼쪽 모서리 · 위 모서리 맞춤 최소 줄 · 열 수
  int bottom_x0{560};        // 아래 끝을 보는 열 범위
  int bottom_x1{630};
};

struct HeldMeasure
{
  bool ok{false};
  std::string why;           // ok 아닐 때 이유
  std::string kind;          // 분할 클래스 (red_box_up 등 — 쥔 퍽은 누움 · 선 판정이 틀리기 쉬움, 색만 믿을 것)
  float conf{0.0f};
  double top_y{std::numeric_limits<double>::quiet_NaN()};     // 위 모서리 높이 [px] (top_ref_x 열), 클수록 아래
  double top_deg{std::numeric_limits<double>::quiet_NaN()};   // 위 모서리 기울기 [°] (영상 x → y, + = 오른쪽이 아래)
  double left_x{std::numeric_limits<double>::quiet_NaN()};    // 왼쪽 모서리 자리 [px] (위 모서리 + 40 줄)
  double left_deg{std::numeric_limits<double>::quiet_NaN()};  // 왼쪽 모서리 기울기 [°] (영상 y → x, + = 아래가 오른쪽). 손가락 위 줄이 모자라면 NaN
  double bottom_y{std::numeric_limits<double>::quiet_NaN()};  // 아래 끝 [px] (bottom 열 범위, 영상 끝에 닿으면 영상 높이)
  int area{0};
  int top_cols{0}, left_rows{0};
  std::array<int, 4> bbox{{0, 0, 0, 0}};                      // x0 y0 x1 y1
};

// 쥔 퍽 후보 고르기: 신뢰도 ≥ p.conf · ROI 안 넓이 ≥ min_area · 오른쪽 끝에 닿음 중 가장 넓은 것. 없으면 -1
int pick_held(const std::vector<SegInstance> & inst, const HeldParams & p);

// 마스크 (원본 크기, 0 · 255) 하나 → 잰 값. ROI 밖은 버리고 잼
HeldMeasure measure_held(const cv::Mat1b & mask, const HeldParams & p);

// 분할 목록 → 쥔 퍽 잰 값 (pick_held + measure_held, kind · conf 채움). 후보가 없으면 ok false
HeldMeasure measure_held(const std::vector<SegInstance> & inst, const HeldParams & p);

}  // namespace perception::wrist
