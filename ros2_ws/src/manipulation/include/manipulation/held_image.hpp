// held_image.hpp — 쥔 퍽 손목 영상 잰 값 (perception /held_puck/image, 픽셀) ↔ 잘 쥔 기준: 앞뒤 mm · 기울기 판단 + 기준 파일 (ROS 없음)
// =====================================================================
// 입력: perception wrist.held 가 쥔 퍽 분할 마스크로 잰 픽셀 값 (wrist_held.hpp) — 위 모서리 y · 기울기, 왼쪽 (손가락 쪽) 모서리 x · 기울기, 아래 끝
// 출력: 기준 대비 손 안 앞뒤 [mm] (+ = pick_offset_x 를 + 로 준 것과 같은 쪽 = 손이 퍽 앞쪽을 쥠) · 기울기 [°] · 다시 쥘지
// 손목 카메라와 그리퍼는 같은 링크 → 쥔 퍽의 영상 자리는 손 안 위치로만 정해짐 (배경 · 조명만 바뀜). 기준은 같은 모양 (네모 · 원기둥) 끼리만 비교
// 11-11 16:4x 실물 (같은 초록 네모, 쥔 뒤 움직이는 중 1.2 s): pick_offset_x −0.5 → +4.5 (5 mm 앞) 위 모서리 +26 px, −5.5 (5 mm 뒤) −73 px
//   → 앞 5.2 px/mm · 뒤 14.7 px/mm (가까운 거리 원근 + 퍽 윗면이 보이기 시작해 비대칭). 다시 쥔 뒤 대각선 (+10 mm) 은 왼쪽 모서리 −30°
#pragma once
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace manipulation
{
struct HeldImageSample
{
  double stamp{0.0};       // 촬영 시각 [s]
  double top_y{0.0};       // 위 모서리 y [px] (클수록 아래 = 앞으로 쥠)
  double top_deg{0.0};     // 위 모서리 기울기 [°]
  double left_x{0.0};      // 왼쪽 모서리 x [px]
  double left_deg{0.0};    // 왼쪽 모서리 기울기 [°] (left_ok 일 때만)
  double bottom_y{0.0};    // 아래 끝 y [px] (영상 끝이면 479)
  bool left_ok{false};
  double conf{0.0};
  std::string label, shape;   // 분할 색 · 모양 (쥔 퍽은 모양 판정이 틀리기 쉬움, 로그만)
};

struct HeldImageRef
{
  bool ok{false};
  HeldImageSample s;      // 기준 (중앙값)
  int n{0};               // 모은 장 수
  std::string when;       // 저장 시각 (로그용)
};

struct HeldImageConfig
{
  double tol_mm{3.0};           // 앞으로 쥔 허용 [mm]
  double back_tol_mm{6.0};      // 뒤로 쥔 허용 [mm] (11-11 실물: 5 mm 뒤로 쥔 퍽은 S12 에 잘 들어감). 0 이하 = tol_mm
  double tilt_deg{5.0};         // 기울기 허용 [°]
  double px_per_mm_front{5.2};  // 위 모서리 픽셀 → 앞 mm
  double px_per_mm_back{14.7};  // 위 모서리 픽셀 → 뒤 mm
};

struct HeldImageVerdict
{
  bool judged{false};     // 기준 · 잰 값이 있어 판단함
  bool bad{false};        // 다시 쥘 것
  double front_mm{0.0};   // + = 앞으로 쥠, − = 뒤로 쥠
  double tilt_deg{0.0};   // 왼쪽 모서리 기울기 차 (못 재면 위 모서리 기울기 차)
  bool tilt_from_top{false};
  double d_top_px{0.0}, d_left_px{0.0}, d_top_deg{0.0};
  std::string text;       // 로그 한 줄 (판단 근거)
};

// 장마다 값의 중앙값 (left 는 left_ok 인 장만, 하나도 없으면 left_ok false). 빈 목록이면 기본값
HeldImageSample median_sample(const std::vector<HeldImageSample> & v);
// buf 중 촬영 시각 [from, to] 인 장
std::vector<HeldImageSample> samples_between(const std::deque<HeldImageSample> & buf, double from, double to);
// 기준과 비교. 기준 없음 · 잰 값 없음이면 judged false (text 에 이유)
HeldImageVerdict judge_held_image(const HeldImageSample & cur, int n, const HeldImageRef & ref, const HeldImageConfig & c);
// 기준 파일: 모양별 한 줄 "box top_y top_deg left_x left_deg bottom_y left_ok n when" (# 줄 = 설명)
bool save_held_image_refs(const std::string & path, const std::map<std::string, HeldImageRef> & refs, std::string & err);
std::map<std::string, HeldImageRef> load_held_image_refs(const std::string & path, std::string & err);
}  // namespace manipulation
