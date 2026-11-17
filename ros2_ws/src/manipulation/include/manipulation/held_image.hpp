// held_image.hpp — 쥔 퍽 손목 영상 잰 값 (perception /held_puck/image, 픽셀) ↔ 잘 쥔 기준: 앞뒤 mm · 기울기 판단 + 기준 파일 (ROS 없음)
// =====================================================================
// 입력: perception wrist.held 가 쥔 퍽 분할 마스크로 잰 픽셀 값 (wrist_held.hpp) — 위 모서리 y · 기울기, 왼쪽 (손가락 쪽) 모서리 x · 기울기, 아래 끝
// 출력: 기준 대비 손 안 앞뒤 [mm] (+ = pick_offset_x 를 + 로 준 것과 같은 쪽 = 손이 퍽 앞쪽을 쥠) · 기울기 [°] · 다시 쥘지
// 손목 카메라와 그리퍼는 같은 링크 → 쥔 퍽의 영상 자리는 손 안 위치로만 정해짐 (배경 · 조명만 바뀜). 기준은 같은 모양 (네모 · 원기둥) 끼리만 비교
// 위 모서리 화소 → mm 는 앞 · 뒤가 비대칭 (실측 앞 5.2 px/mm · 뒤 14.7 px/mm: 가까운 거리 원근 + 퍽 윗면이 보이기 시작함)
#pragma once
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"

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
  double back_tol_mm{6.0};      // 뒤로 쥔 허용 [mm]. 0 이하 = tol_mm
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
  bool tilt_bad{false};   // 기울기만 허용 넘음 여부 (앞뒤와 따로, held_image_tilt_regrasp)
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
// 기울기로 다시 쥘 모양 (held_image_regrasp_tilt_shapes — 기울기는 수평 옮김 보정으로 못 고침):
//   판단함 · 기울기 허용 넘음 · shape ("box" · "cylinder") 이 목록에 있으면 true → held_image_compensate 여도 박스에 내려놓고 다시 쥠.
//   앞뒤만 넘은 경우 · 목록에 없는 모양 = false
bool held_image_tilt_regrasp(const HeldImageVerdict & v, const std::string & shape, const std::vector<std::string> & shapes);
bool save_held_image_refs(const std::string & path, const std::map<std::string, HeldImageRef> & refs, std::string & err);
std::map<std::string, HeldImageRef> load_held_image_refs(const std::string & path, std::string & err);

// ---- 쥔 그대로 보정 (control_master held_image_compensate) ----
// 허용 밖으로 쥔 퍽을 내려놓고 다시 쥐지 않고, 잰 손 안 어긋남만큼 STORE 넣는 점 · PLACE 놓는 점을 옮김
// 손 좌표 (arm_kinematics tcp_hand_to_link5): x = 수직 손 · J5 기본에서 반지름 바깥 (pick_offset_x + 쪽), y = 닫힘축 (close5), z = 위
//   URDF 카메라 (camera_joint): 영상 아래 ≈ 손 −x (내적 −0.84), 영상 오른쪽 ≈ 손 −y (−1.00), 영상 왼쪽 = 왼쪽 손가락 (+close5)
//   앞으로 쥠 front_mm > 0 = 손이 퍽 앞쪽을 쥠 → 퍽 중심은 손끝보다 손 −x 로 front_mm (위 모서리가 영상 아래로)
//   왼쪽 모서리 d_left_px > 0 (영상 오른쪽으로) → 퍽은 손 −y 로 d_left_px / lateral_px_per_mm
struct HeldImageComp
{
  bool ok{false};
  double front_mm{0.0};      // judge_held_image 의 앞뒤 (+ = 앞으로 쥠)
  double lateral_mm{0.0};    // 퍽 중심이 손 +y (닫힘축, 왼쪽 손가락 쪽) 로 비킨 양 [mm] (안 쓰면 0)
  double lateral_px{0.0};    // 잰 왼쪽 모서리 차 [px]
  bool lateral_used{false};
  std::string lateral_note;  // 옆을 안 쓴 이유 (로그)
  double tilt_deg{0.0};      // 잰 기울기 (로그만 — 수평 옮김으로는 못 고침)
  Eigen::Vector3d hand_m{Eigen::Vector3d::Zero()};   // 손 좌표 퍽 중심 어긋남 [m] = (−front, lateral, 0)
};
// 판단 결과 → 손 안 어긋남. judged 아니면 ok false. lateral_px_per_mm ≤ 0 = 옆은 안 씀
//   옆은 왼쪽 모서리를 잰 장 (tilt_from_top 아님) 이고 |기울기| ≤ tilt_tol_deg 일 때만 (기울어 쥔 네모 퍽은 왼쪽 모서리가 돌아서 옮겨 보임)
HeldImageComp held_image_comp(const HeldImageVerdict & v, double lateral_px_per_mm, double tilt_tol_deg);
// 놓는 자세 손목 회전 R (arm_frames(...)[4], link5 → base) 에서 손끝 (넣는 점 · 놓는 점) 을 옮길 양 [m, base xy]
//   = −(R · 손 좌표 어긋남).xy → 퍽 중심이 원래 손끝 자리 (칸 · 목표) 에 옴. STORE held_correct (태그) 와 같은 식 (ox −= dworld)
Eigen::Vector2d held_image_comp_shift(const ArmModel & m, const Eigen::Matrix3d & R, const Eigen::Vector3d & hand_m);
}  // namespace manipulation
