// wrist_rack.hpp — 손목 D435 로 적재함 컵 바닥 AprilTag 를 읽어 칸 중심 · 방향을 구함
//
// 역할
//   - 컵 바닥 태그 36h11 찾기, 칸 ↔ ID 는 cell_tag_ids 표 (S1 … Sn 순서)
//   - 태그 PnP (IPPE_SQUARE 두 해 중 위쪽과 맞는 해) → 칸 바닥 중심 · 칸 프레임 (카메라 좌표)
//   - 비스듬히 본 장 · 재투영 오차 큰 장 거르기
//   - RackFilter: 칸마다 최근 window 장 중 min_hits 장 이상이면 중앙값으로 확정
// 입력
//   - detect_rack_tags: 회색 영상, 내부 파라미터, 왜곡 계수, 카메라 좌표 월드 위쪽 (없으면 nullptr)
//   - RackFilter::push: 로봇 프레임 칸 중심 · yaw
// 출력
//   - RackTag: 칸 번호 · 이름 · 카메라 좌표 칸 자세 · 모서리 · 재투영 오차 · 기운 각
//   - RackFilter::get: 확정된 칸 중심 · yaw · 본 장 수
// 실행
//   - perception_core 라이브러리, WristPipeline 이 씀. 시험 test_wrist_rack
// note
//   - 태그 붙인 방향 = 인쇄물 화살표(태그 위쪽)가 로봇 앞 +X → 칸 x = 태그 y, 칸 y = −태그 x, 칸 z = 태그 법선 (위)
//     → xacro rack_cell_<칸>_link 와 같은 축 (원점 = 칸 바닥 중심)
//   - 태그는 컵 안쪽 36 mm 에 꼭 맞게 잘라 붙임 → 태그 중심 = 칸 중심, 따로 보정 안 함
//   - 컵 벽 28 mm 가 바닥을 가려서 수직 위에서 약 8° 안쪽으로 봐야 태그가 다 보임
//   - 퍽 태그 (0–11) 와 ID 가 안 겹치게 20–31 을 씀. 실물은 위치 순서로 붙여서 S 번호와 순서가 다름 → 표로 맞춤

#pragma once

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/wrist_pose.hpp"

namespace perception::wrist
{

struct RackParams
{
  // S1 … S12 의 태그 ID (실물 2026-11-03 붙인 그대로). 표 안 ID 가 아니면 적재함 태그 아님
  std::vector<int> cell_tag_ids{20, 21, 22, 31, 26, 24, 29, 23, 30, 27, 25, 28};
  double tag_side_m{0.026};    // 검은 테두리 한 변
  double max_view_deg{25.0};   // 태그 법선 ↔ 카메라 시선 각도 한계
  double max_tilt_deg{10.0};   // 태그 법선 ↔ 월드 위쪽 각도 한계 (위쪽 알 때만)
  double max_reproj_px{1.5};   // 모서리 재투영 오차 RMS 한계
};

struct RackTag
{
  int tag_id{-1};
  int cell{0};                              // 1 … cell_tag_ids 개수
  std::string name;                         // "S1" …
  Mat3 R{Mat3::Identity()};                 // 칸 프레임 → 카메라 회전 (x = 로봇 앞, z = 위)
  Vec3 t{Vec3::Zero()};                     // 칸 바닥 중심 (카메라 좌표, m)
  std::array<cv::Point2f, 4> corners{};     // 화소 (aruco 순서)
  double reproj_px{0.0};
  double view_deg{0.0};                     // 법선 ↔ 시선
  double tilt_deg{0.0};                     // 법선 ↔ 위쪽 (위쪽 모르면 0)
};

// 태그 ID → 칸 번호 1 … n, 적재함 태그가 아니면 0
inline int rack_cell_of(int tag_id, const RackParams & p)
{
  for (size_t i = 0; i < p.cell_tag_ids.size(); ++i) {
    if (p.cell_tag_ids[i] == tag_id) {return static_cast<int>(i) + 1;}
  }
  return 0;
}

// 칸 번호 → 태그 ID, 없으면 −1
inline int rack_tag_of(int cell, const RackParams & p)
{
  return (cell >= 1 && cell <= static_cast<int>(p.cell_tag_ids.size())) ? p.cell_tag_ids[cell - 1] : -1;
}

inline std::string rack_cell_name(int cell) {return "S" + std::to_string(cell);}

// 회색 영상 → 적재함 태그 (거른 것은 뺌). rejected 를 주면 거른 태그 이유를 담음 (확인용)
std::vector<RackTag> detect_rack_tags(
  const cv::Mat & gray, const Intrinsics & K, const cv::Mat & dist, const RackParams & p,
  const Vec3 * up_cam, std::vector<std::pair<RackTag, std::string>> * rejected = nullptr);

// 칸 프레임 x 축을 로봇 평면에 눕힌 각도 (R = 칸 → 로봇 회전, 로봇 z = 위)
double planar_yaw(const Mat3 & R);

struct RackEstimate
{
  Vec3 position{Vec3::Zero()};   // 로봇 프레임 칸 바닥 중심 (중앙값)
  double yaw{0.0};               // rad
  int hits{0};                   // 최근 window 장 중 본 장 수
};

// 칸마다 최근 window 장 안에서 min_hits 장 이상 보이면 중앙값으로 확정
class RackFilter
{
public:
  RackFilter(int window = 5, int min_hits = 3)
  : window_(window < 1 ? 1 : window), min_hits_(min_hits) {}

  void next_frame() {++frame_;}   // 영상 한 장마다 push 전에 부름
  void push(int cell, const Vec3 & position, double yaw);
  std::optional<RackEstimate> get(int cell) const;
  std::vector<int> cells() const;   // 확정된 칸 번호 (오름차순)
  void reset() {buf_.clear();}
  int window() const {return window_;}

private:
  struct Sample
  {
    long frame;
    int cell;
    Vec3 p;
    double yaw;
  };
  int window_, min_hits_;
  long frame_{0};
  std::deque<Sample> buf_;
};

}  // namespace perception::wrist
