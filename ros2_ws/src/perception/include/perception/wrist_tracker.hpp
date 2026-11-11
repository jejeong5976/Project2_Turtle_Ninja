// wrist_tracker.hpp — 손목 퍽 거르기 · 여러 장 확인 · Detection 값 만들기
//
// 역할
//   - reject_reason: 못 믿을 퍽 거르기
//       truncated = 화면 끝에 잘림
//       points = 유효 깊이 점 < 50
//       size = 점군 5–95 % 범위 긴 방향 > 8.5 cm 또는 두 번째 > 5 cm, 배경 · 반사 · 붙은 두 퍽
//   - PuckTracker::update: 여러 장을 묶어 확정
//       카메라 좌표 중심 2 cm 안이면 같은 퍽, 가까운 것부터 일대일
//       최근 5 장 중 3 장 이상 보이면 확정
//       누운 퍽 · 위쪽 모름은 긴축 부호를 이전 장에 맞춤
//       확정값 = 선/누움 다수결로 고른 장들의 중심 중앙값 · 긴축 평균 · 면 방향 평균,
//                색 · 모양은 신뢰도 가중 다수결, 태그 ID 는 최빈값
//   - planar_pose: 로봇 프레임 집기 자세 x · y · z · θ + 선/누움
// 입력
//   - 한 장의 RawPuck 목록: seg 클래스 · 신뢰도 · wrist_pose 자세
//   - planar_pose: 촬영 시각 TF 로봇 ← 카메라 회전 · 이동
// 출력
//   - PuckDetection 목록 = interfaces/Detection 필드, 카메라 광학 프레임
//   - orientation z 축 = 긴축, yaw_known 이면 x 축 = 사각기둥 면 방향
//   - PlanarPose: 로봇 프레임 퍽 중심 · θ · 선/누움
// 실행
//   - perception_core 라이브러리, WristPipeline 이 부름. 시험 test_wrist_pose
// note
//   - 카메라 좌표 기준이라 손목이 멈춘 동안만 맞음. 팔이 움직이면 reset()
//   - planar_pose 는 로봇 프레임 z 가 위쪽이라고 가정
//   - 순수 C++, Eigen 만 씀

#pragma once

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "perception/wrist_pose.hpp"

namespace perception::wrist
{

constexpr double MAX_LONG_SPREAD = 0.085;
constexpr double MAX_MID_SPREAD = 0.050;
constexpr int MIN_POINTS = 50;

struct KindInfo
{
  std::string color;                        // red / green
  std::string shape;                        // cylinder / box / "" (모름)
  std::optional<bool> lying;                // 8클래스 seg 이름의 선/누움 (4클래스 · 이름 없음이면 없음)
};

// seg 클래스 이름 → 색 · 모양 · 누움. 'red_cyl', 'red_cyl_up', 'green_box_lying' 받음. 빈 이름이면 fallback 색
KindInfo parse_kind(const std::string & kind, const std::string & fallback_color = "");

struct RawPuck
{
  PuckPose pose;
  KindInfo info;
  std::string kind;                         // seg 클래스 이름 그대로
  double conf{1.0};
  std::string reject;                       // "" = 통과
};

// 거르기 이유 ("" = 통과)
std::string reject_reason(const PuckPose & p);

struct PuckDetection
{
  std::string label, shape, id;             // id = 태그 ID 문자열, 못 읽으면 ""
  double confidence{0.0};
  Vec3 position{Vec3::Zero()};              // 카메라 광학 프레임 m
  std::array<double, 4> orientation{0, 0, 0, 1};   // xyzw, z 축 = 긴축
  std::array<int, 4> bbox{};
  bool yaw_known{false};                    // x 축 = 사각기둥 면 법선 (선 = 윗면 윤곽, 누움 = 위쪽)
  int hits{0};
  std::string axis_source;
  std::optional<bool> upright;
  bool center_from_tag{false};              // 중심 = 태그로 잰 장들의 중앙값 (tag_top_refine · tag_lying_refine)
};

// 긴축 = z 인 회전행렬 (열 x, y, z). face_dir 있으면 x 축 = 면 방향, 없으면 카메라 x 를 긴축에 수직으로 누름
Mat3 axis_frame(const Vec3 & axis, const std::optional<Vec3> & face_dir);

// 회전행렬 → 쿼터니언 xyzw (w ≥ 0)
std::array<double, 4> quat_xyzw(const Mat3 & R);
Mat3 quat_to_mat(const std::array<double, 4> & q);

class PuckTracker
{
public:
  explicit PuckTracker(int window = 5, int min_hits = 3, double match_m = 0.02);

  // 카메라(팔)가 움직인 뒤 호출
  void reset();

  // 이번 장 원시 퍽(거르기 끝난 것 포함) → 이번 장에 보인 확정 퍽
  std::vector<PuckDetection> update(const std::vector<RawPuck> & raws);

  // update 전에: center (카메라 좌표) 에서 match_m 안 가장 가까운 묶음이 다음 장 기준 최근 window 장 안에 본 태그 ID 최빈값,
  //   없으면 -1 (태그 다시 찾기를 할지 · 옆 퍽 ID 를 안 뺏게)
  int known_id_near(const Vec3 & center) const;

  int frame() const {return frame_;}
  size_t n_tracks() const {return tracks_.size();}

private:
  struct Track
  {
    std::deque<std::pair<int, RawPuck>> obs;   // (장 번호, 관측), 최근 16 개
    int last_frame{-1};
  };
  PuckDetection summarize(const std::vector<const RawPuck *> & recent) const;

  int window_, min_hits_;
  double match_m_;
  int frame_{-1};
  std::vector<Track> tracks_;
};

// 로봇 프레임 집기 자세
struct PlanarPose
{
  double x{0}, y{0}, z{0};                  // 퍽 중심 (m)
  double theta{0};                          // 선 사각기둥: 면 방향 [-π/4, π/4), 누운 퍽: 긴축 방향 [-π/2, π/2)
  bool upright{true};
  bool theta_known{false};                  // false = 선 원기둥 · 면 방향 못 구한 선 사각기둥
};

// R_bc, t_bc = 카메라 광학 프레임 → 로봇 프레임 (촬영 시각 TF)
PlanarPose planar_pose(const PuckDetection & d, const Mat3 & R_bc, const Vec3 & t_bc);

}  // namespace perception::wrist
