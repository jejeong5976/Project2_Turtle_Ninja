// wrist_tag_rescue.hpp — 손목 퍽 태그 다시 찾기: 분할로 퍽은 잡았는데 태그 ID 를 못 붙인 퍽만
//
// 역할
//   - 1차 검출 (detect_tags_in: 분할 상자 + wrist.tag_roi_pad_px, 겹친 상자는 합쳐 한 번, OpenCV aruco) 에서
//     태그를 못 받은 퍽마다 넓힌 상자 (마스크 상자 + pad_px) 를 퍽마다 따로 (합치지 않음) 다시 검출
//       0) 1차 검출에서 나왔지만 어느 퍽에도 안 붙은 태그 (상자 ±5 px 밖) 를 먼저 마스크로 붙여 봄 (검출 안 함)
//       1) AprilTag 3 원본 (빌드에 있으면. manipulation rack_tag_node 와 같은 36h11 · quad_decimate 1 · hamming 1)
//       2) AprilTag 3 CLAHE (rack_tag_node 처럼 대비 올림, 그늘진 태그)
//       3) OpenCV aruco 원본 (1차와 같은 36h11 사전 · common.hpp 설정, 넓힌 상자 하나씩)
//       4) aruco CLAHE (AprilTag 3 가 빌드에 없을 때만: 합친 상자에서 건너뛴 CLAHE 를 퍽마다)
//       퍽마다 그 퍽 태그를 찾으면 그 단계에서 멈춤 (합성 시험: 1) 이 가장 잘 읽고 싸서 먼저)
//   - 붙이는 규칙 (옆 퍽 태그를 잘못 붙이지 않게)
//       태그 중심이 어느 퍽 마스크 안 (여유 없이) 이면 그 퍽 것. 여럿이면 마스크 상자 중심이 가까운 퍽
//       어느 마스크에도 없으면 margin_px 안에 마스크가 있는 퍽 중 가장 가까운 퍽, 거리가 같은 퍽이 둘이면 안 붙임
//       그 퍽이 이미 태그 · ID 가 있는 퍽 (need false) 이면 안 붙임 (그 퍽의 다른 면 태그)
//       이미 이번 장 다른 퍽에 붙은 ID · 다른 퍽 묶음이 가진 ID (own_id) 는 안 붙임
//       태그 하나는 퍽 하나에만, 퍽 하나에 태그 하나만
//   - 처리 시간: 이번 장 다시 찾기가 budget_ms 를 넘으면 남은 퍽은 다음 장으로 (최소 1 개는 함)
//     RescueCooldown: 못 찾은 퍽 자리는 retry_frames 장 쉼 (태그가 안 보이는 퍽이 장마다 검출을 먹지 않게)
//     찾은 퍽은 묶음이 ID 를 기억하는 동안 (window 장) 다시 안 찾음 → planning 퍽 기록부는 ID 한 번이면 남음
// 입력
//   - 회색 영상 (원본 크기), 퍽마다 마스크 (원본 크기, 0 아님 = 퍽) · 마스크 상자 · 다시 찾을지 · 이미 가진 ID
//   - 1차 검출 태그 (붙었든 안 붙었든 전부)
// 출력
//   - 퍽마다 붙일 태그 (need 인 퍽만, 못 찾으면 없음). 모서리 원본 좌표 aruco 순서, IPPE 두 해 (wrist_pose TagPose 와 같음)
//   - RescueStats: 넓힌 상자 수 · 검출기별 실행 수 · 붙인 수 · 퍽마다 찾은 방법 (처리 시간 로그 · 확인 그림용)
// 실행
//   - perception_core 라이브러리, WristPipeline 이 부름 (wrist.tag_rescue). 시험 test_wrist_tag_rescue
// note
//   - 순수 C++, OpenCV (+ AprilTag 3 있으면). ROS 없음
//   - AprilTag 3 는 PERCEPTION_HAVE_APRILTAG (CMake 가 apriltag 패키지 찾으면 켬) 일 때만. 없으면 aruco 만
//   - AprilTag 3 모서리는 OpenCV 화소 규약으로 −0.5 px 옮김 (시험에서 참값 투영과 비교)
//   - TagRescuer 하나는 한 번에 한 스레드만 (손목 콜백 그룹 = 상호 배타). AprilTag 3 검출기를 안에 하나 가짐
//   - 넓힌 상자 하나 ≈ 150×150 px → 전체 영상 (640×480) 의 약 7 %. 태그 없는 퍽이 있을 때만 돎

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/wrist_pose.hpp"
#include "perception/wrist_seg.hpp"
#include "perception/wrist_tracker.hpp"

namespace perception::wrist
{

struct TagRescueParams
{
  int pad_px{48};                 // 넓힌 상자 = 마스크 상자 + 이만큼 (1차 wrist.tag_roi_pad_px 24 보다 넓게)
  int margin_px{8};               // 태그 중심이 마스크에서 이만큼 안이면 그 퍽 것으로 봄
  bool apriltag3{true};           // AprilTag 3 를 씀 (빌드에 있을 때만 효과)
  int at3_max_hamming{1};         // AprilTag 3: 고친 비트가 이보다 많으면 버림 (rack_tag_node max_hamming 과 같음)
  double at3_min_margin{20.0};    // AprilTag 3: decision_margin 이 이보다 작으면 버림 (rack_tag_node 30 은 26 mm 칸 태그용.
                                  //   16 mm 퍽 태그 합성 시험: 30 → 20 에서 +3 %, 틀린 ID 0)
  bool aruco{true};               // OpenCV aruco 원본 (AprilTag 3 없으면 CLAHE 판도) 도 씀
  bool clahe{true};               // CLAHE 단계 (2 · 4) 도 씀
  double budget_ms{10.0};         // 이번 장 다시 찾기가 이만큼 넘으면 남은 퍽은 다음 장 (0 = 제한 없음)
};

// 다시 찾기 대상 퍽 하나 (이번 장 퍽 전부를 넘김: 붙이는 규칙에 옆 퍽 마스크가 필요)
struct RescuePuck
{
  cv::Mat1b mask;                 // 원본 크기, 0 아님 = 퍽 (SegInstance::mask, 복사 없이 머리만)
  cv::Rect box;                   // 마스크 상자
  bool need{false};               // true = 이번 장 태그 없음 · 묶음 ID 도 없음 → 다시 찾음
  bool rest{false};               // need 지만 얼마 전 못 찾아 쉼: 넓힌 상자 검출은 안 하고, 1차 · 옆 퍽 상자에서 나온 태그만 받음
  int own_id{-1};                 // 이미 가진 ID (이번 장 태그 또는 묶음 최빈 ID), 없으면 -1
};

struct RescueStats
{
  int need{0};                    // 다시 찾을 퍽 수 (쉬는 퍽 포함)
  int resting{0};                 // 그중 못 찾은 지 얼마 안 돼 넓힌 상자 검출을 쉰 퍽 수 (RescueCooldown)
  int rois{0};                    // 넓힌 상자로 검출한 퍽 수
  int over_budget{0};             // 시간 넘어 다음 장으로 넘긴 퍽 수
  int at3_runs{0};                // AprilTag 3 실행 수
  int aruco_runs{0};              // aruco 실행 수
  int from_primary{0};            // 1차 검출에서 안 붙은 태그를 붙인 수
  int found{0};                   // 붙인 태그 수 (from_primary 포함)
  std::vector<std::string> how;   // 퍽마다 찾은 방법 ("primary" · "at3" · "at3_clahe" · "aruco" · "aruco_clahe" · "")
  std::vector<uint8_t> tried;     // 퍽마다 넓힌 상자 검출을 했는지
  std::vector<cv::Rect> roi_rects;   // 넓힌 상자 (확인 그림용)
};

// 검출 한 번의 결과 태그 (퍽 태그 ID 0–11 만), 모서리 원본 좌표 aruco 순서 (좌상 · 우상 · 우하 · 좌하)
struct RawTag
{
  int id{-1};
  std::array<cv::Point2f, 4> corners{};
  cv::Point2f centre() const;
};

// AprilTag 3 가 빌드에 들어갔는지
bool apriltag3_available();

// 태그 중심 → 주인 퍽 번호 (위 규칙, need 와 무관하게 모든 퍽 중), 없거나 애매하면 -1
int tag_owner(const cv::Point2f & centre, const std::vector<RescuePuck> & pucks, int margin_px);

class TagRescuer
{
public:
  explicit TagRescuer(const TagRescueParams & p = TagRescueParams());
  ~TagRescuer();
  TagRescuer(const TagRescuer &) = delete;
  TagRescuer & operator=(const TagRescuer &) = delete;

  const TagRescueParams & params() const {return p_;}

  // 넓힌 상자 하나에서 퍽 태그 검출, 한 단계 (method) 만: "at3" · "at3_clahe" · "aruco" · "aruco_clahe"
  //   AprilTag 3 가 빌드에 없으면 at3* 는 빈 결과. roi 는 영상 안으로 자름
  std::vector<RawTag> detect_once(const cv::Mat & gray, const cv::Rect & roi, const std::string & method);

  // 다시 찾기 전체: 결과[i] = pucks[i] 에 붙일 태그 (need 인 퍽만, 못 찾으면 nullopt)
  //   primary = 1차 검출 태그 전부 (detect_tags_in 결과). 그중 어느 퍽에도 안 붙은 것 (own_id 에 없는 ID) 을 먼저 씀
  std::vector<std::optional<TagPose>> rescue(
    const cv::Mat & gray, const Intrinsics & K, const std::vector<RescuePuck> & pucks,
    const std::vector<TagPose> & primary, RescueStats * stats = nullptr, const cv::Mat & dist = cv::Mat());

  // 실행할 단계 순서 (설정 · 빌드 반영)
  std::vector<std::string> methods() const;

private:
  struct At3;
  TagRescueParams p_;
  std::unique_ptr<At3> at3_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat equalize(const cv::Mat & sub);   // rack_tag_node 와 비슷한 조각 크기 (약 80 × 60 px) 로 CLAHE
};

// 못 찾은 퍽 자리 (카메라 좌표) 를 기억해 retry_frames 장 동안 그 자리 퍽은 다시 찾지 않음. 카메라가 움직이면 reset
class RescueCooldown
{
public:
  explicit RescueCooldown(int retry_frames = 4, double match_m = 0.02)
  : retry_frames_(retry_frames), match_m_(match_m) {}
  void reset() {failed_.clear();}
  void next_frame() {++frame_;}
  bool resting(const Vec3 & c) const;
  void failed(const Vec3 & c);

private:
  int retry_frames_;
  double match_m_;
  long frame_{0};
  std::vector<std::pair<Vec3, long>> failed_;   // (자리, 다시 해도 되는 장 번호)
};

// 손목 한 장 처리 중 (puck_pose · reject_reason 끝난 뒤, tracker.update 전) 태그 다시 찾기
//   inst = 분할 결과 전부, raw_of[k] = inst[k] 의 raws 번호 (puck_pose 가 없으면 −1)
//   다시 찾는 퍽: 이번 장 태그 없음 + 묶음 (tracker.known_id_near) ID 도 없음 + (통과 또는 truncated 로만 걸림) + 쉬는 중 아님
//     truncated 퍽은 태그를 찾으면 태그로 중심을 잡아 (tag_top_refine) 통과할 수 있음 → 퍽 인지가 늘기만 함
//   찾으면 그 퍽만 puck_pose 를 그 태그 하나로 다시 돌림 (1차에서 찾았을 때와 같은 자세 · 거르기), 붙인 퍽 수를 돌려줌
//   다시 찾을 퍽이 없으면 검출을 안 함 (stats->rois = 0). cooldown 이 nullptr 이면 쉬지 않음
int rescue_raw_pucks(
  TagRescuer & rescuer, const cv::Mat & gray, const cv::Mat1f & depth_m, const Intrinsics & K, const Vec3 & up_cam,
  const std::vector<SegInstance> & inst, const std::vector<int> & raw_of, std::vector<RawPuck> & raws,
  const std::vector<TagPose> & primary, const PuckTracker & tracker, double min_rise_m, bool tag_centre,
  RescueCooldown * cooldown = nullptr, RescueStats * stats = nullptr, PoseTiming * timing = nullptr);

}  // namespace perception::wrist
