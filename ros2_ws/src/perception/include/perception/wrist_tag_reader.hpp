// wrist_tag_reader.hpp — 손목 컬러 영상 한 장에서 AprilTag (tag36h11) ID · 모서리 읽기, ROS 없음
//
// 역할
//   - TagReader::detect: CLAHE (대비를 조각마다 올림) + AprilTag 3 (빌드에 없으면 OpenCV aruco)
//       컵 벽 그림자로 칸 바닥 태그가 어둡게 찍힘 → 실물 스캔 73 장: 그대로 OpenCV 는 12 칸 중 2 칸, CLAHE + AprilTag 3 는 12 칸
//   - TagReader::detect_aruco: 원본 영상 OpenCV aruco (쥔 퍽 태그 다시 찾기 · AprilTag 3 없을 때)
//   - undistort_corners: 화소 → 왜곡 푼 정규 좌표 (camera_info 의 ROI · 비닝을 풀고 전체 해상도 K · D)
//   - tag_near: 정규 좌표 모서리 + 태그 크기 → PnP 로 카메라에서 max_dist 안인지 (쥔 퍽 태그 판정)
//   - TagReader::detect_in · project_box · merge_rois: 태그가 있을 영역만 검출 (전체 영상 AprilTag 3 = Jetson 장당 약 30 ms)
// 받는 것
//   - 회색 영상 (mono8), 카메라 모델 (camera_info 값)
// 주는 것
//   - 태그마다 ID · 모서리 4 개 (좌상 · 우상 · 우하 · 좌하 = aruco 순서)
// note
//   - 위치 계산 (적재함 칸 평면 · 쥔 퍽 손목 좌표) 은 control_master (manipulation rack_tag_stage) 가 함

#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace perception::wrist
{

// camera_info 중 왜곡 풀기에 쓰는 값
struct TagCamera
{
  cv::Matx33d K;
  std::vector<double> D;
  std::string model;                 // plumb_bob · rational_polynomial · equidistant
  float binning_x = 1.0f, binning_y = 1.0f;
  float roi_x = 0.0f, roi_y = 0.0f;
};

// 왜곡 푼 정규 좌표 (x/z, y/z). 모델을 모르면 false (why 에 이유)
bool undistort_corners(const TagCamera & cam, const std::vector<cv::Point2f> & px, std::vector<cv::Point2f> & out,
  std::string * why = nullptr);

// 정규 좌표 모서리 4 개 · 태그 한 변 [m] → 태그 중심이 카메라에서 max_dist 안이면 true
//   모서리 순서는 검출기마다 (AprilTag 반시계 · OpenCV aruco 시계) 달라서 두 순서 중 재투영 오차가 작은 쪽 (control rack_tag_stage held_tag 와 같은 식)
bool tag_near(const std::vector<cv::Point2f> & norm, double size, double max_dist);

// 카메라 광학 좌표 점들 → 핀홀 투영으로 감싼 상자 + pad [px]. z < min_z (카메라 뒤 · 너무 가까움) 점은 뺌, 남은 점이 없으면 빈 상자
//   왜곡은 안 셈 (D435 컬러는 왜곡이 작음) → pad 로 덮음
cv::Rect project_box(const std::vector<cv::Point3d> & cam_pts, const cv::Matx33d & K, double pad_px, double min_z = 0.03);

// 영역 정리: 화면에 자르고 빈 것은 빼고 겹치는 것은 합침. 합친 넓이가 화면의 max_frac 을 넘으면 false (= 전체 영상이 나음)
bool merge_rois(std::vector<cv::Rect> & rois, const cv::Size & img, double max_frac);

class TagReader
{
public:
  struct Params
  {
    int max_hamming = 1;             // AprilTag 3: 고친 비트가 이보다 많으면 버림
    double min_margin = 30.0;        // AprilTag 3: 판정 여유 (decision_margin) 이보다 작으면 버림
    int nthreads = 2;
  };
  explicit TagReader(const Params & p);
  ~TagReader();
  TagReader(const TagReader &) = delete;
  TagReader & operator=(const TagReader &) = delete;

  // CLAHE 뒤 AprilTag 3 (없으면 aruco). corners 는 aruco 순서
  void detect(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids);
  // 영역마다 잘라 같은 검출 (CLAHE 조각 크기는 전체 영상 8 × 8 조각과 비슷하게), 모서리는 전체 영상 화소. 같은 ID 는 처음 것만
  void detect_in(const cv::Mat & gray, const std::vector<cv::Rect> & rois, std::vector<std::vector<cv::Point2f>> & corners,
    std::vector<int> & ids);
  static void detect_aruco(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids);
  static bool have_apriltag();       // AprilTag 3 로 빌드됐는지
  static const char * name();

private:
  Params p_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Ptr<cv::CLAHE> clahe_roi_;     // 영역용 (조각 수를 영역 크기에 맞춤)
  void detect_eq(const cv::Mat & eq, const cv::Point2f & off, std::vector<std::vector<cv::Point2f>> & corners,
    std::vector<int> & ids);         // 대비 올린 영상 → AprilTag 3 (없으면 aruco), 모서리에 off 더함
  void * fam_ = nullptr;             // apriltag_family_t (AprilTag 3 빌드일 때만)
  void * det_ = nullptr;             // apriltag_detector_t
};

}  // namespace perception::wrist
