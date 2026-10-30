// YOLO 밖에서 하는 후처리 — ROS 없이 테스트할 수 있게 여기 모음.
//
// - 퍽 색: bbox 중앙 영역 HSV 중앙값
// - 신호등: bbox 를 세로로 나눠 칸별 밝기 비교
// - AprilTag: OpenCV aruco 의 36h11 사전 (OpenCV 4.6 contrib / 4.7+ objdetect 모두)
// - 1D 바코드: OpenCV barcode 모듈 + 전처리 단계별 재시도
// - 다수결: 최근 N 프레임 중 M 번
#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

// 원본 이미지 픽셀 좌표 상자 (왼쪽 위 x1, y1 / 오른쪽 아래 x2, y2)
struct Box
{
  float x1{0}, y1{0}, x2{0}, y2{0};
};

// YOLO 결과 상자 하나
struct Detection
{
  Box box;          // 원본 이미지 좌표
  float score{0};
  int cls{0};
};

// letterbox (ultralytics LetterBox(auto=False, center=True) 와 같음): 비율 유지 리사이즈 + 회색(114) 채움
// → 결과 이미지, 배율 r, 왼쪽 · 위 패딩
cv::Mat letterbox(const cv::Mat & img, cv::Size size, double & r, int & pad_x, int & pad_y);

// 클래스별 NMS (IoU 가 iou 를 넘는 낮은 점수 상자 제거)
std::vector<Detection> nms(std::vector<Detection> dets, float iou);

// ---------------------------------------------------------------- 퍽 색

struct ColorCfg
{
  int v_min{40};                       // 너무 어두운 픽셀(그림자) 제외
  int min_pixels{30};                  // 유효 픽셀이 이보다 적으면 unknown
  double s_beige_max{70};              // 채도 중앙값이 이보다 낮으면 무채색 계열 → beige 후보
  std::array<double, 2> beige_hue{5, 35};
  std::array<double, 2> red_hue{170, 10};   // lo > hi 이면 0 을 넘나드는 범위
  std::array<double, 2> green_hue{40, 85};
};

// bbox 의 가운데 frac 비율만 잘라 냄 (가장자리 배경 · 그림자 제외). 잘못된 상자면 빈 Mat
cv::Mat center_roi(const cv::Mat & img, const Box & box, double frac);

// 퍽 색 → ("red" | "green" | "beige" | "unknown", 유효 픽셀 수)
std::pair<std::string, int> classify_color(const cv::Mat & roi_bgr, const ColorCfg & cfg);

// ---------------------------------------------------------------- 신호등

struct Tier
{
  std::string name;   // "red" | "orange" | "green"
  double top, bottom; // bbox 높이 대비 위→아래 비율
};

// 타워램프 bbox → "red" | "orange" | "green" | "off". 렌즈가 꺼져도 색이 보이므로 색이 아니라 밝기로 판단
std::string signal_state(
  const cv::Mat & roi_bgr, const std::vector<Tier> & tiers, double v_margin,
  std::map<std::string, double> * score = nullptr);

// ---------------------------------------------------------------- 다수결

// 최근 window 개 중 같은 값이 need 번 이상이면 그 값을 확정
template<typename T>
class Voter
{
public:
  Voter(int window, int need)
  : window_(static_cast<size_t>(std::max(window, 1))), need_(need) {}

  // (확정값 또는 nullopt, 가장 많은 값의 비율)
  std::pair<std::optional<T>, double> push(const T & value)
  {
    buf_.push_back(value);
    while (buf_.size() > window_) {buf_.pop_front();}
    return result();
  }

  std::pair<std::optional<T>, double> result() const
  {
    if (buf_.empty()) {return {std::nullopt, 0.0};}
    // 가장 많은 값 (같으면 먼저 들어온 값)
    const T * best = nullptr;
    int best_n = 0;
    for (const auto & v : buf_) {
      const int n = count(v);
      if (n > best_n) {best_n = n; best = &v;}
    }
    const double ratio = static_cast<double>(best_n) / static_cast<double>(buf_.size());
    if (best_n >= need_) {return {*best, ratio};}
    return {std::nullopt, ratio};
  }

  int count(const T & value) const
  {
    int n = 0;
    for (const auto & v : buf_) {n += (v == value) ? 1 : 0;}
    return n;
  }

  void clear() {buf_.clear();}

private:
  size_t window_;
  int need_;
  std::deque<T> buf_;
};

// ---------------------------------------------------------------- AprilTag

// 퍽 bbox 안의 AprilTag ID 와 코너
class TagReader
{
public:
  explicit TagReader(const std::string & family = "36h11", int min_side_px = 60);
  ~TagReader();

  // → (tag_id, 원본 이미지 좌표의 코너 4개). 못 찾으면 (-1, 빈 벡터). 여러 개면 가장 큰 것
  std::pair<int, std::vector<cv::Point2f>> read(const cv::Mat & img_bgr, const Box & box, double pad = 0.15) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  int min_side_;
};

struct Pose
{
  cv::Vec3d t;      // 카메라 기준 위치 (m)
  cv::Matx33d R;    // 카메라 기준 회전
};

// 태그 코너 → 카메라 기준 자세. 태그 크기는 검은 테두리 바깥 한 변 (m). fisheye 는 왜곡을 먼저 풂
std::optional<Pose> tag_pose(
  const std::vector<cv::Point2f> & corners, double tag_size,
  const cv::Matx33d & K, const cv::Mat & D, bool fisheye);

// 정렬된 depth (m, CV_32F) 의 bbox 중앙 영역 → 카메라 기준 3D 점. 가까운 쪽 25% 분위수 (배경 피함)
std::optional<cv::Vec3d> depth_point(const cv::Mat & depth_m, const Box & box, const cv::Matx33d & K, double frac = 0.4);

// 3x3 회전행렬 → (x, y, z, w)
std::array<double, 4> rot_to_quat(const cv::Matx33d & R);

// ---------------------------------------------------------------- 1D 바코드

// OpenCV barcode 모듈 (EAN-8/13, UPC-A/E). 원본 → CLAHE → 샤프닝 → 이진화 순서로, 하나라도 읽히면 채택
class BarcodeReader
{
public:
  BarcodeReader();
  ~BarcodeReader();

  // → [(text, symbology), ...] (첫 번째로 성공한 전처리 결과).
  // 바코드가 화면을 크게 채우면 OpenCV 검출기가 오히려 못 찾음 → 줄여서도 시도
  std::vector<std::pair<std::string, std::string>> read(
    const cv::Mat & img_bgr, const std::vector<double> & scales = {1.0, 0.5}) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// numpy.percentile (linear) 과 같은 계산. 비어 있으면 0
double percentile(std::vector<double> v, double q);

}  // namespace perception
