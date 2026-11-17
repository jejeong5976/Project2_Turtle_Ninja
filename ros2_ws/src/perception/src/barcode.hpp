// barcode.hpp — 벽 1D 바코드 판독기 BarcodeReader + 노드 단계 BarcodeStage
//
// 역할
//   - BarcodeReader: 영상 한 장 → 바코드 문자열
//       여러 크기 scales × 전처리 원본 · CLAHE 등으로 다시 시도, 하나라도 읽히면 채택
//   - BarcodeStage: barcode.camera 카메라 콜백 안에서, 켜져 있을 때만 barcode.max_hz 마다 한 장 판독
//       최근 5 장 중 3 장이 같고 이번 장도 그 값이면 발행
//       못 읽은 장도 빈 값으로 다수결에 넣음 → 바코드가 안 보이면 확정이 풀림
//       광각이면 같은 영상의 barcode 상자를 확대해 먼저 읽고, 못 읽으면 전체 화면
// 입력
//   - 카메라 BGR 영상 + header, 광각이면 barcode 상자 목록. WidePipeline · WristPipeline 이 넘김
//   - 서비스 ~/barcode/enable std_srvs/SetBool, ~/barcode/reset std_srvs/Trigger. perception_node 가 연결
// 출력
//   - /perception/task_text  interfaces/TaskText
//       raw_text = 바코드 원문, header = 촬영 시각 + URDF frame_id, pose = 모름
//   - /perception/barcode/detections  interfaces/DetectionArray (barcode.detections_topic, "" = 안 냄)
//       켜져 있는 동안 판독한 장마다 (못 찾아도 빈 배열). kind "barcode", label = 읽은 원문 (못 읽고 자리만 찾으면 ""),
//       bbox = 영상 화소 상자, confidence 1 = 읽음 · 0.5 = 자리만 (barcode.locate: OpenCV 바코드 검출기, 있을 때만), pose NaN
//       pose.position = 손목 깊이 (barcode.depth_topic, 켜져 있을 때만 구독) 로 잰 상자 가운데 3D 점 (광학 프레임), 못 재면 NaN
//       + kind "wall" 하나: 화면 가운데 깊이 점 (벽 자리)
//       mission 바코드 찾기가 화면 가운데로 다시 맞추고 크기 (화소/모듈) 를 잼
// 실행
//   - perception_barcode 라이브러리, perception_node 안에서 돎. 시험 test_barcode
// note
//   - 기본 꺼짐. planning 이 READ_BARCODE 단계에서 켜고 끝나면 끔. 바코드는 출발 구역에 하나
//   - 켜져 있는 동안 계속 발행 → planning 은 단계 시작 뒤 온 것만 씀
//   - 켜고 끌 때 다수결을 비움
//   - zxing-cpp libzxing-dev 가 있으면 Code128 포함 1D 전부 읽음
//   - 없으면 OpenCV barcode 모듈 = EAN · UPC 만 → 벽 바코드 Code128 은 못 읽음, 빌드 때 경고

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/task_text.hpp"
#include "perception/common.hpp"
#include "perception/wide_detector.hpp"

namespace perception
{

// 읽은 (또는 자리만 찾은) 바코드 하나: bbox = 원본 영상 화소 [x1, y1, x2, y2]
struct BarcodeFound
{
  std::string text, format;   // 자리만이면 text ""
  std::array<float, 4> bbox{{0.f, 0.f, 0.f, 0.f}};
};

// 흰 라벨 자리 찾기 (cv::barcode 가 못 찾을 때): 회색 벽 위 밝고 (V) 색 없는 (S) 사각 · 가로세로 비 · 크기 (깊이로) · 막대 무늬
struct LabelParams
{
  double s_max{70.0};          // HSV S 이하
  double v_min{110.0};         // V 이상 (Otsu 값과 큰 쪽)
  double aspect_min{1.8}, aspect_max{5.5};
  double min_area_px{300.0};
  double width_m{0.078};       // 라벨 긴 변 (실물 7.8 × 2.7 cm)
  double size_lo{0.5}, size_hi{1.7};   // 깊이로 본 기대 화소 폭에 대한 비 범위
  double min_changes{12.0};    // 긴 축 따라 밝기 부호 바뀜 (막대 무늬) 최소
};
struct LabelBox
{
  std::array<float, 4> bbox{{0.f, 0.f, 0.f, 0.f}};
  double score{0.0};           // 막대 무늬 (부호 바뀜 평균)
};
// depth_at(u, v) = 그 화소 근처 깊이 [m] (모르면 ≤ 0), fx = 컬러 초점 [px]
std::vector<LabelBox> locate_labels(const cv::Mat & img_bgr, const LabelParams & p, double fx,
  const std::function<double(double, double)> & depth_at);

// OpenCV 바코드 검출기로 자리만 찾기 (읽기 아님). 검출기가 없는 빌드면 늘 빈 목록
std::vector<std::array<float, 4>> locate_barcodes(const cv::Mat & img_bgr);
bool barcode_locator_available();

class BarcodeReader
{
public:
  BarcodeReader();
  ~BarcodeReader();

  // read 와 같은 차례로 읽되 자리 (원본 화소 상자) 도 줌
  std::vector<BarcodeFound> read_located(const cv::Mat & img_bgr, const std::vector<double> & scales = {1.0, 0.5}) const;

  // → [(text, symbology), ...] (첫 번째로 성공한 전처리 결과).
  // 바코드가 화면을 크게 채우면 OpenCV 검출기가 오히려 못 찾음 → 줄여서도 시도
  std::vector<std::pair<std::string, std::string>> read(
    const cv::Mat & img_bgr, const std::vector<double> & scales = {1.0, 0.5}) const;

  static const char * backend();   // "zxing-cpp" | "opencv (EAN/UPC 만)"

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class BarcodeStage
{
public:
  explicit BarcodeStage(rclcpp::Node & node);

  const std::string & camera() const {return camera_;}          // "wide" | "wrist"
  const std::string & roi_class() const {return roi_class_;}    // 광각 상자 중 먼저 볼 클래스

  // 서비스 (관리 그룹) 가 부름. 실제 다수결 비우기는 다음 process 에서
  void set_enabled(bool on);   // 손목 깊이 구독도 켬 · 끔
  void request_reset() {++reset_req_;}

  // 켜져 있고 max_hz 차례인가 (카메라 콜백이 영상 변환 전에 물어봄)
  bool due() const;
  // 한 장 판독 → 확정이면 발행. header = 촬영 시각 + URDF frame_id
  void process(const cv::Mat & bgr, const std_msgs::msg::Header & header, const std::vector<wide::Box> & boxes = {});

private:
  using Clock = std::chrono::steady_clock;
  cv::Mat crop(const cv::Mat & img, const wide::Box & b) const;

  rclcpp::Node & node_;
  std::string camera_, roi_class_, last_text_;
  double period_s_{0.1}, roi_pad_{0.3}, roi_scale_{2.0};
  bool full_frame_{true};
  BarcodeReader reader_;
  Voter<std::pair<std::string, std::string>> voter_;
  Clock::time_point last_{};
  std::atomic<bool> enabled_{false};
  std::atomic<uint32_t> reset_req_{0};
  uint32_t reset_done_{0};
  rclcpp::Publisher<interfaces::msg::TaskText>::SharedPtr pub_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub_;   // 자리 출력 (없으면 안 냄)
  bool locate_{true};
  bool label_locate_{true};    // 흰 라벨 자리 찾기
  // 사진: barcode.snapshot_topic 으로 받은 경로에 다음 판독 장을 jpg 로 (미션이 LOOK_AT 머묾 끝마다)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr snap_sub_;
  std::mutex snap_mtx_;
  std::string snap_path_;
  LabelParams label_;
  // 손목 깊이
  bool depth_point(const std::array<float, 4> & bb, geometry_msgs::msg::Point & out) const;
  std::string depth_topic_, info_topic_;
  double depth_scale_{0.001};
  mutable std::mutex depth_mtx_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
};

}  // namespace perception
