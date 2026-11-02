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
// 실행
//   - perception_barcode 라이브러리, perception_node 안에서 돎. 시험 test_barcode
// note
//   - 기본 꺼짐. planning 이 READ_BARCODE 단계에서 켜고 끝나면 끔. 바코드는 출발 구역에 하나
//   - 켜져 있는 동안 계속 발행 → planning 은 단계 시작 뒤 온 것만 씀
//   - 켜고 끌 때 다수결을 비움
//   - zxing-cpp libzxing-dev 가 있으면 Code128 포함 1D 전부 읽음
//   - 없으면 OpenCV barcode 모듈 = EAN · UPC 만 → 벽 바코드 Code128 은 못 읽음, 빌드 때 경고

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "interfaces/msg/task_text.hpp"
#include "perception/common.hpp"
#include "perception/wide_detector.hpp"

namespace perception
{

class BarcodeReader
{
public:
  BarcodeReader();
  ~BarcodeReader();

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
  void set_enabled(bool on) {enabled_ = on; ++reset_req_;}
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
};

}  // namespace perception
