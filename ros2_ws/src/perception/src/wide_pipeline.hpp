// wide_pipeline.hpp — 광각 카메라 처리, perception 노드 안 광각 콜백 그룹에서 돎
//
// 역할
//   - 영상을 wide.max_hz 이하로 골라 상자 검출 → 퍽 · 신호등 발행
//   - 신호등: 점수가 가장 높은 traffic_light 상자의 켜진 칸 판정 → Voter 로 안정화
//   - barcode.camera 가 wide 이고 바코드가 켜져 있으면 같은 영상 + barcode 상자를 BarcodeStage 에 넘김
// 입력
//   - wide.image_topic  sensor_msgs/Image, 기본 /wide_cam/image_raw, Arducam UC-684 1280×720 bgr8
// 출력
//   - /perception/wide/detections  interfaces/DetectionArray
//       퍽만, 처리한 영상마다, 못 찾아도 빈 배열
//       label = 색, shape = cylinder · box, pose = 모름, id = "" ← wide.read_tags 기본 false
//   - /perception/traffic_light  interfaces/TrafficLight, traffic_light.enabled 일 때 처리한 영상마다
//       state = RED · YELLOW · GREEN · OFF · UNKNOWN, visible
//   - /perception/wide_debug_image  sensor_msgs/Image, wide.publish_debug 일 때만 · 구독자 있을 때만
//       검출 상자 전부 + 신호등 칸 경계 · 칸별 밝기(V 90 분위) · 흰색 비율 · 이번 장 판정 · 확정 상태
// 실행
//   - perception_node 가 만듦. ~/wide/enable 로 켜고 끔
// note
//   - header.stamp = 촬영 시각, frame_id = wide.frame_id 기본 birdeye_camera_optical_frame
//   - 이번 영상에서 신호등 상자를 못 찾으면 이전 색을 내지 않고 UNKNOWN + visible=false
//   - 신호등을 traffic_light.timeout 1 s 넘게 못 보면 다수결을 비움
//   - 광각 퍽은 깊이가 없어 위치가 NaN → planning 은 안 받고 RViz · 디버깅용으로만 씀
//   - 엔진을 못 읽으면 검출 · 신호등은 멈추고 바코드 판독만 함
//   - 영상 QoS 기본 reliable depth 1. 1280×720 영상을 best_effort 로 받으면 조각 손실로 대부분 버려짐

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "perception/common.hpp"
#include "perception/traffic_light.hpp"
#include "perception/wide_detector.hpp"
#include "perception/wide_tag_reader.hpp"

namespace perception
{

class BarcodeStage;

class WidePipeline
{
public:
  WidePipeline(
    rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub, BarcodeStage * barcode);
  ~WidePipeline();

  void set_enabled(bool on) {enabled_ = on; ++reset_req_;}
  bool enabled() const {return enabled_;}
  void request_reset() {++reset_req_;}

private:
  using Clock = std::chrono::steady_clock;

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  // 상자 검출 → 퍽 · 신호등 발행. 같은 영상의 barcode 상자는 barcode_boxes 로
  void detect_and_publish(
    const cv::Mat & img, const std_msgs::msg::Header & header, Clock::time_point now, std::vector<wide::Box> & barcode_boxes);
  // 신호등 확인용 값: 상자 · 이번 장 판정 · 칸별 밝기
  struct LightDebug
  {
    cv::Rect roi;
    std::string label;
    std::map<std::string, TrafficLightScore> score;
  };
  // → 발행한 state. dbg 를 넘기면 이번 장 값을 채움
  uint8_t publish_traffic_light(
    const cv::Mat & img, const std_msgs::msg::Header & header, const wide::BoxDetection * best, Clock::time_point now,
    LightDebug * dbg);
  void publish_debug(
    const cv::Mat & img, const std_msgs::msg::Header & header, const std::vector<wide::BoxDetection> & dets,
    const LightDebug & light, uint8_t light_state);

  rclcpp::Node & node_;
  BarcodeStage * barcode_;   // barcode.camera = wide 일 때만
  std::unique_ptr<wide::BoxDetector> detector_;

  // 설정
  std::string frame_id_, light_class_;
  double conf_{0.5}, iou_{0.7}, max_hz_{30.0};
  bool read_tags_{true}, light_enabled_{true}, publish_debug_{false};
  std::set<std::string> puck_classes_;
  std::vector<TrafficLightTier> tiers_;
  TrafficLightRule light_rule_;
  double light_timeout_{1.0};

  // 상태 (광각 콜백 그룹 안에서만 바뀜)
  wide::PuckTagReader tag_reader_;
  Voter<std::string> light_vote_;
  Clock::time_point last_{}, last_light_seen_{};
  std::atomic<bool> enabled_{true};
  std::atomic<uint32_t> reset_req_{0};
  uint32_t reset_done_{0};

  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<interfaces::msg::TrafficLight>::SharedPtr light_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace perception
