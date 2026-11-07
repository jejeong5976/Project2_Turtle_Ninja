// wrist_pipeline.hpp — 손목 D435 퍽 처리, perception 노드 안 손목 콜백 그룹에서 돎
//
// 역할
//   - 컬러 · 깊이를 시각 맞춰 묶고 한 장씩 처리
//       퍽 분할 → 퍽마다 wrist_pose 자세 → 거르기 → PuckTracker 확정 → Detection
//   - 촬영 시각 TF 로 카메라 좌표의 월드 위쪽을 구함
//   - 카메라가 wrist.reset_move_m 또는 wrist.reset_rot_deg 넘게 움직이면 묶음을 지움
//   - barcode.camera 가 wrist 이고 바코드가 켜져 있으면 컬러 영상을 BarcodeStage 에 넘김, 깊이 · TF 와 무관
// 입력
//   - wrist.color_topic  sensor_msgs/Image, 기본 /camera/camera/color/image_raw
//   - wrist.depth_topic  sensor_msgs/Image, 기본 /camera/camera/aligned_depth_to_color/image_raw, 16UC1 mm 또는 32FC1 m
//   - wrist.info_topic   sensor_msgs/CameraInfo, 기본 /camera/camera/color/camera_info
//   - TF wrist.world_frame ← wrist.frame_id, 기본 odom ← d435_color_optical_frame
// 출력
//   - /perception/wrist/detections  interfaces/DetectionArray, 처리한 영상마다, 못 찾으면 빈 배열
//       label · shape · id, pose = 퍽 중심 + 긴축이 z 인 방향, yaw_known, bbox
//   - /perception/wrist_debug_image  sensor_msgs/Image, wrist.publish_debug 일 때만
// 실행
//   - perception_node 가 만듦. ~/wrist/enable 로 켜고 끔
// note
//   - header.stamp = 촬영 시각, frame_id = wrist.frame_id = URDF 이름, 드라이버 이름 아님
//   - TF 를 못 구하면 발행 안 함. 위쪽 없이 내면 긴축이 틀림 → planning 은 검출 끊김으로 앎
//   - 태그는 퍽 ID 0–11 만 쓰고, 방향은 중력 + 점군으로 구함
//   - 엔진을 못 읽으면 분할은 멈추고 바코드 판독만 함
//   - 영상 QoS 기본 reliable, realsense2_camera 기본 발행과 맞음

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "perception/wrist_pose.hpp"
#include "perception/wrist_seg.hpp"
#include "perception/wrist_tracker.hpp"

namespace perception
{

class BarcodeStage;

class WristPipeline
{
public:
  WristPipeline(
    rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
    rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub, BarcodeStage * barcode);

  void set_enabled(bool on) {enabled_ = on; ++reset_req_;}
  bool enabled() const {return enabled_;}
  void request_reset() {++reset_req_;}

private:
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;

  void on_frame(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg);
  void publish_debug(
    const Image::ConstSharedPtr & color, const cv::Mat & bgr, const wrist::Intrinsics & K,
    const std::vector<wrist::RawPuck> & raws, const std::vector<wrist::PuckDetection> & dets);

  rclcpp::Node & node_;
  BarcodeStage * barcode_;   // barcode.camera = wrist 일 때만
  std::unique_ptr<wrist::SegModel> seg_;
  wrist::PuckTracker tracker_;

  std::string frame_id_, world_frame_;
  double tf_timeout_s_{0.05}, reset_move_m_{0.01}, reset_rot_deg_{2.0}, depth_scale_{0.001};
  double min_rise_m_{0.0};      // 납작한 무늬 거르기 (wrist.min_rise_m)
  bool publish_debug_{false};
  std::optional<wrist::Mat3> last_R_;
  wrist::Vec3 last_t_{wrist::Vec3::Zero()};
  std::atomic<bool> enabled_{true};
  std::atomic<uint32_t> reset_req_{0};
  uint32_t reset_done_{0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;   // 손목 콜백 그룹 안에서만 읽고 씀
  message_filters::Subscriber<Image> color_sub_, depth_sub_;
  std::shared_ptr<Sync> sync_;

  int n_frames_{0};
  double sum_seg_ms_{0.0}, sum_all_ms_{0.0};
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
};

}  // namespace perception
