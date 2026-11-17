// wrist_tag_stage.hpp — 손목 컬러 영상 → AprilTag ID · 모서리 (/perception/wrist/tags), perception 노드 안 자기 콜백 그룹
//
// 역할
//   - 켜져 있을 때만 (~/wrist_tags/enable) wrist_tags.max_hz 로 솎아 태그를 읽음. 영상 구독은 시작 때 한 번만 만들고
//       꺼진 동안은 콜백 첫 줄에서 버림 (켜고 끌 때마다 구독을 만들고 지우면 드물게 노드 전체가 멈췄음 → 안 함)
//   - 본 검출: CLAHE + AprilTag 3 (wrist_tag_reader), 왜곡 푼 정규 좌표까지
//   - 영역 검출 (wrist_tags.roi): 칸 (TF rack_cell_*: 바닥 태그 · 칸 안 퍽 윗면) · 쥔 퍽 (TF end_effector_link 둘레) 을 촬영 시각 TF 로
//       투영한 상자만 검출 (전체 영상 Jetson 장당 약 36 ms). TF 없음 · 영역이 화면 roi_max_frac 넘음 · full_every 장마다 = 전체 영상,
//       전체 영상 장에서 영역 밖에 나온 태그를 셈 (로그 '영역 밖') → 0 이 아니면 영역 크기 · TF 확인
//   - 쥔 퍽 다시 찾기: 본 검출에 카메라 held_max_dist_m 안 퍽 태그 (wrist_tags.puck_ids) 가 없으면 원본 영상 OpenCV aruco 로 한 번 더,
//       그 거리 안 퍽 태그 첫 하나만 rescue = true 로 붙임 (쥔 퍽은 화면 끝에 걸쳐 흰 테두리가 잘리면 AprilTag 3 가 놓침)
// 받는 것
//   - wrist.color_topic · wrist.info_topic (WristPipeline 이 선언한 값 그대로)
// 주는 것
//   - /perception/wrist/tags  interfaces/WristTagArray, 처리한 장마다 (태그 없으면 빈 배열), frame_id = wrist.frame_id
// note
//   - 적재함 칸 · 쥔 퍽 위치 계산은 control_master (manipulation rack_tag_stage) 가 이 토픽으로 함
//   - 손목 퍽 그룹과 다른 그룹 → 태그 검출이 퍽 처리를 막지 않음

#pragma once

#include <atomic>
#include <memory>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>

#include "interfaces/msg/wrist_tag_array.hpp"
#include "perception/wrist_tag_reader.hpp"

namespace perception
{

class WristTagStage
{
public:
  WristTagStage(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group, std::shared_ptr<tf2_ros::Buffer> tf);
  void set_enabled(bool on);         // 플래그만 바꿈 (구독은 그대로)

private:
  using Image = sensor_msgs::msg::Image;
  void on_image(const Image::ConstSharedPtr & msg);
  // 촬영 시각 TF 로 칸 · 쥔 퍽 영역 (화면 화소). TF 를 못 얻으면 false. held = 쥔 퍽 영역 (없으면 빈 상자)
  bool make_rois(const builtin_interfaces::msg::Time & stamp, const cv::Matx33d & K, const cv::Size & img,
    std::vector<cv::Rect> & rois, cv::Rect & held);

  // 영역 검출 설정 (wrist_tags.roi*)
  std::shared_ptr<tf2_ros::Buffer> tf_;
  bool roi_on_{true};
  std::string base_frame_, held_frame_;
  std::vector<std::string> cell_frames_;
  double floor_z_{0.1325}, cell_half_m_{0.03}, cell_top_m_{0.075}, held_half_m_{0.05}, roi_pad_px_{24.0};
  double roi_max_frac_{0.5}, tf_timeout_s_{0.05};
  int full_every_{10};
  std::vector<cv::Point3d> cells_base_;               // 칸 중심 (base_frame, 고정 TF 를 처음 한 번 읽음)
  int64_t seq_{0};
  int roi_runs_{0}, full_runs_{0}, outside_{0}, checked_{0};

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::string color_topic_, frame_id_;
  rclcpp::QoS img_qos_{2};
  double max_hz_{5.0}, puck_tag_m_{0.0153}, held_max_m_{0.15};
  std::set<int> puck_ids_;
  std::unique_ptr<wrist::TagReader> reader_;
  std::atomic<bool> enabled_{false};                  // 서비스 스레드가 씀 ↔ 콜백이 읽음
  std::atomic<unsigned> on_gen_{0}, seen_gen_{0};     // 켤 때마다 +1 → 콜백이 솎기 기준 (have_last_) 을 비움
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Publisher<interfaces::msg::WristTagArray>::SharedPtr pub_;
  bool have_last_{false};
  rclcpp::Time last_;
  double ms_sum_{0.0};
  int runs_{0};
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
};

}  // namespace perception
