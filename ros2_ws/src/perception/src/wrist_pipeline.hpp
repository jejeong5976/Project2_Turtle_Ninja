// wrist_pipeline.hpp — 손목 D435 퍽 처리, perception 노드 안 손목 콜백 그룹에서 돎
//
// 역할
//   - 컬러 · 깊이를 시각 맞춰 묶고 한 장씩 처리
//       퍽 분할 → 퍽마다 wrist_pose 자세 → 거르기 → PuckTracker 확정 → Detection
//   - 촬영 시각 TF 로 카메라 좌표의 월드 위쪽을 구함
//   - 카메라가 wrist.reset_move_m 또는 wrist.reset_rot_deg 넘게 움직이면 묶음을 지움
//   - barcode.camera 가 wrist 이고 바코드가 켜져 있으면 컬러 영상을 BarcodeStage 에 넘김, 깊이 · TF 와 무관
//   - 적재함: 컬러 영상에서 컵 바닥 태그 (ID 20–31, 칸 ↔ ID 는 rack.cell_tag_ids) → 칸 바닥 중심 · yaw (rack.frame 기준), 깊이 안 씀
//   - 박스 윗면: 깊이 점군을 box.frame (바닥 = 0, z = 위) 으로 → 높이로 나눔 → 테두리 → 크기 아는 정사각형 맞춤 (wrist_box.hpp)
//       → 중심 · yaw. 장소 · 시작 박스 앞 LOOK 에서 씀, 퍽 분할 엔진과 무관
// 입력
//   - wrist.color_topic  sensor_msgs/Image, 기본 /camera/camera/color/image_raw
//   - wrist.depth_topic  sensor_msgs/Image, 기본 /camera/camera/aligned_depth_to_color/image_raw, 16UC1 mm 또는 32FC1 m
//   - wrist.info_topic   sensor_msgs/CameraInfo, 기본 /camera/camera/color/camera_info
//   - TF wrist.world_frame ← wrist.frame_id, 기본 odom ← d435_color_optical_frame
// 출력
//   - /perception/wrist/detections  interfaces/DetectionArray, 처리한 영상마다, 못 찾으면 빈 배열
//       label · shape · id, pose = 퍽 중심 + 긴축이 z 인 방향, yaw_known, bbox
//   - /perception/wrist_debug_image  sensor_msgs/Image, wrist.publish_debug 일 때만
//   - /perception/rack/cells  interfaces/DetectionArray, 적재함 처리한 영상마다, 확정된 칸 없으면 빈 배열
//       frame_id = rack.frame (기본 base_link), kind "rack_cell", label "S1"…, id = 태그 ID,
//       pose = 칸 바닥 중심 + yaw 만 (x 축 = 칸 앞), confidence = 최근 rack.window 장 중 본 비율
//   - /perception/rack_debug_image  sensor_msgs/Image, rack.publish_debug 일 때만
//   - /perception/wrist/box  interfaces/DetectionArray, 박스 처리한 영상마다, 못 찾으면 빈 배열
//       frame_id = box.frame (기본 base_footprint), kind "place_box", pose = 윗면 중심 (z = box.top_z) + yaw 만,
//       confidence = 맞춤 잔차가 작을수록 1 에 가까움
// 실행
//   - perception_node 가 만듦. ~/wrist/enable 로 켜고 끔, 적재함은 ~/rack/enable, 박스는 ~/box/enable 로 따로
// note
//   - header.stamp = 촬영 시각, frame_id = wrist.frame_id = URDF 이름, 드라이버 이름 아님
//   - TF 를 못 구하면 발행 안 함. 위쪽 없이 내면 긴축이 틀림 → planning 은 검출 끊김으로 앎
//   - 태그는 퍽 ID 0–11 만 쓰고, 방향은 중력 + 점군으로 구함
//   - 엔진을 못 읽으면 분할은 멈추고 바코드 판독만 함
//   - 영상 QoS 기본 reliable, realsense2_camera 기본 발행과 맞음
//   - 적재함은 카메라가 멈춰 있을 때만 셈 (장 사이 rack.still_move_m · rack.still_rot_deg 넘게 움직이면 묶음 지움)
//   - 적재함 칸 위치는 rack.frame ← 카메라 TF (팔 관절값 순기구학) 를 거침 → 팔 FK 오차가 그대로 들어감
//   - rack.compare_nominal 이면 URDF rack_cell_<칸>_link 와 차이를 로그 · 그림에 냄 (확인용, 발행 값엔 안 씀)
//   - 박스: 처음 추정 = box.guess (box.frame 기준 정차 자리에서 본 박스 중심 · yaw), 박스가 이 근처 ±45° · 수 cm 안이어야 맞음
//     자기 몸은 box.min_x 보다 가까운 점을 버려서 뺌. 팔 · 그리퍼는 윗면보다 높아 저절로 빠짐

#pragma once

#include <array>
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
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "perception/wrist_box.hpp"
#include "perception/wrist_pose.hpp"
#include "perception/wrist_rack.hpp"
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
  void set_rack_enabled(bool on) {rack_enabled_ = on; ++rack_reset_req_;}
  void set_box_enabled(bool on) {box_enabled_ = on;}
  void request_reset() {++reset_req_;}

private:
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;

  void on_frame(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg);
  bool on_rack(const Image::ConstSharedPtr & color);   // 이번 장을 처리했으면 true (처리 시간 기록용)
  void on_box(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg);
  void report_timing();   // 5 s 마다 입력 Hz · 단계별 처리 시간 로그
  bool to_depth_m(const Image::ConstSharedPtr & depth_msg, cv::Mat1f & depth);   // 16UC1 mm · 32FC1 m → m
  void publish_rack_debug(
    const Image::ConstSharedPtr & color, const cv::Mat & gray, const std::vector<wrist::RackTag> & tags,
    const std::vector<std::pair<wrist::RackTag, std::string>> & rejected, const std::vector<std::string> & lines);
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
  // wrist.tag_centre: 장마다 읽음 (실행 중 ros2 param set 으로 바꿈). false = 퍽 중심 · 면 방향을 모양으로만 (태그가 비뚤게 붙은 퍽)
  bool tag_roi_{true};          // 태그를 분할 상자 둘레만 검출 (wrist.tag_roi)
  int tag_roi_pad_px_{24};      // 상자 둘레 여유 [px] (wrist.tag_roi_pad_px)
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
  // 손목 카메라가 손목에서 돈 만큼 (manipulation camera_drift_node, 홈 사진 비교): 실제 카메라 = 명목 (TF) · drift_R_
  //   박스 윗면 · 적재함 칸 (로봇 프레임으로 냄) 은 TF 에 곱해 바로잡고, 퍽 검출은 위쪽 방향만 (위치는 카메라 좌표로 내고 control_master 가 되돌림)
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr drift_sub_;
  wrist::Mat3 drift_R_{wrist::Mat3::Identity()};   // 손목 콜백 그룹 안에서만 읽고 씀
  bool drift_correct_{true};
  wrist::Mat3 camera_rot(const wrist::Mat3 & R_tf) const;   // TF 회전 · drift_R_ (보정 꺼짐 · 다른 프레임이면 TF 그대로)
  std::string drift_frame_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;   // 손목 콜백 그룹 안에서만 읽고 씀
  message_filters::Subscriber<Image> color_sub_, depth_sub_;
  std::shared_ptr<Sync> sync_;

  // 적재함
  wrist::RackParams rack_p_;
  wrist::RackFilter rack_filter_;
  std::string rack_frame_;
  double rack_min_period_s_{0.1}, still_move_m_{0.002}, still_rot_deg_{0.5};
  bool rack_publish_debug_{false}, rack_compare_nominal_{true};
  std::optional<wrist::Mat3> rack_last_R_;
  wrist::Vec3 rack_last_t_{wrist::Vec3::Zero()};
  std::optional<rclcpp::Time> rack_last_stamp_;
  std::atomic<bool> rack_enabled_{false};
  std::atomic<uint32_t> rack_reset_req_{0};
  uint32_t rack_reset_done_{0};
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr rack_pub_;
  rclcpp::Publisher<Image>::SharedPtr rack_debug_pub_;

  // 박스 윗면
  wrist::BoxParams box_p_;
  wrist::Vec3 box_guess_{0.2935, 0.0, 0.0};
  std::string box_frame_;
  double box_min_period_s_{0.2};
  std::optional<rclcpp::Time> box_last_stamp_;
  std::atomic<bool> box_enabled_{true};
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr box_pub_;

  // 처리 시간 (5 s 마다 평균 로그): 단계별 합 [ms] + 들어온 장 수 (입력 Hz 확인용)
  struct Timing
  {
    int frames = 0;                 // on_frame 이 분할까지 간 장
    int pucks = 0;                  // 자세 계산한 퍽 수 (퍽당 시간용)
    int box_runs = 0;               // on_box 가 실제로 계산한 횟수
    int tag_runs = 0;               // 태그 검출한 장 (분할이 있는 장만)
    int rack_runs = 0;              // on_rack 이 실제로 처리한 장
    double conv = 0, tf = 0, seg = 0, tags = 0, pose = 0, track = 0, pub = 0, all = 0, box = 0, sync_cb = 0, rack = 0;
    perception::wrist::PoseTiming pose_parts;   // 자세 단계별 (퍽 전체 합)
  };
  std::vector<double> lat_ms_, pair_dt_ms_;   // 짝마다 촬영 → 처리 시작 · 컬러 − 깊이 시각 차 [ms], 보고 때 비움
  Timing tm_;
  std::array<size_t, 3> last_counts_{{0, 0, 0}};   // 마지막 장 분할 · 자세 · 확정 수
  std::atomic<int> n_color_{0}, n_depth_{0}, n_sync_{0};   // 콜백 스레드가 다를 수 있음
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
};

}  // namespace perception
