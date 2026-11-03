// wrist_pipeline.cpp — 손목 D435 퍽 처리 구현, 설명은 wrist_pipeline.hpp

#include "wrist_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/exceptions.hpp>

#include "barcode.hpp"

namespace perception
{

namespace pw = perception::wrist;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

namespace
{
cv::Scalar color_of(const std::string & label)
{
  if (label == "red") {return {0, 0, 255};}
  if (label == "green") {return {0, 255, 0};}
  return {255, 255, 0};
}
}  // namespace

WristPipeline::WristPipeline(
  rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub, BarcodeStage * barcode)
: node_(node), barcode_(barcode), det_pub_(std::move(det_pub))
{
  auto & n = node;
  const auto model = n.declare_parameter<std::string>("wrist.model", "");
  const auto conf = n.declare_parameter<double>("wrist.conf", 0.5);
  const auto iou = n.declare_parameter<double>("wrist.iou", 0.7);
  const auto window = n.declare_parameter<int>("wrist.window", 5);
  const auto min_hits = n.declare_parameter<int>("wrist.min_hits", 3);
  const auto match_m = n.declare_parameter<double>("wrist.match_m", 0.02);
  enabled_ = n.declare_parameter<bool>("wrist.enabled", true);
  frame_id_ = n.declare_parameter<std::string>("wrist.frame_id", "d435_color_optical_frame");
  world_frame_ = n.declare_parameter<std::string>("wrist.world_frame", "odom");
  tf_timeout_s_ = n.declare_parameter<double>("wrist.tf_timeout_s", 0.05);
  reset_move_m_ = n.declare_parameter<double>("wrist.reset_move_m", 0.01);
  reset_rot_deg_ = n.declare_parameter<double>("wrist.reset_rot_deg", 2.0);
  depth_scale_ = n.declare_parameter<double>("wrist.depth_scale", 0.001);
  publish_debug_ = n.declare_parameter<bool>("wrist.publish_debug", false);
  const auto color_topic = n.declare_parameter<std::string>("wrist.color_topic", "/camera/camera/color/image_raw");
  const auto depth_topic = n.declare_parameter<std::string>(
    "wrist.depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const auto info_topic = n.declare_parameter<std::string>("wrist.info_topic", "/camera/camera/color/camera_info");
  // 영상 구독 QoS: 기본 reliable (큰 영상을 프로세스 사이에서 best_effort 로 받으면 조각 손실로 장이 빠짐, 광각과 같은 이유)
  //   realsense2_camera 기본 발행은 reliable → 호환. 드라이버를 best_effort 로 바꿨으면 여기도 best_effort
  const auto qos_mode = n.declare_parameter<std::string>("wrist.image_qos", "reliable");
  rclcpp::QoS img_qos(5);
  if (qos_mode == "best_effort") {img_qos.best_effort();} else {img_qos.reliable();}

  // 엔진: 실패해도 노드는 계속 (광각 · 바코드는 돎)
  if (model.empty()) {
    RCLCPP_ERROR(node.get_logger(), "[wrist] wrist.model 이 비어 있음 — 손목 퍽 검출 안 함");
  } else {
    try {
      seg_ = std::make_unique<pw::SegModel>(model, static_cast<float>(conf), static_cast<float>(iou));
      std::string names;
      for (const auto & c : seg_->names()) {names += c + " ";}
      const auto in = seg_->input_size();
      RCLCPP_INFO(node.get_logger(), "[wrist] 엔진 %s (%dx%d) 클래스 %s", model.c_str(), in.width, in.height, names.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node.get_logger(), "[wrist] 엔진을 못 읽음 %s: %s — 손목 퍽 검출 안 함", model.c_str(), e.what());
    }
  }
  tracker_ = pw::PuckTracker(static_cast<int>(window), static_cast<int>(min_hits), match_m);

  // 적재함 컵 바닥 태그
  rack_enabled_ = n.declare_parameter<bool>("rack.enabled", true);
  rack_frame_ = n.declare_parameter<std::string>("rack.frame", "base_link");
  {
    std::vector<int64_t> def(rack_p_.cell_tag_ids.begin(), rack_p_.cell_tag_ids.end());
    const auto ids = n.declare_parameter<std::vector<int64_t>>("rack.cell_tag_ids", def);
    rack_p_.cell_tag_ids.assign(ids.begin(), ids.end());
    std::string s;
    for (size_t i = 0; i < ids.size(); ++i) {s += " S" + std::to_string(i + 1) + "=" + std::to_string(ids[i]);}
    RCLCPP_INFO(node.get_logger(), "[rack] 칸 태그:%s", s.c_str());
  }
  rack_p_.tag_side_m = n.declare_parameter<double>("rack.tag_side_m", 0.026);
  rack_p_.max_view_deg = n.declare_parameter<double>("rack.max_view_deg", 25.0);
  rack_p_.max_tilt_deg = n.declare_parameter<double>("rack.max_tilt_deg", 10.0);
  rack_p_.max_reproj_px = n.declare_parameter<double>("rack.max_reproj_px", 1.5);
  const auto rack_window = n.declare_parameter<int>("rack.window", 5);
  const auto rack_min_hits = n.declare_parameter<int>("rack.min_hits", 3);
  rack_filter_ = pw::RackFilter(static_cast<int>(rack_window), static_cast<int>(rack_min_hits));
  const auto rack_hz = n.declare_parameter<double>("rack.max_hz", 10.0);
  rack_min_period_s_ = rack_hz > 0.0 ? 1.0 / rack_hz : 0.0;
  still_move_m_ = n.declare_parameter<double>("rack.still_move_m", 0.002);
  still_rot_deg_ = n.declare_parameter<double>("rack.still_rot_deg", 0.5);
  rack_publish_debug_ = n.declare_parameter<bool>("rack.publish_debug", false);
  rack_compare_nominal_ = n.declare_parameter<bool>("rack.compare_nominal", true);
  rack_pub_ = node.create_publisher<interfaces::msg::DetectionArray>("/perception/rack/cells", 10);
  if (rack_publish_debug_) {
    rack_debug_pub_ = node.create_publisher<Image>("/perception/rack_debug_image", rclcpp::SensorDataQoS());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (publish_debug_) {debug_pub_ = node.create_publisher<Image>("/perception/wrist_debug_image", rclcpp::SensorDataQoS());}
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group;   // message_filters 입력 구독도 손목 그룹으로
  info_sub_ = node.create_subscription<CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
      [this](CameraInfo::ConstSharedPtr m) {info_ = m;}, opt);
  color_sub_.subscribe(&node, color_topic, img_qos, opt);
  depth_sub_.subscribe(&node, depth_topic, img_qos, opt);
  if (barcode_) {   // 동기 전 컬러 → 바코드 (깊이 · TF 와 무관, 켜져 있을 때만)
    color_sub_.registerCallback(std::function<void(const Image::ConstSharedPtr &)>(
        [this](const Image::ConstSharedPtr & m) {
          if (!barcode_->due()) {return;}
          try {
            auto header = m->header;
            header.frame_id = frame_id_;
            barcode_->process(cv_bridge::toCvShare(m, "bgr8")->image, header);
          } catch (const cv_bridge::Exception & e) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 바코드 cv_bridge: %s", e.what());
          }
        }));
  }
  // 동기 전 컬러 → 적재함 (깊이 안 씀, 켜져 있을 때만)
  color_sub_.registerCallback(std::function<void(const Image::ConstSharedPtr &)>(
      [this](const Image::ConstSharedPtr & m) {on_rack(m);}));
  sync_ = std::make_shared<Sync>(Policy(10), color_sub_, depth_sub_);
  sync_->registerCallback(
    std::function<void(const Image::ConstSharedPtr &, const Image::ConstSharedPtr &)>(
      [this](const Image::ConstSharedPtr & c, const Image::ConstSharedPtr & d) {on_frame(c, d);}));
  RCLCPP_INFO(node.get_logger(), "[wrist] 컬러 %s · 깊이 %s · 위쪽 TF %s ← %s", color_topic.c_str(), depth_topic.c_str(),
    world_frame_.c_str(), frame_id_.c_str());
}

void WristPipeline::on_frame(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg)
{
  if (reset_req_ != reset_done_) {
    reset_done_ = reset_req_;
    tracker_.reset();
    last_R_.reset();
  }
  if (!enabled_ || !seg_) {return;}
  const auto t0 = std::chrono::steady_clock::now();
  if (!info_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] camera_info 아직 없음");
    return;
  }
  const pw::Intrinsics K{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
  cv::Mat bgr;
  cv::Mat1f depth;
  try {
    bgr = cv_bridge::toCvShare(color, "bgr8")->image;
    const auto d = cv_bridge::toCvShare(depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == "mono16") {
      d->image.convertTo(depth, CV_32F, depth_scale_);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      depth = d->image;
    } else {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 깊이 형식 %s 모름 (16UC1 · 32FC1)",
        depth_msg->encoding.c_str());
      return;
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] cv_bridge: %s", e.what());
    return;
  }
  if (depth.size() != bgr.size()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 깊이 %dx%d ≠ 컬러 %dx%d (align_depth 켜야 함)",
      depth.cols, depth.rows, bgr.cols, bgr.rows);
    return;
  }

  // 촬영 시각 카메라 자세 → 월드 위쪽 · 움직임
  pw::Mat3 R;
  pw::Vec3 t;
  try {
    const auto tf = tf_buffer_->lookupTransform(world_frame_, frame_id_, color->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    const auto & q = tf.transform.rotation;
    R = pw::quat_to_mat({q.x, q.y, q.z, q.w});
    t = pw::Vec3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] TF %s ← %s 없음: %s (발행 안 함)",
      world_frame_.c_str(), frame_id_.c_str(), e.what());
    return;
  }
  const pw::Vec3 up = R.transpose() * pw::Vec3::UnitZ();
  if (last_R_) {
    const double move = (t - last_t_).norm();
    const double rot = std::acos(std::clamp(((last_R_->transpose() * R).trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
    if (move > reset_move_m_ || rot > reset_rot_deg_) {tracker_.reset();}
  }
  last_R_ = R;
  last_t_ = t;

  // 분할 → 자세 → 거르기 → 확정
  std::vector<pw::SegInstance> inst;
  try {
    inst = (*seg_)(bgr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 추론 실패: %s", e.what());
    return;
  }
  const auto t_seg = std::chrono::steady_clock::now();
  std::vector<pw::TagPose> tags;
  if (!inst.empty()) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    tags = pw::detect_tags(gray, K);
  }
  std::vector<pw::RawPuck> raws;
  for (const auto & s : inst) {
    auto p = pw::puck_pose(s.mask, depth, K, tags, &up);
    if (!p) {continue;}
    pw::RawPuck r;
    r.pose = std::move(*p);
    r.kind = s.kind;
    r.info = pw::parse_kind(s.kind);
    r.conf = s.conf;
    r.reject = pw::reject_reason(r.pose);
    raws.push_back(std::move(r));
  }
  const auto dets = tracker_.update(raws);

  interfaces::msg::DetectionArray out;
  out.header = color->header;
  out.header.frame_id = frame_id_;
  for (const auto & d : dets) {
    interfaces::msg::Detection m;
    m.header = out.header;
    m.kind = "puck";
    m.label = d.label;
    m.shape = d.shape;
    m.id = d.id;
    m.confidence = static_cast<float>(d.confidence);
    m.pose.position.x = d.position.x();
    m.pose.position.y = d.position.y();
    m.pose.position.z = d.position.z();
    m.pose.orientation.x = d.orientation[0];
    m.pose.orientation.y = d.orientation[1];
    m.pose.orientation.z = d.orientation[2];
    m.pose.orientation.w = d.orientation[3];
    m.yaw_known = d.yaw_known;
    for (int k = 0; k < 4; ++k) {m.bbox[k] = static_cast<float>(d.bbox[k]);}
    out.detections.push_back(std::move(m));
  }
  det_pub_->publish(out);
  if (publish_debug_ && debug_pub_->get_subscription_count() > 0) {publish_debug(color, bgr, K, raws, dets);}

  // 처리 시간 (5 s 마다 평균)
  const auto t1 = std::chrono::steady_clock::now();
  sum_seg_ms_ += std::chrono::duration<double, std::milli>(t_seg - t0).count();
  sum_all_ms_ += std::chrono::duration<double, std::milli>(t1 - t0).count();
  ++n_frames_;
  if ((t1 - last_report_) > std::chrono::seconds(5)) {
    RCLCPP_INFO(node_.get_logger(), "[wrist] %d 장 평균: 전체 %.1f ms (분할 %.1f ms), 이번 장 분할 %zu · 자세 %zu · 확정 %zu",
      n_frames_, sum_all_ms_ / n_frames_, sum_seg_ms_ / n_frames_, inst.size(), raws.size(), dets.size());
    n_frames_ = 0;
    sum_seg_ms_ = sum_all_ms_ = 0.0;
    last_report_ = t1;
  }
}

// 적재함: 컵 바닥 태그 → 칸 바닥 중심 · yaw (rack.frame 기준), 카메라가 멈춘 장만 다수결에 넣음
void WristPipeline::on_rack(const Image::ConstSharedPtr & color)
{
  if (rack_reset_req_ != rack_reset_done_) {
    rack_reset_done_ = rack_reset_req_;
    rack_filter_.reset();
    rack_last_R_.reset();
  }
  if (!rack_enabled_ || !info_) {return;}
  const rclcpp::Time stamp(color->header.stamp);
  if (rack_last_stamp_ && (stamp - *rack_last_stamp_).seconds() < rack_min_period_s_ &&
    (stamp - *rack_last_stamp_).seconds() >= 0.0)
  {
    return;
  }
  rack_last_stamp_ = stamp;

  // 촬영 시각 rack.frame ← 카메라
  pw::Mat3 R;
  pw::Vec3 t;
  try {
    const auto tf = tf_buffer_->lookupTransform(rack_frame_, frame_id_, color->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    const auto & q = tf.transform.rotation;
    R = pw::quat_to_mat({q.x, q.y, q.z, q.w});
    t = pw::Vec3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[rack] TF %s ← %s 없음: %s (발행 안 함)",
      rack_frame_.c_str(), frame_id_.c_str(), e.what());
    return;
  }
  // 카메라가 움직였으면 이번 장은 안 셈 + 묶음 지움 (흔들린 영상 · TF 시각 어긋남)
  bool still = false;
  if (rack_last_R_) {
    const double move = (t - rack_last_t_).norm();
    const double rot = std::acos(std::clamp(((rack_last_R_->transpose() * R).trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
    still = move <= still_move_m_ && rot <= still_rot_deg_;
  }
  rack_last_R_ = R;
  rack_last_t_ = t;
  if (!still) {rack_filter_.reset();}

  cv::Mat gray;
  try {
    gray = cv_bridge::toCvShare(color, "mono8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[rack] cv_bridge: %s", e.what());
    return;
  }
  const pw::Intrinsics K{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
  cv::Mat D;
  if (!info_->d.empty()) {D = cv::Mat(info_->d, true).reshape(1, 1);}
  const pw::Vec3 up = R.transpose() * pw::Vec3::UnitZ();   // rack.frame z = 위 (적재함은 차체에 붙음)
  std::vector<std::pair<pw::RackTag, std::string>> rejected;
  const auto tags = pw::detect_rack_tags(gray, K, D, rack_p_, &up, rack_publish_debug_ ? &rejected : nullptr);

  rack_filter_.next_frame();
  if (still) {
    for (const auto & tg : tags) {rack_filter_.push(tg.cell, R * tg.t + t, pw::planar_yaw(R * tg.R));}
  }

  interfaces::msg::DetectionArray out;
  out.header = color->header;
  out.header.frame_id = rack_frame_;
  std::vector<std::string> lines;
  for (const int cell : rack_filter_.cells()) {
    const auto e = rack_filter_.get(cell);
    interfaces::msg::Detection m;
    m.header = out.header;
    m.kind = "rack_cell";
    m.label = pw::rack_cell_name(cell);
    m.id = std::to_string(pw::rack_tag_of(cell, rack_p_));
    m.confidence = static_cast<float>(e->hits) / static_cast<float>(rack_filter_.window());
    m.pose.position.x = e->position.x();
    m.pose.position.y = e->position.y();
    m.pose.position.z = e->position.z();
    m.pose.orientation.z = std::sin(e->yaw / 2);
    m.pose.orientation.w = std::cos(e->yaw / 2);
    m.yaw_known = true;
    for (const auto & tg : tags) {
      if (tg.cell != cell) {continue;}
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      for (const auto & c : tg.corners) {
        x0 = std::min(x0, c.x); y0 = std::min(y0, c.y); x1 = std::max(x1, c.x); y1 = std::max(y1, c.y);
      }
      m.bbox = {x0, y0, x1, y1};
    }
    // URDF 칸 프레임과 차이 (확인용)
    if (rack_compare_nominal_) {
      try {
        const auto nom = tf_buffer_->lookupTransform(rack_frame_, "rack_cell_" + m.label + "_link", tf2::TimePointZero);
        const auto & q = nom.transform.rotation;
        const double nyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double dyaw = std::atan2(std::sin(e->yaw - nyaw), std::cos(e->yaw - nyaw));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s dx %+.1f dy %+.1f dz %+.1f mm dyaw %+.1f deg", m.label.c_str(),
          (e->position.x() - nom.transform.translation.x) * 1000, (e->position.y() - nom.transform.translation.y) * 1000,
          (e->position.z() - nom.transform.translation.z) * 1000, dyaw * 180.0 / M_PI);
        lines.emplace_back(buf);
      } catch (const tf2::TransformException &) {
      }
    }
    out.detections.push_back(std::move(m));
  }
  rack_pub_->publish(out);
  if (!lines.empty()) {
    std::string all;
    for (const auto & l : lines) {all += "\n  " + l;}
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "[rack] URDF 칸 대비 차이:%s", all.c_str());
  }
  if (rack_publish_debug_ && rack_debug_pub_->get_subscription_count() > 0) {
    publish_rack_debug(color, gray, tags, rejected, lines);
  }
}

// 확인용 그림: 받은 태그 초록 테두리 + 칸 이름 · 칸 x 축 화살표, 거른 태그 빨강 + 이유, 왼쪽 위 URDF 대비 차이
void WristPipeline::publish_rack_debug(
  const Image::ConstSharedPtr & color, const cv::Mat & gray, const std::vector<pw::RackTag> & tags,
  const std::vector<std::pair<pw::RackTag, std::string>> & rejected, const std::vector<std::string> & lines)
{
  cv::Mat vis;
  cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
  auto poly = [&vis](const pw::RackTag & tg, const cv::Scalar & c) {
      for (int i = 0; i < 4; ++i) {cv::line(vis, tg.corners[i], tg.corners[(i + 1) % 4], c, 2);}
    };
  const pw::Intrinsics K{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
  auto proj = [&K](const pw::Vec3 & P) {
      return cv::Point(static_cast<int>(K.fx * P.x() / P.z() + K.cx), static_cast<int>(K.fy * P.y() / P.z() + K.cy));
    };
  for (const auto & tg : tags) {
    poly(tg, {0, 255, 0});
    const pw::Vec3 tip = tg.t + tg.R.col(0) * 0.015;   // 칸 x (로봇 앞)
    if (tg.t.z() > 0 && tip.z() > 0) {cv::arrowedLine(vis, proj(tg.t), proj(tip), {0, 255, 255}, 2, cv::LINE_8, 0, 0.3);}
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s view %.0f tilt %.0f", tg.name.c_str(), tg.view_deg, tg.tilt_deg);
    cv::putText(vis, buf, tg.corners[3] + cv::Point2f(0, 16), cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 0}, 1);
  }
  for (const auto & [tg, why] : rejected) {
    poly(tg, {0, 0, 255});
    cv::putText(vis, tg.name + " " + why, tg.corners[3] + cv::Point2f(0, 16), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      {0, 0, 255}, 1);
  }
  for (size_t i = 0; i < lines.size(); ++i) {
    cv::putText(vis, lines[i], cv::Point(8, 18 + 16 * static_cast<int>(i)), cv::FONT_HERSHEY_SIMPLEX, 0.45,
      {255, 255, 255}, 1);
  }
  auto msg = cv_bridge::CvImage(color->header, "bgr8", vis).toImageMsg();
  msg->header.frame_id = frame_id_;
  rack_debug_pub_->publish(*msg);
}

// 확인용 그림: 마스크 윤곽(색) · 거른 퍽 빨간 X + 이유 · 확정 퍽 흰 상자 + 긴축 화살표
void WristPipeline::publish_debug(
  const Image::ConstSharedPtr & color, const cv::Mat & bgr, const pw::Intrinsics & K,
  const std::vector<pw::RawPuck> & raws, const std::vector<pw::PuckDetection> & dets)
{
  cv::Mat vis = bgr.clone();
  auto proj = [&K](const pw::Vec3 & P) {
      return cv::Point(static_cast<int>(K.fx * P.x() / P.z() + K.cx), static_cast<int>(K.fy * P.y() / P.z() + K.cy));
    };
  for (const auto & r : raws) {
    const auto & b = r.pose.bbox;
    cv::rectangle(vis, cv::Point(b[0], b[1]), cv::Point(b[2], b[3]), color_of(r.info.color), 1);
    if (!r.reject.empty()) {
      cv::line(vis, cv::Point(b[0], b[1]), cv::Point(b[2], b[3]), {0, 0, 255}, 1);
      cv::line(vis, cv::Point(b[0], b[3]), cv::Point(b[2], b[1]), {0, 0, 255}, 1);
      cv::putText(vis, r.reject, cv::Point(b[0], b[3] + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 0, 255}, 1);
    }
  }
  for (const auto & d : dets) {
    const auto & b = d.bbox;
    cv::rectangle(vis, cv::Point(b[0] - 3, b[1] - 3), cv::Point(b[2] + 3, b[3] + 3), {255, 255, 255}, 2);
    const pw::Vec3 z = pw::quat_to_mat(d.orientation).col(2);
    const pw::Vec3 a = d.position - z * pw::PUCK_H / 2, e = d.position + z * pw::PUCK_H / 2;
    if (a.z() > 0 && e.z() > 0) {cv::arrowedLine(vis, proj(a), proj(e), {0, 255, 255}, 2, cv::LINE_8, 0, 0.25);}
    const std::string txt = "OK " + d.label + " " + d.shape + " id" + (d.id.empty() ? "?" : d.id) +
      (d.upright ? (*d.upright ? " up" : " lying") : "") + (d.yaw_known ? " yaw" : "");
    cv::putText(vis, txt, cv::Point(b[0], b[3] + 14), cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 255, 255}, 1);
  }
  auto msg = cv_bridge::CvImage(color->header, "bgr8", vis).toImageMsg();
  msg->header.frame_id = frame_id_;
  debug_pub_->publish(*msg);
}

}  // namespace perception
