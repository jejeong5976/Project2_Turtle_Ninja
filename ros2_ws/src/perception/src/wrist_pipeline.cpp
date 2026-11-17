// wrist_pipeline.cpp — 손목 D435 퍽 처리 구현, 설명은 wrist_pipeline.hpp

#include "wrist_pipeline.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

#include "image_cv.hpp"
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

double ms_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

double mono_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
  conf_ = static_cast<float>(conf);
  // 쥔 퍽 재기 (wrist_held.hpp): 기본 꺼짐. wrist.held.enable 은 장마다 읽음 (ros2 param set /perception wrist.held.enable true)
  n.declare_parameter<bool>("wrist.held.enable", false);
  held_p_.conf = static_cast<float>(n.declare_parameter<double>("wrist.held.conf", held_p_.conf));
  held_p_.roi_x0 = static_cast<int>(n.declare_parameter<int>("wrist.held.roi_x0", held_p_.roi_x0));
  held_p_.roi_y0 = static_cast<int>(n.declare_parameter<int>("wrist.held.roi_y0", held_p_.roi_y0));
  held_p_.min_area_px = static_cast<int>(n.declare_parameter<int>("wrist.held.min_area_px", held_p_.min_area_px));
  held_p_.right_touch_px = static_cast<int>(n.declare_parameter<int>("wrist.held.right_touch_px", held_p_.right_touch_px));
  held_p_.finger_y = static_cast<int>(n.declare_parameter<int>("wrist.held.finger_y", held_p_.finger_y));
  held_p_.top_ref_x = static_cast<int>(n.declare_parameter<int>("wrist.held.top_ref_x", held_p_.top_ref_x));
  held_pub_ = node.create_publisher<interfaces::msg::DetectionArray>(
    n.declare_parameter<std::string>("wrist.held.topic", "/held_puck/image"), 10);
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
  // 윗면이 둘레 낮은 면보다 이만큼도 안 솟은 분할은 퍽이 아님 (납작한 무늬). 0 = 안 봄
  min_rise_m_ = n.declare_parameter<double>("wrist.min_rise_m", 0.0);
  n.declare_parameter<bool>("wrist.tag_centre", true);
  // 마스크 자세 (wrist_pose.hpp MaskPoseParams): 마스크 안 깊이가 모자라거나 한쪽에 몰리면 중심 · 누운 긴축을 마스크 모양으로
  //   셋 다 장마다 읽음 (ros2 param set /perception wrist.mask_pose false = 끔)
  {
    const pw::MaskPoseParams mp;
    n.declare_parameter<bool>("wrist.mask_pose", mp.enable);
    n.declare_parameter<double>("wrist.mask_pose_min_fill", mp.min_fill);
    n.declare_parameter<double>("wrist.mask_pose_min_span", mp.min_span);
  }
  tag_roi_ = n.declare_parameter<bool>("wrist.tag_roi", true);
  tag_roi_pad_px_ = static_cast<int>(n.declare_parameter<int>("wrist.tag_roi_pad_px", 24));
  // 태그 다시 찾기 (wrist_tag_rescue.hpp): 1차에서 태그를 못 받은 퍽만 넓힌 상자 · AprilTag 3. 켜고 끄기는 장마다 읽음
  n.declare_parameter<bool>("wrist.tag_rescue", true);
  {
    pw::TagRescueParams rp;
    rp.pad_px = static_cast<int>(n.declare_parameter<int>("wrist.tag_rescue_pad_px", rp.pad_px));
    rp.margin_px = static_cast<int>(n.declare_parameter<int>("wrist.tag_rescue_margin_px", rp.margin_px));
    rp.apriltag3 = n.declare_parameter<bool>("wrist.tag_rescue_apriltag3", rp.apriltag3);
    rp.aruco = n.declare_parameter<bool>("wrist.tag_rescue_aruco", rp.aruco);
    rp.clahe = n.declare_parameter<bool>("wrist.tag_rescue_clahe", rp.clahe);
    rp.at3_max_hamming = static_cast<int>(n.declare_parameter<int>("wrist.tag_rescue_max_hamming", rp.at3_max_hamming));
    rp.at3_min_margin = n.declare_parameter<double>("wrist.tag_rescue_min_margin", rp.at3_min_margin);
    rp.budget_ms = n.declare_parameter<double>("wrist.tag_rescue_budget_ms", rp.budget_ms);
    const auto retry = n.declare_parameter<int>("wrist.tag_rescue_retry_frames", 4);
    rescuer_ = std::make_unique<pw::TagRescuer>(rp);
    rescue_cool_ = pw::RescueCooldown(static_cast<int>(retry), match_m);
    std::string ms;
    for (const auto & m : rescuer_->methods()) {ms += (ms.empty() ? "" : " → ") + m;}
    RCLCPP_INFO(node.get_logger(), "[wrist] 태그 다시 찾기 %s: %s, 상자 +%d px, 마스크 여유 %d px, 한 장 %.0f ms, 못 찾으면 %d 장 쉼%s",
      n.get_parameter("wrist.tag_rescue").as_bool() ? "켬" : "끔", ms.c_str(), rp.pad_px, rp.margin_px, rp.budget_ms,
      static_cast<int>(retry),
      pw::apriltag3_available() ? "" : " (AprilTag 3 없이 빌드: sudo apt install ros-$ROS_DISTRO-apriltag 후 perception 다시 빌드)");
  }
  publish_debug_ = n.declare_parameter<bool>("wrist.publish_debug", false);
  const auto color_topic = n.declare_parameter<std::string>("wrist.color_topic", "/camera/camera/color/image_raw");
  const auto depth_topic = n.declare_parameter<std::string>(
    "wrist.depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const auto info_topic = n.declare_parameter<std::string>("wrist.info_topic", "/camera/camera/color/camera_info");
  info_topic_ = info_topic;
  {
    const double stall = n.declare_parameter<double>("wrist.input_stall_s", 5.0);
    color_watch_ = InputWatch(stall, 60.0);
    depth_watch_ = InputWatch(stall, 60.0);
    info_watch_ = InputWatch(stall, 60.0);
  }
  // 영상 구독 QoS: 기본 reliable (큰 영상을 프로세스 사이에서 best_effort 로 받으면 조각 손실로 장이 빠짐, 광각과 같은 이유)
  //   realsense2_camera 기본 발행은 reliable → 호환. 드라이버를 best_effort 로 바꿨으면 여기도 best_effort
  //   깊이 (보관 장 수) 는 작게: 처리가 카메라보다 느리면 쌓인 지난 장 대신 최근 장을 처리 (5 면 약 0.2 s 늦은 장)
  const auto qos_mode = n.declare_parameter<std::string>("wrist.image_qos", "reliable");
  const auto qos_depth = n.declare_parameter<int>("wrist.image_qos_depth", 2);
  rclcpp::QoS img_qos(static_cast<size_t>(std::max<int64_t>(qos_depth, 1)));
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

  // 박스 윗면 (장소 · 시작 박스 앞 LOOK)
  box_enabled_ = n.declare_parameter<bool>("box.enabled", true);
  box_frame_ = n.declare_parameter<std::string>("box.frame", "base_footprint");
  box_p_.size = n.declare_parameter<double>("box.size", box_p_.size);
  box_p_.top_z = n.declare_parameter<double>("box.top_z", box_p_.top_z);
  box_p_.band = n.declare_parameter<double>("box.band", box_p_.band);
  box_p_.drop = n.declare_parameter<double>("box.drop", box_p_.drop);
  box_p_.min_x = n.declare_parameter<double>("box.min_x", box_p_.min_x);
  box_p_.front_min_x = n.declare_parameter<double>("box.front_min_x", box_p_.front_min_x);
  box_p_.min_side_pts = static_cast<int>(n.declare_parameter<int>("box.min_side_pts", box_p_.min_side_pts));
  box_p_.stride = static_cast<int>(n.declare_parameter<int>("box.stride", box_p_.stride));
  // 넓게 찾기: 처음 추정 답이 안 맞으면 테두리 점 범위에서 다시 맞춤 (0 = 끔)
  box_p_.search = n.declare_parameter<double>("box.search", box_p_.search);
  box_p_.max_inside = static_cast<int>(n.declare_parameter<int>("box.max_inside", box_p_.max_inside));
  box_p_.search_min_frac = n.declare_parameter<double>("box.search_min_frac", box_p_.search_min_frac);
  box_p_.search_max_starts = static_cast<int>(n.declare_parameter<int>("box.search_max_starts", box_p_.search_max_starts));
  box_p_.search_budget_ms = n.declare_parameter<double>("box.search_budget_ms", box_p_.search_budget_ms);
  {
    const auto g = n.declare_parameter<std::vector<double>>("box.guess", {0.2935, 0.0, 0.0});
    if (g.size() == 3) {box_guess_ = pw::Vec3(g[0], g[1], g[2]);}
  }
  const auto box_hz = n.declare_parameter<double>("box.max_hz", 5.0);
  box_min_period_s_ = box_hz > 0.0 ? 1.0 / box_hz : 0.0;
  box_pub_ = node.create_publisher<interfaces::msg::DetectionArray>("/perception/wrist/box", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (publish_debug_) {debug_pub_ = node.create_publisher<Image>("/perception/wrist_debug_image", rclcpp::SensorDataQoS());}
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group;   // message_filters 입력 구독도 손목 그룹으로
  group_ = group;
  subscribe_info();
  drift_correct_ = n.declare_parameter<bool>("wrist.drift_correct", true);
  drift_sub_ = node.create_subscription<geometry_msgs::msg::QuaternionStamped>(
    n.declare_parameter<std::string>("wrist.drift_topic", "/wrist_cam/drift"), rclcpp::QoS(1).transient_local().reliable(),
    [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m) {
      const auto & q = m->quaternion;
      const double nq = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (!(std::abs(nq - 1.0) < 0.01)) {return;}
      drift_R_ = pw::quat_to_mat({q.x / nq, q.y / nq, q.z / nq, q.w / nq});
      drift_frame_ = m->header.frame_id;
      RCLCPP_INFO(node_.get_logger(), "[wrist] 손목 카메라 보정 받음: %.2f° (%s)%s",
        std::acos(std::clamp((drift_R_.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI, drift_frame_.c_str(),
        drift_correct_ ? "" : " — wrist.drift_correct false 라 안 씀");
    }, opt);
  color_sub_.subscribe(&node, color_topic, img_qos, opt);
  depth_sub_.subscribe(&node, depth_topic, img_qos, opt);
  if (barcode_) {   // 동기 전 컬러 → 바코드 (깊이 · TF 와 무관, 켜져 있을 때만)
    color_sub_.registerCallback(std::function<void(const Image::ConstSharedPtr &)>(
        [this](const Image::ConstSharedPtr & m) {
          if (!barcode_->due()) {return;}
          try {
            auto header = m->header;
            header.frame_id = frame_id_;
            barcode_->process(image_cv::to_bgr8(*m), header);
          } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 바코드 영상 변환: %s", e.what());
          }
        }));
  }
  color_sub_.registerCallback(std::function<void(const Image::ConstSharedPtr &)>(
      [this](const Image::ConstSharedPtr &) {++n_color_; color_watch_.seen(mono_s());}));   // 입력 Hz 확인용 · 끊김 감시
  depth_sub_.registerCallback(std::function<void(const Image::ConstSharedPtr &)>(
      [this](const Image::ConstSharedPtr &) {++n_depth_; depth_watch_.seen(mono_s());}));   // 입력 Hz 확인용 · 끊김 감시
  {
    const double now = mono_s();
    color_watch_.start(now);
    depth_watch_.start(now);
  }
  input_timer_ = node.create_wall_timer(std::chrono::seconds(1), [this]() {check_input();}, group);
  sync_ = std::make_shared<Sync>(Policy(10), color_sub_, depth_sub_);
  sync_->registerCallback(
    std::function<void(const Image::ConstSharedPtr &, const Image::ConstSharedPtr &)>(
      [this](const Image::ConstSharedPtr & c, const Image::ConstSharedPtr & d) {
        ++n_sync_;
        const rclcpp::Time tc(c->header.stamp, RCL_ROS_TIME), td(d->header.stamp, RCL_ROS_TIME);
        lat_ms_.push_back((node_.now() - tc).seconds() * 1e3);          // 촬영 → 짝 콜백 시작 (늦음)
        pair_dt_ms_.push_back(std::abs((tc - td).seconds()) * 1e3);     // 컬러 · 깊이 촬영 시각 차이
        const auto s0 = std::chrono::steady_clock::now();
        on_box(c, d);
        on_frame(c, d);
        tm_.sync_cb += ms_since(s0);
        report_timing();
      }));
  RCLCPP_INFO(node.get_logger(), "[wrist] 컬러 %s · 깊이 %s · 위쪽 TF %s ← %s", color_topic.c_str(), depth_topic.c_str(),
    world_frame_.c_str(), frame_id_.c_str());
}

void WristPipeline::subscribe_info()
{
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group_;
  info_sub_.reset();
  info_sub_ = node_.create_subscription<CameraInfo>(info_topic_, rclcpp::SensorDataQoS(),
      [this](CameraInfo::ConstSharedPtr m) {info_ = m; info_watch_.seen(mono_s());}, opt);
  info_watch_.start(mono_s());
}

// 카메라는 도는데 이 노드 구독만 못 받으면 (DDS 짝짓기 유실) 그 구독만 새로 만듦.
//   message_filters Subscriber::subscribe() = 같은 토픽 · QoS · 콜백 그룹으로 다시 구독, 동기 (sync_) 연결은 그대로.
//   콜백과 같은 손목 그룹 → 구독 콜백과 겹치지 않음
void WristPipeline::check_input()
{
  const double now = mono_s();
  const bool wanted = enabled_ || box_enabled_ || (barcode_ && barcode_->due());
  if (!wanted) {
    color_watch_.start(now);
    depth_watch_.start(now);
    info_watch_.start(now);
    return;
  }
  auto heal = [&](InputWatch & w, const char * what, const std::string & topic, size_t pubs, auto && resub) {
      if (!w.due(now)) {return;}
      const double last = w.since_msg(now);
      const std::string when = last < 0 ? std::string("한 번도 못 받음") : cv::format("%.0f s 안 옴", last);
      resub();
      if (pubs > 0) {   // 카메라가 보이는데 못 받음 = 짝짓기 유실
        RCLCPP_WARN(node_.get_logger(), "[wrist] %s %s (%s, 발행자 %zu) → 구독 다시 만듦 (%d 번째)", what, when.c_str(),
          topic.c_str(), pubs, w.count());
      } else {          // D435 노드가 아직 없음 (켜지는 중 · respawn 중)
        RCLCPP_INFO(node_.get_logger(), "[wrist] %s %s (%s, 발행자 아직 안 보임) → 구독 새로 만듦 (%d 번째)", what,
          when.c_str(), topic.c_str(), w.count());
      }
    };
  heal(color_watch_, "컬러", color_sub_.getTopic(),
    color_sub_.getSubscriber() ? color_sub_.getSubscriber()->get_publisher_count() : 0, [this]() {color_sub_.subscribe();});
  heal(depth_watch_, "깊이", depth_sub_.getTopic(),
    depth_sub_.getSubscriber() ? depth_sub_.getSubscriber()->get_publisher_count() : 0, [this]() {depth_sub_.subscribe();});
  if (!info_) {   // camera_info 는 한 번 받으면 됨 (마지막 값을 씀)
    heal(info_watch_, "camera_info", info_topic_, info_sub_ ? info_sub_->get_publisher_count() : 0,
      [this]() {subscribe_info();});
  }
}

void WristPipeline::on_frame(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg)
{
  if (reset_req_ != reset_done_) {
    reset_done_ = reset_req_;
    tracker_.reset();
    rescue_cool_.reset();
    last_R_.reset();
  }
  if (!enabled_ || !seg_) {return;}
  const auto t0 = std::chrono::steady_clock::now();
  Timing st;   // 이번 장 단계별 [ms], 분할까지 간 장만 tm_ 에 더함
  if (!info_) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] camera_info 아직 없음");
    return;
  }
  const pw::Intrinsics K{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
  cv::Mat bgr;
  cv::Mat1f depth;
  try {
    bgr = image_cv::to_bgr8(*color);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 영상 변환: %s", e.what());
    return;
  }
  if (!to_depth_m(depth_msg, depth)) {return;}
  st.conv = ms_since(t0);
  if (depth.size() != bgr.size()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 깊이 %dx%d ≠ 컬러 %dx%d (align_depth 켜야 함)",
      depth.cols, depth.rows, bgr.cols, bgr.rows);
    return;
  }

  // 촬영 시각 카메라 자세 → 월드 위쪽 · 움직임
  const auto t_tf = std::chrono::steady_clock::now();
  pw::Mat3 R;
  pw::Vec3 t;
  try {
    const auto tf = tf_buffer_->lookupTransform(world_frame_, frame_id_, color->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    const auto & q = tf.transform.rotation;
    R = camera_rot(pw::quat_to_mat({q.x, q.y, q.z, q.w}));
    t = pw::Vec3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] TF %s ← %s 없음: %s (발행 안 함)",
      world_frame_.c_str(), frame_id_.c_str(), e.what());
    return;
  }
  st.tf = ms_since(t_tf);
  const pw::Vec3 up = R.transpose() * pw::Vec3::UnitZ();
  if (last_R_) {
    const double move = (t - last_t_).norm();
    const double rot = std::acos(std::clamp(((last_R_->transpose() * R).trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
    if (move > reset_move_m_ || rot > reset_rot_deg_) {tracker_.reset(); rescue_cool_.reset();}
  }
  last_R_ = R;
  last_t_ = t;

  // 분할 → 자세 → 거르기 → 확정
  const auto t_s0 = std::chrono::steady_clock::now();
  std::vector<pw::SegInstance> inst;
  const bool held_on = node_.get_parameter("wrist.held.enable").as_bool();
  seg_->set_conf(held_on ? std::min(conf_, held_p_.conf) : conf_);   // 쥔 퍽은 가까워 신뢰도가 낮음 (0.2–0.9)
  try {
    inst = (*seg_)(bgr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 추론 실패: %s", e.what());
    return;
  }
  if (held_on) {
    publish_held(color, pw::measure_held(inst, held_p_));
    // 일반 검출은 wrist.conf 그대로 (NMS 는 높은 점수부터라 낮춘 문턱이 높은 점수 결과를 안 바꿈)
    inst.erase(std::remove_if(inst.begin(), inst.end(), [this](const pw::SegInstance & s) {return s.conf < conf_;}), inst.end());
  }
  const auto t_seg = std::chrono::steady_clock::now();
  st.seg = std::chrono::duration<double, std::milli>(t_seg - t_s0).count();
  {
    const auto & sp = seg_->last_timing();
    st.seg_pre = sp.pre;
    st.seg_infer = sp.infer;
    st.seg_gpu = sp.gpu;
    st.seg_post = sp.post;
  }
  std::vector<pw::TagPose> tags;
  cv::Mat gray;
  if (!inst.empty()) {
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    if (tag_roi_) {   // 퍽 태그는 퍽 위에만 있음 → 분할 상자 둘레만 (전체 영상 두 번 = 장당 약 17 ms, Jetson)
      std::vector<cv::Rect> rois;
      for (const auto & s : inst) {
        const int p = tag_roi_pad_px_;
        rois.emplace_back(cv::Point(static_cast<int>(std::floor(s.box[0])) - p, static_cast<int>(std::floor(s.box[1])) - p),
          cv::Point(static_cast<int>(std::ceil(s.box[2])) + p, static_cast<int>(std::ceil(s.box[3])) + p));
      }
      tags = pw::detect_tags_in(gray, rois, K);
    } else {
      tags = pw::detect_tags(gray, K);
    }
    ++st.tag_runs;
  }
  const auto t_tag = std::chrono::steady_clock::now();
  st.tags = std::chrono::duration<double, std::milli>(t_tag - t_seg).count();
  std::vector<pw::RawPuck> raws;
  std::vector<int> raw_of(inst.size(), -1);   // inst 번호 → raws 번호 (태그 다시 찾기용)
  const bool tag_centre = node_.get_parameter("wrist.tag_centre").as_bool();
  pw::MaskPoseParams mask_p;
  mask_p.enable = node_.get_parameter("wrist.mask_pose").as_bool();
  mask_p.min_fill = node_.get_parameter("wrist.mask_pose_min_fill").as_double();
  mask_p.min_span = node_.get_parameter("wrist.mask_pose_min_span").as_double();
  for (size_t k = 0; k < inst.size(); ++k) {
    const auto & s = inst[k];
    auto p = pw::puck_pose(s.mask, depth, K, tags, &up, min_rise_m_, tag_centre, &st.pose_parts, &mask_p);
    if (!p) {continue;}
    pw::RawPuck r;
    r.pose = std::move(*p);
    r.kind = s.kind;
    r.info = pw::parse_kind(s.kind);
    r.conf = s.conf;
    r.reject = pw::reject_reason(r.pose);
    raw_of[k] = static_cast<int>(raws.size());
    raws.push_back(std::move(r));
  }
  st.pose = ms_since(t_tag);
  st.pucks = static_cast<int>(inst.size());

  // 태그 다시 찾기: 1차에서 태그 못 받고 묶음 ID 도 없는 퍽만 (없으면 검출 안 함)
  pw::RescueStats rs;
  std::vector<std::pair<pw::TagPose, std::string>> rescued;   // 확인 그림 · 로그용
  if (!inst.empty() && rescuer_ && node_.get_parameter("wrist.tag_rescue").as_bool()) {
    const auto t_r0 = std::chrono::steady_clock::now();
    const int got = pw::rescue_raw_pucks(*rescuer_, gray, depth, K, up, inst, raw_of, raws, tags, tracker_,
        min_rise_m_, tag_centre, &rescue_cool_, &rs, &st.pose_parts, &mask_p);
    st.rescue = ms_since(t_r0);
    st.rescue_runs = rs.rois > 0 ? 1 : 0;
    st.rescue_rois = rs.rois;
    st.rescued = got;
    st.rescue_need = rs.need;
    for (size_t k = 0; k < inst.size() && k < rs.how.size(); ++k) {
      if (rs.how[k].empty() || raw_of[k] < 0) {continue;}
      const auto & r = raws[static_cast<size_t>(raw_of[k])];
      if (!r.pose.tag) {continue;}
      rescued.emplace_back(*r.pose.tag, rs.how[k]);
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "[wrist] 태그 다시 찾음: ID %d (%s) · 퍽 상자 (%d, %d)–(%d, %d) %s%s", r.pose.tag->id, rs.how[k].c_str(),
        r.pose.bbox[0], r.pose.bbox[1], r.pose.bbox[2], r.pose.bbox[3], r.kind.c_str(),
        r.reject.empty() ? "" : (" · 거름 " + r.reject).c_str());
    }
  }
  // 마스크 자세로 중심을 잡은 퍽 (깊이 모자람): 1 s 에 한 줄, 첫 퍽만 (Detection 에 빈 칸이 없어 control_master 에는 표시가 안 감)
  //   위치는 box.frame (base_footprint, control_master 의 base_link 와 x · y 같음). 그 TF 가 없으면 world_frame
  for (const auto & r : raws) {
    if (!r.pose.mask_centre) {continue;}
    const auto now = std::chrono::steady_clock::now();
    if (now - mask_log_t_ < std::chrono::seconds(1)) {break;}
    mask_log_t_ = now;
    pw::Mat3 Rb = R;
    pw::Vec3 tb = t;
    std::string fb = world_frame_;
    try {
      const auto tf = tf_buffer_->lookupTransform(box_frame_, frame_id_, color->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_s_));
      const auto & q = tf.transform.rotation;
      Rb = camera_rot(pw::quat_to_mat({q.x, q.y, q.z, q.w}));
      tb = pw::Vec3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      fb = box_frame_;
    } catch (const tf2::TransformException &) {
    }
    const pw::Vec3 bp = Rb * r.pose.center + tb, bd = Rb * r.pose.depth_center + tb;
    RCLCPP_INFO(node_.get_logger(),
      "[wrist] 마스크 중심 (깊이 모자람): %s 상자 (%d, %d)–(%d, %d) · 깊이 점 %.0f %% · 윗면 덮음 %.0f %% → 깊이 중심에서 %.0f mm 옮김 "
      "(%s (%.3f, %.3f) → (%.3f, %.3f), 윗면 높이 그대로%s)%s", r.kind.c_str(), r.pose.bbox[0], r.pose.bbox[1], r.pose.bbox[2],
      r.pose.bbox[3], r.pose.depth_cover[0] * 100.0, r.pose.depth_cover[1] * 100.0, (bp - bd).norm() * 1000.0, fb.c_str(),
      bd.x(), bd.y(), bp.x(), bp.y(), r.pose.axis_source == "mask_flat" ? " · 누운 긴축도 마스크로" : "",
      r.reject.empty() ? "" : (" · 거름 " + r.reject).c_str());
    break;
  }
  const auto t_trk = std::chrono::steady_clock::now();
  const auto dets = tracker_.update(raws);
  st.track = ms_since(t_trk);
  const auto t_pub = std::chrono::steady_clock::now();

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
    m.center_from_tag = d.center_from_tag;
    for (int k = 0; k < 4; ++k) {m.bbox[k] = static_cast<float>(d.bbox[k]);}
    out.detections.push_back(std::move(m));
  }
  det_pub_->publish(out);
  if (publish_debug_ && debug_pub_->get_subscription_count() > 0) {publish_debug(color, bgr, K, raws, dets, tags, rs, rescued);}

  // 처리 시간: 단계별로 더해 두고 report_timing 이 5 s 마다 평균
  st.pub = ms_since(t_pub);
  st.all = ms_since(t0);
  ++tm_.frames;
  tm_.pucks += st.pucks;
  tm_.conv += st.conv;
  tm_.tf += st.tf;
  tm_.seg += st.seg;
  tm_.seg_pre += st.seg_pre;
  tm_.seg_infer += st.seg_infer;
  tm_.seg_gpu += st.seg_gpu;
  tm_.seg_post += st.seg_post;
  tm_.tags += st.tags;
  tm_.tag_runs += st.tag_runs;
  tm_.pose += st.pose;
  tm_.pose_parts.points += st.pose_parts.points;
  tm_.pose_parts.shape += st.pose_parts.shape;
  tm_.pose_parts.ring += st.pose_parts.ring;
  tm_.pose_parts.refine += st.pose_parts.refine;
  tm_.track += st.track;
  tm_.rescue += st.rescue;
  tm_.rescue_runs += st.rescue_runs;
  tm_.rescue_rois += st.rescue_rois;
  tm_.rescued += st.rescued;
  tm_.rescue_need += st.rescue_need;
  tm_.pub += st.pub;
  tm_.all += st.all;
  last_counts_ = {inst.size(), raws.size(), dets.size()};
}

// 쥔 퍽 잰 값 → /held_puck/image (못 재면 빈 배열 — 받는 쪽이 켜져 있음 · 장 시각을 앎). 칸 뜻은 wrist_pipeline.hpp 머리말
void WristPipeline::publish_held(const Image::ConstSharedPtr & color, const pw::HeldMeasure & m)
{
  interfaces::msg::DetectionArray out;
  out.header = color->header;
  out.header.frame_id = frame_id_;
  if (m.ok) {
    interfaces::msg::Detection d;
    d.header = out.header;
    d.kind = "held_image";
    const auto info = pw::parse_kind(m.kind);
    d.label = info.color;
    d.shape = info.shape;
    d.confidence = m.conf;
    d.pose.position.x = m.top_y;
    d.pose.position.y = m.left_x;
    d.pose.position.z = m.bottom_y;
    d.pose.orientation.x = m.top_deg;
    d.pose.orientation.y = m.left_deg;
    d.pose.orientation.z = m.area;
    d.pose.orientation.w = 0.0;
    d.yaw_known = std::isfinite(m.left_deg);
    for (int k = 0; k < 4; ++k) {d.bbox[k] = static_cast<float>(m.bbox[k]);}
    out.detections.push_back(std::move(d));
  } else {
    RCLCPP_DEBUG_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "[wrist] 쥔 퍽 못 잼: %s", m.why.c_str());
  }
  held_pub_->publish(out);
}

// 5 s 마다: 들어온 장 (컬러 · 깊이 · 짝) Hz + on_frame 단계별 평균 [ms] + on_box · 짝 콜백 전체
//   짝 콜백 전체 (on_box + on_frame) 가 1 / 카메라 Hz 를 넘으면 장이 밀려 빠짐
void WristPipeline::report_timing()
{
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_report_).count();
  if (dt < 5.0) {return;}
  const int nc = n_color_.exchange(0), nd = n_depth_.exchange(0), ns = n_sync_.exchange(0);
  const auto & m = tm_;
  const double f = std::max(m.frames, 1);
  const double pk = std::max(m.pucks, 1);
  auto q3 = [](std::vector<double> v) {   // 중앙 · 95 % · 최대
      if (v.empty()) {return std::array<double, 3>{0, 0, 0};}
      return std::array<double, 3>{pw::percentile(v, 50), pw::percentile(v, 95), *std::max_element(v.begin(), v.end())};
    };
  const auto la = q3(lat_ms_), pd = q3(pair_dt_ms_);
  RCLCPP_INFO(node_.get_logger(),
    "[wrist] 분할 장당 ms: 전처리 %.1f · 추론 %.1f (GPU %.1f) · 후처리 %.1f | 자세 퍽당 ms: 점 %.1f · 모양 %.1f · 고리 %.1f · 보정 %.1f | "
    "촬영→처리 시작 ms 중앙 %.0f · 95%% %.0f · 최대 %.0f | 컬러·깊이 시각 차 ms 중앙 %.1f · 95%% %.1f · 최대 %.1f",
    m.seg_pre / f, m.seg_infer / f, m.seg_gpu / f, m.seg_post / f,
    m.pose_parts.points / pk, m.pose_parts.shape / pk, m.pose_parts.ring / pk, m.pose_parts.refine / pk,
    la[0], la[1], la[2], pd[0], pd[1], pd[2]);
  lat_ms_.clear();
  pair_dt_ms_.clear();
  RCLCPP_INFO(node_.get_logger(),
    "[wrist] %.1f s: 입력 컬러 %.1f Hz · 깊이 %.1f Hz · 짝 %.1f Hz · 처리 %.1f Hz | 장당 ms: 변환 %.1f · TF %.1f · 분할 %.1f · "
    "태그 %.1f (회당 %.1f) · 자세 %.1f (퍽당 %.1f) · 다시 찾기 %.1f (회당 %.1f × %d, 상자 %d · 붙임 %d · 대상 %d) · 확정 %.1f · "
    "발행 %.1f = 전체 %.1f | 박스 %.1f ms × %d | "
    "짝 콜백 %.1f ms | 이번 장 분할 %zu · 자세 %zu · 확정 %zu",
    dt, nc / dt, nd / dt, ns / dt, m.frames / dt, m.conv / f, m.tf / f, m.seg / f, m.tags / f,
    m.tag_runs > 0 ? m.tags / m.tag_runs : 0.0, m.pose / f,
    m.pucks > 0 ? m.pose / m.pucks : 0.0, m.rescue / f, m.rescue_runs > 0 ? m.rescue / m.rescue_runs : 0.0, m.rescue_runs,
    m.rescue_rois, m.rescued, m.rescue_need, m.track / f, m.pub / f, m.all / f, m.box_runs > 0 ? m.box / m.box_runs : 0.0,
    m.box_runs, ns > 0 ? m.sync_cb / ns : 0.0,
    last_counts_[0], last_counts_[1], last_counts_[2]);
  tm_ = Timing{};
  last_report_ = now;
}

pw::Mat3 WristPipeline::camera_rot(const pw::Mat3 & R_tf) const
{
  if (!drift_correct_ || drift_frame_.empty() || drift_frame_ != frame_id_) {return R_tf;}
  return R_tf * drift_R_;   // 실제 카메라 좌표 점 p → 로봇 = TF · (drift · p)
}

bool WristPipeline::to_depth_m(const Image::ConstSharedPtr & depth_msg, cv::Mat1f & depth)
{
  try {
    const cv::Mat d = image_cv::wrap(*depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == "mono16") {
      d.convertTo(depth, CV_32F, depth_scale_);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      depth = d;
    } else {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 깊이 형식 %s 모름 (16UC1 · 32FC1)",
        depth_msg->encoding.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist] 깊이 영상 변환: %s", e.what());
    return false;
  }
  return true;
}

// 박스 윗면: 깊이 → box.frame 점군 → 정사각형 맞춤 → /perception/wrist/box (못 찾으면 빈 배열)
void WristPipeline::on_box(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth_msg)
{
  if (!box_enabled_ || !info_) {return;}
  const rclcpp::Time stamp(color->header.stamp);
  if (box_last_stamp_ && (stamp - *box_last_stamp_).seconds() < box_min_period_s_ &&
    (stamp - *box_last_stamp_).seconds() >= 0.0)
  {
    return;
  }
  box_last_stamp_ = stamp;
  const auto b0 = std::chrono::steady_clock::now();
  cv::Mat1f depth;
  if (!to_depth_m(depth_msg, depth)) {return;}
  pw::Mat3 R;
  pw::Vec3 t;
  try {
    const auto tf = tf_buffer_->lookupTransform(box_frame_, frame_id_, color->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    const auto & q = tf.transform.rotation;
    R = camera_rot(pw::quat_to_mat({q.x, q.y, q.z, q.w}));
    t = pw::Vec3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[box] TF %s ← %s 없음: %s (발행 안 함)",
      box_frame_.c_str(), frame_id_.c_str(), e.what());
    return;
  }
  const pw::Intrinsics K{info_->k[0], info_->k[4], info_->k[2], info_->k[5]};
  const auto r = pw::fit_place_box(depth, K, R, t, box_p_, box_guess_);
  if (r.search_starts > 0) {
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "[box] 넓게 찾기 %.1f ms · 시작 %d%s → %s", r.search_ms,
      r.search_starts, r.search_capped ? " (상한)" : "", r.ok ? "찾음" : r.why.c_str());
  }

  interfaces::msg::DetectionArray out;
  out.header = color->header;
  out.header.frame_id = box_frame_;
  if (r.ok) {
    interfaces::msg::Detection m;
    m.header = out.header;
    m.kind = "place_box";
    m.confidence = static_cast<float>(std::clamp(1.0 - r.rms / box_p_.huber, 0.0, 1.0));
    m.pose.position.x = r.x;
    m.pose.position.y = r.y;
    m.pose.position.z = box_p_.top_z;
    m.pose.orientation.z = std::sin(r.yaw / 2);
    m.pose.orientation.w = std::cos(r.yaw / 2);
    m.yaw_known = true;
    out.detections.push_back(std::move(m));
  } else {
    RCLCPP_DEBUG(node_.get_logger(), "[box] 못 찾음: %s", r.why.c_str());
  }
  box_pub_->publish(out);
  tm_.box += ms_since(b0);
  ++tm_.box_runs;
}

// 확인용 그림: 마스크 윤곽(색) · 거른 퍽 빨간 X + 이유 · 확정 퍽 흰 상자 + 긴축 화살표
void WristPipeline::publish_debug(
  const Image::ConstSharedPtr & color, const cv::Mat & bgr, const pw::Intrinsics & K,
  const std::vector<pw::RawPuck> & raws, const std::vector<pw::PuckDetection> & dets,
  const std::vector<pw::TagPose> & tags, const pw::RescueStats & rs,
  const std::vector<std::pair<pw::TagPose, std::string>> & rescued)
{
  cv::Mat vis = bgr.clone();
  auto proj = [&K](const pw::Vec3 & P) {
      return cv::Point(static_cast<int>(K.fx * P.x() / P.z() + K.cx), static_cast<int>(K.fy * P.y() / P.z() + K.cy));
    };
  // 태그: 1차 (aruco) 초록 t<ID> · 다시 찾기 자홍 R<ID> + 방법 · 다시 찾기 넓힌 상자 자홍 얇은 사각
  auto poly = [&vis](const pw::TagPose & t, const cv::Scalar & c) {
      for (int i = 0; i < 4; ++i) {
        cv::line(vis, cv::Point(t.corners[i]), cv::Point(t.corners[(i + 1) % 4]), c, 1);
      }
    };
  for (const auto & t : tags) {
    poly(t, {0, 255, 0});
    cv::putText(vis, "t" + std::to_string(t.id), cv::Point(t.corners[1]) + cv::Point(2, 0), cv::FONT_HERSHEY_SIMPLEX, 0.4,
      {0, 255, 0}, 1);
  }
  for (const auto & roi : rs.roi_rects) {cv::rectangle(vis, roi, {255, 0, 255}, 1);}
  for (const auto & [t, how] : rescued) {
    poly(t, {255, 0, 255});
    cv::putText(vis, "R" + std::to_string(t.id) + " " + how, cv::Point(t.corners[1]) + cv::Point(2, 0),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 0, 255}, 1);
  }
  for (const auto & r : raws) {
    const auto & b = r.pose.bbox;
    cv::rectangle(vis, cv::Point(b[0], b[1]), cv::Point(b[2], b[3]), color_of(r.info.color), 1);
    // 이번 장 태그 ID (분할 상자 왼쪽 위): 없으면 t?
    cv::putText(vis, r.pose.tag ? "t" + std::to_string(r.pose.tag->id) : std::string("t?"), cv::Point(b[0], b[1] - 3),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, r.pose.tag ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 1);
    if (r.pose.mask_centre && r.pose.top_center && r.pose.top_center->z() > 0) {   // 마스크 자세: 윗면 중심 (자홍 점) + "mask"
      cv::circle(vis, proj(*r.pose.top_center), 4, {255, 0, 255}, cv::FILLED);
      cv::putText(vis, "mask", cv::Point(b[0] + 30, b[1] - 3), cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 0, 255}, 1);
    }
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
  auto msg = image_cv::bgr8_msg(color->header, vis);
  msg->header.frame_id = frame_id_;
  debug_pub_->publish(*msg);
}

}  // namespace perception
