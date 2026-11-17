// wrist_tag_stage.cpp — 손목 태그 단계 구현, 설명은 wrist_tag_stage.hpp

#include "wrist_tag_stage.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include <tf2/exceptions.hpp>

#include "image_cv.hpp"

namespace perception
{

namespace pw = perception::wrist;

namespace
{
// 변환 (회전 · 이동) 을 점에 적용
cv::Point3d apply(const geometry_msgs::msg::TransformStamped & T, const cv::Point3d & p)
{
  const auto & q = T.transform.rotation;
  const auto & t = T.transform.translation;
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  const double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - z * w), r02 = 2 * (x * z + y * w);
  const double r10 = 2 * (x * y + z * w), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - x * w);
  const double r20 = 2 * (x * z - y * w), r21 = 2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y);
  return {r00 * p.x + r01 * p.y + r02 * p.z + t.x, r10 * p.x + r11 * p.y + r12 * p.z + t.y,
    r20 * p.x + r21 * p.y + r22 * p.z + t.z};
}
}  // namespace

WristTagStage::WristTagStage(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group, std::shared_ptr<tf2_ros::Buffer> tf)
: tf_(std::move(tf)), node_(node), group_(std::move(group))   // 선언 순서대로 (tf_ 가 먼저)
{
  // 영상 · 프레임 이름은 손목 퍽 단계와 같은 값 (WristPipeline 이 먼저 선언)
  color_topic_ = node.get_parameter("wrist.color_topic").as_string();
  frame_id_ = node.get_parameter("wrist.frame_id").as_string();
  const auto info_topic = node.get_parameter("wrist.info_topic").as_string();
  max_hz_ = node.declare_parameter<double>("wrist_tags.max_hz", 5.0);
  pw::TagReader::Params p;
  p.max_hamming = static_cast<int>(node.declare_parameter<int>("wrist_tags.max_hamming", p.max_hamming));
  p.min_margin = node.declare_parameter<double>("wrist_tags.min_margin", p.min_margin);
  p.nthreads = static_cast<int>(node.declare_parameter<int>("wrist_tags.nthreads", p.nthreads));
  const auto ids = node.declare_parameter<std::vector<int64_t>>("wrist_tags.puck_ids", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  for (const auto id : ids) {puck_ids_.insert(static_cast<int>(id));}
  puck_tag_m_ = node.declare_parameter<double>("wrist_tags.puck_tag_size_m", 0.0153);   // 퍽 윗면 태그 검은 사각 (인쇄 실측 15.2~15.4 mm)
  held_max_m_ = node.declare_parameter<double>("wrist_tags.held_max_dist_m", 0.15);     // 카메라에서 이보다 가까운 퍽 태그 = 쥔 퍽
  // 영상 QoS: reliable KeepLast(2)
  img_qos_ = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  // 영역 검출: 칸 바닥 태그 (floor_z) ~ 칸 안 퍽 윗면 태그 (floor_z + 0.06) 를 칸 중심 ± cell_half_m 사각기둥으로 덮음
  //   cell_half_m 0.03 = 태그 반 대각 1.7 cm + TF · 칸 표 오차 · 퍽이 컵 안에서 치우친 만큼
  roi_on_ = node.declare_parameter<bool>("wrist_tags.roi", true);
  base_frame_ = node.declare_parameter<std::string>("wrist_tags.roi_base_frame", "base_link");
  std::vector<std::string> def_cells;
  for (int k = 1; k <= 12; ++k) {def_cells.push_back("rack_cell_S" + std::to_string(k) + "_link");}
  cell_frames_ = node.declare_parameter<std::vector<std::string>>("wrist_tags.roi_cell_frames", def_cells);
  floor_z_ = node.declare_parameter<double>("wrist_tags.roi_floor_z", 0.1325);       // 칸 바닥 (config/rack_cells.yaml floor_z, base_frame)
  cell_half_m_ = node.declare_parameter<double>("wrist_tags.roi_cell_half_m", 0.03);
  cell_top_m_ = node.declare_parameter<double>("wrist_tags.roi_cell_top_m", 0.075);  // 바닥 위 이 높이까지 (퍽 윗면 0.06 + 여유)
  held_frame_ = node.declare_parameter<std::string>("wrist_tags.roi_held_frame", "end_effector_link");
  held_half_m_ = node.declare_parameter<double>("wrist_tags.roi_held_half_m", 0.05); // 쥔 퍽 태그 = 손끝 둘레 이 정육면체 안
  roi_pad_px_ = node.declare_parameter<double>("wrist_tags.roi_pad_px", 24.0);
  roi_max_frac_ = node.declare_parameter<double>("wrist_tags.roi_max_frac", 0.5);
  full_every_ = static_cast<int>(node.declare_parameter<int>("wrist_tags.roi_full_every", 10));
  tf_timeout_s_ = node.declare_parameter<double>("wrist_tags.roi_tf_timeout_s", 0.05);
  if (!(max_hz_ > 0.0)) {throw std::runtime_error("wrist_tags.max_hz 는 0 보다 커야 함");}
  if (full_every_ < 1) {throw std::runtime_error("wrist_tags.roi_full_every 는 1 이상");}
  reader_ = std::make_unique<pw::TagReader>(p);
  pub_ = node.create_publisher<interfaces::msg::WristTagArray>("/perception/wrist/tags", 10);
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group_;
  image_sub_ = node.create_subscription<Image>(color_topic_, img_qos_, [this](Image::ConstSharedPtr m) {on_image(m);}, opt);
  info_sub_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {info_ = m;}, opt);
  RCLCPP_INFO(node.get_logger(), "[wrist_tags] %s, 영상 %s, 최대 %.0f Hz, 켜기 ~/wrist_tags/enable", pw::TagReader::name(),
    color_topic_.c_str(), max_hz_);
  if (roi_on_) {
    RCLCPP_INFO(node.get_logger(), "[wrist_tags] 영역 검출 켬: 칸 %zu 개 (± %.0f mm, 바닥 %.4f ~ +%.0f mm) · 쥔 퍽 %s ± %.0f mm · "
      "여유 %.0f px · 화면 %.0f %% 넘으면 전체 · %d 장마다 전체로 확인",
      cell_frames_.size(), cell_half_m_ * 1e3, floor_z_, cell_top_m_ * 1e3, held_frame_.c_str(), held_half_m_ * 1e3, roi_pad_px_,
      roi_max_frac_ * 100.0, full_every_);
  }
  if (node.declare_parameter<bool>("wrist_tags.enabled", false)) {set_enabled(true);}
}

void WristTagStage::set_enabled(bool on)
{
  if (on && !enabled_) {++on_gen_;}
  enabled_ = on;
}

void WristTagStage::on_image(const Image::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  if (const unsigned g = on_gen_; g != seen_gen_) {   // 막 켬 → 첫 장부터 처리
    seen_gen_ = g;
    have_last_ = false;
  }
  const rclcpp::Time stamp(msg->header.stamp, node_.get_clock()->get_clock_type());
  if (have_last_ && stamp >= last_ && (stamp - last_).seconds() < 1.0 / max_hz_) {return;}
  const auto info = info_;
  if (!info || !(info->k[0] > 0.0) || !(info->k[4] > 0.0)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist_tags] camera_info 가 아직 없거나 K 가 0");
    return;
  }
  have_last_ = true;
  last_ = stamp;
  const auto t0 = std::chrono::steady_clock::now();
  cv::Mat gray;
  try {
    gray = image_cv::to_mono8(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist_tags] 영상 변환 실패 (%s): %s", msg->encoding.c_str(), e.what());
    return;
  }
  pw::TagCamera cam;
  cam.K = cv::Matx33d(info->k[0], info->k[1], info->k[2], info->k[3], info->k[4], info->k[5], info->k[6], info->k[7], info->k[8]);
  cam.D.assign(info->d.begin(), info->d.end());
  cam.model = info->distortion_model;
  cam.binning_x = info->binning_x ? static_cast<float>(info->binning_x) : 1.0f;
  cam.binning_y = info->binning_y ? static_cast<float>(info->binning_y) : 1.0f;
  cam.roi_x = static_cast<float>(info->roi.x_offset);
  cam.roi_y = static_cast<float>(info->roi.y_offset);

  // 영역 검출: full_every 장마다 · TF 없음 · 영역이 너무 넓음 → 전체 영상
  std::vector<cv::Rect> rois;
  cv::Rect held_roi;
  bool have_rois = roi_on_ && tf_ && make_rois(msg->header.stamp, cam.K, gray.size(), rois, held_roi);
  const bool full_check = (seq_++ % full_every_) == 0;
  const bool use_roi = have_rois && !full_check;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  if (use_roi) {
    reader_->detect_in(gray, rois, corners, ids);
    ++roi_runs_;
  } else {
    reader_->detect(gray, corners, ids);
    ++full_runs_;
    if (have_rois) {   // 확인 장: 영역 밖에 나온 태그 = 영역 검출이었으면 놓쳤을 것
      for (const auto & c : corners) {
        const cv::Point2f m = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
        bool in = false;
        for (const auto & r : rois) {in = in || r.contains(cv::Point(static_cast<int>(m.x), static_cast<int>(m.y)));}
        ++checked_;
        if (!in) {++outside_;}
      }
    }
  }
  interfaces::msg::WristTagArray out;
  out.header = msg->header;
  out.header.frame_id = frame_id_;
  auto add = [&out](int id, const std::vector<cv::Point2f> & px, const std::vector<cv::Point2f> & n, bool rescue) {
      interfaces::msg::WristTag t;
      t.id = id;
      for (int j = 0; j < 4; ++j) {
        t.corners_px[2 * j] = px[j].x;
        t.corners_px[2 * j + 1] = px[j].y;
        t.corners_norm[2 * j] = n[j].x;
        t.corners_norm[2 * j + 1] = n[j].y;
      }
      t.rescue = rescue;
      out.tags.push_back(t);
    };
  bool held_seen = false;
  for (size_t k = 0; k < ids.size(); ++k) {
    if (corners[k].size() != 4) {continue;}
    std::vector<cv::Point2f> n;
    std::string why;
    if (!pw::undistort_corners(cam, corners[k], n, &why) || n.size() != 4) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist_tags] %s → 안 냄", why.c_str());
      return;
    }
    add(ids[k], corners[k], n, false);
    if (puck_ids_.count(ids[k]) && pw::tag_near(n, puck_tag_m_, held_max_m_)) {held_seen = true;}
  }
  // 쥔 퍽은 늘 화면 오른쪽 아래 끝에 걸쳐 보임 (손목 카메라가 손과 같이 움직임) → 흰 테두리가 잘리면 AprilTag 3 가 놓침
  //   (화면 2 장 중 1 장) → OpenCV aruco 로 한 번 더 (퍽 태그만). 대비 올린 (CLAHE) 영상에서는 그 장도 못 찾아서 원본으로
  //   영역 검출 장은 쥔 퍽 영역만 (영역이 화면 밖이면 안 함)
  if (pw::TagReader::have_apriltag() && !held_seen && (!use_roi || held_roi.area() > 0)) {
    std::vector<std::vector<cv::Point2f>> c2;
    std::vector<int> i2;
    if (use_roi) {
      pw::TagReader::detect_aruco(gray(held_roi), c2, i2);
      const cv::Point2f off(static_cast<float>(held_roi.x), static_cast<float>(held_roi.y));
      for (auto & c : c2) {for (auto & p : c) {p += off;}}
    } else {
      pw::TagReader::detect_aruco(gray, c2, i2);
    }
    for (size_t k = 0; k < i2.size(); ++k) {
      std::vector<cv::Point2f> n;
      if (puck_ids_.count(i2[k]) && c2[k].size() == 4 && pw::undistort_corners(cam, c2[k], n) && n.size() == 4 &&
        pw::tag_near(n, puck_tag_m_, held_max_m_))
      {
        add(i2[k], c2[k], n, true);
        break;
      }
    }
  }
  pub_->publish(out);
  ms_sum_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  ++runs_;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_report_).count() >= 5.0) {
    if (roi_on_) {
      RCLCPP_INFO(node_.get_logger(), "[wrist_tags] 5 s: 장당 %.1f ms × %d (영역 %d · 전체 %d) · 확인 장 영역 밖 태그 %d / %d",
        ms_sum_ / std::max(runs_, 1), runs_, roi_runs_, full_runs_, outside_, checked_);
      if (outside_ > 0) {
        RCLCPP_WARN(node_.get_logger(), "[wrist_tags] 영역 밖에서 태그 %d 개 → 영역 검출이면 놓침 (roi_cell_half_m · roi_held_half_m · TF 확인)",
          outside_);
      }
    } else {
      RCLCPP_INFO(node_.get_logger(), "[wrist_tags] 5 s: 장당 %.1f ms × %d", ms_sum_ / std::max(runs_, 1), runs_);
    }
    ms_sum_ = 0.0;
    runs_ = roi_runs_ = full_runs_ = outside_ = checked_ = 0;
    last_report_ = now;
  }
}

bool WristTagStage::make_rois(
  const builtin_interfaces::msg::Time & stamp, const cv::Matx33d & K, const cv::Size & img, std::vector<cv::Rect> & rois,
  cv::Rect & held)
{
  rois.clear();
  held = cv::Rect();
  try {
    // 칸 중심은 고정 TF → 처음 한 번 (base_frame 기준 xy 만 씀, 높이는 roi_floor_z)
    if (cells_base_.empty()) {
      for (const auto & f : cell_frames_) {
        const auto T = tf_->lookupTransform(base_frame_, f, tf2::TimePointZero);
        cells_base_.emplace_back(T.transform.translation.x, T.transform.translation.y, 0.0);
      }
    }
    const auto cam_base = tf_->lookupTransform(frame_id_, base_frame_, stamp, rclcpp::Duration::from_seconds(tf_timeout_s_));
    for (const auto & c : cells_base_) {
      std::vector<cv::Point3d> pts;
      for (double z : {floor_z_, floor_z_ + cell_top_m_}) {
        for (double dx : {-cell_half_m_, cell_half_m_}) {
          for (double dy : {-cell_half_m_, cell_half_m_}) {pts.push_back(apply(cam_base, {c.x + dx, c.y + dy, z}));}
        }
      }
      rois.push_back(pw::project_box(pts, K, roi_pad_px_));
    }
    // 쥔 퍽: 손끝과 카메라가 같은 손목에 붙어 있음 → 촬영 시각 무관 (최신)
    const auto cam_held = tf_->lookupTransform(frame_id_, held_frame_, tf2::TimePointZero);
    std::vector<cv::Point3d> pts;
    for (double dx : {-held_half_m_, held_half_m_}) {
      for (double dy : {-held_half_m_, held_half_m_}) {
        for (double dz : {-held_half_m_, held_half_m_}) {pts.push_back(apply(cam_held, {dx, dy, dz}));}
      }
    }
    held = pw::project_box(pts, K, roi_pad_px_) & cv::Rect(cv::Point(0, 0), img);
    if (held.width < 8 || held.height < 8) {held = cv::Rect();}
    rois.push_back(held);
  } catch (const tf2::TransformException & e) {
    cells_base_.clear();
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wrist_tags] 영역 TF 없음 → 전체 영상: %s", e.what());
    return false;
  }
  return pw::merge_rois(rois, img, roi_max_frac_);
}

}  // namespace perception
