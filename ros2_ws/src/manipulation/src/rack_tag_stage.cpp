// rack_tag_stage.cpp — 손목 태그로 적재함 칸 · 칸 안 퍽 · 쥔 퍽 위치 재기 구현, 설명은 rack_tag_stage.hpp

#include "rack_tag_stage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

#include "tf2/exceptions.hpp"

namespace manipulation
{

namespace
{
// camera_info 를 아직 못 받았을 때 초점 거리 [px] 를 태그 모서리로: 화소 변 길이 / 정규 좌표 변 길이
//   (perception 이 같은 장에서 둘 다 냄, 왜곡이 작은 가운데 태그면 1–2 % 안). PnP 두 해 판정 (재투영 RMS px) 에만 씀. 못 구하면 0
double focal_from_corners(const std::vector<cv::Point2f> & px, const std::vector<cv::Point2f> & n)
{
  if (px.size() != 4 || n.size() != 4) {return 0.0;}
  double sp = 0.0, sn = 0.0;
  for (int j = 0; j < 4; ++j) {
    sp += cv::norm(px[(j + 1) % 4] - px[j]);
    sn += cv::norm(n[(j + 1) % 4] - n[j]);
  }
  return sn > 1e-9 && std::isfinite(sp / sn) ? sp / sn : 0.0;
}
}  // namespace

RackTagStage::RackTagStage(rclcpp::Node & node, tf2_ros::Buffer & tf, rclcpp::CallbackGroup::SharedPtr group)
: node_(node), tf_(tf)
{
  const auto tags_topic = node.declare_parameter<std::string>("rack_tags.tags_topic", "/perception/wrist/tags");
  const auto info_topic = node.declare_parameter<std::string>("rack_tags.info_topic", "/camera/camera/color/camera_info");
  enable_service_ = node.declare_parameter<std::string>("rack_tags.enable_service", "/perception/wrist_tags/enable");
  camera_frame_ = node.declare_parameter<std::string>("rack_tags.camera_frame", "");   // "" = 태그 header 그대로 (perception 이 URDF 이름으로 냄)
  base_frame_ = node.declare_parameter<std::string>("rack_tags.base_frame", "base_link");
  cfg_.plane_z = node.declare_parameter<double>("rack_tags.floor_z", 0.1325);
  cfg_.tag_size_m = node.declare_parameter<double>("rack_tags.tag_size_m", 0.026);
  cfg_.size_tol = node.declare_parameter<double>("rack_tags.size_tol", 0.15);
  cfg_.min_view_cos = node.declare_parameter<double>("rack_tags.min_view_cos", 0.94);   // 수직에서 20° 안 (rack_tags.hpp)
  // 칸 이름 · 표 위치 = control_master 의 rack_cell_* (config/rack_cells.yaml, robot.launch 가 넣음), 칸 태그 ID 는 같은 칸 순서
  names_ = node.get_parameter("rack_cell_names").as_string_array();
  const auto xs = node.get_parameter("rack_cell_x").as_double_array();
  const auto ys = node.get_parameter("rack_cell_y").as_double_array();
  const auto tag_ids = node.declare_parameter<std::vector<int64_t>>("rack_tags.cell_ids", std::vector<int64_t>{});
  mismatch_m_ = node.declare_parameter<double>("rack_tags.mismatch_m", 0.02);
  const auto puck_ids = node.declare_parameter<std::vector<int64_t>>("rack_tags.puck_ids", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  const double puck_tag = node.declare_parameter<double>("rack_tags.puck_tag_size_m", 0.0153);   // 퍽 윗면 태그 검은 사각 (인쇄 실측 15.2~15.4 mm)
  const double puck_h = node.declare_parameter<double>("rack_tags.puck_height_m", 0.06);         // 칸 바닥에 선 퍽 윗면 높이
  // 퍽은 크기 허용을 좁게: 턱에 걸려 들뜬 퍽은 카메라에 가까워 크게 보임 (약 110 mm 거리에서 6 % = 높이 약 7 mm)
  const double puck_size_tol = node.declare_parameter<double>("rack_tags.puck_size_tol", 0.06);
  pose_tol_m_ = node.declare_parameter<double>("rack_tags.pose_tol_m", 0.003);                 // 같은 관찰 자세로 보는 카메라 위치 차
  pose_tol_rad_ = node.declare_parameter<double>("rack_tags.pose_tol_deg", 1.0) * M_PI / 180.0;
  wall_gap_m_ = node.declare_parameter<double>("rack_tags.wall_gap_m", 0.0025);                // 퍽 ↔ 컵 벽 틈 (이보다 어긋나면 벽에 닿았을 수 있음)
  max_rate_hz_ = node.declare_parameter<double>("rack_tags.max_rate_hz", 5.0);                 // perception wrist_tags.max_hz 와 같게 (장 사이 판정)
  still_m_ = node.declare_parameter<double>("rack_tags.still_m", 0.001);
  still_rad_ = node.declare_parameter<double>("rack_tags.still_deg", 0.3) * M_PI / 180.0;
  samples_ = static_cast<int>(node.declare_parameter<int>("rack_tags.samples", 60));   // 칸 하나를 자세 여러 개에서 봐도 자세마다 남게
  min_samples_ = static_cast<int>(node.declare_parameter<int>("rack_tags.min_samples", 5));
  min_span_m_ = node.declare_parameter<double>("rack_tags.min_span_m", 0.06);
  tf_timeout_s_ = node.declare_parameter<double>("rack_tags.tf_timeout_s", 0.05);
  if (names_.empty() || xs.size() != names_.size() || ys.size() != names_.size() || tag_ids.size() != names_.size()) {
    throw std::runtime_error("rack_cell_names · rack_cell_x · rack_cell_y · rack_tags.cell_ids 가 비었거나 길이가 다름 (config/rack_cells.yaml, robot.launch 가 넣음)");
  }
  if (!(max_rate_hz_ > 0.0) || samples_ < 1 || min_samples_ < 1 || min_samples_ > samples_ || !(cfg_.tag_size_m > 0.0)) {
    throw std::runtime_error("rack_tags.max_rate_hz · samples · min_samples · tag_size_m 는 0 보다 커야 하고 min_samples <= samples");
  }
  pivot_.setZero();
  for (size_t i = 0; i < names_.size(); ++i) {
    cells_.emplace_back(xs[i], ys[i]);
    pivot_ += cells_.back();
    if (!index_.emplace(static_cast<int>(tag_ids[i]), i).second) {
      throw std::runtime_error("rack_tags.cell_ids 에 같은 태그 ID 가 두 번: " + std::to_string(tag_ids[i]));
    }
  }
  pivot_ /= static_cast<double>(cells_.size());
  got_.resize(cells_.size());
  puck_got_.resize(cells_.size());
  cfg_puck_ = cfg_;
  cfg_puck_.tag_size_m = puck_tag;
  cfg_puck_.size_tol = puck_size_tol;
  cfg_puck_.plane_z = cfg_.plane_z + puck_h;
  if (!puck_ids.empty() && samples_ < 3) {
    throw std::runtime_error("퍽 비교는 같은 자세 표본 3 개 이상이 필요 → rack_tags.samples >= 3");
  }
  for (const auto id : puck_ids) {
    if (index_.count(static_cast<int>(id)) || !puck_ids_.insert(static_cast<int>(id)).second) {
      throw std::runtime_error("rack_tags.puck_ids 가 칸 태그 ID 와 겹치거나 두 번: " + std::to_string(id));
    }
  }

  pub_ = node.create_publisher<interfaces::msg::DetectionArray>("/rack_tags/detections", 10);
  held_max_dist_m_ = node.declare_parameter<double>("rack_tags.held_max_dist_m", 0.15);   // 카메라에서 이보다 가까운 퍽 태그 = 쥔 퍽
  wrist_frame_ = node.declare_parameter<std::string>("rack_tags.wrist_frame", "wrist_link");
  node.declare_parameter<bool>("rack_tags.held_fresh", false);   // 정지·재투영 검사한 별도 측정 (/held_puck/tag_fresh), /held_puck/tag 는 그대로
  held_fresh_pub_ = node.create_publisher<interfaces::msg::DetectionArray>("/held_puck/tag_fresh", 10);
  held_pub_ = node.create_publisher<interfaces::msg::DetectionArray>("/held_puck/tag", 10);
  // 퍽 윗면 태그 PnP 자세 (넣은 뒤 보기): 거리와 상관없이 퍽 태그마다 카메라 광학 프레임으로 (control_master 가 drift 를 되돌린 뒤 base_link 로)
  //   관찰 자세에서 칸 안 퍽 윗면은 카메라에서 0.136 – 0.144 m 라 쥔 퍽 (held_max_dist_m 0.15) 과 칸 퍽 (평면, 높이 모름) 사이를 오감 → 따로 냄
  pnp_pub_ = node.create_publisher<interfaces::msg::DetectionArray>(
    node.declare_parameter<std::string>("rack_tags.puck_pnp_topic", "/rack_tags/puck_pnp"), 10);
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group;
  info_sub_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::SensorDataQoS(),   // best_effort: 어느 발행 설정과도 맞음
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {info_ = m;}, opt);
  tags_sub_ = node.create_subscription<interfaces::msg::WristTagArray>(tags_topic, rclcpp::QoS(10),
      [this](interfaces::msg::WristTagArray::ConstSharedPtr m) {on_tags(*m);}, opt);
  report_srv_ = node.create_service<std_srvs::srv::Trigger>("~/rack_tags/report",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->message = report();
        res->success = true;
        RCLCPP_INFO(node_.get_logger(), "\n%s", res->message.c_str());
      }, rclcpp::ServicesQoS(), group);
  dump_path_ = node.declare_parameter<std::string>("rack_tags.dump_path",
      std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/rack_tag_samples.csv");
  dump_srv_ = node.create_service<std_srvs::srv::Trigger>("~/rack_tags/dump",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::ofstream f(dump_path_);
        if (!f) {res->success = false; res->message = "못 씀: " + dump_path_; return;}
        f << "t,tag,cam_px,cam_py,cam_pz,cam_qx,cam_qy,cam_qz,cam_qw,u0,v0,u1,v1,u2,v2,u3,v3\n";
        f.precision(9);
        for (const auto & r : raw_) {
          f << r.t << "," << r.tag << "," << r.p.x() << "," << r.p.y() << "," << r.p.z() << "," << r.q.x() << "," << r.q.y() << ","
            << r.q.z() << "," << r.q.w();
          for (const auto & c : r.cn) {f << "," << c.x() << "," << c.y();}
          f << "\n";
        }
        res->success = true;
        res->message = std::to_string(raw_.size()) + " 줄 → " + dump_path_;
      }, rclcpp::ServicesQoS(), group);
  reset_srv_ = node.create_service<std_srvs::srv::Trigger>("~/rack_tags/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        for (auto & g : got_) {g.clear();}
        for (auto & g : puck_got_) {g.clear();}
        wrong_.clear();
        raw_.clear();
        res->success = true;
        res->message = "모은 값 지움";
      }, rclcpp::ServicesQoS(), group);
  if (!enable_service_.empty()) {
    enable_cli_ = node.create_client<std_srvs::srv::SetBool>(enable_service_, rclcpp::ServicesQoS(), group);
  }
  RCLCPP_INFO(node.get_logger(), "적재함 태그: 칸 %zu 개, 바닥 z %.4f m, 태그 %.0f mm, 받기 %s, 켜기 %s",
    cells_.size(), cfg_.plane_z, cfg_.tag_size_m * 1000.0, tags_topic.c_str(), enable_service_.empty() ? "(안 부름)" : enable_service_.c_str());
}

void RackTagStage::want(bool on, double now)
{
  if (!enable_cli_) {return;}
  std::lock_guard<std::mutex> lk(mtx_);
  want_ = on;
  if (pending_) {
    if (now - sent_s_ < 2.0) {return;}
    enable_cli_->prune_pending_requests();   // 늦은 응답은 gen 으로 무시
    pending_ = false;
  }
  // perception 을 다시 켜면 태그 단계는 꺼진 채 시작 (wrist_tags.enabled false) 인데 여기는 켰다고 알고 있음 → 1 s 마다 켜기 서비스가 살아 있는지 보고,
  //   없어졌으면 known_ 을 지움 → 다시 생기면 지금 원하는 값을 다시 보냄
  if (known_ && now - ready_check_s_ >= 1.0) {
    ready_check_s_ = now;
    if (!enable_cli_->service_is_ready()) {known_ = false;}
  }
  if (known_ && on_ == on) {return;}
  if (now - last_try_s_ < (on ? 0.2 : 1.0)) {return;}
  last_try_s_ = now;
  if (!enable_cli_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10000, "적재함 태그: %s 없음 (perception 꺼짐?) → 손목 태그 못 받음",
      enable_service_.c_str());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;
  const int gen = ++gen_;
  pending_ = true;
  sent_s_ = now;
  enable_cli_->async_send_request(req, [this, on, gen](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (gen != gen_) {return;}
      pending_ = false;
      if (f.get()->success) {
        known_ = true;
        on_ = on;
      }
    });
}

void RackTagStage::on_tags(const interfaces::msg::WristTagArray & msg)
{
  const rclcpp::Time stamp(msg.header.stamp, node_.get_clock()->get_clock_type());
  const auto info = info_;
  const std::string cam = camera_frame_.empty() ? msg.header.frame_id : camera_frame_;
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_.lookupTransform(base_frame_, cam, stamp, rclcpp::Duration::from_seconds(tf_timeout_s_));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "TF %s ← %s 없음: %s", base_frame_.c_str(), cam.c_str(), e.what());
    return;
  }
  const bool gap_ok = have_last_ && stamp >= last_ && (stamp - last_).seconds() < 3.0 / max_rate_hz_;
  have_last_ = true;
  last_ = stamp;

  const auto & r = tf.transform.rotation;
  const auto & t = tf.transform.translation;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  // 앞 장과 카메라 자리가 거의 같으면 멈춘 것 (영상이 띄엄띄엄 오면 그 사이를 모르니 안 셈)
  const bool still = gap_ok && (T.translation() - prev_cam_.translation()).norm() < still_m_ &&
    Eigen::AngleAxisd(prev_cam_.linear().transpose() * T.linear()).angle() < still_rad_;
  prev_cam_ = T;

  interfaces::msg::DetectionArray out;
  out.header.stamp = msg.header.stamp;
  out.header.frame_id = base_frame_;
  const bool fresh_enabled = node_.get_parameter("rack_tags.held_fresh").as_bool();   // 장마다 읽음 (실행 중 바꿀 수 있음)
  const double focal = info ? std::max(info->k[0], info->k[4]) : 0.0;
  bool held_seen = false;
  const bool empty_scan = empty_scan_.load();   // 빈손 스캔 중 = 손이 비었음 → 쥔 퍽 판정 안 함 (칸 안 퍽 윗면도 카메라에서 0.15 m 안)
  interfaces::msg::DetectionArray pnp;   // 퍽 윗면 태그 PnP (카메라 광학 프레임, 넣은 뒤 보기)
  pnp.header.stamp = msg.header.stamp;
  pnp.header.frame_id = cam;
  auto corners_of = [](const interfaces::msg::WristTag & tg, std::vector<cv::Point2f> & px, std::vector<cv::Point2f> & n) {
      px.clear();
      n.clear();
      for (int j = 0; j < 4; ++j) {
        px.emplace_back(tg.corners_px[2 * j], tg.corners_px[2 * j + 1]);
        n.emplace_back(static_cast<float>(tg.corners_norm[2 * j]), static_cast<float>(tg.corners_norm[2 * j + 1]));
      }
    };
  {   // 받침 다시 보기용 원본 (본 검출 · 다시 찾은 것 모두, 크기 · 쥔 퍽 검사 전)
    std::lock_guard<std::mutex> lk(seen_mtx_);
    const double ts = stamp.seconds();
    for (const auto & tg : msg.tags) {
      if (!puck_ids_.count(tg.id)) {continue;}
      PuckTagSeen e;
      e.stamp = ts;
      e.id = tg.id;
      e.cam = cam;
      e.T = T;
      for (int j = 0; j < 4; ++j) {e.cn[j] = Eigen::Vector2d(tg.corners_norm[2 * j], tg.corners_norm[2 * j + 1]);}
      e.rescue = tg.rescue;
      recent_puck_tags_.push_back(e);
    }
    while (!recent_puck_tags_.empty() && (recent_puck_tags_.front().stamp < ts - 2.0 || recent_puck_tags_.size() > 200)) {recent_puck_tags_.pop_front();}
  }
  for (const auto & tg : msg.tags) {
    if (tg.rescue) {continue;}   // 다시 찾은 쥔 퍽 태그는 아래 (본 검출에 쥔 퍽이 없을 때만)
    const int id = tg.id;
    const bool puck = puck_ids_.count(id) > 0;
    const auto it = index_.find(id);
    if (!puck && it == index_.end()) {
      continue;   // 모르는 태그
    }
    std::vector<cv::Point2f> corners, n;
    corners_of(tg, corners, n);
    std::array<Eigen::Vector2d, 4> cn;
    for (int j = 0; j < 4; ++j) {cn[j] = Eigen::Vector2d(tg.corners_norm[2 * j], tg.corners_norm[2 * j + 1]);}
    if (puck) {add_pnp(pnp, id, n, corners, T, focal > 0.0 ? focal : focal_from_corners(corners, n));}
    Eigen::Vector3d held;
    Eigen::Matrix3d held_R;
    double held_rms;
    if (puck && !empty_scan && held_tag(n, cfg_puck_.tag_size_m, held, held_R, held_rms)) {
      publish_held(msg.header.stamp, cam, id, held, held_R,
        fresh_enabled && still && std::isfinite(held_rms) && focal > 0.0 && held_rms * focal <= 1.0);
      held_seen = true;
      continue;
    }
    if (still && raw_.size() < 20000) {   // 원본 (크기 · 각도 검사 전) — ~/rack_tags/dump
      raw_.push_back({stamp.seconds(), id, T.translation(), Eigen::Quaterniond(T.linear()), cn});
    }
    const TagOnPlane hit = tag_on_plane(cn, T, puck ? cfg_puck_ : cfg_);
    if (puck) {
      on_puck(id, hit, corners, still, T, out);
      continue;
    }
    const size_t i = it->second;
    if (!hit.ok) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "%s 태그 안 씀: %s (한 변 %.1f mm)",
        names_[i].c_str(), hit.why, hit.side_m * 1000.0);
      continue;
    }
    const size_t near = nearest_cell(hit.center);
    if (near != i && (hit.center - cells_[i]).norm() > mismatch_m_) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "태그 ID %d: %s 자리가 아니라 %s 자리에 있음 → 안 씀 "
        "(~/rack_tags/report 에 고칠 ids)", id, names_[i].c_str(), names_[near].c_str());
      wrong_[id] = near;
      continue;
    }
    out.detections.push_back(detection(out.header, "rack_tag", names_[i], id, hit, cfg_.plane_z, corners, still));
    if (still) {
      auto & g = got_[i];
      g.push_back({hit.center, T.translation(), Eigen::Quaterniond(T.linear()), hit.side_m});
      while (static_cast<int>(g.size()) > samples_) {g.pop_front();}
      if (static_cast<int>(g.size()) == min_samples_) {
        const Eigen::Vector2d m = cell_median(i);
        RCLCPP_INFO(node_.get_logger(), "%s 잼: (%.1f, %.1f) mm, 표와 차 dx %+.1f · dy %+.1f mm",
          names_[i].c_str(), m.x() * 1000.0, m.y() * 1000.0, (m.x() - cells_[i].x()) * 1000.0, (m.y() - cells_[i].y()) * 1000.0);
      }
    }
  }
  pub_->publish(out);
  if (!pnp.detections.empty()) {pnp_pub_->publish(pnp);}
  // 쥔 퍽은 늘 화면 오른쪽 아래 끝에 걸쳐 보임 → 본 검출 (AprilTag 3) 이 놓친 장은 perception 이 원본 영상 aruco 로 다시 찾아 rescue 로 붙임
  if (!held_seen && !empty_scan) {
    for (const auto & tg : msg.tags) {
      if (!tg.rescue || !puck_ids_.count(tg.id)) {continue;}
      std::vector<cv::Point2f> corners, n;
      corners_of(tg, corners, n);
      Eigen::Vector3d held;
      Eigen::Matrix3d held_R;
      double held_rms;
      if (held_tag(n, cfg_puck_.tag_size_m, held, held_R, held_rms)) {
        publish_held(msg.header.stamp, cam, tg.id, held, held_R,
          fresh_enabled && still && std::isfinite(held_rms) && focal > 0.0 && held_rms * focal <= 1.0);
        break;
      }
    }
  }
}

bool RackTagStage::held_tag(const std::vector<cv::Point2f> & n, double size, Eigen::Vector3d & c, Eigen::Matrix3d & R, double & rms) const
{
  rms = std::numeric_limits<double>::infinity();
  const float h = static_cast<float>(size / 2.0);
  const std::vector<cv::Point3f> obj{{-h, h, 0.0f}, {h, h, 0.0f}, {h, -h, 0.0f}, {-h, -h, 0.0f}};   // SOLVEPNP_IPPE_SQUARE 순서
  double best = 1e9;
  for (int order = 0; order < 2; ++order) {
    std::vector<cv::Point2f> img = n;
    if (order == 1) {std::reverse(img.begin() + 1, img.end());}
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(obj, img, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {continue;}
    std::vector<cv::Point2f> back;
    cv::projectPoints(obj, rvec, tvec, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), back);
    double e = 0.0, e2 = 0.0;
    for (int j = 0; j < 4; ++j) {const double v=cv::norm(back[j] - img[j]); e += v; e2 += v*v;}
    if (e < best && tvec.at<double>(2) > 0.0) {
      best = e;
      rms = std::sqrt(e2/4.0);   // 해 선택은 합 오차, RMS 는 tag_fresh 검사에만 씀
      c = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
      cv::Mat Rm;
      cv::Rodrigues(rvec, Rm);
      for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {R(i, j) = Rm.at<double>(i, j);}}
    }
  }
  return best < 1e8 && c.z() > 0.02 && c.norm() < held_max_dist_m_;
}

// 퍽 윗면 태그 자세 (넣은 뒤 보기): IPPE 두 해 (모서리 순서 둘 중 재투영 오차가 작은 쪽) 중 오차가 작은 해.
//   두 해 오차가 비슷하면 (작은 태그를 거의 수직으로 볼 때 흔함: 15 mm 태그 · 0.14 m 에서 기울기 부호가 갈림) 윗면 법선이 base 위쪽에 더 가까운 해
//   → 기울기를 크게 잘못 읽어 멀쩡한 퍽을 다시 넣지 않게 (보수적). 그렇게 고른 장은 confidence 0.5 (아니면 1.0)
//   위치 = 태그 중심 · 자세 z = 윗면 법선 (카메라 쪽) · x = 태그 x 축 (모서리 0 → 1), 카메라 광학 프레임. label = 가장 가까운 칸 (mismatch_m 안, 아니면 "")
void RackTagStage::add_pnp(interfaces::msg::DetectionArray & out, int id, const std::vector<cv::Point2f> & n, const std::vector<cv::Point2f> & px,
  const Eigen::Isometry3d & T, double focal) const
{
  const float h = static_cast<float>(cfg_puck_.tag_size_m / 2.0);
  const std::vector<cv::Point3f> obj{{-h, h, 0.0f}, {h, h, 0.0f}, {h, -h, 0.0f}, {-h, -h, 0.0f}};   // SOLVEPNP_IPPE_SQUARE 순서
  struct Sol {Eigen::Vector3d c; Eigen::Matrix3d R; double e;};
  std::vector<Sol> best;   // 오차 작은 순서의 두 해 (같은 모서리 순서)
  for (int order = 0; order < 2; ++order) {
    std::vector<cv::Point2f> img = n;
    if (order == 1) {std::reverse(img.begin() + 1, img.end());}
    std::vector<cv::Mat> rv, tv;
    cv::Mat err;
    int k = 0;
    try {
      k = cv::solvePnPGeneric(obj, img, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rv, tv, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(),
        cv::noArray(), err);
    } catch (const cv::Exception &) {
      continue;
    }
    cv::Mat e64;   // 재투영 RMS: 점이 float 라 CV_32F 로 옴 → double 로 바꿔 읽음
    if (!err.empty()) {err.convertTo(e64, CV_64F);}
    std::vector<Sol> s;
    for (int i = 0; i < k && i < static_cast<int>(rv.size()); ++i) {
      const double e = e64.total() > static_cast<std::size_t>(i) ? e64.at<double>(i) : std::numeric_limits<double>::quiet_NaN();
      if (tv[i].at<double>(2) <= 0.02 || !std::isfinite(e)) {continue;}
      cv::Mat Rm;
      cv::Rodrigues(rv[i], Rm);
      Sol x{Eigen::Vector3d(tv[i].at<double>(0), tv[i].at<double>(1), tv[i].at<double>(2)), Eigen::Matrix3d::Identity(), e};
      for (int r = 0; r < 3; ++r) {for (int c = 0; c < 3; ++c) {x.R(r, c) = Rm.at<double>(r, c);}}
      // 보이는 태그는 앞면이 카메라 쪽 (법선 z < 0). 거울 순서는 오차가 똑같고 법선만 뒤집힘 → 검출기 모서리 순서와 상관없이 앞면 해만
      if (x.R(2, 2) >= 0.0) {continue;}
      s.push_back(x);
    }
    std::sort(s.begin(), s.end(), [](const Sol & a, const Sol & b) {return a.e < b.e;});
    if (!s.empty() && (best.empty() || s.front().e < best.front().e)) {best = s;}
  }
  if (best.empty()) {return;}
  std::size_t pick = 0;
  bool ambiguous = false;
  if (best.size() >= 2 && focal > 0.0) {   // 재투영 RMS [px] · 윗면 법선 base z (rack_tags.hpp pick_pnp_solution)
    pick = static_cast<std::size_t>(pick_pnp_solution(best[0].e * focal, best[1].e * focal, (T.linear() * best[0].R.col(2)).z(),
      (T.linear() * best[1].R.col(2)).z(), ambiguous));
  }
  const Sol & s = best[pick];
  if (!s.c.allFinite() || !s.R.allFinite()) {return;}
  interfaces::msg::Detection d;
  d.header = out.header;
  d.kind = "puck_pnp";
  d.id = std::to_string(id);
  const Eigen::Vector3d cb = T * s.c;
  const size_t near = nearest_cell(cb.head<2>());
  if ((cb.head<2>() - cells_[near]).norm() <= mismatch_m_) {d.label = names_[near];}
  d.confidence = ambiguous ? 0.5f : 1.0f;
  const Eigen::Quaterniond q(s.R);
  d.pose.position.x = s.c.x();
  d.pose.position.y = s.c.y();
  d.pose.position.z = s.c.z();
  d.pose.orientation.x = q.x();
  d.pose.orientation.y = q.y();
  d.pose.orientation.z = q.z();
  d.pose.orientation.w = q.w();
  d.yaw_known = true;
  d.center_from_tag = true;
  float x0 = px[0].x, y0 = px[0].y, x1 = x0, y1 = y0;
  for (const auto & p : px) {
    x0 = std::min(x0, p.x);
    y0 = std::min(y0, p.y);
    x1 = std::max(x1, p.x);
    y1 = std::max(y1, p.y);
  }
  d.bbox = {x0, y0, x1, y1};
  out.detections.push_back(d);
}

void RackTagStage::publish_held(const builtin_interfaces::msg::Time & stamp, const std::string & cam, int id, const Eigen::Vector3d & c,
  const Eigen::Matrix3d & Rc, bool fresh)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_.lookupTransform(wrist_frame_, cam, tf2::TimePointZero);   // 고정 관절 (카메라 ↔ 손목)
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "TF %s ← %s 없음: %s", wrist_frame_.c_str(), cam.c_str(), e.what());
    return;
  }
  const auto & r = tf.transform.rotation;
  const auto & t = tf.transform.translation;
  const Eigen::Quaterniond qw = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
  const Eigen::Vector3d w = qw * c + Eigen::Vector3d(t.x, t.y, t.z);
  const Eigen::Quaterniond qt(qw.toRotationMatrix() * Rc);
  interfaces::msg::DetectionArray m;
  m.header.stamp = stamp;
  m.header.frame_id = wrist_frame_;
  interfaces::msg::Detection d;
  d.header = m.header;
  d.kind = "held_tag";
  d.id = std::to_string(id);
  d.confidence = 1.0;
  d.pose.position.x = w.x();
  d.pose.position.y = w.y();
  d.pose.position.z = w.z();
  d.pose.orientation.x = qt.x();
  d.pose.orientation.y = qt.y();
  d.pose.orientation.z = qt.z();
  d.pose.orientation.w = qt.w();
  m.detections.push_back(d);
  held_pub_->publish(m);
  if (fresh && w.allFinite() && qt.coeffs().allFinite()) {
    m.detections.front().kind = "held_tag_fresh";
    held_fresh_pub_->publish(m);
  }
  RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "쥔 퍽 태그 %d: 카메라에서 %.0f mm · 손목 좌표 (%.1f, %.1f, %.1f) mm", id,
    c.norm() * 1000.0, w.x() * 1000.0, w.y() * 1000.0, w.z() * 1000.0);
}

interfaces::msg::Detection RackTagStage::detection(const std_msgs::msg::Header & h, const char * kind, const std::string & label, int id,
  const TagOnPlane & hit, double z, const std::vector<cv::Point2f> & corners, bool still)
{
  interfaces::msg::Detection d;
  d.header = h;
  d.kind = kind;
  d.label = label;
  d.id = std::to_string(id);
  d.confidence = still ? 1.0f : 0.5f;   // 0.5 = 팔이 움직이는 중이라 수 mm 틀릴 수 있음
  d.pose.position.x = hit.center.x();
  d.pose.position.y = hit.center.y();
  d.pose.position.z = z;
  d.pose.orientation.z = std::sin(hit.yaw / 2.0);
  d.pose.orientation.w = std::cos(hit.yaw / 2.0);
  d.yaw_known = true;
  float x0 = corners[0].x, y0 = corners[0].y, x1 = x0, y1 = y0;
  for (const auto & p : corners) {
    x0 = std::min(x0, p.x);
    y0 = std::min(y0, p.y);
    x1 = std::max(x1, p.x);
    y1 = std::max(y1, p.y);
  }
  d.bbox = {x0, y0, x1, y1};
  return d;
}

void RackTagStage::on_puck(int id, const TagOnPlane & hit, const std::vector<cv::Point2f> & corners, bool still, const Eigen::Isometry3d & T,
  interfaces::msg::DetectionArray & out)
{
  if (!hit.ok) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "퍽 태그 %d 안 씀: %s (한 변 %.1f mm, 칸 턱에 걸려 기울면 높이가 달라 크기가 안 맞음)",
      id, hit.why, hit.side_m * 1000.0);
    return;
  }
  const size_t j = nearest_cell(hit.center);
  if ((hit.center - cells_[j]).norm() > mismatch_m_) {return;}
  out.detections.push_back(detection(out.header, "rack_puck", names_[j], id, hit, cfg_puck_.plane_z, corners, still));
  if (!still) {return;}
  auto & g = puck_got_[j];
  g.push_back({hit.center, T.translation(), Eigen::Quaterniond(T.linear()), hit.side_m});
  while (static_cast<int>(g.size()) > samples_) {g.pop_front();}
  if (static_cast<int>(g.size()) == min_samples_) {
    Eigen::Vector2d d;
    int pairs = 0;
    if (puck_minus_cell(j, d, pairs)) {
      d *= 1000.0;
      RCLCPP_INFO(node_.get_logger(), "%s 안 퍽 (태그 %d): 칸 중심에서 dx %+.1f · dy %+.1f mm → 경로 옮길 양 %+.1f, %+.1f mm%s",
        names_[j].c_str(), id, d.x(), d.y(), -d.x(), -d.y(), d.norm() > wall_gap_m_ * 1000.0 ? " (컵 벽에 닿았을 수 있음)" : "");
    } else {
      RCLCPP_INFO(node_.get_logger(), "%s 안 퍽 (태그 %d) 잼 — 같은 자세의 빈 칸 값이 없어 차이는 못 냄", names_[j].c_str(), id);
    }
  }
}

std::vector<RackTagStage::PuckTagSeen> RackTagStage::puck_tags_since(double since) const
{
  std::lock_guard<std::mutex> lk(seen_mtx_);
  std::vector<PuckTagSeen> out;
  for (const auto & e : recent_puck_tags_) {
    if (e.stamp > since) {out.push_back(e);}
  }
  return out;
}

size_t RackTagStage::nearest_cell(const Eigen::Vector2d & p) const
{
  size_t best = 0;
  for (size_t j = 1; j < cells_.size(); ++j) {
    if ((p - cells_[j]).norm() < (p - cells_[best]).norm()) {best = j;}
  }
  return best;
}

bool RackTagStage::same_pose(const Sample & a, const Sample & b) const
{
  return (a.cam_p - b.cam_p).norm() < pose_tol_m_ && a.cam_q.angularDistance(b.cam_q) < pose_tol_rad_;
}

Eigen::Vector2d RackTagStage::med(const std::deque<Sample> & g)
{
  std::vector<double> x, y;
  for (const auto & p : g) {
    x.push_back(p.c.x());
    y.push_back(p.c.y());
  }
  return {median(x), median(y)};
}

bool RackTagStage::puck_minus_cell(size_t i, Eigen::Vector2d & d, int & pairs) const
{
  std::vector<double> dx, dy;
  for (const auto & p : puck_got_[i]) {
    std::deque<Sample> m;
    for (const auto & c : got_[i]) {
      if (same_pose(p, c)) {m.push_back(c);}
    }
    if (m.size() < 3) {continue;}
    const Eigen::Vector2d cm = med(m);
    dx.push_back(p.c.x() - cm.x());
    dy.push_back(p.c.y() - cm.y());
  }
  pairs = static_cast<int>(dx.size());
  if (pairs < 3) {return false;}
  d = {median(dx), median(dy)};
  return true;
}

std::string RackTagStage::report() const
{
  std::ostringstream s;
  s << std::fixed << std::setprecision(1);
  s << "칸 · 장 수 · 잰 x, y · 표 x, y · 차 dx, dy · 흩어짐 · 태그 한 변 [mm] (표 = config/rack_cells.yaml, 멈춘 장 " << min_samples_
    << " 개 이상인 칸만. 한 변이 " << cfg_.tag_size_m * 1000.0 << " 에서 벗어나면 칸 바닥 높이 · 카메라 TF 가 틀림)\n";
  std::vector<Eigen::Vector2d> exp, meas;
  std::vector<bool> measured(cells_.size(), false);
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (static_cast<int>(got_[i].size()) < min_samples_) {
      continue;
    }
    const Eigen::Vector2d m = cell_median(i);
    double se = 0.0;
    for (const auto & p : got_[i]) {se += (p.c - m).squaredNorm();}
    const Eigen::Vector2d d = (m - cells_[i]) * 1000.0;
    std::vector<double> sides;
    for (const auto & p : got_[i]) {sides.push_back(p.side);}
    s << "  " << names_[i] << "  " << got_[i].size() << "  (" << m.x() * 1000.0 << ", " << m.y() * 1000.0 << ")  ("
      << cells_[i].x() * 1000.0 << ", " << cells_[i].y() * 1000.0 << ")  " << std::showpos << d.x() << ", " << d.y()
      << std::noshowpos << "  " << std::sqrt(se / static_cast<double>(got_[i].size())) * 1000.0 << "  " << median(sides) * 1000.0 << "\n";
    exp.push_back(cells_[i]);
    meas.push_back(m);
    measured[i] = true;
  }
  bool any_puck = false;
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (static_cast<int>(puck_got_[i].size()) < min_samples_) {continue;}
    if (!any_puck) {
      s << "칸 안 퍽 · 장 수 · 퍽 x, y · 퍽 − 칸 dx, dy · 경로 옮길 양 (칸 − 퍽) [mm] "
        << "(같은 카메라 자세에서 잰 칸 · 퍽끼리 뺌 → 카메라 치우침이 빠짐. 퍽이 컵 벽에 닿으면 약 " << wall_gap_m_ * 1000.0
        << " mm 넘는 어긋남은 못 잼. 카메라 기울기 치우침은 높이 차 60 mm 만큼 남음: 2° 면 약 2 mm)\n";
      any_puck = true;
    }
    const Eigen::Vector2d p = med(puck_got_[i]);
    s << "  " << names_[i] << "  " << puck_got_[i].size() << "  (" << p.x() * 1000.0 << ", " << p.y() * 1000.0 << ")  ";
    Eigen::Vector2d d;
    int pairs = 0;
    if (puck_minus_cell(i, d, pairs)) {
      d *= 1000.0;
      s << std::showpos << d.x() << ", " << d.y() << "  " << -d.x() << "," << -d.y() << std::noshowpos
        << (d.norm() > wall_gap_m_ * 1000.0 ? "  ← 컵 벽에 닿았을 수 있음 (실제로는 더 어긋났을 수 있음)" : "") << "\n";
    } else {
      s << "같은 자세의 빈 칸 값이 없음 → 차이 없음 (빈 칸 스캔 → 넣기 → 같은 스캔, 태그 노드는 그대로)\n";
    }
  }
  if (!wrong_.empty()) {
    // 본 대로 고친 표 (안 본 칸은 그대로) → config/rack_cells.yaml rack_tags.ids 에 붙여 넣기
    std::vector<int> fixed(cells_.size());
    for (const auto & [id, i] : index_) {fixed[i] = id;}
    s << "ID 가 칸과 안 맞는 태그:";
    for (const auto & [id, near] : wrong_) {
      s << " ID " << id << " → " << names_[near] << " 자리";
      fixed[near] = id;
    }
    // 본 것만 반영해 ID 가 겹치면 (자리를 바꾼 짝을 다 못 봄) 붙여 넣을 표를 안 냄 → 노드가 겹친 ID 로 안 뜨는 일 없게
    std::map<int, int> count;
    for (int id : fixed) {++count[id];}
    bool unique = true;
    for (const auto & [id, k] : count) {unique = unique && k == 1;}
    if (unique) {
      s << "\n  고칠 ids (config/rack_cells.yaml rack_tags.ids 에 붙여 넣기): {";
      for (size_t i = 0; i < cells_.size(); ++i) {s << (i ? ", " : "") << names_[i] << ": " << fixed[i];}
      s << "}\n";
    } else {
      s << "\n  자리를 바꾼 짝 태그를 다 못 봄 → 다른 칸 태그도 보이게 한 뒤 다시 report\n";
    }
  }
  if (exp.empty()) {
    s << "잰 칸 없음 (팔을 멈춘 채 칸 태그가 보이게)\n";
    return s.str();
  }
  const RackFit f = fit_rack(exp, meas, pivot_, min_span_m_);
  s << "적재함 전체 이동 (" << f.n << " 칸, 중심 (" << pivot_.x() * 1000.0 << ", " << pivot_.y() * 1000.0 << ") mm 기준): "
    << std::showpos << "dx " << f.dx * 1000.0 << " · dy " << f.dy * 1000.0 << " mm";
  if (f.rotation) {
    s << std::setprecision(2) << " · 회전 " << f.dyaw * 180.0 / M_PI << "°" << std::setprecision(1);
  } else {
    s << std::noshowpos << " · 회전은 안 맞춤 (잰 칸끼리 " << min_span_m_ * 1000.0 << " mm 안: 서로 먼 칸을 더 재기)";
  }
  s << std::noshowpos << ", 맞춘 뒤 남는 오차 " << f.rms * 1000.0 << " mm\n";
  s << "칸별 옮길 양 dx,dy [mm] (tools/arm_real/arm/shift_cell_paths.py 인자, * = 잰 칸). "
    << "경로를 이미 옮긴 칸 (arm_paths.yaml '칸 보정') 은 그만큼 빼서 씀\n ";
  for (size_t i = 0; i < cells_.size(); ++i) {
    const Eigen::Vector2d d = rack_shift(f, cells_[i], pivot_) * 1000.0;
    s << " " << names_[i] << (measured[i] ? "*" : "") << " " << std::showpos << d.x() << "," << d.y() << std::noshowpos;
  }
  s << "\n";
  return s.str();
}

}  // namespace manipulation
