// camera_drift_node — 팔이 홈 자세에 멈출 때마다 손목 카메라 사진을 기준 사진 (잘 될 때 홈 사진) 과 비교해
//   카메라가 손목에서 돈 만큼을 /wrist_cam/drift (QuaternionStamped, 래치) 로 냄 → control_master 가 손목 검출을 그만큼 되돌려 씀
// =====================================================================
// 11-05 실물: 세우기 때 손이 박스를 누르며 카메라 경첩이 3.8° 돌아 어제 맞춘 집기 보정이 다 틀어짐 (팔 관절값은 그대로).
// 홈 자세는 늘 같은 관절이라 사진 속 로봇 몸통이 움직였으면 카메라가 돈 것 (camera_drift.hpp).
//   - 홈 판정: /joint_states 의 joint1..5 가 home_q ± home_tol_rad 안에 still_s 동안 → 사진 한 장 (홈에 올 때마다 한 번)
//   - 사진 아래쪽 (roi_y0 부터, 몸통) ORB 특징 → 기준과 맞춤 (RANSAC) → 회전. 점이 min_points 보다 적거나 남는 차가 max_resid_px 넘거나
//     회전이 max_deg 넘으면 안 냄 (앞 값 유지, 경고 — 크게 돌았으면 경첩 · 나사 확인)
//   - 마지막 값은 ~/.ros/camera_drift/last.txt 에 저장 → 다시 켜도 바로 냄
//   - ~/save_reference (std_srvs/Trigger): 다음 홈 사진을 새 기준으로 (~/.ros/camera_drift/ref.png) 저장하고 보정 0 으로
//     (기준 = 집기 · 적재가 잘 되는 상태에서만). 없으면 reference_image (설치된 config/camera_home_ref.png, 11-04 저녁)

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "manipulation/camera_drift.hpp"

namespace fs = std::filesystem;

class CameraDriftNode : public rclcpp::Node
{
public:
  CameraDriftNode()
  : Node("camera_drift_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    frame_ = declare_parameter<std::string>("camera_frame", "d435_color_optical_frame");
    joints_ = declare_parameter<std::vector<std::string>>("joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    home_q_ = declare_parameter<std::vector<double>>("home_q", {-0.0046, -1.4511, 0.9434, -0.1166, 0.0215});
    home_tol_ = declare_parameter<double>("home_tol_rad", 0.01);
    still_s_ = declare_parameter<double>("still_s", 1.0);
    roi_y0_ = declare_parameter<int>("roi_y0", 190);
    min_points_ = declare_parameter<int>("min_points", 40);
    max_deg_ = declare_parameter<double>("max_deg", 8.0);
    max_resid_ = declare_parameter<double>("max_resid_px", 8.0);
    K_ = declare_parameter<std::vector<double>>("intrinsics", {606.43, 606.16, 325.19, 256.77});   // fx fy cx cy (D435 컬러 640 × 480)
    const char * home = std::getenv("HOME");
    dir_ = fs::path(home ? home : ".") / ".ros" / "camera_drift";
    const std::string ref_default = declare_parameter<std::string>("reference_image", "");
    const fs::path user_ref = dir_ / "ref.png";
    ref_path_ = fs::exists(user_ref) ? user_ref.string() : ref_default;
    ref_ = ref_path_.empty() ? cv::Mat() : cv::imread(ref_path_, cv::IMREAD_GRAYSCALE);
    if (ref_.empty()) {
      RCLCPP_ERROR(get_logger(), "손목 카메라 기준 사진을 못 읽음 (%s) → 보정 안 함. 잘 될 때 홈에서 ros2 service call /camera_drift_node/save_reference "
        "std_srvs/srv/Trigger", ref_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "손목 카메라 기준 사진: %s", ref_path_.c_str());
    }
    pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      declare_parameter<std::string>("out_topic", "/wrist_cam/drift"), rclcpp::QoS(1).transient_local().reliable());
    publish(load_last(), "지난번 값");
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_joints(*m);});
    srv_ = create_service<std_srvs::srv::Trigger>("~/save_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        save_ref_ = true;
        done_this_visit_ = false;
        res->success = true;
        res->message = "다음 홈 자세 사진을 새 기준으로 저장하고 보정을 0 으로 (팔이 홈에서 멈춰 있으면 곧)";
      });
  }

private:
  void on_joints(const sensor_msgs::msg::JointState & m)
  {
    if (home_q_.size() != joints_.size()) {return;}
    bool home = true;
    for (size_t j = 0; j < joints_.size(); ++j) {
      bool found = false;
      for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
        if (m.name[i] == joints_[j]) {found = true; home = home && std::abs(m.position[i] - home_q_[j]) < home_tol_; break;}
      }
      home = home && found;
    }
    const double t = now().seconds();
    if (!home) {home_since_.reset(); done_this_visit_ = false; return;}
    if (!home_since_) {home_since_ = t;}
    if (!done_this_visit_ && !sub_img_ && t - *home_since_ >= still_s_) {
      done_this_visit_ = true;
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr im) {on_image(im);});
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & im)
  {
    sub_img_.reset();   // 한 장만
    cv::Mat gray;
    try {
      gray = cv_bridge::toCvCopy(im, "mono8")->image;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "손목 사진을 못 바꿈 (%s)", e.what());
      return;
    }
    if (save_ref_) {
      save_ref_ = false;
      std::error_code ec;
      fs::create_directories(dir_, ec);
      const std::string p = (dir_ / "ref.png").string();
      if (!cv::imwrite(p, gray)) {RCLCPP_ERROR(get_logger(), "기준 사진 저장 실패: %s", p.c_str()); return;}
      ref_ = gray;
      ref_path_ = p;
      publish(Eigen::Quaterniond::Identity(), "새 기준");
      save_last(Eigen::Quaterniond::Identity());
      RCLCPP_INFO(get_logger(), "손목 카메라: 지금 홈 사진을 새 기준으로 저장 (%s) → 보정 0", p.c_str());
      return;
    }
    if (ref_.empty() || gray.size() != ref_.size()) {
      RCLCPP_WARN(get_logger(), "손목 카메라: 기준 사진이 없거나 크기가 다름 → 보정 안 함");
      return;
    }
    cv::Mat roi = cv::Mat::zeros(gray.size(), CV_8U);
    roi.rowRange(std::clamp(roi_y0_, 0, gray.rows - 1), gray.rows).setTo(255);
    auto orb = cv::ORB::create(4000);
    std::vector<cv::KeyPoint> ka, kb;
    cv::Mat da, db;
    orb->detectAndCompute(ref_, roi, ka, da);
    orb->detectAndCompute(gray, roi, kb, db);
    if (da.empty() || db.empty()) {RCLCPP_WARN(get_logger(), "손목 카메라: 몸통 특징이 없음 (가려졌거나 어두움) → 보정 안 함"); return;}
    std::vector<cv::DMatch> ms;
    cv::BFMatcher(cv::NORM_HAMMING, true).match(da, db, ms);
    std::sort(ms.begin(), ms.end(), [](const cv::DMatch & a, const cv::DMatch & b) {return a.distance < b.distance;});
    if (ms.size() > 400) {ms.resize(400);}
    std::vector<cv::Point2f> pa, pb;
    for (const auto & x : ms) {pa.push_back(ka[x.queryIdx].pt); pb.push_back(kb[x.trainIdx].pt);}
    if (static_cast<int>(pa.size()) < min_points_) {
      RCLCPP_WARN(get_logger(), "손목 카메라: 맞은 점 %zu 개 (%d 개 넘어야) → 보정 안 함", pa.size(), min_points_);
      return;
    }
    std::vector<uchar> inl;
    cv::findHomography(pa, pb, cv::RANSAC, 3.0, inl);
    std::vector<Eigen::Vector2d> ua, ub;
    for (size_t i = 0; i < inl.size(); ++i) {
      if (inl[i]) {ua.emplace_back(pa[i].x, pa[i].y); ub.emplace_back(pb[i].x, pb[i].y);}
    }
    manipulation::CameraDrift d;
    if (static_cast<int>(ua.size()) < min_points_ || K_.size() != 4 ||
      !manipulation::camera_drift(ua, ub, K_[0], K_[1], K_[2], K_[3], d))
    {
      RCLCPP_WARN(get_logger(), "손목 카메라: 맞은 점 %zu 개 (%d 개 넘어야) → 보정 안 함", ua.size(), min_points_);
      return;
    }
    const Eigen::Vector3d rv = Eigen::AngleAxisd(d.R).axis() * Eigen::AngleAxisd(d.R).angle() * 180.0 / M_PI;
    if (d.resid_px > max_resid_ || d.deg > max_deg_) {
      RCLCPP_WARN(get_logger(), "손목 카메라: 기준 대비 %.1f° (x %+.2f · y %+.2f · z %+.2f) · 남는 차 %.1f px · 점 %d — 너무 크거나 안 맞음 → 보정 안 함 "
        "(카메라 경첩 · 나사 확인, 몸통이 가려졌는지 확인)", d.deg, rv.x(), rv.y(), rv.z(), d.resid_px, d.n);
      return;
    }
    const Eigen::Quaterniond q(d.R);
    publish(q, "홈 사진");
    save_last(q);
    RCLCPP_INFO(get_logger(), "손목 카메라: 기준 대비 %.2f° (x %+.2f · y %+.2f · z %+.2f) · 남는 차 %.1f px · 점 %d → 손목 검출 보정%s", d.deg,
      rv.x(), rv.y(), rv.z(), d.resid_px, d.n, d.deg > 2.0 ? " (2° 넘음 — 카메라 경첩이 돌았을 수 있음)" : "");
  }

  void publish(const Eigen::Quaterniond & q, const char * why)
  {
    geometry_msgs::msg::QuaternionStamped m;
    m.header.stamp = now();
    m.header.frame_id = frame_;
    m.quaternion.x = q.x();
    m.quaternion.y = q.y();
    m.quaternion.z = q.z();
    m.quaternion.w = q.w();
    pub_->publish(m);
    RCLCPP_DEBUG(get_logger(), "보정 냄 (%s)", why);
  }

  Eigen::Quaterniond load_last() const
  {
    std::ifstream f(dir_ / "last.txt");
    double x = 0, y = 0, z = 0, w = 1;
    if (f >> x >> y >> z >> w) {
      Eigen::Quaterniond q(w, x, y, z);
      if (std::abs(q.norm() - 1.0) < 0.01) {return q.normalized();}
    }
    return Eigen::Quaterniond::Identity();
  }

  void save_last(const Eigen::Quaterniond & q) const
  {
    std::error_code ec;
    fs::create_directories(dir_, ec);
    std::ofstream(dir_ / "last.txt") << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }

  std::string image_topic_, frame_, ref_path_;
  std::vector<std::string> joints_;
  std::vector<double> home_q_, K_;
  double home_tol_{0.01}, still_s_{1.0}, max_deg_{8.0}, max_resid_{8.0};
  int roi_y0_{190}, min_points_{40};
  fs::path dir_;
  cv::Mat ref_;
  std::optional<double> home_since_;
  bool done_this_visit_{false}, save_ref_{false};
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriftNode>());
  rclcpp::shutdown();
  return 0;
}
