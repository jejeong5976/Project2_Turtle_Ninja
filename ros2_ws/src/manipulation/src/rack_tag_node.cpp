// rack_tag_node.cpp — 손목 카메라로 적재함 칸 바닥 AprilTag 를 재서 칸 위치 · 적재함 이동을 알려 줌 (팔은 안 움직임)
// =====================================================================
// 입력: 손목 컬러 영상 · camera_info, TF base_link ← 손목 카메라 (처짐 보정한 실제 팔 자세, arm_state_publisher)
// 출력: /rack_tags/detections (interfaces/DetectionArray, frame base_link, kind "rack_tag", label = 칸 S1~S12, id = 태그 ID)
//         pose = 칸 바닥 위 태그 중심, z 축 위, x 축 = 태그 x 축. 처리한 영상마다 (없으면 빈 배열)
//       ~/report (std_srvs/Trigger): 칸별 잰 위치 (중앙값) − config/rack_cells.yaml, 적재함 전체 이동 맞춤, 칸별 옮길 양
//       ~/reset (std_srvs/Trigger): 모은 값 지움
// 태그: tag36h11, cell_ids[i] = cell_names[i] 칸의 태그 (config/rack_cells.yaml rack_tags), 검은 사각 26 mm, 칸 바닥 가운데
//   잰 자리가 다른 칸 표 위치에 더 가깝고 제 칸에서 mismatch_m 넘게 떨어지면 ID 를 잘못 붙인 것으로 보고 안 씀 (경고에 그 칸 이름)
// 팔이 멈춰 있을 때 (앞 장과 카메라 자리 차이가 still_m · still_deg 안) 만 모음 (움직이는 중은 영상 · 관절 시각이 어긋나 수 mm 틀림)
// 잰 값 − 표 값에는 태그를 칸 가운데에 붙인 오차도 들어감 → 경로가 잘 맞을 때 한 번 재 두고 그 뒤와 비교하는 게 가장 정확

#include <cmath>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
// OpenCV 4.7 부터 aruco 가 objdetect 로 옮겨짐 (그 전은 contrib 모듈)
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define RACK_TAG_ARUCO_NEW 1
#else
#include <opencv2/aruco.hpp>
#endif

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "manipulation/rack_tags.hpp"

namespace manipulation
{

class RackTagNode : public rclcpp::Node
{
public:
  RackTagNode()
  : Node("rack_tag_node")
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/camera/color/camera_info");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "d435_color_optical_frame");   // "" = 영상 header 그대로
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cfg_.plane_z = declare_parameter<double>("floor_z", 0.1325);
    cfg_.tag_size_m = declare_parameter<double>("tag_size_m", 0.026);
    cfg_.size_tol = declare_parameter<double>("size_tol", 0.15);
    cfg_.min_view_cos = declare_parameter<double>("min_view_cos", 0.4);
    names_ = declare_parameter<std::vector<std::string>>("cell_names", std::vector<std::string>{});
    const auto xs = declare_parameter<std::vector<double>>("cell_x", std::vector<double>{});
    const auto ys = declare_parameter<std::vector<double>>("cell_y", std::vector<double>{});
    const auto tag_ids = declare_parameter<std::vector<int64_t>>("cell_ids", std::vector<int64_t>{});
    mismatch_m_ = declare_parameter<double>("mismatch_m", 0.02);
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 5.0);
    still_m_ = declare_parameter<double>("still_m", 0.001);
    still_rad_ = declare_parameter<double>("still_deg", 0.3) * M_PI / 180.0;
    samples_ = declare_parameter<int>("samples", 30);
    min_samples_ = declare_parameter<int>("min_samples", 5);
    min_span_m_ = declare_parameter<double>("min_span_m", 0.06);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.05);
    if (names_.empty() || xs.size() != names_.size() || ys.size() != names_.size() || tag_ids.size() != names_.size()) {
      throw std::runtime_error("cell_names · cell_x · cell_y · cell_ids 가 비었거나 길이가 다름 (config/rack_cells.yaml, launch 가 넣음)");
    }
    if (!(max_rate_hz_ > 0.0) || samples_ < 1 || min_samples_ < 1 || min_samples_ > samples_ || !(cfg_.tag_size_m > 0.0)) {
      throw std::runtime_error("max_rate_hz · samples · min_samples · tag_size_m 는 0 보다 커야 하고 min_samples <= samples");
    }
    pivot_.setZero();
    for (size_t i = 0; i < names_.size(); ++i) {
      cells_.emplace_back(xs[i], ys[i]);
      pivot_ += cells_.back();
      if (!index_.emplace(static_cast<int>(tag_ids[i]), i).second) {
        throw std::runtime_error("cell_ids 에 같은 태그 ID 가 두 번: " + std::to_string(tag_ids[i]));
      }
    }
    pivot_ /= static_cast<double>(cells_.size());
    got_.resize(cells_.size());

    tf_buf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listen_ = std::make_unique<tf2_ros::TransformListener>(*tf_buf_);
    pub_ = create_publisher<interfaces::msg::DetectionArray>("/rack_tags/detections", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {info_ = m;});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {on_image(m);});
    report_srv_ = create_service<std_srvs::srv::Trigger>("~/report",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
          res->message = report();
          res->success = true;
          RCLCPP_INFO(get_logger(), "\n%s", res->message.c_str());
        });
    reset_srv_ = create_service<std_srvs::srv::Trigger>("~/reset",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
          for (auto & g : got_) {g.clear();}
          wrong_.clear();
          res->success = true;
          res->message = "모은 값 지움";
        });
    RCLCPP_INFO(get_logger(), "칸 태그 %zu 개, 바닥 z %.4f m, 태그 %.0f mm, 영상 %s",
      cells_.size(), cfg_.plane_z, cfg_.tag_size_m * 1000.0, image_topic.c_str());
  }

private:
  // 왜곡을 푼 정규 좌표 (x/z, y/z). camera_info 모델을 모르면 false
  //   K · D 는 전체 해상도 기준 → 영상이 ROI 로 잘렸거나 비닝됐으면 화소를 전체 해상도로 되돌린 뒤 풂
  bool undistort(const sensor_msgs::msg::CameraInfo & info, const std::vector<cv::Point2f> & px, std::vector<cv::Point2f> & out)
  {
    const cv::Matx33d K(info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5], info.k[6], info.k[7], info.k[8]);
    const float bx = info.binning_x ? static_cast<float>(info.binning_x) : 1.0f;
    const float by = info.binning_y ? static_cast<float>(info.binning_y) : 1.0f;
    std::vector<cv::Point2f> full;
    for (const auto & p : px) {
      full.emplace_back(p.x * bx + static_cast<float>(info.roi.x_offset), p.y * by + static_cast<float>(info.roi.y_offset));
    }
    const auto & m = info.distortion_model;
    if (m == "equidistant") {   // 계수가 0 이어도 핀홀과 투영이 다름
      if (info.d.size() < 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info equidistant 인데 계수가 4 개보다 적음 → 안 잼");
        return false;
      }
      cv::fisheye::undistortPoints(full, out, K, cv::Mat(std::vector<double>(info.d.begin(), info.d.begin() + 4), true));
      return true;
    }
    bool zero = true;
    for (double d : info.d) {zero = zero && d == 0.0;}
    if (zero || m == "plumb_bob" || m == "rational_polynomial") {
      cv::undistortPoints(full, out, K, zero ? cv::Mat() : cv::Mat(info.d, true));
      return true;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 왜곡 모델 %s 를 모름 → 안 잼", m.c_str());
    return false;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (have_last_ && stamp >= last_ && (stamp - last_).seconds() < 1.0 / max_rate_hz_) {
      return;
    }
    const auto info = info_;
    if (!info || !(info->k[0] > 0.0) || !(info->k[4] > 0.0)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 가 아직 없거나 K 가 0");
      return;
    }
    const std::string cam = camera_frame_.empty() ? msg->header.frame_id : camera_frame_;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buf_->lookupTransform(base_frame_, cam, stamp, rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s ← %s 없음: %s", base_frame_.c_str(), cam.c_str(), e.what());
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

    cv_bridge::CvImageConstPtr gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "영상 변환 실패 (%s): %s", msg->encoding.c_str(), e.what());
      return;
    }
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detect(gray->image, corners, ids);

    interfaces::msg::DetectionArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = base_frame_;
    for (size_t k = 0; k < ids.size(); ++k) {
      const auto it = index_.find(ids[k]);
      if (it == index_.end() || corners[k].size() != 4) {
        continue;   // 퍽 태그 (0~11) 등 다른 태그
      }
      const size_t i = it->second;
      std::vector<cv::Point2f> n;
      if (!undistort(*info, corners[k], n) || n.size() != 4) {
        return;
      }
      std::array<Eigen::Vector2d, 4> cn;
      for (int j = 0; j < 4; ++j) {cn[j] = Eigen::Vector2d(n[j].x, n[j].y);}
      const TagOnPlane hit = tag_on_plane(cn, T, cfg_);
      if (!hit.ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s 태그 안 씀: %s (한 변 %.1f mm)",
          names_[i].c_str(), hit.why, hit.side_m * 1000.0);
        continue;
      }
      const size_t near = nearest_cell(hit.center);
      if (near != i && (hit.center - cells_[i]).norm() > mismatch_m_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "태그 ID %d: %s 자리가 아니라 %s 자리에 있음 → 안 씀 "
          "(~/report 에 고칠 ids)", ids[k], names_[i].c_str(), names_[near].c_str());
        wrong_[ids[k]] = near;
        continue;
      }
      interfaces::msg::Detection d;
      d.header = out.header;
      d.kind = "rack_tag";
      d.label = names_[i];
      d.id = std::to_string(ids[k]);
      d.confidence = still ? 1.0f : 0.5f;   // 0.5 = 팔이 움직이는 중이라 수 mm 틀릴 수 있음
      d.pose.position.x = hit.center.x();
      d.pose.position.y = hit.center.y();
      d.pose.position.z = cfg_.plane_z;
      d.pose.orientation.z = std::sin(hit.yaw / 2.0);
      d.pose.orientation.w = std::cos(hit.yaw / 2.0);
      d.yaw_known = true;
      float x0 = corners[k][0].x, y0 = corners[k][0].y, x1 = x0, y1 = y0;
      for (const auto & p : corners[k]) {
        x0 = std::min(x0, p.x);
        y0 = std::min(y0, p.y);
        x1 = std::max(x1, p.x);
        y1 = std::max(y1, p.y);
      }
      d.bbox = {x0, y0, x1, y1};
      out.detections.push_back(d);
      if (still) {
        auto & g = got_[i];
        g.push_back(hit.center);
        while (static_cast<int>(g.size()) > samples_) {g.pop_front();}
        if (static_cast<int>(g.size()) == min_samples_) {
          const Eigen::Vector2d m = cell_median(i);
          RCLCPP_INFO(get_logger(), "%s 잼: (%.1f, %.1f) mm, 표와 차 dx %+.1f · dy %+.1f mm",
            names_[i].c_str(), m.x() * 1000.0, m.y() * 1000.0, (m.x() - cells_[i].x()) * 1000.0, (m.y() - cells_[i].y()) * 1000.0);
        }
      }
    }
    pub_->publish(out);
  }

  static void detect(const cv::Mat & gray, std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
  {
#ifdef RACK_TAG_ARUCO_NEW
    static const cv::aruco::ArucoDetector det = [] {
        cv::aruco::DetectorParameters p;
        p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;   // 모서리를 화소 아래까지 (평면 교점 정확도)
        return cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), p);
      }();
    det.detectMarkers(gray, corners, ids);
#else
    static const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    static const auto prm = [] {
        auto p = cv::aruco::DetectorParameters::create();
        p->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        return p;
      }();
    cv::aruco::detectMarkers(gray, dict, corners, ids, prm);
#endif
  }

  size_t nearest_cell(const Eigen::Vector2d & p) const
  {
    size_t best = 0;
    for (size_t j = 1; j < cells_.size(); ++j) {
      if ((p - cells_[j]).norm() < (p - cells_[best]).norm()) {best = j;}
    }
    return best;
  }

  Eigen::Vector2d cell_median(size_t i) const
  {
    std::vector<double> x, y;
    for (const auto & p : got_[i]) {
      x.push_back(p.x());
      y.push_back(p.y());
    }
    return {median(x), median(y)};
  }

  std::string report() const
  {
    std::ostringstream s;
    s << std::fixed << std::setprecision(1);
    s << "칸 · 장 수 · 잰 x, y · 표 x, y · 차 dx, dy · 흩어짐 [mm] (표 = config/rack_cells.yaml, 멈춘 장 " << min_samples_ << " 개 이상인 칸만)\n";
    std::vector<Eigen::Vector2d> exp, meas;
    std::vector<bool> measured(cells_.size(), false);
    for (size_t i = 0; i < cells_.size(); ++i) {
      if (static_cast<int>(got_[i].size()) < min_samples_) {
        continue;
      }
      const Eigen::Vector2d m = cell_median(i);
      double se = 0.0;
      for (const auto & p : got_[i]) {se += (p - m).squaredNorm();}
      const Eigen::Vector2d d = (m - cells_[i]) * 1000.0;
      s << "  " << names_[i] << "  " << got_[i].size() << "  (" << m.x() * 1000.0 << ", " << m.y() * 1000.0 << ")  ("
        << cells_[i].x() * 1000.0 << ", " << cells_[i].y() * 1000.0 << ")  " << std::showpos << d.x() << ", " << d.y()
        << std::noshowpos << "  " << std::sqrt(se / static_cast<double>(got_[i].size())) * 1000.0 << "\n";
      exp.push_back(cells_[i]);
      meas.push_back(m);
      measured[i] = true;
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

  TagPlaneConfig cfg_;
  std::string camera_frame_, base_frame_;
  std::vector<std::string> names_;
  std::map<int, size_t> index_;   // 태그 ID → 칸 번호
  std::vector<Eigen::Vector2d> cells_;
  Eigen::Vector2d pivot_;
  double max_rate_hz_{5.0}, still_m_{0.001}, still_rad_{0.005}, min_span_m_{0.06}, tf_timeout_s_{0.05}, mismatch_m_{0.02};
  int samples_{30}, min_samples_{5};

  std::vector<std::deque<Eigen::Vector2d>> got_;   // 칸마다 멈춘 장에서 잰 중심 (최근 samples_ 개)
  std::map<int, size_t> wrong_;                    // 다른 칸 자리에서 본 태그 ID → 그 칸
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  bool have_last_{false};
  rclcpp::Time last_;
  Eigen::Isometry3d prev_cam_{Eigen::Isometry3d::Identity()};

  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listen_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr report_srv_, reset_srv_;
};

}  // namespace manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manipulation::RackTagNode>());
  rclcpp::shutdown();
  return 0;
}
