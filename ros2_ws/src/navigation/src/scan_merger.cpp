// 앞 · 뒤 라이다 → 몸 중심 360° 스캔 /scan 하나 (slam_toolbox · 앞 물체 정지가 받음).
//
// 받기: scans 의 LaserScan (SensorDataQoS). 내기: out, frame_id = frame, bins 칸 — 칸마다 가장 가까운 점.
// 각 라이다 장착 자세는 TF(고정)에서 처음 한 번 읽음. 자기 몸 점은 버림 (self_filter):
//   box    : 몸 상자(body_half) 안 점 (옆 0.16–0.19 도 지워져 PolygonStop 이 옆을 못 봄)
//   wheels : 차체 외곽 상자(chassis_x · chassis_half_y, 앞뒤는 face_clear_x 까지) + 바퀴 모듈마다 조향 각 (/joint_states) 에 따라 도는 사각형
//            (wheel_half_u · wheel_in · wheel_out). joint_states 가 joint_timeout 넘게 안 오면 모듈 둘레 원 (모서리 반지름)
// 첫 번째 라이다가 올 때마다 냄 (시각 = 그 스캔 시각). 나머지 라이다는 그 시각에서 max_age 안이어야 함.
// 한 라이다라도 max_age 를 넘으면 /scan 을 내지 않음 → 받는 쪽(앞 물체 정지 · 위치 감시)이 "스캔 끊김" 으로 멈춤.
//   (한쪽만으로 내면 그쪽 방향이 비어 "장애물 없음" 과 구분되지 않음)
// 두 라이다 시각 차 동안 몸이 움직인 만큼은 보정하지 않음: 0.10 m/s · 0.1 s = 1 cm.

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "navigation/scan_merge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace navigation
{

class ScanMerger : public rclcpp::Node
{
public:
  ScanMerger()
  : Node("scan_merger"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto topics = declare_parameter<std::vector<std::string>>(
      "scans", {"/lidar_front/scan", "/lidar_rear/scan"});
    frame_ = declare_parameter<std::string>("frame", "base_footprint");
    bins_ = static_cast<int>(declare_parameter<int>("bins", 720));
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    const auto half = declare_parameter<std::vector<double>>("body_half", {0.17, 0.19});
    max_age_ = declare_parameter<double>("max_age", 0.15);
    const auto out = declare_parameter<std::string>("out", "/scan");
    if (topics.empty() || bins_ < 8 || half.size() != 2 || max_age_ <= 0.0) {
      throw std::invalid_argument("scan_merger: scans 비었음 · bins < 8 · body_half 2 개 아님 · max_age <= 0");
    }
    self_.box = BodyBox{half[0], half[1]};
    // 자기 몸 거르기 방식 (코드 기본 box. navigation.yaml 이 wheels 로 켬 — 이유는 README)
    const auto mode = declare_parameter<std::string>("self_filter", "box");
    const auto cx = declare_parameter<std::vector<double>>("chassis_x", {-0.133, 0.141});
    self_.chassis_half_y = declare_parameter<double>("chassis_half_y", 0.150);
    const auto fc = declare_parameter<std::vector<double>>("face_clear_x", {-0.17, 0.17});
    const auto mx = declare_parameter<std::vector<double>>("wheel_module_x", {-0.070, 0.060, -0.070, 0.060});
    const auto my = declare_parameter<std::vector<double>>("wheel_module_y", {0.112, 0.112, -0.112, -0.112});
    steer_joints_ = declare_parameter<std::vector<std::string>>("steer_joints",
      {"steer_fl_joint", "steer_fr_joint", "steer_rl_joint", "steer_rr_joint"});
    self_.wheel_half_u = declare_parameter<double>("wheel_half_u", 0.05);
    self_.wheel_in = declare_parameter<double>("wheel_in", 0.05);
    self_.wheel_out = declare_parameter<double>("wheel_out", 0.05);
    joint_timeout_ = declare_parameter<double>("joint_timeout", 0.5);
    const auto js_topic = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    if (mode != "box" && mode != "wheels") {
      throw std::invalid_argument("scan_merger: self_filter 는 box | wheels");
    }
    self_.box_mode = mode == "box";
    if (!self_.box_mode) {
      if (cx.size() != 2 || !(cx[0] < cx[1]) || fc.size() != 2 || !(fc[0] <= 0.0 && fc[1] >= 0.0) ||
          !(self_.chassis_half_y > 0.0) || mx.empty() || mx.size() != my.size() ||
          steer_joints_.size() != mx.size() || !(self_.wheel_half_u >= 0.0 && self_.wheel_in >= 0.0 &&
          self_.wheel_out >= 0.0) || !(joint_timeout_ > 0.0))
      {
        throw std::invalid_argument("scan_merger wheels: chassis_x [min<max] · face_clear_x [≤0, ≥0] · chassis_half_y > 0 · wheel_module_x/y · "
          "steer_joints 개수 같음 · wheel_* >= 0 · joint_timeout > 0");
      }
      self_.chassis_x_min = cx[0];
      self_.chassis_x_max = cx[1];
      self_.face_clear_x_min = fc[0];
      self_.face_clear_x_max = fc[1];
      for (std::size_t i = 0; i < mx.size(); ++i) {
        self_.modules.push_back(WheelModule{mx[i], my[i], my[i] >= 0.0 ? 1.0 : -1.0});
      }
      steer_now_.assign(mx.size(), std::numeric_limits<double>::quiet_NaN());
      sub_js_ = create_subscription<sensor_msgs::msg::JointState>(js_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_joints(*m);});
    }

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out, rclcpp::SensorDataQoS());
    sources_.resize(topics.size());
    for (std::size_t i = 0; i < topics.size(); ++i) {
      sources_[i].topic = topics[i];
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(i, *m);}));
    }
    if (self_.box_mode) {
      RCLCPP_INFO(get_logger(), "스캔 합치기: 라이다 %zu 개 → %s (%s, %d 칸), 자기 몸 = 상자 ±%.2f × ±%.2f m (self_filter box)",
        topics.size(), out.c_str(), frame_.c_str(), bins_, self_.box.half_x, self_.box.half_y);
    } else {
      RCLCPP_INFO(get_logger(), "스캔 합치기: 라이다 %zu 개 → %s (%s, %d 칸), 자기 몸 = 차체 x %.3f~%.3f (앞뒤 면 띠 %.3f~%.3f) · y ±%.3f m + 바퀴 %zu 개 "
        "(조향 따라 도는 사각형 굴림 ±%.3f · 안 %.3f · 바깥 %.3f m, %s %.1f s 끊기면 반지름 %.3f m 원) (self_filter wheels)",
        topics.size(), out.c_str(), frame_.c_str(), bins_, self_.chassis_x_min, self_.chassis_x_max,
        std::min(self_.chassis_x_min, self_.face_clear_x_min), std::max(self_.chassis_x_max, self_.face_clear_x_max), self_.chassis_half_y,
        self_.modules.size(), self_.wheel_half_u, self_.wheel_in, self_.wheel_out, js_topic.c_str(), joint_timeout_,
        self_.sweep_radius());
    }
  }

private:
  struct Source
  {
    std::string topic;
    bool have{false};
    double stamp{0.0};
    std::vector<Point2> pts;
  };

  bool mount_of(const std::string & frame_id, Mount & m)
  {
    auto it = mounts_.find(frame_id);
    if (it != mounts_.end()) {m = it->second; return true;}
    try {
      const auto t = tf_buffer_.lookupTransform(frame_, frame_id, tf2::TimePointZero).transform;
      const auto & q = t.rotation;
      m = Mount{t.translation.x, t.translation.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      mounts_[frame_id] = m;
      return true;
    } catch (const tf2::TransformException & e) {   // URDF 가 아직 안 떴으면 다음 스캔에서 다시
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s ← %s 없음: %s",
        frame_.c_str(), frame_id.c_str(), e.what());
      return false;
    }
  }

  // 조향 각 (joint_states 에 있는 것만 갱신 — 팔 · 바퀴가 따로 오는 메시지도 받음)
  void on_joints(const sensor_msgs::msg::JointState & m)
  {
    bool any = false;
    for (std::size_t k = 0; k < m.name.size() && k < m.position.size(); ++k) {
      for (std::size_t j = 0; j < steer_joints_.size(); ++j) {
        if (m.name[k] == steer_joints_[j]) {steer_now_[j] = m.position[k]; any = true;}
      }
    }
    if (any) {
      js_time_ = now().seconds();
      have_js_ = std::all_of(steer_now_.begin(), steer_now_.end(), [](double a) {return std::isfinite(a);});
    }
  }

  // 이번 스캔에 쓸 조향 각: 신선하면 그대로, 아니면 비움 (= 모듈 둘레 원)
  void refresh_steer()
  {
    if (self_.box_mode) {return;}
    const bool fresh = have_js_ && now().seconds() - js_time_ <= joint_timeout_;
    if (fresh) {
      self_.steer = steer_now_;
      if (stale_logged_) {RCLCPP_INFO(get_logger(), "조향 각 다시 받음 → 바퀴 사각형으로 거름"); stale_logged_ = false;}
    } else {
      self_.steer.clear();
      if (!stale_logged_) {
        RCLCPP_WARN(get_logger(), "조향 각 (%s) 이 %.1f s 넘게 없음 → 바퀴 모듈 둘레 원 (반지름 %.3f m) 으로 거름",
          steer_joints_.empty() ? "" : steer_joints_[0].c_str(), joint_timeout_, self_.sweep_radius());
        stale_logged_ = true;
      }
    }
  }

  void on_scan(std::size_t i, const sensor_msgs::msg::LaserScan & m)
  {
    Mount mount{};
    if (!mount_of(m.header.frame_id, mount)) {return;}
    auto & s = sources_[i];
    refresh_steer();
    scan_to_points(m.ranges, m.angle_min, m.angle_increment,
      std::max<double>(m.range_min, range_min_), std::min<double>(m.range_max, range_max_), mount, self_, s.pts);
    s.stamp = rclcpp::Time(m.header.stamp).seconds();
    s.have = true;
    if (i == 0) {publish(m);}
  }

  void publish(const sensor_msgs::msg::LaserScan & first)
  {
    merged_.clear();
    for (const auto & s : sources_) {
      if (!s.have || std::fabs(s.stamp - sources_[0].stamp) > max_age_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s 끊김 (%.2f s 넘음) → /scan 안 냄",
          s.topic.c_str(), max_age_);
        return;
      }
      merged_.insert(merged_.end(), s.pts.begin(), s.pts.end());
    }
    auto out = std::make_unique<sensor_msgs::msg::LaserScan>();
    out->header.stamp = first.header.stamp;
    out->header.frame_id = frame_;
    out->angle_increment = static_cast<float>(2.0 * M_PI / bins_);
    out->angle_min = static_cast<float>(-M_PI);
    out->angle_max = out->angle_min + out->angle_increment * static_cast<float>(bins_ - 1);
    out->scan_time = first.scan_time;
    out->time_increment = 0.0f;
    out->range_min = static_cast<float>(range_min_);
    out->range_max = static_cast<float>(range_max_);
    bin_points(merged_, bins_, out->ranges);
    pub_->publish(std::move(out));
  }

  std::string frame_;
  int bins_{720};
  double range_min_{0.05};
  double range_max_{8.0};
  double max_age_{0.15};
  SelfFilter self_;
  std::vector<std::string> steer_joints_;
  std::vector<double> steer_now_;
  double js_time_{-1e9};
  bool have_js_{false};
  bool stale_logged_{false};
  double joint_timeout_{0.5};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, Mount> mounts_;
  std::vector<Source> sources_;
  std::vector<Point2> merged_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
