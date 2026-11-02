// 위치 추정 감시 — SLAM · 라이다 · IMU 를 서로 비교해 끊기거나 튀면 알림. slam.launch.py 가 함께 띄움.
//
// 내기: /localization_health (diagnostic_msgs/DiagnosticStatus, 5 Hz) level 0 정상 · 1 경고(느리게) · 2 정지.
//   경로 주행(path_follow)이 받아서 따름. 정지 · 경고는 원인이 없어진 뒤 hold_s 더 유지 (깜빡임 방지).
// 제일 믿는 값 = SLAM(map). 단 라이다가 지도 벽과 맞을 때만 — 맞음 비율이 낮으면 SLAM 위치를 못 믿음.
//   정지  라이다(/scan) 0.5 s 끊김 · map→odom TF 1 s 멈춤 · 지도 맞음 < 60 % 가 1 s · 지도 맞음을 2 s 넘게 못 잼
//         · map→odom 한 번에 > 25 cm 또는 > 10° 인데 맞음 < 85 % · 방향 고침 5 s 평균 > 3 °/s · 가속도 9.8 ± 3 밖 1 s
//   경고  맞음 < 85 % · map→odom 한 번에 > 10 cm 또는 > 3° · 방향 고침 > 1 °/s · 라이다 < 7 Hz 또는 < 150 점
//         · IMU 0.5 s 끊김 · 자이로 > 5 rad/s 튐
// 자이로 이동 치우침 (/imu_motion_bias, Vector3Stamped z [rad/s]): 움직이는 동안 자이로가 실제 회전을 덜 느끼는 만큼.
//   BiasLearner 가 map→odom 방향 고침 속도로 배움 → swerve_controller 가 움직일 때만 자이로에 더함.
//   판정이 정상이고 map→odom 이 살아 있을 때만 보냄 (SLAM 없이 저장값만 보내지 않음 → 컨트롤러는 5 s 뒤 0 으로).
//   배운 값은 bias_file 에 남겨 다음에 초기값으로 씀.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigation/pose_guard_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace navigation
{

namespace
{
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string expand_home(const std::string & p)
{
  const char * home = std::getenv("HOME");
  return (!p.empty() && p[0] == '~' && home) ? std::string(home) + p.substr(1) : p;
}
}  // namespace

class PoseGuard : public rclcpp::Node
{
public:
  PoseGuard()
  : Node("pose_guard"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    hold_s_ = declare_parameter<double>("hold_s", 1.0);
    publish_bias_ = declare_parameter<bool>("publish_bias", true);
    bias_file_ = expand_home(declare_parameter<std::string>("bias_file", "~/.imu_motion_bias"));
    const double bias_init = declare_parameter<double>("bias_initial", 0.0016);   // 실물 두 번 잰 값 (0.09 °/s)
    const double bias_gain = declare_parameter<double>("bias_gain", 0.3);
    const double bias_max = declare_parameter<double>("bias_max", 0.005);        // swerve_controller imu_motion_bias_max 와 같게
    const double bias_window = declare_parameter<double>("bias_window_s", 5.0);
    double init = bias_init;
    std::ifstream f(bias_file_);
    if (f >> init) {RCLCPP_INFO(get_logger(), "자이로 이동 치우침 초기값 %.4f °/s (%s)", init * 180.0 / M_PI, bias_file_.c_str());}
    bias_ = std::make_unique<BiasLearner>(init, bias_gain, bias_max, bias_window);

    pub_health_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/localization_health", 10);
    pub_bias_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu_motion_bias", 10);
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        scan_ = m;
        scan_times_.push_back(mono());
        while (scan_times_.size() > 50) {scan_times_.pop_front();}
      });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_speed_ = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
      });
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {map_ = m;});
    match_t_ = mono();
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "위치 감시 → /localization_health%s", publish_bias_ ? " · /imu_motion_bias" : "");
  }

  ~PoseGuard() override
  {
    if (!publish_bias_) {return;}
    std::ofstream f(bias_file_);
    if (f) {f << bias_->estimate() << "\n";}
  }

private:
  // 단일 스레드 executor — 콜백과 타이머가 겹치지 않으므로 잠금 없음
  double mono() const {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const double now = mono();
    imu_t_ = now;
    if (std::fabs(m.angular_velocity.z) > 5.0) {spikes_.push_back(now);}
    const auto & a = m.linear_acceleration;
    if (std::fabs(std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) - 9.8) > 3.0) {
      if (!acc_bad_since_) {acc_bad_since_ = now;}
    } else {
      acc_bad_since_.reset();
    }
  }

  // 지금 스캔 점(0.25 ~ 4 m) 중 지도 벽 5 cm 안 비율. 지도 밖 점도 분모에 넣음 (빼면 위치가 크게 틀려도 높게 나옴)
  std::optional<double> match_ratio(double now)
  {
    if (!map_ || !scan_) {return std::nullopt;}
    if (map_ != dist_map_ && now - dist_t_ > 5.0) {   // 지도가 바뀌면 5 s 에 한 번만 다시 계산
      const auto & g = *map_;
      std::vector<std::uint8_t> occ(g.data.size());
      for (std::size_t i = 0; i < occ.size(); ++i) {occ[i] = g.data[i] > 50;}
      dist_ = wall_distance(occ, static_cast<int>(g.info.width), static_cast<int>(g.info.height), 3);
      dist_map_ = map_;
      dist_t_ = now;
    }
    if (!dist_map_) {return std::nullopt;}
    const auto & info = dist_map_->info;
    geometry_msgs::msg::Transform t;
    try {
      t = tf_buffer_.lookupTransform("map", scan_->header.frame_id, tf2::TimePointZero).transform;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    const double yaw = yaw_of(t.rotation), c = std::cos(yaw), s = std::sin(yaw);
    int n = 0, hit = 0;
    for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double r = scan_->ranges[i];
      if (!std::isfinite(r) || r <= 0.25 || r >= 4.0) {continue;}
      const double a = scan_->angle_min + scan_->angle_increment * static_cast<double>(i);
      const double lx = r * std::cos(a), ly = r * std::sin(a);
      const int u = static_cast<int>(std::floor((t.translation.x + c * lx - s * ly - info.origin.position.x) / info.resolution));
      const int v = static_cast<int>(std::floor((t.translation.y + s * lx + c * ly - info.origin.position.y) / info.resolution));
      ++n;
      if (u >= 0 && v >= 0 && u < static_cast<int>(info.width) && v < static_cast<int>(info.height) &&
        dist_[static_cast<std::size_t>(v) * info.width + u] * info.resolution < 0.05)
      {
        ++hit;
      }
    }
    if (n < 30) {return std::nullopt;}
    return static_cast<double>(hit) / n;
  }

  void tick()
  {
    const double now = mono();
    GuardMetrics m;

    // map→odom: 시각이 오래됐나 · 한 번 고친 양 · 5 s 방향 고침 속도
    std::optional<double> m2o_yaw;
    try {
      const auto ts = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
      const rclcpp::Time stamp(ts.header.stamp, get_clock()->get_clock_type());
      m.map_odom_age = std::max(0.0, (get_clock()->now() - stamp).seconds());
      const double x = ts.transform.translation.x, y = ts.transform.translation.y, yaw = yaw_of(ts.transform.rotation);
      if (have_m2o_ && (x != m2o_[0] || y != m2o_[1] || yaw != m2o_[2])) {
        jumps_.push_back({now, std::hypot(x - m2o_[0], y - m2o_[1]), std::fabs(wrap_angle(yaw - m2o_[2])) * 180.0 / M_PI});
      }
      m2o_ = {x, y, yaw};
      have_m2o_ = true;
      if (*m.map_odom_age <= 1.0) {
        m2o_yaw = yaw;
        yaw_hist_.push_back({now, yaw});
      }
    } catch (const tf2::TransformException &) {
    }
    while (!jumps_.empty() && now - jumps_.front()[0] > 1.0) {jumps_.pop_front();}
    while (!yaw_hist_.empty() && now - yaw_hist_.front()[0] > 5.0) {yaw_hist_.pop_front();}
    while (!spikes_.empty() && now - spikes_.front() > 1.0) {spikes_.pop_front();}
    for (const auto & j : jumps_) {m.jump_m = std::max(m.jump_m, j[1]); m.jump_deg = std::max(m.jump_deg, j[2]);}
    if (yaw_hist_.size() > 1 && yaw_hist_.back()[0] - yaw_hist_.front()[0] > 4.0) {
      m.yaw_fix_rate = std::fabs(wrap_angle(yaw_hist_.back()[1] - yaw_hist_.front()[1])) * 180.0 / M_PI /
        (yaw_hist_.back()[0] - yaw_hist_.front()[0]);
    }

    // 지도 맞음: 지금 못 재면 옛 값으로 정상이라 하지 않음
    const auto r = (m.map_odom_age && *m.map_odom_age <= 1.0) ? match_ratio(now) : std::nullopt;
    if (r) {
      match_hist_.push_back(*r);
      while (match_hist_.size() > 5) {match_hist_.pop_front();}
      match_t_ = now;
    } else {
      match_hist_.clear();
    }
    if (!match_hist_.empty()) {
      std::vector<double> v(match_hist_.begin(), match_hist_.end());
      std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
      m.match = v[v.size() / 2];
    }
    if (m.match && *m.match < 0.60) {
      if (!low_since_) {low_since_ = now;}
      m.match_low_s = now - *low_since_;
    } else {
      low_since_.reset();
    }
    m.match_unknown_s = now - match_t_;

    if (!scan_times_.empty()) {
      m.scan_age = now - scan_times_.back();
      if (now - scan_times_.front() > 1.0) {
        int k = 0;
        for (double t : scan_times_) {k += (now - t < 1.0);}
        m.scan_hz = k;
      }
    }
    if (scan_) {
      int k = 0;
      for (float x : scan_->ranges) {k += std::isfinite(x) && x > scan_->range_min;}
      m.scan_points = k;
    }
    if (imu_t_) {m.imu_age = now - *imu_t_;}
    m.gyro_spikes = static_cast<int>(spikes_.size());
    if (acc_bad_since_) {m.acc_bad_s = now - *acc_bad_since_;}

    // 판정: 올라갈 땐 바로, 내려갈 땐 원인 없어진 뒤 hold_s 더
    const auto res = evaluate(m);
    if (res.level == 2) {last_err_ = now;}
    if (res.level >= 1) {last_warn_ = now; reasons_ = res.reasons;}
    level_ = now - last_err_ < hold_s_ ? 2 : now - last_warn_ < hold_s_ ? 1 : 0;
    if (level_ == 0) {reasons_.clear();}

    diagnostic_msgs::msg::DiagnosticStatus out;
    out.level = static_cast<std::uint8_t>(level_);
    out.name = "localization";
    out.hardware_id = base_;
    for (std::size_t i = 0; i < reasons_.size(); ++i) {out.message += (i ? " · " : "") + reasons_[i];}
    if (out.message.empty()) {out.message = "정상";}
    auto kv = [&out](const char * k, std::optional<double> v) {
        diagnostic_msgs::msg::KeyValue x;
        x.key = k;
        x.value = v ? std::to_string(*v) : "";
        out.values.push_back(x);
      };
    kv("scan_age", m.scan_age);
    kv("scan_hz", m.scan_hz);
    kv("scan_points", m.scan_points ? std::optional<double>(*m.scan_points) : std::nullopt);
    kv("match", m.match);
    kv("map_odom_age", m.map_odom_age);
    kv("jump_m", m.jump_m);
    kv("jump_deg", m.jump_deg);
    kv("yaw_fix_rate", m.yaw_fix_rate);
    kv("imu_age", m.imu_age);
    kv("bias_rad_s", bias_->estimate());
    pub_health_->publish(out);

    // 자이로 이동 치우침: 정상이고 map→odom 이 살아 있을 때만 배우고 보냄
    if (publish_bias_ && m2o_yaw) {
      bias_->add(now, *m2o_yaw, odom_speed_ > 0.02, level_);
      if (level_ == 0 && now - bias_pub_t_ >= 1.0) {
        geometry_msgs::msg::Vector3Stamped b;
        b.header.stamp = get_clock()->now();
        b.header.frame_id = base_;
        b.vector.z = bias_->estimate();
        pub_bias_->publish(b);
        bias_pub_t_ = now;
      }
    }
    const std::string match_txt = m.match ? std::to_string(static_cast<int>(*m.match * 100.0)) + "%" : "-";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s · 지도 맞음 %s · 자이로 보정 %+.3f °/s · %s",
      level_ == 0 ? "정상" : level_ == 1 ? "경고" : "정지", match_txt.c_str(),
      bias_->estimate() * 180.0 / M_PI, out.message.c_str());
  }

  std::string base_;
  double hold_s_{1.0};
  bool publish_bias_{true};
  std::string bias_file_;
  std::unique_ptr<BiasLearner> bias_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_, dist_map_;
  std::vector<std::int16_t> dist_;
  double dist_t_{-1e9};
  std::deque<double> scan_times_, spikes_;
  std::optional<double> imu_t_, acc_bad_since_, low_since_;
  double odom_speed_{0.0};
  bool have_m2o_{false};
  std::array<double, 3> m2o_{};
  std::deque<std::array<double, 3>> jumps_;
  std::deque<std::array<double, 2>> yaw_hist_;
  std::deque<double> match_hist_;
  double match_t_{0.0};
  double last_err_{-1e9}, last_warn_{-1e9}, bias_pub_t_{-1e9};
  int level_{0};
  std::vector<std::string> reasons_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_health_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_bias_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<navigation::PoseGuard>();
    rclcpp::spin(node);
  }   // 노드가 사라질 때 배운 치우침 저장
  rclcpp::shutdown();
  return 0;
}
