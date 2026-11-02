// 앞 · 뒤 라이다 → 몸 중심 360° 스캔 /scan 하나 (slam_toolbox · 앞 물체 정지가 받음).
//
// 받기: scans 의 LaserScan (SensorDataQoS). 내기: out, frame_id = frame, bins 칸 — 칸마다 가장 가까운 점.
// 각 라이다 장착 자세는 TF(고정)에서 처음 한 번 읽음. 몸 상자(body_half) 안 점(자기 바퀴 · 기둥 · 선)은 버림.
// 첫 번째 라이다가 올 때마다 냄 (시각 = 그 스캔 시각). 나머지 라이다는 그 시각에서 max_age 안이어야 함.
// 한 라이다라도 max_age 를 넘으면 /scan 을 내지 않음 → 받는 쪽(앞 물체 정지 · 위치 감시)이 "스캔 끊김" 으로 멈춤.
//   (한쪽만으로 내면 그쪽 방향이 비어 "장애물 없음" 과 구분되지 않음)
// 두 라이다 시각 차 동안 몸이 움직인 만큼은 보정하지 않음: 0.10 m/s · 0.1 s = 1 cm.

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "navigation/scan_merge.hpp"
#include "rclcpp/rclcpp.hpp"
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
    body_ = BodyBox{half[0], half[1]};

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out, rclcpp::SensorDataQoS());
    sources_.resize(topics.size());
    for (std::size_t i = 0; i < topics.size(); ++i) {
      sources_[i].topic = topics[i];
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(i, *m);}));
    }
    RCLCPP_INFO(get_logger(), "스캔 합치기: 라이다 %zu 개 → %s (%s, %d 칸), 몸 상자 ±%.2f × ±%.2f m",
      topics.size(), out.c_str(), frame_.c_str(), bins_, body_.half_x, body_.half_y);
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

  void on_scan(std::size_t i, const sensor_msgs::msg::LaserScan & m)
  {
    Mount mount{};
    if (!mount_of(m.header.frame_id, mount)) {return;}
    auto & s = sources_[i];
    scan_to_points(m.ranges, m.angle_min, m.angle_increment,
      std::max<double>(m.range_min, range_min_), std::min<double>(m.range_max, range_max_), mount, body_, s.pts);
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
  BodyBox body_{0.17, 0.19};
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
