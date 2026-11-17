// amcl_settle.cpp — 서 있는 동안 AMCL 을 맞춰 줌 (localization:=amcl 때 nav2.launch.py 가 띄움)
//
// 왜: nav2 AMCL 은 바퀴가 update_min_d (5 cm) · update_min_a 만큼 움직여야 스캔으로 고침. 켤 때 출발 자리에 1–2° · 2–3 cm 틀리게 놓으면
//   서 있는 동안 추정이 initial_pose 에 그대로 → pose_guard '지도 맞음' 이 85 % 아래 (경고) → mission STANDBY 가
//   LOCALIZATION_HEALTH:1 로 안 풀림 (시뮬: 2° 틀리면 맞음 79 %, 계속 경고).
//   또 AMCL 의 initial_pose 파라미터 (set_initial_pose) 는 공분산 0 으로 시작 → 입자가 한 점이라 서서 스캔으로 고쳐도 안 움직임
//   (시뮬: request_nomotion_update 30 번 뒤에도 추정 0,0,0 그대로, 정답과 3.6 cm · 2°).
// 하는 것:
//   - AMCL 이 첫 /amcl_pose 를 내면 (= 켜짐 · 첫 스캔 고침. 켜지기 전 /initialpose 는 AMCL 이 버림 'not yet in the active state')
//     init_x · init_y · init_yaw_deg 자세를 퍼짐 init_std_xy · init_std_yaw_deg 로 /initialpose 한 번
//     (launch 의 initial_pose 와 같은 자리, init_std_xy ≤ 0 이면 안 보냄)
//   - 그 뒤 request_nomotion_update (std_srvs/Empty, 안 움직여도 지금 스캔으로 한 번 고침) 를 count 번 (period 마다, 기본 15)
//   - /initialpose 를 받을 때마다 (rviz 2D Pose Estimate · 운영자) 다시 count 번
//   - 시작 복구 (RESET→STARTUP · lifecycle_manager 재시작) 로 AMCL 이 cleanup 뒤 다시 켜지면 AMCL 은 initial_pose 파라미터가
//     아니라 자기가 마지막에 낸 자세에서 시작함 — 한 번도 켜진 적 없이 cleanup 됐으면 0,0,0.
//     켜진 적 없으면 아래 '켤 때 한 번' 이 시작 자세를 줌 (첫 /amcl_pose 뒤). 켜진 적 있으면 (/amcl/transition_event ACTIVE 두 번째부터)
//     첫 /amcl_pose 뒤, 로봇이 안 움직였을 때만 (odom 5 cm · 3° 안) 꺼지기 전 마지막 AMCL 추정을 퍼짐 있게 /initialpose 로 다시 주고
//     nomotion count 번 (AMCL 이 같은 자리를 써도 퍼짐 · 고침을 다시 함). 움직였으면 안 줌 (로그만)
//   - 서 있을 때만 (/odom 속도 < still_v · still_w). 움직이면 AMCL 이 원래대로 고치므로 안 부름
// 받는 것: /odom, /initialpose, /amcl_pose. 내는 것: /initialpose (켤 때 한 번). 부르는 것: service (기본 /request_nomotion_update)
// 실행: nav2.launch.py localization:=amcl 이 같이 띄움 (amcl_settle:=false 면 안 띄움)

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace navigation
{

class AmclSettle : public rclcpp::Node
{
public:
  AmclSettle()
  : Node("amcl_settle")
  {
    service_ = declare_parameter<std::string>("service", "/request_nomotion_update");
    count_ = static_cast<int>(declare_parameter<int>("count", 15));   // 움직임 없는 고침은 퍼뜨림 없이 고르기만 → 10 번 넘으면 거의 안 바뀜
    const double period = declare_parameter<double>("period", 0.2);
    still_v_ = declare_parameter<double>("still_v", 0.01);
    still_w_ = declare_parameter<double>("still_w", 0.02);
    init_x_ = declare_parameter<double>("init_x", 0.0);
    init_y_ = declare_parameter<double>("init_y", 0.0);
    init_yaw_ = declare_parameter<double>("init_yaw_deg", 0.0) * M_PI / 180.0;
    init_std_xy_ = declare_parameter<double>("init_std_xy", 0.05);
    init_std_yaw_ = declare_parameter<double>("init_std_yaw_deg", 3.0) * M_PI / 180.0;
    frame_ = declare_parameter<std::string>("frame", "map");
    seeded_ = !(init_std_xy_ > 0.0);
    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    if (count_ < 0 || !(period > 0.0)) {
      throw std::invalid_argument("amcl_settle: count >= 0 · period > 0");
    }
    cli_ = create_client<std_srvs::srv::Empty>(service_);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & t = m->twist.twist;
        still_ = std::hypot(t.linear.x, t.linear.y) < still_v_ && std::fabs(t.angular.z) < still_w_;
        const auto & q = m->pose.pose.orientation;
        odom_ = {m->pose.pose.position.x, m->pose.pose.position.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
        have_odom_ = true;
      });
    // AMCL 은 amcl_pose 를 transient_local · reliable 로 냄 → 늦게 떠도 마지막 것을 받음
    sub_amcl_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        amcl_seen_ = true;
        if (amcl_active_ && !restore_ && have_odom_) {   // 켜져 있는 동안의 추정만 (다시 켜진 뒤 0,0,0 은 돌려준 뒤부터)
          last_good_ = *m;
          odom_at_good_ = odom_;
          have_good_ = true;
        }
      });
    // AMCL lifecycle 전환 (RESET→STARTUP · manager 재시작 뒤 다시 켜짐을 앎)
    sub_tr_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>("/amcl/transition_event", 10,
      [this](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr e) {
        if (e->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          amcl_active_ = true;
          if (++activations_ >= 2) {
            amcl_seen_ = false;   // 다시 켜진 뒤 첫 추정을 기다림
            restore_ = true;
            RCLCPP_WARN(get_logger(), "AMCL 다시 켜짐 (%d 번째) → 첫 /amcl_pose 뒤 %s 돌려줌 (안 움직였을 때만)", activations_,
              have_good_ ? "꺼지기 전 마지막 추정" : "시작 자세");
          }
        } else if (e->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          amcl_active_ = false;   // 꺼지는 중: 마지막 추정을 그대로 둠
        }
      });
    sub_init_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        seeded_ = true;  // 자동 맞춤/외부 초기 자세를 launch 시작 좌표로 덮지 않음
        left_ = count_;
        RCLCPP_INFO(get_logger(), "/initialpose 받음 → 서 있는 동안 %s %d 번", service_.c_str(), count_);
      });
    left_ = count_;
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] {tick();});
    RCLCPP_INFO(get_logger(), "AMCL 서 있을 때 맞추기: %s %d 번 (%.1f s 마다, 켤 때 · /initialpose 뒤)", service_.c_str(), count_, period);
  }

private:
  void tick()
  {
    if (busy_) {   // 답이 안 옴 (AMCL 다시 켜짐 · 요청 잃음) → 2 s 뒤 버리고 다시 (busy 로 영영 멈추지 않게)
      if ((now() - sent_).seconds() < 2.0) {return;}
      cli_->prune_pending_requests();
      busy_ = false;
      RCLCPP_WARN(get_logger(), "%s 답 2 s 넘게 없음 → 다시 보냄", service_.c_str());
    }
    if (left_ <= 0 && !restore_) {return;}
    if (!have_odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "/odom 기다림 (서 있는지 몰라 안 부름)");
      return;
    }
    if (!have_start_) {   // 켤 때 odom 자리 (시작 자세를 돌려줄 때 안 움직였는지 비교)
      odom_start_ = odom_;
      have_start_ = true;
    }
    if (!still_) {return;}
    if (!cli_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "%s 기다림 (AMCL 켜지기 전)", service_.c_str());
      return;
    }
    if (!amcl_seen_) {   // AMCL 이 켜져 첫 추정을 내기 전 (그 전 /initialpose · nomotion 은 AMCL 이 버림)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "AMCL 첫 /amcl_pose 기다림 (켜지기 전)");
      return;
    }
    if (restore_) {   // AMCL 이 다시 켜짐 → 0,0,0 이 아니라 꺼지기 전 자세 (없으면 시작 자세) 로
      restore_ = false;
      seeded_ = true;
      const double ox = have_good_ ? odom_at_good_[0] : odom_start_[0];
      const double oy = have_good_ ? odom_at_good_[1] : odom_start_[1];
      const double oyaw = have_good_ ? odom_at_good_[2] : odom_start_[2];
      const double moved = std::hypot(odom_[0] - ox, odom_[1] - oy);
      const double turned = std::fabs(std::remainder(odom_[2] - oyaw, 2.0 * M_PI));
      if (moved > 0.05 || turned > 3.0 * M_PI / 180.0) {
        RCLCPP_WARN(get_logger(), "AMCL 다시 켜졌지만 그새 움직임 (%.0f mm · %.1f°) → 자세 안 돌려줌 (auto_initial_pose · 운영자)",
          moved * 1000.0, turned * 180.0 / M_PI);
        return;
      }
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      if (have_good_) {
        m = last_good_;
      } else {
        m.pose.pose.position.x = init_x_;
        m.pose.pose.position.y = init_y_;
        m.pose.pose.orientation.z = std::sin(init_yaw_ / 2.0);
        m.pose.pose.orientation.w = std::cos(init_yaw_ / 2.0);
      }
      m.header.stamp = now();
      m.header.frame_id = frame_;
      m.pose.covariance = {};
      m.pose.covariance[0] = m.pose.covariance[7] = init_std_xy_ * init_std_xy_;
      m.pose.covariance[35] = init_std_yaw_ * init_std_yaw_;
      pub_init_->publish(m);
      const auto & q = m.pose.pose.orientation;
      RCLCPP_WARN(get_logger(), "AMCL 다시 켜짐 → %s (%.3f, %.3f, %.1f°) 를 /initialpose 로 돌려줌", have_good_ ? "꺼지기 전 추정" : "시작 자세",
        m.pose.pose.position.x, m.pose.pose.position.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 180.0 / M_PI);
      return;   // 다음 주기부터 nomotion (/initialpose 받으면 left_ = count_)
    }
    if (!seeded_) {   // 켤 때 한 번: 같은 시작 자세를 퍼짐 있게 다시 줌 (AMCL initial_pose 는 공분산 0)
      seeded_ = true;
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      m.header.stamp = now();
      m.header.frame_id = frame_;
      m.pose.pose.position.x = init_x_;
      m.pose.pose.position.y = init_y_;
      m.pose.pose.orientation.z = std::sin(init_yaw_ / 2.0);
      m.pose.pose.orientation.w = std::cos(init_yaw_ / 2.0);
      m.pose.covariance[0] = m.pose.covariance[7] = init_std_xy_ * init_std_xy_;
      m.pose.covariance[35] = init_std_yaw_ * init_std_yaw_;
      pub_init_->publish(m);
      RCLCPP_INFO(get_logger(), "시작 자세 (%.3f, %.3f, %.1f°) 를 퍼짐 ±%.0f cm · ±%.1f° 로 /initialpose", init_x_, init_y_,
        init_yaw_ * 180.0 / M_PI, init_std_xy_ * 100.0, init_std_yaw_ * 180.0 / M_PI);
      return;   // 다음 주기부터 nomotion (AMCL 이 새 입자를 만든 뒤)
    }
    busy_ = true;
    sent_ = now();
    cli_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        busy_ = false;
        if (--left_ == 0) {RCLCPP_INFO(get_logger(), "서 있을 때 맞추기 %d 번 끝", count_);}
      });
  }

  std::string service_, frame_;
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0}, init_std_xy_{0.05}, init_std_yaw_{0.05};
  bool seeded_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  int count_{30};
  double still_v_{0.01}, still_w_{0.02};
  std::atomic<int> left_{0};
  std::atomic<bool> still_{false}, busy_{false}, have_odom_{false}, amcl_seen_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_;
  rclcpp::Time sent_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
  rclcpp::TimerBase::SharedPtr timer_;
  // AMCL 다시 켜짐 (모두 단일 스레드 executor 콜백 안에서만)
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub_tr_;
  std::array<double, 3> odom_{{0.0, 0.0, 0.0}}, odom_start_{{0.0, 0.0, 0.0}}, odom_at_good_{{0.0, 0.0, 0.0}};
  bool have_start_{false}, have_good_{false}, amcl_active_{false}, restore_{false};
  int activations_{0};
  geometry_msgs::msg::PoseWithCovarianceStamped last_good_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::AmclSettle>());
  rclcpp::shutdown();
  return 0;
}
