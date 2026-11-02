// cmd_vel_hold.cpp — /mission/hold 를 받아 바퀴 속도 명령을 막는 문 (Nav2 와 swerve_controller 사이)
// =====================================================================
// 역할 : planning 이 멈추라고 하면 (hold = true) Nav2 속도 명령을 버리고 0 을 보냄
// 받는 것
//   cmd_vel_in   (기본 cmd_vel_safe, TwistStamped)  Nav2 collision_monitor 출력
//   /mission/hold (std_msgs/Bool)                   mission_manager (STANDBY · FAULT · 주행 막힘이면 true, 1 Hz + 바뀔 때)
// 보내는 것
//   cmd_vel_out  (기본 /cmd_vel, TwistStamped)      swerve_controller 가 받음
// 규칙
//   - hold = false : 받은 명령을 그대로 넘김 (이 노드가 따로 내는 것 없음)
//   - hold = true  : 받은 명령은 버리고, 0 을 zero_hz 로 계속 보냄 (swerve_controller 의 0.3 s 끊김 정지보다 먼저 확실히 멈춤)
//   - hold 를 한 번도 못 받았으면 start_hold (기본 false = 넘김) — planning 없이 Nav2 만 시험할 때
//   - hold_timeout_s > 0 이고 hold 가 그만큼 안 오면 hold 로 봄 (planning 이 죽었을 때 멈춤). 0 = 마지막 값 유지
// - 수동 조종 (teleop) 은 /cmd_vel 로 직접 보내면 이 문을 안 거침

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using geometry_msgs::msg::TwistStamped;

class CmdVelHold : public rclcpp::Node
{
public:
  CmdVelHold()
  : Node("cmd_vel_hold")
  {
    const auto in_topic = declare_parameter<std::string>("cmd_vel_in", "cmd_vel_safe");
    const auto out_topic = declare_parameter<std::string>("cmd_vel_out", "/cmd_vel");
    const auto hold_topic = declare_parameter<std::string>("hold_topic", "/mission/hold");
    hold_ = declare_parameter<bool>("start_hold", false);
    hold_timeout_s_ = declare_parameter<double>("hold_timeout_s", 0.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    const double zero_hz = declare_parameter<double>("zero_hz", 20.0);

    pub_ = create_publisher<TwistStamped>(out_topic, 10);
    sub_cmd_ = create_subscription<TwistStamped>(in_topic, 10, [this](TwistStamped::ConstSharedPtr m) {
        if (held(now_s())) {return;}   // 막는 중: 버림 (0 은 타이머가 보냄)
        pub_->publish(*m);
      });
    sub_hold_ = create_subscription<std_msgs::msg::Bool>(hold_topic, 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data != hold_ || !have_hold_) {
          RCLCPP_INFO(get_logger(), "hold %s", m->data ? "켬 → 바퀴 0" : "끔 → Nav2 명령 넘김");
        }
        hold_ = m->data;
        have_hold_ = true;
        last_hold_s_ = now_s();
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(zero_hz, 1.0)), [this]() {
        if (!held(now_s())) {return;}
        TwistStamped z;   // 모두 0
        z.header.stamp = get_clock()->now();
        z.header.frame_id = frame_id_;
        pub_->publish(z);
      });
    RCLCPP_INFO(get_logger(), "%s → [%s] → %s (처음 %s, 시간 제한 %.1f s)", in_topic.c_str(), hold_topic.c_str(),
      out_topic.c_str(), hold_ ? "막음" : "넘김", hold_timeout_s_);
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  bool held(double now) const
  {
    if (have_hold_ && hold_timeout_s_ > 0.0 && now - last_hold_s_ > hold_timeout_s_) {return true;}   // planning 끊김
    return hold_;
  }

  bool hold_ = false, have_hold_ = false;
  double hold_timeout_s_ = 0.0, last_hold_s_ = 0.0;
  std::string frame_id_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hold_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelHold>());
  rclcpp::shutdown();
  return 0;
}
