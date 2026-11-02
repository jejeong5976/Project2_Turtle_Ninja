// arm_state_publisher.cpp — 표시 · TF 용 관절 상태: /joint_states(엔코더) → /joint_states_corrected
// =====================================================================
// robot_state_publisher 만 이 출력을 읽음 (TF = 실제 팔 자세). control_master · MoveIt 은 엔코더 /joint_states 그대로
//   (MoveIt 실행의 시작 자세 비교 · 접힘 확인 · 하드웨어 감시는 모터가 보는 값이어야 해서)
// ① 처짐: J2 · J3 · J4 = 엔코더 + c · τ (arm_state.hpp). 손목 카메라 보정이 처짐 보정한 손목 기준이라 TF 가 이래야 카메라 위치가 맞음
//    받침에 얹혀 있으면 0 (rest_near 안), 받침에서 rest_far 만큼 떨어지면 전부 — 사이는 직선 (실물에서 확인할 값)
// ② 그리퍼 연결 막대: gripper_left_joint 로 크랭크 · 양쪽 막대 관절각 (URDF 수동 관절)
// 위치만 내보냄 (속도 · 토크는 측정값이 아님). NaN 인 관절은 빼고 나머지는 그대로 → 한 관절 문제로 TF 전체가 멈추지 않음
//   단 팔 관절이 하나라도 없거나 NaN 이면 그 순간 팔 관절 5 개를 다 뺌 (처짐 보정 없이 나머지 팔 관절만 보내면 TF 의미가 바뀜 → 안 보내 TF 가 멈춘 걸로 드러냄)
// ③ MoveIt 용 /joint_states_moveit: 엔코더 값 그대로 + 그리퍼 연결 막대 3 관절 (MoveIt 은 엔코더 기준, 막대 각도만 필요)
// 입력 시각 그대로, 타이머 · 명령 없음

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "manipulation/arm_model_params.hpp"
#include "manipulation/arm_state.hpp"

namespace manipulation
{

class ArmStatePublisher : public rclcpp::Node
{
public:
  ArmStatePublisher()
  : Node("arm_state_publisher")
  {
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper_left_joint");
    linkage_joints_ = declare_parameter<std::vector<std::string>>(
      "linkage_joints", {"gripper_crank_joint", "gripper_left_rod_joint", "gripper_right_rod_joint"});
    const bool sag_param = declare_parameter<bool>("sag_correction", true);
    sag_on_ = sag_param && load_arm_model(*this, model_);
    // 받침(접은 자세, control.yaml poses.home 과 같은 값) 근처는 받침대가 무게를 받쳐 처짐 없음 → 떨어지는 동안 서서히 켬
    const auto rest = declare_parameter<std::vector<double>>("rest_pose", {-0.0061, -1.4511, 0.9511, -0.1181, 0.0230});
    for (std::size_t k = 0; k < 5 && k < rest.size(); ++k) {rest_[k] = rest[k];}
    rest_near_ = declare_parameter<double>("rest_near", 0.05);
    rest_far_ = declare_parameter<double>("rest_far", 0.15);
    linkage_on_ = declare_parameter<bool>("gripper_linkage_on", true) && load_gripper_linkage(*this, linkage_);
    const auto in = declare_parameter<std::string>("input_topic", "joint_states");
    const auto out = declare_parameter<std::string>("output_topic", "joint_states_corrected");
    const auto moveit_out = declare_parameter<std::string>("moveit_topic", "joint_states_moveit");
    pub_ = create_publisher<sensor_msgs::msg::JointState>(out, 10);
    moveit_pub_ = create_publisher<sensor_msgs::msg::JointState>(moveit_out, 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      in, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_state(*m);});
    RCLCPP_INFO(get_logger(), "%s → %s · 처짐 보정 %s · 그리퍼 연결 막대 %s", in.c_str(), out.c_str(),
      sag_on_ ? "켬" : (sag_param ? "끔 (arm_model 없음)" : "끔"), linkage_on_ ? "켬" : "끔 (gripper_linkage 없음)");
  }

private:
  void on_state(const sensor_msgs::msg::JointState & m)
  {
    if (m.name.size() != m.position.size()) {return;}
    sensor_msgs::msg::JointState o;
    o.header = m.header;
    std::array<int, 5> arm_idx{{-1, -1, -1, -1, -1}};
    double grip = std::nan("");
    bool bad = false;
    for (std::size_t i = 0; i < m.name.size(); ++i) {
      const double p = m.position[i];
      if (!std::isfinite(p)) {bad = true; continue;}
      bool linkage = false;
      for (const auto & l : linkage_joints_) {linkage = linkage || m.name[i] == l;}
      if (linkage) {continue;}   // 계산값으로 바꿈
      for (std::size_t k = 0; k < arm_joints_.size() && k < 5; ++k) {
        if (m.name[i] == arm_joints_[k]) {arm_idx[k] = static_cast<int>(o.name.size());}
      }
      if (m.name[i] == gripper_joint_) {grip = p;}
      o.name.push_back(m.name[i]);
      o.position.push_back(p);
    }
    if (bad) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "측정 없음(NaN) 관절은 빼고 보냄");}
    bool all_arm = true;
    for (int k : arm_idx) {all_arm = all_arm && k >= 0;}
    sensor_msgs::msg::JointState mv = o;   // MoveIt 용: 엔코더 그대로
    if (sag_on_ && all_arm) {
      std::array<double, 5> q;
      for (int k = 0; k < 5; ++k) {q[k] = o.position[arm_idx[k]];}
      const auto c = sag_corrected(model_, q, 0.0, sag_weight(q, rest_, rest_near_, rest_far_));
      for (int k = 0; k < 5; ++k) {o.position[arm_idx[k]] = c[k];}
    } else if (sag_on_ && !all_arm) {
      // 일부 팔 관절만 있음 → 이 순간 팔 관절은 하나도 안 보냄 (처짐 보정 안 된 값이 섞이지 않게)
      sensor_msgs::msg::JointState keep;
      keep.header = o.header;
      for (std::size_t i = 0; i < o.name.size(); ++i) {
        bool arm = false;
        for (std::size_t k = 0; k < arm_joints_.size() && k < 5; ++k) {arm = arm || o.name[i] == arm_joints_[k];}
        if (!arm) {keep.name.push_back(o.name[i]); keep.position.push_back(o.position[i]);}
      }
      o = keep;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "팔 관절 측정이 빠져 이 순간 팔 TF 를 안 보냄");
    }
    std::array<double, 3> lk;
    if (linkage_on_ && std::isfinite(grip)) {
      if (solve_linkage(linkage_, grip, lk)) {
        for (std::size_t k = 0; k < 3 && k < linkage_joints_.size(); ++k) {
          o.name.push_back(linkage_joints_[k]);
          o.position.push_back(lk[k]);
          mv.name.push_back(linkage_joints_[k]);
          mv.position.push_back(lk[k]);
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "그리퍼 %.4f m 에서 연결 막대 계산 안 됨 (범위 밖)", grip);
      }
    }
    pub_->publish(o);
    moveit_pub_->publish(mv);
  }

  std::vector<std::string> arm_joints_, linkage_joints_;
  std::string gripper_joint_;
  ArmModel model_;
  GripperLinkage linkage_;
  bool sag_on_{false}, linkage_on_{false};
  std::array<double, 5> rest_{};
  double rest_near_{0.05}, rest_far_{0.15};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_, moveit_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manipulation::ArmStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
