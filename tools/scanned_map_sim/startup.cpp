#include "common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;
using namespace maze_sim;
using Manage = nav2_msgs::srv::ManageLifecycleNodes;
using State = lifecycle_msgs::srv::GetState;
using Clock = std::chrono::steady_clock;

// A startup barrier only: never publishes velocity, goals, pose or hold release.
// The lifecycle managers retain ownership of transitions and bonds.
class Startup : public rclcpp::Node {
public:
  Startup() : Node("simulation_startup"), tf_(get_clock()), listener_(tf_) {
    const double seconds = declare_parameter("timeout_s", 60.0);
    if (!std::isfinite(seconds) || seconds < 3 || seconds > 120)
      throw std::runtime_error("startup timeout must be 3..120 seconds");
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    for (const auto *name : {"lifecycle_manager_localization", "lifecycle_manager_navigation"})
      managers_.push_back(create_client<Manage>(std::string("/") + name + "/manage_nodes"));
    for (const auto *name : {"map_server", "amcl", "planner_server", "controller_server", "behavior_server", "bt_navigator", "collision_monitor"}) {
      names_.emplace_back(name);
      states_.push_back(create_client<State>(std::string("/") + name + "/get_state"));
    }
  }
  void run() {
    executor_.add_node(get_node_base_interface());
    phase_ = "discovery";
    while (true) {
      checkDeadline();
      bool ready = true;
      for (const auto &client : managers_) if (!client->service_is_ready()) {
        ready = false; detail_ = client->get_service_name();
      }
      for (const auto &client : states_) if (!client->service_is_ready()) {
        ready = false; detail_ = client->get_service_name();
      }
      if (ready) break;
      pump();
    }
    // Service presence alone does not mean reply-reader discovery has settled.
    phase_ = "reply_discovery"; detail_ = "allowing lifecycle reply endpoints to match";
    const auto settle = Clock::now() + 3s;
    while (Clock::now() < settle) {checkDeadline(); pump();}
    startManager(0, "localization_activation");
    phase_ = "localization_transform"; detail_ = "waiting for map -> base_footprint";
    RCLCPP_INFO(get_logger(), "%s", detail_.c_str());
    while (!tf_.canTransform("map", "base_footprint", tf2::TimePointZero)) {
      checkDeadline(); pump();
    }
    startManager(1, "navigation_activation");
    phase_ = "active_state_verification";
    for (std::size_t i = 0; i < states_.size(); ++i) {
      detail_ = names_[i];
      auto request = states_[i]->async_send_request(std::make_shared<State::Request>());
      if (!await(request, 3.0)) {
        states_[i]->remove_pending_request(request);
        throw std::runtime_error("get_state response timeout: " + names_[i]);
      }
      if (request.get()->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        throw std::runtime_error("node not active: " + names_[i]);
    }
    phase_ = "ready"; detail_ = "localization and navigation active; trial may begin";
    write(true, detail_);
    RCLCPP_INFO(get_logger(), "STARTUP READY: %s", detail_.c_str());
  }
  void fail(const std::string &reason) {
    write(false, reason);
    RCLCPP_ERROR(get_logger(), "STARTUP FAILED [%s / %s]: %s", phase_.c_str(), detail_.c_str(), reason.c_str());
  }
private:
  void checkDeadline() {
    if (!rclcpp::ok()) throw std::runtime_error("startup interrupted");
    if (Clock::now() >= deadline_) throw std::runtime_error("startup deadline: " + phase_ + " / " + detail_);
  }
  void pump() {executor_.spin_some(); std::this_thread::sleep_for(20ms);}
  template<class Future> bool await(Future &future, double seconds) {
    const auto until = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    while (future.wait_for(0s) != std::future_status::ready) {
      checkDeadline(); if (Clock::now() >= until) return false; pump();
    }
    return true;
  }
  void startManager(std::size_t index, const std::string &phase) {
    phase_ = phase; detail_ = managers_[index]->get_service_name();
    RCLCPP_INFO(get_logger(), "Startup phase: %s", phase_.c_str());
    auto message = std::make_shared<Manage::Request>(); message->command = Manage::Request::STARTUP;
    auto response = managers_[index]->async_send_request(message);
    if (!await(response, 25.0)) {
      managers_[index]->remove_pending_request(response);
      throw std::runtime_error("lifecycle manager response timeout: " + detail_);
    }
    if (!response.get()->success) throw std::runtime_error("lifecycle manager rejected startup: " + detail_);
  }
  void write(bool success, const std::string &reason) {
    YAML::Node report; report["startup_passed"] = success; report["phase"] = phase_;
    report["detail"] = detail_; report["reason"] = reason;
    std::ofstream out(runRoot() / "startup.yaml"); out << YAML::Dump(report) << '\n';
    if (!out) throw std::runtime_error("cannot write startup report");
  }
  rclcpp::executors::SingleThreadedExecutor executor_;
  tf2_ros::Buffer tf_; tf2_ros::TransformListener listener_;
  Clock::time_point deadline_;
  std::vector<rclcpp::Client<Manage>::SharedPtr> managers_;
  std::vector<rclcpp::Client<State>::SharedPtr> states_;
  std::vector<std::string> names_;
  std::string phase_ = "initialization", detail_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto startup = std::make_shared<Startup>();
    try {startup->run();} catch (const std::exception &e) {startup->fail(e.what()); code = 2;}
  } catch (const std::exception &e) {std::cerr << "STARTUP FAILED: " << e.what() << '\n'; code = 2;}
  rclcpp::shutdown(); return code;
}
