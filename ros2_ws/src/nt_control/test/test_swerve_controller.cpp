#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using controller_interface::CallbackReturn;

class SwerveControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    loader = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
      "controller_interface", "controller_interface::ControllerInterface");
    controller = loader->createSharedInstance("nt_control/SwerveController");
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "position_rate_test";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_namespace = "/position_rate_test";
    params.node_options.use_intra_process_comms(true);
    ASSERT_EQ(controller->init(params), controller_interface::return_type::OK);
  }

  void configure(const std::string & source)
  {
    controller->get_node()->set_parameters({
      rclcpp::Parameter("steer_rate_source", source),
      rclcpp::Parameter("require_motion_mode", false),
      rclcpp::Parameter("use_imu", false),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("steer_tolerance", 0.003),
      rclcpp::Parameter("cmd_vel_topic", "/position_rate_test/cmd_vel")});
    ASSERT_EQ(controller->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (const auto & name : controller->state_interface_configuration().names) {
      const auto slash = name.rfind('/');
      auto state = std::make_shared<hardware_interface::StateInterface>(
        name.substr(0, slash), name.substr(slash+1));
      ASSERT_TRUE(state->set_value(0.0));
      states.push_back(state);
      state_loans.emplace_back(state);
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    for (const auto & name : controller->command_interface_configuration().names) {
      const auto slash = name.rfind('/');
      auto command = std::make_shared<hardware_interface::CommandInterface>(
        name.substr(0, slash), name.substr(slash+1));
      ASSERT_TRUE(command->set_value(0.0));
      commands.push_back(command);
      command_loans.emplace_back(command);
    }
    controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
  }

  void publish_command(double vx = 0.009, double vy = 0.0008)
  {
    auto node = std::make_shared<rclcpp::Node>("rate_test_sender",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/position_rate_test/cmd_vel", 10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller->get_node()->get_node_base_interface());
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(pub->get_subscription_count(), 0u);
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;  // default ~5 degrees: force the steering-before-drive branch
    for (int i = 0; i < 10; ++i) {
      pub->publish(cmd);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    executor.spin_some();
  }

  void update()
  {
    EXPECT_EQ(controller->update(controller->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }

  // Destroy the controller (and its loans) before handles and plugin loader.
  std::unique_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  std::shared_ptr<controller_interface::ControllerInterface> controller;
};

TEST_F(SwerveControllerTest, PositionInterfaceCompensationAndInvalidEstimateStop)
{
  configure("position");
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(states[2*i+1]->get_interface_name(), "position_velocity");
    ASSERT_TRUE(states[2*i+1]->set_value(0.16));
  }
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  publish_command();
  update();
  for (int i = 0; i < 4; ++i) {
    const double offset = i % 2 == 0 ? 0.0269 : -0.0269;
    EXPECT_NEAR(commands[4+i]->get_optional<double>().value(), -0.16*offset/0.033, 1e-8);
  }
  ASSERT_TRUE(states[1]->set_value<double>(NAN));
  update();
  for (int i = 4; i < 8; ++i) {EXPECT_DOUBLE_EQ(commands[i]->get_optional<double>().value(), 0.0);}
  // Recovery without a fresh cmd_vel must not revive the old motion command.
  ASSERT_TRUE(states[1]->set_value(0.16));
  update();
  for (int i = 4; i < 8; ++i) {EXPECT_DOUBLE_EQ(commands[i]->get_optional<double>().value(), 0.0);}
}

TEST_F(SwerveControllerTest, PositionActivationRequiresReadyEstimate)
{
  configure("position");
  ASSERT_TRUE(states[1]->set_value<double>(NAN));
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, ReportedDefaultInterfaceUnchanged)
{
  configure("reported");
  for (int i = 0; i < 4; ++i) {EXPECT_EQ(states[2*i+1]->get_interface_name(), "velocity");}
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(SwerveControllerTest, UnknownSourceRejected)
{
  controller->get_node()->set_parameter(rclcpp::Parameter("steer_rate_source", "typo"));
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, IncompleteModuleRangesRejected)
{
  controller->get_node()->set_parameter(rclcpp::Parameter("steer_min",std::vector<double>{0,0,0,0}));
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()),CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, ReversedModuleRangeRejected)
{
  controller->get_node()->set_parameters({
    rclcpp::Parameter("steer_min",std::vector<double>{1,0,0,0}),
    rclcpp::Parameter("steer_max",std::vector<double>{0,1,1,1})});
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()),CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, ModuleRangesReachCommandsAndRejectDiagonal)
{
  controller->get_node()->set_parameters({
    rclcpp::Parameter("steer_min",std::vector<double>{-M_PI/2,0,0,-M_PI/2}),
    rclcpp::Parameter("steer_max",std::vector<double>{0,M_PI/2,M_PI/2,0})});
  configure("reported");
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  publish_command(0,.05);
  update();
  const double sign[]={-1,1,1,-1};
  for (int i=0;i<4;++i) {
    EXPECT_NEAR(commands[i]->get_optional<double>().value(),sign[i]*M_PI/2,1e-8);
  }
  publish_command(.03,.03);
  update();
  for (int i=0;i<8;++i) {EXPECT_DOUBLE_EQ(commands[i]->get_optional<double>().value(),0.0);}
}
