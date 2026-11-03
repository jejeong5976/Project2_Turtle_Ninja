#include "body_route_controller/controller.hpp"
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller_exceptions.hpp>

TEST(Plugin, DiscoverableAndLoadsAgainstInstalledNav2Api)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  ASSERT_TRUE(loader.isClassAvailable("body_route_controller::BodyRouteController"));
  auto plugin = loader.createSharedInstance("body_route_controller::BodyRouteController");
  ASSERT_NE(plugin, nullptr);
  EXPECT_THROW(plugin->newPathReceived(nav_msgs::msg::Path{}), nav2_core::InvalidPath);
}
