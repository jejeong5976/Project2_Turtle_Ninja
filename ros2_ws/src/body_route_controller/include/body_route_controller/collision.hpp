#pragma once
#include "body_route_controller/core.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace body_route_controller
{
using Footprint = std::vector<geometry_msgs::msg::Point>;
// Caller holds the costmap mutex. Unknown/out-of-map space is rejected.
bool footprintSafe(nav2_costmap_2d::Costmap2D &, const Footprint &, const Pose &);
bool motionSafe(nav2_costmap_2d::Costmap2D &, const Footprint &, Pose,
  Velocity, double seconds, double linear_decel = 0.0, double angular_decel = 0.0);
bool turnSafe(nav2_costmap_2d::Costmap2D &, const Footprint &, Pose, double target_yaw);
}
