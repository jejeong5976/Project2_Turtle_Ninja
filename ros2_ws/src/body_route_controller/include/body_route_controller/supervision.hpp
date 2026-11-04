#pragma once
#include "body_route_controller/core.hpp"
namespace body_route_controller
{
struct TrackingLimits
{
  double measured_speed = .04, measured_yaw_rate = .15;
  double route_distance = .10, translating_heading = .25;
  double turn_drift = .025, turn_speed = .008;
  double braking_deceleration = .05, feedback_delay = .3;
};
// Secondary trial checks, in addition to the controller's live footprint sweep.
// Heading is relative to the current controller target in odom, never to the
// trial's initial yaw. Map pose and path must be in the same map frame.
class TrackingGuard
{
public:
  explicit TrackingGuard(TrackingLimits limits = {}) : limits_(limits) {}
  std::string check(const Pose & map_pose, const Pose & odom_pose,
    const Velocity & measured, const std::vector<Point> & path,
    const std::string & mode, double target_heading, bool custom_mode = true);
  void reset() {turning_ = false; braking_ = false;}
private:
  TrackingLimits limits_;
  bool turning_ = false;
  Point turn_start_, brake_start_;
  bool braking_ = false;
  double brake_distance_ = 0;
};
}
