#pragma once
#include "body_route_controller/collision.hpp"
#include <optional>

namespace body_route_controller
{
// Experimental, opt-in shared B/C behavior. These allowances are hypotheses,
// not a replacement for localization/actuator measurements or outer guards.
struct PassageSettings
{
  bool enabled = false;
  double position_allowance = .05;
  double yaw_allowance = .08726646259971647;
  double response_delay = .5;
  void validate() const;
};

Footprint uncertaintyFootprint(const Footprint &, Point center, double position, double yaw);

class PreparedPassage
{
public:
  PreparedPassage(Settings = {}, PassageSettings = {});
  // Caller has checked hold, freshness, frames and feedback and holds map mutex.
  // A result is a proposal: the existing live collision/observation gates MUST
  // still approve it. nullopt delegates to the unchanged normal controller.
  std::optional<Result> step(nav2_costmap_2d::Costmap2D &, const Footprint &,
    const Pose &, const Velocity &, const std::vector<Point> &, const Pose & goal,
    double dt, double speed_limit, const std::vector<Velocity> & pending);
  void pause();  // Forget motion history, retain geometry only; never permission.
  void reset();  // Cancel, task end, lifecycle reset or invalidated route.
  bool enabled() const {return limits_.enabled;}
  bool active() const {return phase_ != Phase::Idle;}
  const std::string & reason() const {return reason_;}
  Point corner() const {return corner_;}
  Point exit() const {return exit_;}
  double crossingHeading() const {return outgoing_;}
private:
  enum class Phase {Idle, Approach, Align, Cross, Curve};
  Settings cfg_;
  PassageSettings limits_;
  Core curve_core_;
  Phase phase_ = Phase::Idle;
  Point start_, corner_, exit_;
  Pose goal_;
  double incoming_ = 0, outgoing_ = 0, local_cap_ = 0, previous_yaw_ = 0;
  Velocity previous_;
  std::vector<Point> route_;
  std::string reason_;
  bool paused_ = false;
  // Exact lethal/unknown-class comparison over the maneuver region; a new occupied
  // or unknown cell forces full geometry revalidation before another command.
  std::vector<unsigned char> observed_costs_;
  std::vector<double> observed_grid_;
  bool changed(nav2_costmap_2d::Costmap2D &, const Footprint &);
  bool routeMatches(const std::vector<Point> &, const Pose &) const;
  bool geometrySafe(nav2_costmap_2d::Costmap2D &, const Footprint &, double speed, bool check_body_and_turn = true) const;
  bool wideCurveSafe(nav2_costmap_2d::Costmap2D &, const Footprint &) const;
  Result translate(const Pose &, const Velocity &, Point endpoint, double heading,
    double dt, double limit, double hard_limit, bool stop_at_endpoint);
};
}
