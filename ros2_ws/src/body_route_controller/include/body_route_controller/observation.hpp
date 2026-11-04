#pragma once
#include "body_route_controller/collision.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <array>
#include <functional>
#include <string>
#include <vector>
namespace body_route_controller {
enum class UnknownPolicy {Blocked, ObserveBeforeEnter};
UnknownPolicy unknownPolicy(const std::string &);
struct ObservationSettings {
  double ttl=4.0, scan_timeout=.3, latency=.5, linear_decel=.05, angular_decel=.2;
  double self_radius=.08, endpoint_margin=.02;
  void validate() const;
};
// Coordinates and times are already transformed to the raw map frame by the ROS adapter.
struct ObservedRay {Point origin; double angle=0, range=0, stamp=0; bool hit=true; Pose body; double min_range=.05;};
class ObservationSpace {
public:
  explicit ObservationSpace(ObservationSettings settings={}): settings_(settings) {settings_.validate();}
  void setMap(const nav_msgs::msg::OccupancyGrid &);
  bool ready() const {return !identity_.empty();}
  // Invalid scans never refresh evidence or sensor health. Strictly monotonic stamps.
  bool observe(std::size_t sensor, const std::vector<ObservedRay> &, double stamp, double received);
  void invalidate(std::size_t sensor, const std::string & reason);
  std::string health(double now) const;
  unsigned char identity(double x,double y) const;
  bool observedOccupied(double x,double y) const;
  int observedCell(double x,double y) const; // -1 outside; raw identity plus persistent hits
  bool observedFree(double x,double y,double now) const;
  nav2_costmap_2d::Costmap2D rawMap() const;
  nav2_costmap_2d::Costmap2D executionMap(double now,double valid_until) const;
  // Retain ONLY previously cleared cells under the current footprint across continuous
  // pose updates. Never clears occupied cells, external unknown cells or an unknown start.
  bool certify(const Footprint &, Pose, double now);
  std::string check(const Footprint &, Pose, Velocity measured, Velocity requested, double now, const std::vector<Velocity> & pending = {}) const;
  nav_msgs::msg::OccupancyGrid display(double now) const;
  void resetContinuity();
  const ObservationSettings & settings() const {return settings_;}
private:
  int index(double x,double y) const;
  nav2_costmap_2d::Costmap2D window(Pose,double,double,double) const;
  unsigned char executionCell(std::size_t,double,double) const;
  void ray(const ObservedRay &, double range, bool mark_hit);
  bool selfOccluded(const ObservedRay &) const;
  ObservationSettings settings_; nav_msgs::msg::OccupancyGrid map_;
  std::vector<unsigned char> identity_,hit_,carried_;
  std::vector<double> free_,hit_stamp_;
  std::vector<std::size_t> carried_indices_;
  std::array<double,2> stamp_{{-1,-1}}, received_{{-1,-1}};
  std::array<std::string,2> failure_{{"missing_scan","missing_scan"}};
  double previous_time_=-1; Pose previous_pose_;
};
// Conservative polygon-cell coverage shared by collision checking and evidence accounting.
bool visitFootprint(nav2_costmap_2d::Costmap2D &,const Footprint &,Pose,
  const std::function<bool(unsigned,unsigned)> &, bool only_blocking=false);
}
