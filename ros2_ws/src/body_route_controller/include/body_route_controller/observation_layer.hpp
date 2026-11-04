#pragma once
#include "body_route_controller/observation.hpp"
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <mutex>
#include <deque>
namespace body_route_controller {
class ObservationLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override;
  void updateBounds(double,double,double,double *,double *,double *,double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &,int,int,int,int) override;
  void reset() override;
  bool isClearable() override {return false;}
  nav2_costmap_2d::Costmap2D identityMap();
  nav2_costmap_2d::Costmap2D observedMap();
  std::string gate(const Footprint &,Pose,Velocity,Velocity,double);
  std::string goalClear(const Footprint &,Pose,double);
private:
  void scan(std::size_t,const sensor_msgs::msg::LaserScan &);
  std::mutex mutex_; ObservationSpace space_; bool observe_=false;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::array<nav2::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr,2> scans_;
  nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr display_;
  nav2::Publisher<nav_msgs::msg::GridCells>::SharedPtr free_display_;
  double last_display_=-1;
  std::deque<std::pair<double,Velocity>> admitted_;
  std::vector<Velocity> pending(double now);
};
std::shared_ptr<ObservationLayer> observationLayer(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &);
}
