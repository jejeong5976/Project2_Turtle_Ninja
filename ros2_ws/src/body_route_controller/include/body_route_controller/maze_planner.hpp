#pragma once
#include "body_route_controller/collision.hpp"
#include <functional>
#include "body_route_controller/observation.hpp"
namespace body_route_controller
{
struct MazePlannerSettings {
  UnknownPolicy unknown_policy=UnknownPolicy::Blocked;
  double unknown_penalty=2.0;
  double grid_step = .05, turn_cost = .5, cost_weight = .5, max_seconds = 3.0;
};
// Orthogonal lattice for the arena's map-aligned corridors. Each edge sweeps the
// polygon; a heading change requires space for its entire circumscribed circle.
// Unknown traversal is provisional and requires a separate raw identity map.
std::vector<Pose> planMaze(nav2_costmap_2d::Costmap2D &, const Footprint &,
  Pose start, Pose goal, MazePlannerSettings = {}, std::function<bool()> cancelled = [] {return false;},
  const nav2_costmap_2d::Costmap2D * identity = nullptr,
  const nav2_costmap_2d::Costmap2D * observed = nullptr);
}
