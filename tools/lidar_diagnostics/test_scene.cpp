#define LIDAR_SCENE_TEST
#include "analyze_scene.cpp"
int main() {
  require(std::abs(distance({.5,.2},{{0,0},{1,0}})-.2)<1e-12,"Point-line distance");
  require(std::abs(distance({2,0},{{0,0},{1,0}})-1)<1e-12,"Segment endpoint clamp");
  require(distance({0,1},{{0,0},{0,0}})==1,"Degenerate segment");
  const int64_t t=1000000000;
  const Trajectory trajectory({{t,{0,0,0}},{t+100000000,{.002,0,0}}});
  const Pose mount{.110,.005,std::acos(-1.)};
  const Point wall{1,.5};
  const Point observed=inverse(mount,inverse(trajectory.at(t),wall));
  const Point corrected=apply(trajectory.at(t),apply(mount,observed));
  const Point raw=apply(trajectory.at(t+100000000),apply(mount,observed));
  const Segment fixed{{1,0},{1,1}};
  require(distance(corrected,fixed)<1e-12 && std::abs(distance(raw,fixed)-.002)<1e-12,"Static wall correction/world frame");
  std::cout<<"PASS segment distances and synthetic fixed-wall motion residual\n";
}
