#include "motion_model.hpp"
#include "sample_time_model.hpp"
#include <iostream>
#include <functional>
using namespace lidar_motion;
void check(bool ok) {if(!ok) throw std::runtime_error("motion test failed");}
bool near(double a,double b) {return std::abs(a-b)<1e-9;}
void rejects(const std::function<void()> &f) {
  bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected);
}
int main() {
  const int64_t epoch=1793000000000000000LL;
  const double pi=std::acos(-1.);
  const Pose mount{.110,.005,pi};
  Trajectory fixed({{epoch,{0,0,0}},{epoch+100000000,{0,0,0}}});
  auto a=correct(fixed,epoch+100,epoch+90000000,mount,2,.3);
  check(near(a.origin.x,0)&&near(a.origin.y,0));
  check(near(a.endpoint.x,2*std::cos(.3))&&near(a.endpoint.y,2*std::sin(.3)));
  Trajectory forward({{epoch,{0,0,0}},{epoch+100000000,{.01,0,0}}});
  a=correct(forward,epoch,epoch+100000000,mount,2,0);
  check(near(a.origin.x,.01)&&near(a.endpoint.x,2.01));
  check(near(forward.at(epoch+50000000).x,.005));
  // A point on a static world wall remains fixed after correction.
  for(int64_t offset:{0LL,25000000LL,99000000LL,100000000LL}) {
    const auto pose=forward.at(epoch+offset);
    const auto local=inverse(mount,inverse(pose,{-2,0}));
    a=correct(forward,epoch+offset,epoch+100000000,mount,std::hypot(local.x,local.y),std::atan2(local.y,local.x));
    const auto expected=inverse(mount,inverse(forward.at(epoch+100000000),{-2,0}));
    check(near(a.endpoint.x,expected.x)&&near(a.endpoint.y,expected.y));
  }
  Trajectory rotation({{epoch,{0,0,0}},{epoch+100000000,{0,0,.05}}});
  a=correct(rotation,epoch,epoch+100000000,mount,1,0);
  check(std::hypot(a.origin.x,a.origin.y)>.005); // Lever arm matters.
  Trajectory seam({{epoch,{0,0,pi-.01}},{epoch+100000000,{0,0,-pi+.01}}});
  check(near(std::abs(seam.at(epoch+50000000).yaw),pi));
  auto times=lidar_timing::model_times(400,100000000,{399,1,0,398,-1});
  check(times[0].nominal_ns>times[1].nominal_ns && times[2].nominal_ns<times[3].nominal_ns);
  check(std::isnan(times[4].nominal_ns)); // Never unwrap/reorder angular bins.
  check(estimate_time({epoch,epoch+10},5)==epoch);
  rejects([&]{estimate_time({epoch+1,epoch},0);});
  rejects([&]{estimate_time({epoch,epoch},-1);});
  rejects([&]{fixed.at(epoch-1);});rejects([&]{fixed.at(epoch+100000001);});
  rejects([&]{Trajectory t({{epoch,{0,0,0}},{epoch,{0,0,0}}});});
  rejects([&]{Trajectory t({{epoch,{0,0,0}},{epoch-1,{0,0,0}}});});
  rejects([&]{Trajectory t({{epoch,{NAN,0,0}},{epoch+1,{0,0,0}}});});
  rejects([&]{Trajectory t({{epoch,{0,0,0}},{epoch+200000000,{0,0,0}}});t.at(epoch+1);});
  rejects([&]{Trajectory t({{epoch,{0,0,0}},{epoch+100000000,{1,0,0}}});t.at(epoch+1);});
  rejects([&]{correct(fixed,epoch,epoch,mount,NAN,0);});
  std::cout<<"PASS static, translation, world-wall invariance, yaw wrap, lever arm, source seam, epoch precision, invalid timing/odom\n";
}
