// User-run, bounded, receive-only ROS odometry recorder. Never commands hardware.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
class OdomCapture : public rclcpp::Node {
  FILE *file_=nullptr;
  int64_t last_=-1;
  size_t count_=0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
public:
  OdomCapture():Node("capture_motion_odom") {
    if(get_parameter("use_sim_time").as_bool())throw std::runtime_error("Physical capture requires system time");
    const auto path=declare_parameter<std::string>("output_csv","");
    const auto seconds=declare_parameter<int>("duration_s",60);
    if(path.empty() || path.front()!='/' || seconds<1 || seconds>120)
      throw std::runtime_error("Require absolute new output_csv and duration_s in 1..120");
    const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0)throw std::runtime_error("Cannot exclusively create odom output");
    file_=fdopen(fd,"w");if(!file_){close(fd);throw std::runtime_error("fdopen failed");}
    if(std::fprintf(file_,"stamp_ns,x_m,y_m,yaw_rad\n")<0)throw std::runtime_error("Write failed");
    subscription_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(100),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto &q=m->pose.pose.orientation;const auto &p=m->pose.pose.position;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if(get_parameter("use_sim_time").as_bool() || m->header.frame_id!="odom" ||
           m->child_frame_id!="base_footprint" || !std::isfinite(norm) || std::abs(norm-1)>1e-3 ||
           std::abs(q.x)>1e-3 || std::abs(q.y)>1e-3 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
           !std::isfinite(p.z) || std::abs(p.z)>1e-3 || stamp<=last_ || ++count_>10000)
          throw std::runtime_error("Invalid/regressing/nonplanar odometry or capture limit; discard partial CSV");
        last_=stamp;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        if(std::fprintf(file_,"%lld,%.17g,%.17g,%.17g\n",static_cast<long long>(stamp),p.x,p.y,yaw)<0 || std::fflush(file_)!=0)
          throw std::runtime_error("Odometry write failure; discard partial CSV");
      });
    timer_=create_wall_timer(std::chrono::seconds(seconds),[this]{
      if(count_<2)throw std::runtime_error("No usable odometry captured");
      RCLCPP_INFO(get_logger(),"ODOM CAPTURE COMPLETE: %zu samples",count_);rclcpp::shutdown();
    });
    RCLCPP_INFO(get_logger(),"Recording /odom only for %lld seconds: %s",static_cast<long long>(seconds),path.c_str());
  }
  ~OdomCapture() override {if(file_)std::fclose(file_);}
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<OdomCapture>());}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
