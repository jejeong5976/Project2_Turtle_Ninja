#include "lidar_cloud_merge/scan_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

class FrontScanFilter : public rclcpp::Node {
  using Scan=sensor_msgs::msg::LaserScan;
  rclcpp::Publisher<Scan>::SharedPtr publisher_;
  rclcpp::Subscription<Scan>::SharedPtr subscription_;
public:
  FrontScanFilter():Node("stationary_front_scan_filter") {
    if (!declare_parameter("stationary_test",false)) {
      throw std::runtime_error("Requires stationary_test:=true; no motion admission or timing correction");
    }
    publisher_=create_publisher<Scan>("/lidar_front/scan_body_filtered",rclcpp::SensorDataQoS());
    subscription_=create_subscription<Scan>("/lidar_front/scan",rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr input) {
        try {publisher_->publish(lidar_cloud_merge::filter_front_scan(*input));}
        catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Rejected scan: %s",e.what());
        }
      });
    RCLCPP_WARN(get_logger(),"Stationary filter only; no freshness/motion gate. Raw scan and physical origin preserved.");
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<FrontScanFilter>());}
  catch (const std::exception &e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
