#include <slam_toolbox/slam_toolbox_async.hpp>
#include <std_msgs/msg/int64.hpp>

// Instrument the installed mapper without copying or modifying its algorithms.
// Offline lockstep replay waits for this acknowledgement, including scans that
// the mapper intentionally skips due to motion/time admission thresholds.
class AcknowledgedMapper final : public slam_toolbox::AsynchronousSlamToolbox {
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr completed_;
public:
  explicit AcknowledgedMapper(const rclcpp::NodeOptions& options)
      : AsynchronousSlamToolbox(options) {
    completed_=create_publisher<std_msgs::msg::Int64>("/maze/scan_callback_completed",100);
  }
protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override {
    AsynchronousSlamToolbox::laserCallback(scan);
    std_msgs::msg::Int64 ack;
    ack.data=rclcpp::Time(scan->header.stamp).nanoseconds();
    completed_->publish(ack);
  }
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<AcknowledgedMapper>(rclcpp::NodeOptions{});
  if(node->configure().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
     node->activate().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return 2;
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
