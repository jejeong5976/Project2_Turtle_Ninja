// perception_node.cpp — 인지 노드 perception: 광각 · 손목 · 신호등 · 바코드를 한 노드에서 돌림
//
// 역할
//   - 파이프라인 조립만 함. 실제 처리는 각 파이프라인이 자기 콜백 그룹에서 함
//       WidePipeline   광각 상자 검출 → 퍽 · 신호등
//       WristPipeline  손목 퍽 분할 + 깊이 기하 → 퍽 자세
//       BarcodeStage   barcode.camera 카메라 콜백 안에서 판독 → 바코드 원문
//   - 파이프라인을 켜고 끄는 서비스 제공
// 입력
//   - 광각 영상 /wide_cam/image_raw
//   - 손목 /camera/camera/color/image_raw · aligned_depth_to_color/image_raw · color/camera_info, TF
//   - 서비스, ~ = /perception
//       ~/wide/enable · ~/wrist/enable · ~/barcode/enable  std_srvs/SetBool
//       ~/barcode/reset  std_srvs/Trigger, 바코드 다수결 비우기
// 출력
//   - /perception/wide/detections    interfaces/DetectionArray
//   - /perception/wrist/detections   interfaces/DetectionArray
//   - /perception/traffic_light      interfaces/TrafficLight
//   - /perception/task_text          interfaces/TaskText
//   - /perception/wrist_debug_image  sensor_msgs/Image, wrist.publish_debug 일 때만
// 실행
//   - ros2 launch perception perception.launch.py wide_model:=<광각>.engine wrist_model:=<손목>.engine
//   - 설정은 레포 config/perception.yaml
// note
//   - TensorRT 가 있는 Jetson 에서만 빌드됨
//   - 콜백 그룹 3 개 광각 · 손목 · 관리 + MultiThreadedExecutor 3 스레드 → 서로 기다리지 않음
//   - TensorRT 엔진은 파이프라인마다 따로 둠
//   - 서비스는 플래그만 바꾸고, 실제 초기화는 각 파이프라인이 다음 콜백에서 함
//   - wide 를 꺼도 바코드 판독은 ~/barcode/enable 로 따로 켬
//   - 한쪽 엔진을 못 읽거나 카메라가 없어도 나머지는 계속 돎
//   - 토픽 이름 · 기본값 설명은 wide_pipeline.hpp · wrist_pipeline.hpp · barcode.hpp

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "interfaces/msg/detection_array.hpp"

#include "barcode.hpp"
#include "wide_pipeline.hpp"
#include "wrist_pipeline.hpp"

namespace perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PerceptionNode() override;

private:
  rclcpp::CallbackGroup::SharedPtr wide_group_, wrist_group_, admin_group_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr wide_det_pub_, wrist_det_pub_;
  std::unique_ptr<BarcodeStage> barcode_;
  std::unique_ptr<WidePipeline> wide_;
  std::unique_ptr<WristPipeline> wrist_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wide_srv_, wrist_srv_, barcode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr barcode_reset_srv_;
};


using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception", options)
{
  const auto mx = rclcpp::CallbackGroupType::MutuallyExclusive;
  wide_group_ = create_callback_group(mx);
  wrist_group_ = create_callback_group(mx);
  admin_group_ = create_callback_group(mx);

  // 카메라마다 토픽 따로 (받는 쪽 큐 · QoS 를 따로, 한쪽 빈 배열이 다른 쪽 결과로 오해되지 않게)
  wide_det_pub_ = create_publisher<interfaces::msg::DetectionArray>("/perception/wide/detections", 10);
  wrist_det_pub_ = create_publisher<interfaces::msg::DetectionArray>("/perception/wrist/detections", 10);

  // 바코드 먼저 (어느 카메라 콜백에서 읽을지 정함)
  barcode_ = std::make_unique<BarcodeStage>(*this);
  BarcodeStage * to_wide = barcode_->camera() == "wide" ? barcode_.get() : nullptr;
  BarcodeStage * to_wrist = barcode_->camera() == "wrist" ? barcode_.get() : nullptr;
  wide_ = std::make_unique<WidePipeline>(*this, wide_group_, wide_det_pub_, to_wide);
  wrist_ = std::make_unique<WristPipeline>(*this, wrist_group_, wrist_det_pub_, to_wrist);

  auto enable_srv = [this](const std::string & name, auto setter) {
      return create_service<SetBool>(
        "~/" + name + "/enable",
        [this, name, setter](const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
          setter(req->data);
          res->success = true;
          res->message = name + (req->data ? " on" : " off");
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        }, rclcpp::ServicesQoS(), admin_group_);
    };
  wide_srv_ = enable_srv("wide", [this](bool on) {wide_->set_enabled(on);});
  wrist_srv_ = enable_srv("wrist", [this](bool on) {wrist_->set_enabled(on);});
  barcode_srv_ = enable_srv("barcode", [this](bool on) {barcode_->set_enabled(on);});
  barcode_reset_srv_ = create_service<Trigger>(
    "~/barcode/reset", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      barcode_->request_reset();
      res->success = true;
      res->message = "cleared";
    }, rclcpp::ServicesQoS(), admin_group_);
}

// 파이프라인(구독 · 엔진)을 먼저 지우고 나서 노드가 사라지게 순서를 고정
PerceptionNode::~PerceptionNode()
{
  wrist_.reset();
  wide_.reset();
  barcode_.reset();
}

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<perception::PerceptionNode>();
    // 콜백 그룹 3개 (광각 · 손목 · 관리) → 스레드 3개. Jetson 실측 후 조정
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 3);
    ex.add_node(node);
    ex.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
