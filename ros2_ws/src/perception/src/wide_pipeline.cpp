// wide_pipeline.cpp — 광각 카메라 처리 구현

#include "wide_pipeline.hpp"

#include <algorithm>
#include <tuple>
#include <utility>

#include "image_cv.hpp"
#include <opencv2/imgproc.hpp>

#include "barcode.hpp"
#include "msg_utils.hpp"

namespace perception
{

using interfaces::msg::TrafficLight;
using sensor_msgs::msg::Image;

namespace
{
// max_hz 주기 판정 여유. 영상 도착 간격이 흔들려 주기보다 조금 짧으면 한 장 더 건너뜀
// (30 Hz 카메라 + max_hz 15 → 두 번째 장 66.6 ms < 66.7 ms 로 버려져 실제 10 Hz 였음, 실측)
constexpr double kPeriodSlackS = 0.010;

double since(std::chrono::steady_clock::time_point now, std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(now - t).count();
}

// 이미지 구독 QoS. 기본 reliable + depth 1 (최신 한 장만).
// 1280x720 bgr8 (2.7MB) 은 프로세스 사이에서 best_effort 로 받으면 조각 손실로 거의 다 버려짐 (Fast DDS, 실측)
rclcpp::QoS image_qos(const std::string & mode)
{
  rclcpp::QoS q(1);
  if (mode == "best_effort") {
    q.best_effort();
  } else {
    q.reliable();
  }
  return q;
}

const char * light_state_name(uint8_t s)
{
  switch (s) {
    case TrafficLight::RED: return "RED";
    case TrafficLight::YELLOW: return "YELLOW";
    case TrafficLight::GREEN: return "GREEN";
    case TrafficLight::OFF: return "OFF";
    default: return "UNKNOWN";
  }
}
}  // namespace

WidePipeline::WidePipeline(
  rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr det_pub, BarcodeStage * barcode)
: node_(node), barcode_(barcode),
  tag_reader_(static_cast<int>(node.declare_parameter<int64_t>("wide.tag_min_side_px", 60))),
  light_vote_(static_cast<int>(node.declare_parameter<int64_t>("traffic_light.window", 5)),
    static_cast<int>(node.declare_parameter<int64_t>("traffic_light.need", 3))),
  det_pub_(std::move(det_pub))
{
  auto & n = node;
  const auto image_topic = n.declare_parameter<std::string>("wide.image_topic", "/wide_cam/image_raw");
  const auto qos = image_qos(n.declare_parameter<std::string>("wide.image_qos", "reliable"));
  frame_id_ = n.declare_parameter<std::string>("wide.frame_id", "birdeye_camera_optical_frame");  // "" = 이미지 header
  const auto model = n.declare_parameter<std::string>("wide.model", "");
  conf_ = n.declare_parameter<double>("wide.conf", 0.5);
  iou_ = n.declare_parameter<double>("wide.iou", 0.7);
  max_hz_ = n.declare_parameter<double>("wide.max_hz", 30.0);
  enabled_ = n.declare_parameter<bool>("wide.enabled", true);
  read_tags_ = n.declare_parameter<bool>("wide.read_tags", false);   // 광각에서 16 mm 퍽 태그는 거의 안 읽힘 → 기본 끔
  const auto pucks = n.declare_parameter<std::vector<std::string>>(
    "wide.puck_classes", {"red_cyl", "green_cyl", "red_box", "green_box"});
  puck_classes_ = std::set<std::string>(pucks.begin(), pucks.end());

  light_enabled_ = n.declare_parameter<bool>("traffic_light.enabled", true);
  light_class_ = n.declare_parameter<std::string>("traffic_light.class", "traffic_light");
  for (const auto & [tier, def] : std::vector<std::pair<std::string, std::vector<double>>>{
      {"red", {0.36, 0.56}}, {"yellow", {0.60, 0.75}}, {"green", {0.80, 0.98}}})
  {
    const auto v = n.declare_parameter<std::vector<double>>("traffic_light." + tier, def);
    tiers_.push_back({tier, v.at(0), v.at(1)});
  }
  light_rule_.v_margin = n.declare_parameter<double>("traffic_light.v_margin", 40.0);
  light_rule_.white_min = n.declare_parameter<double>("traffic_light.white_min", 0.30);
  light_rule_.white_margin = n.declare_parameter<double>("traffic_light.white_margin", 0.20);
  light_rule_.white_v = n.declare_parameter<double>("traffic_light.white_v", 240.0);
  light_rule_.white_s = n.declare_parameter<double>("traffic_light.white_s", 60.0);
  light_timeout_ = n.declare_parameter<double>("traffic_light.timeout", 1.0);
  publish_debug_ = n.declare_parameter<bool>("wide.publish_debug", false);

  // 엔진: 실패해도 노드는 계속 (손목 · 바코드는 돎)
  if (model.empty()) {
    RCLCPP_ERROR(node.get_logger(), "[wide] wide.model 이 비어 있음 — 광각 탐지 · 신호등 안 함");
  } else {
    try {
      detector_ = std::make_unique<wide::BoxDetector>(model);
      std::string names;
      for (const auto & s : detector_->names()) {names += s + " ";}
      const auto s = detector_->input_size();
      RCLCPP_INFO(node.get_logger(), "[wide] 엔진 %s (%dx%d) 클래스 %s", model.c_str(), s.width, s.height, names.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node.get_logger(), "[wide] 엔진을 못 읽음 %s: %s — 광각 탐지 · 신호등 안 함", model.c_str(), e.what());
    }
  }

  if (light_enabled_) {light_pub_ = node.create_publisher<TrafficLight>("/perception/traffic_light", 10);}
  if (publish_debug_) {debug_pub_ = node.create_publisher<Image>("/perception/wide_debug_image", rclcpp::SensorDataQoS());}
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group;
  image_sub_ = node.create_subscription<Image>(image_topic, qos, [this](Image::ConstSharedPtr m) {on_image(m);}, opt);
  RCLCPP_INFO(node.get_logger(), "[wide] %s → /perception/wide/detections%s (frame_id=%s)", image_topic.c_str(),
    light_enabled_ ? " + /perception/traffic_light" : "", frame_id_.c_str());
}

WidePipeline::~WidePipeline() = default;

void WidePipeline::on_image(const Image::ConstSharedPtr & msg)
{
  const auto now = Clock::now();
  if (reset_req_ != reset_done_) {
    reset_done_ = reset_req_;
    light_vote_.clear();
  }
  const bool detect = enabled_ && detector_ && since(now, last_) >= 1.0 / max_hz_ - kPeriodSlackS;
  const bool read_barcode = barcode_ && barcode_->due();   // 바코드는 planning 이 켰을 때만
  if (!detect && !read_barcode) {return;}

  cv::Mat img;
  try {
    img = image_cv::to_bgr8(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wide] 영상 변환: %s", e.what());
    return;
  }
  auto header = msg->header;   // stamp = 촬영 시각 그대로, frame_id 만 URDF 이름으로
  if (!frame_id_.empty()) {header.frame_id = frame_id_;}

  std::vector<wide::Box> barcode_boxes;
  if (detect) {
    last_ = now;
    detect_and_publish(img, header, now, barcode_boxes);
  }
  if (read_barcode) {barcode_->process(img, header, barcode_boxes);}
}

void WidePipeline::detect_and_publish(
  const cv::Mat & img, const std_msgs::msg::Header & header, Clock::time_point now, std::vector<wide::Box> & barcode_boxes)
{
  std::vector<wide::BoxDetection> dets;
  try {
    dets = detector_->detect(img, static_cast<float>(conf_), static_cast<float>(iou_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[wide] 추론 실패: %s", e.what());
    return;
  }
  interfaces::msg::DetectionArray out;   // 퍽만. 못 찾아도 빈 배열 (= 카메라 살아 있음)
  out.header = header;
  const wide::BoxDetection * best_light = nullptr;
  for (const auto & d : dets) {
    const auto name = detector_->name(d.cls);
    if (barcode_ && name == barcode_->roi_class()) {barcode_boxes.push_back(d.box);}
    if (name == light_class_ && (!best_light || d.score > best_light->score)) {best_light = &d;}
    if (!puck_classes_.count(name)) {continue;}

    interfaces::msg::Detection m;
    m.header = header;
    m.kind = "puck";
    std::tie(m.label, m.shape) = puck_label_shape(name);
    m.confidence = d.score;
    m.bbox = {d.box.x1, d.box.y1, d.box.x2, d.box.y2};
    m.yaw_known = false;
    set_unknown_pose(m.pose);   // 광각은 깊이 없음 → 위치 · 방향 모름
    if (read_tags_) {m.id = puck_id_string(tag_reader_.read(img, d.box).first);}
    out.detections.push_back(std::move(m));
  }
  det_pub_->publish(out);
  const bool debug = publish_debug_ && debug_pub_->get_subscription_count() > 0;
  LightDebug light;
  uint8_t state = TrafficLight::UNKNOWN;
  if (light_pub_) {state = publish_traffic_light(img, header, best_light, now, debug ? &light : nullptr);}
  if (debug) {publish_debug(img, header, dets, light, state);}
}

uint8_t WidePipeline::publish_traffic_light(
  const cv::Mat & img, const std_msgs::msg::Header & header, const wide::BoxDetection * best, Clock::time_point now,
  LightDebug * dbg)
{
  TrafficLight msg;
  msg.header = header;
  msg.visible = false;
  if (best) {
    const cv::Rect r(cv::Point(static_cast<int>(best->box.x1), static_cast<int>(best->box.y1)),
      cv::Point(static_cast<int>(best->box.x2), static_cast<int>(best->box.y2)));
    const cv::Rect roi = r & cv::Rect(0, 0, img.cols, img.rows);
    if (roi.area() > 0) {
      std::map<std::string, TrafficLightScore> score;
      const auto label = classify_traffic_light(img(roi), tiers_, light_rule_, dbg ? &score : nullptr);
      light_vote_.push(label);
      if (dbg) {*dbg = LightDebug{roi, label, std::move(score)};}
      last_light_seen_ = now;
      msg.visible = true;
    }
  }
  if (!msg.visible && since(now, last_light_seen_) > light_timeout_) {light_vote_.clear();}
  // 이번 영상에서 못 찾았으면 이전 색을 내지 않고 UNKNOWN (planning 이 못 본 구간으로 처리)
  const auto value = light_vote_.result().first;
  msg.state = (msg.visible && value) ? traffic_light_code(*value) : TrafficLight::UNKNOWN;
  light_pub_->publish(msg);
  return msg.state;
}

// 확인용 그림: 검출 상자 전부 (클래스 · 점수) + 신호등 칸 경계 · 칸별 V · 흰색 비율 W · 이번 장 판정 · 확정 상태
void WidePipeline::publish_debug(
  const cv::Mat & img, const std_msgs::msg::Header & header, const std::vector<wide::BoxDetection> & dets,
  const LightDebug & light, uint8_t light_state)
{
  cv::Mat vis = img.clone();
  const auto font = cv::FONT_HERSHEY_SIMPLEX;
  for (const auto & d : dets) {
    const auto name = detector_->name(d.cls);
    const cv::Scalar c = name == light_class_ ? cv::Scalar(255, 255, 0) :
      name.rfind("red", 0) == 0 ? cv::Scalar(0, 0, 255) :
      name.rfind("green", 0) == 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 255);
    const cv::Point p1(static_cast<int>(d.box.x1), static_cast<int>(d.box.y1));
    const cv::Point p2(static_cast<int>(d.box.x2), static_cast<int>(d.box.y2));
    cv::rectangle(vis, p1, p2, c, 2);
    cv::putText(vis, name + " " + cv::format("%.2f", d.score), p1 + cv::Point(0, -5), font, 0.5, c, 1);
  }
  if (light.roi.area() > 0) {
    const auto & r = light.roi;
    const int x0 = r.x + r.width / 4, x1 = r.x + r.width * 3 / 4;   // 판정에 쓰는 가운데 절반 열
    const int tx = r.x + r.width + 6;
    for (const auto & t : tiers_) {
      const int y0 = r.y + static_cast<int>(t.top * r.height);
      const int y1 = r.y + static_cast<int>(t.bottom * r.height);
      const cv::Scalar c = t.name == "red" ? cv::Scalar(0, 0, 255) :
        t.name == "yellow" ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
      cv::rectangle(vis, cv::Point(x0, y0), cv::Point(x1, y1), c, 1);
      const auto it = light.score.find(t.name);
      const TrafficLightScore sc = it == light.score.end() ? TrafficLightScore{} : it->second;
      cv::putText(vis, cv::format("%s V%.0f W%.0f%%", t.name.c_str(), sc.v, sc.white * 100.0),
        cv::Point(tx, y0 + 14), font, 0.45, c, 1);
    }
  }
  const std::string txt = "light: frame " + (light.label.empty() ? std::string("-") : light.label) +
    " / state " + light_state_name(light_state) +
    cv::format(" (W>=%.0f%% +%.0f%% | V +%.0f)", light_rule_.white_min * 100.0, light_rule_.white_margin * 100.0,
      light_rule_.v_margin);
  cv::putText(vis, txt, cv::Point(10, 25), font, 0.7, {0, 0, 0}, 4);
  cv::putText(vis, txt, cv::Point(10, 25), font, 0.7, {255, 255, 255}, 1);
  debug_pub_->publish(*image_cv::bgr8_msg(header, vis));
}

}  // namespace perception
