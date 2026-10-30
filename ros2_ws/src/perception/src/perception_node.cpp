// 인지 노드 — 노드 하나(perception)가 광각 · 손목 카메라와 바코드 판독을 모두 맡음.
//
//   광각 image                →  /perception/detections   (interfaces/DetectionArray, camera="wide")
//                             →  /perception/signals      (interfaces/SignalState, publish_signals:=true 인 카메라만)
//   손목 image + 정렬된 depth  →  /perception/detections   (camera="wrist")
//   barcode.camera 의 image   →  /perception/task_text    (interfaces/TaskText)
//
// - 모델: ultralytics 로 내보낸 .onnx 를 ONNX Runtime 으로 실행 (provider: cuda | tensorrt | cpu).
//   두 카메라가 같은 모델 파일이면 세션 하나를 같이 씀 (Session::Run 은 스레드 안전)
// - 퍽: 색(HSV) · AprilTag ID · 카메라 기준 위치(depth 우선, 없으면 태그 PnP)
// - 신호등: 칸별 밝기 + 다수결
// - 바코드: 같은 프레임의 barcode bbox 를 키워서 먼저 읽고, 못 읽으면 전체 화면. 여러 프레임 일치 후 한 번만 보냄
// - 카메라마다 콜백 그룹을 따로 둬서 광각 · 손목이 서로 기다리지 않음 (MultiThreadedExecutor)
//
// 서비스
//   ~/wide/enable, ~/wrist/enable  (std_srvs/SetBool) : 상태 머신이 카메라별 탐지를 켜고 끌 때
//   ~/barcode/enable               (std_srvs/SetBool)
//   ~/barcode/reset                (std_srvs/Trigger) : 확정 기록 비우기 (같은 바코드를 다시 읽어야 할 때)
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/puck_detection.hpp"
#include "interfaces/msg/signal_state.hpp"
#include "interfaces/msg/task_text.hpp"

#include "postproc.hpp"
#include "yolo.hpp"

namespace perception
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;
using SteadyClock = std::chrono::steady_clock;
using Code = std::pair<std::string, std::string>;   // (원문, 종류)

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

double since(SteadyClock::time_point now, SteadyClock::time_point t)
{
  return std::chrono::duration<double>(now - t).count();
}

// 벽 1D 바코드 판독 (barcode.camera 로 지정한 카메라의 이미지에서 돎)
class BarcodeStage
{
public:
  BarcodeStage(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group)
  : node_(node)
  {
    const auto p = [&node](const std::string & k, auto def) {return node.declare_parameter("barcode." + k, def);};
    camera_ = node.get_parameter("barcode.camera").as_string();   // PerceptionNode 가 먼저 선언
    max_hz_ = p("max_hz", 10.0);
    enabled_ = p("enabled", true);
    class_ = p("class", std::string("barcode"));
    roi_pad_ = p("roi_pad", 0.3);
    roi_scale_ = p("roi_scale", 2.0);
    full_frame_ = p("full_frame", true);
    voter_ = std::make_unique<Voter<Code>>(
      static_cast<int>(p("window", int64_t{5})), static_cast<int>(p("need", int64_t{3})));

    pub_ = node.create_publisher<interfaces::msg::TaskText>("/perception/task_text", 10);
    enable_srv_ = node.create_service<SetBool>(
      "~/barcode/enable", [this](const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
        enabled_ = req->data;
        voter_->clear();
        res->success = true;
        res->message = std::string("barcode ") + (enabled_ ? "on" : "off");
        RCLCPP_INFO(node_.get_logger(), "%s", res->message.c_str());
      }, rclcpp::ServicesQoS(), group);
    reset_srv_ = node.create_service<Trigger>(
      "~/barcode/reset", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        sent_.clear();
        voter_->clear();
        res->success = true;
        res->message = "cleared";
      }, rclcpp::ServicesQoS(), group);
  }

  const std::string & camera() const {return camera_;}

  // 이 카메라 프레임에서 바코드를 읽을 차례인지 (꺼져 있거나 max_hz 보다 빠르면 false)
  bool due(SteadyClock::time_point now) const {return enabled_ && since(now, last_) >= 1.0 / max_hz_;}

  // boxes: 같은 프레임의 YOLO 결과 (탐지가 꺼져 있으면 비어 있음 → 전체 화면만)
  void run(
    const cv::Mat & img, const std_msgs::msg::Header & header,
    const std::vector<std::pair<std::string, Box>> & boxes, SteadyClock::time_point now)
  {
    last_ = now;
    std::vector<Code> found;
    for (const auto & [name, b] : boxes) {
      if (name != class_) {continue;}
      found = reader_.read(crop(img, b));
      if (!found.empty()) {break;}
    }
    if (found.empty() && full_frame_) {found = reader_.read(img);}
    if (found.empty()) {return;}

    const auto [value, ratio] = voter_->push(found.front());
    (void)ratio;
    if (!value || sent_.count(value->first)) {return;}
    sent_.insert(value->first);
    interfaces::msg::TaskText out;
    out.header = header;
    out.camera = camera_;
    out.text = value->first;
    out.symbology = value->second;
    out.votes = static_cast<uint8_t>(voter_->count(*value));
    pub_->publish(out);
    RCLCPP_INFO(node_.get_logger(), "바코드 확정: %s (%s)", out.text.c_str(), out.symbology.c_str());
  }

private:
  // bbox 를 roi_pad 만큼 넓혀 자르고 roi_scale 배 키움 (광각에서는 바코드가 작아서 확대가 중요)
  cv::Mat crop(const cv::Mat & img, const Box & b) const
  {
    const double cx = (b.x1 + b.x2) / 2, cy = (b.y1 + b.y2) / 2;
    const double w = (b.x2 - b.x1) * (1 + 2 * roi_pad_), h = (b.y2 - b.y1) * (1 + 2 * roi_pad_);
    const int x1 = static_cast<int>(std::max(0.0, cx - w / 2)), y1 = static_cast<int>(std::max(0.0, cy - h / 2));
    const int x2 = static_cast<int>(std::min(static_cast<double>(img.cols), cx + w / 2));
    const int y2 = static_cast<int>(std::min(static_cast<double>(img.rows), cy + h / 2));
    if (x2 <= x1 || y2 <= y1) {return img;}
    cv::Mat out;
    cv::resize(img(cv::Range(y1, y2), cv::Range(x1, x2)), out, cv::Size(), roi_scale_, roi_scale_, cv::INTER_CUBIC);
    return out;
  }

  rclcpp::Node & node_;
  std::string camera_, class_;
  double max_hz_{10.0}, roi_pad_{0.3}, roi_scale_{2.0};
  bool enabled_{true}, full_frame_{true};
  std::unique_ptr<Voter<Code>> voter_;
  BarcodeReader reader_;
  std::set<std::string> sent_;
  SteadyClock::time_point last_{};

  rclcpp::Publisher<interfaces::msg::TaskText>::SharedPtr pub_;
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;
};

// 카메라 하나의 탐지 파이프라인. 파라미터는 "<camera>." 아래 (예: wide.image_topic)
class CameraStage
{
public:
  CameraStage(
    rclcpp::Node & node, const std::string & camera, std::shared_ptr<Yolo> model,
    rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr pub)
  : node_(node), camera_(camera), model_(std::move(model)), pub_(std::move(pub))
  {
    group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto image_topic = p("image_topic", std::string("/" + camera + "_cam/image_raw"));
    depth_topic_ = p("depth_topic", std::string(""));                            // "" 이면 depth 안 씀
    const auto info_topic = p("camera_info_topic", std::string("/" + camera + "_cam/camera_info"));
    conf_ = p("conf", 0.5);
    iou_ = p("iou", 0.7);
    max_hz_ = p("max_hz", 15.0);
    enabled_ = p("enabled", true);
    const auto pucks = p("puck_classes", std::vector<std::string>{"red_cyl", "green_cyl", "red_box", "green_box"});
    puck_classes_ = std::set<std::string>(pucks.begin(), pucks.end());
    signal_class_ = p("signal_class", std::string("traffic_light"));
    const bool publish_signals = p("publish_signals", false);
    read_tags_ = p("read_tags", true);
    tag_size_ = p("tag.size", 0.025);                                            // [확인 필요] 퍽 태그 한 변 (m)
    tag_reader_ = std::make_unique<TagReader>(
      p("tag.family", std::string("36h11")), static_cast<int>(p("tag.min_side_px", int64_t{60})));
    color_frac_ = p("color.roi_frac", 0.6);
    color_.v_min = static_cast<int>(p("color.v_min", int64_t{40}));
    color_.min_pixels = static_cast<int>(p("color.min_pixels", int64_t{30}));
    color_.s_beige_max = static_cast<double>(p("color.s_beige_max", int64_t{70}));
    color_.beige_hue = hue_range("color.beige_hue", {5, 35});
    color_.red_hue = hue_range("color.red_hue", {170, 10});
    color_.green_hue = hue_range("color.green_hue", {40, 85});
    for (const auto & [tier, def] : std::vector<std::pair<std::string, std::vector<double>>>{
        {"red", {0.10, 0.40}}, {"orange", {0.40, 0.70}}, {"green", {0.70, 1.00}}})
    {
      const auto v = p("signal." + tier, def);
      tiers_.push_back({tier, v.at(0), v.at(1)});
    }
    v_margin_ = p("signal.v_margin", 40.0);
    signal_vote_ = std::make_unique<Voter<std::string>>(
      static_cast<int>(p("signal.window", int64_t{5})), static_cast<int>(p("signal.need", int64_t{3})));
    signal_timeout_ = p("signal.timeout", 1.0);                                  // 이 시간 동안 안 보이면 unknown
    const auto qos = image_qos(p("image_qos", std::string("reliable")));

    rclcpp::SubscriptionOptions opt;
    opt.callback_group = group_;
    if (publish_signals) {
      sig_pub_ = node.create_publisher<interfaces::msg::SignalState>("/perception/signals", 10);
    }
    info_sub_ = node.create_subscription<CameraInfo>(
      info_topic, 1, [this](CameraInfo::ConstSharedPtr m) {on_info(*m);}, opt);
    enable_srv_ = node.create_service<SetBool>(
      "~/" + camera + "/enable", [this](const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
        enabled_ = req->data;
        signal_vote_->clear();
        res->success = true;
        res->message = camera_ + (enabled_ ? " on" : " off");
        RCLCPP_INFO(node_.get_logger(), "%s", res->message.c_str());
      }, rclcpp::ServicesQoS(), group_);
    image_sub_ = node.create_subscription<Image>(
      image_topic, qos, [this](Image::ConstSharedPtr m) {on_image(m);}, opt);
    if (!depth_topic_.empty()) {   // 색 이미지와 시각이 가장 가까운 depth 를 짝지음 (50 ms 이내)
      depth_sub_ = node.create_subscription<Image>(
        depth_topic_, qos, [this](Image::ConstSharedPtr m) {
          depth_buf_.push_back(m);
          while (depth_buf_.size() > 10) {depth_buf_.pop_front();}
        }, opt);
    }

    const std::string src = image_topic + (depth_topic_.empty() ? "" : " + " + depth_topic_);
    RCLCPP_INFO(
      node.get_logger(), "[%s] %s → /perception/detections (enabled=%s)", camera_.c_str(), src.c_str(),
      enabled_ ? "true" : "false");
  }

  // 바코드 판독 서비스가 이 카메라의 콜백과 같은 그룹에서 돌도록
  rclcpp::CallbackGroup::SharedPtr group() const {return group_;}

  // 이 카메라 이미지에서 바코드도 읽게 함
  void set_barcode(BarcodeStage * barcode) {barcode_ = barcode;}

private:
  template<typename T>
  T p(const std::string & key, const T & def) {return node_.declare_parameter<T>(camera_ + "." + key, def);}

  std::array<double, 2> hue_range(const std::string & key, const std::vector<int64_t> & def)
  {
    const auto v = p(key, def);
    return {static_cast<double>(v.at(0)), static_cast<double>(v.at(1))};
  }

  void on_info(const CameraInfo & msg)
  {
    K_ = cv::Matx33d(msg.k.data());
    fisheye_ = msg.distortion_model == "equidistant";
    D_ = cv::Mat(msg.d, true).reshape(1, 1);
    has_info_ = true;
  }

  Image::ConstSharedPtr match_depth(const Image & img) const
  {
    const rclcpp::Time t(img.header.stamp);
    Image::ConstSharedPtr best;
    double best_dt = 0.05;
    for (const auto & d : depth_buf_) {
      const double dt = std::abs((rclcpp::Time(d->header.stamp) - t).seconds());
      if (dt <= best_dt) {best_dt = dt; best = d;}
    }
    return best;
  }

  void on_image(const Image::ConstSharedPtr & msg)
  {
    const auto now = SteadyClock::now();
    const bool detect = enabled_ && model_ && since(now, last_) >= 1.0 / max_hz_;
    const bool barcode = barcode_ && barcode_->due(now);
    if (!detect && !barcode) {return;}

    const cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    std::vector<std::pair<std::string, Box>> boxes;   // 바코드 ROI 용
    if (detect) {
      if (const auto dets = detect_and_publish(img, msg, now)) {boxes = *dets;}
    }
    if (barcode) {barcode_->run(img, msg->header, boxes, now);}
  }

  // 탐지해서 /perception/detections (+ signals) 로 보냄. depth 짝이 없거나 추론 실패면 nullopt
  std::optional<std::vector<std::pair<std::string, Box>>> detect_and_publish(
    const cv::Mat & img, const Image::ConstSharedPtr & msg, SteadyClock::time_point now)
  {
    std::optional<cv::Mat> depth;
    if (!depth_topic_.empty()) {
      const auto dmsg = match_depth(*msg);
      if (!dmsg) {return std::nullopt;}   // 짝이 되는 depth 가 아직 없음
      depth = to_depth_m(*dmsg);
    }
    last_ = now;

    std::vector<Detection> dets;
    try {
      dets = model_->infer(img, static_cast<float>(conf_), static_cast<float>(iou_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[%s] 추론 실패: %s", camera_.c_str(), e.what());
      return std::nullopt;
    }

    interfaces::msg::DetectionArray out;
    out.header = msg->header;
    out.camera = camera_;
    std::vector<std::pair<std::string, Box>> boxes;
    const Detection * best_signal = nullptr;
    for (const auto & d : dets) {
      const auto name = model_->name(d.cls);
      out.detections.push_back(make(img, depth, name, d));
      boxes.emplace_back(name, d.box);
      if (name == signal_class_ && (!best_signal || d.score > best_signal->score)) {best_signal = &d;}
    }
    pub_->publish(out);
    if (sig_pub_) {publish_signal(img, msg->header, best_signal, now);}
    return boxes;
  }

  interfaces::msg::PuckDetection make(
    const cv::Mat & img, const std::optional<cv::Mat> & depth, const std::string & name, const Detection & det)
  {
    interfaces::msg::PuckDetection d;
    d.class_name = name;
    d.score = det.score;
    d.tag_id = -1;
    const auto & b = det.box;
    d.cx = (b.x1 + b.x2) / 2;
    d.cy = (b.y1 + b.y2) / 2;
    d.width = b.x2 - b.x1;
    d.height = b.y2 - b.y1;
    const std::string suffix = "_fallen";
    d.fallen = name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
    const auto base = d.fallen ? name.substr(0, name.size() - suffix.size()) : name;
    if (!puck_classes_.count(base)) {return d;}

    d.color = classify_color(center_roi(img, b, color_frac_), color_).first;

    std::vector<cv::Point2f> corners;
    if (read_tags_) {
      auto [id, c] = tag_reader_->read(img, b);
      d.tag_id = id;
      corners = std::move(c);
    }
    if (!has_info_) {return d;}
    if (depth) {   // 손목: depth 로 퍽 중심 위치
      if (const auto pt = depth_point(*depth, b, K_)) {
        d.has_pose = true;
        d.pose.position.x = (*pt)[0];
        d.pose.position.y = (*pt)[1];
        d.pose.position.z = (*pt)[2];
        d.pose.orientation.w = 1.0;
        return d;
      }
    }
    if (!corners.empty()) {   // depth 없으면 태그 PnP (태그 면의 자세)
      if (const auto pose = tag_pose(corners, tag_size_, K_, D_, fisheye_)) {
        d.has_pose = true;
        d.pose.position.x = pose->t[0];
        d.pose.position.y = pose->t[1];
        d.pose.position.z = pose->t[2];
        const auto q = rot_to_quat(pose->R);
        d.pose.orientation.x = q[0];
        d.pose.orientation.y = q[1];
        d.pose.orientation.z = q[2];
        d.pose.orientation.w = q[3];
      }
    }
    return d;
  }

  void publish_signal(
    const cv::Mat & img, const std_msgs::msg::Header & header, const Detection * best, SteadyClock::time_point now)
  {
    interfaces::msg::SignalState msg;
    msg.header = header;
    msg.camera = camera_;
    msg.cx = msg.cy = -1.0f;
    if (best) {
      const cv::Rect r(cv::Point(static_cast<int>(best->box.x1), static_cast<int>(best->box.y1)),
        cv::Point(static_cast<int>(best->box.x2), static_cast<int>(best->box.y2)));
      const cv::Rect roi = r & cv::Rect(0, 0, img.cols, img.rows);
      signal_vote_->push(signal_state(roi.area() > 0 ? img(roi) : cv::Mat(), tiers_, v_margin_));
      last_signal_seen_ = now;
      msg.cx = (best->box.x1 + best->box.x2) / 2;
      msg.cy = (best->box.y1 + best->box.y2) / 2;
    } else if (since(now, last_signal_seen_) > signal_timeout_) {
      signal_vote_->clear();
    }
    const auto [value, ratio] = signal_vote_->result();
    msg.state = value ? *value : interfaces::msg::SignalState::UNKNOWN;
    msg.confidence = static_cast<float>(ratio);
    sig_pub_->publish(msg);
  }

  static cv::Mat to_depth_m(const Image & msg)
  {
    const cv::Mat d = cv_bridge::toCvCopy(msg, msg.encoding)->image;
    cv::Mat m;
    if (d.type() == CV_16UC1) {   // realsense 기본: mm
      d.convertTo(m, CV_32F, 1.0 / 1000.0);
    } else {
      d.convertTo(m, CV_32F);
    }
    return m;
  }

  rclcpp::Node & node_;

  // 설정
  std::string camera_, depth_topic_, signal_class_;
  double conf_{0.5}, iou_{0.7}, max_hz_{15.0}, tag_size_{0.025}, color_frac_{0.6}, v_margin_{40.0}, signal_timeout_{1.0};
  bool enabled_{true}, read_tags_{true};
  std::set<std::string> puck_classes_;
  ColorCfg color_;
  std::vector<Tier> tiers_;

  // 상태
  std::shared_ptr<Yolo> model_;
  BarcodeStage * barcode_{nullptr};   // 이 카메라가 barcode.camera 일 때만
  std::unique_ptr<TagReader> tag_reader_;
  std::unique_ptr<Voter<std::string>> signal_vote_;
  cv::Matx33d K_;
  cv::Mat D_;
  bool fisheye_{false}, has_info_{false};
  SteadyClock::time_point last_{}, last_signal_seen_{};
  std::deque<Image::ConstSharedPtr> depth_buf_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr pub_;
  rclcpp::Publisher<interfaces::msg::SignalState>::SharedPtr sig_pub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_, depth_sub_;
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
};

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : rclcpp::Node("perception")
  {
    const auto cameras = declare_parameter<std::vector<std::string>>("cameras", {"wide", "wrist"});
    provider_ = declare_parameter<std::string>("provider", "cuda");                // cuda | tensorrt | cpu
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", {"red_cyl", "green_cyl", "red_box", "green_box", "barcode", "traffic_light"});
    // 손목 모델을 안 주면 광각 모델을 같이 씀
    std::string fallback_model;

    auto pub = create_publisher<interfaces::msg::DetectionArray>("/perception/detections", 10);
    for (const auto & cam : cameras) {
      auto path = declare_parameter<std::string>(cam + ".model", "");
      if (path.empty()) {path = fallback_model;}
      if (fallback_model.empty()) {fallback_model = path;}
      cameras_.push_back(std::make_unique<CameraStage>(*this, cam, load_model(cam, path), pub));
    }

    // 바코드는 지정한 카메라의 이미지 콜백 안에서 같이 돎 (그 카메라와 같은 콜백 그룹)
    const auto barcode_cam = declare_parameter<std::string>("barcode.camera", "wrist");
    for (size_t i = 0; i < cameras.size(); ++i) {
      if (cameras[i] != barcode_cam) {continue;}
      barcode_ = std::make_unique<BarcodeStage>(*this, cameras_[i]->group());
      cameras_[i]->set_barcode(barcode_.get());
      RCLCPP_INFO(get_logger(), "[%s] 바코드 → /perception/task_text", barcode_cam.c_str());
    }
    if (!barcode_) {
      RCLCPP_WARN(get_logger(), "barcode.camera=%s 가 cameras 에 없음 — 바코드 판독 안 함", barcode_cam.c_str());
    }
  }

private:
  std::shared_ptr<Yolo> load_model(const std::string & cam, const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "[%s] model 파라미터가 비어 있음 — 탐지 안 함", cam.c_str());
      return nullptr;
    }
    if (const auto it = models_.find(path); it != models_.end()) {return it->second;}
    std::shared_ptr<Yolo> model;
    try {
      model = std::make_shared<Yolo>(path, provider_, class_names_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[%s] 모델을 못 읽음 %s: %s", cam.c_str(), path.c_str(), e.what());
      return nullptr;
    }
    std::string names;
    for (const auto & n : model->names()) {names += n + " ";}
    const auto s = model->input_size();
    RCLCPP_INFO(
      get_logger(), "[%s] 모델 %s (%dx%d, %s) 클래스 %s", cam.c_str(), path.c_str(),
      s.width, s.height, model->provider().c_str(), names.c_str());
    models_[path] = model;
    return model;
  }

  std::string provider_;
  std::vector<std::string> class_names_;
  std::map<std::string, std::shared_ptr<Yolo>> models_;   // 같은 모델 파일은 세션 하나로
  std::vector<std::unique_ptr<CameraStage>> cameras_;
  std::unique_ptr<BarcodeStage> barcode_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception::PerceptionNode>();
  // 카메라마다 콜백 그룹이 따로라 스레드 2개면 광각 · 손목이 동시에 돎
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
