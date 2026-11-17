// perception_node.cpp — 인지 노드 perception: 광각 · 손목 · 신호등 · 바코드를 한 노드에서 돌림
//
// 역할
//   - 파이프라인 조립만 함. 실제 처리는 각 파이프라인이 자기 콜백 그룹에서 함
//       WidePipeline   광각 상자 검출 → 퍽 · 신호등
//       WristPipeline  손목 퍽 분할 + 깊이 기하 → 퍽 자세
//       WristTagStage  손목 컬러 → AprilTag ID · 모서리 (켜져 있을 때만)
//       BarcodeStage   barcode.camera 카메라 콜백 안에서 판독 → 바코드 원문
//   - 파이프라인을 켜고 끄는 서비스 제공
// 입력
//   - 광각 영상 /wide_cam/image_raw
//   - 손목 /camera/camera/color/image_raw · aligned_depth_to_color/image_raw · color/camera_info, TF
//   - 서비스, ~ = /perception
//       ~/wide/enable · ~/wrist/enable · ~/wrist_tags/enable · ~/box/enable · ~/barcode/enable  std_srvs/SetBool
//       ~/barcode/reset  std_srvs/Trigger, 바코드 다수결 비우기
// 출력
//   - /perception/wide/detections    interfaces/DetectionArray
//   - /perception/wrist/detections   interfaces/DetectionArray
//   - /perception/wrist/tags         interfaces/WristTagArray, 손목 AprilTag ID · 모서리 (적재함 칸 · 쥔 퍽, control_master 가 위치 계산)
//   - /perception/wrist/box          interfaces/DetectionArray, 박스 윗면 중심 · yaw (손목 깊이)
//   - /perception/traffic_light      interfaces/TrafficLight
//   - /perception/task_text          interfaces/TaskText
//   - /perception/wrist_debug_image  sensor_msgs/Image, wrist.publish_debug 일 때만
//   - /perception/wide_debug_image   sensor_msgs/Image, wide.publish_debug 일 때만
// 실행
//   - ros2 launch perception perception.launch.py wide_model:=<광각>.engine wrist_model:=<손목>.engine
//   - 설정은 레포 config/perception.yaml
// note
//   - TensorRT 가 있는 Jetson 에서만 빌드됨
//   - 콜백 그룹 4 개 광각 · 손목 · 손목 태그 · 관리 + MultiThreadedExecutor 4 스레드 → 서로 기다리지 않음
//   - TensorRT 엔진은 파이프라인마다 따로 둠
//   - 서비스는 플래그만 바꾸고, 실제 초기화는 각 파이프라인이 다음 콜백에서 함
//   - wide 를 꺼도 바코드 판독은 ~/barcode/enable 로 따로 켬
//   - 멈춤 감시 (watchdog_s, 기본 5 s, 0 = 끔): 콜백 그룹 4 개 + 기본 그룹마다 0.5 s 하트비트 타이머, executor 밖 감시 스레드 (HangWatch) 가 봄.
//       한 그룹이라도 watchdog_s 넘게 하트비트가 없으면
//       1) ~/.ros/perception_hang_<시각>.txt 에 모든 스레드 CPU % · 스택 (+ gdb thread apply all bt, 최대 8 s) 2) stderr ERROR 3) _exit(70)
//       → perception.launch.py respawn (2 s) 이 같은 파라미터로 다시 띄움. 기록이 멈춰도 40 s 뒤 SIGALRM 으로 끝남
//   - respawn 되살리기 (restart_state.hpp): 켜기 서비스 상태 · 실행 중 ros2 param set 값을 ~/.ros/perception_state.txt 에 둠.
//       같은 launch 의 respawn 이면 시작 때 그대로 되살림 (부르는 쪽은 "켰다고 앎" 이면 다시 안 보냄), 새 launch 면 yaml 그대로
//   - /perception/session  std_msgs/String (transient_local): 프로세스마다 다른 글. 바뀌면 = perception 이 다시 뜸
//       → planning 실행기 · mission 이 켜기 상태를 다시 보내고 DETECTION_LOST 를 회복으로 봄
//   - 시작 때 ~/.ros/perception_hang_last.txt 가 있으면 (직전 프로세스가 멈춰 끝남) ERROR 로 그 기록 파일 이름을 알림
//       (스레드 하나가 무한 반복에 빠지면 노드 전체 · DDS 생존 신호가 멈춰도 프로세스는 살아 있어 respawn 이 안 됨)
//   - 한쪽 엔진을 못 읽거나 카메라가 없어도 나머지는 계속 돎
//   - 토픽 이름 · 기본값 설명은 wide_pipeline.hpp · wrist_pipeline.hpp · barcode.hpp

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "interfaces/msg/detection_array.hpp"
#include "perception/hang_watch.hpp"
#include "perception/restart_state.hpp"

#include "barcode.hpp"
#include "wide_pipeline.hpp"
#include "wrist_pipeline.hpp"
#include "wrist_tag_stage.hpp"

namespace perception
{

// ---------------------------------------------------------------- 파라미터 값 ↔ 글 (respawn 되살리기, 파일에는 16 진으로)
//   b:1 · i:42 · d:0.5 · s:<글> · B:1,0 · I:1,2 · D:0.1,0.2 · S:<16진>,<16진>
namespace
{

template<typename V, typename F>
std::string join_vec(const V & v, F && one)
{
  std::string o;
  for (std::size_t i = 0; i < v.size(); ++i) {o += (i ? "," : "") + one(v[i]);}
  return o;
}

std::string num(double x)
{
  char b[40];
  std::snprintf(b, sizeof(b), "%.17g", x);
  return b;
}

std::vector<std::string> split_comma(const std::string & s)
{
  std::vector<std::string> out;
  if (s.empty()) {return out;}
  std::stringstream ss(s);
  std::string x;
  while (std::getline(ss, x, ',')) {out.push_back(x);}
  return out;
}

std::string param_to_text(const rclcpp::ParameterValue & v)
{
  using T = rclcpp::ParameterType;
  switch (v.get_type()) {
    case T::PARAMETER_BOOL: return std::string("b:") + (v.get<bool>() ? "1" : "0");
    case T::PARAMETER_INTEGER: return "i:" + std::to_string(v.get<int64_t>());
    case T::PARAMETER_DOUBLE: return "d:" + num(v.get<double>());
    case T::PARAMETER_STRING: return "s:" + v.get<std::string>();
    case T::PARAMETER_BOOL_ARRAY: return "B:" + join_vec(v.get<std::vector<bool>>(), [](bool b) {return std::string(b ? "1" : "0");});
    case T::PARAMETER_INTEGER_ARRAY: return "I:" + join_vec(v.get<std::vector<int64_t>>(), [](int64_t i) {return std::to_string(i);});
    case T::PARAMETER_DOUBLE_ARRAY: return "D:" + join_vec(v.get<std::vector<double>>(), [](double d) {return num(d);});
    case T::PARAMETER_STRING_ARRAY:
      return "S:" + join_vec(v.get<std::vector<std::string>>(), [](const std::string & s) {return hex_encode(s);});
    default: return "";
  }
}

std::optional<rclcpp::ParameterValue> text_to_param(const std::string & t)
{
  if (t.size() < 2 || t[1] != ':') {return std::nullopt;}
  const std::string d = t.substr(2);
  try {
    switch (t[0]) {
      case 'b': return rclcpp::ParameterValue(d == "1");
      case 'i': return rclcpp::ParameterValue(static_cast<int64_t>(std::stoll(d)));
      case 'd': return rclcpp::ParameterValue(std::stod(d));
      case 's': return rclcpp::ParameterValue(d);
      case 'B': {std::vector<bool> v; for (auto & x : split_comma(d)) {v.push_back(x == "1");} return rclcpp::ParameterValue(v);}
      case 'I': {std::vector<int64_t> v; for (auto & x : split_comma(d)) {v.push_back(std::stoll(x));} return rclcpp::ParameterValue(v);}
      case 'D': {std::vector<double> v; for (auto & x : split_comma(d)) {v.push_back(std::stod(x));} return rclcpp::ParameterValue(v);}
      case 'S': {
          std::vector<std::string> v;
          for (auto & x : split_comma(d)) {
            const auto s = hex_decode(x);
            if (!s) {return std::nullopt;}
            v.push_back(*s);
          }
          return rclcpp::ParameterValue(v);
        }
      default: return std::nullopt;
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string state_path() {return ros_home_dir() + "/perception_state.txt";}

}  // namespace

class PerceptionNode : public rclcpp::Node
{
public:
  // restored: 같은 launch 의 respawn 이면 앞 프로세스가 남긴 상태 (켜기 · 파라미터). 파라미터는 options 에 이미 들어 있음
  PerceptionNode(const rclcpp::NodeOptions & options, std::optional<SavedState> restored);
  ~PerceptionNode() override;
  double watchdog_s() const {return watchdog_s_;}
  static constexpr std::array<const char *, 5> kGroupNames{"wide", "wrist", "wrist_tags", "admin", "default"};
  void set_hang_watch(HangWatch * w) {watch_ = w;}

private:
  void remember_enable(const std::string & name, bool on);
  void save_state();

  double watchdog_s_{5.0};
  std::atomic<HangWatch *> watch_{nullptr};
  std::array<rclcpp::TimerBase::SharedPtr, 5> beat_timers_;
  rclcpp::CallbackGroup::SharedPtr wide_group_, wrist_group_, tags_group_, admin_group_;
  rclcpp::Publisher<interfaces::msg::DetectionArray>::SharedPtr wide_det_pub_, wrist_det_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session_pub_;
  std::unique_ptr<BarcodeStage> barcode_;
  std::unique_ptr<WidePipeline> wide_;
  std::unique_ptr<WristPipeline> wrist_;
  std::unique_ptr<WristTagStage> tags_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr wide_srv_, wrist_srv_, tags_srv_, box_srv_, barcode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr barcode_reset_srv_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  std::mutex state_mtx_;
  SavedState state_;
};


using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options, std::optional<SavedState> restored)
: rclcpp::Node("perception", options)
{
  const auto mx = rclcpp::CallbackGroupType::MutuallyExclusive;
  wide_group_ = create_callback_group(mx);
  wrist_group_ = create_callback_group(mx);
  tags_group_ = create_callback_group(mx);
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
  tags_ = std::make_unique<WristTagStage>(*this, tags_group_, wrist_->tf_buffer());   // wrist.* 영상 이름 · TF 버퍼를 씀 → 손목 다음

  // 켜기 · 끄기 (서비스 · respawn 되살리기 같은 길)
  const std::vector<std::pair<std::string, std::function<void(bool)>>> setters{
    {"wide", [this](bool on) {wide_->set_enabled(on);}},
    {"wrist", [this](bool on) {wrist_->set_enabled(on);}},
    {"wrist_tags", [this](bool on) {tags_->set_enabled(on);}},
    {"box", [this](bool on) {wrist_->set_box_enabled(on);}},
    {"barcode", [this](bool on) {barcode_->set_enabled(on);}},
  };
  auto enable_srv = [this](const std::string & name, std::function<void(bool)> setter) {
      return create_service<SetBool>(
        "~/" + name + "/enable",
        [this, name, setter](const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
          setter(req->data);
          remember_enable(name, req->data);
          res->success = true;
          res->message = name + (req->data ? " on" : " off");
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        }, rclcpp::ServicesQoS(), admin_group_);
    };
  wide_srv_ = enable_srv("wide", setters[0].second);
  wrist_srv_ = enable_srv("wrist", setters[1].second);
  tags_srv_ = enable_srv("wrist_tags", setters[2].second);
  box_srv_ = enable_srv("box", setters[3].second);
  barcode_srv_ = enable_srv("barcode", setters[4].second);
  barcode_reset_srv_ = create_service<Trigger>(
    "~/barcode/reset", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      barcode_->request_reset();
      res->success = true;
      res->message = "cleared";
    }, rclcpp::ServicesQoS(), admin_group_);

  watchdog_s_ = declare_parameter<double>("watchdog_s", 5.0);   // 파라미터 기록 콜백 (아래) 보다 먼저 선언

  // respawn 되살리기: 같은 launch 면 앞 프로세스의 켜기 상태 (서비스와 같은 setter) · 파라미터 (options 로 이미 들어감) 를 이어 씀
  state_.launch = current_launch_mark();
  if (restored) {
    std::string en, pa;
    for (const auto & [name, set] : setters) {
      const auto it = restored->enables.find(name);
      if (it == restored->enables.end()) {continue;}
      set(it->second);
      state_.enables[name] = it->second;
      en += (en.empty() ? "" : " · ") + name + (it->second ? " 켬" : " 끔");
    }
    for (const auto & kv : restored->params) {
      state_.params[kv.first] = kv.second;
      pa += (pa.empty() ? "" : " · ") + kv.first;
    }
    RCLCPP_WARN(get_logger(), "respawn: 앞 perception 상태 되살림 — 켜기 [%s] · 실행 중 바꾼 파라미터 [%s] (%s)",
      en.empty() ? "없음" : en.c_str(), pa.empty() ? "없음" : pa.c_str(), state_path().c_str());
  }
  save_state();   // 새 launch 면 이전 파일을 이 launch 표시로 덮어씀
  // 실행 중 ros2 param set → 기록 (다음 respawn 때 같은 값). declare 도 이 콜백을 부르므로 모든 declare 뒤에 붙임
  param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        for (const auto & p : ps) {
          const std::string t = param_to_text(p.get_parameter_value());
          if (!t.empty()) {state_.params[p.get_name()] = hex_encode(t);}
        }
      }
      save_state();
    });

  // 프로세스마다 다른 글 → 받는 쪽이 respawn 을 앎 (transient_local: 늦게 붙어도 받음)
  {
    session_pub_ = create_publisher<std_msgs::msg::String>("/perception/session", rclcpp::QoS(1).transient_local().reliable());
    std_msgs::msg::String m;
    std::ostringstream o;
    o << "pid=" << ::getpid() << " start_ns=" << std::chrono::system_clock::now().time_since_epoch().count()
      << " restored=" << (restored ? 1 : 0);
    m.data = o.str();
    session_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "세션 %s → /perception/session", m.data.c_str());
  }

  // 직전 프로세스가 멈춰 끝났으면 (멈춤 감시가 남긴 표시) 알리고 표시를 치움
  {
    const std::string last = ros_home_dir() + "/perception_hang_last.txt";
    std::ifstream f(last);
    std::string path, what;
    if (f && std::getline(f, path)) {
      std::getline(f, what);
      RCLCPP_ERROR(get_logger(), "직전 perception 이 멈춰 (%s) 다시 켜짐 — 스택 기록 %s", what.c_str(), path.c_str());
      f.close();
      std::rename(last.c_str(), (last + ".seen").c_str());
    }
  }

  // 멈춤 감시 하트비트: 그룹마다 타이머 하나 (그 그룹 콜백이 멈추면 타이머도 못 돎). 기본 그룹 (파라미터 서비스 등) 도
  const std::array<rclcpp::CallbackGroup::SharedPtr, 5> groups{wide_group_, wrist_group_, tags_group_, admin_group_, nullptr};
  for (std::size_t i = 0; i < groups.size(); ++i) {
    beat_timers_[i] = create_wall_timer(std::chrono::milliseconds(500), [this, i]() {
        if (auto * w = watch_.load()) {w->beat(i);}
      }, groups[i]);
  }
}

void PerceptionNode::remember_enable(const std::string & name, bool on)
{
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    state_.enables[name] = on;
  }
  save_state();   // 잠금을 풀고 다시 잡음 (save_state 가 잡음)
}

void PerceptionNode::save_state()
{
  // 서비스 (관리 그룹) · 파라미터 (기본 그룹) 스레드가 같이 부를 수 있음 → 쓰기도 한 번에 하나 (같은 tmp 를 둘이 쓰면 깨짐)
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!save_state_file(state_path(), state_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "respawn 상태 파일을 못 씀: %s", state_path().c_str());
  }
}

// 파이프라인(구독 · 엔진)을 먼저 지우고 나서 노드가 사라지게 순서를 고정
PerceptionNode::~PerceptionNode()
{
  watch_ = nullptr;
  tags_.reset();
  wrist_.reset();
  wide_.reset();
  barcode_.reset();
}

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  perception::prepare_thread_dump();   // 신호 스택 처리기 · libgcc 를 지금 (멈춘 뒤 할 일 줄임)
  int code = 0;
  try {
    // respawn 되살리기: 같은 launch 면 앞 프로세스 파라미터를 overrides 로 (launch yaml 보다 앞섬), 켜기 상태는 노드가 setter 로
    const std::string spath = perception::ros_home_dir() + "/perception_state.txt";
    std::optional<perception::SavedState> restored;
    if (auto st = perception::load_state_file(spath); perception::should_restore(st, perception::current_launch_mark())) {restored = st;}
    rclcpp::NodeOptions opts;
    if (restored) {
      std::vector<rclcpp::Parameter> over;
      for (const auto & kv : restored->params) {
        const auto txt = perception::hex_decode(kv.second);
        const auto v = txt ? perception::text_to_param(*txt) : std::nullopt;
        if (v) {over.emplace_back(kv.first, *v);}
      }
      opts.parameter_overrides(over);
    }
    auto node = std::make_shared<perception::PerceptionNode>(opts, restored);
    // 콜백 그룹 4개 (광각 · 손목 · 손목 태그 · 관리) → 스레드 4개
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
    ex.add_node(node);
    // 멈춤 감시: executor 밖 스레드. ROS 로그 · DDS 를 안 씀 (멈춘 스레드가 그 잠금을 쥐고 있을 수 있음)
    perception::HangWatch::Options wo;
    wo.limit_s = node->watchdog_s();
    wo.on_hang = [](const std::string & what, double age, const std::string & path) {
        std::fprintf(stderr, "[ERROR] [perception]: 멈춤 감시: %s (%.1f s) — 스택 기록 %s → 종료, launch respawn 이 같은 파라미터로 다시 띄움\n",
          what.c_str(), age, path.empty() ? "(못 씀)" : path.c_str());
        std::fflush(stderr);
        ::_exit(70);   // abort 아님: apport 코어 덤프 (CUDA 프로세스는 수 GB) 로 respawn 이 늦어지지 않게
      };
    std::vector<std::string> names(perception::PerceptionNode::kGroupNames.begin(), perception::PerceptionNode::kGroupNames.end());
    perception::HangWatch watch(names, wo);
    node->set_hang_watch(&watch);
    // Ctrl-C · stop_robot 으로 끝내는 동안은 감시 안 함 (watch 가 사라지기 전에 콜백을 뺌)
    auto ctx = rclcpp::contexts::get_global_default_context();
    auto stop_cb = ctx->add_on_shutdown_callback([&watch]() {watch.request_stop();});
    std::shared_ptr<void> unhook(nullptr, [&](void *) {ctx->remove_on_shutdown_callback(stop_cb);});   // 예외로 나가도 watch 보다 먼저
    watch.start();
    ex.spin();
    watch.stop();
    node->set_hang_watch(nullptr);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
