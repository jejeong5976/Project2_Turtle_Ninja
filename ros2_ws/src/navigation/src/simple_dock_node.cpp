// simple_dock_node.cpp — 규칙 정밀 접근 노드. dock_to_pose (NavigateToPose 형) 를 받아 박스 · 장소 정차점에 붙음
//   nav2.launch.py dock:=simple 이 띄움 (기본 dock:=rl = RL dock3)
//
// 받는 것
//   - 액션 <action_name> (기본 dock_to_pose): 목표 (보통 map, 박스를 봄)
//   - /scan (scan_merger): 박스 앞면 찾기 · 추적
//   - raw_topics (기본 /lidar_front/scan · /lidar_rear/scan): 몸 상자 안 지운 원래 점, 안전 검사
//   - odom_topic (기본 /odom): 넘겨받을 때 아직 미끄러지는지
//   - TF odom → base_footprint, map → odom (목표를 한 번 odom 으로)
// 보내는 것
//   - cmd_vel_nav (TwistStamped, 20 Hz) → collision_monitor → cmd_vel_hold → /cmd_vel. 끝나면 0 몇 번
//   - <cm_node>/set_parameters: 목표 동안 영역 끄기 (박스 면 1 cm 앞이라 PolygonStop 이 걸림), 끝나면 켜기
// 동작
//   - DockCore (simple_dock_core): 제자리로 돌아 박스를 보고 → 곧게 다가감. 대신 라이다 원래 점으로 몸 둘레 물체를 직접 봄 → 막히면 멈추고 실패
//   - 끝나면 (성공 · 실패 · 취소 · 시간 초과 모두) 영역을 다시 켠 뒤 (3 번까지) 결과를 보냄 → 바로 다음 목표가 와도 안 겹침
//     다시 켜기가 3 번 다 실패하면: 결과 = 실패 (도착했어도), 1 s 마다 다시 켜기를 이어가고 켜질 때까지 새 목표 거절
//   - 합친 스캔은 그 스캔 시각의 TF 로 · 안전 틈은 원래 스캔 나이만큼 일찍 · 넘겨받을 때 아직 미끄러지면 설 때까지 0 (최대 0.6 s)
//   - 박스를 한 번도 못 보고 지도 자리에 서면 경고 (--require_box 면 실패)
// note
//   - 명령줄 인자는 옛 파이썬 판 (simple_dock_node.py, 삭제) 과 같음. 시험 = test/test_simple_dock.cpp

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "navigation/simple_dock_core.hpp"

namespace navigation
{

using namespace std::chrono_literals;
using dock::DockCore;
using dock::DockParams;
using dock::DockState;
using rl::Pose2;
using rl::Vec2;
using Nav = nav2_msgs::action::NavigateToPose;
using NavGoal = rclcpp_action::ServerGoalHandle<Nav>;
using SetParameters = rcl_interfaces::srv::SetParameters;
using Scan = sensor_msgs::msg::LaserScan;

struct DockArgs
{
  std::string node_name = "simple_dock", action_name = "dock_to_pose";
  double v_max = 0.15, w_max = 0.3, margin = 0.02, tol_pos = 0.005, tol_ok = 0.015, tol_yaw_deg = 2.0;
  double timeout = 40.0, rate = 20.0, box_dist = 0.2575, search = 0.25;
  bool sim_time = false, box_track = true, require_raw = true, require_box = false;
  std::string raw_topics = "/lidar_front/scan,/lidar_rear/scan", disable_polygons = "PolygonStop,FootprintApproach";
  std::string cm_node = "/collision_monitor", odom_topic = "/odom";
};

// argv 중 --ros-args 앞만 (launch Node 가 뒤에 붙임)
DockArgs parse_args(int argc, char ** argv)
{
  DockArgs a;
  std::map<std::string, std::string *> str{{"--node_name", &a.node_name}, {"--action_name", &a.action_name},
    {"--raw_topics", &a.raw_topics}, {"--disable_polygons", &a.disable_polygons}, {"--cm_node", &a.cm_node},
    {"--odom_topic", &a.odom_topic}};
  std::map<std::string, double *> num{{"--v_max", &a.v_max}, {"--w_max", &a.w_max}, {"--margin", &a.margin},
    {"--tol_pos", &a.tol_pos}, {"--tol_ok", &a.tol_ok}, {"--tol_yaw_deg", &a.tol_yaw_deg}, {"--timeout", &a.timeout},
    {"--rate", &a.rate}, {"--box_dist", &a.box_dist}, {"--search", &a.search}};
  std::map<std::string, std::pair<bool *, bool>> flag{{"--sim_time", {&a.sim_time, true}},
    {"--no_box_track", {&a.box_track, false}}, {"--no_require_raw", {&a.require_raw, false}},
    {"--require_box", {&a.require_box, true}}};
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--ros-args") {break;}
    if (auto f = flag.find(k); f != flag.end()) {*f->second.first = f->second.second; continue;}
    if (i + 1 >= argc) {throw std::runtime_error("값 없음: " + k);}
    const std::string v = argv[++i];
    if (auto s = str.find(k); s != str.end()) {*s->second = v;} else if (auto n = num.find(k); n != num.end()) {
      *n->second = std::stod(v);
    } else {
      throw std::runtime_error("모르는 인자: " + k);
    }
  }
  if (a.rate <= 0) {throw std::runtime_error("--rate 는 0 보다 커야 함");}
  return a;
}

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  for (std::string t; std::getline(ss, t, ',');) {if (!t.empty()) {out.push_back(t);}}
  return out;
}

std::string fmt(const char * f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// 스캔 → 점 (range_min < r < min(range_max, max_r))
std::vector<Vec2> scan_pts(const Scan & m, double max_r)
{
  std::vector<Vec2> out;
  const double hi = std::min(static_cast<double>(m.range_max), max_r);
  for (size_t i = 0; i < m.ranges.size(); ++i) {
    const double r = m.ranges[i];
    if (!std::isfinite(r) || r <= m.range_min || r >= hi) {continue;}
    const double a = m.angle_min + m.angle_increment * static_cast<double>(i);
    out.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return out;
}

class SimpleDockNode : public rclcpp::Node
{
public:
  explicit SimpleDockNode(const DockArgs & a)
  : Node(a.node_name, rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", a.sim_time)})), a_(a)
  {
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cb_;
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    subs_.push_back(create_subscription<Scan>("/scan", rclcpp::SensorDataQoS(), [this](Scan::ConstSharedPtr m) {
        std::lock_guard lk(mtx_);
        scan_ = m;
      }, so));
    for (const auto & t : split(a.raw_topics)) {
      subs_.push_back(create_subscription<Scan>(t, rclcpp::SensorDataQoS(), [this, t](Scan::ConstSharedPtr m) {
          std::lock_guard lk(mtx_);
          raw_[t] = m;
        }, so));
    }
    if (!a.odom_topic.empty()) {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(a.odom_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto & tw = m->twist.twist;
          std::lock_guard lk(mtx_);
          odom_v_ = {std::hypot(tw.linear.x, tw.linear.y), std::abs(tw.angular.z)};
          odom_t_ = std::chrono::steady_clock::now();
        }, so);
    }
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_nav", 10);
    polys_ = split(a.disable_polygons);
    if (!polys_.empty()) {
      cm_ = create_client<SetParameters>(a.cm_node + "/set_parameters", rclcpp::ServicesQoS(), cb_);
      timer_ = create_wall_timer(1s, [this] {retry_restore();}, cb_);
    }
    server_ = rclcpp_action::create_server<Nav>(this, a.action_name,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal>) {return on_goal();},
        [](std::shared_ptr<NavGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<NavGoal> gh) {
          std::lock_guard lk(worker_mtx_);
          if (stopping_) {gh->abort(std::make_shared<Nav::Result>()); return;}
          if (worker_.joinable()) {worker_.join();}       // 이전 목표 스레드가 끝난 뒤 재사용함
          worker_ = std::thread([this, gh] {execute(gh);});
        },
        rcl_action_server_get_default_options(), cb_);
    RCLCPP_INFO(get_logger(), "규칙 정밀 접근: %s · 최고 %.2f m/s · 몸 둘레 %.0f cm 안 물체면 멈춤 · 박스 맞춤 %s · 끝 %.1f cm %.1f° (안 줄면 %.1f cm)",
      a.action_name.c_str(), a.v_max, a.margin * 100, a.box_track ? fmt("켬 (찾기 ±%.0f cm)", a.search * 100).c_str() : "끔",
      a.tol_pos * 100, a.tol_yaw_deg, a.tol_ok * 100);
  }

  ~SimpleDockNode() override {shutdown_worker();}

  // executor 콜백이 끝난 뒤 호출함. 멤버 해제 전에 실행 스레드를 회수함
  void shutdown_worker()
  {
    stopping_ = true;
    if (timer_) {timer_->cancel();}
    std::lock_guard lk(worker_mtx_);
    if (worker_.joinable()) {worker_.join();}
  }

  void stop() noexcept
  {
    for (int i = 0; i < 5; ++i) {
      try {send({0, 0, 0});} catch (const std::exception &) {break;}  // ROS 종료 뒤 발행 실패 허용함
      std::this_thread::sleep_for(20ms);
    }
  }

private:
  // ---------------------------------------------------------------- 입력
  Pose2 tf2d(const std::string & target, const std::string & source, const tf2::TimePoint & t = tf2::TimePointZero)
  {
    const auto tr = tf_->lookupTransform(target, source, t);   // 없으면 tf2::TransformException
    return {tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation)};
  }

  // 라이다 원래 점 (몸 좌표), 쓴 라이다 수, 쓴 것 중 가장 오래된 나이 [s]. 0.3 s 넘게 오래된 것은 안 씀
  std::tuple<std::vector<Vec2>, int, double> raw_body()
  {
    std::vector<Scan::ConstSharedPtr> raws;
    {
      std::lock_guard lk(mtx_);
      for (const auto & [t, m] : raw_) {raws.push_back(m);}
    }
    std::vector<Vec2> out;
    int n = 0;
    double age_max = 0.0;
    const double now = get_clock()->now().seconds();
    for (const auto & m : raws) {
      const double age = now - rclcpp::Time(m->header.stamp).seconds();
      if (age > 0.3) {continue;}
      Pose2 T;
      try {
        T = tf2d("base_footprint", m->header.frame_id);
      } catch (const tf2::TransformException &) {
        continue;
      }
      const auto P = scan_pts(*m, 1.0);
      if (P.empty()) {continue;}
      for (const auto & p : P) {out.push_back(rl::apply(T, p));}
      ++n;
      age_max = std::max(age_max, age);
    }
    return {std::move(out), n, age_max};
  }

  // 합친 스캔 → odom (그 스캔을 잰 시각의 자세로: 최신 TF 면 박스가 v × 지연만큼 멀리 찍혀 면 틈이 줄어듦)
  std::vector<Vec2> scan_odom(const Scan & m)
  {
    const std::string fr = m.header.frame_id.empty() ? "base_footprint" : m.header.frame_id;
    Pose2 T;
    try {
      T = tf2d("odom", fr, tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(m.header.stamp).nanoseconds())));
    } catch (const tf2::TransformException &) {
      try {
        T = tf2d("odom", fr);
      } catch (const tf2::TransformException &) {
        return {};
      }
    }
    auto P = scan_pts(m, 1.5);
    for (auto & p : P) {p = rl::apply(T, p);}
    return P;
  }

  // ---------------------------------------------------------------- 출력
  void send(const std::array<double, 3> & v)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = "base_footprint";
    m.twist.linear.x = v[0];
    m.twist.linear.y = v[1];
    m.twist.angular.z = v[2];
    pub_->publish(m);
  }

  bool set_polygons(bool enabled, int tries = 1)
  {
    if (polys_.empty()) {return true;}
    std::string joined;
    for (const auto & n : polys_) {joined += (joined.empty() ? "" : ",") + n;}
    for (int k = 0; k < tries; ++k) {
      if (!cm_->wait_for_service(1s)) {
        RCLCPP_WARN(get_logger(), "collision_monitor set_parameters 없음 → 영역 못 바꿈");
        continue;
      }
      auto req = std::make_shared<SetParameters::Request>();
      for (const auto & n : polys_) {req->parameters.push_back(rclcpp::Parameter(n + ".enabled", enabled).to_parameter_msg());}
      auto fut = cm_->async_send_request(req);
      if (fut.wait_for(2s) != std::future_status::ready) {continue;}
      const auto res = fut.get();
      if (res && std::all_of(res->results.begin(), res->results.end(), [](const auto & r) {return r.successful;})) {
        RCLCPP_INFO(get_logger(), "collision_monitor %s %s", joined.c_str(), enabled ? "켬" : "끔");
        return true;
      }
    }
    const auto msg = fmt("collision_monitor %s %s 실패 (%d 번)%s", joined.c_str(), enabled ? "켬" : "끔", tries,
        enabled ? " → 영역이 꺼진 채: 다음 주행 전에 collision_monitor 확인" : "");
    if (enabled) {RCLCPP_ERROR(get_logger(), "%s", msg.c_str());} else {RCLCPP_INFO(get_logger(), "%s", msg.c_str());}
    return false;
  }

  // 영역 다시 켜기 실패 뒤 1 s 마다 다시 시도 (목표 실행 중이 아닐 때만)
  void retry_restore()
  {
    {
      std::lock_guard lk(goal_mtx_);
      if (stopping_ || !zones_off_ || busy_) {return;}
      busy_ = true;                                     // 재시도 동안 새 목표가 끼지 않게
    }
    if (set_polygons(true)) {
      RCLCPP_INFO(get_logger(), "collision_monitor 영역 다시 켬 → 새 목표 받음");
      zones_off_ = false;
    }
    std::lock_guard lk(goal_mtx_);
    busy_ = false;
  }

  // 넘겨받을 때 Nav2 · RL 주행 속도로 아직 미끄러지면 설 때까지 0 (첫 명령이 돌기면 안전 검사의 다가가는 속도가 0 이라)
  double settle(double t_max = 0.6, double v_eps = 0.02, double w_eps = 0.08)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const auto since = [&] {return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();};
    while (since() < t_max && rclcpp::ok() && !stopping_) {
      std::optional<std::pair<double, double>> v;
      std::chrono::steady_clock::time_point vt;
      {
        std::lock_guard lk(mtx_);
        v = odom_v_;
        vt = odom_t_;
      }
      if (!v || std::chrono::duration<double>(std::chrono::steady_clock::now() - vt).count() > 0.3 ||
        (v->first < v_eps && v->second < w_eps))
      {
        break;
      }
      send({0, 0, 0});
      std::this_thread::sleep_for(50ms);
    }
    return since();
  }

  // ---------------------------------------------------------------- 실행
  rclcpp_action::GoalResponse on_goal()
  {
    std::lock_guard lk(goal_mtx_);                      // 받는 순간 바로 바쁨 (실행 전에 두 번째 목표가 오면 거절)
    if (stopping_) {return rclcpp_action::GoalResponse::REJECT;}
    if (zones_off_) {                                   // busy 보다 먼저 (다시 켜기 재시도 중에도 이 이유로)
      RCLCPP_ERROR(get_logger(), "collision_monitor 영역을 아직 다시 못 켬 → 목표 거절 (1 s 마다 다시 켜는 중)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_) {
      RCLCPP_WARN(get_logger(), "다른 목표 실행 중 → 거절");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  enum class End {Succeed, Cancel, Abort};

  void execute(std::shared_ptr<NavGoal> gh)
  {
    End end = End::Abort;
    try {
      end = run(gh);
    } catch (const std::exception & e) {                // 예외로 끝나도 멈추고 · 영역 켜고 · 실패로
      RCLCPP_ERROR(get_logger(), "정밀 접근 예외: %s", e.what());
      stop();
    }
    // 결과보다 먼저 (결과 직후 오는 다음 목표 · 주행이 영역 꺼진 채가 안 되게)
    bool restored = false;
    try {
      restored = set_polygons(true, 3);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "영역 복구 예외: %s", e.what());
    }
    if (!restored) {
      zones_off_ = true;                                // 타이머가 다시 켤 때까지 새 목표 거절
      if (end == End::Succeed) {RCLCPP_ERROR(get_logger(), "도착했지만 영역을 다시 못 켜서 실패로 보냄");}
      end = End::Abort;
    }
    auto res = std::make_shared<Nav::Result>();
    try {
      if (end == End::Succeed) {gh->succeed(res);} else if (end == End::Cancel) {gh->canceled(res);} else {gh->abort(res);}
    } catch (const std::exception &) {                  // 종료 중
    }
    std::lock_guard lk(goal_mtx_);                      // 결과를 보낸 뒤에 다음 목표를 받음
    busy_ = false;
  }

  End run(const std::shared_ptr<NavGoal> & gh)
  {
    const auto ps = gh->get_goal()->pose;
    const std::string fr = ps.header.frame_id.empty() ? "map" : ps.header.frame_id;
    Pose2 T;
    try {
      T = tf2d("odom", fr);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "TF %s → odom 없음: %s", fr.c_str(), e.what());
      return End::Abort;
    }
    const Vec2 g = rl::apply(T, {ps.pose.position.x, ps.pose.position.y});
    const double gyaw = yaw_of(ps.pose.orientation) + T.yaw;
    DockParams dp;
    dp.box_track = a_.box_track;
    dp.box_dist = a_.box_dist;
    dp.v_max = a_.v_max;
    dp.w_max = a_.w_max;
    dp.margin = a_.margin;
    dp.tol_pos = a_.tol_pos;
    dp.tol_yaw_deg = a_.tol_yaw_deg;
    dp.tol_ok = a_.tol_ok;
    dp.search = a_.search;
    DockCore core(g, gyaw, dp);
    RCLCPP_INFO(get_logger(), "정밀 접근 목표 odom (%+.3f, %+.3f, %+.1f°)", g.x, g.y, gyaw * 180.0 / M_PI);
    const double ts = settle();                         // 영역을 끄기 전에 (미끄러지는 동안은 collision_monitor 가 그대로 지킴)
    if (ts > 0.06) {RCLCPP_INFO(get_logger(), "넘겨받은 속도가 남아 %.2f s 세운 뒤 시작", ts);}
    if (stopping_ || !rclcpp::ok()) {stop(); return End::Abort;}
    if (!set_polygons(false)) {RCLCPP_WARN(get_logger(), "영역을 못 꺼서 그대로 진행 (박스 앞에서 PolygonStop 이 세울 수 있음)");}
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&] {return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();};
    double last_log = -1e9;
    Scan::ConstSharedPtr seen;
    int n_scan = 0;
    const auto period = std::chrono::duration<double>(1.0 / a_.rate);
    while (rclcpp::ok() && !stopping_) {
      if (gh->is_canceling()) {
        stop();
        RCLCPP_INFO(get_logger(), "취소됨");
        return End::Cancel;
      }
      if (elapsed() > a_.timeout) {
        stop();
        RCLCPP_WARN(get_logger(), "%.0f s 넘음 → 실패", a_.timeout);
        return End::Abort;
      }
      Pose2 p;
      try {
        p = tf2d("odom", "base_footprint");
      } catch (const tf2::TransformException &) {
        send({0, 0, 0});
        std::this_thread::sleep_for(period);
        continue;
      }
      auto [raw, n_raw, age] = raw_body();
      if (n_raw == 0 && a_.require_raw) {
        stop();
        RCLCPP_ERROR(get_logger(), "라이다 원래 점 (안전 검사용) 이 안 옴 → 실패");
        return End::Abort;
      }
      Scan::ConstSharedPtr m;
      {
        std::lock_guard lk(mtx_);
        m = scan_;
      }
      if (m && m != seen) {                             // 새 스캔마다 한 번 (주기 20 Hz > 스캔 10 Hz → 같은 스캔 두 번 넣지 않음)
        seen = m;
        ++n_scan;
        core.update_box(scan_odom(*m), {p.x, p.y});
      }
      const auto [cmd, st] = core.step(p, raw, elapsed(), age);
      if (st == DockState::Fail) {
        stop();
        RCLCPP_ERROR(get_logger(), "멈춤: %s", core.why.c_str());
        return End::Abort;
      }
      const auto & tr = core.tracker();
      if (st == DockState::Done) {
        stop();
        RCLCPP_INFO(get_logger(), "%s%s", core.why.c_str(),
          tr ? fmt(" · 박스 맞춤 %d 번 (%s)", tr->n_ok, tr->last_why.c_str()).c_str() : "");
        if (tr && tr->n_ok == 0 && n_scan >= 3) {       // 지도 자리에 섬 — 박스가 search 밖이거나 다른 물체 (시작부터 정차점이면 안 셈)
          if (a_.require_box) {
            RCLCPP_ERROR(get_logger(), "박스 앞면을 한 번도 못 찾음 → 실패 (--require_box)");
            return End::Abort;
          }
          RCLCPP_WARN(get_logger(), "박스 앞면을 한 번도 못 찾음 → 지도 정차점에 섬 (장소 정차면 정상, 박스면 지도 자리 ±%.0f cm 밖?)",
            a_.search * 100);
        }
        return End::Succeed;
      }
      send(cmd);
      auto fb = std::make_shared<Nav::Feedback>();
      const Pose2 gt = core.target();
      fb->distance_remaining = static_cast<float>(std::hypot(gt.x - p.x, gt.y - p.y));
      gh->publish_feedback(fb);
      if (elapsed() - last_log > 1.0) {
        last_log = elapsed();
        RCLCPP_INFO(get_logger(), "%s · 남은 %.1f cm · 명령 (%+.3f, %+.3f, %+.2f)%s", core.mode.c_str(),
          fb->distance_remaining * 100, cmd[0], cmd[1], cmd[2], tr ? (" · 박스 " + tr->last_why).c_str() : "");
      }
      std::this_thread::sleep_for(period);
    }
    stop();
    return End::Abort;
  }

  std::atomic<bool> stopping_{false};
  std::mutex worker_mtx_;                              // 스레드 생성과 종료 대기를 직렬화함
  std::thread worker_;
  DockArgs a_;
  rclcpp::CallbackGroup::SharedPtr cb_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::vector<rclcpp::Subscription<Scan>::SharedPtr> subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Client<SetParameters>::SharedPtr cm_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<Nav>::SharedPtr server_;
  std::vector<std::string> polys_;

  std::mutex mtx_;                                      // 입력 (구독 ↔ 실행 스레드)
  Scan::ConstSharedPtr scan_;
  std::map<std::string, Scan::ConstSharedPtr> raw_;
  std::optional<std::pair<double, double>> odom_v_;     // (|v| m/s, |w| rad/s)
  std::chrono::steady_clock::time_point odom_t_;

  std::mutex goal_mtx_;
  bool busy_ = false;
  std::atomic<bool> zones_off_{false};                  // 영역 다시 켜기 실패 → 켜질 때까지 새 목표 거절
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  navigation::DockArgs a;
  try {
    a = navigation::parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[simple_dock_node] %s\n", e.what());
    return 2;
  }
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<navigation::SimpleDockNode>(a);
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
    ex.add_node(node);
    ex.spin();
    node->shutdown_worker();
    node->stop();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[simple_dock_node] %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
