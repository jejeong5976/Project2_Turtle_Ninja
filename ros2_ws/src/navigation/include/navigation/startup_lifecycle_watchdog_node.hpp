// startup_lifecycle_watchdog_node.hpp — AMCL→지도·TF→Nav2 시작과 lifecycle만 최대3회 복구함.
// scripts/startup_lifecycle_watchdog.py 의 C++ 판: 같은 노드 이름·파라미터·서비스·상태 JSON·로그 문구.
// 받음: /scan, /map, TF, lifecycle 상태/응답. 보냄: lifecycle 서비스, 상태 JSON.
// /mission/hold·cmd_vel·mission/start·하드웨어 재시작은 다루지 않음.
// recover_only (기본, nav2.launch startup_recovery:=true): Nav2 lifecycle_manager autostart 가 켜고, 이 노드는 실패한 것만 고침
//   - is_active false 가 abort_confirm_s 이어짐 (Aborting bringup) 또는 autostart_grace_s 지나도 준비 안 됨 → 그 manager 만 RESET→정합→STARTUP (최대 3)
//   - is_active 무응답 (manager 가 잃어버린 change_state 응답을 끝없이 기다림) → 그 lifecycle_manager 프로세스를 SIGTERM
//     (같은 사용자 · 같은 ROS_DOMAIN_ID · "__node:=<이름>" 만, restart_kill_s 뒤 남으면 SIGKILL) → launch respawn 이 다시 띄움 (최대 max_restarts)
//   - localization 이 ACTIVE 인데 map→odom TF 가 output_timeout_s 동안 없음 (AMCL 구독 유실) → localization RESET
//   - 자기 /scan · /map 구독이 resubscribe_s 동안 조용하면 구독을 새로 만듦 (DDS 짝짓기 유실 풀기)
// 가볍게: /scan 콜백은 첫 유효 거리에서 멈추고 시각만 저장. 0.1 s tick 은 문자열 버퍼를 다시 써서 할당 거의 없음.
#pragma once

#include <dirent.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iterator>
#include <sstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navigation/lifecycle_recovery_core.hpp"
#include "navigation/resubscribe_watch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace navigation
{

class StartupLifecycleWatchdog : public rclcpp::Node
{
public:
  using Manage = nav2_msgs::srv::ManageLifecycleNodes;
  using Trigger = std_srvs::srv::Trigger;
  using GetState = lifecycle_msgs::srv::GetState;
  using ChangeState = lifecycle_msgs::srv::ChangeState;

  struct Group
  {
    std::string key;   // localization / navigation
    lifecycle::LifecycleRecovery core;
    lifecycle::Snapshot snap;
    rclcpp::Client<Manage>::SharedPtr manager;
    rclcpp::Client<Trigger>::SharedPtr active;
    std::map<std::string, rclcpp::Client<GetState>::SharedPtr> state;
    std::map<std::string, rclcpp::Client<ChangeState>::SharedPtr> change;
    // 보낸 전환 하나 (manager 또는 노드)
    struct Op
    {
      int64_t token = 0;
      bool manager = true;
      std::string target;
      rclcpp::Client<Manage>::SharedFuture fm;
      rclcpp::Client<ChangeState>::SharedFuture fc;
      int64_t request_id = 0;
    };
    std::optional<Op> op;
  };

  // 상태 조회 하나 (is_active 또는 get_state)
  struct Poll
  {
    std::size_t group = 0;
    bool active = true;
    std::string node;
    rclcpp::Client<Trigger>::SharedFuture fa;
    rclcpp::Client<GetState>::SharedFuture fs;
    int64_t request_id = 0;
    double sent = 0.0;
    bool barrier = false;
  };

  static double monotonic()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  template<typename F>
  static bool done(const F & f)
  {
    return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  static std::string strip_slash(const std::string & s)
  {
    const auto b = s.find_first_not_of('/');
    if (b == std::string::npos) {
      return "";
    }
    const auto e = s.find_last_not_of('/');
    return s.substr(b, e - b + 1);
  }

  explicit StartupLifecycleWatchdog(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("startup_lifecycle_watchdog", options), tf_buffer(get_clock()), tf_listener(tf_buffer)
  {
    manage_localization = declare_parameter<bool>("manage_localization", true);
    scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    map_topic = declare_parameter<std::string>("map_topic", "/map");
    base_frame = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    map_frame = declare_parameter<std::string>("map_frame", "map");
    data_timeout = declare_parameter<double>("data_timeout_s", 3.0);
    poll_s = declare_parameter<double>("poll_s", 0.5);
    probe_timeout = declare_parameter<double>("probe_timeout_s", 5.0);
    lifecycle::RecoveryOptions o;
    o.max_recoveries = declare_parameter<int64_t>("max_recoveries", 3);
    o.command_timeout = declare_parameter<double>("manager_timeout_s", 90.0);
    o.node_timeout = declare_parameter<double>("node_timeout_s", 20.0);
    o.drain_timeout = declare_parameter<double>("drain_timeout_s", 120.0);
    o.verify_timeout = declare_parameter<double>("verify_timeout_s", 45.0);
    o.stable_s = declare_parameter<double>("bond_stable_s", 16.0);
    o.native_reconnect_s = declare_parameter<double>("native_reconnect_wait_s", 11.0);
    o.cleanup_timeout = declare_parameter<double>("cleanup_timeout_s", 90.0);
    // recover_only (기본): autostart 가 먼저, 실패한 manager 만 복구. false = 감시기가 STARTUP 을 맡음 (autostart false 와 함께)
    o.recover_only = declare_parameter<bool>("recover_only", true);
    o.autostart_grace_s = declare_parameter<double>("autostart_grace_s", 30.0);
    o.abort_confirm_s = declare_parameter<double>("abort_confirm_s", 6.0);
    o.hang_s = declare_parameter<double>("hang_s", 20.0);
    o.max_restarts = declare_parameter<int64_t>("max_restarts", 2);
    const double loc_output_timeout = declare_parameter<double>("localization_output_timeout_s", 30.0);
    restart_kill_s = declare_parameter<double>("restart_kill_s", 3.0);
    const double resub_s = declare_parameter<double>("resubscribe_s", 10.0);
    own_domain = lifecycle::normalize_domain(std::getenv("ROS_DOMAIN_ID"));
    const auto loc_nodes = declare_parameter<std::vector<std::string>>(
      "localization_nodes", std::vector<std::string>{"map_server", "amcl"});
    const auto nav_nodes = declare_parameter<std::vector<std::string>>(
      "navigation_nodes", std::vector<std::string>{"planner_server", "collision_monitor"});
    const auto loc_manager = declare_parameter<std::string>("localization_manager", "/lifecycle_manager_localization");
    const auto nav_manager = declare_parameter<std::string>("navigation_manager", "/lifecycle_manager_navigation");
    if (manage_localization) {
      lifecycle::RecoveryOptions lo = o;
      lo.output_timeout_s = o.recover_only ? loc_output_timeout : 0.0;   // localization 출력 = map→odom TF (AMCL)
      lo.autostart_needs_inputs = false;   // AMCL activate 는 스캔 없이도 됨 → 감시기 스캔 구독이 늦어도 판단 시작
      groups.push_back(std::make_unique<Group>(Group{"localization",
          lifecycle::LifecycleRecovery(loc_manager, loc_nodes, lo), {}, nullptr, nullptr, {}, {}, std::nullopt}));
    }
    groups.push_back(std::make_unique<Group>(Group{"navigation",
        lifecycle::LifecycleRecovery(nav_manager, nav_nodes, o), {}, nullptr, nullptr, {}, {}, std::nullopt}));
    for (auto & g : groups) {
      g->manager = create_client<Manage>(g->core.name + "/manage_nodes");
      g->active = create_client<Trigger>(g->core.name + "/is_active");
      for (const auto & name : g->core.nodes) {
        if (!g->state.count(name)) {
          g->state[name] = create_client<GetState>("/" + strip_slash(name) + "/get_state");
          g->change[name] = create_client<ChangeState>("/" + strip_slash(name) + "/change_state");
        }
      }
    }
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    scan_watch = ResubscribeWatch(resub_s, 60.0);
    map_watch = ResubscribeWatch(resub_s, 60.0);
    subscribe_scan();
    subscribe_map();
    status_pub = create_publisher<std_msgs::msg::String>("/startup/lifecycle_status", latched);
    // sim /clock이 멈춰도 서비스 timeout과 상태 보고는 단조 시계로 돌아감.
    timer = create_wall_timer(std::chrono::milliseconds(100), [this]() {tick();});
    if (o.recover_only) {
      RCLCPP_INFO(get_logger(),
        "시작 lifecycle 감시 (복구만): Nav2 autostart 먼저 → 실패한 manager 만 RESET (최대 %lld) · 멈춘 manager 는 프로세스 재시작 (최대 %lld)",
        static_cast<long long>(o.max_recoveries), static_cast<long long>(o.max_restarts));
    } else {
      RCLCPP_INFO(get_logger(),
        "시작 lifecycle 감시: 센서·odom → localization → 지도·map TF → navigation; 해당 manager만 최대3회 복구");
    }
  }

  void subscribe_scan()
  {
    sub_scan = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(*m);});
    scan_watch.start(monotonic());
  }

  void subscribe_map()
  {
    sub_map = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {on_map(*m);});
    map_watch.start(monotonic());
  }

  // 조용한 구독을 새로 만듦 (발행자가 있는데 짝짓기를 잃은 경우 풀림. 발행자가 없으면 해 없음)
  // 발행자가 보이는데 못 받으면 WARN, 아직 안 보이면 INFO. 지도는 발행자가 보일 때만
  void heal_subscriptions(double now)
  {
    if (scan_watch.due(now)) {
      const size_t pubs = sub_scan->get_publisher_count();
      sub_scan.reset();
      subscribe_scan();
      if (pubs > 0) {
        RCLCPP_WARN(get_logger(), "%s 발행자 %zu 인데 %s → 구독 다시 만듦 (%d 번째)", scan_topic.c_str(), pubs,
          scan_watch.ever() ? "끊김" : "못 받음", scan_watch.count());
      } else {
        RCLCPP_INFO(get_logger(), "%s 발행자 아직 안 보임 → 구독 새로 만듦 (%d 번째)", scan_topic.c_str(), scan_watch.count());
      }
    }
    if (!have_map && sub_map->get_publisher_count() == 0) {
      map_watch.start(now);
    } else if (!have_map && map_watch.due(now)) {
      const size_t pubs = sub_map->get_publisher_count();
      sub_map.reset();
      subscribe_map();
      RCLCPP_WARN(get_logger(), "%s 발행자 %zu 인데 못 받음 → 구독 다시 만듦 (%d 번째)", map_topic.c_str(), pubs,
        map_watch.count());
    }
  }

  // ---- lifecycle_manager 프로세스 재시작 (recover_only) ----
  static bool read_file(const std::string & path, std::string & out)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      return false;
    }
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
  }

  // 같은 사용자 · 같은 ROS_DOMAIN_ID 에서 "__node:=<node>" 로 뜬 프로세스 (launch_ros Node). 다른 도메인 (다른 사람 실행) 은 안 건드림
  std::vector<pid_t> find_node_pids(const std::string & node) const
  {
    std::vector<pid_t> out;
    DIR * d = opendir("/proc");
    if (!d) {
      return out;
    }
    const pid_t self = getpid();
    const uid_t uid = getuid();
    std::string cmdline, environ;
    while (const dirent * e = readdir(d)) {
      char * end = nullptr;
      const long v = std::strtol(e->d_name, &end, 10);
      if (!end || *end != '\0' || v <= 0 || static_cast<pid_t>(v) == self) {
        continue;
      }
      const std::string dir = std::string("/proc/") + e->d_name;
      struct stat st {};
      if (stat(dir.c_str(), &st) != 0 || st.st_uid != uid) {
        continue;
      }
      if (!read_file(dir + "/cmdline", cmdline) || !lifecycle::cmdline_has_node(cmdline, node)) {
        continue;
      }
      if (!read_file(dir + "/environ", environ) || lifecycle::environ_domain(environ) != own_domain) {
        continue;
      }
      out.push_back(static_cast<pid_t>(v));
    }
    closedir(d);
    return out;
  }

  void restart_process(Group & g, const lifecycle::Request & request)
  {
    const std::string node = strip_slash(request.target);
    const auto pids = find_node_pids(node);
    if (pids.empty()) {
      g.core.complete(request.token, false, monotonic(),
        node + " 프로세스 못 찾음 (같은 사용자 · ROS_DOMAIN_ID=" + own_domain + ")");
      RCLCPP_ERROR(get_logger(), "%s: %s 프로세스를 못 찾아 다시 못 띄움", g.key.c_str(), node.c_str());
      return;
    }
    std::string list;
    const double now = monotonic();
    for (const pid_t pid : pids) {
      if (::kill(pid, SIGTERM) == 0) {
        kill_later.emplace_back(pid, now + restart_kill_s);
      }
      list += (list.empty() ? "" : ",") + std::to_string(pid);
    }
    RCLCPP_WARN(get_logger(), "%s: %s 멈춤 → 프로세스 끝냄 (pid %s) → launch respawn 이 다시 띄우고 autostart (재시작 %lld/%lld): %s",
      g.key.c_str(), node.c_str(), list.c_str(), static_cast<long long>(g.core.restarts),
      static_cast<long long>(g.core.opt.max_restarts), g.core.last_error.c_str());
    g.core.complete(request.token, true, now);
  }

  // SIGTERM 뒤 restart_kill_s 지나도 남은 프로세스는 SIGKILL (멈춘 스레드가 종료를 막을 때)
  void reap(double now)
  {
    for (auto it = kill_later.begin(); it != kill_later.end(); ) {
      if (now < it->second) {
        ++it;
        continue;
      }
      if (::kill(it->first, 0) == 0) {
        ::kill(it->first, SIGKILL);
        RCLCPP_WARN(get_logger(), "pid %d 가 SIGTERM 뒤 %.0f s 남음 → SIGKILL", static_cast<int>(it->first), restart_kill_s);
      }
      it = kill_later.erase(it);
    }
  }

  ~StartupLifecycleWatchdog() override
  {
    if (timer) {
      timer->cancel();
    }
    for (const auto & p : polls) {
      remove_poll_request(p);
    }
    polls.clear();
    for (auto & g : groups) {
      if (g->op) {
        remove_op_request(*g, *g->op);
        g->op.reset();
      }
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    scan_watch.seen(monotonic());
    // 첫 유효 거리에서 멈춤 (파이썬 any() 와 같음)
    for (const float x : msg.ranges) {
      if (std::isfinite(x) && msg.range_min <= x && x <= msg.range_max) {
        scan_received = monotonic();
        return;
      }
    }
  }

  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    map_watch.seen(monotonic());
    have_map = msg.info.width > 0 && msg.info.height > 0 && msg.info.resolution > 0 &&
      msg.data.size() == static_cast<std::size_t>(msg.info.width) * msg.info.height;
  }

  bool fresh_tf(const std::string & target, const std::string & source)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer.lookupTransform(target, source, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }
    const double now = static_cast<double>(get_clock()->now().nanoseconds()) / 1e9;
    const double stamp = static_cast<double>(tf.header.stamp.sec) + static_cast<double>(tf.header.stamp.nanosec) / 1e9;
    return now > 0 && stamp > 0 && -2.0 <= now - stamp && now - stamp <= data_timeout;
  }

  void remove_poll_request(const Poll & p)
  {
    auto & g = *groups[p.group];
    if (p.active) {
      g.active->remove_pending_request(p.request_id);
    } else {
      g.state.at(p.node)->remove_pending_request(p.request_id);
    }
  }

  void remove_op_request(Group & g, const Group::Op & op)
  {
    if (op.manager) {
      g.manager->remove_pending_request(op.request_id);
    } else {
      g.change.at(op.target)->remove_pending_request(op.request_id);
    }
  }

  std::string poll_key(std::size_t group, bool active, const std::string & node) const
  {
    return std::to_string(group) + (active ? std::string("|active") : "|state|" + node);
  }

  void poll_states(std::size_t gi, double now)
  {
    auto & g = *groups[gi];
    const auto & r = g.core.pending;
    // (active, node) 목록
    keys_buf.clear();
    if (r) {
      if (!r->timed_out_at) {
        return;  // manager callback에 상태 조회를 줄줄이 쌓지 않음.
      }
      if (r->kind == "manager") {
        keys_buf.emplace_back(true, std::string());
      } else {
        keys_buf.emplace_back(false, r->target);
      }
    } else {
      keys_buf.emplace_back(true, std::string());
      for (const auto & n : g.core.nodes) {
        keys_buf.emplace_back(false, n);
      }
    }
    for (const auto & [is_active, node] : keys_buf) {
      bool polling = false;
      for (const auto & p : polls) {
        if (p.group == gi && p.active == is_active && (is_active || p.node == node)) {
          polling = true;
          break;
        }
      }
      const std::string key = poll_key(gi, is_active, node);
      const auto lp = last_poll.find(key);
      if (polling || now - (lp == last_poll.end() ? -1e10 : lp->second) < poll_s) {
        continue;
      }
      const bool service_ready = is_active ? g.active->service_is_ready() : g.state.at(node)->service_is_ready();
      if (!service_ready) {
        continue;
      }
      try {
        Poll p;
        p.group = gi;
        p.active = is_active;
        p.node = node;
        p.sent = now;
        p.barrier = r.has_value();
        if (is_active) {
          auto res = g.active->async_send_request(std::make_shared<Trigger::Request>());
          p.request_id = res.request_id;
          p.fa = res.future.share();
        } else {
          auto res = g.state.at(node)->async_send_request(std::make_shared<GetState::Request>());
          p.request_id = res.request_id;
          p.fs = res.future.share();
        }
        polls.push_back(std::move(p));
        last_poll[key] = now;
      } catch (const std::exception & exc) {
        g.core.last_error = std::string("상태 조회 요청 실패: ") + exc.what();
      }
    }
  }

  void receive(double now)
  {
    for (auto & gp : groups) {
      auto & g = *gp;
      if (!g.op) {
        continue;
      }
      const auto & op = *g.op;
      const bool is_done = op.manager ? done(op.fm) : done(op.fc);
      if (is_done) {
        bool success = false;
        std::string detail;
        try {
          if (op.manager) {
            const auto resp = op.fm.get();
            success = resp != nullptr && resp->success;
          } else {
            const auto resp = op.fc.get();
            success = resp != nullptr && resp->success;
          }
          detail = success ? "" : "lifecycle 전환 응답 false";
        } catch (const std::exception & exc) {
          success = false;
          detail = exc.what();
        }
        g.core.complete(op.token, success, now, detail);
        g.op.reset();
      } else if (!g.core.pending || g.core.pending->token != op.token) {
        // timeout 뒤 상태 조회 장벽으로 끝난 이전 요청은 결과를 재적용하지 않음.
        remove_op_request(g, op);
        g.op.reset();
      }
    }
    for (auto it = polls.begin(); it != polls.end(); ) {
      const bool is_done = it->active ? done(it->fa) : done(it->fs);
      if (is_done) {
        auto & g = *groups[it->group];
        try {
          if (it->active) {
            const auto resp = it->fa.get();
            if (resp) {
              g.snap.active = lifecycle::Observation{resp->success ? 1 : 0, it->sent, now};
            }
          } else {
            const auto resp = it->fs.get();
            if (resp) {
              g.snap.states[it->node] = lifecycle::Observation{static_cast<int>(resp->current_state.id), it->sent, now};
            }
          }
        } catch (const std::exception & exc) {
          g.core.last_error = std::string("상태 조회 응답 실패: ") + exc.what();
        }
        it = polls.erase(it);
      } else if ((!it->barrier || groups[it->group]->core.opt.recover_only) && now - it->sent > probe_timeout) {
        // recover_only: 장벽 조회도 probe_timeout 뒤 다시 보냄 (조회 하나가 유실돼도 멀쩡한 manager 를 멈춤으로 보지 않게.
        //   다시 보낸 조회도 timeout 뒤 요청이라 장벽 조건은 같음)
        remove_poll_request(*it);
        it = polls.erase(it);
      } else {
        // timeout 장벽 조회는 응답/전체 drain기한까지 하나만 유지함.
        ++it;
      }
    }
  }

  void send(Group & g, const lifecycle::Request & request)
  {
    if (request.kind == "process") {
      restart_process(g, request);
      return;
    }
    try {
      Group::Op op;
      op.token = request.token;
      op.manager = request.kind == "manager";
      op.target = request.target;
      if (op.manager) {
        auto msg = std::make_shared<Manage::Request>();
        msg->command = static_cast<uint8_t>(request.command);
        auto res = g.manager->async_send_request(msg);
        op.request_id = res.request_id;
        op.fm = res.future.share();
      } else {
        auto msg = std::make_shared<ChangeState::Request>();
        msg->transition.id = static_cast<uint8_t>(request.command);
        auto res = g.change.at(request.target)->async_send_request(msg);
        op.request_id = res.request_id;
        op.fc = res.future.share();
      }
      g.op = std::move(op);
      RCLCPP_INFO(get_logger(), "%s: %s 대상=%s 명령=%d 복구=%lld/%lld", g.key.c_str(), request.purpose.c_str(),
        request.target.c_str(), request.command, static_cast<long long>(g.core.recoveries),
        static_cast<long long>(g.core.opt.max_recoveries));
    } catch (const std::exception & exc) {
      g.core.complete(request.token, false, monotonic(), exc.what());
    }
  }

  bool services_ready(const Group & g) const
  {
    if (!g.manager->service_is_ready() || !g.active->service_is_ready()) {
      return false;
    }
    for (const auto & [name, c] : g.state) {
      if (!c->service_is_ready() || !g.change.at(name)->service_is_ready()) {
        return false;
      }
    }
    return true;
  }

  void tick()
  {
    const double now = monotonic();
    receive(now);
    reap(now);
    heal_subscriptions(now);
    const bool scan_ok = scan_received && now - *scan_received <= data_timeout;
    const bool odom_ok = fresh_tf(odom_frame, base_frame);
    const bool map_tf_ok = fresh_tf(map_frame, odom_frame);
    const bool local_ready = !manage_localization || groups.front()->core.phase == "ready";
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
      auto & g = *groups[gi];
      auto & s = g.snap;
      std::string & reason = s.input_reason;
      reason.assign("기다림: ");
      const std::size_t empty_len = reason.size();
      auto need = [&reason, empty_len](const std::string & a, const char * b = "") {
          if (reason.size() > empty_len) {
            reason += ", ";
          }
          reason += a;
          reason += b;
        };
      if (!scan_ok) {
        need(scan_topic);
      }
      if (!odom_ok) {
        need(odom_frame + "→" + base_frame, " TF");
      }
      if (g.key == "navigation") {
        if (!local_ready) {
          need("localization ACTIVE 안정");
        }
        if (!have_map) {
          need(map_topic);
        }
        if (!map_tf_ok) {
          need(map_frame + "→" + odom_frame, " TF");
        }
      }
      s.inputs_ready = reason.size() == empty_len;
      if (s.inputs_ready) {
        reason.assign("입력 준비됨");
      }
      s.services_ready = services_ready(g);
      s.manager_ready = g.manager->service_is_ready() && g.active->service_is_ready();
      if (g.key == "localization") {
        s.output_ok = map_tf_ok;
        s.output_reason = map_frame + "→" + odom_frame + " TF (AMCL)";
      }
      if (!finished) {
        g.core.recheck_before_handoff(now, s);
      }
      const auto request = g.core.tick(now, s);
      if (request) {
        send(g, *request);
      }
      poll_states(gi, now);
    }
    bool ready = true;
    for (const auto & g : groups) {
      ready = ready && g->core.phase == "ready" && g->core.healthy(g->snap, now) && g->snap.inputs_ready;
    }
    finished = finished || ready;
    lifecycle::WatchdogStatus st;
    st.ready = ready;
    st.finished = finished;
    const Group & nav = *groups.back();
    st.localization_ready = !manage_localization ||
      (groups.front()->core.phase == "ready" && groups.front()->core.healthy(groups.front()->snap, now));
    st.navigation_ready = nav.core.phase == "ready" && nav.core.healthy(nav.snap, now);
    st.scan = scan_ok;
    st.odom_tf = odom_ok;
    st.map = have_map;
    st.map_odom_tf = map_tf_ok;
    for (const auto & g : groups) {
      st.groups.emplace_back(g->key, &g->core);
    }
    lifecycle::write_watchdog_status(encoded, st);
    const bool changed = !last_status || encoded != *last_status;
    if (changed || now - last_publish >= 1.0) {
      std_msgs::msg::String m;
      m.data = encoded;
      status_pub->publish(m);
      last_publish = now;
    }
    if (changed) {
      RCLCPP_INFO(get_logger(), "lifecycle 시작 상태: %s", encoded.c_str());
      if (!last_status) {
        last_status.emplace();
      }
      last_status->swap(encoded);
    }
  }

  // ---- 상태 (파이썬 속성과 같은 이름) ----
  bool manage_localization = true;
  std::string scan_topic, map_topic, base_frame, odom_frame, map_frame;
  double data_timeout = 3.0, poll_s = 0.5, probe_timeout = 5.0;
  std::vector<std::unique_ptr<Group>> groups;   // localization (있으면) → navigation 순
  std::vector<Poll> polls;
  std::map<std::string, double> last_poll;
  std::optional<double> scan_received;
  bool have_map = false;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  std::optional<std::string> last_status;
  double last_publish = -1.0;
  bool finished = false;
  ResubscribeWatch scan_watch, map_watch;
  std::string own_domain = "0";
  double restart_kill_s = 3.0;
  std::vector<std::pair<pid_t, double>> kill_later;   // SIGTERM 보낸 pid · SIGKILL 시각

private:
  std::string encoded;
  std::vector<std::pair<bool, std::string>> keys_buf;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace navigation
