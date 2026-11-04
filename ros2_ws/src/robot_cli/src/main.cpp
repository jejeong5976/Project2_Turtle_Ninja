// main.cpp — robot_cli: 상태 화면 (인자 없음) 또는 한 줄 명령 (스크립트용). 실행기 nt = tools/robot_cli/robot_cli.sh
//
//   nt                         상태 화면 + 입력줄 (= ros2 run robot_cli robot_cli)
//   nt status [쪽] [--watch]   상태 한 번 (쪽 0~7, --watch = 1 s 마다 다시)
//   nt check                   시작 전 점검 (실패 있으면 끝 코드 1)
//   nt h · nt s5 --timeout 60  팔 작업 → 받기 · 결과까지 기다림 (짧은 별칭은 command.cpp kHelp)
//   nt g B1 [--no-wait]        접고 이동: 팔이 받침에 안 접혀 있으면 home 먼저 → nav
//   nt nav B1 · nt drive 0.05 0 0 2 · nt cancel | estop | stop | call /mission/start | stops | loc | teach
//   nt mark S5_tag [--mark-dir <폴더>] · nt note <메모>   기록 (팔은 안 움직임)
//   옵션: --hardware-map <yaml> --dice-map <yaml> --rack-cells <yaml> --mark-dir <폴더> --timeout <s> --no-wait --watch
//   팔 자세 이름 · 접힘 기준 = bringup share config 의 control.yaml · arm_paths.yaml, 주행 한계 = controllers.yaml
//   끝 코드 (bee.py 와 같음): 0 성공 · 1 응답 없음 · 시간 초과 · Ctrl-C · 2 거절 · 실패 · 문법 오류
//   Ctrl-C = 보내기 전 (발견 · 서버 기다리는 중) 이면 아무것도 안 보냄, 보낸 뒤면 이 CLI 의 goal 취소
//            (받음 답이 1 s 안에 안 오면 그 서버의 모든 goal 취소) · 바퀴 0

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_cli/command.hpp"
#include "robot_cli/robot_link.hpp"
#include "robot_cli/status_view.hpp"
#include "robot_cli/teach.hpp"
#include "robot_cli/tui.hpp"

#ifndef ROBOT_CLI_MAPS_DIR
#define ROBOT_CLI_MAPS_DIR "maps"
#endif

namespace
{
using namespace robot_cli;
using namespace std::chrono_literals;

std::atomic<bool> g_interrupted{false};
void on_signal(int) {g_interrupted = true;}

struct Options
{
  std::string hardware_map, dice_map, rack_cells, mark_dir;
  double timeout{-1.0};
  bool no_wait{false}, watch{false};
  std::vector<std::string> words;
};

std::string bringup_config(const std::string & file)
{
  try {
    return (ament_index_cpp::get_package_share_path("bringup") / "config" / file).string();
  } catch (const std::exception &) {
    return "";
  }
}

bool parse_options(const std::vector<std::string> & args, Options & o, std::string & err)
{
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "--no-wait") {o.no_wait = true; continue;}
    if (a == "--watch") {o.watch = true; continue;}
    if (a != "--hardware-map" && a != "--dice-map" && a != "--rack-cells" && a != "--mark-dir" && a != "--timeout") {
      o.words.push_back(a);
      continue;
    }
    if (i + 1 >= args.size()) {err = a + " 값 없음"; return false;}
    const std::string v = args[++i];
    if (a == "--hardware-map") {
      o.hardware_map = v;
    } else if (a == "--dice-map") {
      o.dice_map = v;
    } else if (a == "--rack-cells") {
      o.rack_cells = v;
    } else if (a == "--mark-dir") {
      o.mark_dir = v;
    } else {
      try {o.timeout = std::stod(v);} catch (const std::exception &) {err = "--timeout 숫자"; return false;}
    }
  }
  if (o.hardware_map.empty()) {o.hardware_map = bringup_config("hardware_map.yaml");}
  if (o.rack_cells.empty()) {o.rack_cells = bringup_config("rack_cells.yaml");}
  if (o.dice_map.empty()) {o.dice_map = std::string(ROBOT_CLI_MAPS_DIR) + "/dice_map.yaml";}
  if (o.mark_dir.empty()) {   // 레포 안 (실행 위치) 에 쓰지 않게 홈 아래
    const char * home = std::getenv("HOME");
    o.mark_dir = std::string(home ? home : ".") + "/robot_cli_marks";
  }
  std::error_code ec;
  const auto abs = std::filesystem::absolute(o.mark_dir, ec);   // 화면에 어디인지 그대로 보이게
  if (!ec) {o.mark_dir = abs.lexically_normal().string();}
  return true;
}

Config load_config(const Options & o, std::vector<std::string> & warnings)
{
  Config cfg;
  std::string err;
  if (!load_hardware_map(o.hardware_map, cfg.cal, err)) {warnings.push_back("경고: 눈금 변환 없음 — " + err);}
  if (!load_stops(o.dice_map, cfg.stops, err)) {warnings.push_back("경고: 정차 자세 없음 — " + err);}
  if (!load_slots(o.rack_cells, cfg.slots, err)) {
    warnings.push_back("경고: 칸 목록 없음 — " + err + " (S1~S12 로 둠)");
    for (int i = 1; i <= 12; ++i) {cfg.slots.push_back("S" + std::to_string(i));}
  }
  if (!load_arm_ref(bringup_config("control.yaml"), bringup_config("arm_paths.yaml"), cfg.arm, err)) {
    warnings.push_back("경고: 팔 자세 이름 · 접힘 기준 없음 — " + err);
  }
  load_battery_limits(o.hardware_map, cfg.battery_warn, cfg.battery_stop);
  if (!load_drive_limits(bringup_config("controllers.yaml"), cfg.max_linear, cfg.max_angular)) {
    warnings.push_back("경고: controllers.yaml 주행 한계 못 읽음 — drive 한계 0.20 m/s · 0.5 rad/s 로 둠");
  }
  return cfg;
}

void print_lines(const std::vector<Line> & lines, bool color)
{
  for (const auto & l : lines) {
    const char * c = "";
    switch (l.level) {
      case Level::HEAD: c = "\033[1;36m"; break;
      case Level::OFF: c = "\033[90m"; break;
      case Level::GOOD: c = "\033[32m"; break;
      case Level::WARN: c = "\033[33m"; break;
      case Level::BAD: c = "\033[31m"; break;
      default: break;
    }
    if (color && *c && l.mark_len > 0 && l.mark_from + l.mark_len <= l.text.size()) {   // 배지 줄: 배지만 색
      std::printf("%s%s%s\033[0m%s\n", l.text.substr(0, l.mark_from).c_str(), c, l.text.substr(l.mark_from, l.mark_len).c_str(),
        l.text.substr(l.mark_from + l.mark_len).c_str());
      continue;
    }
    std::printf("%s%s%s\n", color ? c : "", l.text.c_str(), color && *c ? "\033[0m" : "");
  }
  std::fflush(stdout);
}

void flush_log(RobotLink & link)
{
  for (const auto & s : link.take_log()) {std::printf("%s\n", s.c_str());}
  std::fflush(stdout);
}

// 끝나거나 시간이 다 되거나 Ctrl-C 까지 기다리며 기록을 찍음. 끝났으면 true
bool wait_until(RobotLink & link, const std::atomic<bool> & done, double secs)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
  while (!done.load() && !g_interrupted.load() && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(100ms);
    flush_log(link);
  }
  flush_log(link);
  return done.load();
}

// 발견 · 첫 메시지 · 1 Hz 묻기를 기다림. Ctrl-C 면 false (부른 쪽은 아무것도 보내지 말고 끝낼 것)
bool settle(RobotLink & link, double secs)
{
  std::atomic<bool> never{false};
  wait_until(link, never, secs);
  return !g_interrupted.load();
}

// Ctrl-C: 이 CLI 의 goal 취소 (아직 받음 답이 없으면 받는 대로, 1 s 넘으면 서버의 모든 goal) → 취소 요청이 나가게 잠깐
int interrupted_exit(RobotLink & link)
{
  std::printf("Ctrl-C → 이 CLI 가 보낸 goal 취소\n");
  const std::string r = link.finish(1.0);
  if (!r.empty()) {std::printf("%s\n", r.c_str());}
  std::this_thread::sleep_for(500ms);
  flush_log(link);
  return 1;
}

// 서버를 발견하는 대로 바로 모든 goal 취소 (없는 서버는 max_s 뒤 포기). 취소한 서버 수
int cancel_as_found(RobotLink & link, bool arm, bool nav, double max_s)
{
  bool arm_sent = !arm, nav_sent = !nav;
  int n = 0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(max_s);
  while ((!arm_sent || !nav_sent) && !g_interrupted.load() && std::chrono::steady_clock::now() < end) {
    if (!arm_sent && link.servers_ready(true, false)) {link.cancel("arm"); arm_sent = true; ++n;}
    if (!nav_sent && link.servers_ready(false, true)) {link.cancel("nav"); nav_sent = true; ++n;}
    std::this_thread::sleep_for(20ms);
  }
  return n;
}

// 팔 · 주행 goal 하나 보내고 결과까지 (Ctrl-C = 이 CLI 의 goal 취소). 끝 코드 0 · 1 · 2
int send_and_wait(RobotLink & link, const Command & c, const Options & o, bool arm)
{
  struct State
  {
    std::atomic<bool> done{false}, accepted{false};
    std::atomic<int> code{1};
  };
  const auto st = std::make_shared<State>();   // 콜백이 이 함수가 끝난 뒤에 와도 되게
  auto on_done = [st](int k, const std::string &) {st->code = k; st->done = true;};
  auto on_accept = [st]() {st->accepted = true;};
  if (arm) {link.send_arm(c, on_done, on_accept);} else {link.send_nav(c, on_done, on_accept);}
  if (o.no_wait) {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (!st->accepted.load() && !st->done.load() && !g_interrupted.load() && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(50ms);
    }
    if (g_interrupted.load()) {return interrupted_exit(link);}   // 받음과 Ctrl-C 가 겹쳐도 취소가 먼저
    flush_log(link);
    return st->accepted.load() ? 0 : st->done.load() ? st->code.load() : 1;
  }
  const double limit = o.timeout > 0.0 ? o.timeout : arm ? 120.0 : 300.0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(limit);
  auto last_fb = std::chrono::steady_clock::now();
  while (!st->done.load() && !g_interrupted.load() && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(100ms);
    flush_log(link);
    if (!arm && st->accepted.load() && std::chrono::steady_clock::now() - last_fb > 2s) {
      last_fb = std::chrono::steady_clock::now();
      const Snapshot s = link.snapshot();
      if (s.nav_fb_t >= 0.0 && s.now - s.nav_fb_t < 2.0) {
        std::printf("  남은 %.2f m · ETA %.0f s · 복구 %d\n", s.dist_left, s.eta_s, s.recoveries);
      }
    }
  }
  if (g_interrupted.load()) {return interrupted_exit(link);}
  if (st->done.load()) {return st->code.load();}
  std::printf("결과 없음: %.0f s 초과 (nt cancel 로 중단 가능)\n", limit);
  return 1;
}

int run_once(RobotLink & link, const Command & c, const Options & o)
{
  // 서비스 호출 결과 — 콜백이 이 함수가 끝난 뒤에 와도 되게 (시간 초과 · Ctrl-C) 상태는 shared_ptr 로
  struct State
  {
    std::atomic<bool> done{false};
    std::atomic<int> code{1};
  };
  const auto st = std::make_shared<State>();
  auto & done = st->done;
  auto & code = st->code;
  auto on_done = [st](int k, const std::string &) {st->code = k; st->done = true;};

  switch (c.cmd) {
    case Cmd::HELP: std::printf("%s\n", kHelp); return 0;
    case Cmd::QUIT: return 0;
    case Cmd::STOPS: for (const auto & s : stops_text(link.config().stops)) {std::printf("%s\n", s.c_str());} return 0;
    case Cmd::TELEOP: std::printf("teleop 은 화면에서만 (nt 를 인자 없이 실행)\n"); return 2;
    case Cmd::TEACH: std::printf("%s\n", kTeachNote); return 0;
    case Cmd::MARKDIR: std::printf("markdir 는 화면에서만 — 명령 하나는 --mark-dir <폴더>\n"); return 2;
    case Cmd::CYCLE: std::printf("cycle 은 화면에서만 (nt → 7 쪽)\n"); return 2;
    case Cmd::PAGE:
    case Cmd::STATUS: {
      if (!settle(link, 2.0)) {return 1;}   // 1 Hz 묻기 (controller_manager · Nav2) 가 한 번은 답하게
      for (int i = 0; i < 40 && link.missing_data() && !g_interrupted.load(); ++i) {settle(link, 0.1);}   // 발견이 늦으면 6 s 까지
      const bool color = isatty(STDOUT_FILENO);
      UiState ui;
      ui.mark_dir = o.mark_dir;
      do {
        if (o.watch && color) {std::printf("\033[H\033[2J");}
        if (c.page != 0) {std::printf("[%d %s]\n", c.page, kPageNames[c.page]);}
        print_lines(page_lines(c.page, link.snapshot(), link.config(), ui), color);
        if (o.watch) {settle(link, 1.0);}
      } while (o.watch && !g_interrupted.load());
      return 0;
    }
    case Cmd::MARK: {
      if (!settle(link, 1.0)) {return 1;}
      for (int i = 0; i < 20 && link.snapshot().joints_t < 0.0 && !g_interrupted.load(); ++i) {settle(link, 0.1);}
      if (g_interrupted.load()) {return 1;}
      std::string text;
      const bool ok = write_mark(o.mark_dir, c.name, link.snapshot(), link.config(), text);
      std::printf("%s%s\n", text.c_str(), ok ? (" → " + o.mark_dir).c_str() : "");
      return ok ? 0 : link.snapshot().joints_t < 0.0 ? 1 : 2;
    }
    case Cmd::LOC: {
      if (!settle(link, 2.0)) {return 1;}
      for (int i = 0; i < 40 && link.missing_data() && !g_interrupted.load(); ++i) {settle(link, 0.1);}
      for (const auto & l : status_lines(link.snapshot(), link.config())) {
        if (l.text.rfind("위치추정", 0) == 0 || l.text.rfind("map ", 0) == 0 || l.text.rfind("Nav2", 0) == 0) {
          std::printf("%s\n", l.text.c_str());
        }
      }
      for (const auto & s : loc_hint()) {std::printf("%s\n", s.c_str());}
      return 0;
    }
    case Cmd::ARM:
    case Cmd::NAV:
      if (!settle(link, 1.0)) {return 1;}   // Ctrl-C 가 settle 중에 오면 아무것도 안 보냄 (서버 기다리는 중도 RobotLink 가 봄)
      return send_and_wait(link, c, o, c.cmd == Cmd::ARM);
    case Cmd::GO: {   // 접고 이동: 접혀 있지 않으면 home 먼저 → nav
      if (!settle(link, 1.5)) {return 1;}
      const Snapshot s = link.snapshot();
      const std::string why = go_block_reason(s, link.config());
      if (!why.empty()) {std::printf("이동 거절: %s\n", why.c_str()); return 2;}
      Command nav = c;
      nav.cmd = Cmd::NAV;
      if (go_needs_fold(s, link.config())) {
        std::printf("접고 이동: 팔이 받침에 안 접힘 → home 먼저\n");
        Command home;
        home.cmd = Cmd::ARM;
        Options ho = o;
        ho.no_wait = false;
        ho.timeout = 60.0;
        const int rc = send_and_wait(link, home, ho, true);
        if (rc != 0) {std::printf("접고 이동 멈춤: home 실패 — 이동 안 함\n"); return rc;}
        std::printf("접고 이동: 접음 → 이동\n");
      }
      return send_and_wait(link, nav, o, false);
    }
    case Cmd::CHECK: {
      if (!settle(link, 2.0)) {return 1;}
      for (int i = 0; i < 40 && link.missing_data() && !g_interrupted.load(); ++i) {settle(link, 0.1);}
      const char * d = std::getenv("ROS_DOMAIN_ID");
      const auto items = preflight(link.snapshot(), link.config(), d ? d : "");
      print_lines(check_lines(items), isatty(STDOUT_FILENO));
      return preflight_ok(items) ? 0 : 1;
    }
    case Cmd::NOTE: {
      std::error_code ec;
      std::filesystem::create_directories(o.mark_dir, ec);
      std::string e;
      const bool ok = append_csv(o.mark_dir + "/notes.csv", "time,session,note", {wall_clock_text() + ",," + csv_clean(c.name)}, e);
      std::printf("%s\n", ok ? ("메모 → " + o.mark_dir + "/notes.csv").c_str() : e.c_str());
      return ok ? 0 : 2;
    }
    case Cmd::BAG: std::printf("bag 은 화면에서만 (nt → 7 쪽 · bag start)\n"); return 2;
    case Cmd::CANCEL: {
      if (cancel_as_found(link, c.which != "nav", c.which != "arm", 3.0) == 0) {std::printf("취소할 서버 없음\n"); return 1;}
      settle(link, 1.0);   // 취소 답 찍기
      return 0;
    }
    case Cmd::ESTOP:
      if (!c.on) {std::printf("명령 하나로 쓴 estop 은 잠금이 없음 — 풀 것이 없음 (잠금은 화면에서만)\n"); return 0;}
      link.set_estop(true);              // 잠금 + /cmd_vel 0 (20 Hz 타이머) — 서버 취소는 아래에서 발견하는 대로
      cancel_as_found(link, true, true, 3.0);
      settle(link, 1.0);                 // 0 을 더 보내고 취소 답 찍기. 끝나면 잠금도 끝 (이 프로세스만의 잠금)
      return 0;
    case Cmd::DRIVE: {
      if (!settle(link, 1.5)) {return 1;}   // Ctrl-C 뒤에는 속도를 한 틱도 안 보냄
      const std::string why = link.drive(c.vx, c.vy, c.wz, c.secs, c.force);
      if (!why.empty()) {std::printf("drive 거절: %s\n", why.c_str()); return 2;}
      std::printf("drive vx %+.2f vy %+.2f wz %+.2f · %.1f s\n", c.vx, c.vy, c.wz, c.secs);
      while (link.drive_busy() && !g_interrupted.load()) {std::this_thread::sleep_for(50ms);}
      const bool interrupted = g_interrupted.load();
      if (interrupted) {std::printf("Ctrl-C → 바퀴 0\n");}
      link.stop_drive();
      while (link.drive_busy()) {std::this_thread::sleep_for(50ms);}
      flush_log(link);
      std::printf("바퀴 0\n");
      const std::string lost = link.take_drive_lost();
      if (interrupted) {return 1;}
      if (!lost.empty()) {std::printf("drive 중단 (주행 조건을 잃음): %s\n", lost.c_str()); return 2;}
      return 0;
    }
    case Cmd::STOP:
      settle(link, 1.0);
      link.stop_drive();
      while (link.drive_busy()) {std::this_thread::sleep_for(50ms);}
      std::printf("바퀴 0 (/cmd_vel)\n");
      return 0;
    case Cmd::CALL:
      if (!settle(link, 1.0)) {return 1;}   // Ctrl-C 뒤에 mission start 같은 호출이 나가지 않게
      link.call_service(c.service, c.setbool, on_done);
      if (!wait_until(link, done, o.timeout > 0.0 ? o.timeout : 10.0)) {std::printf("응답 없음\n"); return 1;}
      return code.load();
  }
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  args.erase(args.begin());

  Options o;
  std::string err;
  if (!parse_options(args, o, err)) {
    std::fprintf(stderr, "%s\n%s\n", err.c_str(), kHelp);
    rclcpp::shutdown();
    return 2;
  }
  const bool tui = o.words.empty() || (o.words.size() == 1 && o.words[0] == "tui");
  std::vector<std::string> warnings;
  Config cfg = load_config(o, warnings);
  Names names{{}, cfg.slots, cfg.max_linear, cfg.max_angular};
  for (const auto & s : cfg.stops) {names.stops.push_back(s.name);}

  Command cmd;
  if (!tui && !parse_command(o.words, names, cmd, err)) {
    std::fprintf(stderr, "오류: %s  (문법: nt help)\n", err.c_str());
    rclcpp::shutdown();
    return 2;
  }
  if (!tui) {
    for (const auto & w : warnings) {std::fprintf(stderr, "%s\n", w.c_str());}
  }

  auto link = std::make_shared<RobotLink>(std::move(cfg));
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(link);
  std::thread spin([&exec]() {exec.spin();});
  link->set_discovery_wait(tui ? 0.0 : 3.0);
  link->set_interrupt(&g_interrupted);   // 서버 · 서비스를 기다리다 Ctrl-C 면 보내지 않음

  const int rc = tui ? run_tui(*link, g_interrupted, warnings, o.mark_dir) : run_once(*link, cmd, o);
  if (!tui) {flush_log(*link);}

  exec.cancel();
  rclcpp::shutdown();   // spin() 이 아직 안 들어갔어도 ok() = false 로 바로 끝나게 (cancel 만으로는 경합)
  spin.join();
  return rc;
}
