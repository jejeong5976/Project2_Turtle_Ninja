// tui.cpp — ncurses 상태 화면, 설명은 tui.hpp
#define NCURSES_WIDECHAR 1

#include "robot_cli/tui.hpp"

#include <langinfo.h>
#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <clocale>
#include <cstdlib>
#include <ctime>
#include <cwchar>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>

#include "robot_cli/bag.hpp"
#include "robot_cli/status_view.hpp"
#include "robot_cli/teach.hpp"

namespace robot_cli
{

namespace
{
enum Pair {P_HEAD = 1, P_OK, P_WARN, P_ERROR, P_TITLE, P_SEL, P_OFF};
constexpr int kHelpPage = -1, kCheckPage = -2;
constexpr int kLogRows = 6;

std::wstring widen(const std::string & s)
{
  std::wstring out;
  std::mbstate_t st{};
  const char * p = s.data();
  const char * end = p + s.size();
  while (p < end) {
    wchar_t w;
    const std::size_t n = std::mbrtowc(&w, p, end - p, &st);
    if (n == static_cast<std::size_t>(-1) || n == static_cast<std::size_t>(-2)) {out += L'?'; ++p; st = {}; continue;}
    out += w;
    p += n == 0 ? 1 : n;
  }
  return out;
}

std::string narrow(const std::wstring & w)
{
  std::string out;
  std::mbstate_t st{};
  char buf[MB_LEN_MAX];
  for (wchar_t c : w) {
    const std::size_t n = std::wcrtomb(buf, c, &st);
    if (n != static_cast<std::size_t>(-1)) {out.append(buf, n);}
  }
  return out;
}

int width_of(const std::string & text)
{
  int used = 0;
  for (wchar_t c : widen(text)) {used += std::max(0, wcwidth(c));}
  return used;
}

// 화면 칸 수 (한글 = 2 칸) 기준으로 잘라 그림. 그린 칸 수를 돌려줌
int put(int y, int x, const std::string & text, int cols, attr_t attr = A_NORMAL, short pair = 0)
{
  if (cols <= 0) {return 0;}
  const std::wstring w = widen(text);
  int used = 0;
  std::size_t n = 0;
  for (; n < w.size(); ++n) {
    const int cw = std::max(0, wcwidth(w[n]));
    if (used + cw > cols) {break;}
    used += cw;
  }
  attr_set(attr, pair, nullptr);
  mvaddnwstr(y, x, w.c_str(), static_cast<int>(n));
  attr_set(A_NORMAL, 0, nullptr);
  return used;
}

std::string clock_text()
{
  const std::time_t t = std::time(nullptr);
  char b[16];
  std::strftime(b, sizeof(b), "%H:%M:%S", std::localtime(&t));
  return b;
}

short pair_of(Level l)
{
  switch (l) {
    case Level::HEAD: return P_HEAD;
    case Level::GOOD: return P_OK;
    case Level::WARN: return P_WARN;
    case Level::BAD: return P_ERROR;
    case Level::OFF: return P_OFF;
    default: return 0;
  }
}

attr_t attr_of(Level l) {return l == Level::HEAD || l == Level::BAD ? A_BOLD : A_NORMAL;}

// 쪽 글 한 줄: 배지 줄이면 배지만 색, 아니면 줄 전체 색
void put_line(int y, const Line & l, int W)
{
  if (l.mark_len == 0 || l.mark_from + l.mark_len > l.text.size()) {
    put(y, 0, l.text, W, attr_of(l.level), pair_of(l.level));
    return;
  }
  int x = put(y, 0, l.text.substr(0, l.mark_from), W);
  x += put(y, x, l.text.substr(l.mark_from, l.mark_len), W - x, A_BOLD, pair_of(l.level));
  put(y, x, l.text.substr(l.mark_from + l.mark_len), W - x);
}

// ANSI 색 (snap *.ansi): 화면 색쌍 → SGR
std::string sgr(short pair, attr_t attr)
{
  std::string s = "\033[0";
  if (attr & A_BOLD) {s += ";1";}
  switch (pair) {
    case P_HEAD: s += ";36"; break;
    case P_OK: s += ";32"; break;
    case P_WARN: s += ";33"; break;
    case P_ERROR: s += ";31"; break;
    case P_TITLE: s += ";30;46"; break;
    case P_SEL: s += ";30;43"; break;
    case P_OFF: s += ";90"; break;
    default: break;
  }
  return s + "m";
}

// 지금 화면 그대로 저장 (snap). *.ansi 면 색까지 (cat 으로 보면 색이 나옴)
bool dump_screen(const std::string & path)
{
  std::ofstream f(path);
  if (!f) {return false;}
  const bool ansi = path.size() > 5 && path.compare(path.size() - 5, 5, ".ansi") == 0;
  std::vector<wchar_t> buf(COLS + 1);
  for (int y = 0; y < LINES; ++y) {
    const int n = mvinnwstr(y, 0, buf.data(), COLS);
    std::wstring row = n > 0 ? std::wstring(buf.data(), n) : std::wstring();
    if (!ansi) {
      while (!row.empty() && row.back() == L' ') {row.pop_back();}
      f << narrow(row) << "\n";
      continue;
    }
    // 칸마다 색쌍 · 굵기 (넓은 글자는 첫 칸 것을 씀)
    std::vector<short> pairs(COLS, 0);
    std::vector<attr_t> attrs(COLS, A_NORMAL);
    for (int x = 0; x < COLS; ++x) {
      cchar_t cc;
      if (mvin_wch(y, x, &cc) == ERR) {continue;}
      wchar_t wch[CCHARW_MAX + 1];
      attr_t a = A_NORMAL;
      short p = 0;
      if (getcchar(&cc, wch, &a, &p, nullptr) != ERR) {pairs[x] = p; attrs[x] = a;}
    }
    std::string out, cur, pending;   // 기본 색 빈칸은 뒤에 글이 올 때만 씀 (줄 끝 빈칸 버림)
    int col = 0;
    for (wchar_t c : row) {
      if (col >= COLS) {break;}
      const std::string code = sgr(pairs[col], attrs[col] & A_BOLD);
      const bool blank = c == L' ' && pairs[col] == 0;
      std::string piece = (code != cur ? code : "") + narrow(std::wstring(1, c));
      if (code != cur) {cur = code;}
      if (blank) {
        pending += piece;
      } else {
        out += pending + piece;
        pending.clear();
      }
      col += std::max(1, wcwidth(c));
    }
    f << out << "\033[0m\n";
  }
  return true;
}

struct Teleop
{
  double lin{0.05}, ang{0.3};
};

// 팔 작업 결과 (실행기 스레드 → 화면 스레드). kind 0 = cycle, 1 = 접고 이동의 home. gen = 보낸 번호 (멈춘 뒤 늦은 결과 버림)
struct ArmDone
{
  int kind;
  unsigned gen;
  int code;
  std::string text;
};
struct ArmEvents
{
  std::mutex m;
  std::deque<ArmDone> done;
};

void log_lines(const std::string & text, const std::function<void(const std::string &)> & add)
{
  for (std::size_t a = 0, b; a < text.size(); a = b + 1) {
    b = text.find('\n', a);
    if (b == std::string::npos) {b = text.size();}
    add(text.substr(a, b - a));
  }
}

bool all_digits(const std::string & s)
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) {return c >= '0' && c <= '9';});
}

std::string common_prefix(const std::vector<std::string> & v)
{
  if (v.empty()) {return "";}
  std::string p = v[0];
  for (const auto & s : v) {
    std::size_t i = 0;
    while (i < p.size() && i < s.size() && std::tolower(static_cast<unsigned char>(p[i])) == std::tolower(static_cast<unsigned char>(s[i]))) {++i;}
    p.resize(i);
  }
  return p;
}
}  // namespace

int run_tui(RobotLink & link, const std::atomic<bool> & interrupted, const std::vector<std::string> & startup,
  const std::string & mark_dir)
{
  std::setlocale(LC_ALL, "");          // 한글 글자 폭
  const std::string codeset = nl_langinfo(CODESET);
  if (codeset != "UTF-8" && codeset != "utf8") {std::setlocale(LC_CTYPE, "C.UTF-8");}   // ssh 등 LANG 없음 → 한글이 ? 로 안 깨지게
  std::setlocale(LC_NUMERIC, "C");     // 소수점은 늘 '.' (기록 csv · yaml · 숫자 읽기가 지역 설정에 안 흔들리게)
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  set_escdelay(50);
  timeout(100);
  if (has_colors()) {
    start_color();
    use_default_colors();
    init_pair(P_HEAD, COLOR_CYAN, -1);
    init_pair(P_OK, COLOR_GREEN, -1);
    init_pair(P_WARN, COLOR_YELLOW, -1);
    init_pair(P_ERROR, COLOR_RED, -1);
    init_pair(P_TITLE, COLOR_BLACK, COLOR_CYAN);
    init_pair(P_SEL, COLOR_BLACK, COLOR_YELLOW);
    init_pair(P_OFF, COLORS >= 16 ? 8 : COLOR_WHITE, -1);   // 회색 (꺼짐 · 없음)
  }
  link.set_quiet(true);   // 기록 칸은 명령 · 결과 위주 (보냄 · phase 줄 빼기)

  const Config & cfg = link.config();
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  UiState ui;
  ui.mark_dir = mark_dir;

  // 세션 기록: 화면 기록 칸에 나온 모든 줄을 <mark 폴더>/sessions/session_NNN.log 에 (세션마다 한 파일)
  std::ofstream session;
  {
    std::error_code ec;
    const std::string dir = mark_dir + "/sessions";
    std::filesystem::create_directories(dir, ec);
    if (!ec) {
      ui.session_file = reserve_free(dir, "session", ".log");   // O_EXCL 로 이름을 차지 → 화면 두 개가 같은 파일을 안 씀
      if (!ui.session_file.empty()) {session.open(ui.session_file, std::ios::app);}
      if (session) {
        session << "# robot_cli 세션 시작 " << wall_clock_text() << " · ROS_DOMAIN_ID=" << (domain ? domain : "0") << "\n";
        session.flush();
      } else {
        ui.session_file.clear();
      }
    }
  }
  std::deque<std::string> logs;
  auto add_log = [&logs, &session](const std::string & s) {
      logs.push_back(clock_text() + " " + s);
      while (logs.size() > 300) {logs.pop_front();}
      if (session) {
        session << logs.back() << "\n";
        session.flush();
      }
    };
  for (const auto & s : startup) {add_log(s);}
  add_log("? = 도움말 · 0 쪽에서 번호 + Enter = 쪽 · Esc = 전체 요약 · q = 끝");
  if (ui.session_file.empty()) {add_log("주의: 세션 기록 파일을 못 만듦 (" + mark_dir + "/sessions)");}

  Names names{{}, cfg.slots, cfg.max_linear, cfg.max_angular};
  for (const auto & s : cfg.stops) {names.stops.push_back(s.name);}
  std::wstring input;
  std::vector<std::wstring> history;
  std::size_t hist_pos = 0;
  int page = 0;
  int scroll = 0;
  bool teleop = false;
  Teleop tp;
  tp.lin = std::min(tp.lin, cfg.max_linear);   // 설정 한계가 더 낮으면 그 안에서 시작
  tp.ang = std::min(tp.ang, cfg.max_angular);
  bool quit = false;
  auto noop = [](int, const std::string &) {};
  CycleRun cycle;
  BagRecorder bag;
  const auto events = std::make_shared<ArmEvents>();
  double cycle_sent_t = 0.0;
  unsigned cycle_gen = 0, go_gen = 0;
  bool go_active = false;
  Command go_nav;
  bool prompting = false;
  PageAction prompt;
  Snapshot snap;

  auto stop_cycle = [&](const std::string & why) {
      if (cycle.active()) {
        cycle.abort(why);
        add_log("cycle 멈춤 (" + why + ")");
      }
      if (go_active) {
        go_active = false;
        ui.go.clear();
        add_log("접고 이동 멈춤 (" + why + ")");
      }
    };
  auto arm_done = [events](int kind, unsigned gen) {
      return [events, kind, gen](int k, const std::string & t) {
               std::lock_guard<std::mutex> lk(events->m);
               events->done.push_back({kind, gen, k, t});
             };
    };

  std::function<void(const std::string &)> run;
  run = [&](const std::string & line) {
      const auto tok = split_words(line);
      if (tok.empty()) {return;}
      add_log("> " + line);
      if (tok[0] == "snap") {
        std::error_code ec;
        if (tok.size() == 1) {std::filesystem::create_directories(ui.mark_dir, ec);}
        const std::string path = tok.size() > 1 ? tok[1] : reserve_free(ui.mark_dir, "snap", ".txt");   // 레포 안에 안 쓰게
        add_log(dump_screen(path) ? "화면 저장: " + path : "저장 실패: " + path);
        return;
      }
      Command c;
      std::string err;
      if (!parse_command(tok, names, c, err)) {add_log("오류: " + err); return;}
      switch (c.cmd) {
        case Cmd::HELP: page = kHelpPage; scroll = 0; break;
        case Cmd::QUIT: quit = true; break;
        case Cmd::PAGE:
        case Cmd::STATUS:
          page = c.page;
          scroll = 0;
          break;
        case Cmd::CHECK: page = kCheckPage; scroll = 0; break;
        case Cmd::ARM:
        case Cmd::NAV:
          // 접고 이동이 HOME 을 기다리는 중이면 새 팔 · 주행 명령을 받지 않음 (HOME 성공 뒤 옛 목적지가 다시 나가지 않게)
          if (go_active) {add_log("거절: 접고 이동 중 (" + ui.go + ") — 먼저 x 로 멈춤"); break;}
          if (c.cmd == Cmd::ARM) {link.send_arm(c, noop);} else {link.send_nav(c, noop);}
          break;
        case Cmd::GO: {
          const std::string why = go_block_reason(snap, cfg);
          if (!why.empty()) {add_log("이동 거절: " + why); break;}
          if (go_active || cycle.active()) {add_log("이동 거절: 접고 이동 · cycle 진행 중"); break;}
          Command nav = c;
          nav.cmd = Cmd::NAV;
          if (!go_needs_fold(snap, cfg)) {
            link.send_nav(nav, noop);
            break;
          }
          go_active = true;
          go_nav = nav;
          const std::string where = c.stop.empty() ? "목표" : c.stop;
          ui.go = "접고 이동: home 중 → " + where;
          add_log("접고 이동: 팔이 받침에 안 접힘 → home 먼저, 끝나면 " + where + " 로");
          Command home;
          home.cmd = Cmd::ARM;
          home.task = 0;   // HOME
          link.send_arm(home, arm_done(1, ++go_gen));
          break;
        }
        case Cmd::CANCEL: {
          // 접고 이동이 HOME 단계면 멈춤 = 그 HOME 도 취소 (nav 만 취소하면 HOME 은 끝까지 감)
          const std::string which = go_active && c.which == "nav" ? "all" : c.which;
          if (which != "nav") {stop_cycle("cancel");}
          const std::string s = link.cancel(which);
          add_log(s.empty() ? "취소할 서버 없음" : "모든 goal 취소 요청: " + s);
          break;
        }
        case Cmd::ESTOP:
          if (c.on) {stop_cycle("E-STOP");}
          link.set_estop(c.on);
          teleop = false;
          if (c.on) {add_log("잠금 중: 새 팔 · 주행 · drive 명령 거절, 새로 시작된 goal 도 다시 취소 — estop off 로 풀기");}
          break;
        case Cmd::DRIVE: {
          const std::string why = link.drive(c.vx, c.vy, c.wz, c.secs, c.force);
          add_log(why.empty() ? "drive 시작" : "drive 거절: " + why);
          if (why.empty()) {teleop = false;}
          break;
        }
        case Cmd::STOP: link.stop_drive(); add_log("바퀴 0"); break;
        case Cmd::TELEOP: {
          const std::string why = link.set_teleop(true, c.force);
          if (!why.empty()) {add_log("teleop 거절: " + why); break;}
          teleop = true;
          add_log("teleop: w/s 앞뒤 a/d 좌우 q/e 회전 +/- 속도 space 정지 Esc 끝");
          break;
        }
        case Cmd::CALL: link.call_service(c.service, c.setbool, [&link](int, const std::string & t) {link.log(t);}); break;
        case Cmd::STOPS: for (const auto & s : stops_text(cfg.stops)) {add_log(s);} break;
        case Cmd::LOC: page = 1; scroll = 0; break;
        case Cmd::MARK: {
          const Snapshot now = link.snapshot();
          std::string text;
          const bool ok = write_mark(ui.mark_dir, c.name, now, cfg, text);
          add_log(text + (ok ? " → " + ui.mark_dir : ""));
          if (ok) {
            ui.marks.push_back(text);
            while (ui.marks.size() > 20) {ui.marks.erase(ui.marks.begin());}
            ui.last_mark = c.name;
            ui.last_mark_q = now.joints;
          }
          break;
        }
        case Cmd::MARKDIR: {
          std::error_code ec;
          const auto p = std::filesystem::absolute(c.name, ec);
          ui.mark_dir = ec ? c.name : p.lexically_normal().string();
          add_log("mark 폴더: " + ui.mark_dir + " (첫 mark 때 만듦)");
          break;
        }
        case Cmd::CYCLE: {
          if (c.count == 0) {
            cycle.request_stop();
            add_log(cycle.active() ? "cycle: 지금 작업이 끝나면 멈춤 (바로 멈추려면 cancel arm)" : "cycle 안 하는 중");
            break;
          }
          if (snap.estop) {add_log("cycle 거절: E-STOP 중"); break;}
          if (go_active) {add_log("cycle 거절: 접고 이동 중"); break;}
          std::error_code ec;
          std::filesystem::create_directories(ui.mark_dir, ec);
          if (ec) {add_log("cycle 거절: 폴더 " + ui.mark_dir + " 못 만듦"); break;}
          if (!cycle.start(c.slot, c.count, c.store_first, err)) {add_log("cycle 거절: " + err); break;}
          ui.cycle_log.clear();
          add_log("cycle " + c.slot + " " + std::to_string(c.count) + " 번 시작 (" +
            (c.store_first ? "STORE → RETRIEVE" : "RETRIEVE → STORE") + ") · 기록 cycle_" + c.slot + ".csv · 멈춤 cycle stop");
          break;
        }
        case Cmd::NOTE: {
          const std::string wall = wall_clock_text();
          std::string err2;
          std::error_code ec;
          std::filesystem::create_directories(ui.mark_dir, ec);
          const std::string sess = ui.session_file.empty() ? "" : std::filesystem::path(ui.session_file).filename().string();
          const bool ok = append_csv(ui.mark_dir + "/notes.csv", "time,session,note", {wall + "," + sess + "," + csv_clean(c.name)}, err2);
          ui.last_note = wall.substr(11, 8) + " " + c.name;
          add_log("메모: " + c.name + (ok ? "" : " (notes.csv 실패: " + err2 + ")"));
          break;
        }
        case Cmd::BAG: {
          std::string text;
          if (c.on) {
            if (bag.start(ui.mark_dir + "/bags", text)) {
              ui.bag = "기록 중: " + text;
              add_log("bag 기록 시작: " + text + " (토픽 " + std::to_string(kBagTopics.size()) + " 개)");
            } else {
              add_log("bag 거절: " + text);
            }
          } else if (bag.request_stop()) {   // SIGINT 만 보내고 돌아옴 — 끝나는 것은 아래 poll (그동안도 F9 · 입력이 됨)
            ui.bag = "끝내는 중: " + bag.path();
            add_log("bag 끝내는 중 (SIGINT) …");
          } else {
            add_log("bag 기록 중 아님");
          }
          break;
        }
        case Cmd::TEACH: log_lines(kTeachNote, add_log); break;
      }
    };

  // 쪽 번호 동작 (2 3 5 6 7 쪽) — 0 쪽 · 동작 없는 쪽에서는 번호 = 그 쪽
  auto number = [&](int n) {
      const auto acts = page_actions(page, cfg);
      if (n == 0 || page <= 0 || (acts.empty() && page != 3)) {
        if (n >= 0 && n < kPages) {
          page = n;
          scroll = 0;
        } else {
          add_log("쪽은 0~7");
        }
        return;
      }
      if (n < 1 || n > static_cast<int>(acts.size())) {add_log("이 쪽 동작은 1~" + std::to_string(acts.size())); return;}
      const PageAction & a = acts[n - 1];
      if (a.ask.empty()) {
        run(a.command);
        return;
      }
      prompting = true;
      prompt = a;
    };
  auto enter = [&]() {
      const std::string line = narrow(input);
      if (!input.empty()) {history.push_back(input);}
      hist_pos = history.size();
      input.clear();
      if (prompting && !split_words(line).empty() && split_words(line)[0] == "snap") {   // 묻는 중에도 snap 은 화면 저장 (묻기는 그대로)
        run(line);
        return;
      }
      if (prompting) {   // 물은 것의 답
        prompting = false;
        const auto t = split_words(line);
        if (t.empty()) {add_log("그만둠"); return;}
        if (prompt.confirm) {
          if (t[0] == "y" || t[0] == "Y") {run(prompt.command);} else {add_log("그만둠: " + prompt.label);}
          return;
        }
        std::string cmd = prompt.command;
        const std::size_t at = cmd.find("{}");
        if (at != std::string::npos) {cmd.replace(at, 2, line);}
        run(cmd);
        return;
      }
      const std::string t = split_words(line).empty() ? "" : split_words(line)[0];
      if (split_words(line).size() == 1 && all_digits(t) && t.size() <= 3) {
        add_log("> " + t);
        number(std::stoi(t));
        return;
      }
      run(line);
    };

  while (!quit && !interrupted.load()) {
    for (const auto & s : link.take_log()) {add_log(s);}
    snap = link.snapshot();
    if (teleop && !snap.teleop) {teleop = false;}   // estop · 주행 조건을 잃어 꺼짐
    if (snap.estop) {stop_cycle("E-STOP");}

    // 팔 작업 결과: cycle 다음 작업 · 접고 이동의 nav (ArmTask 만, 팔 순서 · 차례는 control_master)
    std::deque<ArmDone> done;
    {
      std::lock_guard<std::mutex> lk(events->m);
      done.swap(events->done);
    }
    if (!done.empty()) {snap = link.snapshot();}   // 결과 뒤의 phase · 끝 시각까지 보이게
    for (const auto & d : done) {
      if (d.kind == 1) {
        if (!go_active || d.gen != go_gen) {continue;}
        go_active = false;
        ui.go.clear();
        if (d.code != 0) {
          add_log("접고 이동 멈춤: home 실패 — 이동 안 함");
          continue;
        }
        add_log("접고 이동: 접음 → 이동");
        link.send_nav(go_nav, noop);
        continue;
      }
      if (d.gen != cycle_gen) {continue;}   // 멈춘 cycle 이 보냈던 작업의 결과
      std::string phases = "phase 없음";
      for (const auto & h : snap.arm_hist) {
        if (h.id == snap.my_arm_id && h.t0 >= cycle_sent_t - 0.5) {phases = phase_text(h, h.t1 >= 0.0 ? h.t1 : snap.now);}
      }
      std::string row;
      const std::string line = cycle.on_done(d.code, d.text, snap.now, phases, &row);
      if (line.empty()) {continue;}
      add_log("cycle " + line);
      ui.cycle_log.push_back(line);
      while (ui.cycle_log.size() > 50) {ui.cycle_log.erase(ui.cycle_log.begin());}
      std::string err;
      if (!append_csv(ui.mark_dir + "/cycle_" + cycle.slot() + ".csv", CycleRun::csv_header(), {row}, err)) {add_log("cycle 기록 실패: " + err);}
      if (!cycle.active()) {add_log("cycle " + cycle.status(snap.now));}
    }
    if (cycle.active() && !cycle.waiting()) {
      Command next;
      if (cycle.next(next, snap.now, wall_clock_text())) {
        cycle_sent_t = snap.now;
        link.send_arm(next, arm_done(0, ++cycle_gen));
      }
    }
    ui.cycle = cycle.status(snap.now);
    if (ui.bag.rfind("끝내는 중", 0) == 0 && bag.poll()) {
      ui.bag = "끝: " + bag.path();
      add_log("bag 끝 (정상 종료): " + bag.path());
    } else if (ui.bag.rfind("기록 중", 0) == 0 && !bag.running()) {
      ui.bag = "혼자 끝남: " + bag.path() + " (" + bag.path() + "_record.log 확인)";
      add_log("bag 이 혼자 끝남 — " + bag.path() + "_record.log");
    }

    std::vector<Line> lines;
    if (page == kHelpPage) {
      lines.push_back({"── 도움말 (Esc · 0 = 전체 요약) ──", Level::HEAD});
      log_lines(kHelp, [&lines](const std::string & t) {lines.push_back({t, Level::NORMAL});});
      lines.push_back({"", Level::NORMAL});
      lines.push_back({"── teach ──", Level::HEAD});
      log_lines(kTeachNote, [&lines](const std::string & t) {lines.push_back({t, Level::NORMAL});});
    } else if (page == kCheckPage) {
      lines.push_back({"── 시작 전 점검 (계속 다시 봄, Esc · 0 = 전체 요약) ──", Level::HEAD});
      for (const auto & l : check_lines(preflight(snap, cfg, domain ? domain : ""))) {lines.push_back(l);}
    } else {
      lines = page_lines(page, snap, cfg, ui);
    }

    erase();
    const int H = LINES, W = COLS;
    const int log_rows = std::min(kLogRows, std::max(2, H / 5));
    const int top_rows = std::max(1, H - 4 - log_rows);
    std::string title = " robot_cli · ROS_DOMAIN_ID=" + std::string(domain ? domain : "0") + " · " + clock_text() +
      " · ? 도움말 · q 끝 · F9 E-STOP";
    if (snap.estop) {title += "  [E-STOP]";}
    if (teleop) {title += "  [TELEOP]";}
    if (cycle.active()) {title += "  [CYCLE]";}
    if (go_active) {title += "  [접고 이동]";}
    if (!bag.path().empty() && ui.bag.rfind("기록 중", 0) == 0) {title += "  [BAG]";}
    attr_set(A_NORMAL, P_TITLE, nullptr);
    mvhline(0, 0, ' ', W);
    put(0, 0, title, W, A_BOLD, P_TITLE);
    // 메뉴: 지금 쪽은 노란 바탕
    int mx = 0;
    for (int k = 0; k < kPages && mx < W; ++k) {
      const std::string item = " " + std::to_string(k) + " " + kPageNames[k] + " ";
      put(1, mx, item, W - mx, k == page ? A_BOLD : A_NORMAL, k == page ? P_SEL : 0);
      mx += width_of(item) + 1;
    }
    if (mx < W) {
      put(1, mx, page == 0 ? " ← 번호+Enter" : page == kHelpPage ? " 도움말 ← Esc" : page == kCheckPage ? " 점검 ← Esc" : " ← Esc · ←/→",
        W - mx, A_NORMAL, P_HEAD);
    }
    scroll = std::clamp(scroll, 0, std::max(0, static_cast<int>(lines.size()) - top_rows));
    for (int i = 0; i < top_rows && scroll + i < static_cast<int>(lines.size()); ++i) {put_line(2 + i, lines[scroll + i], W);}
    const int sep = 2 + top_rows;
    std::string rule;
    for (int i = 0; i < W; ++i) {rule += "─";}
    put(sep, 0, rule, W, A_NORMAL, P_OFF);
    if (scroll + top_rows < static_cast<int>(lines.size())) {put(sep, 2, " ▼ PgDn ", W - 2);}
    if (scroll > 0) {put(sep, 12, " ▲ PgUp ", W - 12);}
    const int n_logs = static_cast<int>(logs.size());
    for (int i = 0; i < log_rows; ++i) {
      const int k = n_logs - log_rows + i;
      if (k >= 0) {put(sep + 1 + i, 0, logs[k], W);}
    }
    const int iy = H - 1;
    if (teleop) {
      char b[200];
      std::snprintf(b, sizeof(b), "[teleop] 직선 %.2f m/s · 회전 %.1f rad/s · w/s a/d q/e · +/- · space 정지 · ! E-STOP · Esc 끝 · 지금 vx %+.2f vy %+.2f wz %+.2f",
        tp.lin, tp.ang, snap.dvx, snap.dvy, snap.dwz);
      put(iy, 0, b, W, A_BOLD, P_OK);
      curs_set(0);
    } else if (prompting) {
      put(iy, 0, "[" + prompt.label + "] " + prompt.ask + " > " + narrow(input), W, A_BOLD, P_WARN);
      curs_set(1);
    } else {
      const std::string tag = page == kHelpPage ? "?" : page == kCheckPage ? "점검" : std::to_string(page);
      put(iy, 0, "[" + tag + "]> " + narrow(input), W);
      curs_set(1);
    }
    refresh();

    wint_t ch;
    const int r = get_wch(&ch);
    if (r == ERR) {continue;}
    if (r == KEY_CODE_YES && ch == KEY_F(9)) {   // 어느 모드에서든 한 키 E-STOP
      stop_cycle("E-STOP");
      link.set_estop(true);
      teleop = false;
      prompting = false;
      add_log("F9 E-STOP — estop off 로 풀기");
      continue;
    }
    if (r == KEY_CODE_YES) {
      switch (ch) {
        case KEY_PPAGE: scroll -= top_rows / 2; break;
        case KEY_NPAGE: scroll += top_rows / 2; break;
        case KEY_LEFT: if (!teleop) {page = page <= 0 ? kPages - 1 : page - 1; scroll = 0;} break;
        case KEY_RIGHT: if (!teleop) {page = page < 0 ? 0 : (page + 1) % kPages; scroll = 0;} break;
        case KEY_BACKSPACE: if (!teleop && !input.empty()) {input.pop_back();} break;
        case KEY_UP: if (!teleop && hist_pos > 0) {input = history[--hist_pos];} break;
        case KEY_DOWN:
          if (!teleop && hist_pos < history.size()) {
            ++hist_pos;
            input = hist_pos < history.size() ? history[hist_pos] : L"";
          }
          break;
        case KEY_ENTER: if (!teleop) {enter();} break;
        default: break;
      }
      continue;
    }
    if (teleop) {
      switch (ch) {
        case L'w': link.teleop_cmd(tp.lin, 0, 0); break;
        case L's': link.teleop_cmd(-tp.lin, 0, 0); break;
        case L'a': link.teleop_cmd(0, tp.lin, 0); break;
        case L'd': link.teleop_cmd(0, -tp.lin, 0); break;
        case L'q': link.teleop_cmd(0, 0, tp.ang); break;
        case L'e': link.teleop_cmd(0, 0, -tp.ang); break;
        case L' ': link.teleop_cmd(0, 0, 0); break;
        case L'!':
          stop_cycle("E-STOP");
          link.set_estop(true);
          teleop = false;
          add_log("! E-STOP — estop off 로 풀기");
          break;
        case L'+': case L'=':
          tp.lin = std::min(cfg.max_linear, tp.lin + 0.01);
          tp.ang = std::min(cfg.max_angular, tp.ang + 0.1);
          break;
        case L'-':
          tp.lin = std::max(std::min(0.01, cfg.max_linear), tp.lin - 0.01);
          tp.ang = std::max(std::min(0.1, cfg.max_angular), tp.ang - 0.1);
          break;
        case 27: case L'x':
          link.set_teleop(false);
          teleop = false;
          add_log("teleop 끝 (바퀴 0)");
          break;
        default: break;
      }
      continue;
    }
    if (ch == L'\n' || ch == L'\r') {
      enter();
    } else if (ch == 127 || ch == 8) {
      if (!input.empty()) {input.pop_back();}
    } else if (ch == L'\t') {   // Tab: 명령 · 칸 · 정차 이름 자동 완성
      const std::string cur = narrow(input);
      const auto cand = complete_candidates(cur, names);
      if (cand.empty()) {continue;}
      const bool new_word = cur.empty() || cur.back() == ' ';
      const std::size_t cut = new_word ? cur.size() : cur.find_last_of(' ') == std::string::npos ? 0 : cur.find_last_of(' ') + 1;
      const std::string fill = cand.size() == 1 ? cand[0] + " " : common_prefix(cand);
      if (fill.size() >= cur.size() - cut) {input = widen(cur.substr(0, cut) + fill);}
      if (cand.size() > 1) {
        std::string t;
        for (std::size_t i = 0; i < cand.size() && i < 14; ++i) {t += cand[i] + " ";}
        add_log("후보: " + t + (cand.size() > 14 ? "…" : ""));
      }
    } else if (ch == 27) {   // Esc: 묻기 그만 → 입력 지우기 → 0 쪽
      if (prompting) {
        prompting = false;
        input.clear();
        add_log("그만둠: " + prompt.label);
      } else if (!input.empty()) {
        input.clear();
      } else {
        page = 0;
        scroll = 0;
      }
    } else if (ch >= 32) {
      input += static_cast<wchar_t>(ch);
    }
  }

  // 끝: cycle · 접고 이동 멈춤 + 이 CLI 의 goal 취소 (답이 아직 안 온 것도, RobotLink::finish) + 바퀴 0 + bag 정상 종료
  stop_cycle("끝냄");
  link.set_teleop(false);
  link.stop_drive();
  const std::string fin = link.finish(1.0);
  if (!fin.empty()) {std::this_thread::sleep_for(std::chrono::milliseconds(300));}
  for (int i = 0; i < 10 && link.drive_busy(); ++i) {std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  const bool bag_was = bag.running();
  if (bag_was) {   // 끝낼 때만 기다림 (이미 goal 취소 · 바퀴 0 뒤)
    put(LINES - 1, 0, "bag 정상 종료 중 (최대 10 s) …", COLS, A_BOLD, P_WARN);
    refresh();
    bag.stop();
  }
  endwin();
  if (!fin.empty()) {std::printf("robot_cli 끝: %s\n", fin.c_str());}
  if (bag_was) {std::printf("bag 기록: %s\n", bag.path().c_str());}
  if (!ui.session_file.empty()) {
    session << wall_clock_text() << " 세션 끝" << (fin.empty() ? "" : " · " + fin) << "\n";
    std::printf("세션 기록: %s\n", ui.session_file.c_str());
  }
  return 0;
}

}  // namespace robot_cli
