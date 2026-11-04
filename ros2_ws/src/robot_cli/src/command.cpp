// command.cpp — 명령 한 줄 읽기, 설명은 command.hpp

#include "robot_cli/command.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <sstream>

#include "interfaces/action/arm_task.hpp"

namespace robot_cli
{

using ArmTask = interfaces::action::ArmTask;

const char * const kHelp =
  "켜기: nt = 화면 · nt <명령> = 한 줄 (예 nt h · nt g B1 · nt check · nt status 2)   설치: tools/robot_cli/install.sh\n"
  "짧게  h home · l look · b barcode · s5 store S5 · r5 retrieve S5 · p x y pick · pl x y place · g B1 접고 이동\n"
  "      x 모두 취소 · ! estop (F9) · c check · m 이름 mark · n 메모 note · ? help · q quit · Tab 자동 완성 · ↑↓ 기록\n"
  "화면  0 쪽에서 번호 + Enter = 쪽 · 2 3 5 6 7 쪽에서 번호 = 그 쪽 동작 · Esc · 0 = 전체 요약 · ←/→ = 옆 쪽 · PgUp/PgDn\n"
  "팔    home [force] · look · barcode · store <칸> · retrieve <칸> (칸 = S5 또는 5)\n"
  "      pick x y [z] [yaw°] · place x y [z] [yaw°] (base_link m, z 0 = 박스 위 기본 높이) · arm <TASK> …\n"
  "      lazy_fold: STORE · PLACE 는 P0 에서 끝 (바퀴 잠금 유지, 다음 팔 작업은 바로) → 주행 전 home (fold 경로로 접음)\n"
  "주행  go <정차|x y [yaw°]> = 접혀 있지 않으면 home 먼저 → nav (퍽 쥐었으면 거절) · nav <정차|x y [yaw°]> = 바로\n"
  "      drive vx vy wz [초] [force] (한계 = controllers.yaml max_linear · max_angular, 기본 1 초) · stop · stops\n"
  "      teleop [force] (화면에서만: w/s 앞뒤 a/d 좌우 q/e 회전 +/- 속도 space 정지 Esc 끝)\n"
  "멈춤  cancel [arm|nav|all] · estop [on|off] (소프트 정지: 모든 팔 · 주행 goal 취소 + /cmd_vel 0)\n"
  "점검  check (시작 전 점검: 노드 · 보드 · 컨트롤러 · 관절 · 접힘 · 라이다 · IMU · 배터리 · 시계, 실패 있으면 끝 코드 1)\n"
  "실험  mark <이름> · markdir <폴더> · cycle <칸> <횟수> [store] · cycle stop · note <메모> · bag start|stop · teach\n"
  "기타  call <서비스> [on|off] · mission start|resume|clear_gripper|reset_signal · status [쪽] · loc · snap [파일]";

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

bool to_num(const std::string & s, double & v)
{
  try {
    std::size_t used = 0;
    v = std::stod(s, &used);
    return used == s.size() && std::isfinite(v);
  } catch (const std::exception &) {
    return false;
  }
}

// tok[from..] 에서 숫자 min_n ~ max_n 개
bool read_nums(const std::vector<std::string> & tok, std::size_t from, std::size_t min_n, std::size_t max_n,
  std::vector<double> & out, std::string & err)
{
  out.clear();
  const std::size_t n = tok.size() > from ? tok.size() - from : 0;
  if (n < min_n || n > max_n) {
    err = "숫자 " + std::to_string(min_n) + (min_n == max_n ? "" : "~" + std::to_string(max_n)) + " 개 필요";
    return false;
  }
  for (std::size_t i = from; i < tok.size(); ++i) {
    double v;
    if (!to_num(tok[i], v)) {err = "숫자가 아님: " + tok[i]; return false;}
    out.push_back(v);
  }
  return true;
}

bool find_name(const std::vector<std::string> & list, const std::string & name, std::string & found)
{
  for (const auto & n : list) {
    if (lower(n) == lower(name)) {found = n; return true;}
  }
  return false;
}

bool all_digits(const std::string & s)
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) {return std::isdigit(c);});
}

// 칸: 이름 (S5) 또는 번호만 (5)
bool find_slot(const std::vector<std::string> & slots, const std::string & name, std::string & found)
{
  return find_name(slots, name, found) || (all_digits(name) && find_name(slots, "S" + name, found));
}

// 짧은 별칭 → 긴 명령 (h → home, s5 → store S5, g B1 → go B1 …). 해당 없으면 그대로
std::vector<std::string> expand_alias(const std::vector<std::string> & tok)
{
  std::vector<std::string> t = tok;
  const std::string c = lower(t[0]);
  static const std::vector<std::pair<std::string, std::string>> words = {
    {"h", "home"}, {"l", "look"}, {"b", "barcode"}, {"p", "pick"}, {"pl", "place"}, {"g", "go"}, {"!", "estop"},
    {"c", "check"}, {"m", "mark"}, {"n", "note"}, {"q", "quit"}, {"s", "store"}, {"r", "retrieve"}};
  for (const auto & w : words) {
    if (c == w.first) {t[0] = w.second; return t;}
  }
  if (c == "x" && t.size() == 1) {return {"cancel", "all"};}
  if (c == "scan" && t.size() == 1) {return {"look", "rack"};}   // 빈 손 적재함 스캔
  if (c.size() >= 2 && (c[0] == 's' || c[0] == 'r') && all_digits(c.substr(1))) {
    t[0] = c[0] == 's' ? "store" : "retrieve";
    t.insert(t.begin() + 1, c.substr(1));
  }
  return t;
}

std::string joined(const std::vector<std::string> & v)
{
  std::string s;
  for (const auto & x : v) {s += (s.empty() ? "" : " ") + x;}
  return s;
}

// tok[0] = 작업 이름 (home · pick …), tok[1..] = 인자
bool parse_arm(const std::vector<std::string> & tok, const Names & names, Command & out, std::string & err)
{
  out.cmd = Cmd::ARM;
  const std::string t = lower(tok[0]);
  std::vector<double> v;
  if (t == "home") {
    out.task = ArmTask::Goal::HOME;
    if (tok.size() == 2 && lower(tok[1]) == "force") {out.slot = "force"; return true;}
    if (tok.size() != 1) {err = "home [force]"; return false;}
    return true;
  }
  if (t == "look" || t == "look_barcode" || t == "barcode") {
    out.task = t == "look" ? ArmTask::Goal::LOOK : ArmTask::Goal::LOOK_BARCODE;
    // look rack = 빈 손 적재함 스캔 (control_master 경로 rack_scan, 받침에서 시작해 받침으로 끝남)
    if (t == "look" && tok.size() == 2 && lower(tok[1]) == "rack") {out.slot = "rack"; return true;}
    if (tok.size() != 1) {err = t == "look" ? "look [rack]" : t + " 는 인자 없음"; return false;}
    return true;
  }
  if (t == "store" || t == "retrieve") {
    out.task = t == "store" ? ArmTask::Goal::STORE : ArmTask::Goal::RETRIEVE;
    if (tok.size() != 2) {err = t + " <칸>  (" + joined(names.slots) + " 또는 번호)"; return false;}
    if (!find_slot(names.slots, tok[1], out.slot)) {err = "모르는 칸 " + tok[1] + " (" + joined(names.slots) + ")"; return false;}
    return true;
  }
  // pick det · pick tag <ID>: 손목 인지가 본 퍽으로 집기 (목표는 보낼 때 TF 로 만듦)
  //   번호로 고르기는 없음: 화면을 그린 뒤 검출 순서가 바뀌면 다른 퍽을 고를 수 있음 → 여러 개면 태그로
  if (t == "pick" && tok.size() >= 2 && (lower(tok[1]) == "det" || lower(tok[1]) == "tag")) {
    out.task = ArmTask::Goal::PICK;
    out.from_det = true;
    if (lower(tok[1]) == "tag") {
      if (tok.size() != 3) {err = "pick tag <퍽 태그 ID>"; return false;}
      out.det_tag = tok[2];
      return true;
    }
    if (tok.size() == 2) {return true;}
    err = "pick det (퍽이 하나일 때) · pick tag <퍽 태그 ID> (여러 개일 때)";
    return false;
  }
  if (t == "pick" || t == "place") {
    out.task = t == "pick" ? ArmTask::Goal::PICK : ArmTask::Goal::PLACE;
    if (!read_nums(tok, 1, 2, 4, v, err)) {err = t + " x y [z] [yaw°] — " + err; return false;}
    out.x = v[0];
    out.y = v[1];
    out.z = v.size() > 2 ? v[2] : 0.0;
    out.yaw_deg = v.size() > 3 ? v[3] : NAN;
    return true;
  }
  err = "모르는 팔 작업 " + tok[0] + " (HOME PICK PLACE STORE RETRIEVE LOOK LOOK_BARCODE)";
  return false;
}
}  // namespace

std::vector<std::string> split_words(const std::string & line)
{
  std::vector<std::string> out;
  std::istringstream in(line);
  for (std::string w; in >> w;) {out.push_back(w);}
  return out;
}

const char * task_name(uint8_t task)
{
  switch (task) {
    case ArmTask::Goal::HOME: return "HOME";
    case ArmTask::Goal::PICK: return "PICK";
    case ArmTask::Goal::PLACE: return "PLACE";
    case ArmTask::Goal::STORE: return "STORE";
    case ArmTask::Goal::RETRIEVE: return "RETRIEVE";
    case ArmTask::Goal::LOOK: return "LOOK";
    case ArmTask::Goal::LOOK_BARCODE: return "LOOK_BARCODE";
    default: return "?";
  }
}

bool parse_command(const std::vector<std::string> & raw, const Names & names, Command & out, std::string & err)
{
  out = Command{};
  err.clear();
  if (raw.empty()) {err = "빈 명령"; return false;}
  const std::vector<std::string> tok = expand_alias(raw);
  const std::string c = lower(tok[0]);
  std::vector<double> v;

  if (c == "help" || c == "?" || c == "-h" || c == "--help") {out.cmd = Cmd::HELP; return true;}
  if (c == "quit" || c == "exit") {out.cmd = Cmd::QUIT; return true;}
  if (c.size() == 1 && c[0] >= '0' && c[0] <= '7' && tok.size() == 1) {out.cmd = Cmd::PAGE; out.page = c[0] - '0'; return true;}
  if (c == "status") {
    out.cmd = Cmd::STATUS;
    if (tok.size() > 2 || (tok.size() == 2 && (tok[1].size() != 1 || tok[1][0] < '0' || tok[1][0] > '7'))) {
      err = "status [쪽 0~7]";
      return false;
    }
    out.page = tok.size() == 2 ? tok[1][0] - '0' : 0;
    return true;
  }
  if (c == "teach") {out.cmd = Cmd::TEACH; return true;}
  if (c == "check") {
    out.cmd = Cmd::CHECK;
    if (tok.size() != 1) {err = "check 는 인자 없음"; return false;}
    return true;
  }
  if (c == "note") {
    out.cmd = Cmd::NOTE;
    if (tok.size() < 2) {err = "note <메모>"; return false;}
    for (std::size_t i = 1; i < tok.size(); ++i) {out.name += (i > 1 ? " " : "") + tok[i];}
    return true;
  }
  if (c == "bag") {
    out.cmd = Cmd::BAG;
    const std::string a = tok.size() == 2 ? lower(tok[1]) : "";
    if (a != "start" && a != "stop") {err = "bag start|stop"; return false;}
    out.on = a == "start";
    return true;
  }
  if (c == "mark") {
    out.cmd = Cmd::MARK;
    if (tok.size() != 2) {err = "mark <이름>  (한 단어)"; return false;}
    out.name = tok[1];
    // csv · yaml 칸에 그대로 들어감 → 구분 글자 · 따옴표 금지
    if (out.name.size() > 64 || out.name.find_first_of(",\"':#{}[]\\") != std::string::npos) {
      err = "mark 이름에 , \" ' : # { } [ ] \\ 금지 (64 바이트까지)";
      return false;
    }
    return true;
  }
  if (c == "markdir") {
    out.cmd = Cmd::MARKDIR;
    if (tok.size() != 2) {err = "markdir <폴더>  (빈칸 없는 경로)"; return false;}
    out.name = tok[1];
    return true;
  }
  if (c == "cycle") {
    out.cmd = Cmd::CYCLE;
    if (tok.size() == 2 && lower(tok[1]) == "stop") {out.count = 0; return true;}
    if (tok.size() < 3 || tok.size() > 4) {err = "cycle <칸> <횟수> [store] · cycle stop"; return false;}
    if (!find_slot(names.slots, tok[1], out.slot)) {err = "모르는 칸 " + tok[1] + " (" + joined(names.slots) + ")"; return false;}
    double n = 0.0;
    if (!to_num(tok[2], n) || n != std::floor(n) || n < 1 || n > 50) {err = "횟수 1~50 (정수)"; return false;}
    out.count = static_cast<int>(n);
    if (tok.size() == 4) {
      if (lower(tok[3]) != "store") {err = "cycle <칸> <횟수> [store]  (store = STORE 먼저: 팔이 기준점에서 퍽을 쥔 채)"; return false;}
      out.store_first = true;
    }
    return true;
  }
  if (c == "stops") {out.cmd = Cmd::STOPS; return true;}
  if (c == "loc") {out.cmd = Cmd::LOC; return true;}
  if (c == "teleop") {
    out.cmd = Cmd::TELEOP;
    out.force = tok.size() == 2 && lower(tok[1]) == "force";
    if (tok.size() > 2 || (tok.size() == 2 && !out.force)) {err = "teleop [force]"; return false;}
    return true;
  }
  if (c == "stop") {out.cmd = Cmd::STOP; return true;}

  if (c == "arm") {
    if (tok.size() < 2) {err = "arm <TASK> …"; return false;}
    return parse_arm({tok.begin() + 1, tok.end()}, names, out, err);
  }
  for (const char * t : {"home", "pick", "place", "store", "retrieve", "look", "look_barcode", "barcode"}) {
    if (c == t) {return parse_arm(tok, names, out, err);}
  }

  if (c == "nav" || c == "goto" || c == "go") {
    out.cmd = c == "go" ? Cmd::GO : Cmd::NAV;
    if (tok.size() == 2) {
      if (!find_name(names.stops, tok[1], out.stop)) {err = "모르는 정차 이름 " + tok[1] + " (" + joined(names.stops) + ")"; return false;}
      return true;
    }
    if (!read_nums(tok, 1, 2, 3, v, err)) {err = "nav <정차이름> 또는 nav x y [yaw°] — " + err; return false;}
    out.x = v[0];
    out.y = v[1];
    out.yaw_nav_deg = v.size() > 2 ? v[2] : 0.0;
    return true;
  }

  if (c == "cancel") {
    out.cmd = Cmd::CANCEL;
    if (tok.size() > 2) {err = "cancel [arm|nav|all]"; return false;}
    out.which = tok.size() == 2 ? lower(tok[1]) : "all";
    if (out.which != "arm" && out.which != "nav" && out.which != "all") {err = "cancel [arm|nav|all]"; return false;}
    return true;
  }

  if (c == "estop") {
    out.cmd = Cmd::ESTOP;
    if (tok.size() > 2) {err = "estop [on|off]"; return false;}
    const std::string s = tok.size() == 2 ? lower(tok[1]) : "on";
    if (s != "on" && s != "off") {err = "estop [on|off]"; return false;}
    out.on = s == "on";
    return true;
  }

  if (c == "drive") {
    out.cmd = Cmd::DRIVE;
    std::vector<std::string> t = tok;
    if (lower(t.back()) == "force") {out.force = true; t.pop_back();}
    if (!read_nums(t, 1, 3, 4, v, err)) {err = "drive vx vy wz [초] [force] — " + err; return false;}
    out.vx = v[0];
    out.vy = v[1];
    out.wz = v[2];
    out.secs = v.size() > 3 ? v[3] : 1.0;
    char lim[96];
    if (std::hypot(out.vx, out.vy) > names.max_linear + 1e-9) {
      std::snprintf(lim, sizeof(lim), "선속도 hypot(vx, vy) ≤ %.2f m/s", names.max_linear);
      err = lim;
      return false;
    }
    if (std::fabs(out.wz) > names.max_angular + 1e-9) {
      std::snprintf(lim, sizeof(lim), "회전 |wz| ≤ %.2f rad/s", names.max_angular);
      err = lim;
      return false;
    }
    if (out.secs <= 0.0 || out.secs > kMaxDriveSecs) {err = "시간 0 < 초 ≤ 10"; return false;}
    return true;
  }

  if (c == "call") {
    out.cmd = Cmd::CALL;
    if (tok.size() < 2 || tok.size() > 3) {err = "call <서비스> [on|off]"; return false;}
    out.service = tok[1][0] == '/' ? tok[1] : "/" + tok[1];
    if (tok.size() == 3) {
      const std::string s = lower(tok[2]);
      if (s == "on" || s == "true" || s == "1") {out.setbool = 1;} else if (s == "off" || s == "false" || s == "0") {
        out.setbool = 0;
      } else {err = "SetBool 값은 on|off"; return false;}
    }
    return true;
  }

  if (c == "mission") {
    out.cmd = Cmd::CALL;
    const std::string s = tok.size() == 2 ? lower(tok[1]) : "";
    if (s != "start" && s != "resume" && s != "clear_gripper" && s != "reset_signal") {
      err = "mission start|resume|clear_gripper|reset_signal";
      return false;
    }
    out.service = "/mission/" + s;
    return true;
  }

  err = "모르는 명령 " + tok[0] + " (help)";
  return false;
}

}  // namespace robot_cli

namespace robot_cli
{
std::vector<std::string> complete_candidates(const std::string & line, const Names & names)
{
  static const std::vector<std::string> cmds = {
    "home", "look", "scan", "barcode", "store", "retrieve", "pick", "place", "arm", "go", "nav", "drive", "stop", "stops", "teleop",
    "cancel", "estop", "check", "mark", "markdir", "cycle", "note", "bag", "teach", "call", "mission", "status", "loc", "snap",
    "help", "quit"};
  const auto tok = split_words(line);
  const bool new_word = line.empty() || line.back() == ' ';
  const std::string word = new_word || tok.empty() ? "" : tok.back();
  const std::size_t index = new_word ? tok.size() : tok.size() - 1;
  std::vector<std::string> pool;
  if (index == 0) {
    pool = cmds;
  } else {
    const std::string c = lower(expand_alias({tok[0]})[0]);
    if (index == 1 && (c == "store" || c == "retrieve" || c == "cycle")) {pool = names.slots;}
    if (index == 1 && (c == "go" || c == "nav" || c == "goto")) {pool = names.stops;}
    if (index == 1 && c == "mission") {pool = {"start", "resume", "clear_gripper", "reset_signal"};}
    if (index == 1 && c == "bag") {pool = {"start", "stop"};}
    if (index == 1 && c == "cancel") {pool = {"arm", "nav", "all"};}
  }
  std::vector<std::string> out;
  for (const auto & p : pool) {
    if (lower(p).rfind(lower(word), 0) == 0) {out.push_back(p);}
  }
  return out;
}
}  // namespace robot_cli
