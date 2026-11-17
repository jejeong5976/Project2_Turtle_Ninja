// teach.cpp — mark 기록 · cycle 반복 시험, 설명은 teach.hpp

#include "robot_cli/teach.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "interfaces/action/arm_task.hpp"

namespace robot_cli
{

namespace
{
using ArmTask = interfaces::action::ArmTask;
const std::vector<std::string> kJ5 = {"joint1", "joint2", "joint3", "joint4", "joint5"};
const char * kGrip = "gripper_left_joint";

std::string fmt(const char * f, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

double deg(double r) {return r * 180.0 / M_PI;}

// 숫자 칸 (없으면 빈 칸 / yaml null)
std::string num(double v, const char * f = "%.5f") {return std::isfinite(v) ? fmt(f, v) : "";}
std::string ynum(double v, const char * f = "%.5f") {return std::isfinite(v) ? fmt(f, v) : "null";}

double get(const std::map<std::string, double> & m, const std::string & k)
{
  const auto it = m.find(k);
  return it == m.end() ? NAN : it->second;
}

// 처짐 보정 값: /joint_states_corrected 도 신선할 때만 (끊긴 뒤의 옛 값을 새 엔코더 값과 짝짓지 않게), 아니면 NaN = 빈 칸 · null
double corr(const Snapshot & s, const std::string & j)
{
  if (s.corrected_t < 0.0 || s.now - s.corrected_t > kMarkMaxAgeS) {return NAN;}
  return get(s.corrected, j);
}

double corr_age(const Snapshot & s) {return s.corrected_t < 0.0 ? NAN : s.now - s.corrected_t;}

double tick_of(const Config & cfg, const std::string & j, double q)
{
  const auto c = cfg.cal.find(j);
  return c == cfg.cal.end() || !std::isfinite(q) ? NAN : std::round(joint_to_tick(c->second, q));
}

std::string mode_text(const Snapshot & s)
{
  if (s.mode_t < 0.0) {return "";}
  static const char * n[] = {"STOP", "DRIVE", "ARM"};
  return s.mode <= 2 ? n[s.mode] : fmt("%u", s.mode);
}

struct PoseInfo
{
  std::string name;
  double diff{NAN};
  bool fold{false};
};

PoseInfo pose_info(const Snapshot & s, const Config & cfg)
{
  PoseInfo p;
  double d = NAN;
  const NamedPose * n = nearest_pose(cfg.arm, s.joints, &d);
  if (n) {
    p.name = d <= cfg.arm.near_tol ? n->name : "";
    p.diff = deg(d);
  }
  p.fold = folded(cfg.arm, s.joints);
  return p;
}

std::string clean(const std::string & s) {return csv_clean(s);}
}  // namespace

// 남이 준 글 (인지 · control_master · 메모) 을 csv 칸 · yaml "…" 안에 넣을 수 있게: , → ; · 따옴표 · \ 빼기 · 줄바꿈 → 빈칸
std::string csv_clean(const std::string & s)
{
  std::string out;
  for (char c : s) {
    if (c == ',') {out += ';';} else if (c == '\n' || c == '\r') {out += ' ';} else if (c != '"' && c != '\\') {out += c;}
  }
  return out;
}

namespace
{

std::string wrist_ids(const Snapshot & s)
{
  std::string out;
  for (const auto & d : s.wrist_dets) {
    if (!d.id.empty()) {out += (out.empty() ? "" : " ") + clean(d.id);}
  }
  return out;
}

double wrist_age(const Snapshot & s) {return s.wrist_t < 0.0 ? NAN : s.now - s.wrist_t;}

}  // namespace

const char * const kTeachNote =
  "손으로 움직이기 (토크 끄기) 는 이 CLI 에 없음 — 지금 인터페이스로는 안전하게 못 함:\n"
  "  ① 토크 끄기는 launch 고정값: ros2_control.xacro torque_off_on_deactivate=false → bus_arm 을 inactive 로 해도 토크 유지\n"
  "     (끄는 길은 cleanup → configure 의 CONFIG 뿐 = 고장 복구 절차, 팔 6 축이 한 번에 떨어짐)\n"
  "  ② 다시 켤 때: OpenCR 은 지금 위치로 켜지만, arm · gripper JTC 는 set_last_command_interface_value_as_state_on_activation\n"
  "     기본 true · 명령 칸이 옛 값 그대로 → 가르치기 전 자세로 튈 수 있음. 지금 위치를 명령으로 넣으려면 arm_controller 에\n"
  "     궤적을 보내야 하는데 그건 control_master 의 바퀴 · 팔 차례를 건너뜀 (금지)\n"
  "  ③ control_master 는 보드가 active 가 아니면 하드웨어 이상 → STOP · 복구에 HOME 이 필요한데 손으로 둔 자세에서는 검사된 경로가 없음\n"
  "지금 할 수 있는 것: 값 보기 (2 · 7 쪽) + mark <이름>. 필요한 지원은 README '손으로 가르치기'";

std::string wall_clock_text()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  std::tm tm{};
  localtime_r(&t, &tm);
  char b[40];
  std::strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%S", &tm);
  return fmt("%s.%03d", b, ms);
}

std::string mark_csv_header()
{
  std::string h = "name,time,joint_age_s,corr_age_s,mode,pose,pose_diff_deg,folded";
  for (const auto & j : kJ5) {h += "," + j + "_tick," + j + "_rad," + j + "_deg," + j + "_corr_rad," + j + "_corr_deg";}
  h += ",gripper_tick,gripper_m,gripper_gap_mm,odom_x,odom_y,odom_yaw_deg,map_x,map_y,map_yaw_deg,wrist_n,wrist_age_s,wrist_ids";
  return h;
}

std::string mark_csv_row(const std::string & name, const std::string & wall, const Snapshot & s, const Config & cfg)
{
  const PoseInfo p = pose_info(s, cfg);
  std::string r = name + "," + wall + "," + num(s.joints_t < 0.0 ? NAN : s.now - s.joints_t, "%.3f") + "," + num(corr_age(s), "%.3f") +
    "," + mode_text(s) + "," +
    p.name + "," + num(p.diff, "%.2f") + "," + (p.fold ? "1" : "0");
  for (const auto & j : kJ5) {
    const double q = get(s.joints, j), c = corr(s, j);
    r += "," + num(tick_of(cfg, j, q), "%.0f") + "," + num(q) + "," + num(deg(q), "%.3f") + "," + num(c) + "," +
      num(deg(c), "%.3f");
  }
  const double g = get(s.joints, kGrip);
  r += "," + num(tick_of(cfg, kGrip, g), "%.0f") + "," + num(g) + "," + num(std::isfinite(g) ? gripper_gap(g) * 1000.0 : NAN, "%.1f");
  r += "," + (s.odom_t < 0.0 ? std::string(",,") : num(s.ox, "%.4f") + "," + num(s.oy, "%.4f") + "," + num(deg(s.oyaw), "%.2f"));
  r += "," + (!s.have_map_pose ? std::string(",,") : num(s.mx, "%.4f") + "," + num(s.my, "%.4f") + "," + num(deg(s.myaw), "%.2f"));
  r += "," + (s.wrist_t < 0.0 ? std::string() : std::to_string(s.wrist_n)) + "," + num(wrist_age(s), "%.2f") + "," + wrist_ids(s);
  return r;
}

std::string mark_yaml_entry(const std::string & name, const std::string & wall, const Snapshot & s, const Config & cfg)
{
  const PoseInfo p = pose_info(s, cfg);
  std::string y = "- name: \"" + name + "\"\n";
  y += "  time: \"" + wall + "\"\n";
  y += "  joint_age_s: " + ynum(s.joints_t < 0.0 ? NAN : s.now - s.joints_t, "%.3f") + "\n";
  y += "  corrected_age_s: " + ynum(corr_age(s), "%.3f") + "   # 1 s 넘으면 corrected 값은 null\n";
  y += "  mode: " + (mode_text(s).empty() ? std::string("null") : mode_text(s)) + "\n";
  y += "  pose: {name: " + (p.name.empty() ? std::string("null") : "\"" + p.name + "\"") + ", diff_deg: " + ynum(p.diff, "%.2f") +
    ", folded: " + (p.fold ? "true" : "false") + "}\n";
  y += "  joints:\n";
  for (const auto & j : kJ5) {
    const double q = get(s.joints, j), c = corr(s, j);
    y += "    " + j + ": {tick: " + ynum(tick_of(cfg, j, q), "%.0f") + ", rad: " + ynum(q) + ", deg: " + ynum(deg(q), "%.3f") +
      ", corrected_rad: " + ynum(c) + ", corrected_deg: " + ynum(deg(c), "%.3f") + "}\n";
  }
  const double g = get(s.joints, kGrip);
  y += std::string("    ") + kGrip + ": {tick: " + ynum(tick_of(cfg, kGrip, g), "%.0f") + ", m: " + ynum(g) + ", gap_mm: " +
    ynum(std::isfinite(g) ? gripper_gap(g) * 1000.0 : NAN, "%.1f") + "}\n";
  y += "  odom: " + (s.odom_t < 0.0 ? std::string("null") :
    "[" + ynum(s.ox, "%.4f") + ", " + ynum(s.oy, "%.4f") + ", " + ynum(deg(s.oyaw), "%.2f") + "]") + "   # x m, y m, yaw deg\n";
  y += "  map: " + (!s.have_map_pose ? std::string("null") :
    "[" + ynum(s.mx, "%.4f") + ", " + ynum(s.my, "%.4f") + ", " + ynum(deg(s.myaw), "%.2f") + "]") + "\n";
  if (s.wrist_t < 0.0) {
    y += "  wrist: null\n";
    return y;
  }
  y += "  wrist:\n";
  y += "    age_s: " + ynum(wrist_age(s), "%.2f") + "\n";
  y += "    frame: \"" + clean(s.wrist_frame) + "\"\n";
  y += "    detections:" + std::string(s.wrist_dets.empty() ? " []\n" : "\n");
  for (const auto & d : s.wrist_dets) {
    y += "      - {id: \"" + clean(d.id) + "\", kind: \"" + clean(d.kind) + "\", label: \"" + clean(d.label) + "\", shape: \"" + clean(d.shape) +
      "\", confidence: " + ynum(d.conf, "%.3f") + ", position: [" + ynum(d.x) + ", " + ynum(d.y) + ", " + ynum(d.z) +
      "], orientation: [" + ynum(d.qx) + ", " + ynum(d.qy) + ", " + ynum(d.qz) + ", " + ynum(d.qw) + "], yaw_known: " +
      (d.yaw_known ? "true" : "false") + "}\n";
  }
  return y;
}

std::string det_csv_header()
{
  std::string h = "name,time,id,kind,label,shape,confidence,frame,x,y,z,qx,qy,qz,qw,yaw_known,age_s";
  for (const auto & j : kJ5) {h += "," + j + "_rad";}
  for (const auto & j : kJ5) {h += "," + j + "_corr_rad";}
  return h;
}

std::vector<std::string> det_csv_rows(const std::string & name, const std::string & wall, const Snapshot & s)
{
  std::vector<std::string> out;
  if (s.wrist_t < 0.0) {return out;}
  for (const auto & d : s.wrist_dets) {
    std::string r = name + "," + wall + "," + clean(d.id) + "," + clean(d.kind) + "," + clean(d.label) + "," + clean(d.shape) + "," +
      num(d.conf, "%.3f") + "," + clean(s.wrist_frame) + "," + num(d.x) + "," + num(d.y) + "," + num(d.z) + "," + num(d.qx) + "," + num(d.qy) + "," + num(d.qz) +
      "," + num(d.qw) + "," + (d.yaw_known ? "1" : "0") + "," + num(wrist_age(s), "%.2f");
    for (const auto & j : kJ5) {r += "," + num(get(s.joints, j));}
    for (const auto & j : kJ5) {r += "," + num(corr(s, j));}
    out.push_back(r);
  }
  return out;
}

bool append_csv(const std::string & path, const std::string & header, const std::vector<std::string> & rows, std::string & err)
{
  std::error_code ec;
  const bool fresh = !std::filesystem::exists(path, ec) || std::filesystem::file_size(path, ec) == 0;
  std::ofstream f(path, std::ios::app);
  if (!f) {err = path + " 열기 실패"; return false;}
  if (fresh) {f << header << "\n";}
  for (const auto & r : rows) {f << r << "\n";}
  f.close();   // 버퍼를 비운 뒤에 판정 (디스크 가득 · 권한 등은 여기서 드러남)
  if (f.fail()) {err = path + " 쓰기 실패"; return false;}
  return true;
}

bool write_mark(const std::string & dir, const std::string & name, const Snapshot & s, const Config & cfg, std::string & text)
{
  if (s.joints_t < 0.0) {text = "mark 안 함: /joint_states 없음"; return false;}
  if (s.now - s.joints_t > kMarkMaxAgeS) {text = fmt("mark 안 함: 관절 측정이 %.1f s 전 것 (끊김?)", s.now - s.joints_t); return false;}
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {text = "mark 안 함: 폴더 " + dir + " 못 만듦 — " + ec.message(); return false;}
  const std::string wall = wall_clock_text();
  std::string err;
  if (!append_csv(dir + "/poses.csv", mark_csv_header(), {mark_csv_row(name, wall, s, cfg)}, err)) {text = "mark 실패: " + err; return false;}
  const auto dets = det_csv_rows(name, wall, s);
  if (!dets.empty() && !append_csv(dir + "/detections.csv", det_csv_header(), dets, err)) {text = "mark 실패: " + err; return false;}
  const std::string ypath = dir + "/poses.yaml";
  const bool fresh = !std::filesystem::exists(ypath, ec) || std::filesystem::file_size(ypath, ec) == 0;
  std::ofstream y(ypath, std::ios::app);
  if (!y) {text = "mark 실패: " + ypath + " 열기 실패"; return false;}
  if (fresh) {y << "# robot_cli mark — 팔 관절 기록 (rad · deg = /joint_states 엔코더, corrected = /joint_states_corrected 처짐 보정)\n";}
  y << mark_yaml_entry(name, wall, s, cfg);
  y.close();
  if (y.fail()) {text = "mark 실패: " + ypath + " 쓰기 실패"; return false;}
  const PoseInfo p = pose_info(s, cfg);
  text = fmt("mark %s %s", name.c_str(), wall.substr(11, 8).c_str());
  for (const auto & j : kJ5) {text += fmt(" %.0f", tick_of(cfg, j, get(s.joints, j)));}
  text += p.name.empty() ? "" : " (" + p.name + ")";
  text += dets.empty() ? "" : fmt(" · 검출 %zu 개", dets.size());
  return true;
}

// ======================================================================== cycle
bool CycleRun::start(const std::string & slot, int rounds, bool store_first, std::string & err)
{
  if (active_) {err = "이미 반복 중 (" + slot_ + ") — cycle stop 뒤에"; return false;}
  if (rounds < 1 || rounds > 50) {err = "횟수 1~50"; return false;}
  *this = CycleRun{};
  slot_ = slot;
  rounds_ = rounds;
  store_first_ = store_first;
  active_ = true;
  return true;
}

void CycleRun::request_stop()
{
  if (!active_) {return;}
  if (!waiting_) {
    abort("cycle stop");
    return;
  }
  stop_req_ = true;
}

void CycleRun::abort(const std::string & why)
{
  if (!active_) {return;}
  active_ = false;
  waiting_ = false;
  end_text_ = fmt("멈춤 (%s): %d/%d 작업 성공", why.c_str(), ok_, rounds_ * 2) + stats();
}

std::string CycleRun::task_name_now() const
{
  const bool retrieve = (step_ % 2 == 0) != store_first_;
  return retrieve ? "RETRIEVE" : "STORE";
}

bool CycleRun::next(Command & c, double now, const std::string & wall)
{
  if (!active_ || waiting_) {return false;}
  c = Command{};
  c.cmd = Cmd::ARM;
  c.task = task_name_now() == "RETRIEVE" ? ArmTask::Goal::RETRIEVE : ArmTask::Goal::STORE;
  c.slot = slot_;
  waiting_ = true;
  t_sent_ = now;
  wall_sent_ = wall;
  return true;
}

std::string CycleRun::on_done(int code, const std::string & text, double now, const std::string & phases, std::string * csv_row)
{
  if (!active_ || !waiting_) {return "";}
  const std::string task = task_name_now();
  const double secs = now - t_sent_;
  const int round = step_ / 2 + 1;
  waiting_ = false;
  ++step_;
  if (code == 0) {
    ++ok_;
    secs_[task].push_back(secs);
  }
  if (csv_row) {
    *csv_row = fmt("%d,%s,%s,", round, task.c_str(), slot_.c_str()) + wall_sent_ + fmt(",%.2f,%s,", secs, code == 0 ? "ok" : "fail") +
      clean(phases) + "," + clean(text);
  }
  const std::string line = fmt("%d/%d %s %s %s %.1f s · ", round, rounds_, task.c_str(), slot_.c_str(), code == 0 ? "성공" : "실패", secs) +
    phases;
  if (code != 0) {
    abort("실패");
  } else if (step_ >= rounds_ * 2) {
    active_ = false;
    end_text_ = fmt("끝: %d/%d 작업 성공", ok_, rounds_ * 2) + stats();
  } else if (stop_req_) {
    abort("cycle stop");
  }
  return line;
}

std::string CycleRun::stats() const
{
  std::string out;
  for (const auto & kv : secs_) {
    if (kv.second.empty()) {continue;}
    double sum = 0.0, lo = kv.second[0], hi = kv.second[0];
    for (double v : kv.second) {
      sum += v;
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
    out += fmt(" · %s 평균 %.1f s (%.1f~%.1f)", kv.first.c_str(), sum / kv.second.size(), lo, hi);
  }
  return out;
}

std::string CycleRun::status(double now) const
{
  if (!active_) {return end_text_.empty() ? "" : slot_ + " " + end_text_;}
  std::string t = fmt("%s %d/%d 회 · ", slot_.c_str(), std::min(step_ / 2 + 1, rounds_), rounds_);
  t += waiting_ ? fmt("%s 진행 %.1f s", task_name_now().c_str(), now - t_sent_) : task_name_now() + " 보낼 차례";
  t += fmt(" · 성공 %d", ok_) + stats();
  if (stop_req_) {t += " · 이 작업 뒤 멈춤";}
  return t;
}

std::string CycleRun::csv_header() {return "round,task,slot,start,secs,result,phases,message";}

}  // namespace robot_cli
