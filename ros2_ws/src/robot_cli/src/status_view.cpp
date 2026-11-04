// status_view.cpp — 0 쪽 (덩어리마다 배지 한 줄) + 1~7 쪽이 같이 쓰는 조각 · 표, 설명은 status_view.hpp. 1~7 쪽은 pages.cpp
//   배지 색: [정상] 등 초록 · [주의] [잠금] 노랑 · [문제] [E-STOP] 빨강 · [꺼짐] [없음] 회색
//   lazy_fold (control.yaml): STORE · PLACE 가 P0 (받침 바로 위) 에서 끝나고 바퀴 잠금 유지 → 주행 전 HOME (fold 경로)

#include "robot_cli/status_view.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <set>

#include "view_parts.hpp"

namespace robot_cli
{

const std::vector<std::string> kArmJoints = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_left_joint"};
const std::vector<std::string> kWatchNodes = {
  "/controller_manager", "/control_master", "/arm_state_publisher", "/robot_state_publisher",
  "/slam_toolbox", "/amcl", "/pose_guard", "/scan_merger", "/bt_navigator", "/cmd_vel_hold",
  "/mission_manager", "/perception"};
const char * const kPageNames[kPages] = {"전체 요약", "위치·SLAM", "팔", "주행·Nav2", "인지", "미션", "진단", "실험"};

int text_width(const std::string & s)
{
  int w = 0;
  for (std::size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    uint32_t cp = c;
    int n = 1;
    if ((c >> 5) == 6) {cp = c & 0x1f; n = 2;} else if ((c >> 4) == 14) {cp = c & 0x0f; n = 3;} else if ((c >> 3) == 30) {
      cp = c & 0x07;
      n = 4;
    }
    for (int k = 1; k < n && i + k < s.size(); ++k) {cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3f);}
    i += n;
    const bool wide = (cp >= 0x1100 && cp <= 0x115F) || (cp >= 0x2E80 && cp <= 0xA4CF) || (cp >= 0xAC00 && cp <= 0xD7A3) ||
      (cp >= 0xF900 && cp <= 0xFAFF) || (cp >= 0xFE30 && cp <= 0xFE4F) || (cp >= 0xFF00 && cp <= 0xFF60) ||
      (cp >= 0xFFE0 && cp <= 0xFFE6);
    w += wide ? 2 : 1;
  }
  return w;
}

namespace parts
{
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

double deg(double rad) {return rad * 180.0 / M_PI;}
double nz(double v) {return std::fabs(v) < 5e-4 ? 0.0 : v;}

std::string age(const Snapshot & s, double t)
{
  if (t < 0.0) {return "없음";}
  return fmt("%.1f s", s.now - t);
}

Level age_level(const Snapshot & s, double t, Level ok)
{
  return t < 0.0 || s.now - t > kStaleS ? Level::WARN : ok;
}

bool active(int8_t st) {return st >= 1 && st <= 3;}

std::string goals_text(const std::vector<GoalView> & goals)
{
  std::string out;
  for (const auto & g : goals) {
    if (active(g.status)) {out += (out.empty() ? "" : ", ") + g.id + " " + goal_status_name(g.status);}
  }
  if (out.empty() && !goals.empty()) {out = "마지막 " + goals.back().id + " " + goal_status_name(goals.back().status);}
  return out.empty() ? "없음" : out;
}

bool any_active(const std::vector<GoalView> & goals)
{
  for (const auto & g : goals) {
    if (active(g.status)) {return true;}
  }
  return false;
}

bool has_node(const Snapshot & s, const std::string & n) {return s.nodes.count(n) > 0;}

std::string pad(const std::string & s, int w)
{
  const int n = text_width(s);
  return n >= w ? s : s + std::string(w - n, ' ');
}

namespace
{
std::string rjust(const std::string & s, int w)
{
  const int n = text_width(s);
  return n >= w ? s : std::string(w - n, ' ') + s;
}
}  // namespace

void head(Lines & L, const std::string & t)
{
  if (!L.empty() && !L.back().text.empty()) {L.push_back({"", Level::NORMAL});}
  L.push_back({"── " + t + " ──", Level::HEAD});
}

namespace
{
std::string label_cell(const std::string & label)
{
  const std::string a = pad(label, kLabelW);
  return text_width(label) >= kLabelW ? a + " " : a;   // 긴 이름도 붙지 않게
}
}  // namespace

void row(Lines & L, const std::string & label, const std::string & text, Level lv)
{
  L.push_back({label_cell(label) + text, lv});
}

void block(Lines & L, const std::string & label, const std::string & badge, Level lv, const std::string & text)
{
  const std::string a = label_cell(label), b = "[" + badge + "]";
  Line l;
  l.text = a + pad(b, kBadgeW) + text;
  while (!l.text.empty() && l.text.back() == ' ') {l.text.pop_back();}
  l.level = lv;
  l.mark_from = a.size();
  l.mark_len = b.size();
  L.push_back(l);
}

PoseView pose_view(const Snapshot & s, const Config & cfg)
{
  PoseView v;
  v.pose = nearest_pose(cfg.arm, s.joints, &v.diff);
  v.near = v.pose && v.diff <= cfg.arm.near_tol;
  v.fold = folded(cfg.arm, s.joints, &v.worst, &v.worst_diff);
  return v;
}

const TaskTiming * running_task(const Snapshot & s)
{
  for (auto it = s.arm_hist.rbegin(); it != s.arm_hist.rend(); ++it) {
    if (it->t1 < 0.0 && active(it->status)) {return &*it;}
  }
  return nullptr;
}

const TaskTiming * last_done_task(const Snapshot & s)
{
  for (auto it = s.arm_hist.rbegin(); it != s.arm_hist.rend(); ++it) {
    if (it->t1 >= 0.0) {return &*it;}
  }
  return nullptr;
}

std::string task_label(const TaskTiming & h) {return h.what.empty() ? std::string("다른 쪽 작업") : h.what;}

std::string result_word(int8_t status)
{
  switch (status) {
    case 4: return "성공";
    case 5: return "취소";
    case 6: return "실패";
    default: return active(status) ? "진행 중" : "?";
  }
}

// ---------------------------------------------------------------- 배지 줄
void wheels_block(Lines & L, const Snapshot & s, const Config & cfg)
{
  if (s.mode_t < 0.0) {
    block(L, "바퀴", "문제", Level::BAD, "차례 신호 없음 — control_master 꺼짐? 바퀴 잠김");
    return;
  }
  const double a = s.now - s.mode_t;
  if (a > kModeFreshS) {
    block(L, "바퀴", "문제", Level::BAD, fmt("차례 신호가 %.1f s 끊김 — 바퀴 잠김", a));
    return;
  }
  const std::string extra = s.ready ? "" : " · 팔이 새 작업을 못 받음 (하드웨어 확인)";
  if (s.mode == 1) {
    block(L, "바퀴", "주행 가능", Level::GOOD, "drive · teleop · nav 됨" + extra);
    return;
  }
  const TaskTiming * last = last_done_task(s);
  const PoseView pv = pose_view(s, cfg);
  std::string why;
  if (s.mode == 2 || any_active(s.arm_goals) || running_task(s)) {
    why = "팔 작업 중";
  } else if (s.reason == "STOPPING") {
    why = "바퀴 세우는 중 (팔 작업 시작)";
  } else if (last && last->status != 4) {
    why = "팔 작업 " + result_word(last->status) + " 뒤 — home 필요 (도중에 멈췄으면 home force)";
  } else if (pv.near && pv.pose->name != "받침") {
    why = "팔 작업 뒤 (" + pv.pose->name + ") — 주행 전 home 필요";
  } else if (pv.fold) {
    why = "접혀 있음 — home 하면 풀림";
  } else {
    why = "home 필요 (팔 주변을 보고)";
  }
  block(L, "바퀴", "잠금", Level::WARN, why + extra);
}

// " · 그리퍼 15 mm" · " · 그리퍼 쥠 (0.32 A)" — 쥠 = 반쯤 닫힘 (control_master 와 같은 식), 전류는 effort 칸
std::string grip_text(const Snapshot & s, const Config & cfg)
{
  const auto g = s.joints.find("gripper_left_joint");
  if (g == s.joints.end() || !std::isfinite(g->second)) {return "";}
  const auto c = s.current.find("gripper_left_joint");
  const std::string amp = c != s.current.end() && std::isfinite(c->second) && c->second != 0.0 ? fmt(" %.2f A", std::fabs(c->second)) : "";
  if (gripper_holding(cfg.arm, g->second)) {return " · 그리퍼 쥠 (" + fmt("%.0f mm", gripper_gap(g->second) * 1000.0) + amp + ")";}
  return fmt(" · 그리퍼 %.0f mm", gripper_gap(g->second) * 1000.0) + amp;
}

void arm_block(Lines & L, const Snapshot & s, const Config & cfg)
{
  if (s.joints_t < 0.0) {
    block(L, "팔", "없음", Level::OFF, "관절 측정 없음 (하드웨어 꺼짐?)");
    return;
  }
  if (s.now - s.joints_t > kStaleS) {
    block(L, "팔", "주의", Level::WARN, fmt("관절 측정이 %.0f s 끊김", s.now - s.joints_t));
    return;
  }
  const std::string grip = grip_text(s, cfg);
  const TaskTiming * run = running_task(s);
  if (run || any_active(s.arm_goals)) {
    std::string t = run ? task_label(*run) : "작업";
    if (s.arm_phase_t >= 0.0 && !s.arm_phase.empty()) {t += " · " + s.arm_phase;}
    if (run) {t += fmt(" · %.0f s", s.now - run->t0);}
    block(L, "팔", "작업 중", Level::GOOD, t + grip);
    return;
  }
  const PoseView pv = pose_view(s, cfg);
  std::string badge, t;
  Level lv = Level::GOOD;
  if (cfg.arm.poses.empty() || !pv.pose) {
    badge = "?";
    lv = Level::NORMAL;
    t = cfg.arm.poses.empty() ? "자세 설정 못 읽음" : "관절 측정이 빠짐";
  } else if (pv.fold) {
    badge = "받침";
    t = "접힘";
  } else if (pv.near) {
    const std::string & n = pv.pose->name;
    badge = n == "LOOK_BARCODE" ? "바코드" : n;
    t = n == "P0" ? (cfg.arm.lazy_fold ? "받침 바로 위 (lazy_fold 끝 자리)" : "받침 바로 위") :
      n == "기준점" ? "퍽 쥔 자세 — 다음 STORE · PLACE" : n == "받침" ? "받침 근처" : "관찰 자세";
  } else {
    badge = "모름";
    lv = Level::WARN;
    t = "이름 자세 아님 — 사람이 보고 home";
  }
  if (const TaskTiming * last = last_done_task(s)) {
    t += " · 마지막 " + task_label(*last) + " " + result_word(last->status) + fmt(" %.1f s", last->t1 - last->t0);
    if (last->status != 4) {lv = Level::WARN;}
  }
  block(L, "팔", badge, lv, t + grip);
}

void limit_block(Lines & L, const Snapshot & s, const Config & cfg)
{
  Level worst = Level::NORMAL;
  const auto w = limit_warnings(s, cfg, worst);
  if (w.empty()) {return;}
  std::string t;
  for (const auto & x : w) {t += (t.empty() ? "" : " · ") + x;}
  block(L, "관절", worst == Level::BAD ? "문제" : "주의", worst, t);
}

void battery_block(Lines & L, const Snapshot & s, const Config & cfg)
{
  std::string t;
  double vmin = 1e9;
  for (const auto & b : s.battery) {
    if (s.now - b.second.t > 60.0) {continue;}
    vmin = std::min(vmin, b.second.mean);
    t += fmt("%s%s %.1f V (최저 %.1f)", t.empty() ? "" : " · ", b.first.c_str(), b.second.mean, b.second.min);
  }
  if (t.empty()) {
    block(L, "배터리", "모름", Level::OFF, "OpenCR 로그에 전압 없음 (mock · 켜는 중)");
  } else if (vmin < cfg.battery_stop) {
    block(L, "배터리", "문제", Level::BAD, t + fmt(" — %.1f V 아래 (곧 멈춤)", cfg.battery_stop));
  } else if (vmin < cfg.battery_warn) {
    block(L, "배터리", "주의", Level::WARN, t + fmt(" — %.1f V 아래, 충전", cfg.battery_warn));
  } else {
    block(L, "배터리", "정상", Level::GOOD, t);
  }
}

void manual_block(Lines & L, const Snapshot & s, bool always)
{
  const bool clock_bad = s.odom_t >= 0.0 && std::fabs(s.clock_off) > kClockTolS;
  if (s.estop) {
    block(L, "수동", "E-STOP", Level::BAD, "새 팔 · 주행 명령 거절 중 — estop off 로 풀기");
  } else if (clock_bad) {
    block(L, "수동", "문제", Level::BAD, fmt("로봇과 시계가 %.2f s 다름 — drive · teleop 막힘 (시각 동기화)", s.clock_off));
  } else if (s.teleop) {
    block(L, "수동", "teleop", Level::GOOD, "키보드 주행 중");
  } else if (s.driving) {
    block(L, "수동", "drive", Level::GOOD, "drive 명령으로 주행 중");
  } else if (always) {
    block(L, "수동", "정상", Level::GOOD, "e-stop 아님 · 수동 주행 안 함");
  }
}

void loc_block(Lines & L, const Snapshot & s, const Config & cfg)
{
  if (s.loc_mode.empty() && !s.have_map_pose && s.health_t < 0.0) {
    block(L, "위치", "꺼짐", Level::OFF, "SLAM · AMCL 안 켜짐 (odom 만)");
    return;
  }
  std::string t = s.loc_mode == "SLAM mapping" ? "SLAM 지도 만들기" : s.loc_mode == "SLAM localization" ? "SLAM 위치 찾기" :
    s.loc_mode.empty() ? "위치 추정" : s.loc_mode;
  std::string badge = "정상";
  Level lv = Level::GOOD;
  if (s.health_t >= 0.0 && s.health_level == 1) {badge = "주의"; lv = Level::WARN;}
  if (s.health_t >= 0.0 && s.health_level >= 2) {badge = "문제"; lv = Level::BAD;}
  if (lv != Level::GOOD && !s.health_msg.empty()) {t += " · " + s.health_msg;}
  if (s.have_map_pose) {
    double d = 0.0;
    const NamedStop * n = nearest_stop(cfg.stops, s.mx, s.my, &d);
    if (n) {t += fmt(" · 가까운 정차 %s %.2f m", n->name.c_str(), d);}
    if (s.map_age > kMapStaleS) {
      t += fmt(" · 지도 위치가 %.0f s 전 것 (TF 끊김?)", s.map_age);
      if (lv == Level::GOOD) {badge = "주의"; lv = Level::WARN;}
    }
  } else {
    t += " · 지도 위의 위치 없음";
    if (lv == Level::GOOD) {badge = "주의"; lv = Level::WARN;}
  }
  block(L, "위치", badge, lv, t);
}

void nav_block(Lines & L, const Snapshot & s)
{
  if (s.nav_active < 0 && !has_node(s, "/bt_navigator")) {
    block(L, "Nav2", "꺼짐", Level::OFF, "nav 명령 안 됨");
    return;
  }
  if (s.nav_active != 1) {
    block(L, "Nav2", "주의", Level::WARN, "켜는 중 (아직 active 아님)");
    return;
  }
  const bool hold = s.hold_t >= 0.0 && s.hold;
  if (any_active(s.nav_goals)) {
    std::string t = s.nav_fb_t >= 0.0 ? fmt("남은 %.1f m · 약 %.0f s", s.dist_left, s.eta_s) : "목표 받음";
    if (s.recoveries > 0) {t += fmt(" · 복구 %d 번", s.recoveries);}
    if (hold) {t += " · 미션 hold 로 멈춤";}
    block(L, "Nav2", hold ? "주의" : "주행 중", hold ? Level::WARN : Level::GOOD, t);
    return;
  }
  if (hold) {
    block(L, "Nav2", "주의", Level::WARN, "미션 hold — Nav2 속도 막힘");
    return;
  }
  block(L, "Nav2", "정상", Level::GOOD, "대기");
}

void percep_block(Lines & L, const Snapshot & s)
{
  if (s.light_t < 0.0 && s.wrist_t < 0.0 && s.wide_t < 0.0 && s.text_t < 0.0) {
    block(L, "인지", "꺼짐", Level::OFF, "신호등 · 손목 · 광각 없음");
    return;
  }
  const bool stale = (s.light_t >= 0.0 && s.now - s.light_t > kStaleS) || (s.wrist_t >= 0.0 && s.now - s.wrist_t > kStaleS);
  std::string t = s.light_t < 0.0 ? "신호등 없음" : "신호등 " + light_name(s.light) + (s.light_visible ? "" : " (안 보임)");
  t += s.wrist_t < 0.0 ? " · 손목 없음" : fmt(" · 손목 검출 %d 개", s.wrist_n);
  t += s.wide_t < 0.0 ? " · 광각 없음" : fmt(" · 광각 검출 %d 개", s.wide_n);
  if (s.text_t >= 0.0) {t += " · 바코드 \"" + s.task_text + "\"";}
  block(L, "인지", stale ? "주의" : "정상", stale ? Level::WARN : Level::GOOD, t + (stale ? " · 끊김" : ""));
}

void mission_block(Lines & L, const Snapshot & s)
{
  if (s.mission_t < 0.0) {
    block(L, "미션", "꺼짐", Level::OFF, "mission_manager 안 켜짐");
    return;
  }
  if (s.m_fault) {
    block(L, "미션", "문제", Level::BAD, s.m_state + (s.m_detail.empty() ? "" : " · " + s.m_detail));
    return;
  }
  std::string t = s.m_state + (s.m_phase.empty() ? "" : " " + s.m_phase) + fmt(" · 퍽 %d/%d", s.pucks_done, s.pucks_total);
  int loaded = 0;
  for (const auto & c : s.cargo_slots) {loaded += c.occupied ? 1 : 0;}
  if (s.cargo_t >= 0.0) {t += fmt(" · 적재 %d 칸", loaded);}
  if (s.hold_t >= 0.0 && s.hold) {t += " · hold";}
  const bool stale = s.now - s.mission_t > kStaleS;
  block(L, "미션", stale ? "주의" : "정상", stale ? Level::WARN : Level::GOOD, t + (stale ? " · 끊김" : ""));
}

void hardware_block(Lines & L, const Snapshot & s)
{
  if (s.hw_t < 0.0 && s.ctrl_t < 0.0) {
    block(L, "하드웨어", "없음", Level::OFF, "controller_manager 답 없음");
    return;
  }
  const double old = std::max(s.hw_t < 0.0 ? 1e9 : s.now - s.hw_t, s.ctrl_t < 0.0 ? 1e9 : s.now - s.ctrl_t);
  if (old > kPollStaleS) {   // 옛 active 목록을 지금처럼 보이지 않게
    block(L, "하드웨어", "주의", Level::WARN, fmt("controller_manager 답이 %.0f s 끊김 — 보드 · 컨트롤러 상태 모름", std::min(old, 999.0)));
    return;
  }
  std::string bad;
  for (const auto & h : s.hardware) {
    if (h.second != "active") {bad += (bad.empty() ? "" : " · ") + h.first + " " + h.second;}
  }
  for (const auto & c : s.controllers) {
    if (c.second != "active") {bad += (bad.empty() ? "" : " · ") + c.first + " " + c.second;}
  }
  int err = 0, warn = 0;
  for (const auto & d : s.diags) {
    err += d.level >= 2 ? 1 : 0;
    warn += d.level == 1 ? 1 : 0;
  }
  const std::string scan = s.scan_t < 0.0 ? " · 라이다 없음" : s.now - s.scan_t > kStaleS ? " · 라이다 끊김" : "";
  if (!bad.empty() || err > 0) {
    block(L, "하드웨어", "문제", Level::BAD, bad + (err > 0 ? fmt("%s진단 오류 %d", bad.empty() ? "" : " · ", err) : ""));
  } else if (warn > 0) {
    block(L, "하드웨어", "주의", Level::WARN, fmt("진단 경고 %d · 6 쪽에서 보기", warn) + scan);
  } else {
    block(L, "하드웨어", "정상", Level::GOOD, fmt("보드 %zu · 컨트롤러 %zu", s.hardware.size(), s.controllers.size()) + scan);
  }
}

// ---------------------------------------------------------------- 표
void joint_table(Lines & L, const Snapshot & s, const Config & cfg, bool wide)
{
  head(L, "관절 (측정 " + age(s, s.joints_t) + " 전 · 처짐 보정 " + age(s, s.corrected_t) + " 전)");
  std::string h = pad("관절", 9) + rjust("눈금", 6) + rjust("[rad]", 10) + rjust("[°]", 9) + rjust("전류[A]", 9);
  if (wide) {h += rjust("보정[rad]", 11) + rjust("보정[°]", 9) + rjust("보정눈금", 9) + rjust("처짐[°]", 9) + rjust("받침과[°]", 11);}
  L.push_back({h, Level::NORMAL});
  const std::vector<double> * home = cfg.arm.poses.empty() ? nullptr : &cfg.arm.poses.front().q;
  for (std::size_t k = 0; k < 5; ++k) {
    const std::string & j = kArmJoints[k];
    const auto it = s.joints.find(j);
    if (it == s.joints.end() || !std::isfinite(it->second)) {
      L.push_back({pad(j, 9) + "측정 없음", Level::WARN});
      continue;
    }
    const double q = it->second;
    const auto c = cfg.cal.find(j);
    const auto a = s.current.find(j);
    std::string r = pad(j, 9) + rjust(c != cfg.cal.end() ? fmt("%.0f", std::round(joint_to_tick(c->second, q))) : "?", 6) +
      rjust(fmt("%+.4f", q), 10) + rjust(fmt("%+.2f", deg(q)), 9) +
      rjust(a != s.current.end() && std::isfinite(a->second) ? fmt("%+.2f", a->second) : "-", 9);
    if (wide) {
      const auto kc = s.corrected.find(j);
      if (kc != s.corrected.end() && std::isfinite(kc->second)) {
        r += rjust(fmt("%+.4f", kc->second), 11) + rjust(fmt("%+.2f", deg(kc->second)), 9) +
          rjust(c != cfg.cal.end() ? fmt("%.0f", std::round(joint_to_tick(c->second, kc->second))) : "?", 9) +
          rjust(fmt("%+.2f", deg(kc->second - q)), 9);
      } else {
        r += rjust("-", 11) + rjust("-", 9) + rjust("-", 9) + rjust("-", 9);
      }
      r += rjust(home && k < home->size() ? fmt("%+.2f", deg(q - (*home)[k])) : "-", 11);
    }
    L.push_back({r, age_level(s, s.joints_t)});
  }
  const auto g = s.joints.find("gripper_left_joint");
  L.push_back({pad("그리퍼", 9) + rjust("눈금", 6) + rjust("[m]", 10) + rjust("간격[mm]", 9) + rjust("전류[A]", 9) + "  상태", Level::NORMAL});
  if (g == s.joints.end() || !std::isfinite(g->second)) {
    L.push_back({pad("", 9) + "측정 없음", Level::WARN});
  } else {
    const auto c = cfg.cal.find("gripper_left_joint");
    const auto a = s.current.find("gripper_left_joint");
    const bool hold = gripper_holding(cfg.arm, g->second);
    const bool open = std::isfinite(cfg.arm.grip_open) && g->second >= cfg.arm.grip_open - 0.004;
    L.push_back({pad("", 9) + rjust(c != cfg.cal.end() ? fmt("%.0f", std::round(joint_to_tick(c->second, g->second))) : "?", 6) +
      rjust(fmt("%+.4f", g->second), 10) + rjust(fmt("%.1f", gripper_gap(g->second) * 1000.0), 9) +
      rjust(a != s.current.end() && std::isfinite(a->second) ? fmt("%+.2f", a->second) : "-", 9) +
      (hold ? "  퍽 쥠 (반쯤 닫힘)" : open ? "  열림" : "  닫힘"), hold ? Level::GOOD : age_level(s, s.joints_t)});
  }
}

void pose_table(Lines & L, const Snapshot & s)
{
  L.push_back({pad("", 9) + rjust("x[m]", 8) + rjust("y[m]", 9) + rjust("yaw[°]", 9) + rjust("나이[s]", 9), Level::NORMAL});
  auto pose = [&](const char * name, bool have, double x, double y, double yaw, double t, Level lv) {
      if (!have) {
        L.push_back({pad(name, 9) + "없음", Level::OFF});
        return;
      }
      L.push_back({pad(name, 9) + rjust(fmt("%+.3f", nz(x)), 8) + rjust(fmt("%+.3f", nz(y)), 9) + rjust(fmt("%+.1f", nz(deg(yaw))), 9) +
        rjust(t < 0.0 ? "-" : fmt("%.1f", s.now - t), 9), lv});
    };
  pose("odom", s.odom_t >= 0.0, s.ox, s.oy, s.oyaw, s.odom_t, age_level(s, s.odom_t));
  pose("map (TF)", s.have_map_pose, s.mx, s.my, s.myaw, s.now - std::max(0.0, s.map_age),
    s.map_age > kMapStaleS ? Level::WARN : Level::NORMAL);
  pose("amcl", s.amcl_t >= 0.0, s.ax, s.ay, s.ayaw, s.amcl_t, Level::NORMAL);
  if (s.odom_t >= 0.0) {
    L.push_back({pad("", 9) + rjust("vx[m/s]", 8) + rjust("vy[m/s]", 9) + rjust("wz[rad/s]", 11), Level::NORMAL});
    L.push_back({pad("속도", 9) + rjust(fmt("%+.3f", nz(s.ovx)), 8) + rjust(fmt("%+.3f", nz(s.ovy)), 9) + rjust(fmt("%+.2f", nz(s.owz)), 11),
      Level::NORMAL});
  }
  if (s.amcl_t >= 0.0) {row(L, "amcl σ", fmt("%.3f m · %.1f°", s.amcl_std_xy, deg(s.amcl_std_yaw)));}
}

void arm_timings(Lines & L, const Snapshot & s, std::size_t n)
{
  if (s.arm_hist.empty()) {
    L.push_back({"이 CLI 를 켠 뒤 본 팔 작업 없음", Level::OFF});
    return;
  }
  L.push_back({pad("id", 10) + pad("작업", 18) + pad("결과", 9) + rjust("걸린[s]", 8) + "  phase[s]", Level::NORMAL});
  std::size_t shown = 0;
  for (auto it = s.arm_hist.rbegin(); it != s.arm_hist.rend() && shown < n; ++it, ++shown) {
    const TaskTiming & h = *it;
    const bool done = h.t1 >= 0.0;
    const double end = done ? h.t1 : s.now;
    L.push_back({pad(h.id, 10) + pad(task_label(h), 18) + pad(result_word(h.status), 9) + rjust(fmt("%.1f", end - h.t0), 8) + "  " +
      phase_text(h, end), h.status == 4 ? Level::NORMAL : active(h.status) ? Level::GOOD : Level::WARN});
  }
}

void component_table(Lines & L, const Snapshot & s)
{
  L.push_back({pad("종류", 10) + pad("이름", 28) + "상태", Level::NORMAL});
  auto group = [&](const char * kind, const std::vector<std::pair<std::string, std::string>> & v, double t) {
      if (t < 0.0) {
        L.push_back({pad(kind, 10) + "controller_manager 답 없음", Level::OFF});
        return;
      }
      for (const auto & c : v) {
        L.push_back({pad(kind, 10) + pad(c.first, 28) + c.second, c.second == "active" ? Level::NORMAL : Level::WARN});
      }
    };
  group("보드", s.hardware, s.hw_t);
  group("컨트롤러", s.controllers, s.ctrl_t);
}

void diagnostics(Lines & L, const Snapshot & s)
{
  if (s.diag_t < 0.0) {
    L.push_back({"/diagnostics 없음", Level::OFF});
    return;
  }
  L.push_back({pad("수준", 7) + "이름 — 내용", Level::NORMAL});
  for (const auto & d : s.diags) {
    static const char * lv[] = {"OK", "WARN", "ERROR", "STALE"};
    L.push_back({pad(d.level >= 0 && d.level <= 3 ? lv[d.level] : "?", 7) + d.name + " — " + d.message,
      d.level == 0 ? Level::NORMAL : d.level == 1 ? Level::WARN : Level::BAD});
  }
}

Lines wrap(const Lines & in, int max)
{
  Lines out;
  const std::string sep = " · ";
  for (const auto & l : in) {
    if (l.level == Level::HEAD || text_width(l.text) <= max) {
      out.push_back(l);
      continue;
    }
    // " · " 에서 나눔: 첫 줄은 그대로 (배지 포함), 다음 줄은 이름 칸만큼 들여씀
    std::vector<std::string> parts;
    for (std::size_t a = 0;;) {
      const std::size_t b = l.text.find(sep, a);
      parts.push_back(l.text.substr(a, b == std::string::npos ? std::string::npos : b - a));
      if (b == std::string::npos) {break;}
      a = b + sep.size();
    }
    Line cur = l;
    cur.text = parts[0];
    bool first = true;
    for (std::size_t i = 1; i < parts.size(); ++i) {
      if (text_width(cur.text + sep + parts[i]) <= max) {
        cur.text += sep + parts[i];
        continue;
      }
      out.push_back(cur);
      if (first && l.mark_len > 0) {cur.level = Level::NORMAL;}   // 배지 줄의 이어지는 줄은 보통 글
      first = false;
      cur.mark_from = cur.mark_len = 0;
      cur.text = std::string(kLabelW + (l.mark_len > 0 ? kBadgeW : 0), ' ') + parts[i];
    }
    out.push_back(cur);
  }
  return out;
}
}  // namespace parts

std::vector<std::string> stops_text(const std::vector<NamedStop> & stops)
{
  std::vector<std::string> out;
  if (stops.empty()) {out.push_back("정차 자세 없음 (dice_map 을 못 읽음)");}
  for (const auto & s : stops) {
    out.push_back(parts::fmt("%-8s map x %+.3f  y %+.3f  yaw %+6.1f°", s.name.c_str(), parts::nz(s.pose.x), parts::nz(s.pose.y),
      parts::nz(parts::deg(s.pose.yaw))));
  }
  return out;
}

std::vector<std::string> loc_hint()
{
  return {
    "SLAM · AMCL · 주행 모드는 실행 중에 바꾸는 인터페이스가 없음 → launch 인자로 다시 띄움",
    "  SLAM 지도 만들기 : ros2 launch navigation slam.launch.py slam_mode:=mapping",
    "  SLAM 위치 찾기   : ros2 launch navigation slam.launch.py slam_mode:=localization map:=$HOME/maps/<이름>",
    "  Nav2 (SLAM 위치) : ros2 launch navigation nav2.launch.py            (localization:=slam 기본)",
    "  Nav2 + AMCL      : ros2 launch navigation nav2.launch.py localization:=amcl map:=<지도.yaml> initial_pose:=x,y,yaw°",
    "  조향 정책        : ros2 launch bringup robot.launch.py steering_policy:=coherent|nearest",
  };
}

std::string goal_status_name(int8_t status)
{
  static const char * names[] = {"UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING", "SUCCEEDED", "CANCELED", "ABORTED"};
  return status >= 0 && status <= 6 ? names[status] : parts::fmt("%d", status);
}

std::string light_name(uint8_t state)
{
  static const char * names[] = {"UNKNOWN", "RED", "YELLOW", "GREEN", "OFF"};
  return state <= 4 ? names[state] : parts::fmt("%u", state);
}

std::string mode_name(uint8_t mode)
{
  static const char * names[] = {"STOP", "DRIVE", "ARM"};
  return mode <= 2 ? names[mode] : parts::fmt("%u", mode);
}

std::string phase_text(const TaskTiming & h, double t_end)
{
  std::string out;
  for (std::size_t i = 0; i < h.phases.size(); ++i) {
    const double until = i + 1 < h.phases.size() ? h.phases[i + 1].second : t_end;
    out += parts::fmt("%s%s %.1f", out.empty() ? "" : " · ", h.phases[i].first.c_str(), until - h.phases[i].second);
  }
  return out.empty() ? "phase 없음" : out;
}

DriveFacts drive_facts(const Snapshot & s)
{
  DriveFacts f;
  f.now = s.now;
  f.estop = s.estop;
  f.odom_t = s.odom_t;
  f.clock_off = s.clock_off;
  f.mode_t = s.mode_t;
  f.mode = s.mode;
  f.nav2_up = s.nodes.count("/bt_navigator") > 0;
  f.nodes_known = s.nodes.count("/controller_manager") > 0;
  return f;
}

std::string drive_block_reason(const Snapshot & s, bool force) {return drive_block_reason(drive_facts(s), force);}

std::string drive_block_reason(const DriveFacts & f, bool force)
{
  if (f.estop) {return "E-STOP 중 — estop off 뒤에";}
  // 시계 차이는 /odom stamp 로 잼 → odom 이 없거나 끊기면 잴 수 없음 = 주행 안 함 (force 로도 못 넘김)
  if (f.odom_t < 0.0) {return "/odom 없음 — 로봇과 시계 차이를 잴 수 없음 (swerve_controller 확인)";}
  if (f.now - f.odom_t > 1.0) {return parts::fmt("/odom 이 %.1f s 끊김 — 시계 차이를 잴 수 없음", f.now - f.odom_t);}
  if (std::fabs(f.clock_off) > kClockTolS) {
    return parts::fmt("로봇과 시계 차이 %+.2f s — swerve_controller 가 /cmd_vel 을 버림 (chrony 등으로 시각 동기화)", f.clock_off);
  }
  if (force) {return "";}
  if (!f.nodes_known) {return "로봇 노드 목록을 아직 못 받음 (Nav2 가 떠 있는지 모름) — 잠시 뒤";}
  if (f.nav2_up) {return "Nav2 가 떠 있음 — Nav2 중 teleop 금지 (nav2.launch.py 운영 규칙). 정말이면 force";}
  if (f.mode_t < 0.0) {return "/control/motion_mode 없음 → swerve_controller 가 바퀴를 안 움직임 (arm:=false 면 force)";}
  const double a = f.now - f.mode_t;
  if (f.mode == 1 && a > kModeFreshS) {
    return parts::fmt("차례 DRIVE 가 %.1f s 끊김 → swerve_controller 가 잠금 (control_master 확인)", a);
  }
  if (f.mode != 1) {
    return "바퀴 잠금 (차례 " + mode_name(f.mode) + ") — home 성공 뒤 주행 (lazy_fold 면 STORE · PLACE 뒤에도 home)";
  }
  return "";
}

std::vector<std::string> limit_warnings(const Snapshot & s, const Config & cfg, Level & worst)
{
  std::vector<std::string> out;
  worst = Level::NORMAL;
  for (const auto & j : kArmJoint5) {
    const auto q = s.joints.find(j);
    const auto lim = s.limits.find(j);
    if (q == s.joints.end() || !std::isfinite(q->second) || lim == s.limits.end()) {continue;}
    const double lo = lim->second.first, hi = lim->second.second;
    const bool upper = hi - q->second < q->second - lo;
    const double margin = upper ? hi - q->second : q->second - lo;
    if (margin >= kLimitWarnRad) {continue;}
    const auto c = cfg.cal.find(j);
    const std::string ticks = c == cfg.cal.end() ? "" : parts::fmt(" (눈금 %.0f · 한계 %.0f)",
      std::round(joint_to_tick(c->second, q->second)), std::round(joint_to_tick(c->second, upper ? hi : lo)));
    if (margin < 0.0) {
      out.push_back(parts::fmt("%s %s 한계를 %.1f° 넘음", j.c_str(), upper ? "위쪽" : "아래쪽", parts::deg(-margin)) + ticks);
      worst = Level::BAD;
    } else {
      out.push_back(parts::fmt("%s %s 한계까지 %.1f°", j.c_str(), upper ? "위쪽" : "아래쪽", parts::deg(margin)) + ticks);
      if (worst != Level::BAD) {worst = Level::WARN;}
    }
  }
  return out;
}

std::vector<CheckItem> preflight(const Snapshot & s, const Config & cfg, const std::string & domain)
{
  using parts::fmt;
  std::vector<CheckItem> v;
  auto add = [&v](const std::string & n, Level l, const std::string & d) {v.push_back({n, l, d});};
  auto fresh = [&s](double t, double max) {return t >= 0.0 && s.now - t <= max;};
  add("도메인", domain.empty() ? Level::WARN : Level::GOOD, "ROS_DOMAIN_ID " + (domain.empty() ? std::string("안 정함 (0) — 로봇과 같은지 확인") : domain));
  std::string miss;
  for (const char * n : {"/controller_manager", "/control_master", "/robot_state_publisher", "/arm_state_publisher"}) {
    if (!s.nodes.count(n)) {miss += std::string(miss.empty() ? "" : " · ") + (n + 1);}
  }
  add("필수 노드", miss.empty() ? Level::GOOD : Level::BAD, miss.empty() ? "controller_manager · control_master · state publisher 2" : "없음: " + miss);
  {
    std::string bad;
    for (const auto & h : s.hardware) {
      if (h.second != "active") {bad += (bad.empty() ? "" : " · ") + h.first + " " + h.second;}
    }
    if (s.hw_t < 0.0 || s.now - s.hw_t > parts::kPollStaleS) {
      add("보드", Level::BAD, s.hw_t < 0.0 ? "controller_manager 답 없음" : fmt("답이 %.0f s 끊김 — 상태 모름", s.now - s.hw_t));
    } else {
      add("보드", bad.empty() && s.hardware.size() >= 2 ? Level::GOOD : Level::BAD,
        bad.empty() ? fmt("%zu 개 active", s.hardware.size()) + (s.hardware.size() < 2 ? " (2 개여야 함)" : "") : bad);
    }
  }
  {
    std::string bad;
    std::set<std::string> have;
    for (const auto & c : s.controllers) {
      have.insert(c.first);
      if (c.second != "active") {bad += (bad.empty() ? "" : " · ") + c.first + " " + c.second;}
    }
    for (const char * n : {"joint_state_broadcaster", "arm_controller", "gripper_controller", "swerve_controller"}) {
      if (!have.count(n)) {bad += std::string(bad.empty() ? "" : " · ") + n + " 없음";}
    }
    const bool stale = s.ctrl_t >= 0.0 && s.now - s.ctrl_t > parts::kPollStaleS;
    add("컨트롤러", s.ctrl_t < 0.0 || stale || !bad.empty() ? Level::BAD : Level::GOOD,
      s.ctrl_t < 0.0 ? "controller_manager 답 없음" : stale ? fmt("답이 %.0f s 끊김 — 상태 모름", s.now - s.ctrl_t) :
      bad.empty() ? fmt("%zu 개 active", s.controllers.size()) : bad);
  }
  {
    bool finite = s.joints_t >= 0.0;
    for (const auto & j : kArmJoint5) {
      const auto it = s.joints.find(j);
      finite = finite && it != s.joints.end() && std::isfinite(it->second);
    }
    add("관절 값", finite && fresh(s.joints_t, 0.5) ? Level::GOOD : Level::BAD,
      s.joints_t < 0.0 ? "없음" : !finite ? "못 읽은 관절 있음 (NaN)" : fmt("%.2f s 전", s.now - s.joints_t));
  }
  {
    const auto pv = parts::pose_view(s, cfg);
    add("받침 접힘", pv.fold ? Level::GOOD : Level::WARN,
      pv.fold ? "접힘" : pv.worst.empty() ? "모름" : fmt("아님 (%s %+.1f°) — 주행 전 home", pv.worst.c_str(), parts::deg(pv.worst_diff)));
  }
  add("라이다 앞", fresh(s.scan_front_t, 1.0) ? Level::GOOD : Level::BAD, s.scan_front_t < 0.0 ? "/lidar_front/scan 없음" :
    fmt("%.1f s 전", s.now - s.scan_front_t));
  add("라이다 뒤", fresh(s.scan_rear_t, 1.0) ? Level::GOOD : Level::BAD, s.scan_rear_t < 0.0 ? "/lidar_rear/scan 없음" :
    fmt("%.1f s 전", s.now - s.scan_rear_t));
  add("IMU", fresh(s.imu_t, 1.0) ? Level::GOOD : Level::BAD, s.imu_t < 0.0 ? "/imu_sensor_broadcaster/imu 없음 (mock 은 없음)" :
    fmt("%.1f s 전", s.now - s.imu_t));
  {
    double vmin = 1e9;
    std::string txt;
    for (const auto & b : s.battery) {
      if (s.now - b.second.t > 60.0) {continue;}
      vmin = std::min(vmin, b.second.mean);
      txt += fmt("%s%s %.1f V", txt.empty() ? "" : " · ", b.first.c_str(), b.second.mean);
    }
    if (txt.empty()) {
      add("배터리", Level::WARN, "모름 (OpenCR 로그에 전압이 아직 없음 · mock)");
    } else {
      add("배터리", vmin < cfg.battery_stop ? Level::BAD : vmin < cfg.battery_warn ? Level::WARN : Level::GOOD, txt);
    }
  }
  if (s.odom_t < 0.0 || s.now - s.odom_t > 1.0) {   // 주행 허용 조건과 같음
    add("시계 차이", Level::BAD, s.odom_t < 0.0 ? "/odom 없음 — 잴 수 없음" : fmt("/odom 이 %.1f s 끊김 — 잴 수 없음", s.now - s.odom_t));
  } else {
    const double c = std::fabs(s.clock_off);
    add("시계 차이", c < 0.1 ? Level::GOOD : c <= kClockTolS ? Level::WARN : Level::BAD,
      fmt("이 PC − 로봇 %+.3f s", s.clock_off) + (c > kClockTolS ? " — drive · teleop 막힘 (시각 동기화)" : ""));
  }
  add("차례", s.mode_t < 0.0 || s.now - s.mode_t > kModeFreshS ? Level::BAD : s.mode == 1 ? Level::GOOD : Level::WARN,
    s.mode_t < 0.0 ? "/control/motion_mode 없음" : s.mode == 1 ? "DRIVE" : mode_name(s.mode) + " — 주행 전 home");
  return v;
}

std::string go_block_reason(const Snapshot & s, const Config & cfg)
{
  if (s.estop) {return "E-STOP 중 — estop off 뒤에";}
  if (parts::any_active(s.arm_goals) || parts::running_task(s)) {return "팔 작업 중 — 끝난 뒤에";}
  const auto g = s.joints.find("gripper_left_joint");
  if (g != s.joints.end() && gripper_holding(cfg.arm, g->second)) {return "퍽을 쥔 채 (그리퍼 반쯤 닫힘) — STORE · PLACE 로 놓은 뒤에";}
  const auto pv = parts::pose_view(s, cfg);
  if (pv.near && pv.pose->name == "기준점") {return "퍽 쥔 자세 (기준점) — STORE · PLACE 로 놓은 뒤에";}
  return "";
}

bool go_needs_fold(const Snapshot & s, const Config & cfg)
{
  const bool drive = s.mode_t >= 0.0 && s.now - s.mode_t <= kModeFreshS && s.mode == 1;
  return !drive || !parts::pose_view(s, cfg).fold;
}

bool preflight_ok(const std::vector<CheckItem> & items)
{
  for (const auto & i : items) {
    if (i.level == Level::BAD) {return false;}
  }
  return true;
}

std::vector<Line> check_lines(const std::vector<CheckItem> & items)
{
  parts::Lines L;
  int bad = 0, warn = 0;
  for (const auto & i : items) {
    bad += i.level == Level::BAD ? 1 : 0;
    warn += i.level == Level::WARN ? 1 : 0;
    parts::block(L, i.name, i.level == Level::GOOD ? "통과" : i.level == Level::WARN ? "주의" : i.level == Level::BAD ? "실패" : "건너뜀",
      i.level, i.detail);
  }
  L.push_back({"", Level::NORMAL});
  L.push_back({bad ? parts::fmt("점검 실패 %d · 주의 %d", bad, warn) : warn ? parts::fmt("점검 통과 (주의 %d)", warn) : "점검 통과",
    bad ? Level::BAD : warn ? Level::WARN : Level::GOOD});
  return parts::wrap(L, kMaxWidth);
}

std::vector<PageAction> page_actions(int page, const Config & cfg)
{
  switch (page) {
    case 2: return {{"HOME", "home", "", false}, {"LOOK", "look", "", false}, {"넣기", "store {}", "넣을 칸? (1~12)", false},
      {"빼기", "retrieve {}", "뺄 칸? (1~12)", false}, {"멈춤", "cancel arm", "", false}};
    case 3: {
      std::vector<PageAction> v;
      for (const auto & st : cfg.stops) {v.push_back({st.name, "g " + st.name, "", false});}
      v.push_back({"멈춤", "cancel nav", "", false});
      return v;
    }
    case 5: return {{"시작", "mission start", "미션 시작? (y)", true}, {"이어서", "mission resume", "", false},
      {"그리퍼 비움 표시", "mission clear_gripper", "그리퍼가 비었다고 표시? (y)", true}, {"신호 다시", "mission reset_signal", "", false}};
    case 6: return {{"점검", "check", "", false}};
    case 7: return {{"mark", "mark {}", "이름?", false}, {"cycle", "cycle {}", "칸 횟수? (예 5 3)", false},
      {"cycle 멈춤", "cycle stop", "", false}, {"메모", "note {}", "메모?", false}, {"bag 시작", "bag start", "", false},
      {"bag 끝", "bag stop", "", false}, {"점검", "check", "", false}};
    default: return {};
  }
}

std::vector<Line> status_lines(const Snapshot & s, const Config & cfg)
{
  using namespace parts;
  Lines L;
  wheels_block(L, s, cfg);
  arm_block(L, s, cfg);
  limit_block(L, s, cfg);
  manual_block(L, s, false);
  L.push_back({"", Level::NORMAL});
  loc_block(L, s, cfg);
  nav_block(L, s);
  L.push_back({"", Level::NORMAL});
  percep_block(L, s);
  mission_block(L, s);
  L.push_back({"", Level::NORMAL});
  hardware_block(L, s);
  battery_block(L, s, cfg);
  L.push_back({"", Level::NORMAL});
  L.push_back({"자세히 → 1 위치·SLAM · 2 팔 · 3 주행·Nav2 · 4 인지 · 5 미션 · 6 진단 · 7 실험 (번호 + Enter)", Level::OFF});
  return wrap(L, kMaxWidth);
}

}  // namespace robot_cli
