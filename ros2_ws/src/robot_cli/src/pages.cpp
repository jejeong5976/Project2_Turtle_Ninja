// pages.cpp — 1~7 쪽 (0 쪽에서 번호 + Enter), 설명은 status_view.hpp
//   맨 위 = 그 쪽 덩어리의 배지 줄 (0 쪽과 같은 색), 아래 = 표 (단위는 머리줄에만), 덩어리 사이 빈 줄, 맨 아래 = 번호 동작

#include <cmath>

#include "robot_cli/status_view.hpp"
#include "view_parts.hpp"

namespace robot_cli
{

namespace
{
using namespace parts;

std::string rj(const std::string & s, int w)
{
  const int n = text_width(s);
  return n >= w ? s : std::string(w - n, ' ') + s;
}

std::string sensor_age(const Snapshot & s, double t, const char * none)
{
  return t < 0.0 ? std::string(none) : fmt("%.1f s 전", s.now - t);
}

// 번호 동작 안내 (쪽 맨 아래)
void actions(Lines & L, int page, const Config & cfg)
{
  const auto acts = page_actions(page, cfg);
  head(L, "번호 동작 (번호 + Enter, 물으면 답 + Enter, Esc = 그만)");
  if (page == 3 && cfg.stops.empty()) {
    L.push_back({"정차 자세가 없어 번호 이동 없음 · 1 = 멈춤", Level::NORMAL});
  } else if (page == 3) {
    L.push_back({fmt("1~%zu = 그 정차로 이동 (팔이 접혀 있지 않으면 home 먼저 = 접고 이동, 퍽을 쥐었으면 거절) · %zu = 멈춤",
      cfg.stops.size(), cfg.stops.size() + 1), Level::NORMAL});
  } else if (acts.empty()) {
    L.push_back({"이 쪽은 동작 없음 — 번호 = 그 쪽으로", Level::NORMAL});
  } else {
    std::string t;
    for (std::size_t i = 0; i < acts.size(); ++i) {
      t += fmt("%s%zu %s", t.empty() ? "" : " · ", i + 1, acts[i].label.c_str()) + (acts[i].ask.empty() || acts[i].confirm ? "" : "(?)");
    }
    L.push_back({t, Level::NORMAL});
  }
  L.push_back({"0 · Esc = 전체 요약 · ←/→ = 옆 쪽", Level::OFF});
}

std::string det_pos(const DetView & d)
{
  if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z)) {return rj("-", 8) + rj("-", 8) + rj("-", 8);}
  return rj(fmt("%+.3f", d.x), 8) + rj(fmt("%+.3f", d.y), 8) + rj(fmt("%+.3f", d.z), 8);
}

// 1 위치 · SLAM
Lines loc_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  loc_block(L, s, cfg);
  head(L, "위치");
  pose_table(L, s);
  head(L, "위치 추정");
  row(L, "종류", s.loc_mode.empty() ? "없음" : s.loc_mode);
  if (s.health_t >= 0.0) {
    static const char * lv[] = {"정상", "경고", "정지", "모름"};
    row(L, "건강", std::string(s.health_level >= 0 && s.health_level <= 3 ? lv[s.health_level] : "?") + " · " + s.health_msg + " · " +
      age(s, s.health_t) + " 전", s.health_level >= 1 ? Level::WARN : Level::NORMAL);
  } else {
    row(L, "건강", "/localization_health 없음", Level::OFF);
  }
  auto up = [&](const char * n) {return has_node(s, n) ? "O" : "-";};
  row(L, "노드", fmt("slam_toolbox %s · amcl %s · pose_guard %s · scan_merger %s", up("/slam_toolbox"), up("/amcl"),
    up("/pose_guard"), up("/scan_merger")));
  auto act = [](int v) {return v < 0 ? std::string("서비스 없음") : v ? std::string("active") : std::string("비활성");};
  row(L, "lifecycle", "localization " + act(s.loc_active) + " · navigation " + act(s.nav_active));
  head(L, "센서");
  row(L, "라이다 앞", sensor_age(s, s.scan_front_t, "없음"), s.scan_front_t < 0.0 ? Level::OFF : age_level(s, s.scan_front_t));
  row(L, "라이다 뒤", sensor_age(s, s.scan_rear_t, "없음"), s.scan_rear_t < 0.0 ? Level::OFF : age_level(s, s.scan_rear_t));
  row(L, "합친 scan", sensor_age(s, s.scan_t, "없음 (scan_merger)"), s.scan_t < 0.0 ? Level::OFF : age_level(s, s.scan_t));
  row(L, "IMU", sensor_age(s, s.imu_t, "없음 (OpenCR 일 때만)"), s.imu_t < 0.0 ? Level::OFF : age_level(s, s.imu_t));
  if (s.odom_t >= 0.0) {
    row(L, "시계", fmt("이 PC − 로봇 %+.3f s (0.25 s 넘으면 drive · teleop 막힘)", s.clock_off),
      std::fabs(s.clock_off) > kClockTolS ? Level::BAD : Level::NORMAL);
  }
  head(L, "다시 띄우기 (실행 중에 바꾸는 방법 없음)");
  for (const auto & t : loc_hint()) {L.push_back({t, Level::NORMAL});}
  actions(L, 1, cfg);
  return L;
}

// 2 팔
Lines arm_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  wheels_block(L, s, cfg);
  arm_block(L, s, cfg);
  limit_block(L, s, cfg);
  joint_table(L, s, cfg, true);
  head(L, "자세");
  const PoseView pv = pose_view(s, cfg);
  if (cfg.arm.poses.empty()) {
    row(L, "이름 자세", "control.yaml · arm_paths.yaml 못 읽음", Level::WARN);
  } else if (!pv.pose) {
    row(L, "이름 자세", "관절 측정 없음", Level::WARN);
  } else {
    row(L, "이름 자세", pv.near ? fmt("%s (차이 %.1f°)", pv.pose->name.c_str(), deg(pv.diff)) :
      fmt("아님 (가장 가까운 %s %.1f°)", pv.pose->name.c_str(), deg(pv.diff)), pv.near ? Level::NORMAL : Level::WARN);
    std::string tol;
    for (double t : cfg.arm.fold_tol) {tol += fmt("%s%.1f°", tol.empty() ? "" : " ", deg(t));}
    row(L, "받침 접힘", pv.fold ? "예" : pv.worst.empty() ? "모름" : fmt("아니오 — %s %+.1f° 벗어남 (기준 ± %s)", pv.worst.c_str(),
      deg(pv.worst_diff), tol.c_str()));
    row(L, "lazy_fold", cfg.arm.lazy_fold ? "켜짐 — STORE · PLACE 는 P0 에서 끝 · 바퀴 잠금 유지, 주행 전 home (fold 경로)" :
      "꺼짐 — STORE · PLACE 는 받침에 접혀 끝 · 주행 가능");
  }
  if (!s.limits.empty()) {
    std::string t;
    for (const auto & j : kArmJoint5) {
      const auto l = s.limits.find(j);
      if (l != s.limits.end()) {t += fmt("%s%s %+.0f~%+.0f°", t.empty() ? "" : " · ", j.c_str(), deg(l->second.first), deg(l->second.second));}
    }
    row(L, "한계", t);
  } else {
    row(L, "한계", "/robot_description 없음 → 한계 경고 못 함", Level::OFF);
  }
  head(L, "차례 · 팔 작업");
  if (s.mode_t >= 0.0) {
    row(L, "차례", fmt("%s · 요청 %u (%s) · ack %s %u · 새 작업 받음 %s", mode_name(s.mode).c_str(), s.request_id, s.reason.c_str(),
      s.ack_t < 0.0 ? "없음" : mode_name(s.ack_mode).c_str(), s.ack_id, s.ready ? "예" : "아니오"));
  }
  row(L, "goal", goals_text(s.arm_goals) + (s.arm_phase_t < 0.0 ? "" : " · phase " + s.arm_phase));
  if (!s.my_arm.empty()) {row(L, "이 CLI", s.my_arm);}
  head(L, "최근 팔 작업 (새 것 먼저)");
  arm_timings(L, s, 8);
  actions(L, 2, cfg);
  return L;
}

// 3 주행 · Nav2
Lines nav_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  wheels_block(L, s, cfg);
  nav_block(L, s);
  manual_block(L, s, true);
  head(L, "Nav2");
  auto act = [](int v) {return v < 0 ? std::string("서비스 없음") : v ? std::string("active") : std::string("비활성");};
  row(L, "lifecycle", "navigation " + act(s.nav_active) + " · localization " + act(s.loc_active));
  row(L, "goal", goals_text(s.nav_goals));
  if (any_active(s.nav_goals) && s.nav_fb_t >= 0.0) {
    L.push_back({pad("", kLabelW) + rj("남은[m]", 8) + rj("ETA[s]", 8) + rj("경과[s]", 9) + rj("복구", 6), Level::NORMAL});
    L.push_back({pad("진행", kLabelW) + rj(fmt("%.2f", s.dist_left), 8) + rj(fmt("%.0f", s.eta_s), 8) + rj(fmt("%.0f", s.nav_time_s), 9) +
      rj(fmt("%d", s.recoveries), 6), age_level(s, s.nav_fb_t)});
  }
  if (s.hold_t >= 0.0) {row(L, "hold", s.hold ? "true — Nav2 속도 막힘 (mission start)" : "false", s.hold ? Level::WARN : Level::NORMAL);}
  row(L, "노드", fmt("bt_navigator %s · cmd_vel_hold %s", has_node(s, "/bt_navigator") ? "O" : "-", has_node(s, "/cmd_vel_hold") ? "O" : "-"));
  if (!s.my_nav.empty()) {row(L, "이 CLI", s.my_nav);}
  head(L, "수동 주행 (drive · teleop)");
  const std::string why = drive_block_reason(s, false);
  row(L, "drive", why.empty() ? fmt("지금 됨 (≤ %.2f m/s · %.1f rad/s · 10 s)", cfg.max_linear, cfg.max_angular) : "막힘: " + why,
    why.empty() ? Level::GOOD : Level::WARN);
  head(L, "정차 (dice_map, map 기준) — 번호 + Enter = 그리로 이동");
  if (cfg.stops.empty()) {L.push_back({"정차 자세 없음 (dice_map 을 못 읽음)", Level::WARN});}
  double best = -1.0;
  const NamedStop * near = s.have_map_pose ? nearest_stop(cfg.stops, s.mx, s.my, &best) : nullptr;
  L.push_back({rj("#", 3) + "  " + pad("이름", 9) + rj("x[m]", 8) + rj("y[m]", 9) + rj("yaw[°]", 8) + rj("거리[m]", 9) + rj("방향차[°]", 11),
    Level::NORMAL});
  for (std::size_t i = 0; i < cfg.stops.size(); ++i) {
    const auto & st = cfg.stops[i];
    std::string rest = rj("-", 9) + rj("-", 11);
    if (s.have_map_pose) {
      rest = rj(fmt("%.2f", std::hypot(st.pose.x - s.mx, st.pose.y - s.my)), 9) + rj(fmt("%+.1f", deg(wrap_angle(st.pose.yaw - s.myaw))), 11);
    }
    L.push_back({rj(fmt("%zu", i + 1), 3) + "  " + pad(st.name, 9) + rj(fmt("%+.3f", nz(st.pose.x)), 8) + rj(fmt("%+.3f", nz(st.pose.y)), 9) +
      rj(fmt("%+.1f", nz(deg(st.pose.yaw))), 8) + rest + (&st == near ? "  ◀ 가까움" : ""), &st == near ? Level::GOOD : Level::NORMAL});
  }
  if (!s.have_map_pose) {L.push_back({"지도 위치 없음 → 거리 모름 (위치 추정을 켜면 보임)", Level::OFF});}
  actions(L, 3, cfg);
  return L;
}

// 4 인지
Lines percep_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  percep_block(L, s);
  head(L, "신호등");
  if (s.light_t < 0.0) {
    row(L, "상태", "없음", Level::OFF);
  } else {
    row(L, "상태", light_name(s.light) + (s.light_visible ? " · 보임" : " · 안 보임") + " · " + age(s, s.light_t) + " 전", age_level(s, s.light_t));
  }
  head(L, "손목 카메라 검출");
  if (s.wrist_t < 0.0) {
    row(L, "상태", "없음 (perception 꺼짐 · 손목 카메라 끔)", Level::OFF);
  } else {
    row(L, "상태", fmt("%d 개 · %s 전 · 촬영→받음 %.2f s · frame %s", s.wrist_n, age(s, s.wrist_t).c_str(), s.wrist_lag,
      s.wrist_frame.c_str()), age_level(s, s.wrist_t));
    if (!s.wrist_dets.empty()) {
      L.push_back({rj("#", 3) + "  " + pad("종류", 7) + pad("색", 7) + pad("모양", 9) + rj("태그", 5) + rj("신뢰", 6) + rj("x[m]", 8) +
        rj("y[m]", 8) + rj("z[m]", 8) + "  방향", Level::NORMAL});
    }
    for (std::size_t i = 0; i < s.wrist_dets.size(); ++i) {
      const DetView & d = s.wrist_dets[i];
      L.push_back({rj(fmt("%zu", i + 1), 3) + "  " + pad(d.kind, 7) + pad(d.label, 7) + pad(d.shape.empty() ? "-" : d.shape, 9) +
        rj(d.id.empty() ? "-" : d.id, 5) + rj(fmt("%.2f", d.conf), 6) + det_pos(d) + (d.yaw_known ? "  앎" : "  모름"), Level::NORMAL});
    }
  }
  head(L, "광각 카메라 검출 (위치 없음)");
  if (s.wide_t < 0.0) {
    row(L, "상태", "없음", Level::OFF);
  } else {
    row(L, "상태", fmt("%d 개 · %s 전", s.wide_n, age(s, s.wide_t).c_str()), age_level(s, s.wide_t));
    if (!s.wide_dets.empty()) {
      L.push_back({rj("#", 3) + "  " + pad("종류", 7) + pad("색", 7) + pad("모양", 9) + rj("태그", 5) + rj("신뢰", 6) + "  bbox[px]", Level::NORMAL});
    }
    for (std::size_t i = 0; i < s.wide_dets.size(); ++i) {
      const DetView & d = s.wide_dets[i];
      L.push_back({rj(fmt("%zu", i + 1), 3) + "  " + pad(d.kind, 7) + pad(d.label, 7) + pad(d.shape.empty() ? "-" : d.shape, 9) +
        rj(d.id.empty() ? "-" : d.id, 5) + rj(fmt("%.2f", d.conf), 6) + fmt("  %.0f %.0f %.0f %.0f", d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]),
        Level::NORMAL});
    }
  }
  head(L, "바코드");
  row(L, "글", s.text_t < 0.0 ? std::string("없음 (barcode → call /perception/barcode_enable on)") : "\"" + s.task_text + "\" · " +
    age(s, s.text_t) + " 전", s.text_t < 0.0 ? Level::OFF : Level::NORMAL);
  row(L, "노드", std::string("perception ") + (has_node(s, "/perception") ? "O" : "-"));
  actions(L, 4, cfg);
  return L;
}

// 5 미션
Lines mission_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  mission_block(L, s);
  head(L, "미션");
  if (s.mission_t < 0.0) {
    row(L, "상태", "mission_manager 안 켜짐", Level::OFF);
  } else {
    row(L, "상태", s.m_state + (s.m_phase.empty() ? "" : " · " + s.m_phase) + " · step " + (s.m_step.empty() ? "-" : s.m_step));
    row(L, "퍽", fmt("%d / %d", s.pucks_done, s.pucks_total));
    row(L, "시작 가능", s.m_ready ? "예" : "아니오");
    if (!s.m_detail.empty()) {row(L, "내용", s.m_detail, s.m_fault ? Level::BAD : Level::NORMAL);}
    if (s.m_fault) {row(L, "고장", "원인 확인 뒤 clear_gripper · resume", Level::BAD);}
  }
  if (s.hold_t >= 0.0) {row(L, "hold", s.hold ? "true — Nav2 속도 막힘" : "false", s.hold ? Level::WARN : Level::NORMAL);}
  head(L, "적재");
  if (s.cargo_t < 0.0) {
    row(L, "상태", "/mission/cargo 없음", Level::OFF);
  } else {
    L.push_back({pad("칸", 8) + pad("퍽", 6) + pad("색", 7) + pad("모양", 10) + rj("태그", 5) + "  확인", Level::NORMAL});
    auto slot_row = [&](const std::string & name, const SlotView & c) {
        L.push_back({pad(name, 8) + pad(c.occupied ? "있음" : "-", 6) + pad(c.color.empty() ? "-" : c.color, 7) +
          pad(c.shape.empty() ? "-" : c.shape, 10) + rj(c.puck_id.empty() ? "-" : c.puck_id, 5) +
          (c.occupied ? (c.verified ? "  카메라" : "  기록만") : ""), c.occupied ? Level::GOOD : Level::NORMAL});
      };
    for (const auto & c : s.cargo_slots) {slot_row(c.slot, c);}
    slot_row("그리퍼", s.cargo_gripper);
  }
  actions(L, 5, cfg);
  return L;
}

// 6 진단
Lines diag_page(const Snapshot & s, const Config & cfg)
{
  Lines L;
  hardware_block(L, s);
  battery_block(L, s, cfg);
  head(L, "노드");
  std::size_t watched = 0;
  for (std::size_t i = 0; i < kWatchNodes.size(); i += 2) {
    std::string t;
    for (std::size_t k = i; k < i + 2 && k < kWatchNodes.size(); ++k) {
      const bool up = has_node(s, kWatchNodes[k]);
      watched += up ? 1 : 0;
      t += pad(std::string(up ? "O " : "- ") + kWatchNodes[k].substr(1), 30);
    }
    L.push_back({t, Level::NORMAL});
  }
  L.push_back({fmt("그 밖 노드 %zu 개", s.nodes.size() - watched), Level::OFF});
  head(L, "ros2_control");
  component_table(L, s);
  head(L, "센서 · 시계");
  L.push_back({pad("", kLabelW) + rj("odom", 8) + rj("관절", 8) + rj("라이다앞", 10) + rj("라이다뒤", 10) + rj("IMU", 7) + "  [s 전]",
    Level::NORMAL});
  auto a = [&](double t) {return t < 0.0 ? std::string("-") : fmt("%.1f", s.now - t);};
  L.push_back({pad("나이", kLabelW) + rj(a(s.odom_t), 8) + rj(a(s.joints_t), 8) + rj(a(s.scan_front_t), 10) + rj(a(s.scan_rear_t), 10) +
    rj(a(s.imu_t), 7), Level::NORMAL});
  if (s.odom_t >= 0.0) {row(L, "시계", fmt("이 PC − 로봇 %+.3f s", s.clock_off));}
  head(L, "/diagnostics (10 s 안)");
  diagnostics(L, s);
  actions(L, 6, cfg);
  return L;
}

// 7 실험
Lines exp_page(const Snapshot & s, const Config & cfg, const UiState & ui)
{
  Lines L;
  arm_block(L, s, cfg);
  head(L, "자세 기록 mark <이름> (팔은 안 움직임)");
  row(L, "폴더", ui.mark_dir.empty() ? std::string("-") : ui.mark_dir);
  row(L, "파일", "poses.csv · poses.yaml · detections.csv · cycle_<칸>.csv · notes.csv (덧붙임)");
  L.push_back({pad("관절", 9) + rj("눈금", 6) + rj("[rad]", 10) + rj("[°]", 9) + rj("보정[°]", 9) + rj("mark 와 Δ눈금", 15) + rj("Δ[°]", 8),
    Level::NORMAL});
  for (const auto & j : kArmJoints) {
    const bool grip = j == "gripper_left_joint";
    const std::string name = grip ? "그리퍼" : j;
    const auto it = s.joints.find(j);
    if (it == s.joints.end() || !std::isfinite(it->second)) {
      L.push_back({pad(name, 9) + "측정 없음", Level::WARN});
      continue;
    }
    const double q = it->second;
    const auto c = cfg.cal.find(j);
    const auto kc = s.corrected.find(j);
    const auto m = ui.last_mark_q.find(j);
    const bool dm = m != ui.last_mark_q.end() && std::isfinite(m->second) && c != cfg.cal.end();
    L.push_back({pad(name, 9) + rj(c != cfg.cal.end() ? fmt("%.0f", std::round(joint_to_tick(c->second, q))) : "?", 6) + rj(fmt("%+.4f", q), 10) +
      rj(grip ? "-" : fmt("%+.2f", deg(q)), 9) +
      rj(!grip && kc != s.corrected.end() && std::isfinite(kc->second) ? fmt("%+.2f", deg(kc->second)) : "-", 9) +
      rj(dm ? fmt("%+.0f", std::round(joint_to_tick(c->second, q)) - std::round(joint_to_tick(c->second, m->second))) : "-", 15) +
      rj(dm && !grip ? fmt("%+.2f", deg(q - m->second)) : "-", 8), age_level(s, s.joints_t)});
  }
  std::string tags;
  for (const auto & d : s.wrist_dets) {
    if (!d.id.empty()) {tags += (tags.empty() ? "" : " ") + d.id;}
  }
  row(L, "손목검출", s.wrist_t < 0.0 ? std::string("없음 → mark 에 관절값만") : fmt("%d 개 · 태그 %s · mark 때 같이 기록", s.wrist_n,
    tags.empty() ? "-" : tags.c_str()), s.wrist_t < 0.0 ? Level::OFF : age_level(s, s.wrist_t));
  row(L, "처짐 보정", "칸 바닥 AprilTag 20~31 이 보이는 자세에서 mark (인지가 그 태그를 내야 검출에 남음)", Level::OFF);
  for (std::size_t i = ui.marks.size() > 3 ? ui.marks.size() - 3 : 0; i < ui.marks.size(); ++i) {row(L, "최근", ui.marks[i]);}
  head(L, "반복 시험 cycle <칸> <횟수> [store] (ArmTask 만)");
  row(L, "상태", ui.cycle.empty() ? std::string("안 함 · 예 cycle 5 3 = RETRIEVE S5 → STORE S5 를 3 번") : ui.cycle,
    ui.cycle.empty() ? Level::NORMAL : Level::GOOD);
  for (std::size_t i = ui.cycle_log.size() > 3 ? ui.cycle_log.size() - 3 : 0; i < ui.cycle_log.size(); ++i) {row(L, "", ui.cycle_log[i]);}
  head(L, "기록");
  row(L, "세션", ui.session_file.empty() ? std::string("안 씀") : ui.session_file);
  row(L, "메모", ui.last_note.empty() ? std::string("없음 · note <메모>") : ui.last_note);
  row(L, "bag", ui.bag.empty() ? std::string("안 함 · bag start") : ui.bag, ui.bag.empty() ? Level::NORMAL : Level::GOOD);
  head(L, "손으로 움직이기 (토크 끄기) — 없음, 읽기 전용");
  row(L, "", "지금 인터페이스로는 안전하게 못 함 (teach 명령 · README '손으로 가르치기')", Level::WARN);
  actions(L, 7, cfg);
  return L;
}
}  // namespace

std::vector<Line> page_lines(int page, const Snapshot & s, const Config & cfg, const UiState & ui)
{
  switch (page) {
    case 1: return wrap(loc_page(s, cfg), kMaxWidth);
    case 2: return wrap(arm_page(s, cfg), kMaxWidth);
    case 3: return wrap(nav_page(s, cfg), kMaxWidth);
    case 4: return wrap(percep_page(s, cfg), kMaxWidth);
    case 5: return wrap(mission_page(s, cfg), kMaxWidth);
    case 6: return wrap(diag_page(s, cfg), kMaxWidth);
    case 7: return wrap(exp_page(s, cfg, ui), kMaxWidth);
    default: return status_lines(s, cfg);
  }
}

}  // namespace robot_cli
