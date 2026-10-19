// arm.cpp — 팔 쪽 순수 계산 (설명은 include/nt_control/arm.hpp)

#include "nt_control/arm.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

namespace nt_control
{

// ============================================================================ ① 동작 순서표
namespace
{
ArmAction arm(const std::string & phase, const std::string & pose) {return {phase, false, pose, 0.0};}
ArmAction grip(const std::string & phase, double v) {return {phase, true, "", v};}
}  // namespace

std::vector<ArmAction> make_arm_plan(uint8_t task, double o, double c)
{
  switch (task) {
    case kArmHome:
      return {arm("retreat", "home")};
    case kArmPick:
      // 위에서 손을 벌리고 → 내려가서 → 쥐고 → 들고 → 접기
      return {arm("approach", "pick_above"), grip("approach", o), arm("approach", "pick_down"),
        grip("grasp", c), arm("lift", "pick_above"), arm("retreat", "home")};
    case kArmPlace:
      // 위로 → 내려가서 → 놓고 → 들고 → 손 닫고 → 접기
      return {arm("approach", "place_above"), arm("approach", "place_down"), grip("release", o),
        arm("retreat", "place_above"), grip("retreat", c), arm("retreat", "home")};
    default:
      return {};
  }
}

// ============================================================================ ② 차례
const char * turn_name(Turn t)
{
  switch (t) {
    case Turn::STOP: return "STOP";
    case Turn::DRIVE: return "DRIVE";
    case Turn::ARM: return "ARM";
    case Turn::STOPPING: return "STOPPING";
  }
  return "?";
}

void MotionArbiter::set(Turn t)
{
  if (t != turn_) {++request_id_;}   // 차례가 바뀌면 새 번호 → 예전 번호의 "잠갔음" 답은 쓸모없어짐
  turn_ = t;
  still_since_ = -1.0;
}

void MotionArbiter::request_arm()
{
  if (turn_ == Turn::ARM || turn_ == Turn::STOPPING) {return;}   // 이미 팔 쪽이면 그대로
  set(Turn::STOPPING);
}

void MotionArbiter::arm_ended(bool drive_ok)
{
  set(drive_ok ? Turn::DRIVE : Turn::STOP);
}

bool MotionArbiter::enforce_drive_guard(bool folded_fresh, bool hardware_healthy)
{
  if (turn_ != Turn::DRIVE || (folded_fresh && hardware_healthy)) {return false;}
  set(Turn::STOP);
  return true;
}

void MotionArbiter::update(double now_s, bool base_still, uint32_t acked_id)
{
  if (turn_ != Turn::STOPPING) {return;}
  if (!base_still || acked_id != request_id_) {
    still_since_ = -1.0;   // 잠금 답이 없거나 다시 움직이면 처음부터 다시 셈
    return;
  }
  if (still_since_ < 0.0) {still_since_ = now_s;}
  if (now_s - still_since_ >= settle_s_) {set(Turn::ARM);}
}

// ============================================================================ ③ 팔 작업 진행
void ArmRunner::start(std::vector<ArmAction> plan)
{
  plan_ = std::move(plan);
  idx_ = 0;
  active_ = true;
  if (busy_) {++gen_;}   // 앞 작업의 궤적 답이 아직 안 왔으면 그 답은 이제 무시
  busy_ = false;
  accepted_ = false;
  failed_ = plan_.empty();
  fail_why_ = plan_.empty() ? "빈 동작 목록" : "";
  cancel_requested_ = false;
  cancel_sent_ = false;
  verifying_ = false;
  forced_ = false;
}

LateGoal ArmRunner::on_goal_response(unsigned gen, bool accepted, double now_s)
{
  if (gen != gen_ || !busy_) {
    // 이미 포기 · 취소한 궤적의 늦은 답. 받아들여졌다면 그 궤적만 골라 취소해야 팔이 혼자 안 움직임
    return accepted ? LateGoal::CANCEL_IT : LateGoal::IGNORE;
  }
  if (!accepted) {
    failed_ = true;
    fail_why_ = "재생기가 '" + current_phase() + "' 동작을 거절";
    busy_ = false;
    return LateGoal::IGNORE;
  }
  accepted_ = true;
  motion_deadline_ = now_s + motion_timeout_s_;
  return LateGoal::USE;
}

void ArmRunner::on_result(unsigned gen, bool ok)
{
  if (gen != gen_ || !busy_) {return;}
  if (!ok) {
    failed_ = true;
    fail_why_ = "'" + current_phase() + "' 동작이 성공하지 못함";
  }
  busy_ = false;
  accepted_ = false;
}

std::vector<ArmCommand> ArmRunner::tick(double now_s, bool arm_may_move, bool fold_confirmed)
{
  std::vector<ArmCommand> out;
  if (!active_) {return out;}

  // ⓪ 하드웨어 이상 등 (fail_now): 진행 중 궤적은 취소 요청만 보내고 바로 실패 (답을 기다리지 않음)
  if (forced_) {
    if (busy_) {
      abandon(out, forced_why_);
    } else {
      failed_ = true;
      fail_why_ = forced_why_;
    }
    out.push_back(finish(ArmEnd::ABORTED, "팔 작업 중단: " + forced_why_));
    return out;
  }

  // ① 취소가 가장 먼저. 진행 중 궤적에 취소를 보내고, 결과(멈췄음)가 오거나 cancel_timeout_s 가 지나면 끝
  if (cancel_requested_) {
    if (!cancel_sent_) {
      cancel_sent_ = true;
      cancel_deadline_ = now_s + cancel_timeout_s_;
      if (busy_) {
        ArmCommand c;
        c.kind = ArmCommand::Kind::CANCEL_ACTIVE;
        c.gen = gen_;
        out.push_back(c);
        if (!accepted_) {   // 아직 받아들여지기 전 — 결과가 올 일이 없음. 늦게 받아들여지면 CANCEL_IT 로 처리
          ++gen_;
          busy_ = false;
        }
      }
    }
    if (busy_ && now_s < cancel_deadline_) {return out;}   // 멈췄다는 답을 기다림
    // ★ 팔을 움직였든 안 움직였든 취소는 언제나 바퀴 잠금 (finish 안에서 drive_ok = false)
    out.push_back(finish(ArmEnd::CANCELED,
      busy_ ? "취소됨 (재생기 정지 답 없음 — 바퀴 잠금 유지)" : "취소됨 (바퀴 잠금 유지 — HOME 으로 풀기)"));
    return out;
  }

  // ② 시간 제한: 받아 주기 / 끝나기 따로
  if (busy_ && !accepted_ && now_s > accept_deadline_) {
    abandon(out, "재생기가 '" + current_phase() + "' 동작을 받아 주지 않음");
  } else if (busy_ && accepted_ && now_s > motion_deadline_) {
    abandon(out, "'" + current_phase() + "' 동작 시간 초과");
  }
  if (busy_) {return out;}

  // ③ 실패는 차례와 상관없이 바로 끝냄 (→ STOP)
  if (failed_) {
    out.push_back(finish(ArmEnd::ABORTED, "팔 동작 실패: " + fail_why_));
    return out;
  }
  if (!arm_may_move) {return out;}   // 바퀴가 아직 안 섰음 (STOPPING)

  // ④ 다 끝났으면: 접기로 끝나는 계획은 실측 확인(VERIFY)까지, 아니면 바로 성공 (drive_ok 는 false)
  if (idx_ >= plan_.size()) {
    const bool ends_folded = !plan_.empty() && !plan_.back().is_gripper && plan_.back().pose == "home";
    if (!ends_folded) {
      out.push_back(finish(ArmEnd::SUCCEEDED, "완료 (접기로 끝나지 않음 — 바퀴 잠금 유지)"));
      return out;
    }
    if (!verifying_) {
      // 이번 tick 에 들어온 fold_confirmed 는 VERIFY 전 측정이라 쓰지 않음 → 다음 tick 부터
      verifying_ = true;
      verify_deadline_ = now_s + verify_timeout_s_;
      ArmCommand c;
      c.kind = ArmCommand::Kind::VERIFY_START;
      out.push_back(c);
      return out;
    }
    if (fold_confirmed) {
      out.push_back(finish(ArmEnd::SUCCEEDED, "완료 (접힘 실측 확인)"));
    } else if (now_s > verify_deadline_) {
      failed_ = true;
      fail_why_ = "HOME 실측 확인 실패 — 관절이 home 범위에 안 들어옴 · 측정이 없거나 오래됨";
      out.push_back(finish(ArmEnd::ABORTED, "팔 동작 실패: " + fail_why_));
    }
    return out;
  }
  ArmCommand c;
  c.kind = ArmCommand::Kind::SEND;
  c.action = plan_[idx_++];
  c.gen = ++gen_;
  busy_ = true;
  accepted_ = false;
  accept_deadline_ = now_s + accept_timeout_s_;
  out.push_back(c);
  return out;
}

ArmCommand ArmRunner::finish(ArmEnd end, const std::string & message)
{
  ArmCommand c;
  c.kind = ArmCommand::Kind::FINISH;
  c.end = end;
  c.message = message;
  // 바퀴 차례로 돌아가는 유일한 길: 끝까지 성공 + 마지막 동작이 "팔을 home 으로" + 실측 접힘 확인(VERIFY 통과)
  const bool ends_folded = !plan_.empty() && !plan_.back().is_gripper && plan_.back().pose == "home";
  c.drive_ok = end == ArmEnd::SUCCEEDED && !failed_ && ends_folded && verifying_;
  if (busy_) {++gen_;}   // 답을 못 받은 궤적이 있으면 그 답은 이제 무시 (다음 작업을 건드리지 않게)
  active_ = false;
  busy_ = false;
  accepted_ = false;
  cancel_requested_ = false;
  cancel_sent_ = false;
  verifying_ = false;
  forced_ = false;
  return c;
}

void ArmRunner::abandon(std::vector<ArmCommand> & out, const std::string & why)
{
  ArmCommand c;
  c.kind = ArmCommand::Kind::CANCEL_ACTIVE;
  c.gen = gen_;
  out.push_back(c);
  ++gen_;   // 포기한 궤적의 늦은 답은 무시 (받아들여지면 CANCEL_IT)
  busy_ = false;
  accepted_ = false;
  failed_ = true;
  fail_why_ = why;
}

std::string ArmRunner::current_phase() const
{
  return idx_ > 0 && idx_ <= plan_.size() ? plan_[idx_ - 1].phase : std::string("시작");
}

}  // namespace nt_control

namespace nt_control
{

// ============================================================================ ④ 접힘 실측 확인
FoldCheck::FoldCheck(std::vector<double> home, std::vector<double> tol, double hold_s, double max_age_s)
: home_(std::move(home)), tol_(std::move(tol)), hold_s_(hold_s), max_age_s_(max_age_s)
{
  if (tol_.size() == 1) {tol_.assign(home_.size(), tol_[0]);}
  if (tol_.size() != home_.size()) {tol_.assign(home_.size(), 0.0);}   // 잘못 주면 0 → 사실상 확인 불가 (안전한 쪽)
}

void FoldCheck::reset()
{
  have_ = false;
  inside_since_ = -1.0;
  last_problem_ = "측정 없음";
}

void FoldCheck::observe(double stamp_s, const std::vector<double> & positions)
{
  if (!std::isfinite(stamp_s)) {reset(); return;}
  if (have_ && stamp_s <= last_stamp_) {return;}   // 시각이 앞으로 안 감 = 같은 옛날 측정
  // (Codex 10/16 재검증) 측정이 끊겼다가 다시 오면, 끊긴 동안은 "범위 안에 머물렀다" 고 볼 수 없음
  //   → 앞 측정과 max_age_s 넘게 벌어지면 유지 시간을 처음부터 (예전: 끊긴 시간까지 유지 시간으로 셈)
  const bool gap = have_ && stamp_s - last_stamp_ > max_age_s_;
  if (gap) {inside_since_ = -1.0;}
  have_ = true;
  last_stamp_ = stamp_s;
  bool inside = positions.size() == home_.size() && !home_.empty();
  if (!inside) {last_problem_ = "관절 수가 맞지 않음";}
  for (std::size_t i = 0; inside && i < home_.size(); ++i) {
    const double p = positions[i];
    char buf[96];
    if (!std::isfinite(p)) {
      std::snprintf(buf, sizeof(buf), "관절 %zu 측정 없음(NaN)", i + 1);
      last_problem_ = buf;
      inside = false;
    } else if (std::fabs(p - home_[i]) > tol_[i]) {
      std::snprintf(buf, sizeof(buf), "관절 %zu 가 home 에서 %.3f rad 벗어남 (허용 %.3f)", i + 1, p - home_[i], tol_[i]);
      last_problem_ = buf;
      inside = false;
    }
  }
  if (!inside) {
    inside_since_ = -1.0;
  } else if (inside_since_ < 0.0) {
    inside_since_ = stamp_s;
  }
}

bool FoldCheck::confirmed(double now_s) const
{
  return have_ && std::isfinite(now_s) && std::isfinite(last_stamp_) &&
         now_s >= last_stamp_ && now_s - last_stamp_ <= max_age_s_ && inside_since_ >= 0.0 &&
         last_stamp_ - inside_since_ >= hold_s_;
}

std::string FoldCheck::status(double now_s) const
{
  if (!have_) {return "측정 없음";}
  if (!std::isfinite(now_s) || !std::isfinite(last_stamp_) || now_s < last_stamp_) {return "측정 시각 이상";}
  if (now_s - last_stamp_ > max_age_s_) {return "측정이 오래됨";}
  if (inside_since_ < 0.0) {return last_problem_;}
  if (last_stamp_ - inside_since_ < hold_s_) {return "home 범위 안 — 유지 시간 세는 중";}
  return "확인됨";
}

}  // namespace nt_control

namespace nt_control
{

// ============================================================================ ⑤ 하드웨어 측정 살아 있나
void HardwareWatch::observe(double stamp_s, bool all_measured)
{
  if (!std::isfinite(stamp_s)) {have_ = false; bad_since_ = -1.0; return;}
  if (have_ && stamp_s <= last_stamp_) {return;}   // 같은 옛날 메시지 반복은 "살아 있음" 이 아님
  have_ = true;
  last_stamp_ = stamp_s;
  if (all_measured) {
    bad_since_ = -1.0;
  } else if (bad_since_ < 0.0) {
    bad_since_ = stamp_s;
  }
}

void HardwareWatch::observe_components(double now_s, bool all_active, const std::string & detail)
{
  comp_have_ = true;
  comp_stamp_ = now_s;
  comp_ok_ = all_active;
  comp_detail_ = detail;
}

bool HardwareWatch::components_ok(double now_s) const
{
  if (component_timeout_s_ <= 0.0) {return true;}   // 보드 상태는 안 봄
  return comp_have_ && comp_ok_ && std::isfinite(now_s) && std::isfinite(comp_stamp_) &&
         now_s >= comp_stamp_ && now_s - comp_stamp_ <= component_timeout_s_;
}

bool HardwareWatch::healthy(double now_s) const
{
  if (!components_ok(now_s)) {return false;}
  if (!have_ || !std::isfinite(now_s) || !std::isfinite(last_stamp_) ||
      now_s < last_stamp_ || now_s - last_stamp_ > state_timeout_s_) {return false;}
  return bad_since_ < 0.0 || last_stamp_ - bad_since_ < bad_hold_s_;
}

std::string HardwareWatch::status(double now_s) const
{
  char buf[96];
  if (!components_ok(now_s)) {
    if (!comp_have_) {return "보드 상태(controller_manager)를 아직 못 받음";}
    if (now_s - comp_stamp_ > component_timeout_s_) {
      std::snprintf(buf, sizeof(buf), "보드 상태 답이 %.1f s 동안 없음", now_s - comp_stamp_);
      return buf;
    }
    return "보드 이상: " + comp_detail_;
  }
  if (!have_) {return "관절 측정(/joint_states)이 아직 없음";}
  if (now_s - last_stamp_ > state_timeout_s_) {
    std::snprintf(buf, sizeof(buf), "관절 측정이 %.2f s 동안 안 옴", now_s - last_stamp_);
    return buf;
  }
  if (bad_since_ >= 0.0) {
    std::snprintf(buf, sizeof(buf), "관절 측정 없음(NaN · 빠짐) %.2f s", last_stamp_ - bad_since_);
    return buf;
  }
  return "정상";
}

}  // namespace nt_control
