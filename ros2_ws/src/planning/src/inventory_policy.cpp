// inventory_policy.cpp — 재고 불확실 판정 · 정합 구현, 설명은 inventory_policy.hpp

#include "planning/inventory_policy.hpp"
#include "planning/types.hpp"

namespace planning
{

const char * doubt_fix_name(DoubtFix f)
{
  switch (f) {
    case DoubtFix::Before: return "before";
    case DoubtFix::Done: return "done";
    case DoubtFix::Removed: return "removed";
  }
  return "?";
}

bool arm_task_risks_inventory(uint8_t task)
{
  return task == kArmPick || task == kArmStore || task == kArmRetrieve || task == kArmPlace || task == kArmLookRack;
}

bool inventory_doubt_after(uint8_t task, ArmEnd end)
{
  return arm_task_risks_inventory(task) &&
         (end == ArmEnd::Failed || end == ArmEnd::Unknown || end == ArmEnd::Mismatch);
}

CargoDoubt make_doubt(const Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  const CargoItem & pick_item, const std::string & step_name, const std::string & reason)
{
  CargoDoubt d;
  d.arm_task = task;
  d.puck_id = puck_id;
  d.slot = slot;
  d.step_name = step_name;
  d.reason = reason;
  if (task == kArmPick) {
    d.item = pick_item;
  } else if (task == kArmRetrieve) {
    for (const auto & s : cargo.slots()) {
      if (s.first == slot && s.second) {d.item = *s.second;}
    }
  } else if (cargo.gripper()) {
    d.item = *cargo.gripper();
  }
  if (d.item.puck_id.empty()) {d.item.puck_id = puck_id;}   // 기록에 없음 → 단계의 퍽 ID 만
  return d;
}

bool doubt_matches(const CargoDoubt & d, uint8_t task, const std::string & puck_id, const std::string & slot)
{
  return d.arm_task == task && d.puck_id == puck_id && d.slot == slot;
}

bool arm_preflight_ok(const Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  std::string * error)
{
  if (!arm_task_risks_inventory(task)) {return true;}
  if (puck_id.empty()) {
    set_error(error, "단계 퍽 ID 없음");
    return false;
  }
  const auto & g = cargo.gripper();
  const std::string held = g ? g->puck_id : "";
  const std::optional<CargoItem> * cell = nullptr;
  for (const auto & s : cargo.slots()) {
    if (s.first == slot) {cell = &s.second;}
  }
  switch (task) {
    case kArmPick:
      if (g) {
        set_error(error, "그리퍼 기록에 이미 퍽 " + held + " 있음");
        return false;
      }
      for (const auto & s : cargo.slots()) {
        if (s.second && s.second->puck_id == puck_id) {
          set_error(error, "퍽 " + puck_id + " 이 이미 칸 " + s.first + " 기록에 있음");
          return false;
        }
      }
      return true;
    case kArmStore:
    case kArmLookRack:
    case kArmPlace:
      if (held != puck_id) {
        set_error(error, "그리퍼 기록 " + (g ? "퍽 " + held : std::string("빔")) + " ≠ 단계 퍽 " + puck_id);
        return false;
      }
      if (task == kArmPlace) {return true;}
      if (!cell) {
        set_error(error, "없는 칸: " + slot);
        return false;
      }
      if (task == kArmStore && *cell) {
        set_error(error, "칸 " + slot + " 기록에 이미 퍽 " + (*cell)->puck_id + " 있음");
        return false;
      }
      return true;
    case kArmRetrieve:
      if (g) {
        set_error(error, "그리퍼 기록에 이미 퍽 " + held + " 있음");
        return false;
      }
      if (!cell) {
        set_error(error, "없는 칸: " + slot);
        return false;
      }
      if (!*cell || (*cell)->puck_id != puck_id) {
        set_error(error, "칸 " + slot + " 기록 " + (*cell ? "퍽 " + (*cell)->puck_id : std::string("빔")) +
          " ≠ 단계 퍽 " + puck_id);
        return false;
      }
      return true;
    default:
      return true;
  }
}

bool apply_arm_success(Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  const CargoItem & pick_item, ArmSuccessApplied * out, std::string * error)
{
  if (!arm_preflight_ok(cargo, task, puck_id, slot, error)) {return false;}
  ArmSuccessApplied r;
  Cargo next = cargo;   // 전환이 실패하면 원래 기록 · 표시 그대로
  bool ok = true;
  switch (task) {
    case kArmPick:
      ok = next.on_pick(pick_item, error);
      r.puck_state = "in_gripper";
      break;
    case kArmStore:
      ok = next.on_store(slot, error);
      r.puck_state = "in_cargo:" + slot;
      break;
    case kArmRetrieve:
      ok = next.on_retrieve(slot, error);
      r.puck_state = "in_gripper";
      break;
    case kArmPlace:
      r.placed = next.on_place(error);
      ok = r.placed.has_value();
      r.puck_state = "delivered";
      break;
    default:
      break;   // HOME · LOOK 류 · LOOK_RACK: 재고 전환 없음
  }
  if (!ok) {return false;}
  r.changed = !r.puck_state.empty();
  if (next.doubt() && doubt_matches(*next.doubt(), task, puck_id, slot)) {   // control 최종 성공 = 실물 확인
    next.clear_doubt();
    r.doubt_cleared = true;
    r.changed = true;
  }
  cargo = next;
  if (out) {*out = r;}
  return true;
}

bool resolve_doubt(Cargo & cargo, DoubtFix fix, std::string * puck_id, std::string * puck_state, std::string * error)
{
  if (!cargo.doubt()) {
    set_error(error, "재고 불확실 표시 없음");
    return false;
  }
  const CargoDoubt d = *cargo.doubt();
  const std::string id = d.item.puck_id.empty() ? d.puck_id : d.item.puck_id;
  std::string state;
  Cargo next = cargo;   // 전환이 실패하면 원래 기록 그대로
  switch (fix) {
    case DoubtFix::Before:   // 기록 = 마지막 확인 재고 그대로
      break;
    case DoubtFix::Done: {
        bool ok = false;
        switch (d.arm_task) {
          case kArmPick:
            ok = next.on_pick(d.item, error);
            state = "in_gripper";
            break;
          case kArmStore:
            ok = next.on_store(d.slot, error);
            state = "in_cargo:" + d.slot;
            break;
          case kArmRetrieve:
            ok = next.on_retrieve(d.slot, error);
            state = "in_gripper";
            break;
          case kArmPlace:
            ok = next.on_place(error).has_value();
            state = "delivered";   // 사람이 칸 위 퍽을 확인함
            break;
          default:
            set_error(error, d.step_name + " 은 재고 전환이 없음 → before 또는 removed");
            return false;
        }
        if (!ok) {return false;}
        break;
      }
    case DoubtFix::Removed:   // 사람이 그 퍽을 로봇에서 뺌 → 그리퍼 · 그 칸 기록에서 그 퍽만 지움
      if (id.empty()) {
        set_error(error, "퍽 ID 모름 → removed 못 함");
        return false;
      }
      if (next.gripper() && next.gripper()->puck_id == id) {next.clear_gripper();}
      for (const auto & s : cargo.slots()) {
        if (s.first == d.slot && s.second && s.second->puck_id == id) {next.clear_slot(s.first);}
      }
      state = "removed";
      break;
  }
  next.clear_doubt();
  cargo = next;
  if (puck_id) {*puck_id = id;}
  if (puck_state) {*puck_state = state;}
  return true;
}

}  // namespace planning
