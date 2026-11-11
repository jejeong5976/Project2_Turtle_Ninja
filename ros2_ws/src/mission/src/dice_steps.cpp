// dice_steps.cpp — dice 흐름 단계 만들기 구현, 설명은 dice_steps.hpp

#include "mission/dice_steps.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace mission
{

namespace
{
// node: 도착 노드 (0 = S, 1–6 = B, 7–9 = L) → box · place 에 적어 둠 (노드가 재계획 때 지금 위치로 씀)
Step nav_step(const Pose2D & p, const std::string & name, int node)
{
  Step s;
  s.type = StepType::Navigate;
  s.target = Target::Fixed;
  s.pose = p;
  s.name = "nav:" + name;
  if (is_box_node(node)) {box_of(s) = node;}
  if (is_place_node(node)) {place_of(s) = node_place(node);}
  s.dock = is_box_node(node) || is_place_node(node);   // 박스 · 장소 정차 = 면 1 cm → 중간 정차 + 정밀 접근
  return s;
}

// 박스 정차면 마지막 접근 (Nav2 는 box_approach_m 앞에서 멈추고 라이다로 면까지 붙음). 멈출 면 거리 = 정차 ↔ 박스 중심 − 반 변
void set_box_approach(Step & s, const DiceMap & arena, int node)
{
  if (!is_box_node(node) || arena.box_approach_m <= 0.0) {return;}
  const auto & b = arena.boxes[node - 1];
  s.approach_m = arena.box_approach_m;
  s.approach_face_m = std::hypot(b.center[0] - b.stop.x, b.center[1] - b.stop.y) - arena.box_size / 2.0;
}

// 관찰 자세: 박스 · 장소 모두 LOOK
Step look_step(int box, int place)
{
  Step s;
  s.type = StepType::Arm;
  s.arm_task = kArmLook;
  box_of(s) = box;
  place_of(s) = place;
  s.name = std::string("arm:LOOK:") + (box > 0 ? "B" + std::to_string(box) : "L" + std::to_string(place + 1));
  return s;
}
}  // namespace

std::vector<Step> collect_steps(const DiceMap & arena, const std::vector<int> & order, std::string * error)
{
  std::vector<Step> steps;
  std::set<int> seen;
  for (int b : order) {
    if (b < 1 || b > kNumBoxes || !seen.insert(b).second) {
      set_error(error, "collect_order 는 1–6 중복 없이: " + std::to_string(b));
      return {};
    }
  }
  if (order.empty()) {
    set_error(error, "collect_order 비어 있음");
    return {};
  }
  for (std::size_t i = 0; i < order.size(); ++i) {
    const int b = order[i];
    steps.push_back(nav_step(arena.boxes[b - 1].stop, "B" + std::to_string(b), b));
    set_box_approach(steps.back(), arena, b);
    steps.push_back(look_step(b, -1));
    Step plan;
    plan.type = StepType::Custom;
    plan.kind = i + 1 == order.size() ? kPlanDelivery : kPlanCollectBox;
    box_of(plan) = b;
    plan.name = std::string(plan.kind == kPlanDelivery ? "plan:delivery:B" : "plan:collect:B") + std::to_string(b);
    steps.push_back(plan);
  }
  return steps;
}

std::vector<Step> arm_steps(
  const std::vector<ArmAction> & actions, bool rack_look, bool box_wait)
{
  std::vector<Step> steps;
  for (const auto & a : actions) {
    Step s;
    s.type = StepType::Arm;
    s.puck_id = a.puck_id;
    switch (a.op) {
      case ArmOp::Look:
        steps.push_back(look_step(a.box, a.place));
        if (box_wait && a.box <= 0 && a.place >= 0 && a.place < kNumPlaces) {   // 장소: 박스 측정으로 칸 기준 확정
          Step w;
          w.type = StepType::Custom;
          w.kind = kPlaceBox;
          place_of(w) = a.place;
          w.name = "box:L" + std::to_string(a.place + 1);
          steps.push_back(w);
        }
        continue;
      case ArmOp::Pick:
        s.arm_task = kArmPick;
        s.target = Target::Puck;
        box_of(s) = a.box;
        s.name = "arm:PICK:" + a.puck_id;
        break;
      case ArmOp::Store:
        if (rack_look) {   // 퍽 쥔 채 그 칸 관찰 → 칸 측정
          Step look = s;
          look.arm_task = kArmLookRack;
          look.slot = a.slot;
          look.name = "arm:LOOK_RACK:" + a.slot;
          steps.push_back(look);
        }
        s.arm_task = kArmStore;
        s.slot = a.slot;
        s.name = "arm:STORE:" + a.puck_id + ">" + a.slot;
        break;
      case ArmOp::Retrieve:
        s.arm_task = kArmRetrieve;
        s.slot = a.slot;
        s.name = "arm:RETRIEVE:" + a.slot + ">" + a.puck_id;
        break;
      case ArmOp::Place:
        s.arm_task = kArmPlace;
        s.target = Target::Custom;
        s.kind = kTargetDiceCell;
        place_of(s) = a.place;
        cell_of(s) = a.cell;
        s.name = "arm:PLACE:" + a.puck_id + ">L" + std::to_string(a.place + 1) + "#" + std::to_string(a.cell);
        break;
    }
    steps.push_back(s);
  }
  return steps;
}

bool is_side_slot(const std::string & slot)
{
  return slot == "right" || slot == "left" || slot == "near" || slot == "far";
}

bool is_side_look(const Step & s)
{
  return s.type == StepType::Arm && s.arm_task == kArmLook && is_side_slot(s.slot);
}

bool is_pick_relook(const Step & s)
{
  // 실행기 insert_relook 이 복사한 것 (puck_id 붙음) 은 아님 → 그 실패는 예전 처리 그대로
  return s.type == StepType::Arm && s.arm_task == kArmLook && s.kind == kPickRelook && s.puck_id.empty();
}

bool starts_box_look(const Step & s)
{
  return s.type == StepType::Arm && is_look_task(s.arm_task) && box_of(s) >= 1 && box_of(s) <= kNumBoxes &&
         !is_side_look(s) && !is_pick_relook(s) && s.puck_id.empty();
}

std::vector<Step> side_look_steps(const Step & plan, std::size_t seen, const SideLookConfig & cfg)
{
  const int b = box_of(plan);
  const bool box_plan = is_custom(plan, kPlanCollectBox) || is_custom(plan, kPlanDelivery);
  if (!cfg.enabled || !box_plan || b < 1 || b > kNumBoxes || static_cast<int>(seen) >= cfg.expected) {return {};}
  std::vector<std::string> slots;
  for (const auto & o : cfg.order) {
    if (is_side_slot(o)) {slots.push_back(o);}
  }
  if (slots.empty()) {return {};}
  const std::string bn = "B" + std::to_string(b);
  std::vector<Step> out;
  for (std::size_t i = 0; i < slots.size(); ++i) {
    Step lk = look_step(b, -1);
    lk.slot = slots[i];
    lk.name += ":" + slots[i];
    out.push_back(lk);
    if (i + 1 < slots.size()) {   // 다음 옆 보기 전에 이 자세로 검출 모음
      Step w;
      w.type = StepType::Custom;
      w.kind = kLookDwell;
      box_of(w) = b;
      w.name = "wait:" + bn + ":" + slots[i];
      out.push_back(w);
    }
  }
  Step again = plan;
  again.name += ":side";
  out.push_back(again);
  return out;
}

std::vector<Step> side_look_on_plan(SideLookState & st, const Step & plan, std::size_t seen, const SideLookConfig & cfg)
{
  if (st != SideLookState::Unchecked) {
    st = SideLookState::Done;
    return {};
  }
  auto more = side_look_steps(plan, seen, cfg);
  st = more.empty() ? SideLookState::Done : SideLookState::Pending;
  return more;
}

std::optional<Step> box_relook_step(const std::vector<Step> & steps, std::size_t at, int box, const std::string & puck_id)
{
  if (box < 1 || box > kNumBoxes) {return std::nullopt;}
  for (std::size_t j = std::min(at, steps.size()); j-- > 0;) {
    const Step & s = steps[j];
    if (s.type == StepType::Navigate) {break;}   // 앞 정차
    if (s.type != StepType::Arm || !is_look_task(s.arm_task) || is_side_look(s) || box_of(s) != box) {continue;}
    Step look = s;   // 같은 자세 · 목표 그대로 (실행기 · PICK 앞 다시 보기 LOOK 이었어도 같은 앞 LOOK)
    look.kind = kNoKind;   // PICK 앞 다시 보기 표시는 안 물려받음 (실패 처리 = 예전 빈손 뒤 다시 보기 그대로)
    look.puck_id = puck_id.empty() ? std::string("?") : puck_id;   // 비면 미션이 새 정차로 보고 옆 보기를 되돌림
    look.name = "arm:LOOK:B" + std::to_string(box) + "(빈손 뒤 다시 보기:" + look.puck_id + ")";
    return look;
  }
  return std::nullopt;
}

PuckViews puck_views(const std::vector<LookWindow> & windows, const std::vector<SeenAt> & seen)
{
  PuckViews out;
  for (const auto & w : windows) {
    for (const auto & o : seen) {
      if (o.id.empty() || o.stamp < w.from || (w.to >= 0.0 && o.stamp > w.to)) {continue;}
      auto & v = out[o.id];
      if (std::find(v.begin(), v.end(), w.slot) == v.end()) {v.push_back(w.slot);}
    }
  }
  return out;
}

namespace
{
// 그 퍽을 그 slot 자세에서 봤나. 기록이 없으면 앞 LOOK ("") 에서 본 것으로 봄
bool seen_from(const PuckViews & views, const std::string & id, const std::string & slot)
{
  const auto it = views.find(id);
  if (it == views.end() || it->second.empty()) {return slot.empty();}
  return std::find(it->second.begin(), it->second.end(), slot) != it->second.end();
}
}  // namespace

std::string relook_slot(const PuckViews & views, const std::string & id)
{
  const auto it = views.find(id);
  if (it == views.end() || seen_from(views, id, "")) {return "";}
  for (const auto & slot : it->second) {
    if (is_side_slot(slot)) {return slot;}
  }
  return "";
}

std::vector<Step> with_pick_relooks(
  const std::vector<Step> & steps, const std::optional<Step> & held, const PuckViews & views)
{
  std::vector<Step> out;
  std::optional<Step> arm = held;   // 지금 팔 자세를 정한 마지막 팔 단계
  for (const auto & s : steps) {
    if (s.type == StepType::Navigate) {arm.reset();}
    const int b = box_of(s);
    if (s.type == StepType::Arm && s.arm_task == kArmPick && b >= 1 && b <= kNumBoxes) {
      const bool sees = arm && arm->type == StepType::Arm && arm->arm_task == kArmLook && box_of(*arm) == b &&
        seen_from(views, s.puck_id, arm->slot);
      if (!sees) {
        Step lk = look_step(b, -1);
        lk.slot = relook_slot(views, s.puck_id);
        lk.kind = kPickRelook;
        lk.name += (lk.slot.empty() ? std::string() : ":" + lk.slot) + "(집기 전 다시 보기:" + s.puck_id + ")";
        Step w;
        w.type = StepType::Custom;
        w.kind = kRelookDwell;
        box_of(w) = b;
        w.puck_id = s.puck_id;
        w.name = "wait:B" + std::to_string(b) + ":" + s.puck_id;
        out.push_back(lk);
        out.push_back(w);
      }
    }
    out.push_back(s);
    if (s.type == StepType::Arm) {arm = s;}
  }
  return out;
}

std::vector<Step> transfer_steps(
  const TransferPlan & plan, const DiceMap & arena, bool rack_look, bool box_wait)
{
  std::vector<Step> steps;
  for (const auto & st : plan.stops) {
    if (st.move) {
      steps.push_back(nav_step(node_stop(arena, st.node), node_name(st.node), st.node));
      set_box_approach(steps.back(), arena, st.node);   // 라이다 마지막 접근 (기본 꺼짐), box_approach_m 0 이면 안 씀 (정밀 접근 dock 이 대신)
    }
    auto arm = arm_steps(st.actions, rack_look, box_wait);
    steps.insert(steps.end(), arm.begin(), arm.end());
  }
  return steps;
}

std::vector<Step> verify_steps(const std::array<int, kNumPlaces> & verify_order, const DiceMap & arena)
{
  std::vector<Step> steps;
  for (int p : verify_order) {
    if (p < 0 || p >= kNumPlaces) {continue;}
    auto nav = nav_step(arena.places[p].stop, "verify:L" + std::to_string(p + 1), place_node(p));
    steps.push_back(nav);
    steps.push_back(look_step(-1, p));
    Step v;
    v.type = StepType::Custom;
    v.kind = kPlanVerify;
    place_of(v) = p;
    v.name = "verify:L" + std::to_string(p + 1);
    steps.push_back(v);
  }
  return steps;
}

std::vector<Step> build_read_steps(const Pose2D & barcode_point, double read_timeout_s, bool look)
{
  Step go = nav_step(barcode_point, "barcode", -1);
  Step read;
  read.type = StepType::Custom;
  read.kind = kReadBarcode;
  read.timeout_s = read_timeout_s;
  read.name = "read_barcode";
  if (!look) {return {go, read};}
  Step lk;
  lk.type = StepType::Arm;
  lk.arm_task = kArmLookBarcode;
  lk.name = "arm:LOOK_BARCODE";
  Step home;
  home.type = StepType::Arm;
  home.arm_task = kArmHome;
  home.name = "arm:HOME";
  return {go, lk, read, home};
}

std::vector<Step> build_return_steps(const Pose2D & start_pose)
{
  return {nav_step(start_pose, "start", kNodeStart)};
}

std::optional<std::array<double, 2>> dice_cell_point(
  const DicePlan & plan, const DiceMap & arena, int place, int cell)
{
  if (place < 0 || place >= kNumPlaces) {return std::nullopt;}
  const auto & offs = plan.places[place].pip_offsets;
  if (cell < 0 || cell >= static_cast<int>(offs.size())) {return std::nullopt;}
  const auto & c = arena.places[place].center;
  return std::array<double, 2>{c[0] + offs[cell][0], c[1] + offs[cell][1]};
}

std::array<std::array<double, 2>, 3> place_view_dirs(const DiceMap & arena)
{
  return {arena.places[0].view_dir, arena.places[1].view_dir, arena.places[2].view_dir};
}

std::array<double, 2> place_offset_map(const std::array<double, 2> & view_dir, const std::array<double, 2> & fwd_left_mm)
{
  const double n = std::hypot(view_dir[0], view_dir[1]);
  if (!(n > 1e-9)) {return {0.0, 0.0};}
  const double fx = view_dir[0] / n, fy = view_dir[1] / n;   // 앞, 왼쪽 = (−fy, fx)
  const double f = fwd_left_mm[0] / 1000.0, l = fwd_left_mm[1] / 1000.0;
  return {f * fx - l * fy, f * fy + l * fx};
}

}  // namespace mission
