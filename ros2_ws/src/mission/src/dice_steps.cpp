// dice_steps.cpp — dice 흐름 단계 만들기 구현, 설명은 dice_steps.hpp

#include "mission/dice_steps.hpp"

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

std::vector<Step> transfer_steps(
  const TransferPlan & plan, const DiceMap & arena, bool rack_look, bool box_wait)
{
  std::vector<Step> steps;
  for (const auto & st : plan.stops) {
    if (st.move) {
      steps.push_back(nav_step(node_stop(arena, st.node), node_name(st.node), st.node));
      set_box_approach(steps.back(), arena, st.node);   // (옛) 라이다 마지막 접근, box_approach_m 0 이면 안 씀 (정밀 접근 dock 이 대신)
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
