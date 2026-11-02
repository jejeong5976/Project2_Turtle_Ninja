// transfer_planner.cpp — 퍽 수집 · 배달 정차 목록 구현, 설명은 transfer_planner.hpp

#include "mission/transfer_planner.hpp"
#include "mission/common.hpp"

#include <yaml-cpp/yaml.h>

namespace mission
{

// ---------------------------------------------------------------- transfer_types.hpp 도우미

std::string node_name(int n)
{
  if (n == kNodeStart) {return "S";}
  if (is_box_node(n)) {return "B" + std::to_string(n);}
  if (is_place_node(n)) {return "L" + std::to_string(node_place(n) + 1);}
  return "?";
}

std::optional<PuckKind> puck_kind(const std::string & color, const std::string & shape)
{
  int base = 0;
  if (color == "red") {base = 0;} else if (color == "green") {base = 2;} else {return std::nullopt;}
  if (shape == "cylinder") {return static_cast<PuckKind>(base);}
  if (shape == "box") {return static_cast<PuckKind>(base + 1);}
  return std::nullopt;
}

const char * kind_color(PuckKind k) {return kind_is_red(k) ? "red" : "green";}
const char * kind_shape(PuckKind k) {return kind_is_box(k) ? "box" : "cylinder";}

// ---------------------------------------------------------------- 내부

namespace
{

CargoItem item_of(const TransferPuck & p)
{
  CargoItem it;
  it.puck_id = p.id;
  it.color = kind_color(p.kind);
  it.shape = kind_shape(p.kind);
  return it;
}

ArmAction action_of(ArmOp op, const CargoItem & it)
{
  ArmAction a;
  a.op = op;
  a.puck_id = it.puck_id;
  a.color = it.color;
  a.shape = it.shape;
  return a;
}

}  // namespace

bool fill_carried(const Cargo & cargo, SolverInput & in, std::string * error)
{
  in.gripper.reset();
  in.in_cargo.clear();
  if (cargo.gripper()) {
    const auto & g = *cargo.gripper();
    const auto k = puck_kind(g.color, g.shape);
    if (!k) {set_error(error, "그리퍼 퍽 " + g.puck_id + " 색 · 모양 모름"); return false;}
    in.gripper = TransferPuck{g.puck_id, *k, 0};
  }
  for (const auto & s : cargo.slots()) {
    if (!s.second) {continue;}
    const auto k = puck_kind(s.second->color, s.second->shape);
    if (!k) {set_error(error, "칸 " + s.first + " 퍽 " + s.second->puck_id + " 색 · 모양 모름"); return false;}
    in.in_cargo.push_back(TransferPuck{s.second->puck_id, *k, 0});
  }
  in.cargo_capacity = static_cast<int>(cargo.capacity());
  return true;
}

namespace
{

// 대체 해: 남은 퍽을 가까운 박스 순으로 다 싣고 (A안과 같은 방식) → 배달은 정확해
std::optional<TransferPlan> collect_then_deliver(
  const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error)
{
  const int carried = (cargo.gripper() ? 1 : 0) + static_cast<int>(cargo.stored_count());
  if (carried + static_cast<int>(in.on_boxes.size()) > static_cast<int>(cargo.capacity()) + 1) {
    set_error(error, "대체 해: 적재함 칸이 모자라 남은 퍽을 다 실을 수 없음");
    return std::nullopt;
  }
  TransferPlan plan;
  Cargo sim = cargo;
  int pos = in.position;
  bool stopped = in.stopped;
  double cost = 0.0;
  std::vector<bool> done(in.on_boxes.size(), false);
  while (true) {
    // 남은 퍽이 있는 박스 중 가장 가까운 곳 (지금 정차 중인 박스가 있으면 거기부터)
    int best = -1;
    double best_t = 0.0;
    for (std::size_t i = 0; i < in.on_boxes.size(); ++i) {
      if (done[i]) {continue;}
      const int b = in.on_boxes[i].box;
      const double t = (b == pos && stopped) ? -1.0 : costs.travel_s[pos][b];
      if (best < 0 || t < best_t) {best = b; best_t = t;}
    }
    if (best < 0) {break;}
    Stop stop;
    stop.node = best;
    stop.move = !(best == pos && stopped);
    stop.name = node_name(best) + " 수집";
    if (stop.move) {
      ArmAction look;
      look.op = ArmOp::Look;
      look.box = best;
      stop.actions.push_back(look);
      cost += costs.travel_s[pos][best] + costs.stop_s + costs.look_s;
    }
    std::vector<TransferPuck> here;
    for (std::size_t i = 0; i < in.on_boxes.size(); ++i) {
      if (!done[i] && in.on_boxes[i].box == best) {here.push_back(in.on_boxes[i]); done[i] = true;}
    }
    const auto acts = collect_box_actions(here, sim, error);
    if (acts.empty()) {return std::nullopt;}
    for (const auto & a : acts) {
      if (a.op == ArmOp::Store) {sim.on_store(a.slot); cost += costs.store_s;}
      if (a.op == ArmOp::Pick) {sim.on_pick(CargoItem{a.puck_id, a.color, a.shape, "", false}); cost += costs.pick_s;}
      stop.actions.push_back(a);
    }
    plan.stops.push_back(stop);
    pos = best;
    stopped = true;
  }
  SolverInput rest = in;
  rest.position = pos;
  rest.stopped = stopped;
  rest.on_boxes.clear();
  if (!fill_carried(sim, rest, error)) {return std::nullopt;}
  auto tail = plan_transfer(rest, costs, sim, cells_done, error, 0);
  if (!tail) {return std::nullopt;}
  // 마지막 수집 정차에서 이어서 하는 동작은 그 정차에 붙임
  auto it = tail->stops.begin();
  if (it != tail->stops.end() && !it->move && !plan.stops.empty() && plan.stops.back().node == it->node) {
    auto & last = plan.stops.back().actions;
    last.insert(last.end(), it->actions.begin(), it->actions.end());
    ++it;
  }
  plan.stops.insert(plan.stops.end(), it, tail->stops.end());
  plan.cost_s = cost + tail->cost_s;
  plan.yellow_shape = tail->yellow_shape;
  plan.verify_order = tail->verify_order;
  plan.place_of = tail->place_of;
  plan.expanded = tail->expanded;
  return plan;
}

}  // namespace

std::optional<TransferPlan> plan_transfer(
  const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error, int max_remaining)
{
  if (static_cast<int>(in.on_boxes.size()) > max_remaining && !in.on_boxes.empty()) {
    return collect_then_deliver(in, costs, cargo, cells_done, error);
  }
  auto sol = solve_transfer(in, costs, error);
  if (!sol) {return std::nullopt;}

  TransferPlan plan;
  plan.cost_s = sol->cost_s;
  plan.yellow_shape = sol->yellow_shape;
  plan.verify_order = sol->verify_order;
  plan.expanded = sol->expanded;

  // 실제 상태를 따라가며 ID · 칸 이름을 붙임
  Cargo sim = cargo;
  std::vector<TransferPuck> boxes = in.on_boxes;
  std::vector<bool> used(boxes.size(), false);
  std::array<int, kNumPlaces> next_cell = cells_done;

  Stop cur;
  cur.node = in.position;
  cur.move = false;
  cur.name = node_name(in.position) + " (이어서)";
  bool cur_open = in.stopped;

  auto close = [&]() {
      if (cur_open && (!cur.actions.empty() || cur.move)) {plan.stops.push_back(cur);}
      cur_open = false;
    };

  for (const auto & st : sol->steps) {
    switch (st.act) {
      case SolverAct::Move: {
          close();
          cur = Stop{};
          cur.node = st.node;
          cur.move = true;
          ArmAction look;
          look.op = ArmOp::Look;
          if (is_box_node(st.node)) {look.box = st.node; cur.name = node_name(st.node) + " 수집";}
          if (is_place_node(st.node)) {look.place = node_place(st.node); cur.name = node_name(st.node) + " 배달";}
          cur.actions.push_back(look);
          cur_open = true;
          break;
        }
      case SolverAct::Pick: {
          // 이 박스에 남은 퍽 중 on_boxes 순서 첫 번째 = 가장 가까운 퍽 (solver 도 이것만 집음)
          std::size_t idx = boxes.size();
          for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (!used[i] && boxes[i].box == st.node) {idx = i; break;}
          }
          if (idx == boxes.size() || boxes[idx].kind != st.kind) {set_error(error, "펼치기 오류: 집을 퍽 없음"); return std::nullopt;}
          used[idx] = true;
          const CargoItem it = item_of(boxes[idx]);
          if (!sim.on_pick(it, error)) {return std::nullopt;}
          ArmAction a = action_of(ArmOp::Pick, it);
          a.box = st.node;
          cur.actions.push_back(a);
          break;
        }
      case SolverAct::Store: {
          const auto slot = sim.free_slot();
          if (!slot || !sim.gripper()) {set_error(error, "펼치기 오류: 넣을 칸 · 퍽 없음"); return std::nullopt;}
          ArmAction a = action_of(ArmOp::Store, *sim.gripper());
          a.slot = *slot;
          if (!sim.on_store(*slot, error)) {return std::nullopt;}
          cur.actions.push_back(a);
          break;
        }
      case SolverAct::Retrieve: {
          // 칸 목록은 팔에서 먼 칸부터 (채우는 순서) → 꺼낼 때는 뒤에서부터 = 가까운 칸 먼저
          std::optional<std::string> slot;
          const auto & slots = sim.slots();
          for (auto s = slots.rbegin(); s != slots.rend(); ++s) {
            if (s->second && puck_kind(s->second->color, s->second->shape) == st.kind) {slot = s->first; break;}
          }
          if (!slot) {set_error(error, "펼치기 오류: 꺼낼 종류가 칸에 없음"); return std::nullopt;}
          if (!sim.on_retrieve(*slot, error)) {return std::nullopt;}
          ArmAction a = action_of(ArmOp::Retrieve, *sim.gripper());
          a.slot = *slot;
          cur.actions.push_back(a);
          break;
        }
      case SolverAct::Place: {
          const auto it = sim.on_place(error);
          if (!it) {return std::nullopt;}
          ArmAction a = action_of(ArmOp::Place, *it);
          a.place = st.place;
          a.cell = next_cell[st.place]++;
          plan.place_of[it->puck_id] = st.place;
          cur.actions.push_back(a);
          break;
        }
    }
  }
  close();
  return plan;
}

std::vector<ArmAction> collect_box_actions(
  const std::vector<TransferPuck> & at_box, const Cargo & cargo, std::string * error)
{
  std::vector<ArmAction> out;
  Cargo sim = cargo;
  for (const auto & p : at_box) {
    if (sim.gripper()) {
      const auto slot = sim.free_slot();
      if (!slot) {set_error(error, "적재함 빈 칸 없음"); return {};}
      ArmAction s = action_of(ArmOp::Store, *sim.gripper());
      s.slot = *slot;
      if (!sim.on_store(*slot, error)) {return {};}
      out.push_back(s);
    }
    const CargoItem it = item_of(p);
    if (!sim.on_pick(it, error)) {return {};}
    ArmAction a = action_of(ArmOp::Pick, it);
    a.box = p.box;
    out.push_back(a);
  }
  return out;
}

bool load_transfer_costs(const std::string & path, TransferCosts & out, std::string * error)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    set_error(error, std::string("비용표 읽기 실패: ") + e.what());
    return false;
  }
  static const char * kNames[kNumNodes] = {"S", "B1", "B2", "B3", "B4", "B5", "B6", "L1", "L2", "L3"};
  const auto nodes = y["nodes"];
  if (!nodes || nodes.size() != kNumNodes) {set_error(error, "nodes 는 10개"); return false;}
  for (int i = 0; i < kNumNodes; ++i) {
    if (nodes[i].as<std::string>() != kNames[i]) {set_error(error, "nodes 순서는 S, B1–B6, L1–L3"); return false;}
  }
  const auto t = y["travel_s"];
  if (!t || t.size() != kNumNodes) {set_error(error, "travel_s 는 10줄"); return false;}
  TransferCosts c = out;
  for (int a = 0; a < kNumNodes; ++a) {
    if (t[a].size() != kNumNodes) {set_error(error, "travel_s 각 줄은 10개"); return false;}
    for (int b = 0; b < kNumNodes; ++b) {
      const double v = t[a][b].as<double>();
      if (!(v >= 0.0)) {set_error(error, "travel_s 음수 · NaN"); return false;}
      c.travel_s[a][b] = v;
    }
  }
  c.stop_s = y["stop_s"].as<double>(c.stop_s);
  c.look_s = y["look_s"].as<double>(c.look_s);
  c.pick_s = y["pick_s"].as<double>(c.pick_s);
  c.place_s = y["place_s"].as<double>(c.place_s);
  c.store_s = y["store_s"].as<double>(c.store_s);
  c.retrieve_s = y["retrieve_s"].as<double>(c.retrieve_s);
  c.verify_tour = y["verify_tour"].as<bool>(c.verify_tour);
  out = c;
  return true;
}

}  // namespace mission
