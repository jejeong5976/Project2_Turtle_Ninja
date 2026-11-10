// transfer_planner.cpp — 퍽 수집 · 배달 정차 목록 구현, 설명은 transfer_planner.hpp

#include "mission/transfer_planner.hpp"
#include "mission/common.hpp"

#include <yaml-cpp/yaml.h>

#include <functional>
#include <set>
#include <unordered_set>

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

// 옛 펼치기 (칸 제약을 다 지키는 배정이 없을 때만): 꺼낼 칸 = 그 종류 중 뒤 칸, 막히면 막는 퍽을 빈 칸으로 옮김, 그래도 안 되면 막지 않음
std::optional<TransferPlan> expand_greedy(
  const SolverResult & sol, const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error);

// 대체 해: 남은 퍽을 가까운 박스 순으로 다 싣고 (A안과 같은 방식) → 배달은 정확해
std::optional<TransferPlan> collect_then_deliver(
  const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error)
{
  const int carried = (cargo.gripper() ? 1 : 0) + static_cast<int>(cargo.stored_count());
  if (carried + static_cast<int>(in.on_boxes.size()) > static_cast<int>(cargo.capacity())) {
    set_error(error, "대체 해: 적재함 칸이 모자라 남은 퍽을 다 실을 수 없음");
    return std::nullopt;
  }
  TransferPlan plan;
  Cargo sim = cargo;
  int pos = in.position;
  bool stopped = in.stopped;
  double cost = 0.0;
  std::vector<bool> done(in.on_boxes.size(), false);
  if (sim.gripper()) {   // 쥔 퍽은 출발 전에 지금 자리에서 넣음 (퍽 쥔 채 주행 안 함)
    const auto slot = sim.free_slot_for(*sim.gripper());
    if (!slot) {set_error(error, "대체 해: 쥔 퍽을 넣을 칸 없음"); return std::nullopt;}
    Stop pre;
    pre.node = pos;
    pre.move = false;
    pre.name = node_name(pos) + " (이어서)";
    ArmAction s = action_of(ArmOp::Store, *sim.gripper());
    s.slot = *slot;
    if (!sim.on_store(*slot, error)) {return std::nullopt;}
    pre.actions.push_back(s);
    cost += costs.store_s;
    plan.stops.push_back(pre);
  }
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
    if (!stop.move && !plan.stops.empty() && !plan.stops.back().move && plan.stops.back().node == stop.node) {
      auto & last = plan.stops.back().actions;
      last.insert(last.end(), stop.actions.begin(), stop.actions.end());
      plan.stops.back().name = stop.name;
    } else {
      plan.stops.push_back(stop);
    }
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

// 칸 제약을 지키는 펼치기: 종류 단위 해 (정차 순서 · 장소마다 놓을 수) 는 그대로, 퍽 · 칸은 DFS 로 고름
//   - 꺼내기: 그 장소 그룹에 맞는 퍽 (R 빨강 · G 초록 · Y 정한 모양) 이 든 칸 중 빼기 제약을 지키는 칸, 목록 뒤 칸 먼저
//     (해가 고른 종류와 달라도 됨 = Y 에 빨강 · 초록 어느 쪽을 쓸지, 같은 장소 안 순서도 여기서 정해짐)
//   - 넣기: 넣기 제약을 지키는 빈 칸, free_slot_for 가 고르는 칸 먼저
//   - 그런 배정이 없으면 nullopt (부르는 쪽이 다른 해 → 옛 펼치기)
//   relocs = 꺼내기 전에 막는 퍽을 빈 칸으로 옮기는 횟수 한도 (옮길 때마다 RETRIEVE + STORE 시간 더함)
std::optional<TransferPlan> expand_checked(
  const SolverResult & sol, const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, int relocs)
{
  const auto & steps = sol.steps;
  const bool y_box = sol.yellow_shape == "box";
  auto fits_place = [&](const CargoItem & it, int place) {
      const auto k = puck_kind(it.color, it.shape);
      if (!k) {return false;}
      const char g = in.place_group[place];
      if (g == 'R') {return kind_is_red(*k);}
      if (g == 'G') {return !kind_is_red(*k);}
      return kind_is_box(*k) == y_box;
    };
  std::vector<ArmAction> acts(steps.size());
  std::vector<std::vector<ArmAction>> pre(steps.size());   // 그 단계 앞에 하는 옮기기 (RETRIEVE → STORE)
  std::vector<bool> used(in.on_boxes.size(), false);
  std::unordered_set<std::string> dead;
  auto key_of = [&](std::size_t i, const Cargo & sim, int left) {
      std::string k = std::to_string(i) + "/" + std::to_string(left) + ":";
      for (const auto & s : sim.slots()) {
        k += s.second ? static_cast<char>('a' + static_cast<int>(*puck_kind(s.second->color, s.second->shape))) : '.';
      }
      k += sim.gripper() ? static_cast<char>('a' + static_cast<int>(*puck_kind(sim.gripper()->color, sim.gripper()->shape))) : '.';
      for (bool u : used) {k += u ? '1' : '0';}
      return k;
    };
  std::function<bool(std::size_t, Cargo &, int)> dfs = [&](std::size_t i, Cargo & sim, int left) -> bool {
      if (i == steps.size()) {return true;}
      const SolverStep & st = steps[i];
      if (st.act == SolverAct::Move) {return dfs(i + 1, sim, left);}
      const std::string key = key_of(i, sim, left);
      if (dead.count(key)) {return false;}
      switch (st.act) {
        case SolverAct::Pick: {
            std::size_t idx = in.on_boxes.size();
            for (std::size_t b = 0; b < in.on_boxes.size(); ++b) {
              if (!used[b] && in.on_boxes[b].box == st.node) {idx = b; break;}
            }
            if (idx == in.on_boxes.size() || in.on_boxes[idx].kind != st.kind) {break;}
            Cargo next = sim;
            const CargoItem it = item_of(in.on_boxes[idx]);
            if (!next.on_pick(it)) {break;}
            used[idx] = true;
            acts[i] = action_of(ArmOp::Pick, it);
            acts[i].box = st.node;
            if (dfs(i + 1, next, left)) {return true;}
            used[idx] = false;
            break;
          }
        case SolverAct::Store: {
            if (!sim.gripper()) {break;}
            std::vector<std::string> cand;
            if (const auto f = sim.free_slot_for(*sim.gripper()); f && sim.store_ok(*f)) {cand.push_back(*f);}
            for (const auto & s : sim.slots()) {
              if (!s.second && sim.store_ok(s.first) && (cand.empty() || cand.front() != s.first)) {cand.push_back(s.first);}
            }
            for (const auto & slot : cand) {
              Cargo next = sim;
              const CargoItem it = *next.gripper();
              if (!next.on_store(slot)) {continue;}
              acts[i] = action_of(ArmOp::Store, it);
              acts[i].slot = slot;
              if (dfs(i + 1, next, left)) {return true;}
            }
            break;
          }
        case SolverAct::Retrieve: {
            if (sim.gripper() || i + 1 >= steps.size() || steps[i + 1].act != SolverAct::Place) {break;}
            const int place = steps[i + 1].place;
            const auto & slots = sim.slots();
            for (auto s = slots.rbegin(); s != slots.rend(); ++s) {
              if (!s->second || !fits_place(*s->second, place) || !sim.retrieve_ok(s->first)) {continue;}
              Cargo next = sim;
              if (!next.on_retrieve(s->first)) {continue;}
              acts[i] = action_of(ArmOp::Retrieve, *next.gripper());
              acts[i].slot = s->first;
              if (dfs(i + 1, next, left)) {return true;}
            }
            // 막혀서 못 꺼냄 → 그 장소에 맞는 퍽이 든 칸을 막는 퍽을 빈 칸으로 옮기고 다시
            if (left > 0) {
              std::set<std::string> blockers;
              for (const auto & s : slots) {
                if (s.second && fits_place(*s.second, place) && !sim.retrieve_ok(s.first)) {
                  if (const auto b = sim.retrieve_blocker(s.first)) {blockers.insert(*b);}
                }
              }
              for (const auto & from : blockers) {
                if (!sim.retrieve_ok(from)) {continue;}
                Cargo mid = sim;
                if (!mid.on_retrieve(from)) {continue;}
                const CargoItem it = *mid.gripper();
                for (const auto & t : mid.slots()) {
                  if (t.second || t.first == from || !mid.store_ok(t.first)) {continue;}
                  Cargo next = mid;
                  if (!next.on_store(t.first)) {continue;}
                  ArmAction r = action_of(ArmOp::Retrieve, it);
                  r.slot = from;
                  ArmAction w = action_of(ArmOp::Store, it);
                  w.slot = t.first;
                  pre[i].push_back(r);
                  pre[i].push_back(w);
                  if (dfs(i, next, left - 1)) {return true;}
                  pre[i].pop_back();
                  pre[i].pop_back();
                }
              }
            }
            break;
          }
        case SolverAct::Place: {
            if (!sim.gripper() || !fits_place(*sim.gripper(), st.place)) {break;}
            Cargo next = sim;
            const auto it = next.on_place();
            if (!it) {break;}
            acts[i] = action_of(ArmOp::Place, *it);
            acts[i].place = st.place;
            if (dfs(i + 1, next, left)) {return true;}
            break;
          }
        default:
          break;
      }
      dead.insert(key);
      return false;
    };
  Cargo sim = cargo;
  if (!dfs(0, sim, relocs)) {return std::nullopt;}

  TransferPlan plan;
  plan.cost_s = sol.cost_s;
  plan.yellow_shape = sol.yellow_shape;
  plan.verify_order = sol.verify_order;
  plan.expanded = sol.expanded;
  std::array<int, kNumPlaces> next_cell = cells_done;
  Stop cur;
  cur.node = in.position;
  cur.move = false;
  cur.name = node_name(in.position) + " (이어서)";
  bool cur_open = in.stopped;
  auto close = [&]() {
      if ((cur_open || !cur.actions.empty()) && (!cur.actions.empty() || cur.move)) {plan.stops.push_back(cur);}
      cur_open = false;
    };
  for (std::size_t i = 0; i < steps.size(); ++i) {
    const SolverStep & st = steps[i];
    if (st.act == SolverAct::Move) {
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
      continue;
    }
    for (const auto & r : pre[i]) {
      cur.actions.push_back(r);
      plan.cost_s += r.op == ArmOp::Retrieve ? costs.retrieve_s : costs.store_s;
    }
    ArmAction a = acts[i];
    if (a.op == ArmOp::Place) {
      a.cell = next_cell[a.place]++;
      plan.place_of[a.puck_id] = a.place;
    }
    cur.actions.push_back(a);
  }
  close();
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
  // 칸 제약을 지키는 펼치기
  //   옮기기 0 번: 가장 빠른 해가 되면 그것. 아니면 다른 해 (장소 방문 순서 6 × Y 모양) 중 되는 것 가장 빠른 것
  //   그래도 없으면 옮기기 한도 1 → 2 번으로 같은 순서 (가장 빠른 해 포함 시간 비교)
  static const int kPerm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  const std::vector<std::string> shapes = in.yellow_shape.empty() ?
    std::vector<std::string>{"cylinder", "box"} : std::vector<std::string>{in.yellow_shape};
  std::vector<std::pair<SolverInput, SolverResult>> alts;   // 다른 해는 한 번만 풂
  bool alts_done = false;
  for (int relocs = 0; relocs <= 2; ++relocs) {
    std::optional<TransferPlan> best = expand_checked(*sol, in, costs, cargo, cells_done, relocs);
    if (best && relocs == 0) {return best;}
    if (!alts_done) {
      for (const auto & shape : shapes) {
        for (const auto & perm : kPerm) {
          SolverInput v = in;
          v.yellow_shape = shape;
          for (int r = 0; r < kNumPlaces; ++r) {v.place_rank[perm[r]] = r;}
          if (auto s2 = solve_transfer(v, costs)) {alts.emplace_back(v, *s2);}
        }
      }
      alts_done = true;
    }
    for (const auto & [v, s2] : alts) {
      auto p2 = expand_checked(s2, v, costs, cargo, cells_done, relocs);
      if (p2 && (!best || p2->cost_s < best->cost_s)) {best = std::move(p2);}
    }
    if (best) {return best;}
  }
  // 칸 제약을 다 지키는 배정이 없음 → 옛 펼치기 (막는 퍽 옮기기, 그래도 안 되면 막지 않음)
  return expand_greedy(*sol, in, costs, cargo, cells_done, error);
}

namespace
{

std::optional<TransferPlan> expand_greedy(
  const SolverResult & sol, const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error)
{
  TransferPlan plan;
  plan.cost_s = sol.cost_s;
  plan.yellow_shape = sol.yellow_shape;
  plan.verify_order = sol.verify_order;
  plan.expanded = sol.expanded;

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
      // 첫 정차: 서 있지 않아도 (재계획) 이동 전에 할 동작 (쥔 퍽 넣기) 이 있으면 지금 자리에서 함
      if ((cur_open || !cur.actions.empty()) && (!cur.actions.empty() || cur.move)) {plan.stops.push_back(cur);}
      cur_open = false;
    };

  // 꺼낼 칸: 그 종류가 든 칸 중 뒤에서부터 (나중에 넣은 칸 먼저), 칸 제약 (S7 ← S12 등) 을 지키는 칸 먼저. ok = 지키는 칸
  auto retrieve_slot = [&](PuckKind kind, bool & ok) {
      std::optional<std::string> any;
      const auto & slots = sim.slots();
      for (auto s = slots.rbegin(); s != slots.rend(); ++s) {
        if (!s->second || puck_kind(s->second->color, s->second->shape) != kind) {continue;}
        if (!any) {any = s->first;}
        if (sim.retrieve_ok(s->first)) {ok = true; return std::optional<std::string>(s->first);}
      }
      ok = false;
      return any;
    };
  std::vector<SolverStep> steps = sol.steps;
  for (std::size_t si = 0; si < steps.size(); ++si) {
    // 같은 장소에서 꺼내기 · 놓기가 이어지면 (한 장소 = 한 색 묶음, 순서 무관) 칸 제약을 지키는 종류부터 (S1 ← S2 · S10 ← S5)
    if (steps[si].act == SolverAct::Retrieve && si + 1 < steps.size() && steps[si + 1].act == SolverAct::Place) {
      bool ok = false;
      retrieve_slot(steps[si].kind, ok);
      for (std::size_t sj = si + 2; !ok && sj + 1 < steps.size() && steps[sj].act == SolverAct::Retrieve && steps[sj].node == steps[si].node &&
        steps[sj + 1].act == SolverAct::Place && steps[sj + 1].place == steps[si + 1].place; sj += 2)
      {
        bool ok2 = false;
        retrieve_slot(steps[sj].kind, ok2);
        if (ok2) {
          std::swap(steps[si], steps[sj]);
          std::swap(steps[si + 1], steps[sj + 1]);
          ok = true;
        }
      }
    }
    const SolverStep & st = steps[si];
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
          if (!sim.gripper()) {set_error(error, "펼치기 오류: 넣을 칸 · 퍽 없음"); return std::nullopt;}
          const auto slot = sim.free_slot_for(*sim.gripper());
          if (!slot) {set_error(error, "펼치기 오류: 넣을 칸 · 퍽 없음"); return std::nullopt;}
          ArmAction a = action_of(ArmOp::Store, *sim.gripper());
          a.slot = *slot;
          if (!sim.on_store(*slot, error)) {return std::nullopt;}
          cur.actions.push_back(a);
          break;
        }
      case SolverAct::Retrieve: {
          // 지키는 칸이 없으면 뒤에서 첫 칸 (막지 않음)
          bool ok = false;
          const auto slot = retrieve_slot(st.kind, ok);
          if (!slot) {set_error(error, "펼치기 오류: 꺼낼 종류가 칸에 없음"); return std::nullopt;}
          // 그래도 제약에 걸리면 (막는 칸 퍽이 다른 장소로 감) 막는 퍽을 빈 칸으로 옮긴 뒤 뺌: RETRIEVE 막는 칸 → STORE 빈 칸
          //   (RETRIEVE 끝 = STORE 시작 = 박스 위 기준점). 빈 칸이 없으면 그대로 (막지 않음).
          for (int guard = 0; !ok && guard < 3; ++guard) {
            const auto blk = sim.retrieve_blocker(*slot);
            if (!blk) {break;}
            const auto& slots = sim.slots();
            std::optional<CargoItem> moved;
            for (const auto & s : slots) {
              if (s.first == *blk && s.second) {moved = s.second;}
            }
            if (!moved || !sim.retrieve_ok(*blk)) {break;}
            Cargo trial = sim;
            if (!trial.on_retrieve(*blk)) {break;}
            const auto to = trial.free_slot_for(*moved);
            if (!to || *to == *blk || !trial.store_ok(*to)) {break;}
            if (!sim.on_retrieve(*blk, error)) {return std::nullopt;}
            ArmAction r = action_of(ArmOp::Retrieve, *moved);
            r.slot = *blk;
            cur.actions.push_back(r);
            if (!sim.on_store(*to, error)) {return std::nullopt;}
            ArmAction w = action_of(ArmOp::Store, *moved);
            w.slot = *to;
            cur.actions.push_back(w);
            plan.cost_s += costs.retrieve_s + costs.store_s;
            ok = sim.retrieve_ok(*slot);
          }
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

}  // namespace

std::vector<ArmAction> collect_box_actions(
  const std::vector<TransferPuck> & at_box, const Cargo & cargo, std::string * error)
{
  std::vector<ArmAction> out;
  Cargo sim = cargo;
  if (sim.gripper()) {   // 쥔 퍽부터 넣음 (박스에 남은 퍽이 없어도: 넣기 실패 뒤 다시 계획할 때)
    const auto slot = sim.free_slot_for(*sim.gripper());
    if (!slot) {set_error(error, "적재함 빈 칸 없음"); return {};}
    ArmAction s = action_of(ArmOp::Store, *sim.gripper());
    s.slot = *slot;
    if (!sim.on_store(*slot, error)) {return {};}
    out.push_back(s);
  }
  for (const auto & p : at_box) {
    const CargoItem it = item_of(p);
    if (!sim.on_pick(it, error)) {return {};}
    ArmAction a = action_of(ArmOp::Pick, it);
    a.box = p.box;
    out.push_back(a);
    // 바로 넣음 (퍽 쥔 채 주행 안 함 → 박스를 떠날 때 그리퍼가 비어 있어야 함)
    const auto slot = sim.free_slot_for(it);
    if (!slot) {set_error(error, "적재함 빈 칸 없음"); return {};}
    ArmAction s = action_of(ArmOp::Store, it);
    s.slot = *slot;
    if (!sim.on_store(*slot, error)) {return {};}
    out.push_back(s);
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
