// transfer_solver.cpp — 퍽 운반 정확해 구현, 설명은 transfer_solver.hpp

#include "mission/transfer_solver.hpp"
#include "mission/common.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace mission
{

namespace
{

using Key = uint64_t;

// 상태 키 비트 배치
//   [0,12)  박스에 남은 퍽 (on_boxes 번호)
//   [12,16) 위치 노드
//   16      정차 중
//   [17,20) 그리퍼 (0 = 빔, 종류 + 1)
//   [20,36) 적재함 종류별 개수 (4 bit × 4)
//   [36,45) 장소별 남은 칸 (3 bit × 3)
//   63      끝 표시
struct State
{
  uint32_t rem = 0;
  int pos = 0;
  bool stopped = false;
  int grip = 0;
  std::array<int, kNumKinds> cargo{{0, 0, 0, 0}};
  std::array<int, kNumPlaces> slots{{0, 0, 0}};

  Key key() const
  {
    Key k = rem;
    k |= static_cast<Key>(pos) << 12;
    k |= static_cast<Key>(stopped ? 1 : 0) << 16;
    k |= static_cast<Key>(grip) << 17;
    for (int i = 0; i < kNumKinds; ++i) {k |= static_cast<Key>(cargo[i]) << (20 + 4 * i);}
    for (int i = 0; i < kNumPlaces; ++i) {k |= static_cast<Key>(slots[i]) << (36 + 3 * i);}
    return k;
  }

  static State from(Key k)
  {
    State s;
    s.rem = static_cast<uint32_t>(k & 0xFFF);
    s.pos = static_cast<int>((k >> 12) & 0xF);
    s.stopped = ((k >> 16) & 1) != 0;
    s.grip = static_cast<int>((k >> 17) & 0x7);
    for (int i = 0; i < kNumKinds; ++i) {s.cargo[i] = static_cast<int>((k >> (20 + 4 * i)) & 0xF);}
    for (int i = 0; i < kNumPlaces; ++i) {s.slots[i] = static_cast<int>((k >> (36 + 3 * i)) & 0x7);}
    return s;
  }

  int cargo_count() const {return cargo[0] + cargo[1] + cargo[2] + cargo[3];}
};

constexpr Key kDone = Key{1} << 63;

struct Rec
{
  double d = 0.0;
  Key parent = 0;
  bool has_parent = false;
  SolverStep step;
};

// 퍽 종류 k 를 그룹 g 장소에 놓을 수 있나 (Y 는 모양 s)
bool fits(PuckKind k, char g, bool s_box)
{
  if (g == 'R') {return kind_is_red(k);}
  if (g == 'G') {return !kind_is_red(k);}
  if (g == 'Y') {return kind_is_box(k) == s_box;}
  return false;
}

// 끝 비용: (재검증 순회) + 출발점 복귀. order 에 재검증 순서
double finish_cost(int pos, const TransferCosts & c, std::array<int, kNumPlaces> & order)
{
  if (!c.verify_tour) {return c.travel_s[pos][kNodeStart];}
  std::array<int, kNumPlaces> p{{0, 1, 2}};
  double best = std::numeric_limits<double>::infinity();
  do {
    double t = c.travel_s[pos][place_node(p[0])];
    for (int i = 0; i + 1 < kNumPlaces; ++i) {t += c.travel_s[place_node(p[i])][place_node(p[i + 1])];}
    t += c.travel_s[place_node(p[kNumPlaces - 1])][kNodeStart];
    t += kNumPlaces * (c.stop_s + c.look_s);
    if (t < best) {best = t; order = p;}
  } while (std::next_permutation(p.begin(), p.end()));
  return best;
}

// 모양 s 로 할당이 가능한가 (개수만 봄): Y 에 넣을 빨강 수 yR, 초록 수 yG 가 맞아떨어지는지
bool counts_feasible(
  const std::array<int, kNumKinds> & n, const std::array<char, kNumPlaces> & g,
  const std::array<int, kNumPlaces> & slots, bool s_box, bool leftover = false)
{
  int need_r = 0, need_g = 0, need_y = 0;
  for (int i = 0; i < kNumPlaces; ++i) {
    if (g[i] == 'R') {need_r += slots[i];}
    if (g[i] == 'G') {need_g += slots[i];}
    if (g[i] == 'Y') {need_y += slots[i];}
  }
  const int red = n[0] + n[1], green = n[2] + n[3];
  if (leftover) {
    // 남아도 됨: R 은 모양 s 아닌 빨강부터 써서 Y 에 모양 s 빨강을 최대한 남김 (초록도 같음)
    if (red < need_r || green < need_g) {return false;}
    const int red_s = s_box ? n[1] : n[0], green_s = s_box ? n[3] : n[2];
    const int y_red_max = std::min(red - need_r, red_s - std::max(0, need_r - (red - red_s)));
    const int y_green_max = std::min(green - need_g, green_s - std::max(0, need_g - (green - green_s)));
    return y_red_max + y_green_max >= need_y;
  }
  const int y_red = red - need_r, y_green = green - need_g;   // Y 로 가야 하는 빨강 · 초록 수
  if (y_red < 0 || y_green < 0 || y_red + y_green != need_y) {return false;}
  const int red_s = s_box ? n[1] : n[0];
  const int green_s = s_box ? n[3] : n[2];
  return red_s >= y_red && green_s >= y_green;
}

}  // namespace

std::optional<SolverResult> solve_transfer(
  const SolverInput & in, const TransferCosts & c, std::string * error, std::size_t max_states)
{
  // ---- 입력 검사
  if (in.on_boxes.size() > 12) {set_error(error, "박스 위 퍽이 12개보다 많음"); return std::nullopt;}
  if (in.position < 0 || in.position >= kNumNodes) {set_error(error, "위치 노드 번호 오류"); return std::nullopt;}
  if (in.cargo_capacity < 0 || in.cargo_capacity > 15) {set_error(error, "적재함 칸 수는 0–15"); return std::nullopt;}
  if (static_cast<int>(in.in_cargo.size()) > in.cargo_capacity) {set_error(error, "적재함 칸 수보다 많이 실림"); return std::nullopt;}
  int slot_sum = 0;
  for (int i = 0; i < kNumPlaces; ++i) {
    const char g = in.place_group[i];
    if (g != 'R' && g != 'Y' && g != 'G') {set_error(error, "장소 그룹은 R · Y · G"); return std::nullopt;}
    if (in.slots_left[i] < 0 || in.slots_left[i] > 6) {set_error(error, "장소 남은 칸은 0–6"); return std::nullopt;}
    slot_sum += in.slots_left[i];
  }
  std::array<int, kNumKinds> n{{0, 0, 0, 0}};
  for (const auto & p : in.on_boxes) {
    if (p.box < 1 || p.box > kNumBoxes) {set_error(error, "박스 번호 오류: 퍽 " + p.id); return std::nullopt;}
    ++n[static_cast<int>(p.kind)];
  }
  for (const auto & p : in.in_cargo) {++n[static_cast<int>(p.kind)];}
  if (in.gripper) {++n[static_cast<int>(in.gripper->kind)];}
  const int puck_sum = n[0] + n[1] + n[2] + n[3];
  if (in.allow_leftover ? puck_sum < slot_sum : puck_sum != slot_sum) {
    set_error(error, "남은 퍽 " + std::to_string(puck_sum) + "개 ≠ 남은 칸 " + std::to_string(slot_sum) + "개");
    return std::nullopt;
  }

  // ---- Y 모양 후보
  std::vector<bool> shapes;   // true = box
  if (!in.yellow_shape.empty()) {
    if (in.yellow_shape != "box" && in.yellow_shape != "cylinder") {set_error(error, "yellow_shape 오류"); return std::nullopt;}
    shapes.push_back(in.yellow_shape == "box");
  } else {
    shapes = {false, true};
  }

  State s0;
  s0.rem = static_cast<uint32_t>((1u << in.on_boxes.size()) - 1u);
  s0.pos = in.position;
  s0.stopped = in.stopped;
  s0.grip = in.gripper ? static_cast<int>(in.gripper->kind) + 1 : 0;
  for (const auto & p : in.in_cargo) {++s0.cargo[static_cast<int>(p.kind)];}
  s0.slots = in.slots_left;

  std::optional<SolverResult> best;
  std::size_t expanded_total = 0;
  bool hit_limit = false;

  for (const bool s_box : shapes) {
    if (!counts_feasible(n, in.place_group, in.slots_left, s_box, in.allow_leftover)) {continue;}

    std::unordered_map<Key, Rec> rec;
    using QE = std::pair<double, Key>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    const Key k0 = s0.key();
    rec[k0] = Rec{0.0, 0, false, {}};
    pq.push({0.0, k0});
    std::optional<Key> done_key;
    std::size_t expanded = 0;

    while (!pq.empty()) {
      const auto [d, k] = pq.top();
      pq.pop();
      if (d > rec[k].d + 1e-9) {continue;}
      if (k & kDone) {done_key = k; break;}
      if (++expanded > max_states) {hit_limit = true; break;}
      const State st = State::from(k);

      auto relax = [&](const State & ns, double cost, const SolverStep & step, Key nk_override = 0) {
          const Key nk = nk_override ? nk_override : ns.key();
          const double nd = d + cost;
          auto it = rec.find(nk);
          if (it == rec.end() || nd < it->second.d - 1e-9) {
            rec[nk] = Rec{nd, k, true, step};
            pq.push({nd, nk});
          }
        };

      // 끝
      if ((in.allow_leftover || (st.rem == 0 && st.grip == 0 && st.cargo_count() == 0)) &&
        st.slots[0] == 0 && st.slots[1] == 0 && st.slots[2] == 0)
      {
        std::array<int, kNumPlaces> order{{0, 1, 2}};
        const double f = finish_cost(st.pos, c, order);
        relax(st, f, SolverStep{SolverAct::Move, kNodeStart, PuckKind::RedCylinder, -1}, kDone | k);
        continue;
      }

      // 팔 작업 (정차 중일 때만)
      if (st.stopped) {
        // Pick: 이 박스에 남은 퍽 중 on_boxes 순서 첫 번째만 = 정차 자리에 가장 가까운 퍽 (먼 퍽부터 집으면 앞 퍽을 칠 수 있음)
        if (is_box_node(st.pos) && st.grip == 0) {
          for (std::size_t i = 0; i < in.on_boxes.size(); ++i) {
            if (!(st.rem >> i & 1u) || in.on_boxes[i].box != st.pos) {continue;}
            State ns = st;
            ns.rem &= ~(1u << i);
            ns.grip = static_cast<int>(in.on_boxes[i].kind) + 1;
            relax(ns, c.pick_s, SolverStep{SolverAct::Pick, st.pos, in.on_boxes[i].kind, -1});
            break;
          }
        }
        // Place
        if (is_place_node(st.pos) && st.grip) {
          const int p = node_place(st.pos);
          const PuckKind kk = static_cast<PuckKind>(st.grip - 1);
          if (st.slots[p] > 0 && fits(kk, in.place_group[p], s_box)) {
            State ns = st;
            ns.grip = 0;
            --ns.slots[p];
            relax(ns, c.place_s, SolverStep{SolverAct::Place, st.pos, kk, p});
          }
        }
      }
      // Store: 어디서든 (계획 중 로봇은 서 있음. 출발 · 재계획 때 쥔 퍽이 있으면 먼저 넣어야 이동 가능)
      if (st.grip && st.cargo_count() < in.cargo_capacity) {
        State ns = st;
        ++ns.cargo[st.grip - 1];
        ns.grip = 0;
        relax(ns, c.store_s, SolverStep{SolverAct::Store, st.pos, static_cast<PuckKind>(st.grip - 1), -1});
      }
      // Retrieve (정차 중일 때만)
      if (st.stopped) {
        // 꺼내기 = 이 장소에 바로 놓을 퍽만 (다음 장소 퍽을 미리 꺼내 쥐고 주행하지 않게. 시간은 같고 떨어뜨릴 위험만 큼)
        if (st.grip == 0 && is_place_node(st.pos) && st.slots[node_place(st.pos)] > 0) {
          const char g = in.place_group[node_place(st.pos)];
          for (int kk = 0; kk < kNumKinds; ++kk) {
            if (!st.cargo[kk] || !fits(static_cast<PuckKind>(kk), g, s_box)) {continue;}
            State ns = st;
            --ns.cargo[kk];
            ns.grip = kk + 1;
            relax(ns, c.retrieve_s, SolverStep{SolverAct::Retrieve, st.pos, static_cast<PuckKind>(kk), -1});
          }
        }
      }

      // Move: 그리퍼가 빌 때만 (퍽 쥔 채 주행 안 함 — control 도 쥔 채 DRIVE · HOME 을 안 함)
      if (st.grip) {continue;}
      const int carried = st.cargo_count();
      const bool room = st.cargo_count() < in.cargo_capacity;   // 집은 퍽을 넣을 칸
      for (int v = 1; v < kNumNodes; ++v) {
        if (v == st.pos && st.stopped) {continue;}
        if (is_box_node(v)) {
          bool has = false;
          for (std::size_t i = 0; i < in.on_boxes.size(); ++i) {
            if ((st.rem >> i & 1u) && in.on_boxes[i].box == v) {has = true; break;}
          }
          if (!has || !room) {continue;}
        } else {
          const int p = node_place(v);
          if (carried == 0 || st.slots[p] == 0) {continue;}
          bool before_open = false;   // 순서 강제: 앞 순번 장소가 아직 안 끝났으면 못 감
          for (int q = 0; q < kNumPlaces && in.place_rank[p] >= 0; ++q) {
            if (in.place_rank[q] >= 0 && in.place_rank[q] < in.place_rank[p] && st.slots[q] > 0) {before_open = true;}
          }
          if (before_open) {continue;}
          bool any = false;
          for (int kk = 0; kk < kNumKinds && !any; ++kk) {
            any = st.cargo[kk] > 0 && fits(static_cast<PuckKind>(kk), in.place_group[p], s_box);
          }
          if (!any) {continue;}
        }
        State ns = st;
        ns.pos = v;
        ns.stopped = true;
        relax(ns, c.travel_s[st.pos][v] + c.stop_s + c.look_s, SolverStep{SolverAct::Move, v, PuckKind::RedCylinder, -1});
      }
    }
    expanded_total += expanded;
    if (!done_key) {continue;}

    // 경로 복원 (끝 표시 단계는 뺌)
    SolverResult r;
    r.cost_s = rec[*done_key].d;
    r.yellow_shape = s_box ? "box" : "cylinder";
    Key k = rec[*done_key].parent;
    finish_cost(State::from(k).pos, c, r.verify_order);   // 끝 직전 위치 기준 재검증 순서
    while (true) {
      const Rec & rr = rec[k];
      if (!rr.has_parent) {break;}
      r.steps.push_back(rr.step);
      k = rr.parent;
    }
    std::reverse(r.steps.begin(), r.steps.end());
    if (!best || r.cost_s < best->cost_s) {best = std::move(r);}
  }

  if (!best) {
    set_error(error, hit_limit ? "상태 수 한도 초과 (탐욕 해로 대신할 것)" : "할당 불가 또는 해 없음");
    return std::nullopt;
  }
  best->expanded = expanded_total;
  return best;
}

}  // namespace mission
