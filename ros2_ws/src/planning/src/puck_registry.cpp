// puck_registry.cpp — 퍽 기록부 구현, 설명은 puck_registry.hpp

#include "planning/puck_registry.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{

namespace
{
// 표 많은 값. 같으면 지금 값 유지 (지금 값이 공동 1 등일 때), 표가 없으면 지금 값
std::string top_vote(const std::map<std::string, int> & v, const std::string & now)
{
  int best = 0;
  std::string out = now;
  for (const auto & [k, n] : v) {
    if (n > best || (n == best && k == now)) {best = n; out = k;}
  }
  return out;
}
}  // namespace

void PuckRegistry::update(const PuckRecord & seen, bool vote_kind)
{
  if (seen.id.empty()) {
    add_candidate(seen);
    return;
  }
  auto it = by_id_.find(seen.id);
  if (it == by_id_.end()) {it = by_id_.emplace(seen.id, seen).first;}
  PuckRecord & r = it->second;
  r.x = seen.x;
  r.y = seen.y;
  r.z = seen.z;
  r.last_seen = seen.last_seen;
  auto & v = votes_[seen.id];
  if (vote_kind && r.state == "on_field") {
    if (!seen.color.empty()) {++v.color[seen.color];}
    if (!seen.shape.empty()) {++v.shape[seen.shape];}
  }
  r.color = top_vote(v.color, r.color.empty() ? seen.color : r.color);
  r.shape = top_vote(v.shape, r.shape.empty() ? seen.shape : r.shape);
  // 같은 자리에 있던 후보는 이 퍽이었던 것 → 삭제
  candidates_.erase(
    std::remove_if(candidates_.begin(), candidates_.end(), [&](const PuckRecord & c) {
      return std::hypot(c.x - seen.x, c.y - seen.y) < merge_radius_;
    }),
    candidates_.end());
}

void PuckRegistry::add_candidate(const PuckRecord & seen)
{
  // ID 퍽 근처면 그 퍽을 다시 본 것 → 위치·시각만 갱신
  for (auto & kv : by_id_) {
    if (std::hypot(kv.second.x - seen.x, kv.second.y - seen.y) < merge_radius_) {
      kv.second.x = seen.x;
      kv.second.y = seen.y;
      kv.second.z = seen.z;
      kv.second.last_seen = seen.last_seen;
      return;
    }
  }
  for (auto & c : candidates_) {
    if (std::hypot(c.x - seen.x, c.y - seen.y) < merge_radius_) {
      c = seen;
      c.id.clear();
      return;
    }
  }
  PuckRecord c = seen;
  c.id.clear();
  candidates_.push_back(c);
}

std::optional<PuckRecord> PuckRegistry::find(const std::string & id) const
{
  const auto it = by_id_.find(id);
  if (it == by_id_.end()) {return std::nullopt;}
  return it->second;
}

void PuckRegistry::set_state(const std::string & id, const std::string & state)
{
  auto it = by_id_.find(id);
  if (it != by_id_.end()) {it->second.state = state;}
}

void PuckRegistry::prune_candidates(double now, double max_age_s)
{
  candidates_.erase(
    std::remove_if(candidates_.begin(), candidates_.end(), [&](const PuckRecord & c) {
      return now - c.last_seen > max_age_s;
    }),
    candidates_.end());
}

std::vector<PuckRecord> PuckRegistry::records() const
{
  std::vector<PuckRecord> out;
  out.reserve(by_id_.size());
  for (const auto & kv : by_id_) {out.push_back(kv.second);}
  std::sort(out.begin(), out.end(), [](const PuckRecord & a, const PuckRecord & b) {
    return a.id < b.id;
  });
  return out;
}

}  // namespace planning
