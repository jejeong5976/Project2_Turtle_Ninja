// puck_registry.hpp — 퍽 기록부
//
// 역할
//   - 돌아다니며 본 퍽을 ID 기준으로 지도 위치에 기록
//   - ID 를 못 읽은 퍽은 후보로 따로 둠, 멀리서 색만 본 것
//   - 퍽 상태 관리
//       on_field       = 현장에 있음, 기본
//       in_gripper     = 집은 뒤
//       in_cargo:<칸>  = 적재함에 넣은 뒤
//       delivered      = 놓기 성공 뒤
// 입력
//   - map 프레임으로 바꾼 검출: ID · 색 · 모양 · 위치
//   - 팔 작업 성공 결과
// 출력
//   - ID 별 위치 · 색 · 모양 · 상태, 후보 목록
// 실행
//   - planning_core 라이브러리, Executor 가 씀. 시험 test_planning_core
// note
//   - 좌표 변환은 ROS 노드에서 끝낸 뒤 넣음
//   - 상태는 팔 작업 성공 결과로만 바꿈
//   - 색 · 모양 = ID 별 다수결 (한 장 오검출이 적재 기록을 덮어쓰지 않게).
//       표는 현장 (on_field) 에 있을 때 · vote_kind 일 때만 셈 → 손 · 칸 · 놓은 뒤 검출은 위치만 바꿈

#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning
{

struct PuckRecord
{
  std::string id;               // AprilTag ID ("" = 후보)
  std::string color;            // "red" | "green" | "beige"
  std::string shape;            // "cylinder" | "box" | ""
  double x = 0.0;               // map [m]
  double y = 0.0;
  double z = 0.0;               // 퍽 중심 높이 [m] (바닥이면 약 0.03)
  double last_seen = 0.0;       // 마지막으로 본 시각 [s]
  std::string state = "on_field";
};

class PuckRegistry
{
public:
  // merge_radius: 후보끼리 같은 퍽으로 볼 거리 [m]
  explicit PuckRegistry(double merge_radius = 0.06)
  : merge_radius_(merge_radius) {}

  // ID 를 읽은 검출 반영. 위치 · 시각은 최신 값, 색 · 모양은 표 많은 쪽, 상태는 유지
  //   vote_kind false 이거나 상태가 on_field 가 아니면 색 · 모양 표를 안 셈 (아직 표가 없을 때 빈 값만 채움)
  void update(const PuckRecord & seen, bool vote_kind = true);

  // ID 못 읽은 검출 반영 (가까운 후보와 합침)
  void add_candidate(const PuckRecord & seen);

  std::optional<PuckRecord> find(const std::string & id) const;
  void set_state(const std::string & id, const std::string & state);

  // 오래된 후보 정리 (max_age_s 보다 오래 안 본 후보 삭제)
  void prune_candidates(double now, double max_age_s);

  std::vector<PuckRecord> records() const;
  const std::vector<PuckRecord> & candidates() const {return candidates_;}

private:
  double merge_radius_;
  std::unordered_map<std::string, PuckRecord> by_id_;
  struct KindVotes {std::map<std::string, int> color, shape;};
  std::unordered_map<std::string, KindVotes> votes_;   // ID → 색 · 모양별 본 장 수
  std::vector<PuckRecord> candidates_;
};

}  // namespace planning
