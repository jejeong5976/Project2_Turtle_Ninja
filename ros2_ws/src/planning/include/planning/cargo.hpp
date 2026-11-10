// cargo.hpp — 적재함 · 그리퍼 재고 기록
//
// 역할
//   - 어느 칸에 어떤 퍽이 있는지, 그리퍼에 무엇을 쥐고 있는지 기록
//   - 팔 작업 성공 결과로만 바꿈, 명령을 보낸 시점에는 안 바꿈
//       PICK 성공     = 그리퍼 ← 퍽
//       STORE 성공    = 칸 ← 그리퍼
//       RETRIEVE 성공 = 그리퍼 ← 칸
//       PLACE 성공    = 그리퍼 비움, 놓은 퍽을 돌려줌
//   - 재고 불확실 표시 doubt: 팔이 움직였는데 결과를 확정 못 한 단계 하나 (원인 단계 · 그 퍽)
//       표시가 있는 동안 기록 = 마지막으로 확인된 재고, 사람이 실물 확인 뒤 inventory_policy resolve_doubt 로만 지움
//   - yaml 저장 · 읽기, 노드 재시작 대비 (불확실 표시 포함)
// 입력
//   - 칸 이름 목록, 팔 작업 성공 결과, 불확실 표시
// 출력
//   - 칸 · 그리퍼 내용, 전환 성공 여부, 불확실 표시
// 실행
//   - planning_core 라이브러리, Executor 가 씀. 시험 test_planning_core
// note
//   - 기록과 맞지 않는 전환은 false 로 거부, 예: 빈 그리퍼로 STORE → 실행기가 불확실 표시 + FAULT (inventory_policy apply_arm_success)
//   - 칸 이름은 control 과 같은 이름을 씀, 팔이 닿는 칸만 등록. 기본 S1–S12 = URDF rack_cell_S*, 팔에서 먼 칸부터
//   - 빈 칸은 목록 순서대로 채움 (free_slot) → 목록 = cargo_slots (뒷줄부터)
//   - 꺼낼 칸은 운반 계획 (transfer_planner) 이 정함: 목록 뒤 = 나중에 넣은 칸부터
//   - 칸 제약 (set_rules): 넣기 · 빼기 전에 비어 있어야 하는 칸 (config/rack_cells.yaml rack_constraints, 팔 경로가 옆 칸 퍽에 닿음)
//       free_slot 은 제약을 지키는 첫 빈 칸, 그런 칸이 없으면 첫 빈 칸 (막지 않음). 빼기는 retrieve_ok 로 고름

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace planning
{

struct CargoItem
{
  std::string puck_id;          // AprilTag ID
  std::string color;
  std::string shape;
  std::string task_id;          // 이 퍽을 실은 task
  bool verified = false;        // 카메라로 확인했는가 (지금은 기록만 → false)
};

// 재고 불확실 원인 (팔 작업 하나)
struct CargoDoubt
{
  uint8_t arm_task = 0;         // ArmTask 번호 (PICK · STORE · RETRIEVE · PLACE · LOOK_RACK)
  std::string puck_id;          // 단계의 대상 퍽
  std::string slot;             // STORE · RETRIEVE · LOOK_RACK 칸
  std::string step_name;        // 원인 단계 이름
  std::string reason;           // 실패 · 취소 · 시간 초과 이유
  CargoItem item;               // 그 퍽 정보 (PICK = 집으려던 퍽, 그 밖 = 그때 기록상 그리퍼 · 칸 내용)
};

class Cargo
{
public:
  explicit Cargo(std::vector<std::string> slot_names = {});

  // ---------------------------------------------------------------- 조회
  bool holding() const {return gripper_.has_value();}
  const std::optional<CargoItem> & gripper() const {return gripper_;}
  std::optional<std::string> free_slot() const;                      // 첫 빈 칸 (칸 제약을 지키는 칸 먼저)
  // 이 퍽을 넣을 칸: free_slot 과 같되, 빼기 제약 짝 칸 (S7 · S12 등) 의 짝에 다른 색 퍽이 있으면 그 칸은 미룸
  //   (같은 색 = 보통 같은 장소 → 배달 때 같은 장소 안에서 순서만 바꿔 제약을 지킴). 다른 칸이 없으면 그 칸
  std::optional<std::string> free_slot_for(const CargoItem & item) const;
  std::size_t stored_count() const;                                  // 칸에 실린 수 (그리퍼 제외)
  std::size_t capacity() const {return slots_.size();}
  const std::vector<std::pair<std::string, std::optional<CargoItem>>> & slots() const {return slots_;}

  // ---------------------------------------------------------------- 칸 제약 (칸, 그 동작 전에 비어 있어야 할 칸)
  using SlotRules = std::vector<std::pair<std::string, std::string>>;
  void set_rules(SlotRules store_needs_empty, SlotRules retrieve_needs_empty);
  bool store_ok(const std::string & slot) const {return rule_ok(store_rules_, slot);}
  bool retrieve_ok(const std::string & slot) const {return rule_ok(retrieve_rules_, slot);}
  // slot 을 빼기 전에 비어 있어야 하는데 찬 칸 (첫 하나), 없으면 nullopt
  std::optional<std::string> retrieve_blocker(const std::string & slot) const;

  // ---------------------------------------------------------------- 전환 (팔 작업 성공 시에만 호출)
  bool on_pick(const CargoItem & item, std::string * error = nullptr);
  bool on_store(const std::string & slot, std::string * error = nullptr);
  bool on_retrieve(const std::string & slot, std::string * error = nullptr);
  std::optional<CargoItem> on_place(std::string * error = nullptr);

  // 그리퍼 기록 지움 (사람이 퍽을 빼고 확인한 뒤). 불확실 표시는 안 건드림
  void clear_gripper() {gripper_.reset();}
  // 칸 기록 지움 (불확실 정합 removed 용). 없는 칸이면 false
  bool clear_slot(const std::string & slot);

  // ---------------------------------------------------------------- 불확실 표시 (원인 하나만, 첫 원인 유지)
  const std::optional<CargoDoubt> & doubt() const {return doubt_;}
  bool set_doubt(const CargoDoubt & d);                              // 이미 있으면 false (그대로 둠)
  void clear_doubt() {doubt_.reset();}

  // ---------------------------------------------------------------- 파일
  bool save(const std::string & path, std::string * error = nullptr) const;
  // 파일의 칸 이름이 지금 설정과 다르면 실패 (잘못된 기록을 쓰지 않게)
  bool load(const std::string & path, std::string * error = nullptr);

private:
  std::optional<CargoItem> * find_slot(const std::string & slot);
  bool rule_ok(const SlotRules & rules, const std::string & slot) const;   // slot 의 제약 칸이 다 비었나

  std::vector<std::pair<std::string, std::optional<CargoItem>>> slots_;
  std::optional<CargoItem> gripper_;
  std::optional<CargoDoubt> doubt_;
  SlotRules store_rules_, retrieve_rules_;
};

}  // namespace planning
