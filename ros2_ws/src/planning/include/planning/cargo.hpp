// cargo.hpp — 적재함 · 그리퍼 재고 기록
//
// 역할
//   - 어느 칸에 어떤 퍽이 있는지, 그리퍼에 무엇을 쥐고 있는지 기록
//   - 팔 작업 성공 결과로만 바꿈, 명령을 보낸 시점에는 안 바꿈
//       PICK 성공     = 그리퍼 ← 퍽
//       STORE 성공    = 칸 ← 그리퍼
//       RETRIEVE 성공 = 그리퍼 ← 칸
//       PLACE 성공    = 그리퍼 비움, 놓은 퍽을 돌려줌
//   - yaml 저장 · 읽기, 노드 재시작 대비
// 입력
//   - 칸 이름 목록, 팔 작업 성공 결과
// 출력
//   - 칸 · 그리퍼 내용, 전환 성공 여부
// 실행
//   - planning_core 라이브러리, Executor 가 씀. 시험 test_planning_core
// note
//   - 기록과 맞지 않는 전환은 false 로 거부, 예: 빈 그리퍼로 STORE → 노드가 경고 · 상태 표시
//   - 칸 이름은 control 과 같은 이름을 씀, 팔이 닿는 칸만 등록. 기본 S1–S12 = URDF rack_cell_S*, 팔에서 먼 칸부터
//   - 빈 칸은 목록 순서대로 채움 (free_slot) → S1 = 가장 먼 칸부터
//   - 꺼낼 칸은 운반 계획 (transfer_planner) 이 정함: 목록 뒤 = 가까운 칸부터

#pragma once

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

class Cargo
{
public:
  explicit Cargo(std::vector<std::string> slot_names = {});

  // ---------------------------------------------------------------- 조회
  bool holding() const {return gripper_.has_value();}
  const std::optional<CargoItem> & gripper() const {return gripper_;}
  std::optional<std::string> free_slot() const;                      // 첫 빈 칸
  std::size_t stored_count() const;                                  // 칸에 실린 수 (그리퍼 제외)
  std::size_t capacity() const {return slots_.size();}
  const std::vector<std::pair<std::string, std::optional<CargoItem>>> & slots() const {return slots_;}

  // ---------------------------------------------------------------- 전환 (팔 작업 성공 시에만 호출)
  bool on_pick(const CargoItem & item, std::string * error = nullptr);
  bool on_store(const std::string & slot, std::string * error = nullptr);
  bool on_retrieve(const std::string & slot, std::string * error = nullptr);
  std::optional<CargoItem> on_place(std::string * error = nullptr);

  // 그리퍼에서 퍽을 놓쳤다고 판단될 때 (집기 실패 확인 등)
  void clear_gripper() {gripper_.reset();}

  // ---------------------------------------------------------------- 파일
  bool save(const std::string & path, std::string * error = nullptr) const;
  // 파일의 칸 이름이 지금 설정과 다르면 실패 (잘못된 기록을 쓰지 않게)
  bool load(const std::string & path, std::string * error = nullptr);

private:
  std::optional<CargoItem> * find_slot(const std::string & slot);

  std::vector<std::pair<std::string, std::optional<CargoItem>>> slots_;
  std::optional<CargoItem> gripper_;
};

}  // namespace planning
