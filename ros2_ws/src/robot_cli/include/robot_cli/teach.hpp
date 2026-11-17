// teach.hpp — 7 실험 쪽 도구 (ROS 없이, test/test_teach.cpp)
//   mark <이름>: 지금 관절값을 덧붙여 기록 (팔은 안 움직임) — poses.csv · poses.yaml (관절 눈금 · rad · deg · 보정값, 그리퍼,
//     차례, 가까운 이름 자세, odom · map 위치, 손목 검출) + detections.csv (검출 하나 + 그때 관절값, 칸 바닥 AprilTag 20~31 로 카메라 자세 재기)
//   cycle <칸> <횟수> [store]: ArmTask 만으로 RETRIEVE → STORE 반복 (store = STORE 먼저), 실패하면 멈춤. cycle_<칸>.csv 에 시간 · phase · 결과
//   손으로 움직이기 (토크 끄기) 는 없음 — 지금 인터페이스로 안전하게 못 함 (README '손으로 가르치기')
#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_cli/command.hpp"
#include "robot_cli/model.hpp"
#include "robot_cli/snapshot.hpp"

namespace robot_cli
{

constexpr double kMarkMaxAgeS = 1.0;   // 관절 측정이 이보다 오래되면 mark 안 함

extern const char * const kTeachNote;  // teach 명령 글: 손으로 움직이기가 왜 없는지 · 무엇이 필요한지

std::string wall_clock_text();   // 이 PC 지역 시각 "YYYY-MM-DDThh:mm:ss.mmm" (기록 칸 값)
std::string csv_clean(const std::string & s);   // csv 칸 · yaml "…" 에 넣을 수 있게 (, → ; · " \ 빼기 · 줄바꿈 → 빈칸)

std::string mark_csv_header();
std::string mark_csv_row(const std::string & name, const std::string & wall, const Snapshot & s, const Config & cfg);
std::string mark_yaml_entry(const std::string & name, const std::string & wall, const Snapshot & s, const Config & cfg);
std::string det_csv_header();
std::vector<std::string> det_csv_rows(const std::string & name, const std::string & wall, const Snapshot & s);

// dir 에 세 파일을 덧붙여 씀 (없으면 폴더 · 머리줄을 만듦). 성공하면 true, text = 한 줄 요약 (실패면 이유)
bool write_mark(const std::string & dir, const std::string & name, const Snapshot & s, const Config & cfg, std::string & text);
// csv 파일에 줄 덧붙이기 (없거나 비면 머리줄 먼저)
bool append_csv(const std::string & path, const std::string & header, const std::vector<std::string> & rows, std::string & err);

class CycleRun
{
public:
  bool start(const std::string & slot, int rounds, bool store_first, std::string & err);
  void request_stop();                     // 지금 작업이 끝나면 멈춤
  void abort(const std::string & why);     // 바로 멈춤 (보낸 작업 취소는 부른 쪽이)
  bool active() const {return active_;}
  bool waiting() const {return waiting_;}
  const std::string & slot() const {return slot_;}
  // 보낼 차례면 c 를 채우고 true (보낸 것으로 표시)
  bool next(Command & c, double now, const std::string & wall);
  // 보낸 작업의 결과 (code 0 = 성공). 사람 글 한 줄을 돌려주고 csv_row 에 기록 줄
  std::string on_done(int code, const std::string & text, double now, const std::string & phases, std::string * csv_row);
  std::string status(double now) const;
  static std::string csv_header();

private:
  std::string task_name_now() const;
  std::string stats() const;

  std::string slot_;
  int rounds_{0};
  bool store_first_{false};
  int step_{0};                            // 보낸 작업 수
  bool active_{false}, waiting_{false}, stop_req_{false};
  double t_sent_{0.0};
  std::string wall_sent_;
  int ok_{0};
  std::map<std::string, std::vector<double>> secs_;   // 작업 이름 → 걸린 시간들
  std::string end_text_;
};

}  // namespace robot_cli
