// bag.hpp — ros2 bag record 를 따로 띄워 기록 (화면 bag start|stop). ROS 없이 (프로세스만)
//   자기 프로세스 그룹으로 띄움 → 터미널 Ctrl-C 가 바로 가지 않고, 끝낼 때 SIGINT 를 보내 정상 종료 (metadata 씀)
//   10 s 안에 안 끝나면 SIGTERM, 다시 3 s 뒤 SIGKILL. 화면은 request_stop + poll 로 기다리지 않음 (F9 · 입력이 막히지 않게)
#pragma once

#include <sys/types.h>

#include <string>
#include <vector>

namespace robot_cli
{

// 기록할 토픽 (팔 · 바퀴 · 위치 · 라이다 · 차례 · 팔 작업 상태 · 진단)
extern const std::vector<std::string> kBagTopics;

class BagRecorder
{
public:
  ~BagRecorder() {stop();}
  // dir 아래 bag_001 … 중 빈 이름으로 기록 시작. 성공하면 true, text = 기록 폴더 (실패면 이유)
  bool start(const std::string & dir, std::string & text);
  // 끝내기 시작 (SIGINT 만 보내고 바로 돌아옴). 기록 중이 아니면 false
  bool request_stop();
  // 끝내는 중이면 시간에 따라 SIGTERM · SIGKILL 을 보내고, 끝났으면 true (기록 중이 아니어도 true)
  bool poll();
  bool stopping() const {return stop_since_ >= 0.0;}
  // 끝날 때까지 기다리며 정상 종료 (끝낼 때만). 멈춘 게 있으면 true
  bool stop();
  bool running();
  const std::string & path() const {return path_;}

private:
  pid_t pid_{-1};
  std::string path_;
  double stop_since_{-1.0};   // SIGINT 보낸 시각 (steady s), -1 = 끝내는 중 아님
  bool sent_term_{false}, sent_kill_{false};
};

// dir 안에 prefix_001 · prefix_002 … 중 아직 없는 첫 이름 (ext 붙여 확인)
std::string next_free(const std::string & dir, const std::string & prefix, const std::string & ext);
// 같은 이름 규칙으로 빈 파일을 O_EXCL 로 만들어 이름을 차지함 (화면 두 개가 같은 이름을 고르지 않게). 경로 (실패면 "")
std::string reserve_free(const std::string & dir, const std::string & prefix, const std::string & ext);

}  // namespace robot_cli
