#include "nt_packet.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {
void require(bool ok,const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
struct Result { std::string text; std::vector<uint8_t> captured; };

Result exercise(const char* executable,const std::filesystem::path& output,
                const std::vector<uint8_t>& incoming) {
  int master=-1,slave=-1,logs[2];
  char device[128]{};
  require(openpty(&master,&slave,device,nullptr,nullptr)==0,"openpty failed");
  require(pipe(logs)==0,"pipe failed");
  const pid_t child=fork();
  require(child>=0,"fork failed");
  if (child==0) {
    close(master); close(slave); close(logs[0]);
    dup2(logs[1],STDOUT_FILENO); dup2(logs[1],STDERR_FILENO); close(logs[1]);
    execl(executable,executable,device,output.c_str(),"200",nullptr);
    _exit(127);
  }
  close(logs[1]);
  Result result;
  bool sent=false,done=false,transmitted=false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while (!done && std::chrono::steady_clock::now()<deadline) {
    pollfd fds[2]={{logs[0],POLLIN,0},{master,POLLIN,0}};
    const int ready=poll(fds,2,50);
    if (ready<0 && errno==EINTR) continue;
    require(ready>=0,"poll failed");
    if (fds[0].revents&(POLLIN|POLLHUP)) {
      char buffer[2048];
      const auto n=read(logs[0],buffer,sizeof(buffer));
      require(n>=0,"log read failed");
      if (n==0) done=true;
      else result.text.append(buffer,std::size_t(n));
    }
    if (!sent && result.text.find("application_serial_writes=0\n")!=std::string::npos) {
      // Splitting at every byte exercises fragmented reads/parser state.
      for (uint8_t byte:incoming) require(write(master,&byte,1)==1,"PTY send failed");
      sent=true;
    }
    if (fds[1].revents&POLLIN) {
      uint8_t buffer[256];
      if (read(master,buffer,sizeof(buffer))>0) transmitted=true;
    }
  }
  if (!done) kill(child,SIGKILL);
  int status=0;
  waitpid(child,&status,0);
  pollfd final_read{master,POLLIN,0};
  if (poll(&final_read,1,0)>0 && (final_read.revents&POLLIN)) transmitted=true;
  close(logs[0]); close(master); close(slave);
  require(done && WIFEXITED(status) && WEXITSTATUS(status)==0,"Listener failed: "+result.text);
  require(sent,"Listener did not reach receive mode");
  require(!transmitted,"Listener or terminal echo transmitted bytes to the simulated device");
  std::ifstream capture(output,std::ios::binary);
  require(capture.good(),"Missing raw capture");
  result.captured=std::vector<uint8_t>(std::istreambuf_iterator<char>(capture),{});
  require(result.captured==incoming,"Raw capture differs from device bytes");
  return result;
}
}  // namespace

int main(int argc,char** argv) {
  try {
    require(argc==2,"Usage: test_listen_status LISTENER_EXECUTABLE");
    char pattern[]="/tmp/opencr-listener-test.XXXXXX";
    const char* made=mkdtemp(pattern);
    require(made,"mkdtemp failed");
    const std::filesystem::path directory(made);
    std::vector<uint8_t> incoming={'?','\r','\n'};
    uint8_t frame[ntp::MAX_FRAME];
    ntp::State state{};
    state.ms=1000; state.status=ntp::ST_IMU_OK; state.imu.quat[0]=1;
    auto size=ntp::encode_state(state,frame,sizeof(frame));
    incoming.insert(incoming.end(),frame,frame+size);
    frame[size-1]^=0x80; // Valid-looking frame with invalid checksum must be rejected.
    incoming.insert(incoming.end(),frame,frame+size);
    ntp::Diag diag{}; diag.ms=1005;
    size=ntp::encode_diag(diag,frame,sizeof(frame));
    incoming.insert(incoming.end(),frame,frame+size);
    state.ms=1010;
    size=ntp::encode_state(state,frame,sizeof(frame));
    incoming.insert(incoming.end(),frame,frame+size);
    const auto recognized=exercise(argv[1],directory/"recognized.bin",incoming);
    require(recognized.text.find("state_frames=2 diag_frames=1")!=std::string::npos,"Incorrect decoded counts");
    require(recognized.text.find("bad_checksums=1")!=std::string::npos,"Checksum error not reported");
    require(recognized.text.find("last_joint_count=0")!=std::string::npos,"Unconfigured STATE not decoded");
    require(recognized.text.find("RESULT: advancing telemetry")!=std::string::npos,"Advancing telemetry not recognized");
    const auto silent=exercise(argv[1],directory/"silent.bin",{});
    require(silent.text.find("bytes=0 state_frames=0")!=std::string::npos,"Silent input miscounted");
    require(silent.text.find("insufficient advancing")!=std::string::npos,"Silent input wrongly recognized");
    const std::vector<uint8_t> text={'H','E','L','L','O','\r','\n'};
    const auto unknown=exercise(argv[1],directory/"unknown.bin",text);
    require(unknown.text.find("state_frames=0")!=std::string::npos,"Text misidentified as project protocol");
    require(unknown.text.find("insufficient advancing")!=std::string::npos,"Unknown input wrongly recognized");
    std::cout << "PASS: fragmented/corrupt protocol, silence, unknown text, exact captures and no PTY output.\n"
              << "Test evidence: " << directory << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
