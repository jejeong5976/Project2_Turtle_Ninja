#include "nt_packet.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// User-run receive-only diagnostic. Never constructs or sends CONFIG/CMD.
// O_RDONLY prevents application writes to the serial descriptor. Setting raw
// mode also disables terminal echo/flow-control output during the capture.
namespace {
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
void system_check(bool ok, const std::string& operation) {
  if (!ok) throw std::runtime_error(operation+": "+std::strerror(errno));
}
struct Descriptor {
  int fd=-1;
  bool exclusive=false;
  explicit Descriptor(int value):fd(value) {}
  ~Descriptor() {
    if (fd>=0) {
      if (exclusive) ioctl(fd,TIOCNXCL);
      close(fd);
    }
  }
  Descriptor(const Descriptor&)=delete;
  Descriptor& operator=(const Descriptor&)=delete;
};
void save_bytes(int fd, const uint8_t* data, std::size_t size) {
  while (size) {
    const auto n=write(fd,data,size); // Capture file only, never the serial FD.
    if (n<0 && errno==EINTR) continue;
    system_check(n>0,"Write capture file");
    data+=n; size-=std::size_t(n);
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc==3 || argc==4, "Usage: listen_status DEVICE NEW_CAPTURE_BIN [DURATION_MS=5000]");
    int duration=5000;
    if (argc==4) {
      std::size_t used=0;
      duration=std::stoi(argv[3],&used);
      require(used==std::strlen(argv[3]) && duration>=100 && duration<=30000,
              "Duration must be 100..30000 milliseconds");
    }
    const uint16_t endian=1;
    require(*reinterpret_cast<const uint8_t*>(&endian)==1 && sizeof(float)==4,
            "Protocol decoding requires little endian and 4-byte float");
    Descriptor serial(open(argv[1],O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC));
    system_check(serial.fd>=0,"Open input device");
    require(isatty(serial.fd),"Input must be a serial/PTY device");
    system_check(flock(serial.fd,LOCK_EX|LOCK_NB)==0,"Lock device");
    system_check(ioctl(serial.fd,TIOCEXCL)==0,"Exclude new device opens");
    serial.exclusive=true;
    Descriptor capture(open(argv[2],O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600));
    system_check(capture.fd>=0,"Create new capture file");
    termios settings{};
    system_check(tcgetattr(serial.fd,&settings)==0,"Read terminal settings");
    cfmakeraw(&settings);
    settings.c_iflag&=~(IXON|IXOFF|IXANY);
    settings.c_cflag&=~CRTSCTS;
    settings.c_cflag|=CLOCAL|CREAD;
    settings.c_cc[VMIN]=0; settings.c_cc[VTIME]=0;
    system_check(cfsetispeed(&settings,B1000000)==0 && cfsetospeed(&settings,B1000000)==0,
                 "Set host line rate");
    system_check(tcsetattr(serial.fd,TCSANOW,&settings)==0,"Set raw receive mode");
    // Leave raw mode in place on exit: restoring an earlier echo-enabled mode
    // could echo a continuing telemetry stream back to the device.
    std::cout << "LISTENING duration_ms=" << duration
              << " serial_access=O_RDONLY application_serial_writes=0\n" << std::flush;

    ntp::Parser parser;
    ntp::State last{};
    std::size_t bytes=0, states=0, diagnostics=0, other_frames=0, malformed=0, nonadvancing=0;
    uint32_t first_ms=0;
    uint8_t status_or=0;
    std::array<uint8_t,32> prefix{};
    std::size_t prefix_size=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(duration);
    while (std::chrono::steady_clock::now()<deadline) {
      pollfd waiting{serial.fd,POLLIN,0};
      const int ready=poll(&waiting,1,25);
      if (ready<0 && errno==EINTR) continue;
      system_check(ready>=0,"Poll input");
      require(!(waiting.revents&(POLLERR|POLLHUP|POLLNVAL)),"Device disconnected or input error");
      if (!(waiting.revents&POLLIN)) continue;
      uint8_t buffer[4096];
      const auto n=read(serial.fd,buffer,sizeof(buffer));
      if (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) continue;
      system_check(n>=0,"Read input");
      require(bytes+std::size_t(n)<=4*1024*1024,"Capture exceeded 4 MiB limit");
      save_bytes(capture.fd,buffer,std::size_t(n));
      bytes+=std::size_t(n);
      for (ssize_t i=0;i<n;++i) {
        if (prefix_size<prefix.size()) prefix[prefix_size++]=buffer[i];
        if (!parser.feed(buffer[i])) continue;
        if (parser.type()==ntp::TYPE_STATE) {
          ntp::State current{};
          if (!ntp::decode_state(parser.data(),parser.len(),current)) {++malformed; continue;}
          if (!states) first_ms=current.ms;
          else {
            const uint32_t step=current.ms-last.ms;
            nonadvancing+=step==0 || step>0x7fffffffU;
          }
          ++states; last=current; status_or|=current.status;
        } else if (parser.type()==ntp::TYPE_DIAG) {
          ntp::Diag diag{};
          if (ntp::decode_diag(parser.data(),parser.len(),diag)) ++diagnostics;
          else ++malformed;
        } else ++other_frames;
      }
    }
    system_check(fsync(capture.fd)==0,"Flush capture file");
    std::cout << "bytes=" << bytes << " state_frames=" << states << " diag_frames=" << diagnostics
              << " other_frames=" << other_frames << " malformed_payloads=" << malformed
              << " bad_checksums=" << parser.bad_checksum_count() << '\n';
    if (states) {
      std::cout << "first_board_ms=" << first_ms << " last_board_ms=" << last.ms
                << " nonadvancing_state_stamps=" << nonadvancing
                << " last_joint_count=" << unsigned(last.n)
                << " last_status=" << unsigned(last.status)
                << " status_or=" << unsigned(status_or)
                << " last_comm_error_mask=" << last.comm_err << '\n'
                << "last_configured=" << bool(last.status&ntp::ST_CONFIGURED)
                << " last_torque_flag=" << bool(last.status&ntp::ST_TORQUE)
                << " last_imu_flag=" << bool(last.status&ntp::ST_IMU_OK) << '\n';
    }
    std::cout << "first_bytes_hex=";
    for (std::size_t i=0;i<prefix_size;++i) {
      std::cout << (i?" ":"") << std::hex << std::setfill('0') << std::setw(2) << unsigned(prefix[i]);
    }
    std::cout << std::dec << '\n';
    if (states>=2 && !nonadvancing) {
      std::cout << "RESULT: advancing telemetry matching the project STATE format observed.\n";
    } else {
      std::cout << "RESULT: insufficient advancing project STATE telemetry; firmware remains unidentified.\n";
    }
    std::cout << "Protocol recognition does not identify a firmware build, motor IDs, calibration, or odometry accuracy.\n";
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
