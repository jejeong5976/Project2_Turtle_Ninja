#pragma once
// Diagnostic only. Fixed-size bounded queue; producers never wait for file I/O.
// Binary schema v1: little-endian, 104-byte records, all fields initialized.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace nt_timing {
enum Kind : uint64_t {
  Header=1, End=2, Read=10, Packet=11, Raw=12, Frame=13, Beam=14, Handoff=15,
  MappingReceived=20, MappingBegin=21, MappingEnd=22, ScanPublishBegin=23,
  ScanPublishEnd=24, OdomReceived=25, QueueDrop=26, OdomWait=27, Rejected=28,
  StateDecoded=30, StateJoint=31, HardwareBegin=32, HardwareEnd=33, StateUsed=34,
  ControlBegin=40, ControlEnd=41, OdomMade=42, OdomPublishBegin=43, OdomPublishEnd=44,
  ScanReceived=50, ObserverOdom=51, MapReceived=52, MapOdomReceived=53
};
inline int64_t clock_ns(clockid_t clock) noexcept {
  timespec t{}; if(clock_gettime(clock,&t)!=0)return -1;
  return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;
}
struct Record {
  uint64_t kind=0,id=0;
  int64_t mono_before=0,system=0,mono_after=0,stamp=0,a=0,b=0,c=0,d=0;
  double x=0,y=0,z=0;
};
static_assert(sizeof(Record)==104,"Unexpected trace layout");
static_assert(std::atomic<size_t>::is_always_lock_free,"Trace queue needs lock-free counters");
// Bounded MPSC queue. A producer gives up after a bounded number of CAS attempts.
// Every loss is counted in the footer. Consumers never skip an unpublished slot.
template<size_t Capacity> class Queue {
  static_assert(Capacity>=2 && (Capacity&(Capacity-1))==0,"Power of two capacity required");
  struct Cell {std::atomic<size_t> sequence{0}; Record record;};
  std::unique_ptr<Cell[]> cells_{new Cell[Capacity]};
  std::atomic<size_t> head_{0}; size_t tail_=0;
public:
  Queue(){for(size_t i=0;i<Capacity;++i)cells_[i].sequence.store(i);}
  bool push(const Record &r) noexcept {
    size_t pos=head_.load(std::memory_order_relaxed);
    for(int attempt=0;attempt<32;++attempt) {
      auto &cell=cells_[pos&(Capacity-1)];
      const size_t seq=cell.sequence.load(std::memory_order_acquire);
      const auto delta=static_cast<intptr_t>(seq-pos);
      if(delta==0) {
        if(head_.compare_exchange_weak(pos,pos+1,std::memory_order_relaxed)) {
          cell.record=r;cell.sequence.store(pos+1,std::memory_order_release);return true;
        }
      } else if(delta<0)return false;
      else pos=head_.load(std::memory_order_relaxed);
    }
    return false;
  }
  bool pop(Record &r) noexcept {
    auto &cell=cells_[tail_&(Capacity-1)];
    if(cell.sequence.load(std::memory_order_acquire)!=tail_+1)return false;
    r=cell.record;cell.sequence.store(tail_+Capacity,std::memory_order_release);++tail_;return true;
  }
};
class Logger {
  std::unique_ptr<Queue<16384>> queue_;
  std::thread worker_; FILE *file_=nullptr;
  std::atomic<bool> stop_{false},io_error_{false};
  std::atomic<uint64_t> accepted_{0},attempted_{0},lost_{0},max_emit_ns_{0};
  uint64_t written_=0,limit_=20000000; bool closed_=false;
  static Record make(Kind kind,uint64_t id,int64_t stamp,int64_t a,int64_t b,int64_t c,int64_t d,double x,double y,double z) noexcept {
    Record r{};r.kind=kind;r.id=id;r.stamp=stamp;r.a=a;r.b=b;r.c=c;r.d=d;r.x=x;r.y=y;r.z=z;
    r.mono_before=clock_ns(CLOCK_MONOTONIC);r.system=clock_ns(CLOCK_REALTIME);r.mono_after=clock_ns(CLOCK_MONOTONIC);return r;
  }
  bool write(const Record *r,size_t n) noexcept {
    if(io_error_)return false;
    if(std::fwrite(r,sizeof(Record),n,file_)!=n){io_error_=true;return false;}return true;
  }
  void drain() noexcept {
    std::array<Record,512> block{};
    while(true) {
      size_t n=0;while(n<block.size()&&queue_->pop(block[n]))++n;
      if(n){if(write(block.data(),n))written_+=n;continue;}
      if(stop_)break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
public:
  explicit Logger(const char *label,uint64_t limit=20000000):limit_(limit) {
    const char *dir=std::getenv("NT_TIMING_DIR");if(!dir||!*dir)return;
    uint16_t endian=1;if(*reinterpret_cast<unsigned char*>(&endian)!=1)throw std::runtime_error("Trace requires little endian");
    const std::string path=std::string(dir)+"/"+label+"_"+std::to_string(getpid())+".nttrace";
    const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
    if(fd<0)throw std::runtime_error("Cannot create new trace: "+path);
    file_=fdopen(fd,"wb");if(!file_){::close(fd);throw std::runtime_error("Trace fdopen failed");}
    try {
      queue_=std::make_unique<Queue<16384>>();
      const auto header=make(Header,1,0,sizeof(Record),getpid(),limit_,0,0,0,0);
      if(!write(&header,1))throw std::runtime_error("Trace header write failed");
      worker_=std::thread([this]{drain();});
    }catch(...){std::fclose(file_);file_=nullptr;throw;}
  }
  Logger(const Logger &)=delete;Logger &operator=(const Logger &)=delete;
  bool enabled()const noexcept{return file_!=nullptr;}
  void event(Kind kind,uint64_t id=0,int64_t stamp=0,int64_t a=0,int64_t b=0,int64_t c=0,int64_t d=0,double x=0,double y=0,double z=0) noexcept {
    if(!file_)return;
    const auto start=clock_ns(CLOCK_MONOTONIC);
    const auto r=make(kind,id,stamp,a,b,c,d,x,y,z);
    if(stop_||io_error_||attempted_.fetch_add(1,std::memory_order_relaxed)>=limit_||!queue_->push(r))++lost_;
    else ++accepted_;
    const auto elapsed=clock_ns(CLOCK_MONOTONIC)-start;
    auto prior=max_emit_ns_.load();
    while(elapsed>=0&&uint64_t(elapsed)>prior&&!max_emit_ns_.compare_exchange_weak(prior,uint64_t(elapsed))){}
  }
  // Call only after producer threads/callbacks have stopped.
  void close() noexcept {
    if(!file_||closed_)return;
    closed_=true;stop_=true;
    if(worker_.joinable())worker_.join();
    const auto footer=make(End,accepted_,0,written_,lost_,io_error_?1:0,max_emit_ns_,0,0,0);
    write(&footer,1);
    if(std::fflush(file_)!=0||fsync(fileno(file_))!=0)io_error_=true;
    if(std::fclose(file_)!=0)io_error_=true;
    file_=nullptr;
    if(io_error_||lost_)std::fprintf(stderr,"TIMING TRACE INVALID: lost=%llu io_error=%d\n",static_cast<unsigned long long>(lost_.load()),int(io_error_.load()));
  }
  ~Logger(){close();}
};
class Span {
  Logger &log_; Kind end_; uint64_t id_;int64_t stamp_;
public:
  Span(Logger &log,Kind begin,Kind end,uint64_t id,int64_t stamp):log_(log),end_(end),id_(id),stamp_(stamp){log_.event(begin,id_,stamp_);}
  ~Span(){log_.event(end_,id_,stamp_);}
};
}
