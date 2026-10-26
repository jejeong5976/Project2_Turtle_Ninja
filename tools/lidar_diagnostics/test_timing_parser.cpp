// Synthetic serial input through a fresh pseudo-terminal; never opens a LiDAR port.
#include <cassert>
#include <fstream>
#include <iterator>
#include <pty.h>
#include <unistd.h>
#include "coin_d4_driver/lidar_sdk/lidar_data_processor.hpp"
std::vector<uint8_t> packet(bool ring,bool valid,uint8_t count=1) {
  const uint16_t angle=(180*64<<1)|1;
  const uint8_t ct=ring ? 1 : 0;
  // One intensity-mode sample: 1000 mm, intensity 100, exposure 0.
  const uint8_t quality=144, low=161, high=15;
  uint16_t sum=PH ^ angle ^ angle ^ (ct | (count<<8));
  for (unsigned i=0;i<count;++i) sum^=quality ^ (low | (high<<8));
  if (!valid) sum^=1;
  std::vector<uint8_t> bytes{0xaa,0x55,ct,count,static_cast<uint8_t>(angle),static_cast<uint8_t>(angle>>8),
    static_cast<uint8_t>(angle),static_cast<uint8_t>(angle>>8),
    static_cast<uint8_t>(sum),static_cast<uint8_t>(sum>>8)};
  for (unsigned i=0;i<count;++i) bytes.insert(bytes.end(),{quality,low,high});
  return bytes;
}
int main(int argc,char** argv) {
  assert(argc==2);
  int master=-1,slave=-1; char path[256]{};
  assert(openpty(&master,&slave,path,nullptr,nullptr)==0);
  close(slave);
  SerialPort serial(path,115200,Timeout::simpleTimeout(1000));
  assert(serial.open());
  LidarTimeStatus times{}; LidarHardwareStatus status{}; LidarGeneralInfo info{}; LidarPackage packages{};
  info.intensity_data_flag=true; status.serial_connected=true;
  auto recorder=std::make_shared<lidar_timing::Recorder>(argv[1],1);
  LidarDataProcessor parser(&times,&status,info,packages);
  parser.package_sample_bytes_=3;
  parser.set_serial_port(&serial); parser.set_timing_recorder(recorder);
  auto feed=[&](bool ring,bool valid) {
    const auto bytes=packet(ring,valid);
    assert(write(master,bytes.data(),bytes.size())==static_cast<ssize_t>(bytes.size()));
    node_info node{};
    assert(parser.wait_package(&node,1000)==RESULT_OK);
    assert(node.timing_checksum_ok==valid);
    assert(node.timing_sample_index==0);
    if (valid) assert(node.distance_q2==1000);
    return node.timing_packet_id;
  };
  assert(feed(true,false)==1); assert(times.tim_scan_start==0 && times.tim_scan_end==0);
  assert(feed(true,true)==2); assert(times.tim_scan_start>0 && times.tim_scan_end==0);
  const auto start=times.tim_scan_start;
  assert(feed(false,true)==3); assert(times.tim_scan_start==start && times.tim_scan_end==0);
  assert(feed(true,false)==4); assert(times.tim_scan_end==0);
  assert(feed(true,true)==5); assert(times.tim_scan_end>=start);
  const auto bytes=packet(false,true,3);
  assert(write(master,bytes.data(),bytes.size())==static_cast<ssize_t>(bytes.size()));
  for (unsigned i=0;i<3;++i) {
    node_info node{};
    assert(parser.wait_package(&node,1000)==RESULT_OK);
    assert(node.timing_packet_id==6 && node.timing_sample_index==i);
    assert(node.distance_q2==1000 && node.timing_checksum_ok==1);
  }
  recorder->published();
  serial.close(); close(master);
  std::ifstream file(argv[1]);
  const std::string content{std::istreambuf_iterator<char>(file),{}};
  assert(content.find("\"kind\":\"read\"")!=std::string::npos);
  assert(content.find("\"kind\":\"packet\"")!=std::string::npos);
  assert(content.find("\"complete\":1")!=std::string::npos);
  std::puts("PASS: synthetic serial packets, checksum-gated ring timing, provenance, read events");
}
