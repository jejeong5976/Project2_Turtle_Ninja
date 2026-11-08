// nt_packet.h — Jetson ↔ OpenCR 패킷 형식 (편지 규칙)
// =====================================================================
// ★ 이 파일 하나를 OpenCR 펌웨어(firmware/nt_opencr)와 ROS 플러그인(ros2_ws/src/nt_hardware)이 **같이** 쓴다.
//   양쪽에 따로 적으면 언젠가 어긋남 → 파일 자체를 하나로 해서 어긋날 수가 없게.
//   (아두이노 스케치는 자기 폴더 밖 파일을 못 읽어서 원본이 여기 있고, nt_hardware 가 이 폴더를 include 한다)
//   그래서 표준 라이브러리(vector 등)도 Arduino 전용 함수도 안 씀 — 순수 C++ 과 <stdint.h> <string.h> 만.
//
// 모양 (택배 상자: 머리 + 종류 + 길이 + 데이터 + 체크섬)
//   | 0xAA | 0x55 | 종류(1) | 길이(1) | 데이터(길이만큼) | 체크섬(1) |
//   체크섬 = (종류 + 길이 + 데이터 바이트 전부의 합) & 0xFF        ← 헤더 0xAA 0x55 는 안 더함
//   숫자는 전부 리틀 엔디언, float = 4바이트
//   길이 칸이 있는 이유: 모터 수가 hardware_map.yaml 에 따라 바뀌어서 상자 크기가 고정이 아님
//
// 종류 6개 (바이트 정의는 아래 구조체와 encode/decode 함수 참고)
//   0x03 CONFIG     Jetson → OpenCR  켤 때 한 번: "모터 n개, 각각 ID · 모드 · 속도/가속 한계"
//   0x04 CONFIG_ACK OpenCR → Jetson  "몇 번 모터가 대답했나" (ping 결과)
//   0x01 CMD        Jetson → OpenCR  100 Hz: 토크 켜기 + 모터별 목표 (rad 또는 rad/s)
//   0x02 STATE      OpenCR → Jetson  100 Hz: 시간 + IMU + 모터별 위치 · 속도 · 전류
//   0x05 DIAG       OpenCR → Jetson  1 Hz: 보드 안에서 잰 시간 (한 바퀴 · Sync Read/Write · 개별 읽기/쓰기 · 고장 정지 · USB 보내기)
//                   + CONFIG 를 받으면 그 순간 구간을 끊어 "CONFIG 전" 표시로 한 번 보냄 (전후 숫자가 섞이지 않게)
//   0x06 TEMP       OpenCR → Jetson  1 Hz (DIAG 바로 뒤): 모터별 Present Temperature [°C]
//   0x07 TORQUE_FAIL OpenCR → Jetson 토크 켜기가 실패하는 동안 0.5 s 마다: 처음 실패한 모터 · 단계 · 모터 오류 · 지금 위치 · 위치 한계
//                   (11-07 팀장님 요청: "토크를 못 켬" 만으로는 어느 모터가 왜 못 켰는지 모름)
//   0x08 POS_LIMITS  OpenCR → Jetson CONFIG_ACK 바로 뒤 한 번: 모터별 EEPROM 위치 한계 (Min/Max Position Limit) [눈금], 위치 모드만
//                   → 플러그인이 켤 때 찍고, 지금 위치가 밖이면 경고 (조향을 손 · 충돌로 돌린 채 켜면 토크가 안 켜짐)
//                   모르는 종류는 받는 쪽이 그냥 버림 → DIAG · TEMP 를 모르는 옛 플러그인과도 같이 쓸 수 있음
//
// 단위 약속 (단위는 한 가지로 — deg/rad 가 섞이면 층마다 헷갈림)
//   위치 rad, 속도 rad/s, 전류 A, 자이로 rad/s, 가속도 m/s², 자세 쿼터니언(w, x, y, z), 온도 °C
//   다이나믹셀 눈금(0~4095 등) ↔ rad 변환은 **펌웨어** 가 한다 (motor_bus.cpp). 그리퍼 rad ↔ m 은 플러그인이 한다.

#pragma once

#include <stdint.h>
#include <string.h>

namespace ntp
{

const uint8_t HEADER1 = 0xAA;
const uint8_t HEADER2 = 0x55;

const uint8_t TYPE_CMD = 0x01;
const uint8_t TYPE_STATE = 0x02;
const uint8_t TYPE_CONFIG = 0x03;
const uint8_t TYPE_CONFIG_ACK = 0x04;
const uint8_t TYPE_DIAG = 0x05;
const uint8_t TYPE_TEMP = 0x06;
const uint8_t TYPE_TORQUE_FAIL = 0x07;
const uint8_t TYPE_POS_LIMITS = 0x08;

const uint8_t MAX_JOINTS = 16;        // 상태 상자가 255 바이트 안에 들어가는 최대 (49 + 12 × 17 = 253)
const uint16_t MAX_FRAME = 4 + 255 + 1;

// 모터 모드 = 다이나믹셀 Operating Mode 값 그대로
const uint8_t MODE_VELOCITY = 1;          // 바퀴
const uint8_t MODE_POSITION = 3;          // 조향 · 팔
const uint8_t MODE_CURRENT_POSITION = 5;  // 그리퍼 (전류 기반 위치 — 물건을 쥐면 그 힘으로 멈춤)

// CMD flags
const uint8_t FLAG_TORQUE = 0x01;         // 1 = 토크 켜기, 0 = 끄기

// STATE status 비트
const uint8_t ST_TORQUE = 0x01;           // 토크 켜짐
const uint8_t ST_HOST_TIMEOUT = 0x02;     // Jetson 명령이 끊김 → 펌웨어가 바퀴를 세움 (경비견)
const uint8_t ST_IMU_OK = 0x04;           // IMU 정상
const uint8_t ST_CONFIGURED = 0x08;       // CONFIG 받음
const uint8_t ST_WRITE_FAIL = 0x10;       // 이번 주기 목표 쓰기(Sync Write) 전송 실패 (R5)
const uint8_t ST_HW_ERROR = 0x20;         // 어느 모터든 Hardware Error Status ≠ 0 (과부하 · 과열 · 전압 → 모터가 스스로 토크 끔)
const uint8_t ST_FAULT_LATCHED = 0x40;    // 펌웨어 고장 잠금 (쓰기 실패로 바퀴를 세움) — CONFIG 를 다시 받을 때까지 계속 켜짐 (R6)

// ------------------------------------------------------------------ 상자 안 내용물
struct JointConfig
{
  uint8_t id;
  uint8_t mode;
  uint16_t profile_vel;    // 다이나믹셀 눈금 그대로 (0 = 제한 없음)
  uint16_t profile_acc;
  uint16_t goal_current;   // 그리퍼(모드 5)만, 눈금 그대로
};

struct Config
{
  uint8_t n;
  uint8_t bus_watchdog;    // 모터 Bus Watchdog (20 ms 단위, 0 = 끔). OpenCR 가 멈추면 모터가 스스로 멈춤
  JointConfig j[MAX_JOINTS];
};

struct ConfigAck
{
  uint8_t n;
  uint16_t ok_mask;        // 비트 i = i 번째 모터가 ping 에 대답함
};

struct Cmd
{
  uint8_t seq;             // 순서 번호 (STATE 에 되돌아옴 → 왕복 확인)
  uint8_t flags;
  uint16_t valid;          // 비트 i = i 번째 목표가 유효 (0 이면 그 모터는 지금 자세 유지 · 바퀴는 0)
  uint8_t n;
  float target[MAX_JOINTS];  // 모드 1 → rad/s, 모드 3 · 5 → rad
};

struct Imu
{
  float quat[4];           // w, x, y, z
  float gyro[3];           // rad/s
  float acc[3];            // m/s²
};

struct State
{
  uint32_t ms;             // OpenCR 시계
  uint8_t seq_echo;        // 마지막으로 받은 CMD 의 seq
  uint8_t status;          // ST_* 비트
  uint16_t comm_err;       // 비트 i = i 번째 모터 읽기 실패
  Imu imu;
  uint8_t n;
  float pos[MAX_JOINTS];   // rad
  float vel[MAX_JOINTS];   // rad/s
  float cur[MAX_JOINTS];   // A
};

// 보드 안 시간 측정 (DIAG) — micros() 로 재서 1 초마다 보냄. 단위 µs (65535 에서 멈춤)
//   왜 보드 안에서: PC 에서 USB 를 거쳐 재면 PC · USB · 중계 펌웨어 시간이 섞여 버스 · 펌웨어 시간을 가를 수 없었음
struct TimeStat
{
  uint16_t n;              // 이 구간(약 1 s) 동안 잰 횟수
  uint16_t fail;           // 그중 실패 (읽기: 대답 없는 모터가 있었음 · 쓰기: 실패)
  uint16_t mean_us;
  uint16_t max_us;
};

struct Diag
{
  uint32_t ms;             // 보낸 시각 (OpenCR 시계)
  uint16_t window_ms;      // 이 숫자들을 모은 시간
  TimeStat loop;           // 한 바퀴 실행 (모터 쓰기 · 읽기 · 되읽기 + 상태 보내기). fail = 10 ms 주기를 넘긴 바퀴
  uint16_t resync;         // 너무 밀려서 주기 기준을 다시 잡은 횟수
  TimeStat sync_read;      // 상태 Sync Read 전체 (연결된 모터 전부)
  TimeStat sync_write;     // 목표 Sync Write (바퀴 · 위치 두 묶음 합)
  TimeStat single_read;    // 개별 읽기 1 회 왕복 (운전 중 되읽기 · 토크 켤 때 위치 읽기)
  TimeStat single_write;   // 개별 쓰기 1 회 왕복 (토크 켜기/끄기 · 고장 정지)
  uint16_t fault_stop_us;  // 마지막 고장 확정 → 개별 정지 시도 끝까지 (0 = 고장 없음)
  uint8_t fault_flags;     // 비트 0 = 고장 잠금, 1 = 버스 침묵, 2 = CONFIG 전 구간 (DIAG_FLAG_*)
  // 배터리 (보드 2장 · 와플파이 배터리): 모터가 재는 입력 전압 [0.1 V], 되읽기 때 1 모터씩. 0 = 이번 구간에 못 읽음
  uint8_t volt_min_dv;     // 이 구간 최저 (출발 순간 처짐 포함)
  uint8_t volt_mean_dv;    // 이 구간 평균
  // 첫 구간에만 한 바퀴 10 ms 초과 · 65535 포화가 보였던 원인 확인용
  TimeStat usb_tx;         // 상태 · DIAG 를 USB 로 보내기 1 회 (기다리지 않는 쓰기). fail = 송신 버퍼가 차서(PC 가 안 읽음) 버린 상자
  uint16_t config_ms;      // 이 구간 바로 앞에서 CONFIG 처리(bus_configure)에 걸린 시간 [ms]. 0 = 이 구간 앞에 CONFIG 없음
};

// fault_flags 비트
const uint8_t DIAG_FLAG_LATCHED = 1;        // 고장 잠금
const uint8_t DIAG_FLAG_SILENT = 2;         // 버스 침묵
const uint8_t DIAG_FLAG_BEFORE_CONFIG = 4;  // CONFIG 를 받는 순간 끊어 보낸 구간 = CONFIG 전 상태의 숫자 (그 뒤 구간과 섞지 말 것)

// 모터 온도 (TEMP) — 되읽기 때 1 모터씩 전압과 한 번에 읽은 Present Temperature (XM430 주소 146, 1 °C 단위)
//   과열 Hardware Error 는 모터 Temperature Limit(기본 80 °C) 에서 걸림 → 그 전에 사람이 보고 쉬게 하려고
const uint8_t TEMP_UNKNOWN = 0xFF;          // 이 구간(약 1 s)에 한 번도 못 읽음 (대답 없음 · 쓰지 않는 모터)

struct Temp
{
  uint8_t n;
  uint8_t c[MAX_JOINTS];   // °C, TEMP_UNKNOWN = 모름
};

// 위치 한계 (POS_LIMITS) — TF_NO_LIMIT = 위치 모드가 아니거나 못 읽음
struct PosLimits
{
  uint8_t n;
  int32_t lo[MAX_JOINTS];
  int32_t hi[MAX_JOINTS];
};

// 토크 켜기 실패 (TORQUE_FAIL) — 위치 모터는 켤 때 ① 지금 위치 읽기 → ② 그 위치를 목표로 쓰기 → ③ Torque Enable 쓰기.
//   ② 전에 지금 위치가 모터 EEPROM 위치 한계 (Min/Max Position Limit) 밖이면 쓰지 않고 TF_RANGE
//   (꺼진 동안 손 · 충돌로 조향이 한계 밖으로 돌면 모터가 그 목표를 거절 → 토크가 계속 안 켜짐, 11-07 실물)
const uint8_t TF_READ = 1;        // ① 지금 위치 읽기 실패 (대답 없음)
const uint8_t TF_RANGE = 2;       // 지금 위치가 위치 한계 밖 → 목표 · 토크를 안 씀
const uint8_t TF_GOAL = 3;        // ② 목표 쓰기 실패 (위치 모터 = 지금 위치, 바퀴 = 속도 0)
const uint8_t TF_TORQUE = 4;      // ③ Torque Enable 쓰기 실패
const uint8_t TF_LOST = 5;        // 켜 둔 토크가 꺼짐 (Torque Enable 되읽기 0 — 과부하 · 과열 · 전압 등으로 모터가 스스로 끔). pos = Hardware Error Status
const uint8_t TF_BUDGET = 6;      // 한 번 켜기 시도가 TORQUE_BUDGET_MS 를 넘음 → 이 모터부터는 다음 바퀴 (버스가 느림)
const int32_t TF_NO_LIMIT = -1;   // 위치 한계를 못 읽음 · 위치 모드가 아님

struct TorqueFail
{
  uint8_t idx;             // 관절 순서 (CONFIG 의 i)
  uint8_t id;              // 다이나믹셀 ID
  uint8_t step;            // TF_*
  uint8_t dxl_err;         // 모터 상태 패킷 Error 칸 (0x04 Data Range · 0x06 Data Limit · 0x07 Access … · 비트 7 = Alert) — 대답이 없으면 0
  uint8_t lib_err;         // Dynamixel2Arduino 오류 코드 (0 = 통신 성공)
  int32_t pos;             // 지금 위치 [눈금] (못 읽었으면 0) · TF_LOST 면 Hardware Error Status
  int32_t min_lim;         // EEPROM Min Position Limit [눈금] (TF_NO_LIMIT = 모름)
  int32_t max_lim;         // EEPROM Max Position Limit [눈금]
};

// ------------------------------------------------------------------ 바이트 넣기 · 꺼내기 (리틀 엔디언)
inline void put(uint8_t *&p, const void *v, uint8_t size) {memcpy(p, v, size); p += size;}
inline void get(const uint8_t *&p, void *v, uint8_t size) {memcpy(v, p, size); p += size;}

// 데이터 → 완성된 상자. 돌려주는 값 = 상자 전체 길이 (0 = 실패)
inline uint16_t frame(uint8_t *out, uint16_t cap, uint8_t type, const uint8_t *data, uint8_t len)
{
  if (cap < static_cast<uint16_t>(len) + 5) {return 0;}
  out[0] = HEADER1;
  out[1] = HEADER2;
  out[2] = type;
  out[3] = len;
  uint8_t sum = static_cast<uint8_t>(type + len);
  for (uint8_t i = 0; i < len; ++i) {
    out[4 + i] = data[i];
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  out[4 + len] = sum;
  return static_cast<uint16_t>(len + 5);
}

// ------------------------------------------------------------------ 종류별 포장 (encode) · 개봉 (decode)
inline uint16_t encode_config(const Config &c, uint8_t *out, uint16_t cap)
{
  if (c.n > MAX_JOINTS) {return 0;}
  uint8_t d[2 + 8 * MAX_JOINTS];
  uint8_t *p = d;
  put(p, &c.n, 1);
  put(p, &c.bus_watchdog, 1);
  for (uint8_t i = 0; i < c.n; ++i) {
    put(p, &c.j[i].id, 1);
    put(p, &c.j[i].mode, 1);
    put(p, &c.j[i].profile_vel, 2);
    put(p, &c.j[i].profile_acc, 2);
    put(p, &c.j[i].goal_current, 2);
  }
  return frame(out, cap, TYPE_CONFIG, d, static_cast<uint8_t>(p - d));
}

inline bool decode_config(const uint8_t *d, uint8_t len, Config &c)
{
  if (len < 2) {return false;}
  const uint8_t *p = d;
  get(p, &c.n, 1);
  get(p, &c.bus_watchdog, 1);
  if (c.n > MAX_JOINTS || len != 2 + 8 * c.n) {return false;}
  for (uint8_t i = 0; i < c.n; ++i) {
    get(p, &c.j[i].id, 1);
    get(p, &c.j[i].mode, 1);
    get(p, &c.j[i].profile_vel, 2);
    get(p, &c.j[i].profile_acc, 2);
    get(p, &c.j[i].goal_current, 2);
  }
  return true;
}

inline uint16_t encode_ack(const ConfigAck &a, uint8_t *out, uint16_t cap)
{
  uint8_t d[3];
  uint8_t *p = d;
  put(p, &a.n, 1);
  put(p, &a.ok_mask, 2);
  return frame(out, cap, TYPE_CONFIG_ACK, d, 3);
}

inline bool decode_ack(const uint8_t *d, uint8_t len, ConfigAck &a)
{
  if (len != 3) {return false;}
  const uint8_t *p = d;
  get(p, &a.n, 1);
  get(p, &a.ok_mask, 2);
  return true;
}

inline uint16_t encode_cmd(const Cmd &c, uint8_t *out, uint16_t cap)
{
  if (c.n > MAX_JOINTS) {return 0;}
  uint8_t d[5 + 4 * MAX_JOINTS];
  uint8_t *p = d;
  put(p, &c.seq, 1);
  put(p, &c.flags, 1);
  put(p, &c.valid, 2);
  put(p, &c.n, 1);
  for (uint8_t i = 0; i < c.n; ++i) {put(p, &c.target[i], 4);}
  return frame(out, cap, TYPE_CMD, d, static_cast<uint8_t>(p - d));
}

inline bool decode_cmd(const uint8_t *d, uint8_t len, Cmd &c)
{
  if (len < 5) {return false;}
  const uint8_t *p = d;
  get(p, &c.seq, 1);
  get(p, &c.flags, 1);
  get(p, &c.valid, 2);
  get(p, &c.n, 1);
  if (c.n > MAX_JOINTS || len != 5 + 4 * c.n) {return false;}
  for (uint8_t i = 0; i < c.n; ++i) {get(p, &c.target[i], 4);}
  return true;
}

inline uint16_t encode_state(const State &s, uint8_t *out, uint16_t cap)
{
  if (s.n > MAX_JOINTS) {return 0;}
  uint8_t d[49 + 12 * MAX_JOINTS];
  uint8_t *p = d;
  put(p, &s.ms, 4);
  put(p, &s.seq_echo, 1);
  put(p, &s.status, 1);
  put(p, &s.comm_err, 2);
  put(p, s.imu.quat, 16);
  put(p, s.imu.gyro, 12);
  put(p, s.imu.acc, 12);
  put(p, &s.n, 1);
  for (uint8_t i = 0; i < s.n; ++i) {
    put(p, &s.pos[i], 4);
    put(p, &s.vel[i], 4);
    put(p, &s.cur[i], 4);
  }
  const uint16_t len = static_cast<uint16_t>(p - d);
  if (len > 255) {return 0;}
  return frame(out, cap, TYPE_STATE, d, static_cast<uint8_t>(len));
}

inline bool decode_state(const uint8_t *d, uint8_t len, State &s)
{
  if (len < 49) {return false;}
  const uint8_t *p = d;
  get(p, &s.ms, 4);
  get(p, &s.seq_echo, 1);
  get(p, &s.status, 1);
  get(p, &s.comm_err, 2);
  get(p, s.imu.quat, 16);
  get(p, s.imu.gyro, 12);
  get(p, s.imu.acc, 12);
  get(p, &s.n, 1);
  if (s.n > MAX_JOINTS || len != 49 + 12 * s.n) {return false;}
  for (uint8_t i = 0; i < s.n; ++i) {
    get(p, &s.pos[i], 4);
    get(p, &s.vel[i], 4);
    get(p, &s.cur[i], 4);
  }
  return true;
}

inline void put_stat(uint8_t *&p, const TimeStat &t)
{
  put(p, &t.n, 2);
  put(p, &t.fail, 2);
  put(p, &t.mean_us, 2);
  put(p, &t.max_us, 2);
}

inline void get_stat(const uint8_t *&p, TimeStat &t)
{
  get(p, &t.n, 2);
  get(p, &t.fail, 2);
  get(p, &t.mean_us, 2);
  get(p, &t.max_us, 2);
}

const uint8_t DIAG_LEN_V1 = 4 + 2 + 8 + 2 + 8 * 4 + 2 + 1 + 2;   // 53 (첫 판 — 받을 때는 usb_tx · config_ms = 0)
const uint8_t DIAG_LEN = DIAG_LEN_V1 + 8 + 2;                    // 63 (v2: + usb_tx + config_ms, 뒤에 붙임)

inline uint16_t encode_diag(const Diag &g, uint8_t *out, uint16_t cap)
{
  uint8_t d[DIAG_LEN];
  uint8_t *p = d;
  put(p, &g.ms, 4);
  put(p, &g.window_ms, 2);
  put_stat(p, g.loop);
  put(p, &g.resync, 2);
  put_stat(p, g.sync_read);
  put_stat(p, g.sync_write);
  put_stat(p, g.single_read);
  put_stat(p, g.single_write);
  put(p, &g.fault_stop_us, 2);
  put(p, &g.fault_flags, 1);
  put(p, &g.volt_min_dv, 1);
  put(p, &g.volt_mean_dv, 1);
  put_stat(p, g.usb_tx);
  put(p, &g.config_ms, 2);
  return frame(out, cap, TYPE_DIAG, d, DIAG_LEN);
}

inline bool decode_diag(const uint8_t *d, uint8_t len, Diag &g)
{
  if (len != DIAG_LEN && len != DIAG_LEN_V1) {return false;}
  const uint8_t *p = d;
  get(p, &g.ms, 4);
  get(p, &g.window_ms, 2);
  get_stat(p, g.loop);
  get(p, &g.resync, 2);
  get_stat(p, g.sync_read);
  get_stat(p, g.sync_write);
  get_stat(p, g.single_read);
  get_stat(p, g.single_write);
  get(p, &g.fault_stop_us, 2);
  get(p, &g.fault_flags, 1);
  get(p, &g.volt_min_dv, 1);
  get(p, &g.volt_mean_dv, 1);
  g.usb_tx = TimeStat{0, 0, 0, 0};
  g.config_ms = 0;
  if (len == DIAG_LEN) {
    get_stat(p, g.usb_tx);
    get(p, &g.config_ms, 2);
  }
  return true;
}

inline uint16_t encode_temp(const Temp &t, uint8_t *out, uint16_t cap)
{
  if (t.n > MAX_JOINTS) {return 0;}
  uint8_t d[1 + MAX_JOINTS];
  uint8_t *p = d;
  put(p, &t.n, 1);
  put(p, t.c, t.n);
  return frame(out, cap, TYPE_TEMP, d, static_cast<uint8_t>(p - d));
}

inline bool decode_temp(const uint8_t *d, uint8_t len, Temp &t)
{
  if (len < 1) {return false;}
  const uint8_t *p = d;
  get(p, &t.n, 1);
  if (t.n > MAX_JOINTS || len != 1 + t.n) {return false;}
  get(p, t.c, t.n);
  return true;
}

inline uint16_t encode_pos_limits(const PosLimits &l, uint8_t *out, uint16_t cap)
{
  if (l.n > MAX_JOINTS) {return 0;}
  uint8_t d[1 + 8 * MAX_JOINTS];
  uint8_t *p = d;
  put(p, &l.n, 1);
  for (uint8_t i = 0; i < l.n; ++i) {
    put(p, &l.lo[i], 4);
    put(p, &l.hi[i], 4);
  }
  return frame(out, cap, TYPE_POS_LIMITS, d, static_cast<uint8_t>(p - d));
}

inline bool decode_pos_limits(const uint8_t *d, uint8_t len, PosLimits &l)
{
  if (len < 1) {return false;}
  const uint8_t *p = d;
  get(p, &l.n, 1);
  if (l.n > MAX_JOINTS || len != 1 + 8 * l.n) {return false;}
  for (uint8_t i = 0; i < l.n; ++i) {
    get(p, &l.lo[i], 4);
    get(p, &l.hi[i], 4);
  }
  return true;
}

inline uint16_t encode_torque_fail(const TorqueFail &f, uint8_t *out, uint16_t cap)
{
  uint8_t d[17];
  uint8_t *p = d;
  put(p, &f.idx, 1);
  put(p, &f.id, 1);
  put(p, &f.step, 1);
  put(p, &f.dxl_err, 1);
  put(p, &f.lib_err, 1);
  put(p, &f.pos, 4);
  put(p, &f.min_lim, 4);
  put(p, &f.max_lim, 4);
  return frame(out, cap, TYPE_TORQUE_FAIL, d, static_cast<uint8_t>(p - d));
}

inline bool decode_torque_fail(const uint8_t *d, uint8_t len, TorqueFail &f)
{
  if (len != 17) {return false;}
  const uint8_t *p = d;
  get(p, &f.idx, 1);
  get(p, &f.id, 1);
  get(p, &f.step, 1);
  get(p, &f.dxl_err, 1);
  get(p, &f.lib_err, 1);
  get(p, &f.pos, 4);
  get(p, &f.min_lim, 4);
  get(p, &f.max_lim, 4);
  return f.step >= TF_READ && f.step <= TF_BUDGET;
}

// ------------------------------------------------------------------ 받는 쪽: 바이트를 하나씩 넣으면 상자를 찾아 줌
// "헤더를 찾아 잘라 내기". 중간에 깨진 상자는 버리고 다음 헤더를 다시 찾음.
class Parser
{
public:
  // 바이트 하나 넣기. 체크섬까지 맞는 상자가 완성되면 true → type() · data() · len() 으로 꺼냄
  bool feed(uint8_t b)
  {
    switch (step_) {
      case 0:
        if (b == HEADER1) {step_ = 1;}
        return false;
      case 1:
        if (b == HEADER2) {step_ = 2;} else if (b != HEADER1) {step_ = 0;}
        return false;
      case 2:
        type_ = b;
        sum_ = b;
        step_ = 3;
        return false;
      case 3:
        len_ = b;
        sum_ = static_cast<uint8_t>(sum_ + b);
        got_ = 0;
        step_ = len_ ? 4 : 5;
        return false;
      case 4:
        data_[got_++] = b;
        sum_ = static_cast<uint8_t>(sum_ + b);
        if (got_ == len_) {step_ = 5;}
        return false;
      default:   // 5: 체크섬
        step_ = 0;
        if (b == sum_) {return true;}
        ++bad_;
        return false;
    }
  }
  uint8_t type() const {return type_;}
  const uint8_t * data() const {return data_;}
  uint8_t len() const {return len_;}
  uint32_t bad_checksum_count() const {return bad_;}

private:
  uint8_t step_ = 0, type_ = 0, len_ = 0, got_ = 0, sum_ = 0;
  uint8_t data_[255];
  uint32_t bad_ = 0;
};

}  // namespace ntp
