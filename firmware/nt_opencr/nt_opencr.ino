// nt_opencr.ino — 닌자터틀 OpenCR 펌웨어: 모터 14개 + IMU
// =====================================================================
// OpenCR 1장 + 펌웨어 (D3 A안 · D12): 모터 14개와 내장 IMU 를 USB 한 줄로
//
//   Jetson (nt_hardware 플러그인) ──USB──▶ [이 펌웨어] ──TTL──▶ 다이나믹셀 14개 (전원은 따로 분배)
//                                 ◀──────          ◀── IMU (보드 안)
//
// 한 바퀴 (100 Hz, 10 ms 마다)
//   ① USB 로 들어온 바이트 → 상자 찾기 (CONFIG / CMD)
//   ② IMU 갱신
//   ③ 경비견: Jetson 명령이 300 ms 끊기면 바퀴 0 (팔 · 조향은 그 자리에서 버팀 — 토크를 끄면 팔이 떨어짐)
//   ④ 목표 쓰기 (Sync Write) → 상태 읽기 (Sync Read)
//   ⑤ 상태 상자 보내기 (시간 · IMU · 모터 14개 위치/속도/전류)
//   ⑥ 1 초마다 보드 안 시간 측정 상자(DIAG) — 한 바퀴 · Sync Read/Write · 개별 읽기/쓰기 · 고장 정지 시간
//
// 안전장치 3겹 (위에서부터)
//   1. swerve_controller (ros2_control 안) : cmd_vel 끊김 · 차례 아님 → 바퀴 0
//   2. 이 펌웨어       : Jetson 명령 300 ms 끊김 → 바퀴 0 (HOST_TIMEOUT_MS)
//   3. 모터 Bus Watchdog: 이 펌웨어가 멈춰 명령이 끊김 → 모터 스스로 정지
//
// 올리는 법: Arduino IDE + OpenCR 보드 패키지, 라이브러리 Dynamixel2Arduino. 보드 = OpenCR, 포트 = /dev/ttyACM0
//   OpenCR 코어 1.5.3 으로 컴파일됨. ⚠ 실제 보드에서 실행은 아직 → 책상 시험 순서는 docs/hardware.md
// 파일: 이 .ino (한 바퀴 + IMU) · motor_bus.h/.cpp (모터 14개 Sync 읽기/쓰기) · nt_packet.h (편지 규칙, 플러그인과 공용)

#include <IMU.h>

#include "motor_bus.h"
#include "nt_packet.h"

// 파일 안에서만 쓰는 값 · 함수는 static (Arduino IDE 가 .ino 에 함수 원형을 자동으로 끼워 넣는데,
// namespace { } 안에 끼워 넣으면 꼬일 수 있어서 static 으로)
static const uint32_t PC_BAUD = 1000000;          // USB 가상 시리얼이라 실제 속도와는 상관없음
static const uint32_t DXL_BAUD = 1000000;         // 다이나믹셀 버스 (hardware_map.yaml baud 와 같게)
static const uint32_t PERIOD_US = 10000;          // 100 Hz
static const uint32_t HOST_TIMEOUT_MS = 300;      // Jetson 명령이 이만큼 끊기면 바퀴 0 (경비견)

static ntp::Parser parser;
static ntp::Cmd cmd;
static ntp::State state;
static uint8_t tx[ntp::MAX_FRAME];
static bool configured = false;
static bool have_cmd = false;
static bool imu_ok = false;
static uint32_t last_cmd_ms = 0;
static uint32_t last_tick_us = 0;
// 보드 안 시간 측정 (DIAG, 2026-10-16): 한 바퀴 실행 시간 · 주기 초과 · 다시 잡기 → 1 초마다 TYPE_DIAG 로
static const uint32_t DIAG_PERIOD_MS = 1000;
static TimeAcc t_loop;
static TimeAcc t_usb;                 // USB 로 상태 · DIAG 보내기 1 회 (fail = 버퍼가 차서 버림)
static uint16_t resync_count = 0;
static uint32_t diag_since_ms = 0;
static uint16_t config_ms_pending = 0;   // 다음 DIAG 에 실을 CONFIG 처리 시간 [ms]

// ------------------------------------------------------------------ USB 보내기 (기다리지 않음, 2026-10-16)
// 왜: OpenCR 코어의 Serial.write(vcp_write)는 송신 버퍼(2 KB)가 모자라면 **최대 100 ms 바쁘게 기다림**.
//     PC 가 포트를 한 번 열었다 닫으면(측정 도구를 끝낸 뒤 등) 펌웨어는 계속 쌓다가 버퍼가 차고, 그때부터 상자마다
//     100 ms 씩 제어 루프가 멈춤 → 첫 DIAG 구간에 "10 ms 초과 3~4 회 · 최대 65535(포화)" 로 보였음.
//     코어의 CDC_Itf_Write 는 자리가 있으면 상자 전체를 쓰고, 없으면 0 · 포트가 안 열렸으면 -1 을 바로 돌려줌 (부분 쓰기 없음)
//     → 자리가 없으면 이번 상자는 버리고 셈. 아무도 안 읽는 상태 상자를 기다리느라 모터 제어를 멈추지 않으려고
extern "C" int32_t CDC_Itf_Write(uint8_t *p_buf, uint32_t length);   // variants/OpenCR/hw/usb_cdc/usbd_cdc_interface.h

static bool usb_send(uint8_t *buf, uint16_t len)
{
  const uint32_t t0 = micros();
  const bool ok = CDC_Itf_Write(buf, len) == static_cast<int32_t>(len);
  t_usb.add(micros() - t0, ok);
  return ok;
}

// ------------------------------------------------------------------ OpenCR 내장 IMU (D12)
// 읽는 법 · 단위 변환은 ROBOTIS 터틀봇3 펌웨어(OpenCR libraries/turtlebot3/turtlebot3_sensor.cpp)와 똑같이:
//   자이로  rad/s = gyroADC × 0.0010642        (±2000 °/s 범위, 16.4 눈금 = 1 °/s)
//   가속도  m/s²  = accADC  × 0.000598550415   (±2 g 범위, 16384 눈금 = 1 g)
//   자세    quat[0..3] = w, x, y, z            (보드 안 Madgwick 필터 결과)
// 켤 때 자이로 영점 잡기(gyro_cali)를 함 → **켤 때 로봇을 움직이지 말 것** (swerve_controller 도 2 s 더 잡음)
static const float GYRO_FACTOR = 0.0010642f;
static const float ACCEL_FACTOR = 0.000598550415f;
static cIMU imu;

static bool imu_begin()
{
  if (imu.begin() != 0) {return false;}   // 0 = 정상 (터틀봇3 펌웨어와 같은 판정)
  imu.SEN.gyro_cali_start();
  const uint32_t t0 = millis();
  while (!imu.SEN.gyro_cali_get_done() && millis() - t0 < 5000) {imu.update();}
  return true;
}

static void imu_fill(ntp::Imu &out)
{
  for (int i = 0; i < 4; ++i) {out.quat[i] = imu.quat[i];}
  for (int i = 0; i < 3; ++i) {
    out.gyro[i] = imu.SEN.gyroADC[i] * GYRO_FACTOR;
    out.acc[i] = imu.SEN.accADC[i] * ACCEL_FACTOR;
  }
}

// ------------------------------------------------------------------ 패킷 처리
// ------------------------------------------------------------------ 보드 안 시간 (DIAG) 보내기 · 새 구간
static void send_diag(uint8_t extra_flags)
{
  ntp::Diag g;
  memset(&g, 0, sizeof(g));
  g.ms = millis();
  g.window_ms = static_cast<uint16_t>(g.ms - diag_since_ms);
  t_loop.take(g.loop);
  g.resync = resync_count;
  resync_count = 0;
  bus_timing_take(g);
  g.fault_flags |= extra_flags;
  t_usb.take(g.usb_tx);
  g.config_ms = config_ms_pending;
  config_ms_pending = 0;
  diag_since_ms = g.ms;
  const uint16_t dlen = ntp::encode_diag(g, tx, sizeof(tx));
  if (dlen) {usb_send(tx, dlen);}   // 이 보내기는 다음 구간의 usb_tx 에 들어감
}

// CONFIG 직후: 모은 숫자를 버리고 새 구간 시작 (CONFIG 전 구간은 바로 앞에서 send_diag 로 따로 보냄)
static void diag_restart()
{
  ntp::Diag g;
  t_loop.take(g.loop);
  t_usb.take(g.usb_tx);
  bus_timing_take(g);
  resync_count = 0;
  diag_since_ms = millis();
}

// ------------------------------------------------------------------ 패킷 처리
static void handle_packet()
{
  if (parser.type() == ntp::TYPE_CONFIG) {
    ntp::Config c;
    if (!ntp::decode_config(parser.data(), parser.len(), c)) {return;}
    // CONFIG 전 구간을 여기서 끊어 따로 보냄 (DIAG_FLAG_BEFORE_CONFIG) — PC 가 포트를 열기 전 · 설정 전 숫자를
    // 설정 뒤 측정과 섞지 않으려고. 숨기지 않고 표시를 달아 보냄
    send_diag(ntp::DIAG_FLAG_BEFORE_CONFIG);
    ntp::ConfigAck ack;
    ack.n = c.n;
    const uint32_t t_cfg = millis();
    ack.ok_mask = bus_configure(c);   // 몇 초 걸릴 수 있음 (모터마다 ping · 설정)
    configured = true;
    have_cmd = false;
    const uint16_t len = ntp::encode_ack(ack, tx, sizeof(tx));
    Serial.write(tx, len);   // ACK 는 PC 가 방금 CONFIG 를 보낸 뒤라 읽는 중 — 꼭 가야 해서 코어 쓰기(최대 100 ms 대기) 그대로
    // CONFIG 뒤 새 구간. bus_configure 시간은 어느 구간의 한 바퀴에도 안 넣고 config_ms 로 따로 알림
    const uint32_t cfg_ms = millis() - t_cfg;
    config_ms_pending = static_cast<uint16_t>(cfg_ms > 65535 ? 65535 : cfg_ms);
    diag_restart();
    last_tick_us = micros();   // CONFIG 로 밀린 시간을 "주기 다시 잡기" 로 세지 않음 (CONFIG 때문인 게 분명하니까)
  } else if (parser.type() == ntp::TYPE_CMD) {
    ntp::Cmd c;
    if (!ntp::decode_cmd(parser.data(), parser.len(), c)) {return;}
    cmd = c;
    have_cmd = true;
    last_cmd_ms = millis();   // 경비견 밥 주기
  }
}

void setup()
{
  Serial.begin(PC_BAUD);
  bus_begin(DXL_BAUD);
  imu_ok = imu_begin();   // 켤 때 로봇을 움직이지 말 것 (자이로 영점)
  memset(&state, 0, sizeof(state));
  last_tick_us = micros();
}

void loop()
{
  // ① 들어온 바이트 처리
  while (Serial.available() > 0) {
    if (parser.feed(static_cast<uint8_t>(Serial.read()))) {handle_packet();}
  }
  // ② IMU 는 자주 갱신 (안에서 주기 맞춤)
  if (imu_ok) {imu.update();}

  const uint32_t now_us = micros();
  if (now_us - last_tick_us < PERIOD_US) {return;}
  last_tick_us += PERIOD_US;
  if (now_us - last_tick_us > PERIOD_US) {   // 많이 밀렸으면 따라잡지 말고 다시 시작
    last_tick_us = now_us;
    if (resync_count < 65535) {++resync_count;}
  }

  // ③ 경비견
  const bool host_ok = have_cmd && (millis() - last_cmd_ms < HOST_TIMEOUT_MS);

  // ④ 모터
  state.comm_err = 0;
  bool write_ok = true;
  if (configured) {
    // 토크는 Jetson 이 켜라고 할 때만. 끊겨도 끄지는 않음 (팔이 떨어지지 않게) — 바퀴만 0
    if (have_cmd) {bus_set_torque((cmd.flags & ntp::FLAG_TORQUE) != 0);}
    write_ok = bus_write_targets(cmd, host_ok);
    state.comm_err = bus_read_states(state);
    bus_health_step();   // 1 모터씩 돌아가며 Hardware Error · Torque Enable 되읽기 (R5)
    // 고장 잠금(쓰기 실패 → 바퀴 정지) 중에는 bus_write_targets 가 계속 false → ST_WRITE_FAIL 도 계속 켜짐
  }

  // ⑤ 상태 보내기
  state.ms = millis();
  state.seq_echo = cmd.seq;
  state.status = 0;
  if (configured) {state.status |= ntp::ST_CONFIGURED;}
  if (configured && bus_torque_is_on()) {state.status |= ntp::ST_TORQUE;}   // 명령이 아니라 실제 결과 (되읽기 포함)
  if (configured && !write_ok) {state.status |= ntp::ST_WRITE_FAIL;}
  if (configured && bus_hw_error_mask()) {state.status |= ntp::ST_HW_ERROR;}
  if (configured && bus_fault_latched()) {state.status |= ntp::ST_FAULT_LATCHED;}   // CONFIG 를 다시 받을 때까지 유지
  if (have_cmd && !host_ok) {state.status |= ntp::ST_HOST_TIMEOUT;}
  if (imu_ok) {
    state.status |= ntp::ST_IMU_OK;
    imu_fill(state.imu);
  }
  const uint16_t len = ntp::encode_state(state, tx, sizeof(tx));
  if (len) {usb_send(tx, len);}   // 기다리지 않음 — PC 가 안 읽어 버퍼가 차면 이번 상자는 버림 (usb_tx.fail)

  // ⑥ 보드 안 시간 (DIAG). 한 바퀴 = ③~⑤ (모터 쓰기 · 읽기 · 되읽기 + 상태 보내기). 10 ms 넘으면 실패로 셈
  const uint32_t exec_us = micros() - now_us;
  t_loop.add(exec_us, exec_us <= PERIOD_US);
  if (millis() - diag_since_ms >= DIAG_PERIOD_MS) {send_diag(0);}
}
