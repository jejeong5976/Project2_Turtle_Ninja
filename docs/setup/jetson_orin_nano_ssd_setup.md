# Jetson Orin Nano SSD 설치 및 JetPack 플래시 기록

중고 NVMe SSD를 Jetson Orin Nano에 장착하고, SDK Manager로 JetPack 7.2.1을 SSD에 직접 플래시한 뒤 상태를 점검한 기록.

- 작업일: 2026-10-16
- 호스트 PC: Ubuntu 24.04.5 (x86_64), SDK Manager 26.1.0
- 결과: SSD 부팅 성공, 구성 요소 30개 전부 설치, SSD 상태 정상 (거의 미사용)

---

## 0. 요약

| 항목 | 내용 |
|---|---|
| 보드 | Jetson Orin Nano 8GB Developer Kit (USB ID `0955:7523`) |
| SSD | Samsung PM991a 256GB (`MZVLQ256HBJD-00B`), M.2 2280 NVMe PCIe Gen3 x4, DRAM-less |
| 부팅 방식 | SD카드 없이 **SSD(NVMe) 부팅** |
| JetPack | 7.2.1 (Jetson Linux R39.2.1), Ubuntu 24.04.4, 커널 6.8.12-tegra |
| 주요 SDK | CUDA 13.2, TensorRT 10.16.2, DeepStream 9.1, Holoscan 4.6, OpenCV 4.8, VPI 4.1 |
| 계정 / 호스트명 | 플래시 때 Pre-Config로 설정 (이 문서에서는 `<user>`, `<hostname>`으로 표기) |
| 접속 | USB-C 연결 시 `ssh <user>@192.168.55.1` |

---

## 1. SSD 장착 전 확인

### 규격

Orin Nano 개발자 키트 뒷면에는 M.2 Key M 슬롯이 2개 있다.

| 슬롯 | 규격 | 사용 |
|---|---|---|
| 2280 | PCIe Gen3 x4 | **여기에 장착** |
| 2230 | PCIe Gen3 x2 | 사용 안 함 |

- **NVMe(PCIe)만 인식**된다. 모양이 같은 M.2 SATA SSD는 인식되지 않는다.
- 홈이 하나(M-key)면 대부분 NVMe, 둘(B+M key)이면 대부분 SATA.

### PM991a 특징

- 삼성 OEM 제품 (레노버·HP·델 노트북 기본 장착품). Samsung Magician 펌웨어 업데이트는 지원하지 않는다.
- DRAM-less(HMB) 방식. 리눅스 nvme 드라이버에서 정상 동작한다.
- 방열판 없이 오래 쓰면 쓰로틀링이 생길 수 있다 (→ 6장 온도).

---

## 2. 작업 전 상태와 문제

전날(10/15) 상태를 SDK Manager 로그(`~/.nvsdkm/`)로 확인했다.

| 확인 | 결과 |
|---|---|
| SDK Manager | JetPack 7.2.1 / Orin Nano 선택, 파일 다운로드만 완료 (`~/Downloads/nvidia/sdkm_downloads`, 약 21GB) |
| 설치·플래시 | 모든 항목 `Failed - start: none` → **시작되지 않음** (보드 미연결) |
| SD카드 | JetPack이 아니라 **터틀봇3 라즈베리파이용 Ubuntu 카드**였음. Jetson에 사용하지 않음 |

→ SD카드를 거치지 않고 **SDK Manager로 SSD에 바로 플래시**하기로 결정.

- SSD가 SD카드보다 빠르고 안정적이다.
- 플래시하면서 **보드 펌웨어(QSPI)도 함께 업데이트**된다. 초기 출고 보드의 구버전 펌웨어 때문에 JetPack 6 이상이 부팅되지 않는 문제를 피할 수 있다.

---

## 3. 복구 모드(Recovery Mode) 진입

SSD가 비어 있어 부팅할 OS가 없으므로, Jetson을 "USB로 명령을 기다리는 모드"로 켜서 PC가 SSD에 직접 기록하게 한다.

순서가 중요하다. Jetson은 **전원이 들어오는 순간에만** FC REC 핀을 확인한다.

1. 전원 분리
2. SSD를 뒷면 2280 슬롯에 장착
3. 모듈 아래 버튼 헤더(J14)의 **`FC REC` ↔ `GND` 쇼트** (점퍼). 보드에 인쇄된 글씨로 확인
4. Jetson USB-C ↔ PC 연결 (데이터 케이블, 허브 없이 직접)
5. 전원 어댑터(배럴잭) 연결 → 복구 모드 진입 (모니터 출력 없음이 정상)

PC에서 인식 확인:

```bash
lsusb | grep -i nvidia
```

| 출력 | 의미 |
|---|---|
| `0955:7523 NVIDIA Corp. APX` | 복구 모드 (Orin Nano 8GB) |
| `0955:7020 NVIDIA Corp. L4T (Linux for Tegra) running on Tegra` | 정상 부팅됨 |

---

## 4. SDK Manager 설정

```bash
sdkmanager
```

### STEP 01

| 항목 | 선택 |
|---|---|
| Product | Jetson |
| Host Machine | 체크 (CUDA·크로스 컴파일·Nsight를 PC에 설치. 플래시에는 필요 없는 선택 사항) |
| Target Hardware | Jetson Orin Nano [8GB developer kit version] (자동 감지) |
| SDK Version | JetPack 7.2.1, **Direct Flash** |
| Additional SDKs | DeepStream 9.1, Holoscan 4.6 체크 |

- **Video Codec SDK는 Orin Nano에서 선택 불가.** 데스크톱 GPU용 API이고, Jetson은 Jetson Linux에 포함된 멀티미디어 API와 GStreamer(`nvv4l2decoder` 등)를 쓴다.
- Orin Nano에는 **하드웨어 영상 인코더(NVENC)가 없다.** 디코딩은 하드웨어로 되지만, 인코딩은 CPU 소프트웨어 인코딩이다.
- 256GB SSD면 전부 설치해도 용량이 충분하다 (설치 후 31GB 사용).

### STEP 02

- 하단 **"Download now. Install later." 체크 해제** 확인. 켜져 있으면 다운로드만 하고 끝난다.
- PC sudo 비밀번호 입력

### STEP 03: 플래시 팝업

| 항목 | 선택 |
|---|---|
| OEM Configuration | **Pre-Config** (사용자 이름과 비밀번호를 미리 설정하므로 모니터 불필요) |
| New Username | `<user>` |
| **Storage Device** | **NVMe** (기본값 SD Card에서 변경) |

- NVMe 플래시는 USB 네트워크로 NFS를 사용한다. PC 방화벽(ufw)이 켜져 있으면 실패할 수 있으므로 `sudo ufw status`로 확인하고, 필요하면 플래시하는 동안만 끈다.
- Jetson에 SD카드가 꽂혀 있지 않으면 부팅 순서는 신경 쓰지 않아도 된다.

### 구성 요소 설치 팝업 (플래시 후)

| 항목 | 값 |
|---|---|
| Connection | USB |
| IP | 192.168.55.1 |
| Username | `<user>` |
| Target Proxy | Do not set proxy (PC가 프록시를 쓰지 않음) |

### 결과

| 단계 | 소요 시간 |
|---|---|
| OS 이미지 생성 | 약 2분 |
| 플래시 (QSPI + NVMe) | 9분 33초 |
| 전체 (구성 요소 설치 포함) | 1시간 17분 (10:28 ~ 11:45) |

- 로그: `Successfully flashed the QSPI`, `Flashing success`, 대상 `external-device nvme0n1p1`
- `Total 30 components: 30 installed, 0 failed`

---

## 5. 설치 후 설정

### SSH 키 등록 (PC에서)

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub <user>@192.168.55.1
```

### 호스트명 변경 (Jetson에서)

기본 호스트명이 `localhost`라서 변경했다. 여러 장비를 함께 쓸 때 구분되는 이름으로 정한다.

```bash
sudo hostnamectl set-hostname <hostname>
echo "127.0.1.1 <hostname>" | sudo tee -a /etc/hosts
```

`/etc/hosts`에 등록하지 않으면 `sudo` 실행 때마다 "unable to resolve host" 경고가 뜬다.

---

## 6. 점검 결과

### 부팅과 연결

| 항목 | 결과 | 판단 |
|---|---|---|
| 루트 파티션 | `/dev/nvme0n1p1` (233G, 사용 31G, 여유 190G) | SSD 부팅 ✅ |
| PCIe 링크 | 8.0 GT/s x4 | 최대 사양 ✅ |
| 펌웨어 | FXM7AK1Q | |

### SMART

| 항목 | 값 | 기준 | 판단 |
|---|---|---|---|
| critical_warning | 0 | 0 | ✅ |
| media_errors / 에러 로그 | 0 / 0 | 0 | ✅ |
| available_spare | 100% | threshold(10%)보다 충분히 높게 | ✅ |
| percentage_used | 0% | 낮을수록 좋음 | ✅ |
| Data Units Written | 90.68 GB | 256GB급 수명은 대략 150TBW | 대부분 당일 플래시·설치분 |
| power_cycles | 11 | | 당일 재부팅 포함 |
| power_on_hours | 0 | | 당일 1시간 이상 켜져 있었는데도 0. 이 제품은 신뢰도가 낮음 |
| unsafe_shutdowns | 7 | | 참고용 |

→ **거의 미사용 제품**. 노트북에 잠깐 장착됐거나 공장 테스트만 거친 수준.

### 속도 (dd, direct I/O)

| 테스트 | 1차 | 2차 |
|---|---|---|
| 순차 읽기 (원시 디스크, 4GB) | 1.5 GB/s | 1.4 GB/s |
| 순차 읽기 (파일, 2GB) | 2.4 GB/s | 2.5 GB/s |
| 순차 쓰기 (파일, 2GB) | 1.1 GB/s | 1.0 GB/s |

256GB DRAM-less 제품 사양 범위로 정상.

### 온도

| 상태 | 온도 |
|---|---|
| 대기 | 39~40°C |
| 속도 테스트 직후 | 57°C |
| 경고 / 위험 임계값 | 83°C / 85°C |

쓰로틀링 기록(Thermal Management)은 0회. 짧은 테스트에도 17°C가 오르므로, rosbag 녹화처럼 오래 쓰는 작업 전에는 방열 패드나 방열판을 붙이는 것을 권장한다.

### 소프트웨어

| 항목 | 값 |
|---|---|
| Jetson Linux | R39.2.1 |
| CUDA | 13.2 (nvcc V13.2.86), 드라이버 595.78, GPU `Orin (nvgpu)` |
| TensorRT | 10.16.2 |
| DeepStream | 9.1 |
| 전원 모드 | 25W (nvpmodel 1번) |

---

## 7. 점검 명령어

### PC에서

```bash
lsusb | grep -i nvidia            # 연결 상태 (APX = 복구 모드)
ping -c 3 192.168.55.1            # USB 네트워크
ssh <user>@192.168.55.1           # 접속
```

### Jetson에서: SSD

```bash
lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS,MODEL     # nvme0n1p1이 / 인지
df -h /
cat /sys/class/nvme/nvme0/model /sys/class/nvme/nvme0/firmware_rev
cat /sys/class/nvme/nvme0/device/current_link_speed /sys/class/nvme/nvme0/device/current_link_width

sudo nvme smart-log /dev/nvme0n1                     # SMART (nvme-cli는 기본 설치됨)
sudo nvme error-log /dev/nvme0n1 -e 8 | grep error_count
```

### Jetson에서: 속도와 온도

```bash
sudo dd if=/dev/nvme0n1 of=/dev/null bs=4M count=1024 iflag=direct      # 읽기 (안전)
dd if=/dev/zero of=~/ddtest.bin bs=4M count=512 oflag=direct conv=fsync # 쓰기 (파일에)
dd if=~/ddtest.bin of=/dev/null bs=4M iflag=direct                      # 파일 읽기
rm ~/ddtest.bin
sudo nvme smart-log /dev/nvme0n1 | grep -i temperature                  # 테스트 직후 온도
```

> ⚠️ dd의 출력(`of=`)을 `/dev/nvme0n1` 같은 디스크 장치로 지정하면 SSD 전체가 지워진다. 쓰기 테스트는 반드시 파일에 한다.

### Jetson에서: JetPack

```bash
cat /etc/nv_tegra_release
grep PRETTY /etc/os-release && uname -r
/usr/local/cuda/bin/nvcc --version
nvidia-smi
dpkg -l | grep -E "tensorrt |deepstream"
nvpmodel -q
```

`nvcc`를 경로 없이 쓰려면 `~/.bashrc`에 `export PATH=/usr/local/cuda/bin:$PATH`를 추가한다.

---

## 8. 남은 작업

- [ ] PC 방화벽을 껐다면 다시 켜기: `sudo ufw enable`
- [ ] 다음 전원 차단 시 **FC REC–GND 점퍼 제거** (꽂아 두면 계속 복구 모드로 켜짐)
- [ ] SSD 방열 패드 또는 방열판 장착
- [ ] 인터넷 연결 후 `sudo apt update && sudo apt upgrade` (접속 시 업데이트 374개 대기)
