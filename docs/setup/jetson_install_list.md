# Jetson 설치 목록 (로봇 코드 빌드 · 실행용)

- 대상: Jetson Orin Nano (JetPack 7.2.1, Ubuntu 24.04 arm64, ROS 2 Lyrical)
- 개발 PC 에서 확인한 것만 적음. Jetson 에서 설치 · 확인하면 표의 "Jetson" 칸을 채움
- 설치는 직접 실행 (Jetson 에서)

| 패키지 | 쓰는 곳 | 설치 | 개발 PC | Jetson |
|---|---|---|---|---|
| `libzxing-dev` (zxing-cpp 2.2.1) | perception 벽 바코드 Code128 판독 (`barcode_reader.cpp`). 없으면 OpenCV 로 빌드되지만 Code128 못 읽음 | `sudo apt install libzxing-dev` | 설치함 (2026-11-01) | 미확인 |
| TensorRT · CUDA (JetPack) | perception `perception_node` (광각 · 손목 엔진). 없으면 노드가 빌드 안 됨 | JetPack 에 포함 | 없음 (테스트만) | 미확인 (`dpkg -l \| grep -i tensorrt`) |

## 확인 명령 (Jetson)

```bash
apt-cache policy libzxing-dev
```

```bash
sudo apt install libzxing-dev
```

```bash
dpkg -l | grep -i -E 'tensorrt|nvinfer'
```

- `libzxing-dev` 가 apt 에 없으면 zxing-cpp 소스 빌드 (github.com/zxing-cpp/zxing-cpp, CMake, `-DZXING_EXAMPLES=OFF`)
- perception 빌드 로그에 `zxing-cpp 없음` 경고가 나오면 바코드는 OpenCV (EAN/UPC 만) 로 빌드된 것
