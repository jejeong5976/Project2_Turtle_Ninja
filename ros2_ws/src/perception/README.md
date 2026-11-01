# perception

YOLO 탐지(ONNX Runtime C++) + 후처리(퍽 색 · AprilTag · 신호등) + 벽 1D 바코드 판독. 토픽은 planning 요청 기준.

| 토픽 | 타입 | 보내는 곳 | 내용 |
|---|---|---|---|
| `/perception/detections` | `interfaces/DetectionArray` | 광각, 손목 | bbox, 퍽 색(red/green/beige), AprilTag ID, 카메라 기준 pose. `camera` 로 구분 |
| `/perception/task_text` | `interfaces/TaskText` | 바코드 (`barcode.camera`) | 바코드 원문. 여러 프레임 일치 후 한 번만 |
| `/perception/signals` | `interfaces/SignalState` | 광각 | red / orange / green / off / unknown |

노드는 `perception` 하나 (실행 파일 `perception_node`). 카메라마다 콜백 그룹이 따로라 광각 · 손목이 동시에 돌고, 바코드는 `barcode.camera` 이미지 콜백 안에서 같은 프레임의 barcode bbox 로 읽음.

서비스: `/perception/wide/enable`, `/perception/wrist/enable`, `/perception/barcode/enable` (`std_srvs/SetBool`), `/perception/barcode/reset` (`std_srvs/Trigger`)

## 파일

```
perception/
├── CMakeLists.txt, package.xml
├── config/perception.yaml     # 노드 파라미터
├── launch/perception.launch.py
├── src/
│   ├── perception_node.cpp    # 노드 하나: 카메라별 탐지(CameraStage) + 바코드(BarcodeStage)
│   ├── yolo.hpp / yolo.cpp    # ONNX Runtime 추론 (letterbox, 출력 해석, NMS)
│   └── postproc.hpp / .cpp    # 색 · 신호등 · 다수결 · AprilTag · depth · 바코드 · letterbox · NMS
└── test/test_postproc.cpp     # gtest 10개 (카메라 · 모델 없이)
```

## ONNX Runtime 설치 (한 번)

rosdep 키가 없어서 따로 받는다. 기본 위치는 `/opt/onnxruntime` (다른 곳이면 `ONNXRUNTIME_ROOT` 로 지정).

```bash
# PC (x86_64, CUDA 12): GPU 버전. 버전은 https://github.com/microsoft/onnxruntime/releases 에서 확인
V=1.22.0   # 예시. 릴리스 페이지의 최신 gpu 버전으로
wget https://github.com/microsoft/onnxruntime/releases/download/v$V/onnxruntime-linux-x64-gpu-$V.tgz
sudo mkdir -p /opt/onnxruntime && sudo tar -xzf onnxruntime-linux-x64-gpu-$V.tgz -C /opt/onnxruntime --strip-components=1
```

- CUDA provider 는 CUDA 12 · cuDNN 9 라이브러리를 찾을 수 있어야 함 (`LD_LIBRARY_PATH`). 못 찾으면 노드가 경고를 내고 CPU 로 돈다
- Jetson 은 JetPack 에 맞는 onnxruntime-gpu (aarch64) 를 같은 위치에 둔다. `provider:=tensorrt` 로 FP16 TensorRT 엔진을 쓴다

## 빌드 · 실행

```bash
cd ros2_ws
colcon build --packages-select interfaces perception     # onnxruntime 이 없으면 테스트만 빌드됨 (경고)
source install/setup.bash
ros2 launch perception perception.launch.py wide_model:=$HOME/models/wide_fp16.onnx provider:=cuda
```

- 모델: ultralytics 로 내보낸 `.onnx` (FP32 · FP16 둘 다). 입력 크기와 클래스 이름은 모델 안에서 읽음
- `provider`: `cuda` (기본) | `tensorrt` (처음 실행 때 엔진을 만들어 모델 옆에 저장, 수 분 걸림) | `cpu`
- 카메라 드라이버는 따로 띄움: 광각 `/wide_cam/image_raw`·`camera_info`, RealSense `/wrist_cam/color/*`, `/wrist_cam/aligned_depth_to_color/image_raw` (align_depth 켜기)
- 한 프로세스라 CUDA context 가 하나. 두 카메라 모델 파일이 같으면 세션도 하나만 만듦
- 파라미터: `config/perception.yaml` (카메라별 `wide.*` · `wrist.*`, 바코드 `barcode.*`. `[확인 필요]` 값은 실측 후 갱신). 클래스 이름은 6클래스 모델 기준, conf 0.5 (planning 기준)

## 위치(pose) 계산

- 손목: 시각이 가장 가까운(50 ms 이내) depth 의 bbox 중앙 영역 → 퍽 중심 3D 점 (자세는 단위 쿼터니언)
- depth 가 없거나 실패하면: AprilTag 코너로 PnP → 태그 면의 위치·자세 (`tag.size` 필요). 광각 fisheye 는 왜곡을 풀고 계산
- 둘 다 안 되면 `has_pose=false`

## 테스트

```bash
cd ros2_ws && colcon test --packages-select perception && colcon test-result --verbose
```

## 주의

- 이미지 구독은 기본 `reliable` (depth 1). 1280×720 이미지를 프로세스 사이에서 `best_effort` 로 받으면 거의 다 유실됨 (Fast DDS 실측)
- OpenCV 바코드 모듈은 EAN-8/13, UPC-A/E 만 읽음. 벽 바코드가 Code128 등이면 zxing-cpp 로 바꿔야 함
- 바코드가 화면을 너무 크게 채우면 OpenCV 가 못 찾아서 절반 크기로도 다시 시도함
- OpenCV 4.6 (contrib aruco · barcode) 와 4.7+ / 5.x (objdetect) 모두 빌드됨
