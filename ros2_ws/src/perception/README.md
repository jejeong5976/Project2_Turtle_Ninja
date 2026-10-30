# perception

YOLO 탐지 + 후처리(퍽 색 · AprilTag · 신호등) + 벽 1D 바코드 판독. 토픽은 planning 요청 기준.

| 토픽 | 타입 | 보내는 노드 | 내용 |
|---|---|---|---|
| `/perception/detections` | `interfaces/DetectionArray` | `wide_detector`, `wrist_detector` | bbox, 퍽 색(red/green/beige), AprilTag ID, 카메라 기준 pose. `camera` 로 구분 |
| `/perception/task_text` | `interfaces/TaskText` | `barcode` | 바코드 원문. 여러 프레임 일치 후 한 번만 |
| `/perception/signals` | `interfaces/SignalState` | `wide_detector` | red / orange / green / off / unknown |

서비스: `/wrist_detector/enable`, `/wide_detector/enable`, `/barcode/enable` (`std_srvs/SetBool`), `/barcode/reset` (`std_srvs/Trigger`)

## 실행

```bash
pip install ultralytics          # Jetson 은 JetPack 에 맞는 torch 를 먼저
ros2 launch perception perception.launch.py \
    wide_model:=$HOME/models/wide_736x1280.engine wrist_model:=$HOME/models/wrist_480x640.engine
```

- 카메라 드라이버는 따로 띄움: 광각 `/wide_cam/image_raw`·`camera_info`, RealSense `/wrist_cam/color/*`, `/wrist_cam/aligned_depth_to_color/image_raw` (align_depth 켜기)
- 두 탐지 노드는 한 프로세스(`detectors`)에서 돌아서 CUDA context 가 하나
- 개발 PC 에서는 `.pt` 와 `device:=cpu` 로도 실행됨
- 파라미터: `config/perception.yaml` (`[확인 필요]` 값은 실측 후 갱신)

## 위치(pose) 계산

- 손목: 정렬된 depth 의 bbox 중앙 영역 → 퍽 중심 3D 점 (자세는 단위 쿼터니언)
- depth 가 없거나 실패하면: AprilTag 코너로 PnP → 태그 면의 위치·자세 (`tag.size` 필요). 광각 fisheye 는 왜곡을 풀고 계산
- 둘 다 안 되면 `has_pose=false`

## 테스트

```bash
cd ros2_ws && PYTHONPATH=src/perception python3 -m pytest src/perception/test
```

## 주의

- 이미지 구독은 기본 `reliable` (depth 1). 1280×720 이미지를 프로세스 사이에서 `best_effort` 로 받으면 거의 다 유실됨 (Fast DDS 실측)
- OpenCV 바코드 모듈은 EAN-8/13, UPC-A/E 만 읽음. 벽 바코드가 Code128 등이면 zxing-cpp 로 바꿔야 함
- 바코드가 화면을 너무 크게 채우면 OpenCV 가 못 찾아서 절반 크기로도 다시 시도함
