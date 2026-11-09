# perception

노드 하나(`perception`, 실행 파일 `perception_node`)가 광각 · 손목 · 신호등 · 바코드를 모두 맡음. 토픽 · 메시지는 planning 합의안(`docs/planning/planning_interfaces.md`) 기준. 추론은 TensorRT 만.

| 파이프라인 | 받는 것 | 보내는 것 |
|---|---|---|
| 광각 `WidePipeline` | 광각 컬러 (Arducam UC-684) | `/perception/wide/detections` (퍽 상자 · 색 · 모양 · 퍽 태그 ID, 위치 NaN), `/perception/traffic_light`, `/perception/wide_debug_image` (선택) |
| 손목 `WristPipeline` | D435 컬러 + 정렬 깊이 + camera_info, TF `odom ← d435_color_optical_frame` | `/perception/wrist/detections` (퍽 중심 · 자세 · 선/누움 · ID), `/perception/wrist_debug_image` (선택) |
| 손목 적재함 (`WristPipeline` 안) | D435 컬러 + camera_info, TF `base_link ← d435_color_optical_frame` | `/perception/rack/cells` (칸 바닥 중심 · yaw), `/perception/rack_debug_image` (선택) |
| 바코드 `BarcodeStage` | `barcode.camera` (wide / wrist) 카메라 콜백 안에서, 켜져 있을 때만 | `/perception/task_text` |

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/perception/wide/detections` · `/perception/wrist/detections` | `interfaces/DetectionArray` | 퍽만. 카메라마다 토픽 따로. 처리한 영상마다, 못 찾으면 빈 배열 (= 그 카메라에서 못 찾음). 광각은 위치 NaN, planning 은 손목만 받음 |
| `/perception/rack/cells` | `interfaces/DetectionArray` | 적재함 칸. 컵 바닥 AprilTag ID 20–31 (36h11, 검은 테두리 26 mm), 칸 ↔ ID 는 `rack.cell_tag_ids` 표 (실물은 위치 순서로 붙여 S 번호 순서와 다름). `frame_id` = `base_link`, `kind` = `rack_cell`, `label` = 칸 이름, `id` = 태그 ID, pose = 칸 바닥 중심 + yaw (x = 로봇 앞, xacro `rack_cell_<칸>_link` 와 같은 축). 카메라가 멈춘 장만 셈, 칸마다 5 장 중 3 장이면 중앙값. `~/rack/enable` 로 켜고 끔 |
| `/perception/traffic_light` | `interfaces/TrafficLight` | RED / YELLOW / GREEN / OFF / UNKNOWN + `visible`. 이번 영상에서 못 찾으면 UNKNOWN + `visible=false` |
| `/perception/task_text` | `interfaces/TaskText` | 바코드 원문 `raw_text`. **기본 꺼짐**, planning 이 READ_BARCODE 단계에서 `~/barcode/enable` 로 켜고 나가면 끔. 켜져 있는 동안 최근 5 장 중 3 장 같고 이번 장도 그 값이면 보냄. 못 읽은 장도 다수결에 들어가 안 보이면 확정이 풀림 |

- `header.stamp` = 촬영 시각 그대로, `header.frame_id` = 배포판 URDF 이름 (광각 `birdeye_camera_optical_frame`, 손목 `d435_color_optical_frame`)
- 퍽 ID = AprilTag 0–11 만. 장소 태그(12–14)는 퍽 ID 로 안 붙임 (`common.hpp` `is_puck_tag`)
- 손목은 TF 를 못 구하면 발행 안 함 (위쪽 없이 내면 긴축이 틀림 → planning 이 검출 끊김으로 앎)

## 구조

파일은 평평하게 두고 접두어로 구분함 (`wide_` = 광각, `wrist_` = 손목, 없음 = 공용). 공개 헤더는 `include/perception/`, 노드 내부 헤더는 `src/`.

```
perception/
├── CMakeLists.txt, package.xml, README.md
├── launch/perception.launch.py     # 설정: params 인자 → 레포 config/perception.yaml (bringup 설치본) → 코드 기본값
├── include/perception/
│   ├── trt_engine.hpp              # TensorRT 엔진 실행 + .names.txt (광각 · 손목 공용, trt = TensorRT)
│   ├── common.hpp                  # 공용: 다수결 Voter (신호등 · 바코드) · 퍽 태그 ID 0–11 약속 · AprilTag 검출 (광각 · 손목 같은 설정)
│   ├── traffic_light.hpp           # 신호등 칸 밝기 판정
│   ├── wide_detector.hpp           # 광각: letterbox · 출력 해석 · NMS + 실행 (BoxDetector)
│   ├── wide_tag_reader.hpp         # 광각: 퍽 상자 안 태그 ID
│   ├── wrist_seg.hpp               # 손목: letterbox · 출력 해석 · 마스크 + 실행 (SegModel)
│   ├── wrist_pose.hpp              # 손목: 마스크 + 깊이 → 퍽 자세
│   ├── wrist_tracker.hpp           # 손목: 여러 장 확정
│   └── wrist_rack.hpp              # 손목: 적재함 컵 바닥 태그 → 칸 중심 · yaw + 다수결
├── src/
│   ├── perception_node.cpp         # 노드 조립 · 서비스 · main (MultiThreadedExecutor 3 스레드)
│   ├── wide_pipeline.{hpp,cpp}     # 광각 처리 (퍽 · 신호등)
│   ├── wrist_pipeline.{hpp,cpp}    # 손목 처리
│   ├── barcode.{hpp,cpp}           # 바코드 판독기 (zxing) + 단계: 다수결 · 발행 (카메라 콜백 안에서)
│   ├── msg_utils.hpp               # planning 메시지 규칙 (NaN · 0 쿼터니언 · ID · 신호등 상수)
│   └── (include 헤더와 같은 이름의 .cpp)
└── test/  test_wide · test_barcode · test_common · test_wrist_pose · test_wrist_seg · test_wrist_rack
```

| 대상 | 내용 | 빌드 조건 |
|---|---|---|
| `perception_core` | ROS 없음: `common` · `wide_detector` · `wide_tag_reader` · `traffic_light` · `wrist_pose` · `wrist_tracker` · `wrist_seg` · `wrist_rack` | 항상 (개발 PC 테스트) |
| ↳ TensorRT 부분 | `trt_engine.cpp` 추가 + `PERCEPTION_HAVE_TRT` → `BoxDetector` · `SegModel` 구현이 켜짐 | TensorRT 있을 때 (Jetson) |
| `perception_barcode` | `barcode` (판독기 + 단계, ROS · zxing 씀) | 항상 (개발 PC 테스트) |
| `perception_node` | 노드 · 파이프라인 3개 | TensorRT 있을 때 |

광각 · 손목 모델은 같은 구조: 전처리 · 후처리 함수 (TensorRT 없이 테스트) + 모델 클래스 (`TrtEngine` 을 직접 가짐, `#ifdef PERCEPTION_HAVE_TRT`)

| | 광각 `wide_detector` | 손목 `wrist_seg` |
|---|---|---|
| 모델 클래스 | `BoxDetector(engine)` → `detect(bgr, conf, iou)` | `SegModel(engine, conf, iou)` → `(bgr)` |
| 전처리 · 후처리 | `letterbox` · `to_chw` · `decode` · `nms` | `letterbox` · `shape_outputs` · `postprocess` |
| 엔진 · 이름 | `TrtEngine` (확장자 검사 · `.names.txt` · 첫 추론 미리) | 같음 |

| 손목 파일 | 내용 | Python 원본 |
|---|---|---|
| `wrist_pose` | 역투영, 튀는 점 제거, 중력 분해, 윗면 높이 · 평면, 선/누움, 발자국 PCA, 면 방향 (선 = 윗면 사각형, 누움 = 위쪽), AprilTag(ID) | `wrist_pose.py` |
| `wrist_tracker` | 거르기(truncated · points · size), 5 장 중 3 장 확정, 다수결 · 중앙값 · 4θ 평균, 쿼터니언, `planar_pose` | `wrist_detector.py` |
| `wrist_seg` | letterbox, 후처리(one2many = NMS / one2one = 점수 순), 마스크 원본 크기 | ultralytics `predict(retina_masks=True)` |

- 광각 letterbox(`wide_detector`)와 손목 letterbox(`wrist_seg`)는 반올림 방식이 달라 따로 둠. 손목 쪽은 ultralytics 결과와 맞춰 검증한 판이라 합치려면 같은 결과인지 먼저 확인해야 함

## 스레드

| 콜백 그룹 | 하는 일 |
|---|---|
| 광각 | 광각 영상 → 상자 검출 · 퍽 태그 · 신호등 → 발행. `barcode.camera=wide` 면 바코드 판독도 |
| 손목 | 컬러 · 깊이 동기 (message_filters 입력 구독도 이 그룹) → 분할 · 기하 · 확정 → 발행. `barcode.camera=wrist` 면 동기 전 컬러로 바코드 판독도 |
| 관리 | enable · reset 서비스 (플래그만 바꿈, 실제 초기화는 각 파이프라인이 다음 콜백에서) |

- `MultiThreadedExecutor` 3 스레드 (Jetson 실측 후 조정)
- TensorRT 엔진은 파이프라인마다 따로 (실행 문맥 · CUDA stream · 버퍼 각자). 한 엔진을 두 스레드가 같이 쓰지 않음
- 바코드는 출발 구역에 하나라 켜져 있을 때(READ_BARCODE)만 그 카메라 콜백 안에서 판독 → 따로 스레드 안 둠. 판독 중엔 그 카메라 처리가 느려짐 (그때 다른 일 없음)
- 손목 바코드는 동기 전 컬러로 읽음 → 깊이 · TF 가 없어도 읽힘
- 한쪽 엔진을 못 읽거나 카메라가 없어도 나머지는 계속 돎 (로그 ERROR)

서비스: `~/wide/enable`, `~/wrist/enable`, `~/barcode/enable` (`std_srvs/SetBool`), `~/barcode/reset` (`std_srvs/Trigger`). `wide` 를 꺼도 바코드 판독은 따로 (`~/barcode/enable`)

## 추론 엔진 = TensorRT (Jetson)

- TensorRT 10 API 로 `.engine` 을 직접 실행 (`TrtEngine`). ONNX Runtime 안 씀
- 빌드 때 `NvInfer.h` · `libnvinfer` · CUDA 를 자동으로 찾음 (JetPack 에 들어 있음). 없으면 `perception_core` 와 테스트만 빌드됨
- 엔진은 **Jetson 에서** 만듦 (다른 GPU · 다른 TensorRT 판 엔진은 안 읽힘). 입출력 float32 만 받으므로 `--inputIOFormats` · `--outputIOFormats` 는 주지 않음
- 광각 · 손목 같은 방식: PC 에서 `export_onnx.py` → `<이름>.onnx` + `<이름>.names.txt` → Jetson 에서 `trtexec` → `<이름>.engine`. 엔진 옆에 `.names.txt` 필수 (없으면 엔진 읽기 실패)
- 확장자 `.engine` / `.plan` 만. ultralytics `export(format='engine')` 결과는 앞에 메타데이터가 붙어 못 읽음 → 쓰지 말 것

```bash
/usr/src/tensorrt/bin/trtexec --onnx=<모델>.onnx --saveEngine=<모델>.engine --fp16
```

| 모델 | ONNX 만들기 (PC) | 출력 |
|---|---|---|
| 광각 상자 6클래스 | `yolo/wide/scripts/export_onnx.py --weights runs/wide/<run>/weights/best.pt` (기본 544×960) → `.onnx` + `.names.txt` | `[1, 4+6, 후보]` 또는 `[1, 300, 6]` |
| 손목 분할 8클래스 | `yolo/wrist/scripts/export_onnx.py --weights runs/wrist/seg8_sim/weights/best.pt` → `.onnx` + `.names.txt` | 검출 + 마스크 원형 2 개 |

## 빌드 · 실행

```bash
colcon build --packages-select interfaces perception
```

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

```bash
ros2 launch perception perception.launch.py wide_model:=<광각>.engine wrist_model:=<손목>.engine publish_debug:=true
```

- 설정 = 레포 `config/perception.yaml` (`wide.*`, `traffic_light.*`, `wrist.*`, `barcode.*`). launch 인자는 준 것만 yaml 을 덮어씀
- 광각 카메라 드라이버는 따로 (`/wide_cam/image_raw`)
- 손목 처리 시간 5 초마다 로그 (`전체 · 분할 ms`)

## 테스트

```bash
colcon test --packages-select perception && colcon test-result --verbose
```

| 시험 | 내용 | 결과 (개발 PC) |
|---|---|---|
| `test_wide` | letterbox · NMS · 출력 두 형식 해석 · 퍽 태그 (장소 태그 버림) · 신호등 칸 판정 (모두 꺼짐 = OFF · 빈 상자 = unknown) | 11 개 통과 |
| `test_barcode` | 합성 EAN-13 · Code128 P2-01–04 (모듈 2 · 3 · 5 px) · 8° 기울고 흐림 · 빈 영상 (zxing 빌드) | 4 개 통과 |
| `test_common` | 다수결 · 퍽 태그 ID · 메시지 규칙 (NaN · 0 쿼터니언 · ID · 색/모양 · 신호등 상수) | 6 개 통과 |
| `test_wrist_pose` | 손목 기하 · 확정 (합성 점군) | 15 개 통과 |
| `test_wrist_seg` | 합성 출력으로 분할 전후처리 · 출력 모양 판정 | 3 개 통과 |
| TensorRT 코드 · 노드 | 개발 PC 엔 TensorRT 없음 → `-DPERCEPTION_HAVE_TRT -fsyntax-only` 로 문법만 확인 (`trt_engine.cpp` 제외) | Jetson 에서 빌드 · 실행 확인 필요 |

## 주의 · 남은 것

- 바코드 판독 = **zxing-cpp** (`sudo apt install libzxing-dev`, Code128 포함 1D 전부). 없으면 OpenCV (EAN/UPC 만, Code128 못 읽음) 로 빌드되고 경고. Jetson 설치 목록 `docs/setup/jetson_install_list.md`
- 읽히는 크기 (합성 시험): 막대 한 칸(모듈) **1.5 px 이상** 안정, 1.0 px 는 흐림 적을 때만. 발제 바코드 (폭 70 mm, 약 110 모듈 = 0.64 mm/모듈) 를 D435 (fx 606) 로 1.5 px 이상 보려면 **약 0.26 m 안**
- 광각 퍽 위치는 NaN (깊이 없음). 지도 위치가 필요하면 상자 아랫변 → 바닥 평면 투영을 추가해야 함
- 광각은 `wide.max_hz` 보다 빠른 영상은 탐지를 건너뜀 (발행도 안 함)
- 이미지 구독은 기본 `reliable` (depth 1). 1280×720 이미지를 프로세스 사이에서 `best_effort` 로 받으면 거의 다 유실됨 (Fast DDS 실측)
- 신호등 칸 위치(`traffic_light.red/yellow/green`)는 실물 신호등으로 다시 잴 것
- Orin Nano 8 GB 에서 두 엔진 동시 실행 메모리 · 지연은 실측 필요 (불리하면 GPU 추론 구간만 직렬화)
