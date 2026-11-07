# wide: 광각 카메라 YOLO

광각 카메라(1280×720, MJPG) 영상에서 퍽 4종과 바코드, 신호등을 찾는 YOLO26n 모델을 학습한다.
학습은 2단계로 진행한다. 1단계는 합성 데이터로 처음부터 학습하고, 2단계는 실제 데이터만으로 미세조정한다.

| 번호 | 클래스 | 설명 |
|---|---|---|
| 0 | red_cyl | 빨간 원기둥 퍽 |
| 1 | green_cyl | 초록 원기둥 퍽 |
| 2 | red_box | 빨간 사각기둥 퍽 |
| 3 | green_box | 초록 사각기둥 퍽 |
| 4 | barcode | 벽·상자 바코드 |
| 5 | traffic_light | 신호등 |

AprilTag 는 YOLO 가 아니라 전용 검출기로 찾는다.

## 폴더

```
yolo/wide/
├── README.md
├── configs/
│   └── hyp.yaml               # 학습 설정 (공통 + 단계별)
├── scripts/
│   ├── extract_frames.py      # 영상 → 라벨링할 프레임
│   ├── build_real_split.py    # 라벨된 실제 데이터 → train/val/test (비슷한 프레임이 섞이지 않게)
│   ├── prepare_data.py        # configs/data_sim.yaml, data_real.yaml 만들기
│   ├── train.py               # 학습 (--stage sim / real)
│   ├── val.py                 # 평가 (클래스별 mAP, 혼동 행렬)
│   ├── predict.py             # 예측 결과 그림 저장
│   ├── live_detect.py         # 카메라 실시간 검출
│   └── export_onnx.py         # .pt → ONNX + .names.txt (C++ perception 노드용, 엔진은 Jetson trtexec)
└── notebooks/
    └── train.ipynb            # 1·2단계 학습 · 평가 · conf 0.5 혼동 행렬 (실행 결과 포함)
```
커밋하지 않는 것: `datasets/`, `runs/`, `models/`, `configs/data_*.yaml` (각자 PC 절대경로), `yolo26n.pt`, 가중치(`*.pt`, `*.onnx`, `*.engine`).
학습한 가중치는 레포에 올리지 않는다 (공유 위치는 팀에서 정함).

## 환경

| 항목 | 값 |
|---|---|
| GPU | RTX 5060 Laptop 8 GB |
| Python | 3.12 |
| ultralytics | 8.4.185 |
| torch | 2.11 (CUDA 12.8) |
| 그 외 | albumentations, tensorboard, opencv-python, pyyaml |

```bash
python3 -m venv ~/Desktop/physical_ai/.venvs/yolo
~/Desktop/physical_ai/.venvs/yolo/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128   # RTX 50 계열
~/Desktop/physical_ai/.venvs/yolo/bin/pip install ultralytics==8.4.185 albumentations tensorboard onnx ipykernel
~/Desktop/physical_ai/.venvs/yolo/bin/python -m ipykernel install --user --name yolo --display-name "Python (yolo venv)"   # 노트북 커널
```

- albumentations 가 `opencv-python-headless` 를 같이 깔아 `cv2.imshow` 가 안 됨 → `live_detect.py` 는 Tk 창으로 대신 보여 줌

`yolo26n.pt` 는 첫 학습 때 ultralytics 가 자동으로 받는다.

## 데이터

### 합성 (1단계 학습용)

시뮬레이터가 만든 사진과 라벨을 그대로 쓴다 (광각 fisheye 왜곡, 아크릴 반사, 신호등 불빛 반영).

| split | 장수 |
|---|---|
| train | 2834 |
| val | 392 |
| test | 374 |

학습 입력 크기는 imgsz 960 이다. 1단계 학습에는 합성 train 만 쓴다. 실제 val 은 학습 곡선 보기용이고, 33 epoch 고정 학습 뒤 `last.pt` 를 2단계로 넘긴다.

### 실제

광각 카메라로 찍은 영상에서 프레임을 뽑아(`extract_frames.py`) CVAT 으로 라벨링한 뒤, `build_real_split.py` 로 나눈다.

- 같은 영상의 이웃 프레임은 거의 같다. 그래서 영상 하나를 통째로 한 split 에 넣는다.
- 어두운 조명 영상처럼 한 split 에 몰리면 안 되는 영상은 시간 순서로 6구간으로 자른다. 구간마다 train, test, train, train, val, train 순서로 배정한다.
- split 이 바뀌는 경계 뒤 90 프레임(3초)은 버린다.
- 밝기가 60(0~255) 보다 낮은 사진을 '어두움'으로 따로 센다.

| split | 장수 | 어두운 사진 | 비고 |
|---|---|---|---|
| train | 490 | 45 | 2단계 학습 |
| val | 68 | – | 1단계 곡선 보기, 2단계 best epoch 고르기 |
| test | 52 | 9 | 상자 103개, 최종 성능은 이것으로만 |

## 실행 순서

모든 명령은 `yolo/wide/scripts` 에서 실행한다.

```bash
cd yolo/wide/scripts

# 0) (새 영상이 있을 때) 프레임 뽑기 → CVAT 라벨링 → 실제 데이터 나누기
python extract_frames.py ~/videos/wide_xxx.mkv
python build_real_split.py --team_zip <선반 영상 zip> --tl_dir <신호등 CVAT 폴더> --cvat_zip <퍽 영상 zip ...>

# 1) data.yaml 만들기 (datasets/sim, datasets/real 기준)
python prepare_data.py

# 2) 1단계: 합성으로 처음부터
python train.py --stage sim

# 3) 2단계: 실제 데이터만으로 미세조정
python train.py --stage real --weights ../runs/wide/sim_pre/weights/last.pt

# 4) 실제 test 평가
python val.py --weights ../runs/wide/real_ft/weights/best.pt --data ../configs/data_real.yaml --split test

# 5) 카메라로 확인 (conf 0.5)
python live_detect.py --model real_ft --dev 2
```

같은 이름의 결과 폴더가 이미 있으면 ultralytics 가 `real_ft2` 처럼 이름을 바꿔 저장한다. 다시 학습할 때는 `--name` 을 새로 준다.

## 학습 설정

공통 설정(`configs/hyp.yaml` 의 `common`):

| 항목 | 값 | 이유 |
|---|---|---|
| model | yolo26n.pt | Jetson 실시간용 |
| imgsz | 960 | 1.2 m 밖 퍽이 10 px 안팎이라 640 은 작음 |
| batch | 16 | 8 GB GPU 한계 |
| optimizer | AdamW | |
| patience | 30 | val 이 30 epoch 동안 안 오르면 멈춤 |
| seed | 42 | |
| hsv_h / hsv_s / hsv_v | 0.01 / 0.5 / 0.3 | 클래스에 색이 들어 있어 색상은 거의 안 바꿈 |
| degrees / translate / scale / fliplr | 3 / 0.1 / 0.2 / 0.5 | 카메라가 로봇에 고정 |
| mosaic | 0 | |
| albumentations | 색온도, 노출, 감마, 흐림, 비네팅 | 실제 카메라 차이 대비 |

### 1차 (합성)

| 항목 | 값 |
|---|---|
| 시작 가중치 | yolo26n.pt |
| 데이터 | 합성 train 2834장 (실제 val 68장은 곡선 보기용) |
| imgsz | 960 |
| lr0 | 0.001 |
| epoch | 33 고정 (patience 0), `last.pt` 사용 |

val 이 68장뿐이라 best epoch 로 고르면 시작점이 운에 따라 크게 바뀐다 (ep9 → 2단계 test 0.907, ep33 → 0.949).

### 2차 (실제만)

| 항목 | 값 |
|---|---|
| 시작 가중치 | 1차 `last.pt` |
| 데이터 | 실제 train 490 / val 68 / test 52 |
| lr0 | 0.0005 |
| epoch | 최대 60, patience 30 (37 epoch 에서 멈춤, best ep7) |
| freeze | 없음 (전체 층 학습) |

## 최종 성능 (실제 test 52장)

mAP 는 conf 0.001(표준), 혼동 행렬은 planning 기준 conf 0.5, IoU 0.5 로 센다.

| 모델 | 구분 | mAP50 | mAP50-95 |
|---|---|---|---|
| 1차만 (합성) | 전체 | 0.597 | 0.383 |
| **2차 (최종)** | 전체 | **0.949** | **0.705** |
| | 밝음 (43장) | 0.950 | 0.690 |
| | 어두움 (9장) | 0.995 | 0.823 |

클래스별 mAP50:

| 모델 | red_cyl | green_cyl | red_box | green_box | barcode | traffic_light |
|---|---|---|---|---|---|---|
| 1차만 | 0.870 | 0.663 | 0.454 | 0.105 | 0.964 | 0.523 |
| 2차 | 0.957 | 0.924 | 0.961 | 0.891 | 0.968 | 0.993 |

conf 0.5 에서 틀린 것 (정답 → 예측, 개수): green_cyl → green_box 7, red_box 놓침 6, barcode 놓침 3, red_box → red_cyl 2, traffic_light 놓침 1, red_cyl 놓침 1 (그림 `runs/eval/confusion_conf05.png`)

실시간 검출(`live_detect.py`, RTX 5060, imgsz 960): 약 32 FPS, 추론 6 ms

## 알려진 한계

- 원기둥을 옆에서 보면 윗면 원이 안 보여 사각기둥으로 잡는다 (green_cyl → green_box).
- 멀리 있는 퍽은 빨간 원기둥과 빨간 사각기둥을 자주 헷갈린다. 가까워졌을 때 여러 프레임 다수결로 확정한다.
- test 가 52장이라 모델 사이 차이가 0.05 안쪽이면 크다고 보기 어렵다.

## Jetson 배포

C++ perception 노드(`ros2_ws/src/perception`, 광각 파이프라인)가 쓰는 방식. 손목 모델과 같음.

```bash
# PC: ONNX + 클래스 이름 파일 (가중치 옆에 best_544x960.onnx, best_544x960.names.txt)
python export_onnx.py --weights ../runs/wide/real_ft/weights/best.pt                         # 기본 544×960
python export_onnx.py --weights ../runs/wide/real_ft/weights/best.pt --height 736 --width 1280

# Jetson: 엔진 빌드 (다른 GPU 에서 만든 엔진은 안 돎)
trtexec --onnx=best_544x960.onnx --saveEngine=best_544x960.engine --fp16
ros2 launch perception perception.launch.py wide_model:=<경로>/best_544x960.engine
```

- 엔진 옆에 같은 이름 `.names.txt` 필수 (없으면 노드가 엔진 읽기에서 멈춤)
- ultralytics `export(format='engine')` 는 쓰지 않음: 엔진 앞에 메타데이터를 붙여 C++ `TrtEngine` 이 못 읽음
- 입력 크기는 노드가 엔진에서 읽음. 544×960 = 학습 imgsz 960 과 같은 축척, 736×1280 = 원본 축척. Jetson `trtexec` 속도로 고름
