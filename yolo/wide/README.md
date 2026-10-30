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
│   └── export.py              # .pt → ONNX (TensorRT 엔진은 Jetson 에서)
└── notebooks/
    └── train_real.ipynb       # 2단계 학습 · 평가 · conf 0.5 혼동 행렬 (실행 결과 포함)
```
커밋하지 않는 것: `datasets/`, `runs/`, `models/`, `configs/data_*.yaml` (각자 PC 절대경로), `yolo26n.pt`, 가중치(`*.pt`, `*.onnx`, `*.engine`).
학습한 가중치는 Google Drive 에 올린다 ([가중치 폴더](⟪Drive 링크⟫)).

## 환경

| 항목 | 값 |
|---|---|
| GPU | RTX 5060 Laptop 8 GB |
| Python | 3.12 |
| ultralytics | 8.4.185 |
| torch | 2.11 (CUDA 12.8) |
| 그 외 | albumentations, tensorboard, opencv-python, pyyaml |

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install ultralytics==8.4.185 albumentations tensorboard
```

`yolo26n.pt` 는 첫 학습 때 ultralytics 가 자동으로 받는다.

## 데이터

### 합성 (1단계 학습용)

시뮬레이터가 만든 사진과 라벨을 그대로 쓴다 (광각 fisheye 왜곡, 아크릴 반사, 신호등 불빛 반영).

| split | 장수 |
|---|---|
| train | 2834 |
| val | 392 |
| test | 374 |

학습 입력 크기는 imgsz 960 이다. 1단계 학습에는 합성 train 만 쓰고, best epoch 는 **실제 val** 로 고른다.

### 실제

광각 카메라로 찍은 영상에서 프레임을 뽑아(`extract_frames.py`) CVAT 으로 라벨링한 뒤, `build_real_split.py` 로 나눈다.

- 같은 영상의 이웃 프레임은 거의 같다. 그래서 영상 하나를 통째로 한 split 에 넣는다.
- 어두운 조명 영상처럼 한 split 에 몰리면 안 되는 영상은 시간 순서로 6구간으로 자른다. 구간마다 train, test, train, train, val, train 순서로 배정한다.
- split 이 바뀌는 경계 뒤 90 프레임(3초)은 버린다.
- 밝기가 60(0~255) 보다 낮은 사진을 '어두움'으로 따로 센다.

| split | 장수 | 어두운 사진 | 비고 |
|---|---|---|---|
| train | 490 | 45 | 2단계 학습 |
| val | 68 | – | 1·2단계 best epoch 고르기 |
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
python train.py --stage real --weights ../runs/wide/sim_pre/weights/best.pt

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
| 데이터 | 합성 train 2834장 / 실제 val 68장으로 best 선택 |
| imgsz | 960 |
| lr0 | 0.001 |
| epoch | ⟪실험 후 확정: 최대 150, patience 30 / 또는 33 고정⟫ |

### 2차 (실제만)

| 항목 | 값 |
|---|---|
| 시작 가중치 | 1차 ⟪best.pt / last.pt⟫ |
| 데이터 | 실제 train 490 / val 68 / test 52 |
| lr0 | 0.0005 |
| epoch | 최대 60, patience 30 (best ⟪ep⟫) |
| freeze | 없음 (전체 층 학습) |

## 최종 성능 (실제 test 52장)

mAP 는 conf 0.001(표준), 혼동 행렬은 planning 기준 conf 0.5, IoU 0.5 로 센다.

| 모델 | 구분 | mAP50 | mAP50-95 |
|---|---|---|---|
| 1차만 (합성) | 전체 | ⟪ ⟫ | ⟪ ⟫ |
| **2차 (최종)** | 전체 | ⟪ ⟫ | ⟪ ⟫ |
| | 밝음 (43장) | ⟪ ⟫ | ⟪ ⟫ |
| | 어두움 (9장) | ⟪ ⟫ | ⟪ ⟫ |

클래스별 mAP50:

| red_cyl | green_cyl | red_box | green_box | barcode | traffic_light |
|---|---|---|---|---|---|
| ⟪ ⟫ | ⟪ ⟫ | ⟪ ⟫ | ⟪ ⟫ | ⟪ ⟫ | ⟪ ⟫ |

conf 0.5 혼동 행렬: 정답 103개 중 맞힘 ⟪ ⟫, 다른 클래스 ⟪ ⟫, 놓침 ⟪ ⟫, 오검출 ⟪ ⟫

실시간 검출(`live_detect.py`, RTX 5060, imgsz 960): 약 32 FPS, 추론 6 ms

## 알려진 한계

- 원기둥을 옆에서 보면 윗면 원이 안 보여 사각기둥으로 잡는다 (green_cyl → green_box).
- 멀리 있는 퍽은 빨간 원기둥과 빨간 사각기둥을 자주 헷갈린다. 가까워졌을 때 여러 프레임 다수결로 확정한다.
- test 가 52장이라 모델 사이 차이가 0.05 안쪽이면 크다고 보기 어렵다.

## Jetson 배포

```bash
python export.py --weights ../runs/wide/real_ft/weights/best.pt      # PC: ONNX (736×1280)
# Jetson 에서 엔진 빌드 (다른 GPU 에서 만든 엔진은 안 돎)
python3 -c "from ultralytics import YOLO; YOLO('best.pt').export(format='engine', imgsz=(736, 1280), half=True)"
```
