# yolo/wrist — 손목 D435 퍽 분할 (YOLO26-seg) · 자세 추정

광각 상자 YOLO(`yolo/wide/`)와 따로 둔 손목 카메라 모델. 퍽 마스크(YOLO-seg) + 정렬 깊이 → 퍽 중심 · 긴축 (카메라 광학 좌표).

| 항목 | 값 |
|---|---|
| 모델 | YOLO26n-seg (2.71 M 파라미터, 9.2 GFLOPs), `yolo26n-seg.pt` 에서 처음부터 학습 |
| 클래스 | 8 = 색 2 × 모양 2 × 서 있음/누움 |
| 데이터 | Isaac Sim 합성 7,539 장 (퍽 15,533 개), 실제 데이터 학습 없음 |
| 결과 (합성 test 771 장) | 마스크 mAP50 **0.992** · mAP50-95 **0.960** / 상자 mAP50 0.993 · mAP50-95 0.980 |
| 배포 | ONNX (480×640, one2many head) → Jetson TensorRT **FP16**, 분할 추론 중앙 **7.9 ms**, 손목 처리 **24.3 Hz** (1 회차 경기 중) |

## 디렉토리 구조

```
yolo/wrist/
├── README.md
├── configs/
│   └── hyp.yaml               # 학습 설정 (yolo26n-seg, imgsz 640, 증강, 데이터 = datasets/sim_seg/data.yaml)
├── scripts/
│   ├── sim_to_seg.py          # 시뮬 생성 데이터 → YOLO-seg 다각형 라벨 + data.yaml
│   ├── train.py               # 학습 (항상 yolo26n-seg.pt 에서 처음부터)
│   ├── val.py                 # seg 평가 (상자 · 마스크 mAP)
│   ├── export_onnx.py         # C++ perception 노드(손목 파이프라인)용 ONNX + .names.txt
│   ├── wrist_pose.py          # (모듈) 마스크 + 깊이 → 퍽 중심 · 긴축 · 선/누움 · 면 방향
│   ├── wrist_detector.py      # (모듈) seg → 자세 → 거르기 → 여러 프레임 확정 → Detection
│   ├── eval_sim.py            # 시뮬 정답 대비 자세 오차
│   └── eval_detector.py       # 검출 최종 결과 정확도 평가 (확정률 · 오탐 · 오차)
├── experiments/
│   └── train_seg8.ipynb       # 라벨 변환 → 학습 → 평가 노트북 (커널 yolo)
├── datasets/                  # 커밋 안 함
│   ├── sim_wrist/             #   시뮬 생성기 출력 (sim/robot_sim/scripts/make_wrist_dataset.py)
│   ├── sim_seg/               #   seg 라벨 + data.yaml (sim_to_seg.py)
│   └── real_d435/             #   tools/wrist_d435/d435_puck_viewer.py 저장 장면
└── runs/                      # 학습 결과 · 로그 · 가중치, 커밋 안 함
```

- 실물 D435 실시간 보기 · 사진 저장은 `tools/wrist_d435/` (같은 폴더에 손목 카메라 장착 자세(핸드-아이) 캘리브레이션)
- C++ ROS 노드는 `ros2_ws/src/perception` (`perception` 노드의 손목 파이프라인, gtest `test/test_wrist_pose.cpp` · `test_wrist_seg.cpp`). Python `wrist_pose.py` · `wrist_detector.py` 는 평가(`eval_*`) · 실물 보기용이고 C++ 과는 따로 관리
- 학습된 가중치 (`best_wrist.pt` · `best_wrist.onnx` · `best_wrist.names.txt`) 는 저장소 Release `project2-submit` 에 첨부

## 1. 데이터셋

**생성**: `sim/robot_sim/scripts/make_wrist_dataset.py` (Isaac Sim, 실물 D435 내부 파라미터 640×480)
- 로봇이 박스 앞 관찰 자세 (LOOK) 에서 손목 카메라로 촬영 → 컬러 · 깊이 · 퍽 마스크 · 정답 자세 (meta)
- 장면 생성 시드 19 개 (`w40`–`w59`), 장면마다 시점 2 개 (`v0` · `v1`)
- 무작위화: 배경 파노라마 · 밝기 · 벽판 반사, 퍽 · 박스 색 흔들기, 박스 위 퍽 1–4 개 (눕힘 50 %), 그리퍼 벌림, 퍽 표면 거칠기, 태그 붙인 면 · 각도 오차, 브래킷 각도, 가짜 물체 0–3 개, 박스 앞 바닥 퍽
- 파란 퍽은 라벨 없이 넣음 → 배경으로 학습 (오탐 방지)

**라벨**: `scripts/sim_to_seg.py` → `datasets/sim_seg` (YOLO 다각형 + data.yaml)
- 장면 단위로 나눔 (같은 장면의 두 시점이 train · val 에 섞이지 않음)

| split | 장 | 퍽 (인스턴스) | 퍽 없는 장 |
|---|---|---|---|
| train | 6,022 | 12,427 | 373 |
| val | 746 | 1,544 | 53 |
| test | 771 | 1,562 | 42 |
| 합 | **7,539** | **15,533** | 468 |

**클래스 분포** (`sim_to_seg.py --classes 8`)

| ID | 클래스 | train | val | test |
|---|---|---|---|---|
| 0 | red_cyl_up | 1,807 | 229 | 227 |
| 1 | green_cyl_up | 1,732 | 205 | 199 |
| 2 | red_box_up | 1,697 | 195 | 204 |
| 3 | green_box_up | 1,836 | 235 | 198 |
| 4 | red_cyl_lying | 1,262 | 169 | 193 |
| 5 | green_cyl_lying | 1,352 | 170 | 196 |
| 6 | red_box_lying | 1,429 | 177 | 183 |
| 7 | green_box_lying | 1,312 | 164 | 162 |

- `--classes 4` 로 바꾸면 `red_cyl · green_cyl · red_box · green_box` (선/누움 구분 없음)

## 2. 학습 설정 (`configs/hyp.yaml`)

| 항목 | 값 | 이유 |
|---|---|---|
| 모델 | `yolo26n-seg.pt` (처음부터) | 이전 best.pt 이어 학습 안 함 |
| imgsz · batch | 640 · 24 | D435 컬러 640×480 그대로 (학습은 정사각 640) |
| optimizer · lr0 · lrf | AdamW · 0.0005 · 0.01 | `auto` 면 lr0 가 무시됨 |
| momentum · weight_decay | 0.937 · 0.0005 | |
| epochs · patience | 150 · 30 | 99 epoch 에서 조기 종료, best = 69 epoch |
| warmup_bias_lr · seed · amp | 0.0 · 42 · true | |
| mosaic · close_mosaic | 0 · 0 | 손목 화면은 퍽 1–4 개를 가까이 봄 |
| overlap_mask | true | |
| hsv_h · hsv_s · hsv_v | 0.01 · 0.3 · 0.3 | 색이 클래스에 들어감 → 색상은 작게, 채도 0.5 는 빨강이 분홍이 됨 |
| degrees · translate · scale · fliplr | 10 · 0.1 · 0.3 · 0.5 | 팔 자세에 따른 기울기, 관찰 거리 0.2–0.45 m |

**추가 증강 (albumentations)**

| 증강 | 확률 | 범위 | 흉내 내는 것 |
|---|---|---|---|
| 색온도 | 0.5 | 4000–8000 K | 조명 종류 |
| 노출 · 대비 | 0.4 | ±0.25 · ±0.2 | 자동 노출 |
| 감마 | 0.3 | 70–140 | |
| 흐림 | 0.3 | 움직임 3–9 · 초점 1–3 | 팔 이동 중 흔들림, 가까운 거리 초점 |
| 부분 그림자 | 0.15 | 0.1–0.2 | 한쪽만 어두움 |
| 색 물듦 | 0.1 | RGB +8–25 | 신호등 빛 (빨강 · 초록 쪽) |
| 조명 단계 | 0.35 | dim 2 : dark 1 : dark_ae 2 | 어두운 방, 자동 노출로 대비 · 채도 낮고 잡음 큼 |

- 학습: RTX 5060 Laptop 8 GB, 99 epoch 1.5 시간

## 3. 결과

**분할 (합성 test 771 장, 퍽 1,562 개, PyTorch FP32)**

| | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| 상자 | 0.983 | 0.966 | 0.993 | 0.980 |
| 마스크 | 0.984 | 0.967 | 0.992 | 0.960 |

**클래스별 마스크 (val 746 장, best.pt)**

| 클래스 | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| red_cyl_up | 0.985 | 0.974 | 0.994 | 0.953 |
| green_cyl_up | 0.993 | 0.990 | 0.995 | 0.961 |
| red_box_up | 0.989 | 0.944 | 0.991 | 0.962 |
| green_box_up | 0.957 | 0.946 | 0.989 | 0.964 |
| red_cyl_lying | 0.981 | 0.988 | 0.994 | 0.967 |
| green_cyl_lying | 0.981 | 0.982 | 0.994 | 0.957 |
| red_box_lying | 0.994 | 0.961 | 0.993 | 0.974 |
| green_box_lying | 0.950 | 0.939 | 0.988 | 0.974 |
| **전체** | 0.979 | 0.966 | 0.992 | 0.964 |

- 상대적으로 낮은 곳: 초록 사각기둥 (선 · 누움 모두 P · R 0.94–0.96)

**최종 검출 · 자세 (`eval_detector.py`, test 771 장 × 3 회, 깊이 잡음 2 mm, 3 장 확정, 잘림 없는 퍽 973 개)**

| 항목 | 결과 |
|---|---|
| 확정률 | 99 % (965 / 973), 오탐 3 |
| 색 · 모양 | 965 / 965 |
| 선 / 누움 | 953 / 965 |
| 중심 xy | 중앙 2.9 · 90 % 5.2 mm |
| 중심 z | 중앙 1.4 · 90 % 2.8 mm |
| 누운 퍽 긴축 θ | 중앙 0.8 · 90 % 3.6 · 최대 27.9° |
| 선 사각기둥 면 θ | 230 개 중 218 개 구함, 중앙 0.2 · 최대 1.8° |

- 화면 끝에 잘린 퍽은 거름 (`truncated`)
- 평가는 `datasets/sim_wrist` 의 test split (학습에 안 쓴 장면). seg 학습용 `sim_seg` 도 같은 split 을 씀

## 4. 경량화

같은 `best.pt` 를 정밀도 · 런타임별로 비교함 (합성 test 771 장).

| 형식 | 런타임 | 크기 | 마스크 mAP50 | 마스크 mAP50-95 | 추론 시간 |
|---|---|---|---|---|---|
| PyTorch FP32 | RTX 5060 Laptop | 6.6 MB | 0.992 | 0.960 | forward 중앙 3.28 ms |
| PyTorch FP16 | RTX 5060 Laptop | — | 0.993 | 0.967 | forward 중앙 2.96 ms, GPU 메모리 99 → 60 MiB |
| ONNX FP32 (480×640) | ONNX Runtime CPU | 11.1 MB | 0.992 | 0.950 | 38.1 ms |
| ONNX INT8 (정적 양자화, train 320 장 보정) | ONNX Runtime CPU | 3.3 MB | 0.987 | 0.909 | 169.4 ms |
| **TensorRT FP16** (`trtexec --fp16`) | **Jetson Orin Nano** | — | — | — | **분할 추론 중앙 7.9 ms** (GPU 6.0 ms) |

**자세 정확도 (seg + 태그 + 자세 계산, 개발 PC)**

| 정밀도 | 찾은 퍽 | 중심 오차 중앙 | 잘림 없는 퍽 중심 90 % | 잘림 없는 퍽 긴축 90 % | 장당 시간 중앙 |
|---|---|---|---|---|---|
| FP32 | 1,536 / 1,552 | 4.13 mm | 5.68 mm | 2.63° | 16.5 ms |
| FP16 | 1,536 / 1,552 | 4.13 mm | 5.68 mm | 2.63° | 16.5 ms |
| INT8 | 1,523 / 1,552 | 4.07 mm | 5.86 mm | 2.87° | 144.7 ms (CPU) |

**선택: TensorRT FP16**
- FP16 은 FP32 와 정확도 차이가 없고 (mAP · 자세 오차 같음) GPU 메모리를 약 40 % 줄임
- INT8 은 파일이 1/3 이지만 마스크 mAP50-95 가 0.950 → 0.909 로 떨어지고 (경계 정밀도 손실), 퍽 13 개를 더 놓침
- Jetson 에서는 GPU 가 여유 있고 CPU 가 병목 → GPU 추론은 FP16 으로 충분히 빠르고, CPU 몫 (전처리 1.0 · 후처리 1.4 ms) 을 줄이는 쪽이 이득
- 1 회차 경기 중 손목 처리 중앙 24.3 Hz (깊이 수신 약 25–28 Hz 가 상한)

**변환**

```bash
cd yolo/wrist/scripts
~/Desktop/physical_ai/.venvs/yolo/bin/python export_onnx.py --weights ../runs/wrist/seg8_sim/weights/best.pt   # → best_wrist.onnx + .names.txt (480×640)
# Jetson
trtexec --onnx=best_wrist.onnx --saveEngine=$HOME/models/best_wrist.engine --fp16
cp best_wrist.names.txt $HOME/models/
```

- 학습 체크포인트는 `end2end=False` → ONNX 도 one2many head + NMS (one2one 은 신뢰도가 다르게 나옴)

## 5. 순서

```bash
# 각 단계는 레포 최상위에서 시작
# 1) 시뮬 데이터 (Isaac Lab, sim/robot_sim)
cd sim/robot_sim && OMNI_KIT_ACCEPT_EULA=YES env -u PYTHONPATH $ISAACLAB_PY scripts/make_wrist_dataset.py --maze real --seed 1 --scenes 170 --views 2

# 2) seg 라벨 · 학습 (YOLO venv)
cd yolo/wrist/scripts
~/Desktop/physical_ai/.venvs/yolo/bin/python sim_to_seg.py --check 4
~/Desktop/physical_ai/.venvs/yolo/bin/python train.py --stage sim   # 합성 데이터만, → runs/wrist/seg8_sim, yolo26n-seg.pt 에서 처음부터

# 3) 확인
cd yolo/wrist/scripts
~/Desktop/physical_ai/.venvs/yolo/bin/python eval_sim.py --data ../datasets/sim_wrist --split test --gravity --mode seg --weights ../runs/wrist/seg8_sim/weights/best.pt
~/Desktop/physical_ai/.venvs/yolo/bin/python eval_detector.py --weights ../runs/wrist/seg8_sim/weights/best.pt --split test
# 실물 D435 (레포 최상위에서)
cd tools/wrist_d435 && ~/Desktop/physical_ai/.venvs/yolo/bin/python d435_puck_viewer.py --seg ../../yolo/wrist/runs/wrist/seg8_sim/weights/best.pt
```

학습 · 평가 노트북: `experiments/train_seg8.ipynb` (커널 `yolo`)
- 8 클래스 라벨 변환 → `train.py` → 그래프 · test · conf 기준값 · 오검출 · 자세 평가 (`eval_sim.py` · `eval_detector.py`)

## 6. 파이프라인 (`scripts/wrist_detector.py`)

```python
import wrist_detector as wd
det = wd.WristPuckDetector('../runs/wrist/seg8_sim/weights/best.pt')    # conf 0.5, 최근 5 장 중 3 장 확정
dets, raws, tags = det.process(bgr, depth_m, K, up_cam)                 # up_cam = 카메라 좌표의 월드 위쪽 (로봇에선 TF)
for d in dets:                                                          # interfaces/Detection 필드 (카메라 광학 프레임)
    pp = wd.planar_pose(d, R_bc, t_bc)                                  # base_link ← d435_color_optical_frame (촬영 시각 TF, 컬러 렌즈)
    print(d.label, d.shape, d.id, pp.x, pp.y, pp.z, pp.theta, pp.upright, pp.theta_known)
det.reset()                                                             # 팔(카메라)을 움직인 뒤
```

- 거르기: 화면 끝에 잘린 퍽(`truncated`), 점군이 퍽보다 큼(`size`: 긴 방향 > 8.5 cm, 두 번째 > 5 cm), 깊이 점 < 50(`points`)
- θ: 선 사각기둥 = 면 방향(90° 주기, 윗면 점에 최소 면적 사각형을 맞춰 구함), 누운 퍽 = 긴축 방향(180° 주기), 선 원기둥 = 필요 없음(`theta_known=False`)
- 태그는 **ID 읽기에만** 씀 (위쪽을 알 때). 실물 태그는 손으로 붙여 면과 나란하다는 보장이 없고 붙인 면도 퍽마다 다름 → 긴축 · 선/누움 · 중심 · 면 방향은 중력 + 점군
- 위쪽(`up_cam`)이 없으면 긴축이 PCA 로만 정해져 크게 틀림(시뮬 90 % 83°) → 로봇에선 TF 로 꼭 넣을 것

## 7. 한계

- 실제 데이터로 학습 · 평가하지 않음 (합성만). 실물은 손목 영상 · 정답 종이 (`tools/wrist_d435`) 로 확인: 위쪽 추정 오차 0.5–1.5°, 수평 중앙 1.5–2.1 mm
- 실물 깊이는 퍽 윗면이 주변 면보다 약 1.3 mm 멀게 나옴 (플라스틱) → 중심 높이는 놓인 면 + 크기 절반으로 구함
- 실물 데모 `tools/wrist_d435/d435_puck_viewer.py` 는 `--seg` 없으면 HSV 색 분할 (모니터 속 퍽 · 피부도 잡음)
- 필요 패키지: `ultralytics albumentations tensorboard pyrealsense2==2.59.1.11053`
