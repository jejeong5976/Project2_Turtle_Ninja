# yolo/wrist — 손목 D435 퍽 분할 (YOLO26-seg) · 자세 추정

광각 상자 YOLO(`yolo/wide/`)와 따로 둔 손목 카메라 모델. 퍽 마스크(YOLO-seg) + 정렬 깊이 → 퍽 중심 · 긴축 (카메라 광학 좌표).

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
│   ├── export_onnx.py         # C++ 노드(wrist_puck_node)용 ONNX + .names.txt
│   ├── wrist_pose.py          # (모듈) 마스크 + 깊이 → 퍽 중심 · 긴축 · 선/누움 · 면 방향
│   ├── wrist_detector.py      # (모듈) seg → 자세 → 거르기 → 여러 프레임 확정 → Detection
│   ├── eval_sim.py            # 시뮬 정답 대비 자세 오차
│   └── eval_detector.py       # 검출 최종 결과 정확도 평가 (확정률 · 오탐 · 오차)
├── experiments/
│   └── train_seg8.ipynb       # 라벨 변환 → 학습 → 평가 노트북 (커널 yolo)
├── datasets/                  # 커밋 안 함
│   ├── sim_wrist/             #   시뮬 생성기 출력 (sim/robot_sim/scripts/make_wrist_dataset.py)
│   ├── sim_seg/               #   seg 라벨 + data.yaml (sim_to_seg.py)
│   └── real_d435/             #   tools/d435_viewer 저장 장면
└── runs/                      # 학습 결과 · 로그 · 가중치, 커밋 안 함
```

- 실물 D435 실시간 보기 · 사진 저장은 `tools/d435_viewer/`, 손목 카메라 장착 자세(핸드-아이) 캘리브레이션은 `tools/wrist_calib/`
- C++ ROS 노드는 `ros2_ws/src/perception` (`wrist_puck_node`, gtest `test/test_wrist_core.cpp`). Python `wrist_pose.py` · `wrist_detector.py` 는 평가(`eval_*`) · 실물 보기용이고 C++ 과는 따로 관리

클래스 (`sim_to_seg.py --classes`)
- 8 (기본): `0 red_cyl_up 1 green_cyl_up 2 red_box_up 3 green_box_up 4 red_cyl_lying 5 green_cyl_lying 6 red_box_lying 7 green_box_lying`
- 4: `0 red_cyl, 1 green_cyl, 2 red_box, 3 green_box`

라벨 변환: `scripts/sim_to_seg.py` → `datasets/sim_seg` (YOLO 다각형 + data.yaml)
- 파란 퍽은 생성기에서 마스크 · meta 에 안 들어감 → 배경 (오탐 방지)

학습 · 평가 노트북: `experiments/train_seg8.ipynb` (커널 `yolo`)
- 8클래스 라벨 변환 → `train.py` → 그래프 · test · conf 기준값 · 오검출 · 자세 평가 (`eval_sim.py` · `eval_detector.py`)

## 순서

```bash
# 각 단계는 레포 최상위에서 시작
# 1) 시뮬 데이터 (Isaac Lab, sim/robot_sim)
cd sim/robot_sim && OMNI_KIT_ACCEPT_EULA=YES env -u PYTHONPATH ~/pa-course/IsaacLab-6.2/.venv/bin/python scripts/make_wrist_dataset.py --maze real --seed 1 --scenes 170 --views 2

# 2) seg 라벨 · 학습 (YOLO venv)
cd yolo/wrist/scripts
~/Desktop/physical_ai/.venvs/yolo/bin/python sim_to_seg.py --check 4
~/Desktop/physical_ai/.venvs/yolo/bin/python train.py --stage sim   # 합성 데이터만, → runs/wrist/seg8_sim, yolo26n-seg.pt 에서 처음부터 (이전 best.pt 이어 학습 안 함)

# 3) 확인
cd yolo/wrist/scripts
~/Desktop/physical_ai/.venvs/yolo/bin/python eval_sim.py --data ../datasets/sim_wrist --split test --gravity --mode seg --weights ../runs/wrist/seg8_sim/weights/best.pt
~/Desktop/physical_ai/.venvs/yolo/bin/python eval_detector.py --weights ../runs/wrist/seg8_sim/weights/best.pt --split test
# 실물 D435 (레포 최상위에서)
cd tools/d435_viewer && ~/Desktop/physical_ai/.venvs/yolo/bin/python d435_puck_viewer.py --seg ../../yolo/wrist/runs/wrist/seg8_sim/weights/best.pt
```

## 파이프라인 (`scripts/wrist_detector.py`)

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
- 위쪽(`up_cam`)이 없으면 긴축이 PCA 로만 정해져 크게 틀림(시뮬 90% 83°) → 로봇에선 TF 로 꼭 넣을 것
- 시뮬 확인 (`eval_detector.py`, `runs/wrist/seg8_sim`, `datasets/sim_wrist` test 771장 × 3회, 깊이 잡음 2 mm, 3장 확정, 잘림 없는 퍽 973개): 확정 99% (965), 오탐 3, 색 · 모양 965/965, 선/누움 953/965, xy 중앙 2.9 · 90% 5.2 mm, z 중앙 1.4 · 90% 2.8 mm, 누운 퍽 θ 중앙 0.8 · 90% 3.6 · 최대 27.9°, 선 사각기둥 면 θ 230개 중 218개 구함 · 중앙 0.2 · 최대 1.8°. 화면 끝에 잘린 퍽은 거름(`truncated`)
- 평가는 `datasets/sim_wrist` 의 test split (학습에 안 쓴 장면). seg 학습용 `sim_seg` 도 같은 split 을 씀

- 실물 데모 `tools/d435_viewer/d435_puck_viewer.py` 는 `--seg` 없으면 HSV 색 분할 (모니터 속 퍽 · 피부도 잡음)
- 필요 패키지: `ultralytics albumentations tensorboard pyrealsense2==2.59.1.11053`
