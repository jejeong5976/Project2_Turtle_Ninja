# tools/d435_viewer — 손목 D435 실시간 퍽 자세 보기 · 사진 저장

실물 D435 를 개발 PC 에 연결해 ROS 없이 퍽 자세(중심 · 긴축 · 선/누움 · 태그 ID)를 바로 확인. 계산은 `yolo/wrist/scripts/wrist_pose.py` · `wrist_detector.py` 를 씀 (로봇에서 도는 건 C++ `wrist_puck_node`).

```bash
cd tools/d435_viewer
PY=python3                     # requirements.txt 설치한 venv 의 python
$PY d435_puck_viewer.py --seg ../../yolo/wrist/runs/wrist/seg8_sim/weights/best.pt   # q 끝, s 저장, p 3D 점군, r 확정 초기화
$PY d435_puck_viewer.py --frames 5          # 창 없이 5 장 저장
$PY d435_puck_viewer.py --offline <저장 폴더> # 카메라 없이 저장 장면 다시 처리
```

- 저장 위치: `yolo/wrist/datasets/real_d435/<시각>/` (커밋 안 함)
- `--seg` 없으면 HSV 색 분할 (모니터 속 퍽 · 피부도 잡음)
- 필요: 레포 최상위 `requirements.txt` (`--seg` 를 쓸 때만 ultralytics 추가). `tools/wrist_calib/handeye.py` 도 이 파일의 화면 창(`Viewer`)을 씀
