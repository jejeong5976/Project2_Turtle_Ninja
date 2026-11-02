# meshes — 로봇 모양 파일

`urdf/robot.urdf.xacro` 가 `package://description/meshes/...` 로 참조한다. 단위는 m(OBJ) 또는 mm(STL, URDF 에서 scale 0.001).
보기용(visual)은 부품별 폴더, 충돌용(collision)은 각 폴더의 `collision/` 아래에 둔다.

| 폴더 | 내용 | 출처 |
|---|---|---|
| `base/` | 와플 판 한 장(`waffle_plate.obj`, 판 20 장 · 하판 4 장이 같은 파일을 위치만 바꿔 씀), 조향 모터 몸체, USB 보드 | ROBOTIS 와플 판 STEP · 팀 Swerve CAD |
| `base/hardware/` | 지지 기둥 · 나사 · 너트 · 브래킷 (CAD 부품 이름 그대로, 같은 부품은 한 파일을 여러 위치에 씀) | 팀 Swerve CAD (`Assembly_1.step`) |
| `swerve/` | 모듈 4 개(`module_*`: 조향하면 같이 도는 브래킷 + 주행 모터), 바퀴 4 개(`wheel_*`) | 팀 Swerve CAD |
| `arm/` | 팔 링크별 프레임 · 모터 (`link1` ~ `link5`, `wrist`), 손가락 · 그리퍼 크랭크 · 링크, 그리퍼 받침(`park_stand`) | ROBOTIS OpenManipulator-X CAD (Apache-2.0, `arm/LICENSE`) + 실측 |
| `arm/collision/` | 팔 · 손가락 충돌 형상 (볼록 조각) | 위 CAD 에서 분해 |
| `camera/` | 손목 D435, 광각 카메라 PCB · 렌즈 · 받침 · 브래킷 | Intel D435 CAD · 실측 |
| `lidar/` | LDS-03 몸체 · 광학부 · 장착 귀 | ROBOTIS LDS-03 CAD |
| `electronics/` | 젯슨 Orin Nano (캐리어 · 모듈 · 방열판 · 팬 · SSD), 배터리 2 개, OpenCR 2 장 (주행 · 팔) | 제조사 CAD, 위치는 위에서 찍은 사진으로 맞춤 |
| `structure/` | 2020 프로파일 기둥 450 mm, 기둥 고정판 | 실측 |

## 원칙
- 파일 하나 10 MB 이하, 전체 30 MB 이하 — 판처럼 평평한 것은 원본 CAD 에서 적은 삼각형으로, 복잡한 부품은 모양을 지키는 줄이기(quadric)로. 원본과 표면 차이 0.5 mm 안.
- 보기용 메시는 질량 · 충돌에 쓰지 않는다. 질량은 `base_link` 등 링크의 `inertial`, 충돌은 `collision/` 또는 단순 도형.
- 새 부품을 넣을 때: 부품 폴더에 `<부품>_<부위>.obj` 이름으로 (날짜 · 해시 · "임시" 같은 말을 이름에 넣지 않는다).
