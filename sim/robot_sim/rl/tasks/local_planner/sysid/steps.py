"""계단 명령 순서 (ROS 없음): record.py (실물 · Isaac) 와 plant_response.py (학습 플랜트) 가 같이 씀"""

VMAX, WMAX = 0.2, 0.5          # 실물 swerve_controller 한계 (config/controllers.yaml)
PAUSE_S = 2.0                  # 계단 사이 0 명령


def plan():
    """(이름, vx, vy, wz, 유지 s, 뒤 정지 s) — 같은 크기 반대 계단을 바로 뒤에 둬서 제자리 근처로 돌아옴"""
    P = []
    def pair(name, v, hold):
        P.append((name + '+', *v, hold, PAUSE_S))
        P.append((name + '-', *(-x for x in v), hold, PAUSE_S))
    for s, h in ((0.05, 3.0), (0.10, 2.0), (0.20, 1.5)):
        pair(f'fwd_{s:.2f}', (s, 0, 0), h)                 # 앞뒤 계단: 지연 · 시간 상수 · 정상 비율
    for s, h in ((0.10, 2.0), (0.20, 1.5)):
        pair(f'side_{s:.2f}', (0, s, 0), h)                # 옆 (조향 90°)
    pair('diag_0.14', (0.1, 0.1, 0), 2.0)                  # 대각 (조향 45°)
    for w, h in ((0.25, 3.0), (0.50, 2.0)):
        pair(f'rot_{w:.2f}', (0, 0, w), h)                 # 제자리 회전
    for s in (0.01, 0.02, 0.03):
        pair(f'slow_{s:.2f}', (s, 0, 0), 3.0)              # 최소 속도 (정지 마찰 · 명령이 묻히는 속도)
    pair('arc', (0.1, 0, 0.3), 2.0)                        # 가면서 돌기
    # 방향 전환 (멈추지 않고 앞 → 옆): 조향이 90° 도는 동안 속도가 얼마나 빠지나
    P.append(('turn90_a', 0.1, 0, 0, 1.5, 0.0)); P.append(('turn90_b', 0, 0.1, 0, 1.5, PAUSE_S))
    P.append(('turn90_back_a', 0, -0.1, 0, 1.5, 0.0)); P.append(('turn90_back_b', -0.1, 0, 0, 1.5, PAUSE_S))
    return P
