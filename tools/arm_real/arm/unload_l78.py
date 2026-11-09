#!/usr/bin/env python3
"""L7 · L8 꺼내기 — 사람 시연 재생 (계산만). 실행은 사람이: arm_pose_runner.py lc_unl78.checked.json --capture

  $ARM_PY unload_l78.py L7      (L7 먼저 — L8 꺼낼 때 L7 위로 다가가야 함)
  $ARM_PY unload_l78.py L8

L7 · L8 은 손을 세운 채로는 퍽을 컵 밖으로 못 듦 (손끝 최고 L8 149 · L7 163 mm). 사람 시연(arm_teach, 토크 끔):
  얕게 쥐고(L8 약 14 · L7 약 21 mm) J4 를 기구 끝(약 2485 틱) 가까이 쓰며 거의 수직으로 들어 올림 → 그 관절 기록을 그대로 재생.
  재생 구간만 J4 상한 2470 (끝 15 틱 앞, 12:15 실물: 2495 에서 8 s 버티다 과부하) — 넘는 값은 2470 으로 자름.
순서: 받침 → 박스 위 → (넣기 때 경유점) → 칸 위에서 그리퍼 '놓기' 간격으로 열기 → 시연 쥐는 자세 → 닫기 → 시연 꺼내기 → 박스 위 → 박스에 놓기 → 받침.
  L8 은 칸 위에 손이 못 가서(퍽 윗면보다 낮음) 넣을 때 빠져나온 길(L7 위 → 옆으로) 을 거꾸로 들어감.
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import grasp_test as gt           # noqa: E402
import pose_safety as ps          # noqa: E402
import rack_load                  # noqa: E402
import sag_model                  # noqa: E402

PY = os.environ.get('ARM_PY', sys.executable)
TEACH = HERE / 'teach_l78'   #  사람 시연 기록 (poses.txt = 자세 폴더 이름, 자세 자료는 POSE_MEASURE/runs/)
POSE_MEASURE = Path(os.environ.get('POSE_MEASURE', HERE.parent / 'arm_workspace_measure_20261024'))
J4_HI = 2470
DROP = (0.235, 0.0)               # 박스 놓는 자리 = 넣기 경유점(L7/L8 설정)의 출발 자리
SHOW = {                          # 칸: (시연 폴더 순서, 쥐는 순간 기록 번호, 꺼낸 끝 기록 번호, 쥔 깊이 m)
    'L8': (0, 198, 240, 0.014),
    'L7': (1, 522, 540, 0.021),
}


def frame(d, k):
    m = {x['id']: x['registers']['present_position']['value'] for x in json.loads(Path(f'{d}/02_track/{k:05d}.json').read_text())['motors']}
    return [m[11], m[12], m[13], m[14], m[16]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cell', choices=('L7', 'L8'))
    ap.add_argument('--out', default='lc_unl78')
    a = ap.parse_args()
    cell = a.cell
    k_dir, k_grasp, k_end, depth = SHOW[cell]
    d = Path((TEACH / 'poses.txt').read_text().split()[k_dir])
    if not d.is_absolute() or not d.exists():
        d = POSE_MEASURE / 'runs' / d.name
    C = ps.Checker()
    rc = rack_load.load_cells()
    if not rc['cells'][cell]['state'].startswith('full'):
        sys.exit(f'칸 {cell} 은 빈 칸')
    if cell == 'L8' and rc['cells']['L7']['state'].startswith('full'):
        sys.exit('L7 이 차 있음 — L7 먼저 꺼낼 것 (L8 은 L7 위로 다가감)')

    def enc(t, kg):
        q, fq = C._qf(t)
        return gt.q_to_ticks(sag_model.encoder_goal_q(C.M, q, fq, held_kg=kg))

    def cap(t, kg):                                                     # 처짐 앞먹임 명령까지 J4 ≤ J4_HI (L8: 명령 2471)
        t = list(t); t[3] = min(t[3], J4_HI)
        while enc(t, kg)[3] > J4_HI:
            t[3] -= 1
        return t

    # 박스 위 · 내려놓기 자세 (퍽 윗면 기준 손끝 높이: 위 40 · 중간 10 · 놓기 = 쥔 깊이 − 3 mm 아래)
    tmp = HERE / f'{a.out}_drop_ik.json'
    hs = ['40', '10', f'{-(depth * 1000 - 3):.0f}']
    r = subprocess.run([PY, str(HERE / 'grasp_test.py'), '--x', str(DROP[0]), '--y', str(DROP[1]), '--heights', *hs, '--yaw', '0',
                        '--note', f'{cell} 꺼내기 놓는 자리', '--out', str(tmp)], text=True, capture_output=True)
    if r.returncode:
        sys.exit('박스 놓는 자리 IK 안 됨')
    ik = json.loads(tmp.read_text())
    P = {p['name']: p for p in ik['poses']}
    up, mid, low = (P[f'h{h}'] for h in hs)
    lift = up['ticks']
    # 다가가는 길 = 넣기 계획(L7/L8 설정)의 경유점 — 목표 칸을 비운 상태로 계획
    rce = json.loads(json.dumps(rc)); rce['cells'][cell]['state'] = 'empty'
    rp, desc = rack_load.plan_rack(C, cell, lift, rce)
    N = {p['name']: p for p in rp}
    carries = [p['ticks'] for p in rp if p['name'].startswith('carry')]
    g0 = cap(frame(d, k_grasp), 0.0)
    poses = [dict(name='lift', ticks=lift, enc=enc(lift, 0.0), stop=False)]
    poses += [dict(name=f'carry{i + 1}', ticks=t, enc=enc(t, 0.0), stop=False, rack=True, speed=25.0) for i, t in enumerate(carries)]
    if cell == 'L8':
        slides = [p['ticks'] for p in rp if p['name'].startswith('slide')]
        if not slides:
            sys.exit('L8 계획에 옆으로 빠지는 길 없음')
        poses += [dict(name='open_at', ticks=slides[-1], enc=enc(slides[-1], 0.0), stop=False, rack=True, grip='release', speed=15.0)]
        poses += [dict(name=f'in{i + 1}', ticks=t, enc=enc(t, 0.0), stop=False, rack=True, speed=10.0) for i, t in enumerate(list(reversed(slides[:-1])) + [N['out_cell']['ticks']])]
    else:
        ta = N['over_cell']['ticks']
        poses += [dict(name='open_at', ticks=ta, enc=enc(ta, 0.0), stop=False, rack=True, grip='release', speed=15.0)]
    # 쥐기 직전 멈춤 (손가락이 퍽 양쪽에 왔는지 눈으로 — 아니면 q 로 받침) → 같은 자리에서 닫기
    poses += [dict(name='grasp_check', ticks=g0, enc=enc(g0, 0.0), stop=True, rack=True, speed=8.0, j4_hi=J4_HI),
              dict(name='grasp', ticks=g0, enc=enc(g0, 0.0), stop=False, rack=True, grip='close', speed=8.0, j4_hi=J4_HI)]
    # 시연 꺼내기 (3 기록마다, J4 자름)
    for k in range(k_grasp + 3, k_end + 1, 3):
        t = cap(frame(d, k), 0.017)
        poses.append(dict(name=f'out{k}', ticks=t, enc=enc(t, 0.017), stop=False, rack=True, held_kg=0.017, speed=8.0, j4_hi=J4_HI))
    held = dict(held_kg=0.017)
    sl = dict(speed=15.0)
    poses += [dict(name='lift_back', ticks=lift, enc=enc(lift, 0.017), stop=False, **held, speed=20.0),
              dict(name='down2', ticks=mid['ticks'], enc=mid['enc'], stop=False, **held, **sl),
              dict(name='drop', ticks=low['ticks'], enc=low['enc'], stop=False, grip='open', **held, **sl),
              dict(name='up2', ticks=mid['ticks'], enc=mid['enc'], stop=False, **sl),
              dict(name='clear', ticks=up['ticks'], enc=up['enc'], stop=False, grip='close', **sl)]
    obstacles = [o for o in ik['obstacles'] if o['name'] != 'puck'] + rack_load.cup_obstacles(rce, cell)
    out = HERE / f'{a.out}.json'
    out.write_text(json.dumps(dict(note=f'칸 {cell} 꺼내기 (시연 재생 {d.name} #{k_grasp}~#{k_end}, 쥠 약 {depth * 1000:.0f} mm, J4 ≤ {J4_HI})',
                                   obstacles=obstacles, poses=poses), ensure_ascii=False, indent=1))
    print(f'칸 {cell} 꺼내기: 다가가기 {desc} · 시연 꺼내기 {len([p for p in poses if p["name"].startswith("out")])} 자세')
    res = subprocess.run([PY, str(HERE / 'pose_safety.py'), str(out)], text=True, capture_output=True)
    print('\n'.join(l for l in res.stdout.splitlines() if '✗' in l or '전체' in l or '저장' in l))
    if res.returncode != 0:
        sys.exit('안전 검사 실패 — 실행 금지')
    print(f'실행: /usr/bin/python3 arm_pose_runner.py {a.out}.checked.json --capture   (끝나면 rack_state.py set {cell} empty)')


if __name__ == '__main__':
    main()
